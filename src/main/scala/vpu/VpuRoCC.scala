package vpu

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

import chisel3._
import chisel3.util.Valid

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.tile._
import gemmini.{GemminiVpuMatrixReadIO, GemminiVpuMatrixWriteReq,
  VsramClientIO}

/** One standalone VPU sharing custom0 with Gemmini0 through an exact funct
  * route.  All memory traffic leaves through the VPU's own TileLink node; no
  * Gemmini scratchpad/accumulator interface is present in this wrapper.
  */
class VpuRoCC(val vpuParams: VpuParams)(implicit p: Parameters) extends LazyRoCC(
  opcodes = OpcodeSet.custom0,
  nPTWPorts = 1,
  usesFPU = false,
  commandRoute = RoCCCommandRoute.exact(VpuRoCC.TransportFunct)) {

  Files.write(Paths.get(vpuParams.headerFilePath),
    vpuParams.generateHeader().getBytes(StandardCharsets.UTF_8))

  val memory = LazyModule(new VpuTLMemory(vpuParams))
  override val tlNode = memory.idNode
  override lazy val module = new VpuRoCCModule(this)
}

object VpuRoCC {
  final val TransportFunct = 64
}

class VpuRoCCModule(outer: VpuRoCC)
    extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {
  require(xLen == 64, "The v1 VPU RoCC transport requires RV64")

  val core = Module(new VpuCore(outer.vpuParams))

  val matrixRead = if (outer.vpuParams.matrixPorts > 0) Some(IO(Vec(
    outer.vpuParams.matrixPorts, Flipped(new GemminiVpuMatrixReadIO(
      outer.vpuParams.matrixRowAddrBits,
      outer.vpuParams.matrixElementsPerRow,
      outer.vpuParams.storageBits))))) else None
  val matrixWrite = if (outer.vpuParams.matrixPorts > 0) Some(IO(Flipped(Vec(
    outer.vpuParams.matrixPorts, Valid(new GemminiVpuMatrixWriteReq(
      outer.vpuParams.matrixRowAddrBits,
      outer.vpuParams.matrixElementsPerRow,
      outer.vpuParams.storageBits)))))) else None
  val vsramDeps = if (outer.vpuParams.enableSharedDeps) Some(IO(
    new VsramClientIO(
      outer.vpuParams.sharedHazardAddressBits,
      VpuReservationStation.totalEntries(outer.vpuParams)))) else None
  if (outer.vpuParams.matrixPorts > 0) {
    core.io.matrixRead.get <> matrixRead.get
    core.io.matrixWrite.get <> matrixWrite.get
  }
  if (outer.vpuParams.enableSharedDeps) {
    core.io.vsramDeps.get <> vsramDeps.get
  }

  /** Present only in a fused configuration. Standalone configurations retain
    * the old closed VPU interface and treat every nonzero upper rs1 bit as a
    * malformed transport command.
    */
  val gemvGroup = if (outer.vpuParams.enableGroupedCommands) {
    Some(IO(new VpuGemvGroupIO(outer.vpuParams.groupIdBits)))
  } else None

  val transportCommand = Wire(new VpuCommand)
  transportCommand.microOp := io.cmd.bits.rs1(31, 0)
  transportCommand.payload := io.cmd.bits.rs2
  transportCommand.rd := io.cmd.bits.inst.rd
  transportCommand.xd := io.cmd.bits.inst.xd
  transportCommand.status := io.cmd.bits.status
  val transportGrouped = Wire(new VpuGroupedCommand(
    outer.vpuParams.groupIdBits,
    outer.vpuParams.enableGroupedCommands))
  transportGrouped.command := transportCommand
  transportGrouped.malformed := io.cmd.bits.rs1(63, 32).orR

  if (outer.vpuParams.enableGroupedCommands) {
    transportGrouped.groupId.get := io.cmd.bits.rs1(34, 32)
    transportGrouped.grouped.get := io.cmd.bits.rs1(35)
    transportGrouped.last.get := io.cmd.bits.rs1(36)
    // Ungrouped commands retain the canonical upper-half-zero contract. For
    // grouped commands only rs1[36:32] is defined.
    transportGrouped.malformed := io.cmd.bits.rs1(63, 37).orR ||
      (!io.cmd.bits.rs1(35) && io.cmd.bits.rs1(34, 32).orR) ||
      (!io.cmd.bits.rs1(35) && io.cmd.bits.rs1(36))
  }

  // PLENA-style LOOP_START/END regions are captured before the group gate and
  // replayed one fine command at a time.  Every replayed command still passes
  // through the ordinary group, reservation-station, and SharedDeps admission
  // path; the loop frontend is only a transport/dispatch compressor.
  val hardwareLoop = Module(new VpuHardwareLoop(
    groupIdBits = outer.vpuParams.groupIdBits,
    enableGroupedCommands = outer.vpuParams.enableGroupedCommands,
    bufferEntries = outer.vpuParams.loopBufferEntries,
    maxDepth = outer.vpuParams.loopStackDepth))
  hardwareLoop.io.in.valid := io.cmd.valid
  hardwareLoop.io.in.bits := transportGrouped
  io.cmd.ready := hardwareLoop.io.in.ready
  val groupGateBusy = WireDefault(false.B)

  io.resp.valid := core.io.response.valid
  io.resp.bits.rd := core.io.response.bits.rd
  io.resp.bits.data := core.io.response.bits.data
  core.io.response.ready := io.resp.ready

  if (outer.vpuParams.enableGroupedCommands) {
    val gate = Module(new VpuGroupCommandGate(outer.vpuParams.groupIdBits))
    val group = gemvGroup.get

    gate.io.in <> hardwareLoop.io.out
    core.io.command <> gate.io.out
    gate.io.reservationAdmission := core.io.commandRsAdmission.get
    gate.io.commandRejected := core.io.commandRejected.get
    gate.io.commandClearsFusionFault := core.io.commandClearsFusionFault.get

    group.pending := gate.io.group.pending
    group.groupId := gate.io.group.groupId
    gate.io.group.groupAllocated := group.groupAllocated
    gate.io.group.dispatchEnable := group.dispatchEnable
    gate.io.group.dispatchReject := group.dispatchReject
    group.lastDispatchFire := gate.io.group.lastDispatchFire
    group.abort := gate.io.group.abort
    groupGateBusy := gate.io.busy
    core.io.fusionFault.get := gate.io.fusionFault

    dontTouch(gate.io.busy)
    dontTouch(gate.io.protocolError)
  } else {
    core.io.command.valid := hardwareLoop.io.out.valid
    // Preserve the original malformed-command/status path for upper rs1 bits.
    core.io.command.bits := hardwareLoop.io.out.bits.command
    core.io.command.bits.microOp :=
      hardwareLoop.io.out.bits.command.microOp |
      Mux(hardwareLoop.io.out.bits.malformed,
        "h80000000".U(32.W), 0.U)
    hardwareLoop.io.out.ready := core.io.command.ready
  }

  outer.memory.module.io.dma <> core.io.dma
  io.ptw.head <> outer.memory.module.io.ptw

  val acceleratorBusy = core.io.busy || outer.memory.module.io.busy ||
    groupGateBusy || hardwareLoop.io.busy
  io.busy := acceleratorBusy
  io.interrupt := false.B

  // The VPU uses its dedicated TL node rather than the cached RoCC memory port.
  io.mem.req.valid := false.B
  io.mem.req.bits := DontCare
  io.mem.s1_kill := false.B
  io.mem.s2_kill := false.B
  io.mem.keep_clock_enabled := true.B

  io.fpu_req.valid := false.B
  io.fpu_req.bits := DontCare
  io.fpu_resp.ready := true.B

  // Keep status and performance state visible in generated RTL/debug builds.
  dontTouch(core.io.status)
  dontTouch(core.io.perfCounters)
  dontTouch(hardwareLoop.io.busy)
  dontTouch(hardwareLoop.io.protocolError)
}
