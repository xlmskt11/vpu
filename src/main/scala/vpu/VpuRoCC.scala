package vpu

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

import chisel3._

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.tile._

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

  core.io.command.valid := io.cmd.valid
  // The transport reserves the upper half of rs1.  Preserve the core's normal
  // malformed-command/status path by setting a reserved micro-op bit instead
  // of silently accepting a non-canonical command word.
  core.io.command.bits.microOp := io.cmd.bits.rs1(31, 0) |
    Mux(io.cmd.bits.rs1(63, 32).orR, "h80000000".U(32.W), 0.U)
  core.io.command.bits.payload := io.cmd.bits.rs2
  core.io.command.bits.rd := io.cmd.bits.inst.rd
  core.io.command.bits.xd := io.cmd.bits.inst.xd
  core.io.command.bits.status := io.cmd.bits.status
  io.cmd.ready := core.io.command.ready

  io.resp.valid := core.io.response.valid
  io.resp.bits.rd := core.io.response.bits.rd
  io.resp.bits.data := core.io.response.bits.data
  core.io.response.ready := io.resp.ready

  outer.memory.module.io.dma <> core.io.dma
  io.ptw.head <> outer.memory.module.io.ptw

  io.busy := core.io.busy || outer.memory.module.io.busy
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
}
