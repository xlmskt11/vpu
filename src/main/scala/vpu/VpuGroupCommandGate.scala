package vpu

import chisel3._
import chisel3.util._

/** One-entry gate between the RoCC transport and VpuCore.
  *
  * Standalone commands retain a flow-through path.  A grouped command is
  * retained until LdBCompleteControl reports that every Gemmini child has
  * entered its reservation station/shared dependency table. The gate
  * deliberately releases the group on admission to
  * VpuCore (and hence its local reservation station), not on RoCC capture and
  * not on eventual functional-unit completion.
  */
class VpuGroupCommandGate(groupIdBits: Int) extends Module {
  require(groupIdBits == 3,
    "rs1[34:32] provides exactly three grouped-command ID bits")

  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new VpuGroupedCommand(groupIdBits, enableGroupedCommands = true)))
    val out = Decoupled(new VpuCommand)
    /** Same-cycle acknowledgement that `out.fire` allocated an LD/EX/ST RS
      * entry. The fused integration extends this with shared-dependency-table
      * allocation before driving it true.
      */
    val reservationAdmission = Input(Bool())
    /** The core consumed `out` through its illegal-command path. */
    val commandRejected = Input(Bool())
    val commandClearsFusionFault = Input(Bool())
    val group = new VpuGemvGroupIO(groupIdBits)
    val busy = Output(Bool())
    val protocolError = Output(Bool())
    val fusionFault = Output(Bool())
  })

  val heldValid = RegInit(false.B)
  val held = Reg(new VpuGroupedCommand(groupIdBits, enableGroupedCommands = true))
  val activeGroupValid = RegInit(false.B)
  val activeGroupId = RegInit(0.U(groupIdBits.W))
  val abortDrainGroupId = RegInit(0.U(groupIdBits.W))
  val abortDrainValid = RegInit(false.B)
  val fusionFault = RegInit(false.B)

  val selectedValid = heldValid || io.in.valid
  val selected = Wire(new VpuGroupedCommand(groupIdBits, enableGroupedCommands = true))
  selected := Mux(heldValid, held, io.in.bits)
  val selectedIsGrouped = selected.grouped.get
  val selectedGroupIdBits = selected.groupId.get
  val selectedIsLast = selected.last.get
  val inputIsGrouped = io.in.bits.grouped.get
  val inputGroupId = io.in.bits.groupId.get
  val inputIsLast = io.in.bits.last.get

  // Once a grouped sequence starts, every command through group_last belongs
  // to that same group.  Treat standalone interleaving as a protocol error as
  // well: configuration/control micro-ops used by a fused sequence must carry
  // the same grouped metadata as its vector operations.
  val sequenceMismatch = activeGroupValid &&
    (!selectedIsGrouped || selectedGroupIdBits =/= activeGroupId)
  val selectedOpcode = selected.command.microOp(5, 0)
  val lastCanAllocateReservation = VpuDecode.isVector(selectedOpcode) ||
    VpuDecode.isScalar(selectedOpcode) || VpuDecode.isState(selectedOpcode) ||
    VpuDecode.isLoad(selectedOpcode) || VpuDecode.isStore(selectedOpcode)
  val invalidLastType = selectedIsGrouped && selectedIsLast &&
    !lastCanAllocateReservation
  // Even a malformed grouped command must identify an allocated group before
  // it is sent to the core and converted into an abort.  Otherwise a command
  // which arrives before LOOP_WS allocation would pulse abort against no
  // group and trip LdBCompleteControl's protocol assertions.
  val selectedTargetsGroup = selectedIsGrouped ||
    (activeGroupValid && sequenceMismatch)
  val groupContractReject = selectedTargetsGroup && io.group.dispatchReject
  val selectedIllegal = selected.malformed || sequenceMismatch ||
    invalidLastType ||
    groupContractReject || (fusionFault && selectedIsGrouped)
  val selectedGrouped = selectedIsGrouped && !selectedIllegal
  val selectedGroupId = Mux(sequenceMismatch, activeGroupId,
    selectedGroupIdBits)
  val dispatchAllowed = Mux(selectedIllegal && selectedTargetsGroup,
    io.group.groupAllocated &&
      (io.group.dispatchEnable || io.group.dispatchReject),
    !selectedGrouped || io.group.dispatchEnable)

  // Once an intermediate command aborts a sequence, software may already
  // have emitted the remaining grouped micro-ops. Consume those commands up
  // to group_last without sending them to the core or querying a group which
  // the completion controller is releasing. Standalone status/clear commands
  // continue to pass through normally.
  val drainSelected = abortDrainValid && !heldValid && io.in.valid &&
    inputIsGrouped
  val drainMatches = drainSelected &&
    inputGroupId === abortDrainGroupId
  // An xd command has already made Rocket wait for a response.  It cannot be
  // silently discarded while draining an aborted sequence; send it through
  // the core's malformed-command path so that the architectural error
  // response is produced.  Fire-and-forget commands can be dropped locally.
  // Only the latched failed sequence may be dropped locally. A command from a
  // different group is still consumed through the core's malformed path so it
  // cannot accidentally terminate the drain or disappear without exercising
  // the architectural illegal-command path.
  val drainForward = drainSelected &&
    (io.in.bits.command.xd || !drainMatches)
  val drainDrop = drainMatches && !io.in.bits.command.xd
  io.out.valid := (selectedValid && dispatchAllowed && !drainSelected) ||
    drainForward
  io.out.bits := selected.command
  when(selectedIllegal || drainForward) {
    // Reuse VpuCore's architectural malformed-command path so the error is
    // visible through the existing sticky illegalCommand status and clear
    // mechanism. Bit 31 is one of the microinstruction's reserved bits.
    io.out.bits.microOp := selected.command.microOp | "h80000000".U
  }

  // The single entry may capture an ordinary blocked command even when
  // VpuCore is not ready. group_last is different: accepting it at the RoCC
  // input before its RS/SharedDeps admission would let Rocket advance to a
  // following Gemmini command while the terminator was still held here. Make
  // a direct group_last a non-bufferable pass-through, so successful software
  // completion of that RoCC instruction is the actual atomic admission point.
  // Abort-drain terminators remain locally consumable.
  val unbufferedGroupedLast = !heldValid && io.in.valid &&
    inputIsGrouped && inputIsLast && !drainSelected
  val ordinaryInputReady = !heldValid && (!drainForward || io.out.ready)
  io.in.ready := ordinaryInputReady &&
    (!unbufferedGroupedLast || io.out.fire)
  val inputFire = io.in.fire
  val outputFire = io.out.fire
  val directOutputFire = outputFire && !heldValid
  val drainDropFire = drainDrop && inputFire
  val drainForwardFire = drainForward && outputFire

  when(heldValid && outputFire) {
    heldValid := false.B
  }.elsewhen(!heldValid && inputFire && !directOutputFire && !drainDropFire) {
    heldValid := true.B
    held := io.in.bits
  }

  // Start tracking on RoCC acceptance so a following command cannot change
  // group IDs while the first command waits in the gate. End tracking only
  // once the final command actually enters VpuCore.
  val inputStartsSequence = inputFire && !abortDrainValid &&
    inputIsGrouped &&
    !io.in.bits.malformed && !activeGroupValid && !inputIsLast
  when(inputStartsSequence) {
    activeGroupValid := true.B
    activeGroupId := inputGroupId
  }
  val lastAdmission = outputFire && selectedGrouped && selectedIsLast &&
    io.reservationAdmission
  // group_last is contractually an LD/EX/ST reservation allocation.  A
  // legal zero-VL no-op has no RS entry, so using it as the final grouped
  // command is an abort instead of a silent permanent group hold.
  val lastMissingAdmission = outputFire && selectedGrouped && selectedIsLast &&
    !io.reservationAdmission
  val groupedProtocolReject = !abortDrainValid && outputFire &&
    selectedTargetsGroup &&
    (selectedIllegal || io.commandRejected || lastMissingAdmission)
  // Abort only a still-live completion-controller entry.
  val groupedAbort = groupedProtocolReject && io.group.groupAllocated
  when(groupedAbort || lastAdmission) {
    activeGroupValid := false.B
  }
  when(groupedAbort) {
    // On an A->B sequence mismatch, the command already at the transport head
    // is B and software's remaining queued micro-ops normally belong to B.
    // Keep that incoming ID separate from the completion-controller target A.
    abortDrainGroupId := selectedGroupIdBits
  }

  when(io.commandClearsFusionFault) {
    fusionFault := false.B
  }
  when(groupedAbort) {
    fusionFault := true.B
    when(!selectedIsLast) {
      abortDrainValid := true.B
    }
  }
  when((drainDropFire || drainForwardFire) && drainMatches &&
      inputIsLast) {
    abortDrainValid := false.B
  }

  io.group.pending := !abortDrainValid && selectedValid &&
    selectedTargetsGroup
  io.group.groupId := Mux(abortDrainValid, abortDrainGroupId,
    Mux(selectedValid, selectedGroupId,
      Mux(activeGroupValid, activeGroupId, abortDrainGroupId)))
  io.group.lastDispatchFire := lastAdmission
  io.group.abort := groupedAbort

  io.protocolError := outputFire &&
    (selectedIllegal || io.commandRejected || lastMissingAdmission) ||
    drainDropFire || drainForwardFire
  io.busy := heldValid || activeGroupValid || abortDrainValid
  io.fusionFault := fusionFault

  when(io.group.dispatchEnable) {
    assert(io.group.groupAllocated,
      "VPU group dispatch was enabled for an unallocated group")
  }
  when(inputFire && inputIsGrouped && inputIsLast &&
      !drainSelected) {
    assert(outputFire,
      "group_last was accepted into the VPU gate without core dispatch")
  }
  when(io.group.lastDispatchFire) {
    assert(io.out.fire,
      "VPU group release occurred without core/RS admission")
  }

  dontTouch(io.group.pending)
  dontTouch(io.group.groupId)
  dontTouch(io.group.lastDispatchFire)
  dontTouch(io.group.abort)
  dontTouch(io.protocolError)
  dontTouch(fusionFault)
  dontTouch(abortDrainValid)
}
