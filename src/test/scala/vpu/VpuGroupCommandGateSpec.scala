package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuGroupCommandGateTester(c: VpuGroupCommandGate)
    extends PeekPokeTester(c) {
  private def defaults(): Unit = {
    poke(c.io.in.valid, 0)
    poke(c.io.in.bits.command.microOp, 0)
    poke(c.io.in.bits.command.payload, 0)
    poke(c.io.in.bits.command.rd, 0)
    poke(c.io.in.bits.command.xd, 0)
    poke(c.io.in.bits.groupId.get, 0)
    poke(c.io.in.bits.grouped.get, 0)
    poke(c.io.in.bits.last.get, 0)
    poke(c.io.in.bits.malformed, 0)
    poke(c.io.out.ready, 1)
    poke(c.io.reservationAdmission, 0)
    poke(c.io.commandRejected, 0)
    poke(c.io.commandClearsFusionFault, 0)
    poke(c.io.group.groupAllocated, 0)
    poke(c.io.group.dispatchEnable, 0)
    poke(c.io.group.dispatchReject, 0)
  }

  private def drive(group: Int, grouped: Boolean, last: Boolean,
                    malformed: Boolean = false,
                    opcode: Int = VpuOpcode.V_ADD_VF,
                    xd: Boolean = false,
                    payload: BigInt = 0): Unit = {
    poke(c.io.in.valid, 1)
    poke(c.io.in.bits.command.microOp, VpuEncoding.pack(opcode))
    poke(c.io.in.bits.command.payload, payload)
    poke(c.io.in.bits.command.xd, if (xd) 1 else 0)
    poke(c.io.in.bits.groupId.get, group)
    poke(c.io.in.bits.grouped.get, if (grouped) 1 else 0)
    poke(c.io.in.bits.last.get, if (last) 1 else 0)
    poke(c.io.in.bits.malformed, if (malformed) 1 else 0)
  }

  private def clearFusionFault(): Unit = {
    drive(group = 0, grouped = false, last = false,
      opcode = VpuOpcode.C_CLEAR_STATUS, payload = 2)
    poke(c.io.commandClearsFusionFault, 1)
    expect(c.io.out.valid, 1)
    step(1)
    poke(c.io.in.valid, 0)
    poke(c.io.commandClearsFusionFault, 0)
  }

  defaults()
  step(2)

  // Standalone commands keep the combinational flow-through behavior.
  drive(group = 0, grouped = false, last = false)
  expect(c.io.in.ready, 1)
  expect(c.io.out.valid, 1)
  expect(c.io.group.pending, 0)
  expect(c.io.group.lastDispatchFire, 0)
  step(1)
  poke(c.io.in.valid, 0)
  expect(c.io.busy, 0)
  expect(c.io.fenceBusy, 0)

  // A live legacy/already-closed group explicitly rejects grouped commands;
  // the gate converts the command into an abort instead of waiting forever
  // for dispatchEnable to become true.
  poke(c.io.group.groupAllocated, 1)
  poke(c.io.group.dispatchReject, 1)
  drive(group = 1, grouped = true, last = true)
  expect(c.io.out.valid, 1)
  expect(c.io.group.abort, 1)
  expect(c.io.fusionFault, 0) // becomes sticky at the accepting edge
  step(1)
  poke(c.io.in.valid, 0)
  poke(c.io.group.groupAllocated, 0)
  poke(c.io.group.dispatchReject, 0)
  expect(c.io.fusionFault, 1)
  clearFusionFault()

  // A grouped command is captured while its Gemmini group is incomplete.
  drive(group = 2, grouped = true, last = false)
  expect(c.io.in.ready, 1)
  expect(c.io.out.valid, 0)
  expect(c.io.group.pending, 1)
  expect(c.io.group.groupId, 2)
  step(1)
  poke(c.io.in.valid, 0)
  expect(c.io.in.ready, 0)
  expect(c.io.group.pending, 1)
  expect(c.io.busy, 1)
  expect(c.io.fenceBusy, 1)

  // The group controller raises dispatchEnable after all Gemmini commands are
  // registered. Permission alone still does not release a command while the
  // core/RS is full.
  poke(c.io.group.groupAllocated, 1)
  poke(c.io.group.dispatchEnable, 1)
  poke(c.io.out.ready, 0)
  expect(c.io.out.valid, 1)
  step(2)
  expect(c.io.group.lastDispatchFire, 0)

  poke(c.io.out.ready, 1)
  expect(c.io.out.valid, 1)
  step(1)
  expect(c.io.in.ready, 1)
  expect(c.io.busy, 1) // sequence remains open until group_last
  expect(c.io.fenceBusy, 0)

  // group_last cannot be captured in the one-entry buffer. Its RoCC input
  // handshake remains blocked until it can pass directly into VpuCore.
  drive(group = 2, grouped = true, last = true)
  poke(c.io.out.ready, 0)
  expect(c.io.in.ready, 0)
  expect(c.io.out.valid, 1)
  expect(c.io.group.lastDispatchFire, 0)
  step(2)
  expect(c.io.in.ready, 0)

  poke(c.io.out.ready, 1)
  poke(c.io.reservationAdmission, 1)
  expect(c.io.in.ready, 1)
  expect(c.io.group.lastDispatchFire, 1)
  step(1)
  poke(c.io.reservationAdmission, 0)
  poke(c.io.in.valid, 0)
  poke(c.io.group.groupAllocated, 0)
  poke(c.io.group.dispatchEnable, 0)
  poke(c.io.group.dispatchReject, 0)
  // The final command releases the group at RS/SharedDeps admission. A
  // following same-ID sequence is therefore an ordinary fresh query: the
  // one-entry gate captures it until the next LOOP_WS group is allocated and
  // enables dispatch.
  drive(group = 2, grouped = true, last = false)
  expect(c.io.in.ready, 1)
  expect(c.io.out.valid, 0)
  expect(c.io.group.pending, 1)
  step(1)
  poke(c.io.in.valid, 0)
  expect(c.io.busy, 1)
  expect(c.io.fenceBusy, 1)

  // Dispatch starts after the replacement group registers its Gemmini
  // children and raises dispatchEnable.
  expect(c.io.group.pending, 1)
  expect(c.io.out.valid, 0)
  step(1)
  poke(c.io.group.groupAllocated, 1)
  poke(c.io.group.dispatchEnable, 1)
  expect(c.io.out.valid, 1)
  step(1)
  expect(c.io.fenceBusy, 0)

  drive(group = 2, grouped = true, last = true)
  poke(c.io.reservationAdmission, 1)
  expect(c.io.group.lastDispatchFire, 1)
  expect(c.io.protocolError, 0)
  step(1)
  poke(c.io.reservationAdmission, 0)
  poke(c.io.in.valid, 0)
  poke(c.io.group.groupAllocated, 0)
  poke(c.io.group.dispatchEnable, 0)
  expect(c.io.busy, 0)
  expect(c.io.fenceBusy, 0)
  expect(c.io.fusionFault, 0)

  // A malformed group_last which arrives before LOOP_WS allocation is not
  // captured.  Allocation alone is not enough either: aborting before the
  // Gemmini children reach RS/SharedDeps could release and reuse the group
  // while those children are still being generated.
  drive(group = 4, grouped = true, last = true, malformed = true)
  expect(c.io.group.pending, 1)
  expect(c.io.out.valid, 0)
  expect(c.io.in.ready, 0)
  step(1)
  poke(c.io.group.groupAllocated, 1)
  expect(c.io.out.valid, 0)
  expect(c.io.in.ready, 0)
  expect(c.io.group.abort, 0)
  step(1)
  // Gemmini member stream completion makes dispatchEnable true. The malformed
  // command can now take the abort path without releasing the group prematurely.
  poke(c.io.group.dispatchEnable, 1)
  expect(c.io.out.valid, 1)
  expect(c.io.in.ready, 1)
  expect(c.io.group.abort, 1)
  step(1)
  poke(c.io.in.valid, 0)
  poke(c.io.group.groupAllocated, 0)
  poke(c.io.group.dispatchEnable, 0)
  expect(c.io.busy, 0)
  clearFusionFault()

  // A core-level decode/range rejection is distinct from malformed transport
  // metadata, but must still terminate its group instead of dropping last.
  drive(group = 5, grouped = true, last = true)
  poke(c.io.group.groupAllocated, 1)
  poke(c.io.group.dispatchEnable, 1)
  poke(c.io.commandRejected, 1)
  expect(c.io.out.valid, 1)
  expect(c.io.group.pending, 1)
  expect(c.io.group.abort, 1)
  expect(c.io.group.lastDispatchFire, 0)
  step(1)
  poke(c.io.commandRejected, 0)
  poke(c.io.in.valid, 0)
  poke(c.io.group.dispatchEnable, 0)
  poke(c.io.group.groupAllocated, 0)
  expect(c.io.busy, 0)
  clearFusionFault()

  // Changing IDs inside an open sequence is forwarded as a malformed core
  // command and aborts the original group instead of waiting forever.
  poke(c.io.group.groupAllocated, 1)
  poke(c.io.group.dispatchEnable, 1)
  drive(group = 1, grouped = true, last = false)
  expect(c.io.out.valid, 1)
  step(1)
  // The live group remains dispatch-enabled after its Gemmini children have
  // registered.  A mismatching ID aborts that group through the same enabled
  // path; it must not bypass the registration barrier.
  drive(group = 3, grouped = true, last = false)
  expect(c.io.out.valid, 1)
  expect(c.io.out.bits.microOp, BigInt(VpuEncoding.pack(VpuOpcode.V_ADD_VF) |
    0x80000000L))
  expect(c.io.group.groupId, 1)
  expect(c.io.group.abort, 1)
  expect(c.io.protocolError, 1)
  step(1)
  poke(c.io.group.groupAllocated, 0)
  poke(c.io.group.dispatchEnable, 0)
  // The aborted sequence is drained through its terminator instead of
  // allowing the next grouped command to wait on a released group forever.
  // An xd command is forwarded through the malformed path rather than being
  // silently dropped, so Rocket can still receive its expected response.
  drive(group = 3, grouped = true, last = false,
    opcode = VpuOpcode.C_READ, xd = true)
  expect(c.io.in.ready, 1)
  expect(c.io.out.valid, 1)
  expect(c.io.out.bits.microOp, BigInt(VpuEncoding.pack(VpuOpcode.C_READ) |
    0x80000000L))
  step(1)
  // An unrelated grouped terminator is reported through the malformed path,
  // but must neither be dropped locally nor end group 3's drain.
  drive(group = 4, grouped = true, last = true)
  expect(c.io.in.ready, 1)
  expect(c.io.out.valid, 1)
  expect(c.io.out.bits.microOp, BigInt(VpuEncoding.pack(VpuOpcode.V_ADD_VF) |
    0x80000000L))
  step(1)
  expect(c.io.busy, 1)
  expect(c.io.fenceBusy, 0)
  drive(group = 3, grouped = true, last = true)
  expect(c.io.in.ready, 1)
  expect(c.io.out.valid, 0)
  step(1)
  poke(c.io.in.valid, 0)
  expect(c.io.busy, 0)

  // A sticky fusion fault prevents a later grouped sequence from waiting on
  // or mutating data until software explicitly clears the status.
  poke(c.io.group.groupAllocated, 1)
  poke(c.io.group.dispatchEnable, 1)
  drive(group = 6, grouped = true, last = true)
  expect(c.io.out.valid, 1)
  expect(c.io.group.abort, 1)
  step(1)
  poke(c.io.in.valid, 0)
  poke(c.io.group.groupAllocated, 0)
  poke(c.io.group.dispatchEnable, 0)
  clearFusionFault()

  // The clear restores normal grouped admission.
  poke(c.io.group.groupAllocated, 1)
  poke(c.io.group.dispatchEnable, 1)
  drive(group = 7, grouped = true, last = true)
  poke(c.io.reservationAdmission, 1)
  expect(c.io.group.lastDispatchFire, 1)
  expect(c.io.group.abort, 0)
  step(1)
  poke(c.io.reservationAdmission, 0)
  poke(c.io.in.valid, 0)
  poke(c.io.group.groupAllocated, 0)
  poke(c.io.group.dispatchEnable, 0)
  expect(c.io.busy, 0)
}

class VpuGroupCommandGateSpec extends ChiselFlatSpec {
  behavior of "VpuGroupCommandGate"

  it should "gate grouped sequences and release only on core admission" in {
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-group-command-gate"),
      () => new VpuGroupCommandGate(3)) { c =>
      new VpuGroupCommandGateTester(c)
    } should be(true)
  }
}
