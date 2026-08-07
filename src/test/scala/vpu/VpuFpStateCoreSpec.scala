package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuFpStateCoreTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  poke(c.io.command.valid, 0)
  poke(c.io.response.ready, 0)
  poke(c.io.dma.readDescriptor.ready, 1)
  poke(c.io.dma.readData.valid, 0)
  poke(c.io.dma.writeDescriptor.ready, 1)
  poke(c.io.dma.writeData.ready, 1)
  poke(c.io.dma.writeCompletion.valid, 0)
  poke(c.io.dma.clearFaultDone, 0)
  poke(c.io.dma.halted, 0)
  step(3)

  private def issue(opcode: Int, rd: Int = 0, rs1: Int = 0,
                    rs2: Int = 0, payload: BigInt = 0,
                    xd: Boolean = false, roccRd: Int = 0): Unit = {
    poke(c.io.command.valid, 1)
    poke(c.io.command.bits.microOp, BigInt(VpuEncoding.pack(
      opcode, rd = rd, rs1 = rs1, rs2 = rs2) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, if (xd) 1 else 0)
    var timeout = 200
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"state command 0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  private def response(expectedRd: Int): BigInt = {
    var timeout = 400
    while (peek(c.io.response.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"response rd=$expectedRd timed out")
    expect(c.io.response.bits.rd, expectedRd)
    val result = peek(c.io.response.bits.data)
    poke(c.io.response.ready, 1)
    step(1)
    poke(c.io.response.ready, 0)
    result
  }

  private def fence(rd: Int): BigInt = {
    issue(VpuOpcode.C_FENCE, xd = true, roccRd = rd)
    response(rd)
  }

  // Store snapshots GP[rd] and reads FP[rs1] at execute issue. A following GP
  // update must not redirect the queued store.
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = 17)
  issue(VpuOpcode.C_WRITE_FP, rd = 1, payload = bits(1.25f))
  issue(VpuOpcode.S_STORE_STATE, rd = 0, rs1 = 1)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = 33)

  // Load similarly snapshots GP[rs1]. It writes FP[rd] through the ordinary
  // execute-RS FP dependency mask.
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = 17)
  issue(VpuOpcode.S_LOAD_STATE, rd = 3, rs1 = 2)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = 34)
  assert((fence(5) & 1) == 0, "legal FP state commands set illegal status")
  issue(VpuOpcode.C_READ, rd = 3, rs1 = VpuReadSelector.Fp,
    xd = true, roccRd = 6)
  assert((response(6) & BigInt("ffffffff", 16)) == bits(1.25f),
    "FP state load did not return the value stored at its snapshotted index")

  // Exercise a second bank and confirm an FP control write waits for an older
  // queued state-store reader before changing its source register.
  issue(VpuOpcode.C_WRITE_GP, rd = 4, payload = 18)
  issue(VpuOpcode.C_WRITE_FP, rd = 5, payload = bits(-3.5f))
  issue(VpuOpcode.S_STORE_STATE, rd = 4, rs1 = 5)
  issue(VpuOpcode.C_WRITE_FP, rd = 5, payload = bits(9.0f))
  issue(VpuOpcode.C_WRITE_GP, rd = 6, payload = 18)
  issue(VpuOpcode.S_LOAD_STATE, rd = 7, rs1 = 6)
  fence(7)
  issue(VpuOpcode.C_READ, rd = 7, rs1 = VpuReadSelector.Fp,
    xd = true, roccRd = 8)
  assert((response(8) & BigInt("ffffffff", 16)) == bits(-3.5f),
    "state store observed a younger FP control-register write")

  // Out-of-range state indices and nonzero unused fields are rejected before
  // SRAM access and report through the existing sticky illegal bit.
  issue(VpuOpcode.C_WRITE_GP, rd = 8, payload = p.fpStateEntries)
  issue(VpuOpcode.S_STORE_STATE, rd = 8, rs1 = 5)
  issue(VpuOpcode.S_LOAD_STATE, rd = 0, rs1 = 6, rs2 = 1)
  assert((fence(9) & 1) == 1,
    "malformed FP state commands did not set sticky illegal status")
}

class VpuFpStateCoreSpec extends ChiselFlatSpec {
  behavior of "VpuCore FP state instructions"

  it should "snapshot state indices and preserve FP dependencies" in {
    val p = VpuConfigs.default
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-fp-state-core"),
      () => new VpuCore(p)) { c =>
      new VpuFpStateCoreTester(c, p)
    } should be(true)
  }
}
