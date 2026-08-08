package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

/** Proves that tile-major VSRAM rows form one logical reduction command. */
private class VpuSegmentedReductionCoreTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  poke(c.io.command.valid, 0)
  poke(c.io.response.ready, 0)
  poke(c.io.dma.readDescriptor.ready, 0)
  poke(c.io.dma.readData.valid, 0)
  poke(c.io.dma.writeDescriptor.ready, 0)
  poke(c.io.dma.writeData.ready, 0)
  poke(c.io.dma.writeCompletion.valid, 0)
  poke(c.io.dma.clearFaultDone, 0)
  poke(c.io.dma.halted, 0)
  poke(c.io.matrixRead.get(0).req.valid, 0)
  poke(c.io.matrixRead.get(0).req.bits.rowAddress, 0)
  poke(c.io.matrixRead.get(0).resp.ready, 0)
  poke(c.io.matrixWrite.get(0).valid, 0)
  poke(c.io.matrixWrite.get(0).bits.rowAddress, 0)
  for (lane <- 0 until p.matrixElementsPerRow) {
    poke(c.io.matrixWrite.get(0).bits.data(lane), 0)
    poke(c.io.matrixWrite.get(0).bits.laneMask(lane), 0)
  }
  step(3)

  private def issue(opcode: Int, rd: Int = 0, rs1: Int = 0,
                    payload: BigInt = 0, xd: Boolean = false,
                    roccRd: Int = 0): Unit = {
    poke(c.io.command.valid, 1)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd = rd, rs1 = rs1) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, if (xd) 1 else 0)
    poke(c.io.command.bits.status.dprv, 3)
    poke(c.io.command.bits.status.prv, 3)
    var timeout = 200
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"command 0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  // Four 8-element matrix rows live 64 elements apart. The 4-lane engine
  // therefore consumes two lane words from each segment, for eight beats in
  // total, while the logical reduction sees one VL=32 stream.
  for (segment <- 0 until 4) {
    poke(c.io.matrixWrite.get(0).valid, 1)
    poke(c.io.matrixWrite.get(0).bits.rowAddress, segment * 8)
    for (lane <- 0 until p.matrixElementsPerRow) {
      poke(c.io.matrixWrite.get(0).bits.data(lane), bits(1.0f))
      poke(c.io.matrixWrite.get(0).bits.laneMask(lane), 1)
    }
    step(1)
  }
  poke(c.io.matrixWrite.get(0).valid, 0)
  step(4)

  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = 0)
  issue(VpuOpcode.C_WRITE_FP, rd = 0, payload = bits(1.0f))
  issue(VpuOpcode.C_SET_VL, payload = 32)
  issue(VpuOpcode.C_SET_VSTRIDE, payload = 64)
  issue(VpuOpcode.V_RED_SUM, rd = 0, rs1 = 0)
  // Resetting the live layout must not alter the command already in the RS.
  issue(VpuOpcode.C_SET_VSTRIDE, payload = 0)
  issue(VpuOpcode.C_READ, rd = 0, rs1 = VpuReadSelector.Fp,
    xd = true, roccRd = 9)

  var timeout = 300
  while (peek(c.io.response.valid) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0, "segmented reduction did not respond")
  assert(peek(c.io.response.bits.rd) == 9)
  assert(peek(c.io.response.bits.data) == bits(33.0f),
    s"segmented RED_SUM returned 0x${peek(c.io.response.bits.data).toString(16)}")
}

class VpuSegmentedReductionCoreSpec extends ChiselFlatSpec {
  behavior of "VpuCore segmented vector addressing"

  it should "reduce tile-major matrix rows with one logical VL" in {
    val p = VpuParams(vLen = 32, nLanes = 4, sfuLanes = 2,
      vSpadKB = 1, vSpadBanks = 1, vSpadSubBanks = 2,
      matrixPorts = 1, matrixRowElements = 8)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-segmented-reduction"),
      () => new VpuCore(p)) { c =>
      new VpuSegmentedReductionCoreTester(c, p)
    } should be (true)
  }
}
