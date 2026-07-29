package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

/** End-to-end Core integration for streamed SRAM reads and shared-lane reduce. */
class VpuReductionCoreTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  private val count = p.vLen - 3
  private val oneStorage = p.storageType match {
    case VpuStorageType.FP32 => bits(1.0f)
    case VpuStorageType.BF16 => BigInt("3f80", 16)
  }

  poke(c.io.command.valid, 0)
  poke(c.io.response.ready, 0)
  poke(c.io.dma.readDescriptor.ready, 1)
  poke(c.io.dma.readData.valid, 0)
  poke(c.io.dma.readData.bits.error, 0)
  poke(c.io.dma.readData.bits.fault.vaddr, 0)
  poke(c.io.dma.readData.bits.fault.cause, VpuDmaFaultCause.None)
  poke(c.io.dma.readData.bits.fault.isWrite, 0)
  poke(c.io.dma.readData.bits.commandTag, 0)
  poke(c.io.dma.writeDescriptor.ready, 0)
  poke(c.io.dma.writeData.ready, 0)
  poke(c.io.dma.writeCompletion.valid, 0)
  poke(c.io.dma.writeCompletion.bits.commandTag, 0)
  poke(c.io.dma.writeCompletion.bits.error, 0)
  poke(c.io.dma.writeCompletion.bits.fault.vaddr, 0)
  poke(c.io.dma.writeCompletion.bits.fault.cause, VpuDmaFaultCause.None)
  poke(c.io.dma.writeCompletion.bits.fault.isWrite, 0)
  poke(c.io.dma.clearFaultDone, 0)
  poke(c.io.dma.halted, 0)
  step(3)

  def issue(opcode: Int, rd: Int = 0, rs1: Int = 0, rs2: Int = 0,
            payload: BigInt = 0, xd: Boolean = false,
            roccRd: Int = 0): Unit = {
    poke(c.io.command.valid, 1)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd, rs1, rs2) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, if (xd) 1 else 0)
    var timeout = 600
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"command 0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  issue(VpuOpcode.C_SET_VL, payload = count)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = 0) // VSRAM vector base
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = 0) // host element offset
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = BigInt("100000", 16))
  issue(VpuOpcode.C_WRITE_FP, rd = 0, payload = bits(1.0f))
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 1, rs2 = 0)

  var timeout = 100
  while (peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0, "reduction input DMA descriptor was not issued")
  assert(peek(c.io.dma.readDescriptor.bits.elementCount) == count)
  val readTag = peek(c.io.dma.readDescriptor.bits.commandTag)
  step(1) // descriptor ready is held high

  val beats = (count + p.dmaElementsPerBeat - 1) / p.dmaElementsPerBeat
  for (beat <- 0 until beats) {
    val remaining = count - beat * p.dmaElementsPerBeat
    val validElements = math.min(p.dmaElementsPerBeat, remaining)
    val packed = (0 until validElements).foldLeft(BigInt(0)) { (word, lane) =>
      word | (oneStorage << (p.storageBits * lane))
    }
    poke(c.io.dma.readData.valid, 1)
    poke(c.io.dma.readData.bits.data, packed)
    poke(c.io.dma.readData.bits.elementMask,
      (BigInt(1) << validElements) - 1)
    poke(c.io.dma.readData.bits.spadElement,
      beat * p.dmaElementsPerBeat)
    poke(c.io.dma.readData.bits.commandTag, readTag)
    poke(c.io.dma.readData.bits.last, if (beat == beats - 1) 1 else 0)
    while (peek(c.io.dma.readData.ready) == 0) { step(1) }
    step(1)
    poke(c.io.dma.readData.valid, 0)
  }

  // The range scoreboard holds this reduction until the final load write has
  // retired. C_READ then waits on the scalar dependency until the pipelined
  // tree and all three rotating accumulators have completed.
  issue(VpuOpcode.V_RED_SUM, rd = 0, rs1 = 0)
  issue(VpuOpcode.C_READ, rd = 0, rs1 = VpuReadSelector.Fp,
    xd = true, roccRd = 11)

  timeout = 100
  while (peek(c.io.response.valid) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0, "reduction scalar read did not respond")
  assert(peek(c.io.response.bits.rd) == 11)
  assert(peek(c.io.response.bits.data) == bits(count.toFloat + 1.0f),
    s"RED_SUM returned 0x${peek(c.io.response.bits.data).toString(16)}")
  poke(c.io.response.ready, 1)
  step(1)
  poke(c.io.response.ready, 0)
}

class VpuReductionCoreSpec extends ChiselFlatSpec {
  behavior of "VpuCore streamed reduction"

  it should "reduce a multiword FP32 vector through the shared FMA lanes" in {
    val p = VpuParams(vLen = 32, nLanes = 4, sfuLanes = 2, vSpadKB = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-reduction-core-fp32"),
      () => new VpuCore(p)) { c =>
      new VpuReductionCoreTester(c, p)
    } should be (true)
  }

  it should "widen BF16 storage before the same streamed reduction" in {
    val p = VpuParams(storageType = VpuStorageType.BF16,
      vLen = 32, nLanes = 8, sfuLanes = 2, vSpadKB = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-reduction-core-bf16"),
      () => new VpuCore(p)) { c =>
      new VpuReductionCoreTester(c, p)
    } should be (true)
  }
}
