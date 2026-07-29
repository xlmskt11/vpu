package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

/** End-to-end BF16-storage check. The reference intentionally rounds after
  * every vector micro-op, matching the architectural SRAM write boundary.
  */
class VpuBf16CoreTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  private def bf16Bits(value: Float): Int = {
    val raw = java.lang.Float.floatToRawIntBits(value)
    val magnitude = raw & 0x7fffffff
    if (java.lang.Integer.compareUnsigned(magnitude, 0x7f800000) > 0) {
      0x7fc0
    } else {
      ((raw + 0x7fff + ((raw >>> 16) & 1)) >>> 16) & 0xffff
    }
  }

  private def bf16Value(raw: Int): Float =
    java.lang.Float.intBitsToFloat((raw & 0xffff) << 16)

  private def roundBf16(value: Float): Float = bf16Value(bf16Bits(value))

  private def pack(values: Seq[Int]): BigInt =
    values.zipWithIndex.map { case (value, lane) =>
      BigInt(value & 0xffff) << (16 * lane)
    }.foldLeft(BigInt(0))(_ | _)

  poke(c.io.command.valid, 0)
  poke(c.io.response.ready, 0)
  poke(c.io.dma.readDescriptor.ready, 0)
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

  private var currentReadTag = BigInt(0)

  private def issue(opcode: Int, rd: Int = 0, rs1: Int = 0, rs2: Int = 0,
                    funct1: Int = 0, payload: BigInt = 0,
                    xd: Boolean = false, roccRd: Int = 0): Unit = {
    poke(c.io.command.valid, 1)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd, rs1, rs2,
        funct1 = funct1) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, xd)
    poke(c.io.command.bits.status.dprv, 3)
    poke(c.io.command.bits.status.prv, 3)
    var timeout = 500
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"command 0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  private def acceptReadDescriptor(base: Int, count: Int): Unit = {
    var timeout = 500
    while (peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"read descriptor base=$base timed out")
    assert(peek(c.io.dma.readDescriptor.bits.spadElement) == base)
    assert(peek(c.io.dma.readDescriptor.bits.elementCount) == count)
    currentReadTag = peek(c.io.dma.readDescriptor.bits.commandTag)
    poke(c.io.dma.readDescriptor.ready, 1)
    step(1)
    poke(c.io.dma.readDescriptor.ready, 0)
  }

  private def sendReadBeat(base: Int, values: Seq[Int], mask: Int,
                           last: Boolean): Unit = {
    require(values.size == p.dmaElementsPerBeat)
    poke(c.io.dma.readData.valid, 1)
    poke(c.io.dma.readData.bits.data, pack(values))
    poke(c.io.dma.readData.bits.elementMask, mask)
    poke(c.io.dma.readData.bits.spadElement, base)
    poke(c.io.dma.readData.bits.commandTag, currentReadTag)
    poke(c.io.dma.readData.bits.last, if (last) 1 else 0)
    var timeout = 500
    while (peek(c.io.dma.readData.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "read beat timed out")
    step(1)
    poke(c.io.dma.readData.valid, 0)
  }

  private def waitResponse(expectedRd: Int): BigInt = {
    var timeout = 3000
    while (peek(c.io.response.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "fence response timed out")
    assert(peek(c.io.response.bits.rd) == expectedRd)
    val result = peek(c.io.response.bits.data)
    poke(c.io.response.ready, 1)
    step(1)
    poke(c.io.response.ready, 0)
    result
  }

  val bank = p.elementsPerBank
  val inputBase = 0
  val negBase = bank
  val expBase = 2 * bank
  val denominatorBase = 3 * bank
  val reciprocalBase = 4 * bank
  val outputBase = 5 * bank
  val sentinel = bf16Bits(37.5f)
  val active = Seq(-2.0f, -1.0f, 0.0f, 1.0f, 2.0f)
    .map(roundBf16)

  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = inputBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = negBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = expBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 3, payload = denominatorBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 4, payload = reciprocalBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 5, payload = outputBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 6, payload = 0)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = BigInt("100000", 16))
  issue(VpuOpcode.C_WRITE_FP, rd = 0, payload = bits(0.0f))
  issue(VpuOpcode.C_WRITE_FP, rd = 1, payload = bits(1.0f))

  // Seed all sixteen output elements so lanes beyond VL detect any tail write.
  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.H_PREFETCH_V, rd = 5, rs1 = 6, rs2 = 0)
  acceptReadDescriptor(outputBase, p.vLen)
  sendReadBeat(outputBase, Seq.fill(p.dmaElementsPerBeat)(sentinel),
    mask = 0xff, last = false)
  sendReadBeat(outputBase + p.dmaElementsPerBeat,
    Seq.fill(p.dmaElementsPerBeat)(sentinel), mask = 0xff, last = true)

  // Load a five-element tail and execute the fine-grained SiLU sequence.
  issue(VpuOpcode.C_SET_VL, payload = active.size)
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 6, rs2 = 0)
  acceptReadDescriptor(inputBase, active.size)
  val inputBeat = active.map(bf16Bits) ++
    Seq.fill(p.dmaElementsPerBeat - active.size)(0)
  sendReadBeat(inputBase, inputBeat, mask = (1 << active.size) - 1,
    last = true)
  issue(VpuOpcode.V_SUB_VF, rd = 1, rs1 = 0, rs2 = 0, funct1 = 1)
  issue(VpuOpcode.V_EXP_V, rd = 2, rs1 = 1)
  issue(VpuOpcode.V_ADD_VF, rd = 3, rs1 = 2, rs2 = 1)
  issue(VpuOpcode.V_RECI_V, rd = 4, rs1 = 3)
  issue(VpuOpcode.V_MUL_VV, rd = 5, rs1 = 0, rs2 = 4)

  // Read the complete row back: active lanes must match per-uop BF16
  // rounding, while every tail lane must retain the sentinel.
  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.H_STORE_V, rd = 5, rs1 = 6, rs2 = 0)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 9)

  var timeout = 1000
  while (peek(c.io.dma.writeDescriptor.valid) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0, "write descriptor timed out")
  assert(peek(c.io.dma.writeDescriptor.bits.spadElement) == outputBase)
  assert(peek(c.io.dma.writeDescriptor.bits.elementCount) == p.vLen)
  val storeTag = peek(c.io.dma.writeDescriptor.bits.commandTag)
  poke(c.io.dma.writeDescriptor.ready, 1)
  step(1)
  poke(c.io.dma.writeDescriptor.ready, 0)

  val stored = scala.collection.mutable.ArrayBuffer.empty[Int]
  for (beat <- 0 until p.vLen / p.dmaElementsPerBeat) {
    timeout = 1000
    while (peek(c.io.dma.writeData.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"write beat $beat timed out")
    assert(peek(c.io.dma.writeData.bits.elementMask) == 0xff)
    assert(peek(c.io.dma.writeData.bits.commandTag) == storeTag)
    assert(peek(c.io.dma.writeData.bits.last) ==
      (if (beat == 1) 1 else 0))
    val packed = peek(c.io.dma.writeData.bits.data)
    for (lane <- 0 until p.dmaElementsPerBeat) {
      stored += ((packed >> (16 * lane)) & 0xffff).toInt
    }
    poke(c.io.dma.writeData.ready, 1)
    step(1)
    poke(c.io.dma.writeData.ready, 0)
  }
  poke(c.io.dma.writeCompletion.valid, 1)
  poke(c.io.dma.writeCompletion.bits.commandTag, storeTag)
  timeout = 1000
  while (peek(c.io.dma.writeCompletion.ready) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0, "write completion timed out")
  step(1)
  poke(c.io.dma.writeCompletion.valid, 0)

  val fenceStatus = waitResponse(9)
  assert((fenceStatus & 3) == 0, s"unexpected status 0x${fenceStatus.toString(16)}")

  val roundedReference = active.map { x =>
    val neg = roundBf16(0.0f - x)
    val exponential = roundBf16(Math.exp(neg.toDouble).toFloat)
    val denominator = roundBf16(exponential + 1.0f)
    val reciprocal = roundBf16(1.0f / denominator)
    roundBf16(x * reciprocal)
  }
  roundedReference.zipWithIndex.foreach { case (expected, index) =>
    assert(stored(index) == bf16Bits(expected),
      f"BF16 SiLU[$index] raw=0x${stored(index)}%04x expected=0x${bf16Bits(expected)}%04x")
    val mathematical = active(index).toDouble /
      (1.0 + Math.exp(-active(index).toDouble))
    val actual = bf16Value(stored(index)).toDouble
    assert(Math.abs(actual - mathematical) <=
      1.0e-3 + 1.0e-2 * Math.max(Math.abs(actual), Math.abs(mathematical)),
      s"BF16 SiLU[$index]=$actual mathematical=$mathematical")
  }
  stored.drop(active.size).zipWithIndex.foreach { case (raw, index) =>
    assert(raw == sentinel,
      f"tail lane ${active.size + index} changed: 0x$raw%04x")
  }
}

class VpuBf16CoreSpec extends ChiselFlatSpec {
  behavior of "VpuCore BF16 storage"

  it should "round every SiLU micro-op to BF16 and preserve the tail" in {
    val p = VpuParams(storageType = VpuStorageType.BF16,
      vLen = 16, nLanes = 8, sfuLanes = 2, vSpadKB = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-core-bf16"), () => new VpuCore(p)) {
      c => new VpuBf16CoreTester(c, p)
    } should be (true)
  }
}
