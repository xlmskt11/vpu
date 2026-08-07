package vpu

import chisel3._
import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

/** Regression for the store-response credit counter at very small VLEN.
  * Holding DMA output blocked lets the serializer consume one word while all
  * four scratchpad response slots fill behind it.  The outstanding counter
  * must therefore represent the queue depth value itself for both one- and
  * two-word architectural vectors.
  */
private class VpuSmallVectorStoreTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  private val rows = 6
  private val hostLoad = BigInt("500000", 16)
  private val hostStore = BigInt("600000", 16)
  require(p.wordsPerVector == 1 || p.wordsPerVector == 2)
  require(p.dmaMaxRows >= rows)
  require(p.nLanes == p.dmaElementsPerBeat)

  private def leaves(data: Data): Seq[Bits] = data match {
    case bits: Bits => Seq(bits)
    case aggregate: Aggregate => aggregate.getElements.flatMap(leaves)
  }
  private def clear(data: Data): Unit =
    leaves(data).foreach(bits => poke(bits, 0))

  poke(c.io.command.valid, 0)
  clear(c.io.command.bits)
  poke(c.io.response.ready, 0)
  poke(c.io.dma.readDescriptor.ready, 0)
  poke(c.io.dma.readData.valid, 0)
  clear(c.io.dma.readData.bits)
  poke(c.io.dma.writeDescriptor.ready, 0)
  poke(c.io.dma.writeData.ready, 0)
  poke(c.io.dma.writeCompletion.valid, 0)
  clear(c.io.dma.writeCompletion.bits)
  poke(c.io.dma.clearFaultDone, 0)
  poke(c.io.dma.halted, 0)
  step(3)

  private def issue(opcode: Int, rd: Int = 0, rs1: Int = 0,
                    rs2: Int = 0, rs3: Int = 0, funct1: Int = 0,
                    payload: BigInt = 0, xd: Boolean = false,
                    roccRd: Int = 0): Unit = {
    clear(c.io.command.bits)
    poke(c.io.command.bits.microOp, BigInt(VpuEncoding.pack(
      opcode, rd, rs1, rs2, rs3, funct1) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, if (xd) 1 else 0)
    poke(c.io.command.bits.status.dprv, 3)
    poke(c.io.command.bits.status.prv, 3)
    poke(c.io.command.valid, 1)
    var timeout = 500
    while(peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"opcode 0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  private def fence(rd: Int): Unit = {
    issue(VpuOpcode.C_FENCE, xd = true, roccRd = rd)
    var timeout = 2000
    while(peek(c.io.response.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "FENCE response timed out")
    assert(peek(c.io.response.bits.rd) == rd)
    assert((peek(c.io.response.bits.data) & 3) == 0)
    poke(c.io.response.ready, 1)
    step(1)
    poke(c.io.response.ready, 0)
  }

  private val values = (0 until rows).map { row =>
    (0 until p.vLen).map(element => (row * 32 + element + 1).toFloat)
  }

  private def pack(word: Seq[Float]): BigInt =
    word.zipWithIndex.map { case (value, lane) =>
      bits(value) << (lane * p.storageBits)
    }.reduce(_ | _)

  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = 0)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = rows)
  issue(VpuOpcode.C_WRITE_GP, rd = 15, payload = 0)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = hostLoad)
  issue(VpuOpcode.C_SET_STRIDE, payload = 64)
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 15, rs2 = 0, rs3 = 2,
    funct1 = 1)

  var timeout = 500
  while(peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0)
  val loadTag = peek(c.io.dma.readDescriptor.bits.commandTag)
  poke(c.io.dma.readDescriptor.ready, 1)
  step(1)
  poke(c.io.dma.readDescriptor.ready, 0)

  for (row <- 0 until rows; word <- 0 until p.wordsPerVector) {
    clear(c.io.dma.readData.bits)
    poke(c.io.dma.readData.bits.data,
      pack(values(row).slice(word * p.nLanes, (word + 1) * p.nLanes)))
    poke(c.io.dma.readData.bits.elementMask,
      (BigInt(1) << p.dmaElementsPerBeat) - 1)
    poke(c.io.dma.readData.bits.spadElement,
      row * p.vLen + word * p.nLanes)
    poke(c.io.dma.readData.bits.commandTag, loadTag)
    poke(c.io.dma.readData.bits.last,
      if (row == rows - 1 && word == p.wordsPerVector - 1) 1 else 0)
    poke(c.io.dma.readData.valid, 1)
    timeout = 500
    while(peek(c.io.dma.readData.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0)
    step(1)
    poke(c.io.dma.readData.valid, 0)
  }
  fence(1)

  issue(VpuOpcode.C_WRITE_H, rd = 1, payload = hostStore)
  issue(VpuOpcode.H_STORE_V, rd = 0, rs1 = 15, rs2 = 1, rs3 = 2,
    funct1 = 1)
  timeout = 500
  while(peek(c.io.dma.writeDescriptor.valid) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0)
  val storeTag = peek(c.io.dma.writeDescriptor.bits.commandTag)
  poke(c.io.dma.writeDescriptor.ready, 1)
  step(1)
  poke(c.io.dma.writeDescriptor.ready, 0)

  // One serializer word plus all four queue slots must become occupied.
  poke(c.io.dma.writeData.ready, 0)
  step(30)
  assert(peek(c.io.dma.writeData.valid) == 1,
    "blocked store never reached its serializer")

  for (row <- 0 until rows; word <- 0 until p.wordsPerVector) {
    timeout = 1000
    while(peek(c.io.dma.writeData.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"store row=$row word=$word timed out")
    if ((row + word) % 2 == 0) {
      poke(c.io.dma.writeData.ready, 0)
      val held = peek(c.io.dma.writeData.bits.data)
      step(2)
      assert(peek(c.io.dma.writeData.valid) == 1)
      assert(peek(c.io.dma.writeData.bits.data) == held)
    }
    poke(c.io.dma.writeData.ready, 1)
    assert(peek(c.io.dma.writeData.bits.commandTag) == storeTag)
    assert(peek(c.io.dma.writeData.bits.spadElement) ==
      row * p.vLen + word * p.nLanes)
    assert(peek(c.io.dma.writeData.bits.data) ==
      pack(values(row).slice(word * p.nLanes, (word + 1) * p.nLanes)))
    assert(peek(c.io.dma.writeData.bits.last) ==
      (if (row == rows - 1 && word == p.wordsPerVector - 1) 1 else 0))
    step(1)
    poke(c.io.dma.writeData.ready, 0)
  }

  clear(c.io.dma.writeCompletion.bits)
  poke(c.io.dma.writeCompletion.bits.commandTag, storeTag)
  poke(c.io.dma.writeCompletion.valid, 1)
  while(peek(c.io.dma.writeCompletion.ready) == 0) step(1)
  step(1)
  poke(c.io.dma.writeCompletion.valid, 0)
  fence(2)
}

class VpuSmallVectorStoreSpec extends ChiselFlatSpec {
  behavior of "VpuCore bounded store-response queue"

  for (words <- Seq(1, 2)) {
    it should s"preserve a six-row store with $words lane words per row" in {
      val p = VpuParams(vLen = 4 * words, nLanes = 4, sfuLanes = 2,
        vSpadKB = 2)
      chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
        "--target-dir", s"test_run_dir/vpu-small-store-$words"),
        () => new VpuCore(p)) { c =>
        new VpuSmallVectorStoreTester(c, p)
      } should be(true)
    }
  }
}
