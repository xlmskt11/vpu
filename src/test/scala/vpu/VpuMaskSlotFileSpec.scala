package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuMaskSlotFileTester(c: VpuMaskSlotFile, p: VpuParams)
    extends PeekPokeTester(c) {
  require(p.vLen == 128 && p.nLanes == 16 && p.maskSlots == 4)

  private val lowA = BigInt("aaaaaaaaaaaaaaaa", 16)
  private val highA = BigInt("5555555555555555", 16)
  private val lowB = BigInt("1111111111111111", 16)
  private val highB = BigInt("2222222222222222", 16)
  private val lowC = BigInt("3333333333333333", 16)
  private val highC = BigInt("4444444444444444", 16)
  private val lowD = BigInt("7777777777777777", 16)
  private val highD = BigInt("8888888888888888", 16)
  private val lowE = BigInt("9999999999999999", 16)

  poke(c.io.write.valid, 0)
  poke(c.io.write.bits.chunk, 0)
  poke(c.io.write.bits.data, 0)
  poke(c.io.slotsInUse, 0)
  poke(c.io.activeSlot, 0)
  poke(c.io.activeWordIndex, 0)
  step(2)

  private def currentSlot: Int = peek(c.io.currentSlot).toInt

  private def writeChunk(chunk: Int, data: BigInt,
                         shouldBeReady: Boolean = true): Unit = {
    poke(c.io.write.bits.chunk, chunk)
    poke(c.io.write.bits.data, data)
    poke(c.io.write.valid, 1)
    assert(peek(c.io.write.ready) == (if (shouldBeReady) 1 else 0),
      s"mask chunk $chunk readiness did not match protected-slot capacity")
    if (shouldBeReady) step(1)
    poke(c.io.write.valid, 0)
  }

  private def maskAt(slot: Int): BigInt = {
    poke(c.io.activeSlot, slot)
    (0 until p.wordsPerVector).foldLeft(BigInt(0)) { (mask, word) =>
      poke(c.io.activeWordIndex, word)
      mask | (peek(c.io.activeWord) << (word * p.nLanes))
    }
  }

  private def expected(high: BigInt, low: BigInt): BigInt =
    (high << 64) | low

  // Build the first architectural mask in the unreferenced reset slot.
  writeChunk(0, lowA)
  writeChunk(1, highA)
  val slotA = currentSlot
  assert(maskAt(slotA) == expected(highA, lowA))

  // A later C_WRITE must clone rather than overwrite a slot referenced by a
  // queued/active execute command.
  poke(c.io.slotsInUse, BigInt(1) << slotA)
  writeChunk(0, lowB)
  val slotB = currentSlot
  assert(slotB != slotA, "copy-on-write reused a referenced mask slot")
  assert(maskAt(slotA) == expected(highA, lowA),
    "copy-on-write changed an older queued command's mask")
  writeChunk(1, highB)
  assert(maskAt(slotB) == expected(highB, lowB))

  // Reconstructing an existing version may switch directly back to its slot.
  // One unreferenced scratch version is enough for all intermediate chunks.
  poke(c.io.slotsInUse, (BigInt(1) << slotA) | (BigInt(1) << slotB))
  writeChunk(0, lowA)
  writeChunk(1, highA)
  assert(currentSlot == slotA,
    "identical RoPE mask version was not deduplicated")

  // Streamed datapaths read only one nLanes-wide word from the selected slot.
  poke(c.io.activeSlot, slotA)
  poke(c.io.activeWordIndex, 0)
  assert(peek(c.io.activeWord) == 0xaaaa)
  poke(c.io.activeWordIndex, 5)
  assert(peek(c.io.activeWord) == 0x5555)

  // Occupy four distinct versions. A fifth unique version must backpressure
  // rather than overwrite any referenced slot, then proceed once one retires.
  writeChunk(0, lowC)
  val slotC = currentSlot
  writeChunk(1, highC)
  var inUse = (BigInt(1) << slotA) | (BigInt(1) << slotB) |
    (BigInt(1) << slotC)
  poke(c.io.slotsInUse, inUse)

  writeChunk(0, lowD)
  val slotD = currentSlot
  writeChunk(1, highD)
  inUse |= BigInt(1) << slotD
  assert(inUse == 0xf, "test did not occupy all four mask slots")
  poke(c.io.slotsInUse, inUse)
  writeChunk(0, lowE, shouldBeReady = false)

  inUse &= ~(BigInt(1) << slotB)
  poke(c.io.slotsInUse, inUse)
  writeChunk(0, lowE)
  assert(currentSlot == slotB,
    "released mask slot was not recycled for a new version")
  assert(maskAt(slotA) == expected(highA, lowA))
  assert(maskAt(slotC) == expected(highC, lowC))
  assert(maskAt(slotD) == expected(highD, lowD))
}

class VpuMaskSlotFileSpec extends ChiselFlatSpec {
  behavior of "VpuMaskSlotFile"

  it should "version arbitrary masks without replicating them in execute RS entries" in {
    val p = VpuParams(maskSlots = 4)
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-mask-slot-file"),
      () => new VpuMaskSlotFile(p)) { c =>
      new VpuMaskSlotFileTester(c, p)
    } should be (true)
  }
}
