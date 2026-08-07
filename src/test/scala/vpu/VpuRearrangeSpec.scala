package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuRearrangeTester(c: VpuRearrangeUnit, p: VpuParams)
    extends PeekPokeTester(c) {
  require(p.storageType == VpuStorageType.FP32)

  private val memory = Array.fill[BigInt](p.totalElements)(0)
  poke(c.io.start.valid, 0)
  for (operand <- 0 until 2) {
    poke(c.io.readRequest(operand).ready, 1)
    poke(c.io.readResponse(operand).valid, 0)
  }
  poke(c.io.maskWord, 0)
  poke(c.io.writeRequest.ready, 1)
  step(3)

  private def writeVector(base: Int, values: Seq[BigInt]): Unit =
    values.zipWithIndex.foreach { case (value, index) =>
      memory(base + index) = value
    }

  private def readVector(base: Int): Seq[BigInt] =
    (0 until p.vLen).map(index => memory(base + index))

  private case class PendingRead(data: Seq[BigInt], tag: BigInt)
  private val requestCount = Array.fill(2)(0)
  private val responseCount = Array.fill(2)(0)

  private def run(gather: Boolean, slideLow: Boolean, destination: Int,
                  source: Int, indices: Int = 0, shift: Int = 0,
                  maskEnable: Boolean = false,
                  mask: BigInt = 0): Unit = {
    poke(c.io.start.bits.gather, if (gather) 1 else 0)
    poke(c.io.start.bits.slideLow, if (slideLow) 1 else 0)
    poke(c.io.start.bits.destination, destination)
    poke(c.io.start.bits.source, source)
    poke(c.io.start.bits.indices, indices)
    poke(c.io.start.bits.shift, shift)
    poke(c.io.start.bits.elementCount, p.vLen)
    poke(c.io.start.bits.maskEnable, if (maskEnable) 1 else 0)
    poke(c.io.maskWord, mask & ((BigInt(1) << p.nLanes) - 1))
    poke(c.io.start.valid, 1)
    while (peek(c.io.start.ready) == 0) { step(1) }
    step(1)
    poke(c.io.start.valid, 0)

    val pending = Array.fill[Option[PendingRead]](2)(None)
    var timeout = 10000
    var finished = false
    while (!finished && timeout > 0) {
      val maskWordIndex = peek(c.io.maskWordIndex).toInt
      val maskWord = (mask >> (maskWordIndex * p.nLanes)) &
        ((BigInt(1) << p.nLanes) - 1)
      poke(c.io.maskWord, maskWord)
      for (operand <- 0 until 2) {
        pending(operand) match {
          case Some(response) =>
            poke(c.io.readResponse(operand).valid, 1)
            response.data.zipWithIndex.foreach { case (value, lane) =>
              poke(c.io.readResponse(operand).bits.data(lane), value)
            }
            poke(c.io.readResponse(operand).bits.tag, response.tag)
          case None => poke(c.io.readResponse(operand).valid, 0)
        }
      }

      val responseFire = (0 until 2).map { operand =>
        pending(operand).nonEmpty &&
          peek(c.io.readResponse(operand).ready) == 1
      }
      val newPending = (0 until 2).map { operand =>
        val requestFire = peek(c.io.readRequest(operand).valid) == 1 &&
          peek(c.io.readRequest(operand).ready) == 1
        if (requestFire) {
          assert(pending(operand).isEmpty,
            s"operand $operand issued another read before its response")
          val address = peek(c.io.readRequest(operand).bits.address).toInt
          requestCount(operand) += 1
          Some(PendingRead(
            (0 until p.nLanes).map(lane => memory(address + lane)),
            peek(c.io.readRequest(operand).bits.tag)))
        } else None
      }

      if (peek(c.io.writeRequest.valid) == 1 &&
          peek(c.io.writeRequest.ready) == 1) {
        val address = peek(c.io.writeRequest.bits.address).toInt
        for (lane <- 0 until p.nLanes) {
          if (peek(c.io.writeRequest.bits.laneMask(lane)) == 1) {
            memory(address + lane) = peek(c.io.writeRequest.bits.data(lane))
          }
        }
      }
      finished = peek(c.io.done) == 1
      step(1)
      for (operand <- 0 until 2) {
        if (responseFire(operand)) responseCount(operand) += 1
        pending(operand) = if (responseFire(operand)) newPending(operand)
        else pending(operand).orElse(newPending(operand))
      }
      timeout -= 1
    }
    for (operand <- 0 until 2) {
      poke(c.io.readResponse(operand).valid, 0)
      assert(pending(operand).isEmpty,
        s"operand $operand still had a response when the command completed")
    }
    assert(timeout > 0, "VPU rearrange command timed out")
    assert(peek(c.io.busy) == 0)
  }

  private val source = 0
  private val destination = p.vLen
  private val indices = 2 * p.vLen
  private val original = (1 to p.vLen).map(BigInt(_))

  // Both slide directions cross lane-word boundaries and zero-fill instead
  // of reading outside the architectural source vector.
  writeVector(source, original)
  writeVector(destination, Seq.fill(p.vLen)(BigInt(99)))
  run(gather = false, slideLow = false, destination, source, shift = 3)
  assert(readVector(destination) ==
    (Seq.fill(3)(BigInt(0)) ++ original.take(p.vLen - 3)))

  writeVector(destination, Seq.fill(p.vLen)(BigInt(99)))
  run(gather = false, slideLow = true, destination, source, shift = 5)
  assert(readVector(destination) ==
    (original.drop(5) ++ Seq.fill(5)(BigInt(0))))

  // Direction-aware traversal makes the PLENA-style in-place form safe.
  writeVector(source, original)
  run(gather = false, slideLow = false, source, source, shift = 1)
  assert(readVector(source) == BigInt(0) +: original.dropRight(1))
  writeVector(source, original)
  run(gather = false, slideLow = true, source, source, shift = 1)
  assert(readVector(source) == original.drop(1) :+ BigInt(0))

  // Raw integer indices may duplicate or cross source words. Out-of-range
  // indices return zero, while masked-off destination elements are untouched.
  writeVector(source, original)
  val gatherIndices = Seq[BigInt](15, 0, 4, 16, 8, 8, 2, 14,
    1, 13, 6, 7, 3, 12, 10, 5)
  writeVector(indices, gatherIndices)
  writeVector(destination, Seq.fill(p.vLen)(BigInt(77)))
  val gatherMask = ((BigInt(1) << p.vLen) - 1) & ~(BigInt(1) << 2)
  run(gather = true, slideLow = false, destination, source,
    indices = indices, maskEnable = true, mask = gatherMask)
  val expectedGather = gatherIndices.zipWithIndex.map { case (index, lane) =>
    if (lane == 2) BigInt(77)
    else if (index >= p.vLen) BigInt(0)
    else original(index.toInt)
  }
  assert(readVector(destination) == expectedGather)

  // Slides that cross a lane-word boundary must use and consume both
  // independent operand ports. Gather intentionally remains on operand 0.
  assert(requestCount(0) > 0 && responseCount(0) == requestCount(0))
  assert(requestCount(1) > 0 && responseCount(1) == requestCount(1))
}

class VpuRearrangeSpec extends ChiselFlatSpec {
  behavior of "VpuRearrangeUnit"

  it should "execute masked zero-fill slides and indexed gathers" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2, vSpadKB = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-rearrange"),
      () => new VpuRearrangeUnit(p)) { c =>
      new VpuRearrangeTester(c, p)
    } should be (true)
  }
}
