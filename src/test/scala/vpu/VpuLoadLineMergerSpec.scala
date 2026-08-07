package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuLoadLineMergerTester(c: VpuLoadLineMerger, p: VpuParams)
    extends PeekPokeTester(c) {
  require(p.nLanes == 16 && p.dmaElementsPerBeat == 4,
    "this directed test assumes sixteen FP32 lanes and four DMA elements")

  poke(c.io.descriptor.valid, 0)
  poke(c.io.descriptor.bits.commandTag, 0)
  poke(c.io.descriptor.bits.spadElement, 0)
  poke(c.io.descriptor.bits.elementCount, 0)
  poke(c.io.descriptor.bits.rowCount, 0)
  poke(c.io.in.valid, 0)
  poke(c.io.in.bits.data, 0)
  poke(c.io.in.bits.elementMask, 0)
  poke(c.io.in.bits.spadElement, 0)
  poke(c.io.in.bits.commandTag, 0)
  poke(c.io.in.bits.last, 0)
  poke(c.io.in.bits.error, 0)
  poke(c.io.in.bits.fault.vaddr, 0)
  poke(c.io.in.bits.fault.cause, 0)
  poke(c.io.in.bits.fault.isWrite, 0)
  poke(c.io.out.ready, 0)
  step(2)

  private def pack(values: Seq[Int]): BigInt = {
    require(values.size == p.dmaElementsPerBeat)
    values.reverse.foldLeft(BigInt(0)) { (acc, value) =>
      (acc << p.storageBits) | (BigInt(value) &
        ((BigInt(1) << p.storageBits) - 1))
    }
  }

  private def descriptor(tag: Int, base: Int, elements: Int,
      rows: Int = 1): Unit = {
    poke(c.io.descriptor.bits.commandTag, tag)
    poke(c.io.descriptor.bits.spadElement, base)
    poke(c.io.descriptor.bits.elementCount, elements)
    poke(c.io.descriptor.bits.rowCount, rows)
    poke(c.io.descriptor.valid, 1)
    var timeout = 50
    while (peek(c.io.descriptor.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"descriptor tag=$tag timed out")
    step(1)
    poke(c.io.descriptor.valid, 0)
  }

  private def fragment(tag: Int, address: Int, values: Seq[Int],
      mask: Int = 0xf, last: Boolean = false): Unit = {
    poke(c.io.in.bits.data, pack(values))
    poke(c.io.in.bits.elementMask, mask)
    poke(c.io.in.bits.spadElement, address)
    poke(c.io.in.bits.commandTag, tag)
    poke(c.io.in.bits.last, if (last) 1 else 0)
    poke(c.io.in.bits.error, 0)
    poke(c.io.in.bits.fault.vaddr, 0)
    poke(c.io.in.bits.fault.cause, 0)
    poke(c.io.in.bits.fault.isWrite, 0)
    poke(c.io.in.valid, 1)
    var timeout = 100
    while (peek(c.io.in.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"fragment tag=$tag address=$address timed out")
    step(1)
    poke(c.io.in.valid, 0)
  }

  private def waitForOutput(): Unit = {
    var timeout = 100
    while (peek(c.io.out.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "merged load word timed out")
  }

  // A 20-element command contains one full word and one four-lane tail.  Let
  // the tail arrive first, then deliver the full word's four DMA fragments in
  // a deliberately non-monotonic order.
  descriptor(tag = 2, base = 0, elements = 20)
  fragment(tag = 2, address = 16, values = Seq(16, 17, 18, 19))
  waitForOutput()
  assert(peek(c.io.out.bits.address) == 16)
  assert(peek(c.io.out.bits.last) == 0,
    "the highest-address word is not necessarily the retirement word")
  for (lane <- 0 until p.nLanes) {
    assert(peek(c.io.out.bits.laneMask(lane)) ==
      (if (lane < 4) 1 else 0))
    if (lane < 4) {
      assert(peek(c.io.out.bits.data(lane)) == 16 + lane)
    }
  }

  fragment(tag = 2, address = 8, values = Seq(8, 9, 10, 11))
  fragment(tag = 2, address = 0, values = Seq(0, 1, 2, 3))
  fragment(tag = 2, address = 12, values = Seq(12, 13, 14, 15))
  fragment(tag = 2, address = 4, values = Seq(4, 5, 6, 7),
    last = true)

  // Backpressure must hold the already assembled tail stable while unrelated
  // fragments continue entering the keyed merge table.
  for (_ <- 0 until 3) {
    assert(peek(c.io.out.valid) == 1)
    assert(peek(c.io.out.bits.address) == 16)
    assert(peek(c.io.out.bits.commandTag) == 2)
    step(1)
  }

  // Consume the tail.  The output register rolls directly to the full word in
  // the same cycle, and only that actually-final emitted word carries last.
  poke(c.io.out.ready, 1)
  step(1)
  poke(c.io.out.ready, 0)
  waitForOutput()
  assert(peek(c.io.out.bits.address) == 0)
  assert(peek(c.io.out.bits.commandTag) == 2)
  assert(peek(c.io.out.bits.last) == 1)
  assert(peek(c.io.out.bits.error) == 0)
  for (lane <- 0 until p.nLanes) {
    assert(peek(c.io.out.bits.laneMask(lane)) == 1)
    assert(peek(c.io.out.bits.data(lane)) == lane,
      s"merged full-word lane $lane was corrupted")
  }
  poke(c.io.out.ready, 1)
  step(1)
  poke(c.io.out.ready, 0)
  step(1)
  assert(peek(c.io.busy) == 0,
    "descriptor remained live after its final merged word")

  // A 2-D descriptor keeps each four-element row in its own VLEN slot.  The
  // inactive lanes and the gap between rows must never become expected data.
  descriptor(tag = 1, base = 64, elements = 4, rows = 2)
  fragment(tag = 1, address = 64, values = Seq(60, 61, 62, 63))
  fragment(tag = 1, address = 64 + p.vLen,
    values = Seq(70, 71, 72, 73), last = true)
  waitForOutput()
  assert(peek(c.io.out.bits.address) == 64)
  assert(peek(c.io.out.bits.last) == 0)
  for (lane <- 0 until p.nLanes) {
    assert(peek(c.io.out.bits.laneMask(lane)) ==
      (if (lane < 4) 1 else 0))
  }
  poke(c.io.out.ready, 1)
  step(1)
  poke(c.io.out.ready, 0)
  waitForOutput()
  assert(peek(c.io.out.bits.address) == 64 + p.vLen)
  assert(peek(c.io.out.bits.last) == 1)
  poke(c.io.out.ready, 1)
  step(1)
  poke(c.io.out.ready, 0)
  step(1)
  assert(peek(c.io.busy) == 0,
    "2-D descriptor remained live after both rows")

  // A terminal error discards a partially assembled word and is itself held
  // as an irrevocable, tagged completion.
  descriptor(tag = 3, base = 32, elements = 16)
  fragment(tag = 3, address = 32, values = Seq(40, 41, 42, 43))
  poke(c.io.in.bits.data, 0)
  poke(c.io.in.bits.elementMask, 0)
  poke(c.io.in.bits.spadElement, 0)
  poke(c.io.in.bits.commandTag, 3)
  poke(c.io.in.bits.last, 1)
  poke(c.io.in.bits.error, 1)
  poke(c.io.in.bits.fault.vaddr, BigInt("12345678", 16))
  poke(c.io.in.bits.fault.cause, VpuDmaFaultCause.Access)
  poke(c.io.in.bits.fault.isWrite, 0)
  poke(c.io.in.valid, 1)
  while (peek(c.io.in.ready) == 0) { step(1) }
  step(1)
  poke(c.io.in.valid, 0)
  waitForOutput()
  assert(peek(c.io.out.bits.commandTag) == 3)
  assert(peek(c.io.out.bits.last) == 1)
  assert(peek(c.io.out.bits.error) == 1)
  assert(peek(c.io.out.bits.fault.vaddr) == BigInt("12345678", 16))
  poke(c.io.out.ready, 1)
  step(1)
  poke(c.io.out.ready, 0)
  step(1)
  assert(peek(c.io.busy) == 0,
    "faulted descriptor left a partial merge entry live")
}

/** Reproduce the full-SoC descriptor geometry and response ordering which
  * originally exposed a stale completed merge entry: 16 rows, VL=125, a
  * cache-line base offset of four bytes, and eight-source response reordering.
  */
class VpuLoadLineMergerMaxRowsTester(c: VpuLoadLineMerger, p: VpuParams)
    extends PeekPokeTester(c) {
  require(p.vLen == 128 && p.nLanes == 16 &&
    p.dmaElementsPerBeat == 4 && p.dmaMaxRows == 16)

  private case class Fragment(address: Int, elements: Int)
  private case class Transaction(fragments: Seq[Fragment])

  poke(c.io.descriptor.valid, 0)
  poke(c.io.in.valid, 0)
  poke(c.io.in.bits.data, 0)
  poke(c.io.in.bits.elementMask, 0)
  poke(c.io.in.bits.spadElement, 0)
  poke(c.io.in.bits.commandTag, 0)
  poke(c.io.in.bits.last, 0)
  poke(c.io.in.bits.error, 0)
  poke(c.io.in.bits.fault.vaddr, 0)
  poke(c.io.in.bits.fault.cause, 0)
  poke(c.io.in.bits.fault.isWrite, 0)
  poke(c.io.out.ready, 0)
  step(2)

  val commandTag = 0
  val rowElements = 125
  val hostBase = 4
  val hostStrideBytes = (p.vLen + 7) * p.storageBytes

  poke(c.io.descriptor.bits.commandTag, commandTag)
  poke(c.io.descriptor.bits.spadElement, 0)
  poke(c.io.descriptor.bits.elementCount, rowElements)
  poke(c.io.descriptor.bits.rowCount, p.dmaMaxRows)
  poke(c.io.descriptor.valid, 1)
  while (peek(c.io.descriptor.ready) == 0) { step(1) }
  step(1)
  poke(c.io.descriptor.valid, 0)

  // Match VpuTLUtil.bestAlignedTransaction and the reader's lane-boundary
  // fragment splitter. Transactions may complete out of order, but fragments
  // within one completed transaction retain their order.
  private val transactions = (0 until p.dmaMaxRows).flatMap { row =>
    var host = hostBase + row * hostStrideBytes
    var local = row * p.vLen
    var remaining = rowElements * p.storageBytes
    val rowTransactions = scala.collection.mutable.ArrayBuffer.empty[Transaction]
    while (remaining > 0) {
      val candidates = Seq(16, 32, 64).map { bytes =>
        val shift = host & (bytes - 1)
        val useful = math.min(remaining, bytes - shift)
        (bytes, useful)
      }
      // Ties retain the smaller candidate, as in reduceLeft's strict `>`.
      val (_, usefulBytes) = candidates.reduceLeft { (old, next) =>
        if (next._2 > old._2) next else old
      }
      var transactionRemaining = usefulBytes
      val fragments = scala.collection.mutable.ArrayBuffer.empty[Fragment]
      while (transactionRemaining > 0) {
        val laneElements = p.nLanes - (local % p.nLanes)
        val takeBytes = math.min(transactionRemaining,
          math.min(p.dmaBusWidth / 8, laneElements * p.storageBytes))
        val elements = takeBytes / p.storageBytes
        fragments += Fragment(local, elements)
        local += elements
        transactionRemaining -= takeBytes
      }
      rowTransactions += Transaction(fragments.toSeq)
      host += usefulBytes
      remaining -= usefulBytes
    }
    rowTransactions.toSeq
  }
  // Eight sources can be live. Reverse each reservation window to model the
  // worst legal D-channel completion order without inventing an impossible
  // response for a transaction which has not yet been reserved.
  private val fragments = transactions.grouped(p.dmaMaxInFlight)
    .flatMap(_.reverse).flatMap(_.fragments).toSeq

  val expectedAddresses = (for {
    row <- 0 until p.dmaMaxRows
    word <- 0 until ((rowElements + p.nLanes - 1) / p.nLanes)
  } yield row * p.vLen + word * p.nLanes).toSet
  val observed = scala.collection.mutable.Set.empty[Int]
  var observedLast = 0
  var cycles = 0

  def outputReady: Boolean = cycles % 13 < 8
  def advance(): Unit = {
    val ready = outputReady
    poke(c.io.out.ready, if (ready) 1 else 0)
    if (ready && peek(c.io.out.valid) != 0) {
      val address = peek(c.io.out.bits.address).toInt
      assert(expectedAddresses.contains(address),
        s"unexpected max-row output address $address")
      assert(!observed.contains(address),
        s"max-row output address $address was emitted twice")
      observed += address
      val rowOffset = address % p.vLen
      val expectedLanes = math.min(p.nLanes, rowElements - rowOffset)
      for (lane <- 0 until p.nLanes) {
        assert(peek(c.io.out.bits.laneMask(lane)) ==
          (if (lane < expectedLanes) 1 else 0))
      }
      if (peek(c.io.out.bits.last) != 0) { observedLast += 1 }
    }
    step(1)
    cycles += 1
    assert(cycles < 20000, "max-row merger regression timed out")
  }

  for ((fragment, index) <- fragments.zipWithIndex) {
    val mask = (1 << fragment.elements) - 1
    val lanes = (0 until p.dmaElementsPerBeat).map { lane =>
      if (lane < fragment.elements) fragment.address + lane else 0
    }
    val data = lanes.reverse.foldLeft(BigInt(0)) { (acc, value) =>
      (acc << p.storageBits) | (BigInt(value) &
        ((BigInt(1) << p.storageBits) - 1))
    }
    poke(c.io.in.bits.data, data)
    poke(c.io.in.bits.elementMask, mask)
    poke(c.io.in.bits.spadElement, fragment.address)
    poke(c.io.in.bits.commandTag, commandTag)
    poke(c.io.in.bits.last, if (index == fragments.size - 1) 1 else 0)
    poke(c.io.in.valid, 1)
    while (peek(c.io.in.ready) == 0) { advance() }
    advance()
    poke(c.io.in.valid, 0)
  }

  while (peek(c.io.busy) != 0) { advance() }
  assert(observed == expectedAddresses,
    s"max-row merger emitted ${observed.size}/${expectedAddresses.size} words")
  assert(observedLast == 1,
    s"max-row merger emitted $observedLast command-last words")
}

class VpuLoadLineMergerSpec extends ChiselFlatSpec {
  behavior of "VpuLoadLineMerger"

  it should "merge out-of-order DMA fragments into one VSRAM write per word" in {
    val p = VpuParams(vLen = 32, nLanes = 16, sfuLanes = 4)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-load-line-merger"),
      () => new VpuLoadLineMerger(p)) { c =>
      new VpuLoadLineMergerTester(c, p)
    } should be (true)
  }

  it should "drain a maximum-row misaligned descriptor under reordering and backpressure" in {
    val p = VpuParams()
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-load-line-merger-max-rows"),
      () => new VpuLoadLineMerger(p)) { c =>
      new VpuLoadLineMergerMaxRowsTester(c, p)
    } should be (true)
  }
}
