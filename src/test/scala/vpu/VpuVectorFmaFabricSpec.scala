package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuVectorFmaFabricTester(c: VpuVectorFmaFabric, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  poke(c.io.aluIn.valid, 0)
  poke(c.io.reciprocalIn.valid, 0)
  poke(c.io.reductionStart, 0)
  poke(c.io.reductionSeed, 0)
  poke(c.io.reductionMaximum, 0)
  poke(c.io.reductionIn.valid, 0)
  for (lane <- 0 until p.nLanes) {
    poke(c.io.aluIn.bits.operandA(lane), 0)
    poke(c.io.aluIn.bits.operandB(lane), 0)
    poke(c.io.aluIn.bits.laneMask(lane), 0)
    poke(c.io.reductionIn.bits.values(lane), 0)
    poke(c.io.reductionIn.bits.laneMask(lane), 0)
  }
  for (lane <- 0 until p.reciprocalLanes) {
    poke(c.io.reciprocalIn.bits.values(lane), bits(1.0f))
  }
  poke(c.io.aluIn.bits.operation, VpuFpAluOp.Add.litValue)
  poke(c.io.aluIn.bits.destination, 0)
  poke(c.io.aluIn.bits.last, 0)
  poke(c.io.reductionIn.bits.last, 0)
  step(3)

  def startReduction(seed: BigInt, maximum: Boolean): Unit = {
    assert(peek(c.io.reductionBusy) == 0)
    poke(c.io.reductionSeed, seed)
    poke(c.io.reductionMaximum, if (maximum) 1 else 0)
    poke(c.io.reductionStart, 1)
    step(1)
    poke(c.io.reductionStart, 0)
    assert(peek(c.io.reductionBusy) == 1)
  }

  def sendWord(values: Seq[BigInt], mask: Seq[Boolean], last: Boolean): Unit = {
    require(values.size == p.nLanes && mask.size == p.nLanes)
    values.zipWithIndex.foreach { case (value, lane) =>
      poke(c.io.reductionIn.bits.values(lane), value)
      poke(c.io.reductionIn.bits.laneMask(lane), if (mask(lane)) 1 else 0)
    }
    poke(c.io.reductionIn.bits.last, if (last) 1 else 0)
    poke(c.io.reductionIn.valid, 1)
    step(1)
    poke(c.io.reductionIn.valid, 0)
  }

  def awaitReduction(expected: BigInt,
                     expectedFlags: Option[BigInt] = Some(0)): Unit = {
    var timeout = 300
    while (peek(c.io.reductionOut.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "shared-lane reduction timed out")
    assert(peek(c.io.reductionOut.bits.data) == expected,
      f"reduction data=0x${peek(c.io.reductionOut.bits.data)}%x expected=0x$expected%x")
    expectedFlags.foreach { flags =>
      assert(peek(c.io.reductionOut.bits.fflags) == flags,
        f"reduction flags=0x${peek(c.io.reductionOut.bits.fflags)}%x expected=0x$flags%x")
    }
    step(1)
    assert(peek(c.io.reductionBusy) == 0)
  }

  // Seven back-to-back words make every rotating accumulator recur at least
  // once. All terms are exact integers, so a missing distance-three forwarding
  // edge cannot hide behind a different legal floating-point rounding order.
  startReduction(bits(1.0f), maximum = false)
  for (word <- 0 until 7) {
    sendWord(Seq.fill(p.nLanes)(bits(1.0f)),
      Seq.fill(p.nLanes)(true), last = word == 6)
  }
  awaitReduction(bits(1.0f + 7.0f * p.nLanes.toFloat))

  // RED_MAX propagates a canonical NaN and raises NV for an active sNaN.
  startReduction(bits(Float.NegativeInfinity), maximum = true)
  sendWord(Seq(bits(1.0f), BigInt("7fc12345", 16),
    BigInt("7f800001", 16), bits(5.0f)), Seq.fill(p.nLanes)(true), last = true)
  awaitReduction(VpuFloat.canonicalNaN,
    expectedFlags = Some(BigInt("10", 16)))

  // A stale signaling NaN in an inactive tail is exactly neutral.
  startReduction(bits(Float.NegativeInfinity), maximum = true)
  sendWord(Seq(bits(3.0f), BigInt("7f800001", 16),
    BigInt("7f800001", 16), BigInt("7f800001", 16)),
    Seq(true, false, false, false), last = true)
  awaitReduction(bits(3.0f))

  // Compare randomized finite sums bit-for-bit with the architectural
  // rounding order: balanced tree per word, three rotating accumulators, then
  // ((acc0 + acc1) + acc2). NaN payloads are intentionally outside this check.
  def balanced(values: Seq[Float]): Float = {
    if (values.size == 1) values.head
    else balanced(values.grouped(2).map {
      case Seq(a, b) => (a + b).toFloat
      case Seq(a) => a
    }.toSeq)
  }
  val random = new scala.util.Random(0x524f5441L)
  for (_ <- 0 until 12) {
    val seed = (random.nextFloat() * 20.0f - 10.0f).toFloat
    val words = Seq.fill(7)(Seq.fill(p.nLanes) {
      (random.nextFloat() * 20.0f - 10.0f).toFloat
    })
    val accumulators = Array(seed, 0.0f, 0.0f)
    words.zipWithIndex.foreach { case (word, index) =>
      accumulators(index % 3) =
        (accumulators(index % 3) + balanced(word)).toFloat
    }
    val expected = ((accumulators(0) + accumulators(1)).toFloat +
      accumulators(2)).toFloat
    startReduction(bits(seed), maximum = false)
    words.zipWithIndex.foreach { case (word, index) =>
      sendWord(word.map(bits), Seq.fill(p.nLanes)(true),
        last = index == words.size - 1)
    }
    awaitReduction(bits(expected), expectedFlags = None)
  }

  // The same physical lanes return to full-width ALU mode after reduction.
  for (lane <- 0 until p.nLanes) {
    poke(c.io.aluIn.bits.operandA(lane), bits(lane.toFloat))
    poke(c.io.aluIn.bits.operandB(lane), bits(2.0f))
    poke(c.io.aluIn.bits.laneMask(lane), 1)
  }
  poke(c.io.aluIn.bits.operation, VpuFpAluOp.Add.litValue)
  poke(c.io.aluIn.bits.destination, 7)
  poke(c.io.aluIn.bits.last, 1)
  poke(c.io.aluIn.valid, 1)
  step(1)
  poke(c.io.aluIn.valid, 0)
  for (_ <- 0 until 2) {
    assert(peek(c.io.aluOut.valid) == 0)
    step(1)
  }
  assert(peek(c.io.aluOut.valid) == 1)
  assert(peek(c.io.aluOut.bits.destination) == 7)
  assert(peek(c.io.aluOut.bits.last) == 1)
  for (lane <- 0 until p.nLanes) {
    assert(peek(c.io.aluOut.bits.data(lane)) == bits(lane.toFloat + 2.0f))
  }

  // Predicated elementwise execution must carry the lane mask through the
  // three-cycle FMA pipe. In particular an inactive signaling NaN neither
  // raises NV nor becomes eligible for SRAM writeback.
  for (lane <- 0 until p.nLanes) {
    poke(c.io.aluIn.bits.operandA(lane),
      if (lane == 1) BigInt("7f800001", 16) else bits(lane.toFloat))
    poke(c.io.aluIn.bits.operandB(lane), bits(1.0f))
    poke(c.io.aluIn.bits.laneMask(lane), if (lane == 1) 0 else 1)
  }
  poke(c.io.aluIn.bits.operation, VpuFpAluOp.Add.litValue)
  poke(c.io.aluIn.bits.destination, 9)
  poke(c.io.aluIn.bits.last, 1)
  poke(c.io.aluIn.valid, 1)
  step(1)
  poke(c.io.aluIn.valid, 0)
  for (_ <- 0 until 2) { step(1) }
  assert(peek(c.io.aluOut.valid) == 1)
  assert(peek(c.io.aluOut.bits.fflags) == 0,
    "masked signaling NaN escaped into elementwise fflags")
  assert(peek(c.io.aluOut.bits.laneMask(1)) == 0)
}

class VpuSharedReciprocalFabricTester(c: VpuVectorFmaFabric, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  require(p.nLanes == 16 && p.reciprocalLanes == 4)
  poke(c.io.aluIn.valid, 0)
  poke(c.io.reciprocalIn.valid, 0)
  poke(c.io.reductionStart, 0)
  poke(c.io.reductionSeed, 0)
  poke(c.io.reductionMaximum, 0)
  poke(c.io.reductionIn.valid, 0)
  poke(c.io.reductionIn.bits.last, 0)
  for (lane <- 0 until p.nLanes) {
    poke(c.io.aluIn.bits.operandA(lane), 0)
    poke(c.io.aluIn.bits.operandB(lane), 0)
    poke(c.io.aluIn.bits.laneMask(lane), 0)
    poke(c.io.reductionIn.bits.values(lane), 0)
    poke(c.io.reductionIn.bits.laneMask(lane), 0)
  }
  poke(c.io.aluIn.bits.operation, VpuFpAluOp.Add.litValue)
  poke(c.io.aluIn.bits.destination, 0)
  poke(c.io.aluIn.bits.last, 0)
  step(2)

  private val random = new scala.util.Random(0x52454349L)
  private val randomNormals = Seq.fill(64) {
    val sign = random.nextInt(2) << 31
    val exponent = (1 + random.nextInt(254)) << 23
    val fraction = random.nextInt(1 << 23)
    BigInt((sign | exponent | fraction).toLong & 0xffffffffL)
  }
  private val corners = Seq(
    bits(0.5f), bits(1.0f), bits(2.0f), bits(3.0f),
    bits(10.0f), bits(-0.75f), bits(java.lang.Float.MIN_NORMAL),
    bits(Float.MaxValue), bits(0.0f), bits(-0.0f),
    bits(Float.PositiveInfinity), bits(Float.NegativeInfinity),
    BigInt("7f800001", 16), BigInt("7fc01234", 16),
    BigInt("00000001", 16), BigInt("80000001", 16))
  private val groups = (corners ++ randomNormals).grouped(p.reciprocalLanes).toSeq
  private val schedule: Vector[Option[Seq[BigInt]]] = groups.zipWithIndex
    .flatMap { case (group, index) =>
      if (index % 7 == 3) Seq(None, Some(group)) else Seq(Some(group))
    }.toVector

  private def check(input: BigInt, actual: BigInt, flags: BigInt): Unit = {
    val raw = input.toLong & 0xffffffffL
    val sign = (raw >>> 31).toInt
    val exponent = ((raw >>> 23) & 0xffL).toInt
    val fraction = raw & 0x7fffffL
    val expectedSpecial = if (exponent == 0 && fraction == 0) {
      Some((if (sign == 0) VpuFloat.positiveInfinity
        else BigInt("ff800000", 16), BigInt(8)))
    } else if (exponent == 0xff && fraction == 0) {
      Some((if (sign == 0) BigInt(0) else BigInt("80000000", 16), BigInt(0)))
    } else if (exponent == 0xff) {
      val signaling = (fraction & 0x400000L) == 0
      Some((VpuFloat.canonicalNaN, if (signaling) BigInt(16) else BigInt(0)))
    } else if (exponent == 0 && fraction == 1) {
      Some((if (sign == 0) VpuFloat.positiveInfinity
        else BigInt("ff800000", 16), BigInt(5)))
    } else None

    expectedSpecial match {
      case Some((expected, expectedFlags)) =>
        assert(actual == expected,
          f"shared reciprocal input=0x$input%x actual=0x$actual%x expected=0x$expected%x")
        assert(flags == expectedFlags,
          f"shared reciprocal input=0x$input%x flags=0x$flags%x expected=0x$expectedFlags%x")
      case None =>
        val expected = bits(1.0f / value(input))
        assert((actual - expected).abs <= 2,
          f"shared reciprocal input=0x$input%x actual=0x$actual%x expected=0x$expected%x")
    }
  }

  var observedGroups = 0
  for (cycle <- 0 until schedule.length + p.reciprocalLatency + 1) {
    val driven = if (cycle < schedule.length) schedule(cycle) else None
    poke(c.io.reciprocalIn.valid, if (driven.isDefined) 1 else 0)
    for (lane <- 0 until p.reciprocalLanes) {
      poke(c.io.reciprocalIn.bits.values(lane),
        driven.map(_(lane)).getOrElse(bits(1.0f)))
    }
    val sourceCycle = cycle - p.reciprocalLatency
    val expected = if (sourceCycle >= 0 && sourceCycle < schedule.length)
      schedule(sourceCycle) else None
    assert(peek(c.io.reciprocalOut.valid) == (if (expected.isDefined) 1 else 0),
      s"shared reciprocal valid misaligned at cycle=$cycle source=$sourceCycle")
    expected.foreach { group =>
      for (lane <- 0 until p.reciprocalLanes) {
        check(group(lane),
          peek(c.io.reciprocalOut.bits.results(lane).data),
          peek(c.io.reciprocalOut.bits.results(lane).fflags))
      }
      observedGroups += 1
    }
    step(1)
  }
  poke(c.io.reciprocalIn.valid, 0)
  assert(observedGroups == groups.size)
  assert(peek(c.io.reciprocalBusy) == 0,
    "shared reciprocal fabric did not drain")

  // After drain, all sixteen physical lanes must immediately be reusable by
  // the full-width elementwise mode.
  for (lane <- 0 until p.nLanes) {
    poke(c.io.aluIn.bits.operandA(lane), bits(lane.toFloat))
    poke(c.io.aluIn.bits.operandB(lane), bits(2.0f))
    poke(c.io.aluIn.bits.laneMask(lane), 1)
  }
  poke(c.io.aluIn.bits.operation, VpuFpAluOp.Add.litValue)
  poke(c.io.aluIn.bits.destination, 3)
  poke(c.io.aluIn.bits.last, 1)
  poke(c.io.aluIn.valid, 1)
  step(1)
  poke(c.io.aluIn.valid, 0)
  for (_ <- 0 until 2) { step(1) }
  assert(peek(c.io.aluOut.valid) == 1)
  for (lane <- 0 until p.nLanes) {
    assert(peek(c.io.aluOut.bits.data(lane)) == bits(lane.toFloat + 2.0f))
  }
}

class VpuVectorFmaFabricSpec extends ChiselFlatSpec {
  behavior of "VpuVectorFmaFabric"
  it should "share its lanes between an II=1 rotating reduction and vector ALU" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2, vSpadKB = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-vector-fma-fabric"),
      () => new VpuVectorFmaFabric(p)) { c =>
      new VpuVectorFmaFabricTester(c, p)
    } should be (true)
  }

  it should "reuse sixteen FMA lanes as four II=1 reciprocal pipelines" in {
    val p = VpuParams(vLen = 16, nLanes = 16, sfuLanes = 4, vSpadKB = 4)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-shared-reciprocal"),
      () => new VpuVectorFmaFabric(p)) { c =>
      new VpuSharedReciprocalFabricTester(c, p)
    } should be (true)
  }
}
