package vpu

import chisel3._
import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

object VpuTestFloat {
  def bits(x: Float): BigInt = BigInt(java.lang.Float.floatToRawIntBits(x) & 0xffffffffL)
  def value(x: BigInt): Float = java.lang.Float.intBitsToFloat((x & 0xffffffffL).toInt)
}

class VpuFmaPipeTester(c: VpuFmaPipe) extends PeekPokeTester(c) {
  import VpuTestFloat._
  poke(c.io.in.valid, 0)
  step(2)

  def runBits(op: Int, a: BigInt, b: BigInt, expected: BigInt,
              expectedFlags: Option[BigInt] = None,
              cValue: BigInt = 0): Unit = {
    poke(c.io.in.valid, 1)
    poke(c.io.in.bits.op, op)
    poke(c.io.in.bits.a, a)
    poke(c.io.in.bits.b, b)
    poke(c.io.in.bits.c, cValue)
    step(1)
    poke(c.io.in.valid, 0)
    var latency = 0
    while (peek(c.io.out.valid) == 0 && latency < 8) {
      step(1)
      latency += 1
    }
    assert(latency == 2, s"capture plus two-stage FMA must be three visible cycles, got ${latency + 1}")
    assert(peek(c.io.out.bits.data) == expected,
      f"op=$op a=0x$a%x b=0x$b%x actual=0x${peek(c.io.out.bits.data)}%x expected=0x$expected%x")
    expectedFlags.foreach(flags => assert(peek(c.io.out.bits.fflags) == flags,
      s"op=$op flags=${peek(c.io.out.bits.fflags)} expected=$flags"))
    step(1)
  }

  def run(op: Int, a: Float, b: Float, expected: Float): Unit =
    runBits(op, bits(a), bits(b), bits(expected))

  run(0, 1.5f, 2.25f, 3.75f)
  run(1, 1.5f, 2.25f, -0.75f)
  run(2, -3.0f, 0.5f, -1.5f)
  run(3, -3.0f, 0.5f, 0.5f)
  run(4, -3.0f, 0.5f, -3.0f)
  runBits(VpuFpAluOp.MulAdd.litValue.toInt, bits(1.5f), bits(2.0f),
    bits(3.25f), cValue = bits(0.25f))
  // This vector differs by one ULP from a separately rounded MUL+SUB and
  // proves that the reciprocal error step uses the ternary HardFloat path.
  runBits(VpuFpAluOp.NegMulAdd.litValue.toInt,
    BigInt("3f1cf44d", 16), BigInt("3f2ee433", 16),
    BigInt("3fca6305", 16), cValue = BigInt("40000000", 16))

  val plusZero = BigInt("00000000", 16)
  val minusZero = BigInt("80000000", 16)
  runBits(3, minusZero, plusZero, plusZero, Some(0))
  runBits(3, plusZero, minusZero, plusZero, Some(0))
  runBits(4, minusZero, plusZero, minusZero, Some(0))
  runBits(4, plusZero, minusZero, minusZero, Some(0))

  val qNaN = BigInt("7fc01234", 16)
  val sNaN = BigInt("7f801234", 16)
  runBits(3, qNaN, bits(2.0f), bits(2.0f), Some(0))
  runBits(4, bits(2.0f), qNaN, bits(2.0f), Some(0))
  runBits(3, sNaN, bits(2.0f), bits(2.0f), Some(BigInt("10", 16)))
  runBits(3, qNaN, qNaN, VpuFloat.canonicalNaN, Some(0))

  // The arithmetic path is Rocket HardFloat, but keep a randomized bit-exact
  // regression at this wrapper boundary so operand selection and pipeline
  // alignment cannot silently regress.
  val random = new scala.util.Random(0x464d4131L)
  for (_ <- 0 until 32) {
    val a = (random.nextFloat() * 200.0f - 100.0f).toFloat
    val b = (random.nextFloat() * 200.0f - 100.0f).toFloat
    run(0, a, b, (a + b).toFloat)
    run(1, a, b, (a - b).toFloat)
    run(2, a, b, (a * b).toFloat)
    run(3, a, b, (if (a >= b) a else b))
    run(4, a, b, (if (a <= b) a else b))
  }
}

class VpuExpTester(c: VpuExpApprox) extends PeekPokeTester(c) {
  import VpuTestFloat._

  case class Stimulus(tag: Int, input: BigInt, label: String)
  private val latency = VpuExpApprox.Latency

  private def check(stimulus: Stimulus): Unit = {
    val input = stimulus.input & BigInt("ffffffff", 16)
    val raw = input.toLong & 0xffffffffL
    val exponent = ((raw >>> 23) & 0xffL).toInt
    val fraction = raw & 0x7fffffL
    val inputIsNaN = exponent == 0xff && fraction != 0
    val inputIsInf = exponent == 0xff && fraction == 0
    val inputSign = (raw & 0x80000000L) != 0
    val actualBits = peek(c.io.out.bits.data)
    val actualFlags = peek(c.io.out.bits.fflags)

    if (inputIsNaN) {
      val signaling = (fraction & 0x400000L) == 0
      assert(actualBits == VpuFloat.canonicalNaN,
        f"EXP tag=${stimulus.tag}%d ${stimulus.label} input=0x$input%x " +
          f"actual=0x$actualBits%x expected=0x${VpuFloat.canonicalNaN}%x")
      assert(actualFlags == (if (signaling) 16 else 0),
        f"EXP tag=${stimulus.tag}%d ${stimulus.label} flags=0x$actualFlags%x")
    } else if (inputIsInf) {
      val expected = if (inputSign) bits(0.0f) else bits(Float.PositiveInfinity)
      assert(actualBits == expected,
        f"EXP tag=${stimulus.tag}%d ${stimulus.label} actual=0x$actualBits%x expected=0x$expected%x")
      assert(actualFlags == 0,
        f"EXP tag=${stimulus.tag}%d ${stimulus.label} flags=0x$actualFlags%x")
    } else {
      val inputValue = value(input)
      val expectedValue = Math.exp(inputValue.toDouble)
      val expectedFp32 = expectedValue.toFloat
      val expectedBits = bits(expectedFp32)
      val expectedExponent = ((expectedBits.toLong >>> 23) & 0xffL).toInt
      val inputIsZero = (raw & 0x7fffffffL) == 0
      val expectedFlags = if (inputIsZero) 0
        else if (expectedExponent == 0xff) 5 // OF | NX
        else if (expectedExponent == 0) 3 // UF | NX
        else 1 // NX

      if (expectedExponent == 0xff || expectedBits == 0 || inputIsZero) {
        assert(actualBits == expectedBits,
          f"EXP tag=${stimulus.tag}%d ${stimulus.label} input=0x$input%x " +
            f"actual=0x$actualBits%x expected=0x$expectedBits%x")
      } else {
        val actual = value(actualBits).toDouble
        val relative = Math.abs(actual - expectedValue) / expectedValue
        val ulps = (actualBits - expectedBits).abs
        val allowedUlps = if (expectedExponent == 0) 1 else 8
        assert(ulps <= allowedUlps ||
          (expectedExponent != 0 && relative <= 1.0e-6),
          s"EXP tag=${stimulus.tag} ${stimulus.label} exp($inputValue)=$actual " +
            s"expected=$expectedValue rel=$relative ulps=$ulps")
      }
      assert(actualFlags == expectedFlags,
        f"EXP tag=${stimulus.tag}%d ${stimulus.label} input=0x$input%x " +
          f"flags=0x$actualFlags%x expected=0x$expectedFlags%x")
    }
  }

  poke(c.io.in.valid, 0)
  poke(c.io.in.bits, 0)
  step(2)

  // The dense interval proves II=1 at the numerical acceptance boundary.  The
  // randomized tail adds bubbles and consecutive runs, while the corner prefix
  // checks that special-case metadata cannot become detached from its data.
  val cornerInputs = Seq[Option[(BigInt, String)]](
    Some(bits(0.0f) -> "+zero"),
    Some(bits(-0.0f) -> "-zero"),
    Some(bits(1.0f) -> "one"),
    None,
    Some(BigInt("7fc00001", 16) -> "+qNaN"),
    Some(BigInt("ffc12345", 16) -> "-qNaN"),
    Some(BigInt("7f800001", 16) -> "+sNaN"),
    Some(BigInt("ff800001", 16) -> "-sNaN"),
    None,
    Some(bits(Float.PositiveInfinity) -> "+infinity"),
    Some(bits(Float.NegativeInfinity) -> "-infinity"),
    Some(bits(-100.0f) -> "subnormal-result"),
    Some(bits(100.0f) -> "overflow"),
    Some(bits(-104.0f) -> "underflow-to-zero"),
    None,
    Some(BigInt("00000001", 16) -> "+min-subnormal-input"),
    Some(BigInt("80000001", 16) -> "-min-subnormal-input"),
    Some(bits(Float.MaxValue) -> "+max-finite"),
    Some(bits(-Float.MaxValue) -> "-max-finite"))
  val denseInputs = (-100 to 100).map { i =>
    val x = i.toFloat / 10.0f
    Some(bits(x) -> s"dense-$i")
  }
  val random = new scala.util.Random(0x45585035L)
  val randomInputs = (0 until 192).map { i =>
    if (i % 13 == 3 || i % 29 == 17) None
    else {
      val x = (random.nextFloat() * 20.0f - 10.0f).toFloat
      Some(bits(x) -> s"random-$i")
    }
  }
  val rawSchedule = (cornerInputs ++ Seq(None) ++ denseInputs ++
    Seq(None, None) ++ randomInputs).toVector
  var nextTag = 0
  val schedule = rawSchedule.map(_.map { case (input, label) =>
    val stimulus = Stimulus(nextTag, input, label)
    nextTag += 1
    stimulus
  })

  var observed = 0
  for (cycle <- 0 until schedule.length + latency + 1) {
    val driven = if (cycle < schedule.length) schedule(cycle) else None
    poke(c.io.in.valid, if (driven.isDefined) 1 else 0)
    poke(c.io.in.bits, driven.map(_.input).getOrElse(BigInt(0)))

    val sourceCycle = cycle - latency
    val expected = if (sourceCycle >= 0 && sourceCycle < schedule.length)
      schedule(sourceCycle) else None
    assert(peek(c.io.out.valid) == (if (expected.isDefined) 1 else 0),
      s"EXP valid misaligned at cycle=$cycle sourceCycle=$sourceCycle " +
        s"expectedTag=${expected.map(_.tag)}")
    expected.foreach { stimulus =>
      check(stimulus)
      observed += 1
    }
    step(1)
  }
  poke(c.io.in.valid, 0)
  assert(observed == nextTag,
    s"EXP pipeline lost or duplicated results: observed=$observed accepted=$nextTag")
}

class VpuDivSqrtTester(c: VpuDivSqrt) extends PeekPokeTester(c) {
  import VpuTestFloat._
  poke(c.io.in.valid, 0)
  poke(c.io.out.ready, 1)
  step(2)

  def run(x: Float, sqrt: Boolean, expected: Float, dz: Boolean = false): Unit = {
    poke(c.io.in.valid, 1)
    poke(c.io.in.bits.data, bits(x))
    poke(c.io.in.bits.sqrt, sqrt)
    while (peek(c.io.in.ready) == 0) { step(1) }
    step(1)
    poke(c.io.in.valid, 0)
    var timeout = 80
    while (peek(c.io.out.valid) == 0 && timeout > 0) { step(1); timeout -= 1 }
    assert(timeout > 0)
    assert(peek(c.io.out.bits.data) == bits(expected))
    assert(((peek(c.io.out.bits.fflags) >> 3) & 1) == (if (dz) 1 else 0))
    step(1)
  }

  run(4.0f, sqrt = false, 0.25f)
  run(9.0f, sqrt = true, 3.0f)
  run(0.0f, sqrt = false, Float.PositiveInfinity, dz = true)
}

class VpuBf16CodecHarness extends Module {
  private val p = VpuParams(storageType = VpuStorageType.BF16,
    vLen = 16, nLanes = 8, sfuLanes = 2, vSpadKB = 1)
  val io = IO(new Bundle {
    val in = Input(UInt(32.W))
    val stored = Output(UInt(16.W))
    val widened = Output(UInt(32.W))
  })
  val converted = VpuFloat.fp32ToStorage(io.in, p)
  io.stored := converted._1
  io.widened := VpuFloat.storageToFp32(converted._1, p)
}

class VpuBf16CodecTester(c: VpuBf16CodecHarness) extends PeekPokeTester(c) {
  import VpuTestFloat._
  poke(c.io.in, bits(1.5f))
  step(1)
  assert(peek(c.io.stored) == BigInt("3fc0", 16))
  assert(peek(c.io.widened) == bits(1.5f))
}

class VpuFloatSpec extends ChiselFlatSpec {
  behavior of "VpuFmaPipe"
  it should "implement FP32 ADD/SUB/MUL/MIN/MAX at the contracted latency" in {
    chisel3.iotesters.Driver.execute(Array("--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-fma"), () => new VpuFmaPipe) {
      c => new VpuFmaPipeTester(c)
    } should be (true)
  }

  behavior of "VpuExpApprox"
  it should "sustain II=1 at fixed eight-cycle latency with bubbles and special values" in {
    chisel3.iotesters.Driver.execute(Array("--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-exp"), () => new VpuExpApprox) {
      c => new VpuExpTester(c)
    } should be (true)
  }

  behavior of "VpuDivSqrt"
  it should "compute exact reciprocal and square root including divide-by-zero" in {
    chisel3.iotesters.Driver.execute(Array("--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-divsqrt"), () => new VpuDivSqrt) {
      c => new VpuDivSqrtTester(c)
    } should be (true)
  }

  behavior of "the BF16 storage codec"
  it should "use the (8,8) BF16 format rather than IEEE binary16" in {
    chisel3.iotesters.Driver.execute(Array("--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-bf16-codec"), () => new VpuBf16CodecHarness) {
      c => new VpuBf16CodecTester(c)
    } should be (true)
  }
}
