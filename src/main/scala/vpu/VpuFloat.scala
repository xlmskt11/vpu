package vpu

import chisel3._
import chisel3.util._
import freechips.rocketchip.tile.MulAddRecFNPipe
import hardfloat._
import hardfloat.consts._

object VpuFpAluOp {
  val Add = 0.U(3.W)
  val Sub = 1.U(3.W)
  val Mul = 2.U(3.W)
  val Max = 3.U(3.W)
  val Min = 4.U(3.W)
  /** Fused `a * b + c`. */
  val MulAdd = 5.U(3.W)
  /** Fused `-(a * b) + c`, used by reciprocal Newton refinement. */
  val NegMulAdd = 6.U(3.W)
}

class VpuFpInput extends Bundle {
  val op = UInt(3.W)
  val a = UInt(32.W)
  val b = UInt(32.W)
  val c = UInt(32.W)
}

class VpuFpResult extends Bundle {
  val data = UInt(32.W)
  /** HardFloat/RISC-V order: {NV,DZ,OF,UF,NX}. */
  val fflags = UInt(5.W)
}

object VpuFloat {
  val canonicalNaN: BigInt = BigInt("7fc00000", 16)
  val positiveInfinity: BigInt = BigInt("7f800000", 16)
  val negativeInfinity: BigInt = BigInt("ff800000", 16)

  def isNaN(x: UInt): Bool = x(30, 23).andR && x(22, 0).orR
  def isSignalingNaN(x: UInt): Bool = isNaN(x) && !x(22)

  /** Convert one build-time storage element to recoded FP32. */
  def storageToRecodedFp32(x: UInt, p: VpuParams): UInt = p.storageType match {
    case VpuStorageType.FP32 => recFNFromFN(8, 24, x)
    case VpuStorageType.BF16 =>
      // BF16 is (exp=8,sig=8), not Rocket's IEEE binary16 FType.H.
      val widen = Module(new RecFNToRecFN(8, 8, 8, 24))
      widen.io.in := recFNFromFN(8, 8, x)
      widen.io.roundingMode := round_near_even
      widen.io.detectTininess := tininess_afterRounding
      widen.io.out
  }

  def storageToFp32(x: UInt, p: VpuParams): UInt =
    fNFromRecFN(8, 24, storageToRecodedFp32(x, p))

  /** Convert recoded FP32 to the configured storage type. */
  def recodedFp32ToStorage(x: UInt, p: VpuParams): (UInt, UInt) =
    p.storageType match {
      case VpuStorageType.FP32 => (fNFromRecFN(8, 24, x), 0.U(5.W))
      case VpuStorageType.BF16 =>
        val narrow = Module(new RecFNToRecFN(8, 24, 8, 8))
        narrow.io.in := x
        narrow.io.roundingMode := round_near_even
        narrow.io.detectTininess := tininess_afterRounding
        (fNFromRecFN(8, 8, narrow.io.out), narrow.io.exceptionFlags)
    }

  def fp32ToStorage(x: UInt, p: VpuParams): (UInt, UInt) =
    recodedFp32ToStorage(recFNFromFN(8, 24, x), p)
}

/** Saturn-compatible depth-four FP32 arithmetic lane.
  *
  * There is one explicit capture stage around Rocket's two-stage
  * MulAddRecFNPipe, hence three visible issue-to-result cycles and II=1.
  * ADD/SUB/MUL and the internal ternary FMA operations share the FMA
  * datapath. MAX/MIN are aligned to that latency.
  */
class VpuFmaPipe extends Module {
  val io = IO(new Bundle {
    val in = Flipped(Valid(new VpuFpInput))
    val out = Valid(new VpuFpResult)
  })

  val validS0 = RegNext(io.in.valid, false.B)
  val inputS0 = RegEnable(io.in.bits, io.in.valid)
  val aRec = recFNFromFN(8, 24, inputS0.a)
  val bRec = recFNFromFN(8, 24, inputS0.b)
  val cRec = recFNFromFN(8, 24, inputS0.c)
  val oneRec = recFNFromFN(8, 24, "h3f800000".U(32.W))
  val signedZeroRec = Cat(aRec(32) ^ bRec(32), 0.U(32.W))

  val fma = Module(new MulAddRecFNPipe(2, 8, 24))
  fma.io.validin := validS0
  // HardFloat op encoding: 00=a*b+c, 01=a*b-c, 10=-(a*b)+c.
  fma.io.op := Mux(inputS0.op === VpuFpAluOp.Sub, "b01".U,
    Mux(inputS0.op === VpuFpAluOp.NegMulAdd, "b10".U, "b00".U))
  fma.io.a := aRec
  val isMul = inputS0.op === VpuFpAluOp.Mul
  val isTernary = inputS0.op === VpuFpAluOp.MulAdd ||
    inputS0.op === VpuFpAluOp.NegMulAdd
  fma.io.b := Mux(isMul || isTernary, bRec, oneRec)
  fma.io.c := Mux(isMul, signedZeroRec, Mux(isTernary, cRec, bRec))
  fma.io.roundingMode := round_near_even
  fma.io.detectTininess := tininess_afterRounding

  val compare = Module(new CompareRecFN(8, 24))
  compare.io.a := aRec
  compare.io.b := bRec
  compare.io.signaling := false.B
  val aNaN = VpuFloat.isNaN(inputS0.a)
  val bNaN = VpuFloat.isNaN(inputS0.b)
  val bothZero = !inputS0.a(30, 0).orR && !inputS0.b(30, 0).orR
  // IEEE/RISC-V min/max-number signed-zero tie breaking: max(-0,+0) is
  // +0, while min(-0,+0) is -0, independent of operand order.
  val signedZeroTie = Cat(Mux(inputS0.op === VpuFpAluOp.Max,
    inputS0.a(31) && inputS0.b(31), inputS0.a(31) || inputS0.b(31)),
    0.U(31.W))
  val chooseAForMax = compare.io.gt || compare.io.eq
  val chooseAForMin = compare.io.lt || compare.io.eq
  val compareChoice = Mux(aNaN && bNaN, VpuFloat.canonicalNaN.U(32.W),
    Mux(aNaN, inputS0.b,
      Mux(bNaN, inputS0.a,
        Mux(bothZero, signedZeroTie,
          Mux(Mux(inputS0.op === VpuFpAluOp.Max, chooseAForMax, chooseAForMin),
            inputS0.a, inputS0.b)))))
  val compareFlags = compare.io.exceptionFlags |
    Mux(VpuFloat.isSignalingNaN(inputS0.a) || VpuFloat.isSignalingNaN(inputS0.b),
      "b10000".U, 0.U)
  val compareResult = Wire(new VpuFpResult)
  compareResult.data := compareChoice
  compareResult.fflags := compareFlags
  val comparePipe = Pipe(validS0, compareResult, 2)
  val opPipe = Pipe(validS0, inputS0.op, 2)

  io.out.valid := fma.io.validout
  io.out.bits.data := Mux(opPipe.bits === VpuFpAluOp.Max ||
    opPipe.bits === VpuFpAluOp.Min, comparePipe.bits.data,
    fNFromRecFN(8, 24, fma.io.out))
  io.out.bits.fflags := Mux(opPipe.bits === VpuFpAluOp.Max ||
    opPipe.bits === VpuFpAluOp.Min, comparePipe.bits.fflags,
    fma.io.exceptionFlags)
  assert(!fma.io.validout || (comparePipe.valid && opPipe.valid))
}

/** Synthesizable FP32 EXP approximation.
  *
  * Input is converted to signed Q39.24, multiplied by log2(e), and split into
  * integer/fractional parts. A 64-entry 2^(i/64) table plus a second-order
  * approximation over the remaining interval reconstructs the IEEE result.
  * The datapath is five cycles and fully pipelined. Subnormal outputs are
  * generated by shifting the normalized significand rather than flushing.
  */
class VpuExpMetadata extends Bundle {
  val sign = Bool()
  val isNaN = Bool()
  val isSignalingNaN = Bool()
  val isInf = Bool()
  val finiteOverflow = Bool()
  val finiteUnderflow = Bool()
  val finiteNonZero = Bool()
}

class VpuExpApprox extends Module {
  val io = IO(new Bundle {
    val in = Flipped(Valid(UInt(32.W)))
    val out = Valid(new VpuFpResult)
  })

  // Stage 1: classify the input and convert its finite magnitude to signed
  // Q39.24.  The explicit register here is the first of the five externally
  // visible pipeline cycles.
  val x = io.in.bits
  val sign = x(31)
  val exponent = x(30, 23)
  val fraction = x(22, 0)
  val isNaN = exponent.andR && fraction.orR
  val isInf = exponent.andR && !fraction.orR
  val isZeroOrSubnormal = exponent === 0.U

  val mantissa = Cat(1.U(1.W), fraction)
  val unbiased = exponent.zext - 127.S
  val binaryShift = unbiased + 1.S // mantissa * 2^(e+1) is Q24
  val shiftLeft = binaryShift >= 0.S
  val shiftMagnitudeWide = Mux(shiftLeft, binaryShift.asUInt,
    (-binaryShift).asUInt)
  val shiftMagnitude = Mux(shiftMagnitudeWide > 63.U, 63.U,
    shiftMagnitudeWide(5, 0))
  val mantissaWide = Wire(UInt(64.W))
  mantissaWide := mantissa
  val magnitudeQ24 = Mux(isZeroOrSubnormal, 0.U,
    Mux(shiftLeft, (mantissaWide << shiftMagnitude)(63, 0),
      mantissaWide >> shiftMagnitude))
  val signedXQ24 = Mux(sign, -magnitudeQ24.asSInt, magnitudeQ24.asSInt)

  // expf overflow/underflow boundaries rounded to FP32.  These predicates are
  // classified before the finite datapath so payload bits from a NaN can never
  // create spurious range-reduction flags.
  val metadata = Wire(new VpuExpMetadata)
  metadata.sign := sign
  metadata.isNaN := isNaN
  metadata.isSignalingNaN := VpuFloat.isSignalingNaN(x)
  metadata.isInf := isInf
  metadata.finiteOverflow := !sign && exponent =/= 255.U &&
    x(30, 0) >= "h42b17218".U
  metadata.finiteUnderflow := sign && exponent =/= 255.U &&
    x(30, 0) >= "h42cff1b5".U
  metadata.finiteNonZero := !isNaN && !isInf && x(30, 0).orR

  val validS1 = RegNext(io.in.valid, false.B)
  val metadataS1 = RegEnable(metadata, io.in.valid)
  val signedXQ24S1 = RegEnable(signedXQ24, io.in.valid)

  // Stage 2: multiply by log2(e), then split floor(x * log2(e)) into its
  // integer exponent and unsigned Q0.24 fractional part.
  val log2eQ24 = BigInt(Math.round((Math.log(Math.E) / Math.log(2.0)) * (1L << 24)))
  val yProduct = signedXQ24S1 * log2eQ24.S
  val yQ24 = (yProduct >> 24).asSInt
  val n = (yQ24 >> 24).asSInt
  val fracQ24 = yQ24.asUInt()(23, 0)

  val validS2 = RegNext(validS1, false.B)
  val metadataS2 = RegEnable(metadataS1, validS1)
  val nS2 = RegEnable(n, validS1)
  val fracQ24S2 = RegEnable(fracQ24, validS1)

  // Stage 3: 64-entry 2^(i/64) lookup and second-order interpolation over the
  // residual interval.  This is an actual registered ROM/polynomial boundary,
  // rather than a combinational approximation followed by a delay-only Pipe.
  val expTable = VecInit((0 until 64).map { i =>
    BigInt(Math.round(Math.pow(2.0, i.toDouble / 64.0) * (1L << 31))).U(32.W)
  })
  val ln2Q31 = BigInt(Math.round(Math.log(2.0) * (1L << 31)))
  val ln2SquaredHalfQ31 = BigInt(Math.round(
    Math.log(2.0) * Math.log(2.0) * 0.5 * (1L << 31)))
  val tableIndex = fracQ24S2(23, 18)
  val remainderQ24 = fracQ24S2(17, 0)
  val linear = (remainderQ24 * ln2Q31.U) >> 24
  val remSquared = remainderQ24 * remainderQ24
  val quadratic = (remSquared * ln2SquaredHalfQ31.U) >> 48
  val localFactorQ31 = (BigInt(1) << 31).U(34.W) +& linear +& quadratic
  val approximationQ31 = (expTable(tableIndex) * localFactorQ31) >> 31

  val validS3 = RegNext(validS2, false.B)
  val metadataS3 = RegEnable(metadataS2, validS2)
  val nS3 = RegEnable(nS2, validS2)
  val approximationQ31S3 = RegEnable(approximationQ31, validS2)

  // Stage 4: normalize and round to nearest-even.  The same stage performs the
  // second rounding needed when the result falls into the FP32 subnormal range.
  val significandBeforeRound = approximationQ31S3(31, 8)
  val roundBit = approximationQ31S3(7)
  val sticky = approximationQ31S3(6, 0).orR
  val roundedSignificand = significandBeforeRound +&
    (roundBit && (sticky || significandBeforeRound(0)))
  val significandCarry = roundedSignificand(24)
  val normalizedSignificand = Mux(significandCarry,
    roundedSignificand(24, 1), roundedSignificand(23, 0))
  val resultExponentS = nS3 + 127.S + significandCarry.asUInt.zext

  val normalResult = Cat(0.U(1.W), resultExponentS.asUInt()(7, 0),
    normalizedSignificand(22, 0))
  val subnormalShiftWide = 1.S - resultExponentS
  val subnormalShift = Mux(subnormalShiftWide <= 0.S, 1.U,
    Mux(subnormalShiftWide > 31.S, 31.U,
      subnormalShiftWide.asUInt()(4, 0)))
  val subnormalSigWide = normalizedSignificand.pad(32)
  val subnormalTruncated = subnormalSigWide >> subnormalShift
  val subnormalRoundMask = (1.U(32.W) << (subnormalShift - 1.U))(31, 0)
  val subnormalRoundBit = (subnormalSigWide & subnormalRoundMask).orR
  val subnormalSticky = (subnormalSigWide & (subnormalRoundMask - 1.U)).orR
  val subnormalRounded = subnormalTruncated +&
    (subnormalRoundBit && (subnormalSticky || subnormalTruncated(0)))
  // The rounded integer directly occupies IEEE bits [23:0]. A carry into bit
  // 23 therefore correctly produces the minimum normal encoding 0x00800000.
  val subnormalResult = subnormalRounded(23, 0).pad(32)

  val exponentOverflow = resultExponentS >= 255.S
  val exponentSubnormal = resultExponentS <= 0.S

  val validS4 = RegNext(validS3, false.B)
  val metadataS4 = RegEnable(metadataS3, validS3)
  val resultExponentS4 = RegEnable(resultExponentS, validS3)
  val normalResultS4 = RegEnable(normalResult, validS3)
  val subnormalResultS4 = RegEnable(subnormalResult, validS3)
  val exponentOverflowS4 = RegEnable(exponentOverflow, validS3)
  val exponentSubnormalS4 = RegEnable(exponentSubnormal, validS3)

  // Stage 5: select architectural special-case results and flags.  Keeping
  // this selection behind its own register guarantees that valid, data, and
  // flags remain aligned for consecutive inputs and bubbles at II=1.
  val rawResult = Mux(metadataS4.isNaN, VpuFloat.canonicalNaN.U(32.W),
    Mux(metadataS4.isInf && !metadataS4.sign,
      VpuFloat.positiveInfinity.U(32.W),
      Mux(metadataS4.isInf && metadataS4.sign, 0.U,
        Mux(metadataS4.finiteOverflow || exponentOverflowS4,
          VpuFloat.positiveInfinity.U(32.W),
          Mux(metadataS4.finiteUnderflow || resultExponentS4 < -23.S, 0.U,
            Mux(exponentSubnormalS4, subnormalResultS4, normalResultS4))))))
  // Do not let the payload bits of a NaN flow into the finite range-reduction
  // flags.  In particular, a large/odd payload can make resultExponentS look
  // subnormal even though the architectural result is simply a quiet NaN.
  val flags = Mux(metadataS4.isNaN,
    Mux(metadataS4.isSignalingNaN, "b10000".U(5.W), 0.U(5.W)),
    Mux(metadataS4.isInf, 0.U(5.W),
      Mux(metadataS4.finiteOverflow || exponentOverflowS4,
        "b00101".U(5.W),
        Mux(metadataS4.finiteUnderflow || exponentSubnormalS4,
          "b00011".U(5.W),
          Mux(metadataS4.finiteNonZero, "b00001".U(5.W), 0.U(5.W))))))

  val result = Wire(new VpuFpResult)
  result.data := rawResult
  result.fflags := flags
  val validS5 = RegNext(validS4, false.B)
  val resultS5 = RegEnable(result, validS4)
  io.out.valid := validS5
  io.out.bits := resultS5
}

class VpuDivSqrtInput extends Bundle {
  val data = UInt(32.W)
  val sqrt = Bool()
}

/** Exact iterative FP32 reciprocal/square-root wrapper. */
class VpuDivSqrt extends Module {
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new VpuDivSqrtInput))
    val out = Decoupled(new VpuFpResult)
  })

  val unit = Module(new DivSqrtRecFN_small(8, 24, 0))
  val resultQueue = Module(new Queue(new VpuFpResult, 1, pipe = false, flow = false))
  val one = recFNFromFN(8, 24, "h3f800000".U(32.W))
  val inputRec = recFNFromFN(8, 24, io.in.bits.data)

  io.in.ready := unit.io.inReady && !resultQueue.io.deq.valid
  unit.io.inValid := io.in.fire
  unit.io.sqrtOp := io.in.bits.sqrt
  unit.io.a := Mux(io.in.bits.sqrt, inputRec, one)
  unit.io.b := inputRec
  unit.io.roundingMode := round_near_even
  unit.io.detectTininess := tininess_afterRounding

  resultQueue.io.enq.valid := unit.io.outValid_div || unit.io.outValid_sqrt
  resultQueue.io.enq.bits.data := fNFromRecFN(8, 24, unit.io.out)
  resultQueue.io.enq.bits.fflags := unit.io.exceptionFlags
  assert(!resultQueue.io.enq.valid || resultQueue.io.enq.ready,
    "VPU iterative FP result queue overflow")
  io.out <> resultQueue.io.deq
}
