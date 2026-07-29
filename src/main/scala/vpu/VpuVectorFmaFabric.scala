package vpu

import chisel3._
import chisel3.util._

/** One streamed base-vector operation presented to the shared FP lane fabric. */
class VpuVectorAluFabricInput(p: VpuParams) extends Bundle {
  val operation = UInt(3.W)
  val operandA = Vec(p.nLanes, UInt(32.W))
  val operandB = Vec(p.nLanes, UInt(32.W))
  val laneMask = Vec(p.nLanes, Bool())
  val destination = UInt(p.elementAddrBits.W)
  val last = Bool()
}

/** One streamed base-vector result returned by the shared FP lane fabric. */
class VpuVectorAluFabricOutput(p: VpuParams) extends Bundle {
  val data = Vec(p.nLanes, UInt(32.W))
  val laneMask = Vec(p.nLanes, Bool())
  val destination = UInt(p.elementAddrBits.W)
  val fflags = UInt(5.W)
  val last = Bool()
}

/** One narrow group entering the shared reciprocal pipeline. */
class VpuReciprocalFabricInput(p: VpuParams) extends Bundle {
  val values = Vec(p.reciprocalLanes, UInt(32.W))
}

/** One narrow group leaving the shared reciprocal pipeline. */
class VpuReciprocalFabricOutput(p: VpuParams) extends Bundle {
  val results = Vec(p.reciprocalLanes, new VpuFpResult)
}

/** One SRAM lane word entering the reduction pipeline. */
class VpuReductionFabricInput(p: VpuParams) extends Bundle {
  val values = Vec(p.nLanes, UInt(32.W))
  val laneMask = Vec(p.nLanes, Bool())
  val last = Bool()
}

class VpuReductionStageMeta extends Bundle {
  val maximum = Bool()
  val last = Bool()
  val sawNaN = Bool()
  val fflags = UInt(5.W)
}

class VpuReductionAccumulatorMeta extends Bundle {
  val tag = UInt(2.W)
  val last = Bool()
  val sawNaN = Bool()
  val fflags = UInt(5.W)
}

class VpuAluFabricMeta(p: VpuParams) extends Bundle {
  val laneMask = Vec(p.nLanes, Bool())
  val destination = UInt(p.elementAddrBits.W)
  val last = Bool()
}

/** Saturn-style shared vector-FMA fabric.
  *
  * Base-vector ADD/SUB/MUL uses every lane and accepts one SRAM lane word per
  * cycle.  During RED_SUM/RED_MAX the same physical lanes are statically
  * partitioned as n/2, n/4, ..., 1 tree nodes, leaving the final lane for the
  * cross-word recurrence.  For the default sixteen lanes that is exactly the
  * PLENA/Saturn-inspired 8/4/2/1 tree plus one accumulator lane.
  *
  * Tree stages are separated by the three-cycle [[VpuFmaPipe]] latency and
  * therefore accept one word per cycle after fill.  The recurrence rotates
  * over three architectural accumulators; same-cycle result forwarding closes
  * the distance-three dependency without a bubble.  Once the last word has
  * retired, the accumulator lane folds the three partials into the scalar
  * result.  Reciprocal mode partitions the same lanes into four-stage groups:
  * fused `2-a*x` and `x*error` for each of two Newton refinements.  Thus no
  * second reduction tree or reciprocal-only FMA array is instantiated.
  */
class VpuVectorFmaFabric(p: VpuParams) extends Module {
  require(p.nLanes >= 2,
    "the shared reduction fabric requires at least two lanes")
  require(p.fmaPipeDepth == 4,
    "the rotating reduction contract assumes three visible FMA cycles")

  val io = IO(new Bundle {
    val aluIn = Flipped(Valid(new VpuVectorAluFabricInput(p)))
    val aluOut = Valid(new VpuVectorAluFabricOutput(p))

    val reciprocalIn = Flipped(Valid(new VpuReciprocalFabricInput(p)))
    val reciprocalOut = Valid(new VpuReciprocalFabricOutput(p))
    val reciprocalBusy = Output(Bool())

    val reductionStart = Input(Bool())
    val reductionSeed = Input(UInt(32.W))
    val reductionMaximum = Input(Bool())
    val reductionIn = Flipped(Valid(new VpuReductionFabricInput(p)))
    val reductionOut = Valid(new VpuFpResult)
    val reductionBusy = Output(Bool())
  })

  private val visibleFmaLatency = p.fmaPipeDepth - 1
  private val neutralMax = VpuFloat.negativeInfinity.U(32.W)
  private val neutralSum = 0.U(32.W)
  private val lanes = Seq.fill(p.nLanes)(Module(new VpuFmaPipe))
  lanes.foreach { lane =>
    lane.io.in.valid := false.B
    lane.io.in.bits := 0.U.asTypeOf(lane.io.in.bits)
  }

  // ----------------------------------------------------------------------
  // Full-width base-vector mode.
  // ----------------------------------------------------------------------
  for (lane <- 0 until p.nLanes) {
    when (io.aluIn.valid) {
      lanes(lane).io.in.valid := true.B
      lanes(lane).io.in.bits.op := io.aluIn.bits.operation
      lanes(lane).io.in.bits.a := io.aluIn.bits.operandA(lane)
      lanes(lane).io.in.bits.b := io.aluIn.bits.operandB(lane)
    }
  }

  val aluMeta = Wire(new VpuAluFabricMeta(p))
  aluMeta.laneMask := io.aluIn.bits.laneMask
  aluMeta.destination := io.aluIn.bits.destination
  aluMeta.last := io.aluIn.bits.last
  val aluMetaPipe = Pipe(io.aluIn.valid, aluMeta, visibleFmaLatency)

  io.aluOut.valid := aluMetaPipe.valid
  io.aluOut.bits.laneMask := aluMetaPipe.bits.laneMask
  io.aluOut.bits.destination := aluMetaPipe.bits.destination
  io.aluOut.bits.last := aluMetaPipe.bits.last
  var aluFlags = 0.U(5.W)
  for (lane <- 0 until p.nLanes) {
    io.aluOut.bits.data(lane) := lanes(lane).io.out.bits.data
    aluFlags = aluFlags | Mux(aluMetaPipe.bits.laneMask(lane),
      lanes(lane).io.out.bits.fflags, 0.U)
  }
  io.aluOut.bits.fflags := aluFlags
  when (aluMetaPipe.valid) {
    assert(lanes.map(_.io.out.valid).reduce(_ && _),
      "VPU base-vector lane/tag pipeline lost alignment")
  }

  // ----------------------------------------------------------------------
  // Shared four-stage reciprocal mode.
  //
  // Each logical reciprocal lane statically owns four of the same physical
  // FMA lanes used above.  HardFloat op=2 performs the fused error step
  // `-(a*x)+2`; the following lane multiplies by that error. Repeating the
  // pair gives two Newton refinements with one rounding at each operation.
  // The Saturn VFREC7 seed is small and remains replicated per result lane.
  // ----------------------------------------------------------------------
  private val reciprocalLatency = p.reciprocalLatency
  private def pipeReciprocalContext(valid: Bool,
      context: VpuReciprocalContext) =
    Pipe(valid, context, visibleFmaLatency)

  val reciprocalValidHistory = RegInit(VecInit(
    Seq.fill(reciprocalLatency)(false.B)))
  reciprocalValidHistory(0) := io.reciprocalIn.valid
  for (stage <- 1 until reciprocalLatency) {
    reciprocalValidHistory(stage) := reciprocalValidHistory(stage - 1)
  }
  io.reciprocalBusy := io.reciprocalIn.valid ||
    reciprocalValidHistory.asUInt.orR

  val reciprocalFinalValids = (0 until p.reciprocalLanes).map { group =>
    val base = group * p.reciprocalFmasPerLane
    val error0Lane = lanes(base)
    val update0Lane = lanes(base + 1)
    val error1Lane = lanes(base + 2)
    val update1Lane = lanes(base + 3)

    val seed = Module(new VpuRec7Seed)
    seed.io.in := io.reciprocalIn.bits.values(group)
    val seedContext = Wire(new VpuReciprocalContext)
    seedContext.operand := Mux(seed.io.refine,
      io.reciprocalIn.bits.values(group), "h3f800000".U)
    seedContext.estimate := Mux(seed.io.refine,
      seed.io.result.data, "h3f800000".U)
    seedContext.specialResult := seed.io.result.data
    seedContext.flags := seed.io.result.fflags
    seedContext.refine := seed.io.refine
    val seeded = Pipe(io.reciprocalIn.valid, seedContext, 1)

    when (seeded.valid) {
      error0Lane.io.in.valid := true.B
      error0Lane.io.in.bits.op := VpuFpAluOp.NegMulAdd
      error0Lane.io.in.bits.a := seeded.bits.operand
      error0Lane.io.in.bits.b := seeded.bits.estimate
      error0Lane.io.in.bits.c := "h40000000".U // 2.0f
    }
    val error0Context = pipeReciprocalContext(seeded.valid, seeded.bits)
    assert(!error0Context.valid || error0Lane.io.out.valid)

    val update0InputContext = WireDefault(error0Context.bits)
    update0InputContext.flags := error0Context.bits.flags |
      error0Lane.io.out.bits.fflags
    when (error0Context.valid) {
      update0Lane.io.in.valid := true.B
      update0Lane.io.in.bits.op := VpuFpAluOp.Mul
      update0Lane.io.in.bits.a := error0Context.bits.estimate
      update0Lane.io.in.bits.b := error0Lane.io.out.bits.data
      update0Lane.io.in.bits.c := 0.U
    }
    val update0Context = pipeReciprocalContext(
      error0Context.valid, update0InputContext)
    assert(!update0Context.valid || update0Lane.io.out.valid)

    val error1InputContext = WireDefault(update0Context.bits)
    error1InputContext.estimate := update0Lane.io.out.bits.data
    error1InputContext.flags := update0Context.bits.flags |
      update0Lane.io.out.bits.fflags
    when (update0Context.valid) {
      error1Lane.io.in.valid := true.B
      error1Lane.io.in.bits.op := VpuFpAluOp.NegMulAdd
      error1Lane.io.in.bits.a := update0Context.bits.operand
      error1Lane.io.in.bits.b := update0Lane.io.out.bits.data
      error1Lane.io.in.bits.c := "h40000000".U
    }
    val error1Context = pipeReciprocalContext(
      update0Context.valid, error1InputContext)
    assert(!error1Context.valid || error1Lane.io.out.valid)

    val update1InputContext = WireDefault(error1Context.bits)
    update1InputContext.flags := error1Context.bits.flags |
      error1Lane.io.out.bits.fflags
    when (error1Context.valid) {
      update1Lane.io.in.valid := true.B
      update1Lane.io.in.bits.op := VpuFpAluOp.Mul
      update1Lane.io.in.bits.a := error1Context.bits.estimate
      update1Lane.io.in.bits.b := error1Lane.io.out.bits.data
      update1Lane.io.in.bits.c := 0.U
    }
    val update1Context = pipeReciprocalContext(
      error1Context.valid, update1InputContext)
    assert(!update1Context.valid || update1Lane.io.out.valid)

    io.reciprocalOut.bits.results(group).data :=
      Mux(update1Context.bits.refine, update1Lane.io.out.bits.data,
        update1Context.bits.specialResult)
    io.reciprocalOut.bits.results(group).fflags :=
      Mux(update1Context.bits.refine,
        update1Context.bits.flags | update1Lane.io.out.bits.fflags,
        update1Context.bits.flags)
    update1Context.valid
  }

  io.reciprocalOut.valid := reciprocalFinalValids.head
  when (reciprocalFinalValids.reduce(_ || _)) {
    assert(reciprocalFinalValids.reduce(_ && _),
      "VPU shared reciprocal lane pipelines lost alignment")
  }
  assert(io.reciprocalOut.valid ===
    reciprocalValidHistory(reciprocalLatency - 1),
    "VPU shared reciprocal pipeline violated its fixed latency")

  // ----------------------------------------------------------------------
  // Pipelined reduction tree.  Stage lane counts sum to nLanes-1, so the
  // remaining highest-numbered lane is reserved for the rotating recurrence.
  // ----------------------------------------------------------------------
  private def reductionStageWidths(inputs: Int): Seq[Int] =
    if (inputs <= 1) Nil
    else (inputs / 2) +: reductionStageWidths((inputs + 1) / 2)
  private val stageWidths = reductionStageWidths(p.nLanes)
  private val stageStarts = stageWidths.scanLeft(0)(_ + _).dropRight(1)
  require(stageWidths.sum == p.nLanes - 1)
  private val accumulatorLaneIndex = p.nLanes - 1

  val Seq(reductionIdle, reductionAccumulate, reductionCombine01Issue,
    reductionCombine01Wait, reductionCombine2Issue,
    reductionCombine2Wait) = Enum(6)
  val reductionState = RegInit(reductionIdle)
  val maximum = RegInit(false.B)

  val partialData = RegInit(VecInit(Seq.fill(3)(0.U(32.W))))
  val partialFlags = RegInit(VecInit(Seq.fill(3)(0.U(5.W))))
  val partialSawNaN = RegInit(VecInit(Seq.fill(3)(false.B)))
  val nextPartial = RegInit(0.U(2.W))

  when (io.reductionStart) {
    assert(reductionState === reductionIdle,
      "a new VPU reduction started before the prior result retired")
    maximum := io.reductionMaximum
    partialData(0) := Mux(io.reductionMaximum &&
      VpuFloat.isNaN(io.reductionSeed), neutralMax, io.reductionSeed)
    partialData(1) := Mux(io.reductionMaximum, neutralMax, neutralSum)
    partialData(2) := Mux(io.reductionMaximum, neutralMax, neutralSum)
    partialFlags(0) := Mux(io.reductionMaximum &&
      VpuFloat.isSignalingNaN(io.reductionSeed), "b10000".U, 0.U)
    partialFlags(1) := 0.U
    partialFlags(2) := 0.U
    partialSawNaN(0) := io.reductionMaximum &&
      VpuFloat.isNaN(io.reductionSeed)
    partialSawNaN(1) := false.B
    partialSawNaN(2) := false.B
    nextPartial := 0.U
    reductionState := reductionAccumulate
  }

  when (io.reductionIn.valid) {
    assert(reductionState === reductionAccumulate,
      "a VPU reduction word arrived outside the accumulation phase")
  }
  assert(!(io.aluIn.valid &&
    (io.reductionIn.valid || reductionState =/= reductionIdle)),
    "base-vector ALU and reduction attempted to share the FMA fabric")
  assert(!(io.reciprocalBusy && io.aluIn.valid),
    "base-vector ALU entered the fabric before reciprocal drain")
  assert(!(io.reciprocalBusy && (io.reductionStart ||
    io.reductionIn.valid || reductionState =/= reductionIdle)),
    "reduction and reciprocal attempted to share the FMA fabric")

  // Stage zero substitutes exact neutral values for inactive tail lanes.
  // MAX also strips NaNs from its numeric tree while carrying a separate
  // canonical-NaN bit, matching the architectural RED_MAX propagation rule.
  var stageValues: Seq[UInt] = (0 until p.nLanes).map { lane =>
    val active = io.reductionIn.bits.laneMask(lane)
    val value = io.reductionIn.bits.values(lane)
    Mux(maximum,
      Mux(active && !VpuFloat.isNaN(value), value, neutralMax),
      Mux(active, value, neutralSum))
  }
  var stageValid: Bool = io.reductionIn.valid
  val initialMeta = Wire(new VpuReductionStageMeta)
  initialMeta.maximum := maximum
  initialMeta.last := io.reductionIn.bits.last
  initialMeta.sawNaN := (0 until p.nLanes).map { lane =>
    io.reductionIn.bits.laneMask(lane) &&
      VpuFloat.isNaN(io.reductionIn.bits.values(lane))
  }.reduce(_ || _)
  initialMeta.fflags := (0 until p.nLanes).map { lane =>
    Mux(maximum && io.reductionIn.bits.laneMask(lane) &&
      VpuFloat.isSignalingNaN(io.reductionIn.bits.values(lane)),
      "b10000".U(5.W), 0.U(5.W))
  }.reduce(_ | _)
  var stageMeta: VpuReductionStageMeta = initialMeta

  for (((width, start), stageIndex) <-
      stageWidths.zip(stageStarts).zipWithIndex) {
    val carry = if (stageValues.size % 2 == 1) {
      Some(Pipe(stageValid, stageValues.last, visibleFmaLatency))
    } else None
    for (node <- 0 until width) {
      when (stageValid) {
        lanes(start + node).io.in.valid := true.B
        lanes(start + node).io.in.bits.op :=
          Mux(stageMeta.maximum, VpuFpAluOp.Max, VpuFpAluOp.Add)
        lanes(start + node).io.in.bits.a := stageValues(2 * node)
        lanes(start + node).io.in.bits.b := stageValues(2 * node + 1)
      }
    }

    val delayedMeta = Pipe(stageValid, stageMeta, visibleFmaLatency)
    val stageOutputs = (0 until width).map(node => lanes(start + node).io.out)
    when (delayedMeta.valid) {
      assert(stageOutputs.map(_.valid).reduce(_ && _),
        s"VPU reduction stage $stageIndex lost lane/tag alignment")
      carry.foreach { delayedCarry =>
        assert(delayedCarry.valid,
          s"VPU reduction stage $stageIndex lost its odd-lane carry")
      }
    }
    val nextMeta = Wire(new VpuReductionStageMeta)
    nextMeta := delayedMeta.bits
    nextMeta.fflags := delayedMeta.bits.fflags |
      stageOutputs.map(_.bits.fflags).reduce(_ | _)
    stageValues = stageOutputs.map(_.bits.data) ++ carry.map(_.bits)
    stageValid = delayedMeta.valid
    stageMeta = nextMeta
  }

  val rootValid = stageValid
  val rootData = stageValues.head
  val rootMeta = stageMeta

  // ----------------------------------------------------------------------
  // Distance-three rotating recurrence in the last physical FMA lane.
  // ----------------------------------------------------------------------
  val accumulatorLane = lanes(accumulatorLaneIndex)
  val recurrenceMeta = Wire(new VpuReductionAccumulatorMeta)
  recurrenceMeta.tag := nextPartial
  recurrenceMeta.last := rootMeta.last
  recurrenceMeta.sawNaN := rootMeta.sawNaN
  recurrenceMeta.fflags := rootMeta.fflags
  val recurrenceMetaPipe = Pipe(rootValid, recurrenceMeta, visibleFmaLatency)

  val recurrenceResultValid = recurrenceMetaPipe.valid &&
    accumulatorLane.io.out.valid && reductionState === reductionAccumulate
  when (recurrenceMetaPipe.valid ||
    (accumulatorLane.io.out.valid && reductionState === reductionAccumulate)) {
    assert(recurrenceMetaPipe.valid && accumulatorLane.io.out.valid,
      "VPU rotating accumulator lost its result tag")
  }
  val recurrenceResultFlags = recurrenceMetaPipe.bits.fflags |
    accumulatorLane.io.out.bits.fflags
  val recurrenceResultSawNaN = recurrenceMetaPipe.bits.sawNaN

  val recurrenceForward = recurrenceResultValid &&
    recurrenceMetaPipe.bits.tag === nextPartial
  val selectedPartialData = Mux(recurrenceForward,
    accumulatorLane.io.out.bits.data, partialData(nextPartial))
  val selectedPartialFlags = Mux(recurrenceForward,
    recurrenceResultFlags, partialFlags(nextPartial))
  val selectedPartialSawNaN = Mux(recurrenceForward,
    recurrenceResultSawNaN | partialSawNaN(nextPartial),
    partialSawNaN(nextPartial))

  when (rootValid) {
    assert(reductionState === reductionAccumulate,
      "a pipelined reduction root escaped its active command")
    accumulatorLane.io.in.valid := true.B
    accumulatorLane.io.in.bits.op :=
      Mux(maximum, VpuFpAluOp.Max, VpuFpAluOp.Add)
    accumulatorLane.io.in.bits.a := selectedPartialData
    accumulatorLane.io.in.bits.b := rootData
    // Capture the accumulator's prior sticky state together with this word.
    recurrenceMeta.fflags := selectedPartialFlags | rootMeta.fflags
    recurrenceMeta.sawNaN := selectedPartialSawNaN | rootMeta.sawNaN
    nextPartial := Mux(nextPartial === 2.U, 0.U, nextPartial + 1.U)
  }

  when (recurrenceResultValid) {
    partialData(recurrenceMetaPipe.bits.tag) := accumulatorLane.io.out.bits.data
    partialFlags(recurrenceMetaPipe.bits.tag) := recurrenceResultFlags
    partialSawNaN(recurrenceMetaPipe.bits.tag) :=
      recurrenceMetaPipe.bits.sawNaN
    when (recurrenceMetaPipe.bits.last) {
      reductionState := reductionCombine01Issue
    }
  }

  val combineData = Reg(UInt(32.W))
  val combineFlags = Reg(UInt(5.W))
  val combineSawNaN = Reg(Bool())

  when (reductionState === reductionCombine01Issue) {
    accumulatorLane.io.in.valid := true.B
    accumulatorLane.io.in.bits.op :=
      Mux(maximum, VpuFpAluOp.Max, VpuFpAluOp.Add)
    accumulatorLane.io.in.bits.a := partialData(0)
    accumulatorLane.io.in.bits.b := partialData(1)
    combineFlags := partialFlags(0) | partialFlags(1)
    combineSawNaN := partialSawNaN(0) | partialSawNaN(1)
    reductionState := reductionCombine01Wait
  }
  when (reductionState === reductionCombine01Wait &&
    accumulatorLane.io.out.valid) {
    combineData := accumulatorLane.io.out.bits.data
    combineFlags := combineFlags | accumulatorLane.io.out.bits.fflags
    reductionState := reductionCombine2Issue
  }
  when (reductionState === reductionCombine2Issue) {
    accumulatorLane.io.in.valid := true.B
    accumulatorLane.io.in.bits.op :=
      Mux(maximum, VpuFpAluOp.Max, VpuFpAluOp.Add)
    accumulatorLane.io.in.bits.a := combineData
    accumulatorLane.io.in.bits.b := partialData(2)
    combineFlags := combineFlags | partialFlags(2)
    combineSawNaN := combineSawNaN | partialSawNaN(2)
    reductionState := reductionCombine2Wait
  }

  val finalValid = reductionState === reductionCombine2Wait &&
    accumulatorLane.io.out.valid
  io.reductionOut.valid := finalValid
  io.reductionOut.bits.data := Mux(maximum && combineSawNaN,
    VpuFloat.canonicalNaN.U, accumulatorLane.io.out.bits.data)
  io.reductionOut.bits.fflags := combineFlags |
    accumulatorLane.io.out.bits.fflags
  when (finalValid) {
    reductionState := reductionIdle
  }

  io.reductionBusy := reductionState =/= reductionIdle
}
