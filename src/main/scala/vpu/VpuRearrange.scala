package vpu

import chisel3._
import chisel3.util._

/** One correctness-first slide/gather command for the existing 1R1W VSRAM.
  *
  * Slide independently requests the at-most-two contiguous source lane words
  * needed by one output word. Different banks can return both words together;
  * a shared bank arbitrates them one at a time, exactly like Gemmini's
  * independent operand streams. Gather uses source-0 for its index and source
  * reads. This deliberately modest implementation adds no multiported gather
  * register file or SRAM crossbar.
  */
class VpuRearrangeCommand(p: VpuParams) extends Bundle {
  val gather = Bool()
  // false: dst[i] = i >= shift ? src[i-shift] : 0 (toward high indices)
  // true:  dst[i] = i+shift < VL ? src[i+shift] : 0 (toward low indices)
  val slideLow = Bool()
  val destination = UInt(p.elementAddrBits.W)
  val source = UInt(p.elementAddrBits.W)
  val indices = UInt(p.elementAddrBits.W)
  val shift = UInt(32.W)
  val elementCount = UInt(p.vlBits.W)
  val maskEnable = Bool()
}

class VpuRearrangeUnit(p: VpuParams) extends Module {
  private val laneIndexBits = math.max(1, log2Ceil(p.nLanes + 1))

  val io = IO(new Bundle {
    val start = Flipped(Decoupled(new VpuRearrangeCommand(p)))
    val readRequest = Vec(2, Decoupled(new VpuSpadReadRequest(p)))
    val readResponse = Flipped(Vec(2,
      Decoupled(new VpuSpadReadResponse(p))))
    // The architectural mask file supplies only the lane word used by the
    // current output word. Keeping a complete VLEN-bit mask in this active
    // command would duplicate state merely because rearrange is iterative.
    val maskWordIndex = Output(UInt(p.maskWordIndexBits.W))
    val maskWord = Input(UInt(p.nLanes.W))
    val writeRequest = Decoupled(new VpuSpadWriteRequest(p))
    val done = Output(Bool())
    val busy = Output(Bool())
  })

  val Seq(sIdle, sSlidePrepare, sSlideReadRequest, sSlideReadResponse,
    sGatherIndexRequest, sGatherIndexResponse, sGatherLane,
    sGatherSourceRequest, sGatherSourceResponse, sWrite) = Enum(10)
  val state = RegInit(sIdle)
  val command = Reg(new VpuRearrangeCommand(p))
  val wordOffset = RegInit(0.U(p.vlBits.W))
  val gatherLane = RegInit(0.U(laneIndexBits.W))
  val indexWord = Reg(Vec(p.nLanes, UInt(p.storageBits.W)))
  val resultWord = Reg(Vec(p.nLanes, UInt(p.storageBits.W)))
  val slideSource0Requested = RegInit(false.B)
  val slideSource1Requested = RegInit(false.B)

  io.start.ready := state === sIdle
  io.done := false.B
  io.busy := state =/= sIdle
  for (operand <- 0 until 2) {
    io.readRequest(operand).valid := false.B
    io.readRequest(operand).bits :=
      0.U.asTypeOf(io.readRequest(operand).bits)
    io.readResponse(operand).ready := false.B
  }
  io.writeRequest.valid := false.B
  io.writeRequest.bits := 0.U.asTypeOf(io.writeRequest.bits)

  io.maskWordIndex := (wordOffset / p.nLanes.U)(p.maskWordIndexBits - 1, 0)
  private val maskBits = VecInit(io.maskWord.asBools)
  private def outputIndex(lane: Int): UInt =
    wordOffset.pad(p.vlBits + 1) + lane.U
  private def outputActive(lane: Int): Bool = {
    val index = outputIndex(lane)
    val inVl = index < command.elementCount
    val selected = maskBits(lane)
    inVl && (!command.maskEnable || selected)
  }

  private val count32 = command.elementCount.pad(32)
  private val offset32 = wordOffset.pad(32)
  private val wordEnd32 = offset32 + (p.nLanes - 1).U
  private val lastElement32 = Mux(count32 === 0.U, 0.U, count32 - 1.U)
  private val lastOutput32 = Mux(wordEnd32 < lastElement32,
    wordEnd32, lastElement32)
  private val rightFirstOutput32 = Mux(offset32 > command.shift,
    offset32, command.shift)
  private val slideFirstSource32 = Mux(command.slideLow,
    offset32 + command.shift, rightFirstOutput32 - command.shift)
  private val slideLastSource32 = Mux(command.slideLow,
    Mux(lastOutput32 + command.shift < lastElement32,
      lastOutput32 + command.shift, lastElement32),
    lastOutput32 - command.shift)
  private val slideHasSource = Mux(command.slideLow,
    command.shift < count32 && offset32 + command.shift < count32,
    command.shift < count32 && lastOutput32 >= command.shift)
  private val slideFirstWord32 =
    (slideFirstSource32 / p.nLanes.U) * p.nLanes.U
  private val slideLastWord32 =
    (slideLastSource32 / p.nLanes.U) * p.nLanes.U

  when(io.start.fire) {
    command := io.start.bits
    gatherLane := 0.U
    slideSource0Requested := false.B
    slideSource1Requested := false.B
    for (lane <- 0 until p.nLanes) {
      resultWord(lane) := 0.U
    }
    val lastWord = ((io.start.bits.elementCount - 1.U) /
      p.nLanes.U) * p.nLanes.U
    wordOffset := Mux(io.start.bits.gather || io.start.bits.slideLow,
      0.U, lastWord)
    when(io.start.bits.elementCount === 0.U) {
      state := sIdle
      io.done := true.B
    }.elsewhen(io.start.bits.gather) {
      state := sGatherIndexRequest
    }.otherwise {
      state := sSlidePrepare
    }
  }

  when(state === sSlidePrepare) {
    when(slideHasSource) {
      slideSource0Requested := false.B
      slideSource1Requested := false.B
      state := sSlideReadRequest
    }.otherwise {
      for (lane <- 0 until p.nLanes) {
        resultWord(lane) := 0.U
      }
      state := sWrite
    }
  }

  private val slideNeedsSource1 = slideLastWord32 =/= slideFirstWord32
  io.readRequest(0).valid :=
    (state === sSlideReadRequest && !slideSource0Requested) ||
      state === sGatherIndexRequest || state === sGatherSourceRequest
  io.readRequest(0).bits.address := MuxLookup(state, 0.U, Seq(
    sSlideReadRequest ->
      (command.source + slideFirstWord32(p.elementAddrBits - 1, 0)),
    sGatherIndexRequest -> (command.indices + wordOffset),
    sGatherSourceRequest -> (command.source +
      ((indexWord(gatherLane) / p.nLanes.U) * p.nLanes.U))))
  io.readRequest(0).bits.tag := 0.U
  io.readRequest(1).valid := state === sSlideReadRequest &&
    slideNeedsSource1 && !slideSource1Requested
  io.readRequest(1).bits.address := command.source +
    slideLastWord32(p.elementAddrBits - 1, 0)
  io.readRequest(1).bits.tag := 0.U

  when(io.readRequest(0).fire) {
    switch(state) {
      is(sGatherIndexRequest) { state := sGatherIndexResponse }
      is(sGatherSourceRequest) { state := sGatherSourceResponse }
    }
  }
  when(state === sSlideReadRequest && io.readRequest(0).fire) {
    slideSource0Requested := true.B
  }
  when(state === sSlideReadRequest && io.readRequest(1).fire) {
    slideSource1Requested := true.B
  }
  val slideSource0RequestDone = slideSource0Requested ||
    io.readRequest(0).fire
  val slideSource1RequestDone = !slideNeedsSource1 ||
    slideSource1Requested || io.readRequest(1).fire
  when(state === sSlideReadRequest && slideSource0RequestDone &&
      slideSource1RequestDone) {
    state := sSlideReadResponse
  }

  val slideResponsesAvailable = io.readResponse(0).valid &&
    (!slideNeedsSource1 || io.readResponse(1).valid)
  io.readResponse(0).ready :=
    (state === sSlideReadResponse && slideResponsesAvailable) ||
      state === sGatherIndexResponse || state === sGatherSourceResponse
  io.readResponse(1).ready := state === sSlideReadResponse &&
    slideNeedsSource1 && slideResponsesAvailable
  val slideResponseFire = state === sSlideReadResponse &&
    io.readResponse(0).fire &&
    (!slideNeedsSource1 || io.readResponse(1).fire)
  when(slideResponseFire) {
      for (lane <- 0 until p.nLanes) {
        val out = outputIndex(lane).pad(32)
        val sourceValid = Mux(command.slideLow,
          out < count32 && out + command.shift < count32,
          out < count32 && out >= command.shift)
        val sourceIndex = Mux(sourceValid,
          Mux(command.slideLow, out + command.shift,
            out - command.shift), 0.U)
        val sourceWord = (sourceIndex / p.nLanes.U) * p.nLanes.U
        val sourceLane = sourceIndex % p.nLanes.U
        val selectedWord = Mux(sourceWord === slideFirstWord32,
          io.readResponse(0).bits.data, io.readResponse(1).bits.data)
        resultWord(lane) := Mux(sourceValid,
          selectedWord(sourceLane), 0.U)
      }
      state := sWrite
  }.elsewhen(state === sGatherIndexResponse && io.readResponse(0).fire) {
      indexWord := io.readResponse(0).bits.data
      gatherLane := 0.U
      state := sGatherLane
  }.elsewhen(state === sGatherSourceResponse &&
      io.readResponse(0).fire) {
      val sourceLane = indexWord(gatherLane) % p.nLanes.U
      resultWord(gatherLane) := io.readResponse(0).bits.data(sourceLane)
      gatherLane := gatherLane + 1.U
      state := sGatherLane
  }

  when(state === sGatherLane) {
    when(gatherLane === p.nLanes.U) {
      state := sWrite
    }.otherwise {
      val active = VecInit((0 until p.nLanes).map(outputActive))(gatherLane)
      val rawIndex = indexWord(gatherLane)
      when(!active) {
        resultWord(gatherLane) := 0.U
        gatherLane := gatherLane + 1.U
      }.elsewhen(rawIndex >= command.elementCount) {
        // Match RVV vrgather's useful out-of-range convention.
        resultWord(gatherLane) := 0.U
        gatherLane := gatherLane + 1.U
      }.otherwise {
        state := sGatherSourceRequest
      }
    }
  }

  io.writeRequest.valid := state === sWrite
  io.writeRequest.bits.address := command.destination + wordOffset
  io.writeRequest.bits.data := resultWord
  for (lane <- 0 until p.nLanes) {
    io.writeRequest.bits.laneMask(lane) := outputActive(lane)
  }

  when(io.writeRequest.fire) {
    val ascending = command.gather || command.slideLow
    val ascendingLast = wordOffset + p.nLanes.U >= command.elementCount
    val descendingLast = wordOffset === 0.U
    when(Mux(ascending, ascendingLast, descendingLast)) {
      state := sIdle
      io.done := true.B
    }.otherwise {
      wordOffset := Mux(ascending,
        wordOffset + p.nLanes.U, wordOffset - p.nLanes.U)
      gatherLane := 0.U
      for (lane <- 0 until p.nLanes) {
        resultWord(lane) := 0.U
      }
      state := Mux(command.gather,
        sGatherIndexRequest, sSlidePrepare)
    }
  }

  when(state =/= sIdle) {
    assert(command.elementCount =/= 0.U,
      "a zero-length VPU rearrange command became active")
  }
}
