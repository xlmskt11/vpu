package vpu

import chisel3._
import chisel3.util._

/** One correctness-first slide/gather command for the existing 1R1W VSRAM.
  *
  * Slide consumes at most two contiguous source lane words for each output
  * word and therefore maps directly onto [[VpuSpadReadRequest]]'s two-address
  * interface. A same-bank pair is serialized by [[VpuBankedScratchpad]].
  * Gather first reads one raw-index word and then performs one source-word
  * read per active destination lane. This deliberately modest implementation
  * adds no multiported gather register file or SRAM crossbar.
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
  val vectorMask = UInt(p.vLen.W)
}

class VpuRearrangeUnit(p: VpuParams) extends Module {
  private val laneIndexBits = math.max(1, log2Ceil(p.nLanes + 1))

  val io = IO(new Bundle {
    val start = Flipped(Decoupled(new VpuRearrangeCommand(p)))
    val readRequest = Decoupled(new VpuSpadReadRequest(p))
    val readResponse = Flipped(Decoupled(new VpuSpadReadResponse(p)))
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

  io.start.ready := state === sIdle
  io.done := false.B
  io.busy := state =/= sIdle
  io.readRequest.valid := false.B
  io.readRequest.bits := 0.U.asTypeOf(io.readRequest.bits)
  io.readResponse.ready := false.B
  io.writeRequest.valid := false.B
  io.writeRequest.bits := 0.U.asTypeOf(io.writeRequest.bits)

  private val maskBits = VecInit(command.vectorMask.asBools)
  private def outputIndex(lane: Int): UInt =
    wordOffset.pad(p.vlBits + 1) + lane.U
  private def outputActive(lane: Int): Bool = {
    val index = outputIndex(lane)
    val inVl = index < command.elementCount
    val selected = maskBits(index(p.vlBits - 1, 0))
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
      state := sSlideReadRequest
    }.otherwise {
      for (lane <- 0 until p.nLanes) {
        resultWord(lane) := 0.U
      }
      state := sWrite
    }
  }

  io.readRequest.valid := state === sSlideReadRequest ||
    state === sGatherIndexRequest || state === sGatherSourceRequest
  io.readRequest.bits.address0 := MuxLookup(state, 0.U, Seq(
    sSlideReadRequest ->
      (command.source + slideFirstWord32(p.elementAddrBits - 1, 0)),
    sGatherIndexRequest -> (command.indices + wordOffset),
    sGatherSourceRequest -> (command.source +
      ((indexWord(gatherLane) / p.nLanes.U) * p.nLanes.U))))
  io.readRequest.bits.address1 := command.source +
    slideLastWord32(p.elementAddrBits - 1, 0)
  io.readRequest.bits.useAddress1 := state === sSlideReadRequest &&
    slideLastWord32 =/= slideFirstWord32
  io.readRequest.bits.tag := 0.U

  when(io.readRequest.fire) {
    switch(state) {
      is(sSlideReadRequest) { state := sSlideReadResponse }
      is(sGatherIndexRequest) { state := sGatherIndexResponse }
      is(sGatherSourceRequest) { state := sGatherSourceResponse }
    }
  }

  io.readResponse.ready := state === sSlideReadResponse ||
    state === sGatherIndexResponse || state === sGatherSourceResponse
  when(io.readResponse.fire) {
    when(state === sSlideReadResponse) {
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
          io.readResponse.bits.data0, io.readResponse.bits.data1)
        resultWord(lane) := Mux(sourceValid,
          selectedWord(sourceLane), 0.U)
      }
      state := sWrite
    }.elsewhen(state === sGatherIndexResponse) {
      indexWord := io.readResponse.bits.data0
      gatherLane := 0.U
      state := sGatherLane
    }.otherwise {
      val sourceLane = indexWord(gatherLane) % p.nLanes.U
      resultWord(gatherLane) := io.readResponse.bits.data0(sourceLane)
      gatherLane := gatherLane + 1.U
      state := sGatherLane
    }
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
