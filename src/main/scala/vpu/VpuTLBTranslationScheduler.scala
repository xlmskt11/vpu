package vpu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.rocket.{MStatus, TLBExceptions}
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.tile.{CoreBundle, HasCoreParameters}

import gemmini.FrontendTLBIO

/** A translation request whose tag is owned by the DMA source-slot allocator.
  *
  * `status` is carried with the request deliberately: a descriptor must be
  * translated with the privilege/virtualization state which was snapshotted
  * when that descriptor was accepted, even if a newer RoCC command arrives
  * while an older request is waiting for a PTW refill.
  */
private[vpu] class VpuTLBTaggedRequest(
    val tagBits: Int,
    val maxSize: Int)(implicit p: Parameters)
    extends CoreBundle with MemoryOpConstants {
  require(tagBits > 0)
  require(maxSize > 0 && isPow2(maxSize))

  val tag = UInt(tagBits.W)
  val vaddr = UInt(vaddrBitsExtended.W)
  val lgSize = UInt(log2Ceil(log2Ceil(maxSize) + 1).W)
  val cmd = UInt(M_SZ.W)
  val status = new MStatus
}

/** The terminal (non-miss) result for one tagged translation request. */
private[vpu] class VpuTLBTaggedResult(
    val tagBits: Int)(implicit p: Parameters) extends CoreBundle {
  require(tagBits > 0)

  val tag = UInt(tagBits.W)
  val paddr = UInt(paddrBits.W)
  val pf = new TLBExceptions
  val gf = new TLBExceptions
  val ae = new TLBExceptions
}

/** Adapts a backpressured tagged request stream to one [[FrontendTLBIO]].
  *
  * FrontendTLB registers a client's Valid request once before producing its
  * response.  `associated` is the matching one-stage tag/payload delay.  On a
  * hit this module can retire that result and launch the next request in the
  * same cycle, so a hit stream sustains one translation per cycle.
  *
  * A Rocket TLB miss is not a terminal response.  While `resp.miss` is high,
  * the request associated with that response is driven back into FrontendTLB
  * combinationally.  Its address, byte-sized TLB size, command, and snapshotted
  * MStatus are therefore identical on every PTW retry; the upstream port is
  * held not-ready until the same request receives a terminal response.
  *
  * `resultQueueEntries` is also the number of logical translations for which
  * this adapter reserves result storage.  Two entries are enough to absorb a
  * one-cycle response while preserving full hit throughput under ordinary
  * ready/valid timing.
  */
private[vpu] class VpuTLBTranslationScheduler(
    tagBits: Int,
    maxSize: Int,
    resultQueueEntries: Int = 2)(implicit val p: Parameters)
    extends Module with HasCoreParameters with MemoryOpConstants {
  require(tagBits > 0)
  require(maxSize > 0 && isPow2(maxSize))
  require(resultQueueEntries > 0)

  val io = IO(new Bundle {
    val request = Flipped(Decoupled(
      new VpuTLBTaggedRequest(tagBits, maxSize)))
    val result = Decoupled(new VpuTLBTaggedResult(tagBits))
    val tlb = new FrontendTLBIO
  })

  // FrontendTLB inserts exactly one request register between this Valid port
  // and the response.  This register is the corresponding tag association.
  val associatedValid = RegInit(false.B)
  val associated = Reg(new VpuTLBTaggedRequest(tagBits, maxSize))

  val responseMiss = associatedValid && io.tlb.resp.miss
  val responseTerminal = associatedValid && !io.tlb.resp.miss

  val resultQueue = Module(new Queue(
    new VpuTLBTaggedResult(tagBits),
    resultQueueEntries,
    pipe = true,
    flow = true))

  resultQueue.io.enq.valid := responseTerminal
  resultQueue.io.enq.bits.tag := associated.tag
  resultQueue.io.enq.bits.paddr := io.tlb.resp.paddr
  resultQueue.io.enq.bits.pf := io.tlb.resp.pf
  resultQueue.io.enq.bits.gf := io.tlb.resp.gf
  resultQueue.io.enq.bits.ae := io.tlb.resp.ae

  io.result.valid := resultQueue.io.deq.valid
  io.result.bits := resultQueue.io.deq.bits
  resultQueue.io.deq.ready := io.result.ready

  // A credit is reserved when a new logical request is launched and is held
  // across all miss retries, queueing, and result backpressure.  Consequently
  // a terminal response can never arrive without an enqueue slot.
  val creditWidth = math.max(1, log2Ceil(resultQueueEntries + 1))
  val reservedResults = RegInit(0.U(creditWidth.W))
  val resultLeaving = io.result.fire
  val freshCapacity = reservedResults < resultQueueEntries.U || resultLeaving

  io.request.ready := !responseMiss && freshCapacity
  val freshRequest = io.request.fire

  // A miss has priority over the upstream stream.  Selecting `associated`
  // here, rather than reconstructing a request from the current DMA state,
  // is what keeps every retry bit-for-bit stable.
  val driven = Wire(new VpuTLBTaggedRequest(tagBits, maxSize))
  driven := io.request.bits
  when(responseMiss) {
    driven := associated
  }

  io.tlb.req.valid := responseMiss || freshRequest
  io.tlb.req.bits.tlb_req.vaddr := driven.vaddr
  io.tlb.req.bits.tlb_req.passthrough := false.B
  // Match Gemmini's DMA translation contract: the TLB translates the first
  // byte, while the independently constructed TileLink A message carries the
  // real 16/32/64-byte lgSize. VpuTLMemory guarantees that every naturally
  // aligned transaction remains within one 4 KiB page.
  io.tlb.req.bits.tlb_req.size := 0.U
  io.tlb.req.bits.tlb_req.cmd := driven.cmd
  io.tlb.req.bits.tlb_req.prv := driven.status.dprv
  io.tlb.req.bits.tlb_req.v := driven.status.dv
  io.tlb.req.bits.status := driven.status

  // Whichever request is presented now owns FrontendTLB's response next
  // cycle.  A miss replay simply reloads this register with identical bits.
  associatedValid := io.tlb.req.valid
  when(io.tlb.req.valid) {
    associated := driven
  }

  when(freshRequest =/= resultLeaving) {
    when(freshRequest) {
      reservedResults := reservedResults + 1.U
    }.otherwise {
      reservedResults := reservedResults - 1.U
    }
  }

  assert(reservedResults <= resultQueueEntries.U,
    "VPU TLB scheduler over-reserved its result queue")
  assert(!responseTerminal || resultQueue.io.enq.ready,
    "VPU TLB scheduler lost a terminal response to result backpressure")

  // A retry must hold both the association and the externally visible TLB
  // request stable for every consecutive miss cycle.
  val wasReplaying = RegNext(responseMiss, false.B)
  val previousReplay = RegEnable(associated, responseMiss)
  when(responseMiss) {
    assert(io.tlb.req.valid,
      "VPU TLB scheduler failed to replay a miss")
    assert(!io.request.ready,
      "VPU TLB scheduler accepted a new request during miss replay")
    assert(driven.asUInt === associated.asUInt,
      "VPU TLB scheduler reconstructed a non-identical miss request")
  }
  when(responseMiss && wasReplaying) {
    assert(associated.asUInt === previousReplay.asUInt,
      "VPU TLB scheduler changed a request while waiting for PTW")
  }

  // DMA source IDs are the intended tags and are small (three bits for the
  // default eight-source engine).  For practical tag widths, this assertion
  // scoreboard proves that retries cannot create a second terminal response
  // for one accepted tag.  Same-cycle retire/reuse of a tag remains legal.
  if (tagBits <= 8) {
    val nTags = 1 << tagBits
    val pendingTags = RegInit(0.U(nTags.W))
    val freshOH = UIntToOH(io.request.bits.tag, nTags)
    val terminalOH = UIntToOH(associated.tag, nTags)
    val sameCycleReuse = freshRequest && responseTerminal &&
      io.request.bits.tag === associated.tag

    when(freshRequest) {
      assert(!(pendingTags & freshOH).orR || sameCycleReuse,
        "VPU TLB scheduler accepted a tag which was already pending")
    }
    when(responseTerminal) {
      assert((pendingTags & terminalOH).orR,
        "VPU TLB scheduler produced more than one terminal result for a tag")
    }

    val afterTerminal = Mux(responseTerminal,
      pendingTags & ~terminalOH, pendingTags)
    pendingTags := Mux(freshRequest, afterTerminal | freshOH, afterTerminal)
  }
}
