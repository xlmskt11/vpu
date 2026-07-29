package vpu

import chisel3._
import chisel3.util._

/** Allocates identities for store commands after their architectural
  * reservation-station entry has retired.
  *
  * A store may release its RS entry as soon as its final VSRAM read request is
  * accepted, but the captured payload can remain in the DMA writer until the
  * corresponding TileLink D response arrives.  Reusing the RS entry number as
  * the DMA command identity would therefore alias an older in-flight store
  * with a newly allocated command.  This allocator provides an independent
  * tag lifetime:
  *
  *   allocate.fire        -> writer descriptor owns the tag
  *   release.valid        -> writer terminal completion (all D responses)
  *
  * A tag released in a cycle is immediately eligible for allocation in that
  * same cycle.  This preserves full throughput when every transport slot is
  * occupied and one completion arrives together with a new descriptor.
  */
class VpuStoreTransportTagAllocator(p: VpuParams) extends Module {
  private val nTags = p.storeQueueEntries
  private val tagBits = p.dmaCommandTagBits
  private val countBits = math.max(1, log2Ceil(nTags + 1))

  require(nTags > 0)
  require((BigInt(1) << tagBits) >= nTags,
    "DMA command tag width must encode every store transport slot")

  val io = IO(new Bundle {
    /** The caller asserts ready only when the writer descriptor is accepted. */
    val allocate = Decoupled(UInt(tagBits.W))
    /** Pulse when the writer's terminal completion is consumed. */
    val release = Flipped(Valid(UInt(tagBits.W)))

    val busy = Output(Bool())
    val full = Output(Bool())
    val outstanding = Output(UInt(countBits.W))
    val available = Output(UInt(countBits.W))
    val capacity = Output(UInt(countBits.W))
    /** Kept visible for waveform/debug assertions. */
    val activeMask = Output(UInt(nTags.W))
  })

  val active = RegInit(0.U(nTags.W))

  val releaseInRange = io.release.bits < nTags.U
  val releaseOH = UIntToOH(io.release.bits, nTags)
  val releaseWasActive = releaseInRange && (active & releaseOH).orR
  val releaseAccepted = io.release.valid && releaseWasActive
  val releaseMask = Mux(releaseAccepted, releaseOH, 0.U(nTags.W))

  when(io.release.valid) {
    assert(releaseInRange,
      "VPU store transport completion tag is outside the allocator")
    assert(releaseWasActive,
      "VPU store transport tag was released twice or before allocation")
  }

  // Treat a valid release as free combinationally.  When the table is full,
  // the same tag can consequently move directly from the retiring writer
  // command to a newly accepted descriptor without an idle cycle.
  val allocatableMask = (~active).asUInt | releaseMask
  val allocateOH = PriorityEncoderOH(allocatableMask)
  io.allocate.valid := allocatableMask.orR
  // Normal UInt connection zero-extends into the public DMA tag width.  An
  // explicit pad is invalid for a few small parameterizations where Chisel's
  // inferred PriorityEncoder width is already wider than log2Ceil(nTags).
  io.allocate.bits := OHToUInt(allocateOH)

  val allocateMask = Mux(io.allocate.fire, allocateOH, 0.U(nTags.W))
  when(io.allocate.fire) {
    assert(allocateOH.orR,
      "VPU store transport allocation fired without a free tag")
    assert(!(active & allocateOH).orR || (releaseMask & allocateOH).orR,
      "VPU store transport allocator issued a duplicate live tag")
  }

  // One net state update handles allocate-only, release-only, and a same-cycle
  // handoff of either the same or different tags without assignment priority.
  active := (active & ~releaseMask) | allocateMask

  val activeCount = PopCount(active)
  io.busy := active.orR
  io.full := active.andR
  io.outstanding := activeCount
  io.available := nTags.U - activeCount
  io.capacity := nTags.U
  io.activeMask := active

  dontTouch(io.busy)
  dontTouch(io.outstanding)
}
