package vpu

import chisel3._
import chisel3.util._
import gemmini.VsramClientIO

/** The VSRAM accesses associated with one reservation-station entry.
  * Allocation records dependencies directly as entry-tag bits, so no
  * wrapping age or sequence number is stored in an entry.
  */
class VpuRsAccessSet(p: VpuParams) extends Bundle {
  val accesses = Vec(3, new VpuHazardAccess(p))
}

class VpuLoadRsAlloc(p: VpuParams) extends Bundle {
  val command = new VpuLoadQueueEntry(p)
  val accessSet = new VpuRsAccessSet(p)
}

class VpuExecuteRsAlloc(p: VpuParams) extends Bundle {
  val command = new VpuExecuteQueueEntry(p)
  val accessSet = new VpuRsAccessSet(p)
}

class VpuStoreRsAlloc(p: VpuParams) extends Bundle {
  val command = new VpuStoreQueueEntry(p)
  val accessSet = new VpuRsAccessSet(p)
}

class VpuLoadRsIssue(p: VpuParams) extends Bundle {
  val command = new VpuLoadQueueEntry(p)
  val tag = UInt(p.rsTagBits.W)
}

class VpuExecuteRsIssue(p: VpuParams) extends Bundle {
  val command = new VpuExecuteQueueEntry(p)
  val tag = UInt(p.rsTagBits.W)
}

class VpuStoreRsIssue(p: VpuParams) extends Bundle {
  val command = new VpuStoreQueueEntry(p)
  val tag = UInt(p.rsTagBits.W)
}

private[vpu] class VpuRsMetadata(p: VpuParams, totalEntries: Int)
    extends Bundle {
  val issued = Bool()
  val deps = UInt(totalEntries.W)
  val accessSet = new VpuRsAccessSet(p)
}

private[vpu] class VpuLoadRsEntry(p: VpuParams, totalEntries: Int)
    extends Bundle {
  val metadata = new VpuRsMetadata(p, totalEntries)
  val command = new VpuLoadQueueEntry(p)
}

private[vpu] class VpuExecuteRsEntry(p: VpuParams, totalEntries: Int)
    extends Bundle {
  val metadata = new VpuRsMetadata(p, totalEntries)
  val command = new VpuExecuteQueueEntry(p)
}

private[vpu] class VpuStoreRsEntry(p: VpuParams, totalEntries: Int)
    extends Bundle {
  val metadata = new VpuRsMetadata(p, totalEntries)
  val command = new VpuStoreQueueEntry(p)
}

object VpuReservationStation {
  // The former execute FIFO could hold p.execQueueEntries waiting commands in
  // addition to the command already resident in the execute engine.  An RS
  // entry lives through completion, so retain that externally visible credit.
  def loadEntries(p: VpuParams): Int = p.loadRsEntries
  def executeEntries(p: VpuParams): Int = p.execRsEntries
  def storeEntries(p: VpuParams): Int = p.storeRsEntries
  def totalEntries(p: VpuParams): Int = p.hazardEntries

  val CompletionPorts = 8
}

/** VPU command reservation station with partitioned LD/EX/ST entries.
  *
  * Each entry owns one global tag and one dependency bitmap.  A new command
  * depends on every conflicting live entry.  It additionally depends on all
  * older, not-yet-issued entries of its own command class, which preserves the
  * FIFO issue contract without an age counter or a physical FIFO.
  *
  * When a producer issues, only same-class, non-conflicting ordering edges are
  * cleared.  A real RAW/WAR/WAW edge (including LD-after-LD WAW) remains until
  * producer completion.  Cross-class edges likewise clear only on completion.
  */
class VpuReservationStation(p: VpuParams) extends Module {
  import VpuReservationStation._

  private val nLd = loadEntries(p)
  private val nEx = executeEntries(p)
  private val nSt = storeEntries(p)
  private val nTotal = totalEntries(p)
  private val exBase = nLd
  private val stBase = nLd + nEx

  require(p.hazardEntries == nLd + nEx + nSt,
    "derived global tag capacity must equal the VPU RS geometry")

  val io = IO(new Bundle {
    val allocate = new Bundle {
      val load = Flipped(Decoupled(new VpuLoadRsAlloc(p)))
      val execute = Flipped(Decoupled(new VpuExecuteRsAlloc(p)))
      val store = Flipped(Decoupled(new VpuStoreRsAlloc(p)))
    }

    val issue = new Bundle {
      val load = Decoupled(new VpuLoadRsIssue(p))
      val execute = Decoupled(new VpuExecuteRsIssue(p))
      val store = Decoupled(new VpuStoreRsIssue(p))
    }

    /** Completion tags may be returned by independent engines in one cycle. */
    val complete = Flipped(Vec(CompletionPorts,
      Valid(UInt(p.rsTagBits.W))))

    /** Atomically discard commands which have not entered an engine yet. */
    val flushUnissued = Input(Bool())

    val vsramDeps = if (p.enableSharedDeps) Some(
      new VsramClientIO(
        p.sharedHazardAddressBits, nTotal)) else None

    val validMask = Output(UInt(nTotal.W))
    val issuedMask = Output(UInt(nTotal.W))
    val flushedMask = Output(UInt(nTotal.W))
    val loadValidMask = Output(UInt(nLd.W))
    val executeValidMask = Output(UInt(nEx.W))
    val storeValidMask = Output(UInt(nSt.W))
    val pendingFpReadMask = Output(UInt(8.W))
    val pendingFpWriteMask = Output(UInt(8.W))
    // Every bit protects one immutable architectural mask version. Issued
    // entries remain valid through completion, so this covers queued and active
    // execute commands without a separately maintained reference counter.
    val maskSlotsInUse = Output(UInt(p.maskSlots.W))
    val loadBusy = Output(Bool())
    val executeBusy = Output(Bool())
    val storeBusy = Output(Bool())
    val busy = Output(Bool())
  })

  val loadEntriesReg = RegInit(VecInit(Seq.fill(nLd)(
    0.U.asTypeOf(Valid(new VpuLoadRsEntry(p, nTotal))))))
  val executeEntriesReg = RegInit(VecInit(Seq.fill(nEx)(
    0.U.asTypeOf(Valid(new VpuExecuteRsEntry(p, nTotal))))))
  val storeEntriesReg = RegInit(VecInit(Seq.fill(nSt)(
    0.U.asTypeOf(Valid(new VpuStoreRsEntry(p, nTotal))))))

  if (p.enableSharedDeps) {
    io.vsramDeps.get.allocate.valid := false.B
    io.vsramDeps.get.allocate.bits :=
      0.U.asTypeOf(io.vsramDeps.get.allocate.bits)
    io.vsramDeps.get.release := 0.U
  }

  private def accessEnd(x: VpuHazardAccess): UInt =
    x.base.pad(p.elementAddrBits + 1) +& x.elementCount

  private def accessesConflict(a: VpuHazardAccess,
                               b: VpuHazardAccess): Bool = {
    val overlap = a.elementCount =/= 0.U && b.elementCount =/= 0.U &&
      a.base.pad(p.elementAddrBits + 1) < accessEnd(b) &&
      b.base.pad(p.elementAddrBits + 1) < accessEnd(a)
    overlap && ((a.write && (b.read || b.write)) ||
      (a.read && b.write))
  }

  private def setsConflict(a: VpuRsAccessSet,
                           b: VpuRsAccessSet): Bool =
    (for {
      ai <- 0 until 3
      bi <- 0 until 3
    } yield accessesConflict(a.accesses(ai), b.accesses(bi))).reduce(_ || _)

  val allValid = VecInit(
    loadEntriesReg.map(_.valid) ++
      executeEntriesReg.map(_.valid) ++
      storeEntriesReg.map(_.valid))
  val allIssued = VecInit(
    loadEntriesReg.map(_.bits.metadata.issued) ++
      executeEntriesReg.map(_.bits.metadata.issued) ++
      storeEntriesReg.map(_.bits.metadata.issued))
  val allAccessSets =
    loadEntriesReg.map(_.bits.metadata.accessSet) ++
      executeEntriesReg.map(_.bits.metadata.accessSet) ++
      storeEntriesReg.map(_.bits.metadata.accessSet)

  io.validMask := allValid.asUInt
  io.issuedMask := allIssued.asUInt & allValid.asUInt
  io.loadValidMask := VecInit(loadEntriesReg.map(_.valid)).asUInt
  io.executeValidMask := VecInit(executeEntriesReg.map(_.valid)).asUInt
  io.storeValidMask := VecInit(storeEntriesReg.map(_.valid)).asUInt
  io.loadBusy := io.loadValidMask.orR
  io.executeBusy := io.executeValidMask.orR
  io.storeBusy := io.storeValidMask.orR
  io.busy := io.validMask.orR
  dontTouch(io.loadBusy)
  dontTouch(io.executeBusy)
  dontTouch(io.storeBusy)

  // Completion is deliberately a bit mask rather than a selected update: it
  // permits an LD, EX, ST, and fault cancellation to retire concurrently.
  def tagToOH(tag: UInt): UInt =
    VecInit((0 until nTotal).map(i => tag === i.U)).asUInt

  // Chisel's ordinary UInt `+` keeps the maximum operand width.  Class bases
  // such as the default ST base (13, four bits) must therefore be widened
  // before adding a local ID: 13 + 3 is global tag 16, not four-bit zero.
  def globalTag(classBase: Int, localId: UInt): UInt =
    classBase.U(p.rsTagBits.W) + localId

  val completionMask = VecInit(io.complete.map { port =>
    Mux(port.valid, tagToOH(port.bits), 0.U(nTotal.W))
  }).reduce(_ | _)
  for (port <- io.complete) {
    when(port.valid) {
      assert(port.bits < nTotal.U,
        "VPU reservation-station completion tag is out of range")
    }
  }
  for (a <- 0 until CompletionPorts; b <- a + 1 until CompletionPorts) {
    assert(!(io.complete(a).valid && io.complete(b).valid &&
      io.complete(a).bits === io.complete(b).bits),
      "VPU reservation-station received a duplicate completion tag")
  }
  for (tag <- 0 until nTotal) {
    when(completionMask(tag)) {
      assert(allValid(tag),
        "VPU reservation-station completed an invalid entry")
      assert(allIssued(tag),
        "VPU reservation-station completed an entry which was never issued")
    }
  }

  val flushMask = Mux(io.flushUnissued,
    allValid.asUInt & ~allIssued.asUInt, 0.U(nTotal.W))
  val releaseMask = completionMask | flushMask
  io.flushedMask := flushMask
  val liveAfterRelease = allValid.asUInt & ~releaseMask
  if (p.enableSharedDeps) {
    io.vsramDeps.get.release := releaseMask
  }

  // Allocation may recycle a completing slot in the same cycle.  It never
  // recycles a merely flushed slot because allocation is disabled on flush.
  val loadReusable = ~io.loadValidMask |
    completionMask(nLd - 1, 0)
  val executeReusable = ~io.executeValidMask |
    completionMask(exBase + nEx - 1, exBase)
  val storeReusable = ~io.storeValidMask |
    completionMask(stBase + nSt - 1, stBase)
  val loadAllocOH = PriorityEncoderOH(loadReusable)
  val executeAllocOH = PriorityEncoderOH(executeReusable)
  val storeAllocOH = PriorityEncoderOH(storeReusable)
  val loadAllocId = OHToUInt(loadAllocOH)
  val executeAllocId = OHToUInt(executeAllocOH)
  val storeAllocId = OHToUInt(storeAllocOH)

  io.allocate.load.ready := !io.flushUnissued && loadReusable.orR
  io.allocate.execute.ready := !io.flushUnissued && executeReusable.orR
  io.allocate.store.ready := !io.flushUnissued && storeReusable.orR
  assert(PopCount(Seq(io.allocate.load.fire, io.allocate.execute.fire,
    io.allocate.store.fire)) <= 1.U,
    "VPU reservation station accepts at most one RoCC command per cycle")

  if (p.enableSharedDeps) {
    val allocationFires = Seq(io.allocate.load.fire,
      io.allocate.execute.fire, io.allocate.store.fire)
    val selectedSlot = Mux1H(Seq(
      io.allocate.load.fire -> loadAllocId,
      io.allocate.execute.fire -> globalTag(exBase, executeAllocId),
      io.allocate.store.fire -> globalTag(stBase, storeAllocId)))
    val selectedSet = Mux1H(Seq(
      io.allocate.load.fire -> io.allocate.load.bits.accessSet,
      io.allocate.execute.fire -> io.allocate.execute.bits.accessSet,
      io.allocate.store.fire -> io.allocate.store.bits.accessSet))

    io.vsramDeps.get.allocate.valid := allocationFires.reduce(_ || _)
    io.vsramDeps.get.allocate.bits.slot := selectedSlot

    for (accessIndex <- 0 until 3) {
      val local = selectedSet.accesses(accessIndex)
      val rangeWidth = math.max(p.elementAddrBits,
        p.dmaTransferElementsBits) + 1
      val base = local.base.pad(rangeWidth)
      val count = local.elementCount.pad(rangeWidth)
      val endElement = base +& count
      val startRow = base / p.matrixElementsPerRow.U
      val endRow = (endElement + (p.matrixElementsPerRow - 1).U) /
        p.matrixElementsPerRow.U
      val shared = io.vsramDeps.get.allocate.bits.accesses(accessIndex)

      shared.valid := local.elementCount =/= 0.U &&
        (local.read || local.write)
      shared.start := startRow
      shared.end := endRow
      shared.wraps_around := false.B
      shared.read := local.read
      shared.write := local.write

      when (io.vsramDeps.get.allocate.valid && shared.valid) {
        assert(endElement <= p.totalElements.U,
          "VPU shared dependency range escaped VSRAM")
        assert(endRow <= p.matrixRows.U,
          "VPU shared dependency matrix-row range escaped VSRAM")
      }
    }
  }

  // A dependency chain, rather than physical index order, identifies the
  // oldest ready entry even after arbitrary slots have been recycled.
  val loadReady = VecInit(loadEntriesReg.zipWithIndex.map { case (entry, slot) =>
    entry.valid && !entry.bits.metadata.issued &&
      !entry.bits.metadata.deps.orR &&
      (if (p.enableSharedDeps) io.vsramDeps.get.ready(slot) else true.B)
  })
  val executeReady = VecInit(executeEntriesReg.zipWithIndex.map { case (entry, slot) =>
    entry.valid && !entry.bits.metadata.issued &&
      !entry.bits.metadata.deps.orR &&
      (if (p.enableSharedDeps) io.vsramDeps.get.ready(exBase + slot) else true.B)
  })
  val storeReady = VecInit(storeEntriesReg.zipWithIndex.map { case (entry, slot) =>
    entry.valid && !entry.bits.metadata.issued &&
      !entry.bits.metadata.deps.orR &&
      (if (p.enableSharedDeps) io.vsramDeps.get.ready(stBase + slot) else true.B)
  })
  val loadIssueOH = PriorityEncoderOH(loadReady)
  val executeIssueOH = PriorityEncoderOH(executeReady)
  val storeIssueOH = PriorityEncoderOH(storeReady)
  val loadIssueId = OHToUInt(loadIssueOH)
  val executeIssueId = OHToUInt(executeIssueOH)
  val storeIssueId = OHToUInt(storeIssueOH)

  io.issue.load.valid := loadReady.asUInt.orR && !io.flushUnissued
  io.issue.load.bits.command := Mux1H(loadIssueOH,
    loadEntriesReg.map(_.bits.command))
  io.issue.load.bits.tag := loadIssueId
  io.issue.execute.valid := executeReady.asUInt.orR && !io.flushUnissued
  io.issue.execute.bits.command := Mux1H(executeIssueOH,
    executeEntriesReg.map(_.bits.command))
  io.issue.execute.bits.tag := globalTag(exBase, executeIssueId)
  io.issue.store.valid := storeReady.asUInt.orR && !io.flushUnissued
  io.issue.store.bits.command := Mux1H(storeIssueOH,
    storeEntriesReg.map(_.bits.command))
  io.issue.store.bits.tag := globalTag(stBase, storeIssueId)

  when(io.issue.execute.valid) {
    assert(io.issue.execute.bits.tag >= exBase.U &&
      io.issue.execute.bits.tag < stBase.U,
      "VPU execute reservation-station tag escaped its class partition")
  }
  when(io.issue.store.valid) {
    assert(io.issue.store.bits.tag >= stBase.U &&
      io.issue.store.bits.tag < nTotal.U,
      "VPU store reservation-station tag escaped its class partition")
  }

  val loadIssueFire = io.issue.load.fire
  val executeIssueFire = io.issue.execute.fire
  val storeIssueFire = io.issue.store.fire
  val loadIssueGlobalOH = Mux(loadIssueFire,
    tagToOH(loadIssueId), 0.U(nTotal.W))
  val executeIssueGlobalOH = Mux(executeIssueFire,
    tagToOH(globalTag(exBase, executeIssueId)), 0.U(nTotal.W))
  val storeIssueGlobalOH = Mux(storeIssueFire,
    tagToOH(globalTag(stBase, storeIssueId)), 0.U(nTotal.W))

  // New allocations see same-cycle completion and issue state.  This avoids
  // leaving a stale edge when a producer issues while a younger command is
  // allocated, and makes same-cycle completion/tag reuse unambiguous.
  def allocationDeps(newSet: VpuRsAccessSet,
                     classBase: Int, classEntries: Int): UInt = {
    VecInit((0 until nTotal).map { tag =>
      val actualConflict = setsConflict(newSet, allAccessSets(tag))
      val sameClass = tag >= classBase && tag < classBase + classEntries
      val issuingThisEntry =
        if (sameClass && classBase == 0) loadIssueGlobalOH(tag)
        else if (sameClass && classBase == exBase) executeIssueGlobalOH(tag)
        else if (sameClass && classBase == stBase) storeIssueGlobalOH(tag)
        else false.B
      val orderDependency = sameClass.B && !allIssued(tag) &&
        !issuingThisEntry
      liveAfterRelease(tag) && (actualConflict || orderDependency)
    }).asUInt
  }

  val newLoadDeps = allocationDeps(io.allocate.load.bits.accessSet, 0, nLd)
  val newExecuteDeps = allocationDeps(
    io.allocate.execute.bits.accessSet, exBase, nEx)
  val newStoreDeps = allocationDeps(
    io.allocate.store.bits.accessSet, stBase, nSt)

  // FP control-register ordering can be derived directly from live EX entries;
  // no wrapping counter or separately maintained pending-count register is
  // required after Core integration.
  io.pendingFpReadMask := executeEntriesReg.map { entry =>
    Mux(entry.valid, entry.bits.command.fpReadMask, 0.U(8.W))
  }.reduce(_ | _)
  io.pendingFpWriteMask := executeEntriesReg.map { entry =>
    Mux(entry.valid, entry.bits.command.fpWriteMask, 0.U(8.W))
  }.reduce(_ | _)
  io.maskSlotsInUse := executeEntriesReg.map { entry =>
    Mux(entry.valid && entry.bits.command.maskEnable,
      UIntToOH(entry.bits.command.maskSlot, p.maskSlots),
      0.U(p.maskSlots.W))
  }.reduce(_ | _)

  when(io.allocate.execute.fire &&
      io.allocate.execute.bits.command.maskEnable) {
    assert(io.allocate.execute.bits.command.maskSlot < p.maskSlots.U,
      "VPU execute command selected an invalid vector-mask slot")
  }

  // Existing entries first consume all completion/flush releases and then the
  // issue-time clear for their own class.  A same-class actual conflict is
  // explicitly retained until completion.
  for ((entry, localId) <- loadEntriesReg.zipWithIndex) {
    val issueOrderingClear = Mux(loadIssueFire &&
      !setsConflict(entry.bits.metadata.accessSet,
        loadEntriesReg(loadIssueId).bits.metadata.accessSet),
      loadIssueGlobalOH, 0.U(nTotal.W))
    val nextDeps = entry.bits.metadata.deps &
      ~releaseMask & ~issueOrderingClear
    val allocateHere = io.allocate.load.fire && loadAllocId === localId.U
    when(allocateHere) {
      entry.valid := true.B
      entry.bits.metadata.issued := false.B
      entry.bits.metadata.deps := newLoadDeps
      entry.bits.metadata.accessSet := io.allocate.load.bits.accessSet
      entry.bits.command := io.allocate.load.bits.command
    }.elsewhen(releaseMask(localId)) {
      entry.valid := false.B
      entry.bits.metadata.deps := 0.U
    }.otherwise {
      entry.bits.metadata.deps := nextDeps
      when(loadIssueFire && loadIssueId === localId.U) {
        entry.bits.metadata.issued := true.B
      }
    }
  }

  for ((entry, localId) <- executeEntriesReg.zipWithIndex) {
    val globalId = exBase + localId
    val issueOrderingClear = Mux(executeIssueFire &&
      !setsConflict(entry.bits.metadata.accessSet,
        executeEntriesReg(executeIssueId).bits.metadata.accessSet),
      executeIssueGlobalOH, 0.U(nTotal.W))
    val nextDeps = entry.bits.metadata.deps &
      ~releaseMask & ~issueOrderingClear
    val allocateHere = io.allocate.execute.fire &&
      executeAllocId === localId.U
    when(allocateHere) {
      entry.valid := true.B
      entry.bits.metadata.issued := false.B
      entry.bits.metadata.deps := newExecuteDeps
      entry.bits.metadata.accessSet := io.allocate.execute.bits.accessSet
      entry.bits.command := io.allocate.execute.bits.command
    }.elsewhen(releaseMask(globalId)) {
      entry.valid := false.B
      entry.bits.metadata.deps := 0.U
    }.otherwise {
      entry.bits.metadata.deps := nextDeps
      when(executeIssueFire && executeIssueId === localId.U) {
        entry.bits.metadata.issued := true.B
      }
    }
  }

  for ((entry, localId) <- storeEntriesReg.zipWithIndex) {
    val globalId = stBase + localId
    val issueOrderingClear = Mux(storeIssueFire &&
      !setsConflict(entry.bits.metadata.accessSet,
        storeEntriesReg(storeIssueId).bits.metadata.accessSet),
      storeIssueGlobalOH, 0.U(nTotal.W))
    val nextDeps = entry.bits.metadata.deps &
      ~releaseMask & ~issueOrderingClear
    val allocateHere = io.allocate.store.fire && storeAllocId === localId.U
    when(allocateHere) {
      entry.valid := true.B
      entry.bits.metadata.issued := false.B
      entry.bits.metadata.deps := newStoreDeps
      entry.bits.metadata.accessSet := io.allocate.store.bits.accessSet
      entry.bits.command := io.allocate.store.bits.command
    }.elsewhen(releaseMask(globalId)) {
      entry.valid := false.B
      entry.bits.metadata.deps := 0.U
    }.otherwise {
      entry.bits.metadata.deps := nextDeps
      when(storeIssueFire && storeIssueId === localId.U) {
        entry.bits.metadata.issued := true.B
      }
    }
  }

  when(io.allocate.load.fire) {
    assert(!newLoadDeps(loadAllocId),
      "new VPU load reservation entry depends on its own recycled tag")
  }
  when(io.allocate.execute.fire) {
    assert(!newExecuteDeps(globalTag(exBase, executeAllocId)),
      "new VPU execute reservation entry depends on its own recycled tag")
  }
  when(io.allocate.store.fire) {
    assert(!newStoreDeps(globalTag(stBase, storeAllocId)),
      "new VPU store reservation entry depends on its own recycled tag")
  }
}
