package vpu

import chisel3._
import chisel3.util._

import freechips.rocketchip.rocket.MStatus
import gemmini.{GemminiVpuMatrixReadIO, GemminiVpuMatrixWriteReq,
  VsramClientIO}

class VpuLoadQueueEntry(p: VpuParams) extends Bundle {
  val vaddr = UInt(64.W)
  val spadBase = UInt(p.elementAddrBits.W)
  val elementCount = UInt(p.vlBits.W)
  val rowCount = UInt(p.dmaRowCountBits.W)
  val hostStrideBytes = UInt(64.W)
  val status = new MStatus
  val hazardTag = UInt(p.rsTagBits.W)
}

class VpuStoreQueueEntry(p: VpuParams) extends Bundle {
  val vaddr = UInt(64.W)
  val spadBase = UInt(p.elementAddrBits.W)
  val elementCount = UInt(p.vlBits.W)
  val rowCount = UInt(p.dmaRowCountBits.W)
  val hostStrideBytes = UInt(64.W)
  val status = new MStatus
  val hazardTag = UInt(p.rsTagBits.W)
  // The reservation-station tag may be recycled once the final VSRAM read is
  // accepted.  Store payload/TL-D retirement therefore uses this independent
  // transport identity.
  val transportTag = UInt(p.dmaCommandTagBits.W)
}

class VpuExecuteQueueEntry(p: VpuParams) extends Bundle {
  val opcode = UInt(6.W)
  val funct1 = UInt(4.W)
  val destination = UInt(p.elementAddrBits.W)
  val source0 = UInt(p.elementAddrBits.W)
  val source1 = UInt(p.elementAddrBits.W)
  val elementCount = UInt(p.vlBits.W)
  // Zero keeps the original contiguous vector layout. A nonzero value is the
  // start-to-start element stride between matrixElementsPerRow-sized segments
  // of this logical vector and is snapshotted at RS allocation.
  val vectorStride = UInt(p.elementAddrBits.W)
  val useSource1 = Bool()
  // rs3=1 selects a dispatch-time mask version. The protected mask file keeps
  // that version immutable while this compact slot ID remains live in the RS.
  val maskEnable = Bool()
  val maskSlot = UInt(p.maskSlotBits.W)
  // V_SLIDE_V consumes GP[rs2] as an element displacement rather than a
  // second VSRAM address. Preserve all 32 bits at dispatch.
  val gpRs2Value = UInt(32.W)
  // FP register indices and access masks are captured at dispatch. Operand
  // values are deliberately read only when this RS entry starts execution;
  // same-class dependency edges and the single execute engine guarantee that
  // every older FP producer has committed first.
  val fpRs1 = UInt(3.W)
  val fpRs2 = UInt(3.W)
  val fpSeed = UInt(3.W)
  val fpReadMask = UInt(8.W)
  val fpWriteMask = UInt(8.W)
  // S_LOAD_STATE uses GP[rs1], while S_STORE_STATE uses GP[rd]. The selected
  // value is range-checked and snapshotted when the command enters the RS.
  val stateIndex = UInt(p.fpStateIndexBits.W)
  val fpDestination = UInt(3.W)
  val hazardTag = UInt(p.rsTagBits.W)
}

class VpuAluWriteback(p: VpuParams) extends Bundle {
  val address = UInt(p.elementAddrBits.W)
  val data = Vec(p.nLanes, UInt(p.storageBits.W))
  val laneMask = Vec(p.nLanes, Bool())
  val fflags = UInt(5.W)
  val last = Bool()
}

/** One SRAM lane word buffered ahead of the narrow SFU issue lanes. */
class VpuSfuInputWord(p: VpuParams) extends Bundle {
  val offset = UInt(p.vlBits.W)
  val data = Vec(p.nLanes, UInt(32.W))
}

/** One completely reassembled SRAM lane word awaiting masked writeback. */
class VpuSfuWriteback(p: VpuParams) extends Bundle {
  val address = UInt(p.elementAddrBits.W)
  val data = Vec(p.nLanes, UInt(p.storageBits.W))
  val laneMask = Vec(p.nLanes, Bool())
  val last = Bool()
}

class VpuHazardAccess(p: VpuParams) extends Bundle {
  val base = UInt(p.elementAddrBits.W)
  // A strided DMA command can conservatively cover one complete VSRAM bank,
  // which is wider than the architectural per-operation VL field.
  val elementCount = UInt(p.dmaTransferElementsBits.W)
  val read = Bool()
  val write = Bool()
}

object VpuCore {
  // Arithmetic results are fixed-latency Valid streams and cannot be
  // backpressured after issue.  Core reserves one of these four word credits
  // before issuing each ALU word or beginning each SFU word.
  final val ArithmeticWritebackQueueDepth = 4
}

class VpuMaskChunkWrite(p: VpuParams) extends Bundle {
  val chunk = UInt(p.maskChunkIndexBits.W)
  val data = UInt(64.W)
}

/** Copy-on-write architectural vector-mask versions.
  *
  * Software continues to observe one current C_WRITE_VMASK register. A masked
  * vector command snapshots only `currentSlot`; the reservation station keeps
  * that slot in `slotsInUse` through command completion. Updating a referenced
  * current mask either reuses an identical version or clones it into a free
  * slot, so an already queued command can never observe a later chunk write.
  */
class VpuMaskSlotFile(p: VpuParams) extends Module {
  val io = IO(new Bundle {
    val write = Flipped(Decoupled(new VpuMaskChunkWrite(p)))
    val slotsInUse = Input(UInt(p.maskSlots.W))
    val currentSlot = Output(UInt(p.maskSlotBits.W))

    // Every active operation, including iterative slide/gather, reads only the
    // lane word matching its current VSRAM word.
    val activeSlot = Input(UInt(p.maskSlotBits.W))
    val activeWordIndex = Input(UInt(p.maskWordIndexBits.W))
    val activeWord = Output(UInt(p.nLanes.W))
  })

  private val masks = RegInit(VecInit(
    Seq.fill(p.maskSlots)(0.U(p.vLen.W))))
  private val current = RegInit(0.U(p.maskSlotBits.W))
  private val currentMask = masks(current)

  val updatedMask = WireDefault(currentMask)
  for (chunk <- 0 until p.vectorMaskChunks) {
    when(io.write.bits.chunk === chunk.U) {
      val lo = chunk * 64
      val hi = math.min(p.vLen, lo + 64) - 1
      val width = hi - lo + 1
      val writeMask = (((BigInt(1) << width) - 1) << lo)
      val keepMask = (((BigInt(1) << p.vLen) - 1) ^ writeMask)
      val shiftedPayload =
        (io.write.bits.data(width - 1, 0).pad(p.vLen) << lo)(p.vLen - 1, 0)
      updatedMask := (currentMask & keepMask.U(p.vLen.W)) | shiftedPayload
    }
  }

  private val matchingSlots = VecInit(masks.map(_ === updatedMask))
  private val matchingOH = PriorityEncoderOH(matchingSlots.asUInt)
  private val matchingSlot = OHToUInt(matchingOH)
  private val hasMatchingSlot = matchingSlots.asUInt.orR
  private val currentReferenced = io.slotsInUse(current)
  private val freeSlots = ~io.slotsInUse
  private val freeSlot = PriorityEncoder(freeSlots)
  private val canWriteCurrent = !currentReferenced

  // A content match is safe even when that slot is referenced: its immutable
  // value already is the architectural next mask. Otherwise overwrite only an
  // unreferenced current/free slot.
  io.write.ready := hasMatchingSlot || canWriteCurrent || freeSlots.orR
  val writeTarget = Mux(canWriteCurrent, current, freeSlot)

  when(io.write.fire) {
    assert(io.write.bits.chunk < p.vectorMaskChunks.U,
      "VPU vector-mask chunk index escaped the architectural register")
    when(hasMatchingSlot) {
      current := matchingSlot
    }.otherwise {
      assert(!io.slotsInUse(writeTarget),
        "VPU vector-mask file overwrote a referenced slot")
      masks(writeTarget) := updatedMask
      current := writeTarget
    }
  }

  io.currentSlot := current
  val activeWords = masks(io.activeSlot)
    .asTypeOf(Vec(p.wordsPerVector, UInt(p.nLanes.W)))
  io.activeWord := activeWords(io.activeWordIndex)
  assert(io.activeWordIndex < p.wordsPerVector.U,
    "VPU vector-mask word index escaped VLEN")
}

/** Standalone VPU with a parameterized LD/EX/ST reservation station.
  *
  * Address-register values, VL, virtual addresses, and privilege status are
  * snapshotted at RS admission. FP register indices are retained, while FP
  * operand values are read when an EX entry issues; dependency bitmaps retain
  * VSRAM RAW/WAR/WAW and same-class issue order without a wrapping age tag.
  * Disjoint ready LD/EX/ST entries can issue together. WAIT snapshots live RS
  * entries, while FENCE additionally drains the DMA transport lifetimes.
  */
class VpuCore(p: VpuParams) extends Module {
  val io = IO(new Bundle {
    val command = Flipped(Decoupled(new VpuCommand))
    val commandRsAdmission = if (p.enableGroupedCommands) Some(Output(Bool())) else None
    // Pulses when a command is consumed by the architectural illegal-command
    // path instead of being allocated in LD/EX/ST.  The grouped wrapper uses
    // this disposition to abort (rather than permanently retain) its Gemmini
    // group.
    val commandRejected = if (p.enableGroupedCommands) Some(Output(Bool())) else None
    // Accepted architectural clear of the sticky illegal/fault class.  The
    // grouped wrapper mirrors that lifetime for its fusion-fault interlock.
    val commandClearsFusionFault = if (p.enableGroupedCommands) Some(Output(Bool())) else None
    // Sticky protocol fault owned by the optional grouped-command gate. It is
    // reflected in C_READ status bit 3 and cleared by C_CLEAR_STATUS bit 1.
    val fusionFault = if (p.enableGroupedCommands) Some(Input(Bool())) else None
    val matrixRead = if (p.matrixPorts > 0) Some(Vec(p.matrixPorts,
      Flipped(new GemminiVpuMatrixReadIO(
        p.matrixRowAddrBits, p.matrixElementsPerRow,
        p.storageBits)))) else None
    val matrixWrite = if (p.matrixPorts > 0) Some(Flipped(Vec(p.matrixPorts,
      Valid(new GemminiVpuMatrixWriteReq(
        p.matrixRowAddrBits, p.matrixElementsPerRow,
        p.storageBits))))) else None
    val vsramDeps = if (p.enableSharedDeps) Some(new VsramClientIO(
      p.sharedHazardAddressBits,
      VpuReservationStation.totalEntries(p))) else None
    val response = Decoupled(new VpuResponse)
    val dma = new VpuDmaIO(p)
    val busy = Output(Bool())
    val status = Output(UInt(64.W))
    val fflags = Output(UInt(5.W))
    val perfCounters = Output(Vec(VpuPerfIndex.Count, UInt(64.W)))
    val debugState = Output(UInt(5.W))
    val debugLoadState = Output(UInt(2.W))
    val debugHeadOpcode = Output(UInt(6.W))
    val debugActiveOpcode = Output(UInt(6.W))
    val debugSfuIssue = Output(Bool())
    val debugSfuResult = Output(Bool())
    val debugSfuWriteback = Output(Bool())
    val debugAluReadIssue = Output(Bool())
    val debugAluLaneIssue = Output(Bool())
    val debugAluResult = Output(Bool())
    val debugAluWriteback = Output(Bool())
  })

  private val hazardTagBits = p.rsTagBits

  // ------------------------------------------------------------------------
  // Architectural state, response buffering, and sticky fault state
  // ------------------------------------------------------------------------
  val gp = RegInit(VecInit(Seq.fill(16)(0.U(32.W))))
  val fp = RegInit(VecInit(Seq.fill(8)(0.U(32.W))))
  val h = RegInit(VecInit(Seq.fill(16)(0.U(64.W))))
  val currentVl = RegInit(p.vLen.U(p.vlBits.W))
  val currentVectorStrideElements = RegInit(0.U(p.elementAddrBits.W))
  val currentDmaStrideBytes = RegInit(0.U(64.W))
  val stickyFflags = RegInit(0.U(5.W))
  val illegalCommand = RegInit(false.B)

  val fpStateSram = Module(new VpuFpStateSram(p))
  fpStateSram.io.read.valid := false.B
  fpStateSram.io.read.bits := 0.U.asTypeOf(fpStateSram.io.read.bits)
  fpStateSram.io.write.valid := false.B
  fpStateSram.io.write.bits := 0.U.asTypeOf(fpStateSram.io.write.bits)

  val faultValid = RegInit(false.B)
  val faultAddress = RegInit(0.U(64.W))
  val faultCause = RegInit(0.U(2.W))
  val faultIsWrite = RegInit(false.B)
  val clearPending = RegInit(false.B)
  val clearAcknowledged = RegInit(false.B)

  val responseQueue = Module(new Queue(new VpuResponse, 2))
  io.response <> responseQueue.io.deq
  responseQueue.io.enq.valid := false.B
  responseQueue.io.enq.bits := 0.U.asTypeOf(responseQueue.io.enq.bits)

  // ------------------------------------------------------------------------
  // Gemmini-style LD/EX/ST reservation station. Entries retain commands from
  // allocation through engine-local completion; dependency bits are released
  // by issue (pure same-class ordering) or completion (real data hazards).
  // ------------------------------------------------------------------------
  val reservationStation = Module(new VpuReservationStation(p))
  val maskFile = Module(new VpuMaskSlotFile(p))
  maskFile.io.slotsInUse := reservationStation.io.maskSlotsInUse
  maskFile.io.write.valid := false.B
  maskFile.io.write.bits := 0.U.asTypeOf(maskFile.io.write.bits)
  maskFile.io.activeSlot := 0.U
  maskFile.io.activeWordIndex := 0.U
  if (p.enableSharedDeps) {
    io.vsramDeps.get <> reservationStation.io.vsramDeps.get
  }
  reservationStation.io.allocate.load.valid := false.B
  reservationStation.io.allocate.load.bits :=
    0.U.asTypeOf(reservationStation.io.allocate.load.bits)
  reservationStation.io.allocate.execute.valid := false.B
  reservationStation.io.allocate.execute.bits :=
    0.U.asTypeOf(reservationStation.io.allocate.execute.bits)
  reservationStation.io.allocate.store.valid := false.B
  reservationStation.io.allocate.store.bits :=
    0.U.asTypeOf(reservationStation.io.allocate.store.bits)
  reservationStation.io.issue.load.ready := false.B
  reservationStation.io.issue.execute.ready := false.B
  reservationStation.io.issue.store.ready := false.B
  reservationStation.io.flushUnissued := false.B
  for (port <- reservationStation.io.complete) {
    port.valid := false.B
    port.bits := 0.U
  }

  // Gemmini's BeatMerger presents one scratchpad row per write arbitration.
  // This associative variant also tolerates out-of-order TL source responses.
  val loadLineMerger = Module(new VpuLoadLineMerger(p))
  loadLineMerger.io.descriptor.valid := false.B
  loadLineMerger.io.descriptor.bits :=
    0.U.asTypeOf(loadLineMerger.io.descriptor.bits)
  loadLineMerger.io.in <> io.dma.readData

  // Accepted store descriptors leave the architectural command queue as soon
  // as the DMA writer snapshots them.  Their SRAM payloads continue through
  // this independent queue while earlier TL Put acknowledgements remain live.
  val storeStreamQueue = Module(new Queue(
    new VpuStoreQueueEntry(p), p.storeQueueEntries))
  storeStreamQueue.io.enq.valid := false.B
  storeStreamQueue.io.enq.bits := 0.U.asTypeOf(storeStreamQueue.io.enq.bits)
  storeStreamQueue.io.deq.ready := false.B

  val ldWritebackDone = WireDefault(false.B)
  val ldWritebackTag = WireDefault(0.U(hazardTagBits.W))
  val ldIssueCanceled = WireDefault(false.B)
  val exDone = WireDefault(false.B)
  val stDone = WireDefault(false.B)
  val stDoneTag = WireDefault(0.U(hazardTagBits.W))
  val stIssueCanceled = WireDefault(false.B)

  // ------------------------------------------------------------------------
  // Vector SRAM.  The physical banks expose one read and one write port each;
  // EX/ST reads and EX/LD writes therefore proceed together whenever their
  // bank sets are disjoint.  Same-bank contenders are selected fairly inside
  // VpuBankedScratchpad rather than being serialized by a global arbiter.
  // ------------------------------------------------------------------------
  val scratchpad = Module(new VpuBankedScratchpad(p))
  if (p.matrixPorts > 0) {
    scratchpad.io.matrixRead.get <> io.matrixRead.get
    scratchpad.io.matrixWrite.get <> io.matrixWrite.get
  }
  val executeWrite = Wire(Decoupled(new VpuSpadWriteRequest(p)))
  val loadWrite = Wire(Decoupled(new VpuSpadWriteRequest(p)))
  executeWrite.valid := false.B
  executeWrite.bits := 0.U.asTypeOf(executeWrite.bits)
  loadWrite.valid := false.B
  loadWrite.bits := 0.U.asTypeOf(loadWrite.bits)
  scratchpad.io.writeRequest(VpuBankedScratchpad.ExecuteWriteClient) <>
    executeWrite
  scratchpad.io.writeRequest(VpuBankedScratchpad.LoadWriteClient) <>
    loadWrite

  val source0Read = Wire(Decoupled(new VpuSpadReadRequest(p)))
  val source1Read = Wire(Decoupled(new VpuSpadReadRequest(p)))
  val storeRead = Wire(Decoupled(new VpuSpadReadRequest(p)))
  source0Read.valid := false.B
  source0Read.bits := 0.U.asTypeOf(source0Read.bits)
  source1Read.valid := false.B
  source1Read.bits := 0.U.asTypeOf(source1Read.bits)
  storeRead.valid := false.B
  storeRead.bits := 0.U.asTypeOf(storeRead.bits)
  scratchpad.io.readRequest(VpuBankedScratchpad.Source0ReadClient) <>
    source0Read
  scratchpad.io.readRequest(VpuBankedScratchpad.Source1ReadClient) <>
    source1Read
  scratchpad.io.readRequest(VpuBankedScratchpad.StoreReadClient) <>
    storeRead
  val source0ReadResponse =
    scratchpad.io.readResponse(VpuBankedScratchpad.Source0ReadClient)
  val source1ReadResponse =
    scratchpad.io.readResponse(VpuBankedScratchpad.Source1ReadClient)
  val storeReadResponse =
    scratchpad.io.readResponse(VpuBankedScratchpad.StoreReadClient)
  source0ReadResponse.ready := false.B
  source1ReadResponse.ready := false.B
  storeReadResponse.ready := false.B

  // ------------------------------------------------------------------------
  // Load engine: descriptor issue and SRAM writeback are independent.
  //
  // The issue FSM only holds a descriptor until the DMA frontend accepts it.
  // Returned beats carry the descriptor's hazard tag; a per-tag byte/element
  // tracker retires each command only after its final SRAM write.  Responses
  // from independent descriptors may therefore be written in any order.
  // ------------------------------------------------------------------------
  val lIdle :: lDescriptor :: lData :: Nil = Enum(3)
  val loadIssueState = RegInit(lIdle)
  val activeLoad = Reg(new VpuLoadQueueEntry(p))
  val loadInFlight = RegInit(VecInit(
    Seq.fill(p.loadRsEntries)(false.B)))

  val dmaFaultStatus = faultValid || io.dma.halted
  val faultActive = dmaFaultStatus || clearPending
  val loadIssueCommand = Wire(new VpuLoadQueueEntry(p))
  loadIssueCommand := reservationStation.io.issue.load.bits.command
  loadIssueCommand.hazardTag := reservationStation.io.issue.load.bits.tag
  val loadStart = loadIssueState === lIdle &&
    reservationStation.io.issue.load.valid && !faultActive
  reservationStation.io.issue.load.ready :=
    loadIssueState === lIdle && !faultActive
  when (loadStart) {
    activeLoad := loadIssueCommand
    loadIssueState := lDescriptor
  }

  // The DMA reader and row merger snapshot the descriptor atomically.  No data
  // beat can then arrive without the merge table knowing its architectural
  // range and expected tail mask.
  val loadDescriptorValid = loadIssueState === lDescriptor && !faultActive
  val activeLoadDmaTag =
    activeLoad.hazardTag(p.dmaCommandTagBits - 1, 0)
  io.dma.readDescriptor.valid := loadDescriptorValid &&
    loadLineMerger.io.descriptor.ready
  io.dma.readDescriptor.bits.vaddr := activeLoad.vaddr
  io.dma.readDescriptor.bits.spadElement := activeLoad.spadBase
  io.dma.readDescriptor.bits.elementCount := activeLoad.elementCount
  io.dma.readDescriptor.bits.rowCount := activeLoad.rowCount
  io.dma.readDescriptor.bits.hostStrideBytes := activeLoad.hostStrideBytes
  io.dma.readDescriptor.bits.commandTag := activeLoadDmaTag
  io.dma.readDescriptor.bits.status := activeLoad.status
  loadLineMerger.io.descriptor.valid := loadDescriptorValid &&
    io.dma.readDescriptor.ready
  loadLineMerger.io.descriptor.bits.commandTag := activeLoadDmaTag
  loadLineMerger.io.descriptor.bits.spadElement := activeLoad.spadBase
  loadLineMerger.io.descriptor.bits.elementCount := activeLoad.elementCount
  loadLineMerger.io.descriptor.bits.rowCount := activeLoad.rowCount
  when (io.dma.readDescriptor.fire) {
    assert(loadLineMerger.io.descriptor.fire,
      "VPU DMA and line-merger descriptors must be accepted atomically")
    assert(activeLoad.hazardTag < p.loadRsEntries.U,
      "VPU load issue carried a non-load global RS tag")
    assert(!loadInFlight(activeLoadDmaTag),
      "VPU load DMA reused a live command tag")
    loadInFlight(activeLoadDmaTag) := true.B
    loadIssueState := lIdle
  }

  // A descriptor which has not reached the wrapper can be canceled at a fault
  // boundary. Already accepted streams remain in the tagged DMA/writeback
  // path and retire independently.
  when (loadIssueState === lDescriptor && faultActive) {
    loadIssueState := lIdle
    ldIssueCanceled := true.B
  }

  val loadWord = loadLineMerger.io.out.bits
  val loadWordTag = loadWord.commandTag
  val loadWordTagInRange = loadWordTag < p.loadRsEntries.U
  loadWrite.valid := loadLineMerger.io.out.valid && loadWordTagInRange &&
    loadInFlight(loadWordTag) && !loadWord.error
  loadWrite.bits.address := loadWord.address
  loadWrite.bits.data := loadWord.data
  // A normal row is always written once with an all-ones mask.  As in
  // Gemmini's BeatMerger, only the architectural tail retains a partial mask
  // so adjacent VSRAM elements are not corrupted.
  loadWrite.bits.laneMask := loadWord.laneMask
  loadLineMerger.io.out.ready := loadWordTagInRange &&
    loadInFlight(loadWordTag) && (loadWord.error || loadWrite.ready)

  when(loadLineMerger.io.out.valid) {
    assert(loadWordTagInRange,
      "VPU merged DMA word tag is outside the reservation station")
    assert(loadInFlight(loadWordTag),
      "VPU merged DMA word belongs to an inactive load")
    when(!loadWord.error) {
      assert((loadWord.address % p.nLanes.U) === 0.U,
        "VPU merged DMA word is not lane-word aligned")
    }
  }

  val dmaReadFire = loadLineMerger.io.in.fire
  val dmaReadByteIncrement =
    PopCount(loadLineMerger.io.in.bits.elementMask) * p.storageBytes.U
  val loadWordFire = loadLineMerger.io.out.fire
  val readFaultEvent = loadWordFire && loadWord.error
  when(loadWordFire && loadWord.last) {
      loadInFlight(loadWordTag) := false.B
      ldWritebackDone := true.B
      ldWritebackTag := loadWordTag
  }

  // Preserve the existing three-state debug encoding while making it describe
  // two independent activities: descriptor backpressure has priority, then
  // buffered/active writeback.  Idle now really means that both are empty.
  val loadState = WireDefault(lIdle)
  when (loadIssueState === lDescriptor) {
    loadState := lDescriptor
  }.elsewhen (loadInFlight.asUInt.orR || loadLineMerger.io.busy) {
    loadState := lData
  }

  // ------------------------------------------------------------------------
  // Store engine: descriptor issue, SRAM streaming, TL A, and TL D retirement
  // are independent lifetimes, matching Gemmini's StoreController/Scratchpad/
  // StreamWriter split.
  // ------------------------------------------------------------------------
  val storeStates = Enum(6)
  val stIdle = storeStates(0)
  val stDescriptor = storeStates(1)
  val stReadRequest = storeStates(2)
  val stReadResponse = storeStates(3)
  val stData = storeStates(4)
  val stCompletion = storeStates(5)

  val siIdle :: siDescriptor :: Nil = Enum(2)
  val storeIssueState = RegInit(siIdle)
  val activeStoreIssue = Reg(new VpuStoreQueueEntry(p))
  val storeTransportTags = Module(new VpuStoreTransportTagAllocator(p))
  storeTransportTags.io.allocate.ready := false.B
  storeTransportTags.io.release.valid := false.B
  storeTransportTags.io.release.bits := 0.U

  val storeIssueCommand = Wire(new VpuStoreQueueEntry(p))
  storeIssueCommand := reservationStation.io.issue.store.bits.command
  storeIssueCommand.hazardTag := reservationStation.io.issue.store.bits.tag
  val storeIssueStart = storeIssueState === siIdle &&
    reservationStation.io.issue.store.valid && !faultActive
  reservationStation.io.issue.store.ready :=
    storeIssueState === siIdle && !faultActive
  when(storeIssueStart) {
    activeStoreIssue := storeIssueCommand
    storeIssueState := siDescriptor
  }

  // Descriptor, transport-tag allocation, and payload-stream admission are
  // atomic. The transport tag outlives (and is intentionally independent of)
  // the reservation-station tag.
  val storeDescriptorValid = storeIssueState === siDescriptor && !faultActive
  io.dma.writeDescriptor.valid := storeDescriptorValid &&
    storeStreamQueue.io.enq.ready && storeTransportTags.io.allocate.valid
  io.dma.writeDescriptor.bits.vaddr := activeStoreIssue.vaddr
  io.dma.writeDescriptor.bits.spadElement := activeStoreIssue.spadBase
  io.dma.writeDescriptor.bits.elementCount := activeStoreIssue.elementCount
  io.dma.writeDescriptor.bits.rowCount := activeStoreIssue.rowCount
  io.dma.writeDescriptor.bits.hostStrideBytes :=
    activeStoreIssue.hostStrideBytes
  io.dma.writeDescriptor.bits.commandTag := storeTransportTags.io.allocate.bits
  io.dma.writeDescriptor.bits.status := activeStoreIssue.status
  storeStreamQueue.io.enq.valid := storeDescriptorValid &&
    io.dma.writeDescriptor.ready && storeTransportTags.io.allocate.valid
  storeStreamQueue.io.enq.bits := activeStoreIssue
  storeStreamQueue.io.enq.bits.transportTag :=
    storeTransportTags.io.allocate.bits
  storeTransportTags.io.allocate.ready := storeDescriptorValid &&
    io.dma.writeDescriptor.ready && storeStreamQueue.io.enq.ready

  when(io.dma.writeDescriptor.fire) {
    assert(storeStreamQueue.io.enq.fire,
      "VPU store descriptor and SRAM stream were not admitted atomically")
    assert(storeTransportTags.io.allocate.fire,
      "VPU store descriptor did not atomically allocate a transport tag")
    storeIssueState := siIdle
  }
  when(storeIssueState === siDescriptor && faultActive) {
    storeIssueState := siIdle
    stIssueCanceled := true.B
  }

  // One descriptor's SRAM words are streamed in order, but all word reads are
  // issued ahead into VpuBankedScratchpad's credit-protected response queue.
  // This removes the old request/response bubble between consecutive words.
  val storeStreamActive = RegInit(false.B)
  val activeStoreStream = Reg(new VpuStoreQueueEntry(p))
  val storeReadIssueRow = RegInit(0.U(p.dmaRowCountBits.W))
  val storeReadIssueOffset = RegInit(0.U(p.vlBits.W))
  // The count includes accepted requests which may occupy every entry in the
  // scratchpad's bounded store-response queue.  It must represent the depth
  // value itself, not merely indices 0..depth-1.
  private val storeReadOutstandingBits = math.max(1, log2Ceil(
    VpuBankedScratchpad.ReadResponseQueueDepth + 1))
  val storeReadsOutstanding =
    RegInit(0.U(storeReadOutstandingBits.W))
  val storeSerializerValid = RegInit(false.B)
  val storeSerializerWord =
    Reg(Vec(p.nLanes, UInt(p.storageBits.W)))
  // Preserve the absolute VSRAM address echoed by the scratchpad.  Unlike a
  // dense one-dimensional stream, adjacent DMA rows are VLEN slots apart
  // even when a row's active VL is shorter than VLEN.
  val storeSerializerAddress = Reg(UInt(p.elementAddrBits.W))
  // Store-client scratchpad responses are FIFO ordered. Track the exact next
  // lane-word address so a row-boundary or tail-counter regression fails at
  // the Core boundary instead of becoming a later DMA stream protocol fault.
  val storeExpectedResponseAddress = Reg(UInt(p.elementAddrBits.W))
  private val storeChunksPerWord = p.nLanes / p.dmaElementsPerBeat
  val storeChunk =
    RegInit(0.U(math.max(1, log2Ceil(storeChunksPerWord)).W))

  val storeStreamStart = !storeStreamActive &&
    storeStreamQueue.io.deq.valid
  storeStreamQueue.io.deq.ready := storeStreamStart
  when(storeStreamQueue.io.deq.fire) {
    activeStoreStream := storeStreamQueue.io.deq.bits
    storeReadIssueRow := 0.U
    storeReadIssueOffset := 0.U
    storeReadsOutstanding := 0.U
    storeSerializerValid := false.B
    storeExpectedResponseAddress := storeStreamQueue.io.deq.bits.spadBase
    storeChunk := 0.U
    storeStreamActive := true.B
  }

  val activeStoreRows = Mux(activeStoreStream.rowCount === 0.U,
    1.U, activeStoreStream.rowCount)
  val storeReadAddress = Wire(UInt(p.elementAddrBits.W))
  storeReadAddress := activeStoreStream.spadBase +
    storeReadIssueRow * p.vLen.U + storeReadIssueOffset
  storeRead.valid := storeStreamActive &&
    storeReadIssueRow < activeStoreRows &&
    storeReadIssueOffset < activeStoreStream.elementCount
  storeRead.bits.address := storeReadAddress
  storeRead.bits.tag := storeReadAddress.pad(p.spadReadTagBits)
  val storeReadFire = storeRead.fire
  val storeReadLastWordInRow = storeReadIssueOffset + p.nLanes.U >=
    activeStoreStream.elementCount
  val storeLastReadAccepted = storeReadFire &&
    storeReadLastWordInRow && storeReadIssueRow === activeStoreRows - 1.U
  when(storeReadFire) {
    when(storeReadLastWordInRow) {
      storeReadIssueOffset := 0.U
      storeReadIssueRow := storeReadIssueRow + 1.U
    }.otherwise {
      storeReadIssueOffset := storeReadIssueOffset + p.nLanes.U
    }
    when(storeLastReadAccepted) {
      // Match Gemmini Scratchpad: the reservation entry completes when the
      // final SRAM read request is accepted. The response/payload/TL-D path is
      // protected by its independent transport tag and can drain afterward.
      stDone := true.B
      stDoneTag := activeStoreStream.hazardTag
    }
  }

  val storeSerializerRelative = storeSerializerAddress -
    activeStoreStream.spadBase
  val storeSerializerRow = storeSerializerRelative / p.vLen.U
  val storeSerializerRowOffset = storeSerializerRelative % p.vLen.U
  val storeChunkRowOffset = storeSerializerRowOffset +
    storeChunk * p.dmaElementsPerBeat.U
  val storeElementMask = Wire(Vec(p.dmaElementsPerBeat, Bool()))
  for(i <- 0 until p.dmaElementsPerBeat) {
    storeElementMask(i) := storeChunkRowOffset + i.U <
      activeStoreStream.elementCount
  }
  val storeChunkData = VecInit((0 until storeChunksPerWord).map { chunk =>
    Cat((0 until p.dmaElementsPerBeat).reverse.map { i =>
      storeSerializerWord(chunk * p.dmaElementsPerBeat + i)
    })
  })
  val storeRowLastChunk = storeChunkRowOffset +
    p.dmaElementsPerBeat.U >= activeStoreStream.elementCount
  val storeStreamLast = storeSerializerRow === activeStoreRows - 1.U &&
    storeRowLastChunk
  val storeWordLastChunk = storeRowLastChunk ||
    storeChunk === (storeChunksPerWord - 1).U

  io.dma.writeData.valid := storeSerializerValid
  io.dma.writeData.bits.data := storeChunkData(storeChunk)
  io.dma.writeData.bits.elementMask := storeElementMask.asUInt
  io.dma.writeData.bits.spadElement :=
    storeSerializerAddress + storeChunk * p.dmaElementsPerBeat.U
  io.dma.writeData.bits.commandTag := activeStoreStream.transportTag
  io.dma.writeData.bits.last := storeStreamLast
  val dmaWriteFire = io.dma.writeData.fire
  val dmaWriteByteIncrement = PopCount(storeElementMask) * p.storageBytes.U

  // Replace the serializer word in the same cycle its final chunk leaves.
  val serializerCanReplace = !storeSerializerValid ||
    (dmaWriteFire && storeWordLastChunk)
  storeReadResponse.ready := serializerCanReplace
  val storeResponseFire = storeReadResponse.fire
  val storeResponseOffset =
    storeReadResponse.bits.tag(p.elementAddrBits - 1, 0)
  val storeResponseRelative = storeResponseOffset -
    activeStoreStream.spadBase
  val storeResponseRow = storeResponseRelative / p.vLen.U
  val storeResponseRowOffset = storeResponseRelative % p.vLen.U
  val storeResponseLastWordInRow = storeResponseRowOffset + p.nLanes.U >=
    activeStoreStream.elementCount

  when(dmaWriteFire) {
    assert(storeSerializerAddress >= activeStoreStream.spadBase,
      "VPU store serializer address precedes its VSRAM base")
    assert(storeSerializerRow < activeStoreRows,
      "VPU store serializer address escaped its row count")
    assert(storeSerializerRowOffset < activeStoreStream.elementCount,
      "VPU store serializer started in inactive row padding")
    assert(storeChunkRowOffset < activeStoreStream.elementCount &&
      storeElementMask.asUInt.orR,
      "VPU store serializer emitted an empty or out-of-row chunk")
    when(storeWordLastChunk) {
      storeSerializerValid := false.B
      storeChunk := 0.U
    }.otherwise {
      storeChunk := storeChunk + 1.U
    }
    when(storeStreamLast) {
      assert(storeReadIssueRow >= activeStoreRows,
        "VPU store emitted last data before issuing all SRAM reads")
      storeStreamActive := false.B
    }
  }
  when(storeResponseFire) {
    assert(storeResponseOffset === storeExpectedResponseAddress,
      "VPU store scratchpad responses did not follow request order")
    assert(storeResponseOffset >= activeStoreStream.spadBase,
      "VPU store scratchpad response precedes its VSRAM base")
    assert(storeResponseRow < activeStoreRows,
      "VPU store scratchpad response escaped its row count")
    assert(storeResponseRowOffset < activeStoreStream.elementCount,
      "VPU store scratchpad response started in inactive row padding")
    storeSerializerWord := storeReadResponse.bits.data
    storeSerializerAddress := storeResponseOffset
    storeSerializerValid := true.B
    storeChunk := 0.U
    when(!(storeResponseLastWordInRow &&
        storeResponseRow === activeStoreRows - 1.U)) {
      storeExpectedResponseAddress := Mux(storeResponseLastWordInRow,
        activeStoreStream.spadBase +
          (storeResponseRow + 1.U) * p.vLen.U,
        storeResponseOffset + p.nLanes.U)
    }
  }

  val readAcceptedOnly = storeReadFire && !storeResponseFire
  val readReturnedOnly = !storeReadFire && storeResponseFire
  when(readAcceptedOnly) {
    storeReadsOutstanding := storeReadsOutstanding + 1.U
  }.elsewhen(readReturnedOnly) {
    assert(storeReadsOutstanding =/= 0.U,
      "VPU store SRAM response count underflow")
    storeReadsOutstanding := storeReadsOutstanding - 1.U
  }
  when(dmaWriteFire && storeStreamLast) {
    val outstandingAfter = storeReadsOutstanding +&
      storeReadFire.asUInt - storeResponseFire.asUInt
    assert(outstandingAfter === 0.U,
      "VPU store completed with outstanding SRAM responses")
  }

  // Tagged TL completions may arrive long after the architectural store entry
  // was released. They retire only the independent transport identity; FENCE
  // observes allocator busy, whereas WAIT(ST) observes the RS entry lifetime.
  val completionTag = io.dma.writeCompletion.bits.commandTag
  val completionTagInRange = completionTag < p.storeQueueEntries.U
  io.dma.writeCompletion.ready := completionTagInRange &&
    storeTransportTags.io.activeMask(completionTag)
  when(io.dma.writeCompletion.valid) {
    assert(completionTagInRange,
      "VPU store completion tag is outside the transport table")
    assert(storeTransportTags.io.activeMask(completionTag),
      "VPU store completion referenced an inactive transport")
  }
  val writeCompletionFire = io.dma.writeCompletion.fire
  val writeFaultEvent =
    writeCompletionFire && io.dma.writeCompletion.bits.error
  storeTransportTags.io.release.valid := writeCompletionFire
  storeTransportTags.io.release.bits := completionTag

  // Preserve the existing six-value debug/performance encoding while deriving
  // it from the now-independent issue, SRAM-stream, and D-retirement stages.
  val storeState = WireDefault(stIdle)
  when(storeIssueState === siDescriptor) {
    storeState := stDescriptor
  }.elsewhen(storeSerializerValid) {
    storeState := stData
  }.elsewhen(storeStreamActive || storeStreamQueue.io.deq.valid) {
    storeState := Mux(storeReadsOutstanding =/= 0.U,
      stReadResponse, stReadRequest)
  }.elsewhen(storeTransportTags.io.busy) {
    storeState := stCompletion
  }

  // ------------------------------------------------------------------------
  // Execute engine and FP/SFU lanes
  // ------------------------------------------------------------------------
  val executeStates = Enum(15)
  val exIdle = executeStates(0)
  val exSfuIssue = executeStates(1)
  val exSfuWait = executeStates(2)
  val exScalarAluIssue = executeStates(3)
  val exScalarAluWait = executeStates(4)
  val exScalarExpIssue = executeStates(5)
  val exScalarExpWait = executeStates(6)
  val exScalarDivIssue = executeStates(7)
  val exScalarDivWait = executeStates(8)
  val exAluStream = executeStates(9)
  val exReductionStream = executeStates(10)
  val exRearrange = executeStates(11)
  val exStateReadIssue = executeStates(12)
  val exStateReadWait = executeStates(13)
  val exStateWrite = executeStates(14)
  val executeState = RegInit(exIdle)
  val activeExecute = Reg(new VpuExecuteQueueEntry(p))
  // FP values are read once when an RS entry issues. They belong only to the
  // active engine slot, rather than being replicated as dead fields in every
  // execute reservation entry.
  val activeScalarA = Reg(UInt(32.W))
  val activeScalarB = Reg(UInt(32.W))
  val aluReadOffset = RegInit(0.U(p.vlBits.W))
  val aluSource1ReadOffset = RegInit(0.U(p.vlBits.W))
  val resultWord = Reg(Vec(p.nLanes, UInt(p.storageBits.W)))
  private val aluWritebackEntries = VpuCore.ArithmeticWritebackQueueDepth
  val aluWritebackQueue = Module(new Queue(
    new VpuAluWriteback(p), aluWritebackEntries,
    pipe = false, flow = false))
  private val aluPipelineCountBits = math.max(1,
    log2Ceil(p.fmaPipeDepth + 1))
  val aluPipelineInFlight = RegInit(0.U(aluPipelineCountBits.W))
  val aluWritebackDequeueFire = WireDefault(false.B)
  val aluReservedResults = aluWritebackQueue.io.count +&
    aluPipelineInFlight
  val aluIssueCredit = aluReservedResults < aluWritebackEntries.U ||
    aluWritebackDequeueFire
  val rearrange = Module(new VpuRearrangeUnit(p))
  rearrange.io.start.valid := false.B
  rearrange.io.start.bits := 0.U.asTypeOf(rearrange.io.start.bits)
  for (operand <- 0 until 2) {
    rearrange.io.readRequest(operand).ready := false.B
    rearrange.io.readResponse(operand).valid := false.B
    rearrange.io.readResponse(operand).bits :=
      0.U.asTypeOf(rearrange.io.readResponse(operand).bits)
  }
  rearrange.io.writeRequest.ready := false.B

  def isVv(opcode: UInt): Bool = Seq(VpuOpcode.V_ADD_VV,
    VpuOpcode.V_SUB_VV, VpuOpcode.V_MUL_VV, VpuOpcode.V_GATHER_VV)
    .map(x => opcode === x.U).reduce(_ || _)
  def isVf(opcode: UInt): Bool = Seq(VpuOpcode.V_ADD_VF,
    VpuOpcode.V_SUB_VF, VpuOpcode.V_MUL_VF, VpuOpcode.V_MAX_VF,
    VpuOpcode.V_MIN_VF).map(x => opcode === x.U).reduce(_ || _)
  def isReduction(opcode: UInt): Bool = opcode === VpuOpcode.V_RED_SUM.U ||
    opcode === VpuOpcode.V_RED_MAX.U
  def isSfuVector(opcode: UInt): Bool = opcode === VpuOpcode.V_EXP_V.U ||
    opcode === VpuOpcode.V_RECI_V.U
  def isRearrange(opcode: UInt): Bool =
    opcode === VpuOpcode.V_SLIDE_V.U ||
      opcode === VpuOpcode.V_GATHER_VV.U
  def isVectorOpcode(opcode: UInt): Bool = VpuDecode.isVector(opcode)
  def isStateOpcode(opcode: UInt): Bool = VpuDecode.isState(opcode)
  def isBaseVectorAlu(opcode: UInt): Bool = isVectorOpcode(opcode) &&
    !isReduction(opcode) && !isSfuVector(opcode) && !isRearrange(opcode)

  /** Map a logical vector offset onto the existing tile-major VSRAM layout.
    * Tags and all arithmetic sequencing remain logical; only SRAM addresses
    * jump between matrix-row segments when vectorStride is nonzero.
    */
  def vectorAddress(base: UInt, logicalOffset: UInt,
                    vectorStride: UInt): UInt = {
    val segment = logicalOffset / p.matrixElementsPerRow.U
    val withinSegment = logicalOffset % p.matrixElementsPerRow.U
    val physicalOffset = Mux(vectorStride === 0.U, logicalOffset,
      segment * vectorStride + withinSegment)
    val address = Wire(UInt(p.elementAddrBits.W))
    address := base + physicalOffset
    address
  }

  val executeCanStart = executeState === exIdle && !faultActive &&
    (!isRearrange(reservationStation.io.issue.execute.bits.command.opcode) ||
      rearrange.io.start.ready)
  val executeStart = reservationStation.io.issue.execute.valid &&
    executeCanStart
  reservationStation.io.issue.execute.ready := executeCanStart
  val executeIssueBits = Wire(new VpuExecuteQueueEntry(p))
  executeIssueBits := reservationStation.io.issue.execute.bits.command
  executeIssueBits.hazardTag := reservationStation.io.issue.execute.bits.tag
  maskFile.io.activeSlot := activeExecute.maskSlot
  val executeScalarA = fp(
    reservationStation.io.issue.execute.bits.command.fpRs1)
  val executeScalarB = fp(
    reservationStation.io.issue.execute.bits.command.fpRs2)
  val executeScalarSeed = fp(
    reservationStation.io.issue.execute.bits.command.fpSeed)
  when (executeStart) {
    activeExecute := executeIssueBits
    activeScalarA := executeScalarA
    activeScalarB := executeScalarB
    aluReadOffset := 0.U
    aluSource1ReadOffset := 0.U
    when (isBaseVectorAlu(executeIssueBits.opcode)) {
      executeState := exAluStream
    }.elsewhen (isReduction(executeIssueBits.opcode)) {
      executeState := exReductionStream
    }.elsewhen (isSfuVector(executeIssueBits.opcode)) {
      // The SFU scheduler owns its SRAM read stream.  It reads lane words
      // ahead into a small FIFO and issues narrow SFU groups continuously;
      // routing vector SFU commands through a word-at-a-time state would serialize a
      // complete SFU drain between adjacent words.
      executeState := exSfuIssue
    }.elsewhen (isRearrange(executeIssueBits.opcode)) {
      executeState := exRearrange
    }.elsewhen (executeIssueBits.opcode === VpuOpcode.S_LOAD_STATE.U) {
      executeState := exStateReadIssue
    }.elsewhen (executeIssueBits.opcode === VpuOpcode.S_STORE_STATE.U) {
      executeState := exStateWrite
    }.elsewhen (executeIssueBits.opcode <= VpuOpcode.S_MUL_FP.U) {
      executeState := exScalarAluIssue
    }.elsewhen (executeIssueBits.opcode === VpuOpcode.S_EXP_FP.U) {
      executeState := exScalarExpIssue
    }.otherwise {
      executeState := exScalarDivIssue
    }
  }

  fpStateSram.io.read.valid := executeState === exStateReadIssue
  fpStateSram.io.read.bits.index := activeExecute.stateIndex
  when(fpStateSram.io.read.fire) {
    executeState := exStateReadWait
  }
  when(executeState === exStateReadWait && fpStateSram.io.readData.valid) {
    fp(activeExecute.fpDestination) := fpStateSram.io.readData.bits
    executeState := exIdle
    exDone := true.B
  }

  fpStateSram.io.write.valid := executeState === exStateWrite
  fpStateSram.io.write.bits.index := activeExecute.stateIndex
  fpStateSram.io.write.bits.data := activeScalarA
  when(fpStateSram.io.write.fire) {
    executeState := exIdle
    exDone := true.B
  }

  rearrange.io.start.valid := executeStart &&
    isRearrange(executeIssueBits.opcode)
  rearrange.io.start.bits.gather :=
    executeIssueBits.opcode === VpuOpcode.V_GATHER_VV.U
  rearrange.io.start.bits.slideLow := executeIssueBits.funct1 === 1.U
  rearrange.io.start.bits.destination := executeIssueBits.destination
  rearrange.io.start.bits.source := executeIssueBits.source0
  rearrange.io.start.bits.indices := executeIssueBits.source1
  rearrange.io.start.bits.shift := executeIssueBits.gpRs2Value
  rearrange.io.start.bits.elementCount := executeIssueBits.elementCount
  rearrange.io.start.bits.maskEnable := executeIssueBits.maskEnable
  rearrange.io.maskWord := maskFile.io.activeWord

  val aluSource0ReadsRemaining = aluReadOffset < activeExecute.elementCount
  val aluSource1ReadsRemaining =
    aluSource1ReadOffset < activeExecute.elementCount
  val reductionStreamingState = executeState === exReductionStream
  val sfuStreamingState = executeState === exSfuIssue ||
    executeState === exSfuWait
  val sfuReadOffset = RegInit(0.U(p.vlBits.W))
  val sfuReadsRemaining = sfuReadOffset < activeExecute.elementCount
  val sfuInputQueue = Module(new Queue(new VpuSfuInputWord(p), 2,
    pipe = false, flow = false))
  val rearrangeActive = executeState === exRearrange
  val regularSource0ReadValid =
    (executeState === exAluStream && aluSource0ReadsRemaining) ||
    (reductionStreamingState && aluSource0ReadsRemaining) ||
    (sfuStreamingState && sfuReadsRemaining)
  val source0RequestedOffset = Mux(executeState === exAluStream ||
    reductionStreamingState,
    aluReadOffset, sfuReadOffset)
  val sourceOperandsAlias = activeExecute.useSource1 &&
    activeExecute.source0 === activeExecute.source1
  val regularSource1ReadValid = executeState === exAluStream &&
    activeExecute.useSource1 && !sourceOperandsAlias &&
    aluSource1ReadsRemaining
  val regularSource0Read = Wire(new VpuSpadReadRequest(p))
  regularSource0Read.address := vectorAddress(activeExecute.source0,
    source0RequestedOffset, activeExecute.vectorStride)
  regularSource0Read.tag := source0RequestedOffset.pad(p.spadReadTagBits)
  val regularSource1Read = Wire(new VpuSpadReadRequest(p))
  regularSource1Read.address := vectorAddress(activeExecute.source1,
    aluSource1ReadOffset, activeExecute.vectorStride)
  regularSource1Read.tag :=
    aluSource1ReadOffset.pad(p.spadReadTagBits)

  source0Read.valid := Mux(rearrangeActive,
    rearrange.io.readRequest(0).valid, regularSource0ReadValid)
  source0Read.bits := Mux(rearrangeActive,
    rearrange.io.readRequest(0).bits, regularSource0Read)
  source1Read.valid := Mux(rearrangeActive,
    rearrange.io.readRequest(1).valid, regularSource1ReadValid)
  source1Read.bits := Mux(rearrangeActive,
    rearrange.io.readRequest(1).bits, regularSource1Read)
  rearrange.io.readRequest(0).ready := rearrangeActive && source0Read.ready
  rearrange.io.readRequest(1).ready := rearrangeActive && source1Read.ready
  when (source0Read.fire && !rearrangeActive) {
    when (sfuStreamingState) {
      sfuReadOffset := sfuReadOffset + p.nLanes.U
    }.otherwise {
      aluReadOffset := aluReadOffset + p.nLanes.U
    }
  }
  when(source1Read.fire && !rearrangeActive) {
    aluSource1ReadOffset := aluSource1ReadOffset + p.nLanes.U
  }

  for (operand <- 0 until 2) {
    val response = if (operand == 0) source0ReadResponse
      else source1ReadResponse
    rearrange.io.readResponse(operand).valid :=
      rearrangeActive && response.valid
    rearrange.io.readResponse(operand).bits := response.bits
  }
  val regularNeedsSource1 = executeState === exAluStream &&
    activeExecute.useSource1 && !sourceOperandsAlias
  val regularTagsMatch = source0ReadResponse.bits.tag ===
    source1ReadResponse.bits.tag
  val regularOperandsAvailable = source0ReadResponse.valid &&
    (!regularNeedsSource1 ||
      (source1ReadResponse.valid && regularTagsMatch))
  val regularOperandConsumerReady =
    Mux(sfuStreamingState, sfuInputQueue.io.enq.ready,
      Mux(executeState === exAluStream, aluIssueCredit,
        reductionStreamingState))
  val regularOperandFire = !rearrangeActive &&
    regularOperandsAvailable && regularOperandConsumerReady
  source0ReadResponse.ready := Mux(rearrangeActive,
    rearrange.io.readResponse(0).ready,
    regularOperandsAvailable && regularOperandConsumerReady)
  source1ReadResponse.ready := Mux(rearrangeActive,
    rearrange.io.readResponse(1).ready,
    regularNeedsSource1 && regularOperandsAvailable &&
      regularOperandConsumerReady)
  // Check as soon as both FIFO heads exist. Gating this assertion with
  // regularOperandFire would turn a tag regression into a silent permanent
  // stall because mismatched tags deliberately make the join unavailable.
  when(!rearrangeActive && regularNeedsSource1 &&
      source0ReadResponse.valid && source1ReadResponse.valid) {
    assert(regularTagsMatch,
      "VPU independent source responses lost their operand alignment")
  }
  val executeResponseFire = regularOperandFire
  val executeResponseTag = source0ReadResponse.bits.tag
  val readData0Fp = VecInit(source0ReadResponse.bits.data.map(x =>
    VpuFloat.storageToFp32(x, p)))
  val source1Data = Mux(sourceOperandsAlias,
    source0ReadResponse.bits.data, source1ReadResponse.bits.data)
  val readData1Fp = VecInit(source1Data.map(x =>
    VpuFloat.storageToFp32(x, p)))
  sfuInputQueue.io.enq.valid := executeResponseFire && sfuStreamingState
  sfuInputQueue.io.enq.bits.offset :=
    executeResponseTag(p.vlBits - 1, 0)
  sfuInputQueue.io.enq.bits.data := readData0Fp

  val vectorFmaFabric = Module(new VpuVectorFmaFabric(p))
  val scalarFma = Module(new VpuFmaPipe)
  scalarFma.io.in.valid := false.B
  scalarFma.io.in.bits := 0.U.asTypeOf(scalarFma.io.in.bits)

  val vectorAluOperation = MuxLookup(activeExecute.opcode, VpuFpAluOp.Add, Seq(
    VpuOpcode.V_ADD_VV.U -> VpuFpAluOp.Add,
    VpuOpcode.V_ADD_VF.U -> VpuFpAluOp.Add,
    VpuOpcode.V_SUB_VV.U -> VpuFpAluOp.Sub,
    VpuOpcode.V_SUB_VF.U -> VpuFpAluOp.Sub,
    VpuOpcode.V_MUL_VV.U -> VpuFpAluOp.Mul,
    VpuOpcode.V_MUL_VF.U -> VpuFpAluOp.Mul,
    VpuOpcode.V_MAX_VF.U -> VpuFpAluOp.Max,
    VpuOpcode.V_MIN_VF.U -> VpuFpAluOp.Min))

  val aluResponseFire = executeResponseFire && executeState === exAluStream
  val aluResponseOffset =
    executeResponseTag(p.elementAddrBits - 1, 0)
  val aluResponseRemaining =
    activeExecute.elementCount.pad(p.elementAddrBits + 1) -
      aluResponseOffset.pad(p.elementAddrBits + 1)
  val aluResponseMask = Wire(Vec(p.nLanes, Bool()))
  for (lane <- 0 until p.nLanes) {
    aluResponseMask(lane) := lane.U < aluResponseRemaining &&
      (!activeExecute.maskEnable || maskFile.io.activeWord(lane))
  }

  vectorFmaFabric.io.aluIn.valid := aluResponseFire
  vectorFmaFabric.io.aluIn.bits.operation := vectorAluOperation
  vectorFmaFabric.io.aluIn.bits.laneMask := aluResponseMask
  vectorFmaFabric.io.aluIn.bits.destination :=
    vectorAddress(activeExecute.destination, aluResponseOffset,
      activeExecute.vectorStride)
  vectorFmaFabric.io.aluIn.bits.last := aluResponseOffset + p.nLanes.U >=
    activeExecute.elementCount
  for (lane <- 0 until p.nLanes) {
    val operandA = readData0Fp(lane)
    val operandB = Mux(activeExecute.useSource1,
      readData1Fp(lane), activeScalarB)
    val reverse = activeExecute.opcode === VpuOpcode.V_SUB_VF.U &&
      activeExecute.funct1 === 1.U
    // Tail lanes travel through the pipe as ordinary operations. The metadata
    // mask suppresses their data and exception flags at writeback while every
    // physical lane retains identical timing.
    vectorFmaFabric.io.aluIn.bits.operandA(lane) :=
      Mux(reverse, operandB, operandA)
    vectorFmaFabric.io.aluIn.bits.operandB(lane) :=
      Mux(reverse, operandA, operandB)
  }

  aluWritebackQueue.io.enq.valid := vectorFmaFabric.io.aluOut.valid
  aluWritebackQueue.io.enq.bits.address :=
    vectorFmaFabric.io.aluOut.bits.destination
  aluWritebackQueue.io.enq.bits.laneMask :=
    vectorFmaFabric.io.aluOut.bits.laneMask
  aluWritebackQueue.io.enq.bits.last := vectorFmaFabric.io.aluOut.bits.last
  var aluFlags = 0.U(5.W)
  for (lane <- 0 until p.nLanes) {
    val converted = VpuFloat.fp32ToStorage(
      vectorFmaFabric.io.aluOut.bits.data(lane), p)
    aluWritebackQueue.io.enq.bits.data(lane) := converted._1
    aluFlags = aluFlags | Mux(
      vectorFmaFabric.io.aluOut.bits.laneMask(lane), converted._2, 0.U)
  }
  aluWritebackQueue.io.enq.bits.fflags :=
    vectorFmaFabric.io.aluOut.bits.fflags | aluFlags
  assert(!aluWritebackQueue.io.enq.valid || aluWritebackQueue.io.enq.ready,
    "VPU ALU writeback queue overflowed")
  val aluInputAccepted = aluResponseFire
  val aluResultProduced = vectorFmaFabric.io.aluOut.valid
  when(aluInputAccepted && !aluResultProduced) {
    aluPipelineInFlight := aluPipelineInFlight + 1.U
  }.elsewhen(!aluInputAccepted && aluResultProduced) {
    assert(aluPipelineInFlight =/= 0.U,
      "VPU ALU pipeline result had no reserved credit")
    aluPipelineInFlight := aluPipelineInFlight - 1.U
  }
  assert(aluReservedResults <= aluWritebackEntries.U,
    "VPU ALU buffered plus in-flight results exceeded credit capacity")
  when (aluWritebackQueue.io.enq.fire) {
    stickyFflags := stickyFflags | aluWritebackQueue.io.enq.bits.fflags
  }

  val reductionStart = executeStart && isReduction(executeIssueBits.opcode)
  vectorFmaFabric.io.reductionStart := reductionStart
  vectorFmaFabric.io.reductionSeed := executeScalarSeed
  vectorFmaFabric.io.reductionMaximum :=
    executeIssueBits.opcode === VpuOpcode.V_RED_MAX.U
  val reductionResponseFire = executeResponseFire && reductionStreamingState
  val reductionResponseOffset =
    executeResponseTag(p.elementAddrBits - 1, 0)
  val reductionResponseRemaining =
    activeExecute.elementCount.pad(p.elementAddrBits + 1) -
      reductionResponseOffset.pad(p.elementAddrBits + 1)
  vectorFmaFabric.io.reductionIn.valid := reductionResponseFire
  vectorFmaFabric.io.reductionIn.bits.values := readData0Fp
  for (lane <- 0 until p.nLanes) {
    vectorFmaFabric.io.reductionIn.bits.laneMask(lane) :=
      lane.U < reductionResponseRemaining &&
        (!activeExecute.maskEnable || maskFile.io.activeWord(lane))
  }
  vectorFmaFabric.io.reductionIn.bits.last :=
    reductionResponseOffset + p.nLanes.U >= activeExecute.elementCount
  when (vectorFmaFabric.io.reductionOut.valid) {
    assert(reductionStreamingState,
      "a shared-lane reduction completed outside its execute command")
    fp(activeExecute.fpDestination) :=
      vectorFmaFabric.io.reductionOut.bits.data
    stickyFflags := stickyFflags |
      vectorFmaFabric.io.reductionOut.bits.fflags
    executeState := exIdle
    exDone := true.B
  }

  private val expChunks = p.nLanes / p.sfuLanes
  private val reciprocalChunks = p.nLanes / p.reciprocalLanes
  private val maxSfuChunks = math.max(expChunks, reciprocalChunks)
  private val sfuChunkBits = math.max(1, log2Ceil(maxSfuChunks))
  val sfuIssueChunk = RegInit(0.U(sfuChunkBits.W))
  val sfuResultChunk = RegInit(0.U(sfuChunkBits.W))
  val sfuResultOffset = RegInit(0.U(p.vlBits.W))
  val activeMaskWordOffset = Mux(sfuStreamingState, sfuResultOffset,
    Mux(reductionStreamingState, reductionResponseOffset,
      Mux(executeState === exAluStream, aluResponseOffset, 0.U)))
  maskFile.io.activeWordIndex := Mux(rearrangeActive,
    rearrange.io.maskWordIndex,
    (activeMaskWordOffset / p.nLanes.U)(p.maskWordIndexBits - 1, 0))
  private val sfuWritebackEntries = VpuCore.ArithmeticWritebackQueueDepth
  val sfuWritebackQueue = Module(new Queue(new VpuSfuWriteback(p),
    sfuWritebackEntries, pipe = false, flow = false))
  private val sfuReservedCountBits = math.max(1,
    log2Ceil(sfuWritebackEntries + 1))
  val sfuWordsReserved = RegInit(0.U(sfuReservedCountBits.W))
  val sfuWritebackDequeueFire = WireDefault(false.B)
  val sfuWordIssueCredit = sfuWordsReserved < sfuWritebackEntries.U ||
    sfuWritebackDequeueFire
  val vectorExp = Seq.fill(p.sfuLanes)(Module(new VpuExpApprox))
  val activeSfuLanes = Mux(activeExecute.opcode === VpuOpcode.V_RECI_V.U,
    p.reciprocalLanes.U, p.sfuLanes.U)
  val activeSfuChunks = Mux(activeExecute.opcode === VpuOpcode.V_RECI_V.U,
    reciprocalChunks.U, expChunks.U)
  val sfuIssueBase = sfuIssueChunk * activeSfuLanes
  val sfuResultBase = sfuResultChunk * activeSfuLanes
  val issuingVectorSfu = executeState === exSfuIssue &&
    sfuInputQueue.io.deq.valid &&
    (sfuIssueChunk =/= 0.U || sfuWordIssueCredit)
  val issuingVectorExp = issuingVectorSfu &&
    activeExecute.opcode === VpuOpcode.V_EXP_V.U
  val issuingVectorReciprocal = issuingVectorSfu &&
    activeExecute.opcode === VpuOpcode.V_RECI_V.U
  val issuingScalarExp = executeState === exScalarExpIssue
  val issuingScalarReciprocal = executeState === exScalarDivIssue &&
    activeExecute.opcode === VpuOpcode.S_RECI_FP.U
  val sfuIssueWordLast = sfuInputQueue.io.deq.bits.offset +
    p.nLanes.U >= activeExecute.elementCount
  val sfuIssueChunkLast = sfuIssueChunk === activeSfuChunks - 1.U
  sfuInputQueue.io.deq.ready := issuingVectorSfu && sfuIssueChunkLast
  for (lane <- 0 until p.sfuLanes) {
    val index = sfuIssueBase + lane.U
    val laneActive = sfuInputQueue.io.deq.bits.offset + index <
      activeExecute.elementCount
    vectorExp(lane).io.in.valid := issuingVectorExp ||
      (lane == 0).B && issuingScalarExp
    vectorExp(lane).io.in.bits := Mux(issuingScalarExp,
      activeScalarA,
      Mux(laneActive, sfuInputQueue.io.deq.bits.data(index), 0.U))
  }
  vectorFmaFabric.io.reciprocalIn.valid := issuingVectorReciprocal ||
    issuingScalarReciprocal
  for (lane <- 0 until p.reciprocalLanes) {
    val index = sfuIssueBase + lane.U
    val laneActive = sfuInputQueue.io.deq.bits.offset + index <
      activeExecute.elementCount
    vectorFmaFabric.io.reciprocalIn.bits.values(lane) :=
      Mux(issuingScalarReciprocal,
        Mux((lane == 0).B, activeScalarA, "h3f800000".U),
        Mux(laneActive, sfuInputQueue.io.deq.bits.data(index),
          "h3f800000".U))
  }
  when (issuingVectorExp || issuingVectorReciprocal) {
    when (sfuIssueChunkLast) {
      sfuIssueChunk := 0.U
      when (sfuIssueWordLast) {
        executeState := exSfuWait
      }
    }.otherwise {
      sfuIssueChunk := sfuIssueChunk + 1.U
    }
  }
  val vectorExpDone = vectorExp.map(_.io.out.valid).reduce(_ && _)
  val vectorReciprocalDone = vectorFmaFabric.io.reciprocalOut.valid
  val vectorSfuDone = Mux(activeExecute.opcode === VpuOpcode.V_EXP_V.U,
    vectorExpDone, vectorReciprocalDone)
  val collectingVectorSfu = (executeState === exSfuIssue ||
    executeState === exSfuWait) && vectorSfuDone
  val sfuResultChunkLast = sfuResultChunk === activeSfuChunks - 1.U
  val sfuResultWordLast = sfuResultOffset + p.nLanes.U >=
    activeExecute.elementCount
  val sfuResultLaneMask = Wire(Vec(p.nLanes, Bool()))
  for (lane <- 0 until p.nLanes) {
    val logicalIndex = sfuResultOffset + lane.U
    sfuResultLaneMask(lane) := logicalIndex < activeExecute.elementCount &&
      (!activeExecute.maskEnable || maskFile.io.activeWord(lane))
  }
  val expAssembledWord = Wire(Vec(p.nLanes, UInt(p.storageBits.W)))
  val reciprocalAssembledWord = Wire(Vec(p.nLanes, UInt(p.storageBits.W)))
  expAssembledWord := resultWord
  reciprocalAssembledWord := resultWord
  val expConvertedData = Wire(Vec(p.sfuLanes, UInt(p.storageBits.W)))
  val reciprocalConvertedData =
    Wire(Vec(p.reciprocalLanes, UInt(p.storageBits.W)))
  var expResultFlags = 0.U(5.W)
  for (lane <- 0 until p.sfuLanes) {
    val index = sfuResultBase + lane.U
    val converted = VpuFloat.fp32ToStorage(vectorExp(lane).io.out.bits.data, p)
    expConvertedData(lane) := converted._1
    expAssembledWord(index) := expConvertedData(lane)
    expResultFlags = expResultFlags | Mux(sfuResultLaneMask(index),
      vectorExp(lane).io.out.bits.fflags | converted._2, 0.U)
  }
  var reciprocalResultFlags = 0.U(5.W)
  for (lane <- 0 until p.reciprocalLanes) {
    val index = sfuResultBase + lane.U
    val result = vectorFmaFabric.io.reciprocalOut.bits.results(lane)
    val converted = VpuFloat.fp32ToStorage(result.data, p)
    reciprocalConvertedData(lane) := converted._1
    reciprocalAssembledWord(index) := reciprocalConvertedData(lane)
    reciprocalResultFlags = reciprocalResultFlags |
      Mux(sfuResultLaneMask(index), result.fflags | converted._2, 0.U)
  }
  val collectingExp = activeExecute.opcode === VpuOpcode.V_EXP_V.U
  val sfuAssembledWord = Mux(collectingExp,
    expAssembledWord, reciprocalAssembledWord)
  val sfuResultFlags = Mux(collectingExp,
    expResultFlags, reciprocalResultFlags)
  sfuWritebackQueue.io.enq.valid := collectingVectorSfu &&
    sfuResultChunkLast
  sfuWritebackQueue.io.enq.bits.address :=
    vectorAddress(activeExecute.destination, sfuResultOffset,
      activeExecute.vectorStride)
  sfuWritebackQueue.io.enq.bits.data := sfuAssembledWord
  sfuWritebackQueue.io.enq.bits.laneMask := sfuResultLaneMask
  sfuWritebackQueue.io.enq.bits.last := sfuResultWordLast
  assert(!sfuWritebackQueue.io.enq.valid || sfuWritebackQueue.io.enq.ready,
    "VPU vector SFU writeback queue overflowed")
  val sfuWordStarted = issuingVectorSfu && sfuIssueChunk === 0.U
  when(sfuWordStarted && !sfuWritebackDequeueFire) {
    sfuWordsReserved := sfuWordsReserved + 1.U
  }.elsewhen(!sfuWordStarted && sfuWritebackDequeueFire) {
    assert(sfuWordsReserved =/= 0.U,
      "VPU SFU writeback consumed a word without reserved credit")
    sfuWordsReserved := sfuWordsReserved - 1.U
  }
  assert(sfuWordsReserved <= sfuWritebackEntries.U,
    "VPU SFU buffered plus in-flight words exceeded credit capacity")
  when (collectingVectorSfu) {
    when (collectingExp) {
      for (lane <- 0 until p.sfuLanes) {
        val index = sfuResultBase + lane.U
        resultWord(index) := expConvertedData(lane)
      }
    }.otherwise {
      for (lane <- 0 until p.reciprocalLanes) {
        val index = sfuResultBase + lane.U
        resultWord(index) := reciprocalConvertedData(lane)
      }
    }
    stickyFflags := stickyFflags | sfuResultFlags
    when (sfuResultChunkLast) {
      sfuResultChunk := 0.U
      sfuResultOffset := sfuResultOffset + p.nLanes.U
    }.otherwise {
      sfuResultChunk := sfuResultChunk + 1.U
    }
  }
  when (executeStart && isSfuVector(executeIssueBits.opcode)) {
    assert(!sfuInputQueue.io.deq.valid && !sfuWritebackQueue.io.deq.valid,
      "VPU vector SFU buffers were not empty at command start")
    assert(sfuWordsReserved === 0.U,
      "VPU vector SFU credit did not drain before command start")
    sfuReadOffset := 0.U
    sfuIssueChunk := 0.U
    sfuResultChunk := 0.U
    sfuResultOffset := 0.U
  }

  io.debugSfuIssue := issuingVectorExp || issuingVectorReciprocal
  io.debugSfuResult := collectingVectorSfu
  io.debugAluReadIssue := source0Read.fire &&
    executeState === exAluStream
  io.debugAluLaneIssue := aluResponseFire
  io.debugAluResult := vectorFmaFabric.io.aluOut.valid

  when (executeState === exScalarExpIssue) { executeState := exScalarExpWait }
  when (executeState === exScalarExpWait && vectorExp.head.io.out.valid) {
    fp(activeExecute.fpDestination) := vectorExp.head.io.out.bits.data
    stickyFflags := stickyFflags | vectorExp.head.io.out.bits.fflags
    executeState := exIdle
    exDone := true.B
  }

  val scalarDiv = Module(new VpuDivSqrt)
  scalarDiv.io.in.valid := executeState === exScalarDivIssue &&
    activeExecute.opcode === VpuOpcode.S_SQRT_FP.U
  scalarDiv.io.in.bits.data := activeScalarA
  scalarDiv.io.in.bits.sqrt :=
    activeExecute.opcode === VpuOpcode.S_SQRT_FP.U
  scalarDiv.io.out.ready := executeState === exScalarDivWait
  when (executeState === exScalarDivIssue) {
    when (issuingScalarReciprocal || scalarDiv.io.in.fire) {
      executeState := exScalarDivWait
    }
  }
  when (executeState === exScalarDivWait) {
    when (activeExecute.opcode === VpuOpcode.S_RECI_FP.U &&
      vectorFmaFabric.io.reciprocalOut.valid) {
      fp(activeExecute.fpDestination) :=
        vectorFmaFabric.io.reciprocalOut.bits.results.head.data
      stickyFflags := stickyFflags |
        vectorFmaFabric.io.reciprocalOut.bits.results.head.fflags
      executeState := exIdle
      exDone := true.B
    }.elsewhen (activeExecute.opcode === VpuOpcode.S_SQRT_FP.U &&
      scalarDiv.io.out.fire) {
      fp(activeExecute.fpDestination) := scalarDiv.io.out.bits.data
      stickyFflags := stickyFflags | scalarDiv.io.out.bits.fflags
      executeState := exIdle
      exDone := true.B
    }
  }

  val scalarAluOperation = MuxLookup(activeExecute.opcode, VpuFpAluOp.Add, Seq(
    VpuOpcode.S_ADD_FP.U -> VpuFpAluOp.Add,
    VpuOpcode.S_SUB_FP.U -> VpuFpAluOp.Sub,
    VpuOpcode.S_MUL_FP.U -> VpuFpAluOp.Mul,
    VpuOpcode.S_MAX_FP.U -> VpuFpAluOp.Max))
  scalarFma.io.in.valid := executeState === exScalarAluIssue
  scalarFma.io.in.bits.op := scalarAluOperation
  scalarFma.io.in.bits.a := activeScalarA
  scalarFma.io.in.bits.b := activeScalarB
  when (executeState === exScalarAluIssue) { executeState := exScalarAluWait }
  when (executeState === exScalarAluWait && scalarFma.io.out.valid) {
    fp(activeExecute.fpDestination) := scalarFma.io.out.bits.data
    stickyFflags := stickyFflags | scalarFma.io.out.bits.fflags
    executeState := exIdle
    exDone := true.B
  }

  val aluWritebackValid = aluWritebackQueue.io.deq.valid
  val sfuWritebackValid = sfuWritebackQueue.io.deq.valid
  val rearrangeWritebackValid = rearrangeActive &&
    rearrange.io.writeRequest.valid
  assert(!(aluWritebackValid && sfuWritebackValid),
    "streaming ALU and SFU writeback queues overlapped")
  assert(!rearrangeWritebackValid ||
    !(aluWritebackValid || sfuWritebackValid),
    "rearrange and arithmetic writeback queues overlapped")
  val regularExecuteWrite = Wire(new VpuSpadWriteRequest(p))
  regularExecuteWrite.address := Mux(aluWritebackValid,
    aluWritebackQueue.io.deq.bits.address,
    sfuWritebackQueue.io.deq.bits.address)
  regularExecuteWrite.data := Mux(aluWritebackValid,
    aluWritebackQueue.io.deq.bits.data, sfuWritebackQueue.io.deq.bits.data)
  regularExecuteWrite.laneMask := Mux(aluWritebackValid,
    aluWritebackQueue.io.deq.bits.laneMask,
    sfuWritebackQueue.io.deq.bits.laneMask)
  executeWrite.valid := rearrangeWritebackValid ||
    aluWritebackValid || sfuWritebackValid
  executeWrite.bits := Mux(rearrangeWritebackValid,
    rearrange.io.writeRequest.bits, regularExecuteWrite)
  rearrange.io.writeRequest.ready := rearrangeWritebackValid &&
    executeWrite.ready
  aluWritebackQueue.io.deq.ready := !rearrangeWritebackValid &&
    aluWritebackValid && executeWrite.ready
  sfuWritebackQueue.io.deq.ready := sfuWritebackValid &&
    !rearrangeWritebackValid && !aluWritebackValid && executeWrite.ready
  aluWritebackDequeueFire := aluWritebackQueue.io.deq.fire
  sfuWritebackDequeueFire := sfuWritebackQueue.io.deq.fire
  when (executeWrite.fire) {
    when (aluWritebackValid) {
      when (aluWritebackQueue.io.deq.bits.last) {
        executeState := exIdle
        exDone := true.B
      }
    }.elsewhen (sfuWritebackValid) {
      when (sfuWritebackQueue.io.deq.bits.last) {
        executeState := exIdle
        exDone := true.B
      }
    }
  }
  when(rearrange.io.done) {
    assert(rearrangeActive,
      "VPU rearrange completion escaped its execute command")
    executeState := exIdle
    exDone := true.B
  }

  // ------------------------------------------------------------------------
  // Reservation completion, fault flush, and scalar dependencies
  // ------------------------------------------------------------------------
  val fpPendingWrite = VecInit((0 until 8).map { reg =>
    reservationStation.io.pendingFpWriteMask(reg)
  })
  val fpPendingAccess = VecInit((0 until 8).map { reg =>
    reservationStation.io.pendingFpReadMask(reg) ||
      reservationStation.io.pendingFpWriteMask(reg)
  })

  reservationStation.io.complete(0).valid := ldWritebackDone
  reservationStation.io.complete(0).bits := ldWritebackTag
  reservationStation.io.complete(1).valid := ldIssueCanceled
  reservationStation.io.complete(1).bits := activeLoad.hazardTag
  reservationStation.io.complete(2).valid := exDone
  reservationStation.io.complete(2).bits := activeExecute.hazardTag
  reservationStation.io.complete(3).valid := stDone
  reservationStation.io.complete(3).bits := stDoneTag
  reservationStation.io.complete(4).valid := stIssueCanceled
  reservationStation.io.complete(4).bits := activeStoreIssue.hazardTag
  // An accepted DMA descriptor is drained even after a fault; only entries
  // which have not reached an engine are discarded immediately.
  reservationStation.io.flushUnissued := faultActive

  val ldNoop = WireDefault(false.B)
  val exNoop = WireDefault(false.B)
  val stNoop = WireDefault(false.B)

  // ------------------------------------------------------------------------
  // Fine-grained front end: validate, snapshot, and enqueue independently
  // ------------------------------------------------------------------------
  val front = io.command.bits
  val decoded = VpuDecode(front.microOp)
  val frontOpcode = decoded.opcode
  val frontVector = VpuDecode.isVector(frontOpcode)
  val frontScalar = VpuDecode.isScalar(frontOpcode)
  val frontState = VpuDecode.isState(frontOpcode)
  val frontStateLoad = frontOpcode === VpuOpcode.S_LOAD_STATE.U
  val frontStateStore = frontOpcode === VpuOpcode.S_STORE_STATE.U
  val frontLoad = frontOpcode === VpuOpcode.H_PREFETCH_V.U
  val frontStore = frontOpcode === VpuOpcode.H_STORE_V.U
  val frontExecute = frontVector || frontScalar || frontState
  val frontReduction = isReduction(frontOpcode)
  val frontRearrange = isRearrange(frontOpcode)
  val frontGather = frontOpcode === VpuOpcode.V_GATHER_VV.U
  val frontSlide = frontOpcode === VpuOpcode.V_SLIDE_V.U
  val frontVv = isVv(frontOpcode)
  val frontVf = isVf(frontOpcode)
  val frontHasVectorDestination = frontVector && !frontReduction
  val frontIsFence = frontOpcode === VpuOpcode.C_FENCE.U
  val frontIsWait = frontOpcode === VpuOpcode.C_WAIT.U
  val frontIsRead = frontOpcode === VpuOpcode.C_READ.U
  val frontIsClear = frontOpcode === VpuOpcode.C_CLEAR_STATUS.U
  val frontMaskWrite = frontOpcode === VpuOpcode.C_WRITE_VMASK.U
  val frontSetVectorStride =
    frontOpcode === VpuOpcode.C_SET_VSTRIDE.U
  val frontIntegerAddi = frontOpcode === VpuOpcode.S_ADDI_INT.U
  val frontLoopControl = VpuDecode.isLoop(frontOpcode)
  val frontNeedsResponse = frontIsFence || frontIsRead || front.xd

  val frontGpRd = gp(decoded.rd)
  val frontGpRs1 = gp(decoded.rs1)
  val frontGpRs2 = gp(decoded.rs2)
  val frontGpRs3 = gp(decoded.rs3)
  val frontVl32 = currentVl.pad(32)
  val frontVectorSegmented = currentVectorStrideElements =/= 0.U
  val frontVectorLastLogical = Mux(currentVl === 0.U, 0.U,
    currentVl - 1.U)
  val frontVectorLastSegment =
    frontVectorLastLogical / p.matrixElementsPerRow.U
  val frontVectorLastWithinSegment =
    frontVectorLastLogical % p.matrixElementsPerRow.U
  // This half-open bounding interval covers every sparse segment and lets the
  // existing RS/shared-dependency comparators conservatively protect the
  // tile-major footprint without a second dependency mechanism.
  val frontVectorFootprintWide = Mux(currentVl === 0.U, 0.U,
    Mux(frontVectorSegmented,
      frontVectorLastSegment * currentVectorStrideElements +
        frontVectorLastWithinSegment + 1.U,
      frontVl32))
  val frontVectorFootprint =
    frontVectorFootprintWide(p.dmaTransferElementsBits - 1, 0)
  // Keep address arithmetic wider than the architectural 64-bit VA.  A
  // wrapped H + GP byte offset must be rejected rather than silently issuing
  // a descriptor at the low wrapped address.
  private val externalRangeBits = 129
  val frontHostByteOffset = frontGpRs1 * p.storageBytes.U
  val frontExternalBaseWide = h(decoded.rs2).pad(externalRangeBits) +
    frontHostByteOffset.pad(externalRangeBits)
  val frontExternalAddress = frontExternalBaseWide(63, 0)
  val frontStridedMemory = (frontLoad || frontStore) &&
    decoded.funct1 === 1.U
  val frontMemoryRows32 = Mux(frontStridedMemory,
    frontGpRs3, 1.U(32.W))
  // Validation below rejects values outside this field; using the descriptor
  // width for range arithmetic avoids synthesizing a needless 32x64-bit
  // multiplier on the RoCC admission path.
  val frontMemoryRows =
    frontMemoryRows32(p.dmaRowCountBits - 1, 0)
  val frontMemoryRowsValid = currentVl === 0.U ||
    (!frontStridedMemory || (frontMemoryRows32 =/= 0.U &&
      frontMemoryRows32 <= p.dmaMaxRows.U))
  val frontMemoryFootprint = Mux(currentVl === 0.U, 0.U,
    (frontMemoryRows - 1.U) * p.vLen.U + currentVl)
  val frontMemoryRowBytes = currentVl * p.storageBytes.U
  val frontLastHostRowOffset = Mux(frontStridedMemory,
    (frontMemoryRows - 1.U) * currentDmaStrideBytes, 0.U)
  val frontExternalEndWide = frontExternalBaseWide +
    frontLastHostRowOffset.pad(externalRangeBits) +
    frontMemoryRowBytes.pad(externalRangeBits)
  // end is exclusive, so exactly 2^64 is legal while any larger value wraps
  // an addressed byte. VL=0 remains a side-effect-free no-op and deliberately
  // bypasses all address validation.
  val frontExternalRangeValid = currentVl === 0.U ||
    frontExternalEndWide <= (BigInt(1) << 64).U(externalRangeBits.W)
  // Multi-source TL puts do not provide an ordering contract between
  // overlapping rows. Loads may intentionally use stride zero as a broadcast,
  // but stores must describe disjoint external row intervals.
  val frontStoreRowsNonOverlapping = currentVl === 0.U || !frontStore ||
    !frontStridedMemory || frontMemoryRows32 <= 1.U ||
    currentDmaStrideBytes >= frontMemoryRowBytes

  def rangeValid(base: UInt, count: UInt): Bool = {
    val end = base +& count
    count === 0.U || (base < p.totalElements.U &&
      // Gemmini's fused matrix port exposes one nLanes-wide matrix row at a
      // time.  Such rows are naturally SRAM-word aligned but are not generally
      // VLEN-slot aligned.  Every VPU engine already advances and writes in
      // nLanes words, so this is the architectural minimum safe alignment.
      (base % p.nLanes.U) === 0.U && end <= p.totalElements.U &&
      (base / p.elementsPerBank.U) ===
        ((end - 1.U) / p.elementsPerBank.U))
  }

  val frontSpadRangesValid = Mux(frontLoad || frontStore,
    rangeValid(frontGpRd, frontMemoryFootprint),
    Mux(frontVector,
      rangeValid(frontGpRs1, frontVectorFootprintWide) &&
        (!frontVv ||
          rangeValid(frontGpRs2, frontVectorFootprintWide)) &&
        (!frontHasVectorDestination ||
          rangeValid(frontGpRd, frontVectorFootprintWide)) &&
        (!frontVectorSegmented ||
          (!frontRearrange &&
            frontGpRs1 % p.matrixElementsPerRow.U === 0.U &&
            (!frontVv ||
              frontGpRs2 % p.matrixElementsPerRow.U === 0.U) &&
            (!frontHasVectorDestination ||
              frontGpRd % p.matrixElementsPerRow.U === 0.U))),
      true.B))
  // Arbitrary gather cannot safely overwrite a source/index vector without
  // buffering the complete vector. Slide has a direction-aware traversal and
  // deliberately permits its PLENA-style in-place form.
  val frontGatherAliases = frontGather &&
    (frontGpRd === frontGpRs1 || frontGpRd === frontGpRs2)
  val frontMemoryAddressAligned = currentVl === 0.U ||
    (!frontLoad && !frontStore) ||
    (frontExternalAddress % p.storageBytes.U === 0.U &&
      (!frontStridedMemory || frontMemoryRows32 <= 1.U ||
        currentDmaStrideBytes % p.storageBytes.U === 0.U))
  val frontRangesValid = frontSpadRangesValid && frontMemoryAddressAligned &&
    frontMemoryRowsValid &&
    (!(frontLoad || frontStore) || frontExternalRangeValid) &&
    frontStoreRowsNonOverlapping && !frontGatherAliases

  val frontScalarBinary = Seq(VpuOpcode.S_ADD_FP, VpuOpcode.S_SUB_FP,
    VpuOpcode.S_MAX_FP, VpuOpcode.S_MUL_FP)
    .map(x => frontOpcode === x.U).reduce(_ || _)
  val frontScalarUnary = frontScalar && !frontScalarBinary
  val scalarIndicesValid = decoded.rd < 8.U && decoded.rs1 < 8.U &&
    (!frontScalarBinary || decoded.rs2 < 8.U)
  val scalarFieldsValid = scalarIndicesValid && decoded.rs3 === 0.U &&
    decoded.funct1 === 0.U && (!frontScalarUnary || decoded.rs2 === 0.U)
  val frontStateIndex32 = Mux(frontStateLoad, frontGpRs1, frontGpRd)
  val stateFieldsValid = decoded.rs2 === 0.U && decoded.rs3 === 0.U &&
    decoded.funct1 === 0.U &&
    Mux(frontStateLoad, decoded.rd < 8.U,
      decoded.rs1 < 8.U) &&
    frontStateIndex32 < p.fpStateEntries.U
  val frontUnaryVector = frontOpcode === VpuOpcode.V_EXP_V.U ||
    frontOpcode === VpuOpcode.V_RECI_V.U
  val vectorFunctValid = Mux(frontOpcode === VpuOpcode.V_SUB_VF.U ||
    frontSlide, decoded.funct1 <= 1.U, decoded.funct1 === 0.U)
  val vectorFieldsValid = decoded.rs3 <= 1.U &&
    (!frontVf || decoded.rs2 < 8.U) &&
    (!(frontUnaryVector || frontReduction) || decoded.rs2 === 0.U) &&
    vectorFunctValid &&
    (!frontReduction || decoded.rd < 8.U)
  // funct1=0 is the legacy single-row encoding and keeps every old binary
  // legal. funct1=1 snapshots GP[rs3] as rowCount and the configured byte
  // stride; no additional command is emitted per row.
  val memoryFieldsValid = decoded.funct1 <= 1.U &&
    (decoded.funct1 === 1.U || decoded.rs3 === 0.U)
  val frontControl = VpuDecode.isControl(frontOpcode)
  val controlUsesRd = frontOpcode === VpuOpcode.C_WRITE_GP.U ||
    frontIntegerAddi ||
    frontOpcode === VpuOpcode.C_WRITE_FP.U ||
    frontOpcode === VpuOpcode.C_WRITE_H.U ||
    frontOpcode === VpuOpcode.C_WRITE_VMASK.U || frontIsRead
  val controlUsesRs1 = frontIsRead || frontIntegerAddi
  // S_ADDI_INT deliberately overlays rs2/rs3/funct1/reserved with imm[17:0],
  // so its format is validated separately from ordinary control commands.
  val ordinaryControlFieldsValid = decoded.rs2 === 0.U &&
    decoded.rs3 === 0.U && decoded.funct1 === 0.U &&
    (controlUsesRd || decoded.rd === 0.U) &&
    (controlUsesRs1 || decoded.rs1 === 0.U) &&
    (frontOpcode =/= VpuOpcode.C_WRITE_FP.U || decoded.rd < 8.U) &&
    (frontOpcode =/= VpuOpcode.C_WRITE_VMASK.U ||
      decoded.rd < p.vectorMaskChunks.U)
  val controlFieldsValid = Mux(frontIntegerAddi,
    front.payload === 0.U, ordinaryControlFieldsValid)
  val setVectorStrideValid = !frontSetVectorStride ||
    (front.payload < p.elementsPerBank.U &&
      (front.payload === 0.U ||
        (front.payload >= p.matrixElementsPerRow.U &&
          front.payload % p.matrixElementsPerRow.U === 0.U)))
  val setVlValid = frontOpcode =/= VpuOpcode.C_SET_VL.U ||
    front.payload <= p.vLen.U
  val waitPayloadValid = !frontIsWait || front.payload(63, 3) === 0.U
  val clearPayloadValid = !frontIsClear || front.payload(63, 3) === 0.U
  val xdValid = Mux(frontIsRead || frontIsFence, front.xd, !front.xd)
  val frontMalformed = (!frontIntegerAddi && decoded.reserved.orR) ||
    !VpuDecode.isSupported(frontOpcode) ||
    // Loop delimiters are interpreted by VpuHardwareLoop.  Seeing one at the
    // execution core means it was unmatched or otherwise malformed.
    frontLoopControl ||
    (frontScalar && !scalarFieldsValid) ||
    (frontState && !stateFieldsValid) ||
    (frontVector && !vectorFieldsValid) ||
    ((frontLoad || frontStore) && !memoryFieldsValid) ||
    (frontControl && !controlFieldsValid) ||
    !xdValid || !setVectorStrideValid || !setVlValid ||
    !waitPayloadValid || !clearPayloadValid

  val perf = RegInit(VecInit(Seq.fill(VpuPerfIndex.Count)(0.U(64.W))))
  io.perfCounters := perf
  val perfRead = Mux(decoded.rd < VpuPerfIndex.Count.U,
    perf(decoded.rd), 0.U)
  val readSelectorValid = decoded.rs1 <= VpuReadSelector.FaultInfo.U &&
    MuxLookup(decoded.rs1, false.B, Seq(
      VpuReadSelector.Status.U -> true.B,
      VpuReadSelector.Gp.U -> true.B,
      VpuReadSelector.Fp.U -> (decoded.rd < 8.U),
      VpuReadSelector.H.U -> true.B,
      VpuReadSelector.Perf.U -> (decoded.rd < VpuPerfIndex.Count.U),
      VpuReadSelector.FaultAddress.U -> true.B,
      VpuReadSelector.FaultInfo.U -> true.B))
  val commandInvalid = frontMalformed || !frontRangesValid ||
    (frontIsRead && !readSelectorValid)

  val frontFpWrites = frontScalar || frontReduction || frontStateLoad
  val frontFpDestination = decoded.rd(2, 0)
  val frontFpRdOH = UIntToOH(decoded.rd, 8)
  val frontFpRs1OH = UIntToOH(decoded.rs1, 8)
  val frontFpRs2OH = UIntToOH(decoded.rs2, 8)
  val frontScalarReadMask = frontFpRs1OH |
    Mux(frontScalarBinary, frontFpRs2OH, 0.U(8.W))
  val frontFpReadMask = Mux(frontScalar, frontScalarReadMask,
    Mux(frontVf, frontFpRs2OH,
      Mux(frontReduction, frontFpRdOH,
        Mux(frontStateStore, frontFpRs1OH, 0.U(8.W)))))
  val frontFpWriteMask = Mux(frontFpWrites,
    frontFpRdOH, 0.U(8.W))

  val loadAdmission = currentVl === 0.U ||
    reservationStation.io.allocate.load.ready
  val storeAdmission = currentVl === 0.U ||
    reservationStation.io.allocate.store.ready
  val executeAdmission = (frontVector && currentVl === 0.U) ||
    reservationStation.io.allocate.execute.ready

  val enginesIdle = loadState === lIdle && !loadInFlight.asUInt.orR &&
    executeState === exIdle &&
    storeState === stIdle
  val enginesDrained = !reservationStation.io.busy && enginesIdle &&
    !loadLineMerger.io.busy && !storeTransportTags.io.busy &&
    !storeStreamQueue.io.deq.valid && !scratchpad.io.busy &&
    !executeWrite.valid && !loadWrite.valid

  // FFLAGS and performance counters are local selective clears. Fault and
  // illegal state share bit 1 because a DMA fault additionally requires the
  // engines to drain before coordinating the wrapper/TLB clear handshake.
  val clearFaultIllegalRequested = frontIsClear && front.payload(1)
  val clearReady = !clearFaultIllegalRequested || enginesDrained
  // A queued reader now observes FP values at execute issue.  An immediate
  // control write must therefore wait for all older users of that register,
  // while a control read only needs to wait for older writers.
  val fpControlReady = Mux(frontOpcode === VpuOpcode.C_WRITE_FP.U,
    !fpPendingAccess(decoded.rd(2, 0)),
    Mux(frontIsRead && decoded.rs1 === VpuReadSelector.Fp.U,
      !fpPendingWrite(decoded.rd(2, 0)), true.B))
  val responseSpace = !frontNeedsResponse || responseQueue.io.enq.ready
  val faultDiscard = faultActive && !frontNeedsResponse && !frontIsClear

  val validCommandReady = Mux(faultDiscard, true.B,
    Mux(frontLoad, loadAdmission,
      Mux(frontStore, storeAdmission,
          Mux(frontExecute, executeAdmission,
          Mux(frontIsRead, responseSpace && fpControlReady,
            Mux(frontOpcode === VpuOpcode.C_WRITE_FP.U, fpControlReady,
              Mux(frontMaskWrite, maskFile.io.write.ready,
                Mux(frontIsClear, clearReady, true.B))))))))

  val barrierActive = RegInit(false.B)
  val barrierFence = RegInit(false.B)
  private val rsLoadEntries = VpuReservationStation.loadEntries(p)
  private val rsExecuteEntries = VpuReservationStation.executeEntries(p)
  private val rsStoreEntries = VpuReservationStation.storeEntries(p)
  private val rsTotalEntries = VpuReservationStation.totalEntries(p)
  val barrierEntries = RegInit(0.U(rsTotalEntries.W))
  val barrierRd = Reg(UInt(5.W))

  io.command.ready := !barrierActive && !clearPending &&
    Mux(commandInvalid, responseSpace,
      validCommandReady && (!frontIsFence || responseQueue.io.enq.ready))
  val commandFire = io.command.fire
  val acceptedValid = commandFire && !commandInvalid && !faultDiscard
  maskFile.io.write.valid := acceptedValid && frontMaskWrite
  maskFile.io.write.bits.chunk :=
    decoded.rd(p.maskChunkIndexBits - 1, 0)
  maskFile.io.write.bits.data := front.payload
  when(acceptedValid && frontMaskWrite) {
    assert(maskFile.io.write.fire,
      "accepted C_WRITE_VMASK did not update the protected mask file")
  }

  val frontAccessSet = Wire(new VpuRsAccessSet(p))
  frontAccessSet := 0.U.asTypeOf(frontAccessSet)
  when (frontLoad) {
    frontAccessSet.accesses(0).base := frontGpRd
    frontAccessSet.accesses(0).elementCount := frontMemoryFootprint
    frontAccessSet.accesses(0).write := true.B
  }.elsewhen (frontStore) {
    frontAccessSet.accesses(0).base := frontGpRd
    frontAccessSet.accesses(0).elementCount := frontMemoryFootprint
    frontAccessSet.accesses(0).read := true.B
  }.elsewhen (frontVector) {
    frontAccessSet.accesses(0).base := frontGpRs1
    frontAccessSet.accesses(0).elementCount := frontVectorFootprint
    frontAccessSet.accesses(0).read := true.B
    when (frontVv) {
      frontAccessSet.accesses(1).base := frontGpRs2
      frontAccessSet.accesses(1).elementCount := frontVectorFootprint
      frontAccessSet.accesses(1).read := true.B
    }
    when (frontHasVectorDestination) {
      frontAccessSet.accesses(2).base := frontGpRd
      frontAccessSet.accesses(2).elementCount := frontVectorFootprint
      frontAccessSet.accesses(2).write := true.B
    }
  }

  reservationStation.io.allocate.load.valid := acceptedValid && frontLoad &&
    currentVl =/= 0.U
  reservationStation.io.allocate.load.bits.command.vaddr := frontExternalAddress
  reservationStation.io.allocate.load.bits.command.spadBase := frontGpRd
  reservationStation.io.allocate.load.bits.command.elementCount := currentVl
  reservationStation.io.allocate.load.bits.command.rowCount :=
    frontMemoryRows32(p.dmaRowCountBits - 1, 0)
  reservationStation.io.allocate.load.bits.command.hostStrideBytes :=
    Mux(frontStridedMemory, currentDmaStrideBytes, 0.U)
  reservationStation.io.allocate.load.bits.command.status := front.status
  reservationStation.io.allocate.load.bits.command.hazardTag := 0.U
  reservationStation.io.allocate.load.bits.accessSet := frontAccessSet

  reservationStation.io.allocate.store.valid := acceptedValid && frontStore &&
    currentVl =/= 0.U
  reservationStation.io.allocate.store.bits.command.vaddr := frontExternalAddress
  reservationStation.io.allocate.store.bits.command.spadBase := frontGpRd
  reservationStation.io.allocate.store.bits.command.elementCount := currentVl
  reservationStation.io.allocate.store.bits.command.rowCount :=
    frontMemoryRows32(p.dmaRowCountBits - 1, 0)
  reservationStation.io.allocate.store.bits.command.hostStrideBytes :=
    Mux(frontStridedMemory, currentDmaStrideBytes, 0.U)
  reservationStation.io.allocate.store.bits.command.status := front.status
  reservationStation.io.allocate.store.bits.command.hazardTag := 0.U
  reservationStation.io.allocate.store.bits.command.transportTag := 0.U
  reservationStation.io.allocate.store.bits.accessSet := frontAccessSet

  reservationStation.io.allocate.execute.valid := acceptedValid && frontExecute &&
    (!frontVector || currentVl =/= 0.U)
  reservationStation.io.allocate.execute.bits.command.opcode := frontOpcode
  reservationStation.io.allocate.execute.bits.command.funct1 := decoded.funct1
  reservationStation.io.allocate.execute.bits.command.destination := frontGpRd
  reservationStation.io.allocate.execute.bits.command.source0 := frontGpRs1
  reservationStation.io.allocate.execute.bits.command.source1 := frontGpRs2
  reservationStation.io.allocate.execute.bits.command.elementCount := currentVl
  reservationStation.io.allocate.execute.bits.command.vectorStride :=
    currentVectorStrideElements
  reservationStation.io.allocate.execute.bits.command.useSource1 := frontVv
  reservationStation.io.allocate.execute.bits.command.maskEnable :=
    decoded.rs3 === 1.U
  reservationStation.io.allocate.execute.bits.command.maskSlot :=
    maskFile.io.currentSlot
  reservationStation.io.allocate.execute.bits.command.gpRs2Value := frontGpRs2
  reservationStation.io.allocate.execute.bits.command.fpRs1 := decoded.rs1(2, 0)
  reservationStation.io.allocate.execute.bits.command.fpRs2 := decoded.rs2(2, 0)
  reservationStation.io.allocate.execute.bits.command.fpSeed := decoded.rd(2, 0)
  reservationStation.io.allocate.execute.bits.command.fpReadMask := frontFpReadMask
  reservationStation.io.allocate.execute.bits.command.fpWriteMask := frontFpWriteMask
  reservationStation.io.allocate.execute.bits.command.stateIndex :=
    frontStateIndex32(p.fpStateIndexBits - 1, 0)
  reservationStation.io.allocate.execute.bits.command.fpDestination :=
    frontFpDestination
  reservationStation.io.allocate.execute.bits.command.hazardTag := 0.U
  reservationStation.io.allocate.execute.bits.accessSet := frontAccessSet

  when(reservationStation.io.allocate.load.valid) {
    assert(reservationStation.io.allocate.load.fire)
  }
  when(reservationStation.io.allocate.execute.valid) {
    assert(reservationStation.io.allocate.execute.fire)
  }
  when(reservationStation.io.allocate.store.valid) {
    assert(reservationStation.io.allocate.store.fire)
  }
  if (p.enableGroupedCommands) {
    io.commandRsAdmission.get := reservationStation.io.allocate.load.fire ||
      reservationStation.io.allocate.execute.fire ||
      reservationStation.io.allocate.store.fire
    // Report every command consumed without architectural execution or an RS
    // allocation.  In particular, a grouped command discarded while a DMA
    // fault is active must abort/drain its fusion group instead of being lost
    // silently and leaving LdBCompleteControl permanently allocated.
    io.commandRejected.get := commandFire && (commandInvalid || faultDiscard)
    io.commandClearsFusionFault.get := acceptedValid && frontIsClear &&
      front.payload(1)
  }

  ldNoop := acceptedValid && frontLoad && currentVl === 0.U
  stNoop := acceptedValid && frontStore && currentVl === 0.U
  exNoop := acceptedValid && frontVector && currentVl === 0.U

  def statusValue(busyValue: Bool): UInt = {
    Cat(0.U(47.W), busyValue, 0.U(3.W), stickyFflags, 0.U(4.W),
      io.fusionFault.getOrElse(false.B),
      io.dma.halted, dmaFaultStatus, illegalCommand)
  }

  val busyValue = !enginesDrained || barrierActive || clearPending ||
    responseQueue.io.deq.valid
  val status = statusValue(busyValue)
  val readStatus = statusValue(!enginesDrained || clearPending ||
    responseQueue.io.deq.valid)
  val faultInfoRead = Cat(0.U(60.W), faultValid,
    faultIsWrite, faultCause)
  val readData = MuxLookup(decoded.rs1, 0.U(64.W), Seq(
    VpuReadSelector.Status.U -> readStatus,
    VpuReadSelector.Gp.U -> gp(decoded.rd).pad(64),
    VpuReadSelector.Fp.U -> fp(decoded.rd(2, 0)).pad(64),
    VpuReadSelector.H.U -> h(decoded.rd),
    VpuReadSelector.Perf.U -> perfRead,
    VpuReadSelector.FaultAddress.U -> faultAddress,
    VpuReadSelector.FaultInfo.U -> faultInfoRead))

  when (commandFire && commandInvalid) {
    illegalCommand := true.B
    when (front.xd) {
      responseQueue.io.enq.valid := true.B
      responseQueue.io.enq.bits.rd := front.rd
      responseQueue.io.enq.bits.data := status | 1.U
    }
  }

  when (acceptedValid) {
    switch (frontOpcode) {
      is (VpuOpcode.C_WRITE_GP.U) { gp(decoded.rd) := front.payload(31, 0) }
      is (VpuOpcode.S_ADDI_INT.U) {
        gp(decoded.rd) := frontGpRs1 + front.microOp(31, 14)
      }
      is (VpuOpcode.C_WRITE_FP.U) {
        fp(decoded.rd(2, 0)) := front.payload(31, 0)
      }
      is (VpuOpcode.C_WRITE_H.U) { h(decoded.rd) := front.payload }
      is (VpuOpcode.C_SET_VSTRIDE.U) {
        currentVectorStrideElements :=
          front.payload(p.elementAddrBits - 1, 0)
      }
      is (VpuOpcode.C_SET_STRIDE.U) {
        currentDmaStrideBytes := front.payload
      }
      is (VpuOpcode.C_WRITE_VMASK.U) {
        // The copy-on-write mask file updates on the same accepted command.
        // Keeping this explicit case documents that the opcode has no other
        // architectural side effect in VpuCore.
      }
      is (VpuOpcode.C_SET_VL.U) {
        currentVl := front.payload(p.vlBits - 1, 0)
      }
      is (VpuOpcode.C_WAIT.U) {
        barrierActive := true.B
        barrierFence := false.B
        val selectedClasses = Cat(
          Fill(rsStoreEntries, front.payload(2)),
          Fill(rsExecuteEntries, front.payload(1)),
          Fill(rsLoadEntries, front.payload(0)))
        barrierEntries := reservationStation.io.validMask & selectedClasses
      }
      is (VpuOpcode.C_FENCE.U) {
        barrierActive := true.B
        barrierFence := true.B
        barrierEntries := reservationStation.io.validMask
        barrierRd := front.rd
      }
      is (VpuOpcode.C_READ.U) {
        responseQueue.io.enq.valid := true.B
        responseQueue.io.enq.bits.rd := front.rd
        responseQueue.io.enq.bits.data := readData
      }
      is (VpuOpcode.C_CLEAR_STATUS.U) {
        when (front.payload(0)) {
          stickyFflags := 0.U
        }
        when (front.payload(1)) {
          illegalCommand := false.B
          when (dmaFaultStatus) {
            clearPending := true.B
            clearAcknowledged := false.B
          }.otherwise {
            faultValid := false.B
            faultAddress := 0.U
            faultCause := 0.U
            faultIsWrite := false.B
          }
        }
      }
    }
  }

  val barrierEntriesDone =
    !(barrierEntries & reservationStation.io.validMask).orR
  val barrierSatisfied = barrierActive && barrierEntriesDone &&
    (!barrierFence || enginesDrained)
  when (barrierSatisfied) {
    when (barrierFence) {
      when (responseQueue.io.enq.ready) {
        responseQueue.io.enq.valid := true.B
        responseQueue.io.enq.bits.rd := barrierRd
        responseQueue.io.enq.bits.data := statusValue(false.B)
        barrierActive := false.B
      }
    }.otherwise {
      barrierActive := false.B
    }
  }

  // ------------------------------------------------------------------------
  // Fault latch and coordinated wrapper/TLB clear handshake
  // ------------------------------------------------------------------------
  when (readFaultEvent && !faultValid) {
    faultValid := true.B
    faultAddress := loadWord.fault.vaddr
    faultCause := loadWord.fault.cause
    faultIsWrite := false.B
  }.elsewhen (writeFaultEvent && !faultValid) {
    faultValid := true.B
    faultAddress := io.dma.writeCompletion.bits.fault.vaddr
    faultCause := io.dma.writeCompletion.bits.fault.cause
    faultIsWrite := true.B
  }

  io.dma.clearFault := clearPending && !clearAcknowledged
  when (io.dma.clearFaultDone) {
    clearAcknowledged := true.B
    faultValid := false.B
    faultAddress := 0.U
    faultCause := 0.U
    faultIsWrite := false.B
  }
  when (clearPending && (clearAcknowledged || io.dma.clearFaultDone) &&
    !io.dma.halted) {
    clearPending := false.B
    clearAcknowledged := false.B
  }

  // ------------------------------------------------------------------------
  // Performance counters and public status/debug signals
  // ------------------------------------------------------------------------
  val clearPerf = acceptedValid && frontIsClear && front.payload(2)
  val executing = executeState =/= exIdle
  val dmaActive = loadState =/= lIdle || storeState =/= stIdle
  val rsUnissuedMask = reservationStation.io.validMask &
    ~reservationStation.io.issuedMask
  val loadWaiting = rsUnissuedMask(rsLoadEntries - 1, 0).orR
  val executeWaiting = rsUnissuedMask(
    rsLoadEntries + rsExecuteEntries - 1, rsLoadEntries).orR
  val storeWaiting = rsUnissuedMask(
    rsTotalEntries - 1, rsLoadEntries + rsExecuteEntries).orR
  val hazardBlocked =
    (loadWaiting && !reservationStation.io.issue.load.valid) ||
    (executeWaiting && !reservationStation.io.issue.execute.valid) ||
    (storeWaiting && !reservationStation.io.issue.store.valid)
  when (clearPerf) {
    for (i <- 0 until VpuPerfIndex.Count) { perf(i) := 0.U }
  }.otherwise {
    perf(VpuPerfIndex.Cycles) := perf(VpuPerfIndex.Cycles) + 1.U
    when (busyValue) {
      perf(VpuPerfIndex.BusyCycles) :=
        perf(VpuPerfIndex.BusyCycles) + 1.U
    }
    when (dmaReadFire) {
      perf(VpuPerfIndex.DmaReadBytes) :=
        perf(VpuPerfIndex.DmaReadBytes) + dmaReadByteIncrement
    }
    when (dmaWriteFire) {
      perf(VpuPerfIndex.DmaWriteBytes) :=
        perf(VpuPerfIndex.DmaWriteBytes) + dmaWriteByteIncrement
    }
    when (dmaActive && executing) {
      perf(VpuPerfIndex.DmaExecuteOverlapCycles) :=
        perf(VpuPerfIndex.DmaExecuteOverlapCycles) + 1.U
    }
    when (scratchpad.io.readConflictStall ||
      scratchpad.io.writeConflictStall) {
      perf(VpuPerfIndex.BankConflictStallCycles) :=
        perf(VpuPerfIndex.BankConflictStallCycles) + 1.U
    }
    when (hazardBlocked) {
      perf(VpuPerfIndex.HazardStallCycles) :=
        perf(VpuPerfIndex.HazardStallCycles) + 1.U
    }
    when (executeState === exSfuIssue || executeState === exSfuWait ||
      executeState === exScalarExpIssue || executeState === exScalarExpWait ||
      executeState === exScalarDivIssue || executeState === exScalarDivWait) {
      perf(VpuPerfIndex.SfuBusyCycles) :=
        perf(VpuPerfIndex.SfuBusyCycles) + 1.U
    }
    when (readFaultEvent || writeFaultEvent) {
      perf(VpuPerfIndex.Faults) := perf(VpuPerfIndex.Faults) + 1.U
    }
  }

  io.busy := busyValue
  io.status := status
  io.fflags := stickyFflags
  io.debugState := executeState.pad(5)
  io.debugLoadState := loadState
  io.debugHeadOpcode := Mux(io.command.valid, decoded.opcode, 0.U)
  io.debugActiveOpcode := Mux(executeState =/= exIdle,
    activeExecute.opcode, 0.U)
  io.debugAluWriteback := executeWrite.fire && aluWritebackValid
  io.debugSfuWriteback := executeWrite.fire && sfuWritebackValid

  // Named, preserved activity signals for waveform-level utilization checks.
  // A unit remains busy for its complete accepted-input -> final-consumed-
  // result lifetime, rather than pulsing only when a request happens to fire.
  val reservationStationBusy = WireInit(reservationStation.io.busy)
  val loadUnitBusy = loadIssueState =/= lIdle ||
    loadInFlight.asUInt.orR || loadLineMerger.io.busy
  val storeUnitBusy = storeIssueState =/= siIdle || storeStreamActive ||
    storeStreamQueue.io.deq.valid || storeSerializerValid ||
    storeReadsOutstanding =/= 0.U || storeTransportTags.io.busy
  val executeUnitBusy = executeState =/= exIdle
  val vectorElementwiseUnitBusy = executeUnitBusy &&
    isBaseVectorAlu(activeExecute.opcode)
  val vectorRearrangeUnitBusy = WireInit(rearrange.io.busy)
  val reductionUnitBusy = executeUnitBusy &&
    isReduction(activeExecute.opcode)
  val expUnitBusy = executeUnitBusy &&
    (activeExecute.opcode === VpuOpcode.V_EXP_V.U ||
      activeExecute.opcode === VpuOpcode.S_EXP_FP.U)
  val reciprocalUnitBusy = executeUnitBusy &&
    (activeExecute.opcode === VpuOpcode.V_RECI_V.U ||
      activeExecute.opcode === VpuOpcode.S_RECI_FP.U)
  val vectorFmaFabricBusy = vectorElementwiseUnitBusy || reductionUnitBusy ||
    reciprocalUnitBusy || aluWritebackQueue.io.deq.valid
  val scalarFmaUnitBusy = executeState === exScalarAluIssue ||
    executeState === exScalarAluWait
  val scalarDivSqrtUnitBusy =
    (executeState === exScalarDivIssue || executeState === exScalarDivWait) &&
      activeExecute.opcode === VpuOpcode.S_SQRT_FP.U
  val fpStateUnitBusy = executeState === exStateReadIssue ||
    executeState === exStateReadWait || executeState === exStateWrite ||
    fpStateSram.io.busy
  val vsramBusy = WireInit(scratchpad.io.busy)
  val loadDmaBusy = loadInFlight.asUInt.orR
  val storeDmaBusy = WireInit(storeTransportTags.io.busy)
  dontTouch(reservationStationBusy)
  dontTouch(loadUnitBusy)
  dontTouch(storeUnitBusy)
  dontTouch(executeUnitBusy)
  dontTouch(vectorElementwiseUnitBusy)
  dontTouch(vectorRearrangeUnitBusy)
  dontTouch(reductionUnitBusy)
  dontTouch(expUnitBusy)
  dontTouch(reciprocalUnitBusy)
  dontTouch(vectorFmaFabricBusy)
  dontTouch(scalarFmaUnitBusy)
  dontTouch(scalarDivSqrtUnitBusy)
  dontTouch(fpStateUnitBusy)
  dontTouch(vsramBusy)
  dontTouch(loadDmaBusy)
  dontTouch(storeDmaBusy)
}
