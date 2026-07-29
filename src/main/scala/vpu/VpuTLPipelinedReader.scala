package vpu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.diplomacy.{IdRange, LazyModule, LazyModuleImp}
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.tile.HasCoreParameters
import freechips.rocketchip.tilelink._

/** A complete TileLink response transaction after ownership has moved out of
  * the source-ID tracker.  This is the VPU equivalent of Gemmini's BeatMerger
  * entry: the source can be reused while this line waits for SRAM writeback.
  */
private[vpu] class VpuDmaReadLine(p: VpuParams) extends Bundle {
  private val requestBytesBits = log2Ceil(p.dmaMaxBytes + 1)
  private val beatsPerLine = p.dmaMaxBytes / (p.dmaBusWidth / 8)

  val commandTag = UInt(p.dmaCommandTagBits.W)
  val spadElement = UInt(p.elementAddrBits.W)
  val usefulBytes = UInt(requestBytesBits.W)
  val usefulShift = UInt(requestBytesBits.W)
  val data = Vec(beatsPerLine, UInt(p.dmaBusWidth.W))
}

/** Gemmini-style VPU read DMA.
  *
  * There are four independent lifetimes:
  *
  *   1. descriptor queue / request generation,
  *   2. source-ID translation and TileLink request tracking,
  *   3. complete response-line buffering and beat unpacking, and
  *   4. command completion after the tagged stream reaches VpuCore.
  *
  * A descriptor leaves the request generator when all of its TL transactions
  * have been reserved, not when its SRAM writes finish.  Source metadata maps
  * an out-of-order D response back to its command tag and absolute VSRAM
  * address.  VpuCore performs the final architectural completion accounting
  * only after the tagged beat is accepted by the physical SRAM writer.
  */
private[vpu] class VpuTLReaderPipelined(vpuParams: VpuParams)
    (implicit params: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(
    TLClientParameters(
      name = "vpu-reader-pipelined",
      sourceId = IdRange(0, vpuParams.dmaMaxInFlight))))))

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) with HasCoreParameters
      with MemoryOpConstants {
    private val beatBytes = vpuParams.dmaBusWidth / 8
    private val requestMaxBytes = vpuParams.dmaMaxBytes
    private val maxBeatsPerRequest = requestMaxBytes / beatBytes
    private val nSources = vpuParams.dmaMaxInFlight
    private val sourceBits = math.max(1, log2Ceil(nSources))
    private val translationTagBits = sourceBits + 1
    // Request-generation offsets are per logical row.  Command completion
    // counters below use the wider, bank-bounded 2-D transfer width.
    private val byteCountBits =
      log2Ceil(vpuParams.vLen * vpuParams.storageBytes + 1)
    private val requestBytesBits = log2Ceil(requestMaxBytes + 1)
    private val responseBeatBits =
      math.max(1, log2Ceil(maxBeatsPerRequest + 1))

    require(requestMaxBytes >= beatBytes &&
      requestMaxBytes % beatBytes == 0)
    require(isPow2(maxBeatsPerRequest))
    require(requestMaxBytes <= (1 << pgIdxBits),
      "VPU DMA transactions must fit within one virtual-memory page")

    val io = IO(new Bundle {
      val descriptor = Flipped(Decoupled(new VpuDmaDescriptor(vpuParams)))
      val data = Decoupled(new VpuDmaReadBeat(vpuParams))
      val translationRequest = Decoupled(
        new VpuTLBTaggedRequest(
          translationTagBits, requestMaxBytes)(p))
      val translationResult = Flipped(Decoupled(
        new VpuTLBTaggedResult(translationTagBits)(p)))
      val busy = Output(Bool())
      val fault = Output(Bool())
    })

    val (tl, edge) = node.out.head

    // ----------------------------------------------------------------------
    // Descriptor admission and per-command completion state
    // ----------------------------------------------------------------------
    val descriptorQueue = Module(new Queue(
      new VpuDmaDescriptor(vpuParams), vpuParams.loadQueueEntries))
    val commandActive = RegInit(VecInit(
      Seq.fill(vpuParams.hazardEntries)(false.B)))
    val commandElementsToGenerate = Reg(Vec(
      vpuParams.hazardEntries,
      UInt(vpuParams.dmaTransferElementsBits.W)))
    val zeroCompletionPending = RegInit(VecInit(
      Seq.fill(vpuParams.hazardEntries)(false.B)))
    val faultPending = RegInit(false.B)
    val faultVaddr = Reg(UInt(64.W))
    val faultCause = Reg(UInt(2.W))

    val descriptorAdmissionBlocked = Wire(Bool())
    val incomingTag = io.descriptor.bits.commandTag
    val incomingTagInRange = incomingTag < vpuParams.hazardEntries.U
    val incomingTagFree = incomingTagInRange && !commandActive(incomingTag)
    descriptorQueue.io.enq.valid := io.descriptor.valid && incomingTagFree &&
      !descriptorAdmissionBlocked
    descriptorQueue.io.enq.bits := io.descriptor.bits
    io.descriptor.ready := descriptorQueue.io.enq.ready && incomingTagFree &&
      !descriptorAdmissionBlocked

    when(io.descriptor.valid) {
      assert(incomingTagInRange,
        "VPU reader descriptor command tag is outside the hazard table")
      val rows = Mux(io.descriptor.bits.rowCount === 0.U,
        1.U, io.descriptor.bits.rowCount)
      val footprintEnd = io.descriptor.bits.spadElement.pad(
        vpuParams.elementAddrBits + 1) +&
        (rows - 1.U) * vpuParams.vLen.U +
        io.descriptor.bits.elementCount
      val footprintStartBank = io.descriptor.bits.spadElement /
        vpuParams.elementsPerBank.U
      val footprintLastBank = (footprintEnd - 1.U) /
        vpuParams.elementsPerBank.U
      val hostRowBytes = io.descriptor.bits.elementCount *
        vpuParams.storageBytes.U
      val hostLastRowOffset = (rows - 1.U) *
        io.descriptor.bits.hostStrideBytes
      val hostEndWide = io.descriptor.bits.vaddr.pad(129) +
        hostLastRowOffset.pad(129) + hostRowBytes.pad(129)
      assert(rows <= vpuParams.dmaMaxRows.U,
        "VPU reader descriptor has too many 2-D rows")
      assert(io.descriptor.bits.elementCount <= vpuParams.vLen.U,
        "VPU reader row exceeds VLEN")
      assert(footprintEnd <= vpuParams.totalElements.U,
        "VPU reader 2-D footprint escapes VSRAM")
      assert(io.descriptor.bits.elementCount === 0.U ||
        footprintStartBank === footprintLastBank,
        "VPU reader 2-D footprint crosses a VSRAM bank")
      assert(rows === 1.U ||
        io.descriptor.bits.hostStrideBytes %
          vpuParams.storageBytes.U === 0.U,
        "VPU reader host row stride is not element aligned")
      assert(hostEndWide <= (BigInt(1) << 64).U(129.W),
        "VPU reader descriptor wraps the 64-bit virtual address space")
    }

    when(io.descriptor.fire) {
      assert(!commandActive(incomingTag),
        "VPU reader accepted a duplicate live command tag")
      commandActive(incomingTag) := true.B
      val rows = Mux(io.descriptor.bits.rowCount === 0.U,
        1.U, io.descriptor.bits.rowCount)
      commandElementsToGenerate(incomingTag) := rows *
        io.descriptor.bits.elementCount
    }

    // Only request generation is serialized by descriptor order.  All source
    // transactions and all returned writeback fragments remain independent.
    val requestActive = RegInit(false.B)
    val requestDescriptor = Reg(new VpuDmaDescriptor(vpuParams))
    val reserveVaddr = Reg(UInt(64.W))
    val reserveSpadRowBase = Reg(UInt(vpuParams.elementAddrBits.W))
    val reserveRow = RegInit(0.U(vpuParams.dmaRowCountBits.W))
    val reserveRows = Reg(UInt(vpuParams.dmaRowCountBits.W))
    val reserveOffset = RegInit(0.U(byteCountBits.W))
    val reserveRemaining = RegInit(0.U(byteCountBits.W))

    descriptorQueue.io.deq.ready := !requestActive && !faultPending
    when(descriptorQueue.io.deq.fire && !faultPending) {
      val totalBytes = descriptorQueue.io.deq.bits.elementCount *
        vpuParams.storageBytes.U
      requestDescriptor := descriptorQueue.io.deq.bits
      reserveVaddr := descriptorQueue.io.deq.bits.vaddr
      reserveSpadRowBase := descriptorQueue.io.deq.bits.spadElement
      reserveRow := 0.U
      reserveRows := Mux(descriptorQueue.io.deq.bits.rowCount === 0.U,
        1.U, descriptorQueue.io.deq.bits.rowCount)
      reserveOffset := 0.U
      reserveRemaining := totalBytes
      when(descriptorQueue.io.deq.bits.elementCount === 0.U) {
        zeroCompletionPending(
          descriptorQueue.io.deq.bits.commandTag) := true.B
        requestActive := false.B
      }.otherwise {
        requestActive := true.B
      }
    }

    // ----------------------------------------------------------------------
    // Source-ID transaction tracker
    // ----------------------------------------------------------------------
    val slotValid = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotTranslationPending = RegInit(VecInit(
      Seq.fill(nSources)(false.B)))
    val slotTranslated = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotIssued = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotCommandTag = Reg(Vec(
      nSources, UInt(vpuParams.dmaCommandTagBits.W)))
    val slotVaddr = Reg(Vec(nSources, UInt(64.W)))
    val slotPaddr = Reg(Vec(nSources, UInt(paddrBits.W)))
    val slotSpadElement = Reg(Vec(
      nSources, UInt(vpuParams.elementAddrBits.W)))
    val slotRequestBytes = Reg(Vec(
      nSources, UInt(requestBytesBits.W)))
    val slotLgSize = Reg(Vec(
      nSources, UInt(edge.bundle.sizeBits.W)))
    val slotUsefulBytes = Reg(Vec(
      nSources, UInt(requestBytesBits.W)))
    val slotUsefulShift = Reg(Vec(
      nSources, UInt(requestBytesBits.W)))
    val slotResponseBeat = RegInit(VecInit(
      Seq.fill(nSources)(0.U(responseBeatBits.W))))
    val slotData = Reg(Vec(nSources,
      Vec(maxBeatsPerRequest, UInt(vpuParams.dmaBusWidth.W))))

    // A D-last transfers the complete line from a source-owned assembly slot
    // into this independent response queue.  `pipe` permits a line to enter
    // in the same cycle that the unpacker removes the old head.
    val completedLines = Module(new Queue(
      new VpuDmaReadLine(vpuParams), nSources,
      pipe = true, flow = false))

    val sourceFree = !slotValid.asUInt.andR
    val freeSourceOH = PriorityEncoderOH(~slotValid.asUInt)
    val freeSource = OHToUInt(freeSourceOH)

    val (plannedBase, plannedBytes, plannedLgSize, plannedUseful,
      plannedShift) = VpuTLUtil.bestAlignedTransaction(
        reserveVaddr, reserveRemaining, beatBytes, requestMaxBytes)

    val translationSource =
      io.translationResult.bits.tag(sourceBits - 1, 0)
    val translationDirection =
      io.translationResult.bits.tag(translationTagBits - 1)
    val translationPageFault = io.translationResult.bits.pf.ld ||
      io.translationResult.bits.gf.ld
    val translationAccessFault = io.translationResult.bits.ae.ld
    val translationFault = translationPageFault || translationAccessFault
    val translationResultValid = io.translationResult.valid &&
      !translationDirection
    io.translationResult.ready := true.B
    val tlbResponse = translationResultValid && io.translationResult.ready
    val tlbFaultEvent = tlbResponse && translationFault

    val dSourceInRange = tl.d.bits.source < nSources.U
    val dSource = tl.d.bits.source(sourceBits - 1, 0)
    val dCanAccept = dSourceInRange && slotValid(dSource) &&
      slotIssued(dSource)
    val dResponseError = tl.d.bits.denied || tl.d.bits.corrupt
    val dFaultEvent = tl.d.valid && dCanAccept && dResponseError
    val dLast = edge.last(tl.d)
    // A successful final beat needs an independent completed-line entry.
    // Error/fault responses are discarded while draining and must never wait
    // for response-buffer space.
    val dNeedsLineEntry = dLast && !dResponseError && !faultPending &&
      !tlbFaultEvent
    tl.d.ready := dCanAccept &&
      (!dNeedsLineEntry || completedLines.io.enq.ready)

    val newFaultEvent = dFaultEvent || tlbFaultEvent
    descriptorAdmissionBlocked := faultPending || newFaultEvent
    io.fault := newFaultEvent

    // Keep an A-channel request irrevocable while allowing translated source
    // entries and the next TLB request to advance independently.
    val aValid = RegInit(false.B)
    val aSource = Reg(UInt(sourceBits.W))
    val issueEligible = VecInit((0 until nSources).map { source =>
      slotValid(source) && slotTranslated(source) && !slotIssued(source) &&
        !(aValid && aSource === source.U)
    })
    val issueEligibleValid = issueEligible.asUInt.orR
    val issueEligibleSource = PriorityEncoder(issueEligible)

    val (getLegal, getRequest) = edge.Get(
      aSource, slotPaddr(aSource), slotLgSize(aSource))
    tl.a.valid := aValid
    tl.a.bits := getRequest
    tl.b.ready := true.B
    tl.c.valid := false.B
    tl.c.bits := DontCare
    tl.e.valid := false.B
    tl.e.bits := DontCare

    when(tl.a.valid) {
      assert(getLegal, "VPU reader generated an unsupported TileLink Get")
      assert(slotValid(aSource) && slotTranslated(aSource) &&
        !slotIssued(aSource),
        "VPU reader exposed an unreserved or duplicate source")
    }
    when(tl.a.fire) {
      slotIssued(aSource) := true.B
      when(issueEligibleValid && !faultPending && !newFaultEvent) {
        aSource := issueEligibleSource
        aValid := true.B
      }.otherwise {
        aValid := false.B
      }
    }.elsewhen(!aValid && issueEligibleValid &&
        !faultPending && !newFaultEvent) {
      aSource := issueEligibleSource
      aValid := true.B
    }

    // Reserve one source metadata entry per accepted translation request.
    // Unlike the old batch FSM, this proceeds while A requests are firing.
    val reserveCandidate = requestActive && reserveRemaining =/= 0.U &&
      sourceFree && !faultPending && !newFaultEvent
    io.translationRequest.valid := reserveCandidate
    io.translationRequest.bits.tag := Cat(0.U(1.W), freeSource)
    io.translationRequest.bits.vaddr := plannedBase
    io.translationRequest.bits.lgSize := plannedLgSize
    io.translationRequest.bits.cmd := M_XRD
    io.translationRequest.bits.status := requestDescriptor.status

    when(io.translationRequest.fire) {
      assert(plannedUseful =/= 0.U && plannedUseful <= reserveRemaining,
        "VPU reader selected an empty or oversized transaction")
      assert((plannedBase & (plannedBytes - 1.U)) === 0.U,
        "VPU reader TileLink request is not naturally aligned")
      assert(plannedBase(pgIdxBits - 1, 0) +& plannedBytes <=
        (1 << pgIdxBits).U,
        "VPU reader TileLink request crosses a virtual-memory page")
      slotValid(freeSource) := true.B
      slotTranslationPending(freeSource) := true.B
      slotTranslated(freeSource) := false.B
      slotIssued(freeSource) := false.B
      slotCommandTag(freeSource) := requestDescriptor.commandTag
      slotVaddr(freeSource) := plannedBase
      slotSpadElement(freeSource) := reserveSpadRowBase +
        reserveOffset / vpuParams.storageBytes.U
      slotRequestBytes(freeSource) := plannedBytes
      slotLgSize(freeSource) := plannedLgSize
      slotUsefulBytes(freeSource) := plannedUseful
      slotUsefulShift(freeSource) := plannedShift
      slotResponseBeat(freeSource) := 0.U
      when(plannedUseful === reserveRemaining) {
        when(reserveRow + 1.U === reserveRows) {
          reserveRemaining := 0.U
          requestActive := false.B
        }.otherwise {
          reserveRow := reserveRow + 1.U
          reserveVaddr := requestDescriptor.vaddr +
            (reserveRow + 1.U) * requestDescriptor.hostStrideBytes
          reserveSpadRowBase := reserveSpadRowBase + vpuParams.vLen.U
          reserveOffset := 0.U
          reserveRemaining := requestDescriptor.elementCount *
            vpuParams.storageBytes.U
        }
      }.otherwise {
        reserveVaddr := reserveVaddr + plannedUseful
        reserveOffset := reserveOffset + plannedUseful
        reserveRemaining := reserveRemaining - plannedUseful
      }
    }

    when(tlbResponse) {
      assert(!translationDirection,
        "reader received a writer translation tag")
      assert(slotValid(translationSource) &&
        slotTranslationPending(translationSource),
        "reader received an unexpected translation result")
      slotTranslationPending(translationSource) := false.B
      when(tlbFaultEvent || faultPending || dFaultEvent) {
        slotValid(translationSource) := false.B
      }.otherwise {
        slotPaddr(translationSource) := io.translationResult.bits.paddr
        slotTranslated(translationSource) := true.B
      }
    }

    when(tl.d.valid) {
      assert(dCanAccept, "unexpected or duplicate VPU read response")
      assert(tl.d.bits.opcode === TLMessages.AccessAckData,
        "VPU read expected AccessAckData")
      assert(tl.d.bits.size === slotLgSize(dSource),
        "VPU read response changed transaction size")
    }

    completedLines.io.enq.valid := tl.d.valid && dCanAccept && dLast &&
      !dResponseError && !faultPending && !tlbFaultEvent
    completedLines.io.enq.bits.commandTag := slotCommandTag(dSource)
    completedLines.io.enq.bits.spadElement := slotSpadElement(dSource)
    completedLines.io.enq.bits.usefulBytes := slotUsefulBytes(dSource)
    completedLines.io.enq.bits.usefulShift := slotUsefulShift(dSource)
    for (beat <- 0 until maxBeatsPerRequest) {
      completedLines.io.enq.bits.data(beat) := Mux(
        slotResponseBeat(dSource) === beat.U,
        tl.d.bits.data, slotData(dSource)(beat))
    }
    when(completedLines.io.enq.fire) {
      assert(tl.d.fire && dLast,
        "VPU reader completed-line transfer did not coincide with D last")
    }

    when(tl.d.fire) {
      val beatIndex = slotResponseBeat(dSource)
      val expectedBeats = slotRequestBytes(dSource) / beatBytes.U
      assert(beatIndex < expectedBeats,
        "VPU read returned too many beats")
      assert(dLast === (beatIndex === expectedBeats - 1.U),
        "VPU read D last marker disagreed with request size")
      slotData(dSource)(beatIndex) := tl.d.bits.data
      when(dLast) {
        slotResponseBeat(dSource) := 0.U
        // Ownership of a successful line moved into completedLines above.
        // A faulting/drained line is discarded. In either case the TileLink
        // source ID is reusable immediately after this D-last handshake.
        slotValid(dSource) := false.B
      }.otherwise {
        slotResponseBeat(dSource) := beatIndex + 1.U
      }
    }

    // ----------------------------------------------------------------------
    // Independent response-line unpacker / VSRAM writeback stream
    // ----------------------------------------------------------------------
    val outValid = RegInit(false.B)
    val outBits = Reg(new VpuDmaReadBeat(vpuParams))
    io.data.valid := outValid
    io.data.bits := outBits
    val outCanReplace = !outValid || io.data.ready

    when(io.data.fire) {
      outValid := false.B
      when(outBits.last) {
        commandActive(outBits.commandTag) := false.B
      }
    }

    val commitActive = RegInit(false.B)
    val commitLine = Reg(new VpuDmaReadLine(vpuParams))
    val commitConsumed = RegInit(0.U(requestBytesBits.W))

    val zeroValid = zeroCompletionPending.asUInt.orR
    val zeroTag = PriorityEncoder(zeroCompletionPending)
    // Zero-length completions and normal fragments share one irrevocable
    // output register. Give an already active line priority so the two paths
    // can never overwrite one another in the same cycle.
    val generateZero = !faultPending && !newFaultEvent &&
      outCanReplace && !commitActive && zeroValid

    val commitTag = commitLine.commandTag
    val commitSourceRemaining = commitLine.usefulBytes - commitConsumed
    val commitSpadElement = commitLine.spadElement +
      commitConsumed / vpuParams.storageBytes.U
    val commitLaneElements = vpuParams.nLanes.U -
      (commitSpadElement % vpuParams.nLanes.U)
    val commitLaneBytes = commitLaneElements *
      vpuParams.storageBytes.U
    val commitBeatLimit = beatBytes.U(requestBytesBits.W)
    val commitBeforeLane = Mux(commitSourceRemaining < commitBeatLimit,
      commitSourceRemaining, commitBeatLimit)
    val commitTake = Mux(commitLaneBytes < commitBeforeLane,
      commitLaneBytes, commitBeforeLane)
    val commitElements = commitTake / vpuParams.storageBytes.U
    val commitLineData = commitLine.data.asUInt
    val commitLineBytes = commitLineData.asTypeOf(
      Vec(requestMaxBytes, UInt(8.W)))
    val commitByteBase = commitLine.usefulShift + commitConsumed
    val commitFragmentBytes = Wire(Vec(beatBytes, UInt(8.W)))
    for (byte <- 0 until beatBytes) {
      val index = commitByteBase + byte.U
      commitFragmentBytes(byte) := Mux(byte.U < commitTake,
        commitLineBytes(index), 0.U)
    }
    val commitFragment = Cat(commitFragmentBytes.reverse)
    val commitMaskWide =
      (1.U((vpuParams.dmaElementsPerBeat + 1).W) << commitElements) - 1.U
    val commitSourceDone = commitTake === commitSourceRemaining
    val commitEmit = commitActive && outCanReplace &&
      !faultPending && !newFaultEvent

    // On the final fragment, dequeue and latch the next completed line in the
    // same cycle. This removes the old one-cycle bubble between 64-byte lines.
    val canLoadCompletedLine = !generateZero &&
      (!commitActive || (commitEmit && commitSourceDone))
    completedLines.io.deq.ready := Mux(
      faultPending || newFaultEvent,
      true.B, canLoadCompletedLine)
    val loadCompletedLine = completedLines.io.deq.fire &&
      !faultPending && !newFaultEvent

    when(commitEmit) {
      assert(commitTake =/= 0.U &&
        commitTake % vpuParams.storageBytes.U === 0.U,
        "VPU reader writeback fragment made no element-aligned progress")
      assert(commandActive(commitTag),
        "VPU reader generated data for an inactive command")
      assert(commandElementsToGenerate(commitTag) >= commitElements,
        "VPU reader generated too many elements for a command")
      outValid := true.B
      outBits.data := commitFragment
      outBits.elementMask :=
        commitMaskWide(vpuParams.dmaElementsPerBeat - 1, 0)
      outBits.spadElement := commitSpadElement
      outBits.commandTag := commitTag
      outBits.last :=
        commandElementsToGenerate(commitTag) === commitElements
      outBits.error := false.B
      outBits.fault := 0.U.asTypeOf(outBits.fault)
      commandElementsToGenerate(commitTag) :=
        commandElementsToGenerate(commitTag) - commitElements
      commitConsumed := commitConsumed + commitTake
      when(commitSourceDone) {
        commitActive := false.B
        commitConsumed := 0.U
      }
    }

    when(loadCompletedLine) {
      commitActive := true.B
      commitLine := completedLines.io.deq.bits
      commitConsumed := 0.U
    }

    when(generateZero) {
      outValid := true.B
      outBits.data := 0.U
      outBits.elementMask := 0.U
      outBits.spadElement := 0.U
      outBits.commandTag := zeroTag
      outBits.last := true.B
      outBits.error := false.B
      outBits.fault := 0.U.asTypeOf(outBits.fault)
      zeroCompletionPending(zeroTag) := false.B
    }

    // ----------------------------------------------------------------------
    // Fault drain
    // ----------------------------------------------------------------------
    val eventVaddr = Mux(dFaultEvent, slotVaddr(dSource),
      slotVaddr(translationSource))
    val eventCause = Mux(dFaultEvent, VpuDmaFaultCause.Access.U,
      Mux(translationPageFault, VpuDmaFaultCause.Translation.U,
        VpuDmaFaultCause.Access.U))

    when(newFaultEvent) {
      when(!faultPending) {
        faultVaddr := eventVaddr
        faultCause := eventCause
      }
      faultPending := true.B
      requestActive := false.B
      commitActive := false.B
      for (tag <- 0 until vpuParams.hazardEntries) {
        zeroCompletionPending(tag) := false.B
      }
      // Requests already visible on A and translations already presented to
      // the TLB are irrevocable.  Issued requests must receive D; everything
      // else can be discarded immediately.
      for (source <- 0 until nSources) {
        val heldByA = aValid && aSource === source.U
        val heldByTlb = slotTranslationPending(source)
        val waitingForD = slotIssued(source)
        when(slotValid(source) && !heldByA && !heldByTlb && !waitingForD) {
          slotValid(source) := false.B
        }
      }
    }

    // Drop descriptors which were accepted into the request queue before the
    // first fault.  commandActive retains their tags until an error completion
    // is emitted, so VpuCore can retire every accepted load exactly once.
    when(faultPending) {
      descriptorQueue.io.deq.ready := true.B
      when(descriptorQueue.io.deq.fire) {
        requestActive := false.B
      }
    }

    val faultDrainQuiescent = faultPending && !requestActive &&
      !descriptorQueue.io.deq.valid && !slotValid.asUInt.orR &&
      !completedLines.io.deq.valid && !aValid && !commitActive && !outValid
    val faultCommandValid = commandActive.asUInt.orR
    val faultCommandTag = PriorityEncoder(commandActive)
    when(faultDrainQuiescent && faultCommandValid) {
      outValid := true.B
      outBits.data := 0.U
      outBits.elementMask := 0.U
      outBits.spadElement := 0.U
      outBits.commandTag := faultCommandTag
      outBits.last := true.B
      outBits.error := true.B
      outBits.fault.vaddr := faultVaddr
      outBits.fault.cause := faultCause
      outBits.fault.isWrite := false.B
    }

    when(faultPending && !commandActive.asUInt.orR &&
        !descriptorQueue.io.deq.valid && !slotValid.asUInt.orR &&
        !completedLines.io.deq.valid && !aValid && !commitActive &&
        !outValid) {
      faultPending := false.B
    }

    io.busy := requestActive || descriptorQueue.io.deq.valid ||
      slotValid.asUInt.orR || completedLines.io.deq.valid || aValid ||
      commitActive || outValid || commandActive.asUInt.orR || faultPending
  }
}
