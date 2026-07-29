package vpu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.diplomacy.{IdRange, LazyModule, LazyModuleImp}
import freechips.rocketchip.rocket.MStatus
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.tile.HasCoreParameters
import freechips.rocketchip.tilelink._

import gemmini.FrontendTLB

/** A compact TileLink DMA frontend for the standalone VPU.
  *
  * The VPU core deals in descriptor-sized, contiguous streams.  This wrapper
  * translates each naturally aligned power-of-two chunk and packs/unpacks it
  * into the configured VPU DMA beat.  Keeping the memory protocol outside of
  * VpuCore makes the SRAM/sequencer independently testable.
  *
  * Reader and writer each own dmaMaxInFlight TileLink source IDs. Responses
  * may return in any source order; the reader attaches a command tag and an
  * absolute VSRAM address before returning each completed line, while the
  * writer counts every acknowledgement before completion.
  */
private[vpu] object VpuTLUtil {
  def chunkBytes(currentAddress: UInt, bytesRemaining: UInt, minimum: Int, maximum: Int): UInt = {
    require(minimum > 0 && isPow2(minimum))
    require(maximum >= minimum && isPow2(maximum))
    val width = log2Ceil(maximum + 1)
    val selected = WireDefault(minimum.U(width.W))
    (minimum to maximum by minimum).filter(x => isPow2(x)).foreach { bytes =>
      val aligned = if (bytes == 1) true.B else currentAddress(log2Ceil(bytes) - 1, 0) === 0.U
      when(bytesRemaining >= bytes.U && aligned) {
        selected := bytes.U
      }
    }
    selected
  }

  def lgBytes(bytes: UInt, minimum: Int, maximum: Int): UInt = {
    val result = WireDefault(log2Ceil(minimum).U(log2Ceil(log2Ceil(maximum) + 1).W))
    (minimum to maximum by minimum).filter(x => isPow2(x)).foreach { n =>
      when(bytes === n.U) { result := log2Ceil(n).U }
    }
    result
  }

  /** Select the naturally aligned power-of-two TileLink transaction which
    * covers the most still-useful bytes starting at `currentAddress`.
    *
    * This is the same policy used by Gemmini's stream DMA: the request address
    * may be rounded down, while `shift` records the prefix bytes which are not
    * part of the architectural stream.  Ties prefer the smaller transaction,
    * which avoids needless over-fetch at a short tail.  In the default VPU
    * configuration the candidates are 16, 32 and 64 bytes.
    */
  def bestAlignedTransaction(
      currentAddress: UInt,
      bytesRemaining: UInt,
      minimum: Int,
      maximum: Int): (UInt, UInt, UInt, UInt, UInt) = {
    require(minimum > 0 && isPow2(minimum))
    require(maximum >= minimum && isPow2(maximum))
    val countBits = log2Ceil(maximum + 1)
    val lgBits = math.max(1, log2Ceil(log2Ceil(maximum) + 1))
    val candidates: Seq[(UInt, UInt, UInt, UInt, UInt)] =
      (minimum to maximum by minimum).
      filter(bytes => isPow2(bytes)).map { bytes =>
      val lg = log2Ceil(bytes)
      val base = Cat(currentAddress(currentAddress.getWidth - 1, lg),
        0.U(lg.W))
      val shift = currentAddress(lg - 1, 0).pad(countBits)
      val capacity = bytes.U(countBits.W) - shift
      val useful = Mux(bytesRemaining < capacity, bytesRemaining, capacity)
      (base, bytes.U(countBits.W), lg.U(lgBits.W), useful, shift)
    }
    candidates.reduceLeft { (older, newer) =>
      val chooseNewer = newer._4 > older._4
      (Mux(chooseNewer, newer._1, older._1),
        Mux(chooseNewer, newer._2, older._2),
        Mux(chooseNewer, newer._3, older._3),
        Mux(chooseNewer, newer._4, older._4),
        Mux(chooseNewer, newer._5, older._5))
    }
  }
}

private[vpu] class VpuTLReader(vpuParams: VpuParams)(implicit params: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "vpu-reader", sourceId = IdRange(0, vpuParams.dmaMaxInFlight))))))

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) with HasCoreParameters with MemoryOpConstants {
    val beatBytes = vpuParams.dmaBusWidth / 8
    val requestMaxBytes = vpuParams.dmaMaxBytes
    val maxBeatsPerRequest = requestMaxBytes / beatBytes
    require(requestMaxBytes >= beatBytes && requestMaxBytes % beatBytes == 0)
    require(isPow2(maxBeatsPerRequest))
    require(requestMaxBytes <= (1 << pgIdxBits),
      "VPU DMA transactions must fit within one virtual-memory page")

    val nSources = vpuParams.dmaMaxInFlight
    val sourceBits = math.max(1, log2Ceil(nSources))
    val translationTagBits = sourceBits + 1

    val io = IO(new Bundle {
      val descriptor = Flipped(Decoupled(new VpuDmaDescriptor(vpuParams)))
      val data = Decoupled(new VpuDmaReadBeat(vpuParams))
      val translationRequest = Decoupled(
        new VpuTLBTaggedRequest(translationTagBits, requestMaxBytes)(p))
      val translationResult = Flipped(Decoupled(
        new VpuTLBTaggedResult(translationTagBits)(p)))
      val busy = Output(Bool())
      val fault = Output(Bool())
    })

    val (tl, edge) = node.out.head
    val byteCountBits = log2Ceil(vpuParams.vLen * vpuParams.storageBytes + 1)
    val packCountBits = log2Ceil(beatBytes + 1)
    val requestBytesBits = log2Ceil(requestMaxBytes + 1)
    val responseBeatBits = math.max(1, log2Ceil(maxBeatsPerRequest + 1))

    val descriptorActive = RegInit(false.B)
    val descriptorBytes = Reg(UInt(byteCountBits.W))
    val descriptorStatus = Reg(new MStatus)
    val descriptorTag = Reg(UInt(vpuParams.dmaCommandTagBits.W))
    val reserveVaddr = Reg(UInt(64.W))
    val reserveOffset = RegInit(0.U(byteCountBits.W))
    val reserveRemaining = RegInit(0.U(byteCountBits.W))
    val commitOffset = RegInit(0.U(byteCountBits.W))
    val packSpadBase = Reg(UInt(vpuParams.elementAddrBits.W))
    val packData = RegInit(0.U(vpuParams.dmaBusWidth.W))
    val packBytes = RegInit(0.U(packCountBits.W))

    // Each source owns a complete maximum-sized response buffer. A 64-byte
    // Get therefore accepts four 128-bit D beats before it becomes commit
    // eligible. Sources remain allocated through ordered commit, forming a
    // small line-granularity reorder buffer.
    val slotValid = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotTranslationPending = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotTranslated = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotIssued = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotReceived = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotVaddr = Reg(Vec(nSources, UInt(64.W)))
    val slotPaddr = Reg(Vec(nSources, UInt(paddrBits.W)))
    val slotDescriptorOffset = Reg(Vec(nSources, UInt(byteCountBits.W)))
    val slotRequestBytes = Reg(Vec(nSources, UInt(requestBytesBits.W)))
    val slotLgSize = Reg(Vec(nSources, UInt(edge.bundle.sizeBits.W)))
    val slotUsefulBytes = Reg(Vec(nSources, UInt(requestBytesBits.W)))
    val slotUsefulShift = Reg(Vec(nSources, UInt(requestBytesBits.W)))
    val slotResponseBeat = RegInit(VecInit(Seq.fill(nSources)(0.U(responseBeatBits.W))))
    val slotData = Reg(Vec(nSources,
      Vec(maxBeatsPerRequest, UInt(vpuParams.dmaBusWidth.W))))
    val slotError = RegInit(VecInit(Seq.fill(nSources)(false.B)))

    val sourceFree = !slotValid.asUInt.andR
    val freeSourceOH = PriorityEncoderOH(~slotValid.asUInt)
    val freeSource = OHToUInt(freeSourceOH)

    val issueArmed = RegInit(false.B)
    val aValid = RegInit(false.B)
    val aSource = Reg(UInt(sourceBits.W))

    val faultPending = RegInit(false.B)
    val faultOffset = Reg(UInt(byteCountBits.W))
    val faultVaddr = Reg(UInt(64.W))
    val faultCause = Reg(UInt(2.W))

    val (plannedBase, plannedBytes, plannedLgSize, plannedUseful,
      plannedShift) = VpuTLUtil.bestAlignedTransaction(
        reserveVaddr, reserveRemaining, beatBytes, requestMaxBytes)

    val outValid = RegInit(false.B)
    val outBits = Reg(new VpuDmaReadBeat(vpuParams))
    io.data.valid := outValid
    io.data.bits := outBits
    val outCanReplace = !outValid || io.data.ready
    when(io.data.fire) {
      outValid := false.B
      when(outBits.last) {
        descriptorActive := false.B
        faultPending := false.B
      }
    }

    io.descriptor.ready := !descriptorActive && !outValid &&
      !slotValid.asUInt.orR && !aValid
    io.busy := descriptorActive || outValid || slotValid.asUInt.orR ||
      aValid

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
      slotIssued(dSource) && !slotReceived(dSource)
    tl.d.ready := dCanAccept
    val dResponseError = tl.d.bits.denied || tl.d.bits.corrupt
    val dFaultEvent = tl.d.valid && dCanAccept && dResponseError
    val dLast = edge.last(tl.d)

    val newFaultEvent = dFaultEvent || tlbFaultEvent
    io.fault := newFaultEvent
    val dEventOffset = slotDescriptorOffset(dSource)
    val tlbEventOffset = slotDescriptorOffset(translationSource)
    val chooseD = dFaultEvent && (!tlbFaultEvent || dEventOffset <= tlbEventOffset)
    val eventOffset = Mux(chooseD, dEventOffset, tlbEventOffset)
    val eventVaddr = Mux(chooseD, slotVaddr(dSource),
      slotVaddr(translationSource))
    val eventCause = Mux(chooseD, VpuDmaFaultCause.Access.U,
      Mux(translationPageFault, VpuDmaFaultCause.Translation.U,
        VpuDmaFaultCause.Access.U))

    // Translated requests are held independently from the translation FSM.
    // Once a batch has filled all free source IDs (or reached descriptor end),
    // the A holding register drains it at one Get per ready cycle.
    val issueEligible = VecInit((0 until nSources).map { source =>
      slotValid(source) && slotTranslated(source) && !slotIssued(source) &&
        !(aValid && aSource === source.U)
    })
    val issueEligibleValid = issueEligible.asUInt.orR
    val issueEligibleSource = PriorityEncoder(issueEligible)
    val untranslatedAllocated = VecInit((0 until nSources).map { source =>
      slotValid(source) && !slotTranslated(source) && !slotIssued(source)
    }).asUInt.orR
    val translatedUnissued = VecInit((0 until nSources).map { source =>
      slotValid(source) && slotTranslated(source) && !slotIssued(source)
    }).asUInt.orR

    val aLgSize = slotLgSize(aSource)
    val (getLegal, getRequest) = edge.Get(aSource,
      slotPaddr(aSource), aLgSize)
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
    }.elsewhen(!aValid && issueArmed && issueEligibleValid &&
        !faultPending && !newFaultEvent) {
      aSource := issueEligibleSource
      aValid := true.B
    }

    when(tl.d.valid) {
      assert(dCanAccept, "unexpected or duplicate VPU read response")
      assert(tl.d.bits.opcode === TLMessages.AccessAckData,
        "VPU read expected AccessAckData")
      assert(tl.d.bits.size === slotLgSize(dSource),
        "VPU read response changed transaction size")
    }
    when(tl.d.fire) {
      val beatIndex = slotResponseBeat(dSource)
      val expectedBeats = slotRequestBytes(dSource) / beatBytes.U
      assert(beatIndex < expectedBeats,
        "VPU read returned too many beats")
      assert(dLast === (beatIndex === expectedBeats - 1.U),
        "VPU read D last marker disagreed with request size")
      slotData(dSource)(beatIndex) := tl.d.bits.data
      slotError(dSource) := slotError(dSource) || dResponseError
      when(dLast) {
        slotReceived(dSource) := true.B
        slotResponseBeat(dSource) := 0.U
      }.otherwise {
        slotResponseBeat(dSource) := beatIndex + 1.U
      }
    }

    when(io.descriptor.fire) {
      val totalBytes = io.descriptor.bits.elementCount *
        vpuParams.storageBytes.U
      descriptorActive := true.B
      descriptorBytes := totalBytes
      descriptorStatus := io.descriptor.bits.status
      descriptorTag := io.descriptor.bits.commandTag
      reserveVaddr := io.descriptor.bits.vaddr
      reserveOffset := 0.U
      reserveRemaining := totalBytes
      commitOffset := 0.U
      packSpadBase := io.descriptor.bits.spadElement
      packData := 0.U
      packBytes := 0.U
      issueArmed := false.B
      aValid := false.B
      faultPending := false.B
      when(io.descriptor.bits.elementCount === 0.U) {
        outValid := true.B
        outBits.data := 0.U
        outBits.elementMask := 0.U
        outBits.spadElement := io.descriptor.bits.spadElement
        outBits.commandTag := io.descriptor.bits.commandTag
        outBits.last := true.B
        outBits.error := false.B
        outBits.fault := 0.U.asTypeOf(outBits.fault)
      }
    }

    val reserveCandidate = descriptorActive && !issueArmed && !aValid &&
      reserveRemaining =/= 0.U && sourceFree &&
      !faultPending && !newFaultEvent
    io.translationRequest.valid := reserveCandidate
    io.translationRequest.bits.tag := Cat(0.U(1.W), freeSource)
    io.translationRequest.bits.vaddr := plannedBase
    io.translationRequest.bits.lgSize := plannedLgSize
    io.translationRequest.bits.cmd := M_XRD
    io.translationRequest.bits.status := descriptorStatus
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
      slotReceived(freeSource) := false.B
      slotVaddr(freeSource) := plannedBase
      slotDescriptorOffset(freeSource) := reserveOffset
      slotRequestBytes(freeSource) := plannedBytes
      slotLgSize(freeSource) := plannedLgSize
      slotUsefulBytes(freeSource) := plannedUseful
      slotUsefulShift(freeSource) := plannedShift
      slotResponseBeat(freeSource) := 0.U
      slotError(freeSource) := false.B
      reserveVaddr := reserveVaddr + plannedUseful
      reserveOffset := reserveOffset + plannedUseful
      reserveRemaining := reserveRemaining - plannedUseful
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

    val batchBoundary = reserveRemaining === 0.U || !sourceFree
    when(descriptorActive && !issueArmed && !aValid &&
        batchBoundary && translatedUnissued &&
        !untranslatedAllocated && !faultPending && !newFaultEvent) {
      issueArmed := true.B
    }
    when(issueArmed && !aValid && !translatedUnissued) {
      issueArmed := false.B
    }

    when(newFaultEvent) {
      when(!faultPending || eventOffset < faultOffset) {
        faultOffset := eventOffset
        faultVaddr := eventVaddr
        faultCause := eventCause
      }
      faultPending := true.B
      issueArmed := false.B
      // Requests already visible on A are irrevocable. A translation already
      // presented to FrontendTLB is also retained until its terminal response.
      for (source <- 0 until nSources) {
        val heldByA = aValid && aSource === source.U
        val heldByTlb = slotTranslationPending(source)
        when(slotValid(source) && !slotIssued(source) &&
            !heldByA && !heldByTlb) {
          slotValid(source) := false.B
        }
      }
    }

    // Commit completed transactions in descriptor order, repacking an
    // aligned-down line (including its ignored prefix) into 128-bit VSRAM
    // stream beats.
    val commitActive = RegInit(false.B)
    val commitSourceReg = Reg(UInt(sourceBits.W))
    val commitConsumed = RegInit(0.U(requestBytesBits.W))
    val commitMatches = VecInit((0 until nSources).map { source =>
      slotValid(source) && slotReceived(source) && !slotError(source) &&
        slotDescriptorOffset(source) === commitOffset &&
        (!faultPending || slotDescriptorOffset(source) < faultOffset)
    })
    val commitMatchValid = commitMatches.asUInt.orR
    val commitMatchSource = PriorityEncoder(commitMatches)

    when(!commitActive && outCanReplace && commitMatchValid) {
      commitActive := true.B
      commitSourceReg := commitMatchSource
      commitConsumed := 0.U
    }

    val commitSourceRemaining = slotUsefulBytes(commitSourceReg) - commitConsumed
    val commitSpace = beatBytes.U - packBytes
    val commitTake = Mux(commitSourceRemaining < commitSpace,
      commitSourceRemaining, commitSpace)
    val commitLine = slotData(commitSourceReg).asUInt
    val commitLineBytes = commitLine.asTypeOf(
      Vec(requestMaxBytes, UInt(8.W)))
    val commitByteBase = slotUsefulShift(commitSourceReg) + commitConsumed
    val commitFragmentBytes = Wire(Vec(beatBytes, UInt(8.W)))
    for (byte <- 0 until beatBytes) {
      commitFragmentBytes(byte) := Mux(byte.U < commitTake,
        commitLineBytes(commitByteBase + byte.U), 0.U)
    }
    val commitFragment = Cat(commitFragmentBytes.reverse)
    val commitMerged = (packData |
      (commitFragment << (packBytes << 3)))(vpuParams.dmaBusWidth - 1, 0)
    val commitMergedBytes = packBytes +& commitTake
    val nextCommitOffset = commitOffset + commitTake
    val commitDescriptorDone = nextCommitOffset === descriptorBytes
    val commitBeatDone = commitMergedBytes === beatBytes.U ||
      commitDescriptorDone
    val commitSourceDone = commitTake === commitSourceRemaining
    val commitValidElements = commitMergedBytes / vpuParams.storageBytes.U
    val commitMaskWide =
      (1.U((vpuParams.dmaElementsPerBeat + 1).W) <<
        commitValidElements) - 1.U

    when(commitActive && outCanReplace) {
      assert(commitTake =/= 0.U,
        "VPU reader commit engine made no progress")
      commitOffset := nextCommitOffset
      commitConsumed := commitConsumed + commitTake
      when(commitSourceDone) {
        slotValid(commitSourceReg) := false.B
        commitActive := false.B
        commitConsumed := 0.U
      }
      when(commitBeatDone) {
        outValid := true.B
        outBits.data := commitMerged
        outBits.elementMask :=
          commitMaskWide(vpuParams.dmaElementsPerBeat - 1, 0)
        outBits.spadElement := packSpadBase
        outBits.commandTag := descriptorTag
        outBits.last := commitDescriptorDone
        outBits.error := false.B
        outBits.fault := 0.U.asTypeOf(outBits.fault)
        packData := 0.U
        packBytes := 0.U
        packSpadBase := packSpadBase + commitValidElements
      }.otherwise {
        packData := commitMerged
        packBytes := commitMergedBytes
      }
    }

    val pendingResponses = VecInit((0 until nSources).map { source =>
      slotValid(source) && slotIssued(source) && !slotReceived(source)
    }).asUInt.orR
    val pendingTranslations = slotTranslationPending.asUInt.orR
    val priorBytesCommitted = faultPending && commitOffset >= faultOffset
    val emitFault = descriptorActive && faultPending &&
      priorBytesCommitted && !pendingResponses && !outValid && !commitActive &&
      !pendingTranslations && !aValid
    when(emitFault) {
      outValid := true.B
      outBits.data := 0.U
      outBits.elementMask := 0.U
      outBits.spadElement := packSpadBase
      outBits.commandTag := descriptorTag
      outBits.last := true.B
      outBits.error := true.B
      outBits.fault.vaddr := faultVaddr
      outBits.fault.cause := faultCause
      outBits.fault.isWrite := false.B
      packData := 0.U
      packBytes := 0.U
      for (source <- 0 until nSources) {
        slotValid(source) := false.B
      }
    }
  }
}

/** Cache-line-capable writer.  VSRAM beats are assembled into one source-owned
  * transaction buffer, translated with Gemmini's byte-sized TLB request, and
  * emitted with the real lgSize as an uninterrupted multi-beat PutPartial.
  */
private[vpu] class VpuTLWriter(vpuParams: VpuParams)(implicit params: Parameters)
    extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "vpu-writer", sourceId = IdRange(0, vpuParams.dmaMaxInFlight))))))

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) with HasCoreParameters with MemoryOpConstants {
    val beatBytes = vpuParams.dmaBusWidth / 8
    val requestMaxBytes = vpuParams.dmaMaxBytes
    val maxBeatsPerRequest = requestMaxBytes / beatBytes
    require(requestMaxBytes >= beatBytes && requestMaxBytes % beatBytes == 0)
    require(isPow2(maxBeatsPerRequest))
    require(requestMaxBytes <= (1 << pgIdxBits),
      "VPU DMA transactions must fit within one virtual-memory page")

    val nSources = vpuParams.dmaMaxInFlight
    val sourceBits = math.max(1, log2Ceil(nSources))
    val translationTagBits = sourceBits + 1

    val io = IO(new Bundle {
      val descriptor = Flipped(Decoupled(new VpuDmaDescriptor(vpuParams)))
      val data = Flipped(Decoupled(new VpuDmaWriteBeat(vpuParams)))
      val completion = Decoupled(new VpuDmaWriteCompletion(vpuParams))
      val translationRequest = Decoupled(
        new VpuTLBTaggedRequest(translationTagBits, requestMaxBytes)(p))
      val translationResult = Flipped(Decoupled(
        new VpuTLBTaggedResult(translationTagBits)(p)))
      val busy = Output(Bool())
      val fault = Output(Bool())
    })

    val (tl, edge) = node.out.head
    val byteCountBits = log2Ceil(vpuParams.vLen * vpuParams.storageBytes + 1)
    val beatByteCountBits = log2Ceil(beatBytes + 1)
    val requestBytesBits = log2Ceil(requestMaxBytes + 1)
    val requestBeatBits = math.max(1, log2Ceil(maxBeatsPerRequest))

    val descriptorActive = RegInit(false.B)
    val descriptorSpadBase = Reg(UInt(vpuParams.elementAddrBits.W))
    val descriptorBytes = Reg(UInt(byteCountBits.W))
    val descriptorStatus = Reg(new MStatus)
    val fillVaddr = Reg(UInt(64.W))
    val fillOffset = RegInit(0.U(byteCountBits.W))
    val fillRemaining = RegInit(0.U(byteCountBits.W))
    val acceptedBytes = RegInit(0.U(byteCountBits.W))
    val streamLastSeen = RegInit(false.B)

    val slotValid = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotTranslationPending = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotPayloadReady = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotTranslated = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotIssued = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotVaddr = Reg(Vec(nSources, UInt(64.W)))
    val slotPaddr = Reg(Vec(nSources, UInt(paddrBits.W)))
    val slotDescriptorOffset = Reg(Vec(nSources, UInt(byteCountBits.W)))
    val slotRequestBytes = Reg(Vec(nSources, UInt(requestBytesBits.W)))
    val slotLgSize = Reg(Vec(nSources, UInt(edge.bundle.sizeBits.W)))
    val slotUsefulBytes = Reg(Vec(nSources, UInt(requestBytesBits.W)))
    val slotUsefulShift = Reg(Vec(nSources, UInt(requestBytesBits.W)))
    val slotPayload = Reg(Vec(nSources,
      Vec(requestMaxBytes, UInt(8.W))))

    val sourceFree = !slotValid.asUInt.andR
    val freeSourceOH = PriorityEncoderOH(~slotValid.asUInt)
    val freeSource = OHToUInt(freeSourceOH)
    val allSourcesIdle = !slotValid.asUInt.orR

    val assemblyActive = RegInit(false.B)
    val assemblySource = Reg(UInt(sourceBits.W))
    val assemblyRemaining = Reg(UInt(requestBytesBits.W))
    val assemblyPosition = Reg(UInt(requestBytesBits.W))

    // A one-beat elastic byte buffer carries a partial tail into the next
    // aligned-down transaction without asking VpuCore to replay data.
    val inputBeatValid = RegInit(false.B)
    val inputBeatData = Reg(UInt(vpuParams.dmaBusWidth.W))
    val inputBeatRemaining = Reg(UInt(beatByteCountBits.W))
    val inputBeatOffset = Reg(UInt(beatByteCountBits.W))

    val aActive = RegInit(false.B)
    val aSource = Reg(UInt(sourceBits.W))
    val aBeat = RegInit(0.U(requestBeatBits.W))

    val faultPending = RegInit(false.B)
    val faultOffset = Reg(UInt(byteCountBits.W))
    val faultVaddr = Reg(UInt(64.W))
    val faultCause = Reg(UInt(2.W))

    val completionValid = RegInit(false.B)
    val completionError = RegInit(false.B)
    val completionFault = Reg(new VpuDmaFaultInfo)
    io.completion.valid := completionValid
    io.completion.bits.commandTag := 0.U
    io.completion.bits.error := completionError
    io.completion.bits.fault := completionFault

    io.descriptor.ready := !descriptorActive && !completionValid &&
      allSourcesIdle && !aActive &&
      !assemblyActive && !inputBeatValid
    io.busy := descriptorActive || completionValid || inputBeatValid ||
      assemblyActive || !allSourcesIdle || aActive

    val translationSource =
      io.translationResult.bits.tag(sourceBits - 1, 0)
    val translationDirection =
      io.translationResult.bits.tag(translationTagBits - 1)
    val translationPageFault = io.translationResult.bits.pf.st ||
      io.translationResult.bits.gf.st
    val translationAccessFault = io.translationResult.bits.ae.st
    val translationFault = translationPageFault || translationAccessFault
    val translationResultValid = io.translationResult.valid &&
      translationDirection
    io.translationResult.ready := true.B
    val tlbResponse = translationResultValid && io.translationResult.ready
    val tlbFaultEvent = tlbResponse && translationFault

    val dSourceInRange = tl.d.bits.source < nSources.U
    val dSource = tl.d.bits.source(sourceBits - 1, 0)
    val dCanAccept = dSourceInRange && slotValid(dSource) &&
      slotIssued(dSource)
    tl.d.ready := dCanAccept
    val dResponseError = tl.d.bits.denied || tl.d.bits.corrupt
    val dFaultEvent = tl.d.valid && dCanAccept && dResponseError

    val issueEligible = VecInit((0 until nSources).map { source =>
      slotValid(source) && slotPayloadReady(source) &&
        slotTranslated(source) && !slotIssued(source) &&
        !(aActive && aSource === source.U)
    })
    val issueEligibleValid = issueEligible.asUInt.orR
    val issueEligibleSource = PriorityEncoder(issueEligible)

    val aTransactionBeatBase = aBeat * beatBytes.U
    val aDataBytes = Wire(Vec(beatBytes, UInt(8.W)))
    val aMask = Wire(Vec(beatBytes, Bool()))
    for (byte <- 0 until beatBytes) {
      val transactionByte = aTransactionBeatBase + byte.U
      aDataBytes(byte) := slotPayload(aSource)(transactionByte)
      aMask(byte) := transactionByte >= slotUsefulShift(aSource) &&
        transactionByte < slotUsefulShift(aSource) +
          slotUsefulBytes(aSource)
    }
    val aData = Cat(aDataBytes.reverse)
    val aExpectedBeats = slotRequestBytes(aSource) / beatBytes.U
    val aLastBeat = aBeat === aExpectedBeats - 1.U
    val (putLegal, putRequest) = edge.Put(aSource,
      slotPaddr(aSource), slotLgSize(aSource), aData, aMask.asUInt)
    tl.a.valid := aActive
    tl.a.bits := putRequest
    tl.b.ready := true.B
    tl.c.valid := false.B
    tl.c.bits := DontCare
    tl.e.valid := false.B
    tl.e.bits := DontCare

    when(tl.a.valid) {
      assert(putLegal, "VPU writer generated an unsupported TileLink Put")
      assert(slotValid(aSource) && slotTranslated(aSource) &&
        slotPayloadReady(aSource) && !slotIssued(aSource),
        "VPU writer exposed an unreserved or duplicate source")
      assert(edge.last(tl.a) === aLastBeat,
        "VPU writer A last marker disagreed with request size")
    }
    when(tl.d.valid) {
      assert(dCanAccept, "unexpected or duplicate VPU write response")
      assert(tl.d.bits.opcode === TLMessages.AccessAck,
        "VPU write expected AccessAck")
      assert(tl.d.bits.size === slotLgSize(dSource),
        "VPU write acknowledgement changed transaction size")
    }

    val inputValidElements = PopCount(io.data.bits.elementMask)
    val inputValidBytes = inputValidElements * vpuParams.storageBytes.U
    val expectedMaskWide = (1.U((vpuParams.dmaElementsPerBeat + 1).W) <<
      inputValidElements) - 1.U
    val inputMaskContiguous = io.data.bits.elementMask ===
      expectedMaskWide(vpuParams.dmaElementsPerBeat - 1, 0)
    val expectedInputSpad = descriptorSpadBase +
      (acceptedBytes / vpuParams.storageBytes.U)

    val assemblyCanConsume = assemblyActive && inputBeatValid
    val assemblyTake = Mux(inputBeatRemaining < assemblyRemaining,
      inputBeatRemaining, assemblyRemaining)
    val inputConsumedThisCycle = assemblyCanConsume &&
      assemblyTake === inputBeatRemaining
    val normalDataReady = descriptorActive && !completionValid &&
      !faultPending && acceptedBytes < descriptorBytes &&
      (!inputBeatValid || inputConsumedThisCycle) && !dFaultEvent
    val drainDataReady = descriptorActive && faultPending &&
      !streamLastSeen
    io.data.ready := normalDataReady || drainDataReady

    val nextAcceptedBytes = acceptedBytes + inputValidBytes
    val malformedInput = io.data.fire && normalDataReady &&
      (inputValidElements === 0.U || !inputMaskContiguous ||
        io.data.bits.spadElement =/= expectedInputSpad ||
        nextAcceptedBytes > descriptorBytes ||
        io.data.bits.last =/= (nextAcceptedBytes === descriptorBytes))
    val protocolFaultEvent = malformedInput
    val newFaultEvent = dFaultEvent || tlbFaultEvent || protocolFaultEvent
    io.fault := newFaultEvent

    val noEventOffset = Fill(byteCountBits, 1.U(1.W))
    val dEventOffset = Mux(dFaultEvent, slotDescriptorOffset(dSource),
      noEventOffset)
    val tlbEventOffset = Mux(tlbFaultEvent,
      slotDescriptorOffset(translationSource), noEventOffset)
    val protocolEventOffset = Mux(protocolFaultEvent, acceptedBytes,
      noEventOffset)
    val earliestTranslation = Mux(tlbEventOffset < protocolEventOffset,
      tlbEventOffset, protocolEventOffset)
    val eventOffset = Mux(dEventOffset <= earliestTranslation,
      dEventOffset, earliestTranslation)
    val eventIsD = dFaultEvent && dEventOffset === eventOffset
    val eventIsTlb = !eventIsD && tlbFaultEvent &&
      tlbEventOffset === eventOffset
    val eventVaddr = Mux(eventIsD, slotVaddr(dSource),
      Mux(eventIsTlb, slotVaddr(translationSource), fillVaddr))
    val eventCause = Mux(eventIsD, VpuDmaFaultCause.Access.U,
      Mux(eventIsTlb,
        Mux(translationPageFault, VpuDmaFaultCause.Translation.U,
          VpuDmaFaultCause.Access.U),
        VpuDmaFaultCause.Protocol.U))

    val (plannedBase, plannedBytes, plannedLgSize, plannedUseful,
      plannedShift) = VpuTLUtil.bestAlignedTransaction(
        fillVaddr, fillRemaining, beatBytes, requestMaxBytes)

    when(io.descriptor.fire) {
      val totalBytes = io.descriptor.bits.elementCount *
        vpuParams.storageBytes.U
      descriptorActive := true.B
      descriptorSpadBase := io.descriptor.bits.spadElement
      descriptorBytes := totalBytes
      descriptorStatus := io.descriptor.bits.status
      fillVaddr := io.descriptor.bits.vaddr
      fillOffset := 0.U
      fillRemaining := totalBytes
      acceptedBytes := 0.U
      inputBeatValid := false.B
      assemblyActive := false.B
      streamLastSeen := false.B
      faultPending := false.B
      aActive := false.B
      completionError := false.B
      completionFault := 0.U.asTypeOf(completionFault)
      when(io.descriptor.bits.elementCount === 0.U) {
        completionValid := true.B
      }
    }

    when(io.data.fire) {
      when(io.data.bits.last) { streamLastSeen := true.B }
      when(normalDataReady && !malformedInput) {
        inputBeatValid := true.B
        inputBeatData := io.data.bits.data
        inputBeatRemaining := inputValidBytes
        inputBeatOffset := 0.U
        acceptedBytes := nextAcceptedBytes
      }
    }

    val reserveAssembly = descriptorActive && !assemblyActive &&
      fillRemaining =/= 0.U && sourceFree && !faultPending &&
      !newFaultEvent
    when(reserveAssembly) {
      assert(plannedUseful =/= 0.U && plannedUseful <= fillRemaining,
        "VPU writer selected an empty or oversized transaction")
      assert((plannedBase & (plannedBytes - 1.U)) === 0.U,
        "VPU writer TileLink request is not naturally aligned")
      assert(plannedBase(pgIdxBits - 1, 0) +& plannedBytes <=
        (1 << pgIdxBits).U,
        "VPU writer TileLink request crosses a virtual-memory page")
      assemblyActive := true.B
      assemblySource := freeSource
      assemblyRemaining := plannedUseful
      assemblyPosition := plannedShift
      slotValid(freeSource) := true.B
      slotPayloadReady(freeSource) := false.B
      slotTranslated(freeSource) := false.B
      slotIssued(freeSource) := false.B
      slotVaddr(freeSource) := plannedBase
      slotDescriptorOffset(freeSource) := fillOffset
      slotRequestBytes(freeSource) := plannedBytes
      slotLgSize(freeSource) := plannedLgSize
      slotUsefulBytes(freeSource) := plannedUseful
      slotUsefulShift(freeSource) := plannedShift
      fillVaddr := fillVaddr + plannedUseful
      fillOffset := fillOffset + plannedUseful
      fillRemaining := fillRemaining - plannedUseful
    }

    val inputBeatBytes = inputBeatData.asTypeOf(Vec(beatBytes, UInt(8.W)))
    val inputBeatRefill = io.data.fire && normalDataReady && !malformedInput
    when(assemblyCanConsume) {
      assert(assemblyTake =/= 0.U,
        "VPU writer assembly engine made no progress")
      for (byte <- 0 until beatBytes) {
        when(byte.U < assemblyTake) {
          slotPayload(assemblySource)(assemblyPosition + byte.U) :=
            inputBeatBytes(inputBeatOffset + byte.U)
        }
      }
      assemblyRemaining := assemblyRemaining - assemblyTake
      assemblyPosition := assemblyPosition + assemblyTake
      when(!inputBeatRefill) {
        inputBeatRemaining := inputBeatRemaining - assemblyTake
        inputBeatOffset := inputBeatOffset + assemblyTake
        when(assemblyTake === inputBeatRemaining) {
          inputBeatValid := false.B
          inputBeatRemaining := 0.U
        }
      }
      when(assemblyTake === assemblyRemaining) {
        slotPayloadReady(assemblySource) := true.B
        assemblyActive := false.B
      }
    }

    val translationEligible = VecInit((0 until nSources).map { source =>
      slotValid(source) && slotPayloadReady(source) &&
        !slotTranslationPending(source) && !slotTranslated(source) &&
        !slotIssued(source)
    })
    val translationRequestSource = PriorityEncoder(translationEligible)
    io.translationRequest.valid := translationEligible.asUInt.orR &&
      !faultPending && !newFaultEvent
    io.translationRequest.bits.tag :=
      Cat(1.U(1.W), translationRequestSource)
    io.translationRequest.bits.vaddr := slotVaddr(translationRequestSource)
    io.translationRequest.bits.lgSize := slotLgSize(translationRequestSource)
    io.translationRequest.bits.cmd := M_XWR
    io.translationRequest.bits.status := descriptorStatus
    when(io.translationRequest.fire) {
      slotTranslationPending(translationRequestSource) := true.B
    }

    when(tlbResponse) {
      assert(translationDirection,
        "writer received a reader translation tag")
      assert(slotValid(translationSource) &&
        slotTranslationPending(translationSource),
        "writer received an unexpected translation result")
      slotTranslationPending(translationSource) := false.B
      when(tlbFaultEvent || faultPending || dFaultEvent) {
        slotValid(translationSource) := false.B
      }.otherwise {
        slotPaddr(translationSource) := io.translationResult.bits.paddr
        slotTranslated(translationSource) := true.B
      }
    }

    when(!aActive && issueEligibleValid && !faultPending && !newFaultEvent) {
      aActive := true.B
      aSource := issueEligibleSource
      aBeat := 0.U
    }
    when(tl.a.fire) {
      when(aLastBeat) {
        slotIssued(aSource) := true.B
        when(issueEligibleValid && !faultPending && !newFaultEvent) {
          aSource := issueEligibleSource
          aBeat := 0.U
          aActive := true.B
        }.otherwise {
          aActive := false.B
        }
      }.otherwise {
        aBeat := aBeat + 1.U
      }
    }

    when(tl.d.fire) {
      slotValid(dSource) := false.B
      slotIssued(dSource) := false.B
    }

    when(newFaultEvent) {
      when(!faultPending || eventOffset < faultOffset) {
        faultOffset := eventOffset
        faultVaddr := eventVaddr
        faultCause := eventCause
      }
      faultPending := true.B
      for (source <- 0 until nSources) {
        val heldByA = aActive && aSource === source.U
        val heldByTlb = slotTranslationPending(source)
        when(slotValid(source) && !slotIssued(source) &&
            !heldByA && !heldByTlb) {
          slotValid(source) := false.B
        }
      }
      inputBeatValid := false.B
      assemblyActive := false.B
    }

    val successReady = descriptorActive && !completionValid &&
      !faultPending && fillRemaining === 0.U &&
      acceptedBytes === descriptorBytes && streamLastSeen &&
      !inputBeatValid && !assemblyActive &&
      !slotTranslationPending.asUInt.orR &&
      !aActive && allSourcesIdle
    val faultReady = descriptorActive && !completionValid && faultPending &&
      streamLastSeen && !slotTranslationPending.asUInt.orR &&
      !aActive && allSourcesIdle

    when(successReady) {
      completionValid := true.B
      completionError := false.B
      completionFault := 0.U.asTypeOf(completionFault)
    }
    when(faultReady) {
      completionValid := true.B
      completionError := true.B
      completionFault.vaddr := faultVaddr
      completionFault.cause := faultCause
      completionFault.isWrite := true.B
    }

    when(io.completion.fire) {
      completionValid := false.B
      descriptorActive := false.B
      faultPending := false.B
      streamLastSeen := false.B
      inputBeatValid := false.B
      assemblyActive := false.B
    }
  }
}

/** Reader and writer share one translated VPU port while retaining independent
  * state machines.  The parent RoCC exposes idNode as its dedicated TL node.
  */
class VpuTLMemory(vpuParams: VpuParams)(implicit params: Parameters) extends LazyModule {
  val reader = LazyModule(new VpuTLReaderPipelined(vpuParams))
  val writer = LazyModule(new VpuTLWriterPipelined(vpuParams))
  private val xbar = TLXbar()
  val idNode = TLIdentityNode()

  xbar := TLBuffer() := reader.node
  xbar := TLBuffer() := writer.node
  idNode := TLWidthWidget(vpuParams.dmaBusWidth / 8) := TLBuffer() := xbar

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) with HasCoreParameters {
    implicit val edge: TLEdgeOut = reader.node.edges.out.head
    private val sourceBits = math.max(1,
      log2Ceil(vpuParams.dmaMaxInFlight))
    private val translationTagBits = sourceBits + 1

    val io = IO(new Bundle {
      val dma = Flipped(new VpuDmaIO(vpuParams))
      val ptw = new freechips.rocketchip.rocket.TLBPTWIO()(p)
      val busy = Output(Bool())
      val fault = Output(Bool())
    })

    val tlb = Module(new FrontendTLB(
      nClients = 1,
      entries = vpuParams.tlbEntries,
      maxSize = vpuParams.dmaMaxBytes,
      // FrontendTLB's optional L0 filter keys only on VPN and leaves the
      // exception fields invalid on a filter hit.  VPU descriptors carry an
      // independently snapshotted MStatus, so reusing that translation across
      // descriptors could both lose precise faults and apply stale privilege
      // context.  Keep the Rocket TLB itself enabled, but bypass this L0
      // register filter for descriptor-correct translation.
      use_tlb_register_filter = false,
      use_firesim_simulation_counters = false,
      use_shared_tlb = true)(edge, p))
    val translationScheduler = Module(new VpuTLBTranslationScheduler(
      translationTagBits, vpuParams.dmaMaxBytes)(p))
    val translationArbiter = Module(new RRArbiter(
      new VpuTLBTaggedRequest(translationTagBits,
        vpuParams.dmaMaxBytes)(p), 2))

    translationArbiter.io.in(0) <> reader.module.io.translationRequest
    translationArbiter.io.in(1) <> writer.module.io.translationRequest
    translationScheduler.io.request <> translationArbiter.io.out
    translationScheduler.io.tlb <> tlb.io.clients.head

    val resultIsWriter =
      translationScheduler.io.result.bits.tag(translationTagBits - 1)
    reader.module.io.translationResult.valid :=
      translationScheduler.io.result.valid && !resultIsWriter
    reader.module.io.translationResult.bits :=
      translationScheduler.io.result.bits
    writer.module.io.translationResult.valid :=
      translationScheduler.io.result.valid && resultIsWriter
    writer.module.io.translationResult.bits :=
      translationScheduler.io.result.bits
    translationScheduler.io.result.ready := Mux(resultIsWriter,
      writer.module.io.translationResult.ready,
      reader.module.io.translationResult.ready)

    val halted = RegInit(false.B)
    val clearPending = RegInit(false.B)
    val enginesDrained = !reader.module.io.busy && !writer.module.io.busy
    val faultEvent = reader.module.io.fault || writer.module.io.fault
    val clearRequested = clearPending || io.dma.clearFault
    // An accepted A-channel request must always receive its D response.  Fault
    // clearing therefore invalidates the TLB only after both engines have
    // delivered their final error/data completion and become idle.
    val clearNow = clearRequested && enginesDrained && !faultEvent

    when(io.dma.clearFault) { clearPending := true.B }
    when(faultEvent || tlb.io.exp.head.interrupt) { halted := true.B }
    when(clearNow) {
      clearPending := false.B
      halted := false.B
    }

    val admitDescriptor = !halted && !clearPending && !io.dma.clearFault &&
      !faultEvent && !tlb.io.exp.head.interrupt
    reader.module.io.descriptor.valid := io.dma.readDescriptor.valid && admitDescriptor
    reader.module.io.descriptor.bits := io.dma.readDescriptor.bits
    io.dma.readDescriptor.ready := reader.module.io.descriptor.ready && admitDescriptor
    io.dma.readData <> reader.module.io.data
    writer.module.io.descriptor.valid := io.dma.writeDescriptor.valid && admitDescriptor
    writer.module.io.descriptor.bits := io.dma.writeDescriptor.bits
    io.dma.writeDescriptor.ready := writer.module.io.descriptor.ready && admitDescriptor
    writer.module.io.data <> io.dma.writeData
    io.dma.writeCompletion <> writer.module.io.completion

    io.ptw <> tlb.io.ptw.head
    tlb.io.exp.head.flush_retry := false.B
    tlb.io.exp.head.flush_skip := clearNow
    tlb.io.counter.external_reset := false.B

    io.dma.clearFaultDone := clearNow
    io.dma.halted := halted || tlb.io.exp.head.interrupt
    io.busy := !enginesDrained || clearPending
    io.fault := halted || tlb.io.exp.head.interrupt
  }
}
