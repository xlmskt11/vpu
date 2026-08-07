package vpu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.diplomacy.{IdRange, LazyModule, LazyModuleImp}
import freechips.rocketchip.rocket.{MStatus}
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.tile.HasCoreParameters
import freechips.rocketchip.tilelink._

/** Gemmini-style store DMA with independent command, payload, A-channel, and
  * D-response lifetimes.
  *
  * VpuCore may send the next descriptor and SRAM payload as soon as the
  * previous payload has been captured.  Each cache-line transaction retains
  * its command tag, translation status, address, and data in a source-owned
  * slot until TileLink returns the corresponding AccessAck.  Command
  * completions retire in descriptor order, while D responses may arrive in
  * any order.
  */
private[vpu] class VpuTLWriterPipelined(vpuParams: VpuParams)
    (implicit params: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(
    TLClientParameters(
      name = "vpu-writer-pipelined",
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
    private val byteCountBits =
      log2Ceil(vpuParams.vLen * vpuParams.storageBytes + 1)
    private val transferByteCountBits =
      log2Ceil(vpuParams.elementsPerBank * vpuParams.storageBytes + 1)
    private val beatByteCountBits = log2Ceil(beatBytes + 1)
    private val requestBytesBits = log2Ceil(requestMaxBytes + 1)
    private val requestBeatBits =
      math.max(1, log2Ceil(maxBeatsPerRequest))
    private val pendingBits = math.max(1, log2Ceil(nSources + 1))
    // VpuCore remaps store RS tags onto a compact zero-based transport-tag
    // pool. Keep the global tag width at the boundary, but size per-command
    // state by the number of store transports that can actually be live.
    private val commandTagCapacity = vpuParams.storeRsEntries

    require(requestMaxBytes >= beatBytes &&
      requestMaxBytes % beatBytes == 0)
    require(isPow2(maxBeatsPerRequest))
    require(requestMaxBytes <= (1 << pgIdxBits),
      "VPU DMA transactions must fit within one virtual-memory page")

    val io = IO(new Bundle {
      val descriptor = Flipped(Decoupled(new VpuDmaDescriptor(vpuParams)))
      val data = Flipped(Decoupled(new VpuDmaWriteBeat(vpuParams)))
      val completion = Decoupled(new VpuDmaWriteCompletion(vpuParams))
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
    // Descriptor admission and ordered command retirement
    // ----------------------------------------------------------------------
    val descriptorQueue = Module(new Queue(
      new VpuDmaDescriptor(vpuParams), vpuParams.storeQueueEntries))
    val commandOrder = Module(new Queue(
      UInt(vpuParams.dmaCommandTagBits.W), vpuParams.storeQueueEntries))
    val commandActive = RegInit(VecInit(
      Seq.fill(commandTagCapacity)(false.B)))
    // InputDone records the architectural last payload beat.  Sealed is
    // stronger: every byte has reached a source-owned transaction buffer.
    val commandInputDone = RegInit(VecInit(
      Seq.fill(commandTagCapacity)(false.B)))
    val commandSealed = RegInit(VecInit(
      Seq.fill(commandTagCapacity)(false.B)))
    val commandPendingTransactions = RegInit(VecInit(Seq.fill(
      commandTagCapacity)(0.U(pendingBits.W))))

    val faultPending = RegInit(false.B)
    val faultVaddr = Reg(UInt(64.W))
    val faultCause = Reg(UInt(2.W))
    val descriptorAdmissionBlocked = Wire(Bool())
    val descriptorDiscardMode = Wire(Bool())

    val incomingTag = io.descriptor.bits.commandTag
    val incomingTagInRange = incomingTag < commandTagCapacity.U
    val incomingTagFree = incomingTagInRange && !commandActive(incomingTag)
    val descriptorCanEnter = descriptorQueue.io.enq.ready &&
      commandOrder.io.enq.ready && incomingTagFree &&
      !descriptorAdmissionBlocked

    io.descriptor.ready := descriptorCanEnter
    descriptorQueue.io.enq.valid := io.descriptor.valid &&
      commandOrder.io.enq.ready && incomingTagFree &&
      !descriptorAdmissionBlocked
    descriptorQueue.io.enq.bits := io.descriptor.bits
    commandOrder.io.enq.valid := io.descriptor.valid &&
      descriptorQueue.io.enq.ready && incomingTagFree &&
      !descriptorAdmissionBlocked
    commandOrder.io.enq.bits := incomingTag

    when(io.descriptor.valid) {
      assert(incomingTagInRange,
        "VPU writer descriptor command tag is outside the store-tag table")
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
        "VPU writer descriptor has too many 2-D rows")
      assert(io.descriptor.bits.elementCount <= vpuParams.vLen.U,
        "VPU writer row exceeds VLEN")
      assert(footprintEnd <= vpuParams.totalElements.U,
        "VPU writer 2-D footprint escapes VSRAM")
      assert(io.descriptor.bits.elementCount === 0.U ||
        footprintStartBank === footprintLastBank,
        "VPU writer 2-D footprint crosses a VSRAM bank")
      assert(rows === 1.U ||
        io.descriptor.bits.hostStrideBytes %
          vpuParams.storageBytes.U === 0.U,
        "VPU writer host row stride is not element aligned")
      assert(rows <= 1.U ||
        io.descriptor.bits.hostStrideBytes >= hostRowBytes,
        "VPU writer descriptor has overlapping external rows")
      assert(hostEndWide <= (BigInt(1) << 64).U(129.W),
        "VPU writer descriptor wraps the 64-bit virtual address space")
    }
    when(io.descriptor.fire) {
      assert(descriptorQueue.io.enq.fire && commandOrder.io.enq.fire,
        "VPU writer descriptor/order queues were not admitted atomically")
      assert(!commandActive(incomingTag),
        "VPU writer accepted a duplicate live command tag")
      commandActive(incomingTag) := true.B
      commandInputDone(incomingTag) := false.B
      commandSealed(incomingTag) := false.B
      commandPendingTransactions(incomingTag) := 0.U
    }

    // ----------------------------------------------------------------------
    // One in-order payload assembler.  Previous descriptors may still own
    // source slots and await D while this context advances to the next command.
    // ----------------------------------------------------------------------
    val fillActive = RegInit(false.B)
    val fillDescriptor = Reg(new VpuDmaDescriptor(vpuParams))
    val fillVaddr = Reg(UInt(64.W))
    val fillRemaining = RegInit(0.U(byteCountBits.W))
    val fillRows = Reg(UInt(vpuParams.dmaRowCountBits.W))
    val fillRequestRow = RegInit(0.U(vpuParams.dmaRowCountBits.W))
    val fillAllRowsReserved = RegInit(false.B)
    val fillDescriptorBytes = Reg(UInt(transferByteCountBits.W))
    val acceptedBytes = RegInit(0.U(transferByteCountBits.W))
    val acceptedRow = RegInit(0.U(vpuParams.dmaRowCountBits.W))
    val acceptedRowElements = RegInit(0.U(vpuParams.vlBits.W))
    val streamLastSeen = RegInit(false.B)
    // Asserted after the final byte has left the input/assembly path.  It is
    // declared here so descriptor dequeue can roll directly from a completed
    // command into the next queued descriptor.
    val fillFinished = Wire(Bool())

    descriptorQueue.io.deq.ready := descriptorDiscardMode || !fillActive ||
      fillFinished
    when(descriptorQueue.io.deq.fire && !descriptorDiscardMode) {
      val rows = Mux(descriptorQueue.io.deq.bits.rowCount === 0.U,
        1.U, descriptorQueue.io.deq.bits.rowCount)
      val rowBytes = descriptorQueue.io.deq.bits.elementCount *
        vpuParams.storageBytes.U
      val totalBytes = rows * rowBytes
      fillDescriptor := descriptorQueue.io.deq.bits
      fillVaddr := descriptorQueue.io.deq.bits.vaddr
      fillRemaining := rowBytes
      fillRows := rows
      fillRequestRow := 0.U
      fillAllRowsReserved := false.B
      fillDescriptorBytes := totalBytes
      acceptedBytes := 0.U
      acceptedRow := 0.U
      acceptedRowElements := 0.U
      streamLastSeen := false.B
      when(descriptorQueue.io.deq.bits.elementCount === 0.U) {
        commandInputDone(descriptorQueue.io.deq.bits.commandTag) := true.B
        commandSealed(descriptorQueue.io.deq.bits.commandTag) := true.B
        fillAllRowsReserved := true.B
        fillActive := false.B
      }.otherwise {
        fillActive := true.B
      }
    }

    // ----------------------------------------------------------------------
    // Source-ID transaction table
    // ----------------------------------------------------------------------
    val slotValid = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotTranslationPending = RegInit(VecInit(
      Seq.fill(nSources)(false.B)))
    val slotPayloadReady = RegInit(VecInit(
      Seq.fill(nSources)(false.B)))
    val slotTranslated = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotIssued = RegInit(VecInit(Seq.fill(nSources)(false.B)))
    val slotCommandTag = Reg(Vec(
      nSources, UInt(vpuParams.dmaCommandTagBits.W)))
    val slotStatus = Reg(Vec(nSources, new MStatus))
    val slotVaddr = Reg(Vec(nSources, UInt(64.W)))
    val slotPaddr = Reg(Vec(nSources, UInt(paddrBits.W)))
    val slotRequestBytes = Reg(Vec(
      nSources, UInt(requestBytesBits.W)))
    val slotLgSize = Reg(Vec(
      nSources, UInt(edge.bundle.sizeBits.W)))
    val slotUsefulBytes = Reg(Vec(
      nSources, UInt(requestBytesBits.W)))
    val slotUsefulShift = Reg(Vec(
      nSources, UInt(requestBytesBits.W)))
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

    // One elastic DMA beat bridges the narrow Core stream into aligned TL
    // transaction payloads.  A beat may straddle two naturally aligned puts.
    val inputBeatValid = RegInit(false.B)
    val inputBeatData = Reg(UInt(vpuParams.dmaBusWidth.W))
    val inputBeatRemaining = Reg(UInt(beatByteCountBits.W))
    val inputBeatOffset = Reg(UInt(beatByteCountBits.W))

    val aActive = RegInit(false.B)
    val aSource = Reg(UInt(sourceBits.W))
    val aBeat = RegInit(0.U(requestBeatBits.W))

    // ----------------------------------------------------------------------
    // Translation and D-response classification
    // ----------------------------------------------------------------------
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

    // ----------------------------------------------------------------------
    // Tagged Core payload stream and protocol checking
    // ----------------------------------------------------------------------
    val inputTag = io.data.bits.commandTag
    val inputTagInRange = inputTag < commandTagCapacity.U
    val inputTagActive = inputTagInRange && commandActive(inputTag)
    val inputValidElements = PopCount(io.data.bits.elementMask)
    val inputValidBytes = inputValidElements * vpuParams.storageBytes.U
    val expectedMaskWide =
      (1.U((vpuParams.dmaElementsPerBeat + 1).W) <<
        inputValidElements) - 1.U
    val inputMaskContiguous = io.data.bits.elementMask ===
      expectedMaskWide(vpuParams.dmaElementsPerBeat - 1, 0)
    val expectedInputSpad = fillDescriptor.spadElement +
      acceptedRow * vpuParams.vLen.U + acceptedRowElements
    val inputRowElementsAfter = acceptedRowElements +& inputValidElements

    val assemblyCanConsume = assemblyActive && inputBeatValid &&
      !faultPending
    val assemblyTake = Mux(inputBeatRemaining < assemblyRemaining,
      inputBeatRemaining, assemblyRemaining)
    val inputConsumedThisCycle = assemblyCanConsume &&
      assemblyTake === inputBeatRemaining
    val normalDataReady = fillActive && !faultPending &&
      acceptedBytes < fillDescriptorBytes &&
      (!inputBeatValid || inputConsumedThisCycle) &&
      !dFaultEvent && !tlbFaultEvent
    // After a local writer fault, already accepted Core descriptors continue
    // to their tagged last beat.  The writer discards these bytes while
    // producing exactly one terminal completion for every live command.
    io.data.ready := Mux(faultPending, inputTagActive, normalDataReady)

    val normalDataFire = io.data.fire && !faultPending
    val nextAcceptedBytes = acceptedBytes + inputValidBytes
    val malformedInput = normalDataFire &&
      (inputTag =/= fillDescriptor.commandTag ||
        inputValidElements === 0.U || !inputMaskContiguous ||
        io.data.bits.spadElement =/= expectedInputSpad ||
        inputRowElementsAfter > fillDescriptor.elementCount ||
        nextAcceptedBytes > fillDescriptorBytes ||
        io.data.bits.last =/= (nextAcceptedBytes === fillDescriptorBytes))
    val protocolFaultEvent = malformedInput
    val newFaultEvent = dFaultEvent || tlbFaultEvent || protocolFaultEvent
    descriptorAdmissionBlocked := faultPending || newFaultEvent
    descriptorDiscardMode := faultPending || newFaultEvent
    io.fault := newFaultEvent

    when(io.data.valid) {
      assert(inputTagInRange,
        "VPU writer payload command tag is outside the store-tag table")
      assert(!faultPending || inputTagActive,
        "VPU writer fault drain received an inactive command tag")
    }
    when(io.data.fire && io.data.bits.last) {
      commandInputDone(inputTag) := true.B
    }
    // If the producer supplies exactly/all remaining bytes but omits `last`,
    // the protocol error itself is terminal for that command.  Waiting for a
    // later last beat would deadlock fault drain because a well-bounded
    // producer has no payload left to send.
    when(protocolFaultEvent && inputTag === fillDescriptor.commandTag &&
        nextAcceptedBytes >= fillDescriptorBytes) {
      commandInputDone(inputTag) := true.B
    }
    when(normalDataFire && !malformedInput) {
      inputBeatValid := true.B
      inputBeatData := io.data.bits.data
      inputBeatRemaining := inputValidBytes
      inputBeatOffset := 0.U
      acceptedBytes := nextAcceptedBytes
      when(inputRowElementsAfter === fillDescriptor.elementCount) {
        acceptedRow := acceptedRow + 1.U
        acceptedRowElements := 0.U
      }.otherwise {
        acceptedRowElements := inputRowElementsAfter
      }
      when(io.data.bits.last) { streamLastSeen := true.B }
    }

    // ----------------------------------------------------------------------
    // Naturally aligned transaction reservation and payload assembly
    // ----------------------------------------------------------------------
    val (plannedBase, plannedBytes, plannedLgSize, plannedUseful,
      plannedShift) = VpuTLUtil.bestAlignedTransaction(
        fillVaddr, fillRemaining, beatBytes, requestMaxBytes)
    val assemblyCompletes = assemblyCanConsume &&
      assemblyTake === assemblyRemaining && !newFaultEvent
    // A completed cache-line buffer hands the assembler directly to another
    // free source.  Without this rollover the 128-bit input stream lost one
    // cycle after every four beats of a 64-byte transaction.
    val reserveAssembly = fillActive &&
      (!assemblyActive || assemblyCompletes) &&
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
      slotTranslationPending(freeSource) := false.B
      slotPayloadReady(freeSource) := false.B
      slotTranslated(freeSource) := false.B
      slotIssued(freeSource) := false.B
      slotCommandTag(freeSource) := fillDescriptor.commandTag
      slotStatus(freeSource) := fillDescriptor.status
      slotVaddr(freeSource) := plannedBase
      slotRequestBytes(freeSource) := plannedBytes
      slotLgSize(freeSource) := plannedLgSize
      slotUsefulBytes(freeSource) := plannedUseful
      slotUsefulShift(freeSource) := plannedShift
      when(plannedUseful === fillRemaining) {
        when(fillRequestRow + 1.U === fillRows) {
          fillRemaining := 0.U
          fillAllRowsReserved := true.B
        }.otherwise {
          fillRequestRow := fillRequestRow + 1.U
          fillVaddr := fillDescriptor.vaddr +
            (fillRequestRow + 1.U) * fillDescriptor.hostStrideBytes
          fillRemaining := fillDescriptor.elementCount *
            vpuParams.storageBytes.U
        }
      }.otherwise {
        fillVaddr := fillVaddr + plannedUseful
        fillRemaining := fillRemaining - plannedUseful
      }
    }

    val inputBeatBytes =
      inputBeatData.asTypeOf(Vec(beatBytes, UInt(8.W)))
    val inputBeatRefill = normalDataFire && !malformedInput
    when(assemblyCanConsume && !newFaultEvent) {
      assert(assemblyTake =/= 0.U,
        "VPU writer assembly engine made no progress")
      for (byte <- 0 until beatBytes) {
        when(byte.U < assemblyTake) {
          slotPayload(assemblySource)(assemblyPosition + byte.U) :=
            inputBeatBytes(inputBeatOffset + byte.U)
        }
      }
      when(!(assemblyCompletes && reserveAssembly)) {
        assemblyRemaining := assemblyRemaining - assemblyTake
        assemblyPosition := assemblyPosition + assemblyTake
      }
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
        when(!reserveAssembly) { assemblyActive := false.B }
      }
    }

    fillFinished := fillActive && !faultPending && !newFaultEvent &&
      fillAllRowsReserved && acceptedBytes === fillDescriptorBytes &&
      streamLastSeen && !inputBeatValid && !assemblyActive
    when(fillFinished) {
      commandSealed(fillDescriptor.commandTag) := true.B
      // The descriptor dequeue block has already initialized a queued
      // successor on a same-cycle rollover.  Do not overwrite that state.
      when(!(descriptorQueue.io.deq.fire && !descriptorDiscardMode)) {
        fillActive := false.B
        streamLastSeen := false.B
      }
    }

    // ----------------------------------------------------------------------
    // Independent TLB and multi-source TileLink A pipelines
    // ----------------------------------------------------------------------
    val translationEligible = VecInit((0 until nSources).map { source =>
      // Address/status are snapshotted when the source is reserved, so TLB
      // lookup can overlap the four-beat cache-line payload assembly.
      slotValid(source) &&
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
    io.translationRequest.bits.status := slotStatus(translationRequestSource)
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
      when(!translationFault && !faultPending && !dFaultEvent) {
        slotPaddr(translationSource) := io.translationResult.bits.paddr
        slotTranslated(translationSource) := true.B
      }
    }

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
    when(!aActive && issueEligibleValid && !faultPending &&
        !newFaultEvent) {
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

    when(tl.d.valid) {
      assert(dCanAccept, "unexpected or duplicate VPU write response")
      assert(tl.d.bits.opcode === TLMessages.AccessAck,
        "VPU write expected AccessAck")
      assert(tl.d.bits.size === slotLgSize(dSource),
        "VPU write acknowledgement changed transaction size")
    }

    // Release an issued slot on its D, a faulting translation on its result,
    // or any unexposed slot after the first global writer fault.  A held
    // multi-beat A transaction and an outstanding TLB request must drain.
    val slotRelease = Wire(Vec(nSources, Bool()))
    for (source <- 0 until nSources) {
      val releaseByD = tl.d.fire && dSource === source.U
      val releaseByTlb = tlbResponse && translationSource === source.U &&
        (translationFault || faultPending || dFaultEvent)
      val heldByA = aActive && aSource === source.U
      val releaseUnexposed = (faultPending || newFaultEvent) &&
        slotValid(source) && !slotIssued(source) &&
        !slotTranslationPending(source) && !heldByA
      slotRelease(source) := releaseByD || releaseByTlb || releaseUnexposed
      when(slotRelease(source)) {
        slotValid(source) := false.B
        slotTranslationPending(source) := false.B
        slotPayloadReady(source) := false.B
        slotTranslated(source) := false.B
        slotIssued(source) := false.B
      }
    }

    // A source reservation and one or more releases may affect the same
    // command in one cycle.  Update each pending count exactly once.
    for (tag <- 0 until commandTagCapacity) {
      val reserveForTag = reserveAssembly &&
        fillDescriptor.commandTag === tag.U
      val releasesForTag = PopCount((0 until nSources).map { source =>
        slotRelease(source) && slotCommandTag(source) === tag.U
      })
      val pendingWithReserve = commandPendingTransactions(tag) +&
        reserveForTag.asUInt
      when(reserveForTag || releasesForTag =/= 0.U) {
        assert(pendingWithReserve >= releasesForTag,
          "VPU writer command transaction count underflow")
        commandPendingTransactions(tag) :=
          pendingWithReserve - releasesForTag
      }
    }

    // ----------------------------------------------------------------------
    // Fault capture, transport drain, and ordered terminal completions
    // ----------------------------------------------------------------------
    val expectedInputVaddr = fillDescriptor.vaddr +
      acceptedRow * fillDescriptor.hostStrideBytes +
      acceptedRowElements * vpuParams.storageBytes.U
    val eventVaddr = Mux(dFaultEvent, slotVaddr(dSource),
      Mux(tlbFaultEvent, slotVaddr(translationSource),
        expectedInputVaddr))
    val eventCause = Mux(dFaultEvent, VpuDmaFaultCause.Access.U,
      Mux(tlbFaultEvent,
        Mux(translationPageFault, VpuDmaFaultCause.Translation.U,
          VpuDmaFaultCause.Access.U),
        VpuDmaFaultCause.Protocol.U))

    when(newFaultEvent) {
      when(!faultPending) {
        faultVaddr := eventVaddr
        faultCause := eventCause
      }
      faultPending := true.B
      fillActive := false.B
      streamLastSeen := false.B
      inputBeatValid := false.B
      assemblyActive := false.B
    }

    val completionValid = RegInit(false.B)
    val completionBits = Reg(new VpuDmaWriteCompletion(vpuParams))
    io.completion.valid := completionValid
    io.completion.bits := completionBits

    val retirementTag = commandOrder.io.deq.bits
    val retirementTagInRange = retirementTag < commandTagCapacity.U
    val normalRetirementReady = retirementTagInRange &&
      commandActive(retirementTag) && commandSealed(retirementTag) &&
      commandPendingTransactions(retirementTag) === 0.U
    val transportDrained = allSourcesIdle && !aActive &&
      !slotTranslationPending.asUInt.orR && !assemblyActive &&
      !inputBeatValid && !fillActive && !descriptorQueue.io.deq.valid
    val faultRetirementReady = faultPending && transportDrained &&
      retirementTagInRange && commandActive(retirementTag) &&
      commandInputDone(retirementTag)
    val generateCompletion = !completionValid &&
      commandOrder.io.deq.valid &&
      Mux(faultPending, faultRetirementReady, normalRetirementReady)

    when(generateCompletion) {
      completionValid := true.B
      completionBits.commandTag := retirementTag
      completionBits.error := faultPending
      completionBits.fault.vaddr := Mux(faultPending, faultVaddr, 0.U)
      completionBits.fault.cause := Mux(faultPending, faultCause, 0.U)
      completionBits.fault.isWrite := faultPending
    }

    commandOrder.io.deq.ready := io.completion.fire
    when(io.completion.fire) {
      assert(commandOrder.io.deq.valid &&
        io.completion.bits.commandTag === commandOrder.io.deq.bits,
        "VPU writer completion did not retire the command-order head")
      completionValid := false.B
      commandActive(io.completion.bits.commandTag) := false.B
      commandInputDone(io.completion.bits.commandTag) := false.B
      commandSealed(io.completion.bits.commandTag) := false.B
      commandPendingTransactions(io.completion.bits.commandTag) := 0.U
      when(io.completion.bits.error && commandOrder.io.count === 1.U) {
        faultPending := false.B
      }
    }

    io.busy := descriptorQueue.io.deq.valid || commandOrder.io.deq.valid ||
      commandActive.asUInt.orR || fillActive || inputBeatValid ||
      assemblyActive || !allSourcesIdle || aActive || completionValid ||
      faultPending
  }
}
