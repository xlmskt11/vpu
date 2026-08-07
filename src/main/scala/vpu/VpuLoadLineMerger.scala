package vpu

import chisel3._
import chisel3.util._

/** Descriptor metadata needed to turn independently returned DMA fragments
  * into complete Vector-SRAM lane words.
  *
  * VSRAM addresses are element addresses.  The descriptor is registered
  * before any fragment carrying the same commandTag is accepted.
  */
class VpuLoadMergeDescriptor(p: VpuParams) extends Bundle {
  val commandTag = UInt(p.dmaCommandTagBits.W)
  val spadElement = UInt(p.elementAddrBits.W)
  val elementCount = UInt(p.vlBits.W)
  // See VpuDmaDescriptor. Zero retains the old one-row convention.
  val rowCount = UInt(p.dmaRowCountBits.W)
}

/** One completely assembled Vector-SRAM lane word.
  *
  * `laneMask` is the architectural descriptor mask, rather than a mask of
  * bytes which happened to arrive on the last DMA beat.  Full words therefore
  * leave with every lane set and require one SRAM write.  A descriptor tail
  * remains explicit so VpuCore may either perform a masked write or merge it
  * with the old word before performing a full-word write.
  *
  * DMA responses can arrive out of address order.  Consequently `last` means
  * that this is the last word of the command to leave the merger; it does not
  * mean that `address` is the numerically highest word address.
  */
class VpuDmaReadWord(p: VpuParams) extends Bundle {
  val data = Vec(p.nLanes, UInt(p.storageBits.W))
  val laneMask = Vec(p.nLanes, Bool())
  val address = UInt(p.elementAddrBits.W)
  val commandTag = UInt(p.dmaCommandTagBits.W)
  val last = Bool()
  val error = Bool()
  val fault = new VpuDmaFaultInfo
}

/** Gemmini BeatMerger-style row assembler for VPU loads.
  *
  * Gemmini first assembles TileLink beats into a scratchpad-row-wide result
  * and only then arbitrates for the SRAM write port.  The VPU reader also
  * supports several outstanding TileLink sources, so an arrival-order shift
  * register is insufficient: a response for a later address may finish first.
  * This module therefore keys each assembly entry by `(commandTag, aligned
  * VSRAM word address)` and records one received bit per SRAM lane.
  *
  * The finite entry set is deliberately independent of TileLink source-ID
  * ordering.  A long 2-D descriptor may contain more words than the merger can
  * retain at once; ordinary Decoupled backpressure then bounds staging while
  * completed words drain to the SRAM write port.
  */
class VpuLoadLineMerger(
    p: VpuParams,
    nEntries: Int = -1) extends Module {
  private val commandTagCapacity = p.loadRsEntries
  // The reader serializes one completed TL line into the merger while its
  // bounded completed-line queue waits upstream. The merger therefore needs
  // staging for that response window, not one entry for every word of every
  // load descriptor. The derived bound also covers an unaligned transaction
  // touching multiple VSRAM words; ordinary Decoupled backpressure stops the
  // unpacker when the entries fill.
  private val entriesCount =
    if (nEntries > 0) nEntries else p.dmaReadMergeEntries
  private val entryIndexBits = math.max(1, log2Ceil(entriesCount))
  private val wordsRemainingBits = p.dmaTransferWordsBits
  private val wordsPerRowBits = math.max(1, log2Ceil(p.wordsPerVector + 1))

  require(entriesCount > 0)
  require(p.nLanes % p.dmaElementsPerBeat == 0,
    "one DMA fragment must fit within one VSRAM lane word")

  val io = IO(new Bundle {
    val descriptor = Flipped(Decoupled(new VpuLoadMergeDescriptor(p)))
    val in = Flipped(Decoupled(new VpuDmaReadBeat(p)))
    val out = Decoupled(new VpuDmaReadWord(p))
    val busy = Output(Bool())
  })

  val descriptorActive = RegInit(VecInit(
    Seq.fill(commandTagCapacity)(false.B)))
  val descriptorBase = Reg(Vec(
    commandTagCapacity, UInt(p.elementAddrBits.W)))
  val descriptorElements = Reg(Vec(
    commandTagCapacity, UInt(p.vlBits.W)))
  val descriptorRows = Reg(Vec(
    commandTagCapacity, UInt(p.dmaRowCountBits.W)))
  val descriptorWordsRemaining = Reg(Vec(
    commandTagCapacity, UInt(wordsRemainingBits.W)))
  val descriptorElementsReceived = Reg(Vec(
    commandTagCapacity, UInt(p.dmaTransferElementsBits.W)))
  val descriptorDropping = RegInit(VecInit(
    Seq.fill(commandTagCapacity)(false.B)))
  val descriptorFault = Reg(Vec(commandTagCapacity, new VpuDmaFaultInfo))

  val descriptorTag = io.descriptor.bits.commandTag
  val descriptorTagInRange = descriptorTag < commandTagCapacity.U
  val descriptorTagFree = descriptorTagInRange &&
    !descriptorActive(descriptorTag)
  io.descriptor.ready := descriptorTagFree

  when(io.descriptor.valid) {
    assert(descriptorTagInRange,
      "VPU load merger descriptor tag is outside the load-tag table")
    assert(io.descriptor.bits.elementCount <= p.vLen.U,
      "VPU load merger descriptor exceeds VLEN")
  }
  when(io.descriptor.fire) {
    val rows = Mux(io.descriptor.bits.rowCount === 0.U,
      1.U, io.descriptor.bits.rowCount)
    // Materialize ceil(elementCount/nLanes) before multiplying by rows.
    // firtool may otherwise legally reassociate
    //   rows * ((elements + lanes - 1) / lanes)
    // into
    //   (rows * (elements + lanes - 1)) / lanes.
    // The intermediate then inherits elementCount's narrow width and the
    // maximum 16*125 descriptor becomes 12 words instead of 128.  A threshold
    // PopCount has the same constant-divisor semantics without an arithmetic
    // expression which can be reassociated.
    val wordsPerRow = Wire(UInt(wordsPerRowBits.W))
    wordsPerRow := PopCount(VecInit((0 until p.wordsPerVector).map { word =>
      io.descriptor.bits.elementCount > (word * p.nLanes).U
    }))
    val rowsWide = rows.pad(wordsRemainingBits)
    val wordsPerRowWide = wordsPerRow.pad(wordsRemainingBits)
    val words = Wire(UInt(wordsRemainingBits.W))
    words := rowsWide * wordsPerRowWide
    assert(words <= (p.dmaMaxRows * p.wordsPerVector).U,
      "VPU load merger descriptor word count overflowed")
    descriptorActive(descriptorTag) := true.B
    descriptorBase(descriptorTag) := io.descriptor.bits.spadElement
    descriptorElements(descriptorTag) := io.descriptor.bits.elementCount
    descriptorRows(descriptorTag) := rows
    descriptorWordsRemaining(descriptorTag) := words
    descriptorElementsReceived(descriptorTag) := 0.U
    descriptorDropping(descriptorTag) := false.B
    descriptorFault(descriptorTag) := 0.U.asTypeOf(descriptorFault(descriptorTag))
  }

  val entryValid = RegInit(VecInit(Seq.fill(entriesCount)(false.B)))
  val entryTag = Reg(Vec(entriesCount,
    UInt(p.dmaCommandTagBits.W)))
  val entryAddress = Reg(Vec(entriesCount,
    UInt(p.elementAddrBits.W)))
  val entryData = Reg(Vec(entriesCount,
    Vec(p.nLanes, UInt(p.storageBits.W))))
  val entryReceived = RegInit(VecInit(Seq.fill(entriesCount)(
    VecInit(Seq.fill(p.nLanes)(false.B)))))
  val entryExpected = Reg(Vec(entriesCount, Vec(p.nLanes, Bool())))

  val inputTag = io.in.bits.commandTag
  val inputTagInRange = inputTag < commandTagCapacity.U
  val inputTagActive = inputTagInRange && descriptorActive(inputTag)
  val inputAddress = io.in.bits.spadElement
  val inputAlignedAddress =
    (inputAddress / p.nLanes.U) * p.nLanes.U
  val inputLaneOffset = inputAddress % p.nLanes.U

  val matchingEntries = VecInit((0 until entriesCount).map { entry =>
    entryValid(entry) && entryTag(entry) === inputTag &&
      entryAddress(entry) === inputAlignedAddress
  })
  val matchValid = matchingEntries.asUInt.orR
  val matchIndex = OHToUInt(PriorityEncoderOH(matchingEntries.asUInt))
  val freeEntries = ~entryValid.asUInt
  val freeValid = freeEntries.orR
  val freeIndex = OHToUInt(PriorityEncoderOH(freeEntries))
  val selectedEntry = Mux(matchValid, matchIndex, freeIndex)

  val inputLaneMask = Wire(Vec(p.nLanes, Bool()))
  val inputLaneData = Wire(Vec(p.nLanes, UInt(p.storageBits.W)))
  for (lane <- 0 until p.nLanes) {
    val candidates = (0 until p.dmaElementsPerBeat).map { element =>
      val selected = io.in.bits.elementMask(element) &&
        inputLaneOffset + element.U === lane.U
      selected -> io.in.bits.data(
        (element + 1) * p.storageBits - 1,
        element * p.storageBits)
    }
    inputLaneMask(lane) := candidates.map(_._1).reduce(_ || _)
    inputLaneData(lane) := Mux1H(candidates)
  }

  val inputExpectedMask = Wire(Vec(p.nLanes, Bool()))
  val inputRelative = inputAlignedAddress - descriptorBase(inputTag)
  val inputRow = inputRelative / p.vLen.U
  val inputRowOffset = inputRelative % p.vLen.U
  for (lane <- 0 until p.nLanes) {
    val rowElement = inputRowOffset + lane.U
    inputExpectedMask(lane) := inputTagActive &&
      inputAlignedAddress >= descriptorBase(inputTag) &&
      inputRow < descriptorRows(inputTag) &&
      rowElement < descriptorElements(inputTag)
  }

  val outValid = RegInit(false.B)
  val outBits = Reg(new VpuDmaReadWord(p))
  io.out.valid := outValid
  io.out.bits := outBits
  val outputFire = io.out.fire
  val outputCanReplace = !outValid || io.out.ready

  val inputDropping = inputTagActive && descriptorDropping(inputTag)
  val inputFaulting = io.in.bits.error || inputDropping
  // A fault does not terminate the input stream by itself.  Like Gemmini's
  // transaction tracker, discard/drain every already-issued response through
  // the descriptor's last marker, then emit one terminal error event.
  val inputIsTerminalOnly = io.in.bits.last &&
    (inputFaulting || !io.in.bits.elementMask.orR)
  val inputDropOnly = inputFaulting && !io.in.bits.last
  val normalInputReady = inputTagActive && (matchValid || freeValid)
  io.in.ready := Mux(inputIsTerminalOnly,
    inputTagActive && outputCanReplace,
    Mux(inputDropOnly, inputTagActive, normalInputReady))

  when(io.in.valid) {
    assert(inputTagInRange,
      "VPU load merger input tag is outside the load-tag table")
    assert(inputTagActive,
      "VPU load merger received data without a live descriptor")
  }

  val normalInputFire = io.in.fire && !inputIsTerminalOnly && !inputDropOnly
  when(normalInputFire) {
    assert(io.in.bits.elementMask.orR,
      "a normal VPU load merger fragment must contain an element")
    assert(inputLaneOffset + PopCount(io.in.bits.elementMask) <= p.nLanes.U,
      "VPU load merger fragment crossed a lane-word boundary")
    assert((inputLaneMask.asUInt & ~inputExpectedMask.asUInt) === 0.U,
      "VPU load merger fragment escaped its descriptor range")
    when(matchValid) {
      assert((entryReceived(matchIndex).asUInt &
        inputLaneMask.asUInt) === 0.U,
        "VPU load merger received a lane twice")
      assert(entryExpected(matchIndex).asUInt ===
        inputExpectedMask.asUInt,
        "VPU load merger key changed its descriptor mask")
    }

    entryValid(selectedEntry) := true.B
    entryTag(selectedEntry) := inputTag
    entryAddress(selectedEntry) := inputAlignedAddress
    entryExpected(selectedEntry) := inputExpectedMask
    for (lane <- 0 until p.nLanes) {
      when(inputLaneMask(lane)) {
        entryData(selectedEntry)(lane) := inputLaneData(lane)
      }
      entryReceived(selectedEntry)(lane) := inputLaneMask(lane) ||
        (matchValid && entryReceived(matchIndex)(lane))
    }

    val receivedAfter = descriptorElementsReceived(inputTag) +&
      PopCount(io.in.bits.elementMask)
    // As above, explicitly retain the bank-bounded product width. This count
    // reaches 2000 for the default max-row VL=125 regression.
    val descriptorTotalElements = Wire(UInt(p.dmaTransferElementsBits.W))
    descriptorTotalElements :=
      descriptorRows(inputTag).pad(p.dmaTransferElementsBits) *
        descriptorElements(inputTag).pad(p.dmaTransferElementsBits)
    assert(receivedAfter <= descriptorTotalElements,
      "VPU load merger received too many descriptor elements")
    when(io.in.bits.last) {
      assert(receivedAfter === descriptorTotalElements,
        "VPU load merger observed last before every element arrived")
    }.otherwise {
      assert(receivedAfter < descriptorTotalElements,
        "VPU load merger received every element without last")
    }
    descriptorElementsReceived(inputTag) := receivedAfter
  }

  val firstFaultFire = io.in.fire && io.in.bits.error && !inputDropping
  when(firstFaultFire) {
    descriptorDropping(inputTag) := true.B
    descriptorFault(inputTag) := io.in.bits.fault
    // Once one fragment faults, no partially assembled row may reach VSRAM.
    for (entry <- 0 until entriesCount) {
      when(entryValid(entry) && entryTag(entry) === inputTag) {
        entryValid(entry) := false.B
        entryReceived(entry) := 0.U.asTypeOf(entryReceived(entry))
      }
    }
  }

  val entryComplete = VecInit((0 until entriesCount).map { entry =>
    entryValid(entry) && entryExpected(entry).asUInt.orR &&
      (entryReceived(entry).asUInt & entryExpected(entry).asUInt) ===
        entryExpected(entry).asUInt
  })
  val completedArbiter = Module(new RRArbiter(
    UInt(entryIndexBits.W), entriesCount))
  for (entry <- 0 until entriesCount) {
    completedArbiter.io.in(entry).valid := entryComplete(entry)
    completedArbiter.io.in(entry).bits := entry.U
  }

  val terminalInputFire = io.in.fire && inputIsTerminalOnly
  completedArbiter.io.out.ready := outputCanReplace && !terminalInputFire
  val completedEntryFire = completedArbiter.io.out.fire
  val completedEntry = completedArbiter.io.out.bits
  val completedTag = entryTag(completedEntry)
  val replacingSameCommand = outputFire && !outBits.error &&
    outBits.commandTag === completedTag
  val completedRemaining = descriptorWordsRemaining(completedTag) -
    replacingSameCommand.asUInt

  when(outputFire) {
    outValid := false.B
    when(outBits.error || !outBits.laneMask.asUInt.orR) {
      descriptorActive(outBits.commandTag) := false.B
      descriptorWordsRemaining(outBits.commandTag) := 0.U
      descriptorDropping(outBits.commandTag) := false.B
    }.otherwise {
      assert(descriptorWordsRemaining(outBits.commandTag) =/= 0.U,
        "VPU load merger word count underflow")
      descriptorWordsRemaining(outBits.commandTag) :=
        descriptorWordsRemaining(outBits.commandTag) - 1.U
      when(outBits.last) {
        assert(descriptorWordsRemaining(outBits.commandTag) === 1.U,
          "VPU load merger retired a descriptor with words remaining")
        descriptorActive(outBits.commandTag) := false.B
      }
    }
  }

  when(terminalInputFire) {
    outValid := true.B
    outBits.data := 0.U.asTypeOf(outBits.data)
    outBits.laneMask := 0.U.asTypeOf(outBits.laneMask)
    outBits.address := 0.U
    outBits.commandTag := inputTag
    outBits.last := true.B
    outBits.error := inputFaulting
    outBits.fault := Mux(io.in.bits.error,
      io.in.bits.fault, descriptorFault(inputTag))
    when(inputFaulting) {
      for (entry <- 0 until entriesCount) {
        when(entryValid(entry) && entryTag(entry) === inputTag) {
          entryValid(entry) := false.B
          entryReceived(entry) := 0.U.asTypeOf(entryReceived(entry))
        }
      }
    }.otherwise {
      assert(descriptorElements(inputTag) === 0.U,
        "an empty VPU load completion was used for a non-empty descriptor")
    }
  }.elsewhen(completedEntryFire) {
    assert(descriptorActive(completedTag),
      "VPU load merger emitted a word for an inactive descriptor")
    assert(completedRemaining =/= 0.U,
      "VPU load merger emitted more words than its descriptor contains")
    outValid := true.B
    outBits.data := entryData(completedEntry)
    outBits.laneMask := entryExpected(completedEntry)
    outBits.address := entryAddress(completedEntry)
    outBits.commandTag := completedTag
    outBits.last := completedRemaining === 1.U
    outBits.error := false.B
    outBits.fault := 0.U.asTypeOf(outBits.fault)
    entryValid(completedEntry) := false.B
    entryReceived(completedEntry) :=
      0.U.asTypeOf(entryReceived(completedEntry))
  }

  io.busy := descriptorActive.asUInt.orR || entryValid.asUInt.orR ||
    outValid
}
