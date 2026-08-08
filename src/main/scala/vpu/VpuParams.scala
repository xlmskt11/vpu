package vpu

import chisel3.util.log2Ceil

sealed trait VpuStorageType {
  def bits: Int
  final def bytes: Int = bits / 8
}

object VpuStorageType {
  case object FP32 extends VpuStorageType { override val bits: Int = 32 }
  case object BF16 extends VpuStorageType { override val bits: Int = 16 }
}

/** Build-time parameters for the standalone VPU.
  *
  * VLEN is expressed in elements, not bits.  Vector SRAM addresses are also
  * element addresses.  The public DMA interface is intentionally independent
  * of TileLink so that a RoCC wrapper can provide the target-specific TLB and
  * beat packing.
  */
case class VpuParams(
  storageType: VpuStorageType = VpuStorageType.FP32,
  computeType: VpuStorageType = VpuStorageType.FP32,
  vLen: Int = 128,
  nLanes: Int = 16,
  sfuLanes: Int = 4,
  vSpadKB: Int = 64,
  vSpadBanks: Int = 8,
  // Optional sub-banking uses the same interleaved row mapping as Gemmini's
  // shared SRAM: logical-row % subBanks selects the sub-bank.
  vSpadSubBanks: Int = 1,
  // Number of Gemmini matrix read/write clients. Zero preserves the closed,
  // standalone VPU interface and elaborates no fusion ports.
  matrixPorts: Int = 0,
  // Number of elements transferred by one Gemmini matrix-row request. Zero
  // keeps the legacy one-VPU-word row (nLanes elements). A wider matrix row
  // is split across matrixWordsPerRow consecutive VSRAM sub-banks while the
  // VPU datapath and each physical SRAM word remain nLanes wide.
  matrixRowElements: Int = 0,
  enableSharedDeps: Boolean = false,
  loadQueueEntries: Int = 4,
  execQueueEntries: Int = 8,
  storeQueueEntries: Int = 4,
  dmaBusWidth: Int = 128,
  dmaMaxBytes: Int = 64,
  // Per direction: reader and writer each own this many TL source IDs. The
  // shared xbar may therefore expose up to 2*dmaMaxInFlight downstream IDs.
  dmaMaxInFlight: Int = 8,
  tlbEntries: Int = 4,
  expTableEntries: Int = 64,
  reciprocalRefineIters: Int = 2,
  fmaPipeDepth: Int = 4,
  // Arbitrary vector masks are versioned in this small architectural file.
  // Execute RS entries retain only a slot ID, rather than one VLEN-bit copy.
  maskSlots: Int = 4,
  fpStateEntries: Int = 256,
  fpStateBanks: Int = 4,
  // Captured fine-grained commands and nested-loop contexts.  The loop
  // frontend is a transport compressor: replayed commands still allocate the
  // ordinary VPU reservation station and shared dependency table.
  loopBufferEntries: Int = 64,
  loopStackDepth: Int = 4,
  enableGroupedCommands: Boolean = false,
  groupIdBits: Int = 3) {

  val computeBits: Int = computeType.bits
  val storageBits: Int = storageType.bits
  val storageBytes: Int = storageType.bytes
  // LD/ST retain their configured queue capacities. EX additionally retains
  // the former active-engine slot because an RS entry lives through command
  // completion. Global tag capacity is therefore derived from the actual RS
  // geometry instead of being a second, independently configurable knob.
  val loadRsEntries: Int = loadQueueEntries
  val execRsEntries: Int = execQueueEntries + 1
  val storeRsEntries: Int = storeQueueEntries
  val hazardEntries: Int = loadRsEntries + execRsEntries + storeRsEntries
  val rsTagBits: Int = math.max(1, log2Ceil(hazardEntries))
  val totalBytes: Int = vSpadKB * 1024
  val totalElements: Int = totalBytes / storageBytes
  val elementsPerBank: Int = totalElements / vSpadBanks
  // A 2-D DMA command places each logical row in one architectural VLEN
  // slot.  Keeping the complete footprint inside one bank bounds both the
  // descriptor counters and the conservative reservation-station range.
  val dmaMaxRows: Int = elementsPerBank / vLen
  val wordBits: Int = nLanes * storageBits
  val wordBytes: Int = wordBits / 8
  // A translated TL line may begin near the end of a VSRAM word, so account
  // for one extra touched word beyond the cache-line/word size ratio. The
  // response path can hold dmaMaxInFlight completed lines plus the line being
  // unpacked. This is the bounded merge-window requirement; it is independent
  // of VLEN and of the number of words in every queued descriptor.
  val dmaMaxWordsPerTransaction: Int =
    (dmaMaxBytes + wordBytes - 1) / wordBytes + 1
  val dmaReadMergeEntries: Int = math.max(2,
    (dmaMaxInFlight + 1) * dmaMaxWordsPerTransaction)
  val totalWords: Int = totalElements / nLanes
  val matrixElementsPerRow: Int =
    if (matrixRowElements == 0) nLanes else matrixRowElements
  val matrixWordsPerRow: Int = matrixElementsPerRow / nLanes
  val matrixRows: Int = totalElements / matrixElementsPerRow
  // Matrix fusion and the cross-accelerator dependency table use Gemmini
  // matrix rows as their common address unit. Physical VSRAM words remain
  // nLanes wide and are an implementation detail of the scratchpad bridge.
  val matrixRowAddrBits: Int = math.max(1, log2Ceil(matrixRows))
  // Half-open shared dependency intervals must represent matrixRows itself
  // as the exclusive end, hence the +1 in the width calculation.
  val sharedHazardAddressBits: Int =
    math.max(1, log2Ceil(matrixRows + 1))
  val wordsPerBank: Int = totalWords / vSpadBanks
  val wordsPerSubBank: Int = wordsPerBank / vSpadSubBanks
  val physicalBanks: Int = vSpadBanks * vSpadSubBanks
  val dmaElementsPerBeat: Int = dmaBusWidth / storageBits
  require(reciprocalRefineIters == 2,
    "The public v1 numerical contract fixes two reciprocal refinements")
  // Two FP32 Newton refinements use one fused error step and one multiply
  // step each.  Reciprocal mode statically partitions the shared FMA fabric
  // into groups of this size, so no reciprocal-only FMA lanes are elaborated.
  val reciprocalFmasPerLane: Int = reciprocalRefineIters * 2
  val reciprocalLanes: Int = nLanes / reciprocalFmasPerLane
  val reciprocalLatency: Int = 1 +
    reciprocalFmasPerLane * (fmaPipeDepth - 1)
  val elementAddrBits: Int = math.max(1, log2Ceil(totalElements))
  // DMA sees zero-based load tags and independently allocated zero-based
  // store transport tags. It never carries execute/global RS identities.
  val dmaCommandTagBits: Int = math.max(1,
    log2Ceil(math.max(loadRsEntries, storeRsEntries)))
  val vlBits: Int = math.max(1, log2Ceil(vLen + 1))
  val wordsPerVector: Int = vLen / nLanes
  val dmaRowCountBits: Int = math.max(1, log2Ceil(dmaMaxRows + 1))
  val dmaTransferElementsBits: Int =
    math.max(1, log2Ceil(elementsPerBank + 1))
  val dmaTransferWordsBits: Int =
    math.max(1, log2Ceil(dmaMaxRows * wordsPerVector + 1))
  val bankBits: Int = math.max(1, log2Ceil(vSpadBanks))
  val rowBits: Int = math.max(1, log2Ceil(wordsPerBank))
  val physicalBankBits: Int = math.max(1, log2Ceil(physicalBanks))
  val subBankRowBits: Int = math.max(1, log2Ceil(wordsPerSubBank))
  val vectorMaskChunks: Int = (vLen + 63) / 64
  val maskSlotBits: Int = math.max(1, log2Ceil(maskSlots))
  val maskChunkIndexBits: Int = math.max(1, log2Ceil(vectorMaskChunks))
  val maskWordIndexBits: Int = math.max(1, log2Ceil(wordsPerVector))
  val gatherIndexBits: Int = math.max(1, log2Ceil(vLen))
  // Source-0, source-1, and store responses have independent queues, so the
  // tag identifies only the element offset/address; no client-owner bit is
  // required.
  val spadReadTagBits: Int = elementAddrBits
  val fpStateIndexBits: Int = math.max(1, log2Ceil(fpStateEntries))
  val fpStateBankBits: Int = math.max(1, log2Ceil(fpStateBanks))
  val fpStateRowsPerBank: Int = fpStateEntries / fpStateBanks
  val loopBufferIndexBits: Int = math.max(1, log2Ceil(loopBufferEntries))
  val loopBufferCountBits: Int = math.max(1, log2Ceil(loopBufferEntries + 1))
  val loopStackIndexBits: Int = math.max(1, log2Ceil(loopStackDepth))
  val loopStackCountBits: Int = math.max(1, log2Ceil(loopStackDepth + 1))

  require(vLen > 0 && nLanes >= 2 && sfuLanes > 0,
    "The shared ALU/reduction fabric requires at least two vector lanes")
  require(computeType == VpuStorageType.FP32,
    "The v1 VPU supports FP32 computation only")
  require(vLen % nLanes == 0, "vLen must be an integer number of lane words")
  require(vectorMaskChunks <= 16,
    "C_WRITE_VMASK's four-bit chunk index supports VLEN up to 1024 elements")
  require(maskSlots >= 2 && maskSlots <= 16 && isPow2(maskSlots),
    "the vector-mask file must contain 2 to 16 power-of-two slots")
  require(storageBits >= gatherIndexBits,
    "one raw VSRAM element must be wide enough to hold a gather index")
  require(nLanes % sfuLanes == 0, "nLanes must be divisible by sfuLanes")
  require(vSpadKB > 0 && vSpadBanks > 0 && isPow2(vSpadBanks),
    "Vector SRAM must have a positive power-of-two bank count")
  require(vSpadSubBanks > 0 && isPow2(vSpadSubBanks) &&
    wordsPerBank % vSpadSubBanks == 0 && isPow2(wordsPerSubBank),
    "VSRAM sub-banks must evenly and power-of-two split each logical bank")
  require(matrixPorts >= 0,
    "the number of Gemmini matrix ports cannot be negative")
  require(matrixElementsPerRow >= nLanes &&
    matrixElementsPerRow % nLanes == 0,
    "a Gemmini matrix row must contain an integer number of VPU lane words")
  require(totalElements % matrixElementsPerRow == 0,
    "VSRAM must contain an integer number of Gemmini matrix rows")
  require(wordsPerBank % matrixWordsPerRow == 0,
    "a Gemmini matrix row must not cross a logical VSRAM bank boundary")
  if (matrixPorts > 0) {
    require(storageType == VpuStorageType.FP32 && storageBits == 32,
      "the v1 Gemmini matrix bridge transports FP32 VSRAM rows")
    require(matrixWordsPerRow <= vSpadSubBanks,
      "one matrix row needs a distinct VSRAM sub-bank per lane word")
  }
  require(!enableSharedDeps || enableGroupedCommands,
    "shared Gemmini/VPU dependencies require grouped command transport")
  require(totalBytes % storageBytes == 0)
  require(totalElements % vSpadBanks == 0)
  require(elementsPerBank % vLen == 0,
    "Each bank must contain an integer number of architectural vector rows")
  require(totalElements % nLanes == 0 && totalWords % vSpadBanks == 0)
  require(isPow2(wordsPerBank), "The v1 SRAM mapper requires power-of-two bank depth")
  require(dmaBusWidth > 0 && dmaBusWidth % 8 == 0 &&
    isPow2(dmaBusWidth / 8),
    "DMA bus width in bytes must be a positive power of two")
  require(dmaBusWidth % storageBits == 0)
  require(nLanes % dmaElementsPerBeat == 0,
    "A DMA beat must not cross a Vector SRAM lane word")
  require(dmaMaxBytes >= dmaBusWidth / 8 && isPow2(dmaMaxBytes))
  require(dmaMaxInFlight > 0 && tlbEntries > 0)
  require(loadQueueEntries > 0 && execQueueEntries > 0 && storeQueueEntries > 0)
  require(expTableEntries == 64,
    "The v1 EXP datapath implements the PLENA/Saturn-inspired 64-entry table")
  require(nLanes % reciprocalFmasPerLane == 0,
    "nLanes must divide evenly into shared reciprocal FMA groups")
  require(fmaPipeDepth == 4,
    "VpuFmaPipe has Saturn-compatible depth four (three visible cycles)")
  require(fpStateEntries > 0 && fpStateBanks > 0 &&
    isPow2(fpStateEntries) && isPow2(fpStateBanks) &&
    fpStateEntries % fpStateBanks == 0,
    "FP state SRAM entries and banks must be positive powers of two")
  require(fpStateEntries == 256 && fpStateBanks == 4,
    "The v1 FlashAttention state contract fixes 256 FP32 entries in four banks")
  require(loopBufferEntries >= 4 && isPow2(loopBufferEntries),
    "The hardware-loop command buffer must have at least four power-of-two entries")
  require(loopStackDepth > 0 && loopStackDepth <= 16 && isPow2(loopStackDepth),
    "The hardware-loop stack depth must be a power of two no greater than 16")
  require(groupIdBits == 3,
    "The grouped RoCC transport reserves rs1[34:32] for a three-bit group ID")
  /** C constants emitted from the selected hardware instance. */
  def generateHeader(): String = {
    val storageKind = storageType match {
      case VpuStorageType.FP32 => "VPU_STORAGE_FP32"
      case VpuStorageType.BF16 => "VPU_STORAGE_BF16"
    }
    val computeKind = computeType match {
      case VpuStorageType.FP32 => "VPU_STORAGE_FP32"
      case VpuStorageType.BF16 => "VPU_STORAGE_BF16"
    }
    s"""// Generated by VpuParams during Chipyard elaboration. Do not edit.
// This single header describes the most recently elaborated VPU configuration.
#ifndef GEMMINI_ROCC_TESTS_INCLUDE_VPU_PARAMS_H_
#define GEMMINI_ROCC_TESTS_INCLUDE_VPU_PARAMS_H_

#define VPU_HAS_GENERATED_PARAMS 1

#ifndef VPU_STORAGE_FP32
#define VPU_STORAGE_FP32 0
#endif
#ifndef VPU_STORAGE_BF16
#define VPU_STORAGE_BF16 1
#endif

#define VPU_VLEN ${vLen}u
#define VPU_NLANES ${nLanes}u
#define VPU_VMASK_CHUNKS ${vectorMaskChunks}u
#define VPU_VMASK_CHUNK_BITS 64u
#define VPU_MASK_SLOTS ${maskSlots}u
#define VPU_GATHER_INDEX_BITS ${gatherIndexBits}u
#define VPU_SFU_LANES ${sfuLanes}u
#define VPU_RECIPROCAL_LANES ${reciprocalLanes}u
#define VPU_RECIPROCAL_FMAS_PER_LANE ${reciprocalFmasPerLane}u
#define VPU_RECIPROCAL_LATENCY ${reciprocalLatency}u
#define VPU_VSPAD_KIB ${vSpadKB}u
#define VPU_VSPAD_BANKS ${vSpadBanks}u
#define VPU_VSPAD_SUBBANKS ${vSpadSubBanks}u
#define VPU_MATRIX_PORTS ${matrixPorts}u
#define VPU_MATRIX_ROW_ELEMENTS ${matrixElementsPerRow}u
#define VPU_MATRIX_WORDS_PER_ROW ${matrixWordsPerRow}u
#define VPU_SHARED_DEPS ${if (enableSharedDeps) 1 else 0}u
#define VPU_DMA_MAX_ROWS ${dmaMaxRows}u
#define VPU_STORAGE_KIND $storageKind
#define VPU_COMPUTE_KIND $computeKind
#define VPU_DMA_BUS_BITS ${dmaBusWidth}u
#define VPU_DMA_MAX_BYTES ${dmaMaxBytes}u
#define VPU_DMA_MAX_IN_FLIGHT ${dmaMaxInFlight}u
#define VPU_DMA_READ_MERGE_ENTRIES ${dmaReadMergeEntries}u
#define VPU_LOAD_QUEUE_ENTRIES ${loadQueueEntries}u
#define VPU_EXEC_QUEUE_ENTRIES ${execQueueEntries}u
#define VPU_STORE_QUEUE_ENTRIES ${storeQueueEntries}u
#define VPU_LOAD_RS_ENTRIES ${loadRsEntries}u
#define VPU_EXEC_RS_ENTRIES ${execRsEntries}u
#define VPU_STORE_RS_ENTRIES ${storeRsEntries}u
#define VPU_HAZARD_ENTRIES ${hazardEntries}u
#define VPU_RS_TAG_BITS ${rsTagBits}u
#define VPU_DMA_COMMAND_TAG_BITS ${dmaCommandTagBits}u
#define VPU_TLB_ENTRIES ${tlbEntries}u
#define VPU_EXP_TABLE_ENTRIES ${expTableEntries}u
#define VPU_RECIPROCAL_REFINE_ITERS ${reciprocalRefineIters}u
#define VPU_FMA_PIPE_DEPTH ${fmaPipeDepth}u
#define VPU_FP_STATE_ENTRIES ${fpStateEntries}u
#define VPU_FP_STATE_BANKS ${fpStateBanks}u
#define VPU_LOOP_BUFFER_ENTRIES ${loopBufferEntries}u
#define VPU_LOOP_STACK_DEPTH ${loopStackDepth}u
#define VPU_GROUP_ID_BITS ${groupIdBits}u
#define VPU_GROUP_ID_SHIFT 32u
#define VPU_GROUPED_SHIFT 35u
#define VPU_GROUP_LAST_SHIFT 36u
#define VPU_GROUPED_COMMANDS ${if (enableGroupedCommands) 1 else 0}u

#if VPU_STORAGE_KIND == VPU_STORAGE_FP32
#define VPU_STORAGE_BITS 32u
#elif VPU_STORAGE_KIND == VPU_STORAGE_BF16
#define VPU_STORAGE_BITS 16u
#else
#error "VPU_STORAGE_KIND must be VPU_STORAGE_FP32 or VPU_STORAGE_BF16"
#endif

#define VPU_STORAGE_BYTES (VPU_STORAGE_BITS / 8u)
#define VPU_VSPAD_BYTES (VPU_VSPAD_KIB * 1024u)
#define VPU_VSPAD_ELEMENTS (VPU_VSPAD_BYTES / VPU_STORAGE_BYTES)
#define VPU_ELEMENTS_PER_BANK (VPU_VSPAD_ELEMENTS / VPU_VSPAD_BANKS)
#define VPU_VECTOR_BYTES (VPU_VLEN * VPU_STORAGE_BYTES)
#define VPU_SLOTS_PER_BANK (VPU_ELEMENTS_PER_BANK / VPU_VLEN)

/* VSRAM addresses are element addresses, not byte addresses. */
#define VPU_BANK_BASE(bank_) ((unsigned)(bank_) * VPU_ELEMENTS_PER_BANK)
#define VPU_SLOT_ADDR(bank_, slot_)                                      \\
  (VPU_BANK_BASE(bank_) + (unsigned)(slot_) * VPU_VLEN)

/* Recommended software-managed double-buffer layout. */
#define VPU_PING_INPUT_ADDR VPU_BANK_BASE(0u)
#define VPU_PING_TEMP0_ADDR VPU_BANK_BASE(1u)
#define VPU_PING_TEMP1_ADDR VPU_BANK_BASE(2u)
#define VPU_PING_OUTPUT_ADDR VPU_BANK_BASE(3u)
#define VPU_PONG_INPUT_ADDR VPU_BANK_BASE(4u)
#define VPU_PONG_TEMP0_ADDR VPU_BANK_BASE(5u)
#define VPU_PONG_TEMP1_ADDR VPU_BANK_BASE(6u)
#define VPU_PONG_OUTPUT_ADDR VPU_BANK_BASE(7u)

#if VPU_COMPUTE_KIND != VPU_STORAGE_FP32
#error "VPU v1 compute type must be FP32"
#endif

#if (VPU_VLEN == 0) || ((VPU_VLEN % VPU_NLANES) != 0)
#error "VPU_VLEN must be a non-zero multiple of VPU_NLANES"
#endif

#if (VPU_SFU_LANES == 0) || ((VPU_NLANES % VPU_SFU_LANES) != 0)
#error "VPU_SFU_LANES must be a non-zero divisor of VPU_NLANES"
#endif

#if (VPU_RECIPROCAL_LANES == 0) || \\
    ((VPU_NLANES % VPU_RECIPROCAL_LANES) != 0) || \\
    ((VPU_RECIPROCAL_LANES * VPU_RECIPROCAL_FMAS_PER_LANE) != VPU_NLANES)
#error "VPU reciprocal lanes must exactly partition the shared FMA lanes"
#endif

#if (VPU_VSPAD_BANKS == 0) || \\
    ((VPU_VSPAD_BANKS & (VPU_VSPAD_BANKS - 1)) != 0)
#error "VPU_VSPAD_BANKS must be a power of two"
#endif

#if (VPU_VSPAD_SUBBANKS == 0) || \\
    ((VPU_VSPAD_SUBBANKS & (VPU_VSPAD_SUBBANKS - 1)) != 0)
#error "VPU_VSPAD_SUBBANKS must be a power of two"
#endif

#if ((VPU_VSPAD_BYTES % VPU_VSPAD_BANKS) != 0) || \\
    ((VPU_VSPAD_BYTES % VPU_STORAGE_BYTES) != 0)
#error "VPU scratchpad size must divide evenly into banks and elements"
#endif

#if (VPU_ELEMENTS_PER_BANK < VPU_VLEN) || \\
    ((VPU_ELEMENTS_PER_BANK % VPU_VLEN) != 0)
#error "Each VPU bank must contain an integral number of architectural vectors"
#endif

#if VPU_SLOTS_PER_BANK == 0
#error "Each VPU bank must expose at least one architectural vector slot"
#endif

#if (VPU_SLOTS_PER_BANK * VPU_VLEN) != VPU_ELEMENTS_PER_BANK
#error "VPU slot geometry must cover each bank exactly"
#endif

#if VPU_VSPAD_BANKS < 8
#error "The public ping/pong VSRAM layout requires at least eight banks"
#endif

#if VPU_FMA_PIPE_DEPTH != 4
#error "The VPU v1 software ABI requires VPU_FMA_PIPE_DEPTH=4"
#endif

#if VPU_SHARED_DEPS && !VPU_GROUPED_COMMANDS
#error "Shared Gemmini/VPU dependencies require grouped commands"
#endif

#if VPU_MATRIX_PORTS && VPU_STORAGE_KIND != VPU_STORAGE_FP32
#error "The v1 Gemmini matrix bridge requires FP32 VPU storage"
#endif

#endif  // GEMMINI_ROCC_TESTS_INCLUDE_VPU_PARAMS_H_
"""
  }

  /** Match Gemmini's root/FireSim-aware generated-header placement. */
  def headerFilePath: String = {
    val chipyardDirectory =
      "./generators/gemmini/software/gemmini-rocc-tests/include"
    val firesimDirectory =
      "../target-design/chipyard/generators/gemmini/software/gemmini-rocc-tests/include"
    val directory = Seq(chipyardDirectory, firesimDirectory)
      .find { path =>
        val file = new java.io.File(path)
        file.exists() && file.isDirectory
      }.getOrElse(".")
    s"$directory/vpu_params.h"
  }

  private def isPow2(x: Int): Boolean = x > 0 && (x & (x - 1)) == 0
}

object VpuConfigs {
  val default: VpuParams = VpuParams()
  val bf16Storage: VpuParams = VpuParams(
    storageType = VpuStorageType.BF16)
}
