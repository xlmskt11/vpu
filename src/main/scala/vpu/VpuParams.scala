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
  loadQueueEntries: Int = 4,
  execQueueEntries: Int = 8,
  storeQueueEntries: Int = 4,
  // LD/ST entries retain the configured capacities.  EX additionally retains
  // the former active-engine slot, so the reservation station requires
  // loadQueueEntries + execQueueEntries + 1 + storeQueueEntries global tags.
  hazardEntries: Int = 17,
  dmaBusWidth: Int = 128,
  dmaMaxBytes: Int = 64,
  // Per direction: reader and writer each own this many TL source IDs. The
  // shared xbar may therefore expose up to 2*dmaMaxInFlight downstream IDs.
  dmaMaxInFlight: Int = 8,
  tlbEntries: Int = 4,
  expTableEntries: Int = 64,
  reciprocalRefineIters: Int = 2,
  fmaPipeDepth: Int = 4,
  headerFileName: String = "vpu_params_generated.h") {

  val computeBits: Int = computeType.bits
  val storageBits: Int = storageType.bits
  val storageBytes: Int = storageType.bytes
  val totalBytes: Int = vSpadKB * 1024
  val totalElements: Int = totalBytes / storageBytes
  val elementsPerBank: Int = totalElements / vSpadBanks
  // A 2-D DMA command places each logical row in one architectural VLEN
  // slot.  Keeping the complete footprint inside one bank bounds both the
  // descriptor counters and the conservative reservation-station range.
  val dmaMaxRows: Int = elementsPerBank / vLen
  val wordBits: Int = nLanes * storageBits
  val wordBytes: Int = wordBits / 8
  val totalWords: Int = totalElements / nLanes
  val wordsPerBank: Int = totalWords / vSpadBanks
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
  val dmaCommandTagBits: Int = math.max(1, log2Ceil(hazardEntries))
  val vlBits: Int = math.max(1, log2Ceil(vLen + 1))
  val wordsPerVector: Int = vLen / nLanes
  val dmaRowCountBits: Int = math.max(1, log2Ceil(dmaMaxRows + 1))
  val dmaTransferElementsBits: Int =
    math.max(1, log2Ceil(elementsPerBank + 1))
  val dmaTransferWordsBits: Int =
    math.max(1, log2Ceil(dmaMaxRows * wordsPerVector + 1))
  val bankBits: Int = math.max(1, log2Ceil(vSpadBanks))
  val rowBits: Int = math.max(1, log2Ceil(wordsPerBank))
  val vectorMaskChunks: Int = (vLen + 63) / 64
  val gatherIndexBits: Int = math.max(1, log2Ceil(vLen))
  // One complete architectural vector can wait between the TileLink response
  // packer and the SRAM writer.  This is the VPU counterpart of Gemmini's
  // BeatMerger/output buffering: returning memory data no longer keeps the
  // load request generator tied to the physical SRAM write port.
  val dmaReadBufferBeats: Int = vLen / dmaElementsPerBeat
  // Scratchpad responses echo an opaque tag.  One high owner bit separates
  // execute and store traffic; the remaining bits carry an element offset.
  val spadReadTagBits: Int = elementAddrBits + 1

  require(vLen > 0 && nLanes >= 2 && sfuLanes > 0,
    "The shared ALU/reduction fabric requires at least two vector lanes")
  require(computeType == VpuStorageType.FP32,
    "The v1 VPU supports FP32 computation only")
  require(vLen % nLanes == 0, "vLen must be an integer number of lane words")
  require(vectorMaskChunks <= 16,
    "C_WRITE_VMASK's four-bit chunk index supports VLEN up to 1024 elements")
  require(storageBits >= gatherIndexBits,
    "one raw VSRAM element must be wide enough to hold a gather index")
  require(nLanes % sfuLanes == 0, "nLanes must be divisible by sfuLanes")
  require(vSpadKB > 0 && vSpadBanks > 0 && isPow2(vSpadBanks),
    "Vector SRAM must have a positive power-of-two bank count")
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
  require(vLen % dmaElementsPerBeat == 0 && dmaReadBufferBeats > 0,
    "The DMA read buffer must hold an integer number of full DMA beats")
  require(dmaMaxBytes >= dmaBusWidth / 8 && isPow2(dmaMaxBytes))
  require(dmaMaxInFlight > 0 && tlbEntries > 0)
  require(loadQueueEntries > 0 && execQueueEntries > 0 && storeQueueEntries > 0)
  require(hazardEntries >=
    loadQueueEntries + execQueueEntries + storeQueueEntries + 1,
    "Global tags must cover all LD/EX/ST reservation-station entries")
  require(expTableEntries == 64,
    "The v1 EXP datapath implements the PLENA/Saturn-inspired 64-entry table")
  require(nLanes % reciprocalFmasPerLane == 0,
    "nLanes must divide evenly into shared reciprocal FMA groups")
  require(fmaPipeDepth == 4,
    "VpuFmaPipe has Saturn-compatible depth four (three visible cycles)")
  require(headerFileName.nonEmpty && !headerFileName.contains('/'),
    "The generated VPU header name must be a plain file name")

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
    val guard = headerFileName.toUpperCase.replaceAll("[^A-Z0-9]", "_")
    s"""// Generated by VpuParams during Chipyard elaboration. Do not edit.
#ifndef ${guard}_
#define ${guard}_

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
#define VPU_GATHER_INDEX_BITS ${gatherIndexBits}u
#define VPU_SFU_LANES ${sfuLanes}u
#define VPU_RECIPROCAL_LANES ${reciprocalLanes}u
#define VPU_RECIPROCAL_FMAS_PER_LANE ${reciprocalFmasPerLane}u
#define VPU_RECIPROCAL_LATENCY ${reciprocalLatency}u
#define VPU_VSPAD_KIB ${vSpadKB}u
#define VPU_VSPAD_BANKS ${vSpadBanks}u
#define VPU_DMA_MAX_ROWS ${dmaMaxRows}u
#define VPU_STORAGE_KIND $storageKind
#define VPU_COMPUTE_KIND $computeKind
#define VPU_DMA_BUS_BITS ${dmaBusWidth}u
#define VPU_DMA_MAX_BYTES ${dmaMaxBytes}u
#define VPU_DMA_MAX_IN_FLIGHT ${dmaMaxInFlight}u
#define VPU_LOAD_QUEUE_ENTRIES ${loadQueueEntries}u
#define VPU_EXEC_QUEUE_ENTRIES ${execQueueEntries}u
#define VPU_STORE_QUEUE_ENTRIES ${storeQueueEntries}u
#define VPU_LOAD_RS_ENTRIES ${loadQueueEntries}u
#define VPU_EXEC_RS_ENTRIES ${execQueueEntries + 1}u
#define VPU_STORE_RS_ENTRIES ${storeQueueEntries}u
#define VPU_HAZARD_ENTRIES ${hazardEntries}u
#define VPU_TLB_ENTRIES ${tlbEntries}u
#define VPU_EXP_TABLE_ENTRIES ${expTableEntries}u
#define VPU_RECIPROCAL_REFINE_ITERS ${reciprocalRefineIters}u
#define VPU_FMA_PIPE_DEPTH ${fmaPipeDepth}u

#endif  // ${guard}_
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
    s"$directory/$headerFileName"
  }

  private def isPow2(x: Int): Boolean = x > 0 && (x & (x - 1)) == 0
}

object VpuConfigs {
  val default: VpuParams = VpuParams()
  val bf16Storage: VpuParams = VpuParams(
    storageType = VpuStorageType.BF16,
    headerFileName = "vpu_params_bf16_generated.h")
}
