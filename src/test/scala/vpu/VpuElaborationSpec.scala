package vpu

import chisel3.stage.ChiselStage
import org.scalatest.flatspec.AnyFlatSpec

class VpuElaborationSpec extends AnyFlatSpec {
  behavior of "VpuCore parameterization"

  it should "elaborate the advertised FP32 and BF16 geometry sweep" in {
    val variants = for {
      storage <- Seq(VpuStorageType.FP32, VpuStorageType.BF16)
      vLen <- Seq(64, 128, 256)
      nLanes <- Seq(8, 16)
      sfuLanes <- Seq(2, 4)
    } yield VpuParams(
      storageType = storage,
      vLen = vLen,
      nLanes = nLanes,
      sfuLanes = sfuLanes)

    assert(variants.size == 24)
    variants.foreach { p =>
      val chirrtl = (new ChiselStage).emitChirrtl(new VpuCore(p))
      withClue(s"$p: ") {
        assert(chirrtl.contains("module VpuCore"))
        assert(chirrtl.contains("module VpuBankedScratchpad"))
      }
    }
  }

  it should "reject geometries outside the public v1 contract" in {
    assertThrows[IllegalArgumentException](VpuParams(
      computeType = VpuStorageType.BF16))
    assertThrows[IllegalArgumentException](VpuParams(vLen = 127))
    assertThrows[IllegalArgumentException](VpuParams(
      nLanes = 8, sfuLanes = 3))
    assertThrows[IllegalArgumentException](VpuParams(
      nLanes = 1, sfuLanes = 1))
    assertThrows[IllegalArgumentException](VpuParams(
      vLen = 24, nLanes = 6, sfuLanes = 2))
    assertThrows[IllegalArgumentException](VpuParams(vSpadBanks = 6))
    assertThrows[IllegalArgumentException](VpuParams(
      vLen = 96, nLanes = 12, sfuLanes = 3, vSpadKB = 3,
      dmaBusWidth = 96))
    assertThrows[IllegalArgumentException](VpuParams(fmaPipeDepth = 3))
    assertThrows[IllegalArgumentException](VpuParams(loopBufferEntries = 3))
    assertThrows[IllegalArgumentException](VpuParams(loopStackDepth = 3))
  }

  it should "generate exact FP32 and BF16 software parameter headers" in {
    val fp32 = VpuConfigs.default.generateHeader()
    assert(fp32.contains("#define VPU_VLEN 128u"))
    assert(fp32.contains("#define VPU_NLANES 16u"))
    assert(fp32.contains("#define VPU_RECIPROCAL_LANES 4u"))
    assert(fp32.contains("#define VPU_RECIPROCAL_FMAS_PER_LANE 4u"))
    assert(fp32.contains("#define VPU_RECIPROCAL_LATENCY 13u"))
    assert(fp32.contains("#define VPU_STORAGE_FP32 0"))
    assert(fp32.contains("#define VPU_STORAGE_KIND VPU_STORAGE_FP32"))
    assert(fp32.contains("#define VPU_COMPUTE_KIND VPU_STORAGE_FP32"))
    assert(fp32.contains("#define VPU_FMA_PIPE_DEPTH 4u"))
    assert(VpuConfigs.default.dmaReadMergeEntries == 18)
    assert(fp32.contains("#define VPU_DMA_READ_MERGE_ENTRIES 18u"))
    assert(VpuConfigs.default.rsTagBits == 5)
    assert(VpuConfigs.default.dmaCommandTagBits == 2)
    assert(fp32.contains("#define VPU_RS_TAG_BITS 5u"))
    assert(fp32.contains("#define VPU_DMA_COMMAND_TAG_BITS 2u"))
    assert(fp32.contains("#define VPU_LOOP_BUFFER_ENTRIES 64u"))
    assert(fp32.contains("#define VPU_LOOP_STACK_DEPTH 4u"))
    assert(!fp32.contains("VPU_STREAM_CHUNK_ROWS"))
    assert(!fp32.contains("VPU_NONLINEAR_CHUNK_ROWS"))
    assert(fp32.contains("#define VPU_STORAGE_BYTES (VPU_STORAGE_BITS / 8u)"))
    assert(fp32.contains("#define VPU_ELEMENTS_PER_BANK"))
    assert(fp32.contains("#define VPU_SLOT_ADDR(bank_, slot_)"))

    val bf16 = VpuConfigs.bf16Storage.generateHeader()
    assert(bf16.contains("#define VPU_STORAGE_BF16 1"))
    assert(bf16.contains("#define VPU_STORAGE_KIND VPU_STORAGE_BF16"))
    assert(VpuConfigs.default.headerFilePath ==
      VpuConfigs.bf16Storage.headerFilePath)
    assert(VpuConfigs.default.headerFilePath.endsWith("/vpu_params.h"))
    assert(fp32.contains(
      "#ifndef GEMMINI_ROCC_TESTS_INCLUDE_VPU_PARAMS_H_"))
    assert(bf16.contains(
      "#ifndef GEMMINI_ROCC_TESTS_INCLUDE_VPU_PARAMS_H_"))

    val fusion = VpuConfigs.default.copy(
      vSpadKB = 128, vSpadSubBanks = 4, matrixPorts = 4,
      enableSharedDeps = true, enableGroupedCommands = true)
    val fusionHeader = fusion.generateHeader()
    assert(fusionHeader.contains("#define VPU_VSPAD_KIB 128u"))
    assert(fusionHeader.contains("#define VPU_VSPAD_SUBBANKS 4u"))
    assert(fusionHeader.contains("#define VPU_MATRIX_PORTS 4u"))
    assert(fusionHeader.contains("#define VPU_SHARED_DEPS 1u"))
    assert(fusionHeader.contains("#define VPU_GROUPED_COMMANDS 1u"))
    assert(fusion.headerFilePath == VpuConfigs.default.headerFilePath)
    assert(VpuConfigs.default.reciprocalLanes == 4)
    assert(VpuConfigs.default.reciprocalLatency == 13)

    val resizedRs = VpuConfigs.default.copy(
      loadQueueEntries = 2, execQueueEntries = 3, storeQueueEntries = 5)
    assert(resizedRs.loadRsEntries == 2)
    assert(resizedRs.execRsEntries == 4)
    assert(resizedRs.storeRsEntries == 5)
    assert(resizedRs.hazardEntries == 11)
    assert(resizedRs.generateHeader().contains(
      "#define VPU_HAZARD_ENTRIES 11u"))
  }


  it should "elaborate the configurable loop command buffer and stack" in {
    val p = VpuConfigs.default.copy(
      loopBufferEntries = 128, loopStackDepth = 8)
    val chirrtl = (new ChiselStage).emitChirrtl(new VpuHardwareLoop(
      groupIdBits = p.groupIdBits,
      enableGroupedCommands = p.enableGroupedCommands,
      bufferEntries = p.loopBufferEntries,
      maxDepth = p.loopStackDepth))
    assert(chirrtl.contains("module VpuHardwareLoop"))
    assert(!chirrtl.contains("captureGrouped"))
    assert(!chirrtl.contains("captureGroupId"))
    assert(!chirrtl.contains("drainEmitTerminator"))
    assert(p.generateHeader().contains(
      "#define VPU_LOOP_BUFFER_ENTRIES 128u"))
    assert(p.generateHeader().contains("#define VPU_LOOP_STACK_DEPTH 8u"))
  }

  it should "elaborate reciprocal arithmetic only inside the shared fabric" in {
    val chirrtl = (new ChiselStage).emitChirrtl(new VpuCore(VpuConfigs.default))
    assert(!chirrtl.contains("VpuLegacyReciprocalPipe"))
    assert(!chirrtl.contains("VpuReciprocalPipe"))
    def countFmaInstances(text: String): Int =
      """(?m)^\s*inst\s+\S+\s+of\s+VpuFmaPipe(?:_[0-9]+)?\b""".r
        .findAllMatchIn(text).size
    // Sixteen physical vector lanes serve elementwise, reduction, and
    // reciprocal modes.  The only additional instance is the scalar ALU;
    // reciprocal must not elaborate another private FMA array.
    assert(countFmaInstances(chirrtl) == VpuConfigs.default.nLanes + 1)

    // Keep the requested DIM-style scaling contract explicit: doubling the
    // shared fabric doubles reciprocal throughput without adding a private
    // reciprocal FMA array.
    val lanes32 = VpuParams(nLanes = 32)
    assert(lanes32.reciprocalLanes == 8)
    val chirrtl32 = (new ChiselStage).emitChirrtl(new VpuCore(lanes32))
    assert(countFmaInstances(chirrtl32) == lanes32.nLanes + 1)
  }
}
