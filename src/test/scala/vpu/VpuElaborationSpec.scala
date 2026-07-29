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

    val bf16 = VpuConfigs.bf16Storage.generateHeader()
    assert(bf16.contains("#define VPU_STORAGE_BF16 1"))
    assert(bf16.contains("#define VPU_STORAGE_KIND VPU_STORAGE_BF16"))
    assert(VpuConfigs.default.headerFileName == "vpu_params_generated.h")
    assert(VpuConfigs.bf16Storage.headerFileName ==
      "vpu_params_bf16_generated.h")
    assert(VpuConfigs.default.headerFilePath !=
      VpuConfigs.bf16Storage.headerFilePath)
    assert(fp32.contains("#ifndef VPU_PARAMS_GENERATED_H_"))
    assert(bf16.contains("#ifndef VPU_PARAMS_BF16_GENERATED_H_"))
    assert(VpuConfigs.default.reciprocalLanes == 4)
    assert(VpuConfigs.default.reciprocalLatency == 13)
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
