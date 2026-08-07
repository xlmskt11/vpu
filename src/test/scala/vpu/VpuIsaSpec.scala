package vpu

import org.scalatest.flatspec.AnyFlatSpec

class VpuIsaSpec extends AnyFlatSpec {
  behavior of "VpuParams and PLENA micro-op encoding"

  it should "derive the default SRAM geometry" in {
    val p = VpuConfigs.default
    assert(p.storageBits == 32)
    assert(p.totalElements == 16384)
    assert(p.elementsPerBank == 2048)
    assert(p.wordsPerBank == 128)
    assert(p.wordsPerVector == 8)
    assert(p.dmaElementsPerBeat == 4)
    assert(p.vectorMaskChunks == 2)
    assert(p.maskSlots == 4)
    assert(p.maskSlotBits == 2)
    assert(p.gatherIndexBits == 7)
    assert(p.fpStateEntries == 256)
    assert(p.fpStateBanks == 4)
    assert(p.loopBufferEntries == 64)
    assert(p.loopStackDepth == 4)
    assert(p.groupIdBits == 3)
    assert(p.generateHeader().contains("#define VPU_MASK_SLOTS 4u"))
  }

  it should "use BF16 (8,8) storage geometry" in {
    val p = VpuConfigs.bf16Storage
    assert(p.storageBits == 16)
    assert(p.totalElements == 32768)
    assert(p.dmaElementsPerBeat == 8)
  }

  it should "pack every instruction field in the PLENA positions" in {
    val word = VpuEncoding.pack(
      VpuOpcode.V_SUB_VF, rd = 1, rs1 = 2, rs2 = 3, rs3 = 4, funct1 = 1)
    assert((word & 0x3f) == VpuOpcode.V_SUB_VF)
    assert(((word >> 6) & 0xf) == 1)
    assert(((word >> 10) & 0xf) == 2)
    assert(((word >> 14) & 0xf) == 3)
    assert(((word >> 18) & 0xf) == 4)
    assert(((word >> 22) & 0xf) == 1)
    assert((word >>> 26) == 0)
  }

  it should "reserve the agreed control and status ABI" in {
    assert(VpuOpcode.S_ADDI_INT == 0x22)
    assert(VpuOpcode.C_LOOP_START == 0x2f)
    assert(VpuOpcode.C_LOOP_END == 0x30)
    assert(VpuOpcode.C_WRITE_GP == 0x38)
    assert(VpuOpcode.S_LOAD_STATE == 0x1e)
    assert(VpuOpcode.S_STORE_STATE == 0x1f)
    assert(VpuOpcode.C_WRITE_VMASK == 0x2e)
    assert(VpuOpcode.V_GATHER_VV == 0x31)
    assert(VpuOpcode.V_SLIDE_V == 0x32)
    assert(VpuOpcode.C_CLEAR_STATUS == 0x3f)
    assert(VpuReadSelector.Perf == 4)
    assert(VpuReadSelector.FaultAddress == 5)
    assert(VpuReadSelector.FaultInfo == 6)
    assert(VpuDmaFaultCause.Translation == 1)
    assert(VpuDmaFaultCause.Access == 2)
    assert(VpuDmaFaultCause.Protocol == 3)
    assert(VpuPerfIndex.DmaExecuteOverlapCycles == 4)
    assert(VpuClearMask.Fflags == 0x1)
    assert(VpuClearMask.FaultIllegal == 0x2)
    assert(VpuClearMask.Perf == 0x4)
    assert(VpuClearMask.Errors == 0x3)
    assert(VpuClearMask.All == 0x7)
    assert(VpuStatusLayout.DmaHalted == 2)
    assert(VpuStatusLayout.FflagsLo == 8)
    assert(VpuStatusLayout.Busy == 16)
  }

  it should "pack PLENA-compatible integer induction and loop formats" in {
    val addi = VpuEncoding.packAddiInt(rd = 7, rs1 = 3,
      immediate = 0x2aaaa)
    assert((addi & 0x3f) == VpuOpcode.S_ADDI_INT)
    assert(((addi >>> 6) & 0xf) == 7)
    assert(((addi >>> 10) & 0xf) == 3)
    assert((addi >>> 14) == 0x2aaaa)

    val start = VpuEncoding.packLoopStart(loopRegister = 12,
      iterations = 0x2abcde)
    assert((start & 0x3f) == VpuOpcode.C_LOOP_START)
    assert(((start >>> 6) & 0xf) == 12)
    assert((start >>> 10) == 0x2abcde)

    val end = VpuEncoding.packLoopEnd(loopRegister = 12)
    assert((end & 0x3f) == VpuOpcode.C_LOOP_END)
    assert(((end >>> 6) & 0xf) == 12)
    assert((end >>> 10) == 0)
  }
}
