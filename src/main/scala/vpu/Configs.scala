package vpu

import chisel3._
import org.chipsalliance.cde.config.{Config, Parameters}

import freechips.rocketchip.diplomacy.{InModuleBody, LazyModule, ValName}
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet, RoCCCommandRoute}

import gemmini._
import gemmini.Arithmetic.FloatArithmetic._

private object FusionGemminiConfig {
  // BF16 doubles the operand bytes relative to the original INT8 datapath.
  // Match that traffic increase with a 256-bit DMA datapath while retaining
  // the existing 64-byte cache-line transaction size.
  val dmaBusWidth = 256

  /** BF16 operands enter a WS array while vertical partial sums and the
    * accumulator RMW path remain FP32.  Keep this integration-only preset in
    * the VPU project so ordinary Gemmini configuration sources stay intact.
    */
  val base: GemminiArrayConfig[Float, Float, Float] =
    GemminiFPConfigs.defaultFPConfig.copy(
      inputType = Float(8, 8),
      spatialArrayOutputType = Float(8, 24),
      accType = Float(8, 24),
      tileRows = 1,
      tileColumns = 1,
      meshRows = 8,
      meshColumns = 8,
      dataflow = Dataflow.WS,
      tile_latency = 2,
      mesh_output_delay = 1,
      sp_capacity = CapacityInKilobytes(256),
      acc_capacity = CapacityInKilobytes(256),
      sp_banks = 4,
      sp_sub_banks = 4,
      sp_singleported = true,
      acc_banks = 2,
      acc_sub_banks = 4,
      acc_singleported = false,
      acc_latency = 4,
      dma_buswidth = dmaBusWidth,
      max_in_flight_mem_reqs = 16,
      use_shared_ext_mem = true,
      use_shared_res_entries = true,
      use_vpu_fusion = true,
      nSharers = 4,
      ex_read_from_acc = false,
      ex_write_to_spad = false,
      ex_write_to_acc = true,
      hardcode_d_to_garbage_addr = true,
      has_training_convs = false,
      has_max_pool = false,
      has_nonlinear_activations = false,
      headerFileName = "gemmini_params.h")
}

/** Append one exact-funct standalone VPU after accelerators already present in
  * BuildRoCC.  In the 4-Gemmini comparison config this preserves ports 0..3
  * for Gemmini broadcast and makes the VPU port 4.
  */
class WithVpu(params: VpuParams = VpuConfigs.default)
    extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q: Parameters = p
      val vpu = LazyModule(new VpuRoCC(params))
      vpu
    })
})

class WithBf16StorageVpu extends WithVpu(VpuConfigs.bf16Storage)

/** One coherent Gemmini + VPU cluster with one to four Gemmini endpoints.
  *
  * Gemmini endpoints retain broadcast indices 0..nGemminis-1. The VPU is the
  * final BuildRoCC entry, with funct 64 as an exact custom0 route; Gemmini0 is
  * the wildcard custom0 fallback and additional Gemminis use custom1--3.
  *
  * A multi-Gemmini cluster externalizes SPAD/ACC and their dependency table so
  * the members can share them. A physically single-Gemmini cluster keeps its
  * ordinary local SPAD/ACC and local reservation dependencies; only the VSRAM
  * bridge, VSRAM dependency side table, and fusion group controller are shared
  * with the VPU.
  */
class WithGemminiVpuFusion(
    gemminiBase: GemminiArrayConfig[Float, Float, Float] =
      FusionGemminiConfig.base,
    vpuBase: VpuParams = VpuParams(
      storageType = VpuStorageType.FP32,
      computeType = VpuStorageType.FP32,
      vLen = 128,
      nLanes = 8,
      sfuLanes = 4,
      vSpadKB = 256,
      vSpadBanks = 8,
      vSpadSubBanks = 4,
      dmaBusWidth = FusionGemminiConfig.dmaBusWidth,
      matrixPorts = 4,
      matrixRowElements = 8,
      enableSharedDeps = true,
      enableGroupedCommands = true),
    nGemminis: Int = 4)
    extends Config((site, here, up) => {
  case BuildRoCC => {
    require(nGemminis >= 1 && nGemminis <= 4,
      "Gemmini/VPU fusion supports one to four RoCC Gemmini endpoints")

    val shareGemminiMem = nGemminis > 1
    val gemminis = Array.ofDim[Gemmini[Float, Float, Float]](nGemminis)
    var fusedVpu: VpuRoCC = null

    def gemminiFactory(index: Int) = (p: Parameters) => {
      implicit val q: Parameters = p
      implicit val valName: ValName = ValName(s"gemmini$index")
      val opcode = if (nGemminis == 1) {
        OpcodeSet.custom3
      } else {
        index match {
          case 0 => OpcodeSet.custom0
          case 1 => OpcodeSet.custom1
          case 2 => OpcodeSet.custom2
          case 3 => OpcodeSet.custom3
        }
      }
      val header = index match {
        case 0 => "gemmini_params.h"
        case 1 => "gemmini_op1_params.h"
        case 2 => "gemmini_op2_params.h"
        case 3 => "gemmini_op3_params.h"
      }
      gemminis(index) = LazyModule(new Gemmini(gemminiBase.copy(
        opcodes = opcode,
        headerFileName = header,
        nSharers = nGemminis,
        use_shared_ext_mem = shareGemminiMem,
        use_shared_res_entries = shareGemminiMem,
        use_vpu_fusion = true),
        commandRoute = RoCCCommandRoute(broadcastIndex = Some(index))))
      gemminis(index)
    }

    val vpuFactory = (p: Parameters) => {
      implicit val q: Parameters = p
      implicit val valName: ValName = ValName("fusedVpu")
      fusedVpu = LazyModule(new VpuRoCC(vpuBase))

      InModuleBody {
        val g0 = gemminis(0)
        require(gemminis.forall(_ != null),
          "all Gemminis must be constructed before fusion wiring")
        require(vpuBase.matrixPorts == nGemminis &&
          vpuBase.enableGroupedCommands && vpuBase.enableSharedDeps)
        require(vpuBase.matrixElementsPerRow == g0.config.meshColumns *
          g0.config.tileColumns)
        require(vpuBase.storageBits == g0.config.accType.getWidth)
        require(vpuBase.matrixRows ==
          g0.config.acc_banks * g0.config.acc_bank_entries,
          "VSRAM and shared ACC must expose the same global row range")
        require(vpuBase.physicalBanks >=
          vpuBase.matrixPorts * vpuBase.matrixWordsPerRow,
          "simultaneous Gemmini matrix rows need enough physical VSRAM banks")
        if (shareGemminiMem) {
          // SharedExtMem may accept one C_TO_VSRAM row from every distinct
          // physical ACC bank/sub-bank in the same cycle. Preserve that
          // distribution in VSRAM: one ACC sub-bank owns a disjoint group of
          // matrixWordsPerRow physical VSRAM banks, and each logical VSRAM
          // bank is wholly contained in one ACC bank's global-row range.
          val physicalBanksPerAccSubBank = vpuBase.matrixWordsPerRow
          require(vpuBase.vSpadSubBanks %
            (g0.config.acc_sub_banks * physicalBanksPerAccSubBank) == 0,
            "multi-Gemmini matrix writes require disjoint VSRAM sub-bank " +
              "groups for every ACC sub-bank")
          require(vpuBase.vSpadBanks % g0.config.acc_banks == 0,
            "VSRAM logical-bank boundaries must subdivide ACC bank ranges")
        }
        require(vpuBase.sharedHazardAddressBits ==
          g0.config.local_addr_t.data.getWidth,
          "Gemmini and VPU shared dependency address widths differ")

        gemminis.tail.foreach { g =>
          require(g.config.sp_banks == g0.config.sp_banks &&
            g.config.sp_sub_banks == g0.config.sp_sub_banks &&
            g.config.sp_bank_entries == g0.config.sp_bank_entries)
          require(g.config.acc_banks == g0.config.acc_banks &&
            g.config.acc_sub_banks == g0.config.acc_sub_banks &&
            g.config.acc_bank_entries == g0.config.acc_bank_entries)
        }

        val sharedMem = if (shareGemminiMem) {
          Some(Module(new SharedExtMem_4(g0.config)))
        } else {
          None
        }
        for (index <- 0 until nGemminis) {
          sharedMem.foreach { mem =>
            mem.io.in(index) <> gemminis(index).module.ext_mem_io.get
          }
          fusedVpu.module.matrixRead.get(index) <>
            gemminis(index).module.vpu_matrix_read_io.get
          if (shareGemminiMem) {
            fusedVpu.module.matrixWrite.get(index) <>
              sharedMem.get.io.vpuWrite.get(index)
          } else {
            fusedVpu.module.matrixWrite.get(index) <>
              gemminis(index).module.vpu_matrix_write_io.get
          }
        }

        // Multi-Gemmini builds preserve the shared SPAD/ACC dependency table.
        // One Gemmini uses its ordinary local reservation dependencies.
        if (shareGemminiMem) {
          val spadAccDeps = Module(new SharedExtEntries(
            g0.config.nSharers,
            g0.config.local_addr_t,
            g0.config.reservation_station_entries_ld,
            g0.config.reservation_station_entries_ex,
            g0.config.reservation_station_entries_st,
            g0.config.res_max_per_type))
          for (index <- 0 until nGemminis) {
            spadAccDeps.io.in(index) <>
              gemminis(index).module.ext_deps_io.get
          }
        }

        // Only cross-accelerator VSRAM hazards need the fusion side table.
        val vsramDeps = Module(new VsramExtEntries(
          g0.config.nSharers,
          g0.config.local_addr_t,
          g0.config.reservation_station_entries_ld,
          g0.config.reservation_station_entries_ex,
          g0.config.reservation_station_entries_st,
          g0.config.res_max_per_type,
          vpuEntries = VpuReservationStation.totalEntries(vpuBase)))
        for (index <- 0 until nGemminis) {
          vsramDeps.io.in(index) <>
            gemminis(index).module.vsram_deps_io.get
        }
        vsramDeps.io.vpu <> fusedVpu.module.vsramDeps.get

        val groupControl = Module(new LdBCompleteControl(
          nSharers = nGemminis,
          useVpuFusion = g0.config.use_vpu_fusion))
        for (index <- 0 until nGemminis) {
          groupControl.io.in(index) <>
            gemminis(index).module.ext_loop_ws_io.get
        }
        groupControl.io.gemv.get.pending :=
          fusedVpu.module.gemvGroup.get.pending
        groupControl.io.gemv.get.group_id :=
          fusedVpu.module.gemvGroup.get.groupId
        groupControl.io.gemv.get.last_dispatch_fire :=
          fusedVpu.module.gemvGroup.get.lastDispatchFire
        groupControl.io.gemv.get.abort :=
          fusedVpu.module.gemvGroup.get.abort
        fusedVpu.module.gemvGroup.get.groupAllocated :=
          groupControl.io.gemv.get.group_allocated
        fusedVpu.module.gemvGroup.get.dispatchEnable :=
          groupControl.io.gemv.get.dispatch_enable
        fusedVpu.module.gemvGroup.get.dispatchReject :=
          groupControl.io.gemv.get.dispatch_reject

        // Shared-reservation Gemmini still exposes the convolution group
        // interface. Fusion does not alter it, so preserve the legacy four-way
        // completion controller even though this inference config disables
        // training convolutions.
        if (shareGemminiMem) {
          val inputGroupControl = Module(new LdICompleteControl(nGemminis))
          for (index <- 0 until nGemminis) {
            inputGroupControl.io.in(index) <>
              gemminis(index).module.ext_loop_conv_ws_io.get
          }
        }
      }
      fusedVpu
    }

    up(BuildRoCC) ++ (0 until nGemminis).map(gemminiFactory) :+ vpuFactory
  }
})

/** One physical 16x16 BF16-input/FP32-accumulate Gemmini fused to one FP32
  * VPU. Unlike the four-Gemmini configuration, Gemmini keeps local SPAD/ACC
  * storage and its local reservation dependencies.
  */
class WithSingle16x16GemminiVpuFusion extends WithGemminiVpuFusion(
  gemminiBase = FusionGemminiConfig.base.copy(
    meshRows = 16,
    meshColumns = 16,
    sp_capacity = CapacityInKilobytes(256),
    acc_capacity = CapacityInKilobytes(256),
    n_dma_engines = 4,
    max_in_flight_mem_reqs = 16),
  vpuBase = VpuParams(
    storageType = VpuStorageType.FP32,
    computeType = VpuStorageType.FP32,
    vLen = 128,
    nLanes = 8,
    sfuLanes = 4,
    vSpadKB = 256,
    vSpadBanks = 8,
    vSpadSubBanks = 2,
    dmaBusWidth = FusionGemminiConfig.dmaBusWidth,
    matrixPorts = 1,
    matrixRowElements = 16,
    enableSharedDeps = true,
    enableGroupedCommands = true),
  nGemminis = 1)
