package vpu

import chisel3._
import org.chipsalliance.cde.config.{Config, Parameters}

import freechips.rocketchip.diplomacy.{InModuleBody, LazyModule, ValName}
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet, RoCCCommandRoute}

import gemmini._
import gemmini.Arithmetic.FloatArithmetic._

private object FusionGemminiConfig {
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
      meshRows = 16,
      meshColumns = 16,
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

/** A single coherent 4x16 Gemmini + VPU cluster.
  *
  * The fifth BuildRoCC entry is deliberately the VPU so Gemmini broadcast
  * indices remain 0..3.  All cross-accelerator wires are created only after
  * those five LazyModules exist, inside the VPU factory's InModuleBody.
  * Gemmini0--3 and the VPU retain their original RoCC command paths: VPU
  * funct 64 is an exact custom0 route, Gemmini0 is the wildcard custom0
  * fallback, and Gemmini1--3 retain custom1--3.
  */
class WithGemminiVpuFusion(
    gemminiBase: GemminiArrayConfig[Float, Float, Float] =
      FusionGemminiConfig.base,
    vpuBase: VpuParams = VpuParams(
      storageType = VpuStorageType.FP32,
      computeType = VpuStorageType.FP32,
      vLen = 128,
      nLanes = 16,
      sfuLanes = 4,
      vSpadKB = 256,
      vSpadBanks = 8,
      vSpadSubBanks = 4,
      matrixPorts = 4,
      enableSharedDeps = true,
      enableGroupedCommands = true))
    extends Config((site, here, up) => {
  case BuildRoCC => {
    val gemminis = Array.ofDim[Gemmini[Float, Float, Float]](4)
    var fusedVpu: VpuRoCC = null

    def gemminiFactory(index: Int) = (p: Parameters) => {
      implicit val q: Parameters = p
      implicit val valName: ValName = ValName(s"gemmini$index")
      val opcode = index match {
        case 0 => OpcodeSet.custom0
        case 1 => OpcodeSet.custom1
        case 2 => OpcodeSet.custom2
        case 3 => OpcodeSet.custom3
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
        nSharers = 4,
        use_shared_ext_mem = true,
        use_shared_res_entries = true,
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
          "all four Gemminis must be constructed before fusion wiring")
        require(vpuBase.matrixPorts == 4 &&
          vpuBase.enableGroupedCommands && vpuBase.enableSharedDeps)
        require(vpuBase.nLanes == g0.config.meshColumns *
          g0.config.tileColumns)
        require(vpuBase.storageBits == g0.config.accType.getWidth)
        require(vpuBase.totalWords ==
          g0.config.acc_banks * g0.config.acc_bank_entries,
          "VSRAM and shared ACC must expose the same global row range")
        require(vpuBase.vSpadBanks >= vpuBase.matrixPorts,
          "each simultaneous Gemmini matrix port needs a VSRAM bank")
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

        val sharedMem = Module(new SharedExtMem_4(g0.config))
        for (index <- 0 until 4) {
          sharedMem.io.in(index) <> gemminis(index).module.ext_mem_io.get
          fusedVpu.module.matrixRead.get(index) <>
            gemminis(index).module.vpu_matrix_read_io.get
          fusedVpu.module.matrixWrite.get(index) <>
            sharedMem.io.vpuWrite.get(index)
        }

        // Preserve Gemmini's original SPAD/ACC dependency tracker.
        val spadAccDeps = Module(new SharedExtEntries(
          g0.config.nSharers,
          g0.config.local_addr_t,
          g0.config.reservation_station_entries_ld,
          g0.config.reservation_station_entries_ex,
          g0.config.reservation_station_entries_st,
          g0.config.res_max_per_type))
        for (index <- 0 until 4) {
          spadAccDeps.io.in(index) <>
            gemminis(index).module.ext_deps_io.get
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
        for (index <- 0 until 4) {
          vsramDeps.io.in(index) <>
            gemminis(index).module.vsram_deps_io.get
        }
        vsramDeps.io.vpu <> fusedVpu.module.vsramDeps.get

        val groupControl = Module(new LdBCompleteControl(
          nSharers = 4,
          useVpuFusion = g0.config.use_vpu_fusion))
        for (index <- 0 until 4) {
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
        val inputGroupControl = Module(new LdICompleteControl(4))
        for (index <- 0 until 4) {
          inputGroupControl.io.in(index) <>
            gemminis(index).module.ext_loop_conv_ws_io.get
        }
      }
      fusedVpu
    }

    up(BuildRoCC) ++ (0 until 4).map(gemminiFactory) :+ vpuFactory
  }
})
