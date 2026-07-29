package vpu

import org.chipsalliance.cde.config.{Config, Parameters}

import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.tile.BuildRoCC

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
