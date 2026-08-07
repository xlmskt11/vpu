package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuFpStateSramTester(c: VpuFpStateSram, p: VpuParams)
    extends PeekPokeTester(c) {
  poke(c.io.read.valid, 0)
  poke(c.io.write.valid, 0)
  step(2)

  private def write(index: Int, data: BigInt): Unit = {
    poke(c.io.write.valid, 1)
    poke(c.io.write.bits.index, index)
    poke(c.io.write.bits.data, data)
    expect(c.io.write.ready, 1)
    step(1)
    poke(c.io.write.valid, 0)
  }

  private def read(index: Int, expected: BigInt): Unit = {
    poke(c.io.read.valid, 1)
    poke(c.io.read.bits.index, index)
    expect(c.io.read.ready, 1)
    step(1)
    poke(c.io.read.valid, 0)
    expect(c.io.readData.valid, 1)
    expect(c.io.readData.bits, expected)
    step(1)
    expect(c.io.readData.valid, 0)
  }

  // Low index bits select the bank, so these cover all four banks and two
  // different rows without depending on SRAM reset contents.
  val samples = Seq(
    0 -> BigInt("3f800000", 16),
    1 -> BigInt("40000000", 16),
    2 -> BigInt("40400000", 16),
    3 -> BigInt("40800000", 16),
    68 -> BigInt("bf000000", 16),
    255 -> BigInt("7f7fffff", 16))
  samples.foreach { case (index, data) => write(index, data) }
  samples.reverse.foreach { case (index, data) => read(index, data) }

  assert(p.fpStateEntries == 256)
  assert(p.fpStateBanks == 4)
}

class VpuFpStateSramSpec extends ChiselFlatSpec {
  behavior of "VpuFpStateSram"

  it should "preserve FP32 state across four 1R1W banks" in {
    val p = VpuConfigs.default
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-fp-state-sram"),
      () => new VpuFpStateSram(p)) { c =>
      new VpuFpStateSramTester(c, p)
    } should be(true)
  }
}
