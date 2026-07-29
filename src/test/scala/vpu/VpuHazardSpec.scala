package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuHazardTester(c: VpuHazardTracker) extends PeekPokeTester(c) {
  poke(c.io.probe.valid, 0)
  poke(c.io.allocate.valid, 0)
  poke(c.io.release.valid, 0)
  step(2)

  def range(base: Int, count: Int, read: Boolean, write: Boolean): Unit = {
    poke(c.io.probe.bits.base, base)
    poke(c.io.probe.bits.elementCount, count)
    poke(c.io.probe.bits.read, read)
    poke(c.io.probe.bits.write, write)
    poke(c.io.probe.bits.tag, 0)
    poke(c.io.allocate.bits.base, base)
    poke(c.io.allocate.bits.elementCount, count)
    poke(c.io.allocate.bits.read, read)
    poke(c.io.allocate.bits.write, write)
    poke(c.io.allocate.bits.tag, 0)
  }

  range(0, 16, read = false, write = true)
  poke(c.io.allocate.valid, 1)
  assert(peek(c.io.allocate.ready) == 1)
  val tag = peek(c.io.allocatedTag)
  step(1)
  poke(c.io.allocate.valid, 0)

  range(8, 4, read = true, write = false)
  poke(c.io.probe.valid, 1)
  assert(peek(c.io.conflict) == 1) // RAW
  poke(c.io.allocate.valid, 1)
  assert(peek(c.io.allocate.ready) == 0)

  range(8, 4, read = false, write = true)
  assert(peek(c.io.conflict) == 1) // WAW
  assert(peek(c.io.allocate.ready) == 0)

  range(32, 16, read = true, write = false)
  assert(peek(c.io.conflict) == 0)
  assert(peek(c.io.allocate.ready) == 1)
  poke(c.io.allocate.valid, 0)
  poke(c.io.probe.valid, 0)

  poke(c.io.release.valid, 1)
  poke(c.io.release.bits, tag)
  step(1)
  poke(c.io.release.valid, 0)
  assert(peek(c.io.empty) == 1)

  // A resident reader allows another reader, but blocks an overlapping
  // writer until it is released (WAR).
  range(40, 16, read = true, write = false)
  poke(c.io.allocate.valid, 1)
  assert(peek(c.io.allocate.ready) == 1)
  val readTag = peek(c.io.allocatedTag)
  step(1)
  poke(c.io.allocate.valid, 0)

  range(44, 4, read = true, write = false)
  poke(c.io.probe.valid, 1)
  assert(peek(c.io.conflict) == 0)

  range(44, 4, read = false, write = true)
  assert(peek(c.io.conflict) == 1) // WAR
  poke(c.io.allocate.valid, 1)
  assert(peek(c.io.allocate.ready) == 0)
  poke(c.io.allocate.valid, 0)
  poke(c.io.probe.valid, 0)

  poke(c.io.release.valid, 1)
  poke(c.io.release.bits, readTag)
  step(1)
  poke(c.io.release.valid, 0)
  assert(peek(c.io.empty) == 1)
}

class VpuHazardSpec extends ChiselFlatSpec {
  behavior of "VpuHazardTracker"
  it should "detect RAW, WAR, and WAW range overlap before allocation" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-hazard"), () => new VpuHazardTracker(p)) {
      c => new VpuHazardTester(c)
    } should be (true)
  }
}
