package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuStoreTransportTagAllocatorTester(
    c: VpuStoreTransportTagAllocator,
    capacity: Int) extends PeekPokeTester(c) {

  poke(c.io.allocate.ready, 0)
  poke(c.io.release.valid, 0)
  poke(c.io.release.bits, 0)
  step(2)

  assert(peek(c.io.busy) == 0)
  assert(peek(c.io.full) == 0)
  assert(peek(c.io.outstanding) == 0)
  assert(peek(c.io.available) == capacity)
  assert(peek(c.io.capacity) == capacity)

  // Allocate every slot and verify that no live identity is duplicated.
  poke(c.io.allocate.ready, 1)
  val allocated = (0 until capacity).map { expectedCount =>
    assert(peek(c.io.allocate.valid) == 1)
    val tag = peek(c.io.allocate.bits).toInt
    assert(tag >= 0 && tag < capacity)
    step(1)
    assert(peek(c.io.outstanding) == expectedCount + 1)
    tag
  }
  assert(allocated.distinct.size == capacity,
    s"allocator duplicated a live tag: $allocated")
  assert(peek(c.io.busy) == 1)
  assert(peek(c.io.full) == 1)
  assert(peek(c.io.available) == 0)
  assert(peek(c.io.allocate.valid) == 0)

  // A completion remains an independent lifetime event.  With allocation
  // backpressured, releasing one tag creates exactly one available slot.
  val released = allocated(1)
  poke(c.io.allocate.ready, 0)
  poke(c.io.release.valid, 1)
  poke(c.io.release.bits, released)
  assert(peek(c.io.allocate.valid) == 1)
  assert(peek(c.io.allocate.bits) == released)
  step(1)
  poke(c.io.release.valid, 0)
  assert(peek(c.io.outstanding) == capacity - 1)
  assert(peek(c.io.available) == 1)
  assert(peek(c.io.allocate.valid) == 1)
  assert(peek(c.io.allocate.bits) == released)

  // Reclaim the slot, returning to a full table.
  poke(c.io.allocate.ready, 1)
  step(1)
  poke(c.io.allocate.ready, 0)
  assert(peek(c.io.outstanding) == capacity)
  assert(peek(c.io.full) == 1)

  // Full-table release+allocate must recycle the completing tag in the same
  // cycle, with no ready/valid bubble and no change in outstanding capacity.
  val recycled = allocated.last
  poke(c.io.release.valid, 1)
  poke(c.io.release.bits, recycled)
  poke(c.io.allocate.ready, 1)
  assert(peek(c.io.allocate.valid) == 1)
  assert(peek(c.io.allocate.bits) == recycled)
  step(1)
  poke(c.io.release.valid, 0)
  poke(c.io.allocate.ready, 0)
  assert(peek(c.io.outstanding) == capacity)
  assert(peek(c.io.available) == 0)
  assert(peek(c.io.full) == 1)

  // Drain all live transport slots.  These releases model ordered or
  // out-of-order TileLink D completions; allocator state is tag-addressed.
  for (tag <- allocated.reverse) {
    poke(c.io.release.valid, 1)
    poke(c.io.release.bits, tag)
    step(1)
  }
  poke(c.io.release.valid, 0)
  assert(peek(c.io.busy) == 0)
  assert(peek(c.io.full) == 0)
  assert(peek(c.io.outstanding) == 0)
  assert(peek(c.io.available) == capacity)
  assert(peek(c.io.activeMask) == 0)
}

class VpuStoreTransportTagAllocatorSpec extends ChiselFlatSpec {
  behavior of "VpuStoreTransportTagAllocator"

  it should "separate DMA lifetime and recycle a completed full-table tag without a bubble" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2,
      storeQueueEntries = 4)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-store-transport-tags"),
      () => new VpuStoreTransportTagAllocator(p)) { c =>
      new VpuStoreTransportTagAllocatorTester(c, p.storeQueueEntries)
    } should be (true)
  }
}
