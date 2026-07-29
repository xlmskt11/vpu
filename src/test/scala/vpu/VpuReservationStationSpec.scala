package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuReservationStationTester(c: VpuReservationStation, p: VpuParams)
    extends PeekPokeTester(c) {
  private val nLd = VpuReservationStation.loadEntries(p)
  private val nEx = VpuReservationStation.executeEntries(p)
  private val stBase = nLd + nEx

  poke(c.io.allocate.load.valid, 0)
  poke(c.io.allocate.execute.valid, 0)
  poke(c.io.allocate.store.valid, 0)
  poke(c.io.issue.load.ready, 0)
  poke(c.io.issue.execute.ready, 0)
  poke(c.io.issue.store.ready, 0)
  poke(c.io.flushUnissued, 0)
  for (port <- c.io.complete) {
    poke(port.valid, 0)
    poke(port.bits, 0)
  }
  step(2)

  private def access(set: VpuRsAccessSet, base: Int, count: Int,
                     read: Boolean, write: Boolean): Unit = {
    for (i <- 0 until 3) {
      poke(set.accesses(i).base, if (i == 0) base else 0)
      poke(set.accesses(i).elementCount, if (i == 0) count else 0)
      poke(set.accesses(i).read, if (i == 0 && read) 1 else 0)
      poke(set.accesses(i).write, if (i == 0 && write) 1 else 0)
    }
  }

  private def allocLoad(id: Int, base: Int, count: Int = 16): Unit = {
    poke(c.io.allocate.load.bits.command.vaddr, 0x1000 + id * 0x100)
    poke(c.io.allocate.load.bits.command.spadBase, base)
    poke(c.io.allocate.load.bits.command.elementCount, count)
    poke(c.io.allocate.load.bits.command.hazardTag, 0)
    poke(c.io.allocate.load.bits.command.sequence, id)
    access(c.io.allocate.load.bits.accessSet, base, count,
      read = false, write = true)
    poke(c.io.allocate.load.valid, 1)
    assert(peek(c.io.allocate.load.ready) == 1,
      s"load $id was not accepted despite a free LD entry")
    step(1)
    poke(c.io.allocate.load.valid, 0)
  }

  private def allocExecute(id: Int, base: Int, count: Int = 16,
                           readFp: Int = 0, writeFp: Int = 0): Unit = {
    val cmd = c.io.allocate.execute.bits.command
    poke(cmd.opcode, VpuOpcode.V_MUL_VF)
    poke(cmd.funct1, 0)
    poke(cmd.destination, base + 32)
    poke(cmd.source0, base)
    poke(cmd.source1, 0)
    poke(cmd.elementCount, count)
    poke(cmd.useSource1, 0)
    poke(cmd.fpRs1, 0)
    poke(cmd.fpRs2, 0)
    poke(cmd.fpSeed, 0)
    poke(cmd.fpReadMask, readFp)
    poke(cmd.fpWriteMask, writeFp)
    poke(cmd.scalarA, 0)
    poke(cmd.scalarB, 0)
    poke(cmd.scalarSeed, 0)
    poke(cmd.writesFp, if (writeFp != 0) 1 else 0)
    poke(cmd.fpDestination, 0)
    poke(cmd.hasHazard, 1)
    poke(cmd.hazardTag, 0)
    poke(cmd.sequence, id)
    access(c.io.allocate.execute.bits.accessSet, base, count,
      read = true, write = false)
    poke(c.io.allocate.execute.valid, 1)
    assert(peek(c.io.allocate.execute.ready) == 1,
      s"execute $id was not accepted despite a free EX entry")
    step(1)
    poke(c.io.allocate.execute.valid, 0)
  }

  private def allocStore(id: Int, base: Int, count: Int = 16,
                         stepAfter: Boolean = true): Unit = {
    poke(c.io.allocate.store.bits.command.vaddr, 0x8000 + id * 0x100)
    poke(c.io.allocate.store.bits.command.spadBase, base)
    poke(c.io.allocate.store.bits.command.elementCount, count)
    poke(c.io.allocate.store.bits.command.hazardTag, 0)
    poke(c.io.allocate.store.bits.command.transportTag, 0)
    poke(c.io.allocate.store.bits.command.sequence, id)
    access(c.io.allocate.store.bits.accessSet, base, count,
      read = true, write = false)
    poke(c.io.allocate.store.valid, 1)
    assert(peek(c.io.allocate.store.ready) == 1,
      s"store $id was not accepted despite a free ST entry")
    if (stepAfter) {
      step(1)
      poke(c.io.allocate.store.valid, 0)
    }
  }

  private def complete(tags: Seq[BigInt]): Unit = {
    tags.zipWithIndex.foreach { case (tag, port) =>
      poke(c.io.complete(port).valid, 1)
      poke(c.io.complete(port).bits, tag)
    }
    step(1)
    tags.indices.foreach { port => poke(c.io.complete(port).valid, 0) }
  }

  // Same-class order is represented by a dependency bit.  A non-conflicting
  // younger load may issue as soon as its predecessor has issued.
  allocLoad(0, base = 0)
  allocLoad(1, base = 32)
  assert(peek(c.io.issue.load.valid) == 1)
  val disjointLoad0 = peek(c.io.issue.load.bits.tag)
  poke(c.io.issue.load.ready, 1)
  step(1)
  val disjointLoad1 = peek(c.io.issue.load.bits.tag)
  assert(peek(c.io.issue.load.valid) == 1,
    "same-class non-conflict ordering edge was not released on issue")
  assert(disjointLoad1 != disjointLoad0)
  step(1)
  poke(c.io.issue.load.ready, 0)
  complete(Seq(disjointLoad0, disjointLoad1))
  assert(peek(c.io.busy) == 0)

  // Unlike Gemmini's historical same-load shortcut, an actual LD/LD WAW edge
  // must survive producer issue and clear only at VSRAM writeback completion.
  allocLoad(2, base = 64)
  allocLoad(3, base = 64)
  val wawProducer = peek(c.io.issue.load.bits.tag)
  poke(c.io.issue.load.ready, 1)
  step(1)
  poke(c.io.issue.load.ready, 0)
  assert(peek(c.io.issue.load.valid) == 0,
    "overlapping younger load escaped before older load completion")
  complete(Seq(wawProducer))
  assert(peek(c.io.issue.load.valid) == 1,
    "LD/LD WAW edge did not clear on producer completion")
  val wawConsumer = peek(c.io.issue.load.bits.tag)
  poke(c.io.issue.load.ready, 1)
  step(1)
  poke(c.io.issue.load.ready, 0)
  complete(Seq(wawConsumer))

  // Cross-class RAW also remains after issue.  Live EX entries directly
  // expose their pending scalar-register masks for Core control commands.
  allocLoad(4, base = 96)
  allocExecute(0, base = 96, readFp = 0x04, writeFp = 0x08)
  assert(peek(c.io.pendingFpReadMask) == 0x04)
  assert(peek(c.io.pendingFpWriteMask) == 0x08)
  assert(peek(c.io.issue.execute.valid) == 0)
  val rawLoad = peek(c.io.issue.load.bits.tag)
  poke(c.io.issue.load.ready, 1)
  step(1)
  poke(c.io.issue.load.ready, 0)
  assert(peek(c.io.issue.execute.valid) == 0,
    "cross-class RAW dependency was incorrectly released on issue")
  complete(Seq(rawLoad))
  assert(peek(c.io.issue.execute.valid) == 1)
  val rawExecute = peek(c.io.issue.execute.bits.tag)
  poke(c.io.issue.execute.ready, 1)
  step(1)
  poke(c.io.issue.execute.ready, 0)
  complete(Seq(rawExecute))
  assert(peek(c.io.pendingFpReadMask) == 0)
  assert(peek(c.io.pendingFpWriteMask) == 0)

  // Both ST slots are live and issued. Recycle one completion tag while a new
  // command is allocated in the same cycle; the new entry must not depend on
  // its own reused tag.
  allocStore(0, base = 128)
  allocStore(1, base = 160)
  poke(c.io.issue.store.ready, 1)
  val store0 = peek(c.io.issue.store.bits.tag)
  step(1)
  val store1 = peek(c.io.issue.store.bits.tag)
  step(1)
  poke(c.io.issue.store.ready, 0)
  assert(store0 == stBase)
  assert(store1 == stBase + 1)
  poke(c.io.complete(0).valid, 1)
  poke(c.io.complete(0).bits, store0)
  allocStore(2, base = 192, stepAfter = false)
  step(1)
  poke(c.io.complete(0).valid, 0)
  poke(c.io.allocate.store.valid, 0)
  assert(peek(c.io.issue.store.valid) == 1,
    "same-cycle completion/reallocation left a self-dependency")
  val recycledStore = peek(c.io.issue.store.bits.tag)
  assert(recycledStore == store0)
  poke(c.io.issue.store.ready, 1)
  step(1)
  poke(c.io.issue.store.ready, 0)
  complete(Seq(store1, recycledStore))

  // Fault flush drops all unissued entries in one cycle, while an already
  // issued producer remains live until its engine explicitly completes it.
  allocLoad(5, base = 224)
  val issuedSurvivor = peek(c.io.issue.load.bits.tag)
  poke(c.io.issue.load.ready, 1)
  step(1)
  poke(c.io.issue.load.ready, 0)
  allocLoad(6, base = 224)
  allocExecute(1, base = 256, readFp = 0x20, writeFp = 0x40)
  assert(peek(c.io.validMask) != 0)
  assert(peek(c.io.pendingFpReadMask) == 0x20)
  poke(c.io.flushUnissued, 1)
  assert(peek(c.io.flushedMask) != 0)
  assert(peek(c.io.allocate.load.ready) == 0)
  assert(peek(c.io.issue.execute.valid) == 0)
  step(1)
  poke(c.io.flushUnissued, 0)
  assert(peek(c.io.validMask) == (BigInt(1) << issuedSurvivor.toInt),
    "fault flush removed an issued entry or retained an unissued entry")
  assert(peek(c.io.pendingFpReadMask) == 0)
  assert(peek(c.io.pendingFpWriteMask) == 0)
  complete(Seq(issuedSurvivor))
  assert(peek(c.io.busy) == 0)

  // Exercise every default-sized ST partition slot.  The final local ID is
  // three, so stBase=13 must produce global tag 16 without UInt-width wrap.
  for (i <- 0 until p.storeQueueEntries) {
    allocStore(10 + i, base = 320 + i * 32)
  }
  poke(c.io.issue.store.ready, 1)
  val allStoreTags = (0 until p.storeQueueEntries).map { localId =>
    assert(peek(c.io.issue.store.valid) == 1)
    val tag = peek(c.io.issue.store.bits.tag)
    assert(tag == stBase + localId,
      s"store local tag $localId wrapped: expected ${stBase + localId}, got $tag")
    step(1)
    tag
  }
  poke(c.io.issue.store.ready, 0)
  complete(allStoreTags)
  assert(peek(c.io.busy) == 0)
}

class VpuReservationStationSpec extends ChiselFlatSpec {
  behavior of "VpuReservationStation"

  it should "schedule with explicit dependencies and no wrapping age" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2,
      loadQueueEntries = 4, execQueueEntries = 8,
      storeQueueEntries = 4, hazardEntries = 17)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-reservation-station"),
      () => new VpuReservationStation(p)) { c =>
      new VpuReservationStationTester(c, p)
    } should be (true)
  }
}
