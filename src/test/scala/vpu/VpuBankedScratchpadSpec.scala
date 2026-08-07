package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}
import chisel3.stage.ChiselStage

class VpuBankedScratchpadTester(c: VpuBankedScratchpad, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuBankedScratchpad._

  private def idleRead(client: Int): Unit = {
    poke(c.io.readRequest(client).valid, 0)
    poke(c.io.readRequest(client).bits.address, 0)
    poke(c.io.readRequest(client).bits.tag, 0)
  }

  private def idleWrite(client: Int): Unit = {
    poke(c.io.writeRequest(client).valid, 0)
    poke(c.io.writeRequest(client).bits.address, 0)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.writeRequest(client).bits.data(lane), 0)
      poke(c.io.writeRequest(client).bits.laneMask(lane), 0)
    }
  }

  private def driveRead(client: Int, address: Int, tag: Int): Unit = {
    poke(c.io.readRequest(client).valid, 1)
    poke(c.io.readRequest(client).bits.address, address)
    poke(c.io.readRequest(client).bits.tag, tag)
  }

  private def driveWrite(client: Int, address: Int,
      values: Seq[Int]): Unit = {
    require(values.size == p.nLanes)
    poke(c.io.writeRequest(client).valid, 1)
    poke(c.io.writeRequest(client).bits.address, address)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.writeRequest(client).bits.data(lane), values(lane))
      poke(c.io.writeRequest(client).bits.laneMask(lane), 1)
    }
  }

  private def writeOne(client: Int, address: Int,
      values: Seq[Int]): Unit = {
    driveWrite(client, address, values)
    while (peek(c.io.writeRequest(client).ready) == 0) { step(1) }
    step(1)
    idleWrite(client)
  }

  private def setResponseReady(client: Int, ready: Boolean): Unit =
    poke(c.io.readResponse(client).ready, if (ready) 1 else 0)

  private def waitResponse(client: Int, timeoutLimit: Int = 20): Unit = {
    var timeout = timeoutLimit
    while (peek(c.io.readResponse(client).valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"read client $client response timeout")
  }

  private def checkResponse(client: Int, expected: Seq[Int],
      tag: Int): Unit = {
    assert(peek(c.io.readResponse(client).valid) == 1)
    assert(peek(c.io.readResponse(client).bits.tag) == tag)
    for (lane <- 0 until p.nLanes) {
      assert(peek(c.io.readResponse(client).bits.data(lane)) ==
        expected(lane))
    }
  }

  private def drainResponse(client: Int): Unit = {
    setResponseReady(client, ready = true)
    step(1)
    setResponseReady(client, ready = false)
  }

  for (client <- 0 until NumReadClients) {
    idleRead(client)
    setResponseReady(client, ready = false)
  }
  for (client <- 0 until NumWriteClients) idleWrite(client)
  step(2)

  val bank0a = (0 until p.nLanes).map(_ + 10)
  val bank0b = (0 until p.nLanes).map(_ + 30)
  val bank0c = (0 until p.nLanes).map(_ + 50)
  val bank1a = (0 until p.nLanes).map(_ + 70)
  val bank2a = (0 until p.nLanes).map(_ + 90)
  val bank3a = (0 until p.nLanes).map(_ + 110)

  writeOne(ExecuteWriteClient, 0, bank0a)
  writeOne(ExecuteWriteClient, p.vLen, bank0b)
  writeOne(ExecuteWriteClient, 2 * p.vLen, bank0c)
  writeOne(ExecuteWriteClient, p.elementsPerBank, bank1a)
  writeOne(ExecuteWriteClient, 2 * p.elementsPerBank, bank2a)
  writeOne(ExecuteWriteClient, 3 * p.elementsPerBank, bank3a)

  // Source 0, source 1, and store can all consume one word in the same cycle
  // when their addresses map to three different physical banks.
  driveRead(Source0ReadClient, 0, tag = 1)
  driveRead(Source1ReadClient, p.elementsPerBank, tag = 2)
  driveRead(StoreReadClient, 2 * p.elementsPerBank, tag = 3)
  driveWrite(ExecuteWriteClient, 2 * p.elementsPerBank, bank2a)
  driveWrite(LoadWriteClient, 3 * p.elementsPerBank, bank3a)
  for (client <- 0 until NumReadClients) {
    assert(peek(c.io.readRequest(client).ready) == 1,
      s"disjoint read client $client was not accepted")
  }
  for (client <- 0 until NumWriteClients) {
    assert(peek(c.io.writeRequest(client).ready) == 1,
      s"disjoint write client $client was not accepted")
  }
  step(1)
  for (client <- 0 until NumReadClients) idleRead(client)
  for (client <- 0 until NumWriteClients) idleWrite(client)

  waitResponse(Source0ReadClient)
  waitResponse(Source1ReadClient)
  waitResponse(StoreReadClient)
  checkResponse(Source0ReadClient, bank0a, tag = 1)
  checkResponse(Source1ReadClient, bank1a, tag = 2)
  checkResponse(StoreReadClient, bank2a, tag = 3)

  // Every client owns an independent response queue. Drain source 1 and the
  // store while source 0 is backpressured, then prove source 0 remains stable.
  drainResponse(Source1ReadClient)
  drainResponse(StoreReadClient)
  for (_ <- 0 until 3) {
    checkResponse(Source0ReadClient, bank0a, tag = 1)
    step(1)
  }
  drainResponse(Source0ReadClient)
  assert(peek(c.io.busy) == 0)

  // Verify the two simultaneous writes through independent read clients.
  driveRead(Source0ReadClient, 2 * p.elementsPerBank, tag = 4)
  driveRead(StoreReadClient, 3 * p.elementsPerBank, tag = 5)
  assert(peek(c.io.readRequest(Source0ReadClient).ready) == 1)
  assert(peek(c.io.readRequest(StoreReadClient).ready) == 1)
  step(1)
  idleRead(Source0ReadClient)
  idleRead(StoreReadClient)
  waitResponse(Source0ReadClient)
  waitResponse(StoreReadClient)
  checkResponse(Source0ReadClient, bank2a, tag = 4)
  checkResponse(StoreReadClient, bank3a, tag = 5)
  drainResponse(Source0ReadClient)
  drainResponse(StoreReadClient)

  // A same-bank write conflict alternates ownership rather than admitting two
  // requests to the bank's single write port.
  val write0 = (0 until p.nLanes).map(_ + 130)
  val write1 = (0 until p.nLanes).map(_ + 150)
  driveWrite(ExecuteWriteClient, 3 * p.vLen, write0)
  driveWrite(LoadWriteClient, 4 * p.vLen, write1)
  assert(peek(c.io.writeRequest(ExecuteWriteClient).ready) == 1)
  assert(peek(c.io.writeRequest(LoadWriteClient).ready) == 0)
  assert(peek(c.io.writeConflictStall) == 1)
  step(1)
  assert(peek(c.io.writeRequest(ExecuteWriteClient).ready) == 0)
  assert(peek(c.io.writeRequest(LoadWriteClient).ready) == 1)
  assert(peek(c.io.writeConflictStall) == 1)
  step(1)
  idleWrite(ExecuteWriteClient)
  idleWrite(LoadWriteClient)

  // Three reads in one bank serialize. Keep each Decoupled request asserted
  // until it fires and verify that exactly one bank access occurs per cycle.
  val sameBankAddresses = Seq(0, p.vLen, 2 * p.vLen)
  val sameBankTags = Seq(10, 11, 12)
  val sameBankData = Seq(bank0a, bank0b, bank0c)
  for (client <- 0 until NumReadClients) {
    driveRead(client, sameBankAddresses(client), sameBankTags(client))
  }
  val pending = Array.fill(NumReadClients)(true)
  var cycles = 0
  while (pending.contains(true) && cycles < 10) {
    val granted = (0 until NumReadClients).filter { client =>
      pending(client) && peek(c.io.readRequest(client).ready) == 1
    }
    assert(granted.size == 1,
      s"expected one same-bank read grant, observed ${granted.size}")
    assert(peek(c.io.readConflictStall) ==
      (if (pending.count(_ == true) > 1) 1 else 0))
    step(1)
    granted.foreach { client =>
      pending(client) = false
      idleRead(client)
    }
    cycles += 1
  }
  assert(!pending.contains(true), "same-bank read arbitration starved a client")

  for (client <- 0 until NumReadClients) {
    waitResponse(client)
    checkResponse(client, sameBankData(client), sameBankTags(client))
  }

  // Hold all responses and confirm their independently tagged data is stable.
  for (_ <- 0 until 3) {
    for (client <- 0 until NumReadClients) {
      checkResponse(client, sameBankData(client), sameBankTags(client))
    }
    step(1)
  }
  for (client <- 0 until NumReadClients) setResponseReady(client, ready = true)
  step(1)
  for (client <- 0 until NumReadClients) setResponseReady(client, ready = false)
  assert(peek(c.io.busy) == 0)
}

class VpuBankedScratchpadSpec extends ChiselFlatSpec {
  behavior of "VpuBankedScratchpad"

  it should "arbitrate independent bank-local read clients without losing responses" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-banked-scratchpad"),
      () => new VpuBankedScratchpad(p)) { c =>
      new VpuBankedScratchpadTester(c, p)
    } should be (true)
  }

  it should "infer exactly one read and one write mport per physical bank" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2)
    val chirrtl = (new ChiselStage).emitChirrtl(
      new VpuBankedScratchpad(p))
    val memoryLines = chirrtl.linesIterator.filter(_.contains("memories_"))
      .toVector
    val readers = memoryLines.count(_.contains("read mport"))
    val writers = memoryLines.count(_.contains("write mport"))
    assert(readers == p.vSpadBanks,
      s"expected one reader per bank, found $readers for ${p.vSpadBanks} banks")
    assert(writers == p.vSpadBanks,
      s"expected one writer per bank, found $writers for ${p.vSpadBanks} banks")
  }
}
