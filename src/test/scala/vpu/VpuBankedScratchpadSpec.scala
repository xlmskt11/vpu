package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}
import chisel3.stage.ChiselStage

class VpuBankedScratchpadTester(c: VpuBankedScratchpad, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuBankedScratchpad._

  private def idleRead(client: Int): Unit = {
    poke(c.io.readRequest(client).valid, 0)
    poke(c.io.readRequest(client).bits.address0, 0)
    poke(c.io.readRequest(client).bits.address1, 0)
    poke(c.io.readRequest(client).bits.useAddress1, 0)
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

  private def driveRead(client: Int, address0: Int, address1: Int,
      useAddress1: Boolean, tag: Int): Unit = {
    poke(c.io.readRequest(client).valid, 1)
    poke(c.io.readRequest(client).bits.address0, address0)
    poke(c.io.readRequest(client).bits.address1, address1)
    poke(c.io.readRequest(client).bits.useAddress1,
      if (useAddress1) 1 else 0)
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

  private def waitResponse(client: Int, timeoutLimit: Int = 20): Unit = {
    var timeout = timeoutLimit
    while (peek(c.io.readResponse(client).valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"read client $client response timeout")
  }

  private def checkResponse(client: Int, expected0: Seq[Int],
      expected1: Option[Seq[Int]], tag: Int, serialized: Boolean): Unit = {
    assert(peek(c.io.readResponse(client).valid) == 1)
    assert(peek(c.io.readResponse(client).bits.tag) == tag)
    assert(peek(c.io.readResponse(client).bits.serialized) ==
      (if (serialized) 1 else 0))
    for (lane <- 0 until p.nLanes) {
      assert(peek(c.io.readResponse(client).bits.data0(lane)) ==
        expected0(lane))
      expected1.foreach { values =>
        assert(peek(c.io.readResponse(client).bits.data1(lane)) ==
          values(lane))
      }
    }
  }

  for (client <- 0 until NumReadClients) {
    idleRead(client)
    poke(c.io.readResponse(client).ready, 0)
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

  // Both read clients and both write clients use disjoint banks in one cycle.
  driveRead(ExecuteReadClient, 0, 0, useAddress1 = false, tag = 1)
  driveRead(StoreReadClient, p.elementsPerBank, 0,
    useAddress1 = false, tag = 2)
  driveWrite(ExecuteWriteClient, 2 * p.elementsPerBank, bank2a)
  driveWrite(LoadWriteClient, 3 * p.elementsPerBank, bank3a)
  for (client <- 0 until NumReadClients) {
    assert(peek(c.io.readRequest(client).ready) == 1,
      "disjoint read clients were not accepted together")
  }
  for (client <- 0 until NumWriteClients) {
    assert(peek(c.io.writeRequest(client).ready) == 1,
      "disjoint write clients were not accepted together")
  }
  step(1)
  for (client <- 0 until NumReadClients) idleRead(client)
  for (client <- 0 until NumWriteClients) idleWrite(client)

  waitResponse(ExecuteReadClient)
  waitResponse(StoreReadClient)
  checkResponse(ExecuteReadClient, bank0a, None, tag = 1,
    serialized = false)
  checkResponse(StoreReadClient, bank1a, None, tag = 2,
    serialized = false)

  // Independent response queues: ST may drain while EX is held stable.
  poke(c.io.readResponse(StoreReadClient).ready, 1)
  step(1)
  poke(c.io.readResponse(StoreReadClient).ready, 0)
  for (_ <- 0 until 3) {
    checkResponse(ExecuteReadClient, bank0a, None, tag = 1,
      serialized = false)
    step(1)
  }
  poke(c.io.readResponse(ExecuteReadClient).ready, 1)
  step(1)
  poke(c.io.readResponse(ExecuteReadClient).ready, 0)

  // Verify the two simultaneous writes through two simultaneous reads.
  driveRead(ExecuteReadClient, 2 * p.elementsPerBank, 0,
    useAddress1 = false, tag = 3)
  driveRead(StoreReadClient, 3 * p.elementsPerBank, 0,
    useAddress1 = false, tag = 4)
  assert(peek(c.io.readRequest(ExecuteReadClient).ready) == 1)
  assert(peek(c.io.readRequest(StoreReadClient).ready) == 1)
  step(1)
  idleRead(ExecuteReadClient)
  idleRead(StoreReadClient)
  waitResponse(ExecuteReadClient)
  waitResponse(StoreReadClient)
  checkResponse(ExecuteReadClient, bank2a, None, tag = 3,
    serialized = false)
  checkResponse(StoreReadClient, bank3a, None, tag = 4,
    serialized = false)
  poke(c.io.readResponse(ExecuteReadClient).ready, 1)
  poke(c.io.readResponse(StoreReadClient).ready, 1)
  step(1)
  poke(c.io.readResponse(ExecuteReadClient).ready, 0)
  poke(c.io.readResponse(StoreReadClient).ready, 0)

  // A same-bank write conflict alternates owners instead of starving one.
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

  // Read conflicts use the same fair RR policy on the single bank read port.
  driveRead(ExecuteReadClient, 0, 0, useAddress1 = false, tag = 10)
  driveRead(StoreReadClient, p.vLen, 0, useAddress1 = false, tag = 11)
  assert(peek(c.io.readRequest(ExecuteReadClient).ready) == 1)
  assert(peek(c.io.readRequest(StoreReadClient).ready) == 0)
  assert(peek(c.io.readConflictStall) == 1)
  step(1)
  assert(peek(c.io.readRequest(ExecuteReadClient).ready) == 0)
  assert(peek(c.io.readRequest(StoreReadClient).ready) == 1)
  assert(peek(c.io.readConflictStall) == 1)
  step(1)
  idleRead(ExecuteReadClient)
  idleRead(StoreReadClient)
  waitResponse(ExecuteReadClient)
  waitResponse(StoreReadClient)
  checkResponse(ExecuteReadClient, bank0a, None, tag = 10,
    serialized = false)
  checkResponse(StoreReadClient, bank0b, None, tag = 11,
    serialized = false)
  poke(c.io.readResponse(ExecuteReadClient).ready, 1)
  poke(c.io.readResponse(StoreReadClient).ready, 1)
  step(1)
  poke(c.io.readResponse(ExecuteReadClient).ready, 0)
  poke(c.io.readResponse(StoreReadClient).ready, 0)

  // Equal VV addresses use one bank read and broadcast the returned word.
  driveRead(ExecuteReadClient, 0, 0, useAddress1 = true, tag = 20)
  assert(peek(c.io.readRequest(ExecuteReadClient).ready) == 1)
  step(1)
  idleRead(ExecuteReadClient)
  waitResponse(ExecuteReadClient)
  checkResponse(ExecuteReadClient, bank0a, Some(bank0a), tag = 20,
    serialized = false)
  poke(c.io.readResponse(ExecuteReadClient).ready, 1)
  step(1)
  poke(c.io.readResponse(ExecuteReadClient).ready, 0)

  // Distinct words in one bank consume two read cycles. The continuation owns
  // the bank, then the waiting other client is accepted in the capture cycle.
  driveRead(ExecuteReadClient, 0, 2 * p.vLen,
    useAddress1 = true, tag = 30)
  assert(peek(c.io.readRequest(ExecuteReadClient).ready) == 1)
  step(1)
  idleRead(ExecuteReadClient)
  driveRead(StoreReadClient, p.vLen, 0,
    useAddress1 = false, tag = 31)
  assert(peek(c.io.serializedRead(ExecuteReadClient)) == 1,
    "serializedRead did not pulse on the second bank-read cycle")
  assert(peek(c.io.readRequest(StoreReadClient).ready) == 0,
    "another client entered a bank reserved by a serialized continuation")
  step(1)
  assert(peek(c.io.serializedRead(ExecuteReadClient)) == 0)
  assert(peek(c.io.readRequest(StoreReadClient).ready) == 1,
    "waiting client did not enter on the serialized capture cycle")
  step(1)
  idleRead(StoreReadClient)
  waitResponse(ExecuteReadClient)
  waitResponse(StoreReadClient)
  checkResponse(ExecuteReadClient, bank0a, Some(bank0c), tag = 30,
    serialized = true)
  checkResponse(StoreReadClient, bank0b, None, tag = 31,
    serialized = false)

  // Hold both final responses and confirm all visible fields stay unchanged.
  for (_ <- 0 until 3) {
    checkResponse(ExecuteReadClient, bank0a, Some(bank0c), tag = 30,
      serialized = true)
    checkResponse(StoreReadClient, bank0b, None, tag = 31,
      serialized = false)
    step(1)
  }
  poke(c.io.readResponse(ExecuteReadClient).ready, 1)
  poke(c.io.readResponse(StoreReadClient).ready, 1)
  step(1)
  assert(peek(c.io.busy) == 0)
}

class VpuBankedScratchpadSpec extends ChiselFlatSpec {
  behavior of "VpuBankedScratchpad"

  it should "arbitrate bank-local 1R1W clients without losing responses" in {
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
