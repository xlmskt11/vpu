package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}
import chisel3.stage.ChiselStage

/** Focused coverage for the Gemmini-facing VSRAM matrix ports.
  *
  * The production fusion configuration uses four Gemminis, eight logical
  * banks, and four sub-banks per logical bank.  Keeping those exact geometry
  * parameters here catches accidental changes to the row-to-sub-bank mapping
  * as well as arbitration regressions between Gemmini and the VPU clients.
  */
private object VpuMatrixScratchpadTestParams {
  val p: VpuParams = VpuParams(
    storageType = VpuStorageType.FP32,
    computeType = VpuStorageType.FP32,
    vLen = 128,
    nLanes = 16,
    sfuLanes = 4,
    vSpadKB = 128,
    vSpadBanks = 8,
    vSpadSubBanks = 4,
    matrixPorts = 4)
}

private object VpuWideMatrixScratchpadTestParams {
  val p: VpuParams = VpuParams(
    storageType = VpuStorageType.FP32,
    computeType = VpuStorageType.FP32,
    vLen = 128,
    nLanes = 16,
    sfuLanes = 4,
    vSpadKB = 128,
    vSpadBanks = 8,
    vSpadSubBanks = 4,
    matrixPorts = 1,
    matrixRowElements = 32)
}

private class VpuMatrixScratchpadTester(
    c: VpuBankedScratchpad,
    p: VpuParams) extends PeekPokeTester(c) {
  import VpuBankedScratchpad._

  private def pattern(seed: Int): Seq[BigInt] =
    (0 until p.nLanes).map(lane => BigInt(seed + lane))

  private def idleRead(client: Int): Unit = {
    poke(c.io.readRequest(client).valid, 0)
    poke(c.io.readRequest(client).bits.address, 0)
    poke(c.io.readRequest(client).bits.tag, 0)
    poke(c.io.readResponse(client).ready, 0)
  }

  private def idleWrite(client: Int): Unit = {
    poke(c.io.writeRequest(client).valid, 0)
    poke(c.io.writeRequest(client).bits.address, 0)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.writeRequest(client).bits.data(lane), 0)
      poke(c.io.writeRequest(client).bits.laneMask(lane), 0)
    }
  }

  private def idleMatrixRead(port: Int): Unit = {
    poke(c.io.matrixRead.get(port).req.valid, 0)
    poke(c.io.matrixRead.get(port).req.bits.rowAddress, 0)
    poke(c.io.matrixRead.get(port).resp.ready, 0)
  }

  private def idleMatrixWrite(port: Int): Unit = {
    poke(c.io.matrixWrite.get(port).valid, 0)
    poke(c.io.matrixWrite.get(port).bits.rowAddress, 0)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.matrixWrite.get(port).bits.data(lane), 0)
      poke(c.io.matrixWrite.get(port).bits.laneMask(lane), 0)
    }
  }

  private def driveMatrixWrite(
      port: Int,
      row: Int,
      values: Seq[BigInt]): Unit = {
    require(values.size == p.nLanes)
    poke(c.io.matrixWrite.get(port).valid, 1)
    poke(c.io.matrixWrite.get(port).bits.rowAddress, row)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.matrixWrite.get(port).bits.data(lane), values(lane))
      poke(c.io.matrixWrite.get(port).bits.laneMask(lane), 1)
    }
  }

  private def driveLocalWrite(
      client: Int,
      elementAddress: Int,
      values: Seq[BigInt]): Unit = {
    require(values.size == p.nLanes)
    poke(c.io.writeRequest(client).valid, 1)
    poke(c.io.writeRequest(client).bits.address, elementAddress)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.writeRequest(client).bits.data(lane), values(lane))
      poke(c.io.writeRequest(client).bits.laneMask(lane), 1)
    }
  }

  private def requestMatrixRead(port: Int, row: Int): Unit = {
    poke(c.io.matrixRead.get(port).req.valid, 1)
    poke(c.io.matrixRead.get(port).req.bits.rowAddress, row)
    assert(peek(c.io.matrixRead.get(port).req.ready) == 1,
      s"matrix port $port did not accept row $row")
  }

  private def waitForMatrixResponses(ports: Seq[Int]): Unit = {
    var timeout = 20
    while (!ports.forall(port =>
        peek(c.io.matrixRead.get(port).resp.valid) == 1) && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"matrix responses timed out on ports ${ports.mkString(",")}")
  }

  private def checkMatrixResponse(
      port: Int,
      expected: Seq[BigInt]): Unit = {
    assert(peek(c.io.matrixRead.get(port).resp.valid) == 1)
    for (lane <- 0 until p.nLanes) {
      assert(peek(c.io.matrixRead.get(port).resp.bits.data(lane)) ==
        expected(lane),
        s"port $port lane $lane returned the wrong matrix data")
    }
  }

  private def consumeMatrixResponses(ports: Seq[Int]): Unit = {
    ports.foreach(port => poke(c.io.matrixRead.get(port).resp.ready, 1))
    step(1)
    ports.foreach(port => poke(c.io.matrixRead.get(port).resp.ready, 0))
  }

  require(p.physicalBanks == 32)
  require(p.wordsPerBank == 256)
  require(p.wordsPerSubBank == 64)

  for (client <- 0 until NumReadClients) idleRead(client)
  for (client <- 0 until NumWriteClients) idleWrite(client)
  for (port <- 0 until p.matrixPorts) {
    idleMatrixRead(port)
    idleMatrixWrite(port)
  }
  step(2)

  // Consecutive matrix rows in one logical bank select sub-banks 0,1,2,3.
  // Sequential round trips prove that those rows do not alias each other.
  val subBankRows = (40 until 44)
  val subBankData = subBankRows.zipWithIndex.map { case (row, index) =>
    row -> pattern(0x1000 + index * 0x100)
  }.toMap
  subBankRows.foreach { row =>
    driveMatrixWrite(port = 0, row, subBankData(row))
    step(1)
    idleMatrixWrite(port = 0)
  }
  subBankRows.foreach { row =>
    requestMatrixRead(port = 0, row)
    step(1)
    poke(c.io.matrixRead.get(0).req.valid, 0)
    waitForMatrixResponses(Seq(0))
    checkMatrixResponse(0, subBankData(row))
    consumeMatrixResponses(Seq(0))
  }

  // A Gemmini matrix write has strict priority over both VPU write clients.
  // Both local requests target the same physical sub-bank as matrix row 52.
  val priorityRow = 52
  val priorityData = pattern(0x4000)
  driveMatrixWrite(port = 0, priorityRow, priorityData)
  driveLocalWrite(ExecuteWriteClient, priorityRow * p.nLanes,
    pattern(0x5000))
  driveLocalWrite(LoadWriteClient,
    (priorityRow + p.vSpadSubBanks) * p.nLanes, pattern(0x6000))
  assert(peek(c.io.writeRequest(ExecuteWriteClient).ready) == 0,
    "matrix write did not backpressure the execute writer")
  assert(peek(c.io.writeRequest(LoadWriteClient).ready) == 0,
    "matrix write did not backpressure the load writer")
  step(1)
  idleMatrixWrite(port = 0)
  idleWrite(ExecuteWriteClient)
  idleWrite(LoadWriteClient)

  requestMatrixRead(port = 0, priorityRow)
  step(1)
  poke(c.io.matrixRead.get(0).req.valid, 0)
  waitForMatrixResponses(Seq(0))
  checkMatrixResponse(0, priorityData)
  consumeMatrixResponses(Seq(0))

  // Four ports can write and then read four distinct logical banks in the
  // same cycle.  Using each bank's first row also makes their physical banks
  // distinct after sub-bank expansion.
  val parallelRows = (0 until p.matrixPorts).map(_ * p.wordsPerBank)
  val parallelData = (0 until p.matrixPorts).map(port =>
    pattern(0x8000 + port * 0x1000))
  for (port <- 0 until p.matrixPorts) {
    driveMatrixWrite(port, parallelRows(port), parallelData(port))
  }
  step(1)
  for (port <- 0 until p.matrixPorts) idleMatrixWrite(port)

  for (port <- 0 until p.matrixPorts) {
    requestMatrixRead(port, parallelRows(port))
  }
  step(1)
  for (port <- 0 until p.matrixPorts) {
    poke(c.io.matrixRead.get(port).req.valid, 0)
  }
  waitForMatrixResponses(0 until p.matrixPorts)
  for (port <- 0 until p.matrixPorts) {
    checkMatrixResponse(port, parallelData(port))
  }
  consumeMatrixResponses(0 until p.matrixPorts)
  assert(peek(c.io.busy) == 0)
}

/** An expected-failure test which proves the software bank-allocation
  * contract is guarded in hardware. */
private class VpuMatrixLogicalBankCollisionTester(
    c: VpuBankedScratchpad,
    p: VpuParams) extends PeekPokeTester(c) {
  for (client <- 0 until VpuBankedScratchpad.NumReadClients) {
    poke(c.io.readRequest(client).valid, 0)
    poke(c.io.readRequest(client).bits.address, 0)
    poke(c.io.readRequest(client).bits.tag, 0)
    poke(c.io.readResponse(client).ready, 0)
  }
  for (client <- 0 until VpuBankedScratchpad.NumWriteClients) {
    poke(c.io.writeRequest(client).valid, 0)
    poke(c.io.writeRequest(client).bits.address, 0)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.writeRequest(client).bits.data(lane), 0)
      poke(c.io.writeRequest(client).bits.laneMask(lane), 0)
    }
  }
  for (port <- 0 until p.matrixPorts) {
    poke(c.io.matrixRead.get(port).req.valid, 0)
    poke(c.io.matrixRead.get(port).req.bits.rowAddress, 0)
    poke(c.io.matrixRead.get(port).resp.ready, 0)
    poke(c.io.matrixWrite.get(port).valid, 0)
    poke(c.io.matrixWrite.get(port).bits.rowAddress, 0)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.matrixWrite.get(port).bits.data(lane), 0)
      poke(c.io.matrixWrite.get(port).bits.laneMask(lane), 0)
    }
  }
  step(1)

  // Rows zero and one occupy different physical sub-banks but the same
  // logical bank, which is forbidden for two simultaneous Gemmini clients.
  poke(c.io.matrixRead.get(0).req.valid, 1)
  poke(c.io.matrixRead.get(0).req.bits.rowAddress, 0)
  poke(c.io.matrixRead.get(1).req.valid, 1)
  poke(c.io.matrixRead.get(1).req.bits.rowAddress, 1)
  step(1)
}

/** One 32-element Gemmini row is atomically split into two physical 16-lane
  * words. The 17-active-lane update crosses the fragment boundary and checks
  * both split/gather ordering and per-fragment write masks. */
private class VpuWideMatrixScratchpadTester(
    c: VpuBankedScratchpad,
    p: VpuParams) extends PeekPokeTester(c) {
  require(p.nLanes == 16 && p.matrixElementsPerRow == 32 &&
    p.matrixWordsPerRow == 2)

  for (client <- 0 until VpuBankedScratchpad.NumReadClients) {
    poke(c.io.readRequest(client).valid, 0)
    poke(c.io.readRequest(client).bits.address, 0)
    poke(c.io.readRequest(client).bits.tag, 0)
    poke(c.io.readResponse(client).ready, 0)
  }
  for (client <- 0 until VpuBankedScratchpad.NumWriteClients) {
    poke(c.io.writeRequest(client).valid, 0)
    poke(c.io.writeRequest(client).bits.address, 0)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.writeRequest(client).bits.data(lane), 0)
      poke(c.io.writeRequest(client).bits.laneMask(lane), 0)
    }
  }
  poke(c.io.matrixRead.get(0).req.valid, 0)
  poke(c.io.matrixRead.get(0).req.bits.rowAddress, 0)
  poke(c.io.matrixRead.get(0).resp.ready, 0)
  poke(c.io.matrixWrite.get(0).valid, 0)
  poke(c.io.matrixWrite.get(0).bits.rowAddress, 0)
  for (element <- 0 until p.matrixElementsPerRow) {
    poke(c.io.matrixWrite.get(0).bits.data(element), 0)
    poke(c.io.matrixWrite.get(0).bits.laneMask(element), 0)
  }
  step(2)

  val row = 5
  // Initialize both fragments, then overwrite exactly 17 elements. Element
  // 16 is lane zero of the second physical word.
  poke(c.io.matrixWrite.get(0).valid, 1)
  poke(c.io.matrixWrite.get(0).bits.rowAddress, row)
  for (element <- 0 until p.matrixElementsPerRow) {
    poke(c.io.matrixWrite.get(0).bits.data(element), 0)
    poke(c.io.matrixWrite.get(0).bits.laneMask(element), 1)
  }
  step(1)
  for (element <- 0 until p.matrixElementsPerRow) {
    poke(c.io.matrixWrite.get(0).bits.data(element), 0x1000 + element)
    poke(c.io.matrixWrite.get(0).bits.laneMask(element),
      if (element < 17) 1 else 0)
  }
  step(1)
  poke(c.io.matrixWrite.get(0).valid, 0)

  // Observe the two halves independently through the ordinary VPU read
  // ports, so a matching split/gather permutation cannot hide an ordering
  // bug in the matrix bridge.
  val elementBase = row * p.matrixElementsPerRow
  for (client <- 0 until 2) {
    poke(c.io.readRequest(client).valid, 1)
    poke(c.io.readRequest(client).bits.address,
      elementBase + client * p.nLanes)
    poke(c.io.readRequest(client).bits.tag, client)
    assert(peek(c.io.readRequest(client).ready) == 1,
      s"local read of matrix fragment $client was not accepted")
  }
  step(1)
  for (client <- 0 until 2) poke(c.io.readRequest(client).valid, 0)
  var timeout = 10
  while ((peek(c.io.readResponse(0).valid) == 0 ||
      peek(c.io.readResponse(1).valid) == 0) && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0, "local fragment reads timed out")
  for (client <- 0 until 2; lane <- 0 until p.nLanes) {
    val element = client * p.nLanes + lane
    val expected = if (element < 17) 0x1000 + element else 0
    assert(peek(c.io.readResponse(client).bits.data(lane)) == expected,
      s"matrix split put element $element in the wrong physical word")
  }
  for (client <- 0 until 2) poke(c.io.readResponse(client).ready, 1)
  step(1)
  for (client <- 0 until 2) poke(c.io.readResponse(client).ready, 0)

  def checkMatrixRow(checkRow: Int, expected: Seq[Int]): Unit = {
    poke(c.io.matrixRead.get(0).req.valid, 1)
    poke(c.io.matrixRead.get(0).req.bits.rowAddress, checkRow)
    assert(peek(c.io.matrixRead.get(0).req.ready) == 1,
      "wide matrix row was not accepted atomically")
    step(1)
    poke(c.io.matrixRead.get(0).req.valid, 0)
    var responseTimeout = 10
    while (peek(c.io.matrixRead.get(0).resp.valid) == 0 &&
        responseTimeout > 0) {
      step(1)
      responseTimeout -= 1
    }
    assert(responseTimeout > 0, "wide matrix response timed out")
    for (element <- 0 until p.matrixElementsPerRow) {
      assert(peek(c.io.matrixRead.get(0).resp.bits.data(element)) ==
        expected(element),
        s"wide matrix element $element did not survive gather")
    }
    poke(c.io.matrixRead.get(0).resp.ready, 1)
    step(1)
    poke(c.io.matrixRead.get(0).resp.ready, 0)
  }

  checkMatrixRow(row, (0 until p.matrixElementsPerRow).map { element =>
    if (element < 17) 0x1000 + element else 0
  })

  // Reverse direction: two ordinary VPU writes populate physical words and
  // one matrix read must gather them in lower-half/upper-half order.
  val localWriteRow = row + 1
  val localWriteBase = localWriteRow * p.matrixElementsPerRow
  for (client <- 0 until 2) {
    poke(c.io.writeRequest(client).valid, 1)
    poke(c.io.writeRequest(client).bits.address,
      localWriteBase + client * p.nLanes)
    for (lane <- 0 until p.nLanes) {
      poke(c.io.writeRequest(client).bits.data(lane),
        0x2000 + client * 0x1000 + lane)
      poke(c.io.writeRequest(client).bits.laneMask(lane), 1)
    }
    assert(peek(c.io.writeRequest(client).ready) == 1,
      s"local write of matrix fragment $client was not accepted")
  }
  step(1)
  for (client <- 0 until 2) poke(c.io.writeRequest(client).valid, 0)
  checkMatrixRow(localWriteRow,
    (0 until p.matrixElementsPerRow).map { element =>
      val client = element / p.nLanes
      val lane = element % p.nLanes
      0x2000 + client * 0x1000 + lane
    })
  assert(peek(c.io.busy) == 0)
}

class VpuMatrixScratchpadSpec extends ChiselFlatSpec {
  behavior of "VpuBankedScratchpad matrix ports"

  it should "round-trip sub-banked rows and prioritize four Gemmini ports" in {
    val p = VpuMatrixScratchpadTestParams.p
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-matrix-scratchpad"),
      () => new VpuBankedScratchpad(p)) { c =>
      new VpuMatrixScratchpadTester(c, p)
    } should be (true)
  }

  it should "retain one read and write mport per physical sub-bank" in {
    val p = VpuMatrixScratchpadTestParams.p
    val chirrtl = (new ChiselStage).emitChirrtl(
      new VpuBankedScratchpad(p))
    val memoryLines = chirrtl.linesIterator.filter(_.contains("memories_"))
      .toVector
    val readers = memoryLines.count(_.contains("read mport"))
    val writers = memoryLines.count(_.contains("write mport"))
    assert(readers == p.physicalBanks,
      s"expected one reader per physical bank, found $readers")
    assert(writers == p.physicalBanks,
      s"expected one writer per physical bank, found $writers")
  }

  it should "split and gather a masked 32-element row over two sub-banks" in {
    val p = VpuWideMatrixScratchpadTestParams.p
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-wide-matrix-scratchpad"),
      () => new VpuBankedScratchpad(p)) { c =>
      new VpuWideMatrixScratchpadTester(c, p)
    } should be (true)
  }

  it should "assert when two matrix clients use one logical bank" in {
    val p = VpuMatrixScratchpadTestParams.p
    assertThrows[treadle.executable.StopException] {
      chisel3.iotesters.Driver.execute(Array(
        "--backend-name", "treadle",
        "--target-dir", "test_run_dir/vpu-matrix-bank-collision"),
        () => new VpuBankedScratchpad(p)) { c =>
        new VpuMatrixLogicalBankCollisionTester(c, p)
      }
    }
  }
}
