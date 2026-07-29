package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

import scala.util.Random

/** Queue-level regression for the three independently scheduled command
  * classes. The DMA model deliberately stretches ready/valid so command loss
  * and completion-counter mistakes surface as a FENCE timeout.
  */
class VpuQueueConcurrencyTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  private val random = new Random(0x564d4151L)
  private val hostBase = BigInt("100000", 16)

  poke(c.io.command.valid, 0)
  poke(c.io.response.ready, 0)
  poke(c.io.dma.readDescriptor.ready, 0)
  poke(c.io.dma.readData.valid, 0)
  poke(c.io.dma.readData.bits.error, 0)
  poke(c.io.dma.readData.bits.fault.vaddr, 0)
  poke(c.io.dma.readData.bits.fault.cause, VpuDmaFaultCause.None)
  poke(c.io.dma.readData.bits.fault.isWrite, 0)
  poke(c.io.dma.readData.bits.commandTag, 0)
  poke(c.io.dma.writeDescriptor.ready, 0)
  poke(c.io.dma.writeData.ready, 0)
  poke(c.io.dma.writeCompletion.valid, 0)
  poke(c.io.dma.writeCompletion.bits.commandTag, 0)
  poke(c.io.dma.writeCompletion.bits.error, 0)
  poke(c.io.dma.writeCompletion.bits.fault.vaddr, 0)
  poke(c.io.dma.writeCompletion.bits.fault.cause, VpuDmaFaultCause.None)
  poke(c.io.dma.writeCompletion.bits.fault.isWrite, 0)
  poke(c.io.dma.clearFaultDone, 0)
  poke(c.io.dma.halted, 0)
  step(3)

  def driveCommand(opcode: Int, rd: Int = 0, rs1: Int = 0, rs2: Int = 0,
                   funct1: Int = 0, payload: BigInt = 0,
                   xd: Boolean = false, roccRd: Int = 0): Unit = {
    poke(c.io.command.valid, 1)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd, rs1, rs2,
        funct1 = funct1) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, xd)
  }

  def issue(opcode: Int, rd: Int = 0, rs1: Int = 0, rs2: Int = 0,
            funct1: Int = 0, payload: BigInt = 0,
            xd: Boolean = false, roccRd: Int = 0): Unit = {
    driveCommand(opcode, rd, rs1, rs2, funct1, payload, xd, roccRd)
    var timeout = 400
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"command 0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  def response(expectedRd: Int): BigInt = {
    var timeout = 1500
    while (peek(c.io.response.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"response rd=$expectedRd timed out")
    assert(peek(c.io.response.bits.rd) == expectedRd)
    val data = peek(c.io.response.bits.data)
    poke(c.io.response.ready, 1)
    step(1)
    poke(c.io.response.ready, 0)
    data
  }

  def acceptReadDescriptor(expectedSpad: Int): Unit = {
    poke(c.io.dma.readDescriptor.ready, 1)
    var timeout = 400
    while (peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"read descriptor base=$expectedSpad timed out")
    assert(peek(c.io.dma.readDescriptor.bits.vaddr) == hostBase)
    assert(peek(c.io.dma.readDescriptor.bits.spadElement) == expectedSpad)
    assert(peek(c.io.dma.readDescriptor.bits.elementCount) == p.vLen)
    readCommandTags.update(expectedSpad,
      peek(c.io.dma.readDescriptor.bits.commandTag))
    step(1)
    poke(c.io.dma.readDescriptor.ready, 0)
  }

  val readCommandTags = scala.collection.mutable.Map.empty[Int, BigInt]
  val writeCommandTags = scala.collection.mutable.Map.empty[Int, BigInt]

  def acceptWriteDescriptor(expectedSpad: Int): Unit = {
    poke(c.io.dma.writeDescriptor.ready, 1)
    var timeout = 500
    while (peek(c.io.dma.writeDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"write descriptor base=$expectedSpad timed out")
    assert(peek(c.io.dma.writeDescriptor.bits.vaddr) == hostBase)
    assert(peek(c.io.dma.writeDescriptor.bits.spadElement) == expectedSpad)
    assert(peek(c.io.dma.writeDescriptor.bits.elementCount) == p.vLen)
    writeCommandTags.update(expectedSpad,
      peek(c.io.dma.writeDescriptor.bits.commandTag))
    step(1)
    poke(c.io.dma.writeDescriptor.ready, 0)
  }

  def packBeat(values: Seq[Float]): BigInt =
    values.zipWithIndex.map { case (x, lane) => bits(x) << (32 * lane) }
      .reduce(_ | _)

  def sendReadVector(spadBase: Int, valueBase: Float): Unit = {
    for (beat <- 0 until p.vLen / p.dmaElementsPerBeat) {
      // Deterministic pseudo-random valid gaps exercise input backpressure.
      if (random.nextBoolean()) { step(1) }
      val values = (0 until p.dmaElementsPerBeat)
        .map(i => valueBase + beat * p.dmaElementsPerBeat + i)
      poke(c.io.dma.readData.valid, 1)
      poke(c.io.dma.readData.bits.data, packBeat(values))
      poke(c.io.dma.readData.bits.elementMask,
        (BigInt(1) << p.dmaElementsPerBeat) - 1)
      poke(c.io.dma.readData.bits.spadElement,
        spadBase + beat * p.dmaElementsPerBeat)
      poke(c.io.dma.readData.bits.commandTag, readCommandTags(spadBase))
      poke(c.io.dma.readData.bits.last,
        if (beat == p.vLen / p.dmaElementsPerBeat - 1) 1 else 0)
      poke(c.io.dma.readData.bits.error, 0)
      var timeout = 200
      while (peek(c.io.dma.readData.ready) == 0 && timeout > 0) {
        step(1)
        timeout -= 1
      }
      assert(timeout > 0, "read data backpressure did not release")
      step(1)
      poke(c.io.dma.readData.valid, 0)
    }
  }

  def completeWrite(expectedSpad: Int): Unit = {
    poke(c.io.dma.writeCompletion.valid, 1)
    poke(c.io.dma.writeCompletion.bits.commandTag,
      writeCommandTags(expectedSpad))
    poke(c.io.dma.writeCompletion.bits.error, 0)
    var timeout = 200
    while (peek(c.io.dma.writeCompletion.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "write completion timed out")
    step(1)
    poke(c.io.dma.writeCompletion.valid, 0)
  }

  /** Drain exactly one vector store payload without supplying its TL result. */
  def drainStoreData(expectedSpad: Int,
                     expected: Option[Seq[Float]] = None): Unit = {
    var sawReadyStall = false
    for (beat <- 0 until p.vLen / p.dmaElementsPerBeat) {
      var timeout = 400
      while (peek(c.io.dma.writeData.valid) == 0 && timeout > 0) {
        poke(c.io.dma.writeData.ready, 0)
        step(1)
        timeout -= 1
      }
      assert(timeout > 0, s"store beat $beat timed out")
      // Force at least one held-valid cycle, then add random stalls.
      val stalls = if (beat == 0) 1 else if (random.nextBoolean()) 1 else 0
      for (_ <- 0 until stalls) {
        poke(c.io.dma.writeData.ready, 0)
        assert(peek(c.io.dma.writeData.valid) == 1)
        step(1)
        sawReadyStall = true
      }
      poke(c.io.dma.writeData.ready, 1)
      assert(peek(c.io.dma.writeData.bits.spadElement) ==
        expectedSpad + beat * p.dmaElementsPerBeat)
      assert(peek(c.io.dma.writeData.bits.commandTag) ==
        writeCommandTags(expectedSpad))
      assert(peek(c.io.dma.writeData.bits.elementMask) ==
        (BigInt(1) << p.dmaElementsPerBeat) - 1)
      assert(peek(c.io.dma.writeData.bits.last) ==
        (if (beat == p.vLen / p.dmaElementsPerBeat - 1) 1 else 0))
      expected.foreach { vector =>
        val packed = peek(c.io.dma.writeData.bits.data)
        for (lane <- 0 until p.dmaElementsPerBeat) {
          val actual = (packed >> (32 * lane)) & BigInt("ffffffff", 16)
          val wanted = bits(vector(beat * p.dmaElementsPerBeat + lane))
          assert(actual == wanted,
            s"store[$beat,$lane]=0x${actual.toString(16)} expected=0x${wanted.toString(16)}")
        }
      }
      step(1)
      poke(c.io.dma.writeData.ready, 0)
    }
    assert(sawReadyStall, "store ready path was never backpressured")
  }

  def drainStore(expectedSpad: Int,
                 expected: Option[Seq[Float]] = None): Unit = {
    drainStoreData(expectedSpad, expected)
    completeWrite(expectedSpad)
  }

  // ----------------------------------------------------------------------
  // 1) The load class admits exactly loadQueueEntries outstanding commands.
  // ----------------------------------------------------------------------
  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = hostBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 15, payload = 0)
  val initialBases = Seq(0, 16, 32, 48, 64)
  initialBases.zipWithIndex.foreach { case (base, index) =>
    issue(VpuOpcode.C_WRITE_GP, rd = index, payload = base)
  }
  for (index <- 0 until p.loadQueueEntries) {
    issue(VpuOpcode.H_PREFETCH_V, rd = index, rs1 = 15, rs2 = 0)
  }

  // Keep the fifth command stable while the first four consume the active
  // slot plus all remaining queue credit.
  driveCommand(VpuOpcode.H_PREFETCH_V, rd = 4, rs1 = 15, rs2 = 0)
  for (_ <- 0 until 3) {
    assert(peek(c.io.command.ready) == 0,
      "load queue admitted more descriptors than configured")
    step(1)
  }
  // Descriptor issue must run ahead of SRAM writeback.  Accept every one of
  // the four non-overlapping loads before returning a single DMA data beat.
  // The old lDescriptor->lData FSM timed out on the second call here because
  // it held the request side until the first vector reached SRAM.
  initialBases.take(p.loadQueueEntries).foreach(acceptReadDescriptor)
  assert(peek(c.io.dma.readData.valid) == 0,
    "test unexpectedly supplied SRAM writeback data during load issue")
  assert(peek(c.io.command.ready) == 0,
    "descriptor issue incorrectly restored architectural load credit")

  sendReadVector(initialBases.head, 0.0f)
  var commandTimeout = 100
  while (peek(c.io.command.ready) == 0 && commandTimeout > 0) {
    step(1)
    commandTimeout -= 1
  }
  assert(commandTimeout > 0, "load queue did not restore credit on completion")
  step(1) // fifth command fires
  poke(c.io.command.valid, 0)
  initialBases.slice(1, p.loadQueueEntries).zipWithIndex.foreach {
      case (base, index) =>
    sendReadVector(base, (index + 1) * 10.0f)
  }
  acceptReadDescriptor(initialBases.last)
  sendReadVector(initialBases.last, p.loadQueueEntries * 10.0f)

  // ----------------------------------------------------------------------
  // 2) WAIT(LD) releases after only preceding loads, even with an older store
  // still blocked on its output ready path.
  // ----------------------------------------------------------------------
  val waitLoadBase = 80
  issue(VpuOpcode.C_WRITE_GP, rd = 5, payload = waitLoadBase)
  issue(VpuOpcode.H_STORE_V, rd = 0, rs1 = 15, rs2 = 0)
  acceptWriteDescriptor(0)
  issue(VpuOpcode.H_PREFETCH_V, rd = 5, rs1 = 15, rs2 = 0)
  acceptReadDescriptor(waitLoadBase)
  issue(VpuOpcode.C_WAIT, payload = VpuWaitMask.Load)

  driveCommand(VpuOpcode.C_WRITE_GP, rd = 6, payload = 0x123)
  assert(peek(c.io.command.ready) == 0,
    "command crossed an unsatisfied WAIT(LD)")
  sendReadVector(waitLoadBase, 50.0f)
  commandTimeout = 200
  while (peek(c.io.command.ready) == 0 && commandTimeout > 0) {
    step(1)
    commandTimeout -= 1
  }
  assert(commandTimeout > 0, "WAIT(LD) did not observe load completion")
  assert(peek(c.io.dma.writeData.valid) == 1,
    "WAIT(LD) incorrectly waited for the outstanding store")
  step(1)
  poke(c.io.command.valid, 0)
  issue(VpuOpcode.C_READ, rd = 6, rs1 = VpuReadSelector.Gp,
    xd = true, roccRd = 6)
  assert(response(6) == 0x123)

  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 7)
  for (_ <- 0 until 4) {
    assert(peek(c.io.response.valid) == 0,
      "FENCE completed while the older store remained outstanding")
    step(1)
  }
  drainStore(0)
  assert((response(7) & 7) == 0)

  // ----------------------------------------------------------------------
  // 3) Disjoint LD/EX/ST descriptors make forward progress together. A
  // younger output store is hazard-stalled until the execute write retires.
  // ----------------------------------------------------------------------
  val executeDestination = 96
  val nextLoadBase = 128
  issue(VpuOpcode.C_WRITE_GP, rd = 7, payload = executeDestination)
  issue(VpuOpcode.C_WRITE_GP, rd = 8, payload = nextLoadBase)
  issue(VpuOpcode.C_WRITE_FP, rd = 0, payload = bits(1.0f))
  issue(VpuOpcode.H_STORE_V, rd = 0, rs1 = 15, rs2 = 0)
  issue(VpuOpcode.V_ADD_VF, rd = 7, rs1 = 2, rs2 = 0)
  issue(VpuOpcode.H_PREFETCH_V, rd = 8, rs1 = 15, rs2 = 0)
  issue(VpuOpcode.H_STORE_V, rd = 7, rs1 = 15, rs2 = 0)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 8)

  acceptWriteDescriptor(0)
  acceptReadDescriptor(nextLoadBase)
  var overlapTimeout = 100
  while ((peek(c.io.dma.writeData.valid) == 0 ||
      peek(c.io.debugState) == 0) && overlapTimeout > 0) {
    step(1)
    overlapTimeout -= 1
  }
  assert(overlapTimeout > 0, "LD/EX/ST engines never became concurrent")
  sendReadVector(nextLoadBase, 80.0f)
  drainStore(0)

  acceptWriteDescriptor(executeDestination)
  val sourceValues = (0 until p.vLen).map(i => 20.0f + i)
  val expectedOutput = sourceValues.map(_ + 1.0f)
  drainStore(executeDestination, Some(expectedOutput))
  assert((response(8) & 7) == 0)

  // ----------------------------------------------------------------------
  // 4) Store descriptor issue and SRAM streaming run ahead of TL D
  // completion. Both descriptors must be accepted while the first payload is
  // held, and both payloads must drain before either completion is returned.
  // ----------------------------------------------------------------------
  issue(VpuOpcode.H_STORE_V, rd = 0, rs1 = 15, rs2 = 0)
  issue(VpuOpcode.H_STORE_V, rd = 7, rs1 = 15, rs2 = 0)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 9)
  acceptWriteDescriptor(0)
  var firstStoreTimeout = 100
  while(peek(c.io.dma.writeData.valid) == 0 && firstStoreTimeout > 0) {
    step(1)
    firstStoreTimeout -= 1
  }
  assert(firstStoreTimeout > 0,
    "first store SRAM stream did not become valid")
  assert(peek(c.io.dma.writeData.valid) == 1,
    "first store did not begin its independent SRAM stream")
  acceptWriteDescriptor(executeDestination)
  assert(writeCommandTags(0) != writeCommandTags(executeDestination),
    "two live stores reused one command tag")
  assert(peek(c.io.dma.writeCompletion.valid) == 0,
    "test supplied a completion before both descriptors were issued")

  drainStoreData(0)
  drainStoreData(executeDestination, Some(expectedOutput))
  for (_ <- 0 until 3) {
    assert(peek(c.io.response.valid) == 0,
      "FENCE retired stores before tagged TL completions")
    step(1)
  }
  completeWrite(0)
  assert(peek(c.io.response.valid) == 0,
    "FENCE retired after only one of two store completions")
  completeWrite(executeDestination)
  assert((response(9) & 7) == 0)

  assert(peek(c.io.perfCounters(VpuPerfIndex.DmaExecuteOverlapCycles)) > 0,
    "disjoint DMA and execute work never overlapped")
  assert(peek(c.io.perfCounters(VpuPerfIndex.HazardStallCycles)) > 0,
    "dependent output store did not exercise the age-ordered scoreboard")
  assert(peek(c.io.perfCounters(VpuPerfIndex.DmaReadBytes)) == 7 * 64)
  assert(peek(c.io.perfCounters(VpuPerfIndex.DmaWriteBytes)) == 5 * 64)
}

class VpuQueueConcurrencySpec extends ChiselFlatSpec {
  behavior of "VpuCore LD/EX/ST queues"
  it should "enforce credit, WAIT sequence, hazards, and concurrent progress" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2, vSpadKB = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-queue-concurrency"),
      () => new VpuCore(p)) { c =>
      new VpuQueueConcurrencyTester(c, p)
    } should be (true)
  }
}
