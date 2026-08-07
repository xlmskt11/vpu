package vpu

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.diplomacy.{AddressSet, LazyModule,
  LazyModuleImp, RegionType, TransferSizes}
import freechips.rocketchip.system.DefaultConfig
import freechips.rocketchip.tile.{RocketTileParams, TileKey,
  TileVisibilityNodeKey}
import freechips.rocketchip.tilelink._

/** Test-only non-FIFO manager for the pipelined writer.
  *
  * Every multi-beat Put is retained by source ID until the tester explicitly
  * selects that source for its AccessAck.  This makes source exhaustion,
  * legal post-D reuse, and arbitrarily reordered D responses deterministic.
  */
private class VpuRollingWriteTLManager(
    beatBytes: Int,
    maxTransactions: Int)(implicit p: Parameters) extends LazyModule {
  require(beatBytes == 16,
    "the rolling writer regression assumes the 128-bit VPU DMA bus")
  require(maxTransactions > 1)
  private val lineBytes = 64

  val node = TLManagerNode(Seq(TLSlavePortParameters.v1(
    managers = Seq(TLSlaveParameters.v1(
      address = Seq(AddressSet(0x0, 0xffff)),
      regionType = RegionType.UNCACHED,
      supportsGet = TransferSizes(1, lineBytes),
      supportsPutFull = TransferSizes(1, lineBytes),
      supportsPutPartial = TransferSizes(1, lineBytes),
      mayDenyPut = true,
      fifoId = None)),
    beatBytes = beatBytes,
    minLatency = 1)))

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (tl, edge) = node.in.head
    private val countBits = math.max(1, log2Ceil(maxTransactions + 1))
    private val sourceBits = math.max(1, edge.bundle.sourceBits)
    private val endSourceId = edge.client.endSourceId
    private val transactionIndexBits =
      math.max(1, log2Ceil(maxTransactions))

    val io = IO(new Bundle {
      val releaseValid = Input(Bool())
      val releaseSource = Input(UInt(16.W))
      val releaseDenied = Input(Bool())
      val releaseFire = Output(Bool())
      val writeAccepted = Output(UInt(countBits.W))
      val writeAcknowledged = Output(UInt(countBits.W))
      val physicalABeats = Output(UInt(16.W))
      val acceptedSources = Output(Vec(maxTransactions, UInt(16.W)))
      val acceptedAddresses = Output(Vec(maxTransactions, UInt(64.W)))
      val acceptedSizes = Output(Vec(maxTransactions, UInt(8.W)))
      val acceptedCycles = Output(Vec(maxTransactions, UInt(32.W)))
      val responseCycles = Output(Vec(maxTransactions, UInt(32.W)))
    })

    val pending = RegInit(VecInit(Seq.fill(endSourceId)(false.B)))
    val requestComplete = RegInit(VecInit(
      Seq.fill(endSourceId)(false.B)))
    val requestSize = Reg(Vec(
      endSourceId, UInt(edge.bundle.sizeBits.W)))
    val requestAddress = Reg(Vec(
      endSourceId, UInt(edge.bundle.addressBits.W)))
    val requestLogIndex = Reg(Vec(
      endSourceId, UInt(transactionIndexBits.W)))

    val accepted = RegInit(0.U(countBits.W))
    val acknowledged = RegInit(0.U(countBits.W))
    val physicalABeats = RegInit(0.U(16.W))
    val acceptedSources = RegInit(VecInit(
      Seq.fill(maxTransactions)(0.U(16.W))))
    val acceptedAddresses = RegInit(VecInit(
      Seq.fill(maxTransactions)(0.U(64.W))))
    val acceptedSizes = RegInit(VecInit(
      Seq.fill(maxTransactions)(0.U(8.W))))
    val acceptedCycles = RegInit(VecInit(
      Seq.fill(maxTransactions)(0.U(32.W))))
    val responseCycles = RegInit(VecInit(
      Seq.fill(maxTransactions)(0.U(32.W))))
    val cycle = RegInit(0.U(32.W))
    cycle := cycle + 1.U

    io.writeAccepted := accepted
    io.writeAcknowledged := acknowledged
    io.physicalABeats := physicalABeats
    io.acceptedSources := acceptedSources
    io.acceptedAddresses := acceptedAddresses
    io.acceptedSizes := acceptedSizes
    io.acceptedCycles := acceptedCycles
    io.responseCycles := responseCycles

    val aIsWrite = tl.a.bits.opcode === TLMessages.PutFullData ||
      tl.a.bits.opcode === TLMessages.PutPartialData
    val aSourceInRange = tl.a.bits.source < endSourceId.U
    val aSource = tl.a.bits.source(sourceBits - 1, 0)
    val collecting = RegInit(false.B)
    val collectingSource = Reg(UInt(sourceBits.W))
    val collectingSize = Reg(UInt(edge.bundle.sizeBits.W))
    val collectingAddress = Reg(UInt(edge.bundle.addressBits.W))

    val firstCanEnter = aSourceInRange && !pending(aSource) &&
      accepted < maxTransactions.U
    val continuationCanEnter = aSourceInRange && collecting &&
      aSource === collectingSource
    tl.a.ready := aIsWrite && Mux(collecting,
      continuationCanEnter, firstCanEnter)

    when(collecting) {
      assert(tl.a.valid,
        "VPU writer inserted a bubble inside a multi-beat Put")
    }
    when(tl.a.valid) {
      assert(aIsWrite, "rolling writer manager only accepts Puts")
      assert(aSourceInRange,
        "rolling writer manager received an invalid source")
    }
    when(tl.a.fire) {
      physicalABeats := physicalABeats + 1.U
      assert(tl.a.bits.size === log2Ceil(lineBytes).U,
        "rolling writer did not issue a size=6 cache-line Put")
      assert(tl.a.bits.address(log2Ceil(lineBytes) - 1, 0) === 0.U,
        "rolling writer cache-line Put was not naturally aligned")
      assert(tl.a.bits.mask.andR,
        "aligned rolling writer Put unexpectedly used a partial beat")

      when(edge.first(tl.a)) {
        assert(!collecting,
          "a new Put started before the previous Put reached A last")
        assert(!pending(aSource),
          "VPU writer reused a TileLink source before its D response")
        assert(accepted < maxTransactions.U,
          "rolling writer transaction log overflowed")
        pending(aSource) := true.B
        requestComplete(aSource) := edge.last(tl.a)
        requestSize(aSource) := tl.a.bits.size
        requestAddress(aSource) := tl.a.bits.address
        requestLogIndex(aSource) :=
          accepted(transactionIndexBits - 1, 0)
        acceptedSources(accepted) := tl.a.bits.source
        acceptedAddresses(accepted) := tl.a.bits.address
        acceptedSizes(accepted) := tl.a.bits.size
        acceptedCycles(accepted) := cycle
        accepted := accepted + 1.U
        when(!edge.last(tl.a)) {
          collecting := true.B
          collectingSource := aSource
          collectingSize := tl.a.bits.size
          collectingAddress := tl.a.bits.address
        }
      }.otherwise {
        assert(collecting && aSource === collectingSource,
          "multi-beat Put changed source ID")
        assert(tl.a.bits.size === collectingSize &&
          tl.a.bits.address === collectingAddress,
          "multi-beat Put changed size or address")
        when(edge.last(tl.a)) {
          collecting := false.B
          requestComplete(aSource) := true.B
        }
      }
    }

    val selectedInRange = io.releaseSource < endSourceId.U
    val selectedSource = io.releaseSource(sourceBits - 1, 0)
    val selectedEligible = selectedInRange && pending(selectedSource) &&
      requestComplete(selectedSource)
    tl.d.valid := io.releaseValid && selectedEligible
    tl.d.bits := edge.AccessAck(
      selectedSource, requestSize(selectedSource))
    tl.d.bits.denied := io.releaseDenied
    io.releaseFire := tl.d.fire

    when(io.releaseValid) {
      assert(selectedInRange,
        "tester selected an out-of-range writer source")
      assert(pending(selectedSource),
        "tester selected a writer source which is not pending")
      assert(requestComplete(selectedSource),
        "tester attempted D before the selected Put reached A last")
    }
    when(tl.d.fire) {
      val logIndex = requestLogIndex(selectedSource)
      pending(selectedSource) := false.B
      requestComplete(selectedSource) := false.B
      responseCycles(logIndex) := cycle
      acknowledged := acknowledged + 1.U
    }

    tl.b.valid := false.B
    tl.b.bits := DontCare
    tl.c.ready := true.B
    tl.e.ready := true.B
  }
}

private class VpuTLRollingWriterHarness(val vpuParams: VpuParams)
    (implicit p: Parameters) extends LazyModule {
  val visibilityNode = TLEphemeralNode()
  private val dutParameters = p.alterMap(Map(
    TileKey -> RocketTileParams(),
    TileVisibilityNodeKey -> visibilityNode))
  val dut = LazyModule(new VpuTLMemory(vpuParams)(dutParameters))
  // Two successful commands plus one faulting command which fills every
  // source; its younger accepted command must be terminated without issuing.
  val maxTransactions = 3 * vpuParams.dmaMaxInFlight
  val manager = LazyModule(new VpuRollingWriteTLManager(
    beatBytes = vpuParams.dmaBusWidth / 8,
    maxTransactions = maxTransactions)(dutParameters))
  manager.node := visibilityNode := dut.idNode

  lazy val module = new VpuTLRollingWriterHarnessImp(this)
}

private class VpuTLRollingWriterHarnessImp(
    outer: VpuTLRollingWriterHarness) extends LazyModuleImp(outer) {
  private val n = outer.maxTransactions
  private val countBits = math.max(1, log2Ceil(n + 1))
  val io = IO(new Bundle {
    val dma = Flipped(new VpuDmaIO(outer.vpuParams))
    val releaseValid = Input(Bool())
    val releaseSource = Input(UInt(16.W))
    val releaseDenied = Input(Bool())
    val releaseFire = Output(Bool())
    val writeAccepted = Output(UInt(countBits.W))
    val writeAcknowledged = Output(UInt(countBits.W))
    val physicalABeats = Output(UInt(16.W))
    val acceptedSources = Output(Vec(n, UInt(16.W)))
    val acceptedAddresses = Output(Vec(n, UInt(64.W)))
    val acceptedSizes = Output(Vec(n, UInt(8.W)))
    val acceptedCycles = Output(Vec(n, UInt(32.W)))
    val responseCycles = Output(Vec(n, UInt(32.W)))
    val busy = Output(Bool())
    val fault = Output(Bool())
  })

  outer.dut.module.io.dma <> io.dma
  outer.manager.module.io.releaseValid := io.releaseValid
  outer.manager.module.io.releaseSource := io.releaseSource
  outer.manager.module.io.releaseDenied := io.releaseDenied
  io.releaseFire := outer.manager.module.io.releaseFire
  io.writeAccepted := outer.manager.module.io.writeAccepted
  io.writeAcknowledged := outer.manager.module.io.writeAcknowledged
  io.physicalABeats := outer.manager.module.io.physicalABeats
  io.acceptedSources := outer.manager.module.io.acceptedSources
  io.acceptedAddresses := outer.manager.module.io.acceptedAddresses
  io.acceptedSizes := outer.manager.module.io.acceptedSizes
  io.acceptedCycles := outer.manager.module.io.acceptedCycles
  io.responseCycles := outer.manager.module.io.responseCycles
  io.busy := outer.dut.module.io.busy
  io.fault := outer.dut.module.io.fault

  // Bare M-mode translation exercises the real FrontendTLB path without a
  // page-table model.
  outer.dut.module.io.ptw.req.ready := true.B
  outer.dut.module.io.ptw.resp.valid := false.B
  outer.dut.module.io.ptw.resp.bits :=
    0.U.asTypeOf(outer.dut.module.io.ptw.resp.bits)
  outer.dut.module.io.ptw.ptbr :=
    0.U.asTypeOf(outer.dut.module.io.ptw.ptbr)
  outer.dut.module.io.ptw.hgatp :=
    0.U.asTypeOf(outer.dut.module.io.ptw.hgatp)
  outer.dut.module.io.ptw.vsatp :=
    0.U.asTypeOf(outer.dut.module.io.ptw.vsatp)
  outer.dut.module.io.ptw.status :=
    0.U.asTypeOf(outer.dut.module.io.ptw.status)
  outer.dut.module.io.ptw.hstatus :=
    0.U.asTypeOf(outer.dut.module.io.ptw.hstatus)
  outer.dut.module.io.ptw.gstatus :=
    0.U.asTypeOf(outer.dut.module.io.ptw.gstatus)
  outer.dut.module.io.ptw.pmp :=
    0.U.asTypeOf(outer.dut.module.io.ptw.pmp)
  outer.dut.module.io.ptw.customCSRs :=
    0.U.asTypeOf(outer.dut.module.io.ptw.customCSRs)
}

private class VpuTLRollingWriterTester(
    c: VpuTLRollingWriterHarnessImp,
    p: VpuParams) extends PeekPokeTester(c) {
  private val nSources = p.dmaMaxInFlight
  private val transactionsPerCommand =
    p.vLen * p.storageBytes / p.dmaMaxBytes
  private val beatsPerCommand = p.vLen / p.dmaElementsPerBeat
  private val beatsPerTransaction = p.dmaMaxBytes /
    (p.dmaBusWidth / 8)
  require(nSources == 8,
    "the rolling writer regression is defined for eight source IDs")
  require(transactionsPerCommand == nSources)
  require(beatsPerCommand == 32 && beatsPerTransaction == 4)

  poke(c.io.dma.readDescriptor.valid, 0)
  poke(c.io.dma.readDescriptor.bits.rowCount, 0)
  poke(c.io.dma.readDescriptor.bits.hostStrideBytes, 0)
  poke(c.io.dma.readData.ready, 0)
  poke(c.io.dma.writeDescriptor.valid, 0)
  poke(c.io.dma.writeDescriptor.bits.rowCount, 0)
  poke(c.io.dma.writeDescriptor.bits.hostStrideBytes, 0)
  poke(c.io.dma.writeData.valid, 0)
  poke(c.io.dma.writeCompletion.ready, 0)
  poke(c.io.dma.clearFault, 0)
  poke(c.io.releaseValid, 0)
  poke(c.io.releaseSource, 0)
  poke(c.io.releaseDenied, 0)
  step(5)

  private def waitFor(signal: Bits, value: BigInt,
      clue: String, cycles: Int = 10000): Unit = {
    var timeout = cycles
    while (peek(signal) != value && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, clue)
  }

  private def issueDescriptor(address: BigInt, spadElement: Int,
      commandTag: Int): Unit = {
    poke(c.io.dma.writeDescriptor.bits.vaddr, address)
    poke(c.io.dma.writeDescriptor.bits.spadElement, spadElement)
    poke(c.io.dma.writeDescriptor.bits.elementCount, p.vLen)
    poke(c.io.dma.writeDescriptor.bits.commandTag, commandTag)
    poke(c.io.dma.writeDescriptor.bits.status.prv, 3)
    poke(c.io.dma.writeDescriptor.bits.status.dprv, 3)
    poke(c.io.dma.writeDescriptor.bits.status.dv, 0)
    poke(c.io.dma.writeDescriptor.valid, 1)
    var timeout = 1000
    while (peek(c.io.dma.writeDescriptor.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"write descriptor tag $commandTag was not admitted")
    step(1)
    poke(c.io.dma.writeDescriptor.valid, 0)
  }

  private def beatData(command: Int, beat: Int): BigInt =
    (0 until p.dmaElementsPerBeat).foldLeft(BigInt(0)) { (acc, lane) =>
      val value = BigInt((command << 24) | (beat << 8) | lane)
      acc | (value << (lane * p.storageBits))
    }

  private def sendBeat(commandTag: Int, spadBase: Int,
      beat: Int): Unit = {
    poke(c.io.dma.writeData.bits.data, beatData(commandTag, beat))
    poke(c.io.dma.writeData.bits.elementMask,
      (BigInt(1) << p.dmaElementsPerBeat) - 1)
    poke(c.io.dma.writeData.bits.spadElement,
      spadBase + beat * p.dmaElementsPerBeat)
    poke(c.io.dma.writeData.bits.commandTag, commandTag)
    poke(c.io.dma.writeData.bits.last,
      if (beat == beatsPerCommand - 1) 1 else 0)
    poke(c.io.dma.writeData.valid, 1)
    var timeout = 3000
    while (peek(c.io.dma.writeData.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"write data tag=$commandTag beat=$beat timed out")
    step(1)
    poke(c.io.dma.writeData.valid, 0)
  }

  private def release(source: BigInt, expectedCount: Int,
      denied: Boolean = false): Unit = {
    poke(c.io.releaseSource, source)
    poke(c.io.releaseDenied, if (denied) 1 else 0)
    poke(c.io.releaseValid, 1)
    var timeout = 1000
    while (peek(c.io.releaseFire) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"D response for source $source was not accepted")
    step(1)
    poke(c.io.releaseValid, 0)
    poke(c.io.releaseDenied, 0)
    waitFor(c.io.writeAcknowledged, expectedCount,
      s"D response count did not reach $expectedCount")
  }

  private def expectHeldCompletion(tag: Int, holdCycles: Int): Unit = {
    waitFor(c.io.dma.writeCompletion.valid, 1,
      s"completion tag $tag did not appear")
    val heldTag = peek(c.io.dma.writeCompletion.bits.commandTag)
    val heldError = peek(c.io.dma.writeCompletion.bits.error)
    val heldVaddr = peek(c.io.dma.writeCompletion.bits.fault.vaddr)
    val heldCause = peek(c.io.dma.writeCompletion.bits.fault.cause)
    val heldIsWrite = peek(c.io.dma.writeCompletion.bits.fault.isWrite)
    assert(heldTag == tag)
    assert(heldError == 0)
    for (_ <- 0 until holdCycles) {
      assert(peek(c.io.dma.writeCompletion.valid) == 1)
      assert(peek(c.io.dma.writeCompletion.bits.commandTag) == heldTag)
      assert(peek(c.io.dma.writeCompletion.bits.error) == heldError)
      assert(peek(c.io.dma.writeCompletion.bits.fault.vaddr) == heldVaddr)
      assert(peek(c.io.dma.writeCompletion.bits.fault.cause) == heldCause)
      assert(peek(c.io.dma.writeCompletion.bits.fault.isWrite) ==
        heldIsWrite)
      step(1)
    }
  }

  private def consumeCompletion(): Unit = {
    poke(c.io.dma.writeCompletion.ready, 1)
    step(1)
    poke(c.io.dma.writeCompletion.ready, 0)
  }

  private def expectHeldFaultCompletion(tag: Int, vaddr: BigInt,
      holdCycles: Int): Unit = {
    waitFor(c.io.dma.writeCompletion.valid, 1,
      s"fault completion tag $tag did not appear")
    assert(peek(c.io.dma.writeCompletion.bits.commandTag) == tag)
    assert(peek(c.io.dma.writeCompletion.bits.error) == 1)
    assert(peek(c.io.dma.writeCompletion.bits.fault.vaddr) == vaddr)
    assert(peek(c.io.dma.writeCompletion.bits.fault.cause) ==
      VpuDmaFaultCause.Access)
    assert(peek(c.io.dma.writeCompletion.bits.fault.isWrite) == 1)
    for (_ <- 0 until holdCycles) {
      assert(peek(c.io.dma.writeCompletion.valid) == 1)
      assert(peek(c.io.dma.writeCompletion.bits.commandTag) == tag)
      assert(peek(c.io.dma.writeCompletion.bits.error) == 1)
      assert(peek(c.io.dma.writeCompletion.bits.fault.vaddr) == vaddr)
      step(1)
    }
  }

  val firstHost = BigInt(0x4000)
  val secondHost = BigInt(0x5000)
  val firstSpad = 0
  val secondSpad = p.vLen
  val firstTag = 0
  val secondTag = 1

  // Descriptor admission is independent of prior payload and D lifetimes.
  issueDescriptor(firstHost, firstSpad, firstTag)
  issueDescriptor(secondHost, secondSpad, secondTag)

  // Fill all eight source slots with the first command while every D response
  // is held.  The second command may place one beat in the elastic payload
  // buffer even though no transaction source is currently free.
  for (beat <- 0 until beatsPerCommand) {
    sendBeat(firstTag, firstSpad, beat)
  }
  waitFor(c.io.writeAccepted, nSources,
    "first store did not occupy every writer source")
  waitFor(c.io.physicalABeats,
    nSources * beatsPerTransaction,
    "first store's final multi-beat Put did not reach A last")
  assert(peek(c.io.writeAcknowledged) == 0)
  assert(peek(c.io.dma.writeCompletion.valid) == 0)
  sendBeat(secondTag, secondSpad, beat = 0)

  val firstSources = (0 until nSources).map(i =>
    peek(c.io.acceptedSources(i)))
  assert(firstSources.distinct.size == nSources,
    s"first store reused a live source: ${firstSources.mkString(",")}")
  for (i <- 0 until nSources) {
    assert(peek(c.io.acceptedAddresses(i)) == firstHost + i * 64)
    assert(peek(c.io.acceptedSizes(i)) == 6)
    if (i > 0) {
      assert(peek(c.io.acceptedCycles(i)) ==
        peek(c.io.acceptedCycles(i - 1)) + beatsPerTransaction,
        s"writer inserted an A-channel bubble between cache lines " +
          s"${i - 1} and $i")
    }
  }

  // Free the most recently issued first-command source.  The second command
  // must reuse it and launch a Put while seven first-command D responses are
  // still outstanding and before the first command completion exists.
  val rollingSource = firstSources.last
  release(rollingSource, expectedCount = 1)
  for (beat <- 1 until beatsPerTransaction) {
    sendBeat(secondTag, secondSpad, beat)
  }
  waitFor(c.io.writeAccepted, nSources + 1,
    "second store waited for first-store command completion")
  assert(peek(c.io.acceptedSources(nSources)) == rollingSource,
    "second store did not reuse the sole returned source")
  assert(peek(c.io.acceptedCycles(nSources)) >
    peek(c.io.responseCycles(nSources - 1)),
    "writer reused a source before its previous D response")
  assert(peek(c.io.dma.writeCompletion.valid) == 0,
    "first command completed with seven D responses outstanding")

  // Keep the other seven old transactions live.  Repeatedly return only the
  // rolling source so all eight second-command transactions finish first.
  var acknowledged = 1
  for (transaction <- 0 until transactionsPerCommand) {
    val logIndex = nSources + transaction
    if (transaction > 0) {
      val firstBeat = transaction * beatsPerTransaction
      for (beat <- firstBeat until firstBeat + beatsPerTransaction) {
        sendBeat(secondTag, secondSpad, beat)
      }
      waitFor(c.io.writeAccepted, logIndex + 1,
        s"second-store transaction $transaction did not launch")
      assert(peek(c.io.acceptedSources(logIndex)) == rollingSource)
    }
    waitFor(c.io.physicalABeats,
      (logIndex + 1) * beatsPerTransaction,
      s"second-store transaction $transaction did not reach A last")
    acknowledged += 1
    release(rollingSource, expectedCount = acknowledged)
  }

  assert(peek(c.io.writeAccepted) == 2 * nSources)
  assert(peek(c.io.dma.writeCompletion.valid) == 0,
    "younger store bypassed the incomplete command-order head")

  // Return the seven older transactions in a deliberately non-FIFO order.
  // Only their final D may make the first ordered completion visible.
  val oldResponseOrder = Seq(3, 0, 6, 1, 5, 2, 4)
  for ((sourceIndex, responseIndex) <- oldResponseOrder.zipWithIndex) {
    acknowledged += 1
    release(firstSources(sourceIndex), expectedCount = acknowledged)
    if (responseIndex != oldResponseOrder.size - 1) {
      assert(peek(c.io.dma.writeCompletion.valid) == 0,
        "first store completed before its final D response")
    }
  }

  // Completion is ordered by descriptor even though every second-command D
  // preceded seven first-command D responses.  Both completions must also be
  // stable under downstream backpressure.
  expectHeldCompletion(firstTag, holdCycles = 4)
  consumeCompletion()
  expectHeldCompletion(secondTag, holdCycles = 3)
  consumeCompletion()

  waitFor(c.io.busy, 0,
    "writer remained busy after both ordered completions retired")
  assert(peek(c.io.fault) == 0)
  assert(peek(c.io.writeAcknowledged) == 2 * nSources)
  assert(peek(c.io.physicalABeats) ==
    2 * nSources * beatsPerTransaction)

  // Keep two more descriptors live, then deny one D from the older command.
  // The seven remaining irrevocable Puts must drain, the younger payload must
  // be consumed/discarded through last, and both tags must receive exactly one
  // ordered terminal completion.
  val faultHost = BigInt(0x6000)
  val faultYoungerHost = BigInt(0x7000)
  val faultTag = 2
  val faultYoungerTag = 3
  issueDescriptor(faultHost, firstSpad, faultTag)
  issueDescriptor(faultYoungerHost, secondSpad, faultYoungerTag)
  for (beat <- 0 until beatsPerCommand) {
    sendBeat(faultTag, firstSpad, beat)
  }
  waitFor(c.io.writeAccepted, 3 * nSources,
    "faulting store did not occupy every writer source")
  waitFor(c.io.physicalABeats,
    3 * nSources * beatsPerTransaction,
    "faulting store did not expose every irrevocable Put")
  sendBeat(faultYoungerTag, secondSpad, beat = 0)

  val faultSources = (2 * nSources until 3 * nSources).map(i =>
    peek(c.io.acceptedSources(i)))
  val deniedSource = faultSources.last
  val deniedVaddr = faultHost + (nSources - 1) * p.dmaMaxBytes
  var faultAcknowledged = 2 * nSources + 1
  release(deniedSource, expectedCount = faultAcknowledged, denied = true)
  waitFor(c.io.dma.halted, 1,
    "denied writer response did not halt DMA admission")

  for (beat <- 1 until beatsPerCommand) {
    sendBeat(faultYoungerTag, secondSpad, beat)
  }
  assert(peek(c.io.writeAccepted) == 3 * nSources,
    "writer issued a younger Put after its first fault")

  for (source <- faultSources.dropRight(1).reverse) {
    faultAcknowledged += 1
    release(source, expectedCount = faultAcknowledged)
  }
  expectHeldFaultCompletion(faultTag, deniedVaddr, holdCycles = 3)
  consumeCompletion()
  expectHeldFaultCompletion(faultYoungerTag, deniedVaddr, holdCycles = 2)
  consumeCompletion()

  waitFor(c.io.busy, 0,
    "writer remained busy after multi-command fault drain")
  assert(peek(c.io.dma.halted) == 1)
  assert(peek(c.io.writeAcknowledged) == 3 * nSources)
  poke(c.io.dma.clearFault, 1)
  waitFor(c.io.dma.clearFaultDone, 1,
    "writer fault clear did not complete")
  step(1)
  poke(c.io.dma.clearFault, 0)
  waitFor(c.io.dma.halted, 0,
    "writer remained halted after fault clear")
}

class VpuTLRollingWriterSpec extends ChiselFlatSpec {
  behavior of "VPU pipelined TL writer"

  it should "pipeline stores, reorder D, and drain multi-command faults" in {
    implicit val params: Parameters = (new DefaultConfig).toInstance
    val vp = VpuParams(
      vLen = 128,
      nLanes = 16,
      sfuLanes = 4,
      vSpadKB = 4,
      dmaMaxInFlight = 8)
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-tl-rolling-writer"),
      () => LazyModule(new VpuTLRollingWriterHarness(vp)).module) {
      c => new VpuTLRollingWriterTester(c, vp)
    } should be (true)
  }
}
