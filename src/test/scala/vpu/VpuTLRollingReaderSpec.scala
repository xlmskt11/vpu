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

/** Test-only manager which permits a source to be reused after its complete
  * D transaction, while retaining a log of both generations of requests.
  *
  * `responseLimit` controls how many transactions may return.  The regression
  * first returns exactly one full source window while the VPU-facing read-data
  * stream is stalled.  A correctly separated reader can then reuse those
  * source IDs to launch the second descriptor before any SRAM writeback fires.
  */
private class VpuRollingTLManager(
    beatBytes: Int,
    totalRequests: Int)(implicit p: Parameters) extends LazyModule {
  require(beatBytes == 16,
    "the rolling reader regression assumes the 128-bit VPU DMA bus")
  require(totalRequests > 1)
  private val lineBytes = 64
  private val beatsPerLine = lineBytes / beatBytes

  val node = TLManagerNode(Seq(TLSlavePortParameters.v1(
    managers = Seq(TLSlaveParameters.v1(
      address = Seq(AddressSet(0x0, 0xffff)),
      regionType = RegionType.UNCACHED,
      supportsGet = TransferSizes(1, lineBytes),
      supportsPutFull = TransferSizes(1, lineBytes),
      supportsPutPartial = TransferSizes(1, lineBytes),
      fifoId = None)),
    beatBytes = beatBytes,
    minLatency = 1)))

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (tl, edge) = node.in.head
    private val countBits = math.max(1, log2Ceil(totalRequests + 1))
    private val sourceBits = math.max(1, edge.bundle.sourceBits)
    private val endSourceId = edge.client.endSourceId
    private val beatIndexBits = math.max(1, log2Ceil(beatsPerLine))

    val io = IO(new Bundle {
      val responseLimit = Input(UInt(countBits.W))
      val readAccepted = Output(UInt(countBits.W))
      val readAcknowledged = Output(UInt(countBits.W))
      val readAcceptedSources = Output(Vec(totalRequests, UInt(16.W)))
      val readAcceptedAddresses = Output(Vec(totalRequests, UInt(64.W)))
      val readAcceptedSizes = Output(Vec(totalRequests, UInt(8.W)))
      val readAcceptedCycles = Output(Vec(totalRequests, UInt(32.W)))
      val readResponseLastCycles = Output(Vec(totalRequests, UInt(32.W)))
    })

    val pending = RegInit(VecInit(Seq.fill(endSourceId)(false.B)))
    val requestSize = Reg(Vec(endSourceId, UInt(edge.bundle.sizeBits.W)))
    val requestAddress = Reg(Vec(endSourceId,
      UInt(edge.bundle.addressBits.W)))
    val readAccepted = RegInit(0.U(countBits.W))
    val readAcknowledged = RegInit(0.U(countBits.W))
    val acceptedSources = RegInit(VecInit(
      Seq.fill(totalRequests)(0.U(16.W))))
    val acceptedAddresses = RegInit(VecInit(
      Seq.fill(totalRequests)(0.U(64.W))))
    val acceptedSizes = RegInit(VecInit(
      Seq.fill(totalRequests)(0.U(8.W))))
    val acceptedCycles = RegInit(VecInit(
      Seq.fill(totalRequests)(0.U(32.W))))
    val responseLastCycles = RegInit(VecInit(
      Seq.fill(totalRequests)(0.U(32.W))))
    val cycle = RegInit(0.U(32.W))
    cycle := cycle + 1.U

    io.readAccepted := readAccepted
    io.readAcknowledged := readAcknowledged
    io.readAcceptedSources := acceptedSources
    io.readAcceptedAddresses := acceptedAddresses
    io.readAcceptedSizes := acceptedSizes
    io.readAcceptedCycles := acceptedCycles
    io.readResponseLastCycles := responseLastCycles

    val aIsRead = tl.a.bits.opcode === TLMessages.Get
    val aSourceInRange = tl.a.bits.source < endSourceId.U
    val aSource = tl.a.bits.source(sourceBits - 1, 0)
    tl.a.ready := aIsRead && aSourceInRange &&
      readAccepted < totalRequests.U && !pending(aSource)

    when(tl.a.valid) {
      assert(aIsRead, "rolling manager only accepts reader Gets")
      assert(aSourceInRange, "rolling manager received an invalid source")
    }
    when(tl.a.valid && aSourceInRange) {
      // `ready` is deliberately low for an occupied source.  Checking only
      // on fire would therefore turn an illegal early-reuse attempt into a
      // timeout instead of identifying the TileLink lifetime violation.
      assert(!pending(aSource),
        "VPU reader presented a reused TileLink source before D last")
    }
    when(tl.a.fire) {
      assert(edge.first(tl.a) && edge.last(tl.a),
        "a cache-line Get must occupy one A beat")
      assert(tl.a.bits.size === log2Ceil(lineBytes).U,
        "rolling reader did not issue a size=6 cache-line Get")
      assert(tl.a.bits.address(log2Ceil(lineBytes) - 1, 0) === 0.U,
        "rolling reader cache-line Get was not naturally aligned")
      // `pending` is cleared only on D last below.  This assertion is the
      // protocol-level source-reuse invariant independently of tester timing.
      assert(!pending(aSource),
        "VPU reader reused a TileLink source before D last")
      pending(aSource) := true.B
      requestSize(aSource) := tl.a.bits.size
      requestAddress(aSource) := tl.a.bits.address
      acceptedSources(readAccepted) := tl.a.bits.source
      acceptedAddresses(readAccepted) := tl.a.bits.address
      acceptedSizes(readAccepted) := tl.a.bits.size
      acceptedCycles(readAccepted) := cycle
      readAccepted := readAccepted + 1.U
    }

    // Return transactions in their A-channel acceptance order.  A source may
    // have been reused by the time a later log entry is selected, but it can
    // only be reused after the earlier transaction's D last, and its current
    // per-source metadata therefore belongs to exactly this next log entry.
    val dActive = RegInit(false.B)
    val dSource = RegInit(0.U(sourceBits.W))
    val dSize = RegInit(0.U(edge.bundle.sizeBits.W))
    val dAddress = RegInit(0.U(edge.bundle.addressBits.W))
    val dBeat = RegInit(0.U(beatIndexBits.W))
    val nextSource =
      acceptedSources(readAcknowledged)(sourceBits - 1, 0)
    val canLaunch = readAcknowledged < io.responseLimit &&
      readAcknowledged < readAccepted && pending(nextSource)

    when(!dActive && canLaunch) {
      dActive := true.B
      dSource := nextSource
      dSize := requestSize(nextSource)
      dAddress := requestAddress(nextSource)
      dBeat := 0.U
    }

    val patternedBytes = Wire(Vec(beatBytes, UInt(8.W)))
    for (i <- 0 until beatBytes) {
      val byteAddress = dAddress + dBeat * beatBytes.U + i.U
      patternedBytes(i) := byteAddress(7, 0)
    }

    tl.d.valid := dActive
    tl.d.bits := edge.AccessAck(dSource, dSize)
    tl.d.bits.opcode := TLMessages.AccessAckData
    tl.d.bits.data := Cat(patternedBytes.reverse)

    when(tl.d.fire) {
      assert(pending(dSource),
        "rolling manager responded to a source which was not pending")
      assert(edge.first(tl.d) === (dBeat === 0.U))
      assert(edge.last(tl.d) ===
        (dBeat === (beatsPerLine - 1).U))
      when(dBeat === (beatsPerLine - 1).U) {
        // This is the only point at which the source becomes reusable.
        pending(dSource) := false.B
        dActive := false.B
        responseLastCycles(readAcknowledged) := cycle
        readAcknowledged := readAcknowledged + 1.U
      }.otherwise {
        dBeat := dBeat + 1.U
      }
    }

    tl.b.valid := false.B
    tl.b.bits := DontCare
    tl.c.ready := true.B
    tl.e.ready := true.B
  }
}

private class VpuTLRollingHarness(val vpuParams: VpuParams)
    (implicit p: Parameters) extends LazyModule {
  val visibilityNode = TLEphemeralNode()
  private val dutParameters = p.alterMap(Map(
    TileKey -> RocketTileParams(),
    TileVisibilityNodeKey -> visibilityNode))
  val dut = LazyModule(new VpuTLMemory(vpuParams)(dutParameters))
  val totalRequests = 2 * vpuParams.dmaMaxInFlight
  val manager = LazyModule(new VpuRollingTLManager(
    beatBytes = vpuParams.dmaBusWidth / 8,
    totalRequests = totalRequests)(dutParameters))
  manager.node := visibilityNode := dut.idNode

  lazy val module = new VpuTLRollingHarnessImp(this)
}

private class VpuTLRollingHarnessImp(outer: VpuTLRollingHarness)
    extends LazyModuleImp(outer) {
  private val total = outer.totalRequests
  private val countBits = math.max(1, log2Ceil(total + 1))
  val io = IO(new Bundle {
    val dma = Flipped(new VpuDmaIO(outer.vpuParams))
    val responseLimit = Input(UInt(countBits.W))
    val readAccepted = Output(UInt(countBits.W))
    val readAcknowledged = Output(UInt(countBits.W))
    val readAcceptedSources = Output(Vec(total, UInt(16.W)))
    val readAcceptedAddresses = Output(Vec(total, UInt(64.W)))
    val readAcceptedSizes = Output(Vec(total, UInt(8.W)))
    val readAcceptedCycles = Output(Vec(total, UInt(32.W)))
    val readResponseLastCycles = Output(Vec(total, UInt(32.W)))
    val busy = Output(Bool())
    val fault = Output(Bool())
  })

  outer.dut.module.io.dma <> io.dma
  outer.manager.module.io.responseLimit := io.responseLimit
  io.readAccepted := outer.manager.module.io.readAccepted
  io.readAcknowledged := outer.manager.module.io.readAcknowledged
  io.readAcceptedSources := outer.manager.module.io.readAcceptedSources
  io.readAcceptedAddresses := outer.manager.module.io.readAcceptedAddresses
  io.readAcceptedSizes := outer.manager.module.io.readAcceptedSizes
  io.readAcceptedCycles := outer.manager.module.io.readAcceptedCycles
  io.readResponseLastCycles :=
    outer.manager.module.io.readResponseLastCycles
  io.busy := outer.dut.module.io.busy
  io.fault := outer.dut.module.io.fault

  // Bare M-mode translations keep the regression deterministic and avoid a
  // page-table model while exercising the real FrontendTLB path.
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

private class VpuTLRollingTester(
    c: VpuTLRollingHarnessImp,
    p: VpuParams) extends PeekPokeTester(c) {
  private val n = p.dmaMaxInFlight
  private val totalRequests = 2 * n
  private val beatsPerDescriptor = p.vLen / p.dmaElementsPerBeat
  require(n == 8,
    "the rolling regression is defined for eight TL source IDs")
  require(beatsPerDescriptor == 32,
    "the rolling regression expects 128 FP32 elements per descriptor")

  poke(c.io.dma.readDescriptor.valid, 0)
  poke(c.io.dma.readDescriptor.bits.rowCount, 0)
  poke(c.io.dma.readDescriptor.bits.hostStrideBytes, 0)
  poke(c.io.dma.readData.ready, 0)
  poke(c.io.dma.writeDescriptor.valid, 0)
  poke(c.io.dma.writeDescriptor.bits.rowCount, 0)
  poke(c.io.dma.writeDescriptor.bits.hostStrideBytes, 0)
  poke(c.io.dma.writeDescriptor.bits.commandTag, 0)
  poke(c.io.dma.writeData.valid, 0)
  poke(c.io.dma.writeCompletion.ready, 0)
  poke(c.io.dma.clearFault, 0)
  poke(c.io.responseLimit, 0)
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
    poke(c.io.dma.readDescriptor.bits.vaddr, address)
    poke(c.io.dma.readDescriptor.bits.spadElement, spadElement)
    poke(c.io.dma.readDescriptor.bits.elementCount, p.vLen)
    poke(c.io.dma.readDescriptor.bits.commandTag, commandTag)
    poke(c.io.dma.readDescriptor.bits.status.prv, 3)
    poke(c.io.dma.readDescriptor.bits.status.dprv, 3)
    poke(c.io.dma.readDescriptor.bits.status.dv, 0)
    poke(c.io.dma.readDescriptor.valid, 1)
    var timeout = 1000
    while (peek(c.io.dma.readDescriptor.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"read descriptor tag $commandTag was not admitted")
    step(1)
    poke(c.io.dma.readDescriptor.valid, 0)
  }

  private def packedAddressBytes(address: BigInt): BigInt =
    (0 until 16).foldLeft(BigInt(0)) { (acc, i) =>
      acc | (((address + i) & 0xff) << (8 * i))
    }

  val firstHost = BigInt(0x4000)
  val secondHost = firstHost + p.vLen * p.storageBytes
  val firstSpad = 0
  val secondSpad = p.vLen
  val firstTag = 0
  val secondTag = 1

  // Both descriptors enter the reader before either one is allowed to return
  // data to the SRAM-facing stream.
  issueDescriptor(firstHost, firstSpad, firstTag)
  issueDescriptor(secondHost, secondSpad, secondTag)

  waitFor(c.io.readAccepted, n,
    "first descriptor did not occupy all eight TL sources")
  assert(peek(c.io.readAcknowledged) == 0)
  assert(peek(c.io.dma.readData.ready) == 0)

  val firstSources = (0 until n).map(i =>
    peek(c.io.readAcceptedSources(i)))
  assert(firstSources.distinct.size == n,
    s"first request window reused a live source: ${firstSources.mkString(",")}")
  val firstAddresses = (0 until n).map(i =>
    peek(c.io.readAcceptedAddresses(i)))
  val expectedFirstAddresses = (0 until n).map(i => firstHost + 64 * i)
  assert(firstAddresses.sorted == expectedFirstAddresses.sorted,
    s"first descriptor used the wrong cache lines: ${firstAddresses.mkString(",")}")
  for (i <- 0 until n) {
    assert(peek(c.io.readAcceptedSizes(i)) == 6)
  }

  // Complete only the first source window. readData.ready remains low, so no
  // VPU/SRAM writeback can fire. The second descriptor must nevertheless use
  // the newly returned source IDs and launch its eight cache-line Gets.
  poke(c.io.responseLimit, n)
  waitFor(c.io.readAcknowledged, n,
    "first source window did not return all D transactions")
  waitFor(c.io.readAccepted, totalRequests,
    "second descriptor TL Gets waited for first-descriptor SRAM writeback")
  assert(peek(c.io.readAcknowledged) == n,
    "manager returned second-window data before it was released")
  assert(peek(c.io.dma.readData.ready) == 0,
    "SRAM-facing stream was unexpectedly enabled during rolling issue")
  assert(peek(c.io.busy) == 1,
    "reader became idle with two descriptors still buffered")

  val secondSources = (n until totalRequests).map(i =>
    peek(c.io.readAcceptedSources(i)))
  assert(secondSources.distinct.size == n,
    s"second request window reused a still-live source: ${secondSources.mkString(",")}")
  val secondAddresses = (n until totalRequests).map(i =>
    peek(c.io.readAcceptedAddresses(i)))
  val expectedSecondAddresses = (0 until n).map(i => secondHost + 64 * i)
  assert(secondAddresses.sorted == expectedSecondAddresses.sorted,
    s"second descriptor used the wrong cache lines: ${secondAddresses.mkString(",")}")
  for (i <- 0 until n) {
    val requestIndex = n + i
    assert(peek(c.io.readAcceptedSizes(requestIndex)) == 6)

    val source = peek(c.io.readAcceptedSources(requestIndex))
    val previousIndex = firstSources.indexWhere(_ == source)
    assert(previousIndex >= 0,
      s"second descriptor used unexpected source $source")
    val dLastCycle = peek(c.io.readResponseLastCycles(previousIndex))
    val reusedCycle = peek(c.io.readAcceptedCycles(requestIndex))
    assert(dLastCycle > 0 && reusedCycle > dLastCycle,
      s"source $source was reused at $reusedCycle before D last $dLastCycle")
  }

  // A completed line has already crossed the TL side but cannot yet cross
  // the SRAM-facing Decoupled interface.  It must remain valid and bit-stable
  // while the consumer is stalled; otherwise a newly completed line could
  // overwrite the pending writeback even though request issue is decoupled.
  waitFor(c.io.dma.readData.valid, 1,
    "reader did not expose a completed line while SRAM writeback was stalled")
  val heldData = peek(c.io.dma.readData.bits.data)
  val heldMask = peek(c.io.dma.readData.bits.elementMask)
  val heldSpadElement = peek(c.io.dma.readData.bits.spadElement)
  val heldTag = peek(c.io.dma.readData.bits.commandTag)
  val heldLast = peek(c.io.dma.readData.bits.last)
  val heldError = peek(c.io.dma.readData.bits.error)
  for (_ <- 0 until 4) {
    assert(peek(c.io.dma.readData.valid) == 1,
      "reader dropped a stalled SRAM writeback beat")
    assert(peek(c.io.dma.readData.bits.data) == heldData)
    assert(peek(c.io.dma.readData.bits.elementMask) == heldMask)
    assert(peek(c.io.dma.readData.bits.spadElement) == heldSpadElement)
    assert(peek(c.io.dma.readData.bits.commandTag) == heldTag)
    assert(peek(c.io.dma.readData.bits.last) == heldLast)
    assert(peek(c.io.dma.readData.bits.error) == heldError)
    step(1)
  }

  // Release the second response window and the SRAM-facing consumer together.
  // Returned lines may be unpacked in any order; commandTag and spadElement
  // must restore both descriptor identities without loss or duplication.
  poke(c.io.responseLimit, totalRequests)
  poke(c.io.dma.readData.ready, 1)
  val seen = Array.fill(2, beatsPerDescriptor)(false)
  val tagBeatCount = Array.fill(2)(0)
  val tagLastCount = Array.fill(2)(0)

  for (beat <- 0 until 2 * beatsPerDescriptor) {
    waitFor(c.io.dma.readData.valid, 1,
      s"rolling VPU output beat $beat timed out")
    assert(peek(c.io.dma.readData.bits.error) == 0)
    assert(peek(c.io.dma.readData.bits.elementMask) == 0xf)

    val tag = peek(c.io.dma.readData.bits.commandTag).toInt
    assert(tag == firstTag || tag == secondTag,
      s"rolling reader returned unknown command tag $tag")
    val descriptorIndex = if (tag == firstTag) 0 else 1
    val spadBase = if (descriptorIndex == 0) firstSpad else secondSpad
    val hostBase = if (descriptorIndex == 0) firstHost else secondHost
    val spadElement = peek(c.io.dma.readData.bits.spadElement).toInt
    val relativeElement = spadElement - spadBase
    assert(relativeElement >= 0 && relativeElement < p.vLen &&
      relativeElement % p.dmaElementsPerBeat == 0,
      s"tag $tag returned invalid SPAD element $spadElement")
    val descriptorBeat = relativeElement / p.dmaElementsPerBeat
    assert(!seen(descriptorIndex)(descriptorBeat),
      s"tag $tag SPAD beat $descriptorBeat was returned twice")
    seen(descriptorIndex)(descriptorBeat) = true
    tagBeatCount(descriptorIndex) += 1

    assert(peek(c.io.dma.readData.bits.data) ==
      packedAddressBytes(hostBase + relativeElement * p.storageBytes),
      s"tag $tag SPAD beat $descriptorBeat carried the wrong bytes")
    val expectedLast = tagBeatCount(descriptorIndex) == beatsPerDescriptor
    assert(peek(c.io.dma.readData.bits.last) ==
      (if (expectedLast) 1 else 0),
      s"tag $tag produced an incorrect command-last marker")
    if (expectedLast) { tagLastCount(descriptorIndex) += 1 }
    step(1)
  }

  assert(seen.forall(_.forall(identity)),
    "rolling reader omitted at least one descriptor SPAD beat")
  assert(tagLastCount.sameElements(Array(1, 1)),
    s"command-last counts were ${tagLastCount.mkString(",")}")
  waitFor(c.io.readAcknowledged, totalRequests,
    "second source window did not drain")
  waitFor(c.io.busy, 0,
    "rolling reader remained busy after both descriptors completed")
  assert(peek(c.io.fault) == 0)
}

class VpuTLRollingReaderSpec extends ChiselFlatSpec {
  behavior of "VPU TL request/SRAM writeback decoupling"

  it should "reuse sources after D last and issue descriptor two before descriptor-one SRAM drain" in {
    implicit val params: Parameters = (new DefaultConfig).toInstance
    val vp = VpuParams(
      vLen = 128,
      nLanes = 16,
      sfuLanes = 4,
      vSpadKB = 4,
      dmaMaxInFlight = 8)
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-tl-rolling-reader"),
      () => LazyModule(new VpuTLRollingHarness(vp)).module) {
      c => new VpuTLRollingTester(c, vp)
    } should be (true)
  }
}
