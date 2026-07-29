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

/** A deliberately non-FIFO cache-line manager.
  *
  * It accepts every size=6 Get before releasing any response. Transactions are
  * then returned in reverse A-channel order; each response consists of four
  * contiguous 128-bit D beats. This forces the reader to use all source IDs and
  * to reorder complete cache lines before exposing descriptor-ordered VPU
  * stream beats.
  */
private class VpuHoldingTLManager(
    beatBytes: Int,
    requestCount: Int)(implicit p: Parameters) extends LazyModule {
  require(beatBytes == 16,
    "the cache-line regression is defined for the configured 128-bit DMA bus")
  require(requestCount > 1)
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
    private val countBits = math.max(1, log2Ceil(requestCount + 1))
    private val sourceBits = math.max(1, edge.bundle.sourceBits)
    private val endSourceId = edge.client.endSourceId
    private val beatIndexBits = math.max(1, log2Ceil(beatsPerLine))
    require(endSourceId >= requestCount,
      "the reader source range must reach the holding manager")

    val io = IO(new Bundle {
      val releaseReads = Input(Bool())
      val readAccepted = Output(UInt(countBits.W))
      val readAcknowledged = Output(UInt(countBits.W))
      val readAcceptedSources = Output(Vec(requestCount, UInt(16.W)))
      val readAcceptedAddresses = Output(Vec(requestCount, UInt(64.W)))
      val readAcceptedSizes = Output(Vec(requestCount, UInt(8.W)))
      val readAcceptedCycles = Output(Vec(requestCount, UInt(32.W)))
      val readResponseSources = Output(Vec(requestCount, UInt(16.W)))
      val readRequestsConsecutive = Output(Bool())
    })

    val pending = RegInit(VecInit(Seq.fill(endSourceId)(false.B)))
    val requestSize = Reg(Vec(endSourceId, UInt(edge.bundle.sizeBits.W)))
    val requestAddress = Reg(Vec(endSourceId,
      UInt(edge.bundle.addressBits.W)))
    val readAccepted = RegInit(0.U(countBits.W))
    val readAcknowledged = RegInit(0.U(countBits.W))
    val readAcceptedSources = RegInit(VecInit(
      Seq.fill(requestCount)(0.U(16.W))))
    val readAcceptedAddresses = RegInit(VecInit(
      Seq.fill(requestCount)(0.U(64.W))))
    val readAcceptedSizes = RegInit(VecInit(
      Seq.fill(requestCount)(0.U(8.W))))
    val readAcceptedCycles = RegInit(VecInit(
      Seq.fill(requestCount)(0.U(32.W))))
    val readResponseSources = RegInit(VecInit(
      Seq.fill(requestCount)(0.U(16.W))))
    val requestsConsecutive = RegInit(true.B)
    val cycle = RegInit(0.U(32.W))
    cycle := cycle + 1.U

    io.readAccepted := readAccepted
    io.readAcknowledged := readAcknowledged
    io.readAcceptedSources := readAcceptedSources
    io.readAcceptedAddresses := readAcceptedAddresses
    io.readAcceptedSizes := readAcceptedSizes
    io.readAcceptedCycles := readAcceptedCycles
    io.readResponseSources := readResponseSources
    io.readRequestsConsecutive := requestsConsecutive

    val aIsRead = tl.a.bits.opcode === TLMessages.Get
    val aSourceInRange = tl.a.bits.source < endSourceId.U
    val aSource = tl.a.bits.source(sourceBits - 1, 0)
    tl.a.ready := aIsRead && aSourceInRange &&
      readAccepted < requestCount.U && !pending(aSource)

    when(tl.a.valid) {
      assert(aIsRead, "holding manager only accepts reader Gets")
      assert(aSourceInRange, "holding manager received an invalid source")
    }
    when(tl.a.fire) {
      assert(edge.first(tl.a) && edge.last(tl.a),
        "a cache-line Get must occupy one A beat")
      assert(tl.a.bits.size === log2Ceil(lineBytes).U,
        "reader did not issue a size=6 cache-line Get")
      assert(tl.a.bits.address(log2Ceil(lineBytes) - 1, 0) === 0.U,
        "cache-line Get was not naturally aligned")
      when(readAccepted =/= 0.U) {
        when(cycle =/= readAcceptedCycles(readAccepted - 1.U) + 1.U) {
          requestsConsecutive := false.B
        }
      }
      pending(aSource) := true.B
      requestSize(aSource) := tl.a.bits.size
      requestAddress(aSource) := tl.a.bits.address
      readAcceptedSources(readAccepted) := tl.a.bits.source
      readAcceptedAddresses(readAccepted) := tl.a.bits.address
      readAcceptedSizes(readAccepted) := tl.a.bits.size
      readAcceptedCycles(readAccepted) := cycle
      readAccepted := readAccepted + 1.U
    }

    // Once a D transaction starts, hold its source/metadata and emit all four
    // beats contiguously. The release input only controls transaction launch.
    val dActive = RegInit(false.B)
    val dSource = RegInit(0.U(sourceBits.W))
    val dSize = RegInit(0.U(edge.bundle.sizeBits.W))
    val dAddress = RegInit(0.U(edge.bundle.addressBits.W))
    val dBeat = RegInit(0.U(beatIndexBits.W))
    val reverseIndex = (requestCount - 1).U - readAcknowledged
    val nextSource =
      readAcceptedSources(reverseIndex)(sourceBits - 1, 0)
    val canLaunch = io.releaseReads &&
      readAccepted === requestCount.U &&
      readAcknowledged < requestCount.U && pending(nextSource)

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
        "holding manager responded to a source which was not pending")
      assert(edge.first(tl.d) === (dBeat === 0.U))
      assert(edge.last(tl.d) === (dBeat === (beatsPerLine - 1).U))
      when(dBeat === (beatsPerLine - 1).U) {
        pending(dSource) := false.B
        dActive := false.B
        readResponseSources(readAcknowledged) := dSource
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

private class VpuTLMultiSourceHarness(val vpuParams: VpuParams)
    (implicit p: Parameters) extends LazyModule {
  val visibilityNode = TLEphemeralNode()
  private val dutParameters = p.alterMap(Map(
    TileKey -> RocketTileParams(),
    TileVisibilityNodeKey -> visibilityNode))
  val dut = LazyModule(new VpuTLMemory(vpuParams)(dutParameters))
  val manager = LazyModule(new VpuHoldingTLManager(
    beatBytes = vpuParams.dmaBusWidth / 8,
    requestCount = vpuParams.dmaMaxInFlight))
  manager.node := visibilityNode := dut.idNode

  lazy val module = new VpuTLMultiSourceHarnessImp(this)
}

private class VpuTLMultiSourceHarnessImp(outer: VpuTLMultiSourceHarness)
    extends LazyModuleImp(outer) {
  private val n = outer.vpuParams.dmaMaxInFlight
  private val countBits = math.max(1, log2Ceil(n + 1))
  val io = IO(new Bundle {
    val dma = Flipped(new VpuDmaIO(outer.vpuParams))
    val releaseReads = Input(Bool())
    val readAccepted = Output(UInt(countBits.W))
    val readAcknowledged = Output(UInt(countBits.W))
    val readAcceptedSources = Output(Vec(n, UInt(16.W)))
    val readAcceptedAddresses = Output(Vec(n, UInt(64.W)))
    val readAcceptedSizes = Output(Vec(n, UInt(8.W)))
    val readAcceptedCycles = Output(Vec(n, UInt(32.W)))
    val readResponseSources = Output(Vec(n, UInt(16.W)))
    val readRequestsConsecutive = Output(Bool())
    val busy = Output(Bool())
    val fault = Output(Bool())
  })

  outer.dut.module.io.dma <> io.dma
  io.busy := outer.dut.module.io.busy
  io.fault := outer.dut.module.io.fault
  outer.manager.module.io.releaseReads := io.releaseReads
  io.readAccepted := outer.manager.module.io.readAccepted
  io.readAcknowledged := outer.manager.module.io.readAcknowledged
  io.readAcceptedSources := outer.manager.module.io.readAcceptedSources
  io.readAcceptedAddresses := outer.manager.module.io.readAcceptedAddresses
  io.readAcceptedSizes := outer.manager.module.io.readAcceptedSizes
  io.readAcceptedCycles := outer.manager.module.io.readAcceptedCycles
  io.readResponseSources := outer.manager.module.io.readResponseSources
  io.readRequestsConsecutive :=
    outer.manager.module.io.readRequestsConsecutive

  // Bare M-mode translations make the test deterministic without a PTW model.
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

private class VpuTLMultiSourceTester(
    c: VpuTLMultiSourceHarnessImp,
    p: VpuParams) extends PeekPokeTester(c) {
  private val n = p.dmaMaxInFlight
  require(n == 8, "this regression is defined for eight outstanding lines")

  poke(c.io.dma.readDescriptor.valid, 0)
  poke(c.io.dma.readDescriptor.bits.commandTag, 0)
  poke(c.io.dma.readDescriptor.bits.rowCount, 0)
  poke(c.io.dma.readDescriptor.bits.hostStrideBytes, 0)
  poke(c.io.dma.readData.ready, 0)
  poke(c.io.dma.writeDescriptor.valid, 0)
  poke(c.io.dma.writeDescriptor.bits.commandTag, 0)
  poke(c.io.dma.writeDescriptor.bits.rowCount, 0)
  poke(c.io.dma.writeDescriptor.bits.hostStrideBytes, 0)
  poke(c.io.dma.writeData.valid, 0)
  poke(c.io.dma.writeCompletion.ready, 0)
  poke(c.io.dma.clearFault, 0)
  poke(c.io.releaseReads, 0)
  step(5)

  private def waitFor(signal: Bits, value: BigInt, clue: String): Unit = {
    var timeout = 2000
    while (peek(signal) != value && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, clue)
  }

  private def packedAddressBytes(address: BigInt): BigInt =
    (0 until 16).foldLeft(BigInt(0)) { (acc, i) =>
      acc | (((address + i) & 0xff) << (8 * i))
    }

  val baseAddress = BigInt(0x4000)
  poke(c.io.dma.readDescriptor.bits.vaddr, baseAddress)
  poke(c.io.dma.readDescriptor.bits.spadElement, 0)
  poke(c.io.dma.readDescriptor.bits.elementCount, 128)
  poke(c.io.dma.readDescriptor.bits.status.prv, 3)
  poke(c.io.dma.readDescriptor.bits.status.dprv, 3)
  poke(c.io.dma.readDescriptor.bits.status.dv, 0)
  poke(c.io.dma.readDescriptor.valid, 1)
  waitFor(c.io.dma.readDescriptor.ready, 1, "read descriptor timed out")
  step(1)
  poke(c.io.dma.readDescriptor.valid, 0)

  // No D response is released until all source IDs have been occupied. This
  // distinguishes a genuinely pipelined issuer from a single candidate FSM
  // which merely keeps several completed requests outstanding.
  waitFor(c.io.readAccepted, n,
    "reader failed to launch eight outstanding cache-line Gets")
  assert(peek(c.io.readAcknowledged) == 0)
  assert(peek(c.io.dma.readData.valid) == 0)
  assert(peek(c.io.readRequestsConsecutive) == 1,
    "cache-line Gets were not accepted on consecutive A-channel cycles")

  val acceptedSources = (0 until n).map(i =>
    peek(c.io.readAcceptedSources(i)))
  assert(acceptedSources.distinct.size == n,
    s"reader reused a source before D: ${acceptedSources.mkString(",")}")
  for (i <- 0 until n) {
    assert(peek(c.io.readAcceptedSizes(i)) == 6)
    assert(peek(c.io.readAcceptedAddresses(i)) == baseAddress + 64 * i,
      s"cache-line Get $i used the wrong address")
    if (i > 0) {
      assert(peek(c.io.readAcceptedCycles(i)) ==
        peek(c.io.readAcceptedCycles(i - 1)) + 1)
    }
  }

  poke(c.io.releaseReads, 1)
  waitFor(c.io.readAcknowledged, n,
    "four-beat D responses did not drain in reverse transaction order")
  poke(c.io.releaseReads, 0)
  val responseSources = (0 until n).map(i =>
    peek(c.io.readResponseSources(i)))
  assert(responseSources == acceptedSources.reverse,
    s"D order ${responseSources.mkString(",")} was not reverse A order " +
      acceptedSources.mkString(","))

  // Gemmini-style source tracking permits complete lines to write VSRAM in
  // response order.  Absolute SPAD addresses and the command tag, rather than
  // FIFO response order, restore identity.  Check that every destination beat
  // appears exactly once and carries the bytes belonging to that address.
  val seenSpadBeats = Array.fill(32)(false)
  for (beat <- 0 until 32) {
    waitFor(c.io.dma.readData.valid, 1,
      s"tagged VPU output beat $beat timed out")
    assert(peek(c.io.dma.readData.bits.error) == 0)
    assert(peek(c.io.dma.readData.bits.commandTag) == 0)
    val spadElement = peek(c.io.dma.readData.bits.spadElement).toInt
    assert(spadElement >= 0 && spadElement < 128 && spadElement % 4 == 0,
      s"invalid VPU output SPAD element $spadElement")
    val spadBeat = spadElement / 4
    assert(!seenSpadBeats(spadBeat),
      s"VPU output SPAD beat $spadBeat was written twice")
    seenSpadBeats(spadBeat) = true
    assert(peek(c.io.dma.readData.bits.data) ==
      packedAddressBytes(baseAddress + 4 * spadElement),
      s"VPU output beat $beat carried data for the wrong SPAD address")
    assert(peek(c.io.dma.readData.bits.elementMask) == 0xf)
    assert(peek(c.io.dma.readData.bits.last) ==
      (if (beat == 31) 1 else 0))
    poke(c.io.dma.readData.ready, 1)
    step(1)
    poke(c.io.dma.readData.ready, 0)
  }
  assert(seenSpadBeats.forall(identity),
    "VPU tagged reader omitted at least one SPAD beat")

  waitFor(c.io.busy, 0, "reader remained busy after its final output beat")
  assert(peek(c.io.fault) == 0)
}

class VpuTLMultiSourceSpec extends ChiselFlatSpec {
  behavior of "VPU pipelined multi-source cache-line reader"

  it should "issue eight consecutive Gets and restore tagged reverse D responses" in {
    implicit val params: Parameters = (new DefaultConfig).toInstance
    val vp = VpuParams(
      vLen = 128,
      nLanes = 16,
      sfuLanes = 4,
      vSpadKB = 4,
      dmaMaxInFlight = 8)
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-tl-multi-source"),
      () => LazyModule(new VpuTLMultiSourceHarness(vp)).module) {
      c => new VpuTLMultiSourceTester(c, vp)
    } should be (true)
  }
}
