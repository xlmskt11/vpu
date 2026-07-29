package vpu

import chisel3._
import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}
import chisel3.util.Decoupled

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.diplomacy.{AddressSet, IdRange, LazyModule,
  LazyModuleImp}
import freechips.rocketchip.rocket.MStatus
import freechips.rocketchip.system.DefaultConfig
import freechips.rocketchip.tile.{RocketTileParams, TileKey,
  TileVisibilityNodeKey}
import freechips.rocketchip.tilelink.{TLClientNode, TLClientParameters,
  TLMasterPortParameters, TLRAM, TLEphemeralNode}

import gemmini.FrontendTLBIO

/** Supplies HasCoreParameters with a negotiated address width while leaving
  * the scheduler's FrontendTLBIO visible for direct response stimulation.
  */
private class VpuTLBTranslationSchedulerHarness(
    val tagBits: Int,
    val maxSize: Int,
    val resultQueueEntries: Int)(implicit p: Parameters) extends LazyModule {
  val visibilityNode = TLEphemeralNode()
  val dummyClient = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(
    TLClientParameters(
      name = "vpu-tlb-scheduler-test",
      sourceId = IdRange(0, 1))))))
  val dummyRam = LazyModule(new TLRAM(
    AddressSet(0x0, 0xffff), beatBytes = 16))
  dummyRam.node := visibilityNode := dummyClient

  val dutParameters = p.alterMap(Map(
    TileKey -> RocketTileParams(),
    TileVisibilityNodeKey -> visibilityNode))

  lazy val module = new VpuTLBTranslationSchedulerHarnessImp(this)
}

/** Adds an observation-only port from inside the scheduler module.  Chisel
  * correctly prevents a parent from reaching into an arbitrary child Wire,
  * so the test subclass exports the otherwise protocol-internal tag without
  * changing production RTL or its public interface.
  */
private class InspectableVpuTLBTranslationScheduler(
    tagBits: Int,
    maxSize: Int,
    resultQueueEntries: Int)(implicit p: Parameters)
    extends VpuTLBTranslationScheduler(
      tagBits, maxSize, resultQueueEntries) {
  val inspect = IO(new Bundle {
    val drivenTag = Output(UInt(tagBits.W))
  })
  inspect.drivenTag := driven.tag
}

private class VpuTLBTranslationSchedulerHarnessImp(
    outer: VpuTLBTranslationSchedulerHarness)
    extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
    val request = Flipped(Decoupled(new VpuTLBTaggedRequest(
      outer.tagBits, outer.maxSize)(outer.dutParameters)))
    val result = Decoupled(new VpuTLBTaggedResult(
      outer.tagBits)(outer.dutParameters))
    val tlb = new FrontendTLBIO()(outer.dutParameters)

    // Tag is intentionally not part of FrontendTLBIO.  Export the selected
    // internal tag only in this harness so replay can be checked end-to-end.
    val drivenTag = Output(UInt(outer.tagBits.W))
  })

  val dut = Module(new InspectableVpuTLBTranslationScheduler(
    outer.tagBits,
    outer.maxSize,
    outer.resultQueueEntries)(outer.dutParameters))

  dut.io.request <> io.request
  io.result.valid := dut.io.result.valid
  io.result.bits := dut.io.result.bits
  dut.io.result.ready := io.result.ready
  io.tlb.req := dut.io.tlb.req
  dut.io.tlb.resp := io.tlb.resp
  io.drivenTag := dut.inspect.drivenTag

  // The dummy diplomatic path exists only to negotiate paddrBits.
  val (tl, _) = outer.dummyClient.out.head
  tl.a.valid := false.B
  tl.a.bits := DontCare
  tl.b.ready := true.B
  tl.c.valid := false.B
  tl.c.bits := DontCare
  tl.d.ready := true.B
  tl.e.valid := false.B
  tl.e.bits := DontCare
}

private class VpuTLBTranslationSchedulerTester(
    c: VpuTLBTranslationSchedulerHarnessImp)
    extends PeekPokeTester(c) {
  case class Request(
      tag: Int,
      vaddr: BigInt,
      lgSize: Int,
      cmd: Int,
      statusSeed: Int)

  case class RequestSnapshot(
      tag: BigInt,
      vaddr: BigInt,
      tlbSize: BigInt,
      cmd: BigInt,
      status: Vector[(String, BigInt)])

  private def statusSnapshot(status: MStatus): Vector[(String, BigInt)] =
    status.elements.toVector.sortBy(_._1).map {
      case (name, bits: Bits) => name -> peek(bits)
      case (name, other) =>
        throw new IllegalArgumentException(
          s"unexpected aggregate MStatus field $name: ${other.getClass}")
    }

  private def selectedRequestSnapshot(): RequestSnapshot = RequestSnapshot(
    tag = peek(c.io.drivenTag),
    vaddr = peek(c.io.tlb.req.bits.tlb_req.vaddr),
    tlbSize = peek(c.io.tlb.req.bits.tlb_req.size),
    cmd = peek(c.io.tlb.req.bits.tlb_req.cmd),
    status = statusSnapshot(c.io.tlb.req.bits.status))

  private def leafBits(data: Data): Seq[Bits] = data match {
    case bits: Bits => Seq(bits)
    case aggregate: Aggregate => aggregate.getElements.flatMap(leafBits)
  }

  private def clear(data: Data): Unit =
    leafBits(data).foreach(bits => poke(bits, 0))

  private def setStatus(status: MStatus, seed: Int): Unit = {
    clear(status)
    poke(status.debug, seed & 1)
    poke(status.isa, BigInt("40000000", 16) + seed)
    poke(status.dprv, (seed % 3) + 1)
    poke(status.dv, (seed >> 1) & 1)
    poke(status.prv, (seed + 1) & 3)
    poke(status.v, (seed >> 2) & 1)
    poke(status.mxr, (seed >> 1) & 1)
    poke(status.sum, (seed >> 2) & 1)
    poke(status.mprv, (seed >> 3) & 1)
    poke(status.mpp, (seed + 2) & 3)
    poke(status.mie, (seed >> 1) & 1)
  }

  private def present(request: Option[Request]): Unit = request match {
    case Some(r) =>
      poke(c.io.request.valid, 1)
      clear(c.io.request.bits)
      poke(c.io.request.bits.tag, r.tag)
      poke(c.io.request.bits.vaddr, r.vaddr)
      poke(c.io.request.bits.lgSize, r.lgSize)
      poke(c.io.request.bits.cmd, r.cmd)
      setStatus(c.io.request.bits.status, r.statusSeed)
    case None =>
      poke(c.io.request.valid, 0)
      clear(c.io.request.bits)
  }

  private def respond(miss: Boolean, paddr: BigInt = 0): Unit = {
    clear(c.io.tlb.resp)
    poke(c.io.tlb.resp.miss, if (miss) 1 else 0)
    poke(c.io.tlb.resp.paddr, paddr)
  }

  private def expectSelected(request: Request): Unit = {
    assert(peek(c.io.tlb.req.valid) == 1,
      s"tag ${request.tag} was not presented to FrontendTLB")
    assert(peek(c.io.drivenTag) == request.tag)
    assert(peek(c.io.tlb.req.bits.tlb_req.vaddr) == request.vaddr)
    assert(peek(c.io.tlb.req.bits.tlb_req.size) == 0,
      "VPU translation must use Gemmini-compatible byte-sized TLB requests")
    assert(peek(c.io.tlb.req.bits.tlb_req.cmd) == request.cmd)
    assert(peek(c.io.tlb.req.bits.tlb_req.prv) ==
      peek(c.io.tlb.req.bits.status.dprv))
    assert(peek(c.io.tlb.req.bits.tlb_req.v) ==
      peek(c.io.tlb.req.bits.status.dv))
  }

  private def expectResult(tag: Int, paddr: BigInt): Unit = {
    assert(peek(c.io.result.valid) == 1,
      s"terminal result for tag $tag was missing")
    assert(peek(c.io.result.bits.tag) == tag)
    assert(peek(c.io.result.bits.paddr) == paddr)
    assert(peek(c.io.result.bits.pf.ld) == 0)
    assert(peek(c.io.result.bits.pf.st) == 0)
    assert(peek(c.io.result.bits.gf.ld) == 0)
    assert(peek(c.io.result.bits.gf.st) == 0)
    assert(peek(c.io.result.bits.ae.ld) == 0)
    assert(peek(c.io.result.bits.ae.st) == 0)
  }

  poke(c.io.request.valid, 0)
  clear(c.io.request.bits)
  poke(c.io.result.ready, 0)
  clear(c.io.tlb.resp)
  step(2)

  // ----------------------------------------------------------------------
  // 1. Three hits: every response retains the preceding request's tag while
  // the next request is launched in the same cycle (II=1).
  // ----------------------------------------------------------------------
  val hitA = Request(0, 0x1000, 6, 0, 3)
  val hitB = Request(1, 0x2000, 5, 1, 6)
  val hitC = Request(2, 0x3000, 4, 0, 9)
  val hitPA = BigInt(0xa100)
  val hitPB = BigInt(0xb200)
  val hitPC = BigInt(0xc300)

  poke(c.io.result.ready, 1)
  present(Some(hitA))
  respond(miss = false)
  assert(peek(c.io.request.ready) == 1)
  expectSelected(hitA)
  step(1)

  present(Some(hitB))
  respond(miss = false, hitPA)
  assert(peek(c.io.request.ready) == 1)
  expectSelected(hitB)
  expectResult(hitA.tag, hitPA)
  step(1)

  present(Some(hitC))
  respond(miss = false, hitPB)
  assert(peek(c.io.request.ready) == 1)
  expectSelected(hitC)
  expectResult(hitB.tag, hitPB)
  step(1)

  present(None)
  respond(miss = false, hitPC)
  assert(peek(c.io.tlb.req.valid) == 0)
  expectResult(hitC.tag, hitPC)
  step(1)
  respond(miss = false)
  assert(peek(c.io.result.valid) == 0)
  step(1)

  // ----------------------------------------------------------------------
  // 2. A is retried until a terminal response. B remains valid upstream but
  // cannot be accepted, and every externally driven retry (including tag in
  // the test-only sideband) is bit-identical to A.
  // ----------------------------------------------------------------------
  val missA = Request(3, 0x4100, 6, 0, 13)
  val afterMissB = Request(4, 0x5200, 5, 1, 18)
  val missPA = BigInt(0xd100)
  val missPB = BigInt(0xe200)

  present(Some(missA))
  respond(miss = false)
  assert(peek(c.io.request.ready) == 1)
  expectSelected(missA)
  step(1)

  present(Some(afterMissB))
  var replayReference: Option[RequestSnapshot] = None
  for (_ <- 0 until 4) {
    respond(miss = true)
    assert(peek(c.io.request.ready) == 0,
      "upstream request advanced while the previous request was a TLB miss")
    assert(peek(c.io.result.valid) == 0)
    expectSelected(missA)
    val snapshot = selectedRequestSnapshot()
    replayReference match {
      case Some(reference) => assert(snapshot == reference,
        s"miss replay changed: reference=$reference current=$snapshot")
      case None => replayReference = Some(snapshot)
    }
    step(1)
  }

  // The terminal A response and launch of B share a cycle. B must never have
  // appeared on FrontendTLB before this point.
  respond(miss = false, missPA)
  assert(peek(c.io.request.ready) == 1)
  expectSelected(afterMissB)
  expectResult(missA.tag, missPA)
  step(1)

  present(None)
  respond(miss = false, missPB)
  expectResult(afterMissB.tag, missPB)
  assert(peek(c.io.tlb.req.valid) == 0)
  step(1)
  respond(miss = false)
  step(1)

  // ----------------------------------------------------------------------
  // 3. Two terminal results fill the queue while the consumer is stalled.
  // A third request receives no credit. Releasing one result permits that
  // request in the same cycle, and A/B/C subsequently leave exactly once.
  // ----------------------------------------------------------------------
  val blockedA = Request(5, 0x6100, 6, 0, 21)
  val blockedB = Request(6, 0x7200, 6, 1, 24)
  val blockedC = Request(7, 0x8300, 4, 0, 27)
  val blockedPA = BigInt(0x9100)
  val blockedPB = BigInt(0x9200)
  val blockedPC = BigInt(0x9300)

  poke(c.io.result.ready, 0)
  present(Some(blockedA))
  respond(miss = false)
  assert(peek(c.io.request.ready) == 1)
  expectSelected(blockedA)
  step(1)

  present(Some(blockedB))
  respond(miss = false, blockedPA)
  assert(peek(c.io.request.ready) == 1)
  expectSelected(blockedB)
  expectResult(blockedA.tag, blockedPA)
  step(1)

  present(Some(blockedC))
  respond(miss = false, blockedPB)
  assert(peek(c.io.request.ready) == 0,
    "scheduler accepted a third result without a queue credit")
  assert(peek(c.io.tlb.req.valid) == 0)
  expectResult(blockedA.tag, blockedPA)
  step(1)

  // Both A and B are queued. Holding backpressure must preserve A and keep C
  // stalled, rather than overwriting either terminal result.
  respond(miss = false)
  for (_ <- 0 until 2) {
    assert(peek(c.io.request.ready) == 0)
    assert(peek(c.io.tlb.req.valid) == 0)
    expectResult(blockedA.tag, blockedPA)
    step(1)
  }

  // Dequeuing A returns one credit, so C launches immediately.
  poke(c.io.result.ready, 1)
  assert(peek(c.io.request.ready) == 1)
  expectSelected(blockedC)
  expectResult(blockedA.tag, blockedPA)
  step(1)

  present(None)
  respond(miss = false, blockedPC)
  expectResult(blockedB.tag, blockedPB)
  step(1)

  respond(miss = false)
  expectResult(blockedC.tag, blockedPC)
  step(1)
  assert(peek(c.io.result.valid) == 0,
    "backpressured result stream emitted a duplicate terminal response")
}

class VpuTLBTranslationSchedulerSpec extends ChiselFlatSpec {
  behavior of "VpuTLBTranslationScheduler"

  it should "pipeline hits, replay misses exactly, and preserve backpressured results" in {
    implicit val params: Parameters = (new DefaultConfig).toInstance
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-tlb-translation-scheduler"),
      () => LazyModule(new VpuTLBTranslationSchedulerHarness(
        tagBits = 3,
        maxSize = 64,
        resultQueueEntries = 2)).module) {
      c => new VpuTLBTranslationSchedulerTester(c)
    } should be (true)
  }
}
