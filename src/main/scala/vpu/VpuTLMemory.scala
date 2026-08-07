package vpu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import freechips.rocketchip.tile.HasCoreParameters
import freechips.rocketchip.tilelink._

import gemmini.FrontendTLB

private[vpu] object VpuTLUtil {
  /** Select the naturally aligned power-of-two TileLink transaction which
    * covers the most still-useful bytes starting at `currentAddress`.
    *
    * This is the same policy used by Gemmini's stream DMA: the request address
    * may be rounded down, while `shift` records the prefix bytes which are not
    * part of the architectural stream. Ties prefer the smaller transaction,
    * which avoids needless over-fetch at a short tail. In the default VPU
    * configuration the candidates are 16, 32 and 64 bytes.
    */
  def bestAlignedTransaction(
      currentAddress: UInt,
      bytesRemaining: UInt,
      minimum: Int,
      maximum: Int): (UInt, UInt, UInt, UInt, UInt) = {
    require(minimum > 0 && isPow2(minimum))
    require(maximum >= minimum && isPow2(maximum))
    val countBits = log2Ceil(maximum + 1)
    val lgBits = math.max(1, log2Ceil(log2Ceil(maximum) + 1))
    val candidates: Seq[(UInt, UInt, UInt, UInt, UInt)] =
      (minimum to maximum by minimum).
      filter(bytes => isPow2(bytes)).map { bytes =>
      val lg = log2Ceil(bytes)
      val base = Cat(currentAddress(currentAddress.getWidth - 1, lg),
        0.U(lg.W))
      val shift = currentAddress(lg - 1, 0).pad(countBits)
      val capacity = bytes.U(countBits.W) - shift
      val useful = Mux(bytesRemaining < capacity, bytesRemaining, capacity)
      (base, bytes.U(countBits.W), lg.U(lgBits.W), useful, shift)
    }
    candidates.reduceLeft { (older, newer) =>
      val chooseNewer = newer._4 > older._4
      (Mux(chooseNewer, newer._1, older._1),
        Mux(chooseNewer, newer._2, older._2),
        Mux(chooseNewer, newer._3, older._3),
        Mux(chooseNewer, newer._4, older._4),
        Mux(chooseNewer, newer._5, older._5))
    }
  }
}

/** Reader and writer share one translated VPU port while retaining independent
  * pipelined engines. The parent RoCC exposes idNode as its dedicated TL node.
  */
class VpuTLMemory(vpuParams: VpuParams)(implicit params: Parameters)
    extends LazyModule {
  val reader = LazyModule(new VpuTLReaderPipelined(vpuParams))
  val writer = LazyModule(new VpuTLWriterPipelined(vpuParams))
  private val xbar = TLXbar()
  val idNode = TLIdentityNode()

  xbar := TLBuffer() := reader.node
  xbar := TLBuffer() := writer.node
  idNode := TLWidthWidget(vpuParams.dmaBusWidth / 8) := TLBuffer() := xbar

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) with HasCoreParameters {
    implicit val edge: TLEdgeOut = reader.node.edges.out.head
    private val sourceBits = math.max(1,
      log2Ceil(vpuParams.dmaMaxInFlight))
    private val translationTagBits = sourceBits + 1

    val io = IO(new Bundle {
      val dma = Flipped(new VpuDmaIO(vpuParams))
      val ptw = new freechips.rocketchip.rocket.TLBPTWIO()(p)
      val busy = Output(Bool())
      val fault = Output(Bool())
    })

    val tlb = Module(new FrontendTLB(
      nClients = 1,
      entries = vpuParams.tlbEntries,
      maxSize = vpuParams.dmaMaxBytes,
      // FrontendTLB's optional L0 filter keys only on VPN and leaves the
      // exception fields invalid on a filter hit. VPU descriptors carry an
      // independently snapshotted MStatus, so reusing that translation across
      // descriptors could both lose precise faults and apply stale privilege
      // context. Keep the Rocket TLB itself enabled, but bypass this L0
      // register filter for descriptor-correct translation.
      use_tlb_register_filter = false,
      use_firesim_simulation_counters = false,
      use_shared_tlb = true)(edge, p))
    val translationScheduler = Module(new VpuTLBTranslationScheduler(
      translationTagBits, vpuParams.dmaMaxBytes)(p))
    val translationArbiter = Module(new RRArbiter(
      new VpuTLBTaggedRequest(translationTagBits,
        vpuParams.dmaMaxBytes)(p), 2))

    translationArbiter.io.in(0) <> reader.module.io.translationRequest
    translationArbiter.io.in(1) <> writer.module.io.translationRequest
    translationScheduler.io.request <> translationArbiter.io.out
    translationScheduler.io.tlb <> tlb.io.clients.head

    val resultIsWriter =
      translationScheduler.io.result.bits.tag(translationTagBits - 1)
    reader.module.io.translationResult.valid :=
      translationScheduler.io.result.valid && !resultIsWriter
    reader.module.io.translationResult.bits :=
      translationScheduler.io.result.bits
    writer.module.io.translationResult.valid :=
      translationScheduler.io.result.valid && resultIsWriter
    writer.module.io.translationResult.bits :=
      translationScheduler.io.result.bits
    translationScheduler.io.result.ready := Mux(resultIsWriter,
      writer.module.io.translationResult.ready,
      reader.module.io.translationResult.ready)

    val halted = RegInit(false.B)
    val clearPending = RegInit(false.B)
    val enginesDrained = !reader.module.io.busy && !writer.module.io.busy
    val faultEvent = reader.module.io.fault || writer.module.io.fault
    val clearRequested = clearPending || io.dma.clearFault
    // An accepted A-channel request must always receive its D response. Fault
    // clearing therefore invalidates the TLB only after both engines have
    // delivered their final error/data completion and become idle.
    val clearNow = clearRequested && enginesDrained && !faultEvent

    when(io.dma.clearFault) { clearPending := true.B }
    when(faultEvent || tlb.io.exp.head.interrupt) { halted := true.B }
    when(clearNow) {
      clearPending := false.B
      halted := false.B
    }

    val admitDescriptor = !halted && !clearPending && !io.dma.clearFault &&
      !faultEvent && !tlb.io.exp.head.interrupt
    reader.module.io.descriptor.valid :=
      io.dma.readDescriptor.valid && admitDescriptor
    reader.module.io.descriptor.bits := io.dma.readDescriptor.bits
    io.dma.readDescriptor.ready :=
      reader.module.io.descriptor.ready && admitDescriptor
    io.dma.readData <> reader.module.io.data
    writer.module.io.descriptor.valid :=
      io.dma.writeDescriptor.valid && admitDescriptor
    writer.module.io.descriptor.bits := io.dma.writeDescriptor.bits
    io.dma.writeDescriptor.ready :=
      writer.module.io.descriptor.ready && admitDescriptor
    writer.module.io.data <> io.dma.writeData
    io.dma.writeCompletion <> writer.module.io.completion

    io.ptw <> tlb.io.ptw.head
    tlb.io.exp.head.flush_retry := false.B
    tlb.io.exp.head.flush_skip := clearNow
    tlb.io.counter.external_reset := false.B

    io.dma.clearFaultDone := clearNow
    io.dma.halted := halted || tlb.io.exp.head.interrupt
    io.busy := !enginesDrained || clearPending
    io.fault := halted || tlb.io.exp.head.interrupt
  }
}
