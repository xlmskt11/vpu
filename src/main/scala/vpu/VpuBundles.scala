package vpu

import chisel3._
import chisel3.util._

import freechips.rocketchip.rocket.MStatus

/** Fault information retained at the first failing virtual byte address.
  * `cause` uses [[VpuDmaFaultCause]] and `isWrite` distinguishes H_STORE from
  * H_PREFETCH without overloading the cause encoding.
  */
class VpuDmaFaultInfo extends Bundle {
  val vaddr = UInt(64.W)
  val cause = UInt(2.W)
  val isWrite = Bool()
}

class VpuDmaDescriptor(p: VpuParams) extends Bundle {
  val vaddr = UInt(64.W)
  val spadElement = UInt(p.elementAddrBits.W)
  // Number of useful elements in each logical row. `elements` is already a
  // method on Chisel Record/Bundle.
  val elementCount = UInt(p.vlBits.W)
  // The local address of row r is spadElement + r*VLEN.  The external address
  // is vaddr + r*hostStrideBytes.  Zero rowCount is a legacy alias for one row
  // so existing one-dimensional producers remain binary-compatible.
  val rowCount = UInt(p.dmaRowCountBits.W)
  val hostStrideBytes = UInt(64.W)
  // Loads use their live RS tag. Stores use an independent transport tag so
  // the RS entry may retire at final VSRAM-read acceptance while TL D remains
  // outstanding.
  val commandTag = UInt(p.dmaCommandTagBits.W)
  // Translation privilege belongs to the command which created this
  // descriptor, not whichever RoCC command happens to be newest later.
  val status = new MStatus
}

/** One tagged memory-bus beat returned by the wrapper's DMA reader.
  *
  * spadElement identifies lane zero of this beat.  A wrapper must split a
  * response before it crosses a VPU lane word; this is naturally true for
  * contiguous transfers because vector bases are VLEN aligned and
  * dmaBusWidth divides the SRAM word width.
  */
class VpuDmaReadBeat(p: VpuParams) extends Bundle {
  val data = UInt(p.dmaBusWidth.W)
  val elementMask = UInt(p.dmaElementsPerBeat.W)
  val spadElement = UInt(p.elementAddrBits.W)
  val commandTag = UInt(p.dmaCommandTagBits.W)
  val last = Bool()
  val error = Bool()
  val fault = new VpuDmaFaultInfo
}

/** SRAM data streamed to the wrapper after a store descriptor. */
class VpuDmaWriteBeat(p: VpuParams) extends Bundle {
  val data = UInt(p.dmaBusWidth.W)
  val elementMask = UInt(p.dmaElementsPerBeat.W)
  val spadElement = UInt(p.elementAddrBits.W)
  // Store payload and completion lifetimes are independent. Preserve the
  // transport tag with every SRAM-sourced beat so a pipelined writer can
  // distinguish descriptors even after their RS slots have been recycled.
  val commandTag = UInt(p.dmaCommandTagBits.W)
  val last = Bool()
}

class VpuDmaWriteCompletion(p: VpuParams) extends Bundle {
  val commandTag = UInt(p.dmaCommandTagBits.W)
  val error = Bool()
  val fault = new VpuDmaFaultInfo
}

class VpuDmaIO(p: VpuParams) extends Bundle {
  val readDescriptor = Decoupled(new VpuDmaDescriptor(p))
  val readData = Flipped(Decoupled(new VpuDmaReadBeat(p)))
  val writeDescriptor = Decoupled(new VpuDmaDescriptor(p))
  val writeData = Decoupled(new VpuDmaWriteBeat(p))
  val writeCompletion = Flipped(Decoupled(new VpuDmaWriteCompletion(p)))
  // C_CLEAR_STATUS bit 1 requests a fault/TLB clear. The wrapper acknowledges
  // only after all accepted TL requests and streamed completions have drained.
  val clearFault = Output(Bool())
  val clearFaultDone = Input(Bool())
  // Sticky wrapper-side admission gate; active descriptors are still drained.
  val halted = Input(Bool())
}

class VpuSpadReadRequest(p: VpuParams) extends Bundle {
  val address0 = UInt(p.elementAddrBits.W)
  val address1 = UInt(p.elementAddrBits.W)
  val useAddress1 = Bool()
  /** Opaque request identity, returned unchanged with the response. */
  val tag = UInt(p.spadReadTagBits.W)
}

class VpuSpadReadResponse(p: VpuParams) extends Bundle {
  val data0 = Vec(p.nLanes, UInt(p.storageBits.W))
  val data1 = Vec(p.nLanes, UInt(p.storageBits.W))
  val serialized = Bool()
  val tag = UInt(p.spadReadTagBits.W)
}

class VpuSpadWriteRequest(p: VpuParams) extends Bundle {
  val address = UInt(p.elementAddrBits.W)
  val data = Vec(p.nLanes, UInt(p.storageBits.W))
  val laneMask = Vec(p.nLanes, Bool())
}

class VpuHazardRange(p: VpuParams) extends Bundle {
  val base = UInt(p.elementAddrBits.W)
  val elementCount = UInt(p.vlBits.W)
  val read = Bool()
  val write = Bool()
  val tag = UInt(log2Ceil(p.hazardEntries).W)
}
