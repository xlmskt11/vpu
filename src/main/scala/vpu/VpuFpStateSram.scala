package vpu

import chisel3._
import chisel3.util._

class VpuFpStateReadRequest(p: VpuParams) extends Bundle {
  val index = UInt(p.fpStateIndexBits.W)
}

class VpuFpStateWriteRequest(p: VpuParams) extends Bundle {
  val index = UInt(p.fpStateIndexBits.W)
  val data = UInt(32.W)
}

/** Small banked FP32 state SRAM for row-persistent nonlinear state.
  *
  * FlashAttention uses this memory for running `m` and `l` values.  It is not
  * part of VSRAM and consequently does not consume a vector-SRAM port or a
  * shared Gemmini/VPU dependency-table entry.  Each physical bank has one
  * synchronous read port and one write port.  The current single VPU execute
  * engine presents at most one architectural request of either kind per
  * cycle, while the banked organization leaves the interface ready for a
  * future multi-row state scheduler.
  */
class VpuFpStateSram(p: VpuParams) extends Module {
  private val bankBits = p.fpStateBankBits
  private val rowBits = math.max(1, log2Ceil(p.fpStateRowsPerBank))

  val io = IO(new Bundle {
    val read = Flipped(Decoupled(new VpuFpStateReadRequest(p)))
    val readData = Valid(UInt(32.W))
    val write = Flipped(Decoupled(new VpuFpStateWriteRequest(p)))
    val busy = Output(Bool())
  })

  val banks = Seq.fill(p.fpStateBanks)(
    SyncReadMem(p.fpStateRowsPerBank, UInt(32.W)))

  val readBank = io.read.bits.index(bankBits - 1, 0)
  val readRow = io.read.bits.index(p.fpStateIndexBits - 1, bankBits)
  val writeBank = io.write.bits.index(bankBits - 1, 0)
  val writeRow = io.write.bits.index(p.fpStateIndexBits - 1, bankBits)

  io.read.ready := true.B
  io.write.ready := true.B

  val bankReadData = Wire(Vec(p.fpStateBanks, UInt(32.W)))
  for (bank <- 0 until p.fpStateBanks) {
    bankReadData(bank) := banks(bank).read(
      readRow(rowBits - 1, 0), io.read.fire && readBank === bank.U)
    when(io.write.fire && writeBank === bank.U) {
      banks(bank).write(writeRow(rowBits - 1, 0), io.write.bits.data)
    }
  }

  val responseValid = RegNext(io.read.fire, false.B)
  val responseBank = RegEnable(readBank, io.read.fire)
  io.readData.valid := responseValid
  io.readData.bits := Mux1H(UIntToOH(responseBank, p.fpStateBanks),
    bankReadData)
  io.busy := responseValid

  when(io.read.fire) {
    assert(io.read.bits.index < p.fpStateEntries.U,
      "VPU FP state SRAM read index is out of range")
  }
  when(io.write.fire) {
    assert(io.write.bits.index < p.fpStateEntries.U,
      "VPU FP state SRAM write index is out of range")
  }

  dontTouch(io.busy)
}
