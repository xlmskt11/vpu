package vpu

import chisel3._
import chisel3.util._

/** Standalone one-range checker retained as a focused RAW/WAR/WAW unit test.
  *
  * This module is not instantiated by [[VpuCore]]. The live core applies the
  * same interval rule to three accesses per command inside
  * [[VpuReservationStation]].
  */
class VpuHazardTracker(p: VpuParams) extends Module {
  val io = IO(new Bundle {
    val probe = Input(Valid(new VpuHazardRange(p)))
    val conflict = Output(Bool())
    val allocate = Flipped(Decoupled(new VpuHazardRange(p)))
    val allocatedTag = Output(UInt(log2Ceil(p.hazardEntries).W))
    val release = Flipped(Valid(UInt(log2Ceil(p.hazardEntries).W)))
    val empty = Output(Bool())
  })

  val valid = RegInit(VecInit(Seq.fill(p.hazardEntries)(false.B)))
  val ranges = Reg(Vec(p.hazardEntries, new VpuHazardRange(p)))
  val hasFree = !valid.asUInt.andR

  // PriorityEncoder above operates on occupied bits; use the complement for
  // allocation.  Keeping the wires separate makes the generated assertions
  // easier to inspect.
  val freeOHCorrect = PriorityEncoderOH(~valid.asUInt)
  val freeIndexCorrect = OHToUInt(freeOHCorrect)
  def end(base: UInt, elements: UInt): UInt = base +& elements
  def overlaps(a: VpuHazardRange, b: VpuHazardRange): Bool =
    a.elementCount =/= 0.U && b.elementCount =/= 0.U &&
      a.base < end(b.base, b.elementCount) &&
      b.base < end(a.base, a.elementCount)
  def conflicts(a: VpuHazardRange, b: VpuHazardRange): Bool =
    overlaps(a, b) && ((a.write && (b.read || b.write)) ||
      (a.read && b.write))

  io.conflict := io.probe.valid && (0 until p.hazardEntries).map { i =>
    valid(i) && conflicts(io.probe.bits, ranges(i))
  }.reduce(_ || _)

  val allocationConflict = (0 until p.hazardEntries).map { i =>
    valid(i) && conflicts(io.allocate.bits, ranges(i))
  }.reduce(_ || _)
  io.allocate.ready := hasFree && !allocationConflict
  io.allocatedTag := freeIndexCorrect

  when (io.allocate.fire) {
    valid(freeIndexCorrect) := true.B
    ranges(freeIndexCorrect) := io.allocate.bits
    ranges(freeIndexCorrect).tag := freeIndexCorrect
  }
  when (io.release.valid) {
    assert(valid(io.release.bits), "Releasing an inactive VPU hazard tag")
    valid(io.release.bits) := false.B
  }

  io.empty := !valid.asUInt.orR
}
