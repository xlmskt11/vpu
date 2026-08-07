package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

import scala.collection.mutable.ArrayBuffer
import scala.util.Random

/** Cycle-accurate regression for the base vector ALU streaming path. */
class VpuAluPipelineTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  private val hostBase = BigInt("400000", 16)
  private val random = new Random(0x414c5531L)

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

  def issue(opcode: Int, rd: Int = 0, rs1: Int = 0, rs2: Int = 0,
            payload: BigInt = 0, xd: Boolean = false,
            roccRd: Int = 0): Unit = {
    poke(c.io.command.valid, 1)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd, rs1, rs2) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, if (xd) 1 else 0)
    var timeout = 300
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"command 0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  def pack(values: Seq[Float]): BigInt =
    values.zipWithIndex.map { case (value, lane) =>
      bits(value) << (lane * p.storageBits)
    }.reduce(_ | _)

  def load(spadBase: Int, values: Seq[Float]): Unit = {
    issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 15, rs2 = 0)
    poke(c.io.dma.readDescriptor.ready, 1)
    var timeout = 200
    while (peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0)
    assert(peek(c.io.dma.readDescriptor.bits.spadElement) == spadBase)
    val readTag = peek(c.io.dma.readDescriptor.bits.commandTag)
    step(1)
    poke(c.io.dma.readDescriptor.ready, 0)

    val beatCount = p.vLen / p.dmaElementsPerBeat
    for (beat <- 0 until beatCount) {
      if (random.nextBoolean()) step(1)
      val slice = values.slice(beat * p.dmaElementsPerBeat,
        (beat + 1) * p.dmaElementsPerBeat)
      poke(c.io.dma.readData.valid, 1)
      poke(c.io.dma.readData.bits.data, pack(slice))
      poke(c.io.dma.readData.bits.elementMask,
        (BigInt(1) << p.dmaElementsPerBeat) - 1)
      poke(c.io.dma.readData.bits.spadElement,
        spadBase + beat * p.dmaElementsPerBeat)
      poke(c.io.dma.readData.bits.commandTag, readTag)
      poke(c.io.dma.readData.bits.last, if (beat == beatCount - 1) 1 else 0)
      timeout = 200
      while (peek(c.io.dma.readData.ready) == 0 && timeout > 0) {
        step(1)
        timeout -= 1
      }
      assert(timeout > 0)
      step(1)
      poke(c.io.dma.readData.valid, 0)
    }
  }

  def collectPulseCycles(signal: => BigInt, count: Int,
                         timeoutLimit: Int = 300): Seq[Int] = {
    val cycles = ArrayBuffer.empty[Int]
    var cycle = 0
    while (cycles.size < count && cycle < timeoutLimit) {
      if (signal == 1) cycles += cycle
      step(1)
      cycle += 1
    }
    assert(cycles.size == count,
      s"saw ${cycles.size}/$count pipeline pulses in $timeoutLimit cycles")
    cycles.toSeq
  }

  // Read issue and writeback overlap once the ALU pipeline fills. Observe both
  // in the same cycle window so a same-bank read cadence cannot hide an early
  // writeback while the test is still waiting for the final read request.
  def collectAluActivity(count: Int,
                         timeoutLimit: Int = 300): (Seq[Int], Seq[Int]) = {
    val reads = ArrayBuffer.empty[Int]
    val writes = ArrayBuffer.empty[Int]
    var cycle = 0
    while ((reads.size < count || writes.size < count) &&
        cycle < timeoutLimit) {
      if (peek(c.io.debugAluReadIssue) == 1 && reads.size < count) {
        reads += cycle
      }
      if (peek(c.io.debugAluWriteback) == 1 && writes.size < count) {
        writes += cycle
      }
      step(1)
      cycle += 1
    }
    assert(reads.size == count,
      s"saw ${reads.size}/$count ALU read pulses in $timeoutLimit cycles")
    assert(writes.size == count,
      s"saw ${writes.size}/$count ALU writeback pulses in $timeoutLimit cycles")
    (reads.toSeq, writes.toSeq)
  }

  def store(spadBase: Int, expected: Seq[Float]): Unit = {
    poke(c.io.dma.writeDescriptor.ready, 1)
    issue(VpuOpcode.H_STORE_V, rd = 2, rs1 = 15, rs2 = 0)
    var timeout = 400
    while (peek(c.io.dma.writeDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0)
    assert(peek(c.io.dma.writeDescriptor.bits.spadElement) == spadBase)
    val storeTag = peek(c.io.dma.writeDescriptor.bits.commandTag)
    step(1)
    poke(c.io.dma.writeDescriptor.ready, 0)

    val beatCount = p.vLen / p.dmaElementsPerBeat
    for (beat <- 0 until beatCount) {
      timeout = 300
      while (peek(c.io.dma.writeData.valid) == 0 && timeout > 0) {
        step(1)
        timeout -= 1
      }
      assert(timeout > 0)
      // Random output backpressure must hold the tagged writeback result.
      val stalls = random.nextInt(3)
      var held = peek(c.io.dma.writeData.bits.data)
      for (_ <- 0 until stalls) {
        poke(c.io.dma.writeData.ready, 0)
        assert(peek(c.io.dma.writeData.valid) == 1)
        assert(peek(c.io.dma.writeData.bits.data) == held)
        step(1)
        held = peek(c.io.dma.writeData.bits.data)
      }
      poke(c.io.dma.writeData.ready, 1)
      assert(peek(c.io.dma.writeData.bits.commandTag) == storeTag)
      val packed = peek(c.io.dma.writeData.bits.data)
      for (lane <- 0 until p.dmaElementsPerBeat) {
        val actual = (packed >> (lane * 32)) & BigInt("ffffffff", 16)
        val wanted = bits(expected(beat * p.dmaElementsPerBeat + lane))
        assert(actual == wanted,
          s"store[$beat,$lane]=0x${actual.toString(16)} " +
            s"expected=0x${wanted.toString(16)}")
      }
      step(1)
      poke(c.io.dma.writeData.ready, 0)
    }
    poke(c.io.dma.writeCompletion.valid, 1)
    poke(c.io.dma.writeCompletion.bits.commandTag, storeTag)
    while (peek(c.io.dma.writeCompletion.ready) == 0) step(1)
    step(1)
    poke(c.io.dma.writeCompletion.valid, 0)
  }

  val sourceA = 0
  val sourceSameBank = p.vLen
  val sourceOtherBank = p.elementsPerBank
  val destination = 2 * p.elementsPerBank
  val a = (0 until p.vLen).map(i => i.toFloat + 0.25f)
  val b = (0 until p.vLen).map(i => 2.0f * i - 3.0f)
  val sum = a.zip(b).map { case (x, y) => x + y }

  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = hostBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 15, payload = 0)

  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = sourceA)
  load(sourceA, a)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = sourceOtherBank)
  load(sourceOtherBank, b)

  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = sourceA)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = sourceOtherBank)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = destination)
  issue(VpuOpcode.V_ADD_VV, rd = 2, rs1 = 0, rs2 = 1)

  val beats = p.wordsPerVector
  val (readCycles, writeCycles) = collectAluActivity(beats)
  assert(readCycles.sliding(2).forall(pair => pair(1) - pair(0) == 1),
    s"disjoint-bank ALU reads were not II=1: $readCycles")
  assert(writeCycles.sliding(2).forall(pair => pair(1) - pair(0) == 1),
    s"ALU writeback was not II=1: $writeCycles")
  store(destination, sum)

  // Repeat with two source vectors in distinct rows of bank zero. Each pair
  // needs two SRAM reads, so request accepts must be exactly two cycles apart.
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = sourceSameBank)
  load(sourceSameBank, b)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = sourceA)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = sourceSameBank)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = destination)
  issue(VpuOpcode.V_ADD_VV, rd = 2, rs1 = 0, rs2 = 1)
  val (conflictReadCycles, conflictWriteCycles) = collectAluActivity(beats)
  assert(conflictReadCycles.sliding(2).forall(pair => pair(1) - pair(0) == 2),
    s"same-bank reads did not use maximal two-cycle cadence: $conflictReadCycles")
  assert(conflictWriteCycles.sliding(2).forall(pair => pair(1) - pair(0) == 2),
    s"same-bank ALU writebacks did not follow the operand cadence: " +
      conflictWriteCycles)
  assert(peek(c.io.perfCounters(VpuPerfIndex.BankConflictStallCycles)) >= beats,
    "same-bank VV operation did not increment bankConflictStall")
  store(destination, sum)
}

class VpuAluPipelineSpec extends ChiselFlatSpec {
  behavior of "VpuCore base vector ALU pipeline"

  it should "sustain II=1 and serialize only true same-bank pairs" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2,
      vSpadKB = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-alu-pipeline"),
      () => new VpuCore(p)) { c =>
      new VpuAluPipelineTester(c, p)
    } should be (true)
  }
}
