package vpu

import chisel3._
import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

import scala.util.Random

/** Blocks the execute VSRAM write port through the higher-priority matrix
  * client while more than four ALU/SFU words are in flight.  The fixed-latency
  * arithmetic pipelines cannot be backpressured, so this directly checks that
  * issue-side credits preserve every result until the bounded WB queues drain.
  */
private class VpuArithmeticCreditTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  private val random = new Random(0x43524544L)
  private val hostBase = BigInt("700000", 16)
  private val inputBase = 0
  private val aluBase = p.elementsPerBank
  private val expBase = 2 * p.elementsPerBank
  require(p.matrixPorts == 1)
  require(p.wordsPerVector > VpuCore.ArithmeticWritebackQueueDepth)

  private def leaves(data: Data): Seq[Bits] = data match {
    case bits: Bits => Seq(bits)
    case aggregate: Aggregate => aggregate.getElements.flatMap(leaves)
  }
  private def clear(data: Data): Unit =
    leaves(data).foreach(bits => poke(bits, 0))

  clear(c.io.command.bits)
  poke(c.io.command.valid, 0)
  poke(c.io.response.ready, 0)
  poke(c.io.dma.readDescriptor.ready, 0)
  poke(c.io.dma.readData.valid, 0)
  clear(c.io.dma.readData.bits)
  poke(c.io.dma.writeDescriptor.ready, 0)
  poke(c.io.dma.writeData.ready, 0)
  poke(c.io.dma.writeCompletion.valid, 0)
  clear(c.io.dma.writeCompletion.bits)
  poke(c.io.dma.clearFaultDone, 0)
  poke(c.io.dma.halted, 0)
  poke(c.io.matrixRead.get(0).req.valid, 0)
  clear(c.io.matrixRead.get(0).req.bits)
  poke(c.io.matrixRead.get(0).resp.ready, 1)
  poke(c.io.matrixWrite.get(0).valid, 0)
  clear(c.io.matrixWrite.get(0).bits)
  step(3)

  private def issue(opcode: Int, rd: Int = 0, rs1: Int = 0,
                    rs2: Int = 0, payload: BigInt = 0,
                    xd: Boolean = false, roccRd: Int = 0): Unit = {
    clear(c.io.command.bits)
    poke(c.io.command.bits.microOp, BigInt(VpuEncoding.pack(
      opcode, rd, rs1, rs2) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, if (xd) 1 else 0)
    poke(c.io.command.bits.status.dprv, 3)
    poke(c.io.command.bits.status.prv, 3)
    poke(c.io.command.valid, 1)
    var timeout = 1000
    while(peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"opcode 0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  private def pack(values: Seq[Float]): BigInt =
    values.zipWithIndex.map { case (value, lane) =>
      bits(value) << (lane * p.storageBits)
    }.reduce(_ | _)

  private val input = (0 until p.vLen).map { index =>
    -2.0f + 4.0f * index / (p.vLen - 1).toFloat
  }

  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = inputBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = aluBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = expBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 15, payload = 0)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = hostBase)
  issue(VpuOpcode.C_WRITE_FP, rd = 0, payload = bits(1.0f))
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 15, rs2 = 0)

  var timeout = 1000
  while(peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0)
  val loadTag = peek(c.io.dma.readDescriptor.bits.commandTag)
  poke(c.io.dma.readDescriptor.ready, 1)
  step(1)
  poke(c.io.dma.readDescriptor.ready, 0)
  for (beat <- 0 until p.vLen / p.dmaElementsPerBeat) {
    val first = beat * p.dmaElementsPerBeat
    clear(c.io.dma.readData.bits)
    poke(c.io.dma.readData.bits.data,
      pack(input.slice(first, first + p.dmaElementsPerBeat)))
    poke(c.io.dma.readData.bits.elementMask,
      (BigInt(1) << p.dmaElementsPerBeat) - 1)
    poke(c.io.dma.readData.bits.spadElement, first)
    poke(c.io.dma.readData.bits.commandTag, loadTag)
    poke(c.io.dma.readData.bits.last,
      if (beat == p.vLen / p.dmaElementsPerBeat - 1) 1 else 0)
    poke(c.io.dma.readData.valid, 1)
    while(peek(c.io.dma.readData.ready) == 0) step(1)
    step(1)
    poke(c.io.dma.readData.valid, 0)
  }

  private def blockExecuteBank(bankBase: Int, enabled: Boolean): Unit = {
    poke(c.io.matrixWrite.get(0).valid, if (enabled) 1 else 0)
    poke(c.io.matrixWrite.get(0).bits.rowAddress,
      (bankBase + p.vLen) / p.nLanes)
    // All-zero lane mask claims the physical port without changing data.
    for (lane <- 0 until p.nLanes) {
      poke(c.io.matrixWrite.get(0).bits.data(lane), 0)
      poke(c.io.matrixWrite.get(0).bits.laneMask(lane), 0)
    }
  }

  private def runBlocked(opcode: Int, destination: Int,
                         sfu: Boolean): Unit = {
    issue(opcode, rd = destination, rs1 = 0, rs2 = 0)
    var reads = 0
    var laneIssues = 0
    var results = 0
    var writes = 0
    var cycle = 0
    timeout = 5000
    while(writes < p.wordsPerVector && timeout > 0) {
      val hardBlock = cycle < 40
      val randomBlock = cycle >= 40 && random.nextInt(4) != 0
      blockExecuteBank(if (destination == 1) aluBase else expBase,
        hardBlock || randomBlock)
      if (!sfu && peek(c.io.debugAluReadIssue) == 1) reads += 1
      if (!sfu && peek(c.io.debugAluLaneIssue) == 1) laneIssues += 1
      if (!sfu && peek(c.io.debugAluResult) == 1) results += 1
      if (!sfu && peek(c.io.debugAluWriteback) == 1) writes += 1
      if (sfu && peek(c.io.debugSfuWriteback) == 1) writes += 1
      step(1)
      cycle += 1
      timeout -= 1
    }
    blockExecuteBank(aluBase, enabled = false)
    assert(timeout > 0,
      s"opcode 0x${opcode.toHexString} did not drain bounded WB credits")
    if (!sfu) {
      assert(reads == p.wordsPerVector)
      assert(laneIssues == p.wordsPerVector)
      assert(results == p.wordsPerVector)
    }
    assert(writes == p.wordsPerVector,
      s"opcode 0x${opcode.toHexString} wrote $writes/${p.wordsPerVector} words")
  }

  runBlocked(VpuOpcode.V_ADD_VF, destination = 1, sfu = false)
  runBlocked(VpuOpcode.V_EXP_V, destination = 2, sfu = true)

  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 3)
  timeout = 2000
  while(peek(c.io.response.valid) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0)
  assert(peek(c.io.response.bits.rd) == 3)
  assert((peek(c.io.response.bits.data) & 3) == 0)
  poke(c.io.response.ready, 1)
  step(1)
}

class VpuArithmeticCreditSpec extends ChiselFlatSpec {
  behavior of "VpuCore bounded arithmetic writeback credits"

  it should "preserve ALU and SFU results under long random matrix-write stalls" in {
    val p = VpuParams(vLen = 128, nLanes = 16, sfuLanes = 4,
      vSpadKB = 64, matrixPorts = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-arithmetic-credit"),
      () => new VpuCore(p)) { c =>
      new VpuArithmeticCreditTester(c, p)
    } should be(true)
  }
}
