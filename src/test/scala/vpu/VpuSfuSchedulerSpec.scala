package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

import scala.collection.mutable.ArrayBuffer

/** Confirms that the execute controller exploits the II=1 Valid pipelines in
  * EXP and reciprocal instead of serializing issue/result per SFU chunk.
  */
class VpuSfuSchedulerTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

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
    poke(c.io.command.bits.xd, xd)
    var timeout = 200
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"command 0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  def consumeResponse(expectedRd: Int): Unit = {
    assert(peek(c.io.response.valid) == 1)
    assert(peek(c.io.response.bits.rd) == expectedRd)
    assert((peek(c.io.response.bits.data) & 7) == 0)
    poke(c.io.response.ready, 1)
    step(1)
    poke(c.io.response.ready, 0)
  }

  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = 0)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = p.elementsPerBank)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = 2 * p.elementsPerBank)
  issue(VpuOpcode.C_WRITE_GP, rd = 15, payload = 0)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = BigInt("100000", 16))
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 15, rs2 = 0)

  var timeout = 100
  while (peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
    step(1)
    timeout -= 1
  }
  assert(timeout > 0)
  val readTag = peek(c.io.dma.readDescriptor.bits.commandTag)
  poke(c.io.dma.readDescriptor.ready, 1)
  step(1)
  poke(c.io.dma.readDescriptor.ready, 0)

  for (beat <- 0 until p.vLen / p.dmaElementsPerBeat) {
    val packed = (0 until p.dmaElementsPerBeat).map { lane =>
      bits(0.5f + beat * p.dmaElementsPerBeat + lane) << (32 * lane)
    }.reduce(_ | _)
    poke(c.io.dma.readData.valid, 1)
    poke(c.io.dma.readData.bits.data, packed)
    poke(c.io.dma.readData.bits.elementMask,
      (BigInt(1) << p.dmaElementsPerBeat) - 1)
    poke(c.io.dma.readData.bits.spadElement,
      beat * p.dmaElementsPerBeat)
    poke(c.io.dma.readData.bits.commandTag, readTag)
    poke(c.io.dma.readData.bits.last,
      if (beat == p.vLen / p.dmaElementsPerBeat - 1) 1 else 0)
    while (peek(c.io.dma.readData.ready) == 0) { step(1) }
    step(1)
    poke(c.io.dma.readData.valid, 0)
  }

  def runVectorSfu(opcode: Int, destinationGp: Int, expectedLatency: Int,
                   responseRd: Int): Unit = {
    issue(opcode, rd = destinationGp, rs1 = 0)
    issue(VpuOpcode.C_FENCE, xd = true, roccRd = responseRd)

    val issueCycles = ArrayBuffer.empty[Int]
    val resultCycles = ArrayBuffer.empty[Int]
    var cycle = 0
    var monitorTimeout = 400
    while (peek(c.io.response.valid) == 0 && monitorTimeout > 0) {
      if (peek(c.io.debugSfuIssue) != 0) { issueCycles += cycle }
      if (peek(c.io.debugSfuResult) != 0) { resultCycles += cycle }
      step(1)
      cycle += 1
      monitorTimeout -= 1
    }
    assert(monitorTimeout > 0, s"SFU opcode 0x${opcode.toHexString} timed out")
    val lanes = if (opcode == VpuOpcode.V_RECI_V)
      p.reciprocalLanes else p.sfuLanes
    val groups = p.vLen / lanes
    assert(issueCycles.size == groups,
      s"SFU issued ${issueCycles.size}/$groups groups")
    assert(resultCycles.size == groups,
      s"SFU collected ${resultCycles.size}/$groups groups")
    assert(issueCycles.sliding(2).forall(x => x(1) == x(0) + 1),
      s"SFU input valid was not II=1: $issueCycles")
    assert(resultCycles.sliding(2).forall(x => x(1) == x(0) + 1),
      s"SFU output valid was not II=1: $resultCycles")
    assert(resultCycles.head - issueCycles.head == expectedLatency,
      s"SFU latency ${resultCycles.head - issueCycles.head}, expected $expectedLatency")
    consumeResponse(responseRd)
  }

  // Four SRAM words deliberately make the old word-at-a-time scheduler fail:
  // it inserted a complete result/write/read drain at each 16-element word
  // boundary.  EXP's five-cycle and reciprocal's thirteen-cycle pipelines
  // must instead remain full across all three word boundaries.
  runVectorSfu(VpuOpcode.V_EXP_V, destinationGp = 1,
    expectedLatency = 5, responseRd = 3)
  runVectorSfu(VpuOpcode.V_RECI_V, destinationGp = 2,
    expectedLatency = p.reciprocalLatency, responseRd = 4)
}

class VpuSfuSchedulerSpec extends ChiselFlatSpec {
  behavior of "VpuCore SFU chunk scheduler"
  it should "stream opcode-specific II=1 groups across SRAM words" in {
    val p = VpuParams(vLen = 64, nLanes = 16, sfuLanes = 2, vSpadKB = 2)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-sfu-scheduler"),
      () => new VpuCore(p)) { c =>
      new VpuSfuSchedulerTester(c, p)
    } should be (true)
  }
}
