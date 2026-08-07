package vpu

import chisel3._
import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

/** Regressions for command admission while an older execute operation is
  * unable to retire.  The command input is shared by LD/EX/ST, so an execute
  * dependency must be represented inside the EX FIFO instead of holding the
  * RoCC command port and hiding younger independent memory commands.
  */
class VpuCommandDispatchTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  private val hostBase = BigInt("200000", 16)
  private val vectorA = 0
  private val storeSource = 32
  private val vectorC = 64
  private val expDestination = 96
  private val loadAOffset = 0
  private val loadCOffset = 64
  private val storeOffset = 128

  private def leafBits(data: Data): Seq[Bits] = data match {
    case bits: Bits => Seq(bits)
    case aggregate: Aggregate => aggregate.getElements.flatMap(leafBits)
  }

  private def clear(data: Data): Unit =
    leafBits(data).foreach(bits => poke(bits, 0))

  // Initialize every DMA input bit.  In particular, inactive error/fault and
  // last/tag fields must never retain values from the previous transaction.
  poke(c.io.command.valid, 0)
  clear(c.io.command.bits)
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
  step(3)

  private def driveCommand(opcode: Int, rd: Int = 0, rs1: Int = 0,
                           rs2: Int = 0, funct1: Int = 0,
                           payload: BigInt = 0, xd: Boolean = false,
                           roccRd: Int = 0): Unit = {
    clear(c.io.command.bits)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd, rs1, rs2,
        funct1 = funct1) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, xd)
    poke(c.io.command.bits.status.dprv, 3)
    poke(c.io.command.bits.status.prv, 3)
    poke(c.io.command.valid, 1)
  }

  private def issue(opcode: Int, rd: Int = 0, rs1: Int = 0,
                    rs2: Int = 0, funct1: Int = 0,
                    payload: BigInt = 0, xd: Boolean = false,
                    roccRd: Int = 0): Unit = {
    driveCommand(opcode, rd, rs1, rs2, funct1, payload, xd, roccRd)
    var timeout = 500
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"command opcode=0x${opcode.toHexString} enqueue timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  /** Require admission in the first cycle in which the command is presented.
    * Consecutive calls therefore exercise a genuinely back-to-back command
    * stream rather than hiding a queue stall in the generic timeout loop.
    */
  private def issueImmediate(opcode: Int, rd: Int = 0, rs1: Int = 0,
                             rs2: Int = 0, funct1: Int = 0,
                             payload: BigInt = 0, xd: Boolean = false,
                             roccRd: Int = 0): Unit = {
    driveCommand(opcode, rd, rs1, rs2, funct1, payload, xd, roccRd)
    assert(peek(c.io.command.ready) == 1,
      s"command opcode=0x${opcode.toHexString} was not admitted immediately")
    step(1)
    poke(c.io.command.valid, 0)
  }

  private def response(expectedRd: Int): BigInt = {
    var timeout = 3000
    while (peek(c.io.response.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"response rd=$expectedRd timed out")
    assert(peek(c.io.response.bits.rd) == expectedRd,
      s"response rd=${peek(c.io.response.bits.rd)} expected=$expectedRd")
    val data = peek(c.io.response.bits.data)
    poke(c.io.response.ready, 1)
    step(1)
    poke(c.io.response.ready, 0)
    data
  }

  private def acceptReadDescriptor(expectedSpad: Int,
                                   expectedVaddr: BigInt): BigInt = {
    var timeout = 500
    while (peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"read descriptor spad=$expectedSpad timed out")
    assert(peek(c.io.dma.readDescriptor.bits.spadElement) == expectedSpad)
    assert(peek(c.io.dma.readDescriptor.bits.elementCount) == p.vLen)
    assert(peek(c.io.dma.readDescriptor.bits.vaddr) == expectedVaddr)
    val tag = peek(c.io.dma.readDescriptor.bits.commandTag)
    poke(c.io.dma.readDescriptor.ready, 1)
    step(1)
    poke(c.io.dma.readDescriptor.ready, 0)
    tag
  }

  private def acceptWriteDescriptor(expectedSpad: Int,
                                    expectedVaddr: BigInt): BigInt = {
    var timeout = 500
    while (peek(c.io.dma.writeDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"write descriptor spad=$expectedSpad timed out")
    assert(peek(c.io.dma.writeDescriptor.bits.spadElement) == expectedSpad)
    assert(peek(c.io.dma.writeDescriptor.bits.elementCount) == p.vLen)
    assert(peek(c.io.dma.writeDescriptor.bits.vaddr) == expectedVaddr)
    val tag = peek(c.io.dma.writeDescriptor.bits.commandTag)
    poke(c.io.dma.writeDescriptor.ready, 1)
    step(1)
    poke(c.io.dma.writeDescriptor.ready, 0)
    tag
  }

  private def packBeat(values: Seq[Float]): BigInt =
    values.zipWithIndex.map { case (value, lane) =>
      bits(value) << (lane * p.storageBits)
    }.reduce(_ | _)

  private def sendReadVector(spadBase: Int, commandTag: BigInt,
                             values: Seq[Float]): Unit = {
    assert(values.length == p.vLen)
    val beatCount = p.vLen / p.dmaElementsPerBeat
    for (beat <- 0 until beatCount) {
      clear(c.io.dma.readData.bits)
      val first = beat * p.dmaElementsPerBeat
      poke(c.io.dma.readData.bits.data,
        packBeat(values.slice(first, first + p.dmaElementsPerBeat)))
      poke(c.io.dma.readData.bits.elementMask,
        (BigInt(1) << p.dmaElementsPerBeat) - 1)
      poke(c.io.dma.readData.bits.spadElement, spadBase + first)
      poke(c.io.dma.readData.bits.commandTag, commandTag)
      poke(c.io.dma.readData.bits.last, if (beat == beatCount - 1) 1 else 0)
      poke(c.io.dma.readData.valid, 1)
      var timeout = 500
      while (peek(c.io.dma.readData.ready) == 0 && timeout > 0) {
        step(1)
        timeout -= 1
      }
      assert(timeout > 0, s"read beat $beat timed out")
      step(1)
      poke(c.io.dma.readData.valid, 0)
    }
  }

  private def drainStore(spadBase: Int, commandTag: BigInt): Unit = {
    val beatCount = p.vLen / p.dmaElementsPerBeat
    for (beat <- 0 until beatCount) {
      var timeout = 500
      while (peek(c.io.dma.writeData.valid) == 0 && timeout > 0) {
        step(1)
        timeout -= 1
      }
      assert(timeout > 0, s"store beat $beat timed out")
      assert(peek(c.io.dma.writeData.bits.spadElement) ==
        spadBase + beat * p.dmaElementsPerBeat)
      assert(peek(c.io.dma.writeData.bits.commandTag) == commandTag)
      assert(peek(c.io.dma.writeData.bits.elementMask) ==
        (BigInt(1) << p.dmaElementsPerBeat) - 1)
      assert(peek(c.io.dma.writeData.bits.last) ==
        (if (beat == beatCount - 1) 1 else 0))
      poke(c.io.dma.writeData.ready, 1)
      step(1)
      poke(c.io.dma.writeData.ready, 0)
    }

    clear(c.io.dma.writeCompletion.bits)
    poke(c.io.dma.writeCompletion.bits.commandTag, commandTag)
    poke(c.io.dma.writeCompletion.valid, 1)
    var timeout = 500
    while (peek(c.io.dma.writeCompletion.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "store completion timed out")
    step(1)
    poke(c.io.dma.writeCompletion.valid, 0)
    clear(c.io.dma.writeCompletion.bits)
  }

  // Common architectural setup. GP13..15 are host element offsets; all VSRAM
  // bases are VLEN-aligned and reside in disjoint banks.
  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = hostBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = vectorA)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = storeSource)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = vectorC)
  issue(VpuOpcode.C_WRITE_GP, rd = 4, payload = expDestination)
  issue(VpuOpcode.C_WRITE_GP, rd = 13, payload = loadAOffset)
  issue(VpuOpcode.C_WRITE_GP, rd = 14, payload = loadCOffset)
  issue(VpuOpcode.C_WRITE_GP, rd = 15, payload = storeOffset)
  issue(VpuOpcode.C_WRITE_FP, rd = 0, payload = bits(1.0f))
  issue(VpuOpcode.C_WRITE_FP, rd = 1, payload = bits(0.0f))
  issue(VpuOpcode.C_WRITE_FP, rd = 3, payload = bits(2.0f))

  // ----------------------------------------------------------------------
  // 1) A stalled load blocks the reduction at the EX head. The dependent
  // scalar op must still enter the EX FIFO, after which disjoint ST and LD
  // commands must remain visible to their own queues without a front-end HOL
  // stall. FP operands are read at issue, so 16 ones produce 16 * 2 = 32.
  // ----------------------------------------------------------------------
  issueImmediate(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 13, rs2 = 0)
  issueImmediate(VpuOpcode.V_RED_SUM, rd = 1, rs1 = 0)
  issueImmediate(VpuOpcode.S_MUL_FP, rd = 2, rs1 = 1, rs2 = 3)
  issueImmediate(VpuOpcode.H_STORE_V, rd = 1, rs1 = 15, rs2 = 0)
  issueImmediate(VpuOpcode.H_PREFETCH_V, rd = 2, rs1 = 14, rs2 = 0)
  issueImmediate(VpuOpcode.C_FENCE, xd = true, roccRd = 5)

  // Both read descriptors must leave Core before any read data is returned;
  // this simultaneously keeps the first reduction blocked and checks that the
  // younger disjoint load was admitted behind the dependent scalar command.
  val loadATag = acceptReadDescriptor(vectorA,
    hostBase + loadAOffset * p.storageBytes)
  val loadCTag = acceptReadDescriptor(vectorC,
    hostBase + loadCOffset * p.storageBytes)
  val storeTag = acceptWriteDescriptor(storeSource,
    hostBase + storeOffset * p.storageBytes)
  assert(loadATag != loadCTag, "two live loads reused one hazard/DMA tag")
  // Reader and writer own independent TileLink source/command namespaces.
  // Store tags deliberately come from the transport allocator rather than the
  // RS/load tag table, so a numeric LD/ST tag match is legal.
  assert(storeTag < p.storeQueueEntries,
    "store descriptor tag escaped the transport allocator")

  // The independent store can fully retire while the first load remains
  // stalled. Its data value is irrelevant here; address/tag/length are checked.
  drainStore(storeSource, storeTag)
  sendReadVector(vectorA, loadATag, Seq.fill(p.vLen)(1.0f))
  sendReadVector(vectorC, loadCTag, Seq.fill(p.vLen)(4.0f))

  val firstFence = response(5)
  assert((firstFence & 7) == 0,
    s"first dispatch fence returned error status 0x${firstFence.toString(16)}")
  issue(VpuOpcode.C_READ, rd = 2, rs1 = VpuReadSelector.Fp,
    xd = true, roccRd = 6)
  val fp2 = response(6) & BigInt("ffffffff", 16)
  assert(fp2 == bits(32.0f),
    s"dependent FP result=0x${fp2.toString(16)} expected=0x${bits(32.0f).toString(16)}")

  // ----------------------------------------------------------------------
  // 2) Once a long vector EXP has moved into activeExecute, the physical EX
  // FIFO has p.execQueueEntries free slots. Fill every one back-to-back while
  // EXP remains active. This distinguishes FIFO capacity from the additional
  // active descriptor and catches aggregate outstanding-credit regressions.
  // ----------------------------------------------------------------------
  issueImmediate(VpuOpcode.V_EXP_V, rd = 4, rs1 = 0)
  var activeTimeout = 500
  while (peek(c.io.debugActiveOpcode) != VpuOpcode.V_EXP_V &&
      activeTimeout > 0) {
    step(1)
    activeTimeout -= 1
  }
  assert(activeTimeout > 0, "V_EXP never entered activeExecute")

  for (entry <- 0 until p.execQueueEntries) {
    assert(peek(c.io.debugActiveOpcode) == VpuOpcode.V_EXP_V,
      s"V_EXP retired before EX FIFO entry $entry was filled")
    issueImmediate(VpuOpcode.V_ADD_VF, rd = 4, rs1 = 4, rs2 = 0)
  }
  assert(peek(c.io.debugActiveOpcode) == VpuOpcode.V_EXP_V,
    "V_EXP was not active concurrently with a full physical EX FIFO")
  issueImmediate(VpuOpcode.C_FENCE, xd = true, roccRd = 7)
  val secondFence = response(7)
  assert((secondFence & 7) == 0,
    s"active-plus-full-EX fence returned error status 0x${secondFence.toString(16)}")
}

class VpuCommandDispatchSpec extends ChiselFlatSpec {
  behavior of "VpuCore command dispatch"

  it should "admit dependent EX and independent memory work without HOL stalls" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2,
      vSpadKB = 1, loadQueueEntries = 2, execQueueEntries = 4,
      storeQueueEntries = 2)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-command-dispatch"),
      () => new VpuCore(p)) { c =>
      new VpuCommandDispatchTester(c, p)
    } should be (true)
  }
}
