package vpu

import chisel3._
import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

/** Core-level coverage for the architectural 2-D DMA controls.
  *
  * The TileLink request planners have their own row-splitting tests.  This
  * test instead starts at the RoCC-facing command port and checks that the
  * Core snapshots GP[rs3], C_SET_STRIDE, VL, and address registers into one
  * DMA descriptor.  A load/store round trip also observes the VLEN-sized gap
  * between local rows, including a partial final SRAM word in every row.
  */
class VpuStridedCoreTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  import VpuTestFloat._

  private val vl = 6
  private val rows = 3
  private val loadBase = 0
  private val legacyBase = p.elementsPerBank
  private val loadHostBase = BigInt("100000", 16)
  private val storeHostBase = BigInt("200000", 16)
  private val legacyHostBase = BigInt("300000", 16)
  private val loadOffsetElements = 3
  private val storeOffsetElements = 5
  private val legacyOffsetElements = 7
  private val loadStrideBytes = 64
  private val storeStrideBytes = 96

  require(p.dmaMaxRows >= rows)
  require(p.nLanes == p.dmaElementsPerBeat,
    "this focused test uses one DMA beat per SRAM lane word")

  private def leafBits(data: Data): Seq[Bits] = data match {
    case bits: Bits => Seq(bits)
    case aggregate: Aggregate => aggregate.getElements.flatMap(leafBits)
  }

  private def clear(data: Data): Unit =
    leafBits(data).foreach(bits => poke(bits, 0))

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
                           rs2: Int = 0, rs3: Int = 0, funct1: Int = 0,
                           payload: BigInt = 0, xd: Boolean = false,
                           roccRd: Int = 0): Unit = {
    clear(c.io.command.bits)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd, rs1, rs2, rs3,
        funct1) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, xd)
    poke(c.io.command.bits.status.dprv, 3)
    poke(c.io.command.bits.status.prv, 3)
    poke(c.io.command.valid, 1)
  }

  private def issue(opcode: Int, rd: Int = 0, rs1: Int = 0,
                    rs2: Int = 0, rs3: Int = 0, funct1: Int = 0,
                    payload: BigInt = 0, xd: Boolean = false,
                    roccRd: Int = 0): Unit = {
    driveCommand(opcode, rd, rs1, rs2, rs3, funct1, payload, xd, roccRd)
    var timeout = 500
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0,
      s"command opcode=0x${opcode.toHexString} admission timed out")
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
    assert(peek(c.io.response.bits.rd) == expectedRd)
    val result = peek(c.io.response.bits.data)
    poke(c.io.response.ready, 1)
    step(1)
    poke(c.io.response.ready, 0)
    result
  }

  private def fence(expectedIllegal: Boolean, roccRd: Int): Unit = {
    issue(VpuOpcode.C_FENCE, xd = true, roccRd = roccRd)
    val status = response(roccRd)
    assert((status & 1) == (if (expectedIllegal) 1 else 0),
      s"FENCE status=0x${status.toString(16)} expected illegal=$expectedIllegal")
  }

  private def acceptReadDescriptor(expectedVaddr: BigInt,
                                   expectedSpad: Int,
                                   expectedRows: Int,
                                   expectedStride: BigInt): BigInt = {
    var timeout = 500
    while (peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "read descriptor timed out")
    assert(peek(c.io.dma.readDescriptor.bits.vaddr) == expectedVaddr)
    assert(peek(c.io.dma.readDescriptor.bits.spadElement) == expectedSpad)
    assert(peek(c.io.dma.readDescriptor.bits.elementCount) == vl)
    assert(peek(c.io.dma.readDescriptor.bits.rowCount) == expectedRows)
    assert(peek(c.io.dma.readDescriptor.bits.hostStrideBytes) ==
      expectedStride)
    val tag = peek(c.io.dma.readDescriptor.bits.commandTag)
    poke(c.io.dma.readDescriptor.ready, 1)
    step(1)
    poke(c.io.dma.readDescriptor.ready, 0)
    tag
  }

  private def acceptWriteDescriptor(expectedVaddr: BigInt,
                                    expectedSpad: Int,
                                    expectedRows: Int,
                                    expectedStride: BigInt): BigInt = {
    var timeout = 500
    while (peek(c.io.dma.writeDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "write descriptor timed out")
    assert(peek(c.io.dma.writeDescriptor.bits.vaddr) == expectedVaddr)
    assert(peek(c.io.dma.writeDescriptor.bits.spadElement) == expectedSpad)
    assert(peek(c.io.dma.writeDescriptor.bits.elementCount) == vl)
    assert(peek(c.io.dma.writeDescriptor.bits.rowCount) == expectedRows)
    assert(peek(c.io.dma.writeDescriptor.bits.hostStrideBytes) ==
      expectedStride)
    val tag = peek(c.io.dma.writeDescriptor.bits.commandTag)
    poke(c.io.dma.writeDescriptor.ready, 1)
    step(1)
    poke(c.io.dma.writeDescriptor.ready, 0)
    tag
  }

  private def pack(values: Seq[Float]): BigInt = {
    require(values.length == p.dmaElementsPerBeat)
    values.zipWithIndex.map { case (value, lane) =>
      bits(value) << (lane * p.storageBits)
    }.reduce(_ | _)
  }

  private val rowValues = (0 until rows).map { row =>
    (0 until vl).map(element => (row * 16 + element + 1).toFloat)
  }

  private def sendLoadRows(spadBase: Int, commandTag: BigInt,
                           values: Seq[Seq[Float]]): Unit = {
    val beatsPerRow = (vl + p.dmaElementsPerBeat - 1) /
      p.dmaElementsPerBeat
    for (row <- values.indices; beat <- 0 until beatsPerRow) {
      val first = beat * p.dmaElementsPerBeat
      val useful = math.min(p.dmaElementsPerBeat, vl - first)
      val beatValues = values(row).slice(first, first + useful) ++
        Seq.fill(p.dmaElementsPerBeat - useful)(0.0f)
      clear(c.io.dma.readData.bits)
      poke(c.io.dma.readData.bits.data, pack(beatValues))
      poke(c.io.dma.readData.bits.elementMask,
        (BigInt(1) << useful) - 1)
      poke(c.io.dma.readData.bits.spadElement,
        spadBase + row * p.vLen + first)
      poke(c.io.dma.readData.bits.commandTag, commandTag)
      poke(c.io.dma.readData.bits.last,
        if (row == values.length - 1 && beat == beatsPerRow - 1) 1 else 0)
      poke(c.io.dma.readData.valid, 1)
      var timeout = 500
      while (peek(c.io.dma.readData.ready) == 0 && timeout > 0) {
        step(1)
        timeout -= 1
      }
      assert(timeout > 0, s"load row=$row beat=$beat timed out")
      step(1)
      poke(c.io.dma.readData.valid, 0)
    }
  }

  private def drainStore(commandTag: BigInt,
                         values: Seq[Seq[Float]]): Unit = {
    val beatsPerRow = (vl + p.dmaElementsPerBeat - 1) /
      p.dmaElementsPerBeat
    for (row <- values.indices; beat <- 0 until beatsPerRow) {
      var timeout = 1000
      while (peek(c.io.dma.writeData.valid) == 0 && timeout > 0) {
        step(1)
        timeout -= 1
      }
      assert(timeout > 0, s"store row=$row beat=$beat timed out")
      val first = beat * p.dmaElementsPerBeat
      val useful = math.min(p.dmaElementsPerBeat, vl - first)
      val expectedValues = values(row).slice(first, first + useful) ++
        Seq.fill(p.dmaElementsPerBeat - useful)(0.0f)
      val validDataBits = useful * p.storageBits
      val validDataMask = (BigInt(1) << validDataBits) - 1
      assert(peek(c.io.dma.writeData.bits.spadElement) ==
        loadBase + row * p.vLen + first,
        s"store row=$row beat=$beat did not preserve VLEN row spacing")
      assert(peek(c.io.dma.writeData.bits.elementMask) ==
        (BigInt(1) << useful) - 1)
      assert((peek(c.io.dma.writeData.bits.data) & validDataMask) ==
        (pack(expectedValues) & validDataMask))
      assert(peek(c.io.dma.writeData.bits.commandTag) == commandTag)
      assert(peek(c.io.dma.writeData.bits.last) ==
        (if (row == values.length - 1 && beat == beatsPerRow - 1) 1 else 0))
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

  private def assertNoReadDescriptor(cycles: Int): Unit = {
    poke(c.io.dma.readDescriptor.ready, 0)
    for (_ <- 0 until cycles) {
      assert(peek(c.io.dma.readDescriptor.valid) == 0,
        "invalid 2-D command emitted a read descriptor")
      step(1)
    }
  }

  // ----------------------------------------------------------------------
  // 1) A 2-D load snapshots row count and byte stride at command dispatch.
  // Hold its descriptor, then overwrite both architectural controls.  The
  // accepted descriptor and returned local addresses must retain old values.
  // ----------------------------------------------------------------------
  issue(VpuOpcode.C_SET_VL, payload = vl)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = loadHostBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = loadBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = loadOffsetElements)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = rows)
  issue(VpuOpcode.C_SET_STRIDE, payload = loadStrideBytes)
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 1, rs2 = 0, rs3 = 2,
    funct1 = 1)

  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = 1)
  issue(VpuOpcode.C_SET_STRIDE, payload = 128)
  val loadTag = acceptReadDescriptor(
    loadHostBase + loadOffsetElements * p.storageBytes,
    loadBase, rows, loadStrideBytes)
  sendLoadRows(loadBase, loadTag, rowValues)
  fence(expectedIllegal = false, roccRd = 1)

  // ----------------------------------------------------------------------
  // 2) Store-side snapshotting is independent of SRAM response/TL-D drain.
  // Reading the loaded rows back checks that local row r starts at
  // base+r*VLEN, not at a densely packed base+r*VL.
  // ----------------------------------------------------------------------
  issue(VpuOpcode.C_WRITE_H, rd = 1, payload = storeHostBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 3, payload = storeOffsetElements)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = rows)
  issue(VpuOpcode.C_SET_STRIDE, payload = storeStrideBytes)
  issue(VpuOpcode.H_STORE_V, rd = 0, rs1 = 3, rs2 = 1, rs3 = 2,
    funct1 = 1)

  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = 1)
  issue(VpuOpcode.C_SET_STRIDE, payload = 160)
  val storeTag = acceptWriteDescriptor(
    storeHostBase + storeOffsetElements * p.storageBytes,
    loadBase, rows, storeStrideBytes)
  drainStore(storeTag, rowValues)
  fence(expectedIllegal = false, roccRd = 2)

  // ----------------------------------------------------------------------
  // 3) Legacy funct1=0 is always exactly one dense row.  A deliberately
  // misaligned current stride and a nontrivial GP0 value must be ignored.
  // ----------------------------------------------------------------------
  issue(VpuOpcode.C_WRITE_H, rd = 2, payload = legacyHostBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = p.dmaMaxRows + 1)
  issue(VpuOpcode.C_WRITE_GP, rd = 3, payload = legacyBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 4, payload = legacyOffsetElements)
  issue(VpuOpcode.C_SET_STRIDE, payload = 66)
  issue(VpuOpcode.H_PREFETCH_V, rd = 3, rs1 = 4, rs2 = 2,
    funct1 = 0)
  val legacyTag = acceptReadDescriptor(
    legacyHostBase + legacyOffsetElements * p.storageBytes,
    legacyBase, expectedRows = 1, expectedStride = 0)
  sendLoadRows(legacyBase, legacyTag, rowValues.take(1))
  fence(expectedIllegal = false, roccRd = 3)

  // ----------------------------------------------------------------------
  // 4) Invalid 2-D row counts and an element-misaligned multi-row stride set
  // sticky illegal status and have no DMA side effects.
  // ----------------------------------------------------------------------
  val invalidBase = p.elementsPerBank * 2
  issue(VpuOpcode.C_WRITE_H, rd = 3, payload = BigInt("400000", 16))
  issue(VpuOpcode.C_WRITE_GP, rd = 5, payload = invalidBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 6, payload = 0)
  issue(VpuOpcode.C_WRITE_GP, rd = 7, payload = 0)
  issue(VpuOpcode.C_SET_STRIDE, payload = 64)
  issue(VpuOpcode.H_PREFETCH_V, rd = 5, rs1 = 6, rs2 = 3, rs3 = 7,
    funct1 = 1)
  assertNoReadDescriptor(4)
  fence(expectedIllegal = true, roccRd = 4)
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Errors)

  issue(VpuOpcode.C_WRITE_GP, rd = 7, payload = p.dmaMaxRows + 1)
  issue(VpuOpcode.H_PREFETCH_V, rd = 5, rs1 = 6, rs2 = 3, rs3 = 7,
    funct1 = 1)
  assertNoReadDescriptor(4)
  fence(expectedIllegal = true, roccRd = 5)
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Errors)

  issue(VpuOpcode.C_WRITE_GP, rd = 7, payload = 2)
  issue(VpuOpcode.C_SET_STRIDE, payload = 6)
  issue(VpuOpcode.H_PREFETCH_V, rd = 5, rs1 = 6, rs2 = 3, rs3 = 7,
    funct1 = 1)
  assertNoReadDescriptor(4)
  fence(expectedIllegal = true, roccRd = 6)
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Errors)
  fence(expectedIllegal = false, roccRd = 7)
}

class VpuStridedCoreSpec extends ChiselFlatSpec {
  behavior of "VpuCore 2-D strided DMA"
  it should "snapshot descriptors, space local rows by VLEN, preserve 1-D encoding, and reject invalid controls" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2,
      vSpadKB = 2)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-strided-core"),
      () => new VpuCore(p)) { c =>
      new VpuStridedCoreTester(c, p)
    } should be (true)
  }
}
