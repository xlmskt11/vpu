package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

/** Focused architectural regressions for status, validation, SRAM boundary,
  * and DMA-fault recovery. These cases intentionally share one elaboration so
  * the full VPU is compiled by Verilator only once.
  */
class VpuCoreProtocolTester(c: VpuCore, p: VpuParams) extends PeekPokeTester(c) {
  import VpuTestFloat._

  poke(c.io.command.valid, 0)
  poke(c.io.response.ready, 0)
  poke(c.io.dma.readDescriptor.ready, 1)
  poke(c.io.dma.readData.valid, 0)
  poke(c.io.dma.readData.bits.commandTag, 0)
  poke(c.io.dma.writeDescriptor.ready, 1)
  poke(c.io.dma.writeData.ready, 1)
  poke(c.io.dma.writeCompletion.valid, 0)
  poke(c.io.dma.writeCompletion.bits.commandTag, 0)
  poke(c.io.dma.writeCompletion.bits.error, 0)
  poke(c.io.dma.writeCompletion.bits.fault.vaddr, 0)
  poke(c.io.dma.writeCompletion.bits.fault.cause, VpuDmaFaultCause.None)
  poke(c.io.dma.writeCompletion.bits.fault.isWrite, 0)
  poke(c.io.dma.clearFaultDone, 0)
  poke(c.io.dma.halted, 0)
  step(3)

  var currentReadCommandTag = BigInt(0)

  def issue(opcode: Int, rd: Int = 0, rs1: Int = 0, rs2: Int = 0,
            rs3: Int = 0, funct1: Int = 0, payload: BigInt = 0, xd: Boolean = false,
            roccRd: Int = 0, statusDprv: Int = 3,
            statusDv: Boolean = false): Unit = {
    poke(c.io.command.valid, 1)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd, rs1, rs2, rs3 = rs3,
        funct1 = funct1) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, xd)
    poke(c.io.command.bits.status.dprv, statusDprv)
    poke(c.io.command.bits.status.dv, statusDv)
    poke(c.io.command.bits.status.prv, statusDprv)
    poke(c.io.command.bits.status.v, statusDv)
    var timeout = 200
    while (peek(c.io.command.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"command opcode=0x${opcode.toHexString} enqueue timed out")
    step(1)
    poke(c.io.command.valid, 0)
  }

  def response(expectedRd: Int, forbidNewDma: Boolean = false): BigInt = {
    var timeout = 1000
    while (peek(c.io.response.valid) == 0 && timeout > 0) {
      if (forbidNewDma) {
        assert(peek(c.io.dma.readDescriptor.valid) == 0,
          "a new DMA read was issued after a sticky fault")
        assert(peek(c.io.dma.writeDescriptor.valid) == 0,
          "a new DMA write was issued after a sticky fault")
      }
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"response rd=$expectedRd timed out")
    assert(peek(c.io.response.bits.rd) == expectedRd)
    val data = peek(c.io.response.bits.data)
    poke(c.io.response.ready, 1)
    step(1)
    poke(c.io.response.ready, 0)
    data
  }

  def waitReadDescriptor(expectedBase: Int, expectedCount: Int,
                         expectedDprv: Option[Int] = None,
                         expectedDv: Option[Boolean] = None,
                         expectedRows: Option[Int] = None,
                         expectedStride: Option[BigInt] = None): Unit = {
    var timeout = 200
    while (peek(c.io.dma.readDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "DMA read descriptor timed out")
    assert(peek(c.io.dma.readDescriptor.bits.spadElement) == expectedBase)
    assert(peek(c.io.dma.readDescriptor.bits.elementCount) == expectedCount)
    expectedRows.foreach { value =>
      assert(peek(c.io.dma.readDescriptor.bits.rowCount) == value,
        "DMA descriptor did not snapshot its 2-D row count")
    }
    expectedStride.foreach { value =>
      assert(peek(c.io.dma.readDescriptor.bits.hostStrideBytes) == value,
        "DMA descriptor did not snapshot its host row stride")
    }
    currentReadCommandTag = peek(c.io.dma.readDescriptor.bits.commandTag)
    expectedDprv.foreach { value =>
      assert(peek(c.io.dma.readDescriptor.bits.status.dprv) == value,
        "DMA descriptor did not retain its command's effective privilege")
    }
    expectedDv.foreach { value =>
      assert(peek(c.io.dma.readDescriptor.bits.status.dv) ==
        (if (value) 1 else 0),
        "DMA descriptor did not retain its command's virtualization context")
    }
    poke(c.io.dma.readDescriptor.ready, 1)
    step(1)
  }

  def readBeat(spadElement: Int, last: Boolean, error: Boolean,
               mask: Int = 0xf, data: BigInt = 0,
               faultVaddr: BigInt = 0,
               faultCause: Int = VpuDmaFaultCause.None): Unit = {
    poke(c.io.dma.readData.valid, 1)
    poke(c.io.dma.readData.bits.spadElement, spadElement)
    poke(c.io.dma.readData.bits.commandTag, currentReadCommandTag)
    poke(c.io.dma.readData.bits.elementMask, mask)
    poke(c.io.dma.readData.bits.data, data)
    poke(c.io.dma.readData.bits.last, last)
    poke(c.io.dma.readData.bits.error, error)
    poke(c.io.dma.readData.bits.fault.vaddr, faultVaddr)
    poke(c.io.dma.readData.bits.fault.cause, faultCause)
    poke(c.io.dma.readData.bits.fault.isWrite, 0)
    var timeout = 200
    while (peek(c.io.dma.readData.ready) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, "DMA read beat was not drained")
    step(1)
    poke(c.io.dma.readData.valid, 0)
  }

  def readStatus(roccRd: Int): BigInt = {
    issue(VpuOpcode.C_READ, rd = 0, rs1 = VpuReadSelector.Status,
      xd = true, roccRd = roccRd)
    response(roccRd)
  }

  // An otherwise-idle status read must not count itself as outstanding work.
  val initialStatus = readStatus(1)
  assert(((initialStatus >> VpuStatusLayout.Busy) & 1) == 0,
    "idle C_READ status reported itself busy")

  // S_MUL is binary just like ADD/SUB/MAX. rs2=8 must be rejected rather than
  // aliasing FP register zero through rs2(2,0).
  issue(VpuOpcode.C_WRITE_FP, rd = 0, payload = bits(2.0f))
  issue(VpuOpcode.C_WRITE_FP, rd = 1, payload = bits(3.0f))
  issue(VpuOpcode.S_MUL_FP, rd = 0, rs1 = 0, rs2 = 8)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 2)
  val illegalFence = response(2)
  assert((illegalFence & 1) == 1, "out-of-range S_MUL rs2 was not illegal")
  issue(VpuOpcode.C_READ, rd = 0, rs1 = VpuReadSelector.Fp,
    xd = true, roccRd = 3)
  assert((response(3) & BigInt("ffffffff", 16)) == bits(2.0f),
    "illegal S_MUL modified aliased FP0")
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Fflags)
  assert((readStatus(4) & 1) == 1,
    "FFLAGS-only clear incorrectly cleared illegal status")
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.FaultIllegal)
  assert((readStatus(4) & 3) == 0, "sticky illegal status did not clear")

  // Every unused microinstruction field is architecturally zero. Confirm
  // scalar rs3/funct1, unary rs2, control rs1, and WAIT high payload bits are
  // rejected without modifying their apparent destinations or activating a
  // malformed barrier.
  issue(VpuOpcode.C_WRITE_GP, rd = 4, payload = 0x55)
  issue(VpuOpcode.S_ADD_FP, rd = 0, rs1 = 0, rs2 = 1, rs3 = 1)
  issue(VpuOpcode.S_MUL_FP, rd = 0, rs1 = 0, rs2 = 1, funct1 = 1)
  issue(VpuOpcode.S_EXP_FP, rd = 0, rs1 = 1, rs2 = 1)
  issue(VpuOpcode.C_WRITE_GP, rd = 4, rs1 = 1, payload = 0xaa)
  issue(VpuOpcode.C_WAIT, payload = 8)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 13)
  assert((response(13) & 1) == 1, "nonzero unused field was not illegal")
  issue(VpuOpcode.C_READ, rd = 0, rs1 = VpuReadSelector.Fp,
    xd = true, roccRd = 14)
  assert((response(14) & BigInt("ffffffff", 16)) == bits(2.0f),
    "malformed scalar command modified FP0")
  issue(VpuOpcode.C_READ, rd = 4, rs1 = VpuReadSelector.Gp,
    xd = true, roccRd = 15)
  assert(response(15) == 0x55, "malformed control command modified GP4")
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Errors)
  assert((readStatus(16) & 1) == 0)
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Perf)
  assert((readStatus(17) & 1) == 0,
    "PERF-only clear incorrectly modified illegal status")
  issue(VpuOpcode.C_CLEAR_STATUS, payload = 8)
  assert((readStatus(17) & 1) == 1,
    "undefined C_CLEAR_STATUS payload bits were silently accepted")

  // Fault/illegal and numerical flags are independently clearable. Generate
  // DZ while the illegal bit from the malformed clear remains sticky.
  issue(VpuOpcode.C_WRITE_FP, rd = 0, payload = bits(0.0f))
  issue(VpuOpcode.S_RECI_FP, rd = 2, rs1 = 0)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 18)
  response(18)
  val statusWithFlags = readStatus(19)
  val fflagsWithDz = (statusWithFlags >> VpuStatusLayout.FflagsLo) & 0x1f
  assert((statusWithFlags & 1) == 1 && fflagsWithDz != 0,
    "test setup did not create independent illegal and fflags state")
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.FaultIllegal)
  val statusAfterFaultClear = readStatus(20)
  assert((statusAfterFaultClear & 1) == 0)
  assert(((statusAfterFaultClear >> VpuStatusLayout.FflagsLo) & 0x1f) ==
    fflagsWithDz, "fault/illegal clear modified fflags")
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Fflags)
  assert(((readStatus(21) >> VpuStatusLayout.FflagsLo) & 0x1f) == 0,
    "FFLAGS clear did not clear numerical flags")

  // Legal final VSRAM row: the old elementAddrBits-wide base+VL check wrapped
  // to zero here. Four complete beats must now commit without an assertion.
  val finalBase = p.totalElements - p.vLen
  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = finalBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = 0)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = BigInt("100000", 16))
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 1, rs2 = 0)
  waitReadDescriptor(finalBase, p.vLen)
  for (beat <- 0 until p.vLen / p.dmaElementsPerBeat) {
    readBeat(finalBase + beat * p.dmaElementsPerBeat,
      last = beat == p.vLen / p.dmaElementsPerBeat - 1, error = false,
      data = BigInt("3f800000", 16))
  }
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 5)
  assert((response(5) & 3) == 0, "legal final-row prefetch faulted")

  // Cache-line alignment is not required, but storage-element alignment is.
  poke(c.io.dma.readDescriptor.ready, 0)
  issue(VpuOpcode.C_SET_VL, payload = 0)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = BigInt("100002", 16))
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 1, rs2 = 0)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 10)
  assert((response(10, forbidNewDma = true) & 3) == 0,
    "VL=0 memory no-op incorrectly validated its external address")

  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 1, rs2 = 0)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 9)
  val misalignedFence = response(9, forbidNewDma = true)
  assert((misalignedFence & 1) == 1,
    "element-misaligned external address was not rejected")
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Errors)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = BigInt("100000", 16))
  poke(c.io.dma.readDescriptor.ready, 1)

  // Two-dimensional loads may deliberately overlap/broadcast host rows, but
  // stores may not: independent TL source IDs do not order overlapping puts.
  // Both policies are checked before a descriptor can leave Core.
  val shortVl = p.dmaElementsPerBeat
  val shortRowBytes = shortVl * p.storageBytes
  val overlappingStride = shortRowBytes - p.storageBytes
  issue(VpuOpcode.C_SET_VL, payload = shortVl)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = 0)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = 0)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = 2)
  issue(VpuOpcode.C_SET_STRIDE, payload = overlappingStride)
  poke(c.io.dma.writeDescriptor.ready, 0)
  issue(VpuOpcode.H_STORE_V, rd = 0, rs1 = 1, rs2 = 0,
    rs3 = 2, funct1 = 1)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 22)
  assert((response(22, forbidNewDma = true) & 1) == 1,
    "overlapping 2-D store rows were not rejected")
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Errors)
  poke(c.io.dma.writeDescriptor.ready, 1)

  poke(c.io.dma.readDescriptor.ready, 0)
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 1, rs2 = 0,
    rs3 = 2, funct1 = 1)
  waitReadDescriptor(0, shortVl, expectedRows = Some(2),
    expectedStride = Some(overlappingStride))
  readBeat(0, last = false, error = false,
    data = BigInt("3f800000", 16))
  readBeat(p.vLen, last = true, error = false,
    data = BigInt("40000000", 16))
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 23)
  assert((response(23) & 3) == 0,
    "legal overlapping 2-D load did not drain")

  // The exclusive final byte may equal 2^64, but it must never exceed it.
  // This catches both a wrapped last row and the simpler one-row end wrap.
  poke(c.io.dma.readDescriptor.ready, 0)
  issue(VpuOpcode.C_WRITE_H, rd = 0,
    payload = (BigInt(1) << 64) - 2 * p.storageBytes)
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 1, rs2 = 0)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 24)
  assert((response(24, forbidNewDma = true) & 1) == 1,
    "64-bit external-address end wrap was not rejected")
  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Errors)

  // Restore the common geometry used by the queued fault/drain test below.
  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = BigInt("100000", 16))
  poke(c.io.dma.readDescriptor.ready, 1)

  // A dependent vector op keeps younger vector/memory work behind the active
  // load. Once the error arrives, the core must suppress writes, drain to
  // `last`, discard that queued chain, issue no new DMA, and answer FENCE.
  // These control-register writes precede the queued descriptors. Execute
  // entries snapshot FP register indices at enqueue and read their values at
  // FIFO-head issue; an accepted architectural write is not rolled back by a
  // later DMA fault.
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = 0)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = p.elementsPerBank)
  issue(VpuOpcode.C_WRITE_FP, rd = 3, payload = bits(7.0f))
  // Hold the descriptor while younger commands are enqueued, so the tester
  // cannot miss a legal one-cycle ready/valid transfer.
  poke(c.io.dma.readDescriptor.ready, 0)
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 1, rs2 = 0,
    statusDprv = 1, statusDv = true)
  issue(VpuOpcode.V_ADD_VF, rd = 2, rs1 = 0, rs2 = 0)
  issue(VpuOpcode.C_WRITE_FP, rd = 3, payload = bits(9.0f))
  issue(VpuOpcode.H_STORE_V, rd = 2, rs1 = 1, rs2 = 0)
  issue(VpuOpcode.H_PREFETCH_V, rd = 2, rs1 = 1, rs2 = 0)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 6)
  waitReadDescriptor(0, p.vLen, expectedDprv = Some(1),
    expectedDv = Some(true))

  // Hold descriptor ports closed after the already-issued read so a forbidden
  // younger transaction cannot escape as a one-cycle pulse.
  poke(c.io.dma.readDescriptor.ready, 0)
  poke(c.io.dma.writeDescriptor.ready, 0)
  val failingVaddr = BigInt("100000", 16)
  // Model the wrapper-side sticky admission gate asserted with the error.
  poke(c.io.dma.halted, 1)
  readBeat(0, last = false, error = true, faultVaddr = failingVaddr,
    faultCause = VpuDmaFaultCause.Access)
  assert(((peek(c.io.status) >> VpuStatusLayout.DmaFault) & 1) == 1,
    "DMA read error did not latch")
  // A non-error trailing beat is drained but must not repair/commit the partial
  // transfer after the earlier failing beat.
  readBeat(p.dmaElementsPerBeat, last = true, error = false,
    data = BigInt("3f800000", 16))
  val faultFence = response(6, forbidNewDma = true)
  assert(((faultFence >> VpuStatusLayout.DmaFault) & 1) == 1,
    "FENCE did not report the sticky DMA fault")
  assert(((faultFence >> VpuStatusLayout.Busy) & 1) == 0,
    "drained fault FENCE reported busy")

  issue(VpuOpcode.C_READ, rs1 = VpuReadSelector.FaultAddress,
    xd = true, roccRd = 11)
  assert(response(11) == failingVaddr, "first failing virtual address was lost")
  issue(VpuOpcode.C_READ, rs1 = VpuReadSelector.FaultInfo,
    xd = true, roccRd = 12)
  val faultInfo = response(12)
  assert(((faultInfo >> VpuFaultInfoLayout.Valid) & 1) == 1)
  assert(((faultInfo >> VpuFaultInfoLayout.IsWrite) & 1) == 0)
  assert((faultInfo & 3) == VpuDmaFaultCause.Access)

  issue(VpuOpcode.C_READ, rd = 3, rs1 = VpuReadSelector.Fp,
    xd = true, roccRd = 7)
  assert((response(7) & BigInt("ffffffff", 16)) == bits(9.0f),
    "accepted control-register write did not retain immediate semantics")

  issue(VpuOpcode.C_CLEAR_STATUS, payload = VpuClearMask.Errors)
  var clearTimeout = 100
  while (peek(c.io.dma.clearFault) == 0 && clearTimeout > 0) {
    step(1)
    clearTimeout -= 1
  }
  assert(clearTimeout > 0, "core never requested wrapper/TLB fault clear")
  poke(c.io.dma.clearFaultDone, 1)
  step(1)
  poke(c.io.dma.clearFaultDone, 0)
  // The wrapper is permitted to keep halted high through the acknowledge
  // cycle and resume one cycle later.
  step(1)
  poke(c.io.dma.halted, 0)
  step(1)
  val recoveredStatus = readStatus(8)
  assert((recoveredStatus & 7) == 0, "DMA fault did not clear after drain")
}

class VpuCoreProtocolSpec extends ChiselFlatSpec {
  behavior of "VpuCore protocol and recovery"
  it should "validate registers, report idle, preserve final-row bounds, and drain faults" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2, vSpadKB = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-core-protocol"), () => new VpuCore(p)) {
      c => new VpuCoreProtocolTester(c, p)
    } should be (true)
  }
}
