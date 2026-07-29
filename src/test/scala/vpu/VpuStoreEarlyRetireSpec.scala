package vpu

import chisel3._
import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

/** Proves that a store's architectural/RS lifetime ends at its final VSRAM
  * read acceptance, while its independent DMA transport lifetime continues
  * until TileLink's terminal completion is returned.
  */
class VpuStoreEarlyRetireTester(c: VpuCore, p: VpuParams)
    extends PeekPokeTester(c) {
  private val hostBase = BigInt("300000", 16)
  private val firstSpad = 0
  private val secondSpad = p.elementsPerBank
  private val firstHostOffset = 0
  private val secondHostOffset = p.vLen

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
                           rs2: Int = 0, payload: BigInt = 0,
                           xd: Boolean = false, roccRd: Int = 0): Unit = {
    clear(c.io.command.bits)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd, rs1, rs2) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, xd)
    poke(c.io.command.bits.status.dprv, 3)
    poke(c.io.command.bits.status.prv, 3)
    poke(c.io.command.valid, 1)
  }

  private def issue(opcode: Int, rd: Int = 0, rs1: Int = 0,
                    rs2: Int = 0, payload: BigInt = 0,
                    xd: Boolean = false, roccRd: Int = 0,
                    timeoutLimit: Int = 500): Int = {
    driveCommand(opcode, rd, rs1, rs2, payload, xd, roccRd)
    var waited = 0
    while (peek(c.io.command.ready) == 0 && waited < timeoutLimit) {
      assert(peek(c.io.dma.writeCompletion.valid) == 0,
        "test supplied a completion while waiting for command admission")
      step(1)
      waited += 1
    }
    assert(waited < timeoutLimit,
      s"command opcode=0x${opcode.toHexString} timed out")
    step(1)
    poke(c.io.command.valid, 0)
    waited
  }

  private def waitForWriteDescriptor(expectedSpad: Int,
                                     expectedVaddr: BigInt): BigInt = {
    var timeout = 500
    while (peek(c.io.dma.writeDescriptor.valid) == 0 && timeout > 0) {
      step(1)
      timeout -= 1
    }
    assert(timeout > 0, s"store descriptor spad=$expectedSpad timed out")
    assert(peek(c.io.dma.writeDescriptor.bits.spadElement) == expectedSpad)
    assert(peek(c.io.dma.writeDescriptor.bits.elementCount) == p.vLen)
    assert(peek(c.io.dma.writeDescriptor.bits.vaddr) == expectedVaddr)
    peek(c.io.dma.writeDescriptor.bits.commandTag)
  }

  private def drainStoreData(expectedSpad: Int, expectedTag: BigInt,
                             fenceMustRemainBlocked: Boolean): Unit = {
    val beats = p.vLen / p.dmaElementsPerBeat
    poke(c.io.dma.writeData.ready, 1)
    for (beat <- 0 until beats) {
      var timeout = 500
      while (peek(c.io.dma.writeData.valid) == 0 && timeout > 0) {
        if (fenceMustRemainBlocked) {
          assert(peek(c.io.response.valid) == 0,
            "C_FENCE responded before the store TileLink completion")
        }
        step(1)
        timeout -= 1
      }
      assert(timeout > 0, s"store payload beat $beat timed out")
      assert(peek(c.io.dma.writeData.bits.commandTag) == expectedTag)
      assert(peek(c.io.dma.writeData.bits.spadElement) ==
        expectedSpad + beat * p.dmaElementsPerBeat)
      assert(peek(c.io.dma.writeData.bits.elementMask) ==
        (BigInt(1) << p.dmaElementsPerBeat) - 1)
      assert(peek(c.io.dma.writeData.bits.last) ==
        (if (beat == beats - 1) 1 else 0))
      if (fenceMustRemainBlocked) {
        assert(peek(c.io.response.valid) == 0,
          "C_FENCE responded while store payload was still draining")
      }
      step(1)
    }
    poke(c.io.dma.writeData.ready, 0)
  }

  private def driveCompletion(tag: BigInt): Unit = {
    clear(c.io.dma.writeCompletion.bits)
    poke(c.io.dma.writeCompletion.bits.commandTag, tag)
    poke(c.io.dma.writeCompletion.bits.error, 0)
    poke(c.io.dma.writeCompletion.valid, 1)
  }

  // Architectural setup. Both vector bases are VLEN-aligned; using separate
  // banks ensures this test measures store lifetime rather than bank conflict.
  issue(VpuOpcode.C_SET_VL, payload = p.vLen)
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = hostBase)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = firstSpad)
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = firstHostOffset)
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = secondSpad)
  issue(VpuOpcode.C_WRITE_GP, rd = 3, payload = secondHostOffset)

  // Allocate the only store RS entry, but hold the writer descriptor so the
  // C_WAIT snapshot is guaranteed to contain this live, issued store.
  issue(VpuOpcode.H_STORE_V, rd = 0, rs1 = 1, rs2 = 0)
  val firstTag = waitForWriteDescriptor(firstSpad, hostBase)
  issue(VpuOpcode.C_WAIT, payload = 4) // ST mask

  // Start the local VSRAM stream, deliberately withholding TL completion.
  poke(c.io.dma.writeDescriptor.ready, 1)
  assert(peek(c.io.dma.writeDescriptor.valid) == 1)
  step(1)
  poke(c.io.dma.writeDescriptor.ready, 0)

  // A command behind C_WAIT can become ready only after the snapshotted store
  // RS entry completes. Keep writeData stalled: the only event capable of
  // releasing the wait is the final VSRAM read-request acceptance, not TL D.
  driveCommand(VpuOpcode.C_WRITE_GP, rd = 10, payload = 0x55)
  var waitReleaseCycles = 0
  while (peek(c.io.command.ready) == 0 && waitReleaseCycles < 100) {
    assert(peek(c.io.dma.writeCompletion.valid) == 0)
    assert(peek(c.io.response.valid) == 0)
    step(1)
    waitReleaseCycles += 1
  }
  assert(waitReleaseCycles > 0,
    "C_WAIT did not observe the live first-store reservation entry")
  assert(waitReleaseCycles < 100,
    "C_WAIT(ST) incorrectly waited for TileLink D completion")
  step(1) // C_WRITE_GP fires, proving the non-response WAIT was released
  poke(c.io.command.valid, 0)

  drainStoreData(firstSpad, firstTag, fenceMustRemainBlocked = false)

  // storeQueueEntries=1: accepting this command proves the first store's RS
  // entry was freed even though its independent DMA tag is still live.
  driveCommand(VpuOpcode.H_STORE_V, rd = 2, rs1 = 3, rs2 = 0)
  assert(peek(c.io.command.ready) == 1,
    "second store was not admitted after final first-store VSRAM read")
  step(1)
  poke(c.io.command.valid, 0)

  // Fence snapshots the second RS entry. It must additionally observe the
  // first store's still-live transport tag.
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 11)
  for (_ <- 0 until 12) {
    assert(peek(c.io.response.valid) == 0,
      "C_FENCE responded before the first store completion")
    assert(peek(c.io.dma.writeDescriptor.valid) == 0,
      "capacity-one transport allocator issued a duplicate live tag")
    step(1)
  }

  // A capacity-one tag can roll directly from the first D completion into the
  // already-issued second descriptor in the same cycle.
  driveCompletion(firstTag)
  poke(c.io.dma.writeDescriptor.ready, 1)
  assert(peek(c.io.dma.writeCompletion.ready) == 1)
  assert(peek(c.io.dma.writeDescriptor.valid) == 1,
    "second store descriptor did not reuse the completing transport tag")
  assert(peek(c.io.dma.writeDescriptor.bits.spadElement) == secondSpad)
  assert(peek(c.io.dma.writeDescriptor.bits.vaddr) ==
    hostBase + secondHostOffset * p.storageBytes)
  val secondTag = peek(c.io.dma.writeDescriptor.bits.commandTag)
  assert(secondTag == firstTag,
    "capacity-one transport allocator changed tag on direct recycle")
  assert(peek(c.io.response.valid) == 0)
  step(1)
  poke(c.io.dma.writeCompletion.valid, 0)
  clear(c.io.dma.writeCompletion.bits)
  poke(c.io.dma.writeDescriptor.ready, 0)

  drainStoreData(secondSpad, secondTag, fenceMustRemainBlocked = true)
  for (_ <- 0 until 12) {
    assert(peek(c.io.response.valid) == 0,
      "C_FENCE responded before the second store completion")
    step(1)
  }

  driveCompletion(secondTag)
  assert(peek(c.io.dma.writeCompletion.ready) == 1)
  assert(peek(c.io.response.valid) == 0)
  step(1)
  poke(c.io.dma.writeCompletion.valid, 0)
  clear(c.io.dma.writeCompletion.bits)

  var fenceTimeout = 200
  while (peek(c.io.response.valid) == 0 && fenceTimeout > 0) {
    step(1)
    fenceTimeout -= 1
  }
  assert(fenceTimeout > 0, "C_FENCE did not complete after both TL D acks")
  assert(peek(c.io.response.bits.rd) == 11)
  assert((peek(c.io.response.bits.data) & 3) == 0)
  poke(c.io.response.ready, 1)
  step(1)
  poke(c.io.response.ready, 0)
}

class VpuStoreEarlyRetireSpec extends ChiselFlatSpec {
  behavior of "VpuCore store retirement"

  it should "free the store RS at final VSRAM read but make FENCE wait for D" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2,
      storeQueueEntries = 1)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-store-early-retire"),
      () => new VpuCore(p)) { c =>
      new VpuStoreEarlyRetireTester(c, p)
    } should be (true)
  }
}
