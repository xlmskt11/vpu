package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

class VpuCoreNonlinearTester(c: VpuCore, p: VpuParams) extends PeekPokeTester(c) {
  import VpuTestFloat._

  poke(c.io.command.valid, 0)
  poke(c.io.response.ready, 1)
  poke(c.io.dma.readDescriptor.ready, 1)
  poke(c.io.dma.readData.valid, 0)
  poke(c.io.dma.readData.bits.commandTag, 0)
  poke(c.io.dma.writeDescriptor.ready, 1)
  poke(c.io.dma.writeData.ready, 1)
  poke(c.io.dma.writeCompletion.valid, 0)
  poke(c.io.dma.writeCompletion.bits.commandTag, 0)
  step(3)

  def issue(opcode: Int, rd: Int = 0, rs1: Int = 0, rs2: Int = 0,
            funct1: Int = 0, payload: BigInt = 0, xd: Boolean = false,
            roccRd: Int = 0): Unit = {
    poke(c.io.command.valid, 1)
    poke(c.io.command.bits.microOp,
      BigInt(VpuEncoding.pack(opcode, rd, rs1, rs2, funct1 = funct1) & 0xffffffffL))
    poke(c.io.command.bits.payload, payload)
    poke(c.io.command.bits.rd, roccRd)
    poke(c.io.command.bits.xd, xd)
    while (peek(c.io.command.ready) == 0) { step(1) }
    step(1)
    poke(c.io.command.valid, 0)
  }

  val input = Seq(-2.0f, -1.0f, 0.0f, 2.0f)
  issue(VpuOpcode.C_SET_VL, payload = input.length)
  issue(VpuOpcode.C_WRITE_GP, rd = 0, payload = 0) // input bank
  issue(VpuOpcode.C_WRITE_H, rd = 0, payload = BigInt("100000", 16))
  issue(VpuOpcode.H_PREFETCH_V, rd = 0, rs1 = 0, rs2 = 0)

  var descriptorSeen = false
  var timeout = 40
  while (!descriptorSeen && timeout > 0) {
    descriptorSeen = peek(c.io.dma.readDescriptor.valid) == 1
    step(1)
    timeout -= 1
  }
  assert(descriptorSeen)
  val firstReadTag = peek(c.io.dma.readDescriptor.bits.commandTag)
  poke(c.io.dma.readData.valid, 1)
  val packedInput = input.zipWithIndex.map { case (x, i) => bits(x) << (32 * i) }.reduce(_ | _)
  poke(c.io.dma.readData.bits.data, packedInput)
  poke(c.io.dma.readData.bits.elementMask, 0xf)
  poke(c.io.dma.readData.bits.spadElement, 0)
  poke(c.io.dma.readData.bits.commandTag, firstReadTag)
  poke(c.io.dma.readData.bits.last, 1)
  poke(c.io.dma.readData.bits.error, 0)
  while (peek(c.io.dma.readData.ready) == 0) { step(1) }
  step(1)
  poke(c.io.dma.readData.valid, 0)

  val bank = p.elementsPerBank
  issue(VpuOpcode.C_WRITE_GP, rd = 1, payload = bank)       // -x
  issue(VpuOpcode.C_WRITE_GP, rd = 2, payload = 2 * bank)   // exp/+1
  issue(VpuOpcode.C_WRITE_GP, rd = 3, payload = 3 * bank)   // reciprocal
  issue(VpuOpcode.C_WRITE_GP, rd = 4, payload = 4 * bank)   // output
  issue(VpuOpcode.C_WRITE_GP, rd = 5, payload = 0)          // host offset
  issue(VpuOpcode.C_WRITE_GP, rd = 6, payload = 5 * bank)   // next ping/pong buffer
  issue(VpuOpcode.C_WRITE_FP, rd = 0, payload = bits(0.0f))
  issue(VpuOpcode.C_WRITE_FP, rd = 1, payload = bits(1.0f))

  // Keep a second prefetch descriptor pending while the independent current
  // buffer executes. This directly exercises software-managed double buffering.
  poke(c.io.dma.readDescriptor.ready, 0)
  issue(VpuOpcode.H_PREFETCH_V, rd = 6, rs1 = 5, rs2 = 0)
  issue(VpuOpcode.V_SUB_VF, rd = 1, rs1 = 0, rs2 = 0, funct1 = 1)
  issue(VpuOpcode.V_EXP_V, rd = 2, rs1 = 1)
  issue(VpuOpcode.V_ADD_VF, rd = 2, rs1 = 2, rs2 = 1)
  issue(VpuOpcode.V_RECI_V, rd = 3, rs1 = 2)
  issue(VpuOpcode.V_MUL_VV, rd = 4, rs1 = 0, rs2 = 3)
  issue(VpuOpcode.H_STORE_V, rd = 4, rs1 = 5, rs2 = 0)
  issue(VpuOpcode.C_FENCE, xd = true, roccRd = 7)

  // Let the pending next-buffer DMA run after execution has already begun.
  poke(c.io.dma.readDescriptor.ready, 1)
  descriptorSeen = false
  timeout = 40
  while (!descriptorSeen && timeout > 0) {
    descriptorSeen = peek(c.io.dma.readDescriptor.valid) == 1
    step(1)
    timeout -= 1
  }
  assert(descriptorSeen)
  val secondReadTag = peek(c.io.dma.readDescriptor.bits.commandTag)
  poke(c.io.dma.readData.valid, 1)
  poke(c.io.dma.readData.bits.data, packedInput)
  poke(c.io.dma.readData.bits.elementMask, 0xf)
  poke(c.io.dma.readData.bits.spadElement, 5 * bank)
  poke(c.io.dma.readData.bits.commandTag, secondReadTag)
  poke(c.io.dma.readData.bits.last, 1)
  poke(c.io.dma.readData.bits.error, 0)
  while (peek(c.io.dma.readData.ready) == 0) { step(1) }
  step(1)
  poke(c.io.dma.readData.valid, 0)

  var output: Option[BigInt] = None
  var completionPending = false
  var completionTag = BigInt(0)
  var fenceSeen = false
  timeout = 1000
  while ((!fenceSeen || output.isEmpty) && timeout > 0) {
    if (peek(c.io.dma.writeData.valid) == 1 && peek(c.io.dma.writeData.ready) == 1) {
      output = Some(peek(c.io.dma.writeData.bits.data))
      assert(peek(c.io.dma.writeData.bits.elementMask) == 0xf)
      assert(peek(c.io.dma.writeData.bits.last) == 1)
      completionTag = peek(c.io.dma.writeData.bits.commandTag)
      completionPending = true
    }
    poke(c.io.dma.writeCompletion.valid, completionPending)
    poke(c.io.dma.writeCompletion.bits.commandTag, completionTag)
    poke(c.io.dma.writeCompletion.bits.error, 0)
    val completionFire = completionPending &&
      peek(c.io.dma.writeCompletion.ready) == 1
    if (peek(c.io.response.valid) == 1) {
      assert(peek(c.io.response.bits.rd) == 7)
      assert((peek(c.io.response.bits.data) & 3) == 0)
      fenceSeen = true
    }
    step(1)
    if (completionFire) {
      completionPending = false
      poke(c.io.dma.writeCompletion.valid, 0)
    }
    timeout -= 1
  }
  assert(timeout > 0, "VPU nonlinear sequence timed out")

  val packed = output.get
  input.zipWithIndex.foreach { case (x, i) =>
    val actual = value((packed >> (32 * i)) & BigInt("ffffffff", 16)).toDouble
    val expected = x.toDouble / (1.0 + Math.exp(-x.toDouble))
    assert(Math.abs(actual - expected) <= 2.0e-4 * Math.max(1.0, Math.abs(expected)),
      s"SiLU($x)=$actual expected=$expected")
  }
  assert(peek(c.io.perfCounters(VpuPerfIndex.DmaReadBytes)) == 32)
  assert(peek(c.io.perfCounters(VpuPerfIndex.DmaWriteBytes)) == 16)
  assert(peek(c.io.perfCounters(VpuPerfIndex.DmaExecuteOverlapCycles)) > 0)
}

class VpuCoreSpec extends ChiselFlatSpec {
  behavior of "VpuCore"
  it should "execute a fine-grained SRAM-resident SiLU sequence" in {
    val p = VpuParams(vLen = 16, nLanes = 4, sfuLanes = 2)
    chisel3.iotesters.Driver.execute(Array("--backend-name", "verilator",
      "--target-dir", "test_run_dir/vpu-core-nonlinear"), () => new VpuCore(p)) {
      c => new VpuCoreNonlinearTester(c, p)
    } should be (true)
  }
}
