package vpu

import chisel3.iotesters.{ChiselFlatSpec, PeekPokeTester}

import scala.collection.mutable.ArrayBuffer

class VpuHardwareLoopTester(c: VpuHardwareLoop)
    extends PeekPokeTester(c) {
  private val LoopStart = 0x2f
  private val LoopEnd = 0x30

  case class Seen(word: BigInt, payload: BigInt, group: BigInt,
                  grouped: BigInt, last: BigInt, malformed: BigInt)

  private def startWord(reg: Int, count: Int): BigInt =
    (BigInt(count) << 10) | (BigInt(reg) << 6) | LoopStart

  private def endWord(reg: Int, upper: Int = 0): BigInt =
    (BigInt(upper) << 10) | (BigInt(reg) << 6) | LoopEnd

  private def drive(word: BigInt, payload: BigInt = 0, group: Int = 0,
                    grouped: Boolean = false, last: Boolean = false,
                    malformed: Boolean = false, xd: Boolean = false): Unit = {
    poke(c.io.in.valid, 1)
    poke(c.io.in.bits.command.microOp, word)
    poke(c.io.in.bits.command.payload, payload)
    poke(c.io.in.bits.command.rd, 0)
    poke(c.io.in.bits.command.xd, if (xd) 1 else 0)
    poke(c.io.in.bits.groupId.get, group)
    poke(c.io.in.bits.grouped.get, if (grouped) 1 else 0)
    poke(c.io.in.bits.last.get, if (last) 1 else 0)
    poke(c.io.in.bits.malformed, if (malformed) 1 else 0)
  }

  private def enqueue(word: BigInt, payload: BigInt = 0, group: Int = 0,
                      grouped: Boolean = false, last: Boolean = false,
                      malformed: Boolean = false, xd: Boolean = false): Unit = {
    drive(word, payload, group, grouped, last, malformed, xd)
    var waited = 0
    while (peek(c.io.in.ready) == 0 && waited < 200) {
      step(1)
      waited += 1
    }
    assert(peek(c.io.in.ready) == 1, "loop input remained backpressured")
    step(1)
    poke(c.io.in.valid, 0)
  }

  private def sample(): Seen = Seen(
    peek(c.io.out.bits.command.microOp),
    peek(c.io.out.bits.command.payload),
    peek(c.io.out.bits.groupId.get),
    peek(c.io.out.bits.grouped.get),
    peek(c.io.out.bits.last.get),
    peek(c.io.out.bits.malformed))

  private def drain(maxCycles: Int = 1000,
                    stall: Int => Boolean = _ => false): Seq[Seen] = {
    val seen = ArrayBuffer.empty[Seen]
    var cycle = 0
    while ((peek(c.io.busy) == 1 || peek(c.io.out.valid) == 1) &&
        cycle < maxCycles) {
      val ready = !stall(cycle)
      poke(c.io.out.ready, if (ready) 1 else 0)
      if (ready && peek(c.io.out.valid) == 1) {
        seen += sample()
      }
      step(1)
      cycle += 1
    }
    assert(cycle < maxCycles, "loop replay did not terminate")
    poke(c.io.out.ready, 1)
    seen.toSeq
  }

  private def expectMalformed(last: Boolean,
                              busyAfterFire: Boolean): Unit = {
    expect(c.io.protocolError, 1)
    expect(c.io.busy, 1)
    expect(c.io.out.valid, 1)
    expect(c.io.out.bits.malformed, 1)
    expect(c.io.out.bits.command.microOp, BigInt("8000000e", 16))
    expect(c.io.out.bits.last.get, if (last) 1 else 0)
    step(1)
    expect(c.io.busy, if (busyAfterFire) 1 else 0)
    expect(c.io.protocolError, 0)
  }

  private def expectTerminalMalformed(): Unit =
    expectMalformed(last = true, busyAfterFire = false)

  poke(c.io.in.valid, 0)
  poke(c.io.out.ready, 1)
  step(2)

  // Commands outside a loop retain a combinational pass-through path and
  // obey downstream backpressure.
  val passWord = BigInt(VpuEncoding.pack(VpuOpcode.V_MUL_VF, rd = 3))
  drive(passWord, payload = 0x55, group = 5, grouped = true, last = true)
  poke(c.io.out.ready, 0)
  expect(c.io.in.ready, 0)
  expect(c.io.out.valid, 1)
  expect(c.io.out.bits.command.microOp, passWord)
  expect(c.io.out.bits.command.payload, 0x55)
  step(2)
  poke(c.io.out.ready, 1)
  expect(c.io.in.ready, 1)
  step(1)
  poke(c.io.in.valid, 0)
  expect(c.io.busy, 0)

  // A three-iteration loop captures without output and replays a stable body
  // under backpressure. The group terminator must be peeled outside the loop.
  val simpleBody = BigInt(VpuEncoding.pack(VpuOpcode.V_ADD_VF, rd = 4,
    rs1 = 1, rs2 = 2))
  enqueue(startWord(reg = 6, count = 3), group = 2, grouped = true)
  expect(c.io.busy, 1)
  expect(c.io.out.valid, 0)
  enqueue(simpleBody, payload = 0x1234, group = 2, grouped = true)
  enqueue(endWord(reg = 6), group = 2, grouped = true)
  val simple = drain(stall = cycle => cycle == 1 || cycle == 2)
  assert(simple.length == 3, s"expected 3 body commands, saw $simple")
  simple.foreach { command =>
    assert(command.word == simpleBody)
    assert(command.payload == 0x1234)
    assert(command.group == 2 && command.grouped == 1)
    assert(command.malformed == 0)
  }
  assert(simple.forall(_.last == 0),
    s"captured loop unexpectedly invented group_last: $simple")

  drive(simpleBody, group = 2, grouped = true, last = true)
  expect(c.io.out.valid, 1)
  expect(c.io.out.bits.last.get, 1)
  step(1)
  poke(c.io.in.valid, 0)

  // count=1 executes the body once without changing its transport metadata.
  enqueue(startWord(reg = 6, count = 1), group = 2, grouped = true)
  enqueue(simpleBody, group = 2, grouped = true)
  enqueue(endWord(reg = 6), group = 2, grouped = true)
  val once = drain()
  assert(once.length == 1 && once.head.word == simpleBody &&
    once.head.last == 0, s"count-one loop replay differs: $once")

  // Nested 2x3 loops execute in program order.
  val outerBody = BigInt(VpuEncoding.pack(VpuOpcode.V_SUB_VF, rd = 1))
  val innerBody = BigInt(VpuEncoding.pack(VpuOpcode.V_EXP_V, rd = 2))
  enqueue(startWord(reg = 1, count = 2), group = 4, grouped = true)
  enqueue(outerBody, payload = 0xa, group = 4, grouped = true)
  enqueue(startWord(reg = 2, count = 3), group = 4, grouped = true)
  enqueue(innerBody, payload = 0xb, group = 4, grouped = true)
  enqueue(endWord(reg = 2), group = 4, grouped = true)
  enqueue(endWord(reg = 1), group = 4, grouped = true)
  val nested = drain()
  val expectedWords = Seq(outerBody, innerBody, innerBody, innerBody,
    outerBody, innerBody, innerBody, innerBody)
  assert(nested.map(_.word) == expectedWords,
    s"nested replay order differs: $nested")
  assert(nested.forall(_.last == 0),
    s"nested loop unexpectedly invented group_last: $nested")

  // Attention-shaped nested replay: one outer iteration represents a query
  // row and the inner loop represents four score/output tiles.  Eight static
  // RoCC commands expand to 64 * (1 + 4 * 2 + 1) = 640 dynamic VPU commands.
  // This is deliberately much larger than the small 2x3 functional example
  // above, and proves that loop counts -- rather than command-buffer capacity
  // -- set the dynamic schedule length.  Periodic output stalls also verify
  // that neither loop counter advances without a real downstream fire.
  val rowAddressStep = BigInt(VpuEncoding.packAddiInt(rd = 1, rs1 = 1,
    immediate = 128))
  val scoreOperation = BigInt(VpuEncoding.pack(VpuOpcode.V_MUL_VF, rd = 2,
    rs1 = 2, rs2 = 0))
  val reductionOperation = BigInt(VpuEncoding.pack(VpuOpcode.V_RED_MAX,
    rd = 1, rs1 = 2))
  val stateAddressStep = BigInt(VpuEncoding.packAddiInt(rd = 3, rs1 = 3,
    immediate = 1))
  val attentionRows = 64
  val tilesPerRow = 4

  enqueue(startWord(reg = 1, count = attentionRows), group = 3,
    grouped = true)
  enqueue(rowAddressStep, payload = 0x10, group = 3, grouped = true)
  enqueue(startWord(reg = 2, count = tilesPerRow), group = 3,
    grouped = true)
  enqueue(scoreOperation, payload = 0x20, group = 3, grouped = true)
  enqueue(reductionOperation, payload = 0x30, group = 3, grouped = true)
  enqueue(endWord(reg = 2), group = 3, grouped = true)
  enqueue(stateAddressStep, payload = 0x40, group = 3, grouped = true)
  enqueue(endWord(reg = 1), group = 3, grouped = true)

  val attentionReplay = drain(maxCycles = 5000,
    stall = cycle => cycle % 37 == 11 || cycle % 37 == 12)
  val expectedAttentionRow = Seq(rowAddressStep) ++
    Seq.fill(tilesPerRow)(Seq(scoreOperation, reductionOperation)).flatten ++
    Seq(stateAddressStep)
  val expectedAttention = Seq.fill(attentionRows)(expectedAttentionRow).flatten
  assert(attentionReplay.map(_.word) == expectedAttention,
    s"attention-shaped nested replay order differs")
  assert(attentionReplay.length == 640,
    s"eight static commands should expand to 640 dynamic commands, saw " +
      attentionReplay.length)
  assert(attentionReplay.forall(command =>
    command.group == 3 && command.grouped == 1 && command.last == 0 &&
      command.malformed == 0),
    s"attention-shaped replay changed grouped transport metadata")

  // An unmatched END is converted to one malformed architectural command.
  enqueue(endWord(reg = 7), group = 1, grouped = true)
  // It must not invent group_last: a real following terminator still belongs
  // to the aborted sequence and is what lets VpuGroupCommandGate drain it.
  expectMalformed(last = false, busyAfterFire = false)
  drive(simpleBody, group = 1, grouped = true, last = true)
  expect(c.io.out.valid, 1)
  expect(c.io.out.bits.last.get, 1)
  step(1)
  poke(c.io.in.valid, 0)

  // If the unmatched END itself is the explicit group terminator, retain that
  // fact on the malformed command. Clearing it would make the downstream group
  // gate enter abort-drain after software has no further terminator to send.
  enqueue(endWord(reg = 7), group = 1, grouped = true, last = true)
  expectTerminalMalformed()

  // A zero trip count is illegal rather than an accidental infinite loop.
  enqueue(startWord(reg = 0, count = 0))
  expectMalformed(last = false, busyAfterFire = true)
  enqueue(endWord(reg = 0))
  expect(c.io.busy, 0)
  expect(c.io.out.valid, 0)
  expect(c.io.protocolError, 1)
  step(1)

  // Capturing an xd command would make Rocket wait before it can deliver the
  // remainder of the region, so response-producing loop commands are illegal.
  // A malformed closing xd END must nevertheless receive its own response;
  // it cannot take the standalone no-terminator shortcut and disappear.
  enqueue(startWord(reg = 0, count = 2), xd = true)
  expect(c.io.out.bits.command.xd, 1)
  expectMalformed(last = false, busyAfterFire = true)
  enqueue(endWord(reg = 0), xd = true)
  expect(c.io.out.bits.command.xd, 1)
  expectMalformed(last = false, busyAfterFire = false)

  // A malformed grouped START opens abort-drain, consumes its complete static
  // region locally, and emits exactly one synthetic grouped terminator at END.
  enqueue(startWord(reg = 5, count = 0), group = 5, grouped = true)
  expect(c.io.out.bits.groupId.get, 5)
  expectMalformed(last = false, busyAfterFire = true)
  enqueue(simpleBody, group = 5, grouped = true)
  expect(c.io.out.valid, 0)
  enqueue(endWord(reg = 5), group = 5, grouped = true)
  expect(c.io.out.bits.groupId.get, 5)
  expectTerminalMalformed()

  // END must close the top frame, not merely any matching frame below it.
  enqueue(startWord(reg = 1, count = 2), group = 3, grouped = true)
  enqueue(startWord(reg = 2, count = 2), group = 3, grouped = true)
  enqueue(endWord(reg = 1), group = 3, grouped = true)
  expect(c.io.out.valid, 0)
  enqueue(endWord(reg = 1), group = 3, grouped = true)
  expect(c.io.out.bits.groupId.get, 3)
  expect(c.io.out.bits.grouped.get, 1)
  expectTerminalMalformed()

  // The configured two-frame stack rejects a third nested START.
  enqueue(startWord(reg = 1, count = 2))
  enqueue(startWord(reg = 2, count = 2))
  enqueue(startWord(reg = 3, count = 2))
  enqueue(endWord(reg = 3))
  enqueue(endWord(reg = 2))
  enqueue(endWord(reg = 1))
  expectTerminalMalformed()

  // No command captured in a loop may own group_last. Accepting it during
  // capture would let a later Gemmini route overtake future replay entries.
  val addi = BigInt(VpuEncoding.packAddiInt(rd = 1, rs1 = 1,
    immediate = 1))
  enqueue(startWord(reg = 1, count = 2), group = 6, grouped = true)
  enqueue(addi, group = 6, grouped = true, last = true)
  expect(c.io.out.valid, 0)
  enqueue(endWord(reg = 1), group = 6, grouped = true)
  expectTerminalMalformed()

  // This applies to reservation-capable data commands as well.
  enqueue(startWord(reg = 1, count = 2), group = 6, grouped = true)
  enqueue(simpleBody, group = 6, grouped = true, last = true)
  enqueue(endWord(reg = 1), group = 6, grouped = true)
  expectTerminalMalformed()

  // A metadata mismatch is reported against the outer region's group so the
  // downstream group gate can abort the group which actually owns the loop.
  enqueue(startWord(reg = 1, count = 2), group = 2, grouped = true)
  enqueue(simpleBody, group = 5, grouped = true)
  enqueue(endWord(reg = 1), group = 2, grouped = true)
  expect(c.io.out.bits.groupId.get, 2)
  expect(c.io.out.bits.grouped.get, 1)
  expectTerminalMalformed()

  // Fill all eight command slots, then prove that the required closing END is
  // rejected cleanly rather than wrapping and corrupting the command buffer.
  enqueue(startWord(reg = 1, count = 2))
  for (i <- 0 until 7) {
    enqueue(simpleBody, payload = i)
  }
  enqueue(endWord(reg = 1))
  expectTerminalMalformed()
}

class VpuHardwareLoopSpec extends ChiselFlatSpec {
  behavior of "VpuHardwareLoop"

  it should "capture, replay and validate nested PLENA-style loops" in {
    chisel3.iotesters.Driver.execute(Array(
      "--backend-name", "treadle",
      "--target-dir", "test_run_dir/vpu-hardware-loop"),
      () => new VpuHardwareLoop(groupIdBits = 3,
        enableGroupedCommands = true,
        bufferEntries = 8, maxDepth = 2)) { c =>
      new VpuHardwareLoopTester(c)
    } should be(true)
  }
}
