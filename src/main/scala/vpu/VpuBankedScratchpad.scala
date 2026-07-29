package vpu

import chisel3._
import chisel3.util._

object VpuBankedScratchpad {
  final val NumReadClients = 2
  final val NumWriteClients = 2

  final val ExecuteReadClient = 0
  final val StoreReadClient = 1
  final val ExecuteWriteClient = 0
  final val LoadWriteClient = 1
}

/** Bank-local 1R1W Vector SRAM with independent EX/ST read and EX/LD write
  * clients.
  *
  * Each bank receives at most one read and one masked write in a cycle. Two
  * client requests are accepted together whenever their bank sets are
  * disjoint. An overlapping pair is selected by a round-robin arbiter. A VV
  * request for one address performs one SRAM read and broadcasts the result;
  * two distinct words in one bank reserve that bank for a second read cycle.
  *
  * Every read client has its own credit-protected response queue. Queue credit
  * is reserved when a request is accepted, so SyncReadMem results cannot be
  * lost or overwritten under arbitrary, independent response backpressure.
  * Keeping arbitration in this module lets VpuCore sequence each engine
  * independently without coupling it to SRAM port timing.
  */
class VpuBankedScratchpad(p: VpuParams) extends Module {
  import VpuBankedScratchpad._

  val io = IO(new Bundle {
    val readRequest = Vec(NumReadClients,
      Flipped(Decoupled(new VpuSpadReadRequest(p))))
    val readResponse = Vec(NumReadClients,
      Decoupled(new VpuSpadReadResponse(p)))
    val writeRequest = Vec(NumWriteClients,
      Flipped(Decoupled(new VpuSpadWriteRequest(p))))
    val busy = Output(Bool())
    // One pulse on the extra bank-read cycle of each serialized VV request.
    val serializedRead = Output(Vec(NumReadClients, Bool()))
    // One pulse when otherwise-ready clients contend for the same bank and
    // one must wait for the bank-local round-robin grant.
    val readConflictStall = Output(Bool())
    val writeConflictStall = Output(Bool())
  })

  private val memories = Seq.fill(p.vSpadBanks) {
    SyncReadMem(p.wordsPerBank, Vec(p.nLanes, UInt(p.storageBits.W)))
  }

  private def wordAddress(address: UInt): UInt = address / p.nLanes.U
  private def bank(address: UInt): UInt =
    wordAddress(address) / p.wordsPerBank.U
  private def row(address: UInt): UInt =
    wordAddress(address) % p.wordsPerBank.U
  private def bankOH(address: UInt): UInt = UIntToOH(bank(address), p.vSpadBanks)
  private def isWordAligned(address: UInt): Bool =
    (address % p.nLanes.U) === 0.U

  private val responseDepth = math.max(4, p.wordsPerVector + 2)
  private val responseQueues = Seq.fill(NumReadClients) {
    Module(new Queue(new VpuSpadReadResponse(p), responseDepth,
      pipe = false, flow = false))
  }
  for (client <- 0 until NumReadClients) {
    io.readResponse(client) <> responseQueues(client).io.deq
  }

  private val reservedWidth = math.max(1, log2Ceil(responseDepth + 1))
  private val reservedResponses = Seq.fill(NumReadClients) {
    RegInit(0.U(reservedWidth.W))
  }

  private val sIdle :: sSerialSecond :: sSerialCapture :: Nil = Enum(3)
  private val readState = Seq.fill(NumReadClients)(RegInit(sIdle))
  private val savedRequest = Seq.fill(NumReadClients) {
    Reg(new VpuSpadReadRequest(p))
  }
  private val savedBank0 = Seq.fill(NumReadClients) {
    Reg(UInt(p.bankBits.W))
  }
  private val savedBank1 = Seq.fill(NumReadClients) {
    Reg(UInt(p.bankBits.W))
  }
  private val savedRow1 = Seq.fill(NumReadClients) {
    Reg(UInt(p.rowBits.W))
  }
  private val firstData = Seq.fill(NumReadClients) {
    Reg(Vec(p.nLanes, UInt(p.storageBits.W)))
  }

  private val inputBank0 = (0 until NumReadClients).map { client =>
    bank(io.readRequest(client).bits.address0)
  }
  private val inputBank1 = (0 until NumReadClients).map { client =>
    bank(io.readRequest(client).bits.address1)
  }
  private val inputRow0 = (0 until NumReadClients).map { client =>
    row(io.readRequest(client).bits.address0)
  }
  private val inputRow1 = (0 until NumReadClients).map { client =>
    row(io.readRequest(client).bits.address1)
  }
  private val inputSameAddress = (0 until NumReadClients).map { client =>
    io.readRequest(client).bits.address0 ===
      io.readRequest(client).bits.address1
  }
  private val inputSerialized = (0 until NumReadClients).map { client =>
    io.readRequest(client).bits.useAddress1 &&
      inputBank0(client) === inputBank1(client) &&
      !inputSameAddress(client)
  }
  private val inputBankMask = (0 until NumReadClients).map { client =>
    val first = bankOH(io.readRequest(client).bits.address0)
    val second = bankOH(io.readRequest(client).bits.address1)
    Mux(io.readRequest(client).bits.useAddress1, first | second, first)
  }

  private val issuingSerialSecond = (0 until NumReadClients).map { client =>
    readState(client) === sSerialSecond
  }
  private val mandatoryBankMask = (0 until NumReadClients).map { client =>
    Mux(issuingSerialSecond(client),
      UIntToOH(savedBank1(client), p.vSpadBanks), 0.U(p.vSpadBanks.W))
  }.reduce(_ | _)

  assert(!(issuingSerialSecond(0) && issuingSerialSecond(1) &&
    savedBank1(0) === savedBank1(1)),
    "serialized read continuations must own disjoint banks")

  private val schedulerCanAccept = (0 until NumReadClients).map { client =>
    readState(client) === sIdle || readState(client) === sSerialCapture
  }
  private val responseCredit = (0 until NumReadClients).map { client =>
    reservedResponses(client) < responseDepth.U ||
      io.readResponse(client).fire
  }
  private val eligible = (0 until NumReadClients).map { client =>
    schedulerCanAccept(client) && responseCredit(client)
  }
  private val blockedByContinuation = (0 until NumReadClients).map { client =>
    (inputBankMask(client) & mandatoryBankMask).orR
  }
  private val contender = (0 until NumReadClients).map { client =>
    io.readRequest(client).valid && eligible(client) &&
      !blockedByContinuation(client)
  }

  private val readRequestsOverlap =
    (inputBankMask(0) & inputBankMask(1)).orR
  private val readConflict = contender(0) && contender(1) &&
    readRequestsOverlap
  // False gives client 0 priority; a contested grant flips the next owner.
  private val readRoundRobin = RegInit(false.B)

  io.readRequest(0).ready := eligible(0) && !blockedByContinuation(0) &&
    (!contender(1) || !readRequestsOverlap || !readRoundRobin)
  io.readRequest(1).ready := eligible(1) && !blockedByContinuation(1) &&
    (!contender(0) || !readRequestsOverlap || readRoundRobin)

  private val acceptingRead = (0 until NumReadClients).map { client =>
    io.readRequest(client).fire
  }
  when (readConflict && (acceptingRead(0) || acceptingRead(1))) {
    readRoundRobin := !readRoundRobin
  }
  io.readConflictStall := readConflict
  assert(!readConflict || PopCount(VecInit(acceptingRead)) === 1.U,
    "an overlapping read pair must have exactly one owner")

  private val readEnables = Wire(Vec(p.vSpadBanks, Bool()))
  private val readRows = Wire(Vec(p.vSpadBanks, UInt(p.rowBits.W)))
  private val readOutputs = Wire(Vec(p.vSpadBanks,
    Vec(p.nLanes, UInt(p.storageBits.W))))

  for (b <- 0 until p.vSpadBanks) {
    val serialHit0 = issuingSerialSecond(0) && savedBank1(0) === b.U
    val serialHit1 = issuingSerialSecond(1) && savedBank1(1) === b.U

    val request0First = acceptingRead(0) && inputBank0(0) === b.U
    val request0Second = acceptingRead(0) &&
      io.readRequest(0).bits.useAddress1 &&
      inputBank1(0) === b.U && inputBank0(0) =/= inputBank1(0)
    val request1First = acceptingRead(1) && inputBank0(1) === b.U
    val request1Second = acceptingRead(1) &&
      io.readRequest(1).bits.useAddress1 &&
      inputBank1(1) === b.U && inputBank0(1) =/= inputBank1(1)

    val accesses = Seq(serialHit0, serialHit1,
      request0First, request0Second, request1First, request1Second)
    assert(PopCount(VecInit(accesses)) <= 1.U,
      "a VSRAM bank received more than one read in a cycle")

    readEnables(b) := accesses.reduce(_ || _)
    readRows(b) := Mux1H(Seq(
      serialHit0 -> savedRow1(0),
      serialHit1 -> savedRow1(1),
      request0First -> inputRow0(0),
      request0Second -> inputRow1(0),
      request1First -> inputRow0(1),
      request1Second -> inputRow1(1)))
    readOutputs(b) := memories(b).read(readRows(b), readEnables(b))
  }

  private val selected0 = (0 until NumReadClients).map { client =>
    Mux1H((0 until p.vSpadBanks).map { b =>
      (savedBank0(client) === b.U) -> readOutputs(b)
    })
  }
  private val selected1 = (0 until NumReadClients).map { client =>
    Mux1H((0 until p.vSpadBanks).map { b =>
      (savedBank1(client) === b.U) -> readOutputs(b)
    })
  }

  for (client <- 0 until NumReadClients) {
    val normalResponse = RegNext(
      acceptingRead(client) && !inputSerialized(client), false.B)
    val serialResponse = readState(client) === sSerialCapture
    val responseQueue = responseQueues(client)

    assert(!(normalResponse && serialResponse),
      "one read client produced two responses in one cycle")
    responseQueue.io.enq.valid := normalResponse || serialResponse
    responseQueue.io.enq.bits.data0 :=
      Mux(serialResponse, firstData(client), selected0(client))
    responseQueue.io.enq.bits.data1 := Mux(serialResponse,
      selected1(client), Mux(savedRequest(client).useAddress1,
        Mux(savedRequest(client).address0 === savedRequest(client).address1,
          selected0(client), selected1(client)),
        0.U.asTypeOf(responseQueue.io.enq.bits.data1)))
    responseQueue.io.enq.bits.serialized := serialResponse
    responseQueue.io.enq.bits.tag := savedRequest(client).tag
    assert(!responseQueue.io.enq.valid || responseQueue.io.enq.ready,
      "per-client VSRAM response credit accounting overflowed")

    when (acceptingRead(client)) {
      assert(isWordAligned(io.readRequest(client).bits.address0),
        "VSRAM read address0 must be lane-word aligned")
      when (io.readRequest(client).bits.useAddress1) {
        assert(isWordAligned(io.readRequest(client).bits.address1),
          "VSRAM read address1 must be lane-word aligned")
      }
      savedRequest(client) := io.readRequest(client).bits
      savedBank0(client) := inputBank0(client)
      savedBank1(client) := inputBank1(client)
      savedRow1(client) := inputRow1(client)
    }

    when (readState(client) === sSerialSecond) {
      firstData(client) := selected0(client)
      readState(client) := sSerialCapture
    }.elsewhen (readState(client) === sSerialCapture) {
      readState(client) := sIdle
    }
    // A new serialized first read in the capture cycle starts immediately;
    // this permits one same-bank VV request every two bank-read cycles.
    when (acceptingRead(client) && inputSerialized(client)) {
      readState(client) := sSerialSecond
    }

    val acceptedOnly = acceptingRead(client) &&
      !io.readResponse(client).fire
    val consumedOnly = !acceptingRead(client) &&
      io.readResponse(client).fire
    when (acceptedOnly) {
      reservedResponses(client) := reservedResponses(client) + 1.U
    }.elsewhen (consumedOnly) {
      assert(reservedResponses(client) =/= 0.U,
        "VSRAM response reservation underflow")
      reservedResponses(client) := reservedResponses(client) - 1.U
    }

    io.serializedRead(client) := issuingSerialSecond(client)
  }

  private val inputWriteBank = (0 until NumWriteClients).map { client =>
    bank(io.writeRequest(client).bits.address)
  }
  private val writesOverlap = io.writeRequest(0).valid &&
    io.writeRequest(1).valid && inputWriteBank(0) === inputWriteBank(1)
  // False gives client 0 priority; contested writes alternate thereafter.
  private val writeRoundRobin = RegInit(false.B)

  io.writeRequest(0).ready := !io.writeRequest(1).valid ||
    inputWriteBank(0) =/= inputWriteBank(1) || !writeRoundRobin
  io.writeRequest(1).ready := !io.writeRequest(0).valid ||
    inputWriteBank(0) =/= inputWriteBank(1) || writeRoundRobin

  private val acceptingWrite = (0 until NumWriteClients).map { client =>
    io.writeRequest(client).fire
  }
  when (writesOverlap && (acceptingWrite(0) || acceptingWrite(1))) {
    writeRoundRobin := !writeRoundRobin
  }
  io.writeConflictStall := writesOverlap
  assert(!writesOverlap || PopCount(VecInit(acceptingWrite)) === 1.U,
    "an overlapping write pair must have exactly one owner")

  for (client <- 0 until NumWriteClients) {
    when (acceptingWrite(client)) {
      assert(isWordAligned(io.writeRequest(client).bits.address),
        "VSRAM write address must be lane-word aligned")
    }
  }
  for (b <- 0 until p.vSpadBanks) {
    val write0 = acceptingWrite(0) && inputWriteBank(0) === b.U
    val write1 = acceptingWrite(1) && inputWriteBank(1) === b.U
    assert(PopCount(VecInit(Seq(write0, write1))) <= 1.U,
      "a VSRAM bank received more than one write in a cycle")
    val selectedWrite = Mux(write0,
      io.writeRequest(0).bits, io.writeRequest(1).bits)
    // Keep exactly one writer mport per physical bank. Multiple Scala
    // `write()` calls infer multiple memory ports even when their enables are
    // mutually exclusive, defeating the advertised bank-local 1R1W SRAM.
    when (write0 || write1) {
      memories(b).write(row(selectedWrite.address),
        selectedWrite.data, selectedWrite.laneMask)
    }
  }

  io.busy := (0 until NumReadClients).map { client =>
    reservedResponses(client) =/= 0.U || readState(client) =/= sIdle
  }.reduce(_ || _)
}
