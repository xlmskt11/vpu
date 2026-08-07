package vpu

import chisel3._
import chisel3.util._
import gemmini.{GemminiVpuMatrixReadIO, GemminiVpuMatrixReadResp,
  GemminiVpuMatrixWriteReq}

object VpuBankedScratchpad {
  final val NumReadClients = 3
  final val NumWriteClients = 2

  // A request reserves response credit before touching SyncReadMem. Four
  // entries per independent client decouple fixed-latency SRAM responses from
  // arbitrary consumer backpressure without scaling with architectural VLEN.
  final val ReadResponseQueueDepth = 4

  final val Source0ReadClient = 0
  final val Source1ReadClient = 1
  final val StoreReadClient = 2
  final val ExecuteWriteClient = 0
  final val LoadWriteClient = 1
}

/** Bank-local 1R1W Vector SRAM with independent source-0, source-1, and store
  * read clients plus execute/load write clients.
  *
  * Each physical bank receives at most one read and one masked write in a
  * cycle. Requests to different banks proceed concurrently; requests to the
  * same bank are selected independently by that bank's round-robin arbiter.
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
    val matrixRead = if (p.matrixPorts > 0) Some(Vec(p.matrixPorts,
      Flipped(new GemminiVpuMatrixReadIO(
        p.matrixRowAddrBits, p.nLanes, p.storageBits)))) else None
    val matrixWrite = if (p.matrixPorts > 0) Some(Flipped(Vec(p.matrixPorts,
      Valid(new GemminiVpuMatrixWriteReq(
        p.matrixRowAddrBits, p.nLanes, p.storageBits))))) else None
    val busy = Output(Bool())
    // One pulse when otherwise-ready clients contend for the same bank and
    // one must wait for the bank-local round-robin grant.
    val readConflictStall = Output(Bool())
    val writeConflictStall = Output(Bool())
  })

  private val memories = Seq.fill(p.physicalBanks) {
    SyncReadMem(p.wordsPerSubBank, Vec(p.nLanes, UInt(p.storageBits.W)))
  }

  private def wordAddress(address: UInt): UInt = address / p.nLanes.U
  private def logicalBank(address: UInt): UInt =
    wordAddress(address) / p.wordsPerBank.U
  private def wordInBank(address: UInt): UInt =
    wordAddress(address) % p.wordsPerBank.U
  private def subBank(address: UInt): UInt =
    wordInBank(address) % p.vSpadSubBanks.U
  private def bank(address: UInt): UInt =
    logicalBank(address) * p.vSpadSubBanks.U + subBank(address)
  private def row(address: UInt): UInt =
    wordInBank(address) / p.vSpadSubBanks.U
  private def matrixElementAddress(matrixRow: UInt): UInt =
    matrixRow * p.nLanes.U
  private def isWordAligned(address: UInt): Bool =
    (address % p.nLanes.U) === 0.U

  private val responseQueues = Seq.fill(NumReadClients) {
    Module(new Queue(new VpuSpadReadResponse(p), ReadResponseQueueDepth,
      pipe = false, flow = false))
  }
  for (client <- 0 until NumReadClients) {
    io.readResponse(client) <> responseQueues(client).io.deq
  }

  private val matrixResponseDepth = 2
  private val matrixResponseQueues = Seq.fill(p.matrixPorts) {
    Module(new Queue(new GemminiVpuMatrixReadResp(
      p.nLanes, p.storageBits), matrixResponseDepth,
      pipe = false, flow = false))
  }
  private val matrixReservedWidth =
    math.max(1, log2Ceil(matrixResponseDepth + 1))
  private val matrixReservedResponses = Seq.fill(p.matrixPorts) {
    RegInit(0.U(matrixReservedWidth.W))
  }
  if (p.matrixPorts > 0) {
    for (port <- 0 until p.matrixPorts) {
      io.matrixRead.get(port).resp <> matrixResponseQueues(port).io.deq
    }
  }

  private val reservedWidth =
    math.max(1, log2Ceil(ReadResponseQueueDepth + 1))
  private val reservedResponses = Seq.fill(NumReadClients) {
    RegInit(0.U(reservedWidth.W))
  }

  private val inputBanks = (0 until NumReadClients).map { client =>
    bank(io.readRequest(client).bits.address)
  }

  private val matrixAddresses = (0 until p.matrixPorts).map { port =>
    matrixElementAddress(io.matrixRead.get(port).req.bits.rowAddress)
  }
  private val matrixBanks = matrixAddresses.map(bank)
  private val matrixLogicalBanks = matrixAddresses.map(logicalBank)
  private val matrixRows = matrixAddresses.map(row)
  private val matrixCredit = (0 until p.matrixPorts).map { port =>
    matrixReservedResponses(port) < matrixResponseDepth.U ||
      io.matrixRead.get(port).resp.fire
  }
  private val matrixContender = (0 until p.matrixPorts).map { port =>
    io.matrixRead.get(port).req.valid && matrixCredit(port)
  }

  // Software assigns each active Gemmini a distinct logical VSRAM bank. A
  // violation is a programming error rather than a reason to drop a Valid
  // matrix write later in the pipeline.
  for (lhs <- 0 until p.matrixPorts; rhs <- lhs + 1 until p.matrixPorts) {
    assert(!(io.matrixRead.get(lhs).req.valid &&
      io.matrixRead.get(rhs).req.valid &&
      matrixLogicalBanks(lhs) === matrixLogicalBanks(rhs)),
      "two Gemminis requested the same logical VSRAM read bank")
  }

  for (port <- 0 until p.matrixPorts) {
    val earlierSamePhysical = (0 until port).map { earlier =>
      matrixContender(earlier) && matrixBanks(earlier) === matrixBanks(port)
    }.reduceOption(_ || _).getOrElse(false.B)
    io.matrixRead.get(port).req.ready := matrixCredit(port) &&
      !earlierSamePhysical
  }

  private val acceptingMatrixRead = (0 until p.matrixPorts).map { port =>
    io.matrixRead.get(port).req.fire
  }
  private val matrixClaimedBankMask = if (p.matrixPorts > 0) {
    matrixContender.zip(matrixBanks).map { case (valid, selectedBank) =>
      Mux(valid, UIntToOH(selectedBank, p.physicalBanks),
        0.U(p.physicalBanks.W))
    }.reduce(_ | _)
  } else {
    0.U(p.physicalBanks.W)
  }

  private val responseCredit = (0 until NumReadClients).map { client =>
    reservedResponses(client) < ReadResponseQueueDepth.U ||
      io.readResponse(client).fire
  }

  // There is one independent arbiter per physical bank. A client can target
  // only one bank in a cycle, while distinct clients may therefore be granted
  // by distinct arbiters concurrently. Gemmini matrix reads claim their banks
  // before these arbiters and retain the highest priority.
  private val readArbiters = Seq.fill(p.physicalBanks) {
    Module(new RRArbiter(new VpuSpadReadRequest(p), NumReadClients))
  }
  for (b <- 0 until p.physicalBanks) {
    val matrixClaimed = matrixClaimedBankMask(b)
    for (client <- 0 until NumReadClients) {
      readArbiters(b).io.in(client).valid :=
        io.readRequest(client).valid && responseCredit(client) &&
          inputBanks(client) === b.U && !matrixClaimed
      readArbiters(b).io.in(client).bits := io.readRequest(client).bits
    }
    readArbiters(b).io.out.ready := !matrixClaimed
  }

  for (client <- 0 until NumReadClients) {
    val selectedBankReady = (0 until p.physicalBanks).map { b =>
      inputBanks(client) === b.U &&
        readArbiters(b).io.in(client).ready
    }.reduce(_ || _)
    io.readRequest(client).ready := responseCredit(client) &&
      !matrixClaimedBankMask(inputBanks(client)) && selectedBankReady
  }

  private val acceptingRead = (0 until NumReadClients).map { client =>
    io.readRequest(client).fire
  }
  private val localReadFire = (0 until p.physicalBanks).map { b =>
    readArbiters(b).io.out.fire
  }
  private val localContender = (0 until NumReadClients).map { client =>
    io.readRequest(client).valid && responseCredit(client) &&
      !matrixClaimedBankMask(inputBanks(client))
  }
  private val readConflicts = (0 until p.physicalBanks).map { b =>
    PopCount(VecInit((0 until NumReadClients).map { client =>
      localContender(client) && inputBanks(client) === b.U
    })) > 1.U
  }
  io.readConflictStall := readConflicts.reduce(_ || _)

  // Each client has one request port, so it must be granted by no more than
  // one bank arbiter in a cycle. Conversely, each arbiter grants one client.
  for (client <- 0 until NumReadClients) {
    assert(PopCount(VecInit((0 until p.physicalBanks).map { b =>
      localReadFire(b) &&
        readArbiters(b).io.chosen === client.U
    })) <= 1.U,
      "one VSRAM read client was granted by multiple banks")
  }

  private val readEnables = Wire(Vec(p.physicalBanks, Bool()))
  private val readRows = Wire(Vec(p.physicalBanks, UInt(p.subBankRowBits.W)))
  private val readOutputs = Wire(Vec(p.physicalBanks,
    Vec(p.nLanes, UInt(p.storageBits.W))))
  private val savedMatrixBank = Seq.fill(p.matrixPorts) {
    Reg(UInt(p.physicalBankBits.W))
  }
  private val readClientBits = math.max(1, log2Ceil(NumReadClients))
  private val savedReadClient = Seq.fill(p.physicalBanks) {
    Reg(UInt(readClientBits.W))
  }
  private val savedReadTag = Seq.fill(p.physicalBanks) {
    Reg(UInt(p.spadReadTagBits.W))
  }

  for (b <- 0 until p.physicalBanks) {
    val matrixHits = (0 until p.matrixPorts).map { port =>
      acceptingMatrixRead(port) && matrixBanks(port) === b.U
    }
    val accesses = matrixHits :+ localReadFire(b)
    assert(PopCount(VecInit(accesses)) <= 1.U,
      "a VSRAM bank received more than one read in a cycle")

    readEnables(b) := accesses.reduce(_ || _)
    val matrixRowsForBank = matrixHits.zip(matrixRows)
    readRows(b) := Mux1H(matrixRowsForBank :+
      (localReadFire(b) -> row(readArbiters(b).io.out.bits.address)))
    readOutputs(b) := memories(b).read(readRows(b), readEnables(b))

    when (localReadFire(b)) {
      savedReadClient(b) := readArbiters(b).io.chosen
      savedReadTag(b) := readArbiters(b).io.out.bits.tag
    }
  }

  private val selectedMatrix = (0 until p.matrixPorts).map { port =>
    Mux1H((0 until p.physicalBanks).map { b =>
      (savedMatrixBank(port) === b.U) -> readOutputs(b)
    })
  }

  for (port <- 0 until p.matrixPorts) {
    val responseValid = RegNext(acceptingMatrixRead(port), false.B)
    val responseQueue = matrixResponseQueues(port)
    responseQueue.io.enq.valid := responseValid
    responseQueue.io.enq.bits.data := selectedMatrix(port)
    assert(!responseQueue.io.enq.valid || responseQueue.io.enq.ready,
      "Gemmini VSRAM matrix response credit accounting overflowed")

    when (acceptingMatrixRead(port)) {
      assert(io.matrixRead.get(port).req.bits.rowAddress < p.totalWords.U,
        "Gemmini VSRAM matrix row is out of range")
      savedMatrixBank(port) := matrixBanks(port)
    }

    val acceptedOnly = acceptingMatrixRead(port) &&
      !io.matrixRead.get(port).resp.fire
    val consumedOnly = !acceptingMatrixRead(port) &&
      io.matrixRead.get(port).resp.fire
    when (acceptedOnly) {
      matrixReservedResponses(port) := matrixReservedResponses(port) + 1.U
    }.elsewhen (consumedOnly) {
      assert(matrixReservedResponses(port) =/= 0.U,
        "Gemmini VSRAM matrix response reservation underflow")
      matrixReservedResponses(port) := matrixReservedResponses(port) - 1.U
    }
  }

  private val localResponseValid = (0 until p.physicalBanks).map { b =>
    RegNext(localReadFire(b), false.B)
  }
  for (client <- 0 until NumReadClients) {
    val responseHits = (0 until p.physicalBanks).map { b =>
      localResponseValid(b) && savedReadClient(b) === client.U
    }
    assert(PopCount(VecInit(responseHits)) <= 1.U,
      "one VSRAM read client received multiple bank responses")

    val responseQueue = responseQueues(client)
    responseQueue.io.enq.valid := responseHits.reduce(_ || _)
    responseQueue.io.enq.bits.data := Mux1H(
      (0 until p.physicalBanks).map { b =>
        responseHits(b) -> readOutputs(b)
      })
    responseQueue.io.enq.bits.tag := Mux1H(
      (0 until p.physicalBanks).map { b =>
        responseHits(b) -> savedReadTag(b)
      })
    assert(!responseQueue.io.enq.valid || responseQueue.io.enq.ready,
      "VSRAM read response credit accounting overflowed")

    when (acceptingRead(client)) {
      assert(isWordAligned(io.readRequest(client).bits.address),
        "VSRAM read address must be lane-word aligned")
    }

    val acceptedOnly = acceptingRead(client) &&
      !io.readResponse(client).fire
    val consumedOnly = !acceptingRead(client) &&
      io.readResponse(client).fire
    when (acceptedOnly) {
      reservedResponses(client) := reservedResponses(client) + 1.U
    }.elsewhen (consumedOnly) {
      assert(reservedResponses(client) =/= 0.U,
        "VSRAM read response reservation underflow")
      reservedResponses(client) := reservedResponses(client) - 1.U
    }
    assert(reservedResponses(client) <= ReadResponseQueueDepth.U,
      "VSRAM read response reservations exceeded queue depth")
  }

  private val inputWriteBank = (0 until NumWriteClients).map { client =>
    bank(io.writeRequest(client).bits.address)
  }
  private val matrixWriteAddresses = (0 until p.matrixPorts).map { port =>
    matrixElementAddress(io.matrixWrite.get(port).bits.rowAddress)
  }
  private val matrixWriteBanks = matrixWriteAddresses.map(bank)
  private val matrixWriteLogicalBanks = matrixWriteAddresses.map(logicalBank)
  private val matrixWriteRows = matrixWriteAddresses.map(row)
  private val matrixWriteValid = (0 until p.matrixPorts).map { port =>
    io.matrixWrite.get(port).valid
  }
  private val matrixWriteClaimedBankMask = if (p.matrixPorts > 0) {
    matrixWriteValid.zip(matrixWriteBanks).map { case (valid, selectedBank) =>
      Mux(valid, UIntToOH(selectedBank, p.physicalBanks),
        0.U(p.physicalBanks.W))
    }.reduce(_ | _)
  } else {
    0.U(p.physicalBanks.W)
  }

  for (port <- 0 until p.matrixPorts) {
    when (matrixWriteValid(port)) {
      assert(io.matrixWrite.get(port).bits.rowAddress < p.totalWords.U,
        "Gemmini VSRAM matrix write row is out of range")
    }
  }
  for (lhs <- 0 until p.matrixPorts; rhs <- lhs + 1 until p.matrixPorts) {
    assert(!(matrixWriteValid(lhs) && matrixWriteValid(rhs) &&
      matrixWriteLogicalBanks(lhs) === matrixWriteLogicalBanks(rhs)),
      "two Gemminis wrote the same logical VSRAM bank")
  }

  private val writesOverlap = io.writeRequest(0).valid &&
    io.writeRequest(1).valid && inputWriteBank(0) === inputWriteBank(1)
  // False gives client 0 priority; contested writes alternate thereafter.
  private val writeRoundRobin = RegInit(false.B)

  if (p.matrixPorts > 0) {
    val executeBlockedByMatrix =
      matrixWriteClaimedBankMask(inputWriteBank(0))
    val loadBlockedByMatrix =
      matrixWriteClaimedBankMask(inputWriteBank(1))
    io.writeRequest(0).ready := !executeBlockedByMatrix
    io.writeRequest(1).ready := !loadBlockedByMatrix &&
      (!io.writeRequest(0).valid ||
        inputWriteBank(0) =/= inputWriteBank(1))
  } else {
    io.writeRequest(0).ready := !io.writeRequest(1).valid ||
      inputWriteBank(0) =/= inputWriteBank(1) || !writeRoundRobin
    io.writeRequest(1).ready := !io.writeRequest(0).valid ||
      inputWriteBank(0) =/= inputWriteBank(1) || writeRoundRobin
  }

  private val acceptingWrite = (0 until NumWriteClients).map { client =>
    io.writeRequest(client).fire
  }
  when (writesOverlap && (acceptingWrite(0) || acceptingWrite(1))) {
    writeRoundRobin := !writeRoundRobin
  }
  io.writeConflictStall := writesOverlap
  val overlappedWritesBlockedByMatrix =
    matrixWriteClaimedBankMask(inputWriteBank(0))
  assert(!writesOverlap || overlappedWritesBlockedByMatrix ||
    PopCount(VecInit(acceptingWrite)) === 1.U,
    "an overlapping write pair must have exactly one owner")

  for (client <- 0 until NumWriteClients) {
    when (acceptingWrite(client)) {
      assert(isWordAligned(io.writeRequest(client).bits.address),
        "VSRAM write address must be lane-word aligned")
    }
  }
  for (b <- 0 until p.physicalBanks) {
    val matrixHits = (0 until p.matrixPorts).map { port =>
      matrixWriteValid(port) && matrixWriteBanks(port) === b.U
    }
    val write0 = acceptingWrite(0) && inputWriteBank(0) === b.U
    val write1 = acceptingWrite(1) && inputWriteBank(1) === b.U
    val accesses = matrixHits ++ Seq(write0, write1)
    assert(PopCount(VecInit(accesses)) <= 1.U,
      "a VSRAM physical bank received more than one write in a cycle")
    val matrixData = matrixHits.zipWithIndex.map { case (hit, port) =>
      hit -> io.matrixWrite.get(port).bits.data
    }
    val matrixMasks = matrixHits.zipWithIndex.map { case (hit, port) =>
      hit -> io.matrixWrite.get(port).bits.laneMask
    }
    val matrixRowSelect = matrixHits.zip(matrixWriteRows)
    val selectedData = Mux1H(matrixData ++ Seq(
      write0 -> io.writeRequest(0).bits.data,
      write1 -> io.writeRequest(1).bits.data))
    val selectedMask = Mux1H(matrixMasks ++ Seq(
      write0 -> io.writeRequest(0).bits.laneMask,
      write1 -> io.writeRequest(1).bits.laneMask))
    val selectedRow = Mux1H(matrixRowSelect ++ Seq(
      write0 -> row(io.writeRequest(0).bits.address),
      write1 -> row(io.writeRequest(1).bits.address)))
    // Keep exactly one writer mport per physical bank. Multiple Scala
    // `write()` calls infer multiple memory ports even when their enables are
    // mutually exclusive, defeating the advertised bank-local 1R1W SRAM.
    when (accesses.reduce(_ || _)) {
      memories(b).write(selectedRow, selectedData, selectedMask)
    }
  }

  io.busy := reservedResponses.map(_ =/= 0.U).reduce(_ || _) ||
    matrixReservedResponses.map(_ =/= 0.U)
      .reduceOption(_ || _).getOrElse(false.B)
}
