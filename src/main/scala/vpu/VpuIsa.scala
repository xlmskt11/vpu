package vpu

import chisel3._
import chisel3.util._

import freechips.rocketchip.rocket.MStatus

/** Six-bit micro-opcodes plus the v1 control extension. */
object VpuOpcode {
  val V_ADD_VV      = 0x0d
  val V_ADD_VF      = 0x0e
  val V_SUB_VV      = 0x0f
  val V_SUB_VF      = 0x10
  val V_MUL_VV      = 0x11
  val V_MUL_VF      = 0x12
  val V_EXP_V       = 0x13
  val V_RECI_V      = 0x14
  val V_RED_SUM     = 0x15
  val V_RED_MAX     = 0x16

  val S_ADD_FP      = 0x17
  val S_SUB_FP      = 0x18
  val S_MAX_FP      = 0x19
  val S_MUL_FP      = 0x1a
  val S_EXP_FP      = 0x1b
  val S_RECI_FP     = 0x1c
  val S_SQRT_FP     = 0x1d
  val S_LOAD_STATE  = 0x1e
  val S_STORE_STATE = 0x1f

  // PLENA-compatible scalar-integer address induction.  Unlike the ordinary
  // six-field microinstruction, bits [31:14] are an unsigned immediate.
  val S_ADDI_INT    = 0x22

  val H_PREFETCH_V  = 0x29
  val H_STORE_V     = 0x2a

  // Optional start-to-start stride, in VSRAM elements, between consecutive
  // matrix-row segments of one logical vector command. Zero retains the
  // original contiguous addressing mode.
  // 0x2c is PLENA's scale-register opcode. Keep that encoding free even
  // though v1 has no scale SRAM, and place this local extension at 0x37.
  val C_SET_VSTRIDE = 0x37

  // Keep the public encodings aligned with PLENA where an equivalent
  // operation exists.  The control operations use RoCC rs2/payload directly
  // instead of requiring PLENA's scalar integer register file.
  val C_SET_STRIDE  = 0x2d
  val C_WRITE_VMASK = 0x2e
  // PLENA-compatible hardware-loop delimiters. C_LOOP_START uses rd as the
  // loop tag and bits [31:10] as a positive iteration count; C_LOOP_END only
  // carries the matching rd.  VpuHardwareLoop consumes both before VpuCore.
  val C_LOOP_START  = 0x2f
  val C_LOOP_END    = 0x30
  val V_GATHER_VV   = 0x31
  val V_SLIDE_V     = 0x32

  val V_MAX_VF      = 0x35
  val V_MIN_VF      = 0x36

  val C_WRITE_GP    = 0x38
  val C_WRITE_FP    = 0x39
  val C_WRITE_H     = 0x3a
  val C_SET_VL      = 0x3b
  val C_WAIT        = 0x3c
  val C_FENCE       = 0x3d
  val C_READ        = 0x3e
  val C_CLEAR_STATUS = 0x3f

  val supported: Set[Int] = Set(
    V_ADD_VV, V_ADD_VF, V_SUB_VV, V_SUB_VF, V_MUL_VV, V_MUL_VF,
    V_EXP_V, V_RECI_V, V_RED_SUM, V_RED_MAX, V_MAX_VF, V_MIN_VF,
    S_ADD_FP, S_SUB_FP, S_MAX_FP, S_MUL_FP, S_EXP_FP, S_RECI_FP,
    S_SQRT_FP, S_LOAD_STATE, S_STORE_STATE, S_ADDI_INT,
    H_PREFETCH_V, H_STORE_V, C_SET_VSTRIDE, C_SET_STRIDE, C_WRITE_VMASK,
    C_LOOP_START, C_LOOP_END,
    V_GATHER_VV,
    V_SLIDE_V, C_WRITE_GP, C_WRITE_FP,
    C_WRITE_H, C_SET_VL, C_WAIT, C_FENCE, C_READ, C_CLEAR_STATUS)
}

object VpuReadSelector {
  val Status = 0
  val Gp = 1
  val Fp = 2
  val H = 3
  val Perf = 4
  val FaultAddress = 5
  val FaultInfo = 6
}

object VpuDmaFaultCause {
  val None = 0
  val Translation = 1
  val Access = 2
  val Protocol = 3
}

object VpuFaultInfoLayout {
  val CauseLo = 0
  val CauseHi = 1
  val IsWrite = 2
  val Valid = 3
}

object VpuWaitMask {
  val Load = 1
  val Execute = 2
  val Store = 4
}

object VpuPerfIndex {
  val Cycles = 0
  val BusyCycles = 1
  val DmaReadBytes = 2
  val DmaWriteBytes = 3
  val DmaExecuteOverlapCycles = 4
  val BankConflictStallCycles = 5
  val HazardStallCycles = 6
  val SfuBusyCycles = 7
  val Faults = 8
  val Count = 9
}

/** Independent C_CLEAR_STATUS payload masks. */
object VpuClearMask {
  val Fflags = 1 << 0
  val FaultIllegal = 1 << 1
  val Perf = 1 << 2
  // Compatibility aliases used by the software header.
  val Errors = Fflags | FaultIllegal
  val All = Errors | Perf
}

object VpuStatusLayout {
  val IllegalCommand = 0
  val DmaFault = 1
  val DmaHalted = 2
  val FusionFault = 3
  val FflagsLo = 8
  val FflagsHi = 12
  val Busy = 16
}

/** Pure-Scala encoder shared by tests and header-generation code. */
object VpuEncoding {
  def pack(opcode: Int, rd: Int = 0, rs1: Int = 0, rs2: Int = 0,
           rs3: Int = 0, funct1: Int = 0): Int = {
    require((opcode & ~0x3f) == 0)
    Seq(rd, rs1, rs2, rs3, funct1).foreach(x => require((x & ~0xf) == 0))
    opcode | (rd << 6) | (rs1 << 10) | (rs2 << 14) |
      (rs3 << 18) | (funct1 << 22)
  }

  def packAddiInt(rd: Int, rs1: Int, immediate: Int): Int = {
    require((rd & ~0xf) == 0 && (rs1 & ~0xf) == 0)
    require(immediate >= 0 && immediate < (1 << 18))
    VpuOpcode.S_ADDI_INT | (rd << 6) | (rs1 << 10) | (immediate << 14)
  }

  def packLoopStart(loopRegister: Int, iterations: Int): Int = {
    require((loopRegister & ~0xf) == 0)
    require(iterations > 0 && iterations < (1 << 22))
    VpuOpcode.C_LOOP_START | (loopRegister << 6) | (iterations << 10)
  }

  def packLoopEnd(loopRegister: Int): Int = {
    require((loopRegister & ~0xf) == 0)
    VpuOpcode.C_LOOP_END | (loopRegister << 6)
  }
}

class VpuDecodedMicroOp extends Bundle {
  val opcode = UInt(6.W)
  val rd = UInt(4.W)
  val rs1 = UInt(4.W)
  val rs2 = UInt(4.W)
  val rs3 = UInt(4.W)
  val funct1 = UInt(4.W)
  val reserved = UInt(6.W)
}

object VpuDecode {
  def apply(word: UInt): VpuDecodedMicroOp = {
    require(word.getWidth == 32)
    val out = Wire(new VpuDecodedMicroOp)
    out.opcode := word(5, 0)
    out.rd := word(9, 6)
    out.rs1 := word(13, 10)
    out.rs2 := word(17, 14)
    out.rs3 := word(21, 18)
    out.funct1 := word(25, 22)
    out.reserved := word(31, 26)
    out
  }

  def isSupported(opcode: UInt): Bool =
    VpuOpcode.supported.toSeq.sorted.map(x => opcode === x.U).reduce(_ || _)

  def isVector(opcode: UInt): Bool = Seq(
    VpuOpcode.V_ADD_VV, VpuOpcode.V_ADD_VF, VpuOpcode.V_SUB_VV,
    VpuOpcode.V_SUB_VF, VpuOpcode.V_MUL_VV, VpuOpcode.V_MUL_VF,
    VpuOpcode.V_EXP_V, VpuOpcode.V_RECI_V, VpuOpcode.V_RED_SUM,
    VpuOpcode.V_RED_MAX, VpuOpcode.V_GATHER_VV, VpuOpcode.V_SLIDE_V,
    VpuOpcode.V_MAX_VF, VpuOpcode.V_MIN_VF
  ).map(x => opcode === x.U).reduce(_ || _)

  def isScalar(opcode: UInt): Bool = (opcode >= VpuOpcode.S_ADD_FP.U) &&
    (opcode <= VpuOpcode.S_SQRT_FP.U)

  def isState(opcode: UInt): Bool =
    opcode === VpuOpcode.S_LOAD_STATE.U ||
      opcode === VpuOpcode.S_STORE_STATE.U

  def isLoad(opcode: UInt): Bool = opcode === VpuOpcode.H_PREFETCH_V.U
  def isStore(opcode: UInt): Bool = opcode === VpuOpcode.H_STORE_V.U
  def isLoop(opcode: UInt): Bool =
    opcode === VpuOpcode.C_LOOP_START.U ||
      opcode === VpuOpcode.C_LOOP_END.U
  def isControl(opcode: UInt): Bool =
    opcode === VpuOpcode.S_ADDI_INT.U ||
      opcode === VpuOpcode.C_SET_VSTRIDE.U ||
      opcode === VpuOpcode.C_SET_STRIDE.U ||
      opcode === VpuOpcode.C_WRITE_VMASK.U ||
      opcode >= VpuOpcode.C_WRITE_GP.U
}

class VpuCommand extends Bundle {
  val microOp = UInt(32.W)
  val payload = UInt(64.W)
  val rd = UInt(5.W)
  val xd = Bool()
  val status = new MStatus
}

/** Command envelope used by the hardware-loop transport.
  *
  * Group metadata is physically present only in a fused build.  Standalone
  * VPUs still use the same loop frontend and retain the transport-malformed
  * bit, but do not elaborate unused group ID/last wires into the loop buffer.
  * The optional metadata is consumed by [[VpuGroupCommandGate]] and never
  * enters the 32-bit PLENA-style microinstruction decoder.
  */
class VpuGroupedCommand(
    groupIdBits: Int,
    enableGroupedCommands: Boolean) extends Bundle {
  val command = new VpuCommand
  val groupId = if (enableGroupedCommands) Some(UInt(groupIdBits.W)) else None
  val grouped = if (enableGroupedCommands) Some(Bool()) else None
  val last = if (enableGroupedCommands) Some(Bool()) else None
  val malformed = Bool()
}

/** VPU-side view of the LdBCompleteControl group handshake.
  *
  * `pending/groupId` query the group controller. A grouped command may enter
  * the VPU reservation station once all Gemmini children are registered and
  * `dispatchEnable` is asserted. The final two outputs are one-cycle
  * notifications; in particular,
  * `lastDispatchFire` is not asserted merely because RoCC accepted a command
  * into the one-entry gate.
  */
class VpuGemvGroupIO(groupIdBits: Int) extends Bundle {
  val pending = Output(Bool())
  val groupId = Output(UInt(groupIdBits.W))
  val groupAllocated = Input(Bool())
  val dispatchEnable = Input(Bool())
  val dispatchReject = Input(Bool())
  val lastDispatchFire = Output(Bool())
  val abort = Output(Bool())
}

class VpuResponse extends Bundle {
  val data = UInt(64.W)
  val rd = UInt(5.W)
}
