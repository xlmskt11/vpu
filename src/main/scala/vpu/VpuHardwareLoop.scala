package vpu

import chisel3._
import chisel3.util._
import freechips.rocketchip.rocket.MStatus

/** Per-static-instruction storage for a legal loop body.  Legal captured
  * commands cannot request a response (`xd=false`), terminate a group, or
  * change privilege/group context, so only the actual VPU uop and its payload
  * need to be replicated per buffer entry.
  */
class VpuLoopBufferedCommand extends Bundle {
  val microOp = UInt(32.W)
  val payload = UInt(64.W)
}

/** Capture/replay front end for the PLENA-compatible hardware-loop opcodes.
  *
  * The RoCC transport has no accelerator instruction memory or PC.  The first
  * outer C_LOOP_START therefore switches this block into capture mode.  The
  * complete, properly nested region is retained locally and replayed after the
  * matching outer C_LOOP_END arrives.  Loop-control commands are consumed by
  * this block; all other commands retain their original payload and transport
  * metadata.
  *
  * `group_last` is deliberately illegal inside a captured region.  Capturing
  * it would acknowledge the RoCC instruction before the final dynamic command
  * has reached the reservation station and SharedDeps, allowing a following
  * Gemmini command to overtake future replay writes.  Software peels the final
  * iteration and sends its terminating data command after C_LOOP_END instead.
  */
class VpuHardwareLoop(
    groupIdBits: Int,
    enableGroupedCommands: Boolean,
    bufferEntries: Int = 64,
    maxDepth: Int = 4) extends Module {
  require(groupIdBits > 0)
  require(bufferEntries >= 2 && (bufferEntries & (bufferEntries - 1)) == 0,
    "the hardware-loop command buffer must have a power-of-two capacity")
  require(maxDepth > 0)

  private val CLoopStart = 0x2f
  private val CLoopEnd = 0x30
  private val pcBits = math.max(1, log2Ceil(bufferEntries))
  private val countBits = math.max(1, log2Ceil(bufferEntries + 1))
  private val depthBits = math.max(1, log2Ceil(maxDepth + 1))

  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new VpuGroupedCommand(groupIdBits,
      enableGroupedCommands)))
    val out = Decoupled(new VpuGroupedCommand(groupIdBits,
      enableGroupedCommands))
    val busy = Output(Bool())
    /** One-cycle pulse when a malformed loop region is accepted or detected. */
    val protocolError = Output(Bool())
  })

  val sPass :: sCapture :: sReplay :: sDrain :: sError :: Nil = Enum(5)
  val state = RegInit(sPass)

  // A register array gives combinational replay reads. Region-common MStatus
  // and group metadata are retained once below instead of once per entry.
  val commandBuffer = Reg(Vec(bufferEntries, new VpuLoopBufferedCommand))
  val captureCount = RegInit(0.U(countBits.W))
  val programLength = RegInit(0.U(countBits.W))
  val captureDepth = RegInit(0.U(depthBits.W))
  val captureRegisters = Reg(Vec(maxDepth, UInt(4.W)))
  val captureStatus = Reg(new MStatus)
  val captureGrouped = if (enableGroupedCommands) {
    Some(RegInit(false.B))
  } else None
  val captureGroupId = if (enableGroupedCommands) {
    Some(RegInit(0.U(groupIdBits.W)))
  } else None

  val replayPc = RegInit(0.U(pcBits.W))
  val replayDepth = RegInit(0.U(depthBits.W))
  val replayStartPc = Reg(Vec(maxDepth, UInt(pcBits.W)))
  val replayRemaining = Reg(Vec(maxDepth, UInt(22.W)))
  val replayRegisters = Reg(Vec(maxDepth, UInt(4.W)))

  val errorCommand = Reg(new VpuGroupedCommand(groupIdBits,
    enableGroupedCommands))
  val errorResumeDrain = RegInit(false.B)
  // Drain depth is intentionally wider than the legal nesting stack.  Once a
  // malformed START exceeds maxDepth, bracket counting must still reach the
  // corresponding outer END without indexing any finite loop-register stack.
  val drainDepth = RegInit(0.U(32.W))
  // Only the fused transport needs a synthetic group terminator.  Standalone
  // malformed-loop recovery simply drains the lexical region and reports the
  // protocol error, without carrying any group lifetime state.
  val drainEmitTerminator = if (enableGroupedCommands) {
    Some(RegInit(false.B))
  } else None
  val protocolError = RegInit(false.B)
  protocolError := false.B

  private def opcode(command: VpuGroupedCommand): UInt =
    command.command.microOp(5, 0)
  private def isLoopStart(command: VpuGroupedCommand): Bool =
    opcode(command) === CLoopStart.U
  private def isLoopEnd(command: VpuGroupedCommand): Bool =
    opcode(command) === CLoopEnd.U
  private def grouped(command: VpuGroupedCommand): Bool =
    if (enableGroupedCommands) command.grouped.get else false.B
  private def groupId(command: VpuGroupedCommand): UInt =
    if (enableGroupedCommands) command.groupId.get else 0.U(groupIdBits.W)
  private def last(command: VpuGroupedCommand): Bool =
    if (enableGroupedCommands) command.last.get else false.B
  private def capturedGrouped: Bool =
    if (enableGroupedCommands) captureGrouped.get else false.B
  private def capturedGroupId: UInt =
    if (enableGroupedCommands) captureGroupId.get else 0.U(groupIdBits.W)
  private def emitTerminator: Bool =
    if (enableGroupedCommands) drainEmitTerminator.get else false.B

  private def storeBuffered(index: UInt,
      command: VpuGroupedCommand): Unit = {
    commandBuffer(index).microOp := command.command.microOp
    commandBuffer(index).payload := command.command.payload
  }

  io.in.ready := false.B
  io.out.valid := false.B
  io.out.bits := 0.U.asTypeOf(io.out.bits)

  switch(state) {
    is(sPass) {
      val start = isLoopStart(io.in.bits)
      val end = isLoopEnd(io.in.bits)
      val loopControl = start || end

      // Ordinary commands retain a zero-latency ready/valid path.  Loop
      // controls are consumed locally and never reach VpuCore's decoder.
      io.out.valid := io.in.valid && !loopControl
      io.out.bits := io.in.bits
      io.in.ready := Mux(loopControl, true.B, io.out.ready)

      when(io.in.fire && start) {
        val count = io.in.bits.command.microOp(31, 10)
        // A captured RoCC command cannot request an architectural response:
        // Rocket would wait for that response before it could send the rest
        // of the region, while replaying it would create multiple responses.
        val malformedStart = count === 0.U || last(io.in.bits) ||
          io.in.bits.command.xd || io.in.bits.malformed
        when(malformedStart) {
          // A malformed START still opens a lexical region in the incoming
          // stream.  Report it as a non-terminating error, then discard through
          // its matching outer END.  Grouped recovery emits a synthetic final
          // terminator there so VpuGroupCommandGate's abortDrain is cleared;
          // standalone recovery already reported the architectural error here.
          errorCommand := io.in.bits
          errorCommand.command.microOp := "h8000000e".U
          if (enableGroupedCommands) {
            errorCommand.last.get := false.B
          }
          errorCommand.malformed := true.B
          if (enableGroupedCommands) {
            captureGrouped.get := grouped(io.in.bits)
            captureGroupId.get := groupId(io.in.bits)
          }
          drainDepth := 1.U
          if (enableGroupedCommands) {
            drainEmitTerminator.get := grouped(io.in.bits)
          }
          errorResumeDrain := true.B
          state := sError
          protocolError := true.B
        }.otherwise {
          storeBuffered(0.U, io.in.bits)
          captureCount := 1.U
          captureDepth := 1.U
          captureRegisters(0) := io.in.bits.command.microOp(9, 6)
          captureStatus := io.in.bits.command.status
          if (enableGroupedCommands) {
            captureGrouped.get := grouped(io.in.bits)
            captureGroupId.get := groupId(io.in.bits)
          }
          state := sCapture
        }
      }.elsewhen(io.in.fire && end) {
        // There is no matching region to replay.  This is not an inferred
        // group terminator: preserve only an explicitly supplied group_last.
        // With last=false the group gate aborts and drains until software's real
        // terminator.  With last=true this END already was that terminator, so
        // clearing it would leave the gate permanently waiting for another one.
        errorCommand := io.in.bits
        errorCommand.command.microOp := "h8000000e".U
        if (enableGroupedCommands) {
          errorCommand.last.get := last(io.in.bits)
        }
        errorCommand.malformed := true.B
        errorResumeDrain := false.B
        state := sError
        protocolError := true.B
      }
    }

    is(sCapture) {
      io.in.ready := true.B

      val start = isLoopStart(io.in.bits)
      val end = isLoopEnd(io.in.bits)
      val loopControl = start || end
      val startCount = io.in.bits.command.microOp(31, 10)
      val endUpper = io.in.bits.command.microOp(31, 10)
      val endRegisterMatches = captureDepth =/= 0.U &&
        io.in.bits.command.microOp(9, 6) ===
          captureRegisters(captureDepth - 1.U)
      val metadataMatches = if (enableGroupedCommands) {
        grouped(io.in.bits) === capturedGrouped &&
          groupId(io.in.bits) === capturedGroupId
      } else true.B
      val statusMatches =
        io.in.bits.command.status.asUInt === captureStatus.asUInt
      val bufferFull = captureCount === bufferEntries.U
      val badStart = start &&
        (startCount === 0.U || captureDepth === maxDepth.U)
      val badEnd = end &&
        (endUpper =/= 0.U || captureDepth === 0.U ||
          !endRegisterMatches)
      // A loop-body group_last would be accepted by Rocket during capture,
      // before its final dynamic instance has reached RS + SharedDeps.  A
      // following Gemmini opcode could then pass through the RoCC router and
      // allocate an older conflicting entry.  Require software to peel the
      // final iteration so the real last command uses the gate's direct,
      // non-bufferable admission path.
      val invalidLast = last(io.in.bits)
      val malformedCapture = io.in.bits.malformed ||
        io.in.bits.command.xd || !metadataMatches || !statusMatches ||
        bufferFull || (loopControl && last(io.in.bits)) || badStart || badEnd ||
        invalidLast
      val depthAfterOffender = Wire(UInt(32.W))
      depthAfterOffender := captureDepth
      when(start) {
        depthAfterOffender := captureDepth + 1.U
      }.elsewhen(end) {
        depthAfterOffender := Mux(captureDepth === 0.U, 0.U,
          captureDepth - 1.U)
      }

      when(io.in.fire) {
        when(malformedCapture) {
          // Discard the rest of the malformed static region locally.  At its
          // outer END, emit a synthetic malformed group terminator, so a
          // downstream group gate cannot remain in abortDrain waiting for the
          // swallowed static `last`.  An xd offender must first be forwarded
          // to produce Rocket's one expected response, then draining resumes.
          drainDepth := depthAfterOffender
          if (enableGroupedCommands) {
            drainEmitTerminator.get := true.B
          }
          when(depthAfterOffender === 0.U || io.in.bits.command.xd ||
              (!enableGroupedCommands).B) {
            errorCommand := io.in.bits
            errorCommand.command.microOp := "h8000000e".U
            if (enableGroupedCommands) {
              errorCommand.grouped.get := capturedGrouped
              errorCommand.groupId.get := capturedGroupId
              errorCommand.last.get := depthAfterOffender === 0.U
            }
            errorCommand.malformed := true.B
            errorResumeDrain := depthAfterOffender =/= 0.U
            // A standalone xd error has already produced the only required
            // architectural indication; it needs no second terminator.
            if (enableGroupedCommands) {
              when(io.in.bits.command.xd && !capturedGrouped) {
                drainEmitTerminator.get := false.B
              }
            }
            state := sError
          }.otherwise {
            state := sDrain
          }
          captureCount := 0.U
          captureDepth := 0.U
          protocolError := true.B
        }.otherwise {
          storeBuffered(captureCount(pcBits - 1, 0), io.in.bits)
          captureCount := captureCount + 1.U

          when(start) {
            captureRegisters(captureDepth) :=
              io.in.bits.command.microOp(9, 6)
            captureDepth := captureDepth + 1.U
          }.elsewhen(end) {
            when(captureDepth === 1.U) {
              // The outer matching END is now resident in the buffer.  Replay
              // starts at its START on the following cycle.
              programLength := captureCount + 1.U
              replayPc := 0.U
              replayDepth := 0.U
              captureDepth := 0.U
              state := sReplay
            }.otherwise {
              captureDepth := captureDepth - 1.U
            }
          }
        }
      }
    }

    is(sReplay) {
      val buffered = commandBuffer(replayPc)
      val current = Wire(new VpuGroupedCommand(groupIdBits,
        enableGroupedCommands))
      current := 0.U.asTypeOf(current)
      current.command.microOp := buffered.microOp
      current.command.payload := buffered.payload
      current.command.rd := 0.U
      current.command.xd := false.B
      current.command.status := captureStatus
      current.malformed := false.B
      if (enableGroupedCommands) {
        current.grouped.get := capturedGrouped
        current.groupId.get := capturedGroupId
        current.last.get := false.B
      }
      val start = isLoopStart(current)
      val end = isLoopEnd(current)
      val startCount = current.command.microOp(31, 10)
      val endRegisterMatches = replayDepth =/= 0.U &&
        current.command.microOp(9, 6) === replayRegisters(replayDepth - 1.U)
      val badReplayControl = (start &&
        (startCount === 0.U || replayDepth === maxDepth.U)) ||
        (end && (current.command.microOp(31, 10) =/= 0.U ||
          replayDepth === 0.U || !endRegisterMatches))

      when(badReplayControl) {
        errorCommand := current
        errorCommand.command.microOp := "h8000000e".U
        if (enableGroupedCommands) {
          errorCommand.last.get := true.B
        }
        errorCommand.malformed := true.B
        errorResumeDrain := false.B
        state := sError
        replayDepth := 0.U
        protocolError := true.B
      }.elsewhen(start) {
        replayStartPc(replayDepth) := replayPc + 1.U
        replayRemaining(replayDepth) := startCount
        replayRegisters(replayDepth) := current.command.microOp(9, 6)
        replayDepth := replayDepth + 1.U
        replayPc := replayPc + 1.U
      }.elsewhen(end) {
        val top = replayDepth - 1.U
        when(replayRemaining(top) > 1.U) {
          replayRemaining(top) := replayRemaining(top) - 1.U
          replayPc := replayStartPc(top)
        }.otherwise {
          replayDepth := replayDepth - 1.U
          when(replayDepth === 1.U) {
            // The outer frame completed.  No command may be accepted in this
            // same cycle; the ordinary pass-through path resumes next cycle.
            replayPc := 0.U
            programLength := 0.U
            captureCount := 0.U
            state := sPass
          }.otherwise {
            replayPc := replayPc + 1.U
          }
        }
      }.otherwise {
        io.out.valid := true.B
        io.out.bits := current
        // Legal captured commands can never terminate a grouped generation.
        if (enableGroupedCommands) {
          io.out.bits.last.get := false.B
        }
        when(io.out.fire) {
          replayPc := replayPc + 1.U
        }
      }

      assert(replayPc < programLength,
        "VPU hardware-loop replay PC escaped the captured program")
    }

    is(sDrain) {
      // Once a malformed region is being discarded, only START/END bracket
      // structure matters. Register IDs and group metadata may themselves be
      // corrupt, so consulting either could make recovery impossible.
      io.in.ready := true.B
      val start = isLoopStart(io.in.bits)
      val end = isLoopEnd(io.in.bits)
      val closesOuter = end && drainDepth === 1.U
      val nextDepth = Wire(UInt(32.W))
      nextDepth := drainDepth
      when(start) {
        nextDepth := drainDepth + 1.U
      }.elsewhen(end && drainDepth =/= 0.U) {
        nextDepth := drainDepth - 1.U
      }

      when(io.in.fire) {
        drainDepth := nextDepth
        when(io.in.bits.command.xd || closesOuter) {
          // A response-producing command cannot be silently discarded.  If
          // it is not the closing END, emit an error and resume the drain.  A
          // grouped error intentionally has last=false here: the final
          // synthetic terminator below clears the gate's abort-drain state.
          errorCommand := io.in.bits
          errorCommand.command.microOp := "h8000000e".U
          if (enableGroupedCommands) {
            errorCommand.grouped.get := capturedGrouped
            errorCommand.groupId.get := capturedGroupId
            errorCommand.last.get := closesOuter && emitTerminator
          }
          errorCommand.malformed := true.B
          errorResumeDrain := !closesOuter
          when(closesOuter && !emitTerminator &&
              !io.in.bits.command.xd) {
            state := sPass
          }.otherwise {
            state := sError
          }
          protocolError := true.B
        }
      }

      assert(drainDepth =/= 0.U,
        "VPU hardware-loop recovery entered drain mode at depth zero")
    }

    is(sError) {
      io.out.valid := true.B
      io.out.bits := errorCommand
      when(io.out.fire) {
        state := Mux(errorResumeDrain, sDrain, sPass)
        errorResumeDrain := false.B
      }
    }
  }

  io.busy := state =/= sPass
  io.protocolError := protocolError

  dontTouch(state)
  dontTouch(captureCount)
  dontTouch(captureDepth)
  dontTouch(replayPc)
  dontTouch(replayDepth)
  dontTouch(replayRemaining)
  dontTouch(drainDepth)
  dontTouch(io.busy)
  dontTouch(io.protocolError)
}
