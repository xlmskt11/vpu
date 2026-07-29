// BSD 3-Clause License
//
// Copyright (c) 2024, The Regents of the University of California (Regents)
// All Rights Reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

package vpu

import chisel3._
import chisel3.util._

/** FP32 specialization of Saturn's architectural VFREC7 seed.
  *
  * The table and boundary construction are adapted from Saturn's VFREC7 in
  * `src/main/scala/exu/fp/FPDiv.scala` at commit
  * d44530e8d10a706aecd42911513a07f9dc786c65. Fixed RNE is used by VPU v1.
  * Newton arithmetic is performed by the shared lanes in
  * [[VpuVectorFmaFabric]]; this file intentionally contains no FMA pipeline.
  */
class VpuRec7Seed extends Module {
  val io = IO(new Bundle {
    val in = Input(UInt(32.W))
    val result = Output(new VpuFpResult)
    val refine = Output(Bool())
  })

  private val table = Seq(
    127, 125, 123, 121, 119, 117, 116, 114,
    112, 110, 109, 107, 105, 104, 102, 100,
    99, 97, 96, 94, 93, 91, 90, 88,
    87, 85, 84, 83, 81, 80, 79, 77,
    76, 75, 74, 72, 71, 70, 69, 68,
    66, 65, 64, 63, 62, 61, 60, 59,
    58, 57, 56, 55, 54, 53, 52, 51,
    50, 49, 48, 47, 46, 45, 44, 43,
    42, 41, 40, 40, 39, 38, 37, 36,
    35, 35, 34, 33, 32, 31, 31, 30,
    29, 28, 28, 27, 26, 25, 25, 24,
    23, 23, 22, 21, 21, 20, 19, 19,
    18, 17, 17, 16, 15, 15, 14, 14,
    13, 12, 12, 11, 11, 10, 9, 9,
    8, 8, 7, 7, 6, 5, 5, 4,
    4, 3, 3, 2, 2, 1, 1, 0)

  val sign = io.in(31)
  val exponent = io.in(30, 23)
  val fraction = io.in(22, 0)
  val isZero = exponent === 0.U && !fraction.orR
  val isSubnormal = exponent === 0.U && fraction.orR
  val isInf = exponent.andR && !fraction.orR
  val isNaN = exponent.andR && fraction.orR
  val isSignalingNaN = isNaN && !fraction(22)

  val leadingZeros = PriorityEncoder(Reverse(fraction))
  val normalizedExponent = WireDefault(exponent)
  val normalizedFraction = WireDefault(fraction)
  when(isSubnormal) {
    normalizedExponent := exponent - leadingZeros
    normalizedFraction := (fraction << (leadingZeros +& 1.U))(22, 0)
  }

  val index = normalizedFraction(22, 16)
  val lookup = VecInit(table.map(_.U(7.W)))(index)
  val defaultFraction = Cat(lookup, 0.U(16.W))
  val exponentWide = 254.U(9.W) +& (~normalizedExponent).asUInt
  val defaultExponent = exponentWide(7, 0)
  val adjustedFraction = (defaultFraction >> 1) | (1.U << 22)
  val estimateExponent = WireDefault(defaultExponent)
  val estimateFraction = WireDefault(defaultFraction)
  when(defaultExponent === 0.U || defaultExponent.andR) {
    estimateFraction := adjustedFraction
    when(defaultExponent.andR) {
      estimateExponent := 0.U
      estimateFraction := adjustedFraction >> 1
    }
  }
  val estimate = Cat(sign, estimateExponent, estimateFraction)

  // For the smallest subnormals the correctly rounded reciprocal overflows.
  // Saturn detects this through wrapped normalized exponent values.
  val abnormalOverflow = isSubnormal &&
    normalizedExponent =/= 0.U && !normalizedExponent.andR

  io.refine := !isZero && !isInf && !isNaN && !abnormalOverflow
  io.result.data := Mux(isNaN, VpuFloat.canonicalNaN.U(32.W),
    Mux(isZero, Cat(sign, VpuFloat.positiveInfinity.U(31.W)),
      Mux(isInf, Cat(sign, 0.U(31.W)),
        Mux(abnormalOverflow,
          Cat(sign, VpuFloat.positiveInfinity.U(31.W)), estimate))))
  io.result.fflags := Mux(isSignalingNaN, "b10000".U(5.W),
    Mux(isZero, "b01000".U(5.W),
      Mux(abnormalOverflow, "b00101".U(5.W), 0.U(5.W))))
}

private[vpu] class VpuReciprocalContext extends Bundle {
  val operand = UInt(32.W)
  val estimate = UInt(32.W)
  val specialResult = UInt(32.W)
  val flags = UInt(5.W)
  val refine = Bool()
}
