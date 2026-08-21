# Standalone VPU

This generator is the standalone VPU used by
`GemminiComparison4x16VpuRocketConfig` and
`GemminiComparison4x16Bf16VpuRocketConfig`. The architectural interface is a
stream of 32-bit micro-ops carried by `custom0/funct7=64`.
Gemmini0 remains the wildcard `custom0` target, while exact-funct routing gives
funct 64 only to the VPU. Gemmini1--3 retain custom1--3, and funct 8/15 group
broadcast is limited to the four explicitly indexed Gemminis.

In the standalone configurations, vector data lives in a private banked SRAM.
The VPU has its own TileLink DMA reader/writer and a shared four-entry
FrontendTLB/PTW port; those configurations do not connect it to Gemmini
SharedExtMem, scratchpad, accumulator, or controllers. Their Gemmini/VPU
handoff therefore uses memory plus an explicit fence. The fused configuration
described below adds the explicit matrix/VSRAM bridge and shared scheduling
interfaces without changing the standalone ABI.

## Fused RoCC configuration

`GemminiComparison4x16Bf16FusionVpuRocketConfig` keeps the original RoCC
transport for all five accelerators. Gemmini0--3 use custom0--3, while the VPU
uses the exact custom0/funct64 route described above. The RoCC router applies
funct 8/15 group broadcast only to the four indexed Gemminis.

Fused LOOP_WS and grouped VPU commands use the same three-bit group ID. QK,
online softmax, PV, and output update for one attention block therefore share
one numeric ID; LoopMatmul's `loop_full` holds the successor matrix phase while
the current generation is allocated, and the shared dependency table orders
conflicting VSRAM ranges after allocation. A hardware-only one-bit allocation
epoch toggles on every reuse of a numeric ID. The VPU gate tracks closing IDs
as a bitmap plus those epochs, allowing ping and pong generations to overlap
without mistaking a newly allocated group for an old `dispatchReject` response.

The RoCC transport enters `VpuHardwareLoop` before that group gate. A
PLENA-compatible `C_LOOP_START`/`C_LOOP_END` region is sent once, captured in
the configurable local command buffer, and replayed by a fire-driven PC and
nested-loop stack. Replayed commands still pass through the group gate,
reservation station, and shared dependency table exactly like individually
issued commands. This removes repeated Rocket/RoCC issue gaps without adding
a second dependency mechanism or a fixed-function softmax opcode.

## Implemented v1 behavior

- Build-time FP32 or BF16 storage, with FP32 computation and scalar state.
- Fine-grained vector ADD/SUB/MUL/MIN/MAX, EXP, reciprocal, SUM, and MAX.
- Fine-grained scalar ADD/SUB/MUL/MAX, EXP, reciprocal, and square root.
- PLENA-compatible `S_ADDI_INT` (`GP[rd] = GP[rs1] + unsigned imm18`) plus
  positive-trip-count `C_LOOP_START rd,imm22`/`C_LOOP_END rd`. The default
  frontend holds 64 static commands and four nested frames. Loop-control `rd`
  is a pairing tag owned by the frontend rather than a body-visible iteration
  index; software advances address/state cursors explicitly with `S_ADDI_INT`.
  Captured commands obey ordinary ready/valid backpressure. A grouped
  `group_last` is illegal inside a loop: software peels the final iteration and
  issues its terminating data operation after `C_LOOP_END`, so group release
  still occurs only at real reservation-station + SharedDeps admission.
  The RoCC router can independently accept a following non-VPU opcode after
  the outer END has been captured. Therefore a standalone loop whose result is
  consumed by Gemmini must put a VPU `C_WAIT`/`C_FENCE` before that Gemmini
  command. Grouped fusion sequences satisfy the same ordering rule with their
  peeled VPU `group_last` operation.
  Response-producing (`xd`) commands are illegal inside a captured region,
  because Rocket could otherwise wait for a response before delivering the
  matching END.
- One architectural mask bit per vector element. `C_WRITE_VMASK` writes the
  mask in 64-bit chunks; `rs3=0` selects unmasked execution and `rs3=1`
  snapshots the current mask version. A small copy-on-write mask-slot file
  retains each live version; an execute reservation entry stores only its slot
  ID, and an active unit reads only the current `nLanes`-bit word. Ordinary VL
  iteration and its final partial-word mask are derived directly from the
  command's element count. Masked arithmetic, EXP/reciprocal, slide/gather,
  and reductions intersect an arbitrary mask word with that VL tail.
  Inactive destination elements remain unchanged, SUM supplies zero, MAX
  supplies negative infinity, and inactive elements do not contribute FP
  exceptions.
- `V_SLIDE_V` performs zero-filled element movement within the current VL.
  `funct1=0` moves toward higher indices
  (`dst[i]=i>=shift?src[i-shift]:0`); `funct1=1` moves toward lower indices
  (`dst[i]=i+shift<VL?src[i+shift]:0`). The shift is read from `GP[rs2]`, and
  direction-aware traversal permits an in-place source/destination.
- `V_GATHER_VV` treats each raw storage-width element in the `rs2` VSRAM
  vector as an unsigned index relative to the `rs1` vector. An index outside
  VL produces positive zero. The destination may not alias either source.
  This is a correctness-first unit which reads one index word and then one
  source word per active destination lane; it deliberately does not add a
  multiported gather register file or VSRAM crossbar.
- Eight-bank 1R1W SRAM with independent source-0, source-1, and store read
  streams. Different-bank requests proceed in parallel, same-bank requests
  are serialized by a bank-local round-robin arbiter, and equal VV source
  addresses require one read whose response is broadcast to both operands.
- Tagged base-vector ALU streaming issues one lane word per cycle for
  different-bank operands, keeps all HardFloat lanes aligned through the
  three-cycle FMA pipe, and sustains one writeback per cycle. A true same-bank
  VV pair uses the required two-cycle read cadence.
- RED_SUM/MAX time-multiplex those same physical FMA lanes as a registered
  `n/2, n/4, ..., 1` tree plus one recurrence lane (the default is
  `8/4/2/1 + 1`). Three rotating accumulators with distance-three forwarding
  accept one SRAM word per cycle and remove the former combinational tree.
- Vector EXP/reciprocal prefetch SRAM words into a bounded input FIFO. EXP
  issues `sfuLanes` elements/cycle; reciprocal statically partitions the
  shared FMA fabric and issues `nLanes/4` elements/cycle for FP32's two Newton
  refinements. Complete SRAM words are reassembled in a bounded writeback
  FIFO. Scalar EXP uses EXP lane zero, while scalar reciprocal uses group zero
  of the same shared FMA fabric.
- A partitioned, register-based LD/EX/ST reservation station holds commands
  from allocation through engine-local completion. The default live capacities
  are 4 LD, 9 EX, and 4 ST entries; the ninth EX entry preserves the former
  eight-waiting-plus-one-active credit. Every entry records one global-tag
  dependency bitmap, with no dispatch age or wrapping sequence counter.
  Same-class, non-conflicting ordering edges clear when the older command
  issues. Actual same-class or cross-class RAW/WAR/WAW edges clear only when
  that producer completes, including overlapping LD/LD writes. Independent
  ready LD, EX, and ST entries can therefore issue concurrently.
- `C_WAIT` snapshots the live reservation tags in the selected LD/EX/ST
  classes and completes when those exact tags retire. `C_FENCE` snapshots all
  live tags and additionally drains active engines, the SRAM/merge paths, and
  DMA transport state. In particular, a store transport tag remains busy until
  its final TileLink D response, so FENCE cannot complete at the earlier store
  reservation-retirement point.
- Descriptor plus 128-bit beat DMA interface. Like Gemmini, command/control,
  external-memory request generation, and physical SRAM transfer are separate
  stages: Core snapshots an RS command into a descriptor, the TileLink
  reader/writer independently generates requests, and the merge/store streams
  arbitrate for VSRAM only when data is ready. Reader and writer use the
  configured `dmaMaxInFlight` TileLink source slots, preserve descriptor stream
  order across out-of-order responses, and split at beat, transaction, and
  translation boundaries.
- `C_SET_STRIDE` records a host byte stride. With `funct1=1`,
  `H_PREFETCH_V`/`H_STORE_V` read row count from `GP[rs3]` and snapshot it with
  H/GP addresses, VL, and stride at dispatch. For row `r`, the external address
  is `base + offset*storageBytes + r*hostStrideBytes`, while the local address
  is `vspadBase + r*VLEN`; current VL is the useful element count in every row.
  A descriptor is limited to `VPU_DMA_MAX_ROWS`, and its VLEN-spaced local
  footprint must remain in one bank. Multi-row host stride is storage-element
  aligned. The legacy `funct1=0,rs3=0` form remains a one-row transfer and
  ignores the configured stride.
- The load path merges tagged 128-bit response fragments into one complete
  `nLanes`-element VSRAM row before arbitration. A full row performs one
  all-lane write; the architectural tail performs one masked row write, rather
  than one masked SRAM write for every incoming DMA fragment. A load
  reservation retires only after its final merged row is accepted by VSRAM.
- A store reservation retires when its final VSRAM read request is accepted,
  matching Gemmini's local-memory dependency lifetime. Returned SRAM data,
  serialization, TileLink A requests, and TileLink D responses continue under
  an independent store transport tag, which is released only at TL completion.
- Sticky status/fflags, fence/read responses, fault address/cause capture, and
  the public performance-counter ABI.
- Faulted reads suppress all remaining SRAM writes, drain accepted response
  streams, atomically flush unissued reservation entries, and preserve
  response-bearing READ/FENCE commands until software completes the explicit
  core/wrapper/TLB clear handshake. The first virtual fault address, cause,
  direction, and wrapper-halted state are readable.
- Saturn-style three-cycle visible FMA wrapper around Rocket HardFloat,
  synthesizable table/polynomial EXP, a Saturn VFREC7 seed followed by a
  four-stage two-Newton mode in the shared FMA fabric, and exact iterative
  HardFloat scalar square root.

## Deliberate v1 scheduling simplifications

These points are architectural implementation details, not ISA differences.
They are recorded here so later throughput work does not need to rediscover
them.

- Each class has one command-facing issue engine behind its reservation
  partition. Thus multiple commands can remain allocated, but v1 presents at
  most one new load descriptor and one new store descriptor at each respective
  core-facing DMA port at a time. The TL wrapper can still have multiple
  requests from accepted descriptors in flight.
- As in Gemmini, the FrontendTLB request uses `size=0` only to translate the
  first byte. The independently generated TileLink A request retains its real
  naturally aligned size, up to `dmaMaxBytes=64`, and page-boundary splitting
  guarantees that it never reaches into the next translation. `TLWidthWidget`
  transfers those requests as 128-bit beats, with up to `dmaMaxInFlight`
  source IDs outstanding independently in each direction.
- Execute/load writes and source-0/source-1/store reads enter independent
  clients of the bank-local arbiter. Requests to disjoint banks are accepted
  together; same-bank contenders alternate fairly. Each read client has its own
  credit-protected response queue, and every physical bank still infers
  exactly one read and one masked-write mport. The mask remains necessary for
  architectural tails and execute tails; normal DMA loads reach that port as
  one merged, all-lane write per VSRAM row.
- Because the arithmetic pipelines have no output backpressure, their fixed
  four-word writeback FIFOs reserve credit for both queued and in-flight
  results before issue. This prevents result loss under temporary SRAM
  write-port contention without scaling buffer capacity with architectural
  VLEN.
- Q/DQ is not implemented. Standalone configurations still require the memory
  round trip described above. The BF16 fused configuration instead exposes
  Gemmini matrix reads and FP32 VSRAM writes through the shared matrix/VSRAM
  bridge; it does not add an INT8 quantize/dequantize path.
- DMA faults are polled through status/read selectors; v1 does not raise a VPU
  interrupt.

## Generated parameters and software

Every VPU configuration writes its exact software ABI directly to
`generators/gemmini/software/gemmini-rocc-tests/include/vpu_params.h` during
Chipyard elaboration. There are no FP32/BF16/fusion selector headers or
fallback defaults: like Gemmini's `gemmini_params.h`, this one file always
describes the most recently elaborated hardware configuration. Elaborate the
intended standalone, BF16-storage, or fusion Chipyard config before compiling
software for it; elaborating a different config intentionally replaces the
file. `VPU_HAS_GENERATED_PARAMS` is always defined by the generated header.

`vpu.h` provides uop packing, register access,
mask programming and masked operations, slide/gather, one- and two-dimensional
prefetch/store, hardware-loop capture/address-induction, wait/fence/status, and
performance-counter helpers. The 2-D
helpers require distinct GP registers for the VSRAM address, host offset, and
row count; the supplied test/example uses GP0, GP1, and GP2 respectively.
Software calls `vpu_publish_cpu_writes()` once after producing input buffers;
individual prefetch helpers deliberately do not issue a RISC-V fence because
Rocket waits for every RoCC to become idle at that instruction, which would
serialize ping/pong DMA and execution.
`vpu_kernels.h` emits only fine-grained instructions for tiled RMSNorm/final
norm, ReLU, sigmoid, tanh, PLENA-style approximate GELU
(`x * sigmoid(1.702*x)`), SiLU, SwiGLU, stable softmax, RoPE, and VL-local
permutation. For each equal-VL run, one 2-D DMA descriptor transfers up to
`min(VPU_NONLINEAR_CHUNK_ROWS, VPU_SLOTS_PER_BANK, VPU_DMA_MAX_ROWS,
VPU_LOOP_COUNT_MAX)` consecutive rows into one bank for nonlinear kernels.
`VPU_NONLINEAR_CHUNK_ROWS` is a software-only policy in `vpu_kernels.h`
(default 4) and may be defined by an application before including that header.
It is deliberately absent from `VpuParams` and the generated `vpu_params.h`;
those expose only physical hardware limits. Rearrangement kernels retain the
full physical bank-sized batch because their per-row gather/slide work already
sustains the pipeline. A
hardware loop captures one row body, replays it for all rows, and uses
`S_ADDI_INT(+VLEN)` for VSRAM address induction. The next ping/pong batch is
prefetched before current replay begins. The final partial-VL row is peeled
because every row of one DMA descriptor snapshots the same VL. Replayed
operations still pass through the normal reservation station, SharedDeps, and
backpressure; this reduces Rocket/RoCC issue traffic, not the number of VPU
arithmetic operations.

RMSNorm and softmax choose full-resident, resident-prefix/streaming, or pure
streaming schedules from the generated VSRAM geometry. Resident data is kept
across global reduction passes, while both resident and overflow work is issued
in bounded nonlinear chunks. Elementwise activation uses the same bounded
ping/pong chunks; RoPE and local-permute use physical bank-sized batched
ping/pong streaming and fence before returning. The ping/pong convention
occupies banks 0--3 and 4--7 and keeps descriptors independent by snapshotting
GP/H/VL state. `vpu_rope_auto` supports interleaved and NeoX layouts with one
expanded cosine/sine table shared by the supplied rows; `vpu_permute_auto`
implements one reusable VL-local gather map per contiguous group (not an
arbitrary cross-group permutation). The nonlinear bare-metal test allocation
defaults to 20,000 elements and can be changed with
`VPU_NONLINEAR_MAX_ELEMENTS`; its detailed CPU/VPU timing row is selected
separately with `VPU_NONLINEAR_BENCHMARK_LENGTH`.
VPU test binaries are compiled with strict floating-point semantics rather than
the Gemmini suite's global `-ffast-math` setting.

The fused FlashAttention bare-metal scheduler now follows PLENA's resident-mask
organization. It DMA-loads one packed `DIM x DIM` `0/-Inf` triangular score
mask into a reserved VSRAM tile. For a diagonal block, a hardware row loop
advances both the score-row and mask-row addresses and performs `V_ADD_VV`;
complete future score tiles are overwritten with `-Inf`. A second homogeneous
row loop then executes online softmax for strict-past, diagonal, and partially
visible rows alike. Runs split at matrix-tile and causal-mask row boundaries,
so non-DIM tails and a nonzero query base remain correct without reverting to
one Rocket-issued command sequence per row.

The same schedule is exposed as the reusable, header-only
`vpu_flashattention_auto()` kernel. After elaborating the fusion config,
applications include the active `gemmini_params.h`, define
`VPU_ENABLE_GEMMINI_FLASHATTENTION=1`, and include `vpu_kernels.h`.
The active `vpu_params.h` is included by `vpu.h`; no selector macro or
fusion-specific parameter filename is required.
`vpu_flashattention_config_t` supplies runtime Q/K/V/output pointers, logical
dimensions, element strides, the global query-position base, and the Gemmini
mask. A zero stride selects packed storage. The caller also supplies one
aligned `DIM x DIM` FP32 causal-mask workspace whose lifetime extends through
the call. `vpu_flashattention_make_plan()` exposes the automatically selected
query/KV/depth tiling without issuing commands; `vpu_flashattention_auto()`
executes the grouped QK--online-softmax--PV schedule, stores FP32 output, waits
for the final VPU fence, and returns the selected plan, scheduling statistics,
VPU status, and a typed software status. The API currently implements one
causal attention head per call and requires the BF16-Gemmini/FP32-accumulator,
FP32-VPU fusion configuration. Q/K/V are read-only and may alias one another;
output and the causal-mask workspace must be disjoint from each other and from
all input ranges. Calls are synchronous but not concurrently re-entrant, so
the caller must exclusively own the selected Gemminis and VPU until return.

```c
#include "gemmini_params.h"
#define VPU_ENABLE_GEMMINI_FLASHATTENTION 1
#include "vpu_kernels.h"

float causal_mask[VPU_FLASHATTENTION_CAUSAL_MASK_ELEMENTS]
    __attribute__((aligned(64)));

vpu_flashattention_config_t config = {
    .queries = q, .keys = k, .values = v, .output = o,
    .query_rows = m, .sequence = n,
    .q_dim = d, .k_dim = d, .value_dim = dv,
    .score_scale = 1.0f / sqrtf((float)d),
    .query_base = query_position,
    .query_stride = d, .key_stride = d,
    .value_stride = dv, .output_stride = dv,
    .gemmini_mask = 0xf,
    .causal_mask_workspace = causal_mask,
    .causal_mask_workspace_elements =
        VPU_FLASHATTENTION_CAUSAL_MASK_ELEMENTS,
};
vpu_flashattention_result_t result = vpu_flashattention_auto(&config);
```

The stable-softmax helper follows the literal IEEE sequence `x-max`, EXP, SUM,
and reciprocal. It is stable for finite logits and maps isolated `-Inf` logits
to zero when another finite maximum exists. NaN, `+Inf` (through `Inf-Inf`), or
an all-`-Inf` row propagates canonical NaN and the applicable sticky flags; v1
has no vector classify/compare instruction or device-side mask construction
with which to define a one-hot infinity policy from the input values.

## Tests

From the Chipyard root:

```sh
source env.sh
sbt 'project vpu' test
```

### Per-operation Rocket CPU versus VPU benchmarks

`vpu_nonlinear-baremetal` and `vpu_flashattention-baremetal` remain broad
functional and numerical tests.  The performance programs deliberately put
one operation in each binary so that a slow scalar-libm reference can be run
only for the operation of interest.  These names are both the Make target
bases and the corresponding `bareMetalC/<name>.c` source names:

- `rmsnorm_vs_CPU_perf`, `final_norm_vs_CPU_perf`, `relu_vs_CPU_perf`,
  `sigmoid_vs_CPU_perf`, `tanh_vs_CPU_perf`, `gelu_vs_CPU_perf`,
  `silu_vs_CPU_perf`, `swiglu_vs_CPU_perf`, `softmax_vs_CPU_perf`,
  `rope_vs_CPU_perf`, and `permute_vs_CPU_perf` use the standalone VPU.
- `vpu_flashattention_vs_CPU_perf` uses the BF16 Gemmini/FP32-accumulator
  fusion configuration and compares scalar causal attention against the
  complete fused QK--online-softmax--PV kernel.

The ordinary nonlinear benchmark sizes are compile-time parameters, like the
shape parameters in `llama_shape_test0.c`.  `VPU_PERF_ELEMENTS` sets the
one-dimensional problem length.  RoPE instead uses
`VPU_PERF_ROPE_ROWS` and `VPU_PERF_ROPE_DIM`, with layout 0 for interleaved and
1 for NeoX.  Permute uses `VPU_PERF_PERMUTE_GROUPS` and
`VPU_PERF_PERMUTE_GROUP_ELEMENTS`; one group's width may not exceed the
generated `VPU_VLEN`.  `VPU_PERF_IN_PLACE=1` selects the supported in-place
form for RoPE and permute.  The remaining controls are
`VPU_PERF_ITERATIONS`, `VPU_PERF_WARMUP_ITERATIONS`, `VPU_PERF_CHECK`,
`VPU_PERF_RMSNORM_EPSILON`, and `VPU_PERF_FINAL_NORM_EPSILON`.

For example, this builds all standalone per-operation programs for a 2,048
element Llama hidden vector.  Any resulting binary can then be selected
independently for RTL simulation.

```sh
cd generators/gemmini/software/gemmini-rocc-tests
./build.sh -B vpu-perf-baremetal \
  VPU_PERF_ELEMENTS=2048 \
  VPU_PERF_ITERATIONS=1 \
  VPU_PERF_WARMUP_ITERATIONS=1 \
  VPU_PERF_CHECK=1

cd ../../../..
make -C sims/verilator \
  CONFIG=GemminiComparison4x16VpuRocketConfig \
  run-binary-debug \
  BINARY="$PWD/generators/gemmini/software/gemmini-rocc-tests/build/bareMetalC/rmsnorm_vs_CPU_perf-baremetal"
```

FlashAttention retains the existing compile-time Q/K/V shape controls and has
separate repetition controls.  A typical fused comparison is:

```sh
cd generators/gemmini/software/gemmini-rocc-tests
./build.sh -B fusion-vpu-perf-baremetal \
  FA_QUERY_ROWS=64 FA_SEQUENCE=256 \
  FA_Q_DIM=64 FA_K_DIM=64 FA_V_DIM=64 \
  FA_QUERY_BASE=192 FA_GEMMINI_MASK=0xf \
  FA_PERF_WARMUPS=0 FA_PERF_REPEATS=1 FA_PERF_CHECK=1

cd ../../../..
make -C sims/verilator \
  CONFIG=GemminiComparison4x16Bf16FusionVpuRocketConfig \
  run-binary-debug \
  BINARY="$PWD/generators/gemmini/software/gemmini-rocc-tests/build/bareMetalC/vpu_flashattention_vs_CPU_perf-baremetal"
```

Both sides use `rdcycle`, with input initialization, warm-up, checksums,
checking, counter reads, and UART output outside the measured intervals.  The
CPU interval contains only the scalar FP32 reference body (including scalar
libm where required).  The VPU interval contains the complete synchronous
`*_auto()` call, including command/configuration issue, DMA, execution, output
store, and final fence.  Thus the reported `cpu_over_vpu` or
`cpu_over_fusion` value is an end-to-end speedup, not a comparison of arithmetic
pipelines alone.  Checksums keep both computations observable to the compiler,
and `*_PERF_CHECK=1` validates the final result outside the timing interval.

The standalone programs also print a unit-issue lower bound named
`ideal_compute_per_iteration`.  It is a diagnostic based on the generated
lane counts.  RoPE additionally includes the slide FSM's minimum per-word
states, and permute includes gather's serialized per-destination source reads;
those two operations therefore are not modeled as ordinary one-word/cycle
ALU instructions.  The bound intentionally excludes DMA, command and fence
overhead, pipeline fill/dependencies, SRAM conflicts, scalar/reduction
folding, and backpressure, so `vpu_e2e_over_ideal` is not expected to approach
one for every kernel.  The printed VPU performance counters diagnose busy,
SFU, DMA traffic/overlap, bank
conflict, and hazard time.  They are not the primary end-to-end timer: the
standalone counter scope is all measured VPU iterations, while the fused
FlashAttention line explicitly reports the last VPU iteration only.  Use the
`rdcycle` totals for CPU/VPU speedup comparisons.

The suite checks ISA/parameter geometry, strict unused-field validation, SRAM
bank conflicts, FMA/reciprocal latency, signed-zero/NaN behavior, EXP accuracy
and flags, reciprocal/sqrt special cases, masked-tail reductions, randomized
bit-exact rotating-accumulator order, BF16 conversion, and FP32/BF16
elaboration. Mask tests cover per-element holes, VL intersection, dispatch-time
mask snapshots, inactive-destination preservation, reduction neutral values,
and suppressed inactive exceptions. Rearrangement tests cover both zero-fill
slide directions across SRAM words, in-place traversal, masked gather,
duplicate and out-of-range indices, and forbidden gather aliases. Its
Verilator integration tests execute
an SRAM-resident fine-grained SiLU sequence; saturate reservation capacity under
held ready/valid; verify simultaneous disjoint LD/EX/ST progress, dependency-bit
issue/completion release, class-selective valid-tag WAIT, and transport-draining
FENCE; exercise merged full-row and masked-tail loads plus the final legal SRAM
row; and check first-fault capture plus DMA drain/flush/clear recovery. A
separate BF16 core test rounds its SiLU reference after every vector micro-op,
compares raw BF16 results exactly, checks mathematical-gold tolerance, and
verifies that a partial-VL tail leaves the rest of the SRAM row unchanged.
Dedicated scheduler tests sustain EXP and reciprocal issue/result II=1 across
four SRAM words. FP32 and BF16 Core integration tests stream multiword
reductions from the live banked SRAM through the shared FMA fabric into scalar
state. A store-lifetime test separately proves that `C_WAIT(ST)` and the store
RS entry finish at the final physical VSRAM read request, while `C_FENCE`
remains blocked until the corresponding TileLink D completions arrive.
Core-level 2-D DMA tests check GP/stride/VL/address snapshotting, VLEN-spaced
local rows, partial row tails, maximum-one-bank footprints, malformed row
counts and strides, and legacy one-row encodings. TileLink tests check distinct
host strides, row-address generation, later-row fault reporting/drain, and
multi-row load merging and store completion. The TileLink multi-source test
fills multiple source IDs, deliberately returns
responses in reverse request order, checks ordered read assembly, and verifies
earliest-fault reporting for writes. The banked-SRAM test also checks the
generated CHIRRTL structurally so a client-arbitration edit cannot silently
turn a 1R1W bank into a multi-writer memory.
`VpuHardwareLoopSpec` additionally checks pass-through behavior, nested replay,
output backpressure, rejection of captured `group_last`, zero counts,
unmatched/misnested delimiters, metadata mismatches, duplicate terminators,
stack overflow, and command-buffer overflow recovery.

The host-side `vpu_kernel_schedule_test.c` traces an auto-scheduled ReLU large
enough to reuse ping after pong. It checks that every full-row chunk uses one
2-D load, one captured hardware loop, two useful address increments, and one
2-D store, while the final partial-VL row is peeled into ordinary 1-D
descriptors. A fixed 2,048-element trace additionally checks the generated
chunk bound, next-load-before-current-execute lookahead, and the independent
RMSNorm/softmax output rings. The nonlinear RTL program also exercises every
auto kernel at
lane/VLEN tails and arbitrary multi-row lengths; its 10,000/20,000 analytic
cases avoid scalar-libm simulation cost while still running the VPU datapaths.
The `vpu_flashattention` bare-metal program is now only a numerical driver for
the public kernel: compile-time test dimensions populate a runtime config,
while input generation, the block-matched BF16 reference, the mathematical
reference, and result checking remain outside the kernel. Reference-enabled
RTL regressions cover both a 16x16 base case and an irregular case with 17
query rows, sequence length 37, Q/K depth 48, V width 20, and a nonzero query
base.

```sh
cd generators/gemmini/software/gemmini-rocc-tests
./build.sh -B fusion-vpu-baremetal \
  FA_QUERY_ROWS=17 FA_SEQUENCE=37 \
  FA_Q_DIM=48 FA_K_DIM=48 FA_V_DIM=20 \
  FA_QUERY_BASE=20 FA_GEMMINI_MASK=0xf FA_CHECK_REFERENCE=1

cd ../../../..
make -C sims/verilator \
  CONFIG=GemminiComparison4x16Bf16FusionVpuRocketConfig \
  run-binary-debug \
  BINARY="$PWD/generators/gemmini/software/gemmini-rocc-tests/build/bareMetalC/vpu_flashattention-baremetal"
```

The `vpu_mask_rearrange` bare-metal program checks the public C API across
noncontiguous masks, masked SUM/MAX, signaling-NaN suppression, cross-word and
in-place slides, and raw-index gathers. `vpu_strided_dma` performs a maximum
one-bank 2-D load/store with different source and destination pitches,
VL tails, inter-row/outer guards, DMA byte accounting, and a legacy 1-D
compatibility case. Both programs build for FP32 and BF16 storage and remain
outside the Spike list.

The elaboration suite covers the full advertised cross-product
`vLen={64,128,256}`, `nLanes={8,16}`, `sfuLanes={2,4}`, and FP32/BF16 storage.
The bare-metal programs are separate from the default Spike list because Spike
does not model this custom VPU. The generated `vpu_params.h` must match the
selected RTL config. Run the binaries only on the VPU RTL simulator (or under
PK on that simulator); there is intentionally no Spike run target.

```sh
cd generators/gemmini/software/gemmini-rocc-tests
./build.sh
```

With a Linux cross-compiler installed, that one command creates bare-metal,
Linux, and PK binaries together under `build/bareMetalC`; otherwise it builds
the bare-metal variants. The default build subdirectories remain the legacy
`bareMetalC`, `imagenet`, `transformers`, and `mlps`. Explicit `vpu-*` and `fusion-vpu-*`
targets are still available when only one subset should be rebuilt, but they
write into the same `build/bareMetalC` directory.

A full FP32 or BF16-storage SoC generation/build uses:

```sh
source env.sh
make -C sims/verilator CONFIG=GemminiComparison4x16VpuRocketConfig
make -C sims/verilator CONFIG=GemminiComparison4x16Bf16VpuRocketConfig
```

For memory-constrained hosts, direct Chipyard generation may need a larger JVM
heap, for example `JAVA_TOOL_OPTIONS='-Xmx8G -Xss8M'`.

## Attribution and v1 boundary

`VpuReciprocal.scala` adapts Saturn's VFREC7 seed and retains the upstream
BSD-3-Clause copyright, conditions, disclaimer, source path, and pinned commit
identifier. The FMA wrapper uses Rocket's existing HardFloat
`MulAddRecFNPipe`; PLENA informs the scheduling/SRAM/microinstruction
organization, not copied RTL.

The standalone v1 configurations intentionally have no scale SRAM, Matrix
SRAM, Q/DQ, local-matmul change, or SPAD/ACC bridge. The separate BF16 fusion
configuration adds the matrix/VSRAM bridge and a FlashAttention validation
path, but still has no Q/DQ or INT8 fused data path. These distinctions must
not be interpreted as a general Llama E2E speedup claim for this revision.
