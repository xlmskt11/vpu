# Standalone VPU

This generator is the standalone VPU used by
`GemminiComparison4x16VpuRocketConfig` and
`GemminiComparison4x16Bf16VpuRocketConfig`. The architectural interface is a
stream of 32-bit micro-ops carried by `custom0/funct7=64`.
Gemmini0 remains the wildcard `custom0` target, while exact-funct routing gives
funct 64 only to the VPU. Gemmini1--3 retain custom1--3, and funct 8/15 group
broadcast is limited to the four explicitly indexed Gemminis.

Vector data lives in a private banked SRAM. The VPU has its own TileLink DMA
reader/writer and a shared four-entry FrontendTLB/PTW port; it is not connected
to Gemmini SharedExtMem, scratchpad, accumulator, or controllers. Gemmini/VPU
handoff in v1 therefore uses memory plus an explicit fence.

## Implemented v1 behavior

- Build-time FP32 or BF16 storage, with FP32 computation and scalar state.
- Fine-grained vector ADD/SUB/MUL/MIN/MAX, EXP, reciprocal, SUM, and MAX.
- Fine-grained scalar ADD/SUB/MUL/MAX, EXP, reciprocal, and square root.
- One architectural mask bit per vector element. `C_WRITE_VMASK` writes the
  mask in 64-bit chunks; `rs3=0` selects unmasked execution and `rs3=1`
  snapshots the current mask into the reservation entry. Masked arithmetic,
  EXP/reciprocal, slide/gather, and reductions intersect the mask with VL.
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
- Eight-bank 1R1W SRAM with broadcast, different-bank parallel reads, and
  same-bank/different-row read serialization.
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
- Execute/load writes and execute/store reads enter independent clients of the
  bank-local arbiter. Requests to disjoint banks are accepted together;
  same-bank contenders alternate fairly. Each read client has its own
  credit-protected response queue, and every physical bank still infers
  exactly one read and one masked-write mport. The mask remains necessary for
  architectural tails and execute tails; normal DMA loads reach that port as
  one merged, all-lane write per VSRAM row.
- Because the EXP/reciprocal pipelines have no output backpressure, their
  writeback FIFO reserves one architectural vector of register storage. This
  keeps long-vector II=1 independent of temporary SRAM write-port contention.
- Q/DQ, Matrix SRAM, and direct Gemmini SPAD/accumulator bridges are not
  accepted by this revision. Gemmini/VPU exchange therefore still requires
  the memory round trip described above.
- DMA faults are polled through status/read selectors; v1 does not raise a VPU
  interrupt.

## Generated parameters and software

Constructing the FP32 SoC writes the selected parameters to
`generators/gemmini/software/gemmini-rocc-tests/include/vpu_params_generated.h`;
the BF16-storage SoC writes the distinct
`vpu_params_bf16_generated.h`. Keeping both artifacts prevents an elaboration
of one storage ABI from silently changing binaries for the other. The stable
`vpu_params.h` wrapper selects the FP32 header by default, while
`VPU_PARAMS_GENERATED_HEADER` selects BF16 or a relocated generated file.
Without either header, software receives documented FP32 defaults.

Software that must never use fallback defaults can check
`VPU_HAS_GENERATED_PARAMS`. `vpu.h` provides uop packing, register access,
mask programming and masked operations, slide/gather, one- and two-dimensional
prefetch/store, wait/fence/status, and performance-counter helpers. The 2-D
helpers require distinct GP registers for the VSRAM address, host offset, and
row count; the supplied test/example uses GP0, GP1, and GP2 respectively.
Software calls `vpu_publish_cpu_writes()` once after producing input buffers;
individual prefetch helpers deliberately do not issue a RISC-V fence because
Rocket waits for every RoCC to become idle at that instruction, which would
serialize ping/pong DMA and execution.
`vpu_kernels.h` emits only fine-grained instructions for tiled RMSNorm/final
norm, SiLU, SwiGLU, and stable softmax. Its ping/pong convention occupies banks
0--3 and 4--7 and keeps descriptors independent by snapshotting GP/H/VL state.
VPU test binaries are compiled with strict floating-point semantics rather than
the Gemmini suite's global `-ffast-math` setting.

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
does not model this custom VPU. The forwarding targets below keep binaries for
the two storage ABIs in separate directories. Run them only on the VPU RTL
simulator (or under PK on that simulator); there is intentionally no Spike run
target.

```sh
cd generators/gemmini/software/gemmini-rocc-tests
./build.sh vpu-baremetal VPU_BUILD_DIR=vpu-fp32
./build.sh vpu-pk VPU_BUILD_DIR=vpu-fp32

# The BF16 build must select the BF16-storage ABI header explicitly.
./build.sh vpu-baremetal VPU_BUILD_DIR=vpu-bf16 \
  VPU_GENERATED_HEADER=vpu_params_bf16_generated.h
./build.sh vpu-pk VPU_BUILD_DIR=vpu-bf16 \
  VPU_GENERATED_HEADER=vpu_params_bf16_generated.h
```

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

There is intentionally no scale SRAM, Matrix SRAM, Q/DQ, flash attention,
Gemmini local-matmul change, or SPAD/ACC bridge in v1. Those omissions must not
be interpreted as a Llama E2E speedup claim for this revision.
