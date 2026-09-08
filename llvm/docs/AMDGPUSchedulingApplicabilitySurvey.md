# Where GPU instruction scheduling matters — independent evidence survey

2026-09-08. Answers two questions: (1) which GPU kernels/classes receive
heavy optimization attention generally, and (2) where compiler
instruction scheduling specifically has leverage — as a *fresh outside
view*, deliberately not derived from this project's own prior
investigations, so it can later be compared against where we have
already been looking (and against HeCBench) without circularity.

## 0. Methodology: why this was run blind

A first attempt at question (2) was contaminated: it was written after
(and anchored on) this project's own amenable-kernel criteria and
benchmark history, and produced essentially the same list with the
serial numbers filed off. It was discarded.

The survey was then re-run as three parallel agents, each receiving
ONLY its question — no conversation history, no project files, web
evidence only:

- **Front 1**: where does instruction ORDERING itself matter (fixed
  occupancy: load-to-use issue distance, waitcnt/scoreboard placement,
  clause formation, cross-pipe interleave, dual-issue packing)?
- **Front 2**: where does the scheduling <-> register-pressure <->
  occupancy coupling decide performance, and what kind of code creates
  the pressure?
- **Front 3**: where does scheduling-adjacent effort OBSERVABLY go —
  an inventory of artifacts (autotuner suites, vendor case studies,
  graphics talks, compiler trackers, hand-written assembly), no
  opinions about importance.

Evidence-quality rules: mechanistic evidence (ISA docs, vendor
engineering write-ups, compiler source/discussion, reproducible
practitioner reports) preferred over published speedup claims; academic
numbers used only where methodology is inspectable. Lab judgment
applied at synthesis: the ACO-group scheduling papers (Shobaki et al.,
incl. CGO'24) are treated as prior art only — their evaluations are
not trusted, so none of their numbers back any magnitude claim below.
(Both technical agents found the paper independently, which at least
confirms the blinding held.)

## 1. Context: kernel classes with heavy optimization attention generally

Conventional-knowledge baseline (not part of the blind evidence): dense
GEMM family; convolution; attention/transformer fusions; FFT; sparse
linear algebra; parallel primitives (reduction/scan/sort/histogram);
structured-grid stencils; dense factorizations; MD force kernels;
N-body; graph analytics; ray tracing; crypto/hash/RNG; image/DSP
pipelines; embedding/scatter-gather; transpose/layout + elementwise
fusion. The famous ones carry their scheduling solutions privately —
hand-pipelined into libraries and generators (CUTLASS pipelines,
[Tensile's assembly GEMMs](https://github.com/ROCm/Tensile/wiki),
[FlashAttention-3](https://tridao.me/publications/flash3/flash3.pdf)).

## 2. Front 1 — where instruction ORDER itself matters (fixed occupancy)

### 2.1 Classes with the most ordering leverage

1. **Low-occupancy dense-math tile kernels (GEMM / attention /
   convolution on MFMA/tensor cores).** Run at 1-2 waves/SIMD by design
   (registers spent on tiles), so wave-switching is unavailable and
   intra-wave order is the only latency-hiding mechanism. Strongest
   evidence that the default compiler order fails here: LLVM/AMDGPU
   grew user-facing ordering-override intrinsics — `sched_barrier`,
   `sched_group_barrier`, `iglp_opt`
   ([D128158](https://reviews.llvm.org/D128158),
   [AMDGPUUsage](https://github.com/llvm/llvm-project/blob/main/llvm/docs/AMDGPUUsage.rst),
   [sched_group_barrier rules PR](https://www.mail-archive.com/cfe-commits@lists.llvm.org/msg417205.html)) —
   and AMD's engineering blog uses them to stop the compiler "batching
   all memory instructions after all MFMA, eliminating the overlap"
   ([4-wave FP8 GEMM](https://rocm.blogs.amd.com/software-tools-optimization/4wave-fp8gemm/README.html)).
   AMD states plainly that high-performance kernels often run at low
   occupancy ([HIP performance doc](https://rocm.docs.amd.com/projects/HIP/en/docs-7.2.0/understand/performance_optimization.html)).
   Scheduling-pattern choice (ping-pong vs interleave vs wave
   specialization) decides whether CDNA3/4 kernels reach peak
   ([HipKittens](https://arxiv.org/abs/2511.08083)).

2. **Barrier-synchronized ("lock-stepped") LDS-pipelined kernels.**
   Shape: load tile global->LDS; barrier; compute from LDS; barrier;
   repeat. The barrier forces all waves of the workgroup to the same
   point, so they stall TOGETHER — wave B cannot cover wave A because
   it waits at the same barrier for the same tile. Occupancy-based
   hiding is structurally defeated (other-workgroup waves are scarce
   under big LDS allocations, and often in-phase too); the only
   latency-hiding tool left is the order within each barrier-to-barrier
   phase (issue next tile's loads before computing the current one,
   sink the waitcnt late). This is the exact framing of AMD's MI300X
   series: [Memory Instruction Scheduling for Lock-Stepped Kernels on
   MI300X](https://rocm.blogs.amd.com/software-tools-optimization/scheduling_memory_ops_gfx942/README.html).

3. **Latency-bound kernels with many independent loads** (gathers,
   table lookups, register-blocked primitives). AMD mechanism: vmcnt
   loads return IN ORDER (pre-RDNA4), so the issue order of the loads
   defines what a single `s_waitcnt vmcnt(N)` can overlap — batching N
   independent loads before one wait is a pure ordering transform with
   Nx latency-overlap payoff
   ([Chips and Cheese on RDNA4's move away from this](https://chipsandcheese.com/p/rdna-4s-out-of-order-memory-accesses)).
   NVIDIA analogue: the general load-to-use issue-distance leverage
   fully applies (visible as "Long Scoreboard" stalls, remedy is
   scheduling independent work between load and use —
   [Nsight guide](https://docs.nvidia.com/nsight-compute/ProfilingGuide/index.html),
   [NVIDIA ADO part 3](https://developer.nvidia.com/blog/analysis-driven-optimization-finishing-the-analysis-with-nvidia-nsight-compute-part-3/));
   the in-order quirk does NOT (each load gets one of six per-instruction
   dependency barriers, tracked out of order — the six slots being their
   own scarce scheduling resource:
   [maxas control codes](https://github.com/NervanaSystems/maxas/wiki/Control-Codes),
   [ptxas RE reference](https://gh.evko.io/nvopen-tools/ptxas/)).
   Practitioner form: batching many independent texture reads before a
   single waitcnt
   ([Wronski, GCN latency hiding](https://bartwronski.com/2014/03/27/gcn-two-ways-of-latency-hiding-and-wave-occupancy/));
   register blocking in primitives
   ([moderngpu performance](https://moderngpu.github.io/performance.html)).

4. **Mixed-pipe kernels** (VALU + scalar + MFMA + LDS + VMEM active
   together). Clumped same-pipe orders serialize one pipe while others
   idle; `sched_group_barrier` pipelines ("1 VMEM, 4 MFMA, 1 DS") exist
   to force the interleave. NVIDIA analogue: keeping dual dispatch
   ports fed ([maxas SGEMM](https://github.com/nervanasystems/maxas/wiki/sgemm)).

5. **NVIDIA fixed-latency scheduling as compiler responsibility.**
   Since Kepler there are no interlocks for fixed-latency instructions:
   ptxas encodes stall counts, dependency barriers, yield and `.reuse`
   bits per instruction; ordering changes dead cycles, operand-reuse
   cache hits, and register-bank conflicts
   ([maxas control codes](https://github.com/NervanaSystems/maxas/wiki/Control-Codes)).

6. **(Smaller) RDNA clause formation / waitcnt granularity** — memory-op
   adjacency controls clause bursts and wait precision; RDNA4 broke the
   monolithic counters into typed out-of-order queues precisely because
   the old scheme created false ordering dependencies
   ([Chips and Cheese RDNA4-LLVM](https://chipsandcheese.com/p/examining-amds-rdna-4-changes-in-llvm)).

### 2.2 Anti-list (ordering cannot matter much)

- DRAM-bandwidth-saturated streaming kernels at healthy occupancy.
- Serially dependent chains with nothing to hoist (pointer chasing,
  dependent lookups, ray traversal — RDNA4 added out-of-order memory
  queues partly because compiler-visible ordering could not help
  raytracing: [Chips and Cheese](https://chipsandcheese.com/p/rdna-4s-out-of-order-memory-accesses)).
- Divergent, tiny-basic-block kernels (schedulers reorder within
  regions; most regions are tiny or already optimal).
- High-occupancy latency-bound kernels (warp scheduler already has an
  eligible wave nearly every cycle).
- Contention-/launch-overhead-bound kernels.

### 2.3 Trusted magnitudes (ordering-only)

| Evidence | Number | Why trusted |
|---|---|---|
| [CuAsmRL, CGO'25](https://arxiv.org/html/2501.08071v1): RL over SASS reorderings on top of ptxas -O3, A100, Triton LLM kernels | geomean +9%, max +26% (rmsnorm); +7% on fused GEMM from ONE adjacent HMMA/LDGSTS swap | Same instructions, same registers, same occupancy — only order changes; Nsight-verified mechanism; artifact-evaluated |
| [maxas SGEMM](https://github.com/nervanasystems/maxas/wiki/sgemm) | ~20% of FFMA throughput at stake in register-bank conflicts avoided by ordering/reuse flags | Cycle-accounting on a fully understood inner loop; mechanism independently rediscovered a decade later |
| [HipKittens](https://arxiv.org/abs/2511.08083) | wrong scheduling pattern ≈ 80% of peak vs ~peak; 1.2-2.4x over compiler baselines | Reference point is AMD's own hand-written asm; corroborates AMD blog + intrinsic design |
| [Volkov GTC'10](https://www.nvidia.com/content/gtc-2010/pdfs/2238_gtc2010.pdf) | ~84-87% of peak bandwidth at ~4% occupancy via per-thread ILP/MLP | Clean microbenchmark; principle replicated for 15 years ([Blackwell microbenchmarks](https://arxiv.org/pdf/2507.10789)) |

Cross-source picture: ordering-only deltas over a reasonable compiler
baseline cluster at **~5-25%**; pathological orders (loads sunk behind
compute, pipeline overlap destroyed) cost **integer factors** — which
is why hand-scheduling ecosystems persist for GEMM/attention on both
vendors.

## 3. Front 2 — where the pressure/occupancy coupling bites

### 3.1 Classes

1. **High-order finite-difference / seismic stencils** (register-tiled
   wavefield planes; live FD coefficients). AMD's flagship example
   SW4CK sits at 232 VGPRs = the 2-wave tier; unrolling pushed it to
   256 VGPRs PLUS spilling — occupancy unchanged — and runtime went
   12 ms -> 6.81 ms (~1.76x FASTER): the added per-wave MLP/ILP bought
   more than the spill traffic cost
   ([ORNL register-pressure deck](https://www.olcf.ornl.gov/wp-content/uploads/Intro_Register_pressure_ORNL_20220812_2083.pdf)).
   The trade is genuinely two-sided: sometimes shave registers to gain
   a wave, sometimes spend registers past the spill cliff for ILP — a
   scheduler that always minimizes pressure is provably wrong here.
2. **Items-per-thread register-tiled primitives** (CUB/rocPRIM-style
   scan/sort/reduce). Each thread holds an ITEMS_PER_THREAD register
   array — a designed-in register tile that amortizes synchronization
   and feeds the scheduler independent loads, tuned right against the
   occupancy-tier boundary
   ([moderngpu](https://moderngpu.github.io/performance.html)).
3. **Real-time ray-tracing shaders** — payload/attribute state live
   across TraceRay, driver-reserved registers, traversal state in
   inline RT; NVIDIA's guidance ties each directly to occupancy and
   recommends splitting pipelines so each ray type runs at its own
   occupancy ([RTX best practices](https://developer.nvidia.com/blog/best-practices-for-using-nvidia-rtx-ray-tracing-updated/)).
4. **Game shaders with DATA-DEPENDENT fetch loops** (SSR, ray
   marching, parallax) — when the next fetch depends on the last one, a
   single wave cannot hide its own latency, so occupancy is the only
   mechanism and VGPR pressure gates it directly. Sharp practitioner
   distinction vs independent-iteration loops (Poisson DOF), which one
   wave can hide via batching
   ([Wronski](https://bartwronski.com/2014/03/27/gcn-two-ways-of-latency-hiding-and-wave-occupancy/),
   [GCN shader programming GDC'17](https://gpuopen.com/download/GDC2017-Advanced-Shader-Programming-On-GCN.pdf)).
   AMD ships per-instruction live-VGPR tooling for this
   ([RGA live VGPR](https://gpuopen.com/learn/live-vgpr-analysis-radeon-gpu-analyzer/)).
5. **Directive-ported (OpenACC/OpenMP) Fortran physics** — monolithic
   loop bodies, dozens of temporaries, inlined math, little per-kernel
   register control. MURaM MHD kernels: 122 regs -> 25% occupancy,
   flagged as the port limiter ([MURaM](https://arxiv.org/pdf/2107.08145));
   register-lifetime optimization: +33% occupancy -> ~+30% perf
   ([OpenACC register study](https://www.researchgate.net/publication/308584783_Optimizing_GPU_Register_Usage_Extensions_to_OpenACC_and_Compiler_Optimizations)).
6. **Chemical kinetics / stiff ODE integrators** (ODE = ordinary
   differential equation: each thread integrates its own system —
   species concentrations, neuron models — with the whole state vector
   plus integrator temporaries live for the entire kernel). Integrator
   CHOICE is driven by register state; spills attributed to species
   count ([combustion RHS study](https://www.sciencedirect.com/science/article/am/pii/S0010218017300354)).
7. **Monte Carlo particle transport** — history-based single kernel
   carries full particle state: Shift measured 168 regs -> 12.5%
   occupancy vs event-based 127 regs -> 25%; pressure is the stated
   reason the community restructures to event-based pipelines
   ([Shift/ORNL](https://www.osti.gov/servlets/purl/1492181);
   proxy apps [XSBench](https://github.com/ANL-CESAR/XSBench),
   [RSBench](https://github.com/ANL-CESAR/RSBench)).
8. **Fused ML kernels** — fusion keeps intermediates in registers by
   design; over-fusion tips into spills or drops a CTA per SM.
   FlashAttention-3 needs explicit warp-group register reallocation
   ([Colfax](https://research.colfax-intl.com/flashattention-3-fast-and-accurate-attention-with-asynchrony-and-low-precision/));
   Triton autotunes block size / num_warps to trade pressure vs
   occupancy ([ROCm Triton guide](https://rocm.docs.amd.com/en/docs-6.1.1/how-to/llm-fine-tuning-optimization/optimizing-triton-kernel.html)).
9. **Lattice QCD** (SU(3) matrices + spinors per thread) — community
   routinely tunes launch_bounds/maxrregcount accepting spills to buy
   occupancy tiers ([QUDA-era tuning](https://arxiv.org/pdf/1212.5221));
   showcase workload for CUDA 13's shared-memory spill redirection
   (5-10%: [NVIDIA blog](https://developer.nvidia.com/blog/how-to-improve-cuda-kernel-performance-with-shared-memory-register-spilling/)).
10. **What creates pressure, per AMD's canonical list**: forced full
    inlining (a `pow(x,2.0)` costs registers), aggressive unrolling,
    big by-value kernel arguments, stack objects, long def-to-use
    distances, doubles (2 VGPRs each)
    ([AMD register-pressure lab notes](https://gpuopen.com/learn/amd-lab-notes/amd-lab-notes-register-pressure-readme/)).

### 3.2 Anti-list

Streaming bandwidth-bound kernels; ALU-saturated kernels ("a workload
that is not latency bound will not benefit from increased occupancy" —
[AMD occupancy explained](https://gpuopen.com/learn/occupancy-explained/));
abundant-ILP kernels where one wave hides its own latency; kernels
occupancy-limited by LDS/thread-group size rather than registers;
kernels sitting mid-tier far from any register/occupancy quantization
boundary; cache-sensitive kernels where MORE occupancy hurts
([NVIDIA forum measurement](https://forums.developer.nvidia.com/t/understanding-degraded-kernel-performance-with-higher-occupancy/335345)).

### 3.3 Convergent condition

Across sources, the coupling bites where three things coincide:
**(a) large per-thread live state BY DESIGN** (register tiles, fused
accumulators, particle/species state, payload across trace),
**(b) latency-bound execution with data-dependent serialization** (so
waves are the only latency-hiding mechanism), and **(c) proximity to a
coarse occupancy-tier boundary or the spill cliff**. Sources agree the
sensitive set is a minority of kernels/regions, with effects of tens of
percent in either direction inside it.

## 4. Front 3 — where effort observably goes (artifact inventory)

Five artifact categories: (1) autotuner example suites
([Kernel Tuner](https://github.com/KernelTuner/kernel_tuner/tree/master/examples),
[CLTune](https://github.com/CNugteren/CLTune/tree/master/samples),
[ATF/pyATF](https://atf-tuner.org/),
[KernelBench](https://github.com/ScalingIntelligence/KernelBench),
[KTT](https://github.com/HiPerCoRe/KTT),
[BAT](https://github.com/NTNU-HPC-Lab/BAT),
[CLBlast](https://github.com/CNugteren/CLBlast/tree/master/src/tuning/kernels),
[GPTune](https://github.com/gptune/GPTune),
[ytopt](https://github.com/ytopt-team/ytopt));
(2) vendor case studies (NVIDIA:
[i-cache vs unrolling on Smith-Waterman](https://developer.nvidia.com/blog/improving-gpu-performance-by-reducing-instruction-cache-misses-2/),
[prefetch/register-budget on a financial kernel](https://developer.nvidia.com/blog/boosting-application-performance-with-gpu-memory-prefetching/),
[shared-memory spill redirection](https://developer.nvidia.com/blog/how-to-improve-cuda-kernel-performance-with-shared-memory-register-spilling/),
[CUTLASS](https://developer.nvidia.com/blog/cutlass-linear-algebra-cuda/);
AMD: [LBM register-pressure lab note](https://gpuopen.com/learn/amd-lab-notes/amd-lab-notes-register-pressure-readme/),
[Laplacian launch-bounds](https://gpuopen.com/learn/amd-lab-notes/amd-lab-notes-finite-difference-docs-laplacian_part3/),
[Claybook VGPR 40->32 = +50%](https://gpuopen.com/learn/optimizing-gpu-occupancy-resource-usage-large-thread-groups/),
[seismic stencils](https://rocm.blogs.amd.com/high-performance-computing/seismic-stencils/part-2/README.html),
[stiff-ODE profiling](https://rocm.blogs.amd.com/software-tools-optimization/profiling-guide/ai-assist-optimization/README.html);
Intel: [high-register-pressure porting guide](https://www.intel.com/content/www/us/en/docs/oneapi/optimization-guide-gpu/2025-0/porting-code-with-high-register-pressure-to-intel.html));
(3) graphics talks (Persson
[low-level shader optimization](https://gdcvault.com/play/1020352/Low-Level-Shader-Optimization-for),
Wihlidal [Frostbite culling VGPR numbers](https://www.gdcvault.com/play/1023109/Optimizing-the-Graphics-Pipeline-With),
[Ubisoft cloth compute](https://gdcvault.com/play/1020939/Efficient-Usage-of-Compute-Shaders),
[Nanite](https://advances.realtimerendering.com/s2021/Karis_Nanite_SIGGRAPH_Advances_2021_final.pdf),
[Doom Eternal](https://advances.realtimerendering.com/s2020/RenderingDoomEternal.pdf),
[Anagnostou on occupancy](https://interplayoflight.wordpress.com/2020/11/11/what-is-shader-occupancy-and-why-do-we-care-about-it/));
(4) compiler trackers/forums
([rocFFT VGPR regression](https://github.com/llvm/llvm-project/issues/94092),
[MFMA matmul spills](https://github.com/llvm/llvm-project/issues/131954),
[OpenMC offload regression](https://github.com/llvm/llvm-project/issues/123092),
[GpuOwl FFT occupancy workaround +33%](https://github.com/ROCm/ROCm/issues/1002),
[miniBUDE occupancy](https://github.com/RadeonOpenCompute/ROCm/issues/1679),
[Triton MI300X waitcnt over-conservatism](https://github.com/triton-lang/triton/issues/6310),
[ptxas spill/reorder threads](https://forums.developer.nvidia.com/t/preventing-ptxas-from-reordering-instructions/28682),
[Vello clip-stack pressure](https://github.com/linebender/vello/issues/83));
(5) hand-written assembly
([maxas](https://github.com/NervanaSystems/maxas),
[NervanaGPU](https://github.com/NervanaSystems/neon),
[maxDNN conv](https://arxiv.org/pdf/1501.06633),
[TuringAs Winograd](https://cse.hkust.edu.hk/~weiwa/papers/yan-ppopp20.pdf),
[openai-gemm](https://github.com/openai/openai-gemm),
[blocksparse](https://github.com/openai/blocksparse),
[DeepGEMM SASS-patching](https://github.com/deepseek-ai/DeepGEMM),
[Tensile](https://github.com/ROCm/Tensile/wiki),
[AITER CDNA asm](https://github.com/ROCm/aiter),
[composable_kernel](https://github.com/ROCm/composable_kernel),
[GCNGEMM](https://github.com/hyln9/GCNGEMM),
[ECC2K-130 cryptanalysis asm](https://eprint.iacr.org/2012/002.pdf),
[hashcat](https://hashcat.net/hashcat/),
[TeamRedMiner](https://github.com/todxx/teamredminer),
[CLRX assembler](https://github.com/CLRX/CLRX-mirror)).

**Frequency (how many of the 5 categories a domain appears in):**

| Domain | Categories |
|---|---|
| Dense GEMM / matmul | 4/5 (all but graphics talks) |
| DL operators broadly (conv, attention, norms, MoE) | 4/5 |
| Convolution specifically | 3/5 |
| Hashing / PoW crypto | 3/5 (deepest hand-asm record) |
| Game/graphics shaders (culling, lighting, RT, post) | 3/5 (where stall/VGPR TOOLING concentrates) |
| Attention specifically | 3/5 |
| Reduction/elementwise (mostly pedagogical) | 3-4/5 |
| Stencils / structured grid | 2/5 (autotuners + vendor) |
| Sparse / iterative solvers | 2/5 |
| FFT | 2-3/5 |
| N-body / molecular / docking | 2/5 (autotuners + trackers) |
| Monte Carlo transport | 2/5 |
| Image pipelines / post-processing | 2/5 |
| Physics sim (cloth, SDF, LBM) | 2/5 |

Reading: hand-written assembly is the most expensive form of effort,
and its record contains ONLY dense math, DL ops, and crypto — the
domains where a cycle is worth money. Stencils, sparse solvers, and
N-body get systematic-but-cheaper attention (autotuners, case studies)
and no hand-asm. Graphics shaders are unique: the effort shows up as
vendor tooling (RGP instruction timing, RGA live-VGPR) because the
kernel population is too large and churning to hand-tune individually —
structurally, the same situation a compiler scheduler faces.

## 5. Synthesis: the combined map

Classes appearing on multiple fronts, ranked by convergence:

1. **Low-occupancy dense-math tile kernels** — fronts 1+2+3. Served by
   hand-asm/libraries today; the `sched_barrier` intrinsic ecosystem is
   standing evidence the automatic scheduler fails there.
2. **Barrier-lock-stepped LDS-pipelined kernels** — fronts 1+2 (+3 via
   the MI300X series). Ordering leverage independent of occupancy.
3. **Register-tiled primitives (items-per-thread)** — fronts 1+2+3.
4. **Latency-bound independent-gather kernels** (table lookups, MC
   transport) — fronts 1+2 (+3 via proxy apps/trackers).
5. **Graphics/RT shaders, esp. data-dependent fetch loops** — fronts
   2+3; enormous population, compiler-only (no hand-tuning possible at
   scale).
6. **High-order stencils at low tiers** — fronts 2+3; NOT the
   streaming-stencil case (that one is bandwidth-bound and on the
   anti-list — the two must not be conflated).
7. **Directive-ported physics / kinetics / QCD** — front 2 (+3
   partially).
8. **Crypto/hash** — front 3 dominant (+1 mechanisms); economically
   proven ordering value, though dependence chains limit reorder
   freedom.

## 6. What blind search corrected vs. the anchored draft

Missed entirely by the anchored version: low-occupancy dense math as
the #1 ordering class (had been dismissed as "libraries have it
covered"); barrier lock-stepping as an occupancy-independent mechanism;
items-per-thread primitives as designed pressure; directive-ported
Fortran physics; crypto/hash; the full breadth of graphics shaders.

Overstated by the anchored version: "stencils are bandwidth-bound"
(true only for streaming stencils — high-order register-tiled stencils
are a headline pressure class); "MD force loops" as a leading item
(weak independent support); "odd-shape dense algebra" (essentially no
independent evidence).

Survived: ODE/kinetics, Monte Carlo table lookups, lattice QCD, fused
ML output, register-fat shaders.

## 7. Open questions

- No trustworthy PUBLISHED measurement exists of how much compiler
  instruction scheduling buys on AMDGPU (the only published magnitudes
  come from evaluations this lab does not credit). The magnitude
  question is genuinely open.
- HeCBench mapping against sections 2-5: done, section 8.

## 8. HeCBench mapping (WORKING set, 2026-09-08)

Mapping of the 356 benchmarks in gpu2_benchmarks'
`hec_bench_all_WORKING` group onto the survey's categories. Method:
name-based classification for the well-known ports, plus code
inspection by four read-only agents (identification of ~55 opaque
names; structural verification of library-vs-own-kernel, LDS-tile /
barrier-loop patterns, items-per-thread register arrays, stencil
order and register rotation). A ✓ marks code-verified structure.

### 8.1 View 1: per-category listings

#### Scheduling-leverage classes (section 5)

**B1 low-occupancy dense-math tile kernels** (own kernel with
register/LDS tiles): winograd✓ (four 4x4 register tiles, no LDS),
dct8x8✓ (D[8] register butterflies + LDS transpose), hexciton✓
(batched 7x7 complex commutator, register-blocked variants),
ccsd-trpdrv✓ (16 scalar regs + wide FMA), gpp✓ (BerkeleyGW complex
contraction, live accumulator arrays), mdh✓ (LDS atom tiles),
quantAQLM✓ / quant3MatMul✓ (LDS-tiled quantized matvec),
bezier-surface. EXCLUDED after code check: blas-gemm /
blas-gemmBatched / blas-gemmStridedBatched / geam are hipBLAS library
calls (no compiler-scheduled kernel); gemv, quantVLLM, quantBnB are
streaming.

**B2 barrier-lock-stepped LDS-pipelined kernels** (verified
load-LDS/sync/compute/sync loops): lud✓, nw✓ (barrier per
anti-diagonal), pathfinder✓, hotspot✓, lavaMD✓ (best non-LA example:
per-neighbor-box tile loop), fwt✓, minimod✓ (3D LDS brick),
stencil3d✓ (LDS plane rotation), stencil1d✓, laplace3d✓, adv✓
(spectral element; largest combined LDS+register footprint in the
set), tridiagonal✓ (PCR/cyclic kernels), tsa✓, merge✓, split✓,
radixsort✓, sort✓, scan2✓, is✓, hybridsort✓ (bucket kernels), topk✓,
mdh✓, quantAQLM✓, b+tree✓ (barrier-lockstepped WITHOUT a tile),
reaction, asmooth. Incidental find: dct8x8's HIP port dropped the
__syncthreads of the OpenCL original while still sharing an LDS
transpose buffer — suspected upstream correctness bug.

**B3 items-per-thread register-tiled primitives** (verified grain-size
register arrays): segsort✓ (up to 32 keys/thread in scalarized
registers — strongest case), warpsort✓ (T arr[N] + unroll, zero LDS),
merge✓ (4/thread), sort✓ (4 keys + digit_counts[16]), split✓ /
radixsort✓ (uint4), hybridsort✓ (float4 merge pass), topk✓ (hipcub
items_per_thread), sc✓ (REGS-element tile across compaction phases).
Not in class after check: scan2 (two scalars), bscan, histogram,
bitonic-sort (1 elem/thread); scan3 and segment-reduce are
thrust/hipCUB library calls.

**B4 latency-bound independent-gather kernels**: xsbench, rsbench
(the survey's literal proxy apps), tissue✓ (Green's-function table
gather), compute-score✓ (bloom lookups, deliberate
manual-vectorization register pressure), s8n✓ (neighborhood search
with register arrays), henry✓, aidw✓, expdist (Kernel Tuner's own
benchmark), tpacf, knn, degrid, car✓, all-pairs-distance, hausdorff,
word2vec, mcpr✓ (w[21] live across the MC loop).

**B5 data-dependent-fetch / traversal (shader-shaped)**: bh (tree
traversal), b+tree, bsearch, quicksort, aobench (path tracer), grrt
(GR ray integration), surfel✓ (ray-cast), face (cascade), eikonal,
asmooth✓ (data-dependent radius loop), snake✓ (data-dependent while,
~12 live regs), diamond✓ (HMM with large per-thread local arrays),
wyllie (list ranking), sss✓, minimap2, particlefilter, fsm / grep
(automata).

**B6 high-order / wide-expression stencils**: fdtd3d✓ (radius-4
register-rotation queue — the textbook member), wsm5✓ (~20 private
column-length arrays — guaranteed spill class), sw4ck✓ (radius-2
elastic; pressure from expression width — the literal kernel from
AMD's ORNL deck), hypterm✓ (radius-4 monolithic flux expressions),
rtm8✓ (dual 25-point), minimod✓, adv✓, tsa✓ (register arrays across
timesteps). Streaming anti-side (verified radius-1
load-compute-store): heat, heat2d, iso2dfd, burger, che, ace, plus
d2q9-bgk / d3q19-bgk, lid-driven-cavity, hotspot3D✓ (small rolling
registers), srad✓ (flat elementwise).

**B7 per-thread carried-state evolution loops** (each thread advances
its own state vector through an iteration — integration steps, path
steps, particle histories — so a large live set persists across a long
loop body; domains are examples, not the definition): s3d✓ (hundreds
of live kinetics temporaries in one basic block — the classic),
rushlarsen✓ (19 states, 474 straight-line double-precision lines,
zero loops/barriers), cobahh✓ (7 states + 33 constants, expf chains),
myocyte✓ (39 KB straight-line RHS launched nearly serially), goulash✓,
lci✓, egs✓ (full particle struct live across the MC history loop),
wsm5✓, pns✓, aop✓, feynman-kac, ising, vmc, mcmd, libor (MC
forward-rate path evolution — the NVIDIA-case-study finance family),
binomial, bonds✓ (per-bond cashflow-date loops with live leg state),
gibbs (marginal: rejection-sampling loop carries RNG state but the
live set is small). Reclassified out on structural grounds:
black-scholes — closed-form per-element evaluation, no loop, no
carried state; it belongs with the long-transcendental-chain
elementwise batch below.

**B8 crypto/hash**: aes, bitcracker, keccaktreehash, md5hash,
murmurhash3, jenkins-hash, secp256k1, ecdh, mr✓ (Miller-Rabin powmod
ladders), present, chacha20, ntt, crc64. Label honesty note: this is
a DOMAIN class earned by the effort-inventory evidence (section 4's
hand-asm record), not a structural one — its structural content (long
fixed serial rounds, integer-heavy, little reorder freedom) is
anti-class-adjacent for ordering leverage.

#### General optimization-attention classes (section 1)

GEMM family: the B1 list plus the three hipBLAS wrappers.
Convolution: convolution1D/3D/Deformable/Separable, dwconv, winograd,
vol2col, unfold. Attention / DL operators: attentionMultiHead,
softmax, softmax-online, layernorm, rmsnorm, rotary, moe, moe-sum,
gru, gelu, relu, swish, adam, adamw, crossEntropy, scel, logprob,
perplexity, resnet-kernels, backprop, word2vec, wedford,
rowwiseMoments, channelSum, zeropoint, quant*. FFT/transforms: fft,
zmddft, dct8x8, fwt, hwt1d, ntt. Sparse LA: simpleSpmv, ge-spmm,
sptrsv, spm, mmcsf, amgmk, miniFE, jacobi, lanczos, slu. Primitives:
the B2/B3 lists plus histogram, bincount, filter, rfs, minmax,
stddev, kurtosis, entropy. Factorizations: lud, gaussian, eigenvalue,
determinant, tridiagonal, thomas. MD / N-body: md, lavaMD, nbody,
minibude, testSNAP, haccmk, clenergy, sph, wlcpow, bh; lattice QCD:
su3, dslash. Graph: bfs, sssp, page-rank, cc, mis, hbc,
floydwarshall(2), tsp, streamcluster, kmeans. Ray/rendering: aobench,
grrt, surfel, marchingCubes, voxelization, flame. Crypto/compression:
the B8 list plus bwt, lzss, rle, ans, bitpacking, fpc, mtf, sa.
Bioinformatics alignment (the Smith-Waterman case-study class): bsw,
logan, extend2, minimap2, nw, phmm, snake, diamond. Image/DSP: ~30
(bm3d, ced, opticalFlow, sad, seam-carving, debayer, hogbom, ddbp,
...). Layout/elementwise: matrixT, tensorT, matrix-rotate, reverse2D,
axpby, plus the long-transcendental-chain batch (fresnel✓, geodesic✓,
qem✓, black-scholes — serial FP chains, closer to issue-bound
controls).

### 8.2 View 2: importance x leverage quadrants

The two views measure different things — A membership = "stands in
for a kernel class the world demonstrably invests optimization effort
in" (relevance); B membership = "has the structural properties that
give a compiler instruction scheduler something to do" (sensitivity)
— and they dissociate in this set:

| | in B (scheduling leverage) | not in B |
|---|---|---|
| **in A** | winograd, dct8x8, hexciton, segsort, fdtd3d, minibude, quantAQLM, ... — important AND schedulable; real-world competition is hand-tuned libraries | blas-gemm* (hipBLAS call: GEMM-class importance, zero compiler-scheduler surface); streaming DL elementwise (relu, mask, ...) |
| **not in A** | s3d, rushlarsen, cobahh, sw4ck, wsm5, egs, myocyte, ... — no library or framework serves these; **the compiler's schedule is the only schedule they will ever get** | filler (projectile, overlay, ...) |

The bottom-left quadrant (B without A) is the strategic one for
compiler-scheduling research: structurally the same situation as the
graphics-shader population of section 4 — too many, too churning, too
niche to hand-tune — where a compiler improvement is the only possible
delivery mechanism. The top-left quadrant is where wins are flashiest
but benchmarked against Tensile-grade hand assembly.

### 8.3 Practical notes

- Many `-hip` directories pull kernel source from outside their own
  directory (Makefile `-I ../<name>-cuda`, direct `../<name>-cuda/x.cu`
  source paths, or `#include "../..."`). A full scan found 176 of the
  356 WORKING benchmarks do this. It is BENIGN for our purposes — the
  borrowed source still compiles through our toolchain as part of the
  benchmark build — but tooling that globs `*-hip/**` for kernel
  source silently misses those kernels.
- Representatives per pattern, if picking few: grain-size register
  arrays -> segsort, warpsort; barrier-tiled loop -> lavaMD, lud,
  hotspot, fwt; heavy straight-line live state -> rushlarsen, s3d,
  cobahh; register-rotation vs LDS-rotation stencil contrast pair ->
  fdtd3d vs laplace3d/stencil3d.

### 8.4 Compilability filter: library front-ends (2026-09-08 refinement)

For compiler-scheduling experiments a benchmark is only usable if its
HOT kernels are compiled from benchmark source by our compiler.
Mechanical scan of all 356 WORKING benchmarks (own `__global__` count,
external-source references, device-library usage) plus timed-path
inspection of the mixed cases gives:

**EXCLUDED — timed kernels are external library launches (11):**

- blas-gemm, blas-gemmBatched, blas-gemmStridedBatched, geam — hipBLAS
  is the timed path (blas-gemm's own naive kernel is not what is
  measured).
- determinant — hipsolver Cholesky (+ thrust).
- scan3 — hipcub::DeviceScan / thrust::exclusive_scan; no own kernel.
- segment-reduce — thrust::reduce_by_key; no own kernel.
- scatterThrust — thrust::scatter; no own kernel.
- coordinates — thrust::transform; no own kernel.
- nonzero — timed regions are hipcub::DeviceReduce + DeviceSelect;
  the only own kernel is a trivial index-writing epilogue.
- remap — timed pipeline is thrust::sort_by_key + unique_by_key with
  a small own gather kernel at the end.

**MIXED — kept, with external stages to keep in mind:** sa (own
skew-algorithm kernels interleaved with thrust::sort_by_key stages),
ans and lzss (own compression kernels; one thrust/hipcub scan stage),
tsne (27 own kernels; hipfft + thrust stages between them), ssim (own
kernel; thrust final reduction). Verified own-kernel timed paths
despite library mentions: convolution3D (miopen present but the timed
loops launch its own conv3d_s1/s2/s3), sort (thrust::sort is a
reference check only), ising (hiprand is host-side initialization).
Header-only thrust types inside own kernels (su3's thrust::complex,
rotary/rowwiseMoments tuples) are not library dependencies in this
sense at all — that code compiles through our scheduler.

Category-list impact: the B1 exclusions in 8.1 already covered the
hipBLAS wrappers; additionally determinant leaves the factorization
candidates and scan3 / segment-reduce / scatterThrust / coordinates /
nonzero / remap leave the primitives/DL-op candidates.

### 8.5 The shortlist: sched_shortlist_2026_09 (31 benchmarks)

Registered as BenchmarkGroup `sched_shortlist_2026_09` in
gpu2_benchmarks profile_in/benchmarks.json. Selection criteria, all
from existing on-disk data (the 2026-08-07 -pm survey + accumulated
timing history) unless noted:

1. Important or representative — a member of the survey's leverage
   classes (sections 2-5) or a key general-attention class (section 1).
2. Median app runtime <= 60 s.
3. GPU saturation >= 1.0 for at least one kernel at as-run problem
   size (survey column; sat = launched waves / 2400-wave capacity).
   Caveat: max-over-kernels, not hot-kernel-weighted.
4. Hot kernels compiled from benchmark source (section 8.4 filter).
5. Baseline build <= 120 s — informational, excluded nothing (worst:
   segsort 43 s, hexciton 24 s). NOTE: baseline build time is a FLOOR;
   search-based scheduler arms multiply it on big-single-region
   kernels — measured: hexciton 6.6 s baseline -> 102 s under ACO
   (July 2026 sweep). rushlarsen and testSNAP share the giant-region
   shape and carry the same warning, unmeasured.
6. Every category has at least 2 members.

Full rebuild of all 31 at baseline: ~4.7 min. Summed app runtime:
~7 min per arm.

| Category | Picked | Excluded (reason; rescue prospect where relevant) |
|---|---|---|
| Dense math | winograd, quantAQLM, ccsd-trpdrv, hexciton (as `hexciton-hip_jbaile_num16384`: default NUM=2048 fills 13% of the machine; NUM=16384 measured sat~1.0 at 53 s wall) | blas-gemm/-Batched/-StridedBatched, geam (library front-ends); gpp (slow 276 s); mdh (slow 206 s); dct8x8 (cut: transform-tile covered by winograd AND suspected dropped-barrier bug, 8.1); quant3MatMul (cut: near-duplicate of quantAQLM); bezier-surface (sat 0.09; rescue unknown) |
| Lock-stepped LDS / primitives | lavaMD, lud, adv, sort, segsort | Eligible, cut as redundant: pathfinder, hotspot, stencil1d, scan2, split, radixsort, fwt, hybridsort, b+tree, reaction, asmooth, minimod. Low-sat: is (0.43; has size classes — likely rescuable), merge (0.27), nw (0.85; wavefront-limited, partial). Slow: topk (99 s), stencil3d (123 s). No data: laplace3d, tridiagonal |
| FFT / transforms | fft, zmddft | ntt (sat 0.13; small fixed input); hwt1d (not in leverage classes) |
| Lattice QCD | su3, dslash | — (both picked) |
| MD force | testSNAP, minibude | clenergy (eligible; cut: issue-wall control profile); md (sat 0.48), nbody (0.10), sph (0.08) — all default-size fills; input-size rescue plausible but unverified; haccmk (A-class member, not in leverage classes) |
| Gather / MC transport | rsbench, xsbench | Eligible, cut as redundant: egs, pns, henry, tpacf, expdist, word2vec, knn, degrid, car, aidw, all-pairs-distance, hausdorff. Near-miss: tissue (sat 0.95; slight size bump likely rescues) |
| Ray / traversal | aobench, quicksort | Eligible, cut: bsearch, eikonal, wyllie, particlefilter, fsm, sss. Low-sat: face (0.03; fixed cascade — hard), snake (0.39), minimap2 (0.75). Slow+low-sat: grrt (93 s, 0.10). No data: bh, grep, diamond, surfel |
| Graphics-shaped | perlin, tonemapping | flame (sat 0.53; size is compile-time — source-edit rescue); voxelization (slow 140 s). No data: marchingCubes, dxtc2. STANDING GAP: HeCBench has no game-shader corpus (no culling, RT-payload, or SSR-shaped dependent-fetch kernels) — external-validity limit, not fixable by selection |
| High-order stencil | sw4ck, rtm8 | hypterm (64 s — borderline, admit if the cap loosens); fdtd3d (sat 0.29; option-parser size flags — LIKELY RESCUABLE and the class's flagship register-rotation kernel: recommended follow-up); wsm5 (sat 0.05; fixed WRF slab — hard) |
| Per-thread carried-state loops | cobahh, rushlarsen, libor | Eligible, cut: goulash, binomial, bonds, gibbs (marginal member). Low-sat: s3d (0.23; SHOC size classes — LIKELY RESCUABLE, the register-pressure classic: recommended follow-up), aop (0.41), vmc (0.93 near-miss), myocyte (0.00; near-serial launch — structurally hard), lci (0.00; single block by design — hard). Slow: feynman-kac (448 s), mcmd (129 s), ising (100 s) |
| Large thread-local arrays | compute-score, tsa, mcpr | Eligible, cut: s8n, sc, goulash. wsm5 (the class's extreme member; sat 0.05 — hard, see stencil row). No data: diamond, tridiagonal (both would-be members — the one cheap -pm run below settles them). Never evaluated: crs (register-heavy erasure coding; not carried into the leverage classes — candidate for a future audit) |
| Crypto / hash | keccaktreehash, md5hash | Eligible, cut: jenkins-hash, ecdh, murmurhash3, present. Near-miss: bitcracker (sat 0.98; larger dictionary trivially rescues). Low-sat small fixed inputs: mr (0.81), ntt (0.13), chacha20 (0.00), crc64 (0.20), secp256k1 (0.00). No data: aes |

Cross-cutting follow-ups: (a) one -pm run over the 7 no-data
benchmarks (aes, bh, diamond, grep, laplace3d, surfel, tridiagonal)
classifies them — diamond and tridiagonal are would-be
large-thread-local-array members; (b) size-flag rescues worth doing
if their classes need depth: fdtd3d and s3d (both flagship class
members, both likely one flag away); (c) attentionMultiHead remains
the documented DL/attention gap — naive implementation AND sat 0.11,
failing two criteria independently.
