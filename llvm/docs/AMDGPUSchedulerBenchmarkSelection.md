# Selecting Benchmarks Sensitive to Instruction Scheduling on AMDGPU

## Purpose

When evaluating a machine instruction scheduler (e.g. an experimental AMDGPU
scheduler against `MaxOccupancy` / `MaxIlp` / OptSched), a benchmark is only
informative if the **schedule measurably moves its performance**. Plenty of
kernels are nearly scheduling-insensitive — bandwidth-bound, or with too little
region or parallelism for the ordering to matter — so a scheduler change barely
moves them; they make poor *evaluation* targets even when they are important
workloads. How large that fraction is depends on the workload mix — it's an
empirical question, not a given. This doc is about finding the kernels at the
other end: where the schedule genuinely swings performance, so the comparison
actually discriminates between schedulers.

Treat the class list as a prior, not a verdict: membership predicts sensitivity,
it does not guarantee it. Every pick must be confirmed by measurement (last
section). Numbers below are for **gfx906** (wave64, GCN5); other arches shift the
thresholds but not the reasoning.

## When scheduling can move performance at all

There are two levers, and the interesting kernels live in the tension between
them:

- **Latency hiding within a wave (ILP / MLP).** A GPU normally hides latency with
  occupancy — many waves time-sharing the SIMD. When occupancy is *low*, the only
  way to fill the shadow of a memory load or a long-latency op (rsqrt, div, exp,
  an FMA accumulation chain) is to have *independent* instructions ready to issue.
  The scheduler creates that by reordering. It matters most when there is latency
  to hide **and** occupancy is too low to hide it for free.

- **Register pressure → occupancy (and spilling).** On gfx906 occupancy is
  quantized by VGPRs per SIMD: `≤24→10` waves, `32→8`, `40→6`, `48→5`, `64→4`,
  `84→3`, `128→2`, `256→1` (4-VGPR granularity), and past 256 the allocator
  *spills* to scratch. Live-range overlap is a scheduling decision, so the
  scheduler sets peak VGPRs; shaving across a bracket boundary buys a whole wave,
  and shaving below the spill threshold removes spill code entirely. The cliffs are
  steepest at *low* occupancy — a kernel at 130 VGPRs runs 1 wave, and reaching 128
  doubles it to 2.

**The tension is the whole game.** Scheduling for ILP (hoist loads early, widen
the live set) *raises* peak VGPRs and can cost a wave. Scheduling to minimize
pressure (keep live ranges tight) *re-serializes* and re-exposes latency. The best
benchmarks are where this trade-off is steep, because that's where a smarter
scheduler can beat the heuristics. (`MaxOccupancy` and `MaxIlp` are two *greedy
heuristics* pulling toward opposite objectives — low pressure vs high ILP. They
are **not** bounds and need not reach the extreme schedules, so a better scheduler
can land outside the interval they span. A large gap between them is strong
evidence a kernel is schedule-sensitive; a small gap is only weak evidence of the
opposite, since both are greedy and can co-fail. See screening.)

**Roofline position.** Once a kernel is *already pinned* at the bandwidth roof or
the FMA-issue roof, the bottleneck is a fixed resource and reordering adds no
throughput. The target is the **latency-bound valley**: below both roofs, limited
by insufficient parallelism in flight.

## The three criteria, sharpened

The usual shorthand — "large basic blocks, not bandwidth-saturated, high register
pressure" — is right but each clause needs a sharper edge:

1. **Large region, but *wide*, not just big.** Scheduling freedom comes from the
   *width* of the dependence DAG (independent chains to interleave), not its size.
   A 500-instruction block that is one serial dependent chain (a recurrence, a deep
   reduction) has almost no legal reorderings. Look for many *independent* chains
   in one region.
2. **Latency-bound, not merely "compute."** Must sit in the valley: not
   bandwidth-saturated, and below FMA-issue peak because of latency/occupancy
   stalls. "Compute-bound" is not enough if it's already at peak issue.
3. **Register pressure that is high *and malleable*.** Pressure only helps if the
   scheduler can *move* it — across an occupancy bracket, or, best of all, below the
   spill threshold. A kernel that **spills under the baseline schedule but need
   not** is among the highest-value targets: spill code costs extra instructions and
   memory traffic, so a schedule that eliminates it is a large, unambiguous win. The
   real non-target is *irreducible* pressure — where even the best achievable
   schedule stays over the cliff or still spills; there it's an allocation /
   algorithmic problem, not a scheduling one. So chase malleable pressure (poised at
   a cliff, or spilling-but-avoidable); avoid only irreducible pressure.

## Classes to look for (ordered by confidence)

### Strong — multiple independent analyses converge, and the mechanism is clean

- **High-order structured-grid / stencil / lattice updates.** Per cell, a wide
  neighborhood or multi-field state is simultaneously live and combined in a large
  arithmetic block: high *and* malleable pressure, real ILP, intensity between the
  roofs. *Order matters:* low-order stencils are bandwidth-bound (see red flags);
  high order is the target. Domains: CFD, seismic/geophysics, electromagnetics,
  plasma, climate, lattice methods.

- **Cryptographic permutation–substitution networks.** Fully-unrolled rounds on a
  register-resident state: enormous integer basic blocks, high state pressure,
  integer-ALU-bound (not memory-bound), trivially controllable. *Important nuance:*
  a single hash/cipher instance is a *deep dependent* chain with little internal
  ILP — the schedulable parallelism comes from processing **multiple independent
  states per thread/wave** (independent messages, counter-mode blocks, lanes). Set
  them up that way or there's little to reorder. Domains: hashing, block/stream
  ciphers, PRNGs, coding.

- **Tiled pairwise / N-body interaction.** A tiled inner loop holds a block of
  neighbors plus accumulators; each interaction is a dependent rsqrt/exp chain
  (latency) but independent across pairs (ILP). Aggressive inlining of the
  math fattens the region. Domains: molecular dynamics, astrophysics,
  particle-in-cell, SPH, neighbor search.

### Plausible — sound mechanism, one caveat each

- **Register-blocked dense tensor contraction** (GEMM, implicit-GEMM conv,
  attention blocks). The accumulator tile gives many parallel live ranges and a
  textbook ILP-vs-pressure trade-off. The one real caveat is the *roofline*: a
  heavily-tiled GEMM can reach the FMA-issue roof, where scheduling adds nothing —
  so keep the compiled kernel moderately tiled, in the valley. The existence of a
  hand-tuned vendor library (rocBLAS/Tensile) is **not** a reason to exclude it: we
  evaluate the scheduler on the code the compiler emits, and improving a generic
  compiled GEMM is a legitimate result even if production would use the library.

- **Fixed-structure fast transforms** (FFT/NTT butterflies, DCT, comparator
  networks). A static, register-resident dataflow graph, wide and independent, with
  twiddle/operand handling driving pressure. *Caveat:* this holds for the
  **in-register stage/radix**; a *large* multi-pass transform is dominated by the
  memory passes and is bandwidth-bound. Test the butterfly, not the whole FFT.

- **Fat pointwise / local-system evaluation** (ODE/PDE right-hand-sides, chemistry
  kinetics, equations of state, dense local solves). One element → one huge
  straight-line expression: the register-pressure champions, latency from
  div/sqrt/exp chains, compute-bound but far below peak. ILP needs *multiple
  elements per thread*. *Caveat:* these are the hardest to obtain or strip down to a
  clean benchmark.

### Weaker — frequently miscited

- **Data-parallel primitives** (reductions, prefix scans, batched/segmented ops).
  Often suggested, but plain reductions/scans are *small-region* and
  bandwidth/latency-bound with low pressure — little to schedule. They enter the
  zone only when **heavily batched/unrolled** so each thread carries a wide
  independent working set. Don't assume the library primitive qualifies; check the
  unrolled region.

## Red flags — looks promising, usually isn't

- **Bandwidth-bound streaming**: BLAS-1, element-wise, copy/transpose, low-order
  stencils. Reordering rarely moves throughput when bandwidth is the ceiling.
- **Memory-hard crypto**: Scrypt, CryptoNight, Argon2 are bandwidth-bound *by
  design* (large scratchpads to resist ASICs) — the opposite of what you want, even
  though "crypto" sounds right. Use compute-bound hashes/ciphers instead.
- **Irregular / divergent / pointer-chasing**: graph traversal, sparse, sort, FSM —
  data-dependent control flow → tiny regions, divergence, little to reorder.
- **The code *you compile* is already at peak.** If the compiled kernel under test
  is genuinely issue-bound at the FMA roof, scheduling can't add throughput. This is
  about the *compiled* code — the mere existence of a faster hand-tuned library
  (rocBLAS, rocFFT) does **not** disqualify a generic kernel you compile and
  improve.
- **Big but serial**: a large region that is one dependent chain. Size without
  width is not freedom.

## Screening a candidate (cheap → definitive)

1. **VGPR count vs the brackets.** Compile and read VGPRs/occupancy. Is it
   occupancy-limited by VGPRs and sitting *just above* a bracket boundary
   (e.g. ~130, ~86, ~66)? Even better, does it **spill** — a spilling baseline whose
   pressure is scheduling-reducible is a top-headroom target. The only pressure
   regime to skip is irreducible (best schedule still spills / still over the cliff).
2. **Region size *and* width.** Use region-size stats, but don't stop at size —
   confirm the big region has independent chains, not one long dependence.
3. **Roofline / profiler.** Confirm latency-bound: *both* low achieved bandwidth and
   low VALU utilization, with issue stalls. If it's at either roof, drop it.
4. **The cheap sensitivity probe (read it the right way).** Compile under
   `MaxOccupancy` vs `MaxIlp` and compare runtime / occupancy / peak VGPRs. A large
   delta is strong positive evidence — exactly the kernels worth testing a new
   scheduler on. A *small* delta is **not** proof of insensitivity: both are greedy
   heuristics and may be co-failing, or both missing a schedule a smarter scheduler
   would find — corroborate with the roofline/region checks before discarding.
5. **The unroll dial.** Classes that only reach the regime *after* unrolling/tiling
   (dense LA, stencils, N-body, primitives) can be *pushed* into it with the
   misched.txt unroll knobs — raise `unroll_threshold` / `runtime_unroll_factor`
   until VGPRs approach a cliff (or a deliberate, scheduling-reducible spill). This
   converts a borderline kernel into a stress test, but mind the representativeness
   caveat below.

## Caveats — none of this is gospel

- **Class ≠ guarantee.** The list is a prior; steps 3–4 of screening are what
  actually decide.
- **The probe is one-sided.** `MaxOccupancy`/`MaxIlp` are greedy heuristics, not
  bounds — a big gap confirms sensitivity, a small gap confirms little.
- **Isolation vs representativeness.** Crypto and FFT butterflies are easy to
  isolate — clean, controllable, low-noise — and that same cleanliness makes them
  *unrepresentative*; a win on a fully-unrolled microbenchmark may not generalize.
  Over-unrolling (the unroll dial) has the same hazard — see the trip-count realism
  discussion in the loop-unrolling guide. Pair clean microbenchmarks with at least
  one realistic application kernel.
- **Arch-specificity.** The bracket numbers are gfx906; the *reasoning* ports, the
  thresholds don't.
