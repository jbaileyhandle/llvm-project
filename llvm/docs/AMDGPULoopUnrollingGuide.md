# LLVM Loop Unrolling: An Introductory Guide (with AMDGPU Focus)

## Purpose of this Document

This document explains how LLVM decides to unroll loops, where the machinery
lives in the source tree, and how the AMDGPU backend specializes it. It covers
**all three unrolling modes** — full unroll, partial unroll, and runtime
(unknown-trip-count) unroll — not just the full-unroll case, because each is
reached by a different path through the same decision function and each produces
a different shape of code.

The intended reader already understands LLVM IR, basic blocks, and PHI nodes, and
wants a precise mental model of the unroller: which pass runs when, what cost
model gates each mode, what knobs exist, and what the emitted code looks like.
No prior knowledge of ScalarEvolution (SCEV) is assumed — §3 is a self-contained
primer, since SCEV's output is what drives the entire mode decision. Every
factual claim carries a `path:line` citation relative to the `llvm/` directory so
it can be checked against the source.

Two notes on scope:

- **This is the in-tree (this-fork) view.** The line numbers and field set match
  *this* checkout. In particular, this fork's `UnrollingPreferences` struct does
  **not** contain the upstream `MaxUpperBound` or `SCEVExpansionBudget` fields
  (`include/llvm/Analysis/TargetTransformInfo.h:503-585`); newer upstream LLVM
  adds them. Where this matters it is called out.
- **The worked examples (§16) are empirical.** They were produced with the local
  gfx906 toolchain by compiling two small HIP kernels and reading the unroller's
  own `-Rpass=loop-unroll` remarks and the emitted ISA/IR. They ground the theory
  in observable behavior and demonstrate the full → partial → runtime transition
  on real code.

## Table of Contents

1. [The Big Picture: Three Modes, Two Passes](#1-the-big-picture-three-modes-two-passes)
2. [Terminology](#2-terminology)
3. [ScalarEvolution (SCEV): a Primer](#3-scalarevolution-scev-a-primer)
4. [Where the Passes Live: Pipeline Placement](#4-where-the-passes-live-pipeline-placement)
5. [Anatomy of the Unroller: From Pass Entry to Transform](#5-anatomy-of-the-unroller-from-pass-entry-to-transform)
6. [Measuring Loop Size](#6-measuring-loop-size)
7. [`computeUnrollCount`: The Seven-Priority Decision](#7-computeunrollcount-the-seven-priority-decision)
8. [Full Unroll: Static Size Test and Dynamic Cost Model](#8-full-unroll-static-size-test-and-dynamic-cost-model)
9. [Partial Unroll](#9-partial-unroll)
10. [Runtime Unroll and the Remainder Loop](#10-runtime-unroll-and-the-remainder-loop)
11. [`UnrollLoop`: The Mechanical Transform](#11-unrollloop-the-mechanical-transform)
12. [Trip Counts via SCEV](#12-trip-counts-via-scev)
13. [Pragmas and Loop Metadata](#13-pragmas-and-loop-metadata)
14. [AMDGPU Specialization](#14-amdgpu-specialization)
15. [Reference: `UnrollingPreferences` Fields, Defaults, and Flags](#15-reference-unrollingpreferences-fields-defaults-and-flags)
16. [Worked Examples](#16-worked-examples)
17. [Loop Structure in Real Workloads: A HeCBench Survey](#17-loop-structure-in-real-workloads-a-hecbench-survey)
18. [Real-World vs. Benchmark Loop Bounds: Is §17 Representative?](#18-real-world-vs-benchmark-loop-bounds-is-17-representative)
19. [Making Benchmark Unrolling Realistic](#19-making-benchmark-unrolling-realistic)
20. [Tuning and Controlling Unrolling](#20-tuning-and-controlling-unrolling)

---

## 1. The Big Picture: Three Modes, Two Passes

Unrolling replicates a loop body so that one trip of the new loop does the work
of several trips of the old one. LLVM does this in **three modes**, distinguished
by what is known about the trip count and what happens to the loop control:

| Mode | Trip count | What happens to the loop | Result enum |
|------|-----------|--------------------------|-------------|
| **Full** | known constant `N` | body replicated `N` times, **loop deleted** — straight-line code, no back-edge | `FullyUnrolled` |
| **Partial** | known constant `N` (too big to fully unroll) | body replicated `C` times where `C` divides `N`; **one loop survives**, running `N/C` times | `PartiallyUnrolled` |
| **Runtime** | unknown at compile time | body replicated `C` times; **one loop survives** plus a **remainder loop** handles the leftover `N mod C` iterations | `PartiallyUnrolled` |

The result enum is `LoopUnrollResult` with exactly these three values
(`include/llvm/Transforms/Utils/UnrollLoop.h:54-66`).

Two distinct passes perform this work, and the split matters:

- **`LoopFullUnrollPass`** — runs **early**, in the function-simplification
  pipeline, and does **full unroll only**. Its job is to delete small
  constant-trip-count loops so later passes (SROA, GVN, instcombine) see
  straight-line code. It is a **loop pass** (invoked once per loop).
- **`LoopUnrollPass`** — runs **late**, after vectorization, and does the
  general thing: full, partial, **and** runtime unroll, plus peeling. It is a
  **function pass** (it walks the function's loops itself).

Both passes funnel into one shared core, `tryToUnrollLoop`, which gathers
preferences, computes a size, asks `computeUnrollCount` for a factor, and then
calls the actual transform `UnrollLoop`. The difference between the two passes is
entirely in the flags they pass to that core (§5).

A crucial architectural fact for AMDGPU: **the target adds no unroll pass of its
own.** `AMDGPUTargetMachine.cpp` schedules none (`lib/Target/AMDGPU/AMDGPUTargetMachine.cpp:767-768`
is only an ordering comment). Everything AMDGPU-specific happens through the
`TargetTransformInfo` hook `getUnrollingPreferences`, which seeds the cost
thresholds the generic passes consult (§14).

---

## 2. Terminology

First, the anatomy of a loop, since the rest of the guide leans on these block
names constantly:

```
       preheader          <- one block that jumps into the loop; a tidy spot for setup
          |
          v
  .----> header           <- loop entry; the ONLY way into the loop from outside
  |        |
  |      body             <- the per-iteration work
  |        |
  |      latch ---> exit  <- the latch picks: take the back-edge (loop again), or leave
  |        |
  '--------'
   back-edge
```

- **Header** — the loop's single entry block; control can enter the loop only here.
- **Preheader** — the one block that falls straight into the header; the natural
  place to put loop setup or hoisted-out code.
- **Latch** — the block at the bottom holding the branch that either loops again or
  leaves.
- **Back-edge** — the edge from the latch back to the header; it carries the loop's
  exit test (a compare + conditional branch) and is what makes the CFG a loop.
- **Exit block** — a block *outside* the loop that an exiting block branches to when
  the loop finishes.
- **Loop-simplify form** — a loop normalized so it has exactly one preheader,
  exactly one latch (hence a single back-edge), and **dedicated** exit blocks (exit
  blocks reached *only* from inside this loop, not also from unrelated code).
  `simplifyLoop` is the routine that rewrites a loop into this shape; the unroller
  requires it so all the blocks above are guaranteed to exist and be unique.
- **LCSSA form** (Loop-Closed SSA) — a normalization that funnels **every value
  crossing out of the loop through a PHI node at the exit block**. (Recall a PHI
  node picks a value based on which predecessor block control came from.) Ordinarily
  a value computed inside the loop, say `x`, can be used directly by code anywhere
  after the loop. LCSSA forbids that — it inserts a "closing" PHI in the exit block
  and points every outside use at it:

  ```
  loop:  x = ...                loop:  x = ...
  ...                     -->   exit:  x.lcssa = phi [x, latch]   <- closing PHI
  after: use(x)                 after: use(x.lcssa)               <- now reads the PHI
  ```

  Why this helps unrolling: duplicating the body turns `x` into several copies
  (`x.0`, `x.1`, …), and code after the loop wants the *last* iteration's value.
  Because every outside use already reads the single closing PHI, the unroller only
  updates that one PHI to forward the last copy — instead of hunting down and
  rewriting arbitrarily many uses of `x` scattered across the function. `formLCSSA`
  establishes this form.
- **Trip count** — the number of times a loop's body executes. *Exact* trip
  count is a compile-time constant; *max* trip count is a constant upper bound;
  *unknown* means neither is a small constant.
- **Trip multiple** — the largest constant the trip count is provably divisible
  by, even when the exact count is unknown. If a loop runs `4*n` times, the trip
  multiple is `4` (or a multiple thereof). Used to decide whether a remainder
  loop is needed.
- **BEInsns** — the unroller's estimate of how many instructions are "the
  back-edge test" (compare + branch), which are *not* replicated when unrolling.
- **`LoopSize`** — a code-size estimate of the loop body in abstract instruction
  units (see §6).
- **Unrolled size** — the estimated size after unrolling by some `Count`:
  `(LoopSize − BEInsns) × Count + BEInsns` (§6).
- **Remainder loop** (a.k.a. prologue/epilogue loop) — a small loop generated to
  run the `TripCount mod Count` iterations that the unrolled-by-`Count` main loop
  can't cover when the trip count isn't a multiple of `Count` (§10).
- **`UnrollingPreferences` (UP)** — the struct of thresholds and flags that
  parameterizes every decision; seeded by generic defaults, then overridden by
  the target and command line (§15).
- **TTI** — `TargetTransformInfo`, the target hook layer; `getUnrollingPreferences`
  is where AMDGPU injects its thresholds (§14).
- **Peeling** — a sibling transform (not unrolling) that pulls a fixed number of
  the *first* iterations out of a loop. It shares the dispatch path and is
  mutually exclusive with unrolling; mentioned where it appears but not the focus.

---

## 3. ScalarEvolution (SCEV): a Primer

The unroller's entire mode decision (§7) hinges on *what is known about the trip
count*, and the analysis that answers that is **ScalarEvolution (SCEV)**. SCEV
also powers the dynamic cost model's constant-folding (§8.2) and the runtime
path's computability check (§10.3). It is foundational enough — and referenced
often enough below — to be worth a few minutes up front. If you already know
SCEV, skip to §4.

### 3.1 What SCEV computes

**SCEV is LLVM's algebra engine for values that change across loop iterations.**
Given an integer or pointer scalar, it produces a *closed-form symbolic
expression* describing how that value evolves as a function of the iteration
number. That is what lets the compiler reason about induction variables — and the
loop's trip count — without simulating the loop.

The expression objects are `SCEV`s. The kinds that matter here are
(`include/llvm/Analysis/ScalarEvolutionExpressions.h:41-56`):

- `scConstant` — a compile-time constant.
- `scUnknown` — an opaque runtime value SCEV cannot see inside: a function
  argument, a load result, a kernel parameter like the `n` in §16.2.
- `scAddExpr` / `scMulExpr` — sums and products of other SCEVs.
- `scAddRecExpr` — the **add-recurrence**, the central object.

### 3.2 The add-recurrence

An add-recurrence is written `{start, +, step}<L>`: a value equal to `start` on
the first iteration of loop `L` and increasing by `step` each subsequent
iteration. It is described in the source as "a polynomial recurrence on the trip
count" (`SCEVAddRecExpr`,
`include/llvm/Analysis/ScalarEvolutionExpressions.h:340-348`) and is the closed
form of an induction variable. Using the §16 kernels:

- The inner loop counter `for (j = 0; j < N; j++)` has SCEV `{0, +, 1}<inner>`.
- The byte address `&in[t*N*N + i*N + j]` that the inner loop walks is an
  add-recurrence too: `{base, +, 4}<inner>` — 4 bytes per `float`, stepping with
  `j`.
- A non-affine value such as `j*j` is also a recurrence, just a higher-degree
  one — `{0, +, 1, +, 2}<inner>`. §3.3 unpacks what that means and why it is the
  uncommon case.

The operation the unroller's cost model relies on is
**`evaluateAtIteration(k)`**: substitute a concrete iteration index `k` into an
add-recurrence to get its value there
(`include/llvm/Analysis/ScalarEvolutionExpressions.h:396-402`). `{0,+,1}<inner>`
at iteration 5 is the constant `5`; the address chrec at iteration 5 is
`base + 20`. This is exactly the call `analyzeLoopUnrollCost` makes per simulated
iteration to discover which instructions fold to constants
(`lib/Analysis/LoopUnrollAnalyzer.cpp:31-65`, §8.2) — and, when an address chrec
lands inside a constant global, why a load can be folded away entirely.

### 3.3 How a recurrence is evaluated, and why higher-order ones are rare

The notation `{0, +, 1, +, 2}<inner>` is really a recipe for a tiny adding
machine. Read it as: start a `value` at 0 and a `step` at 1, then each iteration

```
value += step
step  += 2
```

Ticking that reproduces `j*j` using nothing but additions: `0, 1, 4, 9, 16, ...`.
That is the whole idea of an add-recurrence — express a loop value as a few
running numbers updated by plain `+`, instead of recomputing it from scratch every
iteration. An affine value `{base, +, 4}` is the one-register version
(`value += 4`); a quadratic like `j*j` needs a second register whose own step is
the constant `2`.

**Why every polynomial fits this shape — finite differences.** Tabulate `j*j` and
take successive differences (the discrete analogue of derivatives):

```
j*j:    0    1    4    9   16
          +1   +3   +5   +7        first differences
             +2   +2   +2          second differences  (constant!)
```

A degree-`d` polynomial always has constant `d`-th differences (and zero beyond),
just as the `d`-th derivative of a degree-`d` polynomial is constant. The starting
differences down the left edge — here `0, 1, 2` — *are* the recurrence
`{0, +, 1, +, 2}`. So the number of `+` terms equals the polynomial's degree.

**Jumping straight to iteration `n` — the closed form.** You rarely want to tick
the machine `n` times; you want the value at iteration `n` directly. Newton's
forward-difference formula gives it from the starting differences `d_k`:

```
value(n) = d0·C(n,0) + d1·C(n,1) + d2·C(n,2) + ...
```

where `C(n,k)` is the binomial coefficient "n choose k" — here it counts how many
of the `n` iterations a degree-`k` difference contributes to the running value.
For `j*j` (`d0=0, d1=1, d2=2`): `0 + n + 2·[n(n−1)/2] = n + n(n−1) = n²`. This is
the same shape as a Taylor series, with *difference* in place of *derivative*.
This closed form is exactly what **`evaluateAtIteration(n)`** (§3.2) computes — it
lets the cost model ask "what is this value at iteration 7?" without running the
loop to 7.

**Why the unroller cares.** When `analyzeLoopUnrollCost` (§8.2) judges a full
unroll, it evaluates each expression at iterations `0, 1, 2, ...` via this closed
form. If an array *index* comes out constant at every iteration and the array is
itself constant, the **load folds to a constant** — the memory read is replaced by
the literal value sitting there, the unrolled body is far cheaper than its
instruction count suggested, and the decision tips toward unrolling (the boost of
§8.1).

**The caveat — the simple case is overwhelmingly the common one.** Almost every
induction variable and address a loop actually contains is **affine**: a single
constant step, `{start, +, step}`, degree 1. Loop counters, `a[i]`,
`a[stride*i + c]`, pointer bumps — all affine. That is the case the unroller's
folding leans on, and the case the rest of this guide assumes. Higher-order
recurrences like `j*j` do occur, and SCEV represents them faithfully with the
extra `+` terms above, but they are uncommon. In the affine case the machine
degenerates to a single `value += step` with a fixed step, and the closed form
collapses to the familiar `value(n) = start + step·n`.

### 3.4 Trip count, backedge-taken count, and "could not compute"

SCEV expresses how many times a loop runs via the **backedge-taken count
(BECount)** — the number of times the back-edge is taken, one less than the trip
count (`TripCount = BECount + 1`). SCEV derives BECount symbolically from the exit
condition; for `for (j = 0; j < N; j++)` it gets `BECount = N − 1`, hence
`TripCount = N`.

The answer comes in three flavors (`ScalarEvolution::ExitCountKind`,
`include/llvm/Analysis/ScalarEvolution.h:864-893`):

- **`Exact`** — the precise BECount, when SCEV can prove it. Yields an exact trip
  count (our `constexpr N`).
- **`ConstantMaximum`** — a constant *upper bound*, when the exact count is
  symbolic but bounded. Yields the *max* trip count used by bounded unroll (§7,
  rung 4).
- **`SCEVCouldNotCompute`** — SCEV gives up; the trip count is "unknown."

SCEV cannot compute a count when the exit is not an affine function of an
induction variable: a data-dependent exit (`while (work_left())`), a non-affine or
pointer-chasing recurrence, a condition involving an opaque `scUnknown`, or
aliasing it can't disambiguate. The §16.2 runtime kernel is the simplest case —
the bound `n` is a kernel argument (an `scUnknown`), so although SCEV knows the
counter is `{0,+,1}<inner>`, it cannot turn "`< n`" into a constant; both the
exact and max trip counts are unavailable, leaving only the runtime path.

One more derived fact SCEV provides is the **trip multiple**: even with an unknown
exact count, the *structure* of the BECount expression can prove the trip count is
always divisible by some constant (a `for (i = 0; i < 4*m; i++)` loop runs a
multiple of 4 times). This is what lets runtime unrolling skip the remainder loop
when the unroll factor divides the multiple (§10.1).

§12 returns to SCEV to cover the three specific trip-count queries the unroller
calls and how they map onto the three unroll modes.

---

## 4. Where the Passes Live: Pipeline Placement

All unroll-pass insertion happens in the new Pass Manager builder
`lib/Passes/PassBuilderPipelines.cpp`; there is no legacy `PassManagerBuilder` in
this tree, so this one file is authoritative.

### 4.1 The master switch

First, one definition: a loop is **forced** if it carries an explicit unroll
request — a `#pragma unroll` or the `llvm.loop.unroll.*` metadata it lowers to
(§13). Everything else relies on the cost model and is "automatic."

There is one global on/off switch, `PipelineTuningOptions::LoopUnrolling`,
defaulting to `true` (`lib/Passes/PassBuilderPipelines.cpp:281`); the frontend
sets it to `false` for `-fno-unroll-loops`. It reaches the passes through a single
constructor argument, `OnlyWhenForced`, which is its **logical opposite**:
`OnlyWhenForced = !PTO.LoopUnrolling`. A pass built with `OnlyWhenForced = true`
skips every loop *except* forced ones. The two booleans say the same thing from
opposite directions, so it is easiest to read as a table:

| `PTO.LoopUnrolling` | ⇒ `OnlyWhenForced` | what the passes unroll |
|---|---|---|
| `true` (default) | `false` | automatic (cost-model) loops **and** forced loops |
| `false` (`-fno-unroll-loops`) | `true` | **only** forced loops |

So `-fno-unroll-loops` does not disable unrolling outright — it disables the
*automatic* path while still honoring an explicit `#pragma unroll`. This flows
uniformly into **every** unroll pass (the early full-unroll pass and the late
general one), since they all take the same `OnlyWhenForced` argument.

### 4.2 The early full-unroll-only pass

`LoopFullUnrollPass` is added inside the **function-simplification** pipeline.
That pipeline runs interleaved with inlining: LLVM inlines bottom-up over the call
graph (in what it calls the CGSCC pass manager — the scope that processes call-graph
groups), and right after a function has its callees inlined, this simplification
pipeline cleans the result up. So the early full unroll happens on freshly-inlined
code, **well before** the loop vectorizer and the late unroll pass (§4.3) ever run.
It is added at:

- O2/O3: `lib/Passes/PassBuilderPipelines.cpp:633-635`, inside
  `buildFunctionSimplificationPipeline`.
- O1: a parallel clone in `buildO1FunctionSimplificationPipeline` at
  `lib/Passes/PassBuilderPipelines.cpp:454-456`.

Both construct it as
`LoopFullUnrollPass(Level.getSpeedupLevel(), /*OnlyWhenForced=*/!PTO.LoopUnrolling, PTO.ForgetAllSCEVInLoopUnroll)`.
The three arguments are:

- `Level.getSpeedupLevel()` — the optimization level as a plain integer
  (O1→1, O2→2, O3→3). This is how "are we at `-O3`?" reaches the cost model: the
  default size threshold is 300 at O3 and 150 below it (§6, §14).
- `OnlyWhenForced` — the master-switch argument from §4.1.
- `PTO.ForgetAllSCEVInLoopUnroll` — unrolling rewrites the loop, so SCEV's cached
  facts about it go stale and must be discarded ("forgotten"). This flag chooses
  *how much* to discard: all of SCEV's cached loop info (`true`), or just the
  unrolled loop's (`false`, the default — cheaper, less to recompute). Purely a
  compile-time economy; it does not change which loops get unrolled.

It sits in a **loop-pass manager** (`LPM2`) — a sequence of passes each run over
every loop — next to three neighbors worth knowing by name:

- `LoopIdiomRecognizePass` — spots a loop that implements a standard operation
  (zeroing or copying memory, a popcount, …) and replaces the whole loop with a
  `memset`/`memcpy`/intrinsic.
- `IndVarSimplifyPass` — tidies a loop's induction variables: canonicalizes them,
  widens narrow ones to avoid repeated sign/zero-extends, and rewrites exit values
  in closed form via SCEV, giving later passes a clean shape to analyze.
- `LoopDeletionPass` — removes loops that are provably dead (no side effects and
  unused results, or that never execute).

Right after `LPM2`, the enclosing function pipeline runs `SROAPass` — **Scalar
Replacement of Aggregates**, the pass that breaks a local `alloca`'d struct or
array into individual SSA values living in registers instead of memory. The
comment at `lib/Passes/PassBuilderPipelines.cpp:652-653` ("Delete small array
after loop unroll") captures why full unroll runs *here*: once the loop is
straight-line and its array indices are constants, SROA can dissolve the stack
array into registers entirely.

There is one narrow exception where the early pass is left out. Some builds tune
themselves using a **profile** — measurements from a real run of the program,
recording which parts are hot and how often each loop spins. One common way of
collecting that profile ties each measurement to the code by **source line
number**. Unrolling copies a loop's body, so its source lines would suddenly
appear several times and that line-by-line matching would stop lining up. So in
that kind of build the early full unroll is skipped — and done later instead, once
the profile is already attached — to keep the measurements matched to the right
code. (For readers who know the terminology: this is the ThinLTO pre-link phase
under sample-based PGO, `lib/Passes/PassBuilderPipelines.cpp:631-632`.)

This pass does **full unroll only** — but it has no private logic of its own for
that. It funnels into the very same decision function (`tryToUnrollLoop` →
`computeUnrollCount`, §5) as the **late general pass** of §4.3, the one that *also*
does partial and runtime unrolling. Both passes share that core; the early pass
simply hands it a flag (`OnlyFullUnroll`) that throws away any answer other than a
complete unroll. §5.1 shows exactly how that flag enforces the restriction.

### 4.3 The late general pass

The late `LoopUnrollPass` is added by a pipeline-building helper called
`addVectorPasses`. That helper is used in two different builds, and it has an `if`
inside that picks one of two paths depending on which build is running:

- **The ordinary path** (`if (!IsFullLTO)`, `lib/Passes/PassBuilderPipelines.cpp:1236-1238`,
  reached from `buildModuleOptimizationPipeline` at `:1364`) — taken for a normal
  per-file compile. **This is the path hipcc's default `-O3` takes**, and the one
  this section describes.
- **The LTO path** (`if (IsFullLTO)`, `:1144-1146`) — taken only for a *link-time
  optimization* build. LTO is an opt-in mode (`-flto`) where, instead of optimizing
  each source file on its own, the compiler waits until link time and optimizes
  across all the files together. It adds its own near-identical copy of the unroll
  pass (and the early full-unroll pass of §4.2 likewise has an LTO copy at
  `:1836-1838`).

Both copies behave the same, so from here on this guide follows the ordinary path
and ignores the LTO one. On that path the pass is constructed as
`LoopUnrollPass(LoopUnrollOptions(Level.getSpeedupLevel(), !PTO.LoopUnrolling, PTO.ForgetAllSCEVInLoopUnroll))`.

It runs *after* the loop vectorizer (also in `addVectorPasses`), surrounded by
`InstCombinePass`, an (off-by-default) unroll-and-jam pass, a
`WarnMissedTransformationsPass`, a CFG-preserving `SROAPass`, and a final `LICM`.
This is the pass that performs partial and runtime unrolling.

The settings handed to the pass (a `LoopUnrollOptions` bundle) include three
optional switches that *could* force the unroll modes on or off: `AllowPartial`
(partial unroll), `AllowRuntime` (runtime unroll), and `AllowPeeling` (peeling,
defined in §2). Each is a **three-way** value — it can say "yes," say "no," or be
**left blank** (`std::nullopt` is C++'s "no value set" state for such a field). The
pipeline sets only `(OptLevel, OnlyWhenForced, ForgetSCEV)` and leaves all three of
those switches **blank**. Blank means the pipeline is not forcing anything, so
whether a given loop ends up fully unrolled, partially unrolled, runtime-unrolled,
or peeled is decided entirely downstream — by the unroller's cost model (§6) and
the target's preferences (for AMDGPU, §13–§14). That is a big part of *why* the
target preferences carry so much weight: nothing in the pipeline has overridden
them. (The LTO copy at `lib/Passes/PassBuilderPipelines.cpp:1144-1146` follows the
same pattern.)

### 4.4 Unroll-and-jam is off by default

`LoopUnrollAndJamPass` is added at both unroll sites but gated by
`EnableUnrollAndJam && PTO.LoopUnrolling`, and `EnableUnrollAndJam` is
`cl::init(false)` (`lib/Passes/PassBuilderPipelines.cpp:199-201`). So it is added to
the pipeline only when someone passes `-mllvm -enable-unroll-and-jam`.

For HIP specifically, **it never runs**: hipcc passes no such flag, and the AMDGPU
backend never enables it either (the string `UnrollAndJam` does not appear anywhere
in `lib/Target/AMDGPU/`). Dumping the actual pass pipeline of a default `-O3`
device compile (`-Xclang -fdebug-pass-manager`) confirms it — `LoopUnrollAndJamPass`
runs zero times, and appears only once `-mllvm -enable-unroll-and-jam` is added by
hand. (Unroll-and-jam unrolls an *outer* loop and fuses the resulting inner-loop
copies; it is out of scope here beyond establishing that it is off.)

### 4.5 Optimization-level summary

| Level | Early `LoopFullUnrollPass` | Late `LoopUnrollPass` |
|-------|---------------------------|------------------------|
| O0 | none | none (`:1910-2009`, no unroll of any kind) |
| O1 | yes (`:454-456`) | yes (via `addVectorPasses`) |
| O2 | yes (`:633-635`) | yes |
| O3 | yes (`:633-635`); threshold default is higher (§15) | yes |

HIP device compilation runs at `-O3` by default, so both passes are active with
the aggressive threshold.

---

## 5. Anatomy of the Unroller: From Pass Entry to Transform

The end-to-end call chain is:

```
LoopFullUnrollPass::run  (loop pass)  ─┐
                                       ├─► tryToUnrollLoop ─► computeUnrollCount ─► UnrollLoop
LoopUnrollPass::run      (func pass)  ─┘        │                                      │
                                               ├─ gatherUnrollingPreferences          ├─ UnrollRuntimeLoopRemainder (runtime mode)
                                               └─ ApproximateLoopSize                  └─ clone + rewire + (maybe) delete loop
```

### 5.1 The two entry points

The two entry points differ deliberately in **when** they run and **what kind of
pass** they are, because full unroll and partial/runtime unroll unlock different
*kinds* of benefit:

- **Early, a *loop* pass** — `LoopFullUnrollPass`. Full unroll is a **scalar**-
  optimization enabler: with the trip count gone, every copy has a *constant*
  induction value, so loads fold, constants propagate, and local arrays become
  promotable (§4.2). It runs early so the whole scalar mid-end (SROA, GVN,
  instcombine, more inlining) can cash those in. Being a loop pass lets it ride in
  the early loop-canonicalization manager (`LPM2`) beside `IndVarSimplify`/
  `LoopDeletion`, sharing one loop-nest walk and the cached analyses; and since it
  only ever *deletes* a loop, the loop-pass framework handles its structural updates
  cleanly (the bookkeeping below).
- **Late, a *function* pass** — `LoopUnrollPass`. Partial/runtime unroll is instead
  an **instruction-level-parallelism** enabler: its copies keep *runtime* induction
  values (structural duplicates with shifted indices), so they expose little *new
  scalar* work — the payoff is more independent work per real iteration and fewer
  branches, realized downstream by the backend scheduler and register allocator. It
  also *must* run **after** the loop vectorizer (unrolling first would inhibit
  vectorization, and the vectorizer can shrink the body — §4.3), which sits after the
  heavy mid-end, so it necessarily lands late. The new bodies are still cleaned up —
  a trailing `InstCombine` + CFG-preserving `SROA` + `LICM` follow (§4.3) — there is
  just no reason to re-run the *entire* scalar pipeline over `C` near-identical
  copies. Being a function pass gives it whole-function latitude the per-loop
  contract can't: it canonicalizes every loop up front (which can expose new loops),
  runs its own worklist, and *creates* new loops (the runtime remainder loops) as it
  goes.

Both then funnel into the same shared core, `tryToUnrollLoop` (covered below).

**`LoopFullUnrollPass::run`** (`lib/Transforms/Scalar/LoopUnrollPass.cpp:1480`)
is a *loop* pass — it runs once per loop, receiving the loop and the bundled
`LoopStandardAnalysisResults`. It calls `tryToUnrollLoop` with
`OnlyFullUnroll=true` and hard-disables the other modes:
`AllowPartial=false, Runtime=false, UpperBound=false` (but `AllowPeeling=true`)
(`lib/Transforms/Scalar/LoopUnrollPass.cpp:1499-1509`). After a successful full
unroll it has to do some **loop-nest bookkeeping**. Loops form a tree: an outer
loop is the *parent* of the inner loops directly inside it, and inner loops sharing
a parent are *siblings*. The loop pass manager walks that tree and runs the pass on
each loop, so a transform that reshapes the tree must report what changed. Fully
unrolling an outer loop reshapes it two ways: the outer loop is **deleted** (it is
now straight-line code, no loop left), and any inner loop it contained is **copied
once per unrolled iteration** — and those copies, no longer sitting inside the
now-gone outer loop, pop up one level as new sibling loops. So the pass tells the
manager "this loop is deleted, don't visit it again" and "these new sibling loops
appeared, do visit them," so the copies still get their own optimization turn
(`:1510-1563`).

**`LoopUnrollPass::run`** (`lib/Transforms/Scalar/LoopUnrollPass.cpp:1566`) is a
*function* pass. It first forces every loop into loop-simplify + LCSSA form —
`simplifyLoop` + `formLCSSARecursively` on each loop, so simplification happens
even if nothing is unrolled (`:1571-1600`). It then walks loops via a priority
worklist and calls `tryToUnrollLoop` with `OnlyFullUnroll=false` and the
flavor knobs taken from its options (`:1626-1633`).

Finally, it can switch peeling off for a loop when profile data says the program
has a huge **working set** — a rough, profile-derived measure of how much *hot code*
there is (roughly, the number of frequently-executed basic blocks); a large one
implies instruction-cache pressure, so duplicating code is risky. But this is gated
on actually having a profile: the code is `if (PSI && PSI->hasHugeWorkingSetSize())`,
so for a default no-PGO build — **including HIP device compiles** — there is no
profile, the condition is false, and peeling is never throttled here (`:1604-1633`).

**Both passes visit loops bottom-up — innermost first.** The late pass fills its
worklist with `appendLoopsToWorklist` and pops from the back with `pop_back_val()`
(`:1604-1612`); that helper lays the loops out in preorder (parent before child)
and the worklist is LIFO, so the deepest loops come off first
(`lib/Transforms/Utils/LoopUtils.cpp:1556-1592`). The early `LoopFullUnrollPass` is
driven by `FunctionToLoopPassAdaptor`, which uses the same
`appendLoopsToWorklist` + `pop_back_val()`
(`lib/Transforms/Scalar/LoopPassManager.cpp:262,287`). So an inner loop is always
unrolled *before* its enclosing loop — which means the outer loop's size budget
(§6) is measured against a body that *already* contains the unrolled inner. That
ordering is the direct cause of the "inside-out" behavior in §16.1.

Both share `tryToUnrollLoop` (`:1123`), whose `OnlyFullUnroll` parameter is the
single bit that makes the early pass full-only: after a count is chosen, if
`OnlyFullUnroll && !(UP.Count >= MaxTripCount)` it returns `Unmodified`
(`:1308-1313`). So the early pass uses the *same* `computeUnrollCount` but throws
away any answer that isn't a complete unroll.

### 5.2 `tryToUnrollLoop`, step by step

`tryToUnrollLoop` (`lib/Transforms/Scalar/LoopUnrollPass.cpp:1123-1361`) is the
shared driver — **both** entry points (§5.1) call it, so every step and bailout
below applies equally to the early full-unroll pass and the late general pass. In
order:

1. **Metadata-disable bailout.** If `hasUnrollTransformation(L) & TM_Disable`
   (i.e. `llvm.loop.unroll.disable` or `unroll.count == 1`), return `Unmodified`
   (`:1140-1142`).
2. **Unroll-and-jam interaction bailouts.** Don't auto-unroll a loop whose
   parent — or which itself — is the target of an unroll-and-jam pragma, unless
   this loop is itself force-unrolled (`:1148-1166`).
3. **Form check.** Bail unless the loop is in loop-simplify form (§2)
   (`:1168-1172`). In practice this always passes: the late pass simplified every
   loop itself just before (§5.1, `:1571-1600`), and the early pass's loop adaptor
   prepends `LoopSimplifyPass` + `LCSSAPass` to every loop pipeline
   (`include/llvm/Transforms/Scalar/LoopPassManager.h:450-451`). It is a defensive
   guard, not a common exit.
4. **`OnlyWhenForced` gate.** `TM` here is the loop's *transformation mode* — a
   small flag from `hasUnrollTransformation(L)` describing any explicit unroll
   directive on the loop (§12); `TM_Enable` is the bit meaning "the loop asks to be
   unrolled," set by `#pragma unroll` / `unroll(enable)` / `#pragma unroll N` (N>1)
   / `unroll.full`. So this step says: when automatic unrolling is globally off
   (`OnlyWhenForced`, e.g. `-fno-unroll-loops`, §4.1), skip every loop that lacks
   such a directive — honor only pragma-forced loops (`:1176-1177`).
5. **Gather preferences.** Build the structs that hold every unroll setting the
   later steps consult: `UP` (`UnrollingPreferences`) via
   `gatherUnrollingPreferences`, and `PP` (`PeelingPreferences`) via
   `gatherPeelingPreferences` (`:1179-1188`). What those settings *mean* comes later
   — §6 (the cost model) and §14–§15 (the full list); the point *here* is just **how
   they are assembled** — five layers, each able to override the one before it, so
   the last to speak wins:
   - the built-in **defaults**;
   - the **target's** adjustments (`TTI.getUnrollingPreferences`, where AMDGPU tweaks
     them — §14);
   - a **size-mode** override (when the function is compiled for minimum code size,
     `-Os`/`-Oz`, the settings are dialed down so unrolling won't grow the code);
   - any **command-line** `-unroll-*` flags that were actually passed;
   - finally, any overrides **baked into the pass when it was constructed** (e.g. a
     forced unroll count handed to the pass at creation time) — left unset in the
     normal pipeline, so this last layer is usually a no-op.

   So, for example, a command-line flag beats AMDGPU's target setting, which beats
   the built-in default.
6. **Cheap early-out.** A loop is only unrolled if the unrolled result stays within
   a **size budget** — a cap on how much bigger unrolling may make it (this budget is
   the "threshold"; the cost model around it is §6). The full-unroll budget is
   `UP.Threshold`. If it is zero, and partial unrolling is likewise off or
   zero-budget, nothing could ever be unrolled — so, unless the function is being
   built for minimum code size, give up now without even measuring the loop
   (`:1190-1194`).
7. **Measure size.** Estimate the loop's code size as `LoopSize` via
   `ApproximateLoopSize` (§6). Then bail in three cases: the size can't be estimated
   (some instruction has no known size); the loop contains something that can't be
   copied (an indirect branch, or an instruction marked "no-duplicate") and unrolling
   would have to copy it; or it still has function calls that might later be inlined
   (unrolling now would be premature, since inlining changes the size) (`:1196-1225`).
8. **Find the trip count.** Ask SCEV (§3) how many times the loop runs. A loop can
   have **more than one** *exiting* block — a block with an edge leaving the loop —
   e.g. one for an early `break` and one at the latch for the `i < n` test
   (loop-simplify form forces a single *latch*/back-edge, but **not** a single exit).
   So scan every exiting block and take the smallest **exact** constant count any of
   them yields — the exit guaranteed to fire first, which bounds the whole loop. If
   none is constant, fall back to a trip *multiple* — the constant the count is
   provably divisible by (§2) (`:1227-1252`).
9. **Convergence.** If the loop contains *convergent* operations — GPU instructions
   that only work if a group of threads runs them together (cross-lane/subgroup ops,
   barriers) — forbid a remainder loop by setting `UP.AllowRemainder = false`, because
   a remainder loop (§2) adds branching that can split up which threads run together
   and break those operations (`:1252-1265`).
10. **Max trip count.** Only when no *exact* count was found, fall back to a constant
    **upper bound** on the count (§3), plus a flag `MaxOrZero` that records whether the
    loop runs either exactly that many times or not at all (never in between). Both
    feed the "bounded" full-unroll path later (`:1267-1274`).
11. **Decide the count.** Hand off to `computeUnrollCount`, the master decision
    function (§7). It picks the unroll factor and writes it into `UP.Count` (along with
    `UP.Runtime`, etc.), and reports whether that factor was forced by an explicit user
    request (a `#pragma` or `-unroll-count`) or chosen automatically. If it lands on
    `UP.Count == 0` ("don't unroll"), bail (`:1276-1283`).
12. **Peeling shortcut.** If the decision instead chose to *peel* (`PP.PeelCount > 0`
    — pull a few leading iterations out of the loop, §2), do the peel and return;
    peeling and unrolling are mutually exclusive (`:1285-1306`).
13. **Full-only guard** (only fires for the early pass). The early `LoopFullUnrollPass`
    and the late general pass run this same decision code (§5.1), so the step-11
    decision can come back as a *partial* or *runtime* unroll — but the early pass is
    only allowed to do *full* unrolls. So when this run is on the early pass's behalf
    (it set the `OnlyFullUnroll` flag) and the chosen unroll isn't a full one (a loop
    would still remain afterward), abandon it here and leave the loop for the late pass
    (`:1308-1313`).
14. **Final runtime gate.** A last condition decides whether runtime unrolling
    *actually* happens. Even if step 11 turned it on, it stays on only when **both**
    hold: the trip count is genuinely unknown, **and** the chosen `Count` does not
    evenly divide the provable trip multiple. (If `Count` *does* divide the multiple,
    the leftover iterations work out to zero — there would be nothing for a remainder
    loop to do — so runtime unrolling is dropped, §10.) That single line in the source
    is `UP.Runtime &= (TripCount == 0 && TripMultiple % UP.Count != 0)` (`:1315-1320`).
15. **Transform.** Bundle the chosen settings into a `UnrollLoopOptions` and call
    `UnrollLoop` (§11) — the routine that actually rewrites the IR — capturing any
    remainder loop it produces. Afterward, carry the loop's metadata over to the result
    and, when appropriate, mark the loop "already unrolled" so a later pass won't unroll
    it again (`:1322-1361`).

---

## 6. Measuring Loop Size

Every threshold comparison is against a code-size estimate, so the estimate is
where to start.

### 6.1 Computing `LoopSize` with `ApproximateLoopSize`

`ApproximateLoopSize` (`lib/Transforms/Scalar/LoopUnrollPass.cpp:665-690`) runs
`CodeMetrics::analyzeBasicBlock` over every block of the loop and returns
`Metrics.NumInsts` as the loop size. It also reports separately whether the loop has
calls that might still be inlined (`NumInlineCandidates`), is non-duplicatable, or
is convergent.

One subtlety: *ephemeral* instructions are excluded from the count. An `llvm.assume`
is a special call that only tells the optimizer a fact it may rely on (e.g. "this
value is non-negative") and generates no machine code; any instructions that exist
*solely* to compute what such an assume checks are "ephemeral" — they emit no real
code either, so counting them would inflate the size estimate.

`NumInsts` is an abstract cost figure from the target's cost model
(`TargetTransformInfo`, abbreviated TTI — §2), not a literal count of IR
instructions, but for intuition it tracks "how many machine-ish operations the body
is."

The size is floored at `BEInsns + 1` (default `3`) so a degenerate near-empty
loop can't be assigned a giant unroll count (`:679-689`).

### 6.2 `getUnrolledLoopSize` and the back-edge term

The estimated size *after* unrolling by `Count` is computed by
`UnrollCostEstimator::getUnrolledLoopSize`
(`lib/Transforms/Scalar/LoopUnrollPass.cpp:753-773`):

```
unrolledSize = (LoopSize − BEInsns) × Count + BEInsns
```

The `−BEInsns ... + BEInsns` structure is the model of what unrolling actually
does to the back-edge test. When you unroll by `Count`, the `Count` body copies
are chained back-to-back with **no exit test between them** — they fall through
one into the next. Only at the bottom of all `Count` copies is there a single
compare-and-branch deciding whether to iterate again. So each copy contributes
its real work *minus* the back-edge test it no longer needs
(`LoopSize − BEInsns` per copy), and the still-looping result keeps exactly one
back-edge (`+ BEInsns`). (That single bottom test means the loop can only exit at a
*multiple* of `Count` — fine for a known trip count, but for an **unknown** one it is
exactly why a runtime remainder loop is needed; see §10.)

For **partial** and **runtime** unroll this is exact: the loop survives, with one
back-edge. For **full** unroll the loop is deleted and there is no back-edge at
all, so the formula over-estimates by one back-edge's worth — a small, deliberate
conservative constant, since the helper is shared across all three modes and the
over-count (2 instructions, or 5 on AMDGPU) is negligible against
`(LoopSize − BEInsns) × Count`.

`BEInsns` defaults to `2` (`:204`); **AMDGPU bumps it to 5** (`+= 3`, §14).

---

## 7. `computeUnrollCount`: The Seven-Priority Decision

`computeUnrollCount` (`lib/Transforms/Scalar/LoopUnrollPass.cpp:889-1120`) is the
heart of the unroller — the function that actually decides **how much** to unroll.
Its real result is a *side effect*: it writes the chosen unroll factor into
`UP.Count` (where `0` means "don't unroll"). Its separate `bool` **return value** is
a secondary signal — it just reports whether that factor was *forced by the user* (a
`#pragma` or `-unroll-count`) rather than chosen automatically; a later step uses that
flag for bookkeeping (e.g. tagging the loop "already unrolled," §5.2 step 15). To pick
the factor it walks a strict priority ladder, stopping at the first rung that fires;
the rungs map directly onto the three unroll modes.

First it computes the explicit-request signals
(`lib/Transforms/Scalar/LoopUnrollPass.cpp:898-909`). The three `Pragma*` ones all
come from source `#pragma`s (the clang frontend lowers each pragma to a piece of
`!llvm.loop` metadata, §12); only `UserUnrollCount` is a compiler flag:

- `UserUnrollCount` — the `-unroll-count` command-line flag was passed (a testing
  knob, not a source directive);
- `PragmaFullUnroll` — the loop carries `llvm.loop.unroll.full`, emitted by
  `#pragma clang loop unroll(full)` ("fully unroll this loop");
- `PragmaCount` — the loop carries `llvm.loop.unroll.count` = N, emitted by
  `#pragma unroll N` (with a number) or `#pragma clang loop unroll_count(N)`;
- `PragmaEnableUnroll` — the loop carries `llvm.loop.unroll.enable`, emitted by a
  bare `#pragma unroll` or `#pragma clang loop unroll(enable)`;
- `ExplicitUnroll` — true if any of the above is set.

Then the ladder, checked top to bottom — the first rung whose **condition** holds
wins. Read each row as *condition* → *what it does* (with the function that makes the
call):

| Priority | Rung | Mode | Citation |
|----------|------|------|----------|
| 0 | an explicit peel count was set → peel (`Count=1`) | (peel) | `:910-919` |
| 1 | a `-unroll-count` flag forces the factor (`shouldPragmaUnroll`) | any | `:784-788` |
| 2 | a `#pragma unroll` directive forces the factor (`shouldPragmaUnroll`) | any | `:791-797` |
| 3 | known **exact** trip count → try full unroll (`shouldFullUnroll`, §8) | **full** | `:945-956` |
| 4 | no exact count but a small **max** trip count → bounded full unroll (`shouldFullUnroll`) | **full (bounded)** | `:958-979` |
| 5 | peeling chosen (`computePeelCount`) → peel (`Count=1`) | (peel) | `:981-987` |
| 6 | otherwise, try partial unroll (`shouldPartialUnroll`, §9) | **partial** | `:989-1024` |
| 7 | trip count unknown → runtime unroll (§10) | **runtime** | `:1037-1119` |

Key behaviors per rung:

- **Pragma force (1–2).** When the user asked for a specific unroll, this rung
  **bypasses the cost model** and just honors the request: `shouldPragmaUnroll`
  returns the `-unroll-count` value, the `#pragma unroll N` count, or the full trip
  count for `unroll(full)` (`:775-801`). Two follow-on behaviors matter:
  - *If a specific count was demanded* (the rung "fires"), the unroller turns on the
    permissions needed to actually deliver it, even where it would normally hold back
    (`:926-932`): `UP.Force` (unroll even if the usual heuristics would say no);
    `UP.AllowExpensiveTripCount` (allow costly trip-count math — e.g. a division — to
    set the unroll up); and, for a pragma count, `UP.Runtime` (allow a runtime
    remainder loop, §10, so the requested count still applies when the trip count is
    unknown).
  - *If the request was only "please unroll" with no number* (a bare `#pragma unroll`
    / `unroll(enable)`), there is no specific count for `shouldPragmaUnroll` to return,
    so instead the size budgets are raised to `PragmaUnrollThreshold` (`16384`)
    (`:935-942`). That forces nothing directly, but makes the ordinary cost-model rungs
    (3–7) almost always choose to unroll — in effect, "the user wants this unrolled, so
    be generous."
- **A pragma count slips a *partial* unroll through the "full-only" early pass.**
  A genuine quirk worth understanding. Rung 1–2 fires *before* the full-unroll rung
  and sets `UP.Count = N` directly, so a `#pragma unroll N` is honored even by the
  early `LoopFullUnrollPass` — and on a loop with a *known* trip count it produces a
  **partial** unroll-by-`N` there, despite that pass nominally doing full unrolls only.
  (Verified: `#pragma unroll 4` on a 100-iteration loop replicates the body 4× right
  after `LoopFullUnrollPass`.)

  Why doesn't the early pass stop this? Two separate mechanisms are meant to keep it
  full-only, and a forced pragma count evades both:
  - *Automatic* (non-pragma) partials are blocked by `AllowPartial = false`, which the
    early pass passes in (§5.1): the partial rung then leaves `UP.Count = 0`, and
    `tryToUnrollLoop` bails at `if (!UP.Count)` (`:1282`, the tail of §5.2 step 11) —
    before any guard. But a pragma count is set by rung 1–2, *bypassing* the partial
    rung, so `Count` is never 0 here.
  - The explicit *full-only guard* (§5.2 step 13, `:1309`) is
    `if (OnlyFullUnroll && !(UP.Count >= MaxTripCount)) bail` — read as "in the
    full-only pass, only proceed if the count reaches the whole loop
    (`Count >= MaxTripCount` = a *complete* unroll)." But `MaxTripCount` is filled in
    *only when the exact trip count is unknown* (`:1271`); for a known trip count it
    stays `0`, so `Count >= 0` is always true and the guard never fires.

  So `AllowPartial=false` is what actually blocks *automatic* partials; this guard only
  really bites in the *unknown-but-bounded* case (there `MaxTripCount` is the real
  bound, and a too-small count is deferred to the late pass). The lone thing that slips
  past both is an explicit `#pragma unroll N` on a known-trip-count loop — which is
  benign, since it is exactly the unroll the user asked for.
- **Exact full unroll (3).** Reached when the loop has a known exact trip count.
  `UP.Count` is set to that trip count as a *working value* — `shouldFullUnroll` needs
  it to estimate the size of the fully-unrolled loop (the size formula multiplies the
  body by `UP.Count`, §6.2). `shouldFullUnroll` is a **separate helper called from
  here** (it is *not* part of `computeUnrollCount`; detailed in §8); it decides whether
  that full unroll is worth it:
  - if **yes**, the count sticks (`UP.Count` stays = trip count), the ladder stops, and
    this is the rung that ultimately **deletes the loop** (full unroll → straight-line
    code, no loop left);
  - if **no**, this rung does *not* commit — execution falls through to the later rungs
    (partial/runtime), which overwrite `UP.Count` with a different factor.

  (`UseUpperBound = false` just records that this used the *exact* trip count, as
  opposed to the upper-bound fallback in rung 4.)
- **Bounded full unroll (4).** A fallback for when the exact trip count is *unknown*
  but the compiler knows a constant **upper bound** on it (`MaxTripCount`, from §5.2
  step 10). It replicates the body by that upper bound and removes the **back-edge**
  (so it is no longer a loop) — but, *unlike* exact full unroll, it **keeps the exit
  tests**, because it knows only the maximum, not the real stopping point. So the result
  is a *chain* of up to `MaxTripCount` body copies, each able to branch out to the exit
  early (not straight-line). How many tests survive depends on `MaxOrZero`: an
  all-or-nothing loop keeps just the **first** test — one "does it run at all?" guard,
  then straight-line copies — while a generic upper bound keeps all but the last (a
  fully branchy chain). It runs only when **all** of:
  - the upper bound is **small**: `MaxTripCount <= UnrollMaxUpperBound`
    (`-unroll-max-upperbound`, default `8`) — unrolling N copies for *maybe* N
    iterations isn't worth it once N is large;
  - **either** the target enabled it (`UP.UpperBound`, off by default) **or** the loop
    is all-or-nothing (`MaxOrZero`, step 10: it runs either exactly `MaxTripCount` times
    or zero). **On AMDGPU `UP.UpperBound` is not set** — the string `UpperBound` appears
    nowhere in its TTI — so this rung is reachable on AMDGPU **only via `MaxOrZero`,
    meaning AMDGPU's bounded unrolls keep just that single front guard** (straight-line
    after it), never the branchy chain;
  - **and** `shouldFullUnroll` (§8) approves the size, just as in rung 3.

  On success, `UP.Count = MaxTripCount` and `UseUpperBound = true` — that flag just
  records that the count came from the *upper bound* rather than an exact trip count
  (rung 3 set it `false`) (`:958-979`).
- **Partial (6).** Reached, with a known trip count, when no full unroll fired above.
  First it runs `UP.Partial |= ExplicitUnroll`: `|=` is "OR-assign," so this turns the
  *partial-unrolling-allowed* flag (`UP.Partial`) **on** whenever the user explicitly
  asked for an unroll (`ExplicitUnroll`, the §7 signal above) — even if partial was
  otherwise off. (On AMDGPU `UP.Partial` is already `true`, §14.1, so this is usually a
  no-op there.) Then `shouldPartialUnroll` (§9) chooses the actual partial factor.
- **Runtime (7).** Reached only when the trip count is **unknown** (`TripCount == 0`);
  by here every constant-trip-count case has been handled above (an assert notes this,
  `:1025-1026`). The rung first checks three reasons *not* to runtime-unroll:
  - the loop carries metadata explicitly disabling it
    (`llvm.loop.unroll.runtime.disable`);
  - it has a known small **upper bound** and isn't force-unrolled
    (`MaxTripCount < UnrollMaxUpperBound`, i.e. `< 8`) — it runs too few times to be
    worth the remainder loop;
  - profiling marks it a **"flat" loop** — its measured average trip count is below
    `FlatLoopTripCountThreshold` (`5`) — so again the remainder-loop overhead wouldn't
    pay off.

  If none of those fire, it requires `UP.Runtime` to be on, sets the factor to
  `DefaultUnrollRuntimeCount` (`8`), and **halves** it until the unrolled body fits
  `PartialThreshold` (the partial-unroll size budget, §6). Finally — **and only if a
  remainder loop is *not* allowed** (`AllowRemainder = false`, e.g. a convergent loop,
  §5.2 step 9) — it keeps halving `Count` until it evenly **divides the provable trip
  multiple** (§2). That last step is the no-remainder fallback from §10: with no
  remainder loop available to mop up leftover iterations, the only safe factor is one
  the trip count is *always* divisible by — so `Count` is shrunk to a divisor of the
  trip multiple (and if that multiple is just 1, this drives `Count` to 1, i.e. no
  unroll) (`:1037-1119`).

---

## 8. Full Unroll: Static Size Test and Dynamic Cost Model

`shouldFullUnroll` (`lib/Transforms/Scalar/LoopUnrollPass.cpp:803-831`) decides
whether to fully unroll a loop of a given `FullUnrollTripCount`. It has three
escalating gates:

1. **Hard cap.** If `FullUnrollTripCount > UP.FullUnrollMaxCount`
   (default `UINT_MAX`, so effectively never) → `nullopt` (`:810-811`).
2. **Cheap static size test.** If `getUnrolledLoopSize(UP) < UP.Threshold`
   (strict `<`), return the trip count — unroll
   (`:815-816`). This is the common, fast path: if the whole unrolled body is
   small, just do it. (The caller set `UP.Count` to the trip count before
   calling, so `getUnrolledLoopSize` uses it.)
3. **Dynamic profitability.** If gate 2 failed — the raw unrolled body looks too big —
   don't give up yet. Full unroll makes the loop counter a *constant* in every copy,
   which can fold away a lot (constant math, address calculations, loads from constant
   tables, dead branches), so the raw size badly *overestimates* the real cost. This
   gate measures the real cost instead: `analyzeLoopUnrollCost` (§8.2) simulates the
   unrolled loop and returns `UnrolledCost`, counting only the instructions that
   *survive* folding. The loop is fully unrolled if that real cost fits a budget that
   has been **stretched** according to how much unrolling saves — up to 4× the normal
   `Threshold`; otherwise → `nullopt` (`:821-830`). §8.1 works through exactly how that
   stretched budget is computed (and why the formula is a strange one).

### 8.1 The boost (and why the formula is strange)

The boost is computed by `getFullUnrollBoostingFactor`
(`lib/Transforms/Scalar/LoopUnrollPass.cpp:737-747`) from the two costs that
`analyzeLoopUnrollCost` (§8.2) produces:

- **`UnrolledCost`** — the cost of the fully-unrolled code, charging only the
  instructions that *survive* folding. The constant-folding discount is already baked
  in here.
- **`RolledDynamicCost`** — the *baseline*: the cost the original *rolled* loop actually
  executes (every instruction, every iteration), with **no** folding discount. It
  measures "how much work the loop does."

The boost is their ratio, capped:

```
Boost = min(100 × RolledDynamicCost / UnrolledCost, MaxPercentThresholdBoost)   // cap 400 by default
```

So it is a *benefit-per-cost* number: a loop whose work (`RolledDynamicCost`) collapses
into little surviving code (`UnrolledCost`) earns a high boost, up to **4×**. Gate 3
then stretches the size budget by it: full-unroll iff
`UnrolledCost < Threshold × Boost / 100`.

**Watch what that actually computes — the test is quadratic in `UnrolledCost`.** The
folding-reduced `UnrolledCost` appears on *both* sides: directly on the left, and inside
the boost on the right. Substituting the boost (while it is below the 4× cap):

```
UnrolledCost   <  Threshold × (RolledDynamicCost / UnrolledCost)
UnrolledCost²  <  Threshold × RolledDynamicCost
UnrolledCost   <  √(Threshold × RolledDynamicCost)
```

So the real criterion is: *the folded code size must be below the **geometric mean** of
the base budget and the loop's raw dynamic work.* Because the same `UnrolledCost` does
double duty, folding is rewarded **twice over** — a smaller `UnrolledCost` both lowers
the left side *and* raises the budget — weighed against the folding-independent baseline
`RolledDynamicCost`.

This is an odd way to express a profitability test. It reads like a heuristic — "stretch
the threshold by the benefit/cost ratio" — where the quadratic sensitivity is a *side
effect* of using a ratio as a multiplier, not a deliberate decision to compare
`UnrolledCost²`. (Above the cap the quadratic shape disappears: once
`RolledDynamicCost / UnrolledCost ≥ 4`, `Boost` pins at `400` and the test becomes the
plain linear `UnrolledCost < 4 × Threshold`, i.e. ≤ 1200 with the AMDGPU defaults.)
Directionally it does what's intended — strongly favor fully unrolling loops that fold a
lot — but the precise shape looks more like a consequence of the arithmetic than a
chosen design.

### 8.2 The dynamic cost model: `analyzeLoopUnrollCost`

`analyzeLoopUnrollCost` (`lib/Transforms/Scalar/LoopUnrollPass.cpp:333-662`) is
exclusively a **full-unroll** profitability model. It **symbolically simulates
every iteration** of an innermost loop with a known constant trip count to see
whether folding makes the unrolled body cheaper than its raw size suggests.

- **Applicability gates.** Only innermost loops (`:360`); only if
  `TripCount != 0 && TripCount <= MaxIterationsCountToAnalyze` (default `10`,
  **AMDGPU raises to `32`**, §14) (`:364-365`).
- **Two accumulators.** `UnrolledCost` is the estimated cost of the unrolled loop. An
  instruction is charged toward it only if **both** of these hold (`:397-476`):
  1. *It is reachable from a root.* The model doesn't sum every instruction; it walks
     **backward** from the instructions that can't be deleted — the *observable roots*:
     a side-effecting instruction (e.g. a store — it must happen), a block's terminator
     (control flow must be preserved), or a value used after the loop. From each root it
     follows operands backward.
  2. *It didn't fold to a constant.* Each reached instruction is charged only if its
     per-iteration **"free" flag is clear**, and that flag is set whenever the
     instruction folded to a constant (by the forward `visit`, next bullet).

  An example shows why *both* filters are needed. A **store** is always charged (it's a
  root). The address arithmetic feeding it is *reached* by the backward walk — but it is
  charged **only if it didn't fold**: if unrolling made the loop index constant so the
  address collapses to a constant, that math is free and *not* charged (just the store
  costs); if the address still depends on a runtime value, the math *is* charged. So in
  short, an instruction counts iff **reachable-from-a-root *and* not-folded**.
  `RolledDynamicCost`, by contrast, is the naive baseline: it counts *every* instruction
  executed in *every* simulated iteration (`:532-540`). The gap between the two is the
  "savings" unrolling would unlock.
- **Constant tracking.** Per iteration, an `UnrolledInstAnalyzer`
  (`lib/Analysis/LoopUnrollAnalyzer.cpp`) resolves each instruction's SCEV at the
  current iteration index (this is `evaluateAtIteration`, §3.2). If it is a
  constant — or an add-recurrence that evaluates to a constant at this iteration —
  it is recorded in `SimplifiedValues`
  and treated as free (`lib/Analysis/LoopUnrollAnalyzer.cpp:31-65`). Header PHIs
  are pre-seeded each iteration with their resolved incoming value
  (`lib/Transforms/Scalar/LoopUnrollPass.cpp:496-519`).
- **The killer feature — folding loads.** `visitLoad` folds a load to a constant
  when its address is a previously-tracked offset into a **constant global** with
  a `ConstantDataSequential` initializer and the index is in bounds
  (`lib/Analysis/LoopUnrollAnalyzer.cpp:97-143`). This is why fully unrolling a
  loop that reads a `const` lookup table can collapse to a sequence of constants.
- **Early exits**, each returning `nullopt` ("no usable dynamic estimate," so gate 3
  is skipped and gate 2's static test stands):
  - a **real (lowered) call** is hit (`:555-563`). This analysis only pays off by
    finding *folding* savings, and a real function call is an opaque black box — it
    can't be folded or seen through, and N copies of it is just N calls, no win. Only
    calls that lower to an *actual* function call trip this (via `TTI.isLoweredToCall`),
    plus indirect/unknown callees; intrinsics that compile to inline code (`memcpy`,
    math intrinsics, …) are fine.
  - `UnrolledCost` exceeds the boosted ceiling — too big to be worth analyzing further
    (`:571-577`);
  - after one full iteration `UnrolledCost == RolledDynamicCost` — no folding happened
    on the first iteration, so none will on later ones either (`:631-635`).
- **No comparison inside.** The function just returns `{UnrolledCost,
  RolledDynamicCost}` (`:654-662`); the decisive `<` comparison is back in
  `shouldFullUnroll` (§8, gate 3).

The practical consequence for typical AMDGPU kernels — and the key to the §16 worked
example. **Watch out: "global" means two unrelated things here.**

- AMDGPU **address space 1** is *global device memory* — the GPU's main memory, where a
  kernel's input/output buffers live. Its contents are **runtime data the compiler does
  not know**.
- The only loads `visitLoad` can fold (above) are loads from a **constant global
  *variable*** — a compile-time-known table baked into the program (e.g. a `const`
  lookup array with a fixed initializer), whose contents the compiler **does** know.

A normal kernel's `in[i]` reads address-space-1 device memory (the first kind), not a
constant global variable (the second kind). So even when unrolling makes the *index* a
constant, `visitLoad` can't fold the load — it knows the **address** but not the
**contents**. With the per-cell load uncollapsible, `UnrolledCost` stays close to
`RolledDynamicCost`, the boost stays near 1×, and such a loop fully unrolls only if it
already fits the *static* threshold (gate 2) — never via the dynamic boost. That is
exactly what §16 shows.

---

## 9. Partial Unroll

`shouldPartialUnroll` (`lib/Transforms/Scalar/LoopUnrollPass.cpp:833-880`) is
reached (rung 6) only with a **known** trip count that was too big to fully
unroll. It picks a `Count` that (a) fits under `UP.PartialThreshold` and
(b) divides the trip count, so no remainder loop is needed:

1. Return `nullopt` if there is no trip count; return `0` if `UP.Partial` is off
   (`:838-845`).
2. **Fit the budget.** Start from `UP.Count` (or the trip count). If unrolling by that
   count would push the body over `PartialThreshold`, shrink it to the largest count
   that fits. This is just the §6.2 size formula solved for `count`: the unrolled size is
   `(LoopSize − BEInsns) × count + BEInsns`, and requiring that `≤ PartialThreshold`
   gives

   ```
   count = (PartialThreshold − BEInsns) / (LoopSize − BEInsns)
   ```

   i.e. *(budget minus the one retained back-edge) ÷ (cost of one body copy)* — how many
   copies fit in the budget. (The code actually writes `max(PartialThreshold, BEInsns+1)`
   in the numerator — a guard so it can't go `≤ 0` if the budget were ever below the
   back-edge cost — keeping `count ≥ 1`.) (`:851-853`)
3. Clamp to `UP.MaxCount`, then **reduce to a divisor of the trip count**:
   `while (count != 0 && TripCount % count != 0) count--` (`:854-857`).
4. If that collapses to ≤1 and remainders are allowed, fall back to the largest
   **power-of-two** factor that fits (seeded from `DefaultUnrollRuntimeCount = 8`,
   halving until it fits) (`:858-867`).
5. `count < 2` becomes `0` (don't bother) (`:868-870`).

So partial unroll keeps one loop, replicated `Count` times, running `TripCount /
Count` times — and because `Count` divides the trip count exactly, there is no
leftover.

`UP.PartialThreshold` defaults to `150` (`:198`) and — importantly — **AMDGPU
does not raise it** (it only sets the *full*-unroll `Threshold` to 300, §14). So
partial unrolling on AMDGPU is governed by the generic 150 budget unless a pragma
or flag intervenes. This asymmetry is visible in §16: the outer loop's partial
factor is small because 150 is a tight budget once the inner loop has been fully
unrolled into it.

---

## 10. Runtime Unroll and the Remainder Loop

When the trip count is unknown at compile time, unrolling hits a problem worth
spelling out, because it is the whole reason this machinery exists.

Unrolling by `Count` chains `Count` body copies together with **one** exit test, at
the bottom (§6.2) — there are no checks on the induction variable *between* copies. So
the unrolled loop can only ever stop at a **multiple of `Count`**. If the compiler
doesn't know the trip count, it can't guarantee the count *is* a multiple of `Count` —
so, with nothing else to go on, the only safe choice would be `Count = 1` (no unroll).

The fix is a **remainder loop**, and the trick is that it manufactures the needed
divisibility **at runtime**. Say `Count = 4` and the real trip count turns out to be
`14` (unknown to the compiler):

- The emitted code first computes, at runtime, `rem = TripCount % Count` →
  `14 % 4 = 2`, and `main = TripCount − rem` → `12`. By construction `main` is now a
  multiple of `Count`.
- The **main** unrolled loop runs the `main = 12` part: `12 / 4 = 3` trips, each
  executing all 4 copies, with one bottom check per trip. No inter-copy checks are
  needed, because we now *know at runtime* there are exactly `12 = 3×4` iterations here.
- The **remainder loop** runs the leftover `rem = 2` iterations one at a time, each
  with an ordinary per-iteration check.

Total `12 + 2 = 14`. The compiler never knew the trip count; it emitted code that
*computes* the clean multiple at runtime and peels off the rest. That is what lets
`Count` be 4, 8, … for an unknown trip count.

The corollary (which §10.1 formalizes): if a remainder loop is **not** allowed and the
trip count's provable *multiple* (§2) isn't divisible by `Count`, there is no way to
avoid leftovers, so the unroller reduces `Count` until it divides the multiple — and
when the multiple is just 1, that bottoms out at `Count = 1`, i.e. no unroll.

### 10.1 How it is reached

Runtime unroll is rung 7 of `computeUnrollCount` (§7), reached only when
`TripCount == 0`. Crucially, **`UP.Runtime` is off by default** (`:206`) and the
generic passes never turn it on for an arbitrary loop — it must come from
`-unroll-runtime`, a pragma, an explicit count, or a *target*. On AMDGPU the only
thing that turns it on is a loop containing a qualifying **LDS (local-memory) access**
— §14.4 explains exactly what that means and why only LDS qualifies; a loop that only
touches global device memory leaves it off, so **runtime unrolling does not happen by
default for typical AMDGPU kernels** (confirmed empirically in §16.2).

The final gate in `tryToUnrollLoop` is
`UP.Runtime &= TripCount == 0 && TripMultiple % UP.Count != 0` (`:1315-1320`):
runtime unroll proceeds only if the trip count is unknown *and* the chosen
`Count` does not divide the provable trip multiple. If SCEV can prove
divisibility (e.g. the loop demonstrably runs a multiple-of-4 times and `Count`
is 4), the remainder is always empty and is skipped.

### 10.2 The transform: `UnrollRuntimeLoopRemainder`

`UnrollRuntimeLoopRemainder`
(`lib/Transforms/Utils/LoopUnrollRuntime.cpp:563`) builds the remainder loop. It
is called from `UnrollLoop` **before** the main body is cloned
(`lib/Transforms/Utils/LoopUnroll.cpp:412-428`).

- **Prologue vs epilogue.** The remainder loop (the leftover `TripCount % Count`
  iterations) can run *before* the unrolled main loop (a **prologue**) or *after* it (an
  **epilogue**). The choice is made two ways:
  - There is a command-line override, `-unroll-runtime-epilog`. If the user *explicitly
    passed it* (`getNumOccurrences() > 0` means "this flag appeared on the command line"),
    its value wins; otherwise the heuristic `isEpilogProfitable(L)` decides
    (`lib/Transforms/Utils/LoopUnroll.cpp:412-414`).
  - `isEpilogProfitable` prefers an **epilogue** when some loop-header PHI starts from a
    **constant** (`:201-210`). Why that matters: take an induction variable
    `i = phi [0, preheader], [i+1, latch]` — its *start* value (from the preheader) is the
    constant `0`. With an **epilogue**, the main loop runs *first*, so it keeps that
    constant start `0`. With a **prologue**, the remainder runs first and the main loop
    then starts from whatever value the prologue left behind — a *runtime* value, no
    longer a constant. A constant start is worth preserving (the compiler can reason about
    it), so when one exists, epilogue is preferred.
  - **Default** (no flag): **prologue, unless** that constant-start heuristic picks
    epilogue.
- **The remainder count.** `CreateTripRemainder` emits
  `ModVal = (BECount + 1) % Count`, named `xtraiter` in the IR. For a
  power-of-two `Count` this is a single mask `TripCount & (Count − 1)`; otherwise
  it is computed overflow-safely with two `urem`s and an add
  (`lib/Transforms/Utils/LoopUnrollRuntime.cpp:498-522`). You can see this exact
  IR in §16.2.
- **The around-branch.** In epilogue mode the code tests `BECount <u (Count−1)`
  to decide whether to skip the main loop; in prologue mode it tests
  `ModVal != 0` (`lcmp.mod`) (`lib/Transforms/Utils/LoopUnrollRuntime.cpp:770-778`).
- **`Count == 2` special case.** When you unroll by 2, the leftover is `TripCount % 2`,
  which is always `0` or `1` — so the remainder handles **at most one** iteration. A
  block that runs at most once isn't really a loop (it needs no back-edge to iterate
  again), so the code doesn't build a remainder *loop* at all: it **breaks the back-edge**
  (`breakLoopBackedge`, removing the loop-closing edge) and **merges** the leftover block
  into the surrounding control flow, leaving just a single *conditionally-run* copy of the
  body instead of a loop (`lib/Transforms/Utils/LoopUnrollRuntime.cpp:945-978`).

### 10.3 When runtime unroll is refused

`UnrollRuntimeLoopRemainder` bails (returns false) on, among others:

- not in loop-simplify form (`:574-577`);
- latch not terminated by a conditional branch (`:583-591`);
- neither latch successor exits the loop (`:596-603`);
- **multiple exits** — a loop with more than one exiting block (§5.2 step 8), e.g. the
  latch test *plus* an early `break`. Runtime unrolling refuses these by default: the
  remainder/prologue scaffolding and the trip-count math assume a single clean exit (the
  latch), and extra exits complicate it. It is allowed only when the pass is maintaining
  LCSSA form (`PreserveLCSSA`, so the SSA fix-ups across the extra exits stay tractable)
  **and** `canProfitablyUnrollMultiExitLoop` approves (`:606-624`, `:420-465`), which
  happens in just two cases:
  - the user explicitly opted in with `-unroll-runtime-multi-exit`; or
  - there is exactly **one** extra (non-latch) exit and it is *predictable* — either a
    cold **deoptimize** exit (a postdominating `@llvm.experimental.deoptimize` call: a
    rarely-taken bail-to-slow-path used by managed runtimes, so the loop almost always
    exits via the latch anyway), or the user asserted it with
    `-unroll-runtime-other-exit-predictable`.

  The "predictable" requirement is the whole point: runtime-unrolling a multi-exit loop
  only pays off if the loop nearly always runs to its latch exit, so the extra exit can
  be treated as a rare side-channel rather than something that keeps aborting the
  unrolled body partway through;
- no computable backedge SCEV (`:627-650`);
- **expensive trip-count expansion.** To runtime-unroll, the unroller must emit IR that
  computes the loop's trip count *at runtime* — it's needed for the `TripCount % Count`
  remainder math (§10.2). That code is materialized from the trip-count SCEV by the SCEV
  expander, and some trip counts are cheap (`n`, `n − 1`) while others are costly. The
  guard `isHighCostExpansion` estimates that materialization cost against
  `SCEVCheapExpansionBudget` (default `4`) — a budget measured in **`TargetTransformInfo`
  instruction-cost units** (it sums each SCEV operation's TTI cost: an add or cast is
  ~1, an integer **division** is rated very high). So `4` is roughly "four cheap
  instructions," and a trip count needing a `udiv` blows past it on its own. If the
  expansion is over budget **and** the unroller isn't allowed to pay for it
  (`AllowExpensiveTripCount = false`), it refuses (`:656-661`) — the setup overhead would
  likely cancel the unrolling benefit. (`AllowExpensiveTripCount` is turned *on* for
  forced/pragma unrolls and when profile data justifies it, §7.)
- **overflow risk** — `Log2(Count) > BEWidth` (`:665-670`). `BEWidth` is the bit width of
  the backedge-count type (e.g. `32` for an `i32` count), and `Log2(Count)` is roughly how
  many bits `Count` needs. The remainder math (`(BECount + 1) % Count`, §10.2) is done in
  that bit width and deliberately lets `BECount + 1` overflow; for the modulo to stay
  correct, `Count` has to *fit* — i.e. `Log2(Count) ≤ BEWidth`. If `Count` is so large it
  needs more bits than the type has, the arithmetic can't be done safely, so it bails. In
  practice this never fires (real unroll counts are 2/4/8, widths are 32/64) — it's a
  defensive guard against a pathologically large forced count.

**When remainder generation fails.** A runtime unroll *needs* a remainder loop, so if
`UnrollRuntimeLoopRemainder` bailed for any reason above, what happens next depends on
whether the unroll was *forced* (`UP.Force`, set by a `#pragma`, §7):

- **Forced** → don't give up. `UnrollLoop` clears the `Runtime` flag and falls back to a
  plain **partial unroll**: replicate the body `Count` times but *keep each copy's exit
  test* (no remainder loop). That's still correct for an unknown trip count — every copy
  checks whether to leave — just less optimized than the remainder form (you keep all the
  per-iteration tests; you only save back-edge branches).
- **Not forced** (an ordinary cost-model unroll) → give up entirely and return
  `Unmodified` (no unroll). A degraded unroll isn't worth it when nobody demanded one.

(`lib/Transforms/Utils/LoopUnroll.cpp:416-428`)

---

## 11. `UnrollLoop`: The Mechanical Transform

`UnrollLoop` (`lib/Transforms/Utils/LoopUnroll.cpp:269`) performs the actual
cloning. It takes the loop, a by-value `UnrollLoopOptions` (six fields: `Count`,
`Force`, `Runtime`, `AllowExpensiveTripCount`, `UnrollRemainder`, `ForgetAllSCEV`
— `include/llvm/Transforms/Utils/UnrollLoop.h:68-75`), and an out-param for any
remainder loop produced.

### 11.1 Validation and the full-vs-partial decision

It returns `Unmodified` immediately unless the loop passes four structural checks
(`lib/Transforms/Utils/LoopUnroll.cpp:277-298`). The first two are exactly the
**loop-simplify-form** guarantees (§2):

- a **preheader** — one block that falls into the header, giving unrolling a single,
  well-defined place to put any "before the loop" setup;
- a **single latch** — one block holding the back-edge, which the clone/rewire logic
  relies on (it chains copy *i*'s latch to copy *i+1*'s header; multiple back-edges aren't
  supported).

By the path this guide follows these already hold — `tryToUnrollLoop` required
loop-simplify form back at §5.2 step 3, which *guarantees* a preheader and a single latch.
`UnrollLoop` re-checks anyway because it is a **reusable utility** called from more than
one place (e.g. it also unrolls the runtime *remainder* loop, §10.2), so it can't assume
every caller pre-simplified; on the normal path these two always pass.

The other two checks are about **duplicability** — unrolling copies the loop body, and
some code can't be copied correctly:

- **safe to clone** (`isSafeToClone`) rules out a loop containing an `indirectbr` — an
  *indirect branch*, i.e. a "computed goto" that jumps to a block address chosen at
  runtime (used e.g. by threaded interpreters). Such code can't be cloned correctly: its
  jump targets *are* block addresses, so copying the blocks scrambles which address means
  which copy.
- **header's address not taken** (`hasAddressTaken`) — a block's address can be captured
  as a value (the `blockaddress` constant, or GNU `&&label`, again for computed gotos). If
  some code holds a pointer to the loop header, unrolling — which makes several copies of
  that header — would leave that pointer ambiguous, so the loop can't be unrolled.

It then clamps `Count` down to the max trip count (`:317-320`) and makes the central
decision:

```
CompletelyUnroll = (Count == MaxTripCount)        // LoopUnroll.cpp:361
```

If `CompletelyUnroll`, runtime is forced off (`:365-368`). Note there is no
separate "trip count" variable here — the (clamped) `Count` compared against
SCEV's max trip count *is* the full-unroll test.

### 11.2 Cloning and remapping

This is the core of *every* unroll — full, partial, and runtime all share the same
cloning. (One mode-specific aside: in **runtime** mode the remainder loop has already
been built first by `UnrollRuntimeLoopRemainder`, §10.2; full and partial unroll skip
straight to the cloning below.)

The main clone loop replicates the body:

```
for (unsigned It = 1; It != Count; ++It)           // LoopUnroll.cpp:539
```

i.e. `Count − 1` additional copies beyond the original (iteration 0). Each block
is cloned with a `.It` name suffix and registered with `LoopInfo` and the
dominator tree (`:538-617`). The crucial SSA trick: when the header is cloned,
each cloned header PHI is **replaced by the value that flowed in along the
back-edge** — for iteration 1 directly, for later iterations via `LastValueMap`
from the previous copy — and the cloned PHI is erased (`:556-567`). That is how
iteration *i* consumes iteration *i−1*'s results, turning the induction PHIs into
straight-line dataflow.

`LastValueMap` is the rolling cross-iteration map; after each copy it is used to
remap operands and to extend exit-block PHIs with an incoming value for the new
copy (`:569-587`).

### 11.3 Back-edge rewiring and loop deletion

After cloning, every copy is still a *self-contained* loop — each copy's latch (the block
that held the back-edge) branches back to **its own** header. We need the copies to run in
**sequence** instead: copy 0, then copy 1, …, then copy Count−1, then around again. So each
copy's latch is redirected from its own header to the **next** copy's header (all line
numbers below are in `lib/Transforms/Utils/LoopUnroll.cpp`):

```
for (i = 0; i != e; ++i) {                          // LoopUnroll.cpp:654-659
  j = (i + 1) % e;                                  // e = number of copies
  Latches[i]->getTerminator()->replaceSuccessorWith(Headers[i], Headers[j]);
}
```

Key point: most of these edges go **forward** — `latch[0] → header[1]`, `latch[1] →
header[2]`, … are "fall into the next copy," **not** back-edges. *Only* the wrap-around
`latch[Count−1] → header[0]` is a real back-edge. So there is exactly **one** back-edge,
not one between every pair of copies — which is the intuition from §6.2.

There's still a loose end: each copy's latch was cloned *with its conditional exit test*
("exit the loop, or keep going?"), so right now the copies are joined by *conditional*
branches, not clean fall-throughs. **Branch folding** resolves that — two helper lambdas
inside `UnrollLoop`: `WillExit` (`LoopUnroll.cpp:704-735`) asks, per copy, whether its exit
is statically decided, and `SetDest` (`:687-702`) rewrites a decided conditional branch
into an unconditional one.

- **Partial / runtime.** These run a whole multiple of `Count` iterations (the remainder
  loop, §10, peels off any leftover), so copies 0…Count−2 *provably never exit* — their
  tests fold to **unconditional fall-throughs**. Only the wrap keeps a real exit test.
  That is exactly the §6.2 shape: `Count` bodies falling through one into the next, with a
  single back-edge test at the bottom.
- **Full.** The exact trip count is known, so *every* test folds — including the wrap, so
  even the one back-edge disappears. The loop object is then erased:
  `if (CompletelyUnroll) { LI->erase(L); }` (`:836-839`). (A non-exiting, unconditional
  latch in a complete unroll has its terminator turned into `unreachable`, `:789-796`.)

In both cases the fall-throughs leave a chain of blocks joined by **unconditional**
branches, and a merging pass then collapses them: each unconditional latch is a candidate
for `MergeBlockIntoPredecessor` (`:798-817`), which folds a single-predecessor block into
the predecessor that branches to it — so consecutive copies are stitched into actual
straighter-line code rather than a string of one-branch blocks. (§11.4's
`simplifyLoopAfterUnroll` then constant-folds and DCEs whatever is left.)

### 11.4 Cleanup and return

Three bits of cleanup, then the result (line numbers in
`lib/Transforms/Utils/LoopUnroll.cpp`):

- **Simplify the unrolled code** (`simplifyLoopAfterUnroll`, `:826-829`). First — *only*
  for a genuine partial unroll (`!CompletelyUnroll && Count > 1`, i.e. a loop still
  exists) — it runs **induction-variable simplification**. An *induction variable* (IV)
  is a value that advances by a regular step each iteration — the loop counter `i`, a
  pointer walking an array, etc. (exactly what the add-recurrences of §3 are the closed
  form of). Unrolling replicates these into several near-duplicate copies (`i`, `i+1`,
  `i+2`, …), so this step tidies them back into clean canonical form. (A full unroll has
  no loop left, so there's nothing to simplify.) Then, in all cases, it **constant-folds
  and dead-code-eliminates** the unrolled blocks — this is where the simplifications
  unrolling exposed (constant indices, folded loads, dead branches) are actually cashed
  in.
- **Fix the trip-count estimate** (partial only, `:840-844`). A loop can carry profiling
  metadata estimating how many times it runs; after unrolling by `Count` it runs `Count`×
  *fewer* times (each trip now does `Count` iterations), so that estimate is divided by
  `Count` to stay accurate. (A fully-unrolled loop is gone, so there's nothing to update.)
- **Restore canonical forms** (`:847-892`). Unrolling can break LCSSA and loop-simplify
  form (§2) — e.g. deleting a loop can disturb an *enclosing* loop's LCSSA — so they are
  reformed where needed, so later passes still get the shapes they expect.

Finally, the return value (`:894-895`): **`FullyUnrolled`** if it was a complete unroll
(the loop was deleted), otherwise **`PartiallyUnrolled`** (a loop remains). (`Unmodified`
is only returned by the early bailouts of §11.1, never here.)

---

## 12. Trip Counts via SCEV

With the SCEV background from §3 in hand, the unroller's use of it is small and
concrete: it never touches raw `SCEV` objects, but calls three convenience
wrappers over the backedge-count machinery of §3.4. The distinction between the
three queries is exactly the distinction between the three unroll modes.

| Query | Returns | Used for |
|-------|---------|----------|
| `getSmallConstantTripCount(L, ExitingBlock)` | exact constant trip count, or `0` if unknown | full / partial unroll |
| `getSmallConstantMaxTripCount(L)` | constant **upper bound**, or `0` | bounded full unroll (rung 4) |
| `getSmallConstantTripMultiple(L, ExitingBlock)` | largest constant the trip count is **divisible by** (≥1) | deciding whether a remainder is needed |

- **Exact** (`lib/Analysis/ScalarEvolution.cpp:8096-8105`): the backedge-taken
  count must be a `SCEVConstant`; the trip count is that plus one. Anything
  needing more than 32 bits is reported as `0` = unknown
  (`getConstantTripCount`, `:8077-8089`).
- **Max** (`:8107-8111`): reads the **`ConstantMaximum`** backedge-taken count of
  §3.4 — a *constant ceiling* SCEV can prove on how many times the back-edge fires
  — rather than the **`Exact`** count the row above uses. That difference is what
  lets this query succeed where `getSmallConstantTripCount` returns `0`: for a loop
  like `for (i = 0; i < n; ++i) { if (i >= 8) break; }` the exact count is symbolic
  (it depends on the runtime `n`), but SCEV can still prove the back-edge fires at
  most 7 times, so the max trip count is `8`. That proven ceiling is exactly what
  the bounded-full-unroll rung (§7, rung 4) unrolls to — emit `8` copies and
  branch-guard the surplus — for loops whose exact count is unknown but bounded.
- **Multiple** (`:8247-8283`): returns the largest constant divisor of the trip
  count, or `1` if unknown. This is the value that lets the runtime path skip the
  remainder (§10.1): if `TripMultiple % Count == 0`, the unrolled loop always
  exits cleanly.

In `tryToUnrollLoop`, the trip count is discovered in two phases: take the
**smallest exact** per-exit trip count across all exiting blocks (which
guarantees at least one exit's branches can be eliminated); if none is constant,
take the trip *multiple* of the latch or single exiting block; and only if there
is no exact count, fetch the max trip count (`:1227-1274`).

This is precisely the difference between the §16 const kernel (exact trip count
known → full/partial) and the runtime kernel (trip count is a kernel argument →
multiple is `1`, only runtime unroll is possible).

---

## 13. Pragmas and Loop Metadata

User unroll directives are carried as `!llvm.loop` metadata and read by small
helpers in `lib/Transforms/Scalar/LoopUnrollPass.cpp:692-730`:

| Metadata string | Reader | Meaning |
|-----------------|--------|---------|
| `llvm.loop.unroll.full` | `hasUnrollFullPragma` | force full unroll (`#pragma clang loop unroll(full)`) |
| `llvm.loop.unroll.enable` | `hasUnrollEnablePragma` | enable unrolling (bare `#pragma unroll`, `#pragma clang loop unroll(enable)`) |
| `llvm.loop.unroll.count` | `unrollCountPragmaValue` | unroll by exactly N (`#pragma unroll N`); a value of **1** means *do not unroll* |
| `llvm.loop.unroll.runtime.disable` | `hasRuntimeUnrollDisablePragma` | forbid the runtime/remainder path |
| `llvm.loop.unroll.disable` | (via `hasUnrollTransformation`) | disable unrolling entirely |

A subtlety worth calling out: a **bare `#pragma unroll`** (no count) lowers to
`llvm.loop.unroll.**enable**`, *not* `.full` — only `#pragma clang loop
unroll(full)` produces `.full`. The distinction matters because the two reach a
full unroll by different routes. `.full` is *forced*: `shouldPragmaUnroll` returns
the trip count directly (rung 1–2, `:796-797`), bypassing the cost model. `.enable`
is *not* forced — `shouldPragmaUnroll` ignores it (`:775-801` handles only count
and full) and returns `nullopt`. What makes a bare `#pragma unroll` still fully
unroll a constant-trip loop is the threshold boost below: being explicit raises the
size budget to `16384`, after which the ordinary exact-full-unroll rung (rung 3,
`shouldFullUnroll`) succeeds on its own. So the *outcome* is usually identical, but
`.enable` goes through the cost model with a huge budget while `.full` skips it.

Clang records each pragma as a tag attached to the loop. The unroller reads those
tags two ways:

- The **specific readers** in the table above each look for one kind of request —
  is there a `full` tag? a `count N` tag? — and are what the cost-model ladder (§7)
  consults when it needs the exact directive.
- A **summarizer**, `hasUnrollTransformation`
  (`lib/Transforms/Utils/LoopUtils.cpp:352-371`), ignores the specifics and reduces
  everything to one verdict: the user *forced* unrolling (`.enable`, `.full`, or
  `.count > 1`), the user *forbade* it (`.disable`, or `.count == 1`), or said
  nothing. The pass's early go/no-go checks (§5.2, steps 1 and 4) only need that
  verdict — they don't care which pragma produced it.

By default the unroller refuses to unroll a loop when the unrolled body would be
too big — it has a size budget (about 300 instructions on AMDGPU, §6). Any
`#pragma` makes that budget stop blocking the loop, in one of two ways:

- If the pragma names an **exact amount** — `unroll(full)` or `#pragma unroll N` —
  the unroller just does that amount and never checks the size at all.
- If the pragma only says "unroll this" with **no number** — bare `#pragma unroll`
  — there is no exact amount to obey, so the size check can't be skipped. Instead
  the unroller raises the budget to a fixed ceiling of **16384** instructions
  (16 × 1024) before running the check, which it then passes for any normal loop.
  That ceiling, `PragmaUnrollThreshold`, is the default of the hidden
  `-pragma-unroll-threshold` command-line option
  (`lib/Transforms/Scalar/LoopUnrollPass.cpp:142-145`) — a fixed constant, not
  computed per-loop.

**AMDGPU does not override that 16384** — it is private to the unroll pass, not one
of the `UnrollingPreferences` fields the target fills in (§14), so on AMDGPU it is
the same value as everywhere else. The raise actually takes the *larger* of the
ceiling and the loop's current budget (`std::max(UP.Threshold, 16384)`, `:939-941`),
so if AMDGPU's own per-loop boosts (§14.2) already pushed the budget above 16384 the
loop keeps that higher number. 16384 is the floor the pragma guarantees, not a cap.

Either way the size budget no longer stands in the way — which is why `#pragma
unroll` works even on bodies far larger than 300.

---

## 14. AMDGPU Specialization

AMDGPU customizes unrolling **only** through the TTI hook
`getUnrollingPreferences`. There is one implementation, `AMDGPUTTIImpl`, that both
`GCNTTIImpl` and `R600TTIImpl` forward to via a `CommonTTI` member
(`lib/Target/AMDGPU/AMDGPUTargetTransformInfo.cpp:1260-1264`). It lives at
`lib/Target/AMDGPU/AMDGPUTargetTransformInfo.cpp:103-263`.

### 14.1 Unconditional preferences

At the top of the function (`:106-117`):

| Field | Value | Citation | Note |
|-------|-------|----------|------|
| `UP.Threshold` | `300` | `:107-108` | from function attribute `amdgpu-unroll-threshold`, default 300 — overwrites the generic O3 default (also 300) |
| `UP.MaxCount` | `UINT_MAX` | `:109` | no target cap on partial/runtime count |
| `UP.Partial` | `true` | `:110` | partial unrolling enabled |
| `UP.BEInsns` | `+= 3` (→ 5) | `:114` | a conditional back-edge needs ~3 extra exec-mask manipulations on GPU |
| `UP.UnrollVectorizedLoop` | `true` | `:117` | keep unrolling already-vectorized loops |

Note what is **not** set: `UP.PartialThreshold` is left at the generic default
(150) unless the `amdgpu.loop.unroll.threshold` metadata path runs (§14.3), and
`UP.Runtime` is **not** turned on here — the comment at `:119` literally says
`// TODO: Do we want runtime unrolling?`. Runtime unroll is enabled only in the
LDS path (§14.4).

### 14.2 The per-loop threshold boosts

**The idea first.** The base budget is 300 instructions (§14.1). But on a GPU,
certain memory-access patterns become *dramatically cheaper if — and only if — the
loop is unrolled*, because unrolling is what exposes the optimization. So AMDGPU
scans the loop body, and when it spots one of those patterns it grants a much larger
size budget, buying the unroll that unlocks the win. A loop that addresses only
ordinary global memory unlocks nothing this way, so it never gets a boost and stays
at 300. The rest of this section is just the catalogue of patterns and *why
unrolling helps each one*.

**One piece of vocabulary used throughout.** A **GEP** (`getelementptr`) is the LLVM
instruction that computes an address — a base pointer plus index arithmetic (e.g.
`&array[i]`). It does no memory access itself; it's the address calculation that
feeds a load or store. Every GEP carries an **address space** number saying *which
GPU memory* it targets, and that number is what the scan keys off:

| Address space | Name | What it is |
|---|---|---|
| 5 | `PRIVATE_ADDRESS` | per-thread **scratch** (a thread's private stack/arrays — `alloca`s) |
| 3 | `LOCAL_ADDRESS` | **LDS**, on-chip memory shared within a workgroup |
| 2 | `REGION_ADDRESS` | **GDS**, a smaller global-on-chip variant of LDS |
| 1 | `GLOBAL_ADDRESS` | device DRAM — the kernel's input/output buffers (§8) |

The scan walks only the blocks that belong **directly to this loop**, skipping any
block owned by a nested inner loop (`:150-152`) — an inner loop gets its own
`getUnrollingPreferences` call, so boosting it here would double-count. Within those
blocks it looks for three things:

- **The `if`-bonus** (`:160-177`). A conditional branch *inside* the loop whose
  condition is computed from a value that changes every iteration earns
  `amdgpu-unroll-threshold-if` (default **200**) added to the budget. "Changes every
  iteration" is detected as the condition depending on a **PHI** belonging to this
  loop — a PHI is the SSA node that picks a value based on which edge entered the
  block, which is how a per-iteration value (the loop counter, a running flag) is
  represented (`dependsOnLocalPhi`, `:78-95`). Why it's worth paying for: on a GPU
  the threads of a wavefront run in lockstep, so a branch where different threads
  take different sides is **divergent** — the hardware executes *both* sides under a
  mask, and the PHI that merges them costs registers. If unrolling turns the
  per-iteration condition into a per-copy constant, the compiler can fold the branch
  away and delete the PHI, removing both costs. Loop-*exit* branches are excluded
  (`:164-166`) — unrolling doesn't remove those.
- **A private/scratch GEP** (`:179-204`). An address calculation into AS 5 (a
  per-thread `alloca`'d array) raises the budget toward
  `amdgpu-unroll-threshold-private` (default **2700**) — but only if it indexes a
  *static* `alloca` no bigger than `MaxAlloca = (256 − 16) × 4 = 960` bytes (small
  enough to plausibly live in registers, reserving 16 of the 256 regs). Why:
  scratch memory is slow off-chip storage, and the compiler would much rather hold
  that little array in registers. **SROA** (Scalar Replacement of Aggregates, §5)
  is the pass that does that promotion — but it can only replace an array element
  with a register when the *index is a compile-time constant*. In a rolled loop the
  index is the variable `i`; unrolling replaces it with constants `0, 1, 2, …`,
  which is exactly what lets SROA promote the array afterward. So the boost pays for
  the unroll that makes register promotion possible.
- **An LDS/GDS GEP** (`:205-219`). An address calculation into AS 3 or AS 2 raises
  the budget toward `amdgpu-unroll-threshold-local` (default **1000**). Why:
  unrolling brings several LDS accesses at *different fixed offsets* into one
  straight-line body, and the backend can fuse adjacent ones into wider
  `ds_read`/`ds_write` instructions (the machine ops that touch LDS) — for example
  four `ds_read_b32` at offsets 0/4/8/12 collapsing into a single `ds_read_b128`.

  Three gates keep this from firing where the combine won't actually pay off — any
  one of them denies the boost for that GEP (`:212-215`):

  - **One good LDS access is enough — it does not have to be the only one.** A block
    can contain several LDS GEPs; the boost fires as soon as one of them qualifies, so
    the answer is "at least one," not "exactly one." The `LocalGEPsSeen` counter
    (`:207-215`) is only a safety catch in the other direction: if the block contains
    an LDS access that *can't* be combined — such as one with a non-variable base (the
    third gate below) — the heuristic reads the block as having messy LDS addressing
    and gives up on the boost there. So: one combinable access turns the boost on; an
    un-combinable one seen first turns it off for that block.
  - **Loop nesting depth ≤ 2.** `getLoopDepth()` is the nesting level — `1` for an
    outermost loop, `2` for one level in, and so on; the gate rejects depth 3 or
    deeper (`getLoopDepth() > 2`). Two things make this less surprising than it first
    looks. First, it withholds only the *LDS budget boost* (and the runtime-unroll
    enable) from a deep inner loop — it does **not** stop that loop from unrolling
    under the base 300 budget. Second, the rationale is tied to processing order: the
    unroller visits loops **innermost-first** (postorder, `appendLoopsToWorklist`), so
    an inner loop is unrolled *before* its enclosing loops are even examined, and its
    unrolled body becomes part of their measured size. If a deeply nested inner loop
    grabbed the full 1000-instruction LDS budget and unrolled aggressively, it could
    bloat every loop above it past that loop's own threshold — silently blocking their
    unrolling, since they are judged later. Capping the LDS boost at depth ≤ 2 keeps a
    deep inner loop from spending the size budget the outer levels need to stay
    unrollable (the source comment: "*give a chance to unroll an outer loop for a more
    important reason*"). The combine benefit at the innermost level is still captured
    whenever that loop sits at depth 1 or 2.
  - **Base must be a global variable or a kernel argument.** Combining only works if
    the backend can see that the several accesses share one *recognizable* base and
    differ only by a constant offset. A `__shared__` LDS array shows up in IR as a
    module-scope **global variable** (in address space 3), and a pointer handed to
    the kernel is an **argument** — each is a single stable symbol, so `base+0`,
    `base+4`, … are plainly the same array indexed at different offsets. If the base
    is instead a computed pointer (a load, a PHI, pointer arithmetic), the compiler
    can't prove the accesses share a base, combining is unlikely, and the boost
    would be wasted (`!isa<GlobalVariable> && !isa<Argument>` denies it).

  This branch is *also* the one place `UP.Runtime` is turned on — see §14.4.

A GEP earns its boost only if its address **actually varies across iterations**:
at least one of its index operands must be defined *inside* this loop
(`HasLoopDef`, `:221-235`). If the address is loop-invariant, every unrolled copy
touches the same location, so unrolling exposes nothing to combine or promote and
no boost is given.

The boost is monotonic — it only ever *raises* the budget, never lowers it — and is
capped at `MaxBoost = max(2700, 1000) = 2700`. The moment the budget reaches
`MaxBoost` the scan returns immediately (`:172-173`, `:254-255`): it's already as
high as any pattern can push it, so there is nothing left to look for.

(The same scan also bumps `UP.MaxIterationsCountToAnalyze` to **32** for small
innermost loops — that is the "AMDGPU raises to 32" referenced from §8.2, covered
in detail in §14.5.)

**This is why an ordinary global-memory kernel sees none of this.** A GEP into AS 1
matches none of the three patterns (`else continue` at `:189-190`), so the budget
stays at the base 300. The §16 worked example is exactly this situation.

### 14.3 Threshold via loop metadata

The metadata `amdgpu.loop.unroll.threshold` (a 2-operand `!llvm.loop` entry) can
set `UP.Threshold` and — uniquely in this function — also set
`UP.PartialThreshold` to the same value (`:126-143`). It can only *lower* the
private/local caps, never raise them above their cl::opt defaults.

### 14.4 The one runtime-enabling path

The LDS-GEP branch is the only place AMDGPU sets `UP.Runtime`:
`UP.Runtime = UnrollRuntimeLocal` (`amdgpu-unroll-runtime-local`, default **true**) at
`:218`. So `UP.Runtime` becomes `true` *only* for a loop containing a qualifying LDS
address calculation, and stays at its generic `false` otherwise.

**What "LDS-addressing loop" means here.** It is *not* about the loop being purely LDS —
a loop can mix global, LDS, private, and scalar work. AMDGPU's hook just scans the loop's
instructions for **GEPs** (address computations, §14.2) and asks: is there at least one
GEP into **local memory** (address space 3) that varies with the loop counter and passes
the §14.2 gating (≤ 1 local GEP per block, loop depth ≤ 2, a global-variable or argument
base)? If yes, it flips `UP.Runtime` on for the *whole* loop. A loop with no such access
— e.g. one that only reads/writes **global device memory** (address space 1) — never
trips it.

**Why LDS but not global.** The trigger exists for a specific codegen payoff: LDS
instructions (`ds_read`/`ds_write`) carry an immediate byte offset, and the backend can
**combine** several LDS accesses at different static offsets into fewer, wider ops (e.g.
`ds_read2_b32`). Unrolling turns "the same LDS access across N iterations" into N accesses
at *statically-known relative offsets* in one body — exactly what the combiner wants, and
the AMDGPU source comment justifies the LDS boost in just these terms ("*let ds
instructions with different offsets combine*").

It would be wrong, though, to say global memory gets no such benefit — it does. Adjacent
global accesses are vectorized by the middle-end **`LoadStoreVectorizer`** (four
`global_load_dword` → one `global_load_dwordx4`), merged again at the MIR level by
AMDGPU's **`SILoadStoreOptimizer`**, and kept together by the **MachineScheduler's memory
clustering**, which adds artificial *cluster edges* between neighboring memory ops (gated
by `shouldClusterMemOps`) so the scheduler places them adjacently for the merger and
`SIInsertHardClauses` to act on. Unrolling a global loop exposes constant-offset accesses
for exactly these passes, just as it does for LDS.

So the real asymmetry is narrower than "global can't combine," and rests on two honest
points. **First**, the AMDGPU code only *positively* opts into **runtime** unrolling for
LDS and leaves global an explicit open question — the function literally carries
`// TODO: Do we want runtime unrolling?` (`:119`); it never asserts global is hopeless.
**Second**, the likely reason for that conservatism (the code does not state it) is cost:
global memory is long-latency DRAM, so runtime-unrolling a global loop multiplies the
in-flight long-latency loads and the live values they produce — raising register pressure
and risking lower occupancy — whereas LDS is on-chip and low-latency, so its combine is a
cleaner win. And global's adjacency combining does not *require* runtime unrolling anyway:
a known-trip-count global loop still unrolls (full/partial) and feeds the same
vectorizer/merger. So `UP.Runtime` is set only on the LDS path; for everything else it
stays `false`, and §10.1's gate keeps runtime unroll off.

### 14.5 Iteration-analysis cap

This one is a compile-time knob for the dynamic cost model of §8.2, not a threshold
boost.

Recall (§8.2) that `analyzeLoopUnrollCost` is the *folding-aware* profitability model:
it symbolically simulates every iteration of an innermost, constant-trip loop to see
whether the unrolled body folds down below its raw size. Because it walks every
instruction of every iteration, its cost is roughly *trip count × instructions per
iteration*. To keep that bounded, it only runs when the trip count is small enough:
`TripCount <= UP.MaxIterationsCountToAnalyze`, whose generic default is **10**
(`:364-365`). A loop with, say, 20 iterations normally skips the analysis entirely and
is judged on raw size alone.

`amdgpu-unroll-max-block-to-analyze` is a hidden command-line option (default **32**,
`:55-58`) holding a *block-size* threshold — a count of the instructions in a basic
block. After scanning each block of the loop, AMDGPU raises the iteration cap to 32 when
two conditions hold (`:258-261`):

- the loop is **innermost** (`L->isInnermost()`), and
- the block is **small** — fewer than `amdgpu-unroll-max-block-to-analyze` (32)
  instructions (`BB->size() < UnrollMaxBlockToAnalyze`).

The two thresholds work as a pair. Since the simulation cost is ~ *iterations ×
block size*, AMDGPU is willing to simulate up to **32 iterations**, but only when each
iteration is **cheap** to simulate (a block under 32 instructions). For a large loop
body it leaves the cap at the generic 10, so it never spends much compile time
simulating a big block many times over.

Why it matters: the §8.2 model is what lets a loop that *exceeds* the static size budget
still get fully unrolled when it folds heavily afterward — e.g. an innermost loop
indexing a constant array, whose GEP index becomes a per-copy constant and folds the load
away (the source comment: "*got a GEP in a small BB … increase max trip count to analyze
for better estimation cost*"). Small AMDGPU inner loops with 11–32 iterations are common
enough that lifting the cap to 32 meaningfully widens which of them the folding model can
vouch for. (One honest caveat: the literal condition checks only *innermost* + *small
block* — despite the comment, it does not actually require that a GEP was seen.)

### 14.6 Address-space numbers

`PRIVATE_ADDRESS = 5`, `LOCAL_ADDRESS = 3`, `REGION_ADDRESS = 2` come from the
`AMDGPUAS` enum (`lib/Target/AMDGPU/AMDGPU.h:387-398`). Global is AS 1 (no boost).

### 14.7 Peeling and the target machine

Two loose ends, both of which amount to "AMDGPU does nothing special here" — but it is
worth being explicit about what that means.

**Peeling is entirely generic.** *Peeling* pulls a few leading iterations out of a loop
and emits them as straight-line code before the loop runs the rest (§2; it is rung 5 of
the §7 ladder). AMDGPU's `getPeelingPreferences` (`:265-268`) does nothing but forward to
the base, `BasicTTIImpl::getPeelingPreferences` (`include/llvm/CodeGen/BasicTTIImpl.h:616-622`),
which sets the standard defaults:

| Field | Value | Meaning |
|-------|-------|---------|
| `PP.PeelCount` | `0` | no forced peel count — let the cost model decide |
| `PP.AllowPeeling` | `true` | peeling is permitted |
| `PP.AllowLoopNestsPeeling` | `false` | only innermost / non-nested loops may be peeled |
| `PP.PeelProfiledIterations` | `true` | profile data may drive a peel count |

So AMDGPU contributes **no** peeling heuristic of its own; the actual peel count is then
computed downstream by the generic `computePeelCount` / `gatherPeelingPreferences`
(`lib/Transforms/Utils/LoopPeel.cpp`), exactly as for any other target. (Contrast
§14.1–14.5, where unrolling is heavily customized — peeling is the part AMDGPU leaves
alone.) The override exists only to satisfy the TTI interface.

**The target machine adds no unroll (or peel) pass.** The string "unroll" appears nowhere
in `AMDGPUTargetMachine.cpp` except a single comment (`:767-768`). AMDGPU injects no
unrolling pass into its codegen pipeline — which is the structural punchline of this whole
section: AMDGPU's *entire* influence on unrolling is the two TTI hooks
(`getUnrollingPreferences` and `getPeelingPreferences`). The unroll machinery itself —
`LoopFullUnrollPass` and `LoopUnrollPass` from §4 — is the generic, target-agnostic
middle-end, added by the common `PassBuilder` pipeline. Everything in §4–§13 therefore
applies to AMDGPU verbatim; the only AMDGPU-specific deltas are the preference values
catalogued in §14.1–14.6.

The one place the target pipeline *interacts* with unrolling is sequencing, not addition:
AMDGPU schedules `AMDGPUPromoteAllocaToVectorPass` to run **before** SROA and the generic
unroll, on the stated reasoning that eliminating allocas first means it "*may choose to
unroll less*" (`:766-771`). So the target shapes the pre-unroll IR, but the unroll decision
and transform themselves remain the generic passes this guide describes.

---

## 15. Reference: `UnrollingPreferences` Fields, Defaults, and Flags

The struct is at `include/llvm/Analysis/TargetTransformInfo.h:503-585`. (This
fork lacks the upstream `MaxUpperBound` and `SCEVExpansionBudget` fields.) Every
field except `UnrollVectorizedLoop` is seeded by `gatherUnrollingPreferences`
(`lib/Transforms/Scalar/LoopUnrollPass.cpp:183-272`).

### 15.1 Fields, generic defaults, and overriding flags

| Field | Generic default | cl::opt override | Meaning |
|-------|-----------------|------------------|---------|
| `Threshold` | `300` at O3 (`UnrollThresholdAggressive`), else `150` (`UnrollThresholdDefault`) | `-unroll-threshold` | full-unroll size budget |
| `MaxPercentThresholdBoost` | `400` | `-unroll-max-percent-threshold-boost` | max threshold boost (4×) from dynamic savings |
| `OptSizeThreshold` | `0` | `-unroll-optsize-threshold` | full-unroll budget under `-Os` |
| `PartialThreshold` | `150` | `-unroll-partial-threshold` | partial/runtime size budget |
| `PartialOptSizeThreshold` | `0` | `-unroll-optsize-threshold` | partial budget under `-Os` |
| `Count` | `0` | `-unroll-count` (via the user path) | forced factor; 0 = "decide" |
| `DefaultUnrollRuntimeCount` | `8` | — | starting factor for runtime unroll |
| `MaxCount` | `UINT_MAX` | `-unroll-max-count` | cap on partial/runtime factor |
| `FullUnrollMaxCount` | `UINT_MAX` | `-unroll-full-max-count` | cap on full-unroll factor |
| `BEInsns` | `2` (**AMDGPU 5**) | — | back-edge instructions not replicated |
| `Partial` | `false` (**AMDGPU true**) | `-unroll-allow-partial` | allow partial unroll |
| `Runtime` | `false` | `-unroll-runtime` | allow runtime unroll |
| `AllowRemainder` | `true` | `-unroll-allow-remainder` | allow a remainder loop |
| `AllowExpensiveTripCount` | `false` | — | allow costly trip-count math (e.g. division) |
| `Force` | `false` | — | force unroll even when runtime fails |
| `UpperBound` | `false` | — | allow using the max trip count |
| `UnrollRemainder` | `false` | `-unroll-remainder` | unroll the remainder loop too |
| `UnrollAndJam` | `false` | — | enable unroll-and-jam for the target |
| `UnrollAndJamInnerLoopThreshold` | `60` | — | inner-loop budget for U&J |
| `MaxIterationsCountToAnalyze` | `10` (**AMDGPU 32** for small inner loops) | `-unroll-max-iteration-count-to-analyze` | iterations the dynamic model simulates |
| `UnrollVectorizedLoop` | `false` (**AMDGPU true**) | — | keep unrolling vectorized loops |

Other relevant non-field knobs (not part of `UP`):

| Flag | Default | Effect |
|------|---------|--------|
| `-pragma-unroll-threshold` | `16384` | size budget granted to explicitly-pragma'd loops |
| `-flat-loop-tripcount-threshold` | `5` | profile trip count below which runtime unroll is suppressed |
| `-unroll-max-upperbound` | `8` | max constant max-trip-count eligible for bounded unroll |
| `-unroll-runtime-epilog` | `false` | force epilogue (vs prologue) remainder |
| `-unroll-runtime-multi-exit` | `false` | allow runtime unroll of multi-exit loops |
| `-forget-scev-loop-unroll` | `false` | forget all SCEV (not just the top loop) — compile-time tradeoff |

### 15.2 Override precedence

`gatherUnrollingPreferences` applies five layers in this fixed order
(`lib/Transforms/Scalar/LoopUnrollPass.cpp:183-272`):

1. **Generic defaults** (`:193-214`).
2. **Target hook** `TTI.getUnrollingPreferences` (`:217`) — this is where AMDGPU
   (§14) overrides.
3. **Opt-for-size** attribute (`:219-229`): when the function is compiled for size —
   `-Os` ("optimize for size") or `-Oz` (size at all costs), which mark the function
   `optsize`/`minsize` — the size budgets are swapped for their opt-size counterparts:
   `Threshold := OptSizeThreshold`, `PartialThreshold := PartialOptSizeThreshold`,
   `MaxPercentThresholdBoost := 100`. Since both opt-size thresholds default to `0`,
   this effectively turns *automatic* unrolling off in those modes (the static "unroll
   only if size `< Threshold`" gate can never pass) — only forced `#pragma unroll`
   still fires, via the separate `PragmaUnrollThreshold` (§13).
4. **Command-line flags** (`:231-253`), each applied only if explicitly passed
   (`getNumOccurrences() > 0`).
5. **Pass/user arguments** (`:255-269`), last and winning (e.g. a user threshold
   sets both `Threshold` and `PartialThreshold`).

So a `-mllvm -unroll-threshold=N` from the command line (layer 4) overrides
AMDGPU's target preference (layer 2) — which is what makes the causal experiment
in §16.1 work.

---

## 16. Worked Examples

These were produced on the local gfx906 toolchain (ROCm 5.7.1, the in-tree
clang) by compiling two HIP kernels device-only and reading the unroller's own
`-Rpass=loop-unroll` remarks plus the emitted ISA/IR. HIP device code compiles at
`-O3`, so both unroll passes (§4) are active with `Threshold = 300` and the
AMDGPU preferences of §14. The kernels are an `N×N` grid reduction with a tiny
per-cell body (`acc += in[...] * bi * bj`), a nested `for i { for j { ... } }`,
and **no** unroll pragmas — the decision is entirely the compiler's. The only
difference between the two is where the bound `N` comes from.

The kernels, the build `Makefile`, and a full reproduction recipe live in a
committed companion repository,
[**`unroll_study`**](https://github.com/jbaileyhandle/unroll_study):
`kernel_const.hip.cpp` (the `constexpr` bound), `kernel_runtime.hip.cpp` (the
kernel-argument bound), and a `README.md`. §16.3 gives the exact commands; the rest
of this section reads the results.

### 16.1 Constant trip count → full and partial unroll

In the const variant, `N` is a `constexpr`, so SCEV knows the exact trip count of
both loops. Sweeping `N` and reading the per-loop remarks:

| `N` | inner loop | outer loop | `s_cbranch` in ISA |
|-----|-----------|-----------|--------------------|
| 4, 6, 8, 9 | completely unrolled | completely unrolled | 0 (straight-line) |
| 10 | completely unrolled (10×) | **partial, factor 2** | 1 |
| 12, 16, 32 | completely unrolled | not unrolled (×1) | 1 |
| 64 | **partial, factor 8** | not unrolled | 1 |

The structure to read out of this:

- **Unrolling is inside-out.** The tiny inner body fully unrolls at every `N` up
  to 32; the *outer* loop is the one that crosses the budget first. This follows
  directly from the unroller visiting loops **innermost-first** (§5.1): it is rung
  3 of `computeUnrollCount` (§7) succeeding for the inner loop and then, once the
  inner is unrolled into it, succeeding or failing for the outer depending on the
  resulting size.
- **Full → partial → none is a size transition.** As `N` grows, the unrolled
  outer body `~ (per-cell size)·N` grows past the full-unroll `Threshold` (300,
  §8 gate 2), then past what a factor-2 partial unroll fits under
  `PartialThreshold` (150, §9). Note the asymmetry: the outer drops to partial at
  `N=10` and stops entirely by `N=12` precisely because the partial budget (150)
  is *tighter* than the full budget (300) — AMDGPU raised only the full threshold
  (§14.1).
- **The per-cell load does not fold.** The dynamic cost model (§8.2) can't help
  here: `in[...]` is a **global** load, not a constant-global load, so
  `visitLoad` can't fold it, the boost stays ~1×, and the decision falls back to
  the static size test.

**Causal proof that it's size-vs-threshold.** Moving `-unroll-threshold` moves
the boundary, in both directions:

| Configuration | outer loop | `s_cbranch` |
|---------------|-----------|-------------|
| `N=16`, default threshold 300 | not unrolled | 1 |
| `N=16`, `-mllvm -unroll-threshold=5000` | **completely unrolled** | 0 |
| `N=8`, `-mllvm -unroll-threshold=40` | **drops to partial ×2** | 1 |

Raising the budget makes a previously-too-big loop fully unroll; lowering it
makes a previously-fully-unrolled loop fall back to partial. That is exactly the
`getUnrolledLoopSize < Threshold` test of §8 (gate 2) and the `PartialThreshold`
fit of §9.

### 16.2 Runtime trip count → no unroll by default, remainder loop when forced

In the runtime variant, the bound `n` is a kernel argument, so SCEV cannot
produce an exact or max trip count — only a trip multiple of 1. That rules out
full and partial unroll (§12); only the runtime path (rung 7) is possible.

- **Default: no unrolling.** With no flags, *neither* loop is unrolled (no
  remarks; 3 `s_cbranch` in the ISA). This is §14.4 in action: AMDGPU leaves
  `UP.Runtime = false` for a global-memory loop, and the generic passes never
  turn it on, so rung 7 bails at `if (!UP.Runtime)` (§7).
- **Forced on: runtime unroll + remainder.** With `-mllvm -unroll-runtime`, the
  inner loop reports *"unrolled loop by a factor of 8 with run-time trip count"* —
  factor 8 is `DefaultUnrollRuntimeCount` (§7) — and an epilogue remainder loop
  appears (`s_cbranch` rises to 7). The remainder arithmetic is emitted exactly
  as §10.2 describes (here `%4` is `n`):

  ```llvm
  %17 = and i32 %4, 7      ; xtraiter = n % 8     (ModVal; & 7 because 8 is a power of two)
  %18 = icmp ult i32 %4, 8 ; if n < 8, skip the unrolled main loop entirely
  %19 = and i32 %4, -8     ; n & ~7 = main-loop trip count (largest multiple of 8 ≤ n)
  %20 = icmp eq i32 %17, 0 ; if remainder == 0, skip the epilogue loop
  ```

  This is the `TripCount & (Count − 1)` mask from
  `CreateTripRemainder` (`lib/Transforms/Utils/LoopUnrollRuntime.cpp:498-522`),
  named `xtraiter`, plus the around-branches of §10.2.

The contrast between §16.1 and §16.2 is the whole point: identical bodies, but a
`constexpr` bound takes the full/partial path and deletes or shrinks the loop,
while a runtime bound can only be runtime-unrolled — and on AMDGPU that is off by
default for global-memory loops, so the loop is left intact.

### 16.3 Reproducing these results

From the companion `unroll_study` repository (point `HIP_CLANG_PATH` at your built
in-tree `clang` and `ROCM_PATH` / `ARCH` at your toolchain — see its `README.md`
for prerequisites and the full sweep):

- **§16.1 sweep** (constant bound) — for each grid size, read the remarks and count
  back-edges (`s_cbranch = 0` ⇒ fully unrolled, nonzero ⇒ a loop survives):

  ```sh
  make remarks DEFS=-DGRID=10        # the unroller's own loop-unroll remarks
  make asm     DEFS=-DGRID=10
  grep -c s_cbranch kernel_const.s
  ```
- **§16.1 causal check** — move the size budget, move the boundary:

  ```sh
  make asm DEFS=-DGRID=16 EXTRA='-mllvm -unroll-threshold=5000'  # now fully unrolls
  make asm DEFS=-DGRID=8  EXTRA='-mllvm -unroll-threshold=40'    # now drops to partial
  ```
- **§16.2** (runtime bound) — off by default, runtime-unrolled when forced:

  ```sh
  make asm                                     # default: not unrolled (3 s_cbranch)
  make remarks EXTRA='-mllvm -unroll-runtime'  # forced: factor-8 runtime unroll
  make ir      EXTRA='-mllvm -unroll-runtime'  # shows the n&7 / n&-8 remainder math
  ```

---

## 17. Loop Structure in Real Workloads: A HeCBench Survey

Everything up to here is about how the unroller *decides*. This section asks the
empirical question it all hinges on: in real GPU kernels, what do the loops actually
look like — and therefore which unroll mode, if any, can even fire?

**Method.** From the HeCBench suite, the benchmarks that build and run on this gfx906 /
ROCm 5.7.1 setup are grouped by application domain (the "WORKING" category groups in
the benchmark harness). For each of the 17 categories, 3–4 benchmarks were chosen as
representative of the kernels most often run in that domain, and their `__global__`
kernels were read directly — **63 benchmarks** in all — to answer two questions about
each kernel's *dominant* (hot-path) inner loops:

1. **Shape** — a clean *counted* `for` loop whose trip count is an affine function of a
   numeric bound (`for (i=0;i<N;i++)`), or a `while` / data-dependent loop whose exit
   turns on an unpredictable condition (convergence, search-until-found, neighbor-list
   traversal)?
2. **Bound** — a **compile-time constant** the compiler can see (a `#define`,
   `constexpr`, template parameter, or literal), or **runtime-determined** (a kernel
   argument, a value loaded from memory, `gridDim`/`blockDim`, the problem size)?

This is the same source-reading method as §16, applied broadly rather than to two
kernels. It is static analysis, not profiling — "dominant/hot" is judged from code
structure — and the per-category selection is representative, not exhaustive.

### 17.1 Headline: the shape is friendly, the bound is the limiter

Each of the 63 benchmarks is classified by its dominant loop's **shape** (the rows)
*and* its **bound** (the columns), and counted once. Each cell is the number of
benchmarks with that combination of the two:

| shape ↓ \ bound → | const | runtime | mixed¹ | none | **total** |
|---|---|---|---|---|---|
| clean counted | **14** | **17** | 5 | 0 | **36** |
| mixed² | 0 | 1 | 11 | 0 | 12 |
| data-dependent | 0 | 6 | 0 | 1 | 7 |
| no significant loop | 0 | 0 | 0 | 8 | 8 |
| **total** | **14** | 24 | 16 | 9 | **63** |

The word "mixed" appears on both axes but means different things — it is worth keeping
them straight:
- ¹ **mixed *bound*** (column): the kernel's loops are all the same shape, but their
  *trip counts* differ — some constant, some runtime. (E.g. `mt`: a `for (i < 19)`
  state-init loop alongside a `for (i < nPerRng)` output loop whose bound is a kernel
  argument — both clean `for`-loops, only the bound visibility differs.)
- ² **mixed *shape*** (row): the kernel contains *different kinds* of loop — some clean
  counted `for`-loops and some `while` / data-dependent loops. (E.g. `bonds`: counted
  leg-loops plus a Brent/secant convergence `while` solver.)

Read the row totals against the column totals and the story falls out. The *shape* is
mostly **unroll-friendly** — 36 of 63 are clean counted loops, only 7 are purely
data-dependent. But the *bound* is where it breaks down: only **14 of 63** have a
compile-time-constant trip count (the entire `const` column — and notice every one of
them is also clean-counted, the top-left cell). That 14 is the precondition for *full*
unroll (§8); nothing else qualifies.

The gap between "nicely shaped" (36) and "constant-bounded" (14) is the whole point,
and it sits almost entirely in **one cell: clean counted ∩ runtime = 17**, the largest
in the table. These are loops perfectly shaped to unroll whose trip count is a runtime
problem size SCEV cannot see (§12) — so full unroll never fires, and only partial (§9)
or runtime (§10) unroll is even on the table.

### 17.2 Per-category breakdown

| Category (reps) | Representative benchmarks | Hot-loop shape | Hot-loop bound | Reachable unroll |
|---|---|---|---|---|
| automotive (1) | daphne | counted | const (3×3) | **full** |
| bioinformatics (4) | nw, bsw, minimap2, minibude | counted | mixed | full (const tiles) / runtime (seq lengths) |
| CV & image (4) | convolutionSeparable, sobel, srad, bilateral | counted / hand-unrolled | mostly const | **full** (filter / window / reduction) |
| cryptography (4) | aes, chacha20, keccaktreehash, ecdh | mixed | mixed | full (fixed rounds) / runtime (drivers) / none (ecdh) |
| compression & reduction (4) | histogram, scan2, bitpacking, segment-reduce | counted | mixed / runtime | partial/runtime (data) / full (reduction, channels) |
| encoding & verify (4) | crc64, md5hash, murmurhash3, ans | mixed | mixed / runtime | full (hash rounds) / runtime / none (ans) |
| finance (4) | black-scholes, binomial, libor, bonds | mixed | mixed | full (binomial) / runtime (libor) / none (bonds solver) |
| geoscience (4) | haversine, aidw, geodesic, hausdorff | mixed | mixed | full (aidw tile) / runtime (hausdorff) / none (geodesic) |
| graph & tree (4) | sssp, floydwarshall, cc, mis | **data-dependent** | runtime | **none** (traversal / convergence) |
| machine learning (4) | softmax, layernorm, attentionMultiHead, gelu | counted | **runtime** | **partial / runtime only** (problem size) |
| math (4) | blas-gemm, gemv, jacobi, lud | counted | split | runtime (GEMM/GEMV `K`) / full (jacobi, lud) |
| random numbers (4) | mt, sobol, qrg, rng-wallace | counted | mixed | full (state init) / runtime (output count) |
| robotics (2) | inversek2j, rodrigues | counted / none | const | **full** |
| search (4) | bfs, bsearch, b+tree, tsp | mixed | runtime | runtime (tree height) / none (binary search) |
| signal processing (4) | fft, sosfil, lombscargle, zmddft | counted | mixed | full (FFT radix) / runtime (spectral loops) |
| simulation (4) | hotspot, nbody, lavaMD, xsbench | counted | **runtime** | partial/runtime (problem size) / full (fixed inner) |
| sorting (4) | radixsort, bitonic-sort, merge, quicksort | mixed | runtime | runtime / none (binary search) / full (bitonic scan) |

### 17.3 Three recurring patterns

Across the survey, essentially every kernel's loops fall into three buckets:

1. **Small constant-bound structural loops → fully unrollable, often already
   pragma'd.** Fixed matrix dimensions (daphne's 3×3 transform, jacobi's 4), filter or
   stencil radii (convolutionSeparable's `KERNEL_RADIUS` → 17 iterations), reduction
   trees (srad's `NUMBER_THREADS` = 256, warp-size reductions), channel counts, and
   fixed crypto rounds (chacha20's 10 / 16). These satisfy §8's static size test and
   fully unroll — and the benchmark authors frequently `#pragma unroll` them already.
   **But they are usually not the dominant cost**: they are small inner loops nested
   inside a larger, runtime-bounded one.

2. **Runtime-bounded counted loops → the hot path, but only partial/runtime unroll
   applies.** The loop that actually dominates runtime almost always iterates over the
   *problem size*: GEMM's contraction dimension `K`, softmax's slice length, attention's
   sequence length, n-body's particle count `n`, hotspot's grid. These are clean counted
   loops — perfectly legal to unroll — but their trip count arrives as a kernel argument
   or a value from memory, invisible to SCEV (§12), so full unroll (§8) is impossible
   and only partial (§9) or runtime (§10) unroll could fire.

3. **Data-dependent loops → not unrollable.** Graph traversal (sssp, cc, mis walking
   per-node neighbor lists with early exit), iterative solvers (bonds' Brent/secant
   yield solver, geodesic's convergence loop), and search-until-found (bsearch, merge's
   binary searches, ecdh's GCD `while`). The unroller leaves these rolled.

### 17.4 What it means for unrolling on AMDGPU

The practical upshot ties straight back to §14:

- The **full-unroll machinery this guide spends the most on (§8's dynamic cost model)
  fires mainly on the small structural loops** of pattern 1 — which are frequently
  already pragma-unrolled, so the compiler is often just confirming a decision the
  author already forced.
- The **arithmetic-heavy hot loops (pattern 2) are predominantly runtime-bounded**, so
  the only question for them is partial or runtime unroll — and that is exactly where
  AMDGPU is conservative: it leaves `PartialThreshold` at the generic 150 (§14.1) and
  leaves runtime unroll **off** for global-memory loops (§14.4). So for a large fraction
  of real kernels the **dominant loop is left rolled unless the author forces it** — the
  §16.2 runtime kernel, not the §16.1 const kernel, is the common case.
- **The cheapest lever is often in the source, not the compiler.** When a runtime bound
  is really a small fixed value the programmer knows (libor's `Nmat`/`N`, and others),
  promoting it to a compile-time constant (`constexpr` / template) converts a
  runtime-only loop into a full-unroll candidate — moving it from pattern 2 to pattern 1
  for free. The survey flagged this explicitly for libor; it applies wherever a "problem
  size" is actually a fixed small constant in disguise.

The 63 surveyed kernels live under `HeCBench/src/<name>/` in the `gpu2_benchmarks`
tree; the per-category selections are exactly those listed in §17.2.

---

## 18. Real-World vs. Benchmark Loop Bounds: Is §17 Representative?

§17 found that in HeCBench the dominant hot loop is usually clean-counted but
**runtime-bounded** — which would mean full unroll (§8) rarely fires on the loops that
matter. The natural worry: is that a property of *real* GPU kernels, or an artifact of
how benchmark kernels are written? This section answers it — first by reasoning, then by
spot-checking real, as-shipped kernels against matched HeCBench analogs.

### 18.1 The general picture

The honest answer splits along one axis, and flips depending on how you weight:

- **By raw kernel count, runtime bounds dominate.** The default idiom is a grid-stride
  loop over a problem size (`for (i = tid; i < n; i += stride)`) where `n` is a runtime
  argument.
- **By execution time / FLOPs, compile-time-constant inner loops dominate.** The kernels
  that consume the cycles — GEMM, convolution, attention, reductions — are specialized or
  generated with baked-in tile sizes.

These reconcile structurally: in optimized code the runtime-ness is **isolated to an
outer loop**, and the hot inner loop is **deliberately made constant** by tiling —
`for (k_tile = 0; k_tile < K; k_tile += BK)` (runtime `K/BK`) wrapping
`for (kk = 0; kk < BK; kk++)` where `BK` is a compile-time tile constant (fully unrolled).
GPUs push toward this harder than CPUs: fixed-size register arrays need *constant* indices
(SROA, §14.2), occupancy makes the unroll factor a first-class knob, and there is no
runtime branch predictor to lean on. This is why the high-performance ecosystem is built on
specialization — libraries even ship a *constant-K* GEMM kernel alongside a *variable-K*
one because the constant-K version optimizes better ([USPTO 10073815]), and modern codegen
(Triton marks `BLOCK_SIZE` as `tl.constexpr` and JIT-compiles a specialized kernel per
shape — [PyTorch Triton docs]) bakes shapes in before the backend unroller runs. The
consequence: the *compiler-level* constant rate is higher than a *source-level* survey like
§17 shows, because templating/JIT can turn a source-runtime bound into a compile-time
constant before `LoopUnroll` sees it.

### 18.2 Method: matched-pair spot check

To test this without drowning in build systems, each real-world kernel was paired with the
**comparable HeCBench kernel** (same computation), and their inner-loop bounds compared by
reading the shipped source and, where it built cleanly on gfx906, compiling a single
translation unit with `-fsave-optimization-record` / `-Rpass=loop-unroll`. The real-world
corpus deliberately spans general application code (**ggml**, the llama.cpp inference
backend) and optimized libraries (**rocPRIM**, **Composable Kernel**). This is a spot
check — four matched pairs — not an exhaustive study.

### 18.3 Matched pairs and their loop bounds

Every bound below was read from the actual source (not inferred):

| Computation | Real-world kernel — inner bound | HeCBench analog — inner bound |
|---|---|---|
| Quantized matrix×vector | **ggml** `mul_mat_vec_q`: `constexpr` (`qk`, `nwarps`, …) → **constant** | **gemv-hip** `gemv_fp16` (`kernels.h:77`): `num_per_thread >> 3`, `num_per_thread` a **runtime** arg |
| Tiled GEMM | **CK** `gemm_dl`: tile consts `K0PerBlock=16`, `K1=1`, `M1/N1PerThread=4` → **constant** | **blas-gemm-hip** `matrix_mul` (`main.cu:21`): `for k < K`, `K` a **runtime** arg, untiled |
| Quantized matmul | **ggml** `mul_mat_vec_q`: `constexpr` → **constant** | **quantVLLM-hip** (`main.cu:22`): `for i < hidden_size`, **runtime** arg |
| Block reduction | **rocPRIM** `block_reduce`: `ItemsPerThread` template const + `ROCPRIM_UNROLL` → **constant** | **softmax-hip** `softMax2`: data loop `< sliceSize` **runtime**; warp-reduce tree `WarpSize` const |

In **every pair**, the real-world/library kernel makes the per-thread / inner-tile count a
**compile-time constant** (template or `constexpr`), while the HeCBench analog passes the
*identical quantity* as a **runtime kernel argument**. The difference is not algorithmic —
same GEMV, GEMM, quant, reduction — it is purely *where the size lives*. Two details, both
verified: HeCBench's `gemv` even puts `#pragma unroll` on its runtime loop (intent was
there; the runtime bound defeats it), and a `const int sliceSize` parameter is still a
runtime value (the `const` only forbids reassignment).

### 18.4 The trip-count magnitudes are the same — only visibility differs

Resolving the HeCBench runtime bounds against the actual run commands in the benchmark
harness, and the real-world bounds against the source, the inner trip counts land in the
**same small range — single digits to ~64, mostly powers of two — on both sides**:

| Kernel | inner trip count | kind |
|---|---|---|
| ggml `mul_mat_vec_q` | **8** (dp4a over a 32-elem chunk; helpers 6, 2) | constant |
| CK `gemm_dl` | **16** (K0 tile), **4** (M1×N1 thread tile) | constant |
| gemv-hip (`size=16384`, `-x 512/128/32`) | **4 / 16 / 64** (`= (16384/x)/8`) | runtime |
| softmax-hip (`argv[2]=784`) | **784** data; **6** warp-reduce tree | runtime / const |
| quantVLLM-hip (`argv[2]=5137`) | bound **5137** (grid-stride) | runtime |
| blas-gemm-hip (`argv[2]`) | **91** and **4096** (whole untiled K) | runtime |

So the benchmark loops are not large or oddly shaped — gemv-hip's inner loop literally runs
**4, 16, or 64** times, textbook unroll-friendly counts. ggml computes the same-magnitude
quantity as a hardcoded **8** and fully unrolls it; gemv-hip computes 4/16/64 at runtime
from `size ÷ block_dim` and cannot, even with `#pragma unroll`. *Same numbers, opposite
outcome — the only difference is compile-time visibility.*

The one revealing exception is **GEMM**: blas-gemm-hip's inner loop is the *entire* untiled
runtime `K` (**91** or **4096**), while CK tiles that same `K` into `K0PerBlock = 16`
constant chunks. The production library *creates* a small constant inner loop by tiling a
large runtime dimension; the benchmark just loops over the whole runtime `K`. That is the
tiling difference made concrete: **4096 (runtime, naive) vs 16 (constant, tiled).**

### 18.5 Compiler confirmation

Compiling ggml's `mmvq.cu` for gfx906 (a partial run — the TU instantiates a kernel per
quant type and is large) emitted **959 "completely unrolled" remarks** for exactly these
inner loops — `vecdotq.cuh:697/710` at **8** iterations, `common.cuh:449` at **6**,
`mmvq.cu` sites at **2** — confirming the compiler treats the `constexpr` bounds as
constant-trip and fully unrolls them. The HeCBench analogs cannot reach that state: their
matching loops carry a runtime trip count, so `LoopUnroll` is left with at most partial or
runtime unroll (§9, §10), which on AMDGPU is mostly off for global-memory loops (§14.4).

### 18.6 Conclusion and honest caveats

**§17's runtime-bound finding is real and representative — of the hand-written / benchmark
population (count-weighted).** It is *not* representative of where production cycles go:
performance-conscious general code (ggml) and optimized libraries (rocPRIM, CK) bake the
inner dimensions into compile-time constants, or *create* a constant inner tile by tiling a
runtime dimension, and those inner loops fully unroll. Weighted by execution time, the
answer to "static or runtime bounds?" most likely **inverts** relative to §17.

Caveats, stated plainly:
- **Spot check, not a survey** — four matched pairs, chosen for comparability, not a random
  sample. The general-picture claims in §18.1 are reasoned synthesis, not a measured
  distribution (no clean published statistic for this metric exists — part of why it was
  worth checking at all).
- **Asymmetric evidence** — ggml was confirmed by compilation (partial: the TU timed out
  after 959 unroll remarks); CK and rocPRIM are source-confirmed only. CK's standalone build
  needs a CMake-generated `config.h`, and gfx906 has no matrix cores, so matrix-core GEMM
  libraries (rocWMMA, CK-XDL) cannot be compiled here at all — the FMA `gemm_dl` path was
  read instead.
- **gfx906-specific, single-TU** — bounds reflect this target and isolated translation
  units, not whole-program builds.

[USPTO 10073815]: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10073815
[PyTorch Triton docs]: https://docs.pytorch.org/tutorials/recipes/torch_compile_user_defined_triton_kernel_tutorial.html

---

## 19. Making Benchmark Unrolling Realistic

§18 established that HeCBench's hot loops are usually runtime-bounded where the
equivalent real-world kernels are constant-bounded, so the benchmarks **under-unroll**
relative to production code. For research on **instruction scheduling and register
pressure** this is not a cosmetic gap: unrolling is a primary lever on basic-block size
(hence scheduling freedom) and on register pressure, so a benchmark that fails to unroll
the way real code would puts the study on an unrepresentative code shape. This section
records the options for closing that gap and the path chosen.

### 19.1 The requirement, and the §18 guardrail

The goal is to reproduce the **real shape**, not merely "more unrolling": fully-unrolled,
constant-trip inner loops that collapse to large straight-line basic blocks (and let an
outer loop's unrolled inner body inline into one block). A transform that produces
unrolling of a *different* shape — e.g. a loop plus a remainder loop — does not satisfy
the requirement, because the block structure and register-pressure profile it presents
are artifacts of that transform, not of real code.

What may be made constant is bounded by the §18 finding, which doubles as the guardrail
against over-specialization: **make the inner tile / per-thread / reduction count a
compile-time constant; keep the grid / workload / problem size runtime.** Freezing the
workload would optimize in ways no real kernel does and risks invalidating the benchmark.

### 19.2 The multi-configuration complication

A benchmark is not run at one size. `benchmarks.json` gives each benchmark a list of
`run_commands` with different parameters, and a single baked-in constant is correct for
only one of them (and is *undefined behavior* for the others under an `assume`-based
approach). The correct unit is therefore **(benchmark, configuration)**, with two
simplifications that keep it tractable:

- Specialize per **distinct inner trip count**, not per run command — many commands
  collapse to the same inner loop. Measured examples: `gemv-hip` (`-x 512/128/32`) →
  **3** distinct inner trips (4, 16, 64); `blas-gemm-hip` (`K = 91`, `4096`) → **2**;
  `softmax-hip` (both runs `sliceSize = 784`) → **1**. Typically 1–3 variants per
  benchmark.
- The variant set is **derivable automatically** from `run_commands` by applying the same
  argument→trip-count derivation §18 used, then de-duplicating.

### 19.3 The options

| # | Method | Mechanism | Effort / kernel | Real shape (single big BB)? | Remainder loop? | Handles multiple configs? | Faithfulness |
|---|--------|-----------|-----------------|-----------------------------|-----------------|---------------------------|--------------|
| **1** | Runtime unroll + raise threshold | Global TTI change (`UP.Runtime`, threshold) | zero | **No** (always loop+remainder) | always | **native** (runtime) | low — wrong shape |
| **2** | `constexpr`/template (single value) | Move arg → compile-time constant in source | tiny (or rewrite if workload-bounded) | yes | no | **no** alone (→ becomes #6) | high |
| **3** | `__builtin_assume(bound==C)` | One-line hint; arg stays runtime | 1 line | yes, *if* it folds to an exact constant trip (verify) | no | **no** alone (UB on other configs; needs #4) | high |
| **4** | `-D` macro / per-config build | Constant from the build flag; one binary per config | tiny + build matrix | yes | no | **native** (one build per config) | highest (Triton/JIT) |
| **5** | Compiler specialization attribute | Fork feature: attribute/table → clone + const-prop + unroll | 1 annotation (+ build the feature) | yes | no | yes, if it emits a *set* + dispatch | high |
| **6** | Multi-versioning dispatch | Template kernel + host `switch` + generic fallback | most code | yes (specialized path) | no | **native by design** | highest for libraries (incl. dispatch cost) |

Under the multi-configuration constraint the single-value methods do not stand on their
own: **#2** needs a dispatch added (which turns it into **#6**), and **#3** is only sound
inside a per-config build (**#4**), where `-D` is the cleaner way to supply the constant.
The methods that handle every configuration as shipped are therefore **#1** (runtime,
natively), **#4** (one specialized build per configuration), **#6** (one binary, dispatch
on the runtime argument, generic fallback — exactly how cuBLAS/rocBLAS ship), and **#5**
(the same as #6 but generated by a compiler feature from the `benchmarks.json` variant
list). The §18 guardrail (constant inner tile, runtime grid) governs which quantities each
specialization is allowed to freeze.

### 19.4 Chosen path

**Runtime unroll (#1) as the first pass; multi-versioning dispatch (#6) as the
refinement.** Rationale:

- **#1 first** because it is near-zero effort (a single TTI change), is **configuration-
  agnostic** (runtime unrolling works for every size with no per-config work), and gets
  the benchmarks unrolling *at all* quickly so the study can begin. Its limitations are
  understood and accepted for a first pass: it leaves a remainder loop, the unroll factor
  is chosen without knowledge of the actual bound (so the unrolled body can be skipped
  while the remainder carries the work), and it **cannot** produce the single large basic
  block — so results from this pass are a coarse approximation, not the representative
  shape.
- **#6 as the refinement** because it is the option that delivers the real shape for
  *every* configuration in one binary, mirrors how production libraries actually dispatch,
  and keeps the build system clean. **Per-config builds (#4) were rejected** specifically
  to avoid a per-configuration build matrix complicating the build system.

In short: #1 to start cheaply and unblock measurement, #6 to make the unrolling genuinely
representative if the direction proves worth pursuing.

---

## 20. Tuning and Controlling Unrolling

Ordered roughly from most targeted to most global:

- **`#pragma unroll` / `#pragma unroll N`** (in the kernel source). Emits
  `llvm.loop.unroll.enable` / `...count` (§13), bypasses the cost model where it
  can, and otherwise raises the budget to `16384`. The most reliable way to force
  a *specific* loop.
- **`amdgpu-unroll-threshold` function attribute.** Overrides `UP.Threshold` for
  one function (§14.1) without touching others.
- **`amdgpu.loop.unroll.threshold` metadata.** Sets both `Threshold` and
  `PartialThreshold` for one loop (§14.3).
- **`-mllvm -unroll-threshold=N`.** Global full-unroll budget override (layer 4,
  §15.2); demonstrated causal in §16.1. Reliable for experiments.
- **`-mllvm -unroll-runtime`.** Globally enable runtime unrolling — required to
  unroll runtime-trip-count loops on AMDGPU (§16.2). Pair with
  `-mllvm -unroll-count=N` to force a specific runtime factor.
- **`-mllvm -unroll-partial-threshold=N`, `-unroll-max-count=N`,
  `-unroll-allow-partial`.** Tune the partial path (§9, §15).
- **`-fno-unroll-loops`** (frontend). Clears `PTO.LoopUnrolling`, making every
  pass `OnlyWhenForced` — automatic unrolling off, pragmas still honored (§4.1).

When investigating *why* a particular loop did or didn't unroll, the first tools
to reach for are the optimization remarks — they work in a release (no-asserts)
build, unlike `-debug-only=loop-unroll`:

```
-Rpass=loop-unroll            # what was unrolled, and how (full / factor N / runtime)
-Rpass-missed=loop-unroll     # loops the unroller considered but skipped
-Rpass-analysis=loop-unroll   # cost-model analysis remarks
```

These are what produced every number in §16.

---

*Cross-references: this guide is a companion to `AMDGPUMachineSchedulerGuide.md`
(machine scheduling, which runs much later, on MIR). Unrolling is a middle-end IR
transform and is complete long before instruction scheduling sees the function.*
