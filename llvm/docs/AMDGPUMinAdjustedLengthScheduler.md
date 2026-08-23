# The Min-Adjusted-Length Scheduler: Objective and Derivation

This document records **what the min-adjusted-length pass optimizes and why** —
the machine model behind its score, the derivation, the worked examples that
motivated it, and the assumptions it knowingly makes. The pass itself
(`RunMinimizeAdjustedLengthPass`, misched.txt flag `min_adjusted_length`) lives
in `llvm/lib/Target/AMDGPU/HierarchicalScheduler/ScheduleDAGHierarchicalScheduler.{h,cpp}`;
this doc is the reference for the *reasoning*, which is too long for a header
comment. See `AMDGPUMaxOccupancyScheduler.md` for the stock scheduler this
builds on.

## Goal

Keep the GPU busy. That has two coupled halves: how often an individual wave
can issue (its schedule's stalls), and how many concurrent waves exist to cover
each other's stalls (occupancy, which trades off against per-wave register
budget). The pass chooses the schedule **and** the occupancy jointly, by
scheduling the kernel once per reachable occupancy tier and committing the tier
whose predicted time is smallest.

The prediction is the score derived below. The sums run across regions
**before** the max is taken — see "Scoring granularity" for why:

```
kernel time per wave  =  max( Σ_r w_r·I_r ,  (Σ_r w_r·L_raw_r) / o )

I_r     = region r's issue-slot demand (instruction count, flat model)
L_raw_r = region r's raw schedule length: the wave's UNCONTENDED lifetime
          in that region (issue cycles + exposed latency stalls),
          divisor-1 lens
o       = resident waves per SIMD (the candidate's ACTUAL occupancy)
w_r     = region weight (1, or a static hotness estimate)
```

## Derivation: the assembly-line model

Work in one SIMD's **issue slots**: the SIMD issues at most one instruction per
cycle, from whichever resident wave is ready. A kernel runs W waves through the
SIMD (W is large); in steady state a finished wave comes off the line every `S`
cycles, so total time ≈ `W · S`. The question is the smallest achievable `S`.
There are two independent floors.

**Floor 1 — the issue port.** Every wave needs `I` slots and the port hands out
one per cycle, shared. No amount of concurrency changes the total:

```
S ≥ I
```

**Floor 2 — residency slots (Little's law).** A wave occupies one of the `o`
residency slots for its whole lifetime, *including* its latency stalls — it
does not yield the slot while waiting on a load. Uncontended, that lifetime is
the raw schedule length `L_raw`. Little's law is the accounting identity:
(waves in flight) = (completion rate) × (lifetime), so with at most `o` in
flight:

```
completion rate ≤ o / L_raw      ⇒      S ≥ L_raw / o
```

Read `L_raw/o` as: `o` slots, each recycled every `L_raw` cycles, can deliver
at most one finished wave every `L_raw/o` cycles. No individual wave finishes
that fast — each still lives `L_raw` — the *line* delivers one that often
because `o` are in flight at staggered stages.

Both floors always apply, so:

```
S = max( I , L_raw / o )
```

Per-wave lifetime under contention can stretch beyond `L_raw`, but only when
the port is busy — which is exactly when the `I` arm is binding anyway, so the
max is self-consistent.

### Worked example 1: the arm switch ("squeezed" latency)

Region: a 100-op dependent ALU chain in parallel with one λ=200 load, both
feeding a final consumer. `I ≈ 101`, `L_raw = max(100, 200) ≈ 200`.

| o | L_raw/o | I   | S = max | binding constraint            |
|---|---------|-----|---------|-------------------------------|
| 1 | 200     | 101 | 200     | load latency — occupancy helps |
| 2 | 100     | 101 | 101     | boundary                       |
| 4 | 50      | 101 | 101     | issue port — occupancy is free |

At o=4 the load path has been "squeezed" (its per-wave throughput share is 50)
below the ALU work, and the binding constraint switches to the port — where the
chain's 100 issues live, since issue demand cannot be overlapped away. The
intuition "occupancy changes which path is critical" is captured as *which arm
of the max binds*, with no per-edge re-weighting.

Sanity check: at o=4, waves actually in flight = lifetime/S = 200/101 ≈ 2 of
the 4 slots — the occupancy cap isn't binding, which is precisely why raising
it further buys nothing.

### Worked example 2: why NOT per-edge λ/o (the original "adjusted lens")

The pass's first version scored tiers by schedule length computed with every
edge latency divided by o (`max(1, λ/o)`). Consider 1000 independent ALU ops
followed by one λ vmem load and its consumer:

```
per-edge lens:  L_adj(o) = 1000 + λ/o          — strictly decreasing in o, forever
correct model:  S(o)     = max(1001, (1000+λ)/o) — flat once o ≥ (1000+λ)/1001
```

The per-edge form keeps paying occupancy a dividend after the ALU work has long
since saturated the issue port. The error is structural: dividing the edge and
*serially appending* it models the shrunken shadow as extra time after our
wave's work, when in the machine the coverage is *concurrent* — other waves'
issue work fills the shadow. Equivalently, dividing every interval by o assumes
full port contention on every issue, which only holds when the port is
saturated — and when it is, latencies are fully hidden and the answer is just
`I`. The per-edge lens double-counts contention and latency; `max(I, L_raw/o)`
is the consistent fluid limit in both regimes. Consequence of the bug: the
tier sweep systematically over-valued occupancy against register room — at
exactly the decision it exists to make.

Note the equivalent "coverage" form, sometimes more intuitive: with `B = L_raw
− I` (the schedule's bubbles), `max(I, (I+B)/o) = I + max(0, B − (o−1)·I)/o` —
other waves' issue work `(o−1)·I` covers your bubbles; only the uncovered
remainder is machine idle.

## Scoring granularity: sum across regions BEFORE the max

The derivation above is about a wave's **whole life**, and a wave flows through
every region. Applying the formula per region and summing —
`Σ_r max(I_r, L_r/o)` — is a different (and wrong-by-default) model: it
implicitly asserts that a region's stalls can only be covered by other waves
executing *the same region*. In the machine, interleaved waves are at different
points of the kernel at the same moment, so one region's stalls are covered by
issue work from whatever regions the other waves are in.

Concretely: region A = 1000 ALU ops, no stalls; region B = 10 ops plus a long
load, raw length 2000. Per-region scoring gives B
`max(10, 2000/o)` — a huge exposed stall, since B's own 10 instructions can't
cover it — and calls the kernel latency-bound. Summed-first scoring at o=3
gives `max(1010, 3000/3) = 1010`: B's stalls are fully covered by A's
instructions, executed by other waves that are in A while this wave stalls in
B. That is what actually happens when waves are spread through the kernel.

The error is not just pessimism; it mis-ranks tiers. Per-region scoring
overvalues anything that shrinks a stall-heavy region — including dropping to
a lower tier for more registers — even when those stalls are already covered
kernel-wide and the occupancy sacrifice buys nothing. Same failure direction
as the per-edge lens, one level up.

When per-region *would* be right: all resident waves in one workgroup with
barriers re-syncing them — then waves do march through regions together and
coverage really is region-local. That is the convoying caveat (see Assumptions)
at kernel scope; the summed form is the deliberate default, revisit if
barrier-heavy kernels underperform the model.

## Consequences for the algorithm

1. **The tier does not enter the search objective.** `Σ I_r` is
   schedule-invariant, and the lifetime sum `Σ L_raw_r` decomposes over
   regions, so minimizing each region's raw length independently is optimal at
   every tier. Region graphs are built at divisor 1; the tier enters only as
   the register budget (via the MFI occupancy target the DFS policies read
   live) and in the cross-tier arithmetic.

2. **Scoring needs no re-measurement.** `L_raw` is occupancy-independent, so
   scoring a candidate at its actual occupancy is pure arithmetic on numbers
   recorded at search time.

3. **Actual occupancy, not the searched tier.** The budget only bounds pressure
   from above: a min-length schedule found under `budget(o)` can land in a
   higher occupancy bracket than o (never lower — the policy gate forbids it).
   The waves that exist at runtime are the actual bracket's, so the score and
   the committed occupancy target both use the kernel-wide min over regions of
   each schedule's launch-floor-clamped all-factors occupancy.

4. **Saturation is kernel-wide only.** The score never goes below `Σ w·I`, and
   any tier achieving `Σ w·L_raw ≤ o·Σ w·I` sits at that floor. There is NO
   per-region saturation target — stopping a region's search at `o·I_r` would
   forgo raw-length reductions that still pay whenever the kernel as a whole
   is latency-bound. Every tier is searched fully; compile-time shortcuts
   derived from saturation are deliberately not taken in this draft (constant
   factors, and the full tier table is diagnostic output worth having while
   the model is unvalidated).

5. **Ties are common and meaningful.** Every tier whose candidate is
   issue-bound scores `Σ w·I` — the model saying "occupancy is irrelevant for
   this kernel". Ties go to the **higher actual occupancy**: more waves dampen
   the damage of a latency mis-estimate, so the insurance is free.

## Robustness: the two-sided stress-lens tie-break (planned)

The model's latencies are estimates, and measured reality is far from them
(VMEM median ≈ 3.4× the model, population spread 52–964 SQ ticks, errors in
both directions — no constant fixes it). Two schedules that tie under model-λ
can differ sharply in how they degrade when λ is wrong:

- **λ under-estimated (real loads slower):** overlapped loads (MLP) share their
  extra exposure once; serialized loads pay it per load. Under model-λ both may
  fit inside other work and tie.
- **λ over-estimated (real loads faster):** consider a shadow the model says is
  1000 cycles, filled with ALU work that could run serially (span 500, internal
  stalls) or with ILP (span 200, none). Both fit; raw length identical; the
  schedules are indistinguishable to any length measure under model-λ. If the
  real shadow is 300, the serial version's chain pokes out past the load and
  the ILP'd one doesn't.

So MLP and shadow-interior ILP are both *robustness* properties, invisible to
the primary objective precisely when they matter. They cannot be recovered
post-hoc: the DFS keeps one best schedule and discards an equal-length
schedule at the moment of comparison — there is nothing left to tie-break
afterwards. The mechanism must therefore be **score-recipe dimensions evaluated
during the search**: the working schedule additionally tracks its length under
perturbed lenses (λ×K and λ/K, K from config), and the recipe orders candidates
lexicographically — raw length, then stress-up length, then stress-down length.
The comparison that today discards an equal candidate instead resolves the tie
on robustness, mid-search. (Cost: extra length trackers in the search's hot
loop; they are enabled only for this pass, and can be restricted to the
post-feasibility phase.)

## The pass, end to end

```
RunMinimizeAdjustedLengthPass:
    tiers = MFI ceiling after occupancy pass  ..down to..  launch floor
    for o in tiers:                              # order irrelevant (SetOccupancyTarget)
        SetOccupancyTarget(o)                    # budget(o), read live by DFS policies
        for r in regions:
            sched[o][r] = min-raw-length search  # divisor-1 graph, seeded w/ MF order
            record raw_length, issue_slots, achieved_occupancy
        actual[o] = min_r achieved_occupancy     # >= o, can exceed it
        score[o]  = max(Σ_r w_r·issue_slots_r,
                        ceil(Σ_r w_r·raw_length_r / actual[o]))
    winner = argmin score, ties -> higher actual occupancy
    SetOccupancyTarget(actual[winner]); apply winner's buffered orders
```

Requires the occupancy pass (enforced at config build): its ceiling is the top
tier, and its committed schedules are a seed feasible at every tier at or below
it.

## Assumptions and known gaps

- **Smooth stagger.** The aggregate model treats bubbles as spreadable across
  the resident set. If waves convoy — most plausibly just after a barrier —
  they enter their shadows together and coverage fails even though the totals
  say otherwise. Second-order; revisit if wave-state data near barriers says it
  bites.
- **High turnover.** `W ≫ o`, retired waves replaced promptly. True for the
  kernels of interest (thousands of waves per SIMD slot); the model degrades
  for persistent-style kernels.
- **Flat issue port.** `I` = instruction count assumes one uniform port. gfx906
  co-issues across categories (VALU/VMEM/scalar) from different waves, so the
  true bound is per-category: a VMEM-heavy region can saturate the memory pipe
  at `I_vmem < I_total`. Refinement: `I = max` over per-category counts, using
  the same classification the characterize arbiter counters validated. Start
  flat.
- **λ independent of o.** On bandwidth-bound kernels more waves raise real
  latency (queueing), so the `L_raw/o` arm is optimistic there. Parked with the
  empirical lat(O) work (forced-occupancy characterize runs).
- **Region weights.** Loop trip counts are unknown at sched time; `w_r` is the
  estimated executions per wave, entering BOTH sums. Default:
  `loop_weight_base ^ loop_depth` (`min_adjusted_length.region_weighting =
  loop_depth`, base tunable via `min_adjusted_length.loop_weight_base`,
  default 10) — chosen over MBFI's static frequencies because the base is
  sweepable against measured GRBM and the resulting vote is auditable by hand,
  where MBFI's profile-free guess (~31 per loop level) is neither; `none`
  (all-1) is the opt-out. MBFI remains the upgrade path if PGO arrives or
  branchy kernels are shown to mispick tiers. Any flavor is a guess; PGO would
  make it real.
- **Pre-RA pressure is an estimate** (inherited caveat — see the MaxOccupancy
  doc): a schedule the pass considers to fit at a tier can still spill at
  allocation.

## Validation

The score is a falsifiable prediction. Gates, in order: (1) A/B on kernels with
reachable tiers and binding pressure (d3q19-collide first), GRBM/sclk as truth,
never wall-clock; (2) the scheduler-sensitivity correlation study — the model
predicts insensitivity exactly where measured arm spread is flat (issue-bound /
ceiling-saturated kernels) and sensitivity where the `L_raw/o` arm binds; a
mismatch there is evidence against the model, not against the kernels.
