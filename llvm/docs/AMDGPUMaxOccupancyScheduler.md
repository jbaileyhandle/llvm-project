# The AMDGPU MaxOccupancy Scheduler

This document describes how the default AMDGPU pre-RA machine scheduler
(`GCNMaxOccupancySchedStrategy`, driven by `GCNScheduleDAGMILive`) works. Code
lives in `llvm/lib/Target/AMDGPU/GCNSchedStrategy.{h,cpp}`; the node comparison
is the generic scheduler's, in `llvm/lib/CodeGen/MachineScheduler.cpp`. File/line
references are indicative and may drift.

Pseudo-code below is simplified for clarity — it omits SGPR/AGPR parallels, debug
plumbing, and edge cases. It conveys the shape of the algorithm, not something to
transcribe back into C++.

## Goal

Schedule each region for the **highest occupancy** (waves per SIMD/EU) that its
register pressure allows, and, where occupancy is already maxed, use the
remaining freedom to improve instruction-level parallelism / schedule length.
Occupancy is register-bound: fewer simultaneously-live registers ⇒ more waves fit
⇒ more latency hiding. So register pressure is the primary cost, but only up to
the point where it would cost a wave — below that line the scheduler is free to
optimize for latency.

## The occupancy model

"Occupancy" is waves/SIMD, capped by the hardware, by LDS use, by the workgroup
size, and — the part the scheduler controls — by register pressure.

- **`MFI.Occupancy`** — the occupancy the function is *compiled for*. Starts at
  `computeOccupancy()` (max the arch / LDS / launch bounds allow, ignoring
  registers) and is only ever **lowered** from there, when register pressure
  can't be kept low enough to sustain it.
- **`StartingOccupancy`** — `MFI.getOccupancy()` captured once when the DAG is
  built ([:501](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)). Fixed for the run.
- **`MinOccupancy`** — the lowest *real* occupancy recorded across the regions
  scheduled so far: the function-wide occupancy currently being committed to.
  Starts at `StartingOccupancy`, pushed **down** when a region can't hold the
  current occupancy, pushed **up** by the reschedule stages when they recover it.
  `MFI.Occupancy` is kept in lock-step with it.

From the current target, `GCNSchedStrategy::initialize`
([:91](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)) derives two register limits:

```
TargetOccupancy   = MFI.getOccupancy()
VGPRCriticalLimit = getMaxNumVGPRs(TargetOccupancy) - bias   # budget to *hold* the target occupancy
VGPRExcessLimit   = numAllocatableVGPRs             - bias   # absolute ceiling; beyond it we spill
# (SGPR analogues computed the same way)
```

- **Critical limit** — cross it and occupancy drops below the target.
- **Excess limit** — cross it and the register allocator will *spill*.

> Everything here is pre-RA register **pressure** (peak simultaneous live
> registers), an estimate. The real allocator runs much later and can need more
> registers than the pressure count suggests (aligned wide-load tuples,
> live-range fragmentation), so a schedule the scheduler considers spill-free can
> still spill at allocation.

## Scheduling one region: the candidate comparison

For each open slot the list scheduler compares ready instructions pairwise and
keeps the better one (`pickNodeFromQueue`,
[:221](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)). MaxOccupancy does **not**
override the comparison — it uses the generic `GenericScheduler::tryCandidate`
([MachineScheduler.cpp:3902](../lib/CodeGen/MachineScheduler.cpp)) — but feeds it
AMDGPU register-pressure deltas via `GCNSchedStrategy::initCandidate`
([:133](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)).

### Pressure is a threshold, not a gradient

This is the key subtlety. `initCandidate` records a candidate's pressure delta
**only when scheduling it would push pressure to or past a limit**:

```
# curPressure = pressure before this candidate; newPressure = after scheduling it.
#
# Excess: only start watching once curPressure is within a 16-register lookahead
# of the ceiling, then flag a candidate whose own pressure actually reaches it:
if curPressure + 16 >= ExcessLimit  and  newPressure >= ExcessLimit:
    RPDelta.Excess   = newPressure - ExcessLimit
# Critical: flagged as soon as a candidate's pressure reaches the budget:
if newPressure >= CriticalLimit:
    RPDelta.Critical = newPressure - CriticalLimit
# below the limit the delta stays unset (zero);
# reaching either limit also sets a per-region HasHighPressure flag
```

So **below the occupancy budget the scheduler does not rank candidates by
register pressure at all**: both deltas are unset, the pressure tests are no-ops,
and it falls through to the latency/ordering heuristics. It is not "prefer the
default only when pressure is exactly equal" — it is "neither candidate threatens
the budget, so pressure is irrelevant here, no matter how their raw pressures
differ." The pressure tests bite only once a candidate would cross the occupancy
budget (Critical) or come within 16 registers of / over the spill ceiling
(Excess), and then they prefer the candidate that overshoots **least**.

That is exactly "max occupancy": hold pressure under the target-occupancy budget,
but spend everything below that budget on latency/ILP.

### The comparison order

`tryCandidate` runs these tests in order; the first one that finds a difference
decides the winner and the rest are skipped:

```
tryCandidate(best, cand):
    1.  biasPhysReg                     # shorten physreg live ranges
    2.  RegExcess                       # least overshoot of the spill ceiling   (AMDGPU delta)
    3.  RegCritical                     # least overshoot of the occupancy budget (AMDGPU delta)
    4.  Latency*                        # acyclic latency-limited loops: chase the critical path now
    5.  Stall*                          # unbuffered-resource op that's ready soonest
    6.  Cluster                         # keep cluster-edged memory ops adjacent
    7.  Weak*                           # satisfy weak ordering edges
    8.  RegMax                          # least increase of the region's overall peak (see note)
    9.  ResourceReduce / ResourceDemand*# balance functional-unit usage
    10. Latency*                        # otherwise: prefer the shorter critical path (ILP)
    11. NodeOrder*                      # tie-break: original program order
    (* these apply only when both candidates are on the same boundary — top vs
       bottom picks compare only the always-on tests.)
```

Steps 2, 3 (and nominally 8) are the occupancy guards — the AMDGPU-specific part,
fed by `initCandidate`'s deltas. The rest are the stock generic heuristics. In
detail:

1. **`biasPhysReg`** — Physical registers can't be renamed by the allocator, so a
   fixed physreg (a call argument, a special register, the source/dest of a
   `COPY` to/from a physreg) that stays live across many instructions blocks that
   register the whole span. This test scores a candidate higher when scheduling
   it *shortens* such a live range — e.g. for a `COPY`, if the physreg's producer
   or consumer on the other end is already scheduled, schedule the copy right next
   to it. `tryGreater` keeps the higher-bias candidate. Goal: minimize
   fixed-register live ranges for the allocator.

2. **`RegExcess`** — Compares each candidate's `RPDelta.Excess`: how far scheduling
   it would push pressure past the *absolute allocatable* register ceiling
   (`VGPRExcessLimit`). Prefer the smaller overshoot. This is the hard spill
   guard — crossing the excess limit means there are no free registers left and
   the allocator must spill to scratch. Unset (and so a no-op) until pressure is
   within the 16-register lookahead of, or over, the ceiling.

3. **`RegCritical`** — Compares each candidate's `RPDelta.CriticalMax`: the
   overshoot of the *target-occupancy* budget (`getMaxNumVGPRs(TargetOccupancy)`).
   Prefer the smaller overshoot. Crossing this doesn't spill, but it drops the
   kernel below the occupancy it is compiling for — this is the occupancy guard
   that makes it the "max occupancy" scheduler. Like `RegExcess`, it is unset (a
   no-op) until a candidate would actually reach the budget.

4. **`Latency`** (early) — `tryLatency` compares the candidates' position on the
   critical path using *depth* and *height*. Top-down, a node's `depth` is the
   longest latency-weighted path from the region entry to it (its earliest
   possible issue cycle) and its `height` is the longest path from it to the
   region exit (how much work depends on it). It prefers the smaller depth when a
   node would otherwise stall (depth exceeds the latency scheduled so far),
   otherwise the greater height (further along the critical path). This *early*
   copy runs only for loops the analysis found *acyclic latency limited* — bounded
   by the critical path rather than by resources — so latency is worth chasing
   before the cheaper heuristics.

5. **`Stall`** — `getLatencyStallCycles(SU)` is 0 unless `SU` uses an *unbuffered*
   processor resource: a functional unit with no reservation station, which must
   issue exactly when ready and can't queue work to hide latency. For such an
   instruction it returns `ReadyCycle − CurrCycle` — how many cycles it would sit
   not-ready if picked now. `tryLess` prefers the candidate that stalls least, so
   an in-order/unbuffered unit isn't left idle waiting on operands. Instructions
   that don't use an unbuffered resource score 0 and are unaffected.

6. **`Cluster`** — The DAG tracks a "next cluster" neighbor: the instruction the
   clustering mutation wants scheduled adjacent to the one just placed (e.g. the
   next contiguous load). `tryGreater` prefers the candidate that *is* that
   neighbor, keeping clustered memory ops back-to-back so the backend can coalesce
   them into wider transactions. (This is exactly the edge the high-RP reschedule
   stage drops to cut pressure.)

7. **`Weak`** — `getWeakLeft(SU)` counts the node's *weak* predecessors (top) or
   successors (bottom) still unscheduled. Weak edges are soft ordering constraints
   (added by mutations such as clustering, or artificial anti-ordering) that don't
   reflect a real data dependence. `tryLess` prefers the candidate with fewer weak
   edges left dangling, so those constraints get satisfied instead of repeatedly
   deferred.

8. **`RegMax`** — The generic "don't raise the region's overall peak pressure"
   test, comparing `RPDelta.CurrentMax`.
   > **GCN note — no-op.** AMDGPU's `initCandidate` populates only the `Excess`
   > and `Critical` deltas, never `CurrentMax`, so this test never fires. The two
   > pressure guards that actually run are `RegExcess` and `RegCritical`.

9. **`ResourceReduce` / `ResourceDemand`** — Balances functional-unit usage. For
   the current zone `setPolicy` identifies the most over-subscribed processor
   resource (`ReduceResIdx`) and an under-subscribed one to steer toward
   (`DemandResIdx`); `initResourceDelta` tallies, from the instruction's schedule
   class, how many cycles it spends on each. `ResourceReduce` (`tryLess
   CritResources`) prefers the candidate that uses *less* of the bottleneck unit;
   `ResourceDemand` (`tryGreater DemandedResources`) prefers the one that uses
   *more* of the idle unit. Together they even the load across the pipeline.
   > **GCN note — near-inert on gfx906.** Under the gfx906 timing model every
   > non-MFMA write holds its functional unit for a single cycle (`ResourceCycles`
   > defaults to 1) and issue is single-wide, so a unit is free the next cycle and
   > there is almost no functional-unit contention to balance — `setPolicy` rarely
   > even flags a critical resource. This only bites for the multi-cycle MFMA unit
   > (`HWXDL`, `ResourceCycles` 2/4/8/16). (This is about *resource contention*
   > only; the latency/ILP heuristics above are unaffected and still matter.)

10. **`Latency`** (late) — The same critical-path comparison as step 4, for every
    region *not* flagged acyclic-latency-limited. Placed after the
    clustering/resource heuristics, it is the general ILP tie-break: prefer the
    node deeper on the critical path, or the one that won't stall.

11. **`NodeOrder`** — Final tie-break: when nothing above is decisive, keep the
    original program order (lower node number top-down, higher bottom-up). Makes
    the result deterministic and avoids gratuitous reordering.

(For contrast, `GCNMaxILPSchedStrategy` *does* override `tryCandidate`
([:413](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)): it keeps the `RegExcess`
spill guard but moves `Latency`/`Stall` ahead of `RegCritical`, so it chases ILP
first and only defends the occupancy budget as a lower-priority tie-break.)

## After a region: measure, bookkeep, maybe revert

`initGCNRegion` saves the incoming instruction order (`Unsched`) so the schedule
can be undone. After scheduling, `checkScheduling`
([:928](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)) records the outcome and
decides whether to keep it:

```
finalizeRegion(region):                       # finalizeGCNRegion
    if HasHighPressure:                        # a candidate hit a pressure limit while scheduling
        RegionsWithHighRP[region] = true
    checkScheduling(region)                     # keep the new schedule, or undo it

checkScheduling(region):
    pressureAfter = realRegPressure(region)

    # Case A -- the schedule fits the target-occupancy budget: keep it, done.
    if pressureAfter <= CriticalLimits:         # SGPR & VGPR under getMaxNum*GPRs(TargetOccupancy)
        record pressureAfter
        RegionsWithMinOcc[region] = (occupancy(pressureAfter) == MinOccupancy)
        return

    # Case B -- over CriticalLimits: this schedule can't hold the target occupancy.
    wavesAfter  = occupancy(pressureAfter)       # occupancy the NEW order sustains
    wavesBefore = occupancy(pressureBefore)      # occupancy the INCOMING order sustained

    # Occupancy this region will really end up at. A worse new order (wavesAfter <
    # wavesBefore) gets reverted at the bottom, so we'd realize wavesBefore; a good
    # one is kept, realizing wavesAfter. The end result is the max either way.
    newOcc = max(wavesAfter, wavesBefore)

    # MinOccupancy = the target occupancy we're currently trying to hold for the
    #   kernel (a running value, only ever ratcheted down).
    # getMinAllowedOccupancy() = the lowest MinOccupancy is *allowed* to reach:
    #   the current occupancy for an ordinary kernel (no drop permitted), or 4 for a
    #   memory-bound / wave-limited one.
    # So this fires only for a memory-bound OR wave-limited kernel: the new order
    # sits below the target (wavesAfter < MinOccupancy) but not below what's permitted
    # -- accept the drop rather than revert: fewer concurrent waves share the cache,
    # avoiding the thrashing such kernels suffer at high occupancy.
    # Taking wavesAfter lowers MinOccupancy below, which is what stops
    # the revert. For an ordinary kernel the permitted low == the target, so the two
    # conditions can't both hold and this never fires.
    if wavesAfter < wavesBefore and wavesAfter < MinOccupancy
                                and wavesAfter >= getMinAllowedOccupancy():
        newOcc = wavesAfter

    # Did this region drag the whole kernel's occupancy target down?
    if newOcc < MinOccupancy:
        MinOccupancy = newOcc
        MFI.limitOccupancy(MinOccupancy)         # keep MFI in sync (only ever lowers)
        RegionsWithMinOcc.reset()                # re-test every region against the lower bar

    if pressureAfter > AbsoluteMax:              # over getMaxNum*GPRs(MF): the allocator will spill
        RegionsWithExcessRP[region] = true
        RegionsWithHighRP[region]   = true
        RescheduleRegions[region]   = true

    # The actual keep-or-undo. Reverting restores the incoming order (wavesBefore) --
    # which is the assumption the max above was built on.
    if shouldRevertScheduling(wavesAfter):       # per-stage test: occupancy dropped and/or spills
        revert to Unsched (the original order)
    else:
        record pressureAfter
        RegionsWithMinOcc[region] = (occupancy(pressureAfter) == MinOccupancy)
```

**Two different thresholds are in play.** *CriticalLimits* is
`getMaxNum{V,S}GPRs(TargetOccupancy)` (minus a bias) — the budget to *hold* the
target occupancy; staying under it (Case A) means the schedule is fine and is
kept unconditionally. *AbsoluteMax* is `getMaxNum{V,S}GPRs(MF)` — the *hard*
register ceiling; over it there are no free registers and the allocator must
spill. Between the two is the "occupancy dropped but not spilling" zone.

**The memory-bound occupancy drop.** Normally a schedule that lowers a region's
occupancy below `MinOccupancy` is reverted (see the revert test below). The
`newOcc = wavesAfter` branch is the sole exception — it lets a region *keep* a
lower-occupancy schedule when that is expected to help. It is gated by
`getMinAllowedOccupancy()` ([SIMachineFunctionInfo.h:1086](../lib/Target/AMDGPU/SIMachineFunctionInfo.h)):

```
getMinAllowedOccupancy():
    if not memory-bound and not wave-limited: return currentOccupancy   # no drop allowed
    return min(4, currentOccupancy)                                     # may drop to 4
```

So for an ordinary function the floor equals the current occupancy and the branch
never fires — occupancy is held at the target. For a **memory-bound** or
**wave-limited** function the floor is 4, so occupancy may fall as low as 4 in
exchange for a leaner schedule.

Those two properties are set by `AMDGPUPerfHintAnalysis`, a compile-time
heuristic recorded as the `amdgpu-memory-bound` / `amdgpu-wave-limiter` function
attributes and read into MFI. The analysis gives each memory instruction a *cost*
= the number of 32-bit words it moves (`ceil(accessBits / 32)` — a static
estimate of memory *traffic*, not latency), summed into `MemInstCost` against a
total `InstCost`, then:

- **memory-bound** — a block has dense global-memory access, or `MemInstCost`
  exceeds ~50% of `InstCost` (`amdgpu-membound-threshold`).
- **wave-limited** — the same >50% test, but the numerator adds **1000×** the cost
  of two cache-hostile access kinds (on top of their normal traffic): *indirect*
  accesses, where the address is itself loaded from memory (`a[b[i]]`, pointer
  chasing — unpredictable addresses), and *large-stride* accesses, where
  consecutive accesses land far apart (`a[i+1000]` — a new cache line each time).
  The 1000× weight means their *presence*, not their volume, decides it: even a
  few trip the threshold, so a kernel whose access *pattern* thrashes the cache is
  flagged even when its raw memory volume alone wouldn't be "memory-bound"
  (`amdgpu-limit-wave-threshold`, `amdgpu-{indirect-access,large-stride}-weight`).

Rationale, from that analysis's own comments: it flags kernels that "may benefit
from limiting number of waves **to reduce cache thrashing**," noting that
"reverting optimal scheduling in favour of occupancy with basic block(s) having
dense global memory access can potentially hurt performance." For these kernels
the bottleneck is the cache / memory system, not per-wave latency: piling on
resident waves multiplies concurrent memory streams and thrashes the cache, so it
is worth trading occupancy (down to 4) for an otherwise-better schedule. Ordinary
compute-bound kernels keep their occupancy — a schedule that drops it is reverted.

**What the three per-region bitsets mean:**
- **`RegionsWithMinOcc[region]`** — is this region's (register-only) occupancy
  exactly the current kernel-wide `MinOccupancy`? i.e. is it one of the *bottleneck*
  regions. Only a bottleneck region can raise the kernel's occupancy, so the
  reschedule stages target these. Reset whenever `MinOccupancy` drops, so every
  region is re-classified against the new minimum.
- **`RegionsWithHighRP[region]`** — scheduling this region hit a pressure problem:
  a candidate reached the **critical** limit (`getMaxNumVGPRs(TargetOccupancy)` — the
  budget to *hold* the target occupancy) or the **excess** limit (`≈` the allocatable
  register count — the *spill* line), setting `HasHighPressure`; or the finished
  schedule went over the hard register ceiling. Sticky (only ever set true). Marks
  the region worth a high-pressure retry.
- **`RegionsWithExcessRP[region]`** — pressure exceeded `AbsoluteMax`: this region
  will spill.

**`RegionsWithHighRP` ⊇ `RegionsWithMinOcc`** (essentially). `MinOccupancy` only ever
falls below `TargetOccupancy`, and the critical limit is keyed to `TargetOccupancy`,
so any region *at* `MinOccupancy` has pressure at/above that critical limit — it trips
`HasHighPressure` by construction. The converse fails: a region can hit `HighRP` while
finishing *above* the min occupancy (it recovered), or by nearing the *excess* (spill)
line at any occupancy — so `HighRP` is the broad "had a pressure problem" set and
`MinOcc` the narrow "is the occupancy bottleneck" subset. (The only `MinOcc`-not-`HighRP`
slivers are the bias/`ErrorMargin` subtracted from the critical limit, and cross-stage
timing — `MinOcc` is recomputed on a drop while `HighRP` is only refreshed when a region
is actually rescheduled.)

The revert test (`shouldRevertScheduling`,
[:1164](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)) is only reached in Case B; it
differs per stage but shares two base checks: **dropped occupancy**
(`wavesAfter < MinOccupancy` ⇒ the schedule made the function worse) and
**`mayCauseSpilling`** (it sits at/below the min occupancy, didn't reduce
pressure, and the region has excess RP ⇒ it would spill more). These bitsets are
exactly what the later stages consult to decide whether they have work to do.

## The pipeline of stages

Scheduling runs as a sequence of stages; each sweeps every region. A stage may
decline to run (its `init` returns false), and may skip individual regions
(`initGCNRegion` returns false). The driver, `runSchedStages`
([:647](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)):

```
for stage in [OccInitialSchedule,
              UnclusteredHighRPReschedule,
              ClusteredLowOccupancyReschedule,
              PreRARematerialize]:
    if not stage.init():              # whole stage is a no-op unless its trigger fired
        continue
    for region in regions:            # program order
        if not stage.initRegion():    # region skipped this stage
            continue
        schedule(region)              # list scheduler, guided by tryCandidate above
        stage.finalizeRegion()        # measure; revert if worse (checkScheduling)
    stage.finalize()
```

Memory-op **clustering** matters because the stages toggle it. A DAG mutation
chains loads/stores to nearby addresses so they schedule back-to-back, letting
the backend coalesce them into wider transactions; that also lengthens live
ranges and raises pressure, which is why the high-RP retry removes it.

### 1. `OccInitialSchedule`

The real first pass. Runs unconditionally; schedules **every** region for the
structural max occupancy. This is where each region gets its baseline schedule,
where `MinOccupancy` first drops for regions whose pressure won't fit, and where
the `RegionsWith*` bitsets are populated.

Revert ([:1171](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)): keep the new order
unless it changed pressure *and* either dropped occupancy or would spill more —
i.e. never accept a first schedule that is strictly worse than the incoming one.

### 2. `UnclusteredHighRPReschedule`

Runs iff some region is high-RP or excess-RP. Per region it processes only those
that are stuck at the min occupancy (and the min actually dropped) or are
spilling ([:919](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)). For each it:

```
drop the load/store clustering mutations          # clustering inflates pressure
apply aggressive high-RP pressure biases          # tighten the effective limits
if getMaxWavesPerEU() > MinOccupancy:             # room below the attribute max
    MinOccupancy += 1                             # temporarily raise the target...
    MFI.increaseOccupancy(MinOccupancy)           # ...to push pressure down harder
reschedule the region
```

Revert ([:1184](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)) is stricter than the
base: if the reschedule didn't reduce pressure it is thrown away; if it's already
spilling it is kept as-is; otherwise a length/latency **profit** metric decides
whether the unclustered order is actually better. `finalizeStage` restores the
clustering mutations and biases, and, if occupancy rose, refreshes
`RegionsWithMinOcc`.

### 3. `ClusteredLowOccupancyReschedule`

Runs iff occupancy actually dropped (`StartingOccupancy > MinOccupancy`). It
reschedules — with clustering back **on** — the regions flagged high-RP
([:930](../lib/Target/AMDGPU/GCNSchedStrategy.cpp)), now targeting the lower
`MinOccupancy`. The point is the opposite of stage 2: occupancy is already lost,
so the register budget is looser, and clustering + the extra freedom can buy back
ILP / shorter schedules in regions that aren't the occupancy bottleneck. Revert
is the base test (don't drop occupancy, don't add spilling).

### 4. `PreRARematerialize`

Runs iff the structural max occupancy is still above `MinOccupancy` (there is
occupancy to recover), there is more than one region, and some region is at the
min. It finds **trivially rematerializable** defs (values cheaper to recompute
than to keep live), sinks them closer to their uses to shorten the live range,
and if that lifts a region's occupancy, raises `MinOccupancy` accordingly. Revert
is the base test.

## Putting it together

For a typical register-heavy kernel:

1. `OccInitialSchedule` schedules everything for the structural max; bottleneck
   regions drop `MinOccupancy` (and `MFI.Occupancy`) and are tagged high-RP /
   excess-RP.
2. `UnclusteredHighRPReschedule` retries those regions without clustering,
   trying to claw the occupancy back.
3. `ClusteredLowOccupancyReschedule` re-optimizes the regions that settled lower
   for length within their now-looser budget.
4. `PreRARematerialize` makes a last attempt to recover occupancy via remat.

The kernel's final occupancy is `MinOccupancy` — the lowest any single region had
to settle for, since one bottleneck region caps the whole kernel. That value is
written to `MFI.Occupancy` and consumed downstream (register allocation,
AsmPrinter, the code-object occupancy field).
