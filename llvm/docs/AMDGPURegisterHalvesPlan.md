# Register halves plan: correct dependences between the two halves of a register

| Step | What | Status |
|---|---|---|
| 1 | Correct the arrows after LLVM's graph build | Not started |
| 2 | Shakedown: corrected graph matches LLVM's lane-tracked graph | Not started |
| 3 | Hier and the analysis pass use the corrected graph | Not started |
| 4 | Re-mark `undef` when Hier applies a schedule | Not started |
| 5 | Instruction matching ignores `undef` | Not started |
| 6 | Measure on rtm8 | Not started |

**Parked (2026-11-18).** The expected benefit is small (see Evidence). Revisit if a
kernel shows many binding false arrows on its critical path, or if locked halves get
in the way of a hand schedule in the memory viewer. Step 5 is independent and cheap,
and can be done on its own.

## Background

**Halves.** GPU registers are 32 bits. A 64-bit value, such as a memory address, lives
in two registers side by side. LLVM names the pair one virtual register, `%360`, and
its halves `%360.sub0` (low) and `%360.sub1` (high). Wider values have more parts
(`sub0` to `sub3`, and so on). LLVM calls the parts *lanes*.

**Building a value one half at a time.** From rtm8, "base address minus 680":

```
undef %360.sub0 = V_ADD_U32 -680, %370.sub0    ; low half  = base's low half - 680
      %360.sub1 = PRED_COPY %370.sub1          ; high half = base's high half, copied
```

Neither instruction uses the other's result, so either order is correct.

**`undef`.** Writing one half normally means "change this half, keep the other one",
so the write also counts as reading the register. The first write to a fresh
register has nothing to keep, so it carries `undef`: "the other half holds nothing
yet." `undef` marks which half is written first. It moves when the halves are
reordered.

**Lane tracking.** LLVM builds dependence arrows with `buildSchedGraph`, which has a
lane-tracking switch:
- **On:** it records which lanes each instruction writes and reads, and only draws
  arrows between instructions that touch the same lanes. The two halves above get no
  arrow.
- **Off:** every write is treated as a write of the whole register.

AMDGPU's own scheduler turns it on (`GCNSubtarget::overrideSchedPolicy`,
`Policy.ShouldTrackLaneMasks = true`). Hier and the analysis pass call
`buildSchedGraph(AA)`, which leaves it off.

**Lane tracking writes to the code.** With the switch on, `ScheduleDAGInstrs::addVRegDefDeps`
clears `undef` from every half-write as it builds ("Clear undef flag, we'll re-add
it later once we know which subregister Def is first"). LLVM's scheduler re-adds the
marks for the final order as it places each instruction (`ScheduleDAGMILive::scheduleMI`,
`RegisterOperands::adjustLaneLiveness`).

## What goes wrong today

With lane tracking off, every write to `%360` gets an output arrow to the next write
to `%360`, including a write to the other half. That arrow is false: the two writes
touch different lanes. Consequences:

- **Hier can't reorder the halves.** Whichever half LLVM's scheduler left first stays
  first. The input schedule decides part of Hier's search space.
- **The viewer can't either.** The dumps' legal windows come from the analysis pass's
  graph, which has the same false arrows.
- **Floors and ideals include the false arrows**, so they can be slightly pessimistic.
- **Instruction text isn't stable across compiles.** The viewer and the order replay
  identify instructions by text, and `undef` moves when LLVM's scheduler orders the
  halves differently. rtm8's latency-80 and latency-400 dumps differ in 6 rows of
  region 1 for this reason alone, so "Load replay" refuses one against the other.

Today's setup is correct, only stricter than needed. The false arrows keep the first
half first, so `undef` stays on the right instruction even though Hier never re-marks.

## Decisions from review

- **The analysis pass stays read-only.** No graph build may write to the code. This
  rules out LLVM's lane-tracked build in normal compiles.
- **`undef` changes only when a schedule is applied.** Nothing happens at build time
  or during the search.
- **One graph-building path** for Hier and the analysis pass.
- **No edits to LLVM outside `HierarchicalScheduler/`.** Where we need LLVM logic, we
  use a local copy with a comment naming its source.

Rejected:
- **Turn lane tracking on and re-mark after every build.** Writes to the code at build
  time, including in the analysis pass.
- **Turn lane tracking on, save the marks before the build, restore them after.** The
  analysis pass still writes to the code, if only briefly.
- **Delete the false arrows and keep lane tracking off.** Unsafe. With tracking off, an
  instruction `R` that reads the low half is linked only to the *second* half-write:

  ```
  Wlo ──false──▶ Whi ──▶ R
  ```

  `R` has no arrow from `Wlo`. The false arrow is what keeps `R` below `Wlo`. Delete it,
  and `R` could be scheduled before the low half it reads has been written.

## Step 1: Correct the arrows after LLVM's graph build

A function, in a new `HierarchicalScheduler/RegisterHalves.{h,cpp}` (plus a CMake line),
that takes a region's SUnits as `buildSchedGraph(AA)` built them (lane tracking off,
read-only) and turns their register arrows into the ones lane tracking would have
drawn. It reads the code and never writes to it.

Only virtual registers written more than once in the region can have false arrows
(LLVM's own shortcut skips registers with one write, `MRI.hasOneDef`). For each such
register:

1. Remove every register arrow (data, anti, output) that names it (`SDep::getReg`).
2. Re-add them as the lane-tracked build would, by walking the region bottom-up the
   way `buildSchedGraph` does and tracking which lanes each write covers
   (`getSubRegIndexLaneMask`):
   - **Data:** from a write to each later read of lanes it wrote, unless a write in
     between covers those lanes. Latency from `computeOperandLatency`, then
     `adjustSchedDependency`, as LLVM does (this is where Hier's latency overrides
     apply).
   - **Output:** between two writes only if their lanes overlap.
   - **Anti:** from a read to a later write only if their lanes overlap.
   - A half-write without `undef` doesn't count as reading the other half. The
     lane-tracked build ignores that read too.

The faithful way to do this is a local copy of the lane-tracking branches of
`ScheduleDAGInstrs::addVRegDefDeps` and `addVRegUseDeps`, without the line that clears
`undef`, and with a comment naming them.

## Step 2: Shakedown against LLVM's lane-tracked graph

For each region, compare the corrected graph with the graph LLVM builds with lane
tracking on: the same arrows, with the same kind, register and latency.

Building LLVM's lane-tracked graph clears `undef`, so the shakedown saves the marks
before that build and restores them after. Shakedowns are opt-in validation runs
(`run_shakedowns`), so this never happens in a normal compile.

Run it on dfs_test and on the seven sandbox kernels
(`gpu2/sandbox/reorder_test_cases`, `MISCHED_EXTRA=run_shakedowns`).

## Step 3: Hier and the analysis pass use the corrected graph

Call the Step 1 function right after `buildSchedGraph(AA)` in:
- Hier: `ScheduleDAGHierarchicalScheduler::WithRegionGraph`.
- The analysis pass: `ScheduleDAGLengthAnalyzer::schedule` (dumps, floors, ideals,
  legal windows).

The malicious test scheduler's build (`RunMaliciousScheduler`) stays as it is.

Step 4 must land with this step: once Hier's graph has no false arrows, Hier may
reorder the halves.

## Step 4: Re-mark `undef` when Hier applies a schedule

At the end of `ApplyScheduleOrder`, after the moves: for each instruction in the
region, clear `undef` on its writes, then apply LLVM's rule. LLVM's rule is that a
half-write gets `undef` if, just after it, the lanes it wrote are the only live part
of the register. It lives in `RegisterOperands::collect` and `adjustLaneLiveness`.

The local copy comes from `GCNSchedStage::revertScheduling` (GCNSchedStrategy.cpp),
which does the same after its own reorder. Clearing first is required: LLVM's rule
only adds `undef`, never removes it, and after a reorder the old mark may be on the
half that's now second.

This is the only place in Hier that changes `undef`. It relies on `LiveIntervals`
being up to date for the moved instructions, which `moveInstruction` already ensures
(`LIS->handleMove`), the same sequence LLVM's scheduler uses.

Nothing else in Hier needs the marks. Hier's register tracker ignores `undef` on
writes (`GetDefMask` in NodeRegInfo.cpp).

## Step 5: Instruction matching ignores `undef`

After Step 4, a replay that reorders halves produces a dump whose texts differ from
the original's, and two compiles with different base schedules already do (rtm8 at
latency 80 and 400). Ignore `undef` on the written operands (the part of the text
before `=`) when matching:
- the viewer's `loadReplay` row matching (`viz/memory_viewer.html`);
- the order replay's text matching (`sunits_by_text` in OrderReplay.cpp), so an order
  exported from a replay's dump still matches.

`undef` on read operands (after `=`) is a different flag and stays part of the text.

## Step 6: Measure on rtm8

rtm8 region 1 has the most binding false arrows (17). Compare Hier before and after
Steps 1–4, three builds each (Hier's search has a time budget, so results vary):
region 1's length and floor, memory time and ideal, VGPRs and spills.

## How we'll check it

- **Step 2's shakedown:** the corrected graph matches LLVM's lane-tracked graph, arrow
  for arrow.
- **The analysis pass stays read-only:** MaxOccupancy's assembly is byte-identical before
  and after, on all seven sandbox kernels. The stock scheduler is untouched and the
  analysis pass must not change the code.
- **`undef` is right after a reorder:** Hier builds with `-mllvm -verify-machineinstrs`
  on the sandbox kernels.
- **The code still computes the right thing:** run rtm8 built with Hier and compare
  its output with the MaxOccupancy build's.
- **Nothing else broke:** dfs_test shakedowns.

## Evidence (2026-11-18)

Counted from MaxOccupancy dumps at latency 80, in the sandbox
(`<benchmark>/variants/MaxOccupancy/memory_viz/`). A false arrow is an output arrow
between writes to non-overlapping lanes of one register. It's *binding* when nothing
else orders the pair. Most aren't binding, because in a real 64-bit add the high half
(`V_ADDC_U32`) reads the low half's carry and has to come second anyway.

| Kernel | False arrows | Binding |
|---|---|---|
| rtm8 `rtm8` | 65 | 17 |
| hypterm `hypterm_2` / `hypterm_3` | 79 each | 1 each |
| hypterm `hypterm_1` | 12 | 1 |
| quant3MatMul (both kernels) | 12 each | 1 each |
| su3 `k_mat_nn` | 6 | 1 |
| minimod (three kernels), cobahh, hotspot | 2–20 | 0 |

rtm8's binding pairs are offset adds whose high half is a plain copy, like `%360`
above. They're cheap instructions, and a load needs both halves anyway, so the cost
is probably small. It hasn't been measured.

rtm8, region 1: at latency 80, LLVM's scheduler puts `%360.sub0` first (rows 33–34 of
the MaxOccupancy dump). At latency 400, it puts `%360.sub1` first (rows 156–157).
Hier's dumps keep the same order as their MaxOccupancy input at each latency.
