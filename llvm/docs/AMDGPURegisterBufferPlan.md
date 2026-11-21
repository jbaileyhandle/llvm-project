# Register buffer plan: keep Hier a few registers under its limits

| Step | What | Status |
|---|---|---|
| 1 | `registers.vgpr_buffer` / `registers.sgpr_buffer` settings | Not started |
| 2 | Buffered limit and occupancy helpers | Not started |
| 3 | Use them wherever Hier decides | Not started |
| 4 | Test, then measure on rtm8 | Not started |

## Context

HierarchicalScheduler treats its register limits as hard bounds and can fill right
to them. On rtm8 it schedules the big block to 63 VGPRs at occupancy 4 (limit 64),
and the register allocator then spills 8. Most values live at the peak are 64- or
128-bit and need adjacent registers, so 63 values don't always fit in 64
(`gpu2/sandbox/reorder_test_cases/investigations/2026_11_18_rtm8_hier_spills/README.md`).

LLVM's own scheduler stops short of its limits for a similar reason:
`ErrorMargin = 3` in `GCNSchedStrategy.h`, subtracted from its VGPR and SGPR limits
in `GCNSchedStrategy::initialize`. Hier has no margin.

In the shortlist run (`logs/2026_11/2026_11_17/ideal_time_all4/comparison_build_scorecard.tsv`),
two kernels spill under Hier and not under MaxOccupancy: rtm8 (8 VGPRs) and sort's
`bottom_scan` (8 SGPRs).

## Decisions from review

- **Two settings, one per register file, plain numbers.** VGPR and SGPR budgets differ
  in scale, SGPR pairs must also be aligned, and spill costs differ. Default 0 for
  both, which leaves Hier's behavior unchanged.
- **New scope `registers`:** `registers.vgpr_buffer=N`, `registers.sgpr_buffer=N`. Hier
  only, like every scoped setting. The existing scopes are per pass, and the buffer
  applies to every pass.
- **The buffer lowers both of Hier's limits:** the target limit (registers allowed at
  the occupancy target) and the spill cap (registers allowed at the launch floor).
- **A buffer that depends on the live mix** (how many 64-/128-bit values are live) is
  deferred. Start with a flat number.

## Rule

Hier counts a schedule's registers as its pressure plus the buffer, wherever it
compares pressure with a limit or turns pressure into an occupancy. That is the same
as lowering every limit by the buffer, and it keeps all of Hier's comparisons
consistent with each other.

Analysis and reporting don't change. The analysis pass runs for every scheduler, and
the dumps and CSVs report the hardware's budgets and occupancy.

## Step 1: Settings

In `HierarchicalConfig.{h,cpp}`:
- A `RegisterBufferConfig { int vgpr_buffer = 0; int sgpr_buffer = 0; }` field on
  `HierarchicalConfig`.
- Accept `registers` in `RejectUnknownScopes`.
- A `BuildRegisters` that reads `vgpr_buffer` and `sgpr_buffer` with `ParseInt`, and
  fatals on a negative value or an unknown key, like the other scopes.
- Print both in `DebugPrint`.

## Step 2: Helpers

Three functions, in `GCNRegisterTracker` (which owns Hier's limits):
- Buffered VGPR limit at an occupancy: `getMaxNumVGPRs(occupancy) - vgpr_buffer`,
  never below 0.
- Buffered SGPR limit: `getMaxNumSGPRs(occupancy, /*Addressable=*/true) - sgpr_buffer`,
  never below 0.
- Buffered occupancy of a pressure: the smaller of
  `getOccupancyWithNumVGPRs(vgpr + vgpr_buffer)` and
  `getOccupancyWithNumSGPRs(sgpr + sgpr_buffer)`. This mirrors
  `GCNRegPressure::getOccupancy`.

## Step 3: Use them wherever Hier decides

| Site | What it decides |
|---|---|
| `GCNRegisterTracker::GetRegisterOnlyOccupancy` | A schedule's occupancy (gates, scores, the occupancy Hier commits) |
| `GCNRegisterTracker::IsVGPRInSpillRegime`, `IsSGPRInSpillRegime` | Spill regime at the launch floor (also used for the input schedule in `RegionInfo`) |
| `GCNRegisterTracker` target-limit functions (`IsCurVGPRCount...TargetLimit`, `GetCurVGPRCount...TargetLimit`, SGPR versions) | Over or under the occupancy target |
| `GCNRegisterTracker` spill-cap functions (`GetCurVGPRCount...SpillCap`, SGPR versions) | How far into spilling |
| `ContinuousOccupancyScorer` count-to-occupancy tables | The continuous occupancy score |
| `PartitionDag` (register-occupancy score) | BFS-DP's occupancy |
| `RegionInfo` (`original_register_only_occupancy_`) | The input schedule's occupancy, so Hier compares candidates with the input on the same terms |

Unchanged, because they report rather than decide:
- `ScheduleConstructor`'s cross-check against LLVM's own tracker (`llvm_occupancy`).
- `MemoryViz` and `ScheduleLengthAnalysis` budgets (dumps and CSVs).

Consequence: the occupancy Hier commits to the function (`SetOccupancyTarget`) is the
buffered one. With a buffer of 3, a kernel whose best schedule peaks at 62 VGPRs is
committed at occupancy 3, not 4. That's the intended meaning of the buffer.

## Step 4: Test, then measure

- **Parsing:** an unknown key and a negative value are fatal; `DebugPrint` shows the
  values.
- **Default unchanged:** dfs_test assembly identical with no `registers` lines, and the
  dfs_test shakedowns pass.
- **rtm8**, in `gpu2/sandbox/reorder_test_cases/rtm8`, Hier at latency 80 with
  `vgpr_buffer` 0, 3 and 6, two builds each: VGPRs, spills, occupancy, region 1's
  modeled peak (should be at most 64 minus the buffer), memory time and length. The
  sandbox doesn't put `MISCHED_EXTRA` in the variant's name, so give each build its
  own `VARIANT`, e.g.
  `make build SCHED=HierarchicalScheduler MISCHED_EXTRA=registers.vgpr_buffer=3 VARIANT=HierarchicalScheduler-vbuf3`.
- **Optional:** sort's `bottom_scan` with `sgpr_buffer`, through the harness (sort isn't
  in a sandbox).
