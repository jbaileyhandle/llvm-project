# MemoryGraph implementation plan: classifier, graph, ideal MLP, and scoring an ordering

| Step | What | Status |
|---|---|---|
| 0 | Verify the compiler's wait behavior | Done |
| 1 | Classify an instruction | Done |
| 2 | Build the graph | Not started |
| 3 | Ideal | Not started |
| 4 | Score one ordering | Not started |
| 5 | Commit | Not started |

## Context

This is the first step of `llvm/docs/AMDGPUMemoryGraphDesign.md` section 11.1.

The goal is two answers about a region's memory operations:
- What is the best MLP this region could reach if register limits didn't exist?
- Given one specific order of its memory operations, what MLP does that order get?

The MLP analysis pass, which comes next, is built on these. Nothing calls this code
from the scheduler yet, so compiled output doesn't change.

Decisions from review:
- **Start with a simple model, and add fidelity only when it proves necessary.** Cases we
  deliberately leave out are listed under "Left out for now".
- **The code handles every memory type. A configuration picks which to count.** For
  now it counts only vector and FLAT. Turning on scalar, LDS or GDS later is a
  configuration change, not new code.
- **FLAT is its own type**, because some FLAT operations make waits more expensive
  (Step 4). Its latency needs nothing special: LLVM already gives it the same latency
  as vector.
- **No edits to LLVM outside `HierarchicalScheduler/`.** The scheduler should plug into a
  fresh LLVM repo as easily as possible, so we use local copies with a comment naming
  their source, not moved or shared helpers.

## Files

- New: `HierarchicalScheduler/MemoryGraph.h`, `MemoryGraph.cpp`
- `HierarchicalScheduler/CMakeLists.txt`: add the `.cpp`
- `HierarchicalScheduler/ScheduleGraph.{h,cpp}`: one small test-graph factory, in the
  same style as `BuildTestDAG`
- `HierarchicalScheduler/Shakedowns.cpp`: new tests
- `llvm/docs/AMDGPUMemoryGraphDesign.md`: the design
- `llvm/docs/AMDGPUMemoryGraphImplementationPlan.md`: this plan, with each step marked
  done as it finishes
- `llvm/docs/AMDGPUWaitcntRules.md`: the compiler's full wait rules

## Sources

Where the wait behavior in this plan comes from. `AMDGPUWaitcntRules.md` has the compiler's
full rules as pseudo-code, with a reference to the pass and an experiment case for each. Code
comments and docs cite these by function or test name, since line numbers drift.

| Claim | Source | Status |
|---|---|---|
| A pending FLAT turns vmcnt and lgkmcnt waits into waits for 0 | `WaitcntBrackets::determineWait`, `hasPendingFlat` in `SIInsertWaitcnts.cpp` | **Observed**, Step 0 |
| Two FLATs with global memory info get `vmcnt(1)`; flat memory info gets 0 | LLVM test `flat_zero_waitcnt` in `test/CodeGen/AMDGPU/waitcnt.mir` | LLVM's own test; same pattern **observed**, Step 0 |
| Why: "flat memory ops may decrement VMCNT out-of-order as well as LGKMCNT out-of-order" | Commit `f0b93f1e9e64`, https://reviews.llvm.org/D46616 | Commit message only |
| Which FLATs count in which counter | `mayAccessVMEMThroughFlat`, `mayAccessLDSThroughFlat` | **Observed**, Step 0 |
| Cautious mode lasting past the first wait | `hasPendingFlat`, `applyWaitcnt` | **Observed**, Step 0 |
| Wait for 0 on a five-operation sequence | Step 0, case c02 | **Observed**, Step 0 |
| `s_barrier` sits inside regions; memory ops can't cross it, other instructions can | `SIInstrInfo::isSchedulingBoundary`, `isGlobalMemoryObject` and `buildSchedGraph` in `ScheduleDAGInstrs.cpp`; the `barrier_mix_test` kernel in `dfs_test` | Read in source; a mid-region barrier seen in the shakedown log |

"Observed" means the compiler emits this. It doesn't show the hardware requires it. No AMD
hardware documentation has been checked for any of this.

## Step 0: Verify the compiler's wait behavior (done)

Hand-ordered instruction sequences, compiled with
`llc -mcpu=gfx906 -run-pass=si-insert-waitcnts`, which runs only the pass that inserts waits,
plus two HIP kernels end to end. All 42 cases match the pass's code.

The experiment is in `/home/jbaile/gpu2/sandbox/investigations/2026_11_01_flat_waitcnt/`.

The design is in `AMDGPUMemoryGraphDesign.md`. Each step below says what to build and points
to the section that describes it, plus anything specific to building it.

## Step 1: Classify an instruction

`ClassifyMemOp` and `MemOpInfo`, as in design section 2.

**Local copy of the FLAT rules.** `mayAccessVMEMThroughFlat` and
`mayAccessLDSThroughFlat` in `SIInsertWaitcnts.cpp` decide the FLAT row. `MemoryGraph.cpp`
gets its own short copy, with a comment naming those functions, so nothing outside
`HierarchicalScheduler/` changes. The risk is that LLVM changes those functions and our copy
doesn't follow. No test catches that, so it's something to recheck when merging from upstream.

## Step 2: Build the graph

`MemoryGraph::Build`, `MemoryGraphOptions` and the node layout, as in design sections 2 and 3.

- **Testable build.** The public `Build` calls an inner step that receives each node's
  `MemOpInfo` and latency as input, so tests can use graphs with no real instructions.
- **`OrderFromSchedule`.** Takes a full schedule and returns just its memory nodes in that
  order, ending with the sink. The analysis pass needs this to score the schedule we actually
  produced.
- **Local copy of the latency settings lookup.** The `misched.txt` load-latency settings are
  applied only in `GCNSubtarget::adjustSchedDependency`. `MemoryGraph.cpp` gets a short local
  function that reads the same settings through accessors that already exist
  (`MachineInstrSchedulerConfig::GetConfig`, `getKernelLatencyOverrides`).
- **Scalar latency (not part of this work).** The permanent fix for scalar's default 5 is an
  alternate timing model in `SIScheduleJbaileCustomGfx906.td`. `SICommonWriteRes` hardcodes
  scalar's 5, so it would need a template argument the way `VMEMLatency` has one.

## Step 3: Ideal

`Ideal()` and `IdealScore`, as in design section 4.

## Step 4: Score one ordering

`ScoreOrder`, `OrderScore`, `WaitQueue` and `WaitQueues`, as in design section 5.

## Step 5: Commit

The design doc was brought up to date before implementation started. If the code makes
anything in it wrong, fix the doc in the same commit. Then ask for commit approval and push.

## Left out for now

Real compiler behavior the first model deliberately ignores. Each is only worth adding if the
evidence says it matters. `AMDGPUWaitcntRules.md` has the details of each.

| Case | What the compiler does | Effect of leaving it out | Why it can wait |
|---|---|---|---|
| FLAT stores that count in both counters | Start cautious mode. Nothing waits on a store's data, so it can last a long time, and in a loop every vector wait can become `vmcnt(0)` | MLP overstated, possibly by a lot, where these exist | We don't know how often they occur. Turning on `track_writes` models them. |
| Atomics with no return value, in any family | Count in their counter like a store; nothing waits on them | Same as other writes: out of the graph by default, so the walk's counts ignore them | Rarely changes results. Turning on `track_writes` includes them. |
| LDS or scalar waits ending cautious mode | Any `lgkmcnt(0)` counts toward ending it | Slightly pessimistic while those types are untracked | Goes away when they're turned on |
| State from earlier code | Outstanding ops and cautious mode carry across blocks and into a region | Each region starts clean, so inherited waits and cautious mode are missed | Complicated to model |
| Overwriting a register | Waits for the load that last wrote it, unless the new write is a same-kind vector load | Missed waits | Registers aren't assigned yet when this runs, so it can't be seen |
| Fences with no barrier | `ATOMIC_FENCE` becomes a wait | Missed waits | `__syncthreads` already has a barrier beside it |
| Counter limits | Waits are capped at `vmcnt(62)` and `lgkmcnt(14)` | Waits slightly too short | For vector, more than 62 ops after a load in one region seems unlikely. The lgkmcnt limit is more plausible once LDS is on. Peak outstanding is reported, so we'd notice. |
| Issue stalls when a counter is full | From memory, not verified: the next op can't issue until one finishes | In-flight count could be overstated | Same as counter limits |
| Loop preheader flush | `vmcnt(0)` at the end of the block before some loops | Missed wait | Outside the loop's own regions |
| Store to a pointer a scalar load read | `lgkmcnt(0)` | Missed wait | Only matters once scalar is on |
| `BUFFER_LOAD_LDS_*`, which load global memory straight into LDS | Counts in vmcnt. Writes no register, so later code waits on it through LDS instead: any LDS access waits on vmcnt for it | Classified as a vector write (out of the graph by default), and its LDS-based waits are missed | Only produced by specific intrinsics; not seen in our kernels |
| Function return | Waits for everything | None | Reads already count until the end of the region |
| MLP by type | Not applicable | One MLP over all tracked types (design 5.4) | Tracking one type at a time gives that type's MLP; worth a comment in the code |

## How we'll check it

**Build and run:**
```
cd /home/jbaile/gpu2/llvm_repos/llvm-project/build && ninja LLVMAMDGPUCodeGen
cd /home/jbaile/gpu2/sandbox/HierarchicalSchedulerUnitTests/dfs_test
make SCHEDULER="HierarchicalScheduler run_shakedowns" > shakedown.log 2>&1
```
The new tests print PASS or FAIL lines in `shakedown.log`, like the existing ones, and a
failure stops the suite.

**Small test graphs, with answers worked out by hand:**

| Test graph | What it checks |
|---|---|
| Load whose address comes from another load's data (pointer chasing) | The chain time adds both latencies, and the edge waits for data |
| Load, then store, then load, kept in order only | Ordering-only edge; no wait between the loads |
| Three loads, then a user of the middle one | The wait covers the first two loads, not the third |
| One vector load (100) next to a chain of three LDS loads (1 each) | Ideal `mlp` is 103 / 100 |
| Two vector loads next to a chain of four LDS loads | Ideal `mlp` is 204 / 100 |
| Loads, `s_barrier`, more loads | The barrier waits for everything before it |
| A load, `s_barrier`, then a user of the load | The user costs nothing, since the barrier already waited |
| Scalar read outstanding, then a wait on an LDS load (both on) | The wait becomes a drain |
| LDS and GDS outstanding together (both on) | The wait becomes a drain |
| The six-step cautious mode example (`AMDGPUWaitcntRules.md`, case c08) | Each wait's cost, and that it ends when F's own data is used |
| Two FLATs in a row, and a second FLAT after a drain | A new FLAT restarts cautious mode; one drain per queue ends it for all |
| A `flat_*` known to touch global memory only | Normal waits, no cautious mode |
| Same graph under different configurations | Switching a type or writes on or off adds or removes the expected nodes |
| One order, scored | `use_based_mlp`, `latency_based_mlp` and `memory_time` match the values worked out by hand |

**On the real instructions in `dfs_test`:**
- Every counter the classifier assigns is one LLVM's instruction definitions say the instruction
  uses (`SIInstrInfo::usesVM_CNT`, `usesLGKM_CNT`). This catches basic classifier mistakes, like
  an LDS op in vmcnt. It can't check the FLAT rules, because those flags are set on every `flat_*`.
- A load's latency matches the latency on its data edges in a graph built with no latency
  divisor. That guards the local copy of the settings lookup.
- Graph edges match `IsReachableInDag` for every pair of nodes.
- For the region's real order, `latency_based_mlp` never exceeds ideal's `mlp`, and
  `memory_time` is never less than ideal's.
- Turning on all types only adds nodes; it never removes any.

**Compiled output doesn't change:** compare the `dfs_test` assembly before and after,
ignoring the `.ident` line.

**Process:** edit each file at most once per turn, explained in chat first and shown as a
VS Code diff. No commit without approval at the commit point.
