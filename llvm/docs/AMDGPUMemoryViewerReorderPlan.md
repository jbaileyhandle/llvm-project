# Memory viewer: trying a reorder

## Goal

While reading a region in the memory viewer (`HierarchicalScheduler/viz/memory_viewer.html`),
ask "what if this part of the code went there?" and get an answer at three levels:

1. The model's numbers: memory time, schedule length, register pressure, tiers.
2. The compiled code: the real `s_waitcnt`s, register allocation, spills.
3. Measured time on the GPU.

## How it works

The viewer edits the order; the real compiler scores it. Nothing is re-implemented in
JavaScript, so the viewer's answers are the compiler's.

1. You move rows in the viewer, within the range their dependences allow. The viewer exports
   the function's order to a file.
2. A `misched.txt` line names that file. After the function is scheduled as usual, its regions
   are put in the file's order, and the normal pipeline runs after that: the length analysis,
   the memory viewer's dump, register allocation, wait insertion.
3. That compile writes a new `memory_viz/<function>.json`, which opens in the viewer like any
   other dump.

The pieces for the replay exist. `ScheduleGraph::MakeConstructorForInstrOrder` schedules a list
of a region's instructions in a given order, and stops with a fatal error if the list isn't
exactly the region's instructions or puts an instruction before something it depends on.
`ScheduleDAGHierarchicalScheduler::ApplyScheduleOrder` writes an order into the function. Both
are in use today by the min_adjusted_length pass.

## Order of work

1. The `misched.txt` setting.
2. The skeleton: the code path and files, printing a line where the replay will go.
3. The replay: (a) match the file to the instructions, (b) apply the order.
4. The viewer: legal windows, moving rows, export.

The compiler side comes first because that's where the unknowns are; if the replay can't be
made exact, the viewer's editing is wasted. Step 4's legal-windows view doesn't depend on the
replay and could be pulled forward if it's wanted sooner.

Steps 1-3 are approved one edit at a time, as VS Code diffs in Manual mode. Step 4 (the viewer,
the dumper `MemoryViz.{h,cpp}`) and this doc don't need that. Each step ends with its checks,
then a commit, asked for at the commit point.

## Naming instructions across compiles

The dump is written after scheduling, so its row numbers are positions in the final order. The
compile that replays the order schedules from a different starting order, so row numbers can't
name instructions. The order file names each instruction by its MIR text, as the dump prints it.

That text is the same before and after a reorder:
- There are no waits in it. The dump is taken before register allocation; `SIInsertWaitcnts`
  runs much later. The waits the viewer shows are the model's, not instructions.
- The one order-dependent MIR flag, `killed` (last use), isn't printed at this point. Checked
  on every row of the hip_stencil `high_rp` and `dfs_test` dumps: 0 `S_WAITCNT`, 0 `killed`.
  `dead` defs do appear, but a def nothing reads is dead in any order.
- Virtual register numbers are the same in both compiles: same source, same compiler, and
  everything before the replay is unchanged.

Two instructions with identical text are matched in order: the k-th in the file is the k-th in
the region. That's always safe. If a dependence path connects them, every legal order keeps them
in the same order. If none does, they're interchangeable, since they're the same instruction.

Any mismatch is a fatal error naming the region, never a best guess. An example is an
instruction one compile has and the other doesn't, such as one created by GCN's
rematerialization stage.

## Step 1: the `misched.txt` setting

One line per order file, `order_file <path>`, alongside whatever scheduler is configured. A
relative path resolves against the compiler's working directory, as `misched.txt` itself does.

**`MachineInstrSchedulerConfig.{h,cpp}`:** the line is dispatched on its keyword, like
`kernel_latencies`, so the rest of the line is the path. It's a fatal error if the path is
empty, not readable, or listed twice. The paths are returned by an accessor and printed in the
config echo.

**`HierarchicalConfig.{h,cpp}`:** `Build` copies the paths into a typed field, printed as the
last line of its echo.

**Check:** both echoes list the paths, absolute and relative; a missing file, an empty path, or
a duplicate line stops with the fatal error.

## Step 2: the skeleton

Plugged in the way the malicious scheduler is: a setting in `misched.txt`, a call in
`ScheduleDAGHierarchicalScheduler::finalizeSchedule`, and the logic in its own files.

**When Hier's pass runs:** if any of three is wanted: the hierarchical scheduler, the malicious
scheduler, or an order file. `ScheduleDAGHierarchicalScheduler::IsPassWanted()` says whether
`misched.txt` asks for any of them, reading the generic config. When it does, `GCNPassConfig`
(`AMDGPUTargetMachine.cpp`) inserts Hier's pass right after the configured scheduler's last
pass (OptSched's when OptSched is configured, else the normal one), and the length analysis
goes after Hier's pass. So with `MaxOccupancy`, `MaxIlp` or OptSched (ACO) configured, that
scheduler runs as usual, and Hier's pass then only does what `misched.txt` asks of it. One
consequence: `MaxOccupancy` plus `malicious` used to do nothing, and now runs the malicious
scheduler after MaxOccupancy.

**`finalizeSchedule`** does each wanted part in turn:
1. The malicious scheduler if `malicious` is set, else the hierarchical scheduler if it is the
   configured scheduler.
2. Right after it, if an order file names this function, the replay (`RunOrderReplay`). In
   this step it only prints a line.

The `kernel_scheduler` filter's early return at the top stays as it is, so a function the
filter excludes isn't replayed. A function no order file names keeps the schedule of the
scheduler in use.

**New files `OrderReplay.{h,cpp}`** read the order files, enough to know which function each
names (`FindOrderFile`). That fixes the file format here, not in step 4, since this is its
first reader. It mirrors the `memory_viz` dump's fields:

```json
{"function": "_Z8dfs_testPKfPfi",
 "regions": [{"index": 3, "block": 0, "instrs": ["%2:sgpr_64 = PRED_COPY $sgpr4_sgpr5", "..."]}, ...]}
```

- `function`: the function's name, as LLVM has it (mangled).
- `regions`: each region with its index, its block number, and its instructions' texts in
  order.

**A region's index** names it across compiles. It's the order the scheduling driver
(`scheduleRegions`) visits regions in: blocks in function order, each block's regions bottom-up.
That's the dump's `index` (the length analyzer counts every region it's given), and Hier's pass
records the same number in `schedule()` (`RegionInfo::GetIndex`), before `finalizeSchedule`
sorts its regions by occupancy. Both passes get the same regions from the same driver, and
region boundaries don't move when a region is reordered. The block number and the texts are
then checked against the compiler's region, so a misnumbered region is a fatal error, not a
silent mismatch.

A file that can't be read, isn't valid JSON or has no `function` string is a fatal error, and
so are two files naming one function.

**Checks** on `dfs_test`:
- With an order file naming a function, the line prints for that function only.
- `MaxOccupancy` plus an `order_file` line gives the same assembly as plain `MaxOccupancy`.
- `HierarchicalScheduler` with no order file gives the same assembly as before this step.

## Step 3: the replay

Until the viewer exists, test files come from a throwaway script in the scratchpad: it writes
an existing `memory_viz/<function>.json` out as an order file, unedited. An edited one is that
file with two independent rows swapped by hand.

`RunOrderReplay` does what `RunMaliciousScheduler` does, with a different order. Like
`ComputeMaliciousSchedule`, `OrderReplay.{h,cpp}` returns a region's order, and the caller
builds the graph and applies it.

**3a: match only.** The compiler and the dump print instructions with one shared function,
`InstrText` (`InstrText.{h,cpp}`). For each region of a named function, `ComputeReplayOrder`:
1. Takes the order file's region with the same index. It's a fatal error if there is none, if
   its block differs, or if its texts (with their counts) differ from the region's.
   `RunOrderReplay` also checks that the file has as many regions as the function.
2. Pairs each text with the first unpaired instruction that has it, and returns the order.
   `RunOrderReplay` prints whether it equals the current order, without applying it.

**Check:** an unedited file, replayed after the same scheduler that made the dump, matches the
current order in every region. Done for MaxOccupancy on `dfs_test` and hip_stencil `high_rp`,
and Hier on `dfs_test`.

This needed a fix to `RegionInfo`: it stored the region's first instruction, which a stage that
reorders the region can move down, so a later stage saw the region with its top cut off. It now
anchors on the scheduling boundary above the region, which no scheduler moves. Before the fix,
Hier's own length pass hit this on d3q19 (7 regions one instruction short).

**Hier isn't deterministic on every kernel.** Its occupancy search has a wall-clock budget
(`occupancy.timeout_us`, 10 s by default). On hip_stencil `high_rp` region 7 (639 instructions)
it reaches occupancy 4 or 5 depending on how far it gets (20.5M to 22.1M search calls seen). A
replay compile then starts from a different order, and sets a different occupancy target, than
the dump compile did. So checks that expect identical output under Hier are run only where Hier
is deterministic (`dfs_test`); `high_rp` is checked under MaxOccupancy.

**3b: apply.** Check and measure the order with `MakeConstructorForInstrOrder`, which is fatal
if it isn't legal. Then write it with `ApplyScheduleOrder`.

**Checks:**
- Replaying an unedited export changes nothing: with the dump compile's `misched.txt` plus the
  `order_file` line, the device assembly is identical to the dump compile's. Run it for
  hip_stencil `high_rp` and `dfs_test` under MaxOccupancy, and `dfs_test` under Hier (see the
  determinism note in 3a).
- Replaying an edited file: the new dump has the moved rows where they were put.
- A hand-edited file that puts a use above its def stops with the fatal error.

## Step 4: the viewer

**Legal windows.**

*Dumper* (`MemoryViz.{h,cpp}`): each row gets its predecessors that force order, from the
`ScheduleGraph` the dumper already builds. These are the strong edges, the ones
`ScheduleConstructor` waits on before an instruction is ready. Each is written as
`{row, kind}`, with kind one of data, anti, output, barrier, may-alias, must-alias or
artificial. Edges from the region's entry node are left out.

*Viewer:*
- Click a row number to select that row. Shift-click to extend the selection to a contiguous
  range. Esc clears it.
- The selection's legal window is shaded: the rows after the last row outside the selection
  that the selection depends on, and before the first row outside it that depends on the
  selection. Moving the range as a block anywhere in that window keeps every dependence.
- The two rows that bound the window are marked, with the reason as a tooltip. A data edge
  names the register, taken from the predecessor's defs and this row's reads, for example
  "row 19 reads %86". Other edges give the kind, for example "may-alias memory".

*Check:* on hip_stencil `high_rp` region 7, the window of a vector load runs from just after
the last instruction of its address computation down to just before the first use of its
result.

**Moving rows and exporting.**
- With a selection, alt-click a row in the window to move the selection to just above that row.
  The row that bounds the window from below counts, and so does the region-end row when nothing
  bounds it, so every position in the window can be reached.
- An edited region shows its rows in the new order, each keeping its original row number, with
  moved rows marked. Every model column describes the dumped order, so they're greyed out as
  stale until the order is replayed: clocks, lanes, waits, "+N", pressure, tiers.
- Undo and redo, and reset of the region.
- "Export order" downloads `<function>.order.json` in step 2's format. Unedited regions are
  included as dumped, so a replay changes only what you moved.

*Checks:* moving a range and moving it back gives an export identical to the unedited one, and
an unedited export is identical to what step 3's script writes.

## Later: measured time

Run a replay as a `profile.py` variant to get GPU time. Use GRBM cycles, not wall time: rocprof
v1's counter pass corrupts wall times for kernels launched only a few times
(`sandbox/investigations/2026_08_05_mriq_rocprof_wall_time_bug`). First check how `profile.py`
turns a variant label into `misched.txt` contents.

## Existing code this edits

- `MachineInstrSchedulerConfig.{h,cpp}` (step 1). Created by jbaile, so no markers.
- `HierarchicalConfig.{h,cpp}` (step 1) and `ScheduleDAGHierarchicalScheduler.{h,cpp}`
  (steps 2-3).
- `AMDGPUTargetMachine.cpp` (step 2): pre-existing LLVM. The pass insertion is already inside
  a jbaile block.

## Out of scope

- Moving code across a region boundary, or past something it depends on. That changes the
  program, not its schedule. A model-only mode that drops a dependence could come later, but
  its result couldn't be compiled or run.
- Editing instructions.
- A before/after view of two dumps in the viewer. Considered and skipped as not worth it.
