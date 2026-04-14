# LLVM Machine Scheduler: An Introductory Guide (with AMDGPU Focus)

## Table of Contents

1. [The Big Picture](#1-the-big-picture)
2. [Core Data Structures](#2-core-data-structures)
3. [The Scheduler Class Hierarchy](#3-the-scheduler-class-hierarchy)
4. [The Outer Driver](#4-the-outer-driver)
5. [The Pass Layer](#5-the-pass-layer)
6. [The Strategy Pattern](#6-the-strategy-pattern)
7. [DAG Mutations](#7-dag-mutations)
8. [AMDGPU Schedulers](#8-amdgpu-schedulers)
9. [How the Schedule is Communicated Downstream](#9-how-the-schedule-is-communicated-downstream)

10. [The HierarchicalScheduler](#10-the-hierarchicalscheduler)

**Appendices:**

- [A. Memory Dependencies and Alias Analysis](#appendix-a-memory-dependencies-and-alias-analysis)
- [B. The Topological Sort](#appendix-b-the-topological-sort)
- [C. Weak vs Strong Edges and Clustering](#appendix-c-weak-vs-strong-edges-and-clustering)
- [D. The SILoadStoreOptimizer](#appendix-d-the-siloadstoreoptimizer)
- [E. Pre-RA vs Post-RA Scheduling](#appendix-e-pre-ra-vs-post-ra-scheduling)
- [F. OptSched's Register Pressure Tracking (BBWithSpill)](#appendix-f-optscheds-register-pressure-tracking-bbwithspill)
- [G. Manual Scheduling Hints (SCHED_BARRIER)](#appendix-g-manual-scheduling-hints)
- [H. Asynchronous Memory and s_waitcnt](#appendix-h-asynchronous-memory-and-s_waitcnt)
- [I. Memory Clauses](#appendix-i-memory-clauses)
- [J. Post-Scheduling Passes](#appendix-j-post-scheduling-passes)
- [K. The -misched Registry](#appendix-k-the--misched-registry)
- [L. SIScheduleDAGMI (Older Hierarchical Scheduler)](#appendix-l-sischeduledag-mi-older-hierarchical-scheduler)
- [M. Instruction Latency, Throughput, and Schedule Length](#appendix-m-instruction-latency-throughput-and-schedule-length)
- [N. Occupancy](#appendix-n-occupancy)

---

## 1. The Big Picture

A **MachineFunction** (one compiled function) contains **MachineBasicBlocks**, each
containing a linear sequence of **MachineInstrs**. The scheduler's job is to
**reorder the MachineInstrs within each scheduling region** to minimize pipeline
stalls, reduce register pressure, and improve overall performance — without
breaking correctness.

- **Input:** A region of MachineInstrs in their original order.
- **Output:** The same MachineInstrs, physically reordered in the MachineBasicBlock.

The scheduler does **not** create or delete instructions — it only **moves** them.
A scheduling region is always contained within a single basic block; it never
crosses basic block boundaries.

---

## 2. Core Data Structures

### 2.1 SUnit (Scheduling Unit)

An `SUnit` is a **node** in the scheduling DAG. Each `SUnit` wraps one
`MachineInstr`. Defined in `include/llvm/CodeGen/ScheduleDAG.h:242`.

Key fields:

```
class SUnit:
    Instr: MachineInstr*           // the actual instruction this node represents
    NodeNum: unsigned              // index into the SUnits vector

    Preds: vector<SDep>            // incoming edges (things I depend on)
    Succs: vector<SDep>            // outgoing edges (things that depend on me)

    Latency: unsigned              // cycles this instruction takes to produce its result

    NumPredsLeft: unsigned         // strong predecessors not yet scheduled
    NumSuccsLeft: unsigned         // strong successors not yet scheduled
    WeakPredsLeft: unsigned        // weak predecessors not yet scheduled
    WeakSuccsLeft: unsigned        // weak successors not yet scheduled

    isScheduled: bool              // has this node been placed in the schedule?
    TopReadyCycle: unsigned        // earliest cycle this can be scheduled (top-down)
    BotReadyCycle: unsigned        // earliest cycle this can be scheduled (bottom-up)

    getDepth()                     // longest path from any root to this node
    getHeight()                    // longest path from this node to any leaf
```

An SUnit becomes **ready** to schedule when `NumPredsLeft == 0` (top-down) or
`NumSuccsLeft == 0` (bottom-up). Weak dependencies (`WeakPredsLeft` /
`WeakSuccsLeft`) are tracked separately and do **not** affect readiness (see
[Appendix C](#appendix-c-weak-vs-strong-edges-and-clustering)).

`NumPredsLeft` is initialized during `buildSchedGraph` — each call to
`SUnit::addPred` increments it for strong edges. By the time the DAG is built,
`NumPredsLeft` reflects the total number of strong predecessors.

In addition to the `SUnits` vector, `ScheduleDAG` has two special boundary
nodes:

- **`EntrySU`** — represents "everything before the region." Instructions that
  use values defined before the region have `EntrySU` as a predecessor.
- **`ExitSU`** — represents "everything after the region." Instructions whose
  results are live-out have `ExitSU` as a successor. Also, instructions with
  latency but no in-region users get an artificial edge to `ExitSU` to model
  the latency of their results being consumed later.

These are **not** in the `SUnits` vector — they are separate members of
`ScheduleDAG`. They have edges (they participate in the DAG) but should never
be scheduled. `isBoundaryNode()` returns true for both.

### 2.2 SDep (Scheduling Dependency)

An `SDep` is a **directed edge** in the DAG. Defined in
`include/llvm/CodeGen/ScheduleDAG.h:49`. Each edge connects two SUnits and
has:

```
class SDep:
    Dep: SUnit*                    // the other node
    Kind: enum { Data, Anti, Output, Order }
    Latency: unsigned              // minimum cycles between the two instructions

    // For register deps (Data/Anti/Output):
    Reg: unsigned                  // which register creates this dependency

    // For Order deps:
    OrderKind: enum { Barrier, MayAliasMem, MustAliasMem,
                      Artificial, Weak, Cluster }
```

`Kind` and `OrderKind` are two separate fields. `Kind` is always one of the four
top-level types. When `Kind == Order`, then `OrderKind` further specifies *why*
the ordering constraint exists.

The four `Kind` values:

| Kind       | Meaning                            | Example                                         | Typical Latency |
|------------|------------------------------------|-------------------------------------------------|-----------------|
| **Data**   | True dependence (RAW). B reads a register that A writes. | `A: v1 = ADD ...` → `B: ... = MUL v1, ...` | > 0 (production latency) |
| **Anti**   | Anti-dependence (WAR). B writes a register that A reads. | `A: ... = MUL v1, ...` → `B: v1 = ADD ...` | 0               |
| **Output** | Output dependence (WAW). Both write the same register.   | `A: v1 = ADD ...` → `B: v1 = SUB ...`       | 0               |
| **Order**  | Non-register ordering constraint (memory, barriers, etc.) | `STORE [x]` → `LOAD [x]`                   | 0 or 1          |

The `OrderKind` sub-types fall into two categories:

- **Strong** (must be respected): `Barrier`, `MayAliasMem`, `MustAliasMem`, `Artificial`
- **Weak** (hints that the strategy may ignore): `Weak`, `Cluster`

See [Appendix C](#appendix-c-weak-vs-strong-edges-and-clustering) for details on
the weak/strong distinction.

---

## 3. The Scheduler Class Hierarchy

```
ScheduleDAG                                     (ScheduleDAG.h:554)
│   Base class. No parent. Owns the SUnits vector,
│   holds references to TM, TII, TRI, MF, MRI.
│
└─ ScheduleDAGInstrs                            (ScheduleDAGInstrs.h:120)
    │   Builds the dependency DAG from MachineInstrs in a region.
    │   Adds register and memory dependency edges.
    │
    ├─ ScheduleDAGMI                            (MachineScheduler.h:276)
    │   │   The scheduling loop driver. Owns a MachineSchedStrategy*
    │   │   (via composition). Calls pickNode() each iteration.
    │   │
    │   └─ ScheduleDAGMILive                    (MachineScheduler.h:398)
    │       │   Adds live register pressure tracking.
    │       │
    │       ├─ GCNIterativeScheduler             (GCNIterativeScheduler.h:29)
    │       │       AMDGPU-specific. Iteratively reschedules regions.
    │       │
    │       └─ ScheduleDAGOptSched               (OptSched/.../OptimizingScheduler.h:110)
    │               OptSched wrapper. Overrides schedule() to run
    │               ACO / Enumerator internally.
    │
    ├─ SchedulePostRATDList                     (PostRASchedulerList.cpp:110)
    ├─ SwingSchedulerDAG                        (MachinePipeliner.h:114)
    └─ DefaultVLIWScheduler                     (DFAPacketizer.h:51)
```

### 3.1 Level 1: ScheduleDAG — The Abstract Graph Container

**File:** `include/llvm/CodeGen/ScheduleDAG.h:554`

The root class. It knows nothing about MachineInstrs or scheduling regions. It
just holds the graph infrastructure:

```
class ScheduleDAG:
    members:
        TM, TII, TRI, MRI         // target info (what machine are we on?)
        SUnits: vector<SUnit>      // the nodes of the DAG
        EntrySU, ExitSU            // special boundary nodes

    methods:
        clearDAG()                 // wipe all SUnits between regions
        viewGraph()                // dump to GraphViz for debugging
        dumpNode(), dump()         // text debug output
```

### 3.2 Level 2: ScheduleDAGInstrs — Building the DAG from MachineInstrs

**File:** `include/llvm/CodeGen/ScheduleDAGInstrs.h:120`

This is where the DAG gets **built**. It knows about MachineInstrs, registers,
and memory. Key additions over `ScheduleDAG`:

```
class ScheduleDAGInstrs extends ScheduleDAG:
    members:
        BB: MachineBasicBlock*
        RegionBegin, RegionEnd     // iterators bounding the current region
        NumRegionInstrs: unsigned
        MISUnitMap: map<MachineInstr* → SUnit*>
        SchedModel: TargetSchedModel    // knows instruction latencies
        AAForDep: AAResults*            // alias analysis for memory deps
        BarrierChain: SUnit*            // current memory barrier
        Topo: ScheduleDAGTopologicalSort  // maintains topological ordering

    key methods:
        enterRegion(MBB, begin, end, N)   // set up for a new region
        exitRegion()                      // clean up
        buildSchedGraph(AA)               // THE DAG BUILDER
        schedule() = 0                    // pure virtual — subclasses decide ordering
```

#### buildSchedGraph — The DAG Builder

This is the core of Level 2. It walks the region **bottom-to-top** and creates
SUnits and SDep edges:

```
buildSchedGraph(AA):
    clearDAG()

    // Step 1: Create one SUnit per MachineInstr
    for each MachineInstr MI in [RegionBegin, RegionEnd):
        SU = new SUnit(MI)
        MISUnitMap[MI] = SU
        SU.Latency = SchedModel.computeInstrLatency(MI)

    // Step 2: Walk bottom-to-top, adding edges
    // Track which SUnit last defined/used each register
    Defs: map<PhysReg → list<SUnit>>
    Uses: map<PhysReg → list<SUnit>>
    Stores: map<MemoryValue → list<SUnit>>
    Loads:  map<MemoryValue → list<SUnit>>

    for each MI from RegionEnd-1 down to RegionBegin:
        SU = MISUnitMap[MI]

        // --- Register dependencies ---
        for each register DEF in MI.operands:
            // Data dep (RAW): SU defines Reg, later SU2 uses Reg
            for each SU2 in Uses[Reg]:
                SU.addSucc(SDep(SU2, Data, Reg))

            // Output dep (WAW): SU defines Reg, later SU2 also defines Reg
            for each SU2 in Defs[Reg]:
                SU.addSucc(SDep(SU2, Output, Reg))

            Defs[Reg] = {SU}

        for each register USE in MI.operands:
            // Anti dep (WAR): SU uses Reg, later SU2 defines Reg
            for each SU2 in Defs[Reg]:
                SU.addSucc(SDep(SU2, Anti, Reg))

            Uses[Reg].add(SU)

        // --- Memory dependencies ---
        if MI is a Store:
            // RAW: later loads from same address must see this store's value
            for each SU2 in Loads that may alias MI:
                SU.addSucc(SDep(SU2, Order, MayAliasMem))

            // WAW: later stores to same address must follow this one
            for each SU2 in Stores that may alias MI:
                SU.addSucc(SDep(SU2, Order, MayAliasMem))

            Stores[MI.memoryValue].add(SU)

        if MI is a Load:
            // WAR: this load must come before later stores to same address
            for each SU2 in Stores that may alias MI:
                SU.addSucc(SDep(SU2, Order, MayAliasMem))

            Loads[MI.memoryValue].add(SU)

        // --- Barrier instructions (calls, inline asm, etc.) ---
        if MI is a global memory object (scheduling barrier):
            if BarrierChain:
                BarrierChain.addPredBarrier(SU)
            BarrierChain = SU
            addBarrierChain(Stores)     // edge from barrier → all stores below
            addBarrierChain(Loads)      // edge from barrier → all loads below
```

Note that "later" here means later in program order (below in the basic block),
which corresponds to instructions we have already seen in our bottom-to-top walk.

For memory dependencies, the "may alias" check is where **alias analysis** comes
in. If alias analysis can prove two memory operations access different addresses,
no edge is added. See [Appendix A](#appendix-a-memory-dependencies-and-alias-analysis).

**Barrier instructions** (calls, inline asm, instructions with unmodeled side
effects) use normal `SDep::Barrier` (an Order edge) — not a special mechanism
outside the DAG. The barrier gets edges to/from all memory operations, preventing
any from being reordered across it. Register dependencies for barrier
instructions (e.g., a call clobbering caller-saved registers) are handled
through the normal register dependency mechanism.

### 3.3 Level 3: ScheduleDAGMI — The Scheduling Loop

**File:** `include/llvm/CodeGen/MachineScheduler.h:276`

This adds the actual **scheduling algorithm driver**. Key additions:

```
class ScheduleDAGMI extends ScheduleDAGInstrs:
    members:
        SchedImpl: unique_ptr<MachineSchedStrategy>  // THE STRATEGY (composition)
        Mutations: vector<ScheduleDAGMutation>       // post-build DAG transforms
        CurrentTop: iterator      // top of unscheduled zone (grows downward)
        CurrentBottom: iterator   // bottom of unscheduled zone (grows upward)
```

#### The schedule() method

```
schedule():
    // Step 1: Build the dependency DAG (calls Level 2)
    buildSchedGraph(AA)

    // Step 2: Apply mutations (e.g., load clustering, macro fusion)
    for each mutation in Mutations:
        mutation.apply(this)    // may add/remove/modify edges

    // Step 3: Find root nodes
    TopRoots = nodes with no predecessors   // ready to schedule top-down
    BotRoots = nodes with no successors     // ready to schedule bottom-up

    // Step 4: Initialize the strategy
    SchedImpl.initialize(this)
    initQueues(TopRoots, BotRoots)

    // Step 5: THE MAIN LOOP
    //
    //   CurrentTop -----> [scheduled top-down instructions]
    //                     [  unscheduled zone shrinks...  ]
    //   CurrentBottom --> [scheduled bottom-up instructions]
    //
    while there are unscheduled nodes:
        (SU, IsTopNode) = SchedImpl.pickNode()
        if SU is null: break

        MI = SU.getInstr()
        if IsTopNode:
            moveInstruction(MI, CurrentTop)
            advance CurrentTop downward
        else:
            moveInstruction(MI, CurrentBottom)
            advance CurrentBottom upward

        SchedImpl.schedNode(SU, IsTopNode)     // notify strategy
        updateQueues(SU, IsTopNode)            // release newly ready nodes

    // CurrentTop == CurrentBottom → unscheduled zone is empty, done!
    placeDebugValues()
```

The key insight: **`ScheduleDAGMI` does not decide which instruction to pick**.
It asks `SchedImpl->pickNode()` each iteration. The strategy returns both which
SUnit to schedule and whether to place it at the top or bottom. This is where
different scheduling strategies plug in (see
[Section 6](#6-the-strategy-pattern)).

The `moveInstruction` call **physically moves** the MachineInstr within the
MachineBasicBlock's instruction list. By the time `schedule()` returns, the
instructions have been reordered in-place.

#### The release mechanism

An SUnit is **released** when it becomes **ready to schedule** — meaning all of
its strong dependencies have been satisfied. The scheduler maintains a **ready
queue** of released nodes, and `pickNode()` can only choose from that queue.

```
// Before the loop starts:
initQueues(TopRoots, BotRoots):
    for each root node SU (NumPredsLeft == 0):
        SchedImpl->releaseTopNode(SU)    // adds SU to the ready queue

// Inside updateQueues, after scheduling SU:
releaseSuccessors(SU):
    for each outgoing edge (SU → SuccSU):
        releaseSucc(SU, edge):
            if edge.isWeak():
                --SuccSU->WeakPredsLeft
                if edge.isCluster():
                    NextClusterSucc = SuccSU    // hint for the strategy
                return    // does NOT touch NumPredsLeft

            // Strong edge:
            --SuccSU->NumPredsLeft
            if SuccSU->NumPredsLeft == 0:
                SchedImpl->releaseTopNode(SuccSU)    // now ready
```

Concrete example:

```
DAG:     A ──Data──→ C
         B ──Data──→ C

C has NumPredsLeft = 2
A and B have NumPredsLeft = 0 (roots, immediately released)
```

1. Ready queue: `{A, B}`. Strategy picks A.
2. Release A's successors: `C.NumPredsLeft` = 2 → 1. Not ready yet.
3. Ready queue: `{B}`. Strategy picks B.
4. Release B's successors: `C.NumPredsLeft` = 1 → 0. C is released.
5. Ready queue: `{C}`. Strategy picks C.

#### Cycle and slot modeling

The scheduler doesn't just decide an order — it also models a **cycle-level
pipeline simulation** internally. Each SUnit tracks `TopReadyCycle` /
`BotReadyCycle` (the earliest cycle it can be scheduled, respecting latencies),
and the strategy uses these to avoid scheduling instructions before their
operands are ready.

The target's `MCSchedModel::IssueWidth` defines how many instructions can issue
per cycle (the "issue rate"). Multiple instructions may be assigned to the same
cycle in different "slots."

This simulation is used **only during scheduling** to make good decisions. The
cycle/slot assignments are **not** passed downstream — only the final instruction
order matters (see [Section 9](#9-how-the-schedule-is-communicated-downstream)).

### 3.4 Level 4: ScheduleDAGMILive — Register Pressure Tracking

**File:** `include/llvm/CodeGen/MachineScheduler.h:398`

This adds **live register pressure tracking** on top of the scheduling loop:

```
class ScheduleDAGMILive extends ScheduleDAGMI:
    members:
        RegClassInfo: RegisterClassInfo*
        ShouldTrackPressure: bool
        RPTracker: RegPressureTracker          // current pressure state
        TopRPTracker: RegPressureTracker       // pressure at top boundary
        BotRPTracker: RegPressureTracker       // pressure at bottom boundary
        SUPressureDiffs: PressureDiffs         // per-SUnit pressure change
        RegionCriticalPSets: vector<PressureChange>  // which pressure sets are critical

    // Provides queries for strategies:
    getRegPressure()              // current region pressure
    getTopPressure()              // pressure if we schedule next at top
    getBotPressure()              // pressure if we schedule next at bottom
    getRegionCriticalPSets()      // which register classes are under pressure
```

This is what lets strategies like `GCNMaxOccupancySchedStrategy` make
pressure-aware decisions. The strategy can ask: "if I schedule instruction X
next at the top, what will the register pressure be?" and compare that against
the occupancy target.

**Why this matters for GPUs:** On AMDGPU, more live registers → fewer wavefronts
can run simultaneously (lower occupancy). The strategy tries to find an
instruction order that keeps register pressure below the threshold needed for the
target occupancy level.

LLVM's `RegPressureTracker` is sophisticated — it handles lane masks,
sub-register tracking, overlapping pressure sets, and ties into `LiveIntervals`.
This accuracy comes at a cost: it's designed for incremental forward/backward
tracking and does **not** support arbitrary backtracking (see
[Appendix F](#appendix-f-optscheds-register-pressure-tracking-bbwithspill)).

---

## 4. The Outer Driver

Before any DAG class is involved, there is a **pass** method
(`MachineSchedulerBase::scheduleRegions`) that drives everything. It splits each
basic block into scheduling regions and calls `schedule()` on each.

```
scheduleRegions(Scheduler, FixKillFlags):
    for each MachineBasicBlock in MachineFunction:
        Scheduler.startBlock(MBB)

        // Split the block at "scheduling boundaries" (calls, barriers, etc.)
        regions = getSchedRegions(MBB)

        for each region [RegionBegin, RegionEnd):
            Scheduler.enterRegion(MBB, RegionBegin, RegionEnd, numInstrs)

            if region has < 2 instructions:
                Scheduler.exitRegion()     // nothing to reorder
                continue

            Scheduler.schedule()           // <--- THE MAIN ENTRY POINT
            Scheduler.exitRegion()

        Scheduler.finishBlock()

    Scheduler.finalizeSchedule()   // called once after all blocks
```

### What is enterRegion?

`enterRegion` is simple setup — it saves the region boundaries so that
`buildSchedGraph` and `schedule()` know which instructions to work with:

```
ScheduleDAGInstrs::enterRegion(MBB, begin, end, regioninstrs):
    RegionBegin = begin
    RegionEnd = end
    NumRegionInstrs = regioninstrs

ScheduleDAGMI::enterRegion(MBB, begin, end, regioninstrs):
    ScheduleDAGInstrs::enterRegion(...)          // save boundaries
    SchedImpl->initPolicy(begin, end, regioninstrs)  // let strategy configure itself
```

### What is finalizeSchedule?

`finalizeSchedule()` is called once after all regions in all blocks have been
processed. The default implementation is a no-op. `ScheduleDAGOptSched` overrides
it for two-pass scheduling (see [Section 8.2](#82-scheduledag-optsched)).

### What defines a scheduling boundary?

Scheduling boundaries are instructions that the scheduler cannot reorder past.
These include calls, inline assembly, instructions with unmodeled side effects,
and terminators (branches). The function `isSchedBoundary()` determines this.
A basic block may contain multiple scheduling regions separated by boundaries.

---

## 5. The Pass Layer

Above the outer driver, there are **MachineFunctionPass** subclasses that
integrate scheduling into LLVM's pass pipeline.

### 5.1 The three scheduling passes

```
MachineSchedulerBase : public MachineSchedContext, public MachineFunctionPass
├─ MachineScheduler           // pre-RA scheduling
├─ MachineSchedulerOptSched   // pre-RA scheduling (OptSched, AMDGPU only)
└─ PostMachineScheduler       // post-RA scheduling
```

| Pass                        | When it runs                              | What it creates               |
|-----------------------------|-------------------------------------------|-------------------------------|
| `MachineScheduler`          | Pre-RA (after coalescing, before regalloc) | `ScheduleDAGMILive` + strategy |
| `MachineSchedulerOptSched`  | Pre-RA (AMDGPU only, after `MachineScheduler`) | `ScheduleDAGOptSched`       |
| `PostMachineScheduler`      | Post-RA (after regalloc, near code emission)  | `ScheduleDAGMI` + strategy  |

Each pass's `runOnMachineFunction` follows the same pattern:

```
runOnMachineFunction(MF):
    1. Gather analysis results (LiveIntervals, AliasAnalysis, LoopInfo, etc.)
    2. Scheduler = createMachineScheduler()    // factory method
    3. scheduleRegions(*Scheduler, FixKillFlags)   // the outer driver
    4. Verify the MachineFunction is still valid
```

### 5.2 MachineSchedContext

`MachineSchedulerBase` inherits from `MachineSchedContext`, which is just a bag
of pointers passed down to the `ScheduleDAGInstrs` constructor:

```
MachineSchedContext:
    MF*              // the function being compiled
    MLI*             // loop info
    MDT*             // dominator tree
    PassConfig*      // target pass configuration
    AA*              // alias analysis
    LIS*             // live intervals
    RegClassInfo*    // register class info
```

### 5.3 Analysis dependencies

The scheduling passes declare what analyses they need:

- **LiveIntervals** — needed by `ScheduleDAGMILive` for pressure tracking
- **AliasAnalysis** — needed by `buildSchedGraph` for memory dependency precision
- **MachineLoopInfo** — used for loop-aware heuristics
- **SlotIndexes** — the mapping between MachineInstrs and their positions in
  LiveIntervals

The scheduler **preserves** these analyses, meaning the reordering must be
consistent with the live interval data.

### 5.4 How the scheduler is chosen

The factory method `createMachineScheduler()` determines which
`ScheduleDAGInstrs` subclass and which strategy to use. The target can override
this via `TargetPassConfig::createMachineScheduler()`. For AMDGPU, this returns a
`ScheduleDAGMILive` with `GCNMaxOccupancySchedStrategy`.

The `-misched=` command-line option can override the target's default choice.
This flag only affects the `MachineScheduler` pass — it has no effect on
`MachineSchedulerOptSched` or `PostMachineScheduler`.

### 5.5 The AMDGPU pass pipeline

#### Without OptSched:

```
MachineScheduler                    (GCNMaxOccupancySchedStrategy)
    ↓
SIWholeQuadMode                     (inserts exec manipulation instructions)
SIPreAllocateWWMRegs
SIOptimizeExecMaskingPreRA
SIFormMemoryClauses
    ↓
Register Allocation
    ↓
PostMachineScheduler                (PostGenericScheduler)
    ↓
Code Emission
```

#### With OptSched:

```
MachineScheduler                    (GCNMaxOccupancySchedStrategy — AMD's default)
    ↓
MachineSchedulerOptSched            (ScheduleDAGOptSched — ACO / Enumerator)
    ↓
SIWholeQuadMode                     (inserted after MachineSchedulerOptSchedID)
SIPreAllocateWWMRegs
SIOptimizeExecMaskingPreRA
SIFormMemoryClauses
    ↓
Register Allocation
    ↓
PostMachineScheduler                (PostGenericScheduler — same either way)
    ↓
Code Emission
```

The key difference: with OptSched, there are **two** pre-RA scheduling passes.
AMD's default runs first, then OptSched potentially improves on it. The
SIWholeQuadMode and related passes are inserted **after** OptSched because they
introduce exec manipulation instructions that act as scheduling barriers — they
must run after all scheduling is complete.

Post-RA scheduling is the same regardless of whether OptSched is enabled.

See [Appendix E](#appendix-e-pre-ra-vs-post-ra-scheduling) for more on why
both pre-RA and post-RA scheduling exist.

---

## 6. The Strategy Pattern

`ScheduleDAGMI` uses the **Strategy design pattern**. It holds a pointer to a
`MachineSchedStrategy` and delegates the "which instruction next?" decision to
it.

```
class MachineSchedStrategy:                     (MachineScheduler.h:211)
    // Lifecycle:
    initPolicy(begin, end, numInstrs)           // configure per-region
    initialize(DAG)                             // called after DAG is built
    registerRoots()                             // after roots are found

    // The core interface:
    pickNode(IsTopNode) → SUnit*                // which instruction next?
    schedNode(SU, IsTopNode)                    // notification: SU was scheduled
    releaseTopNode(SU)                          // SU is now ready (top-down)
    releaseBottomNode(SU)                       // SU is now ready (bottom-up)

    // Optional:
    shouldTrackPressure() → bool
    shouldTrackLaneMasks() → bool
```

### 6.1 The inheritance chain for strategies

```
MachineSchedStrategy                             (MachineScheduler.h:211)
  └─ GenericSchedulerBase                        (MachineScheduler.h:1066)
      └─ GenericScheduler                        (MachineScheduler.h:1209)
          └─ GCNSchedStrategy                    (GCNSchedStrategy.h:42)
              └─ GCNMaxOccupancySchedStrategy    (GCNSchedStrategy.h:123)
```

### 6.2 Strategy vs DAG scheduler

This is a composition relationship, not inheritance:

- The **DAG scheduler** (`ScheduleDAGMILive`) drives the overall scheduling
  loop: building the dependency graph, managing the ready queue, physically
  moving instructions.
- The **strategy** (`MachineSchedStrategy`) is a policy object that answers
  "given these ready instructions, which one should I pick next?"

`ScheduleDAGMI` holds a `std::unique_ptr<MachineSchedStrategy> SchedImpl` member.
You can swap in different strategies without changing the DAG scheduler.

---

## 7. DAG Mutations

Mutations are **post-processing steps** applied to the DAG after
`buildSchedGraph` but before the strategy's `initialize()`. They can add, remove,
or modify edges.

```
schedule():
    buildSchedGraph(AA)       // build the DAG (Level 2)
    postProcessDAG():         // apply mutations
        for each mutation in Mutations:
            mutation.apply(this)
    SchedImpl.initialize(this)   // then initialize strategy
    ...
```

### 7.1 Mutations used by GCNMaxOccupancySchedStrategy

| Mutation                          | Description |
|-----------------------------------|-------------|
| `LoadClusterDAGMutation`          | Adds weak Cluster edges between loads to nearby addresses, enabling the downstream `SILoadStoreOptimizer` to merge them into wider memory operations. (Generic LLVM code in `MachineScheduler.cpp`.) |
| `StoreClusterDAGMutation`         | Same as above but for stores. Only active on GFX11+ (`shouldClusterStores()` returns false on older chips like gfx906). |
| `IGroupLPDAGMutation`            | Processes `SCHED_BARRIER`, `SCHED_GROUP_BARRIER`, and `IGLP_OPT` pseudo-instructions that a programmer may insert into kernel source code (via `__builtin_amdgcn_sched_barrier()` etc.). Adds strong Artificial edges to enforce manually-specified scheduling constraints. See [Appendix G](#appendix-g-manual-scheduling-hints). |
| `AMDGPUMacroFusionDAGMutation`   | Clusters instructions that define VCC (the condition code register) with instructions that consume VCC as `src2` (e.g., `V_ADDC_U32`, `V_CNDMASK_B32`). Keeping them adjacent allows the instruction to be shrunk to a shorter VOP2 encoding. (`AMDGPUMacroFusion.cpp`.) |
| `AMDGPUExportClusteringDAGMutation` | Clusters pixel/vertex shader export instructions together, with position exports ordered first. Removes unnecessary barrier edges between exports, then adds strong Barrier + weak Cluster edges to keep them adjacent. Only relevant for graphics shaders, not compute kernels. (`AMDGPUExportClustering.cpp`.) |

### 7.2 Mutations used by ScheduleDAGOptSched

| Mutation                          | Present |
|-----------------------------------|---------|
| `LoadClusterDAGMutation`          | Yes     |
| `StoreClusterDAGMutation`         | No      |
| `IGroupLPDAGMutation`            | No      |
| `AMDGPUMacroFusionDAGMutation`   | Yes     |
| `AMDGPUExportClusteringDAGMutation` | Yes  |

OptSched gets fewer mutations. Notably `IGroupLPDAGMutation` is absent — likely
because OptSched's own algorithms handle instruction grouping differently and the
extra artificial edges would constrain its search space.

### 7.3 Post-RA mutations

| Mutation                          | Description |
|-----------------------------------|-------------|
| `LoadClusterDAGMutation`          | Same as pre-RA |
| `StoreClusterDAGMutation`         | Conditional |
| `FillMFMAShadowMutation`         | Fills the shadow of MFMA (matrix multiply) instructions with other work |
| `IGroupLPDAGMutation`            | Same as pre-RA |
| `VOPDPairingMutation`            | Pairs VALU instructions for dual-issue on supported hardware |

### 7.4 How clustering works

See [Appendix C](#appendix-c-weak-vs-strong-edges-and-clustering) for a detailed
explanation of the clustering mechanism and the weak/strong edge distinction.

---

## 8. AMDGPU Schedulers

### 8.1 GCNMaxOccupancySchedStrategy

This is the default pre-RA scheduling strategy for AMDGPU. It inherits from
`GCNSchedStrategy` → `GenericScheduler` → `GenericSchedulerBase` →
`MachineSchedStrategy`.

It plugs into the standard `ScheduleDAGMILive` scheduling loop via the strategy
pattern — it provides `pickNode()` and the DAG scheduler drives everything else.

The strategy uses `ScheduleDAGMILive`'s pressure tracking to make
occupancy-aware decisions. It may reschedule regions multiple times through
different stages (e.g., `OccInitialSchedule`,
`UnclusteredHighRPReschedule`, `ClusteredLowOccupancyReschedule`) to find a
good balance between register pressure and ILP.

### 8.2 ScheduleDAGOptSched

OptSched takes a fundamentally different approach. Instead of providing a
strategy, it **subclasses `ScheduleDAGMILive`** and overrides `schedule()` to
replace the entire scheduling loop. It overrides two inherited virtual methods:

1. **`schedule()`** — the per-region entry point
2. **`finalizeSchedule()`** — called once after all regions (used for two-pass
   scheduling)

#### Single-pass flow

```
schedule():
    1. Optionally run LLVM's normal schedule first
       (ScheduleDAGMILive::schedule()) to get a baseline heuristic order

    2. Build OptSched's internal representation:
       - Convert LLVM's SUnits/SDeps → OptSched's DataDepGraph (DDG)
       - Convert register files
       - Apply OptSched's own graph transformations

    3. Create a BBWithSpill region (OptSched's internal region representation
       that tracks register pressure with spill cost simulation)

    4. region->FindOptimalSchedule()
       Runs OptSched's scheduling algorithms:
       - Heuristic list scheduler
       - ACO (Ant Colony Optimization)
       - Enumerator (branch-and-bound exact solver)

    5. Convert back to LLVM:
       Walk the resulting InstSchedule, call ScheduleNode() for each
       instruction, which calls moveInstruction() to physically reorder
       the MachineInstrs
```

#### Two-pass flow

When two-pass scheduling is enabled, the flow splits between `schedule()` and
`finalizeSchedule()`:

**During the normal per-region driver calls:**

```
schedule():
    // Don't actually schedule — just record the region boundaries
    Regions.push_back(RegionBegin, RegionEnd)
    return
```

**After all regions have been visited:**

```
finalizeSchedule():
    for each pass in [OptSchedMinRP, OptSchedBalanced]:
        for each recorded region:
            enterRegion(...)
            runSchedPass(pass)
            exitRegion()
```

Pass 1 (`OptSchedMinRP`) schedules with a heuristic list scheduler using unity
latencies, minimizing register pressure. Pass 2 (`OptSchedBalanced`) schedules
with the enumerator using rough latencies, balancing register pressure and ILP.

#### Dependency type handling

When converting LLVM's SDep edges to OptSched's internal representation, OptSched
maps the four LLVM `SDep::Kind` values:

| LLVM `SDep::Kind` | OptSched `DependenceType` |
|--------------------|--------------------------|
| `SDep::Data`       | `DEP_DATA`               |
| `SDep::Anti`       | `DEP_ANTI`               |
| `SDep::Output`     | `DEP_OUTPUT`             |
| `SDep::Order`      | `DEP_OTHER` (or `DEP_DATA` if `TREAT_ORDER_DEPS_AS_DATA_DEPS` is set) |

The distinction matters for latency assignment: with `DEP_OTHER`, the machine
model may assign zero latency (ordering must be respected but there's no pipeline
stall), while `DEP_DATA` gets a real latency.

OptSched also distinguishes artificial edges (Cluster, Artificial) from real ones.
The `IgnoreArtificialEdges` flag lets the second pass of two-pass scheduling
strip out artificial edges, giving the enumerator more freedom.

### 8.3 OptSched's own class hierarchy

OptSched has its own internal hierarchy for schedulers and regions, separate from
LLVM's:

**Schedulers:**

```
InstScheduler            (gen_sched.h:51)
  └─ ConstrainedScheduler (gen_sched.h:109)
      ├─ ACOScheduler     (aco.h:49)
      └─ Enumerator       (enumerator.h:333)
```

**Regions:**

```
SchedRegion              (sched_region.h:46)
  └─ BBWithSpill         (bb_spill.h:30)
```

`BBWithSpill` ("Basic Block With Spill cost tracking") is the concrete region
class. It maintains its own register pressure tracking with simple bitvectors,
enabling the fast schedule/unschedule (backtracking) that the enumerator requires.
See [Appendix F](#appendix-f-optscheds-register-pressure-tracking-bbwithspill).

---

## 9. How the Schedule is Communicated Downstream

There is no separate "schedule" data structure that gets passed to later passes.
The scheduler **physically reorders the MachineInstrs** inside the
MachineBasicBlock's instruction list. That reordered list **is** the schedule.

When `schedule()` finishes, the MachineBasicBlock's instruction list is in the
new order. The next pass simply iterates over the block and sees the instructions
in the order the scheduler chose.

For `ScheduleDAGMI`, this happens via `moveInstruction()` inside the `pickNode()`
loop. For `ScheduleDAGOptSched`, same thing — it walks OptSched's resulting
`InstSchedule` and calls `ScheduleNode()` for each instruction, which calls
`moveInstruction()` to place it.

The cycle/slot information that the scheduler uses internally (for pipeline
simulation) is **not** passed downstream. Only the linear order matters. The
hardware (or a post-RA hazard recognizer) handles actual pipeline timing.

OptSched's internal data structures (`BBWithSpill`, `DataDepGraph`,
`InstSchedule`) are temporary and destroyed after scheduling. No later pass
sees them.

---

## Appendix A: Memory Dependencies and Alias Analysis

Memory dependencies are ordering edges between instructions that access memory.
They enforce correctness — you cannot freely reorder memory operations.

| Dependency | Example                            | Why                                    |
|------------|------------------------------------|-----------------------------------------|
| **RAW**    | `STORE [addr]; LOAD [addr]`        | Load must see the store's value         |
| **WAR**    | `LOAD [addr]; STORE [addr]`        | Load must get the old value             |
| **WAW**    | `STORE [addr], v1; STORE [addr], v2` | Final memory state must reflect second store |

The hard part is that the scheduler often cannot tell whether two memory
operations alias (touch the same address). When it can prove they don't alias
(via alias analysis), no edge is needed and they can be freely reordered. When
it cannot prove it, it conservatively adds a dependency edge, which constrains
scheduling freedom.

More alias information = fewer conservative edges = more freedom for the
scheduler to find a better order.

---

## Appendix B: The Topological Sort

`ScheduleDAGInstrs` maintains a `ScheduleDAGTopologicalSort` (the `Topo` member)
that provides an efficient topological ordering of the DAG. It maintains two
arrays:

```
Node2Index[NodeNum] → topological position
Index2Node[position] → NodeNum
```

**Invariant:** For every edge A → B, `Node2Index[A] < Node2Index[B]`.

### When is it computed?

`InitDAGTopologicalSorting()` runs after `buildSchedGraph`. It uses Kahn's
algorithm: start from leaf nodes (no successors), assign them the highest indices,
then work backward through predecessors.

### How it speeds up reachability

`IsReachable(SU, TargetSU)` asks: "is there a path from TargetSU to SU?"

```
LowerBound = Node2Index[TargetSU]
UpperBound = Node2Index[SU]

if LowerBound < UpperBound:
    // TargetSU comes before SU in topo order — a path MIGHT exist.
    // Must do DFS to confirm.
    DFS(TargetSU, UpperBound, HasLoop)
else:
    // Wrong direction — impossible. O(1) answer.
    return false
```

The topo order gives a **cheap O(1) negative test**. The expensive O(V+E) DFS
only runs when the topo order says a path might exist.

### Incremental updates

When mutations add new edges, the topo sort can be updated incrementally via
`AddPredQueued`. If too many updates accumulate (> 10), it sets a `Dirty` flag
and recomputes from scratch on the next query (`FixOrder()`). Queries
(`IsReachable`, `WillCreateCycle`) call `FixOrder()` lazily before proceeding.

### Primary use: cycle prevention

The main use in the machine scheduler context is `ScheduleDAGInstrs::addEdge()`:

```
addEdge(SuccSU, PredDep):
    // Would adding PredSU → SuccSU create a cycle?
    if Topo.IsReachable(PredDep.getSUnit(), SuccSU):
        return false    // reject the edge
    Topo.AddPredQueued(SuccSU, PredDep.getSUnit())
    SuccSU.addPred(PredDep)
    return true
```

This is what mutations (like clustering) call when adding edges to the DAG.

---

## Appendix C: Weak vs Strong Edges and Clustering

### Strong vs weak edges

| Property         | Strong edges                      | Weak/Cluster edges                     |
|------------------|-----------------------------------|----------------------------------------|
| **Correctness**  | Must be respected                 | Can be ignored — correct either way    |
| **Readiness**    | Block readiness (`NumPredsLeft`)  | Don't block readiness (`WeakPredsLeft`) |
| **Strategy**     | Structurally enforced             | Priority hint only                     |
| **Examples**     | Data, Anti, Output, Barrier, MayAliasMem | Cluster, Weak              |

### How clustering works

Clustering is a DAG mutation that finds pairs of loads or stores to nearby memory
addresses and links them with weak edges, encouraging the strategy to schedule
them adjacently. The purpose: a downstream pass (`SILoadStoreOptimizer`, see
[Appendix D](#appendix-d-the-siloadstoreoptimizer)) can merge adjacent loads/stores
into wider memory operations.

The clustering algorithm:

```
BaseMemOpClusterMutation::apply(DAG):
    1. Collect all loads (or stores) in the region
    2. Sort them by base register + offset
    3. For each adjacent pair (A, B) in sorted order:
       - Ask the target: TII->shouldClusterMemOps(A, B)?
       - Check they don't already have a path between them
         (via IsReachable — if a path exists, they can't be
         adjacent no matter what, so clustering is pointless)
       - If yes: add SDep(A, SDep::Cluster) edge from A → B
       - Copy surrounding edges to prevent other instructions
         from being scheduled between A and B
```

The `shouldClusterMemOps` call is the only **architecture-specific** part of
clustering. The rest is target-independent infrastructure.

### How the strategy uses cluster hints

When the scheduler schedules SU_A and releases the Cluster edge, it sets
`NextClusterSucc = SU_B`. In the next `pickNode()` call, if SU_B is among the
ready candidates, it gets a **priority boost**. But if scheduling SU_B next would
cause a register pressure spike, the strategy can pick something else instead.

For large regions, the reachability check becomes expensive (O(V+E) DFS per pair).
A **fast cluster** mode activates when `numMemOps * numSUnits > 1,000,000`,
skipping the reachability check and instead grouping memory ops by their control
predecessor to reduce the chance of wasted cluster edges.

---

## Appendix D: The SILoadStoreOptimizer

Clustering puts loads/stores next to each other; a later pass actually **merges**
them. The `SILoadStoreOptimizer` runs after scheduling and walks the basic block
looking for adjacent memory operations to consecutive addresses:

- Two `BUFFER_LOAD_DWORD` (32-bit each) → one `BUFFER_LOAD_DWORDX2` (64-bit)
- Two `FLAT_LOAD_DWORD` → one `FLAT_LOAD_DWORDX2`
- Similarly for stores and wider combinations (x2 → x4, etc.)

The optimizer scans linearly with a limited window. If the scheduler scatters
two mergeable loads across the region with unrelated instructions between them,
the optimizer will not find them. This is why the scheduler's clustering matters.

```
Scheduler (clustering mutation)     →  places loads adjacent
SILoadStoreOptimizer (later pass)   →  sees adjacent loads, merges into wider op
```

---

## Appendix E: Pre-RA vs Post-RA Scheduling

### Pre-RA scheduling

Operates on **virtual registers**. There are unlimited virtual registers, but
after scheduling, register allocation assigns them to physical registers. If the
schedule keeps too many values live simultaneously, the allocator runs out of
physical registers and must **spill** (store to memory and reload). This is why
pre-RA scheduling cares about register pressure.

### Post-RA scheduling

Operates on **physical registers**. Register allocation is done, so the scheduler
knows exactly which physical registers are in use. It doesn't worry about
pressure (spills are already resolved) — it focuses on **ILP** and hiding
latencies.

### Why post-RA scheduling exists

Pre-RA scheduling makes compromises to keep register pressure low — it may choose
a worse instruction order to avoid spills. After register allocation, those
compromises may no longer be necessary. Post-RA scheduling can:

- **Hide latencies** by moving independent work between a long-latency
  instruction and its use.
- **Avoid pipeline hazards** by inserting independent instructions between
  sequences that would cause hardware stalls.
- **Account for real register constraints** that were unknown before allocation.

Not all targets benefit equally. Out-of-order processors (like modern x86)
reorder instructions in hardware, so post-RA scheduling has diminishing returns.
In-order processors benefit more since the compiler's order is what executes.

On AMDGPU, post-RA scheduling uses `PostGenericScheduler` and is unchanged
regardless of whether OptSched is enabled for pre-RA scheduling.

---

## Appendix F: OptSched's Register Pressure Tracking (BBWithSpill)

### Why OptSched can't use LLVM's RegPressureTracker

LLVM's `RegPressureTracker` is designed for the `pickNode()` loop — it advances
one instruction at a time in a single direction. The enumerator does something
fundamentally different:

```
enumerate(partial_schedule, depth):
    for each ready instruction X:
        SchdulInst(X)                    // try X here
        if cost_so_far > best_known:
            UnschdulInst(X)              // prune — undo X
            continue
        enumerate(partial_schedule, depth + 1)   // recurse
        UnschdulInst(X)                  // backtrack — undo X, try next
```

`RegPressureTracker` has no mechanism to undo an arbitrary instruction placement
and restore the prior state. Copying the full tracker state at every branch point
would be prohibitively expensive given the millions of nodes explored.

### BBWithSpill's approach

`BBWithSpill` maintains its own simple, reversible state:

```
BBWithSpill:
    liveRegs_: WeightedBitVector[]     // one bitvector per register type
    regPressures_: int[]               // current # live regs per type
    peakRegPressures_: int[]           // peak # live regs per type
    sumOfLiveIntervalLengths_: int[]   // SLIL cost metric per type
    crntSpillCost_: InstCount          // current total spill cost
```

#### Schedule an instruction

```
UpdateSpillInfoForSchdul_(inst):
    // Process uses — last use kills the register
    for each register USE of inst:
        use.AddCrntUse()
        if all uses consumed:
            liveRegs_[regType].clear(regNum)     // no longer live

    // Process defs — defining a register makes it live
    for each register DEF of inst:
        liveRegs_[regType].set(regNum)           // now live

    // Compute pressure
    for each register type i:
        liveRegs = liveRegs_[i].count()
        regPressures_[i] = liveRegs
        peakRegPressures_[i] = max(peak, liveRegs)

    spillCost = computeCostFunction(...)   // PERP or SLIL
```

#### Unschedule an instruction (backtrack)

`UnschdulInst` reverses the bitvector changes and restores the saved cost from
the enumeration tree node.

### Accuracy tradeoff

BBWithSpill is **less accurate** than `RegPressureTracker`:

- Tracks whole registers, not sub-register lanes
- Uses OptSched's own non-overlapping register type classification, not LLVM's
  overlapping pressure sets
- Reconstructs liveness from def/use counts rather than using LLVM's
  `LiveIntervals`

However, for comparing candidate schedules against each other, the relative
ranking is what matters — and BBWithSpill is fast enough to be called millions
of times during enumeration.

OptSched verifies its result by checking `getRealRegPressure()` (using LLVM's
`GCNDownwardRPTracker`) **after** scheduling, before committing to a schedule.

---

## Summary: Data Flow Through the Stack

```
MachineBasicBlock (linear list of MachineInstrs)
        │
        ▼  enterRegion(): set RegionBegin/RegionEnd
┌─ ScheduleDAGInstrs::buildSchedGraph() ─┐
│   Walk instrs bottom-to-top              │
│   Create SUnits (1 per instr)            │
│   Add SDep edges:                        │
│     - register Data/Anti/Output          │
│     - memory Order (using alias analysis)│
│     - barrier Order                      │
└──────────────┬───────────────────────────┘
               ▼
        DAG: nodes = SUnits, edges = SDeps
               │
               ▼  postProcessDAG(): apply Mutations
               │
    ┌──────────┴──────────┐
    ▼                     ▼
 ScheduleDAGMI         ScheduleDAGOptSched
 pickNode() loop:      Own algorithms:
 asks Strategy         ACO / Enumerator
 "which next?"         solve internally
    │                     │
    ▼                     ▼
 moveInstruction() ── physically reorder MachineInstrs in the MBB
               │
               ▼
MachineBasicBlock (same instrs, new order)
```

---

## 10. The HierarchicalScheduler

The `ScheduleDAGHierarchicalScheduler` is a custom AMDGPU scheduler that runs
as a second pre-RA scheduling pass after the normal AMDGPU scheduler
(`GCNMaxOccupancySchedStrategy`).

### 10.1 Architecture

The scheduler has three components:

```
THE PASS (generic — lives in lib/CodeGen/MachineScheduler.cpp)
MachineSchedulerHierarchical
│
│  Target-independent pass shell. Knows nothing about AMDGPU.
│  Registered with LLVM's pass manager. When the pass manager calls
│  runOnMachineFunction(), it:
│
│    1. Gathers analyses (LiveIntervals, AliasAnalysis, etc.)
│    2. Asks the factory for a scheduler:
│         PassConfig->createHierarchicalScheduler(this)
│    3. Calls scheduleRegions(scheduler), which:
│         - Calls scheduler.schedule() once per region
│         - Calls scheduler.finalizeSchedule() once at the end
│
│  The pass must live in MachineScheduler.cpp because it inherits from
│  MachineSchedulerBase and uses scheduleRegions(), INITIALIZE_PASS,
│  EnableMachineSched, etc. — all defined in that file's anonymous namespace.
│
│  Calls into ─────────────────────────────────────────────┐
└──────────────────────────────────────────────────────────┼───────────┘
                                                           │
                                                           ▼
THE FACTORY (AMDGPU-specific — lives in AMDGPUTargetMachine.cpp)
createHierarchicalSchedulerGCN()
GCNPassConfig::createHierarchicalScheduler()
│
│  Knows about AMDGPU. Creates the scheduler with:
│    - GCNMaxOccupancySchedStrategy (required by ScheduleDAGMILive)
│    - AMDGPU-specific mutations:
│        LoadCluster, StoreCluster (gfx11+), IGroupLP,
│        MacroFusion, ExportClustering
│
│  Connected to the pass via a virtual method on TargetPassConfig
│  that GCNPassConfig overrides. This is how the generic pass gets
│  an AMDGPU-specific scheduler without knowing about AMDGPU.
│
│  Creates and returns ────────────────────────────────────┐
└──────────────────────────────────────────────────────────┼───────────┘
                                                           │
                                                           ▼
THE SCHEDULER (AMDGPU-specific — lives in
    lib/Target/AMDGPU/HierarchicalScheduler/)
ScheduleDAGHierarchicalScheduler
(namespace: llvm::hierarchical_scheduler)
│
│  Inherits from ScheduleDAGMILive. This is where scheduling logic
│  lives.
│
│  schedule():          Called once per region by scheduleRegions().
│                       Records the region as a RegionInfo object.
│
│  finalizeSchedule():  Called once after all regions are recorded.
│                       Checks config and dispatches to the configured
│                       scheduler (e.g., RunMaliciousScheduler).
│
│  Key infrastructure:
│    RegionInfo          — read-only wrapper around region boundaries
│    BeginRegion/EndRegion — sets up ScheduleDAGMILive state for replay
│    ProcessRegion(region, action) — template that wraps
│         BeginRegion/action/EndRegion with return value forwarding
│    ApplyScheduleOrder  — moves MachineInstrs to match a computed order
│    RunMaliciousScheduler — iterates regions, builds DAGs, scores, applies
└──────────────────────────────────────────────────────────────────────
```

### 10.2 Pipeline position

The pass is inserted into the AMDGPU pipeline via `insertPass` in
`GCNPassConfig::addOptimizedRegAlloc()`, guaranteeing it runs immediately
after the normal `MachineScheduler` pass:

```
MachineScheduler                         (GCNMaxOccupancySchedStrategy)
    ↓
MachineSchedulerHierarchical             (our pass)
    ↓
SIWholeQuadMode
SIPreAllocateWWMRegs
SIOptimizeExecMaskingPreRA
SIFormMemoryClauses
    ↓
Register Allocation
    ↓
PostMachineScheduler
```

### 10.3 Configuration

Enabled by setting the first line of `misched.txt` to `HierarchicalScheduler`.
Options follow on the same line (like OptSched options):

```
HierarchicalScheduler MaliciousScheduler
```

This is read by `MachineInstrSchedulerConfig`, which provides
`IsHierarchicalScheduler()` and `HasHierarchicalSchedulerOption()`.

Currently restricted to `gfx906` (Radeon VII) via a check in
`MachineSchedulerHierarchical::runOnMachineFunction()`.

### 10.4 The MaliciousScheduler

A deliberately bad list scheduler used as an experimental baseline.
Enabled via the `MaliciousScheduler` config option. It aims to maximize
register pressure and minimize latency hiding.

**Flow per region:**

```
ProcessRegion(region, [&]() {
    buildSchedGraph(AA);                    // build DAG with alias analysis
    order = ComputeMaliciousSchedule(SUnits);  // score and pick
    ApplyScheduleOrder(region, order);      // move MachineInstrs
});
```

**Scoring heuristic** (higher = preferred, worse for performance):

| Points | Condition |
|--------|-----------|
| +0     | Base |
| +2     | Just-scheduled instruction is a predecessor/producer of this candidate (favors back-to-back dependencies — bad for latency hiding) |
| +1     | This instruction is NOT the last consumer of any input register (favors keeping live ranges open — increases register pressure) |

**List scheduler pattern:**

1. `InitReadyList` — all SUnits with `NumPredsLeft == 0`
2. `FindBestCandidate` — score each ready SUnit, pick highest
3. `ScheduleInstruction` — mark as scheduled, remove from ready list
4. `ReleaseSuccessors` — decrement `NumPredsLeft` for strong successors,
   add to ready list if they become ready
5. Repeat until ready list is empty

Only Data edges (SDep::Data) are used for the "last consumer" check, since
only Data edges represent true register value flow. Anti/Output edges are
ordering constraints that don't create live ranges. Weak edges are skipped
in `ReleaseSuccessors` since they don't affect `NumPredsLeft`. Boundary
nodes (`EntrySU`/`ExitSU`) are skipped since they are not real instructions.

### 10.5 Region replay and debug instructions

When `finalizeSchedule()` replays recorded regions, it must re-enter each
region to set up `ScheduleDAGMILive` state (`BB`, `RegionBegin`, `RegionEnd`,
`CurrentTop`, `CurrentBottom`). The `BeginRegion`/`EndRegion` methods handle
this by calling `startBlock`/`enterRegion` and `exitRegion`/`finishBlock`.

`CurrentTop` and `CurrentBottom` are NOT set by `enterRegion` — they are
normally set by `initQueues()` inside `schedule()`. During replay, we set
them manually in `BeginRegion`.

Scheduling regions may contain **debug instructions** (`DBG_VALUE` etc.)
interspersed with real instructions. Debug instructions don't get SUnits —
they're skipped during DAG building. `NextIfDebug` (a local copy of a
static helper from `MachineScheduler.cpp`) advances iterators past debug
instructions. After all real instructions are placed, `placeDebugValues()`
repositions debug instructions next to the real instructions they're
associated with.

### 10.6 Register Pressure Tracking (GCNRegisterTracker)

The HierarchicalScheduler has its own register pressure tracker,
`GCNRegisterTracker`, built on top of LLVM's AMDGPU-specific
`GCNRegPressure` infrastructure. This provides accurate sub-register
and tuple handling without reimplementing pressure arithmetic.

#### Why not use existing trackers?

- **`RegPressureTracker`** (LLVM generic): non-copyable (`SparseSet`),
  reference members, no undo support. Designed for `pickNode()` loop
  in a single forward direction.
- **`GCNUpwardRPTracker`/`GCNDownwardRPTracker`**: require sequential
  instruction processing. `recede()` can evaluate a tentative complete
  schedule in reverse, but can't track pressure during incremental
  forward construction (needed for beam search / BnB).
- **`BBWithSpill`** (OptSched): whole-register only. Decomposes each
  LLVM virtual register into individual 32-bit "OptSched registers"
  with its own type system (one type per register file, weight always
  1). This loses tuple weight information that `GCNRegPressure` tracks
  and doesn't use LLVM's liveness data.

#### What GCNRegisterTracker uses from LLVM

- **`GCNRegPressure`**: 7-element array tracking SGPR32, SGPR_TUPLE,
  VGPR32, VGPR_TUPLE, AGPR32, AGPR_TUPLE counts. Provides
  `getOccupancy()` and occupancy-aware comparison via `less()`.
- **`GCNRegPressure::inc(Reg, PrevMask, NewMask, MRI)`**: the core
  pressure update function. Takes previous and new lane masks for a
  register and correctly adjusts pressure, handling sub-register
  partial writes and tuple weights. Reversible: `inc(R, A, B)` followed
  by `inc(R, B, A)` is a no-op.
- **`LiveRegSet`** (`DenseMap<unsigned, LaneBitmask>`): tracks which
  lanes of each register are currently live.
- **`getLiveLaneMask(Reg, SlotIndex, LIS, MRI)`**: queries LiveIntervals
  for the exact lanes live at a given program point. Used for
  entry/exit node construction and use-mask extraction.

#### Pressure model

The tracker supports two pressure models, controlled by the
`PressureModel` enum:

- **`kAMDGPU`** (current default): defs first, peak, then dying uses
  removed. Matches `GCNDownwardRPTracker`. The peak at each instruction
  includes both the new defs and everything already live — conservative,
  since the hardware needs registers for both inputs and outputs
  simultaneously.
- **`kUsesFirst`**: dying uses removed first, then defs, then peak.
  Matches OptSched. Optimistic — assumes the register allocator can
  reuse a dying input's physical register for the output.

#### Extraction

Register defs/uses are pre-extracted once at construction into a
per-node `NodeRegInfo` (deduplicated register + lane mask pairs):

- **Leaf nodes with MachineInstr**: iterates operands. Dead defs
  (`isDead()`) are filtered. Sub-register defs of the same register
  are merged (masks ORed). Uses skip `undef` operands (syntactic uses
  that don't read the register). Lane masks come from
  `GetDefMask`/`GetUseMask` (local reimplementations of the static
  helpers in `GCNRegPressure.cpp`).
- **Entry/exit nodes**: read from `ScheduleNode::RegDefs()`/`RegUses()`,
  which carry per-lane masks from `getLiveLaneMask()` (set during
  graph construction in `CreateEntryAndExitNodes`).
- **Group nodes**: not yet supported (fatal error).

#### Kill detection

Currently **whole-register**: a `DenseMap<unsigned, int>` counts
remaining uses per register. When the count hits 0, the register
dies and is removed from the live set. This can overestimate pressure
when sub-register uses finish at different times — a partially dead
register is counted as fully live until its last use of any lane.

Upgrading to per-lane kill detection would only change
`remaining_uses_` and the kill logic in `Schedule`/`Unschedule`.

#### Undo mechanism

Each `Schedule()` call pushes a `ScheduleStep` record:

- **`saved_max`**: peak pressure before this step (7 ints). Can't be
  reverse-computed since it's a running maximum.
- **`def_prev_masks`**: for each def, the register's lane mask before
  the def was added. Allows reversing the `inc()` call.
- **`kill_masks`**: for each register that died (remaining_uses hit 0),
  the lane mask it had before being erased from the live set. Needed
  because the mask was erased and can't be recovered otherwise.
- Non-killing uses save nothing — `remaining_uses_` is trivially
  reversed by incrementing.

`Unschedule()` pops the record, restores killed registers, reverses
def `inc()` calls, and re-increments use counts.

#### Cross-check

`VerifyGCNRegisterTracker` walks the same instruction order with both
our tracker and LLVM's `GCNUpwardRPTracker`, printing per-instruction
pressure side by side. This verifies correctness — any difference is
either the known whole-register kill overestimate or a bug.

### 10.7 Schedule Length Tracking (ScheduleLengthTracker)

`ScheduleLengthTracker` tracks schedule length and bubbles (stall
cycles) during incremental forward schedule construction. It uses
the `getScheduleMetrics()` model (see Appendix M.7): one instruction
per cycle (`IssueWidth = 1`), with stalls when a dependency isn't
ready.

For each scheduled instruction:
```
ready_cycle = max(current_cycle,
                  max(pred.scheduled_cycle + edge.latency)
                      for all scheduled data-dependency predecessors)
bubbles += ready_cycle - current_cycle
scheduled_cycle[node] = ready_cycle
current_cycle = ready_cycle + 1
```

Supports do/undo and is fully copyable. Validates at construction
that `IssueWidth == 1` and no node uses a reserved/unbuffered
resource (e.g., MFMA on HWXDL). Validation is cached by graph ID.

### 10.8 ScheduleConstructor

`ScheduleConstructor` is a unified interface that wraps
`GCNRegisterTracker`, `ScheduleLengthTracker`, and a ready list
into a single object for incremental schedule construction.

**Ready list:** A node is ready when all its strong predecessors
have been scheduled. Weak edges (cluster hints, etc.) do not block
readiness. Uses `SmallDenseSet` for O(1) membership checks.

**Usage:**
```cpp
ScheduleConstructor sc(graph, st, mf, lis);
while (!sc.IsDone()) {
    // pick from sc.GetReadyList() using some strategy
    sc.Schedule(chosen_node);
}
// sc.GetLengthTracker() has length/bubbles
// sc.GetPressureTracker() has register pressure/occupancy
```

**Do/undo:** `Schedule()` updates both trackers, the ready list,
and the schedule order. `Unschedule()` reverses all of these.
`ReleaseSuccessors`/`UnreleaseSuccessors` handle the pred-count
bookkeeping for the ready list.

**Copyable** for beam search: all members are value types or
owning containers.

Currently leaf-only. Group nodes will be handled by a future
`HierarchicalScheduleConstructor` layer.

### 10.9 Occupancy Handling

At the start of `RunHierarchicalScheduler`, `InitFunction()` calls
`resetInitialOccupancy()` on the `SIMachineFunctionInfo`. This
restores `MFI.Occupancy` to the pre-GCN-scheduler value — the
structural ceiling based on hardware max, LDS, and launch bounds,
before any scheduler reduced it due to register pressure. This is
the same approach OptSched uses (`GCNOptSched.cpp:58`).

`GCNRegisterTracker` provides three occupancy query functions:

- **`GetRegisterOccupancy()`** — occupancy from peak register
  pressure only (SGPR + VGPR limits). Does not account for LDS
  or launch bounds.

- **`GetRegionOccupancy()`** — `min(MFI.getOccupancy(),
  register_occupancy)`. Incorporates the function-level ceiling
  (hardware, LDS, launch bounds) and any reductions from this
  scheduler processing other regions.

- **`GetStandaloneRegionOccupancy()`** — recomputed from scratch
  via `GCNSubtarget::computeOccupancy()` with this region's peak
  pressure, the kernel's LDS, and launch bounds. Ignores any
  occupancy limit set on the MachineFunction. Useful for
  evaluating what a region could achieve independently.

- **`GetContinuousOccupancyScore()`** — smooth version of
  `GetRegisterOccupancy()`. Integer occupancy is stair-stepped
  (e.g., on gfx906, 24 VGPRs → 10 waves, 25 VGPRs → 9 waves),
  which hides incremental progress: two schedules at 25 and 28
  VGPRs both report occupancy 9 and look identical to a search.
  The continuous score is `M * occ + M * (ceil - num_regs) /
  (ceil - floor)`, where `M = kOccScoreMultiplier = 1000`,
  `ceil`/`floor` are the bracket bounds, and each integer
  occupancy level is worth `M` points. It's the minimum of the
  VGPR- and SGPR-dimension scores, uncapped by design — the
  early-exit check `IsAtOccupancyCeiling()` handles saturation
  separately, so capping would only collapse the top bracket's
  resolution without changing any decision.

- **`GetFunctionOccupancyLimit()`** — wraps
  `MFI->getOccupancy()`, the current function-level clamp
  (structural ceiling after any reductions from earlier regions).

#### 10.9.1 SGPR ceiling table (why we roll our own)

The continuous score uses `getOccupancyWithNumVGPRs` /
`getOccupancyWithNumSGPRs` as its classifier (matching
`GCNRegPressure::getOccupancy` in LLVM itself), and it needs
bracket boundaries to interpolate. For VGPR, LLVM's
`getMaxNumVGPRs(occ)` agrees with the classifier and we use it
directly. For **SGPR, the two functions disagree**, and the
scheduler's score was initially broken because of it.

The two functions exist for different purposes:

- `getOccupancyWithNumSGPRs(N)` — *"If the program uses N SGPRs,
  what occupancy does the hardware give you?"* On gfx8+ it's
  literally a hardcoded if-else table from the hardware docs
  (`AMDGPUSubtarget.cpp:637`): 80→10, 88→9, 100→8, else→7.

- `getMaxNumSGPRs(wavesPerEU, false)` — *"How many SGPRs should
  the register allocator budget to reliably achieve `wavesPerEU`?"*
  Computed as `alignDown(totalSGPRs/waves, granule)` minus
  trap-handler reservation. Deliberately conservative so the
  allocator can't overshoot a granule boundary.

On gfx906, `getMaxNumSGPRs(10) = 64` (allocator budget) but
`getOccupancyWithNumSGPRs(80) = 10` (hardware cliff). An SGPR
count of 80 is "top of the occ=10 bracket" by the hardware, but
below the allocator budget's view. Using `getMaxNumSGPRs` as the
bracket-boundary source in the score helper produced nonsense:
the classifier put 80 in the occ=10 bracket, the scorer set
`ceil = 64` for that bracket, yielded `within = 1000*(64-80)/64
= -250`, and returned `9750` where you'd expect `10000`.

The fix is `GCNRegisterTracker::GetMaxNumSGPRsForOcc(st, occ)`:
a file-local table built once by walking the classifier from 0
upward and recording the last `num_regs` at each occupancy
before it drops. Reachable brackets get real ceilings; the
bottom (flat) bracket and any unreachable occupancies get the
sentinel `kNoSGPRCliff = 255`. By construction, this helper
agrees with the classifier — `getMaxNumSGPRsForOcc(occ)` is
always the largest N such that `getOccupancyWithNumSGPRs(N) ==
occ`. VGPR still uses LLVM's `getMaxNumVGPRs` unchanged.

#### 10.9.2 Why we don't correct for reserved SGPRs

The SGPR count tracked by the scheduler is a *virtual register
count* — it doesn't include VCC, FLAT_SCRATCH, XNACK, trap
handler SGPRs, or any other reservations. The final physical
SGPR count after allocation is typically higher, which means
our classification is optimistic: a schedule we score as
occ=10 might actually hit occ=9 once reservations are added.

`GCNRegPressure::getOccupancy` in LLVM has the same blind spot
— it passes VR counts straight to `getOccupancyWithNumSGPRs`
with no correction. Our score matches that behavior, which
keeps `GetRegisterOccupancy()` and `GetContinuousOccupancyScore()`
in sync with each other and with the rest of the compiler.

In principle, a schedule can end up optimizing the wrong
dimension if reservations shift which of SGPR/VGPR is the
binding constraint, but in practice this rarely matters: gfx906
reservations are small (VCC is 2 SGPRs, plus a few more with
flat scratch / trap handler), and SGPR is almost never the
limiting factor for occupancy on realistic kernels. See the
TODO section for a note on revisiting this.

#### 10.9.3 `ScheduleConstructor` comparison interface

`ScheduleConstructor::IsBetterThan(other, metric)` compares two
schedule-construction states under one of three `ScheduleMetric`
values:

- `kRegisterOccupancy` — integer register occupancy (higher is
  better).
- `kContinuousRegisterOccupancyScore` — smooth occupancy score
  (higher is better).
- `kScheduleLength` — current cycle count (lower is better).

`IsBetterThan` is strict: ties return false. `IsAtOccupancyCeiling()`
returns true when `GetRegisterOccupancy() >= GetFunctionOccupancyLimit()`,
i.e. when further register improvements cannot raise region
occupancy — the natural early-exit condition for occupancy-focused
search.

### 10.10 Target Architecture (not yet implemented)

The long-term design supports multiple scheduling passes with
different algorithms and objectives. This is a roadmap, not
current code.

**Pass types:**

- **ConstructionPass** — builds a schedule from scratch. Configured
  with a search algorithm, objective function, and group handler.

- **RefinementPass** — takes an existing schedule and improves it
  by rescheduling portions (rolling window) while replaying the
  rest. Configured with window size and stride.

**Search algorithms** (each owns its control flow):

- **Greedy** — simple forward loop, pick best ready node per
  objective.
- **Beam search** — maintain K copies of the constructor, expand
  each, keep top K.
- **BnB/enumerator** — recursive DFS with backtracking, prune via
  bound checking.
- **ACO** — N iterations of probabilistic schedule construction,
  update pheromone trails between iterations.

**Objective functions:** score a schedule and compare two schedules.
MinPressure, MinLength, Balanced (weighted combination).

**Group handling:** when a group node is picked from the ready list,
the group's internal nodes are scheduled as a contiguous block
through the same `ScheduleConstructor` (same pressure/length state,
no interleaving with outer nodes). Options: schedule now (run a
search algorithm recursively), replay a cached schedule, or
cache-or-schedule. Groups may be nested — the same mechanism
applies recursively.

**Constraints:** a pass can constrain the next pass (e.g., "don't
drop below occupancy 8"). Derived from the previous pass's result.

**Example multi-pass configuration:**
```
Pass 1: ConstructionPass
    search: BeamSearch(width=8)
    objective: MinPressure
    → establishes occupancy target

Pass 2: ConstructionPass
    search: BeamSearch(width=8)
    objective: MinLength
    constraint: occupancy >= pass 1 result

Pass 3: RefinementPass
    search: Greedy
    objective: Balanced
    window: 30 nodes, stride: 15
    → smooths subgraph boundaries
```

**Implementation approach:** implement example algorithms concretely first, 
extract common interfaces as patterns emerge. The design above is the target,
not themaxWavesPerEU starting point.

### 10.11 Files

| File | Role |
|------|------|
| **Pass and pipeline (pre-existing LLVM files)** | |

| File | Role |
|------|------|
| **Pass and pipeline (pre-existing LLVM files)** | |
| `lib/CodeGen/MachineScheduler.cpp` | Pass shell (class, registration, `runOnMachineFunction`) |
| `lib/CodeGen/CodeGen.cpp` | Pass initialization registration |
| `lib/CodeGen/TargetPassConfig.cpp` | Pipeline integration |
| `lib/Target/AMDGPU/AMDGPUTargetMachine.cpp` | Factory, pipeline insertion (`insertPass`), pass ordering |
| `include/llvm/CodeGen/TargetPassConfig.h` | `createHierarchicalScheduler` virtual method |
| `include/llvm/CodeGen/Passes.h` | `MachineSchedulerHierarchicalID` extern declaration |
| `include/llvm/InitializePasses.h` | `initializeMachineSchedulerHierarchicalPass` declaration |
| `include/llvm/Analysis/MachineInstrSchedulerConfig.h` | `HierarchicalScheduler` enum, options, `IsHierarchicalScheduler()` |
| **HierarchicalScheduler directory** | |
| `HierarchicalScheduler/CMakeLists.txt` | Sets `HIERARCHICAL_SCHEDULER_SOURCES` with `PARENT_SCOPE`; files compile as part of AMDGPUCodeGen (not a separate library) so we can call `GCNRegPressure::inc()` etc. without circular link dependencies |
| `HierarchicalScheduler/RegionInfo.h` | Read-only region boundary wrapper |
| `HierarchicalScheduler/ScheduleDAGHierarchicalScheduler.h` | Scheduler class (region recording, replay, `ProcessRegion` template) |
| `HierarchicalScheduler/ScheduleDAGHierarchicalScheduler.cpp` | Scheduler implementation, shakedowns, cross-check verification |
| `HierarchicalScheduler/MaliciousScheduler.h/.cpp` | Deliberately bad list scheduler for baseline testing |
| `HierarchicalScheduler/ScheduleGraph.h/.cpp` | Recursive node/edge graph structure (`ScheduleNode`, `ScheduleGraph`, `ScheduleEdge`). Topo sort (Kahn's), transitive reduction, entry/exit nodes with per-lane register info |
| `HierarchicalScheduler/DominatorTree.h/.cpp` | Dominator tree from transitively reduced DAGs (CHK algorithm) |
| `HierarchicalScheduler/RegisterTracker.h/.cpp` | Original register pressure tracker (whole-register, custom pressure model). Superseded by `GCNRegisterTracker` but kept for reference |
| `HierarchicalScheduler/GCNRegisterTracker.h/.cpp` | Register pressure tracker using `GCNRegPressure`/`LiveRegSet`. Occupancy queries, function call warning |
| `HierarchicalScheduler/ScheduleLengthTracker.h/.cpp` | Schedule length and bubble tracking (`IssueWidth=1`). Validates no reserved resources |
| `HierarchicalScheduler/ScheduleConstructor.h/.cpp` | Unified interface wrapping pressure tracker, length tracker, and ready list. Copyable for beam search |

### 10.8 TODOs and Future Work

- **Per-lane kill detection.** Currently, a register dies when its last
  use of any lane is scheduled. This overestimates pressure when
  sub-register uses finish at different times (confirmed: 1 VGPR
  overestimate on the stencil kernel from `%114.sub0` dying before
  `sub1`). The infrastructure already supports per-lane tracking —
  `GCNRegPressure::inc()` handles partial lane transitions, and
  entry/exit nodes store per-lane masks via `getLiveLaneMask()`. The
  upgrade requires changing `remaining_uses_` from a per-register int
  to a per-lane count structure, and updating the kill logic in
  `Schedule`/`Unschedule`. Everything else (pressure computation, undo
  mechanism, extraction) stays the same.

- **Group node register extraction.** `GCNRegisterTracker` currently
  reports a fatal error for group nodes (subgraphs). Supporting them
  requires computing aggregate register effects at subgraph
  boundaries — which defs are visible outside the group, which uses
  come from outside. The `AddRegMask` dedup helper is designed to be
  reusable for this.

- **Dominator tree update.** The dominator tree construction should be
  updated to account for the custom entry/exit nodes. Currently it
  operates on the original graph structure without awareness of entry/
  exit node semantics.

- **SGPR occupancy scoring and reserved registers (low priority).**
  Both `GetRegisterOccupancy()` (via `GCNRegPressure::getOccupancy`)
  and `GetContinuousOccupancyScore()` pass raw virtual-register counts
  to the hardware classifier. This ignores reserved SGPRs (VCC,
  FLAT_SCRATCH, XNACK, trap handler), so our SGPR→occupancy mapping
  is slightly optimistic: a schedule we report as occ=10 might
  actually land at occ=9 once reservations are added. Could be
  corrected by adding `getReservedNumSGPRs(MF)` to the SGPR count
  before looking up the bracket, in both code paths so they stay
  consistent. Deferred because (1) reservations are small (2–6 SGPRs
  on gfx906), (2) SGPR is almost never the binding constraint on
  realistic kernels, so even if the bias flips which dimension the
  search sees as the min, it rarely changes outcomes, and (3) fixing
  it would diverge from LLVM's own `GCNRegPressure::getOccupancy`
  and from every other AMDGPU scheduling heuristic — worth doing
  only if we have evidence the bias is hurting schedule quality.
  See section 10.9.2 for the analysis.

- **Exit node latency for high-latency leaf instructions.** LLVM's
  `buildSchedGraph` adds an artificial edge from high-latency leaf
  instructions (e.g., VMEM loads with no successors in the region)
  to ExitSU with `latency = SU->Latency - 1`. Our exit node edges
  all have latency 0. This is fine for within-region schedule length
  computation but may underestimate costs for cross-region analysis.
  See `ScheduleDAGInstrs.cpp`, line 877.

- **Sort regions by input register pressure for occupancy passes.**
  When the scheduler runs, the input instruction order in each region
  reflects the GCN scheduler's output, and its peak register pressure
  is already a known scalar per region. For an occupancy-targeted
  pass (e.g., "raise every region to at least occupancy N"), we can
  pre-sort regions by input pressure descending and iterate in that
  order. As soon as we find a region where we *cannot* raise
  occupancy past its current ceiling — the hardest region by input
  pressure — we can break out of the loop: every remaining region
  has strictly lower input pressure, so they already achieve at
  least that occupancy in the unscheduled input and nothing we do
  to them can *lower* the function-wide ceiling we just discovered.
  This avoids scheduling work on the easy regions whenever the hard
  region is the binding constraint, which is the common case.

- **Scheduling algorithm.** The core hierarchical scheduling algorithm
  is not yet implemented — `RunHierarchicalScheduler` currently builds
  the graph and runs shakedowns but does not reorder instructions.
  See section 10.10 for the target architecture (multi-pass with
  construction and refinement passes, multiple search algorithms,
  group/subgraph handling). The infrastructure (`ScheduleConstructor`,
  pressure and length tracking, ready list, do/undo, copyable state)
  is in place to support this.

- **Remove shakedown/debug output.** The current `llvm::outs()` prints
  throughout the scheduler are for development. They should be removed
  or moved behind `LLVM_DEBUG` before the scheduler is used in
  production.

- **Verify LLVM's scheduling model latencies for gfx906.** The latency
  numbers in `SIQuarterSpeedModel` were written in 2015 for early GCN
  hardware and inherited unchanged when gfx906 was added in 2018.
  Several values are suspect — most notably `WriteFloatFMA = 16`,
  which implies FP32 FMA is 16x slower than FP32 add/multiply on
  gfx906. A 2014 GCN optimization talk (Michal Drobot, "Low Level
  Optimizations for GCN", Digital Dragons 2014) lists FP32 arithmetic
  as full-rate and only int32 multiply, FP64, and transcendentals as
  quarter-rate — FP32 FMA is not in the quarter-rate category.
  Both the Drobot talk (2014) and the LLVM model (2015) describe
  the same era of GCN hardware, yet they disagree: Drobot says
  FP32 arithmetic is full-rate, while the LLVM model gives FP32
  FMA 16 cycles. Neither source has been verified for gfx906
  (released 2019), which inherited these numbers unchanged. If
  FMA is actually full-rate on gfx906, the model's 16-cycle
  latency is wrong by 16x. Other values (VMEM, LDS, SMEM) may also be
  approximate. These should be measured experimentally on gfx906
  hardware. Our scheduler uses `SDep::getLatency()` (which reads
  these values), so inaccurate latencies directly affect scheduling
  quality. See Appendix M.6 for details on the model's provenance.

- **Handle non-inlined function calls.** Currently
  `GCNRegisterTracker` warns when it encounters a function call
  node but proceeds without accounting for the callee's register
  usage (matching LLVM's default behavior). In practice, `hipcc`
  force-inlines all functions (except those marked `__noinline__`),
  so this is rarely an issue. If non-inlined calls become common,
  we could process functions in reverse call graph order and
  incorporate callee register usage when scheduling callers. See
  Appendix N.6 for how `AMDGPUResourceUsageAnalysis` propagates
  register counts through the call graph.

- **Investigate occupancy targeting based on actual launch bounds.**
  When no `__launch_bounds__` or `amdgpu-waves-per-eu` attributes
  are specified, the compiler assumes a max workgroup size of 1024
  and targets max occupancy (10 waves for gfx906). The actual
  launch bounds may be determinable at compile time by inspecting
  the host-side callsites (e.g., `hipLaunchKernelGGL` arguments).
  If so, the scheduler could compute a tighter occupancy target.
  Another approach: generate multiple versions of the kernel with
  different occupancy levels (different register budgets) and emit
  a dispatch function that selects the appropriate version based on
  runtime launch bounds. See Appendix N.3 for how launch bounds
  affect occupancy.

---

## Appendix G: Manual Scheduling Hints

*AMDGPU-specific.*

AMDGPU provides intrinsics that allow programmers to manually constrain
instruction scheduling from kernel source code:

- **`__builtin_amdgcn_sched_barrier(mask)`** — Prevents instructions of the
  types specified by `mask` from being reordered across this point.

- **`__builtin_amdgcn_sched_group_barrier(mask, size, sync_id)`** — Defines a
  group of `size` instructions matching `mask`. Groups with the same `sync_id`
  are ordered sequentially. This lets the programmer specify interleaving
  patterns (e.g., "4 VMEM loads, then 8 VALU ops, then 4 more VMEM loads").

These lower to `SCHED_BARRIER` and `SCHED_GROUP_BARRIER` pseudo-instructions,
which `IGroupLPDAGMutation` processes during the mutation phase. The mutation
adds **strong Artificial edges** — these are hard constraints that the scheduler
cannot violate.

This mechanism is intended for hand-tuning critical kernels where the programmer
knows the optimal instruction pattern better than the compiler's heuristics
(e.g., software pipelining for inter-wavefront latency hiding).

---

## Appendix H: Asynchronous Memory and s_waitcnt

*AMDGPU-specific.*

On AMDGPU, memory load operations (VMEM, SMEM, LDS) are **asynchronous**. When
a load is issued, execution continues immediately — the result arrives later.
The hardware tracks outstanding operations using counters (`vmcnt` for VMEM,
`lgkmcnt` for LDS/SMEM, etc.).

`s_waitcnt vmcnt(N)` means "stall until at most N VMEM operations are still
outstanding." This is how the compiler ensures a load result is ready before it
is used.

**`s_waitcnt` instructions are NOT present during scheduling.** They are
inserted very late in the pipeline by the `SIInsertWaitcnts` pass, which runs
in `addPreEmitPass()` — after both pre-RA and post-RA scheduling. The
scheduler models load latencies through DAG edges, not through explicit wait
instructions.

ALU operations (VALU, SALU) are synchronous and do not require wait
instructions.

---

## Appendix I: Memory Clauses

*AMDGPU-specific. This topic needs further exploration — the interaction
between memory clauses and load clustering (Appendix C/D) is not fully
understood.*

A **memory clause** (or "soft clause") is a hardware optimization where a
sequence of adjacent same-type memory instructions (all VMEM or all SMEM)
are issued as a batch without the hardware checking for completion between them.
All loads in the clause are in-flight simultaneously, so their latencies
overlap.

Clause formation is **positional** — the hardware looks at adjacency in the
instruction stream, not at data dependencies. If a non-memory instruction
appears between two loads, the hardware breaks the clause. This is one reason
load clustering matters: it ensures loads end up adjacent so the hardware can
batch them.

### Clause-breaking register conflicts (XNACK)

When XNACK is enabled (a hardware mode for retrying memory accesses on page
faults), an additional constraint applies: if a load in a clause writes its
result to a register that a later load in the same clause uses as a pointer,
the hardware must break the clause (because it may need the pointer again for
a retry).

`SIFormMemoryClauses` prevents this by adding `implicit-def early-clobber`
operands to instructions within the clause. This tells the register allocator
that the pointer register is "in use" throughout the clause, forcing the
allocator to assign the pointer and load results to different physical
registers.

**Note:** The exact relationship between scheduler-level load clustering
(which adds DAG edges to encourage adjacency) and `SIFormMemoryClauses`
(which constrains register allocation to preserve clauses) deserves further
investigation.

---

## Appendix J: Post-Scheduling Passes

*AMDGPU-specific.*

The following passes run after all pre-RA scheduling is complete. They are
inserted after whichever scheduler pass runs last
(`MachineScheduler`, `MachineSchedulerOptSched`, or
`MachineSchedulerHierarchical`).

| Pass | What it does |
|------|-------------|
| **SIWholeQuadMode** | Inserts exec mask save/restore instructions (`S_WQM`, `S_AND_SAVEEXEC`, etc.) for pixel shader derivative computations. These modify the EXEC register, which all VGPR instructions implicitly depend on, making them scheduling barriers. |
| **SIPreAllocateWWMRegs** | Pre-allocates physical registers for virtual registers used in whole wavefront mode (WWM) regions. Modifies register allocation state but does not insert instructions. |
| **SIOptimizeExecMaskingPreRA** | Peephole-optimizes exec mask handling by folding/removing redundant `V_CNDMASK` + `V_CMP` + `S_AND` sequences into `S_ANDN2`. May reduce the number of exec-manipulating instructions. |
| **SIFormMemoryClauses** | Adds `implicit-def early-clobber` operands to instructions in memory clause sequences, preventing the register allocator from creating clause-breaking register conflicts. Only relevant when XNACK is enabled. See [Appendix I](#appendix-i-memory-clauses). |

---

## Appendix K: The -misched Registry

The `-misched` command-line option (`-mllvm -misched=<name>`) overrides which
scheduler the `MachineScheduler` pass uses. Schedulers register via
`MachineSchedRegistry`:

```cpp
static MachineSchedRegistry
    GCNMaxOccupancySchedRegistry("gcn-max-occupancy",
                                 "Run GCN scheduler to maximize occupancy",
                                 createGCNMaxOccupancyMachineScheduler);
```

AMDGPU-registered schedulers include `si`, `gcn-max-occupancy`, `gcn-max-ilp`,
`gcn-iterative-max-occupancy-experimental`, `gcn-iterative-minreg`,
`gcn-iterative-ilp`, `optsched`, and `gcn-optsched`.

**Important:** `-misched` only affects the single `MachineScheduler` pass. It
swaps out which `ScheduleDAGInstrs` that pass creates. It does NOT affect
second-pass schedulers like `MachineSchedulerOptSched` or
`MachineSchedulerHierarchical`.

This means that if OptSched is configured via `misched.txt` AND selected via
`-misched=gcn-optsched`, OptSched could run twice — once inside the
`MachineScheduler` pass and once as its own `MachineSchedulerOptSched` pass.
Even without `-misched`, the config-based setup may already run OptSched in
both passes depending on how `createMachineScheduler` is configured.

---

## Appendix L: SIScheduleDAGMI (Older Hierarchical Scheduler)

*AMDGPU-specific. This section is based on reading the source code
(`SIMachineScheduler.h/cpp`) but has not been deeply verified. Further
exploration is needed to fully understand the coloring heuristics and
performance tradeoffs.*

`SIScheduleDAGMI` is an older AMDGPU scheduler that uses a two-level
hierarchical approach. It is still available via `-misched=si` but is no
longer the default (replaced by `GCNMaxOccupancySchedStrategy`).

### Approach

**Level 1 — Partition instructions into blocks:**
`SIScheduleBlockCreator` assigns a "color" to each SUnit, grouping them into
`SIScheduleBlock` objects. The coloring algorithm is organized around
high-latency instructions (e.g., texture fetches):
- High-latency instructions get their own reserved colors
- Other instructions are colored based on their dependency relationships to
  the high-latency groups
- Various merging/splitting heuristics refine the groups

Three block creation variants:
- `LatenciesAlone`: each high-latency instruction = its own block
- `LatenciesGrouped`: nearby high-latency instructions grouped together
- `LatenciesAlonePlusConsecutive`: alone + forced consecutive ordering

**Level 2 — Schedule at two granularities:**
1. **Inter-block scheduling** (`SIScheduleBlockScheduler`): A list scheduler
   over blocks (not instructions). Picks blocks based on register pressure,
   latency, and high-latency block positions.
2. **Intra-block scheduling** (`SIScheduleBlock::schedule()`): Within each
   block, a local list scheduler with its own `pickNode()` and register
   pressure tracking.

### Trial-and-error

The scheduler tries multiple combinations of block creation and scheduling
variants, evaluates each by VGPR usage, and keeps the best result. It backs
up the DAG state between trials. If VGPR usage exceeds 180, it tries
additional variants; if it exceeds 200, even more.

### Post-processing

After choosing the best variant, `moveLowLatencies()` moves low-latency
instructions (LDS accesses) closer to their users.

### Key observations

- Operates on a **single region** (overrides `schedule()`, not
  `finalizeSchedule()`) — no cross-region view.
- The block-based decomposition is a practical implementation of hierarchical
  scheduling, relevant to the design of `ScheduleDAGHierarchicalScheduler`.
- The trial-and-error approach is brute-force but effective — it explores
  the space of decompositions rather than relying on a single heuristic.

---

## Appendix M: Instruction Latency, Throughput, and Schedule Length

This appendix explains how LLVM models instruction timing — how long
instructions take, when dependent instructions can execute, and how
fast a sequence of instructions can run. It starts with LLVM's general
infrastructure, then covers AMDGPU specifics, then OptSched's approach.

### M.1 What "latency" means

When a GPU executes an instruction like `V_ADD_F32 %3, %1, %2`, the
result (`%3`) is not available instantaneously. There is a delay
between when the instruction starts executing and when its output can
be read by another instruction. This delay is the instruction's
**latency**.

If instruction B depends on instruction A's result (a data dependency),
B cannot begin executing until A's latency has elapsed. Scheduling B
too early would read stale data. Scheduling B too late wastes cycles.
The scheduler's job is to fill the gap between A and B with other
useful work.

A separate concept is **throughput** — how quickly a pipeline can
accept new instructions. A pipeline might accept a new instruction
every cycle (throughput = 1 per cycle) even though each instruction
takes many cycles to produce its result (latency >> 1). This is called
**pipelining**. A memory load might have latency = 80 cycles but
throughput = 1 per cycle: you can issue a new load every cycle, but
you wait 80 cycles for each result.

### M.2 The .td scheduling model

LLVM defines instruction timing in `.td` (TableGen) files. These are
declarative descriptions that get compiled into C++ data tables at
build time. The system has several pieces that build on each other.

#### Piece 1: ProcResource — defining pipelines

A `ProcResource` declares that a processor has a hardware pipeline:

```
def HWSALU : ProcResource<1>;   // One scalar ALU pipeline
def HWVALU : ProcResource<1>;   // One vector ALU pipeline
def HWVMEM : ProcResource<1>;   // One vector memory pipeline
def HWLGKM : ProcResource<1>;   // One LDS/scalar-memory pipeline
```

The `<1>` means there is one instance of this pipeline. A processor
with two independent VALU pipelines would say `ProcResource<2>`.

These just declare that the pipelines exist. They carry no timing
information.

**File:** `lib/Target/AMDGPU/SISchedule.td` (lines 94–121)

#### Piece 2: SchedWrite and SchedRead — naming output and input types

A `SchedWrite` is a tag for a category of instruction **output**.
A `SchedRead` is a tag for a category of instruction **input**.
Neither carries timing information by itself — they are just names:

```
// Output types (what kind of result the instruction produces):
def WriteSALU     : SchedWrite;   // "scalar ALU result"
def Write32Bit    : SchedWrite;   // "32-bit vector ALU result"
def WriteVMEM     : SchedWrite;   // "vector memory result"
def WriteFloatFMA : SchedWrite;   // "floating-point FMA result"

// Input types (what kind of operand the instruction reads):
def MIVGPRRead    : SchedRead;    // "reads a VGPR"
def MIMFMARead    : SchedRead;    // "reads an MFMA accumulator"
```

Think of these as enum values. They exist so that instructions,
timing tables, and latency adjustments can all refer to the same
names.

**File:** `lib/Target/AMDGPU/SISchedule.td` (lines 19–66)

#### Piece 3: SchedRW on instructions — tagging instructions

Each instruction definition has a `SchedRW` field that holds a list
of SchedWrite and SchedRead tags. Instructions inherit their `SchedRW`
from their base class in the `.td` class hierarchy. For example:

```
// SOPInstructions.td — SOP1_Pseudo is the base class for
// single-operand scalar ALU instructions:
class SOP1_Pseudo<...> : SOP_Pseudo<...> {
    let SchedRW = [WriteSALU];      // Set on the base class
    // ... other fields ...
}

// Individual instructions inherit from the base class:
def S_MOV_B32 : SOP1_Pseudo<"s_mov_b32", ...>;
def S_NOT_B32 : SOP1_Pseudo<"s_not_b32", ...>;
// These inherit SchedRW = [WriteSALU] automatically.
```

Similarly for VALU instructions:

```
// SIInstrFormats.td — base class for vector ALU instructions:
class VOP_Pseudo<...> {
    let SchedRW = [Write32Bit];     // default for VALU
}
```

Specific instructions or groups can override with different tags:

```
// VOP3Instructions.td — FMA instructions override the default:
let SchedRW = [WriteFloatFMA, WriteSALU] in {
    // V_FMA_F32 gets [WriteFloatFMA, WriteSALU] instead of [Write32Bit]
}
```

An instruction can have multiple tags if it has multiple outputs
going to different pipelines. `V_FMA_F32` has `[WriteFloatFMA,
WriteSALU]` because it produces a VALU result (WriteFloatFMA) AND
writes a scalar status flag (WriteSALU). Each output gets its own
tag, and dependency edges on each output get the latency of that
output's tag.

Most instructions have just one SchedWrite tag.

**Files:** `lib/Target/AMDGPU/SOPInstructions.td` (line 49),
`lib/Target/AMDGPU/SIInstrFormats.td` (line 242),
`lib/Target/AMDGPU/VOP3Instructions.td`

#### Piece 4: WriteRes — connecting output tags to pipelines and latencies

A `WriteRes` gives a SchedWrite tag its meaning for a specific
processor. It says: "on this processor, an output tagged X uses
pipeline Y, and the result is ready after Z cycles":

```
WriteRes<WriteSALU, [HWSALU]> { let Latency = 1; }
// "WriteSALU uses the HWSALU pipeline, result ready in 1 cycle"

WriteRes<WriteVMEM, [HWVMEM]> { let Latency = 80; }
// "WriteVMEM uses the HWVMEM pipeline, result ready in 80 cycles"
```

Without a WriteRes, a SchedWrite tag is meaningless. Different
processors can have different WriteRes entries for the same tag —
that is how `WriteFloatFMA` means 1 cycle on one processor but
16 cycles on another.

**Convenience wrappers.** The AMDGPU `.td` files define shorthand
to reduce typing. These are NOT new concepts — they produce WriteRes
entries with less syntax:

```
// HWWriteRes: bundles resource + latency into one line
class HWWriteRes<SchedWrite write, list<ProcResourceKind> resources,
                 int latency>
    : WriteRes<write, resources> {
  let Latency = latency;
}

// So this:
def : HWWriteRes<WriteSALU, [HWSALU], 1>;
// produces exactly:
// WriteRes<WriteSALU, [HWSALU]> { let Latency = 1; }

// HWVALUWriteRes: even shorter — hardcodes HWVALU as the pipeline
class HWVALUWriteRes<SchedWrite write, int latency>
    : HWWriteRes<write, [HWVALU], latency>;

// So this:
def : HWVALUWriteRes<Write32Bit, 1>;
// produces exactly:
// WriteRes<Write32Bit, [HWVALU]> { let Latency = 1; }
```

Whenever you see `HWWriteRes` or `HWVALUWriteRes` in the `.td` files,
mentally replace them with the WriteRes they produce.

**File:** `lib/Target/AMDGPU/SISchedule.td` (lines 123–129 for the
wrappers, lines 141–228 for the per-model timing tables)

#### Piece 5: ReadAdvance — adjusting latency for specific consumers

A `ReadAdvance` modifies the latency that a consumer sees, based on
the consumer's SchedRead tag. It is the input-side counterpart of
WriteRes.

WriteRes says: "producing this output takes N cycles." ReadAdvance
says: "consuming this input takes M extra (or fewer) cycles on top
of the producer's latency."

```
def : ReadAdvance<MIVGPRRead, -2>;
```

This says: any consumer input tagged `MIVGPRRead` adds 2 extra
cycles of latency beyond what the producer's WriteRes specifies.
(The sign convention is: positive = consumer can read early,
reducing latency. Negative = consumer needs extra time, increasing
latency. AMDGPU's value is -2, meaning 2 extra cycles.)

**Example:** A producer tagged `Write32Bit` has WriteRes latency = 1.
A consumer tagged `MIVGPRRead` has ReadAdvance = -2. The effective
edge latency is `1 - (-2) = 3` cycles. The consumer must wait 3
cycles after the producer, not 1.

This models the cost of forwarding data between pipeline stages.
Not all consumers have a ReadAdvance — if none is defined, the
consumer sees the producer's base latency unchanged.

**File:** `lib/Target/AMDGPU/SISchedule.td` (line 170)

#### Piece 6: SchedMachineModel — top-level processor description

A `SchedMachineModel` defines processor-wide characteristics:

```
class SISchedMachineModel : SchedMachineModel {
  let IssueWidth = 1;          // Max instructions issued per cycle
  let MicroOpBufferSize = 1;   // Pipeline buffering depth
  let PostRAScheduler = 1;     // Enable post-RA scheduling
  let CompleteModel = 1;       // All instructions have scheduling data
}
```

`IssueWidth` is the most important field. It says how many
instructions a single thread/wavefront can issue per cycle. For all
AMDGPU targets, this is 1.

Specific processor models inherit from this base. The WriteRes
entries for each model are defined inside a `let SchedModel = X`
block, which scopes them to that model:

```
def SIQuarterSpeedModel : SISchedMachineModel;

let SchedModel = SIQuarterSpeedModel in {
    defm : SICommonWriteRes;                   // shared entries
    def : HWVALUWriteRes<WriteFloatFMA, 16>;   // FMA = 16 cycles
    def : HWVALUWriteRes<WriteDouble,   16>;   // FP64 = 16 cycles
    // ...
}
```

Then each processor chip is assigned to a model:

```
// GCNProcessors.td:
def : ProcessorModel<"gfx906", SIQuarterSpeedModel, ...>;
def : ProcessorModel<"gfx600", SIFullSpeedModel, ...>;
```

**Files:** `lib/Target/AMDGPU/SISchedule.td` (lines 72–92),
`lib/Target/AMDGPU/GCNProcessors.td`

#### Putting the pieces together

```
ProcResource      →  "these pipelines exist"
SchedWrite        →  "these output type names exist"
SchedRead         →  "these input type names exist"
SchedRW           →  "this instruction has these output/input types"
                     (inherited from base class in .td hierarchy)
WriteRes          →  "output type X uses pipeline Y, latency Z"
                     (per processor model)
ReadAdvance       →  "input type X adjusts latency by N cycles"
                     (per processor model)
SchedMachineModel →  "this processor issues 1 instruction per cycle"
ProcessorModel    →  "gfx906 uses SIQuarterSpeedModel"
```

At build time, TableGen compiles all of this into C++ lookup tables
(`AMDGPUGenSubtargetInfo.inc`). At scheduling time, the scheduler
calls `computeOperandLatency()` which looks up the tables, applies
the ReadAdvance adjustment, and returns the effective latency for
a specific producer-output → consumer-input edge.

### M.3 Pipelining and ResourceCycles

Most instructions are **pipelined**: they occupy their pipeline for
just 1 cycle, even if their result takes many cycles. A VMEM load has
latency 80 but uses the VMEM pipeline for only 1 cycle — you can
issue another load the very next cycle, even though the first result
hasn't arrived yet.

The `ResourceCycles` field on a WriteRes controls how long an
instruction occupies its pipeline. By default it is 1 (pipelined).

The exception is MFMA matrix operations:

```
let ResourceCycles = [16] in
def : HWWriteRes<Write16PassMAI, [HWXDL], 16>;
```

This MFMA instruction occupies the HWXDL pipeline for 16 cycles AND
has 16 cycles of result latency. While it's running, no other MFMA
can start on the XDL pipeline.

Compare with a VMEM load:

```
def : HWWriteRes<WriteVMEM, [HWVMEM], 80>;
// ResourceCycles defaults to [1]
// Pipeline busy for 1 cycle, result ready in 80 cycles
```

The VMEM load is the key example of ResourceCycles ≠ Latency. The
pipeline is busy for only 1 cycle (pipelined — fire and forget), but
the result takes 80 cycles to come back from memory. You can issue
another load the very next cycle, but you cannot read the result for
80 cycles.

The scheduler treats pipelined vs non-pipelined resources differently
based on the pipeline's `BufferSize` (see M.5).

### M.4 How latency reaches the DAG

During scheduling, latency values from the timing tables are attached
to SDep edges in the dependency DAG. The flow:

```
Step 1: initSUnits()
  (ScheduleDAGInstrs.cpp, line 580)

  For each MachineInstr, create an SUnit and set its overall latency:

      SU->Latency = SchedModel.computeInstrLatency(SU->getInstr())

  This looks up the instruction's SchedWrite tag, finds the WriteRes
  for the current processor model, and returns the latency. For
  instructions with multiple SchedWrite tags, it returns the maximum.


Step 2: buildSchedGraph()
  (ScheduleDAGInstrs.cpp, lines 273, 442)

  For each data dependency edge (producer → consumer), compute the
  operand-specific latency:

      SDep Dep(ProducerSU, SDep::Data, Reg)
      Dep.setLatency(
          SchedModel.computeOperandLatency(
              DefMI, DefOpIdx, UseMI, UseOpIdx))

  computeOperandLatency() (TargetSchedule.cpp, line 168) does:

      1. Find which SchedWrite tag corresponds to this specific
         output operand of the producer
      2. Look up that tag's WriteRes → get the base latency
      3. Find which SchedRead tag (if any) corresponds to this
         specific input operand of the consumer
      4. Look up that SchedRead's ReadAdvance (if any) → get the
         adjustment
      5. Return: base_latency - read_advance

  For example, if the producer is tagged Write32Bit (base latency 1)
  and the consumer's input is tagged MIVGPRRead (ReadAdvance -2),
  the edge latency is 1 - (-2) = 3 cycles.

  If the consumer has no ReadAdvance for this input, the edge latency
  is just the base latency from the WriteRes.


Step 3: adjustSchedDependency()
  (Called immediately after setLatency)

      ST.adjustSchedDependency(
          ProducerSU, DefOpIdx, ConsumerSU, UseOpIdx, Dep)

  This is a target-specific hook that can override the latency after
  the general computation. The AMDGPU version (AMDGPUSubtarget.cpp,
  line 832) adjusts latencies for bundled instructions and fixes VCC
  implicit operand edge cases. For non-bundled instructions (the
  common case in pre-RA scheduling), it doesn't change anything.
```

**Two kinds of latency on an SUnit:**

- `SDep::getLatency()` is the edge-specific latency between one
  producer output and one consumer input. This is what the scheduler
  uses to compute ready cycles and schedule length — the critical
  path is the longest chain of these edge latencies through the DAG.
- `SU->Latency` is the instruction's overall latency (max across all
  outputs). It's a convenience field, mainly used for one edge case:
  when a high-latency instruction has no successors in the region
  (e.g., a VMEM load whose result is only used in a later region),
  LLVM adds an artificial edge to ExitSU with `latency =
  SU->Latency - 1` so the scheduler knows the result won't be
  available for a while (ScheduleDAGInstrs.cpp, line 877).

### M.5 The scheduling loop and cycle tracking

The scheduler maintains a cycle counter (`CurrCycle`). When it picks
an instruction to schedule, it checks: can this instruction execute
now, or does it have to wait?

The core logic is in `SchedBoundary::bumpNode()`
(`MachineScheduler.cpp`, line 2913):

```
bumpNode(SUnit *SU):
    // 1. When is this instruction ready?
    //    ReadyCycle was computed from predecessor edges:
    //    for each already-scheduled predecessor P with edge latency L,
    //      ReadyCycle = max(ReadyCycle, P.scheduled_cycle + L)
    ReadyCycle = SU->TopReadyCycle

    // 2. Do we need to stall?
    NextCycle = CurrCycle
    if ReadyCycle > CurrCycle:
        NextCycle = ReadyCycle    // can't execute yet — wait

    // 3. Check pipeline resource availability (see below)
    for each pipeline resource this instruction uses:
        ResourceAvailCycle = countResource(resource, cycles)
        NextCycle = max(NextCycle, ResourceAvailCycle)

    // 4. Advance the cycle if needed
    if NextCycle > CurrCycle:
        bumpCycle(NextCycle)

    // 5. Issue width check
    CurrMOps += 1
    while CurrMOps >= IssueWidth:    // hit the per-cycle limit
        bumpCycle(++NextCycle)        // move to next cycle
```

For gfx906 with `IssueWidth = 1`, step 5 fires after every single
instruction — the cycle advances after each one. The scheduling
decisions come from step 2: the gap between when the scheduler wants
to place an instruction and when it's actually ready is a **stall**
(wasted cycle).

**Pipeline resource tracking (step 3):**

Each pipeline resource (HWVALU, HWSALU, etc.) has a `BufferSize` that
controls how the scheduler models its availability:

**BufferSize = 1** (HWVALU, HWSALU, HWVMEM, HWLGKM, etc.):
The pipeline is pipelined and buffered. The scheduler tracks
producer/consumer stalls via latency edges but does not track whether
the pipeline itself is busy. It assumes the pipeline always accepts
new work. For gfx906 with IssueWidth=1, this creates no additional
constraints — only one instruction issues per cycle anyway, so the
pipeline can never be double-booked.

**BufferSize = 0** (HWXDL — the MFMA matrix unit):
The pipeline is unbuffered. The scheduler tracks exactly when the
resource becomes free using a `ReservedCycles[]` array. If an MFMA
reserves XDL for 16 cycles (via `ResourceCycles = [16]`), the next
MFMA must wait until those cycles pass. Other instructions on other
pipelines can still issue during that time.

**Where this code lives:**

| What | Where |
|------|-------|
| `bumpNode()` | `lib/CodeGen/MachineScheduler.cpp:2913` |
| `bumpCycle()` | `lib/CodeGen/MachineScheduler.cpp:2854` |
| `countResource()` | `lib/CodeGen/MachineScheduler.cpp:2877` |
| `SchedBoundary` class | `include/llvm/CodeGen/MachineScheduler.h` |
| `computeOperandLatency()` | `lib/CodeGen/TargetSchedule.cpp:168` |
| `computeInstrLatency()` | `lib/CodeGen/TargetSchedule.cpp:257` |
| `adjustSchedDependency()` | `lib/Target/AMDGPU/AMDGPUSubtarget.cpp:832` |

### M.6 AMDGPU-Specific Details

**gfx906 complete latency table.**

gfx906 uses `SIQuarterSpeedModel`. The full table combines the shared
`SICommonWriteRes` entries with the quarter-speed overrides. All
`ResourceCycles` are 1 (pipelined) unless noted.

**Non-VALU instructions:**

| Tag | Pipeline | Latency | Notes |
|-----|----------|---------|-------|
| `WriteSALU` | HWSALU | 1 | Scalar ALU |
| `WriteSMEM` | HWLGKM | 5 | Scalar memory |
| `WriteLDS` | HWLGKM | 5 | LDS (shared memory). "Can be between 2 and 64" |
| `WriteVMEM` | HWVMEM | 80 | Global memory |
| `WriteBranch` | HWBranch | 8 | Branch |
| `WriteExport` | HWExport | 4 | Pixel export (graphics) |
| `WriteBarrier` | HWBranch | 500 | Barrier. "XXX: Guessed ???" |

**VALU instructions (shared across models):**

| Tag | Pipeline | Latency | Example instructions |
|-----|----------|---------|---------------------|
| `Write32Bit` | HWVALU | 1 | V_ADD_F32, V_MUL_F32, V_SUB_F32, bitwise ops |
| `WriteFloatCvt` | HWVALU | 4 | Float conversion (not FP64) |
| `WriteTrans32` | HWVALU | 4 | V_RCP_F32, V_SQRT_F32, V_SIN_F32, V_COS_F32 |
| `WriteQuarterRate32` | HWVALU | 4 | Other quarter-rate 32-bit ops |

**VALU instructions (quarter-speed specific):**

| Tag | Pipeline | Latency | Example instructions |
|-----|----------|---------|---------------------|
| `Write64Bit` | HWVALU | 2 | 64-bit moves, shifts |
| `WriteIntMul` | HWVALU | 4 | V_MUL_LO_U32, V_MUL_HI_U32 |
| `WriteFloatFMA` | HWVALU | 16 | V_FMA_F32, V_FMAC_F32. See provenance note below |
| `WriteDouble` | HWVALU | 16 | V_MUL_F64 |
| `WriteDoubleAdd` | HWVALU | 8 | V_ADD_F64 |
| `WriteDoubleCvt` | HWVALU | 4 | FP64 conversion |
| `WriteTrans64` | HWVALU | 16 | V_RCP_F64, V_SQRT_F64 |

**MFMA instructions (gfx908+, not available on gfx906 hardware):**

| Tag | Pipeline | Latency | ResourceCycles | Instructions |
|-----|----------|---------|----------------|--------------|
| `Write2PassMAI` | HWXDL | 2 | **2** | V_MFMA 4x4 |
| `Write4PassMAI` | HWXDL | 4 | **4** | (gfx940+ only) |
| `Write8PassMAI` | HWXDL | 8 | **8** | V_MFMA 16x16 |
| `Write16PassMAI` | HWXDL | 16 | **16** | V_MFMA 32x32 |
| `Write4PassDGEMM` | HWVALU | 4 | 1 | FP64 matrix multiply (gfx940+) |
| `Write8PassDGEMM` | HWVALU | 16 | 1 | FP64 matrix multiply (gfx940+) |

MFMA instructions are the only ones with `ResourceCycles > 1`,
meaning they block the HWXDL pipeline for their full duration
(`BufferSize = 0`).

**Special:**

| Tag | Pipeline | Latency | Notes |
|-----|----------|---------|-------|
| `WriteCopy` | varies | varies | COPY/PRED_COPY — dispatches to Write32Bit, Write64Bit, or WriteSALU based on operand type |

**ReadAdvances (latency adjustments on the consumer side):**

| Tag | Adjustment | Meaning |
|-----|-----------|---------|
| `MIVGPRRead` | -2 (adds 2 cycles) | Any consumer reading a VGPR result |
| `MIMFMARead` | -4 (adds 4 cycles) | MFMA accumulator reads |

All pipelines have `NumUnits = 1` (one instance each).
All pipelines except HWXDL have `BufferSize = 1` (pipelined).
HWXDL has `BufferSize = 0` (unbuffered).
All pipelines except HWXDL have `BufferSize = 1` (pipelined).
HWXDL has `BufferSize = 0` (unbuffered — blocks for full duration).

ReadAdvance: `MIVGPRRead` adds 2 extra cycles to any edge where a
consumer reads a VGPR result. `MIMFMARead` adds 4 extra cycles for
MFMA accumulator reads.

**Quarter-speed vs full-speed.** Both models share all the same
base definitions (`SICommonWriteRes` — SALU, VMEM, LDS, etc.) and
the same `IssueWidth = 1`. They override the same seven VALU tags.
Of those seven, only four actually differ:

| Tag | What it covers | FullSpeed | QuarterSpeed |
|-----|----------------|-----------|--------------|
| `WriteFloatFMA` | FP32 fused multiply-add | 1 | 16 |
| `WriteDouble` | FP64 multiply | 4 | 16 |
| `WriteDoubleAdd` | FP64 add | 2 | 8 |
| `WriteTrans64` | FP64 transcendental | 4 | 16 |
| `Write64Bit` | 64-bit moves/shifts | 2 | 2 (same) |
| `WriteIntMul` | Integer multiply | 4 | 4 (same) |
| `WriteDoubleCvt` | FP64 conversion | 4 | 4 (same) |

The origin of the "quarter speed" name is unclear. The FP64 ops
(WriteDouble, WriteDoubleAdd, WriteTrans64) are consistently 4x
slower, but WriteFloatFMA goes from 1 to 16 (16x). These may
reflect different hardware generations with different pipeline
designs rather than a simple speed scaling factor.

Which model a chip uses is assigned in `GCNProcessors.td`:
gfx600 (Tahiti) → FullSpeed, gfx906 (Radeon VII) → QuarterSpeed.

**Provenance and accuracy of these numbers.** The scheduling model
was originally written by Tom Stellard (AMD) in January 2015
(commit `ae38f30d7b79`, "R600/SI: Define a schedule model"), for
early GCN (Southern Islands) hardware. The commit message says
"The schedule model is not complete yet, and could be improved,"
and the `.td` comment says "The latency numbers are taken from AMD
Accelerated Parallel Processing guide. They may not be accurate."
Many original values were rough guesses (Branch=100, VMEM=450,
SMEM=10, all with "XXX: Guessed ???" comments). Some were improved
in a 2016 follow-up by Stellard (commit `1d5e6d4bdcc3`: Branch→8,
VMEM→80, SMEM→5), but the VALU latencies were never revisited.

The original `.td` file also contains the comment "The latency values
are 1 / (operations / cycle) / 4." This appears to describe how the
latency numbers were derived from the AMD guide's throughput figures,
but the formula does not produce the actual numbers in the file
(e.g., full-rate operations at 1 op/cycle would give 0.25, not 1).
The formula's meaning is unclear.

When gfx906 was added in April 2018 (commit `0084adc51656` by Matt
Arsenault, AMD), it was assigned to `SIQuarterSpeedModel` with a
single line — no gfx906-specific latency tuning or verification.
The latency numbers gfx906 uses today are inherited unchanged from
the 2015 model written for earlier hardware.

A contemporaneous external source ([Michal Drobot, "Low Level
Optimizations for GCN", Digital Dragons 2014](https://michaldrobot.com/wp-content/uploads/2014/05/gcn_alu_opt_digitaldragons2014.pdf))
lists FP32 arithmetic as full-rate on the same era of GCN hardware,
with only int32 multiply, FP64, and transcendentals at quarter-rate
— FP32 FMA is not in the quarter-rate category. This contradicts
the LLVM model's `WriteFloatFMA = 16` for `SIQuarterSpeedModel`,
despite both sources describing the same hardware generation.
Neither has been verified for gfx906 (released 2019), which
inherited the LLVM model's numbers unchanged. Hardware measurement
on gfx906 is needed to resolve this.

**IssueWidth = 1.** All AMDGPU models set `IssueWidth = 1`. The
hardware has independent SALU, VALU, VMEM, LDS pipelines that can
have instructions in flight simultaneously, but a single wavefront
issues one instruction per cycle from its instruction stream. The
parallelism across pipelines comes from the hardware interleaving
different wavefronts, not from one wavefront issuing to multiple
pipelines at once.

**MFMA and HWXDL.** MFMA (Matrix Fused Multiply-Add) instructions
perform small matrix multiplies on the XDL (eXtended Dot-product
Logic) pipeline. Available on gfx908+ (MI-series accelerators).
LLVM defines MFMA entries for gfx906's model because gfx906 serves
as a base for gfx908, but gfx906 hardware does not have an XDL unit.

### M.7 Schedule Length: Post-Hoc Computation

LLVM's GCN scheduler has a simpler way to compute the schedule length
of a completed instruction ordering. This is
`GCNSchedStage::getScheduleMetrics()` (`GCNSchedStrategy.cpp`,
line 1028):

```
getScheduleMetrics(schedule):
    CurrCycle = 0
    ReadyCycles = {}    // map: SUnit → cycle it was scheduled at

    for each SUnit in schedule order:
        // When is this instruction ready?
        ReadyCycle = CurrCycle
        for each predecessor P with a data dependency edge:
            ReadyCycle = max(ReadyCycle,
                            ReadyCycles[P] + edge_latency)

        // Record stall (bubble) if we had to wait
        Bubbles += ReadyCycle - CurrCycle

        // Schedule it
        ReadyCycles[this_SUnit] = ReadyCycle
        CurrCycle = ReadyCycle + 1    // IssueWidth = 1

    return ScheduleMetrics(CurrCycle, Bubbles)
```

This only considers data dependency latencies. It does not model
resource contention or hazards. For gfx906 with `IssueWidth = 1` and
all resources pipelined (`BufferSize = 1`), this produces the same
result as the full `bumpNode()` model for code without MFMA.

**When is `getScheduleMetrics()` used vs `bumpNode()`?**

`bumpNode()` runs **during** scheduling — inside the pick-node loop
of `GenericScheduler`. Every time the scheduler places an instruction,
`bumpNode()` updates the cycle counter, resource state, and hazard
recognizer. It drives real-time scheduling decisions.

`getScheduleMetrics()` runs **after** scheduling — in the GCN
rescheduling stages. The GCN scheduler sometimes schedules a region,
then checks whether the result is better than the original ordering.
`getScheduleMetrics()` evaluates a finished schedule by walking the
instruction order and computing length + bubbles. If the new schedule
is worse, the scheduler reverts:

```
MBefore = getScheduleMetrics(original_order)
MAfter  = getScheduleMetrics(new_order)
if new_is_worse:
    revert to original
```

**Where this code lives:**

| What | Where |
|------|-------|
| `getScheduleMetrics()` | `lib/Target/AMDGPU/GCNSchedStrategy.cpp:1028` |
| `ScheduleMetrics` struct | `lib/Target/AMDGPU/GCNSchedStrategy.h` |
| `shouldRevertScheduling()` | `lib/Target/AMDGPU/GCNSchedStrategy.cpp:1100` |

### M.8 OptSched's Latency Model

OptSched has its own machine model config file
(`optsched-cfg/machine_model.cfg`), separate from LLVM's `.td` model.

**Config format:**

```
MODEL_NAME: Simple
ISSUE_RATE: 1
ISSUE_TYPE_COUNT: 1
Default 1

DEP_LATENCY_ANTI: 0
DEP_LATENCY_OUTPUT: 1
DEP_LATENCY_OTHER: 1
```

`ISSUE_RATE` is analogous to LLVM's `IssueWidth`. The config format
supports multiple issue types with per-type slot counts:

```
ISSUE_RATE: 4
ISSUE_TYPE_COUNT: 3
VALU 2
SALU 1
VMEM 1
```

This would model 4 instructions per cycle with 2 VALU slots, 1 SALU
slot, and 1 VMEM slot. However, the actual config used for AMDGPU
sets `ISSUE_RATE: 1` with a single "Default" issue type. There are no
alternate config files in the repository for GCN. The multi-issue
infrastructure exists (from OptSched's origins as a CPU scheduler)
but is not exercised for AMDGPU.

**Where latency comes from.** OptSched has three latency modes,
configured via `LATENCY_PRECISION` in `sched.ini`
(`OptSchedDDGWrapperBasic.cpp`, line 449):

| Setting | Mode | Source |
|---------|------|--------|
| `PRECISE` / `FILE` | `LTP_PRECISE` | Per-opcode latencies from `machine_model.cfg` |
| `LLVM` / `ROUGH` | `LTP_ROUGH` | LLVM's `SDep::getLatency()` — same values as LLVM's `.td` model |
| `UNIT` / `UNITY` | `LTP_UNITY` | All latencies = 1 (ignores ILP, schedules only for register pressure) |

The default config sets `LATENCY_PRECISION LLVM`. The `PRECISE` mode
would read from `machine_model.cfg`, but that file has
`INST_TYPE_COUNT: 0` — no per-opcode latencies are defined. So for
GCN, **OptSched uses the exact same latency numbers as LLVM**,
including any potentially inaccurate values like `WriteFloatFMA = 16`.

**Ready cycle computation** (`sched_basic_data.hip.cpp`, line 683):

```
PrdcsrSchduld(predecessorIndex, predecessorCycle):
    // Contribution from this predecessor
    rdyCyclePerPrdcsr_[predecessorIndex] =
        predecessorCycle + latencyFromThisPredecessor

    // Ready cycle = latest of all predecessors
    minRdyCycle_ = max(minRdyCycle_,
                       rdyCyclePerPrdcsr_[predecessorIndex])

    // Track how many predecessors remain
    unschduldPrdcsrCnt_--
    return (unschduldPrdcsrCnt_ == 0)  // true when all done
```

An instruction can enter the ready list when all predecessors are
scheduled AND the current cycle >= `minRdyCycle_`.

**Scheduling and NOPs.** OptSched's list scheduler always picks an
instruction from the ready list if one is available. It never
voluntarily delays a ready instruction.

The enumerator (branch-and-bound) is different. At each time step,
it branches on every ready instruction AND on a NOP (empty slot).
This means it explores orderings where a ready instruction is
deliberately delayed — for example, waiting a few cycles so that
two independent results become available at the same time, enabling
a better ordering later. This is how the enumerator can find
schedules that a greedy list scheduler would miss.

**Cycles and slots.** OptSched maps cycles and slots to a single
linear "time" dimension: `time = cycle * issueRate + slot + 1`. With
`ISSUE_RATE = 1`, this simplifies to `time = cycle + 1`.

**Schedule length as a search bound.** The enumerator uses a target
schedule length for pruning. It tries to find a feasible schedule of
that length. If an instruction would miss its deadline (can't fit
within the target length), that branch is pruned.

**File locations:**

| What | Where |
|------|-------|
| Machine model config | `lib/Target/AMDGPU/OptSched/optsched-cfg/machine_model.cfg` |
| Machine model class | `lib/Target/AMDGPU/OptSched/include/opt-sched/Scheduler/machine_model.h` |
| Ready cycle logic | `lib/Target/AMDGPU/OptSched/lib/Scheduler/sched_basic_data.hip.cpp:683` |
| Cycle/slot tracking | `lib/Target/AMDGPU/OptSched/lib/Scheduler/gen_sched.hip.cpp` |
| Enumerator pruning | `lib/Target/AMDGPU/OptSched/lib/Scheduler/enumerator.cpp:607` |

### M.9 Summary: What This Means for Our Scheduler

**Key insight: for `IssueWidth = 1`, ordering and timing are
separable.** With one instruction per cycle and no resource
contention (all pipelines buffered), the schedule length is fully
determined by the instruction ordering. We can:

1. Pick an ordering using whatever algorithm (beam search, BnB, etc.)
2. Compute the timing afterward using `getScheduleMetrics()`-style
   evaluation

The scheduling algorithm doesn't need to track cycles as it builds
the schedule — it just needs to produce orderings, and the latency
tracker evaluates them. This is simpler than OptSched's cycle-by-
cycle approach (which is needed for multi-issue processors where
resource conflicts within a cycle matter).

**Schedule length computation:**

```
For each instruction in the chosen order:
    ready_cycle = max(current_cycle,
                      max(pred_cycle + edge_latency)
                          for all scheduled predecessors)
    this_instruction_cycle = ready_cycle
    current_cycle = ready_cycle + 1     // IssueWidth = 1

Schedule length = current_cycle after last instruction
Bubbles = sum of (ready_cycle - previous_current_cycle) at each step
```

This matches both LLVM's `getScheduleMetrics()` and OptSched's
ready-cycle computation. Edge latencies are already on our
`ScheduleEdge::latency_` (copied from `SDep::getLatency()` during
`BuildFromSUnits`).

**Future extensions:**

- **Multi-issue.** If we target architectures with `IssueWidth > 1`,
  ordering and timing are no longer separable — two instructions
  might share a cycle if they use different pipelines, or conflict
  if they use the same one. We would need cycle/slot tracking like
  OptSched's model.
- **Resource contention.** For MFMA-heavy code, we would need to
  track XDL pipeline reservation (`BufferSize = 0` behavior).

---

## Appendix N: Occupancy

### N.1 What occupancy is

On AMDGPU, multiple wavefronts (waves) can execute on the same
Execution Unit (EU) simultaneously. The hardware switches between
waves to hide latency — when one wave stalls waiting for memory or
a long-latency instruction, another wave can execute. **Occupancy**
is the number of waves per EU. Higher occupancy = more waves = more
latency hiding.

Occupancy is limited by three hardware resources. Each wave needs
registers and LDS, and each EU has a finite supply. If a kernel
uses too many registers or too much LDS, fewer waves can fit.

### N.2 The three occupancy limits

Occupancy is the **minimum** of three independent limits:

```
occupancy = min(
    max_waves_per_EU,           // hardware limit (10 for gfx906)
    occupancy_from_SGPRs,       // based on peak SGPR count
    occupancy_from_VGPRs,       // based on peak VGPR count
    occupancy_from_LDS          // based on LDS usage per workgroup
)
```

Each limit is computed by a separate function on `GCNSubtarget`.
They are combined by `GCNSubtarget::computeOccupancy()`
(`AMDGPUSubtarget.cpp:697`):

```cpp
unsigned GCNSubtarget::computeOccupancy(const Function &F,
                                        unsigned LDSSize,
                                        unsigned NumSGPRs,
                                        unsigned NumVGPRs) const {
  unsigned Occupancy =
      std::min(getMaxWavesPerEU(),
               getOccupancyWithLocalMemSize(LDSSize, F));
  if (NumSGPRs)
    Occupancy = std::min(Occupancy, getOccupancyWithNumSGPRs(NumSGPRs));
  if (NumVGPRs)
    Occupancy = std::min(Occupancy, getOccupancyWithNumVGPRs(NumVGPRs));
  return Occupancy;
}
```

Note: if `NumSGPRs` or `NumVGPRs` is 0, that limit is skipped.
This matters for the initial occupancy computation (see N.4).

#### SGPR occupancy

`GCNSubtarget::getOccupancyWithNumSGPRs()` (`AMDGPUSubtarget.cpp:637`)
uses a threshold table. For gfx906 (VOLCANIC_ISLANDS generation):

| SGPRs used | Max waves |
|------------|-----------|
| ≤ 80       | 10        |
| ≤ 88       | 9         |
| ≤ 100      | 8         |
| > 100      | 7         |

#### VGPR occupancy

`GCNSubtarget::getOccupancyWithNumVGPRs()` (`AMDGPUSubtarget.cpp:663`)
delegates to `AMDGPU::IsaInfo::getNumWavesPerEUWithNumVGPRs()`
(`AMDGPUBaseInfo.cpp:1077`). For gfx906:

- Total VGPRs per EU = 256
- Allocation granularity = 4
- Formula: `min(max(256 / alignTo(NumVGPRs, 4), 1), 10)`

| VGPRs used | Rounded | Max waves |
|------------|---------|-----------|
| 1–24       | 24      | 10        |
| 25–28      | 28      | 9         |
| 29–32      | 32      | 8         |
| 33–36      | 36      | 7         |
| 37–40      | 40      | 6         |
| 41–48      | 48      | 5         |
| 49–64      | 64      | 4         |
| 65–84      | 84      | 3         |
| 85–128     | 128     | 2         |
| 129–256    | 256     | 1         |

#### LDS occupancy

`AMDGPUSubtarget::getOccupancyWithLocalMemSize()` (`AMDGPUSubtarget.cpp:336`)
computes how many waves can run on one EU, given a workgroup's LDS
usage. This is the most complex of the three limits because it
involves several interacting factors:

```
// Step 1: How big could a workgroup be?
MaxWorkGroupSize = getFlatWorkGroupSizes(F).second
    // From amdgpu-flat-work-group-size attribute.
    // Default: 1024 for compute kernels.

// Step 2: How many workgroups can physically run on a CU?
MaxWorkGroupsPerCu = getMaxWorkGroupsPerCU(MaxWorkGroupSize)
    // Hardware limit — based on wave slots and barrier resources.
    // (See explanation below.)

// Step 3: How many workgroups' LDS fits in the CU?
NumGroups = LDSPerCU / LDSPerWorkgroup
    // LDSPerCU = 65536 (64KB) for gfx906
    // LDSPerWorkgroup = kernel's __shared__ allocation (fixed per kernel)
NumGroups = min(MaxWorkGroupsPerCu, NumGroups)
    // Can't exceed the hardware workgroup limit

// Step 4: How many waves do those workgroups produce?
MaxGroupNumWaves = ceil(MaxWorkGroupSize / WaveSize)
    // Each workgroup needs this many waves
MaxWaves = NumGroups * MaxGroupNumWaves
    // Total waves across the CU

// Step 5: Divide by EUs to get per-EU occupancy
MaxWaves = ceil(MaxWaves / EUsPerCU)    // 4 EUs per CU on gfx906
MaxWaves = min(MaxWaves, MaxWavesPerEU) // cap at 10
```

**What `getMaxWorkGroupsPerCU` does** (`AMDGPUBaseInfo.cpp:875`):
This computes a hardware limit on concurrent workgroups, independent
of LDS. Two constraints:

1. **Wave slots:** Each CU has `MaxWavesPerEU × EUsPerCU` total wave
   slots (10 × 4 = 40 for gfx906). Each workgroup uses
   `ceil(MaxWorkGroupSize / WaveSize)` waves. So at most
   `40 / wavesPerGroup` workgroups can fit.

2. **Barrier resources:** Multi-wave workgroups need a hardware
   barrier for `__syncthreads()`. gfx906 has 16 barrier slots per CU.
   Single-wave workgroups (MaxWorkGroupSize ≤ 64) don't need barriers
   and skip this limit.

   Result: `min(40 / wavesPerGroup, 16)`.

**Worked example — 8KB LDS, `__launch_bounds__(256)`:**

```
MaxWorkGroupSize = 256
wavesPerGroup = ceil(256/64) = 4
MaxWorkGroupsPerCu = min(40/4, 16) = min(10, 16) = 10
NumGroups = 65536/8192 = 8     (LDS-limited to 8)
NumGroups = min(10, 8) = 8
MaxWaves = 8 × 4 = 32 total
MaxWaves = ceil(32/4) = 8 per EU
Occupancy from LDS = 8
```

**Same example, no `__launch_bounds__` (default max = 1024):**

```
MaxWorkGroupSize = 1024
wavesPerGroup = ceil(1024/64) = 16
MaxWorkGroupsPerCu = min(40/16, 16) = min(2, 16) = 2
NumGroups = 65536/8192 = 8     (LDS could fit 8)
NumGroups = min(2, 8) = 2      (but only 2 workgroups fit in wave slots)
MaxWaves = 2 × 16 = 32 total
MaxWaves = ceil(32/4) = 8 per EU
Occupancy from LDS = 8
```

Same result here, but for different reasons: with `__launch_bounds__`
the limit is LDS (8 groups); without it, the limit is wave slots
(only 2 groups fit but each produces 16 waves).

**Same example, `__launch_bounds__(64)`:**

```
MaxWorkGroupSize = 64
wavesPerGroup = ceil(64/64) = 1
MaxWorkGroupsPerCu = 40         (single-wave, no barrier limit)
NumGroups = 65536/8192 = 8
NumGroups = min(40, 8) = 8
MaxWaves = 8 × 1 = 8 total
MaxWaves = ceil(8/4) = 2 per EU
Occupancy from LDS = 2
```

Occupancy drops to 2. Each workgroup only produces 1 wave, so even
though 8 workgroups fit in LDS, they only produce 8 waves total —
2 per EU.

**Key point:** LDS per workgroup is a fixed property of the kernel
(from `__shared__` declarations). It doesn't change with workgroup
size. But the number of waves per workgroup does change, and that
directly affects occupancy.

### N.3 Launch bounds and workgroup size attributes

The programmer can constrain occupancy via function attributes on
the kernel. These are **compile-time** — the compiler cannot see the
actual launch configuration, which is a runtime decision made by the
host code. If no attributes are specified, the compiler uses defaults.

**`amdgpu-flat-work-group-size`** — sets the min and max workgroup
size. In HIP, this comes from `__launch_bounds__(maxThreads)` or
`[[amdgpu::flat_work_group_size(min, max)]]`. Read by
`AMDGPUSubtarget::getFlatWorkGroupSizes()` (`AMDGPUSubtarget.cpp:400`).
Default: (1, 1024) for compute kernels.

This feeds into the LDS occupancy calculation (via `MaxWorkGroupSize`,
see N.2). It also affects `getMaxWorkGroupsPerCU()`, which determines
how many workgroups can physically run on a CU.

**`amdgpu-waves-per-eu`** — sets the min and max waves per EU. In
HIP: `[[clang::amdgpu_waves_per_eu(min, max)]]`. Read by
`AMDGPUSubtarget::getWavesPerEU()` (`AMDGPUSubtarget.cpp:450`).
Default: (1, 10) for gfx906.

This directly constrains occupancy:
- The **max** (e.g., 8) caps occupancy. `SIMachineFunctionInfo::
  getMaxWavesPerEU()` returns `WavesPerEU.second`, and
  `limitOccupancy()` uses it. Even if register pressure and LDS
  would allow 10 waves, the compiler targets at most 8.
- The **min** (e.g., 4) sets a floor for register allocation. The
  register allocator must leave enough registers for at least 4
  waves, preventing it from consuming so many registers that
  occupancy drops below the minimum.

**Why would a programmer limit occupancy?** Higher occupancy isn't
always better. More concurrent waves means more competition for
cache and memory bandwidth. For memory-bound kernels, reducing
occupancy can improve performance by giving each wave more cache
capacity. The programmer profiles their kernel and sets the sweet
spot via this attribute.

**Compile-time vs runtime.** The compiler uses the attribute values
(or defaults) to make scheduling and register allocation decisions.
At runtime, the kernel could be launched with any workgroup size
within the declared range. The actual runtime occupancy could differ
from the compiler's estimate in either direction — for example,
smaller workgroups produce fewer waves per group (lower occupancy),
but single-wave workgroups don't need barrier hardware which
removes one limit (potentially higher occupancy). The compiler's
decisions are based on the declared range, not the actual launch
configuration.

### N.4 When occupancy is computed and by whom

Occupancy flows through several owners at different stages of
compilation:

#### Stage 1: SIMachineFunctionInfo construction

**When:** MachineFunction creation (before any scheduling)
**Owner:** `SIMachineFunctionInfo` (`SIMachineFunctionInfo.cpp:60-65`)
**What:**

```cpp
FlatWorkGroupSizes = ST.getFlatWorkGroupSizes(F);
WavesPerEU = ST.getWavesPerEU(F);
Occupancy = ST.computeOccupancy(F, getLDSSize());
```

Three things happen here. The first two store attribute values as
fields on `SIMachineFunctionInfo` for use by other parts of the
compiler (register allocator, code emitter, etc.):

- `FlatWorkGroupSizes` — the (min, max) workgroup size from the
  `amdgpu-flat-work-group-size` attribute, or (1, 1024) by default.
- `WavesPerEU` — the (min, max) waves per EU from the
  `amdgpu-waves-per-eu` attribute, or (1, 10) by default for gfx906.

The third line computes the initial occupancy. `computeOccupancy`
is called with `NumSGPRs=0` and `NumVGPRs=0` (default parameters),
so register limits are NOT applied. It computes:

```
Occupancy = min(getMaxWavesPerEU(),                         // hardware max (10)
                getOccupancyWithLocalMemSize(LDSSize, F))   // LDS limit
```

Note: `getMaxWavesPerEU()` here is the **subtarget** method
(`AMDGPUSubtarget.h:269`), which returns the hardware constant (10
for gfx906). It does NOT read the `amdgpu-waves-per-eu` attribute.
And `getOccupancyWithLocalMemSize` does read `amdgpu-flat-work-group-
size` internally (via `getFlatWorkGroupSizes(F)`), so the workgroup
size attribute IS accounted for.

At this point, occupancy reflects the hardware max and LDS limit,
but NOT the `amdgpu-waves-per-eu` attribute cap. That cap is
applied later.

#### Stage 1b: ISel finalization

**When:** End of instruction selection (`SIISelLowering.cpp:13392`)
**Owner:** `SIMachineFunctionInfo`
**What:**

```cpp
Info->limitOccupancy(MF);
```

This calls `limitOccupancy(const MachineFunction &MF)` (overload 1),
which applies two caps:

```cpp
void SIMachineFunctionInfo::limitOccupancy(const MachineFunction &MF) {
  limitOccupancy(getMaxWavesPerEU());  // WavesPerEU.second (from attribute)
  limitOccupancy(ST.getOccupancyWithLocalMemSize(getLDSSize(), ...));
}
```

`getMaxWavesPerEU()` here is the **SIMachineFunctionInfo** method
(`SIMachineFunctionInfo.h:1057`), which returns `WavesPerEU.second`
— the max from the `amdgpu-waves-per-eu` attribute. This is where
the attribute cap gets applied to `Occupancy`.

The LDS cap is re-applied as well (redundant with Stage 1, but
harmless — `limitOccupancy(unsigned)` only decreases, never
increases).

After this point, `Occupancy` = min(hardware max, LDS limit,
waves-per-eu attribute max). Still no register pressure.

There is a second overload that takes a plain number:

```cpp
void limitOccupancy(unsigned Limit) {
  if (Occupancy > Limit)
    Occupancy = Limit;
}
```

This is how register pressure enters the picture later — the
scheduler calls it with a register-derived occupancy value (see
Stage 3).

#### Stage 2: GCN scheduler initialization

**When:** Start of the machine scheduling pass
**Owner:** `GCNSchedStrategy` (`GCNSchedStrategy.cpp:84`)
**What:**
```cpp
TargetOccupancy = MFI.getOccupancy();
// = min(hardware max, LDS limit, attribute max). No register info yet.
SGPRCriticalLimit = ST.getMaxNumSGPRs(TargetOccupancy, true);
VGPRCriticalLimit = ST.getMaxNumVGPRs(TargetOccupancy);
```

The scheduler computes register pressure limits *from* the target
occupancy: "to achieve N waves, we can use at most X SGPRs and Y
VGPRs." These limits guide scheduling decisions — if a region's
pressure exceeds the critical limit, the scheduler tries harder to
reduce it.

#### Stage 3: Per-region scheduling and occupancy tracking

**When:** After each region is scheduled
**Owner:** `GCNSchedStage::checkScheduling()` (`GCNSchedStrategy.cpp:912`)
**What:** This is where register pressure finally enters the
occupancy picture. After scheduling a region, the scheduler measures
the actual register pressure and converts it to occupancy:

```cpp
// Measure actual register pressure after scheduling this region
PressureAfter = getRealRegPressure(RegionIdx);  // GCNDownwardRPTracker

// Convert register pressure to occupancy (register-only, no LDS)
unsigned RegisterOccupancy = PressureAfter.getOccupancy(ST);
// = min(ST.getOccupancyWithNumSGPRs(SGPRs),
//       ST.getOccupancyWithNumVGPRs(VGPRs))

// Combine with target occupancy (which already has LDS + attribute limits)
WavesAfter = min(TargetOccupancy, RegisterOccupancy);
```

If `WavesAfter < MinOccupancy`:
- The schedule is reverted (instructions put back in original order)
- `DAG.MinOccupancy` is updated — this is the scheduler's working
  copy, used to compare against future regions within this pass
- `MFI.limitOccupancy(MinOccupancy)` is called (overload 2, takes
  `unsigned`) — this writes the new value to
  `SIMachineFunctionInfo::Occupancy`, the permanent per-function
  field visible to later passes (register allocator, code emitter)

#### Stage 4: GCNRegPressure::getOccupancy()

**When:** Called whenever someone has a `GCNRegPressure` value and
needs occupancy
**Owner:** `GCNRegPressure` (`GCNRegPressure.h:63`)
**What:**
```cpp
unsigned getOccupancy(const GCNSubtarget &ST) const {
  return std::min(ST.getOccupancyWithNumSGPRs(getSGPRNum()),
                  ST.getOccupancyWithNumVGPRs(getVGPRNum(...)));
}
```

This is a pure register-to-occupancy conversion. It does NOT account
for LDS or launch bounds — those are already baked into
`TargetOccupancy` by the time this is called. The GCN scheduler
always takes `min(TargetOccupancy, PressureAfter.getOccupancy(ST))`
to combine both.

### N.5 Occupancy is per-kernel, not per-region

A kernel's occupancy is determined by the **worst-case** (highest
pressure) region across the entire function. If one region uses 64
VGPRs (occupancy 4) and another uses 24 VGPRs (occupancy 10), the
kernel runs at occupancy 4 — the hardware allocates registers for
the worst case at kernel launch time.

The GCN scheduler tracks this via `MinOccupancy` on
`GCNScheduleDAGMILive` (`GCNSchedStrategy.cpp:481`):

```cpp
StartingOccupancy(MFI.getOccupancy()), MinOccupancy(StartingOccupancy)
```

As it schedules each region, if any region would drop occupancy
below `MinOccupancy`, it either reverts the schedule or updates
`MinOccupancy` downward.

### N.6 Function calls and inlining

`AMDGPUTargetMachine.cpp` (lines 1073-1075) adds two passes early
in the pipeline:

```cpp
// Function calls are not supported, so make sure we inline everything.
addPass(createAMDGPUAlwaysInlinePass());
addPass(createAlwaysInlinerLegacyPass());
```

The first pass (`AMDGPUAlwaysInlinePass`) **marks** functions for
inlining. It has two modes controlled by the
`-amdgpu-function-calls` flag (defined in `R600TargetMachine.cpp:34`,
LLVM default: `true`):

- **`-amdgpu-function-calls=false`**: marks ALL non-declaration
  functions as `alwaysinline`, except those explicitly marked
  `__noinline__` (`AMDGPUAlwaysInlinePass.cpp:134-147`).
- **`-amdgpu-function-calls=true`**: only marks functions that
  use LDS globals (`AMDGPUAlwaysInlinePass.cpp:125-131`).

The second pass (`AlwaysInlinerLegacyPass`, generic LLVM) does
the actual inlining of all functions marked `alwaysinline`.

**What `hipcc` does:** `hipcc` passes both
`-mllvm -amdgpu-function-calls=false` and
`-mllvm -amdgpu-early-inline-all=true` behind the scenes
(verified with `hipcc -###`). This overrides the LLVM default
and forces all functions to be inlined, except those marked
`__noinline__`. So with `hipcc`, by the time the scheduler runs,
there are no function calls unless the programmer explicitly
requested them.

**Impact on occupancy for non-inlined calls:** If a non-inlined
call survives (via `__noinline__` or other toolchains), the
scheduler cannot see the callee's register usage. It schedules
the caller's region based only on the caller's own register
pressure. The callee's register usage is accounted for later by
`AMDGPUResourceUsageAnalysis` (`AMDGPUResourceUsageAnalysis.cpp`),
which walks the call graph and propagates register counts upward:
the caller's final register count becomes the max of its own
usage and all callees' usage. This means the scheduler's occupancy
estimate may be optimistic — the actual occupancy at runtime could
be lower if a callee uses more registers than the caller.

### N.7 OptSched's occupancy handling

OptSched computes occupancy using the same LLVM functions as the
GCN scheduler (`getOccupancyWithNumSGPRs`, `getOccupancyWithNumVGPRs`,
`getOccupancyWithLocalMemSize`), but adds its own configuration
layer.

**Per-region target occupancy** (`OptSchedGCNTarget::initRegion()`,
`OptSchedGCNTarget.cpp:92`):

```cpp
MaxOccLDS = ST->getOccupancyWithLocalMemSize(*MF);
RegionStartingOccupancy =
    getAdjustedOccupancy(ST, VGPRCount, SGPRCount, MaxOccLDS);
TargetOccupancy =
    shouldLimitWaves(MFI) ? getOccupancyLimit(OccFile) : MFI->getOccupancy();
```

OptSched computes the region's starting register pressure using
`GCNDownwardRPTracker` to get VGPRCount and SGPRCount, then computes
`RegionStartingOccupancy = min(MaxOccLDS, MaxOccVGPR, MaxOccSGPR)`.

**Occupancy limiting:** OptSched has an `occupancy_limits.ini` config
file (`optsched-cfg/occupancy_limits.ini`) that can set per-function
occupancy targets. Each line is a function name and a target:

```
_Z10ilp_kernelPfPKfS1_i 10
```

OptSched also supports heuristic occupancy limiting: if the function
is marked `isMemoryBound()` or `needsWaveLimiter()`, it may reduce
the target to 4 waves.

**Two-pass approach:** OptSched's GCN variant (`ScheduleDAGOptSchedGCN`,
`GCNOptSched.cpp:64`) runs two scheduling passes:
1. `OptSchedMaxOcc` — schedule to maximize occupancy (minimize
   register pressure)
2. `OptSchedBalanced` — schedule to balance occupancy and ILP

The first pass establishes the best achievable occupancy. The second
pass tries to reduce schedule length without dropping occupancy below
the target established by the first pass.

### N.8 File locations

| What | Where |
|------|-------|
| `computeOccupancy()` | `lib/Target/AMDGPU/AMDGPUSubtarget.cpp:697` |
| `getOccupancyWithNumSGPRs()` | `lib/Target/AMDGPU/AMDGPUSubtarget.cpp:637` |
| `getOccupancyWithNumVGPRs()` | `lib/Target/AMDGPU/AMDGPUSubtarget.cpp:663` → `lib/Target/AMDGPU/Utils/AMDGPUBaseInfo.cpp:1077` |
| `getOccupancyWithLocalMemSize()` | `lib/Target/AMDGPU/AMDGPUSubtarget.cpp:336` |
| `getMaxWavesPerEU()` | `lib/Target/AMDGPU/Utils/AMDGPUBaseInfo.cpp:898` (10 for gfx906) |
| `getTotalNumVGPRs()` | `lib/Target/AMDGPU/Utils/AMDGPUBaseInfo.cpp:1060` (256 for gfx906) |
| `getVGPRAllocGranule()` | `lib/Target/AMDGPU/Utils/AMDGPUBaseInfo.cpp:1030` (4 for gfx906) |
| `SIMachineFunctionInfo::Occupancy` | `lib/Target/AMDGPU/SIMachineFunctionInfo.h:267` |
| `SIMachineFunctionInfo::limitOccupancy()` | `lib/Target/AMDGPU/SIMachineFunctionInfo.cpp:218` |
| `GCNRegPressure::getOccupancy()` | `lib/Target/AMDGPU/GCNRegPressure.h:63` |
| `GCNSchedStrategy::TargetOccupancy` | `lib/Target/AMDGPU/GCNSchedStrategy.h:63` |
| `checkScheduling()` | `lib/Target/AMDGPU/GCNSchedStrategy.cpp:912` |
| Inlining passes | `lib/Target/AMDGPU/AMDGPUTargetMachine.cpp:1073` |
| OptSched occupancy config | `lib/Target/AMDGPU/OptSched/optsched-cfg/occupancy_limits.ini` |
| OptSched target init | `lib/Target/AMDGPU/OptSched/lib/Wrapper/AMDGPU/OptSchedGCNTarget.cpp:92` |
| OptSched two-pass setup | `lib/Target/AMDGPU/OptSched/lib/Wrapper/AMDGPU/GCNOptSched.cpp:64` |

### N.9 What this means for our scheduler

For our scheduler, we need to:

1. **Read the target occupancy** from `SIMachineFunctionInfo::getOccupancy()`
   at the start of scheduling. This already accounts for LDS and
   launch bounds.

2. **Convert register pressure to occupancy** using
   `GCNRegPressure::getOccupancy(ST)` on our tracker's peak pressure.
   Our `GCNRegisterTracker` already stores a `GCNRegPressure`, so
   this is a method call away.

3. **Compare against the target.** The actual occupancy for a schedule
   is `min(target_occupancy, register_pressure_occupancy)`. If a
   candidate schedule's register pressure drops occupancy below the
   target, that's a cost to weigh against any latency improvement.

4. **Track occupancy across regions.** Like the GCN scheduler's
   `MinOccupancy`, we should track the lowest occupancy across all
   regions. The kernel's actual occupancy is the minimum.

The occupancy calculation itself doesn't need a new class — it's
a few calls to existing `GCNSubtarget` and `GCNRegPressure` methods.
What we need is the logic for deciding how to balance occupancy
against schedule length, which is a scheduling strategy concern.
