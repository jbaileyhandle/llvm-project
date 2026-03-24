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
(`GCNMaxOccupancySchedStrategy`). It is currently a no-op — the scheduling
logic will be implemented later.

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
│
│  Inherits from ScheduleDAGMILive. This is where our scheduling
│  logic will live.
│
│  schedule():          Called once per region by scheduleRegions().
│                       Records the region boundaries in regions_.
│
│  finalizeSchedule():  Called once after all regions are recorded.
│                       Currently a no-op — TODO.
└──────────────────────────────────────────────────────────────────────
```

### 10.2 Pipeline position

The pass is inserted into the AMDGPU pipeline via `insertPass` in
`GCNPassConfig::addOptimizedRegAlloc()`, guaranteeing it runs immediately
after the normal `MachineScheduler` pass:

```
MachineScheduler                         (GCNMaxOccupancySchedStrategy)
    ↓
MachineSchedulerHierarchical             (our pass — no-op for now)
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
This is read by `MachineInstrSchedulerConfig`, which provides
`IsHierarchicalScheduler()`.

Currently restricted to `gfx906` (Radeon VII) via a check in
`MachineSchedulerHierarchical::runOnMachineFunction()`.

### 10.4 Files

| File | Role |
|------|------|
| `lib/CodeGen/MachineScheduler.cpp` | Pass shell (class, registration, `runOnMachineFunction`) |
| `lib/CodeGen/CodeGen.cpp` | Pass initialization registration |
| `lib/Target/AMDGPU/AMDGPUTargetMachine.cpp` | Factory function, pipeline insertion, pass ordering |
| `lib/Target/AMDGPU/HierarchicalScheduler/ScheduleDAGHierarchicalScheduler.h` | Scheduler class declaration |
| `lib/Target/AMDGPU/HierarchicalScheduler/ScheduleDAGHierarchicalScheduler.cpp` | Scheduler implementation |
| `include/llvm/CodeGen/TargetPassConfig.h` | `createHierarchicalScheduler` virtual method |
| `include/llvm/CodeGen/Passes.h` | `MachineSchedulerHierarchicalID` extern declaration |
| `include/llvm/InitializePasses.h` | `initializeMachineSchedulerHierarchicalPass` declaration |
| `include/llvm/Analysis/MachineInstrSchedulerConfig.h` | `HierarchicalScheduler` enum + `IsHierarchicalScheduler()` |

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
