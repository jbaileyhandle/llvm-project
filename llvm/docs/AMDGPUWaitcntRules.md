# The rules SIInsertWaitcnts follows on gfx906

Distilled from `llvm/lib/Target/AMDGPU/SIInsertWaitcnts.cpp` at commit 0b42eede6b5e. `[123]`
is a line in that file at that commit.

`(c05)` is an experiment case, in `results_mir.txt` of the sandbox repo
`/home/jbaile/gpu2/sandbox/investigations/2026_11_01_flat_waitcnt/`. `waits_summary.txt`
there lists just the waits. These are what the compiler emits, not proof the hardware needs
them.

Left out, because they don't apply to our gfx906 compute kernels: the export counter,
calls, the VCCZ bug, and gfx10+ `vscnt`.

## State

The pass walks the instructions in order and numbers the ops in each counter: 1, 2, 3...
It keeps this, per counter T (vmcnt, lgkmcnt):

```
issued[T]     // number of the newest op issued in T                ScoreUBs
finished[T]   // ops numbered up to here are known to have finished ScoreLBs
op_of[T][reg] // number of the op that last wrote this register     VgprScores, SgprScores
kinds[T]      // kinds of op outstanding (LDS, GDS, scalar, ...)    PendingEvents
last_flat[T]  // number, in T, of the newest FLAT that counts in    LastFlat
              // both counters. 0 if there hasn't been one.
```

A FLAT that counts in both counters gets a number in each. For example, it could be op 5
in vmcnt and op 2 in lgkmcnt, which makes `last_flat[vmcnt] = 5` and `last_flat[lgkmcnt] = 2`.

## For each instruction

```
Process(mi):                                      // insertWaitcntInBlock [1558]
  wait = WaitsNeededBefore(mi)                    // [1603]
  emit s_waitcnt(wait) before mi, if any
  for each counter T: Apply(T, wait[T])           // [1297]
  Issue(mi)                                       // [1652]
```

## Which waits an instruction needs

```
WaitsNeededBefore(mi):                            // generateWaitcntInstBefore [1007]
  wait = {vmcnt: none, lgkmcnt: none}             // each keeps the smallest N asked for

  // Unconditional waits
  if mi is a function return:                     // [1033]
    wait = {vmcnt: 0, lgkmcnt: 0}
  if mi is s_barrier:                             // [1229], gfx906 has neither barrier feature (c15)
    wait = {vmcnt: 0, lgkmcnt: 0}
  if mi stores to a pointer an earlier non-invariant scalar load read:
    wait[lgkmcnt] = 0                             // [1170], [1641]

  // Register reads and writes
  for each register r that mi reads OR writes:    // [1190]
    if r is a VGPR:
      if mi reads r, or mi is not a vector load of the same kind:   // [1209] (c19, c20)
        wait[vmcnt] = min(wait[vmcnt], Needed(vmcnt, r))            // (c18)
    wait[lgkmcnt] = min(wait[lgkmcnt], Needed(lgkmcnt, r))          // [1219]

  // Drop a wait that can't change anything
  for each counter T:                             // simplifyWaitcnt [775]
    if wait[T] >= issued[T] - finished[T]: wait[T] = none

  // A wait already in the code, such as a fence from SIMemoryLegalizer
  merge any preexisting s_waitcnt into wait       // [893] (c14)

  // Loop preheader: flush vmcnt so the loop's own waits don't include these ops
  if mi ends the block just before a loop, and ShouldFlush(loop):   // [1258] (c34, c35)
    wait[vmcnt] = 0
  return wait

ShouldFlush(loop):                                // shouldFlushVmCnt [1733]
  // The loop must not read any register that the loop itself loaded with a vector load
  // (returns false as soon as it sees one), and must read a VGPR still outstanding from
  // before the loop.
  return that, and (the loop has a vector load, or the loop has a vector store)
```

## How big one wait is

```
Needed(T, r):                                     // determineWait [784]
  op = op_of[T][r]
  if op <= finished[T]:                           // already finished: no wait (c16, c17)
    return none                                   // [792]

  if CautiousMode():                              // [793]  (c02, c04, c07)
    return 0
  if T == lgkmcnt and scalar is in kinds[T]:      // [839]  (c22)
    return 0
  if kinds[T] holds more than one kind:           // [841]  (c23)
    return 0

  return min(issued[T] - op, limit[T] - 1)        // [808]  (c01, c21)
                                                  // limit: vmcnt 63, lgkmcnt 15 (c24, c25)

CautiousMode():                                   // hasPendingFlat [272]
  // True while that FLAT isn't known to have finished in at least one of its counters.
  return last_flat[vmcnt]   > finished[vmcnt]
      or last_flat[lgkmcnt] > finished[lgkmcnt]   // (c08, c09)
```

Only the newest FLAT needs tracking: a wait for 0 proves everything issued before it has
finished, so a drain after the newest FLAT covers the older ones too (c36, c37). A new FLAT
turns cautious mode back on, even right after a drain (c38, c39).

## What a wait records

```
Apply(T, N):                                      // applyWaitcnt [821]
  if N is none: return
  if N == 0:
    finished[T] = issued[T]
    kinds[T] = {}
  else if T is not out of order:                  // [826], the scalar and mixed checks above
    finished[T] = max(finished[T], issued[T] - N)
```

Only a wait for 0 clears `kinds`.

## What issuing records

```
Issue(mi):                                        // updateEventWaitcntAfter [1401]
  if mi is DS:              Count(lgkmcnt, GDS if it's GDS else LDS)        // [1407]
  else if mi is FLAT:                                                       // [1416]
    v = mi may touch global memory                // mayAccessVMEMThroughFlat [1347]
    l = mi may touch LDS                          // mayAccessLDSThroughFlat [1375]
    if v: Count(vmcnt, vector)
    if l: Count(lgkmcnt, LDS)
    if v and l:                                   // setPendingFlat [279], only for both (c03, c05)
      last_flat[vmcnt] = issued[vmcnt]
      last_flat[lgkmcnt] = issued[lgkmcnt]
  else if mi is vector memory, not a buffer invalidate:
    Count(vmcnt, vector)                                                    // [1440]
  else if mi is scalar memory, s_memtime or s_memrealtime:
    Count(lgkmcnt, scalar)                                                  // [1448], [1479]
  else if mi is s_sendmsg:  Count(lgkmcnt, message)                         // [1473]

Count(T, kind):                                   // updateByEvent [554]
  issued[T] += 1
  add kind to kinds[T]
  for each register r that mi writes:
    op_of[T][r] = issued[T]                       // [688]
```

Why a store causes no waits of its own: it writes no register, so no `op_of` entry ever
points at it. It still adds to `issued`, so waits on earlier loads count past it.

## Across basic blocks

The state above carries from block to block. Nothing resets at a block boundary, a loop back
edge, or `s_endpgm` (c27).

```
RunOnFunction():                                  // runOnMachineFunction [1787]
  repeat, visiting blocks in reverse post order, until nothing changes:   // [1845], [1850], [1898]
    for each block whose incoming state changed:
      Process every instruction, starting from that incoming state, and update any
        wait the pass emitted on an earlier visit
      if anything is still outstanding at the end:   // [1874]
        for each successor: successor's incoming = Merge(its incoming, this state)
```

A block is visited again whenever a later path makes its incoming state worse, which is how
loops work. The waits left at the end are the ones safe for every path into each point.

```
Merge(a, b), for each counter T:                  // merge [1501], mergeScore [1487]
  kinds[T] = kinds in a, plus kinds in b          // [1513]

  // The two states number their ops differently, so line them up by counting back from
  // the newest op. What's preserved is "how many ops came after this one".
  outstanding = max(a.issued - a.finished, b.issued - b.finished)
  issued      = a.finished + outstanding          // [1518]
  finished    = a.finished

  for each op_of[reg], and for last_flat:
    if that number is finished on its own side: treat it as nothing outstanding   // [1489]
    else: renumber it to keep its distance from the newest op
    keep the larger of the two                    // [1492]
```

Keeping the larger number means keeping the op with fewer ops after it, so the later wait
uses the smaller N, the more cautious one. Three consequences, all observed:

| Situation | Result | Case |
|---|---|---|
| A load has 0 ops after it on one path, 1 on the other | `vmcnt(0)` | c29 |
| A FLAT store on only one side of an if/else | Cautious after the join, on both paths | c28 |
| A load in only one side of an if/else | Waits as though that path was taken: `vmcnt(2)` | c30 |

And in loops:

| Situation | Result | Case |
|---|---|---|
| Loaded late in the body, used at the top of the next iteration | `vmcnt(1)` at the top | c31 |
| FLAT store before the loop, no lgkmcnt wait inside it | `vmcnt(0)` on every iteration | c27 |
| FLAT store at the *end* of the body | Waits *earlier* in the body become `vmcnt(0)`, via the back edge | c32 |
| Same, with an LDS wait in the body | Still `vmcnt(0)`: the store comes right back around | c33 |

## What ends cautious mode

It ends only when both counters have had a wait for 0 after the FLAT. The `vmcnt` half
normally comes quickly, since any vector wait is already forced to 0. The `lgkmcnt(0)` can
come from any of:

| Source | Line | Case |
|---|---|---|
| Using the FLAT's own loaded data | [1219] | c08, c12 |
| Using an LDS load's data | [1219] | c10, c11 |
| Using a scalar load's data | [1219] | c13 |
| A fence's preexisting wait, e.g. from `__syncthreads` | [893] | c14 |
| `s_barrier` | [1229] | c15 |
| A function return | [1033] | not run |
| A store to a pointer a scalar load read | [1170] | not run |

A FLAT *store* has no data to use, so the first row can't happen for it.
