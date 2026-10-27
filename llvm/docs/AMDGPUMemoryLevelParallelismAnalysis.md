# Measuring memory-level parallelism (MLP)

2026-10-27. How to measure **memory-level parallelism (MLP)**: how many
memory loads are waiting on memory at the same time. Nothing in the
scheduler implements these methods yet. This is the design background
for doing so.

The doc covers two main methods, in order of importance:

1. **Measuring the MLP of a schedule** (section 2). Given a schedule,
   compute exactly how much its loads overlap, using real latencies.
2. **Estimating MLP from the DAG alone** (section 3). With no schedule,
   only the dependence graph, compute a quick best-case number.

Section 4 is secondary background. Section 5 lists caveats that apply to
everything.

## 1. Vocabulary

- **G** is the dependence DAG of one scheduling region.
- **Load**: any memory load in G. **|M|** is the number of loads.
- Load b **depends on** load a if there is a path from a to b in G. The
  path may pass through non-load instructions, such as an add that
  computes b's address from a's data.
- **Schedule**: the list of instructions together with the cycle at
  which each one issues. The scheduler produces one for every region.
- **ASAP schedule** ("as soon as possible"): an idealized schedule.
  Every instruction issues the moment all its inputs are ready, any
  number of instructions can issue in the same cycle, and registers are
  unlimited. It's the fastest schedule the dependences allow. Real
  hardware can't run it, but it's useful as a best case when no real
  schedule is available.
- **Latency of a load, L_v**: the number of cycles from when load v
  issues until its data comes back.

## 2. Measuring the MLP of a schedule

### 2.1 The method

**(a) Get the issue cycles.** Let t_v be the cycle at which instruction
v issues.

- **For a real schedule**, such as the scheduler's output for a region,
  read t_v directly from the schedule.
- **For the ASAP schedule**, compute it:

    t_v = max over each predecessor u of (t_u + latency of u),
    and t_v = 0 if v has no predecessors.

  This is a longest-path computation, done in one pass over G in
  topological order. The weight of an edge u → v is the latency of the
  producer u: how long v must wait for u's result.

**(b) Turn each load into an interval.** Load v is in flight from the
cycle it issues until its data comes back: the half-open interval
[t_v, t_v + L_v). Only loads get intervals. The interval ends when the
data returns, not when some instruction uses it.

**(c) Count how many loads are in flight at each moment.** Let n(t) be
the number of loads in flight at cycle t. It only changes when a load
starts or finishes, so there's no need to check every cycle:

1. Make a +1 event at each interval start and a −1 event at each end:
   2|M| events in total.
2. Sort the events by time. On ties, put ends before starts. Without
   that rule, a load that finishes in the same cycle another issues would
   be counted as overlapping it, which inflates the peak.
3. Walk the sorted events with a running counter. Between consecutive
   events, n(t) equals the counter.

**(d) Read off the numbers.**

- **Peak MLP** = the largest value the counter reaches.
- **Average MLP** = (area under n(t)) ÷ (number of cycles where n(t) ≥ 1).

### 2.2 Worked example

Four loads with a latency of 100 cycles each, plus one 4-cycle add. Issue
cycles are from the ASAP schedule.

| instr | what it is                | depends on | t_v           |
|-------|---------------------------|------------|---------------|
| A     | load                      | —          | 0             |
| B     | load                      | —          | 0             |
| x     | add, x = A + 1 (4 cycles) | A          | 0 + 100 = 100 |
| C     | load from address x       | x          | 100 + 4 = 104 |
| D     | load from address B       | B          | 0 + 100 = 100 |

Intervals:

```
A: [  0, 100)
B: [  0, 100)
D: [100, 200)
C: [104, 204)
```

The in-flight count over time:

```
cycle:      0 ──────────── 100 ── 104 ─────────── 200 ── 204
n(t):             2         │  1  │       2        │  1  │
in flight:      A, B        │  D  │     D, C       │  C  │
```

- Peak MLP = 2.
- Area under n(t) = 2·100 + 1·4 + 2·96 + 1·4 = 400 load-cycles.
- Cycles with at least one load in flight: 0 to 204, so 204.
- **Average MLP = 400 / 204 ≈ 1.96.**

### 2.3 Shortcut for the average

Each load adds exactly L_v to the area under n(t), wherever it sits. So
the area always equals Σ L_v, the sum of all the load latencies. In the
example, 4 × 100 = 400. That gives:

  **Average MLP = Σ L_v ÷ (total length of the union of the load intervals)**

The only real work is measuring how much of the timeline the intervals
cover together, which the sweep in step (c) does: it counts the cycles
where the counter is above 0.

### 2.4 Which denominator: busy cycles or the whole schedule?

Let:

- **T_total** be the length of the whole schedule.
- **T_busy** be the number of cycles in which at least one load is in
  flight.

Dividing Σ L_v by T_busy or by T_total gives two different numbers, and
they are related exactly:

  Σ L_v ÷ T_total  =  (T_busy ÷ T_total)  ×  (Σ L_v ÷ T_busy)

- **T_busy ÷ T_total** is the fraction of the run spent waiting on memory
  at all.
- **Σ L_v ÷ T_busy** is how many loads overlap during that waiting. This
  is the average MLP from section 2.1.

So the whole-schedule number mixes two things: how much time is spent
waiting on memory, and how well the loads overlap while waiting.

Example: add 500 cycles of pure ALU work to the worked example, so
T_total = 704. Then 400 / 704 ≈ 0.57 = (204 / 704) × 1.96 ≈ 0.29 × 1.96.
The loads overlap exactly as well as before (1.96), but the
whole-schedule number dropped because the extra compute diluted it.

**Busy-cycle denominator (the MLP).** Use this to ask: *when we are
waiting on memory, how many requests are we waiting on together?* It
doesn't change when compute is added or removed. This is the standard
definition of MLP (Chou, Fahs and Abraham, ISCA 2004).

**Whole-schedule denominator.** Use this to ask: *how much memory
bandwidth does this schedule demand?* By Little's law, the average
number of outstanding requests = request rate × latency. So the
whole-schedule average, divided by latency, is the rate at which the
schedule issues memory requests.

Report both, or report MLP together with the busy fraction
T_busy / T_total. The two can be multiplied to recover the
whole-schedule number.

## 3. Estimating MLP from the DAG alone (unit-latency stage method)

### 3.1 The method

This needs no schedule, only G. It assumes **unit latency**: every load
takes 1 time unit and everything else takes 0.

1. **Give each load a stage.** A load is in stage 1 if it depends on no
   other load. Otherwise its stage is 1 + the highest stage among the
   loads it depends on. Each stage is one memory round trip. Loads in the
   same stage don't depend on each other, so they can all be in flight
   together.
2. **Let D be the number of stages.**
3. **Average MLP = |M| ÷ D** (number of loads ÷ number of stages).

The stage counts, listed in order, also show the shape of the overlap:
how many loads each round trip carries.

### 3.2 Worked example

```
A1  A2  A3        3 loads, independent
  \  |  /
   B1  B2         each B depends on all three A's
    |\/|
   C1  C2         each C depends on both B's
    |\/|
   D1  D2         each D depends on both C's
```

Stages: {A1, A2, A3}, {B1, B2}, {C1, C2}, {D1, D2}. Counts (3, 2, 2, 2).
|M| = 9, D = 4, **average MLP = 9 / 4 = 2.25**.

### 3.3 Why the number of stages is the right denominator

No schedule can finish the loads in fewer than D round trips. The
longest chain of dependent loads has D loads, and each must wait for the
one before it. Stage assignment achieves exactly D. Mirsky's theorem
(1971) states this formally: the fewest groups of mutually independent
loads needed to cover every load equals the number of loads on the
longest chain. So D is the shortest possible length of the ideal
schedule, measured in round trips, and |M| / D is the average number of
loads in flight over it.

### 3.4 Relationship to section 2

The stage method is section 2's measurement applied to the ASAP schedule
under three assumptions:

1. Every load has the same latency, L.
2. Non-load instructions take zero time.
3. Therefore every load in stage s is in flight during exactly
   [(s − 1)·L, s·L).

Under these assumptions:

- **Numerator:** Σ L_v = |M| · L.
- **Denominator:** the stages sit end to end with no gaps and no overlap,
  and every stage contains at least one load, so the union of the
  intervals is [0, D·L), of length D · L.
- **Ratio:** (|M| · L) ÷ (D · L) = |M| ÷ D. The latency cancels.

**When the two disagree.** In the section 2.2 example, C and D are both
in stage 2, so the stage method says 4 loads / 2 stages = 2.0. In reality
C waits 4 extra cycles for the add, so the second round trip stretches
from 100 to 204 instead of 100 to 200, and the measured average is 1.96.
Mixed latencies within a stage, such as a scalar load next to a vector
load, shift the number in the same way.

Use the stage method for a quick ceiling from the DAG. Use section 2
for what a schedule actually achieves.

## 4. Secondary background

### 4.1 Peak MLP from the DAG

The largest stage count is a quick estimate of peak MLP, but it can
undercount, because loads with slack can be moved later. Example: two
unrelated pieces.

- x1 → {x2, x3}: two loads depending on x1.
- y1 → y2 → {y3, y4}: two loads depending on y2.

Stage counts: {x1, y1}, {x2, x3, y2}, {y3, y4} = (2, 3, 2). Largest
stage: 3. But x2 and x3 have slack. Nothing depends on them, so they can
be delayed to stage 3 and overlap with y3 and y4, giving 4 loads in
flight at once.

The exact peak is the largest set of loads in which no load depends on
another. (In order theory, such a set is an *antichain*.) To compute it:

- **Dilworth's theorem (1950):** that largest set has the same size as
  the fewest chains of dependent loads needed to cover every load.
- **Fulkerson (1956)** turned this into bipartite matching:
  1. Compute which loads depend on which (the transitive closure of G,
     restricted to loads).
  2. Build a bipartite graph with a left copy and a right copy of every
     load, and an edge u_left → v_right whenever v depends on u.
  3. Peak MLP = |M| − (size of the maximum matching). Hopcroft–Karp
     computes the matching.

A larger peak is not automatically better. Delaying loads to bunch them
together can lengthen the schedule (see section 5).

### 4.2 The stage profile is not unique

For the same reason, the stage counts in section 3 depend on placing
every load as early as possible. Example: a chain of four dependent loads
L1 → L2 → L3 → L4, plus four independent loads X1–X4.

- Earliest placement gives (5, 1, 1, 1).
- One X per stage gives (2, 2, 2, 2).

Both take the same 4 stages, so |M| / D (the average) is the same: 8 / 4
= 2. Only the shape changes. The legal range of stages for load v is:

- earliest(v) = its stage from section 3.1, and
- latest(v) = D − (number of loads on the longest chain starting at v,
  counting v) + 1.

### 4.3 The Greene–Kleitman theorem

A structural result that generalizes Dilworth's theorem. It came up
while looking for an MLP-over-time metric. It turned out not to fit,
because it carries no timing, but it is recorded here for reference.

Define:

- **d_k** = the most loads that can be covered by k sets of mutually
  independent loads. The k sets are chosen from scratch for each k. They
  need not be stages, and need not be in time order.
- **c_k** = the most loads that can be covered by k chains of dependent
  loads.

Example: load A, with loads B, C and D each depending on A. Then
d₁ = 3 ({B, C, D}) and d₂ = 4 (add {A}). Also c₁ = 2 (A → B), c₂ = 3,
and c₃ = 4.

**Part 1 (Greene & Kleitman, 1976).** The increments
λ_k = d_k − d_{k−1} never increase: λ₁ ≥ λ₂ ≥ …. In the example,
λ = (3, 1). This holds only because the sets are chosen without regard
to order. Building them in time order, as stages, gives (1, 3) for the
same example.

**Part 2 (Greene, 1976).** Let μ_k = c_k − c_{k−1}. Draw λ as rows of
boxes. The column heights are exactly μ (μ is the *conjugate partition*
of λ):

```
λ = (3, 1)    ■ ■ ■
              ■
columns:      2 1 1   = μ
```

**Computing it.** Computing d_k directly amounts to finding large sets
of mutually unrelated nodes, which is NP-hard for general graphs (Karp,
1972). It is tractable here only because dependence is transitive, which
lets the question be answered through chains. Chains are paths, so c_k
comes from a min-cost flow:

1. Split each load into an in-node and an out-node, joined by an edge
   with capacity 1 and cost −1.
2. Add an edge from u's out-node to v's in-node, with capacity 1 and
   cost 0, whenever v depends on u.
3. Add a source and a sink connected to every load.

Push one unit of flow at a time with successive shortest paths. After k
units, −(total cost) = c_k. Then flip μ to get λ. Frank (1980) gives one
flow algorithm that produces all of d_k and c_k together.

**Limits.** λ₁ is the peak, and |M| ÷ (number of rows) is the
unit-latency average. But λ is a sorted list with no timing in it, and
the theorem doesn't promise a single split of the loads into sets with
exactly those sizes.

## 5. Caveats

- **The DAG-only numbers are upper bounds.** Sections 3 and 4, and
  section 2 applied to the ASAP schedule, assume unlimited issue slots
  and registers. Every in-flight load holds its destination registers
  until its data is consumed, so on AMDGPU more MLP directly costs
  register pressure and therefore occupancy. Only section 2 applied to a
  real schedule reports what is actually achieved.
- **More overlap does not mean a faster kernel.** Delaying loads so more
  of them overlap raises peak MLP but can lengthen the schedule. Compare
  MLP only between schedules of similar length.
- **Latencies come from a model.** For a real schedule, L_v comes from
  the scheduler's latency model or from measurement. The measured MLP is
  only as accurate as those numbers.
- **In-order memory counters.** AMDGPU's `vmcnt` counter is in-order:
  waiting for load b also waits for every vector-memory load issued
  before b. That adds ordering the DAG doesn't contain, so the effective
  dependences depend on issue order and are tighter than G suggests.

## 6. References

- Y. Chou, B. Fahs, S. Abraham, "Microarchitecture optimizations for
  exploiting memory-level parallelism," ISCA 2004. (Busy-cycle MLP
  definition.)
- A. Glew, "MLP yes! ILP no!," ASPLOS Wild and Crazy Ideas, 1998.
  (Origin of the term.)
- J. D. C. Little, "A proof for the queuing formula L = λW,"
  Operations Research, 1961.
- L. Mirsky, "A dual of Dilworth's decomposition theorem," American
  Mathematical Monthly, 1971.
- R. P. Dilworth, "A decomposition theorem for partially ordered sets,"
  Annals of Mathematics, 1950.
- D. R. Fulkerson, "Note on Dilworth's decomposition theorem for
  partially ordered sets," Proc. AMS, 1956.
- C. Greene and D. J. Kleitman, "The structure of Sperner k-families,"
  J. Combinatorial Theory A, 1976.
- C. Greene, "Some partitions associated with a partially ordered set,"
  J. Combinatorial Theory A, 1976.
- A. Frank, "On chain and antichain families of a partially ordered
  set," J. Combinatorial Theory B, 1980.
- R. M. Karp, "Reducibility among combinatorial problems," 1972.

The theorem statements are standard. The paper titles and venues were
written from memory and have not been checked against the originals.
Verify them before citing.
