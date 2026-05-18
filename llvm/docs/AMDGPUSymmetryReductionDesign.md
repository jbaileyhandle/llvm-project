# HierarchicalScheduler: Symmetry Reduction for BFS-DP Scheduling Design

## Purpose of this Document

This doc specifies **symmetry reduction** for the BFS-DP partition-lattice
scheduler (`BfsDpSearch` / `PartitionDag`; see
`AMDGPUPartitionLatticeDpDesign.md`).

When a region contains structurally interchangeable subgraphs — the
canonical case being the bodies of an unrolled loop — the partition
lattice contains large families of `PartitionNode`s that are images of
one another under a graph automorphism, and therefore have **identical
DP values**. BFS-DP currently explores every one of them. Symmetry
reduction collapses each automorphism orbit to a single `PartitionNode`
by making the search key *canonical* under the region's symmetry group.

**Prior-art disclosure up front.** Symmetry reduction is **not novel**.
It is a standard technique in model checking (Ip & Dill 1996; Emerson &
Sistla 1996; Clarke et al. 1996), in SAT / constraint solving (Crawford
et al. 1996), and in isomorph-free combinatorial generation (McKay
1998). The orbit / multiset counting in §5 is elementary combinatorics.
This doc claims novelty for none of it. The only engineering-specific
content is (a) the observation that BFS-DP's `DenseMap<PartitionKey>`
dedup makes canonicalization a *key-only* change with no search-loop
edits, and (b) the recommendation to *source* the symmetry group from
subgraph formation rather than from a general `Aut(G)` computation. §13
takes this up.

**Status.** Design only. Nothing here is implemented. The recommended
path (§7, §8, §12) depends on subgraph-formation work that is itself not
yet done; this doc should be read alongside `AMDGPUSubgraphFormationDesign.md`
and whatever `ScheduleSubgraph` design supersedes it.

## Table of Contents

1. [Context and Prerequisites](#1-context-and-prerequisites)
2. [Terminology](#2-terminology)
3. [The Symmetry in the Partition Lattice](#3-the-symmetry-in-the-partition-lattice)
4. [The Mechanism: Canonical Partition Keys](#4-the-mechanism-canonical-partition-keys)
5. [Search-Space Reduction](#5-search-space-reduction)
6. [Soundness](#6-soundness)
7. [Where the Symmetry Comes From](#7-where-the-symmetry-comes-from)
8. [Synergy with Subgraph Decomposition](#8-synergy-with-subgraph-decomposition)
9. [Worked Example: the high_rp Kernel](#9-worked-example-the-high_rp-kernel)
10. [Concerns and Mitigations](#10-concerns-and-mitigations)
11. [Open Questions](#11-open-questions)
12. [Implementation Phases](#12-implementation-phases)
13. [Prior Art and Novelty Assessment](#13-prior-art-and-novelty-assessment)
14. [Citations](#14-citations)

---

## 1. Context and Prerequisites

### 1.1 Problem statement

BFS-DP's cost is dominated by the number of distinct `PartitionNode`s it
materializes, which equals the number of distinct reachable
scheduled-sets. On a region with wide independent parallelism this is
exponential in the width. The per-region wall-clock budget
(`BfsDpSettings::timeout_ms`) then fires having explored only a tiny
prefix of the lattice — on the `-DHIGH_PRESSURE_TEST` stencil region the
timeout diagnostic reports **5 of 641 levels (~0.78%)** explored in 10s.

Symmetry reduction attacks that node count directly: it removes
`PartitionNode`s that are provably redundant because some graph
automorphism maps them onto a node already explored.

### 1.2 What already exists

The relevant infrastructure is in
`llvm/lib/Target/AMDGPU/HierarchicalScheduler/`:

- `PartitionDag` — BFS-by-layer enumeration of the partition lattice.
  Each `PartitionNode` is an equivalence class of partial schedules with
  the same scheduled *set*.
- `PartitionDag::FindOrInsert` — the dedup core: a
  `DenseMap<PartitionKey, PartitionNode*>` lookup. A miss creates a new
  node; a hit returns the existing one. **This is the integration
  point.**
- `ScheduledSetTracker` — maintains the scheduled-set bitset (one bit
  per node by topo index) and an incrementally-maintained XOR
  `prefix_signature_`. The XOR signature is *already* an
  order-independent hash of the scheduled set.
- `PartitionKey` = `{uint32_t signature, BitVector scheduled_set}`.
  Critically, `DenseMapInfo<PartitionKey>::isEqual` compares the
  **bitset only** — the signature is a pure hash of it. So the key is
  the *exact labeled scheduled-set*.
- `SubgraphInfo` — per-subgraph metadata (`members`, `ext_predecessors`,
  `ext_successors`, `initial_members`). Does **not** currently record
  any cross-subgraph isomorphism relationship.

### 1.3 What this doc adds

- A canonicalization layer between "the current scheduled-set" and "the
  `DenseMap` lookup key", so that orbit-equivalent partitions resolve to
  one `PartitionNode`.
- A specification of where the symmetry group comes from (§7) — the
  recommended source is subgraph formation, not a general automorphism
  computation.
- The soundness conditions the symmetry group must satisfy (§6).

---

## 2. Terminology

- **Attributed schedule graph** — the `ScheduleGraph` together with the
  data BFS-DP's DP actually reads: precedence edges with latencies, each
  node's register footprint (defs, uses, register class, width), and the
  region's live-in / live-out sets.
- **Automorphism** — a permutation σ of graph nodes that preserves the
  attributed schedule graph (made precise in §6). The identity is always
  an automorphism.
- **Symmetry group `G`** — the group of all automorphisms, typically
  represented by a small set of generators.
- **Orbit of a scheduled-set `S`** — `{ σ(S) : σ ∈ G }`. Every member of
  an orbit has the same DP value (§6).
- **Canonical form `canon(S)`** — a fixed, deterministically chosen
  representative of `S`'s orbit. `canon(S) == canon(S')` iff `S` and `S'`
  are in the same orbit.
- **Within-set symmetry** — reordering instructions inside one scheduled
  set. Already quotiented by BFS-DP (§3).
- **Cross-set symmetry** — two *different* scheduled-sets related by an
  automorphism. *Not* quotiented today; the target of this doc.
- **Interchangeable units** — the formation-level instance of the above:
  two subgraphs that are isomorphic under the attributed-graph relation.

---

## 3. The Symmetry in the Partition Lattice

A `PartitionNode` is keyed by the scheduled **set**, so ordering *within*
a set is already collapsed: scheduling `X1` then `X2` and scheduling `X2`
then `X1` both produce the set `{X1, X2}`, hence one `PartitionNode`. The
XOR `prefix_signature_` is literally an order-independent set hash; this
collapse is free and already done. The DP-merge in
`PartitionDag::VisitSuccessor` further merges all *paths* into a given
set at that one node, keeping the best `best_path_bottleneck`.

What is **not** collapsed is cross-set symmetry. Consider two
structurally identical, mutually independent two-node chains
`X1→X2` and `Y1→Y2`. The orderings

```
X1, X2, Y1, Y2      and      Y1, Y2, X1, X2
```

pass through *different* intermediate sets — `{X1,X2}` versus `{Y1,Y2}` —
which are different bitsets, hence different `PartitionKey`s, hence
different `PartitionNode`s. BFS-DP explores the `{X1,X2}` subtree and the
`{Y1,Y2}` subtree **in full, separately**. Yet if some automorphism σ of
the region swaps the X-chain with the Y-chain, then `{Y1,Y2} = σ({X1,X2})`
and the two nodes have identical futures and identical reachable
bottleneck values. One of the two subtrees is pure redundancy.

Symmetry reduction extends BFS-DP's merging from **"same set"** to
**"same orbit"**.

---

## 4. The Mechanism: Canonical Partition Keys

The entire mechanism is one inserted step. Today:

```
key = (prefix_signature, scheduled_set_bitset)
node = FindOrInsert(key)
```

Proposed:

```
key = (canon_signature, canon(scheduled_set_bitset))
node = FindOrInsert(key)
```

where `canon` maps every scheduled-set in an orbit to one representative
bitset, and `canon_signature` is the XOR signature taken over the
canonical bit positions.

Because `FindOrInsert` is already a `DenseMap` keyed lookup, two
orbit-equivalent partitions now hash and compare equal and therefore
resolve to the **same `PartitionNode`** automatically. The BFS layering
loop, the DP-merge, the score-bound prune (`SetInitialBestScore`), and
schedule reconstruction are all **unchanged**. The only new code is the
`canon` function and a `GetCanonicalPartitionKey()` accessor alongside
the existing `GetPartitionKey()`.

This is why BFS-DP is an unusually good host for symmetry reduction: the
search is already a canonical-form dedup; symmetry reduction only changes
*what counts as canonical*. (Compare model checking, where symmetry
reduction likewise canonicalizes the state hash — Ip & Dill 1996.)

The cost of `canon` depends entirely on the structure of `G` (§7). For a
*general* group it is as hard as graph canonical labeling. For the
*structured* group that subgraph formation can provide — a product of
symmetric groups acting on labeled blocks — `canon` is just **sorting
the per-block progress states**, `O(k log k)`.

---

## 5. Search-Space Reduction

Symmetry reduction reduces the `PartitionNode` count by a factor of at
most `|G|`; the *realized* factor is the average orbit size, which
approaches `|G|` precisely in the wide middle layers where the blowup
hurts most.

Two clean models bound the payoff.

**(a) `k` interchangeable, independent, single-node instructions.**
The reachable scheduled-sets over them are all `2^k` subsets. With
`S_k ⊆ G`, an orbit is determined solely by *cardinality*, giving
**`k + 1`** orbits. BFS layer `L` holds `C(k, L)` subsets, all in one
orbit — so layer `L` collapses to **1** node. The whole sub-lattice:
`2^k → k + 1`.

**(b) `k` interchangeable, independent chains, each a total order of
`m` nodes.** A reachable scheduled-set restricted to one chain is a
prefix of length `0..m` — `m + 1` states; `k` independent chains give
`(m + 1)^k` sets. Under `S_k` permuting the chains, an orbit is a
*multiset* of `k` prefix-lengths drawn from an `(m+1)`-element set:

```
        C(k + m, m)      orbits
```

i.e. exponential-in-`k` collapses to polynomial-in-`k`.

Worked figure, sized to the high_rp kernel (`k = 48` load/use pairs,
`m = 2`):

```
        without symmetry:   3^48      ≈ 7.98 × 10^22  partition nodes
        with symmetry:      C(50, 2)  = 1225          orbits
```

This is the **largest single lever** available to BFS-DP — *when the
symmetry is real and static*. §9 is the honest counterweight: it usually
is not, on a flat graph.

---

## 6. Soundness

**Definition (attributed automorphism).** A node permutation σ is an
*automorphism of the attributed schedule graph* if it preserves all of:

1. the precedence edges and each edge's latency;
2. each node's register footprint — defs, uses, register class, and
   register width;
3. the region's live-in and live-out sets, setwise.

**Claim.** If σ is an attributed automorphism, then for every reachable
scheduled-set `S`, the BFS-DP DP value at `S` equals the DP value at
`σ(S)`. Hence keeping one representative per `⟨G⟩`-orbit preserves the
optimum.

**Proof sketch.** σ extends to a bijection on partial schedules. It maps
any valid completion of `S` to a valid completion of `σ(S)`: condition 1
keeps precedence (and therefore schedule validity and per-edge cycle
relationships) intact. The DP metric is a function of `GCNRegPressure`,
which counts live registers per class; condition 2 makes σ permute
registers of identical class and width, leaving every `GCNRegPressure`
tuple — and hence every per-edge metric score — invariant. Condition 3
ensures the boundary contributes identically. So the *set* of achievable
path-bottleneck values from `S` and from `σ(S)` coincides, and BFS-DP's
max-over-paths-of-min-along-path DP value is equal at the two. ∎

**Why condition 3 is not optional.** Two subgraphs that are isomorphic
*internally* can still connect to the rest of the region differently.
They are interchangeable for the *whole-region* search only if their
external predecessor / successor structure also corresponds under σ. A
subgraph-isomorphism oracle that ignores the boundary is unsound at the
top level.

**Error direction.** A *too-loose* oracle (declaring non-equivalent
nodes equivalent) is **unsound** — it merges nodes with different futures
and can return a non-optimal schedule. A *too-tight* oracle (missing real
automorphisms) is merely **suboptimal in search effort** — it leaves
reductions on the table but the result stays correct. The oracle must
therefore err tight: conservative attributed hashing plus explicit
verification.

---

## 7. Where the Symmetry Comes From

Three options to obtain `G`:

**(1) General `Aut(G)` via a graph-automorphism tool** (nauty, saucy,
bliss). *Rejected.* Computing automorphisms of an arbitrary region in a
compiler pass is heavy; canonical-labeling cost is unbounded in the worst
case; and — see §9 — most of the automorphism group of a real flat
region is not the cheaply-exploitable kind anyway.

**(2) Structural, from subgraph formation.** *Recommended.* Subgraph
formation already partitions the region into subgraphs. If formation
*additionally* hashes each subgraph's attributed shape and groups
isomorphic subgraphs into **isomorphism classes**, the resulting group is
a **product of symmetric groups** — one `S_{k_i}` per class of `k_i`
isomorphic subgraphs — acting on a labeled set of subgraph units. This
group is structured, so `canon` is just "sort the interchangeable units
by progress state" (§4) — no graph canonical labeling required. The
isomorphism test is local, bounded by subgraph size, and can be
conservative (attributed hash, then verify).

**(3) Twin / module detection.** A cheap *partial* method needing no
formation: two nodes are *twins* if they have identical predecessor
sets, identical successor sets, and identical attributes. Twins generate
`S_k` orbits directly and are detectable in near-linear time (a
refinement of modular decomposition — Habib & Paul 2010). Useful as a
*within-subgraph* refinement, independent of formation.

**Recommendation.** Adopt (2) as the primary mechanism. Treat (3) as an
optional later refinement for residual symmetry inside coarse subgraphs.
Do **not** build (1).

---

## 8. Synergy with Subgraph Decomposition

This is the central architectural point: **decomposition converts
symmetry from the un-exploitable kind into the exploitable kind.**

A flat region has mostly **transient** symmetry — interchangeability that
holds only during part of the search and is broken once a consumer or a
serializing dependence enters (§9 makes this concrete). A *static,
global* canonical key, computed once from whole-graph automorphisms,
captures only symmetry that holds for the entire search; it misses
transient symmetry entirely.

Decomposition fixes this. When subgraphs are scheduled independently and
their internal order is **locked**, the top-level search ranges over
subgraph *units*. Two isomorphic subgraphs are now **statically**
interchangeable units — the messy transient symmetry of the flat graph
has been refactored into clean structural symmetry that a static
canonical key *does* capture. The `X1,X2,Y1,Y2 == Y1,Y2,X1,X2` example of
§3 is exactly the top-level search over two locked, isomorphic two-node
subgraphs.

Two top-level regimes, matching the two decomposition policies under
consideration:

- **Atomic subgraphs (no interleaving with free nodes / other
  subgraphs).** The top-level search permutes `k` units: `k!` orderings,
  collapsing under symmetry to the multiset of unit *types* — tiny.
- **Interleavable serialized subgraphs.** The top-level search interleaves
  `k` locked chains freely: `n! / (m!)^k` linear extensions. The
  partition lattice already collapses orderings; symmetry reduction then
  collapses the `(m+1)^k` reachable sets to `C(k+m, m)` orbits per
  §5(b). This is the regime where symmetry reduction pays the most.

Finally: scheduling **one representative per isomorphism class** and
reusing the cached schedule for every instance — which the
`ScheduleSubgraph` plan's "store the schedule on `SubgraphInfo`" already
enables — is itself symmetry reduction, applied one level down at the
subgraph-construction layer. Same principle, different layer.

---

## 9. Worked Example: the high_rp Kernel

The `-DHIGH_PRESSURE_TEST` block of the hip_stencil kernel is, on its
face, a poster child for symmetry — and on closer reading, a poster
child for why §8 is necessary. It is included here as the honest
counterweight to §5's headline numbers.

The block: 48 mutually independent loads `hp_vals[i]`, then a combine
loop `hp_sum += hp_vals[i] * hp_vals[(i+7)%48] + hp_vals[(i+13)%48]`,
accumulated into one running sum.

- The 48 **loads** are mutually independent and isomorphic. Among
  themselves, they admit the full symmetric group `S_48`.
- But the **combine loop is a circulant**: combine-step `i` reads
  `hp_vals` at offsets `i`, `i+7`, `i+13` (mod 48). The automorphism
  group of a circulant graph on `Z/48` is roughly the cyclic rotation
  group (order ~48), possibly with a few multiplier automorphisms — it
  is **not** `S_48`.
- And the **`hp_sum +=` accumulator is a serial chain** through all 48
  combine-steps. A total order kills permutation symmetry of the steps
  outright.

Net: the whole-graph, *static* automorphism group of the flat high_rp
region is small. A global canonical key would barely reduce anything. The
symmetry that genuinely exists is **transient** — it lives in the
load-heavy prefix, before the combine steps and the accumulator
distinguish the loads — and a static key cannot see it.

Conclusion: high_rp is a **weak** example for symmetry reduction on the
flat graph, and a **strong** example of why §8 matters. Cutting each load
together with its consumers into a subgraph (or cutting the load fan-in
from the combine fan-out) is what would turn high_rp's transient symmetry
into the static, cheap kind. **Symmetry reduction on the flat high_rp
graph is not worth implementing; symmetry reduction on the *decomposed*
high_rp graph is.**

---

## 10. Concerns and Mitigations

- **Input-order tiebreak interaction.** The DP-merge breaks score ties
  by input-order index (`PartitionDag::GetInputOrderIndex`, i.e.
  `SUnit::NodeNum`; see the "input-order tiebreak" commit) to inherit the
  input schedule's ILP / length tuning. Collapsing an orbit removes the
  ability to pick the input-closest member among symmetric completions.
  Every orbit member has identical register pressure, so the *pressure
  result stays sound* — but the tiebreak intent is partly lost.
  *Mitigation:* choose `canon`'s representative to be the
  input-order-minimal member of the orbit, so reconstruction still
  favors the input-closest schedule.
- **Reconstruction.** A canonical `PartitionNode` still stores one
  concrete `best_incoming_edge`; backtrace yields a concrete, valid,
  optimal schedule (the representative's). No soundness issue — see the
  tiebreak point for *which* representative.
- **Transient symmetry is not captured.** A static key misses it (§9).
  Chasing it would require per-state stabilizer subgroups, which is
  expensive and explicitly **out of scope**. The decomposition route
  (§8) is the supported answer.
- **Signature recomputation cost.** The XOR `prefix_signature_` is
  maintained incrementally today. If `canon` is a fixed relabeling of bit
  positions, the canonical signature can also be maintained
  incrementally; if `canon` re-sorts dynamically per lookup, the
  signature is recomputed in `O(set size)` per key. Measure before
  committing to an approach.
- **A loose oracle is unsound.** Per §6, the isomorphism oracle must err
  tight. Conservative attributed hashing plus verification.
- **Composition with the score-bound prune.** `canon` changes neither
  per-edge scores nor path bottlenecks, so `SetInitialBestScore` pruning
  is orthogonal and composes without interaction.
- **Test mode.** Shakedowns inject synthetic per-topo-index VGPR deltas.
  An automorphism must permute those deltas consistently, or a test-mode
  run is not actually symmetric. Either canonicalize the deltas with the
  graph, or disable symmetry reduction under test mode.

---

## 11. Open Questions

- The exact `canon` function for the structured group, and whether it is
  maintained incrementally or recomputed per lookup.
- Subgraph **granularity** versus residual within-subgraph symmetry — a
  formation knob, deferred to the formation / `ScheduleSubgraph` design.
- Whether within-subgraph twin detection (§7, option 3) earns its keep,
  or whether finer subgraph granularity makes it moot.
- A cheap, sound subgraph-isomorphism test for formation: the shape of
  the attributed hash, and the verification step behind it.
- Whether to expose symmetry reduction as a `BfsDpSettings` toggle
  (likely yes — needed for A/B measurement of node-count reduction).
- Interaction with a future multi-objective (schedule-length) extension:
  `canon` must be sound for the length metric too, which adds latency and
  issue-width to the attributes σ must preserve.

---

## 12. Implementation Phases

Each phase is independently measurable via
`PartitionDag::GetPartitionNodeCount()` and the timeout diagnostic
(`BfsDpSearch::GetLevelsExplored()`).

- **Phase 0 — prerequisite, in formation.** `SubgraphInfo` gains an
  isomorphism-class id; `SubgraphFormation` computes attributed-shape
  hashes for subgraphs and verifies candidate matches. Tracked by the
  formation / `ScheduleSubgraph` work, not this doc.
- **Phase 1 — top-level canonical key.** Add `GetCanonicalPartitionKey()`
  (canonicalizing the top-level subgraph-unit progress by sorting
  interchangeable units), gate it behind a `BfsDpSettings` toggle, and
  measure partition-node count with and without it on (a) a synthetic
  cleanly-symmetric workload and (b) the *decomposed* high_rp region.
- **Phase 2 — optional.** Within-subgraph twin detection for residual
  symmetry inside coarse subgraphs.

---

## 13. Prior Art and Novelty Assessment

Symmetry reduction is long-established. In model checking, equivalent
states under a symmetry group are collapsed by canonicalizing the state
representative (Ip & Dill 1996; Emerson & Sistla 1996; Clarke et al.
1996). In SAT and constraint solving, symmetry-breaking predicates prune
symmetric assignments (Crawford et al. 1996). In combinatorial
enumeration, canonical augmentation generates exactly one structure per
isomorphism class (McKay 1998), and practical graph canonical labeling is
a solved engineering problem (nauty / Traces — McKay & Piperno 2014). The
orbit and multiset counting of §5 is elementary combinatorics.

This doc claims **no novelty** for the technique. Its engineering-specific
content is narrow and twofold:

1. BFS-DP's `FindOrInsert` is *already* a canonical-form dedup
   (`DenseMap<PartitionKey>`), so symmetry reduction is a key-only change
   — it touches neither the BFS loop, the DP-merge, the prune, nor
   reconstruction.
2. The symmetry group should be *sourced from subgraph formation* — where
   it is a cheap, structured product of symmetric groups — rather than
   from a general `Aut(G)` computation on the flat region. §9 shows why
   the flat region is the wrong place to look.

For the BFS-DP algorithm itself, its prior art (Kessler 1998), and the
AMDGPU-specific competition (Shobaki et al. CGO 2024), see the
corresponding section of `AMDGPUPartitionLatticeDpDesign.md`.

---

## 14. Citations

- C. N. Ip and D. L. Dill. *Better Verification Through Symmetry.* Formal
  Methods in System Design, 9(1–2):41–75, 1996.
- E. A. Emerson and A. P. Sistla. *Symmetry and Model Checking.* Formal
  Methods in System Design, 9(1–2):105–131, 1996.
- E. M. Clarke, R. Enders, T. Filkorn, and S. Jha. *Exploiting Symmetry
  in Temporal Logic Model Checking.* Formal Methods in System Design,
  9(1–2):77–104, 1996.
- J. Crawford, M. Ginsberg, E. Luks, and A. Roy. *Symmetry-Breaking
  Predicates for Search Problems.* Principles of Knowledge Representation
  and Reasoning (KR), 1996.
- B. D. McKay. *Isomorph-Free Exhaustive Generation.* Journal of
  Algorithms, 26(2):306–324, 1998.
- B. D. McKay and A. Piperno. *Practical Graph Isomorphism, II.* Journal
  of Symbolic Computation, 60:94–112, 2014.
- M. Habib and C. Paul. *A Survey of the Algorithmic Aspects of Modular
  Decomposition.* Computer Science Review, 4(1):41–59, 2010.

### Companion design docs

- `AMDGPUPartitionLatticeDpDesign.md` — the BFS-DP scheduler this doc
  extends.
- `AMDGPUSubgraphFormationDesign.md` — subgraph formation, the proposed
  source of the symmetry group (§7).
- `AMDGPUClusteringDesign.md` — the proxy / subgraph model.
- `AMDGPUHistoryDominationDesign.md` — the DFS+history scheme; its
  same-set prefix dominance is the contrast drawn in §3.
