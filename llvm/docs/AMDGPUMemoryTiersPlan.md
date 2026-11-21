# Memory tiers plan

Group a region's memory nodes (the MemoryGraph's nodes) into tiers, and add edges
so that no node of tier k+1 can go before any node of tier k. The first structure
is the ideal tiers (the memory viewer's). Later, MemoryGraph offers a menu of
structures to try in the length pass.

## 1. MemoryGraph

```cpp
// A tier structure: each memory node's tier, the sink included.
struct MemoryTiers {
  std::vector<int> tier_by_node_id;  // indexed by MemoryNode id
};

// The ideal tiers: how many rounds of waiting must come before each node in
// any order. The rule moves here from MemoryViz.
MemoryTiers GetIdealTiers() const;

// Any tier structure, as nodes of the original graph. The sink is dropped.
ScheduleTiers ToOriginalTiers(const MemoryTiers &tiers) const;
```

MemoryViz calls `GetIdealTiers` in place of its own loop. The dumps don't change.

## 2. ScheduleGraph

```cpp
// New edge kind, in the latency range just before kArtificial.
kMemoryTier,

// A tier structure on this graph: element k holds the graph-local ids of the
// nodes in tier k. Nodes outside the memory graph aren't listed.
struct ScheduleTiers {
  std::vector<std::vector<int>> graph_local_ids_by_tier;
};

// kMemoryTier edges (latency 0) from every node of a tier to every node of
// the next non-empty tier, then recomputes the topological order and
// critical paths.
void AddTierEdges(const ScheduleTiers &tiers);

// The input schedule becomes this graph's topological order (the existing
// ...ByTopoOrderForTest, made a production function).
void PopulateInputScheduleConstructorByTopoOrder(const GCNSubtarget &st,
                                                 const MachineFunction &mf);

// A copy that keeps every node's graph-local id. Stub for now: fails with
// "not yet implemented".
std::unique_ptr<ScheduleGraph> Clone() const;

// Builds this graph's MemoryGraph, takes its ideal tiers, adds the tier
// edges, and switches the input schedule to a topological order if the
// input breaks them.
void EnforceIdealMemoryTiers(const GCNSubtarget &st, const MachineFunction &mf);
```

Why:
- Numbers on the memory side: structures get made, edited and checked against the
  graph there, and all three need one node's tier. Groups on the schedule side:
  applying a structure needs only the groups.
- Ids, not pointers: a structure translated against one graph applies to its clone.
- `kMemoryTier` in the latency range: a tier edge joins two real instructions, so it
  counts the one-cycle gap like any strong edge. Non-latency edges are assumed to
  touch proxy nodes (ScheduledSetTracker).
- Input swapped only when broken: the length pass's input is the occupancy pass's
  result, which already respects the tiers.
- The ideal tiers can't form a cycle, since every MemoryGraph edge goes to the same
  tier or a later one. A generated structure can, so the menu needs a check.

## 3. Flag and hook

Bare flag `enforce_ideal_memory_tiers` in MachineInstrSchedulerConfig, like
`live_out_latency`, mirrored into HierarchicalConfig. When it's on, `WithRegionGraph`
calls `graph->EnforceIdealMemoryTiers(st, MF)` right after building the graph. Every
pass builds its own graph there, so this covers the occupancy and length passes.

## 4. Checks

- Shakedown: a small test graph, with the tiers and edges worked out by hand.
- `dfs_test` assembly identical with the flag off.
- Flag on, `hip_stencil make high_rp`: along the final order in the memory_viz dump,
  `ideal_tier` never goes down.

## Later: a menu of tier structures

```
base = this pass's graph            (never gets tier edges)
memory_graph = MemoryGraph::Build(base)
for each structure on memory_graph's menu:
    tiers = memory_graph.ToOriginalTiers(structure)
    trial = base.Clone()
    trial.AddTierEdges(tiers)
    run the length pass on trial, keep the best order
apply the best order
```

Clone rather than apply/undo on the base: tries stay isolated, and nothing depends
on the pass leaving its graph alone.
