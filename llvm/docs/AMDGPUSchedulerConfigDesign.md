# AMDGPU Scheduler Configuration Design

Status: **design** — agreed shape, not yet implemented. Describes the
`misched.txt` configuration scheme: the generic layer shared by every
AMDGPU scheduler, and the structured per-pass configuration for the
HierarchicalScheduler.

Related: `AMDGPUSubgraphSchedulingDesign.md` (subgraph scheduling, the
serialized/interleaved `mode`, §8), `AMDGPUSubgraphFormationDesign.md`
(formation policies).

## Contents
1. Motivation
2. Goals
3. Two-layer architecture
4. The HierarchicalScheduler axis model
5. Presets and precedence
6. File format
7. Generic layer
8. HierarchicalScheduler layer
9. Other schedulers are unaffected
10. Future directions
11. Migration plan

---

## 1. Motivation

Today `misched.txt` is: line 1 = `<Scheduler>` followed by whitespace-
separated **boolean** option tokens (parsed into a `std::set<SchedulerOption>`,
queried with `HasSchedulingOption`); lines 2+ = per-kernel settings
(`m|d / signature / waves_per_eu`).

Two problems have grown with the HierarchicalScheduler:

1. **Pass-specific flags share one global namespace.** `DecomposeForOccupancy`
   and `BfsDpForOccupancy` are occupancy-only; `LengthMinRefineIlp`,
   `LengthMinRefineOccupancy`, `MaximizeLength` are length-only — yet all are
   global booleans. There is no way to configure the occupancy and length
   passes independently (e.g. min-cut formation in occupancy, dom-tree in
   length).

2. **N-way choices are encoded as mutually-exclusive booleans.** The occupancy
   search is one of {DFS, BFS-DP, decompose}; formation one of {dom-tree,
   min-cut, none}; the length policy one of {min, min+refine-ilp,
   min+refine-occupancy, max}. Encoding an N-way choice as N booleans forces
   hand-written mutex checks (e.g. `MinCutFormation && SkipSubgraphFormation`
   → fatal) and grows the enum with every new variant.

## 2. Goals

- Configure the occupancy and length passes **independently**.
- Replace the boolean bloat with **typed choices + parameters**, so
  exclusivity is automatic and new variants don't grow a global enum.
- Keep every existing scheduler (`MaxOccupancy`, `MaxIlp`, `Iterative*`,
  the OptSched variants) working **unchanged**.
- Accommodate **future recursive decomposition** without schema rework.

## 3. Two-layer architecture

The config splits along a layering boundary that already exists in the tree:
`MachineInstrSchedulerConfig.h` lives in `llvm/include/llvm/Analysis/`
(generic, included widely) and **cannot** depend on AMDGPU types such as
`SubgraphScheduleMode`. So:

- **Generic layer — `MachineInstrSchedulerConfig` (every scheduler).**
  Selects the scheduler, holds the flat global/OptSched flags and the
  per-kernel settings, and additionally holds a *dumb* `scope → key → value`
  string store. It does **not** interpret the scoped strings — it is
  scheduler-agnostic.

- **HierarchicalScheduler layer — AMDGPU `HierarchicalConfig` (HS only).**
  `HierarchicalConfig::From(cfg)` runs only when the scheduler is
  `HierarchicalScheduler`. It expands presets, maps the scoped strings to
  typed enums, validates them, and fatals on anything unknown.

Other schedulers ride the generic layer and never touch the scoped layer.

## 4. The HierarchicalScheduler axis model

Each pass (`occupancy`, `length`) is configured along a small set of
**orthogonal axes**, with `formation` as the pivot:

| axis | values | meaning |
|---|---|---|
| `formation` | `none` \| `domtree` \| `mincut` | carve the region into subgraphs |
| `search` | `dfs` \| `bfsdp` \| `bfsdp+dfs` | base scheduling algorithm |
| `decompose` | `on` \| `off` | schedule subgraphs in isolation then combine, vs a single search over the formed graph |
| `mode` | `serialized` \| `interleaved` | how formed subgraphs install (proxies vs flat + order-edges; see SubgraphSchedulingDesign §8) |

`search=bfsdp` may return *no* schedule (it bails on timeout / score-bound,
and the input order is kept); `bfsdp+dfs` falls back to DFS so a schedule is
always produced.

The `length` pass carries the same shared axes **plus** a pass-specific
`policy` axis (`min` \| `min+refine-ilp` \| `min+refine-occupancy` \| `max`).
Pass-specific axes are handled exactly like the shared ones — scoped keys
mapping to typed fields in that pass's config.

### 4.1 Why these are the axes (and what they are not)

- **`decompose` is higher-order.** It is "form → schedule each piece →
  combine," *built out of* a formation and searches. It is **not** a peer of
  `dfs`/`bfsdp` (those are flat algorithms), so it is its own axis, not a
  `search` value.
- **`formation` is a free axis.** Any forming strategy can use any formation
  policy — a DFS over a min-cut carving is as legitimate as decompose over it.
  (Today the code only flows min-cut into decompose and pins DFS to dom-tree;
  that is an implementation limitation, not the design.)
- **The genuine tether is `decompose ⟹ formation`** — you cannot decompose
  without a carving. **`decompose ⟂ search`** — decompose runs whatever
  `search` is chosen as its engine (`decompose=on` + `search=dfs` is decompose
  with a DFS engine; `+ bfsdp+dfs` is the current preset).
- **`formation` is the pivot.** `formation=none` ⟹ a flat single search
  (`decompose`/`mode` do not apply); any real formation unlocks `decompose`
  and `mode`.
- **Internals stay internal.** A decompose level's per-subgraph ("leaf") and
  combine ("outer") searches, and the recurse threshold, are *not* config
  axes — they default (today: a uniform `search`; recursion off). They become
  tuning keys only if a need appears.

### 4.2 Parameters

Each axis may carry parameters, nested under it:

- `formation.ratio`, `formation.target_size` — min-cut knobs (these replace
  the values currently hardcoded in `MinCutSettings`).
- `search.timeout` — primary (e.g. BFS-DP) budget, ms.
- `search.fallback_timeout` — the DFS-fallback budget; only for
  `search=bfsdp+dfs` (defaults to the primary).

### 4.3 Constraints

Validation replaces the hand-coded mutex checks:

- `decompose` and `mode` require `formation ≠ none`.
- `search.fallback_timeout` requires `search = bfsdp+dfs`.
- Any `occupancy.*` / `length.*` scoped key requires
  `scheduler = HierarchicalScheduler`.

N-way exclusivity is automatic — each axis is one key holding one value, so
"two searches at once" is unrepresentable.

## 5. Presets and precedence

A **preset** is a code-defined bundle of axis values/parameters (a small
table in the AMDGPU `HierarchicalConfig`). Applied via `preset = <name>`
(whole-config) or `occupancy.preset = <name>` (per pass). Explicit keys
override the preset:

> **precedence: explicit key > preset > built-in default.**

Common presets (illustrative): `flat-dfs`, `decompose-mincut`,
`decompose-mincut-interleave`. Adding or renaming presets never touches the
generic config.

## 6. File format

Line-oriented; each line self-describing; order-independent; `#` comments.
Line grammar:

- `scheduler = <name>` — select the scheduler.
- `<scope>.<key> = <value>` — scoped setting, e.g. `occupancy.formation =
  mincut`, `occupancy.search.timeout = 5000`. To the generic layer `scope`
  and `key` are arbitrary dotted strings; the HS layer gives them meaning.
- `<flag>` (bare word) — global boolean, e.g. `dump_subgraph_dag`,
  `disable_post_ra`.
- `<key> = <value>` (no dot) — top-level setting, e.g. `preset`.
- `kernel m|d/<signature>/ <key>=<value> …` — per-kernel entry (`waves_per_eu`
  becomes a key rather than a positional field).

HierarchicalScheduler example:
```
scheduler = HierarchicalScheduler
occupancy.preset         = decompose-mincut
occupancy.mode           = interleaved
occupancy.search.timeout = 5000
length.policy            = min+refine-ilp
dump_subgraph_dag
kernel m/_Z10stencil_1dPiS_S_ii/ waves_per_eu=8
```

Non-HS example (parses as today's equivalent did):
```
scheduler = MaxOccupancy
disable_post_ra
kernel m/_Z.../ waves_per_eu=4
```

## 7. Generic layer

`MachineInstrSchedulerConfig`:

- **Keeps:** the `Scheduler` enum + selection; a flat `SchedulerOption` set
  for generic + OptSched + truly-global HS flags (`RunShakedowns`,
  `DumpSubgraphDag`, …); the per-kernel `FunctionConfig`.
- **Adds:** `std::map<std::string, std::map<std::string, std::string>>
  scoped_;` and `std::optional<StringRef> GetScopedSetting(scope, key)`.
- **Parser:** the line grammar of §6. A token/line with `<scope>.<key>=<value>`
  goes to `scoped_`; a bare flag goes through the (now much shorter)
  `SchedulerOption` path.
- **Removes:** the per-pass HS entries from `SchedulerOption`, their
  `StringSwitch` cases, their `IsValidOptionForScheduler` cases, and the
  cross-option **mutex block** — exclusivity now comes from one-value-per-key.

## 8. HierarchicalScheduler layer

AMDGPU `HierarchicalScheduler/HierarchicalConfig.{h,cpp}` (sketch):

```cpp
enum class Formation { kNone, kDomTree, kMinCut };
enum class Search    { kDfs, kBfsDp, kBfsDpDfs };

struct PassConfig {                       // shared per-pass axes + params
  Formation formation = Formation::kNone;
  Search    search    = Search::kDfs;
  bool      decompose = false;
  SubgraphScheduleMode mode = SubgraphScheduleMode::kSerialized;
  double ratio = 1.5; int target_size = 24;             // formation params
  int timeout_ms = 5000, fallback_timeout_ms = 5000;    // search params
};

struct HierarchicalConfig {
  PassConfig occupancy;
  PassConfig length;          // + a length-specific `policy` field
  bool dump_subgraph_dag = false, run_shakedowns = false;  // mirror flags

  static HierarchicalConfig From(const MachineInstrSchedulerConfig &cfg);
};
```

`From()` per pass: start from built-in defaults → apply the named preset (if
any) → apply explicit scoped keys; map strings to enums; enforce §4.3
constraints; fatal on an unknown key or value. Call sites read typed fields
(`hs.occupancy.formation == Formation::kMinCut`) instead of
`HasSchedulingOption(...)`. The HS scheduler builds one `HierarchicalConfig`
at init.

This is also where the recently hardcoded knobs become real config:
`occupancy.mode` selects the serialized/interleaved install, and
`formation.ratio` / `formation.target_size` feed `MinCutSettings`.

## 9. Other schedulers are unaffected

`MaxOccupancy` / `MaxIlp` / `Iterative*` / the OptSched variants parse exactly
as today — scheduler selection + flat flags + per-kernel settings. They never
touch the scoped layer, and `HierarchicalConfig::From` is not invoked for
them. A scoped `occupancy.*` key under a non-HS scheduler is a validation
error.

## 10. Future directions

- **Recursive decomposition.** `decompose=on` recurses on subgraphs above a
  threshold ("form subgraphs of subgraphs … schedule at the bottom, then
  interleave/schedule walking back up"). This is *internal* to `decompose`;
  the recurse threshold is a future `decompose`/`search` parameter — additive,
  no schema change.
- **Per-depth overrides.** If recursion levels ever need different policy, a
  nested scope (`occupancy.depth1.formation = domtree`) layers on without
  rework.
- **Per-kernel scoped overrides.** `kernel` lines could carry `occupancy.*`
  overrides for per-function tuning. Additive.

## 11. Migration plan

Incremental; a clean cutover (no back-compat shim — every `misched.txt` is
ours):

1. **Generic side.** Add `scoped_` + `GetScopedSetting` + the line-grammar
   parser; keep the flat options for non-HS schedulers + global flags +
   per-kernel. No behavior change for any non-HS scheduler; testable on its
   own.
2. **HS side.** Add `HierarchicalConfig`, the preset table, and `From()`.
3. **Migrate call sites** from `HasSchedulingOption(...)` to typed reads, pass
   by pass; then retire the dead per-pass `SchedulerOption` entries and the
   mutex block.
4. **Rewrite the (few) `misched.txt` files** to the new format.
