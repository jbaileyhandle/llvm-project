//===- HierarchicalConfig.h - Typed HS config from misched.txt --*- C++ -*-===//
//
// HierarchicalConfig is the HierarchicalScheduler's typed view of the
// scoped "<scope>.<key> = <value>" settings that
// MachineInstrSchedulerConfig parses from misched.txt and stores
// uninterpreted (the generic layer). HierarchicalConfig::Build(cfg) runs
// only for the HierarchicalScheduler: it expands presets, maps the
// scoped strings to typed enums + params, validates the axis
// constraints, and fatals on any unknown scope, key, or value.
//
// The occupancy and length passes get *different* config structs,
// because they vary along different dimensions: the occupancy pass has
// a fixed objective (maximize occupancy) but a choosable search
// algorithm; the length pass has a fixed algorithm (DFS) but a
// choosable objective (its `policy`). The two share only the formation
// axis + params (both carve subgraphs the same way). Global toggles that
// belong to neither pass (malicious variant, shakedowns, DAG dump,
// edge-latency scaling) are top-level fields, set in misched.txt as
// "name = true|false".
//
// See llvm/docs/AMDGPUSchedulerConfigDesign.md (§4 axis model, §8 layer).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HIERARCHICALCONFIG_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HIERARCHICALCONFIG_H

#include "HierarchicalConfigEnums.h" // Search, OccupancyPolicy, LengthPolicy
#include "SubgraphFormation.h" // SubgraphScheduleMode, SubgraphFormationStrategy
#include <cstdint>
#include <optional>
#include <string>

namespace llvm {

class MachineInstrSchedulerConfig;

namespace hierarchical_scheduler {

/// Occupancy target reported when occupancy.optimize_every_region_past_occupancy_target
/// is set: one past any achievable waves/SIMD (gfx906's hardware max is 10),
/// so the occupancy search is never "already at the target" and keeps
/// maximizing occupancy (= minimizing register pressure) on every region.
/// Consulted by GCNRegisterTracker::GetConfiguredMachineFunctionOccupancyTarget
/// and the occupancy pass's per-region skip.
inline constexpr unsigned kAboveHardwareMaxOccupancy = 11;

/// Occupancy-pass configuration. Fixed objective (maximize occupancy),
/// so no `policy`; varies on the search algorithm and (when a formation
/// is chosen) the decompose/mode install. Default (no scoped keys) is a
/// flat DFS search, formation off -- deliberately flat; note the
/// historical default instead formed dom-tree subgraphs.
struct OccupancyConfig {
  FormationConfig formation; // strategy + mode + min-cut settings
  // For a flat (non-decompose) run this is the whole search; for decompose it
  // is the OUTER (among-subgraphs) search.
  Search search = Search::kDfs;
  // The INNER (within-subgraph "make") search used by decompose. Default kDfs
  // (DFS-only make, comparable to Hier-Plain). occupancy.inner_search=bfsdp+dfs
  // selects the historical behavior (BFS-DP first, DFS on timeout). Ignored
  // when decompose is off.
  Search inner_search = Search::kDfs;
  // Spill-aware by default: continuous occupancy with VGPR spill area as a
  // same-score tiebreak. Requires search=dfs (the default above); a config
  // that switches to BFS-DP must also set a non-refine policy.
  OccupancyPolicy policy = OccupancyPolicy::kContinuousOccupancyRefineSpillArea;
  bool decompose = false;
  // Recursive decompose: when on (with decompose + mincut formation), split
  // each level into at most 4 subgraphs and recurse down to
  // ~target_subgraph_size-node leaves, instead of one flat decomposition.
  // Composes with the serialized/interleaved subgraph install mode
  // (formation.mode) and with the (policy, search) pair; deeper levels'
  // outer searches are always continuous.
  bool decompose_recursive = false;
  // Max subgraphs per level when decompose_recursive is on (the mincut part
  // cap). Default 4 ("at most 4"); e.g. 2 gives a binary recursion. Ignored
  // unless decompose_recursive is on.
  int decompose_max_parts = 4;

  // Occupancy-target cap. When set to X, lower the function occupancy target
  // to min(structural_max, input_occ + X) before the pass, so the search aims
  // for at most X notches above the input schedule's occupancy instead of
  // squeezing maximally (which can over-constrain registers at the cost of
  // ILP/length). Also switches decompose's OUTER search to DFS — the only
  // outer search that honors the lowered target (BFS-DP maximizes
  // regardless). Unset = no cap, BFS-DP outer (default).
  std::optional<int> max_occ_above_input;

  // When set, the occupancy pass optimizes EVERY region to its minimum register
  // pressure instead of stopping once the function occupancy target is met: it
  // neither skips a region whose input occupancy already meets the target nor
  // lets a region's search early-exit at the target. Implemented by reporting
  // kAboveHardwareMaxOccupancy as the target (so it is never reached) and not
  // lowering the MFI target between regions. For measuring the true achievable
  // pressure floor; mutually exclusive with max_occ_above_input (the opposite
  // direction).
  bool optimize_every_region_past_occupancy_target = false;

  // Per-region wall-clock budget, us (occupancy.search.timeout), for the
  // PRIMARY search whichever it is: flat BFS-DP, flat DFS, and the BFS-DP of
  // bfsdp+dfs. 0 means "no timeout" — run the search to completion. 10s default.
  // The config key is given in milliseconds and scaled to microseconds on parse;
  // the search carries the budget in microseconds so sub-millisecond
  // per-instruction budgets are enforced without truncation.
  int timeout_us = 10000000;
  /// Budget for the DFS BACKUP in bfsdp+dfs (occupancy.search.fallback_timeout),
  /// us. nullopt means "same as timeout_us"; 0 means "no timeout".
  std::optional<int> fallback_timeout_us;

  /// Effective fallback budget in us (explicit value, else the primary); 0 is
  /// "no timeout". Used for display.
  int GetFallbackTimeoutUs() const {
    return fallback_timeout_us.value_or(timeout_us);
  }

  /// Primary / fallback budgets as the search classes' optional<int64_t>, where
  /// a config value of 0 maps to nullopt ("run to completion").
  std::optional<int64_t> SearchTimeoutOrUnlimited() const {
    return timeout_us == 0 ? std::nullopt : std::optional<int64_t>(timeout_us);
  }
  std::optional<int64_t> FallbackTimeoutOrUnlimited() const {
    int us = GetFallbackTimeoutUs();
    return us == 0 ? std::nullopt : std::optional<int64_t>(us);
  }

  // Decompose-only: budgets for the INNER (within-subgraph "make") search,
  // separate from the outer/flat search above. occupancy.inner_search.timeout
  // is the primary; occupancy.inner_search.fallback_timeout is the DFS backup
  // when inner_search is bfsdp+dfs (nullopt = same as inner_timeout_us). 0 =
  // "no timeout". Default 5s matches the historical per-subgraph make budget.
  int inner_timeout_us = 5000000;
  std::optional<int> inner_fallback_timeout_us;

  int GetInnerFallbackTimeoutUs() const {
    return inner_fallback_timeout_us.value_or(inner_timeout_us);
  }
  std::optional<int64_t> InnerSearchTimeoutOrUnlimited() const {
    return inner_timeout_us == 0 ? std::nullopt
                                 : std::optional<int64_t>(inner_timeout_us);
  }
  std::optional<int64_t> InnerFallbackTimeoutOrUnlimited() const {
    int us = GetInnerFallbackTimeoutUs();
    return us == 0 ? std::nullopt : std::optional<int64_t>(us);
  }

  // Per-instruction occupancy-search budget (us/instruction), mirrored from the
  // misched global time_per_instr_occupancy_us. When set, overrides the flat
  // search.timeout config; disallowed alongside an explicit flat timeout, a
  // non-dfs search, or recursive decompose (all enforced in Build).
  std::optional<int> time_per_instr_us;
  // Whether the flat search.timeout / inner_search.timeout were set as explicit
  // user keys (not via a preset). Only the explicit form conflicts with a
  // per-instruction budget.
  bool timeout_explicitly_set = false;
  bool inner_timeout_explicitly_set = false;

  /// Effective search timeout (us) over `graph_size` nodes: the per-instruction
  /// budget when set (halved under decompose, since each instruction is
  /// scheduled twice -- inner make + outer), else `flat`. nullopt = unlimited.
  /// No floor: a sub-millisecond total budget stays sub-millisecond.
  std::optional<int64_t> EffectiveTimeout(int graph_size,
                                          std::optional<int64_t> flat) const {
    if (!time_per_instr_us.has_value()) {
      return flat;
    }
    int rate_us = decompose ? (*time_per_instr_us + 1) / 2 : *time_per_instr_us;
    int64_t us = static_cast<int64_t>(rate_us) * static_cast<int64_t>(graph_size);
    return us <= 0 ? std::nullopt : std::optional<int64_t>(us);
  }
};

/// Length-pass configuration. Fixed algorithm (DFS), so no `search` /
/// `decompose`; varies on the objective `policy`. Shares the formation
/// axis + params with the occupancy pass. Default (no scoped keys) is
/// flat length-min, formation off -- deliberately flat; note the
/// historical default instead formed dom-tree subgraphs.
struct LengthConfig {
  FormationConfig formation; // strategy + mode + min-cut settings
  // Spill-aware by default: minimize length but never let the length pass
  // push VGPR spill area above the input schedule's. (No search constraint —
  // the length pass is DFS-only regardless.)
  LengthPolicy policy = LengthPolicy::kMinBoundedSpillSignals;

  // Per-region wall-clock budget, us (length.search.timeout), for each length
  // DFS phase. 0 means "no timeout". DFS-only, so there is no fallback budget.
  // 10s default matches the historical DfsSearch default the length pass used.
  // The config key is given in milliseconds and scaled to microseconds on parse.
  int timeout_us = 10000000;
  // Whether length.search.timeout was set as an explicit user key (conflicts
  // with a per-instruction budget). No preset sets it, so any set is explicit;
  // tracked the same way as the occupancy timeouts for consistency.
  bool timeout_explicitly_set = false;

  std::optional<int64_t> SearchTimeoutOrUnlimited() const {
    return timeout_us == 0 ? std::nullopt : std::optional<int64_t>(timeout_us);
  }

  // Per-instruction length-search budget (us/instruction), mirrored from the
  // misched global time_per_instr_length_us. The length pass is not
  // hierarchical, so the full rate applies (no halving).
  std::optional<int> time_per_instr_us;

  /// Effective length-search timeout (us) over `graph_size` nodes: the
  /// per-instruction budget (full rate) when set, else `flat`. No floor.
  std::optional<int64_t> EffectiveTimeout(int graph_size,
                                          std::optional<int64_t> flat) const {
    if (!time_per_instr_us.has_value()) {
      return flat;
    }
    int64_t us = static_cast<int64_t>(*time_per_instr_us) *
                 static_cast<int64_t>(graph_size);
    return us <= 0 ? std::nullopt : std::optional<int64_t>(us);
  }
};

/// The HierarchicalScheduler's whole typed configuration, built once at
/// scheduler init by Build(). Call sites read typed fields (e.g.
/// `hs.occupancy.search == Search::kBfsDp`) instead of querying the generic
/// MachineInstrSchedulerConfig.
struct HierarchicalConfig {
  OccupancyConfig occupancy;
  LengthConfig length;

  // Global toggles that belong to neither pass. Set in misched.txt as bare
  // flags (e.g. `run_shakedowns`); the generic config owns the spelling and
  // validation, and Build() mirrors them here. Defaults below apply when the
  // flag is absent.
  bool malicious = false;            // run the malicious variant instead
  bool run_shakedowns = false;       // run validation harnesses
  bool dump_subgraph_dag = false;    // dump region DAGs for the viewer
  bool dump_search_outcomes = false; // per-search outcome CSV
  bool scale_edge_latencies = false; // scale edge latency by target occupancy
  bool skip_occupancy_pass = false;  // bypass RunMaximizeOccupancyPass entirely
  bool skip_length_pass = false;     // bypass RunLengthPass entirely
  // Length pass ignores the occupancy target: the length DFS skips its
  // occupancy prune ("Gate 1") so it optimizes length unconstrained by
  // occupancy. Implies skip_occupancy_pass (set in Build).
  bool length_ignore_occupancy = false;
  // Replace the length pass with the min-adjusted-length pass
  // (RunMinimizeAdjustedLengthPass): schedule the whole kernel once per
  // reachable occupancy tier — each region min-length searched under that
  // tier's register budget with edge latencies divided by the tier (the
  // adjusted lens) — then commit the tier whose kernel-wide adjusted
  // length sum is smallest and pin the occupancy target there. Requires
  // the occupancy pass (it establishes the top tier and a schedule
  // feasible at every tier), so incompatible with skip_occupancy_pass
  // and length_ignore_occupancy (enforced in Build).
  bool min_adjusted_length = false;

  /// Build the typed config from the generic config's scoped settings.
  /// Per pass: built-in defaults -> named preset (if any) -> explicit
  /// scoped keys; strings map to enums; the axis constraints are
  /// enforced; an unknown scope, key, or value is a fatal error. Only
  /// call for the HierarchicalScheduler.
  static HierarchicalConfig Build(const MachineInstrSchedulerConfig &cfg);

  /// Cached singleton: Build(MachineInstrSchedulerConfig::GetConfig())
  /// once, then return that instance on every call. The config is static
  /// for the run, so HS call sites read this rather than rebuilding or
  /// threading a reference. Only meaningful for the HierarchicalScheduler.
  static const HierarchicalConfig &Get();

  std::string ToString() const;
  void DebugPrint() const;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HIERARCHICALCONFIG_H
