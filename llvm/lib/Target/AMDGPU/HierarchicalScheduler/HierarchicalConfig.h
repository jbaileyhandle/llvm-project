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
  Search search = Search::kDfs;
  OccupancyPolicy policy = OccupancyPolicy::kContinuousOccupancy;
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

  // Per-region wall-clock budget, ms (occupancy.search.timeout), for the
  // PRIMARY search whichever it is: flat BFS-DP, flat DFS, and the BFS-DP of
  // bfsdp+dfs. 0 means "no timeout" — run the search to completion. 10s default.
  int timeout_ms = 10000;
  /// Budget for the DFS BACKUP in bfsdp+dfs (occupancy.search.fallback_timeout),
  /// ms. nullopt means "same as timeout_ms"; 0 means "no timeout".
  std::optional<int> fallback_timeout_ms;

  /// Effective fallback budget in ms (explicit value, else the primary); 0 is
  /// "no timeout". Used for display.
  int GetFallbackTimeoutMs() const {
    return fallback_timeout_ms.value_or(timeout_ms);
  }

  /// Primary / fallback budgets as the search classes' optional<int64_t>, where
  /// a config value of 0 maps to nullopt ("run to completion").
  std::optional<int64_t> SearchTimeoutOrUnlimited() const {
    return timeout_ms == 0 ? std::nullopt : std::optional<int64_t>(timeout_ms);
  }
  std::optional<int64_t> FallbackTimeoutOrUnlimited() const {
    int ms = GetFallbackTimeoutMs();
    return ms == 0 ? std::nullopt : std::optional<int64_t>(ms);
  }
};

/// Length-pass configuration. Fixed algorithm (DFS), so no `search` /
/// `decompose`; varies on the objective `policy`. Shares the formation
/// axis + params with the occupancy pass. Default (no scoped keys) is
/// flat length-min, formation off -- deliberately flat; note the
/// historical default instead formed dom-tree subgraphs.
struct LengthConfig {
  FormationConfig formation; // strategy + mode + min-cut settings
  LengthPolicy policy = LengthPolicy::kMin;
};

/// The HierarchicalScheduler's whole typed configuration, built once at
/// scheduler init by Build(). Call sites read typed fields (e.g.
/// `hs.occupancy.search == Search::kBfsDp`) instead of querying
/// MachineInstrSchedulerConfig::HasSchedulingOption.
struct HierarchicalConfig {
  OccupancyConfig occupancy;
  LengthConfig length;

  // Global toggles that belong to neither pass. Set in misched.txt as
  // top-level "name = true|false" settings; the defaults below apply
  // when the setting is absent.
  bool malicious = false;            // run the malicious variant instead
  bool run_shakedowns = false;       // run validation harnesses
  bool dump_subgraph_dag = false;    // dump region DAGs for the viewer
  bool dump_search_outcomes = false; // per-search outcome CSV
  bool scale_edge_latencies = false; // scale edge latency by target occupancy
  bool skip_occupancy_pass = false;  // bypass RunMaximizeOccupancyPass entirely
  bool skip_length_pass = false;     // bypass RunLengthPass entirely

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
