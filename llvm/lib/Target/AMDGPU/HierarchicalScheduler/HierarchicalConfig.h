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

#include "SubgraphFormation.h" // SubgraphScheduleMode
#include <optional>
#include <string>

namespace llvm {

class MachineInstrSchedulerConfig;

namespace hierarchical_scheduler {

/// `formation` axis: how a pass carves its region into subgraphs.
///   kNone    - no formation; a single flat search over the region.
///   kDomTree - the dominator-tree formation pipeline.
///   kMinCut  - acyclic min-cut of the data-dependency DAG (dagP).
enum class Formation { kNone, kDomTree, kMinCut };

/// `search` axis (occupancy pass only): the base scheduling algorithm.
///   kDfs      - depth-first occupancy search.
///   kBfsDp    - BFS / dynamic-programming partition search; may bail
///               (timeout / score-bound) and return no schedule.
///   kBfsDpDfs - BFS-DP with a DFS fallback, so a schedule is always
///               produced.
enum class Search { kDfs, kBfsDp, kBfsDpDfs };

/// `policy` axis (length pass only): the length pass's objective.
///   kMin                - minimize schedule length (base).
///   kMinRefineIlp       - minimize, then refine ILP among same-length
///                         completions.
///   kMinRefineOccupancy - minimize, then refine occupancy likewise.
///   kMax                - maximize length (control / worst-legal
///                         baseline).
enum class LengthPolicy { kMin, kMinRefineIlp, kMinRefineOccupancy, kMax };

/// Occupancy-pass configuration. Fixed objective (maximize occupancy),
/// so no `policy`; varies on the search algorithm and (when a formation
/// is chosen) the decompose/mode install. Defaults reproduce today's
/// behavior with no scoped keys: a flat DFS occupancy search.
struct OccupancyConfig {
  Formation formation = Formation::kNone;
  Search search = Search::kDfs;
  bool decompose = false;
  SubgraphScheduleMode mode = SubgraphScheduleMode::kSerialized;

  // Min-cut formation params (ignored unless formation == kMinCut);
  // defaults mirror MinCutSettings.
  float ratio = 1.5f;
  int target_size = 24;

  // BFS-DP search params (ignored unless search uses BFS-DP).
  int timeout_ms = 5000;
  /// DFS-fallback budget, ms; only valid for search == kBfsDpDfs.
  /// nullopt means "same as timeout_ms".
  std::optional<int> fallback_timeout_ms;

  /// Effective fallback budget: the explicit value, else the primary.
  int GetFallbackTimeoutMs() const {
    return fallback_timeout_ms.value_or(timeout_ms);
  }
};

/// Length-pass configuration. Fixed algorithm (DFS), so no `search` /
/// `decompose`; varies on the objective `policy`. Shares the formation
/// axis + params with the occupancy pass. Defaults reproduce today's
/// behavior with no scoped keys: flat length-min.
struct LengthConfig {
  Formation formation = Formation::kNone;
  SubgraphScheduleMode mode = SubgraphScheduleMode::kSerialized;

  float ratio = 1.5f;
  int target_size = 24;

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
  bool dump_subgraph_dag = true;     // dump region DAGs for the viewer
  bool scale_edge_latencies = false; // scale edge latency by target occupancy

  /// Build the typed config from the generic config's scoped settings.
  /// Per pass: built-in defaults -> named preset (if any) -> explicit
  /// scoped keys; strings map to enums; the axis constraints are
  /// enforced; an unknown scope, key, or value is a fatal error. Only
  /// call for the HierarchicalScheduler.
  static HierarchicalConfig Build(const MachineInstrSchedulerConfig &cfg);

  std::string ToString() const;
  void DebugPrint() const;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HIERARCHICALCONFIG_H
