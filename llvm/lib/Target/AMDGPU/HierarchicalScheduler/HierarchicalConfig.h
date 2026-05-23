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

#include "SubgraphFormation.h" // SubgraphScheduleMode, SubgraphFormationStrategy
#include <optional>
#include <string>

namespace llvm {

class MachineInstrSchedulerConfig;

namespace hierarchical_scheduler {

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
/// is chosen) the decompose/mode install. Default (no scoped keys) is a
/// flat DFS search, formation off -- deliberately flat; note the
/// historical default instead formed dom-tree subgraphs.
struct OccupancyConfig {
  FormationConfig formation; // strategy + mode + min-cut settings
  Search search = Search::kDfs;
  bool decompose = false;

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
  bool dump_subgraph_dag = true;     // dump region DAGs for the viewer
  bool scale_edge_latencies = false; // scale edge latency by target occupancy

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
