//===- PartitionDag.h - BFS-DP partition graph for min-pressure -*- C++ -*-===//
//
// Breadth-first / dynamic-programming search over the *partition dag*
// of a single region. A PartitionNode represents one equivalence class
// of partial schedules: all orderings that result in the same scheduled
// set of instructions. Equivalence is keyed by PartitionKey (signature
// + scheduled-set bitset), so orderings that produce the same set
// collapse to one PartitionNode.
//
// Build proceeds BFS-by-layer from the empty-set source to the all-
// scheduled sink. At each PartitionNode we keep:
//   - the slim ScheduleConstructor snapshot needed to enumerate
//     successor extensions (dropped once the node has been expanded);
//   - the best-path DP value (highest "min continuous-occupancy score
//     along the path" found so far);
//   - the incoming edge on that best path (for schedule reconstruction).
//
// After Build() completes, ReconstructSchedule walks sink → source via
// best_incoming_edge to recover the chosen ordering, exposed via
// GetSchedule().
//
// See: AMDGPUPartitionLatticeDPDesign.md (§ "BFS-DP for occupancy").
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PARTITIONDAG_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PARTITIONDAG_H

#include "GCNRegPressure.h"
#include "ScheduleConstructor.h"
#include "ScheduledSetTracker.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include <climits>
#include <memory>
#include <optional>
#include <vector>

namespace llvm {

class GCNSubtarget;
class MachineFunction;

namespace hierarchical_scheduler {

class ScheduleGraph;
class ScheduleNode;

struct PartitionNode;

/// One directed edge in the PartitionDag: scheduling `scheduled` on top
/// of `source`'s partial schedule yields the partition of the
/// PartitionNode that owns this edge.
struct PartitionEdge {
  PartitionNode *source;
  const ScheduleNode *scheduled;
};

/// The worst (bottleneck) edge along a path through the dag. The
/// continuous-occupancy score is the comparator (used everywhere the
/// DP picks a best path); register_pressure is the GCNRegPressure
/// paired with that bottleneck edge, carried alongside the score so
/// reconstruction / diagnostics can report the actual pressure
/// numbers behind the score, not just the score itself.
struct PathBottleneck {
  int continuous_occupancy_score = INT_MAX;
  GCNRegPressure register_pressure;
};

/// One node in the PartitionDag. Identity (PartitionKey) lives in the
/// dag's lookup map, not here.
struct PartitionNode {
  /// Slim ScheduleConstructor snapshot — the state needed to enumerate
  /// successor extensions. Populated when the node is created (by
  /// PartitionDag::FindOrInsert on a miss), reset by
  /// PartitionDag::ExpandSource once the node has been fully expanded.
  /// Empty at the source (the dag populates it on construction) and at
  /// every node after expansion.
  std::optional<ScheduleConstructor> schedule_state;

  /// DP value: max over all paths reaching this PartitionNode of the
  /// bottleneck-edge metric (min continuous-occupancy score along the
  /// path; pressure at that bottleneck edge).
  ///
  /// Default {INT_MAX, zeroed-pressure}:
  ///   - On the source, never overwritten (no incoming edges). For
  ///     the first edge out: min(INT_MAX, edge_score) = edge_score,
  ///     so no special case is needed.
  ///   - On non-source nodes, never read until the first writer
  ///     overwrites it. First-writer detection keys off
  ///     best_incoming_edge.has_value(), not the score value.
  PathBottleneck best_path_bottleneck;

  /// The incoming edge on the best path discovered so far. nullopt at
  /// the source — terminates ReconstructSchedule.
  std::optional<PartitionEdge> best_incoming_edge;
};

class PartitionDag {
 public:
  /// `graph`, `st`, and `mf` must outlive this dag. `mf` is needed
  /// only to construct the source PartitionNode's initial
  /// ScheduleConstructor (subsequent PartitionNodes get their
  /// snapshots via NoHistoryClone). Build() is called separately.
  PartitionDag(const ScheduleGraph *graph, const GCNSubtarget *st,
               const MachineFunction *mf);

  /// BFS from the empty-set source to the all-scheduled sink:
  ///   - creates the source PartitionNode with a fresh BfsDp-preset
  ///     ScheduleConstructor;
  ///   - expands each layer via ExpandSource, dropping each source's
  ///     schedule_state once expansion completes;
  ///   - records the sink (the unique all-scheduled PartitionNode);
  ///   - calls ReconstructSchedule to populate schedule_.
  ///
  /// After Build returns, GetSchedule() and GetSource()/GetSink() are
  /// valid. Build is intended to be called exactly once on a given
  /// PartitionDag instance.
  void Build();

  PartitionNode *GetSource() const { return source_; }
  PartitionNode *GetSink() const { return sink_; }

  /// The recovered schedule (source → sink ordering of ScheduleNodes).
  /// Only valid after Build() has completed.
  ArrayRef<const ScheduleNode *> GetSchedule() const { return schedule_; }

  /// Test-only: enable delta-based synthetic pressure on the source
  /// PartitionNode's GCNRegisterTracker. Deltas propagate to every
  /// cloned PartitionNode via GCNRegisterTracker::NoHistoryClone (which
  /// preserves test_mode_ / test_vgpr_deltas_). Must be called before
  /// Build(). Stores the deltas; CreateSourceNode applies them after
  /// constructing the source's ScheduleConstructor.
  void EnableTestModeForTest(const std::vector<int> &per_node_vgpr_deltas) {
    test_vgpr_deltas_ = per_node_vgpr_deltas;
  }

 private:
  /// Create the empty-set source PartitionNode: allocate it, build
  /// its initial BfsDp-preset ScheduleConstructor, register it in
  /// partition_node_by_key_, and set source_. Returns the new
  /// PartitionNode so Build can seed current_layer with it. Called
  /// exactly once, at the start of Build.
  PartitionNode *CreateSourceNode();

  /// Enumerate `src`'s ready instructions and call VisitSuccessor for
  /// each. After the loop, drops src's schedule_state — src has been
  /// fully expanded and the snapshot is no longer needed (reconstruction
  /// reads best_incoming_edge, not schedule_state).
  void ExpandSource(PartitionNode *src,
                    std::vector<PartitionNode *> &next_layer);

  /// Probe-schedule `next` on a NoHistoryClone of src's state, compute
  /// the resulting partition's continuous-occupancy edge score, hand
  /// the probe to FindOrInsert, then DP-merge into the returned
  /// PartitionNode's best_path_score / best_incoming_edge.
  void VisitSuccessor(PartitionNode *src, const ScheduleNode *next,
                      std::vector<PartitionNode *> &next_layer);

  /// Look up the partition identified by probe_state's PartitionKey.
  ///   Hit: return the existing PartitionNode unchanged; probe_state is
  ///        discarded by the caller.
  ///   Miss: NoHistoryClone probe_state into a new PartitionNode,
  ///        register it in partition_node_by_key_, push it to
  ///        `next_layer` so the BFS will expand it on the next iteration,
  ///        record it as the sink if its scheduled-set is all-ones, and
  ///        return the new node.
  PartitionNode *FindOrInsert(const ScheduleConstructor &probe_state,
                              std::vector<PartitionNode *> &next_layer);

  /// Walk sink → source via best_incoming_edge, collecting
  /// `scheduled` ScheduleNodes, then reverse so the result is in
  /// scheduling order (source → sink). Populates schedule_.
  void ReconstructSchedule();

  const ScheduleGraph *graph_;
  const GCNSubtarget *st_;
  const MachineFunction *mf_;

  /// Stable storage of all PartitionNodes. unique_ptr because
  /// partition_node_by_key_ borrows pointers into this vector — a
  /// reallocation on push_back would invalidate them. unique_ptr<T>
  /// is stored by value, so the pointers it owns stay put across
  /// vector growth.
  std::vector<std::unique_ptr<PartitionNode>> nodes_;

  /// Identity → PartitionNode lookup. Borrows pointers from nodes_.
  DenseMap<PartitionKey, PartitionNode *> partition_node_by_key_;

  PartitionNode *source_ = nullptr;
  PartitionNode *sink_ = nullptr;

  /// Recovered schedule, populated by ReconstructSchedule at the end of
  /// Build().
  SmallVector<const ScheduleNode *> schedule_;

  /// Test-only: per-topo-index VGPR deltas for synthetic-pressure
  /// shakedowns. Empty in production. When non-empty, CreateSourceNode
  /// applies them to the source's tracker via EnableTestModeForTest;
  /// from there the deltas propagate to every clone (NoHistoryClone
  /// preserves test_mode_ / test_vgpr_deltas_).
  std::vector<int> test_vgpr_deltas_;
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PARTITIONDAG_H
