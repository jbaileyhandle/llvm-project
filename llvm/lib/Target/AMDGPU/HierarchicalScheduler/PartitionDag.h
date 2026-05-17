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
  /// min continuous-occupancy score encountered along that path.
  /// Higher = better. INT_MAX at the source so the first edge sets the
  /// value to the edge's score.
  int best_path_score = INT_MAX;

  /// The incoming edge on the best path discovered so far. nullopt at
  /// the source — terminates ReconstructSchedule.
  std::optional<PartitionEdge> best_incoming_edge;
};

class PartitionDag {
 public:
  /// `graph` and `st` must outlive this dag. Build() is called separately.
  PartitionDag(const ScheduleGraph *graph, const GCNSubtarget *st);

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

 private:
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
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PARTITIONDAG_H
