//===- ScheduleConstructor.h - Combined scheduling interface ----*- C++ -*-===//
//
// Unified interface for incrementally constructing a schedule. Wraps
// register pressure tracking (GCNRegisterTracker) and schedule length
// tracking (ScheduleLengthTracker), maintains the ready list, and
// records the schedule order.
//
// Copyable — for search algorithms (beam search) that need to branch
// and explore multiple schedule orderings from the same state.
//
// Currently supports leaf-only graphs. Group nodes (subgraphs) will
// be handled by a future HierarchicalScheduleConstructor layer that
// decomposes groups into leaf sequences and delegates to this class.
//
// Usage:
//   ScheduleConstructor sc(graph, subtarget, mri, tri, lis);
//   while (!sc.IsDone()) {
//       for (auto *node : sc.GetReadyList()) { ... }
//       sc.Schedule(chosen_node);
//   }
//
// If the loop body calls Schedule/Unschedule while iterating (e.g.,
// enumerating alternatives), iterate a GetReadyListSnapshot() copy
// instead — Schedule/Unschedule mutates the underlying set and
// invalidates iterators.
//   // sc.GetScheduleOrder() has the full schedule
//   // sc.GetPressureTracker() / sc.GetLengthTracker() have metrics
//
// Supports do/undo:
//   sc.Schedule(node_a);
//   sc.Schedule(node_b);
//   sc.Unschedule();  // undoes node_b
//   sc.Schedule(node_c);  // try node_c instead
//
// Ready list: a node is ready when all of its strong predecessors
// have been scheduled. Weak edges (cluster hints, etc.) do not
// block readiness.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULECONSTRUCTOR_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULECONSTRUCTOR_H

#include "GCNRegisterTracker.h"
#include "ScheduleGraph.h"
#include "ScheduleLengthTracker.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"

namespace llvm {

class GCNSubtarget;
class LiveIntervals;
class MachineFunction;

namespace hierarchical_scheduler {

/// Criterion by which two ScheduleConstructor states are compared.
/// Used with ScheduleConstructor::IsBetterThan.
enum class ScheduleMetric {
  /// Integer register occupancy (GetRegisterOccupancy). Coarse:
  /// schedules in the same occupancy bracket tie. Higher is better.
  kRegisterOccupancy,

  /// Continuous register occupancy score (GetContinuousOccupancyScore).
  /// Smooth within brackets — useful when search needs to see
  /// progress toward the next higher bracket. Higher is better.
  kContinuousRegisterOccupancyScore,

  /// Current schedule length in cycles. Lower is better.
  kScheduleLength,
};

class ScheduleConstructor {
public:
  /// Construct from a graph and target info. The graph must outlive
  /// this object. Reports fatal error if the graph contains group
  /// nodes (not yet supported at this level).
  ScheduleConstructor(const ScheduleGraph &graph,
                      const GCNSubtarget &st,
                      const MachineFunction &mf,
                      const LiveIntervals &lis);

  /// Schedule a node. The node must be in the ready list.
  /// Updates register pressure, schedule length, ready list, and
  /// appends the node to the schedule order.
  void Schedule(const ScheduleNode *node);

  /// Undo the last Schedule() call. Restores register pressure,
  /// schedule length, ready list, and removes the node from the
  /// schedule order.
  void Unschedule();

  /// True when all nodes have been scheduled.
  bool IsDone() const {
    return static_cast<int>(schedule_order_.size()) == graph_->Size();
  }

  /// The current ready list — nodes whose strong predecessors are
  /// all scheduled.
  ///
  /// WARNING: Do NOT iterate this set across calls to
  /// Schedule()/Unschedule(). Those mutate the set (erase the
  /// scheduled node, insert newly-released successors) which
  /// invalidates iterators. For loops that schedule and unschedule,
  /// call GetReadyListSnapshot() to get a stable copy and iterate
  /// that instead.
  const SmallDenseSet<const ScheduleNode *, 16> &GetReadyList() const {
    return ready_list_;
  }

  /// Append the current ready-list contents to `out`. Use this
  /// instead of iterating `GetReadyList()` directly when the loop
  /// body will call `Schedule()`/`Unschedule()` — those mutate the
  /// set and invalidate iterators. The snapshot is stable across
  /// subsequent Schedule/Unschedule pairs, and the pointer values
  /// are unaffected (ScheduleNode storage is graph-owned and
  /// stable). The order in the snapshot is the set's current
  /// iteration order, which is unspecified; callers that need a
  /// deterministic ordering should sort the result.
  void GetReadyListSnapshot(
      SmallVectorImpl<const ScheduleNode *> &out) const;

  /// The schedule order built so far.
  ArrayRef<const ScheduleNode *> GetScheduleOrder() const {
    return schedule_order_;
  }

  /// Access the underlying trackers for querying metrics.
  const GCNRegisterTracker &GetPressureTracker() const {
    return pressure_tracker_;
  }
  const ScheduleLengthTracker &GetLengthTracker() const {
    return length_tracker_;
  }

  /// Access the graph.
  const ScheduleGraph &GetGraph() const { return *graph_; }

  /// Number of nodes scheduled so far.
  int GetNumScheduled() const {
    return static_cast<int>(schedule_order_.size());
  }

  /// True if this schedule is strictly better than `other` under the
  /// given metric. Ties return false — callers that want "at least as
  /// good" should negate IsBetterThan with arguments swapped.
  bool IsBetterThan(const ScheduleConstructor &other,
                    ScheduleMetric metric) const;

  /// True when this region's register occupancy has reached the
  /// function-level ceiling (hardware max, LDS, launch bounds, and
  /// any reductions from earlier regions). At this point no further
  /// register improvements can raise actual occupancy — a search can
  /// exit early.
  bool IsAtOccupancyCeiling() const;

  /// Human-readable summary of current state.
  std::string Describe() const;

private:
  const ScheduleGraph *graph_;
  GCNRegisterTracker pressure_tracker_;
  ScheduleLengthTracker length_tracker_;

  /// Nodes scheduled so far, in order.
  SmallVector<const ScheduleNode *> schedule_order_;

  /// Nodes ready to be scheduled (all strong predecessors done).
  SmallDenseSet<const ScheduleNode *, 16> ready_list_;

  /// Per-node count of strong predecessors not yet scheduled.
  /// When this reaches 0, the node enters the ready list.
  DenseMap<const ScheduleNode *, int> remaining_strong_preds_;

  /// Initialize remaining_strong_preds_ and ready_list_ from the graph.
  void InitReadyList();

  /// Decrement strong-pred counts for the node's successors. If any
  /// successor's count reaches 0, add it to the ready list. A node
  /// may have multiple strong edges to the same successor; each edge
  /// decrements separately, and the successor is added to the ready
  /// list only when its count reaches exactly 0.
  void ReleaseSuccessors(const ScheduleNode *node);

  /// Reverse ReleaseSuccessors: for each strong successor edge, if
  /// the successor's count is currently 0, remove it from the ready
  /// list (it's about to become non-ready). Then increment the count.
  /// A successor with multiple edges from this node is removed from
  /// the ready list on the first edge (when count is 0) and
  /// subsequent edges just increment.
  void UnreleaseSuccessors(const ScheduleNode *node);

  /// Count strong predecessors for a node.
  static int CountStrongPreds(const ScheduleNode *node);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULECONSTRUCTOR_H
