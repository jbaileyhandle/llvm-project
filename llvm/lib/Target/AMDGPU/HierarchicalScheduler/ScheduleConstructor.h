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
class MachineRegisterInfo;
class TargetRegisterInfo;

namespace hierarchical_scheduler {

class ScheduleConstructor {
public:
  /// Construct from a graph and target info. The graph must outlive
  /// this object. Reports fatal error if the graph contains group
  /// nodes (not yet supported at this level).
  ScheduleConstructor(const ScheduleGraph &graph,
                      const GCNSubtarget &st,
                      const MachineRegisterInfo &mri,
                      const TargetRegisterInfo &tri,
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
  const SmallDenseSet<const ScheduleNode *, 16> &GetReadyList() const {
    return ready_list_;
  }

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
