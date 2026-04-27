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
// Currently supports scheduling-unit-only graphs. Subgraph proxies
// (the Approach-B handle for grouped nodes — see
// AMDGPUClusteringDesign.md) will be handled by a future Phase 2
// extension that adds Schedule/Unschedule dispatch on
// IsSubgraphProxy and a scope push/pop on proxy-schedule.
//
// Usage:
//   ScheduleConstructor sc(graph, subtarget, mri, tri, lis);
//   while (!sc.IsDone()) {
//       for (auto *node : sc.GetReadyList()) { ... }
//       sc.Schedule(chosen_node);
//   }
//
// Supports do/undo:
//   sc.Schedule(node_a);
//   sc.Schedule(node_b);
//   sc.Unschedule();  // undoes node_b
//   sc.Schedule(node_c);  // try node_c instead
//
// Ready list: a node is ready when all of its strong predecessors
// have been scheduled. Weak edges (cluster hints, etc.) do not
// block readiness. The list is maintained in a stable sorted
// order under a caller-supplied strict total order comparator
// (default: topo_index ascending). Schedule/Unschedule reverse
// each other exactly, so after a round-trip the list's contents
// AND indices are restored. This lets DFS iterate by index across
// Schedule/Unschedule pairs without snapshotting:
//
//   for (int i = 0; i < sc.GetReadyList().size(); ++i) {
//     const ScheduleNode *n = sc.GetReadyList()[i];
//     sc.Schedule(n);
//     Recurse();
//     sc.Unschedule();       // ready list back to identical state;
//                            // GetReadyList()[i] is still n
//   }
//
// Iterators/pointers into the current scope's ready list are NOT
// stable across Schedule — memmove shifts entries. Iterate by
// index, or snapshot explicitly via GetReadyListSnapshot.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULECONSTRUCTOR_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULECONSTRUCTOR_H

#include "GCNRegisterTracker.h"
#include "ScheduleGraph.h"
#include "ScheduleLengthTracker.h"
#include "ScheduledSetTracker.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include <vector>

namespace llvm {

class GCNSubtarget;
class LiveIntervals;
class MachineFunction;

namespace hierarchical_scheduler {

/// Criterion by which two ScheduleConstructor states are compared.
/// Used with ScheduleConstructor::IsBetterThan. Names are explicit
/// about direction (kMaximize*, kMinimize*) so the call site doesn't
/// have to remember which way each metric is "better".
enum class ScheduleMetric {
  /// Integer register occupancy (GetRegisterOnlyOccupancy).
  /// Coarse — schedules in the same occupancy bracket tie.
  kMaximizeRegisterOccupancy,

  /// Continuous register occupancy score (GetContinuousOccupancyScore).
  /// Smooth within brackets — useful when search needs to see
  /// progress toward the next higher bracket.
  kMaximizeContinuousRegisterOccupancyScore,

  /// Current schedule length in cycles.
  kMinimizeScheduleLength,

  /// Inverted register occupancy: lower GetRegisterOnlyOccupancy is
  /// "better." TEST-ONLY — used to drive a search toward worse
  /// register occupancy so we can verify search infrastructure
  /// (DFS, etc.) actually explores and selects against the input
  /// baseline. Not a useful production metric.
  kMinimizeRegisterOccupancy,

  /// Inverted continuous register occupancy score: lower
  /// GetContinuousOccupancyScore is "better." TEST-ONLY, parallel
  /// to kMinimizeRegisterOccupancy but uses the smooth score, so
  /// schedules that differ in within-bracket pressure (not just
  /// integer occupancy) are distinguishable. Useful for verifying
  /// DFS picks WORSE schedules even when no integer-occupancy
  /// cliff is crossed.
  kMinimizeContinuousRegisterOccupancyScore,
};

class ScheduleConstructor {
public:
  /// Comparator for ready-list ordering. Must be a strict total
  /// order — no two distinct nodes may compare equal. The default
  /// (topo_index ascending) satisfies this because topo_index is
  /// unique per node. Policies with multi-criterion ordering must
  /// still bottom out in a uniquely-identifying tiebreaker.
  ///
  /// Strict total order is required for index-based iteration to be
  /// stable across Schedule/Unschedule round-trips — ties would let
  /// a re-inserted element land at a different index.
  ///
  using ReadyComparator = bool (*)(const ScheduleNode *,
                                   const ScheduleNode *);

  /// Default ready-list comparator: topo_index ascending. Stateless,
  /// strict total order.
  static bool DefaultReadyComparator(const ScheduleNode *a,
                                     const ScheduleNode *b) {
    return a->GetTopoIndex() < b->GetTopoIndex();
  }

  /// Construct from a graph and target info. The graph must outlive
  /// this object. Reports fatal error if the graph contains group
  /// nodes (not yet supported at this level). `ready_cmp` defines the
  /// ready-list order; default is topo_index ascending.
  ScheduleConstructor(const ScheduleGraph &graph,
                      const GCNSubtarget &st,
                      const MachineFunction &mf,
                      const LiveIntervals &lis,
                      ReadyComparator ready_cmp = DefaultReadyComparator);

  /// Schedule a node. The node must be in the ready list.
  /// Updates register pressure, schedule length, ready list, and
  /// appends the node to the schedule order.
  void Schedule(const ScheduleNode *node);

  /// Schedule the node currently at the current scope's ready list
  /// at `index`. Skips the binary search used by Schedule(const
  /// ScheduleNode*) — DFS knows the index from its iteration loop,
  /// so it can erase directly. Otherwise identical to
  /// Schedule(const ScheduleNode*).
  void ScheduleByIndex(int index);

  /// Undo the last Schedule() call. Restores register pressure,
  /// schedule length, ready list, and removes the node from the
  /// schedule order.
  void Unschedule();

  /// True when all nodes have been scheduled. Uses the scope-stack
  /// form so it stays correct once subgraph proxies are introduced
  /// (a pushed scope means "in the middle of a subgraph," not done).
  bool IsDone() const {
    return scopes_.size() == 1 && scopes_[0].ready.empty();
  }

  /// The current scope's ready list — nodes whose strong predecessors
  /// are all scheduled AND that are visible in the currently-active
  /// scope. Maintained in sorted order under the constructor-supplied
  /// comparator.
  ///
  /// Iteration-across-mutation: iterate by INDEX if the loop body
  /// calls Schedule/Unschedule. After a round-trip the current
  /// scope's list has the same contents in the same order, so
  /// `ready_list[i]` refers to the same node. Pointers/iterators
  /// into the returned ArrayRef are NOT stable across Schedule
  /// (memmove shifts entries).
  ArrayRef<const ScheduleNode *> GetReadyList() const {
    return scopes_.back().ready;
  }

  /// Append the current scope's ready-list contents (in the
  /// maintained sort order) to `out`. Convenience for callers that
  /// want a stable copy to iterate across Schedule/Unschedule
  /// without using the index-based pattern.
  void GetReadyListSnapshot(
      SmallVectorImpl<const ScheduleNode *> &out) const {
    const auto &ready = scopes_.back().ready;
    out.append(ready.begin(), ready.end());
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
  const ScheduledSetTracker &GetScheduledSetTracker() const {
    return scheduled_set_tracker_;
  }

  /// Access the graph.
  const ScheduleGraph &GetGraph() const { return *graph_; }

  /// Number of nodes scheduled so far.
  int GetNumScheduled() const {
    return static_cast<int>(schedule_order_.size());
  }

  /// Cumulative count of Schedule / ScheduleByIndex calls on this
  /// constructor since construction. Schedule funnels through
  /// ScheduleByIndex, so this counts every actual scheduling
  /// operation without double-counting. Unschedule does NOT
  /// decrement (the count is effort spent, not depth). Useful for
  /// assessing search effort in DFS-style traversals.
  int64_t GetScheduleCallCount() const { return schedule_call_count_; }

  /// True if this schedule is strictly better than `other` under the
  /// given metric. Ties return false — callers that want "at least as
  /// good" should negate IsBetterThan with arguments swapped.
  bool IsBetterThan(const ScheduleConstructor &other,
                    ScheduleMetric metric) const;

  /// True when this region's register-only occupancy meets or
  /// exceeds the MachineFunction's currently-configured occupancy
  /// limit (hardware max, LDS, launch bounds, and any reductions
  /// from earlier passes/regions calling MFI->limitOccupancy).
  /// "AtOrAbove" rather than just "At" because register-only
  /// occupancy can exceed the function cap when structural factors
  /// (LDS, launch bounds) are binding below the register max — in
  /// that case extra register headroom doesn't translate to extra
  /// effective occupancy. A search can exit early either way.
  bool IsAtOrAboveFunctionOccupancyCeiling() const;

  /// Human-readable summary of current state.
  std::string Describe() const;

private:
  const ScheduleGraph *graph_;
  GCNRegisterTracker pressure_tracker_;
  ScheduleLengthTracker length_tracker_;
  ScheduledSetTracker scheduled_set_tracker_;

  /// Nodes scheduled so far, in order.
  SmallVector<const ScheduleNode *> schedule_order_;

  /// One active scheduling scope. Base scope has subgraph_proxy ==
  /// nullptr and covers the whole graph. Phase 0 always has exactly
  /// one scope; the scope-stack shape is in place now so subgraph
  /// integration (AMDGPUClusteringDesign.md Approach B) can push/pop
  /// scopes without further refactoring of ScheduleConstructor.
  struct SubgraphScheduleScope {
    /// The subgraph proxy whose members this scope is scheduling, or
    /// nullptr for the base scope.
    const ScheduleNode *subgraph_proxy;
    /// Nodes visible in this scope that are currently schedulable,
    /// maintained in sorted order under ready_comparator_. Inline
    /// capacity sized to cover typical ready-list sizes without
    /// heap spill.
    SmallVector<const ScheduleNode *, 64> ready;
  };

  /// Stack of active scheduling scopes. scopes_.back() is the current
  /// scope — the one DFS picks from and that Schedule/Unschedule
  /// operate on. Invariant: scopes_.size() >= 1; scopes_[0] is the
  /// base scope. Phase 2+ pushes additional scopes when subgraph
  /// proxies are scheduled (one scope per active subgraph).
  std::vector<SubgraphScheduleScope> scopes_;

  /// Comparator defining ready-list sort order within each scope.
  ReadyComparator ready_comparator_;

  /// Incremented by ScheduleByIndex. Tracks search effort; see
  /// GetScheduleCallCount.
  int64_t schedule_call_count_ = 0;

  /// Per-node count of strong predecessors not yet scheduled,
  /// indexed by ScheduleNode::GetTopoIndex(). Sized to graph.Size()
  /// at construction. When an entry reaches 0 the corresponding node
  /// enters its home scope's ready list.
  std::vector<int> remaining_strong_predecessors_by_topo_index_;

  /// Initialize remaining_strong_predecessors_by_topo_index_ and the
  /// base scope's ready list from the graph.
  void InitReadyList();

  /// Return the ready list of the scope that `node` belongs in (its
  /// "home scope"). Used by Release / Unrelease / Unschedule's
  /// re-insert — anywhere a node is transitioning into or out of
  /// ready-list membership.
  ///
  /// Routes via FindScopeOnStack(node->GetParentSubgraphProxy()):
  ///   - Top-level node (parent_subgraph_proxy == nullptr): finds
  ///     the base scope (its subgraph_proxy is also nullptr).
  ///   - Member of a currently-active subgraph X: finds the
  ///     pushed scope whose subgraph_proxy is X.
  ///
  /// Distinct from `scopes_.back().ready`, which is what
  /// Schedule / ScheduleByIndex operate on. Those sites are
  /// semantically "current scope" (DFS picks from the active scope);
  /// release sites are semantically "node's home scope."
  SmallVectorImpl<const ScheduleNode *> &
  GetReadyListForNode(const ScheduleNode *node) {
    return FindScopeOnStack(node->GetParentSubgraphProxy()).ready;
  }

  /// Walk scopes_ from top to find the scope whose subgraph_proxy
  /// equals `target_proxy`. report_fatal_error if not found —
  /// callers rely on the invariant that every node's
  /// parent_subgraph_proxy points to a scope currently on the
  /// stack at the moment its pred_count reaches 0 (the
  /// proxy→member artificial edge guarantees this for members,
  /// and the base scope's subgraph_proxy=nullptr matches every
  /// top-level node's parent_subgraph_proxy=nullptr).
  ///
  /// Top-down walk: same-scope releases (the common case for
  /// intra-subgraph successors when DFS is inside that subgraph)
  /// hit on iteration 1.
  SubgraphScheduleScope &
  FindScopeOnStack(const ScheduleNode *target_proxy);

  /// Return the index of `node` in `ready_list`, or -1 if absent.
  /// Binary search under ready_comparator_ (O(log K)).
  int GetReadyListIndexOf(
      const SmallVectorImpl<const ScheduleNode *> &ready_list,
      const ScheduleNode *node) const;

  /// Insert `node` into `ready_list` at its sorted position under
  /// ready_comparator_. Precondition: node is not already present.
  void ReadyListInsert(SmallVectorImpl<const ScheduleNode *> &ready_list,
                       const ScheduleNode *node);

  /// Erase the entry at `index` in `ready_list` (direct erase, no
  /// search).
  void ReadyListEraseAt(SmallVectorImpl<const ScheduleNode *> &ready_list,
                        int index);

  /// Erase `node` from `ready_list`, locating it via
  /// GetReadyListIndexOf. Precondition: node IS present. Used by
  /// Schedule(node) and UnreleaseSuccessors.
  void ReadyListErase(SmallVectorImpl<const ScheduleNode *> &ready_list,
                      const ScheduleNode *node);

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
  static int CountStrongPredecessors(const ScheduleNode *node);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULECONSTRUCTOR_H
