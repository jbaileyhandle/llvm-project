//===- ScheduledSetTracker.h - Track scheduled-set state --------*- C++ -*-===//
//
// Self-contained snapshot of the partial-schedule state that the
// history-based domination trackers (LengthHistoryTracker,
// PressureHistoryTracker — separate files) consult. Lives on
// ScheduleConstructor alongside ScheduleLengthTracker and
// GCNRegisterTracker; receives Schedule/Unschedule notifications in
// the same lockstep pattern. See AMDGPUHistoryDominationDesign.md.
//
// State maintained:
//   - Per-instruction 64-bit signatures, immutable after
//     construction (deterministic-seeded mt19937_64 — reproducible
//     across runs).
//   - Running prefix XOR signature: XOR of signatures of currently-
//     scheduled nodes. XOR is commutative, so this is a hash of the
//     scheduled SET regardless of scheduling order. O(1) maintenance.
//   - Scheduled bitset: one bit per node by topo index. Used by
//     history trackers as the exact-match key; the XOR signature
//     is a fast filter for hash-bucket lookup, the bitset confirms
//     the match.
//   - Frontier: unscheduled real nodes with at least one scheduled
//     real-instruction latency-bearing predecessor. Each frontier
//     entry holds a per-prefix lower bound on the node's earliest
//     issue cycle, computed from currently-scheduled latency-
//     bearing predecessors only.
//
// All maintained incrementally on Schedule/Unschedule notifications
// from ScheduleConstructor (matches the API used by
// ScheduleLengthTracker and GCNRegisterTracker).
//
// Predecessor cycles for the LB computation come from
// ScheduleLengthTracker (single source of truth — no duplicate
// cycle storage).
//
// Proxy handling:
//   - Signatures and bitset INCLUDE proxies. Two prefixes with the
//     same real-instruction set but different proxy state are at
//     different search states (P_end fired vs not), and dominance
//     comparisons should treat them as distinct — see §7 of the
//     design doc.
//   - Frontier EXCLUDES proxies. Proxy successors aren't entered;
//     proxy predecessors don't contribute to LB. Proxy
//     contributions to LB are 0 (no cycle delta — proxies don't
//     issue), so they carry no dominance information; including
//     them would just clutter the frontier with spurious zeros.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEDSETTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEDSETTRACKER_H

#include "ScheduleGraph.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/DenseMap.h"
#include <cstdint>
#include <utility>
#include <vector>

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleLengthTracker;

/// Per-frontier-node info maintained by ScheduledSetTracker.
struct FrontierEntry {
  /// Lower bound on this node's earliest issue cycle, computed
  /// from currently-scheduled latency-bearing predecessors only:
  ///   max over scheduled latency-bearing predecessors P of
  ///       (P.cycle + max(edge.Latency(),
  ///                      P.IssueSlotsConsumed())).
  /// 0 if no scheduled latency-bearing predecessors.
  ///
  /// This is a *valid* lower bound (the node really cannot be
  /// issued earlier than this) but NOT tight — unscheduled
  /// predecessors will impose additional constraints via their own
  /// LBs cascading through the unscheduled subgraph. Those aren't
  /// captured here.
  ///
  /// Ignoring the unscheduled cascade is sufficient for history
  /// dominance: two prefixes at the same partition see the same
  /// unscheduled subgraph, so the cascade applies identically;
  /// only the prefix-derived component (this value) varies. By
  /// monotonicity of max-over-paths, dominance on this value
  /// implies dominance on the tight LB everywhere downstream.
  int lower_bound;
};

class ScheduledSetTracker {
 public:
  /// Initialize per-instruction signatures from `graph`. Sized to
  /// graph->Size() at construction; the graph must not grow
  /// afterward (otherwise the signature array is undersized). In
  /// the production flow the tracker is constructed from a fully-
  /// built graph (BuildFromSUnits Phase 4 or DfsSearch ctor after
  /// formation), so the size is stable for the tracker's lifetime.
  ///
  /// `length_tracker` is the source of truth for scheduled cycles
  /// used by the frontier-LB computation.
  ///
  /// Both `graph` and `length_tracker` are stored by pointer; both
  /// must outlive this tracker and must not be null.
  ScheduledSetTracker(const ScheduleGraph *graph,
                      const ScheduleLengthTracker *length_tracker);

  /// Notification hooks called by ScheduleConstructor in lockstep
  /// with its Schedule/Unschedule (matches the API used by
  /// ScheduleLengthTracker and GCNRegisterTracker). LIFO contract:
  /// the sequence of Schedule/Unschedule calls must mirror
  /// ScheduleConstructor's; an Unschedule unwinds the most-recent
  /// matching Schedule.
  ///
  /// Call ordering relative to ScheduleConstructor's other trackers:
  ///   Schedule must run AFTER length_tracker_'s Schedule (so the
  ///     just-scheduled node's cycle is set when we read it).
  ///   Unschedule may run before or after length_tracker_'s
  ///     Unschedule — we never query the just-unscheduled node's
  ///     own cycle (we mark it unscheduled in our bitset first),
  ///     only other still-scheduled latency-bearing predecessors'
  ///     cycles.
  void Schedule(const ScheduleNode *node);
  void Unschedule(const ScheduleNode *node);

  /// 64-bit XOR hash of the currently-scheduled set, including
  /// proxies. O(1).
  int64_t GetPrefixSignature() const { return prefix_signature_; }

  /// Bit-per-node scheduled set, indexed by topo index. Includes
  /// proxies. Used by history trackers as the exact-match key
  /// behind the XOR-signature hash filter.
  const BitVector &GetScheduledSet() const { return scheduled_set_; }

  /// Frontier: topo_idx → entry. A node is in this map iff it is
  /// an unscheduled real instruction with at least one scheduled
  /// latency-bearing predecessor.
  const DenseMap<int, FrontierEntry> &GetFrontier() const {
    return frontier_;
  }

 private:
  /// Walk `node`'s predecessors and compute (count, LB) from
  /// currently-scheduled latency-bearing predecessors only. Count
  /// is returned alongside LB so callers can decide whether to
  /// erase (count == 0) or keep the entry (count > 0). Count is
  /// not stored anywhere — it's a transient value used only at
  /// the call site.
  std::pair<int, int> ComputeCountAndLowerBoundFromPredecessors(
      const ScheduleNode *node) const;

  /// On Schedule: walk `node`'s latency-bearing successors and
  /// max-merge `node`'s contribution into each successor's LB
  /// (inserting the entry if it didn't exist). Looks up `node`'s
  /// cycle from length_tracker_ internally.
  ///
  /// Caller must ensure `node` is a real instruction (not a
  /// proxy); Schedule's outer guard handles that filtering.
  void UpdateSuccessorFrontierForSchedule(const ScheduleNode *node);

  /// On Unschedule: walk `node`'s latency-bearing successors and
  /// recompute each successor's frontier entry from its currently-
  /// scheduled predecessors. Erase the entry if no scheduled
  /// predecessors remain; otherwise update its LB to the
  /// recomputed value.
  ///
  /// Caller must ensure `node` is a real instruction.
  void UpdateSuccessorFrontierForUnschedule(const ScheduleNode *node);

  /// On Unschedule: add `node` back to the frontier if it has at
  /// least one currently-scheduled latency-bearing predecessor.
  ///
  /// Caller must ensure `node` is a real instruction.
  void MaybeAddSelfToFrontier(const ScheduleNode *node);

  const ScheduleGraph *graph_;
  const ScheduleLengthTracker *length_tracker_;

  /// Per-node random 64-bit signatures, indexed by topo index.
  /// Initialized once in the ctor with a deterministic seed.
  std::vector<int64_t> per_node_signatures_;

  /// XOR of signatures of currently-scheduled nodes (proxies
  /// included).
  int64_t prefix_signature_ = 0;

  /// Bit per node by topo index; set if node is scheduled
  /// (proxies included).
  BitVector scheduled_set_;

  /// Frontier: topo_idx → entry. See GetFrontier.
  DenseMap<int, FrontierEntry> frontier_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEDSETTRACKER_H
