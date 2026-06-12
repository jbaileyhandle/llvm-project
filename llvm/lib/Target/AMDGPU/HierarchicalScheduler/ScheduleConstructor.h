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
#include "IlpTracker.h"
#include "ScheduleGraph.h"
#include "ScheduleLengthTracker.h"
#include "Score.h"
#include "ScheduledSetTracker.h"
#include "SearchStats.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include <optional>
#include <vector>

namespace llvm {

class GCNSubtarget;
class LiveIntervals;
class MachineFunction;

namespace hierarchical_scheduler {

/// Construction-time configuration for ScheduleConstructor. Holds
/// only knobs orthogonal to the recipe; length / ILP tracker
/// enablement is now derived from the optional recipe passed to
/// the SC ctor (see ScheduleConstructor::ScheduleConstructor).
struct ScheduleConstructorOptions {
  /// Forwarded to the inner GCNRegisterTracker's same-named flag.
  /// Default false to match the tracker's default — the only
  /// reader of GetPressureHistory() is shakedown code; nothing in
  /// production consults it.
  bool track_pressure_history = false;
};

class ScheduleConstructor {
public:
  /// Construct from a graph and target info. The graph must outlive
  /// this object. Reports fatal error if the graph contains group
  /// nodes (not yet supported at this level). The ready list is held
  /// in arbitrary insertion order; per-Recurse iteration priority is
  /// the search policy's responsibility (see Policy::
  /// FilterAndSortReadyList in SearchPolicies.h).
  ///
  /// Tracker enablement is recipe-driven:
  ///   - nullopt (default) -> length and ILP trackers BOTH enabled.
  ///     This is the "full-featured" mode -- correct for shakedowns,
  ///     input baselines, and anywhere the caller wants every tracker
  ///     available regardless of what the search compares on.
  ///   - present recipe -> length tracker enabled iff
  ///     `recipe.HasDim(kScheduleLength)`; ILP tracker enabled iff
  ///     `recipe.IsLengthPrimary()`.
  ///
  /// Caveats with the ILP rule -- it's empirical, not derived from
  /// the recipe's dim list:
  ///   * The rule exists because the length-min sort heuristic in
  ///     `DfsMinimizeLengthPolicy::FilterAndSortReadyList` consults
  ///     the ILP tracker even though `kIlpScore` isn't in its
  ///     comparison recipe. Tying ILP to `IsLengthPrimary()` covers
  ///     that dependency without forcing kIlpScore into recipes that
  ///     don't compare on it.
  ///   * `DfsMaximizeLengthPolicy` is length-primary but its sort
  ///     consults only the length tracker. The rule enables ILP for
  ///     it anyway -- wasted construction, no correctness issue.
  ///   * A future occupancy-primary policy that wanted ILP for sort
  ///     heuristics would break the rule. There is no such policy
  ///     today; adding one means revisiting this rule (perhaps by
  ///     letting the policy override SC's ILP gate, or by promoting
  ///     "sort-time tracker dependencies" to a recipe annotation).
  ///
  /// `options` carries the non-recipe knobs (today just
  /// `track_pressure_history`); a default-constructed Options is
  /// the right pick for nearly every caller.
  ScheduleConstructor(const ScheduleGraph &graph,
                      const GCNSubtarget &st,
                      const MachineFunction &mf,
                      std::optional<ScoreRecipe> recipe = std::nullopt,
                      ScheduleConstructorOptions options = {});

  /// Return a fresh ScheduleConstructor whose live state (register
  /// tracker live regs / cur pressure / remaining uses,
  /// scheduled-set bitset + frontier, ready list, remaining-strong-
  /// pred counts, scope stack) is copied from `this`, but with all
  /// past-step history cleared: schedule_order_ empty,
  /// schedule_call_count_ reset, and the inner GCNRegisterTracker
  /// constructed via its own NoHistoryClone (empty undo_stack_ and
  /// pressure_history_, default max_pressure_).
  ///
  /// Intended for BFS-DP: each LatticeNode owns one of these
  /// snapshots, capturing the state at partition P so the parent
  /// search can branch on each ready instruction without copying
  /// the parent's accumulated history.
  ///
  /// Fatal error if length / ILP tracking are enabled on the
  /// source (the inner trackers don't have history-clearing
  /// clones yet; only GCNRegisterTracker does). BFS-DP constructs
  /// the source with a pressure-primary recipe so both trackers
  /// are off and this path is well-defined.
  ScheduleConstructor NoHistoryClone() const;

  /// Schedule a node. The node must be in the ready list.
  /// Updates register pressure, schedule length, ready list, and
  /// appends the node to the schedule order. Returns the inner
  /// GCNRegisterTracker's edge-peak pressure for this Schedule (see
  /// GCNRegisterTracker::Schedule for the precise definition) so
  /// BFS-DP can read it directly without a separate query. Other
  /// callers (DFS) discard the return.
  GCNRegPressure Schedule(const ScheduleNode *node);

  /// Schedule the node currently at the current scope's ready list
  /// at `index`. Skips the binary search used by Schedule(const
  /// ScheduleNode*) — DFS knows the index from its iteration loop,
  /// so it can erase directly. Otherwise identical to
  /// Schedule(const ScheduleNode*).
  GCNRegPressure ScheduleByIndex(int index);

  /// Undo the last Schedule() call. Restores register pressure,
  /// schedule length, ready list, and removes the node from the
  /// schedule order.
  void Unschedule();

  /// Prepare for a new search Run() — drive the constructor back
  /// to its initial empty state and clear .current_run on per-run
  /// counters (.lifetime values persist). Same end-state as a
  /// freshly-constructed instance for everything except the
  /// lifetime totals. Used by DfsSearch::ResetForReuse so a single
  /// ScheduleConstructor can be re-used across multiple Run()
  /// calls in an outer loop.
  ///
  /// The rewind to empty state goes through the established
  /// Unschedule path so all bound trackers (length, pressure,
  /// scheduled-set) get their per-step rollback notifications
  /// rather than being side-channel-cleared.
  void Reset();

  /// True when all nodes have been scheduled. Uses the scope-stack
  /// form so it stays correct once subgraph proxies are introduced
  /// (a pushed scope means "in the middle of a subgraph," not done).
  bool IsDone() const {
    return scopes_.size() == 1 && scopes_[0].ready.empty();
  }

  /// Total cycles used by the schedule. Only meaningful when
  /// IsDone() — reports fatal error on a partial schedule, since
  /// "the schedule's length" isn't defined until the schedule is
  /// complete. For running cycle counts during construction, read
  /// GetLengthTracker().GetCurrentCycle() directly. Fatal error
  /// if length tracking was disabled at construction (the value
  /// isn't being maintained).
  int GetScheduleLength() const {
    if (!length_tracker_) {
      report_fatal_error(
          "ScheduleConstructor::GetScheduleLength called on a "
          "constructor whose recipe has no kScheduleLength dim");
    }
    if (!IsDone()) {
      report_fatal_error(
          "ScheduleConstructor::GetScheduleLength called on "
          "incomplete schedule");
    }
    return length_tracker_->GetCurrentCycle();
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
  /// Test-only mutable access. Used by DfsSearch::EnableTestModeForTest
  /// to plumb GCNRegisterTracker::EnableTestModeForTest through.
  /// Production code uses the const accessor below.
  GCNRegisterTracker &GetPressureTrackerForTest() {
    return pressure_tracker_;
  }

  const GCNRegisterTracker &GetPressureTracker() const {
    return pressure_tracker_;
  }
  /// Fatal error if length tracking was disabled at construction.
  const ScheduleLengthTracker &GetLengthTracker() const {
    if (!length_tracker_) {
      report_fatal_error(
          "ScheduleConstructor::GetLengthTracker called on a "
          "constructor whose recipe has no kScheduleLength dim");
    }
    return *length_tracker_;
  }
  /// Fatal error if ILP tracking was disabled at construction.
  const IlpTracker &GetIlpTracker() const {
    if (!ilp_tracker_) {
      report_fatal_error(
          "ScheduleConstructor::GetIlpTracker called on a "
          "constructor whose recipe is not length-primary "
          "(see ScheduleConstructor ctor doc for the ILP gating rule)");
    }
    return *ilp_tracker_;
  }

  /// Set the maximum schedule length the search will accept for
  /// the next stretch of work on this constructor and populate the
  /// length tracker's per-node max-schedule-cycle table accordingly.
  /// Idempotent: subsequent calls overwrite. Production callers
  /// pass min(iteration_target, best.length - 1). Fatal error if
  /// length tracking was disabled at construction.
  void SetMaxAcceptableScheduleLength(int max_acceptable_schedule_length) {
    if (!length_tracker_) {
      report_fatal_error(
          "ScheduleConstructor::SetMaxAcceptableScheduleLength "
          "called on a constructor whose recipe has no "
          "kScheduleLength dim");
    }
    length_tracker_->SetMaxAcceptableScheduleLength(
        max_acceptable_schedule_length);
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

  /// Read-only access to the schedule-call counter.
  /// `.current_run` is the count of Schedule / ScheduleByIndex
  /// calls during the current Run() (cleared by Reset).
  /// `.lifetime` is the cumulative total since construction
  /// (never cleared). Schedule funnels through ScheduleByIndex,
  /// so each scheduling operation is counted exactly once.
  /// Unschedule does NOT decrement — the count is effort spent,
  /// not depth.
  const DualRunAndLifetimeCounter &ScheduleCallCount() const {
    return schedule_call_count_;
  }

  /// Canonical comparable Score for this schedule under `recipe`. Same
  /// recipe on two schedules → Scores are directly comparable by < / >.
  /// Tracker-presence requirements per ScoreDimension: kScheduleLength
  /// requires length tracking enabled; kIlpScore requires ILP
  /// tracking enabled. Fatal error otherwise.
  Score GetScore(const ScoreRecipe &recipe) const;

  /// True if this schedule is strictly better than `other` under
  /// `recipe`. Ties return false — callers that want "at least as
  /// good" should negate IsBetterThan with arguments swapped.
  bool IsBetterThan(const ScheduleConstructor &other,
                    const ScoreRecipe &recipe) const {
    return GetScore(recipe) > other.GetScore(recipe);
  }

  /// True iff `*this`, the in-progress schedule, cannot produce a
  /// completion whose Score beats `other`'s on `recipe`'s bound-safe
  /// slot prefix. Backs the score-bound prune.
  ///
  /// Strict vs non-strict semantics are derived from the recipe:
  ///   - All recipe slots are bound-safe -> non-strict
  ///     (equality on the full recipe prefix means no improvement
  ///     possible).
  ///   - Recipe has additional non-bound-safe tiebreak slots ->
  ///     strict (equality on the bound-safe prefix doesn't preclude
  ///     improvement via the tiebreak slots in the suffix, so don't
  ///     prune ties).
  bool CompletionCannotImproveUpon(const ScheduleConstructor &other,
                                   const ScoreRecipe &recipe) const {
    int num_safe = recipe.NumLeadingOnlyWorseningSlots();
    bool strict = recipe.NumSlots() > num_safe;
    Score working_score = GetScore(recipe);
    Score other_score = other.GetScore(recipe);
    return strict
               ? working_score.IsWorseOnLeadingSlots(other_score, num_safe)
               : working_score.IsAtMostAsGoodOnLeadingSlots(other_score,
                                                            num_safe);
  }

  /// True when this region's register-only occupancy meets or
  /// exceeds the MachineFunction's currently-configured occupancy
  /// limit (hardware max, LDS, launch bounds, and any reductions
  /// from earlier passes/regions calling MFI->limitOccupancy).
  /// "AtOrAbove" rather than just "At" because register-only
  /// occupancy can exceed the function cap when structural factors
  /// (LDS, launch bounds) are binding below the register max — in
  /// that case extra register headroom doesn't translate to extra
  /// effective occupancy. A search can exit early either way.
  bool RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget() const;

  /// Strict-above counterpart: true only if peak register-only
  /// occupancy is *strictly greater* than the function ceiling.
  /// Used by the length-min refine-occupancy policy's
  /// ShouldEndSearch — when we've already exceeded the function
  /// target, there's no value in continuing to refine pressure
  /// within the same length.
  bool RegisterOnlyOccupancyExceedsFunctionOccupancyTarget() const;

  /// Same shape as RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget,
  /// but uses effective occupancy (max of register-only and
  /// structural floor) instead of raw register-only. Used by the
  /// length-pass ShouldBoundSearch gates: a raw register-only gate
  /// over-prunes in the spill regime (where every candidate has
  /// reg-only < floor); the effective version keeps the search alive
  /// there, letting the no-spill-regression gate and score-based
  /// dominance discriminate among spilling candidates. See
  /// GCNRegisterTracker::GetLaunchFloorClampedOccupancy for the semantics.
  bool LaunchFloorClampedOccupancyIsAtOrAboveFunctionOccupancyTarget() const;

  /// Result of cross-checking the candidate schedule's register
  /// pressure with LLVM's tracker. See VerifyPressureWithLlvmTracker.
  struct LlvmTrackerVerification {
    GCNRegPressure llvm_peak;
    GCNRegPressure ours_peak;
    int llvm_occupancy = 0;
    int ours_occupancy = 0;
    /// True iff llvm_occupancy >= ours_occupancy — LLVM confirms our
    /// tracker didn't over-estimate occupancy. DfsSearch rejects the
    /// candidate when false (our tracker over-claimed; committing would
    /// silently drop occupancy). A tracker-honesty check, not a goal
    /// check — the occupancy goal is IsBetterThan / ShouldEndSearch.
    bool tracker_confirmed = false;
  };

  /// Run LLVM's GCNUpwardRPTracker over this schedule's order to
  /// compute the LLVM-ground-truth peak pressure, then compare with
  /// our own tracker. Used as a per-candidate verification in
  /// DfsSearch before accepting a candidate as new best — guards
  /// against silently letting an occupancy-dropping schedule
  /// through if our tracker's pressure estimate disagrees with
  /// LLVM's. Walks the schedule order backward via recede(); cost
  /// is O(N) per call. Callers that don't have direct access to
  /// MF/LIS can store the result; it's a value type.
  LlvmTrackerVerification VerifyPressureWithLlvmTracker(
      const MachineFunction &mf, const LiveIntervals &lis) const;

  /// Human-readable summary of current state.
  std::string Describe() const;

private:
  /// Tag type for the NoHistoryClone-private constructor below.
  struct NoHistoryCloneTag {};

  /// Private constructor used by NoHistoryClone. Member-init-lists
  /// only the fields the clone keeps (pressure_tracker via its own
  /// NoHistoryClone, scheduled_set_tracker, scopes including the
  /// ready list, remaining-strong-pred counts, and the bookkeeping
  /// fields). Leaves length_tracker_ / ilp_tracker_ as empty
  /// optionals (the public NoHistoryClone preconditions both off),
  /// schedule_order_ empty, and schedule_call_count_ defaulted —
  /// no wasted copy of history-bearing fields.
  ScheduleConstructor(const ScheduleConstructor &source,
                      NoHistoryCloneTag);

  /// Drive the constructor back to its initial empty state by
  /// repeatedly invoking Unschedule(). Internal helper used by
  /// Reset(); not exposed publicly because callers should always
  /// go through Reset() (which also clears per-run counters).
  /// No-op if already empty.
  void UnscheduleAll();

  /// Read the raw value of `dim` from the appropriate bound tracker
  /// (pressure / length / ILP). GetScore uses this to populate each
  /// recipe slot before polarity is applied inside Score::Make.
  /// Fatal-errors if the dim requires a tracker that wasn't enabled
  /// at construction time (e.g., kScheduleLength with length
  /// tracking off).
  int64_t GetScoreDimensionValue(ScoreDimension dim) const;

  const ScheduleGraph *graph_;
  ScheduleConstructorOptions options_;
  GCNRegisterTracker pressure_tracker_;
  /// std::optional so we can decline to build a length/ILP tracker
  /// when the constructor's recipe doesn't request it. See the
  /// public ctor for the recipe-driven gating rule. Schedule /
  /// Unschedule / Reset and the public accessors gate on
  /// has_value().
  std::optional<ScheduleLengthTracker> length_tracker_;
  std::optional<IlpTracker> ilp_tracker_;
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
    /// Nodes visible in this scope that are currently schedulable.
    /// Maintained in topo_index ascending order so insert/lookup
    /// are O(log K). The maintenance order is fixed (not policy-
    /// tunable); per-Recurse iteration priority is the search
    /// policy's responsibility (see Policy::FilterAndSortReadyList
    /// in SearchPolicies.h, which produces a per-Recurse snapshot
    /// in policy-defined order). Inline capacity sized to cover
    /// typical ready-list sizes without heap spill.
    SmallVector<const ScheduleNode *, 64> ready;
  };

  /// Stack of active scheduling scopes. scopes_.back() is the current
  /// scope — the one DFS picks from and that Schedule/Unschedule
  /// operate on. Invariant: scopes_.size() >= 1; scopes_[0] is the
  /// base scope. Phase 2+ pushes additional scopes when subgraph
  /// proxies are scheduled (one scope per active subgraph).
  std::vector<SubgraphScheduleScope> scopes_;

  /// Incremented by ScheduleByIndex. .current_run is cleared by
  /// Reset; .lifetime persists. Tracks search effort; see
  /// ScheduleCallCount().
  DualRunAndLifetimeCounter schedule_call_count_;

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
  /// Binary search by topo_index (O(log K)).
  int GetReadyListIndexOf(
      const SmallVectorImpl<const ScheduleNode *> &ready_list,
      const ScheduleNode *node) const;

  /// Insert `node` into `ready_list` at its sorted position by
  /// topo_index ascending. Precondition: node is not already present.
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
