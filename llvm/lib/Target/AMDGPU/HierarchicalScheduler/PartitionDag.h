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

#include "BfsDpSettings.h"
#include "GCNRegPressure.h"
#include "ScheduleConstructor.h"
#include "Score.h"
#include "ScheduledSetTracker.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/ErrorHandling.h"
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

/// One directed edge in the PartitionDag: scheduling `scheduled_node`
/// on top of `source_partition`'s partial schedule yields the
/// partition of the PartitionNode that owns this edge.
struct PartitionEdge {
  PartitionNode *source_partition;
  const ScheduleNode *scheduled_node;
};

/// The worst (bottleneck) edge along a path through the dag. `score`
/// is the comparator (used everywhere the DP picks a best path) —
/// its meaning depends on the metric the dag was constructed with
/// (continuous-occupancy score for
/// kMaximizeContinuousRegisterOccupancyScore, integer occupancy
/// for kMaximizeRegisterOccupancy; both MAX-direction, higher =
/// better). `register_pressure` is the GCNRegPressure paired with
/// that bottleneck edge, carried so reconstruction / diagnostics
/// can report the actual pressure numbers behind the score.
struct PathBottleneck {
  int score = INT_MAX;
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
  /// bottleneck-edge metric (min metric-score along the path; pressure
  /// at that bottleneck edge). The metric is selected at dag
  /// construction; see PartitionDag's ctor.
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
  /// snapshots via NoHistoryClone).
  ///
  /// `settings` configures the search (see BfsDpSettings).
  /// `settings.metric` selects the per-edge score function and must
  /// be a MAX-direction metric (higher = better) — BFS-DP's
  /// bottleneck DP (max-over-paths of min-along-path edge score)
  /// only makes sense when higher is better. Supported:
  ///   - kMaximizeRegisterOccupancy: integer occupancy bracket
  ///       derived from peak register pressure. Coarse — many
  ///       distinct pressures map to the same integer level, so the
  ///       score-bound prune fires aggressively against any baseline
  ///       at the same level. Matches the production "replace input
  ///       only on strict occupancy improvement" intent.
  ///   - kMaximizeContinuousRegisterOccupancyScore: fine-grained,
  ///       distinguishes within-bracket pressure differences. Use
  ///       when a strict occupancy improvement isn't required — the
  ///       prune still fires when a path can't even match the
  ///       baseline's continuous score, but ties in occupancy are
  ///       distinguished by within-bracket pressure.
  /// Any other metric fatal-errors at construction.
  ///
  /// `settings.timeout_ms` is the per-region wall-clock budget; see
  /// Build() for what happens when it fires and BfsDpSettings for
  /// the value's meaning.
  ///
  /// Build() is called separately.
  PartitionDag(const ScheduleGraph *graph, const GCNSubtarget *st,
               const MachineFunction *mf, BfsDpSettings settings = {});

  /// BFS from the empty-set source to the all-scheduled sink:
  ///   - creates the source PartitionNode with a fresh BfsDp-preset
  ///     ScheduleConstructor;
  ///   - expands each layer via ExpandSource, dropping each source's
  ///     schedule_state once expansion completes;
  ///   - records the sink (the unique all-scheduled PartitionNode);
  ///   - calls ReconstructSchedule to populate schedule_.
  ///
  /// Returns true on success: sink was reached and schedule_ is
  /// populated. Returns false in two cases, both leaving
  /// GetSchedule empty so the caller falls back to the baseline
  /// schedule the seed represents:
  ///   - the score-bound prune (see SetInitialBestScore)
  ///     eliminated every path to the sink — no schedule strictly
  ///     beats the seed, and sink_ stays null;
  ///   - the per-region timeout (settings.timeout_ms) was exhausted
  ///     before the BFS drained — TimedOut() returns true,
  ///     distinguishing this from the prune case.
  ///
  /// Must be called exactly once per PartitionDag instance. A second
  /// call would re-traverse the search redundantly and produce a
  /// dag with duplicate work in nodes_; we fatal-error on entry if
  /// source_ is already set.
  ///
  /// After Build returns true, GetSchedule() and GetSink() are valid.
  /// GetSource() is always valid post-Build.
  bool Build();

  PartitionNode *GetSource() const { return source_; }
  PartitionNode *GetSink() const { return sink_; }

  /// The recovered schedule (source → sink ordering of ScheduleNodes).
  /// Only valid after a Build() that returned true; empty otherwise.
  ArrayRef<const ScheduleNode *> GetSchedule() const { return schedule_; }

  /// The recovered schedule as a fully-populated ScheduleConstructor:
  /// a fresh Default-preset constructor with GetSchedule()'s nodes
  /// Schedule()d into it in order. This is what most callers
  /// actually want — it can be handed to ApplyScheduleOrder and its
  /// trackers queried for pressure / occupancy / length, exactly
  /// like the ScheduleConstructor that DfsSearch::Run returns.
  /// Built by ReconstructSchedule at the end of a successful Build.
  /// Fatal-errors if Build() did not return true.
  const ScheduleConstructor &GetScheduleConstructor() const {
    if (!reconstructed_schedule_constructor_) {
      report_fatal_error(
          "PartitionDag::GetScheduleConstructor called when no "
          "schedule was reconstructed (Build did not return true)");
    }
    return *reconstructed_schedule_constructor_;
  }

  /// Number of Schedule() calls issued on PartitionNode schedule_states
  /// across the entire Build (matched by an equal number of Unschedule
  /// calls). Matches the counting convention of
  /// DfsSearch::ScheduleCallCount — Unschedule is not counted, so this
  /// is comparable apples-to-apples with the DFS counter.
  int GetScheduleCallCount() const { return schedule_call_count_; }

  /// Number of unique PartitionNodes in the dag (one per distinct
  /// scheduled-set encountered during Build, including source and sink).
  int GetPartitionNodeCount() const {
    return static_cast<int>(nodes_.size());
  }

  /// Number of completed BFS layers (= number of current/next layer
  /// swaps performed). For an N-node graph the source counts as level
  /// 0 and the sink is reached at level N, so this ends equal to N
  /// after a successful Build.
  int GetCurrentLevel() const { return current_level_; }

  /// Number of (src, next) extensions that were pruned by the
  /// score-bound check (see SetInitialBestScore) — the per-edge
  /// path bottleneck dropped strictly below the seed, so neither
  /// the successor partition nor the dag edge was created. Zero
  /// unless a seed score has been set.
  int GetPruneCount() const { return prune_count_; }

  /// True iff Build() stopped early because the settings.timeout_ms
  /// budget was exhausted before the BFS drained. False when no
  /// timeout was set, or the search completed within budget —
  /// whether it reached the sink or the score-bound prune emptied
  /// the frontier. Meaningful only after Build() has run.
  bool TimedOut() const { return timed_out_; }

  /// Score-bound prune (analog of DfsSearch's). Pre-seed with the
  /// score of a known baseline schedule; during Build, per-edge
  /// probes whose resulting path bottleneck is <= this seed are
  /// not followed. "Tied with seed" is pruned because a schedule
  /// with the same score as the baseline gives no improvement —
  /// the caller would take the baseline anyway. Skips creating the
  /// successor PartitionNode entirely; a later (strictly better)
  /// path to the same partition will create it on demand via
  /// FindOrInsert.
  ///
  /// If no schedule strictly beats the seed, Build returns false
  /// and GetSchedule returns empty — the caller falls back to the
  /// baseline schedule the seed represents.
  ///
  /// Default unset (INT_MIN sentinel) disables pruning entirely:
  /// `score <= INT_MIN` is never true for real scores.
  void SetInitialBestScore(int score) { initial_best_score_ = score; }

  /// Convenience overload: extract the score from `init`'s pressure
  /// tracker using this dag's metric, then SetInitialBestScore on
  /// the result. Use when the caller already has a baseline
  /// ScheduleConstructor (e.g., from replaying the input order
  /// through a fresh tracker) and doesn't want to spell out the
  /// metric a second time. `init` must be in the same test/production
  /// mode the dag will run in (so the extracted score is comparable
  /// to BFS-DP's per-edge scores).
  void SetInitialBestScore(const ScheduleConstructor &init) {
    SetInitialBestScore(
        init.GetPressureTracker().GetMetricScore(settings_.metric));
  }

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

  /// Enumerate `source_partition`'s ready instructions and call
  /// VisitSuccessor for each. After the loop, drops
  /// source_partition's schedule_state — it has been fully
  /// expanded and the snapshot is no longer needed (reconstruction
  /// reads best_incoming_edge, not schedule_state).
  void ExpandSource(PartitionNode *source_partition,
                    std::vector<PartitionNode *> &next_layer);

  /// Probe-schedule `scheduled_node` on source_partition's state,
  /// compute the resulting partition's per-metric edge score, hand
  /// the probe to FindOrInsert, then DP-merge into the returned
  /// PartitionNode's best_path_bottleneck / best_incoming_edge.
  /// Arg names match PartitionEdge's field names so the
  /// `PartitionEdge{source_partition, scheduled_node}` construction
  /// at the merge site reads as field-by-name init.
  void VisitSuccessor(PartitionNode *source_partition,
                      const ScheduleNode *scheduled_node,
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

  /// Convert a GCNRegPressure to an int score using
  /// settings_.metric's formula. Used both for per-edge edge_score
  /// computation in VisitSuccessor and for the bottleneck-pressure
  /// soundness assert. Fatal-errors on metric values rejected by
  /// the ctor.
  int ComputeScoreFromPressure(const GCNRegPressure &pressure) const;

  /// Input-order index for `node`: SUnit::NodeNum (LLVM's pre-RA
  /// scheduler emission order, register-pressure-aware) on
  /// scheduling-unit nodes; min over members' indices on proxies
  /// (matches EffectiveNodeNum in SearchPolicies.cpp). Returns 0
  /// for synthetic test nodes (no SUnit) and for proxies with no
  /// members.
  ///
  /// Used to break score ties in the DP merge: when multiple paths
  /// reach the same partition with the same metric score, prefer
  /// the path whose just-scheduled instruction is later in input
  /// order (higher index). Biases the recovered schedule toward
  /// the input schedule's ordering, which in production is already
  /// a reasonable register-pressure-aware schedule. Same trick as
  /// the SortKey.nid tiebreak in DfsMaximizeOccupancyPolicy and
  /// OptSched's NID heuristic.
  static int GetInputOrderIndex(const ScheduleNode *node);

  const ScheduleGraph *graph_;
  const GCNSubtarget *st_;
  const MachineFunction *mf_;

  /// Search configuration (metric + timeout). Set once at
  /// construction; see BfsDpSettings.
  const BfsDpSettings settings_;

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
  /// Build(). Empty when Build returned false.
  SmallVector<const ScheduleNode *> schedule_;

  /// Recovered schedule as a populated ScheduleConstructor. Built by
  /// ReconstructSchedule alongside schedule_ (a fresh Default-preset
  /// constructor with schedule_'s nodes Schedule()d in). nullopt when
  /// Build returned false. See GetScheduleConstructor.
  std::optional<ScheduleConstructor> reconstructed_schedule_constructor_;

  /// Test-only: per-topo-index VGPR deltas for synthetic-pressure
  /// shakedowns. Empty in production. When non-empty, CreateSourceNode
  /// applies them to the source's tracker via EnableTestModeForTest;
  /// from there the deltas propagate to every clone (NoHistoryClone
  /// preserves test_mode_ / test_vgpr_deltas_).
  std::vector<int> test_vgpr_deltas_;

  /// Stats — populated incrementally during Build.
  int schedule_call_count_ = 0;
  int current_level_ = 0;
  int prune_count_ = 0;

  /// Score-bound prune threshold (see SetInitialBestScore). INT_MIN
  /// sentinel means no seed → prune never fires.
  int initial_best_score_ = INT_MIN;

  /// Set by Build() when the timeout_ms_ budget was exhausted
  /// before the BFS drained. Read by TimedOut().
  bool timed_out_ = false;
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PARTITIONDAG_H
