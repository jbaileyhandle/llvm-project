//===- SearchPolicies.h - Per-pass search policies ------------*- C++ -*-===//
//
// Policies parameterize a search (DfsSearch, future AcoSearch, etc.)
// for a specific pass objective. A policy is search-strategy-coupled
// by design — bounding and ordering heuristics that are sound for DFS
// may not be sound for ACO or other strategies. Policy class names
// therefore telegraph the coupling (e.g., DfsMaximizeOccupancyPolicy).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHPOLICIES_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHPOLICIES_H

#include "LengthHistoryTracker.h"
#include "PressureHistoryTracker.h"
#include "ScheduleConstructor.h"
#include "SubgraphFormation.h"
#include "llvm/ADT/SmallVector.h"

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleNode;

// Common defaults for all search policies (e.g. FilterAndSortReadyList).
// Static methods are inherited (not virtual): a derived policy that
// doesn't override one gets the base's version.
class SearchPolicyBase {
 public:
  // Per-Recurse iteration shape and order. DfsSearch calls this
  // once at each Recurse() entry and iterates the resulting vector
  // via ScheduleConstructor::Schedule(node) — so a policy can:
  //   - filter the ready list down to a subset (omitted nodes are
  //     not scheduled in this Recurse iteration);
  //   - impose any priority order, including ones that depend on
  //     dynamic per-search state (min/max schedule cycles, register
  //     pressure, etc.).
  //
  // Default: copy the ready list as-is. The underlying list is
  // maintained in topo_index ascending order by ScheduleConstructor,
  // so a no-op default reproduces the previous static-order
  // behavior. Concrete policies override for filtering or dynamic
  // priority.
  static void FilterAndSortReadyList(
      const ScheduleConstructor &working,
      SmallVectorImpl<const ScheduleNode *> &out) {
    ArrayRef<const ScheduleNode *> ready = working.GetReadyList();
    out.assign(ready.begin(), ready.end());
  }

  // No per-policy configuration constants. Each derived policy
  // declares its objective via a single `kScoreRecipe` constant; the
  // recipe's IsLengthPrimary / IsLengthMaxMode / RefinesAtSameLength
  // / HasDim queries drive all the per-dim plumbing (LHT/PHT
  // activation, length-axis direction, refinement bound relaxation,
  // LHT dim-inclusion flags) in DfsSearch and the history trackers.
  // Adding a new dimension or composite recipe doesn't add per-policy
  // boilerplate.
};

// Policy for DFS when the objective is to minimize schedule length for
// a single region, SUBJECT TO not dropping the region's register-only
// occupancy below the function-wide ceiling set by an earlier
// occupancy pass.
class DfsMinimizeLengthPolicy : public SearchPolicyBase {
 public:
  static constexpr ScoreRecipe kScoreRecipe =
      score_recipes::kMinimizeScheduleLength;

  // Override SearchPolicyBase: two-level filter+sort, modelled on
  // OptSched's cycle-by-cycle window-then-priority pattern.
  //
  //   Level 1 — no-bubble candidates: ready nodes whose effective
  //   min_schedule_cycle <= current_cycle. These can be scheduled
  //   at current_cycle with zero bubble. Sort ascending by
  //   effective max_schedule_cycle (most deadline-pressured first),
  //   tiebreak by topo_index.
  //
  //   Level 2 — bubble fallback: only if level 1 is empty (no ready
  //   node can issue at current_cycle without stalling). Take all
  //   ready nodes; sort ascending by effective min_schedule_cycle
  //   (smallest forced bubble first), then by effective
  //   max_schedule_cycle ascending (most deadline-pressured among
  //   same-min), then topo_index.
  //
  // Effective min/max for a scheduling-unit node is its own value
  // from the length tracker. For a subgraph proxy, it's the min
  // (smallest) of the corresponding values across the proxy's
  // subgraph members — so the proxy is treated as a composite of
  // its members.
  //
  // Falls back to topo-only when no max acceptable schedule length
  // is set (max_schedule_cycle would be undefined). DfsSearch
  // always sets one for this policy, so the fallback only matters
  // for the corner case of a search with no target seed.
  static void FilterAndSortReadyList(
      const ScheduleConstructor &working,
      SmallVectorImpl<const ScheduleNode *> &out);

  // Bound the current subtree if any of:
  //   (a) the working schedule's length lower bound exceeds the
  //       max acceptable schedule length stored on working's length
  //       tracker (read via GetMaxAcceptableScheduleLength) — i.e.,
  //       no completion of working can yield a schedule we'd accept.
  //       The max acceptable length is maintained by DfsSearch as
  //       min(requested_target_length, best.length - 1): we accept
  //       a completion if it's both at or below the iteration's
  //       target AND strictly better than current best; or
  //   (b) the working schedule's register-only occupancy has dropped
  //       below the function occupancy target — no completion can
  //       recover, and we must not degrade occupancy; or
  //   (c) (when kUseLengthHistoryPruning is true) some prior visit
  //       to this same scheduled-set partition recorded a state
  //       that dominates the current prefix on every Pareto
  //       dimension — see AMDGPUHistoryDominationDesign.md §5.
  //
  // SOUNDNESS:
  //   (a) LB is monotonically non-decreasing as nodes are scheduled
  //       (current_cycle + num_unscheduled never decreases, and the
  //       running max of scheduled_cycle + cp + 1 only grows), so
  //       once LB exceeds max acceptable length, every completion's
  //       length will too.
  //   (b) Peak register pressure is monotonically non-decreasing, so
  //       register-only occupancy is monotonically non-increasing.
  //       Once working drops below the function ceiling, no
  //       completion restores it.
  //   (c) See LengthHistoryTracker class comment.
  //
  // SIDE EFFECT: when (c) does NOT prune, the current prefix is
  // recorded in `length_history` for future-sibling comparison.
  // (IsDominatedElseInsert combines the check and record so we
  // don't pay two bucket lookups per visit.)
  //
  // `pressure_history` parameter is unused here — present only
  // because DfsSearch invokes ShouldBoundSearch with a uniform
  // signature across policies; this policy doesn't opt into
  // pressure-history pruning.
  //
  static bool ShouldBoundSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor,
      LengthHistoryTracker &length_history,
      PressureHistoryTracker &pressure_history);

  // End the search globally once best matches the graph-level length
  // floor (max(NumSchedulingUnits, cp_length + 1)) — no schedule can
  // be shorter,
  // so further exploration is pointless. The bound check in
  // ShouldBoundSearch handles incremental pruning; this is the cheap
  // "we hit the optimum" exit.
  static bool ShouldEndSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);

  // Enable length history-domination pruning. DFS records each
  // visited prefix's (end_cycle, frontier-LBs) in a per-partition
  // Pareto frontier; on a later visit to the same partition, if any
  // recorded entry dominates the current prefix on every dimension,
  // the subtree is pruned. See AMDGPUHistoryDominationDesign.md §5.
  static constexpr bool kUseLengthHistoryPruning = true;
};

// Length-min variant that, after finding a length-optimal schedule,
// continues exploring same-length completions to refine continuous
// register-occupancy score. Inherits everything from
// DfsMinimizeLengthPolicy except:
//   - kMetric: kMinimizeScheduleLengthThenMaximizeContinuousOccupancyScore
//     — Score has length asc primary, continuous register occupancy
//     desc as the same-length tiebreak. (The base
//     kMinimizeScheduleLength is single-slot; this variant is what
//     makes the cont_occ tiebreak part of the comparison.)
//   - kRefineOccupancyAtSameLength flipped to true (DfsSearch uses
//     this to relax the max_acceptable bound from best.length - 1
//     to best.length, allowing same-length completions to be
//     produced).
//   - ShouldEndSearch overridden to also require occupancy strictly
//     above the function target — when occupancy already exceeds
//     target, further pressure refinement won't unlock anything,
//     so we end at length floor as in the no-refine path; when
//     occupancy equals target, we keep searching to maximize
//     continuous score within the bracket.
//
// Subclassing gives us a distinct type that DfsSearch can
// instantiate alongside DfsMinimizeLengthPolicy in the same
// binary — the orchestrator picks which to use.
class DfsMinimizeLengthRefineOccupancyPolicy
    : public DfsMinimizeLengthPolicy {
 public:
  static constexpr ScoreRecipe kScoreRecipe =
      score_recipes::kMinimizeScheduleLengthThenMaximizeContinuousOccupancyScore;

  static bool ShouldEndSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);
};

// Length-min variant that, after finding a length-optimal schedule,
// continues exploring same-length completions to refine ILP score
// (defer first-uses to grow producer windows). Inherits everything
// from DfsMinimizeLengthPolicy except:
//   - kMetric: kMinimizeScheduleLengthThenMaximizeIlpScoreThenMaximizeContinuousOccupancyScore — IsBetterThan
//     tiebreaks length asc, then ILP desc, then occupancy desc.
//   - kRefineIlpAtSameLength = true — relaxes the bound to allow
//     same-length completions through, AND gates the
//     LengthHistoryTracker's ILP dim (per-open-producer
//     inst_counts and locked-in ILP score participate in
//     dominance, so the search isn't pruned prematurely on
//     paths that could refine ILP).
//   - ShouldEndSearch never auto-stops at the length floor (no
//     natural "no further refinement possible" condition for
//     ILP — unlike occupancy refine, where exceeding the
//     function target stops the search). Relies on the per-
//     region timeout to bound total search.
class DfsMinimizeLengthRefineIlpPolicy
    : public DfsMinimizeLengthPolicy {
 public:
  static constexpr ScoreRecipe kScoreRecipe =
      score_recipes::kMinimizeScheduleLengthThenMaximizeIlpScoreThenMaximizeContinuousOccupancyScore;

  static bool ShouldEndSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);
};

// Policy for DFS when the objective is to MAXIMIZE schedule length
// for a single region, SUBJECT TO not dropping the region's
// register-only occupancy below the function-wide ceiling. Useful as
// a control / worst-legal-schedule baseline for comparing against
// the length-min objective.
//
// Inverts the length axis everywhere it appears (IsBetterThan via
// kMetric, LengthHistoryTracker dominance via kLengthMaxMode), but
// leaves pressure semantics unchanged: occupancy floor is still
// enforced, pressure-history pruning would still use "lower peak
// dominates" if enabled.
//
// Differences from DfsMinimizeLengthPolicy:
//   - kMetric = kMaximizeScheduleLength (longer wins).
//   - kLengthMaxMode = true (LengthHistoryTracker dominance flips
//     end_cycle and frontier-LB directions).
//   - ShouldBoundSearch drops every length-deadline bound check
//     (length-min prunes via max-acceptable / per-node max-cycle
//     deadlines; length-max has no comparable upper bound in use)
//     and keeps only the occupancy floor and history-dominance
//     pruning. An upper bound on remaining achievable length
//     exists in principle — e.g. sum of all unscheduled op
//     latencies — but any tight version is more work than it's
//     worth here, and a loose version contracts too little during
//     search to prune meaningfully, so we skip bound pruning
//     entirely and rely on dominance + timeout.
//   - ShouldEndSearch returns false unconditionally: we don't
//     maintain a length upper bound (see above), so there's no
//     cheap "we hit the optimum" early-exit. Termination is wall-
//     clock budget only.
//   - FilterAndSortReadyList overridden: primary key is the
//     largest effective_min_schedule_cycle (= earliest_issue_cycle)
//     among ready candidates. Picking that one advances
//     current_cycle to max(current_cycle, picked.min) + 1,
//     forcing the biggest clock jump available at this step.
//     Tiebreak on topo_index ascending for determinism.
class DfsMaximizeLengthPolicy : public SearchPolicyBase {
 public:
  static constexpr ScoreRecipe kScoreRecipe =
      score_recipes::kMaximizeScheduleLength;

  // See class comment for the ranking semantics.
  static void FilterAndSortReadyList(
      const ScheduleConstructor &working,
      SmallVectorImpl<const ScheduleNode *> &out);


  // Bound the current subtree if any of:
  //   (a) the working schedule's register-only occupancy has dropped
  //       below the function occupancy target — same monotonicity
  //       argument as length-min: pressure only grows, occupancy
  //       only drops, so no completion can recover.
  //   (b) (when kUseLengthHistoryPruning is true) history dominance
  //       fires under the flipped length-axis direction — see
  //       LengthHistoryTracker.
  //
  // `pressure_history` parameter unused — uniform signature with
  // other policies.
  static bool ShouldBoundSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor,
      LengthHistoryTracker &length_history,
      PressureHistoryTracker &pressure_history);

  // Always returns false — see class comment for the rationale.
  static bool ShouldEndSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);

  // Enable length history-domination pruning, with the axis
  // direction flipped via the recipe's IsLengthMaxMode().
  static constexpr bool kUseLengthHistoryPruning = true;
};

// Policy for DFS when the objective is to maximize register-only
// occupancy for a single region.
class DfsMaximizeOccupancyPolicy : public SearchPolicyBase {
 public:
  // Use the continuous score, not the integer one. The integer
  // recipe ties any two schedules in the same occupancy bracket;
  // continuous distinguishes within-bracket pressure too, which
  // gives ShouldBoundSearch real prune signal even when we're not
  // crossing a cliff.
  static constexpr ScoreRecipe kScoreRecipe =
      score_recipes::kMaximizeContinuousRegisterOccupancyScore;

  // Override SearchPolicyBase: pressure-aware filter + sort.
  //
  //   Filter: if any "pure reader" (def_count == 0) is in the ready
  //   list, keep ONLY pure readers — they consume registers
  //   without producing new live ranges, so picking them now
  //   strictly relieves pressure (or holds steady). Defer all
  //   def-producing instructions to a later step.
  //
  //   Sort:
  //     - Primary: net_def_minus_kill ascending (most pressure-
  //       relief first; pure readers naturally have net <= 0).
  //     - NID ascending (LLVM input order — implicitly pressure-
  //       aware via LLVM's pre-RA scheduler).
  //     - topo_index ascending (deterministic final tiebreak).
  //
  // Subgraph proxies (no register effect on their own) treat their
  // members as the composite — see Effective* helpers in the .cpp.
  static void FilterAndSortReadyList(
      const ScheduleConstructor &working,
      SmallVectorImpl<const ScheduleNode *> &out);

  // Return true if no completion of the current partial schedule
  // can improve on best_schedule_constructor. Bounds:
  //   (a) Score-bound: working's GetMetricScore(kMetric) is
  //       non-increasing as nodes are scheduled (peak pressure
  //       grows monotonically). If working's score is already
  //       <= best's, no completion of working can strictly beat
  //       best.
  //   (b) History dominance (when kUsePressureHistoryPruning is
  //       true): the pressure-history tracker reports the
  //       current state can be pruned. See PressureHistoryTracker
  //       for what the tracker checks and why pruning is safe.
  //
  // SIDE EFFECT: IsDominatedElseRecord can mutate the tracker
  // (recording state for future comparison). See
  // PressureHistoryTracker.
  //
  // `length_history` parameter is unused here — present only
  // because DfsSearch invokes ShouldBoundSearch with a uniform
  // signature across policies. This policy doesn't opt into
  // length-history pruning.
  static bool ShouldBoundSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor,
      LengthHistoryTracker &length_history,
      PressureHistoryTracker &pressure_history);

  // Called on completed schedules after the IsBetterThan/update step.
  // Return true to end the entire search and have DfsSearch::Run
  // return whatever best is currently held. For maximize: end as
  // soon as best is at or above the function-wide occupancy ceiling
  // (no register-side improvement can raise effective occupancy
  // further).
  static bool ShouldEndSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);

  // Enable pressure history-domination pruning. See
  // PressureHistoryTracker.
  static constexpr bool kUsePressureHistoryPruning = true;
};

// Like DfsMaximizeOccupancyPolicy, but among schedules with equal peak
// occupancy score it prefers the one with the higher occupancy area
// under the curve (kMaximizeContinuousOccupancyScoreThenMaximizeContinuousOccupancyArea) — pressure
// kept low throughout, not just at the peak. Inner-search use only
// (decompose), enabled by occupancy.decompose_inner_area_tiebreak.
//
// Inherits the occupancy policy's ready-list filter/sort, and keeps
// pressure-history pruning on — that tracker is (peak, area)
// lexicographic, so it already accounts for the area tiebreak. The two
// overrides below are the rest of what the tiebreak requires.
class DfsMaximizeContinuousOccupancyThenAreaPolicy
    : public DfsMaximizeOccupancyPolicy {
 public:
  static constexpr ScoreRecipe kScoreRecipe =
      score_recipes::kMaximizeContinuousOccupancyScoreThenMaximizeContinuousOccupancyArea;

  // Strict < on the fixed best-bound (base uses <=): working's
  // current peak score is an upper bound on any completion's peak, so
  // a completion can still tie best's peak and then win on area.
  // Pruning the equal-peak case would discard those, so only prune
  // when strictly below best. The (area-aware) history check still
  // handles same-partition domination.
  static bool ShouldBoundSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor,
      LengthHistoryTracker & /*length_history*/,
      PressureHistoryTracker &pressure_history) {
    if (schedule_constructor.CompletionCannotImproveUpon(
            best_schedule_constructor, kScoreRecipe)) {
      return true;
    }
    return pressure_history.IsDominatedElseRecord();
  }

  // Never end early: the base stops once best hits the occupancy
  // ceiling, but we keep exploring same-peak schedules for a better
  // area. The strict-< bound and history check confine that to the
  // max-peak frontier; the inner DFS timeout caps the cost.
  static bool ShouldEndSearch(
      const ScheduleConstructor & /*schedule_constructor*/,
      const ScheduleConstructor & /*best_schedule_constructor*/) {
    return false;
  }
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHPOLICIES_H
