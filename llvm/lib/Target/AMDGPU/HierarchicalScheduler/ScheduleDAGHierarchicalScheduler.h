//===- ScheduleDAGHierarchicalScheduler.h - Hierarchical Scheduler -*- C++ -*-===//
//
// Hierarchical instruction scheduler for AMDGPU.
//
// This scheduler runs as a second pre-RA scheduling pass after the normal
// AMDGPU scheduler (GCNMaxOccupancySchedStrategy). It inherits from
// ScheduleDAGMILive (the same base class used by ScheduleDAGOptSched).
//
// Like OptSched, this scheduler uses schedule() to record regions during
// the normal per-region driver loop, and finalizeSchedule() to perform
// the actual scheduling after all regions have been collected.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_H

#include "DfsSearch.h"
#include "RegionInfo.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/STLFunctionalExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include <optional>
#include <type_traits>
#include <vector>

namespace llvm {

class SIMachineFunctionInfo;

namespace hierarchical_scheduler {

class ScheduleConstructor;
class ScheduleGraph;
class ScheduleNode;

class ScheduleDAGHierarchicalScheduler : public ScheduleDAGMILive {
  // Regions recorded during schedule() for later processing in
  // finalizeSchedule().
  SmallVector<RegionInfo, 32> regions_;

  // Per-function state, set by InitFunction().
  SIMachineFunctionInfo *mfi_ = nullptr;

public:
  ScheduleDAGHierarchicalScheduler(MachineSchedContext *C,
                                   std::unique_ptr<MachineSchedStrategy> S);

  // Called per-region by the outer driver. Records the region for later
  // processing in finalizeSchedule().
  void schedule() override;

  // Called once after all regions have been visited. Dispatches to the
  // configured scheduling algorithm (e.g., MaliciousScheduler).
  void finalizeSchedule() override;

  // Sort `regions_` ascending by the integer occupancy implied by
  // each region's original peak pressure. After the sort, `regions_[0]`
  // is the region whose original order produces the lowest occupancy
  // — the binding constraint on kernel-level occupancy.
  //
  // Uses a stable sort so that regions with the same original
  // occupancy keep their original recording order. This gives us
  // deterministic output across runs and library implementations,
  // and makes any secondary behavior (diagnostic printing,
  // downstream iteration) consistent with the order regions
  // appeared in the MachineFunction.
  void SortRegionsByOriginalRegisterOnlyOccupancyAscending();

  // Run the malicious scheduler over all recorded regions. For each region,
  // builds the DAG, computes the malicious schedule, and applies it.
  void RunMaliciousScheduler();

  // Main hierarchical scheduling path. Builds the LLVM DAG and our
  // ScheduleGraph for each region. Currently a no-op for scheduling — this
  // is where the hierarchical algorithm will be implemented.
  void RunHierarchicalScheduler();

  // Per-region graph-construction helper used by pass drivers.
  // Wraps ProcessRegion, calls buildSchedGraph, constructs a
  // ScheduleGraph from the fresh SUnits, computes its topo order,
  // and hands the graph to the callback. The graph is destroyed
  // when the callback returns; anything the callback wants to
  // persist across calls must be saved in a form stable across
  // buildSchedGraph rebuilds (e.g., MachineInstr* rather than
  // ScheduleNode* or SUnit*). The graph's edge latency divisor is
  // GetLatencyDivisorForScheduling()'s normal rule; delegates to the
  // explicit-divisor overload below.
  void WithRegionGraph(
      const RegionInfo &region,
      function_ref<void(ScheduleGraph &)> callback);

  // Overload with an explicit edge latency divisor in place of the
  // normal rule. The min-adjusted-length pass passes its tier here so
  // each tier's graphs carry that tier's adjusted lens regardless of
  // the scale_edge_latencies flag.
  void WithRegionGraph(
      const RegionInfo &region, int latency_divisor,
      function_ref<void(ScheduleGraph &)> callback);

  // The scheduling-view latency divisor passed to BuildFromSUnits: the
  // kernel's occupancy under the ScaleEdgeLatencies option, else 1. Kept
  // separate from the schedule-length analyzer's own divisors so the
  // scheduling view and the analysis view never affect one another.
  int GetLatencyDivisorForScheduling() const;

  // Stub pass: schedule every region in topo order and apply.
  // Exercises the full pipeline (WithRegionGraph → ScheduleConstructor
  // → ApplyScheduleOrder) without any real search logic. Useful for
  // verifying the plumbing.
  void RunTopoPass();

  // Outer loop of the occupancy-maximization pass. Iterates regions
  // ascending by original register-only occupancy and calls
  // ScheduleRegionForMaximumOccupancy on each, tracking a running
  // kernel ceiling with the early-exit conditions documented at the
  // call site. Cross-checks MFI->getOccupancy() against
  // ComputeNonRegisterOccupancy before starting.
  void RunMaximizeOccupancyPass();

  // Per-region worker for RunMaximizeOccupancyPass. Returns:
  //   - all_factors_occupancy: the maximum achieved all-factors
  //     occupancy for this region (arch max ∩ LDS ∩ launch bounds
  //     ∩ best register-pressure dimension). Per-region contribution
  //     to the kernel-wide occupancy min.
  //   - termination_cause: how the per-region search ended (see
  //     SearchTerminationCause). The pass aggregates these into
  //     counts of regions that timed out vs. ran to completion vs.
  //     ended because the policy was satisfied.
  struct MaxOccupancyRegionResult {
    // The honest GCNSubtarget::computeOccupancy result for the
    // region's final schedule -- highest waves/SIMD the schedule's
    // SGPR/VGPR/LDS usage can fit at. Can be BELOW
    // GetLaunchOccupancyFloor for a region that ended up in the
    // spill regime (register pressure too high to fit at the
    // launch-attribute minimum).
    int raw_all_factors_occupancy;
    // raw_all_factors_occupancy clamped up to the launch floor --
    // the occupancy the kernel actually launches at for this
    // region. Equal to raw_all_factors_occupancy when no spilling
    // is required; equal to the floor when raw is below it.
    int launch_floor_clamped_all_factors_occupancy;
    // True iff raw_all_factors_occupancy < GetLaunchOccupancyFloor
    // -- the region's schedule exceeded the launch floor's
    // register budget and is spilling.
    bool in_spill_regime;
    SearchTerminationCause termination_cause;
    std::string winner;
    std::optional<float> bfs_pct;
    // Per-backend throughput of the region's search (ms + Schedule
    // steps); both present on a DFS-fallback. See SearchResult.
    std::optional<int> bfs_ms, dfs_ms;
    std::optional<int> bfs_steps, dfs_steps;
    int orig_vgpr = 0, orig_sgpr = 0, fin_vgpr = 0, fin_sgpr = 0;
    // Accumulated VGPR spill area
    // (GCNRegisterTracker::GetVGPRSpillArea) for the region's
    // input and final schedules respectively. Parallel to
    // orig/fin_vgpr/sgpr above: a per-region snapshot of the
    // spill-area metric for telemetry.
    int64_t orig_spill_area = 0;
    int64_t fin_spill_area = 0;

    // Per-subgraph search outcome for the search_outcomes.csv "sub{n}"
    // rows. Only the per-search fields are recorded; region-level
    // pressure/improved totals live on the region/outer row, not here.
    struct SubgraphOutcome {
      int nodes;
      SearchTerminationCause termination_cause;
      std::string winner;
      std::optional<float> bfs_pct;
      std::optional<int> bfs_ms, dfs_ms;
      std::optional<int> bfs_steps, dfs_steps;
    };
    // Non-empty only for decompose regions: one entry per scheduled
    // subgraph, in formation order (sub0, sub1, ...).
    std::vector<SubgraphOutcome> subgraph_rows;
  };
  MaxOccupancyRegionResult ScheduleRegionForMaximumOccupancy(
      RegionInfo &region);

  // Outer loop of the length pass. Iterates every region and calls
  // ScheduleRegionForLengthPass. Runs after RunMaximizeOccupancyPass
  // so MFI->getOccupancy() already reflects the kernel-wide ceiling
  // — the per-region DFS's occupancy-drop bound prevents the length
  // search from degrading occupancy. Direction (minimize vs maximize
  // length) is resolved from the typed config: default is minimize;
  // length.policy = max (LengthPolicy::kMax) flips to maximize.
  void RunLengthPass();

  // Aggregate stats returned by ScheduleRegionForLengthPass. Used by
  // the driver to print a PASS RESULT line at the end of the length
  // pass. Direction-neutral on its own — the driver interprets
  // "improved" against the active direction.
  struct LengthRegionStats {
    int input_length = 0;
    int output_length = 0;
    int floor = 0;
    bool timed_out = false;
    int nodes = 0;
    int orig_vgpr = 0, orig_sgpr = 0, fin_vgpr = 0, fin_sgpr = 0;
    // Accumulated VGPR spill area (GCNRegisterTracker::GetVGPRSpillArea)
    // for the region's input and final schedules respectively. Parallel
    // to the matching fields on MaxOccupancyRegionResult.
    int64_t orig_spill_area = 0;
    int64_t fin_spill_area = 0;
    // Length pass is DFS-only; bfs_* stay blank on len rows.
    std::optional<int> dfs_ms, dfs_steps;
  };

  // Per-region worker for RunLengthPass. Resolves the length-pass
  // policy from misched.txt and dispatches accordingly — to one of
  // the length-MIN templated workers or to the dedicated length-MAX
  // worker. DfsSearch seeds best with the input schedule in every
  // case, so the output is guaranteed no worse than the current MF
  // order under the active direction. Returns per-region stats for
  // the outer loop to aggregate.
  LengthRegionStats ScheduleRegionForLengthPass(RegionInfo &region);

  // The min-adjusted-length pass (misched.txt flag `min_adjusted_length`;
  // replaces the length pass). Chooses the kernel's schedule and
  // occupancy jointly: for every reachable occupancy tier o — the
  // post-occupancy-pass ceiling down to the launch floor — schedule
  // every region for minimum RAW length under tier o's register budget
  // (divisor-1 graphs; the tier does not enter the search objective),
  // buffer without applying, and score each candidate at its ACTUAL
  // occupancy with the steady-state time-per-wave model, summed
  // across regions before the max (interleaved waves execute
  // different regions at the same time, so coverage crosses regions):
  //
  //   score = max(Σ_r issue_slots_r, ceil(Σ_r raw_length_r / o_act))
  //
  // Smallest score wins, ties to the higher actual occupancy; the
  // winner's schedules are committed and the occupancy target pinned
  // at its actual occupancy. The derivation of the score (assembly-
  // line/Little's-law model, why raw length replaced the per-edge
  // divided lens, worked examples, assumptions) is recorded in
  // llvm/docs/AMDGPUMinAdjustedLengthScheduler.md — read that before
  // changing the objective. Must run after RunMaximizeOccupancyPass:
  // its ceiling is the top tier and its committed schedules are a
  // search seed feasible at every tier at or below it (enforced at
  // config build time).
  void RunMinimizeAdjustedLengthPass();

  // One region's buffered schedule within a tier candidate. Produced
  // by SearchRegionAtOccupancyTier.
  struct MinAdjustedLengthRegionSchedule {
    // The best order the tier's search found, held as MachineInstr*
    // (stable across the graph rebuilds between search and commit).
    std::vector<MachineInstr *> order;
    // The order's raw schedule length (divisor-1 lens): the wave's
    // uncontended lifetime for this region, issue cycles plus exposed
    // latency stalls. Occupancy-independent, so it is measured once at
    // search time and the tier score is pure arithmetic on it.
    int raw_length = 0;
    // The region's issue-slot demand: its real-instruction count (one
    // slot per instruction — the flat single-port model; per-category
    // refinement is deferred). Schedule-order-independent; recorded
    // here so scoring needs no graph access.
    int issue_slots = 0;
    // The order's launch-floor-clamped all-factors occupancy — this
    // region's contribution to the candidate's kernel-wide min.
    int achieved_occupancy = 0;
    // The region's search hit its wall-clock budget.
    bool timed_out = false;
  };

  // One occupancy tier's candidate kernel schedule, produced by
  // ScheduleKernelForOccupancyTier and scored/committed by
  // RunMinimizeAdjustedLengthPass. region_schedules is parallel to
  // regions_.
  struct MinAdjustedLengthCandidate {
    // The tier the searches ran under (sets the register budget).
    int searched_occupancy_tier = 0;
    // The kernel-wide occupancy the buffered schedules actually
    // achieve: min over regions of each schedule's launch-floor-clamped
    // all-factors occupancy. Never below searched_occupancy_tier (the
    // policy gate forbids dropping under the budget tier) but can be
    // ABOVE it — the budget only bounds pressure from above, so a
    // min-length schedule may land in a higher bracket than it was
    // searched under.
    int actual_occupancy = 0;
    // The candidate's score — the steady-state time per wave, with
    // the sums taken across regions BEFORE the max (interleaved waves
    // execute different regions at the same time, so one region's
    // stalls are covered by other regions' issue work):
    //   max(Σ issue_slots, ceil(Σ raw_length / actual_occupancy))
    // The first arm is the wave's total issue-port demand (occupancy
    // cannot reduce it); the second is the wave's total uncontended
    // lifetime divided by the resident-wave count (Little's law on
    // the residency slots). Occupancy only pays while the second arm
    // binds.
    int64_t score_sum = 0;
    bool timed_out = false;
    std::vector<MinAdjustedLengthRegionSchedule> region_schedules;
  };

  // Per-tier worker for RunMinimizeAdjustedLengthPass. Sets MFI's
  // occupancy target to `tier` (SetOccupancyTarget, position-
  // independent; the DFS policies read their register budget from the
  // target live), then runs SearchRegionAtOccupancyTier on every
  // region, buffering each best order. Computes the candidate's
  // actual occupancy and its score — pure arithmetic over the
  // recorded raw lengths and issue-slot counts (raw length is
  // occupancy-independent, so nothing is re-measured; see score_sum).
  MinAdjustedLengthCandidate ScheduleKernelForOccupancyTier(int tier);

  // Per-region worker for ScheduleKernelForOccupancyTier: min-RAW-
  // length search the region (divisor-1 graph — the tier does not
  // enter the search objective; the score's lifetime sum decomposes
  // over regions, so minimizing each region's raw length is optimal
  // at every tier), via the same two-phase length-min worker the
  // length pass uses (plain min policy), seeded with the current MF
  // order. The best schedule is returned in buffered form, NOT
  // applied — every tier must search from the same input. The tier
  // reaches the search only through the register budget: the caller
  // must already have set MFI's occupancy target to it.
  MinAdjustedLengthRegionSchedule SearchRegionAtOccupancyTier(
      RegionInfo &region);

  // Measure a buffered region order's schedule length under
  // `latency_divisor`: rebuild the region graph with that divisor and
  // replay the order through ScheduleGraph::MakeConstructorForInstrOrder.
  // The MF order is untouched during the sweep, so the rebuilt graph
  // has the same nodes and edges the tier's search saw — only the edge
  // weights differ.
  int MeasureBufferedOrderLength(const RegionInfo &region,
                                 const std::vector<MachineInstr *> &order,
                                 int latency_divisor);

  // Initialize per-function state. Called at the start of
  // RunHierarchicalScheduler / RunMaliciousScheduler. Stores mfi_
  // and resets occupancy to the pre-GCN-scheduler value.
  void InitFunction();

private:
  // Run every shakedown / validation test. Called by
  // RunHierarchicalScheduler during development. All the individual
  // shakedown helpers (per-graph register trackers, schedule-
  // constructor round-trip, critical-path checks, etc.) live in
  // the anonymous namespace of Shakedowns.cpp and take whatever
  // state they need as explicit parameters; this method
  // orchestrates them with class state (MF, *LIS, EntrySU,
  // ExitSU, regions_).
  void RunAllShakedowns();

  // Shakedown for the min-adjusted-length pass's building blocks
  // (sentinel accessors, GetInstrOrder, MakeConstructorForInstrOrder):
  // replays the first region's input order over graphs built at
  // divisor 1 and at an adjusted divisor, checking each replay against
  // its graph's input constructor and the adjusted length against the
  // raw one. A member (defined in Shakedowns.cpp, like
  // RunAllShakedowns) because it builds region graphs via
  // WithRegionGraph.
  void RunInstrOrderReplayShakedowns();

protected:
  // Apply a computed schedule order to the given region. Physically moves
  // MachineInstrs to match the order given by |scheduled_units|.
  // Every SUnit* must be non-null with a valid MachineInstr —
  // guaranteed when drawing from the SUnits vector that buildSchedGraph
  // populates (which only contains real instruction SUnits, not
  // the boundary EntrySU/ExitSU).
  // Must be called within a BeginRegion/EndRegion pair.
  void ApplyScheduleOrder(
      const RegionInfo &region,
      const std::vector<SUnit *> &scheduled_units);

  // Overload: extract the real-instruction order from a completed
  // ScheduleConstructor (GetInstrOrder) and delegate to the
  // MachineInstr* overload below.
  void ApplyScheduleOrder(
      const RegionInfo &region,
      const ScheduleConstructor &sc);

  // Overload: apply an order held as MachineInstr pointers — the form
  // the min-adjusted-length pass buffers per tier. A ScheduleConstructor
  // cannot be buffered across regions/tiers: it is a view over a
  // ScheduleGraph that WithRegionGraph destroys, and the graph's SUnit
  // backpointers go stale on the next buildSchedGraph anyway.
  // MachineInstr* is owned by the MachineFunction and survives every
  // rebuild (see WithRegionGraph's persistence contract). This is the
  // primitive that does the actual instruction moves; the SUnit*
  // overload reduces to it.
  void ApplyScheduleOrder(
      const RegionInfo &region,
      const std::vector<MachineInstr *> &scheduled_instrs);

  // Set up ScheduleDAGMILive state for the given region so that
  // moveInstruction() and other inherited methods work correctly.
  // Calls startBlock and enterRegion.
  void BeginRegion(const RegionInfo &region);

  // Clean up after scheduling a region.
  // Calls exitRegion and finishBlock.
  void EndRegion(const RegionInfo &region);

  // Process a region by calling BeginRegion, the provided action, then
  // EndRegion. Returns whatever the action returns.
  template <typename F>
  auto ProcessRegion(const RegionInfo &region, F action) {
    BeginRegion(region);
    if constexpr (std::is_void_v<decltype(action())>) {
      action();
      EndRegion(region);
    } else {
      auto result = action();
      EndRegion(region);
      return result;
    }
  }
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_H
