//===- ScheduleDAGHierarchicalScheduler.cpp - Hierarchical Scheduler ------===//
//
// Implementation of the hierarchical instruction scheduler for AMDGPU.
//
// Currently a no-op: schedule() records regions and finalizeSchedule()
// does nothing. The actual scheduling logic will be added later.
//
//===----------------------------------------------------------------------===//

#include "ScheduleDAGHierarchicalScheduler.h"
#include "BranchAndBoundSearch.h"
#include "DfsSearch.h"
#include "GCNRegisterTracker.h"
#include "MaliciousScheduler.h"
#include "ScheduleConstructor.h"
#include "SearchPolicies.h"
#include "GCNSubtarget.h"
#include "SIMachineFunctionInfo.h"
#include "ScheduleGraph.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "machine-scheduler"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

// Local copy of the static NextIfDebug helper from MachineScheduler.cpp.
// Advances an iterator past any debug/pseudo instructions. Not publicly
// accessible from MachineScheduler.cpp, so we reproduce it here (as
// OptSched also does).
static MachineBasicBlock::iterator
NextIfDebug(MachineBasicBlock::iterator I,
            MachineBasicBlock::const_iterator End) {
  for (; I != End; ++I) {
    if (!I->isDebugOrPseudoInstr()) {
      break;
    }
  }
  return I;
}

ScheduleDAGHierarchicalScheduler::ScheduleDAGHierarchicalScheduler(
    MachineSchedContext *C, std::unique_ptr<MachineSchedStrategy> S)
    : ScheduleDAGMILive(C, std::move(S)) {}

// Called per-region by the outer driver (scheduleRegions). We record the region
// boundaries for later use in finalizeSchedule(), rather than scheduling now.
// This is the same deferred-scheduling pattern used by ScheduleDAGOptSched
// when two-pass scheduling is enabled.
void ScheduleDAGHierarchicalScheduler::schedule() {
  // Use the pressure-aware constructor so each region records the
  // occupancy of its incoming instruction order (what the prior
  // scheduler gave us). Kernel-level occupancy is the min over
  // all regions, so the region with the lowest original occupancy
  // is the binding constraint on kernel occupancy. finalizeSchedule
  // sorts regions by this value ascending and processes the
  // binding region first; if we can't raise its occupancy, no
  // amount of work on the other regions can raise the kernel
  // ceiling, and the occupancy pass can exit immediately.
  RegionInfo region(RegionBegin, RegionEnd, *LIS);
  regions_.push_back(region);

  // TODO: Remove this temporary print once we've confirmed the pass runs.
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());
  const GCNRegPressure &rp = region.GetOriginalPeakPressure();
  llvm::outs() << "HierarchicalScheduler: recorded region "
               << regions_.size() << " (" << region.GetNumInstrs()
               << " instrs, orig_reg_occ="
               << region.GetOriginalRegisterOnlyOccupancy()
               << " vgpr=" << rp.getVGPRNum(st.hasGFX90AInsts())
               << " sgpr=" << rp.getSGPRNum() << ")\n";
}

// Sort `regions_` ascending by the integer occupancy implied by each
// region's original peak pressure. See the header for rationale and the
// stable-sort justification.
void ScheduleDAGHierarchicalScheduler::SortRegionsByOriginalRegisterOnlyOccupancyAscending() {
  std::stable_sort(regions_.begin(), regions_.end(),
                   [](const RegionInfo &a, const RegionInfo &b) {
                     return a.GetOriginalRegisterOnlyOccupancy() <
                            b.GetOriginalRegisterOnlyOccupancy();
                   });
}

// Called once after all regions in all blocks have been visited.
// Sorts recorded regions, then dispatches to the configured
// scheduling algorithm.
void ScheduleDAGHierarchicalScheduler::finalizeSchedule() {
  llvm::outs() << "HierarchicalScheduler: finalizeSchedule called with "
               << regions_.size() << " regions\n";

  SortRegionsByOriginalRegisterOnlyOccupancyAscending();

  // TODO: Remove this temporary print once we've confirmed the sort.
  llvm::outs() << "HierarchicalScheduler: region order after sort:\n";
  for (size_t i = 0; i < regions_.size(); ++i) {
    const RegionInfo &r = regions_[i];
    llvm::outs() << "  [" << i << "] " << r.GetNumInstrs()
                 << " instrs, orig_reg_occ="
                 << r.GetOriginalRegisterOnlyOccupancy() << "\n";
  }

  const MachineInstrSchedulerConfig &config =
      MachineInstrSchedulerConfig::GetConfig();

  if (config.HasSchedulingOption(
          MachineInstrSchedulerConfig::SchedulerOption::
              MaliciousScheduler)) {
    RunMaliciousScheduler();
  } else {
    RunHierarchicalScheduler();
  }

  ScheduleDAGMILive::finalizeSchedule();
}

// Run the malicious scheduler over all recorded regions. For each region,
// uses ProcessRegion to handle BeginRegion/EndRegion, builds the DAG
// (using alias analysis for memory dependency precision), computes the
// malicious schedule order, and applies it.
void ScheduleDAGHierarchicalScheduler::RunMaliciousScheduler() {
  // TODO: Remove this temporary print.
  llvm::outs() << "RunMaliciousScheduler: scheduling " << regions_.size()
               << " regions\n";

  for (auto &region : regions_) {
    ProcessRegion(region, [&]() {
      buildSchedGraph(AA);
      auto order = ComputeMaliciousSchedule(SUnits);
      // TODO: Remove this temporary print.
      llvm::outs() << "  Malicious scheduled region with " << order.size()
                   << " instructions\n";
      ApplyScheduleOrder(region, order);
    });
  }
}

// Per-region graph-construction helper. See header for rationale.
void ScheduleDAGHierarchicalScheduler::WithRegionGraph(
    const RegionInfo &region,
    function_ref<void(ScheduleGraph &)> callback) {
  ProcessRegion(region, [&]() {
    buildSchedGraph(AA);

    const GCNSubtarget &st =
        static_cast<const GCNSubtarget &>(MF.getSubtarget());
    auto graph = ScheduleGraph::BuildFromSUnits(
        SUnits, st, MF, *LIS, MF.getRegInfo(), region);

    callback(*graph);
  });
}

// Initialize per-function state. Stores mfi_ and resets occupancy
// to the pre-GCN-scheduler value so we can aim for the best possible
// occupancy with our own schedule (same approach as OptSched,
// GCNOptSched.cpp:58).
void ScheduleDAGHierarchicalScheduler::InitFunction() {
  mfi_ = const_cast<SIMachineFunctionInfo *>(
      MF.getInfo<SIMachineFunctionInfo>());
  mfi_->resetInitialOccupancy(MF);
}

// Apply a schedule from a completed ScheduleConstructor. Extracts
// the order, skips synthetic entry/exit nodes (null SUnit), and
// delegates to the SUnit* overload. Group nodes are not expected
// and trigger a fatal error — the flat-only ScheduleConstructor
// should never produce them.
// Materializes the SUnit* order from the ScheduleConstructor and
// delegates to the vector<SUnit*> overload. This allocates an
// intermediate vector, which could be avoided using LLVM's
// make_filter_range + map_range to lazily iterate
// sc.GetScheduleOrder() directly. Not worth it: we apply a
// schedule at most once per region per pass, so the cost is
// negligible.
void ScheduleDAGHierarchicalScheduler::ApplyScheduleOrder(
    const RegionInfo &region,
    const ScheduleConstructor &sc) {
  std::vector<SUnit *> sunit_order;
  for (const ScheduleNode *node : sc.GetScheduleOrder()) {
    // Subgraph proxies appear in schedule_order_ for scope-stack
    // bookkeeping (start = push, end = pop) but don't correspond to
    // any MachineInstr — filter them out here. Real instruction
    // ordering lives entirely on the scheduling-unit nodes.
    if (!node->IsSchedulingUnit()) {
      continue;
    }
    SUnit *su = node->GetSUnit();
    if (su) {
      sunit_order.push_back(su);
    }
  }
  ApplyScheduleOrder(region, sunit_order);
}

// Stub pass: schedule every region in topo order and apply. Exercises
// the full pipeline without any real search logic.
void ScheduleDAGHierarchicalScheduler::RunTopoPass() {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  for (auto &region : regions_) {
    WithRegionGraph(region, [&](ScheduleGraph &graph) {
      ScheduleConstructor sc(graph, st, MF, *LIS);

      for (ScheduleNode *node : graph.GetTopoOrder()) {
        sc.Schedule(node);
      }

      ApplyScheduleOrder(region, sc);
    });
  }
}

// Maximize-occupancy outer loop. See header for detail.
void ScheduleDAGHierarchicalScheduler::RunMaximizeOccupancyPass() {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  // Grab initial occupancy, ignoring registers
  // This should match the configured limit for the function,
  // which we reset previously
  int configured_limit = static_cast<int>(mfi_->getOccupancy());
  int non_register =
      GCNRegisterTracker::ComputeNonRegisterOccupancy(st, MF);
  if (configured_limit != non_register) {
    report_fatal_error(
        "RunMaximizeOccupancyPass: MFI->getOccupancy() disagrees with "
        "ComputeNonRegisterOccupancy — something lowered the MFI value "
        "without a matching reset");
  }

  int kernel_occupancy_so_far = configured_limit;

  // TODO: Remove this temporary print once the pass is wired up.
  llvm::outs() << "RunMaximizeOccupancyPass: starting with "
               << "configured_limit=" << configured_limit << "\n";

  for (size_t i = 0; i < regions_.size(); ++i) {
    RegionInfo &region = regions_[i];
    int original_register_only_occupancy =
        region.GetOriginalRegisterOnlyOccupancy();

    // Stop when this region's original register-only occupancy is
    // already at or above the running kernel ceiling. Ascending
    // iteration guarantees every later region's original is >= this one's,
    // so their achievable occupancies are also >= kernel_occupancy_so_far,
    // and no further work can raise the kernel above the value already
    // pinned by an earlier region.
    //
    // On the first iteration, kernel_occupancy_so_far ==
    // configured_limit, so this also catches the case where region 0's
    // occupancy is already maxed out by the ceiling set by other factors 
    // (e.g. arch max / LDS / launch bounds).
    if (original_register_only_occupancy >= kernel_occupancy_so_far) {
      int num_skipped = static_cast<int>(regions_.size() - i);
      int total = static_cast<int>(regions_.size());
      int percent_skipped = (num_skipped * 100) / total;
      llvm::outs() << "  [" << i << "] stop: orig_reg_only="
                   << original_register_only_occupancy
                   << " >= kernel_so_far=" << kernel_occupancy_so_far
                   << " (skipping " << num_skipped << "/" << total
                   << " regions, " << percent_skipped << "%)\n";
      break;
    }

    // Determine highest occupancy achievable for region
    int best_region_occupancy = ScheduleRegionForMaximumOccupancy(region);

    // Update kernel_occupancy_so_far
    int kernel_occupancy_after_region =
        std::min(kernel_occupancy_so_far, best_region_occupancy);
    llvm::outs() << "  [" << i << "] orig_reg_only="
                 << original_register_only_occupancy
                 << " -> best_region_occupancy=" << best_region_occupancy
                 << "  kernel_occupancy_so_far: " << kernel_occupancy_so_far << " -> "
                 << kernel_occupancy_after_region << "\n";
    kernel_occupancy_so_far = kernel_occupancy_after_region;

    // Tighten MFI's occupancy limit immediately so the next region's
    // search sees the real running kernel ceiling (via
    // ScheduleConstructor::RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget and
    // any other code that consults MFI->getOccupancy()).
    // limitOccupancy only lowers; kernel_occupancy_so_far is
    // monotonically non-increasing, so this is always a no-op or
    // tightening.
    mfi_->limitOccupancy(static_cast<unsigned>(kernel_occupancy_so_far));
  }

  llvm::outs() << "RunMaximizeOccupancyPass: final kernel_occupancy="
               << kernel_occupancy_so_far
               << " (MFI->Occupancy now " << mfi_->getOccupancy() << ")\n";
}

// Pre-search per-region telemetry: shape of the region the DFS is
// about to schedule. Pressure / length / formation are all printed
// so that the same line-set is useful regardless of which driver is
// running. Subgraph formation already ran inside DfsSearch's ctor by
// the time this fires.
static void PrintPreScheduleInfo(const ScheduleGraph &graph,
                                 const ScheduleConstructor &input,
                                 const GCNSubtarget &st) {
  const GCNRegPressure &input_peak =
      input.GetPressureTracker().GetPeakPressure();
  llvm::outs()
      << "    [Pre]  nodes=" << graph.NumSchedulingUnits()
      << " cp=" << graph.GetCriticalPathLength() << "\n"
      << "    [Pre]  pressure: input_peak: vgpr="
      << input_peak.getVGPRNum(st.hasGFX90AInsts())
      << " sgpr=" << input_peak.getSGPRNum()
      << " | orig_reg_only_occ="
      << input.GetPressureTracker().GetRegisterOnlyOccupancy()
      << " | orig_all_factors_occ="
      << input.GetPressureTracker().GetAllFactorsRegionOnlyOccupancy()
      << "\n"
      << "    [Pre]  length:   input_length="
      << input.GetLengthTracker().GetCurrentCycle()
      << " | length_floor=" << graph.GetGraphLengthFloor() << "\n";
  graph.PrintSubgraphInfos(llvm::outs());
}

// Post-search per-region telemetry: shape of the schedule DFS
// produced. Both pressure and length results are emitted (with a
// `pressure:` / `length:` / `common:` demarcation) regardless of
// which driver invoked the search — the metrics are always
// available on the resulting ScheduleConstructor / DfsSearch
// trackers, and showing both makes cross-pass comparisons easy.
template <typename PolicyT>
static void PrintPostScheduleInfo(const ScheduleGraph &graph,
                                  const ScheduleConstructor &dfs_best,
                                  const DfsSearch<PolicyT> &search,
                                  const GCNSubtarget &st,
                                  bool order_changed) {
  const GCNRegPressure &dfs_peak =
      dfs_best.GetPressureTracker().GetPeakPressure();
  llvm::outs()
      << "    [Post] pressure: dfs_peak: vgpr="
      << dfs_peak.getVGPRNum(st.hasGFX90AInsts())
      << " sgpr=" << dfs_peak.getSGPRNum()
      << " | dfs_reg_only_occ="
      << dfs_best.GetPressureTracker().GetRegisterOnlyOccupancy()
      << " | dfs_all_factors_occ="
      << dfs_best.GetPressureTracker().GetAllFactorsRegionOnlyOccupancy()
      << "\n"
      << "    [Post] length:   dfs_length="
      << dfs_best.GetLengthTracker().GetCurrentCycle()
      << " | length_floor=" << graph.GetGraphLengthFloor() << "\n"
      << "    [Post] common:   schedule_calls=" << search.ScheduleCallCount().lifetime
      << " | length_history_prunes="
      << search.GetLengthHistoryTracker().PruneCount().lifetime
      << " | pressure_history_prunes="
      << search.GetPressureHistoryTracker().PruneCount().lifetime
      << " | order_changed=" << (order_changed ? "yes" : "no")
      << " | region_timed_out="
      << (search.RegionTimedOut() ? "yes" : "no")
      << " | length_history_cap_hit="
      << (search.GetLengthHistoryTracker().MemoryCapHit().lifetime ? "yes" : "no")
      << " | pressure_history_cap_hit="
      << (search.GetPressureHistoryTracker().MemoryCapHit().lifetime ? "yes" : "no")
      << "\n";

  // [Post] rates: per-region throughput so cross-scheduler
  // comparisons aren't sensitive to total budget. When elapsed
  // rounds to zero (trivially-small regions that complete in
  // <1ms), the rate is undefined — emit a "-" placeholder so
  // the elapsed measurement is still visible.
  int64_t region_elapsed_ms = search.GetRegionElapsedMs();
  int64_t schedule_calls = search.ScheduleCallCount().lifetime;
  llvm::outs() << "    [Post] rates:    ";
  if (region_elapsed_ms > 0) {
    double calls_per_sec = schedule_calls / (region_elapsed_ms / 1000.0);
    llvm::outs() << "schedule_calls/s=" << calls_per_sec;
  } else {
    llvm::outs() << "schedule_calls/s=-";
  }
  llvm::outs() << " | region_elapsed_ms=" << region_elapsed_ms << "\n";
}

// Runs DFS with DfsMaximizeOccupancyPolicy on the region's graph and
// applies whatever schedule it returns. DfsSearch seeds best with the
// graph's input ScheduleConstructor (the region's current MF-order
// schedule), so the returned schedule is guaranteed to be at least
// as good as the input — no non-regression check needed here. If
// DFS found nothing better, ApplyScheduleOrder is a no-op move-wise
// because every MI is already at its CurrentTop position.
int ScheduleDAGHierarchicalScheduler::ScheduleRegionForMaximumOccupancy(
    RegionInfo &region) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  int achieved_all_factors_occupancy = 0;
  WithRegionGraph(region, [&](ScheduleGraph &graph) {
    const ScheduleConstructor &input_schedule_constructor =
        graph.GetInputScheduleConstructor();

    DfsSearch<DfsMaximizeOccupancyPolicy> search(graph, st, MF, *LIS);
    PrintPreScheduleInfo(graph, input_schedule_constructor, st);

    ScheduleConstructor dfs_best_schedule_constructor = search.Run();

    bool changed =
        input_schedule_constructor.GetScheduleOrder() !=
            dfs_best_schedule_constructor.GetScheduleOrder();

    PrintPostScheduleInfo(graph, dfs_best_schedule_constructor, search, st,
                          changed);

    ApplyScheduleOrder(region, dfs_best_schedule_constructor);

    achieved_all_factors_occupancy =
        dfs_best_schedule_constructor.GetPressureTracker()
            .GetAllFactorsRegionOnlyOccupancy();
  });
  return achieved_all_factors_occupancy;
}

// Outer loop of the length-minimization pass. See header.
void ScheduleDAGHierarchicalScheduler::RunMinimizeLengthPass() {
  // TODO: Remove this temporary print once the pass is wired up.
  llvm::outs() << "RunMinimizeLengthPass: starting with "
               << regions_.size() << " regions\n";

  for (size_t i = 0; i < regions_.size(); ++i) {
    llvm::outs() << "  [" << i << "] ";
    ScheduleRegionForMinimumLength(regions_[i]);
  }

  llvm::outs() << "RunMinimizeLengthPass: done\n";
}

// Per-iteration log entry, buffered during the outer loop so we
// can choose to emit per-iteration lines only when the loop ran
// more than one iteration.
struct LengthPassIterationLog {
  int target;
  int result_length;
  int64_t elapsed_ms;
  int64_t schedule_calls;
  int64_t length_history_prunes;
  int64_t pressure_history_prunes;
  bool length_history_cap_hit;
  bool pressure_history_cap_hit;
  bool region_timed_out_observed;
};

// Snapshot the search's current-run stats plus per-iteration
// context into a log entry. Reads .current_run on every dual
// counter / flag so the captured values reflect just the
// iteration that just finished.
template <typename PolicyT>
static LengthPassIterationLog CaptureIterationLog(
    int target, int result_length, const DfsSearch<PolicyT> &search) {
  LengthPassIterationLog log;
  log.target = target;
  log.result_length = result_length;
  log.elapsed_ms = search.GetCurrentRunElapsedMs();
  log.schedule_calls = search.ScheduleCallCount().current_run;
  log.length_history_prunes =
      search.GetLengthHistoryTracker().PruneCount().current_run;
  log.pressure_history_prunes =
      search.GetPressureHistoryTracker().PruneCount().current_run;
  log.length_history_cap_hit =
      search.GetLengthHistoryTracker().MemoryCapHit().current_run;
  log.pressure_history_cap_hit =
      search.GetPressureHistoryTracker().MemoryCapHit().current_run;
  log.region_timed_out_observed = search.RegionTimedOut();
  return log;
}

// Print buffered per-iteration log entries when there were more
// than one. Single-iteration runs get only the [Post] aggregate
// (the per-iteration values would duplicate what [Post] shows).
static void MaybePrintIterationLogs(
    ArrayRef<LengthPassIterationLog> logs) {
  if (logs.size() <= 1) {
    return;
  }
  for (size_t i = 0; i < logs.size(); ++i) {
    const auto &e = logs[i];
    double calls_per_sec =
        (e.elapsed_ms > 0)
            ? (e.schedule_calls / (e.elapsed_ms / 1000.0))
            : 0.0;
    llvm::outs()
        << "    [Iter " << i << "]   target=" << e.target
        << " result_length=" << e.result_length
        << " elapsed_ms=" << e.elapsed_ms
        << " schedule_calls=" << e.schedule_calls
        << " schedule_calls/s=" << calls_per_sec
        << " length_prunes=" << e.length_history_prunes
        << " pressure_prunes=" << e.pressure_history_prunes
        << " length_cap_hit=" << (e.length_history_cap_hit ? "yes" : "no")
        << " pressure_cap_hit="
        << (e.pressure_history_cap_hit ? "yes" : "no")
        << " region_timed_out="
        << (e.region_timed_out_observed ? "yes" : "no")
        << "\n";
  }
}

// Print the [Post] outer line summarizing how the outer loop
// terminated.
static void PrintOuterLoopSummary(int iterations_run,
                                  StringRef terminated_via,
                                  int final_target) {
  llvm::outs()
      << "    [Post] outer:   iterations_run=" << iterations_run
      << " | terminated_via=" << terminated_via;
  if (terminated_via == "feasible") {
    llvm::outs() << " | final_target=" << final_target;
  }
  llvm::outs() << "\n";
}

// Compile-time gate for the target-feasibility outer loop in the
// length pass. When false (default), ScheduleRegionForMinimumLength
// makes a single search.Run() with the policy's INT_MAX target —
// the plain-min-search path. When true, the search is driven by
// an outer loop walking target from the static graph floor up to
// input_length-1, calling search.Run() at each step.
//
// The iteration mode is off until the per-instruction earliest /
// latest window machinery is wired into the inner search. Without
// that, the inner search at a tight target burns the region
// budget without detecting infeasibility, regressing hard regions
// (e.g., the 641-instr stencil region: plain min-search reaches
// 666, target=floor=641 hits the timeout and exits with the input
// length 700). Flipping this on requires the deadline-based prune
// inside the inner search and a budget-exhausted fallback to
// plain search.
constexpr bool kUseTargetFeasibilityIteration = false;

// Per-region worker. Runs DFS with DfsMinimizeLengthPolicy.
//
// In the default plain-search mode, calls search.Run() once and
// applies the resulting schedule. Output is no worse than the
// region's current MF order (DfsSearch seeds best with the input).
//
// In target-feasibility iteration mode (gated by
// kUseTargetFeasibilityIteration), drives an outer for-loop that
// walks a target length from floor up to input_length-1, calling
// search.Run() at each step. The first iteration to find a
// complete schedule with length <= target produces the optimum
// (targets are tested in ascending order). The loop exits on
// feasibility, region timeout, or naturally at the end of the
// range.
void ScheduleDAGHierarchicalScheduler::ScheduleRegionForMinimumLength(
    RegionInfo &region) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  WithRegionGraph(region, [&](ScheduleGraph &graph) {
    const ScheduleConstructor &input_schedule_constructor =
        graph.GetInputScheduleConstructor();
    const int floor = graph.GetGraphLengthFloor();
    const int input_length =
        input_schedule_constructor.GetLengthTracker().GetCurrentCycle();

    DfsSearch<DfsMinimizeLengthPolicy> search(graph, st, MF, *LIS);
    PrintPreScheduleInfo(graph, input_schedule_constructor, st);

    ScheduleConstructor best_schedule_constructor =
        input_schedule_constructor;

    if constexpr (kUseTargetFeasibilityIteration) {
      // Default outcome before any iteration runs:
      //   - "input_optimal" when the loop range is empty
      //     (input_length already at the floor, search would have
      //     no room to improve). The loop body below won't execute
      //     in this case, leaving this default in place.
      //   - "infeasible" otherwise — overridden when the loop
      //     breaks on feasibility or timeout, and stays as
      //     "infeasible" iff the loop walks the full target range
      //     without success and without timing out.
      StringRef terminated_via =
          (input_length <= floor) ? "input_optimal" : "infeasible";
      int final_target = -1;
      int iterations_run = 0;
      SmallVector<LengthPassIterationLog, 8> iter_logs;

      for (int target = floor; target < input_length; ++target) {
        search.ResetForReuse(target);
        ScheduleConstructor result = search.Run();
        ++iterations_run;

        const int result_length =
            result.GetLengthTracker().GetCurrentCycle();
        iter_logs.push_back(
            CaptureIterationLog(target, result_length, search));

        if (result_length <= target) {
          best_schedule_constructor = result;
          terminated_via = "feasible";
          final_target = target;
          break;
        }
        if (search.RegionTimedOut()) {
          terminated_via = "timeout";
          break;
        }
      }

      bool changed =
          input_schedule_constructor.GetScheduleOrder() !=
          best_schedule_constructor.GetScheduleOrder();

      MaybePrintIterationLogs(iter_logs);
      PrintPostScheduleInfo(graph, best_schedule_constructor, search, st,
                            changed);
      PrintOuterLoopSummary(iterations_run, terminated_via, final_target);

      ApplyScheduleOrder(region, best_schedule_constructor);
    } else {
      // Plain min-search: single Run() with the policy's INT_MAX
      // target. floor and input_length are unused on this branch
      // (they only feed the loop body above).
      (void)floor;
      (void)input_length;

      best_schedule_constructor = search.Run();

      bool changed =
          input_schedule_constructor.GetScheduleOrder() !=
          best_schedule_constructor.GetScheduleOrder();

      PrintPostScheduleInfo(graph, best_schedule_constructor, search, st,
                            changed);

      ApplyScheduleOrder(region, best_schedule_constructor);
    }
  });
}

// Main hierarchical scheduling path.
void ScheduleDAGHierarchicalScheduler::RunHierarchicalScheduler() {
  InitFunction();

  llvm::outs() << "RunHierarchicalScheduler: processing " << regions_.size()
               << " regions, target occupancy " << mfi_->getOccupancy()
               << "\n";

  RunMaximizeOccupancyPass();
  RunMinimizeLengthPass();

  RunAllShakedowns();
}

// Set up ScheduleDAGMILive state for the given region. Calls startBlock and
// enterRegion to ensure BB, RegionBegin, RegionEnd, etc. are properly
// initialized. Also sets CurrentTop/CurrentBottom, which are normally set by
// initQueues() inside schedule() but must be set manually here since we are
// replaying regions outside of the normal schedule() flow.
//
// We always call startBlock even if the block hasn't changed, since it just
// sets BB and notifies the (unused) strategy — no harm in calling it
// redundantly.
void ScheduleDAGHierarchicalScheduler::BeginRegion(const RegionInfo &region) {
  startBlock(region.GetBlock());
  enterRegion(region.GetBlock(), region.Begin(), region.End(),
              region.GetNumInstrs());

  // Skip past leading debug instructions to get to the first real
  // instruction. Debug instructions don't have SUnits and are repositioned
  // separately by placeDebugValues().
  CurrentTop = NextIfDebug(RegionBegin, RegionEnd);
  CurrentBottom = RegionEnd;
}

// Clean up after scheduling a region.
void ScheduleDAGHierarchicalScheduler::EndRegion(const RegionInfo &region) {
  exitRegion();
  finishBlock();
}

// Apply a computed schedule order to the current region. Walks the list of
// SUnits in the desired order and physically moves each MachineInstr into
// position using moveInstruction().
//
// Must be called within a BeginRegion/EndRegion pair (which sets up
// CurrentTop/CurrentBottom).
//
// TODO: Consider moving the CurrentTop/CurrentBottom initialization from
// BeginRegion into this function, since they are only needed here.
//
// TODO: Consider wrapping this in a higher-level function that uses
// ProcessRegion, so callers don't have to manage BeginRegion/EndRegion
// themselves when applying a schedule.
void ScheduleDAGHierarchicalScheduler::ApplyScheduleOrder(
    const RegionInfo &region,
    const std::vector<SUnit *> &scheduled_units) {
  for (SUnit *su : scheduled_units) {
    MachineInstr *mi = su->getInstr();
    if (&*CurrentTop == mi) {
      // Already in the right position — just advance the cursor past it
      // (and past any debug instructions that follow).
      CurrentTop = NextIfDebug(++CurrentTop, CurrentBottom);
    } else {
      moveInstruction(mi, CurrentTop);
    }
  }

  // Reposition debug instructions next to the real instructions they are
  // associated with.
  placeDebugValues();
}
