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
#include "SubgraphFormation.h"
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
  llvm::outs() << "HierarchicalScheduler: recorded region["
               << (regions_.size() - 1) << "]: instrs="
               << region.GetNumInstrs()
               << " orig_reg_occ="
               << region.GetOriginalRegisterOnlyOccupancy()
               << " vgpr=" << rp.getVGPRNum(st.hasGFX90AInsts())
               << " sgpr=" << rp.getSGPRNum() << "\n";
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
  llvm::outs() << "HierarchicalScheduler: finalizeSchedule ("
               << regions_.size() << " regions)\n";

  SortRegionsByOriginalRegisterOnlyOccupancyAscending();

  // TODO: Remove this temporary print once we've confirmed the sort.
  llvm::outs() << "HierarchicalScheduler: region order after sort:\n";
  for (size_t i = 0; i < regions_.size(); ++i) {
    const RegionInfo &r = regions_[i];
    llvm::outs() << "\tregion[" << i << "]: instrs=" << r.GetNumInstrs()
                 << " orig_reg_occ="
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
  llvm::outs() << "\n=== Pass: MaximizeOccupancy === (configured_limit="
               << configured_limit << ")\n";

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
      llvm::outs() << "\tregion[" << i << "]: SKIPPED (orig_reg_only="
                   << original_register_only_occupancy
                   << " >= kernel_so_far=" << kernel_occupancy_so_far
                   << "; " << num_skipped << "/" << total
                   << " remaining, " << percent_skipped << "%)\n";
      break;
    }

    // Region heading printed before the per-region work so input:/output:
    // blocks below nest under it visually.
    llvm::outs() << "\n\tregion[" << i << "]: instrs="
                 << region.GetNumInstrs()
                 << " orig_reg_occ=" << original_register_only_occupancy
                 << "\n";

    // Determine highest occupancy achievable for region.
    int best_region_occupancy = ScheduleRegionForMaximumOccupancy(region);

    // Update kernel_occupancy_so_far.
    int kernel_occupancy_after_region =
        std::min(kernel_occupancy_so_far, best_region_occupancy);
    llvm::outs() << "\t\tkernel_occupancy: " << kernel_occupancy_so_far
                 << " -> " << kernel_occupancy_after_region << "\n";
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

  llvm::outs() << "\n\tPASS RESULT: kernel_occupancy="
               << kernel_occupancy_so_far
               << " (MFI->Occupancy now " << mfi_->getOccupancy() << ")\n";
}

// Pre-search per-region telemetry: shape of the region the DFS is
// about to schedule. Emits an `input:` block with grouped sub-keys
// (pressure / length / ilp / graph / subgraphs). `header_indent` is
// the column of the `input:` header line; sub-keys are nested one
// tab deeper, and subgraph items one tab deeper again.
static void PrintPreScheduleInfo(const ScheduleGraph &graph,
                                 const ScheduleConstructor &input,
                                 const GCNSubtarget &st,
                                 StringRef header_indent) {
  const GCNRegPressure &input_peak =
      input.GetPressureTracker().GetPeakPressure();
  const std::string key_indent = (header_indent + "\t").str();
  llvm::outs()
      << header_indent << "input:\n"
      << key_indent << "pressure: vgpr="
      << input_peak.getVGPRNum(st.hasGFX90AInsts())
      << " sgpr=" << input_peak.getSGPRNum()
      << " reg_only_occ="
      << input.GetPressureTracker().GetRegisterOnlyOccupancy()
      << " all_factors_occ="
      << input.GetPressureTracker().GetAllFactorsRegionOnlyOccupancy()
      << "\n"
      << key_indent << "length:   cycles="
      << input.GetLengthTracker().GetCurrentCycle()
      << " floor=" << graph.GetGraphLengthFloor() << "\n"
      << key_indent << "ilp:      score="
      << input.GetIlpTracker().GetIlpScore() << "\n"
      << key_indent << "graph:    nodes=" << graph.NumSchedulingUnits()
      << " critical_path=" << graph.GetCriticalPathLength() << "\n";
  graph.PrintSubgraphInfos(llvm::outs(), key_indent);
}

// Post-search per-region telemetry: shape of the schedule DFS
// produced. Emits an `output:` block with grouped sub-keys
// (pressure / length / ilp / search / rates). `header_indent` is
// the column of the `output:` header line; sub-keys are nested one
// tab deeper.
template <typename PolicyT>
static void PrintPostScheduleInfo(const ScheduleGraph &graph,
                                  const ScheduleConstructor &dfs_best,
                                  const DfsSearch<PolicyT> &search,
                                  const GCNSubtarget &st,
                                  bool order_changed,
                                  StringRef header_indent) {
  const GCNRegPressure &dfs_peak =
      dfs_best.GetPressureTracker().GetPeakPressure();
  const std::string key_indent = (header_indent + "\t").str();
  llvm::outs()
      << header_indent << "output:\n"
      << key_indent << "pressure: vgpr="
      << dfs_peak.getVGPRNum(st.hasGFX90AInsts())
      << " sgpr=" << dfs_peak.getSGPRNum()
      << " reg_only_occ="
      << dfs_best.GetPressureTracker().GetRegisterOnlyOccupancy()
      << " all_factors_occ="
      << dfs_best.GetPressureTracker().GetAllFactorsRegionOnlyOccupancy()
      << "\n"
      << key_indent << "length:   cycles="
      << dfs_best.GetLengthTracker().GetCurrentCycle()
      << " floor=" << graph.GetGraphLengthFloor() << "\n"
      << key_indent << "ilp:      score="
      << dfs_best.GetIlpTracker().GetIlpScore() << "\n"
      // `search:` carries 8 metrics — too wide for one line. Split into
      // counters (line 1) and boolean flags (line 2). The continuation
      // uses 10 spaces of within-line padding so values stack visually
      // under `calls=` — these spaces are alignment, not indentation
      // (the indent tabs come from key_indent).
      << key_indent << "search:   calls=" << search.ScheduleCallCount().lifetime
      << " length_prunes="
      << search.GetLengthHistoryTracker().PruneCount().lifetime
      << " pressure_prunes="
      << search.GetPressureHistoryTracker().PruneCount().lifetime
      << " complete_schedules=" << search.CompleteSchedulesCount()
      << " best_updates=" << search.BestUpdatesCount()
      << "\n"
      << key_indent << "          timed_out="
      << (search.RegionTimedOut() ? "yes" : "no")
      << " length_cap_hit="
      << (search.GetLengthHistoryTracker().MemoryCapHit().lifetime ? "yes" : "no")
      << " pressure_cap_hit="
      << (search.GetPressureHistoryTracker().MemoryCapHit().lifetime ? "yes" : "no")
      << "\n";

  // rates: per-region throughput so cross-scheduler comparisons
  // aren't sensitive to total budget. When elapsed rounds to zero
  // (trivially-small regions that complete in <1ms), the rate is
  // undefined — emit a "-" placeholder so the elapsed measurement
  // is still visible.
  int64_t region_elapsed_ms = search.GetRegionElapsedMs();
  int64_t schedule_calls = search.ScheduleCallCount().lifetime;
  llvm::outs() << key_indent << "rates:    ";
  if (region_elapsed_ms > 0) {
    double calls_per_sec = schedule_calls / (region_elapsed_ms / 1000.0);
    llvm::outs() << "calls/s=" << calls_per_sec;
  } else {
    llvm::outs() << "calls/s=-";
  }
  llvm::outs() << " elapsed_ms=" << region_elapsed_ms
               << " order_changed=" << (order_changed ? "yes" : "no")
               << "\n";
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
    // Occupancy pass has no phases: input:/output: live directly under
    // region[N] at indent level 2 (\t\t).
    PrintPreScheduleInfo(graph, input_schedule_constructor, st, "\t\t");

    ScheduleConstructor dfs_best_schedule_constructor = search.Run();

    bool changed =
        input_schedule_constructor.GetScheduleOrder() !=
            dfs_best_schedule_constructor.GetScheduleOrder();

    PrintPostScheduleInfo(graph, dfs_best_schedule_constructor, search, st,
                          changed, "\t\t");

    ApplyScheduleOrder(region, dfs_best_schedule_constructor);

    achieved_all_factors_occupancy =
        dfs_best_schedule_constructor.GetPressureTracker()
            .GetAllFactorsRegionOnlyOccupancy();
  });
  return achieved_all_factors_occupancy;
}

// Outer loop of the length-minimization pass. See header.
void ScheduleDAGHierarchicalScheduler::RunMinimizeLengthPass() {
  llvm::outs() << "\n=== Pass: MinimizeLength === (" << regions_.size()
               << " regions)\n";

  int regions_improved = 0;
  int regions_at_floor = 0;
  int regions_timed_out = 0;
  int regions_unchanged = 0;

  for (size_t i = 0; i < regions_.size(); ++i) {
    // Region heading printed before the per-region work so input:/
    // phase blocks below nest under it visually.
    llvm::outs() << "\n\tregion[" << i
                 << "]: instrs=" << regions_[i].GetNumInstrs() << "\n";
    LengthRegionStats stats = ScheduleRegionForMinimumLength(regions_[i]);

    if (stats.output_length < stats.input_length) {
      ++regions_improved;
    } else {
      ++regions_unchanged;
    }
    if (stats.output_length == stats.floor) {
      ++regions_at_floor;
    }
    if (stats.timed_out) {
      ++regions_timed_out;
    }
  }

  llvm::outs() << "\n\tPASS RESULT: regions=" << regions_.size()
               << " improved=" << regions_improved
               << " unchanged=" << regions_unchanged
               << " at_floor=" << regions_at_floor
               << " timed_out=" << regions_timed_out << "\n";
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
// than one. Single-iteration runs are skipped — the per-iteration
// values would duplicate what the subsequent `output:` block shows.
// `indent` is the column for the `iter[N]:` lines themselves.
static void MaybePrintIterationLogs(
    ArrayRef<LengthPassIterationLog> logs, StringRef indent) {
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
        << indent << "iter[" << i << "]: target=" << e.target
        << " result=" << e.result_length
        << " calls=" << e.schedule_calls
        << " calls/s=" << calls_per_sec
        << " elapsed_ms=" << e.elapsed_ms
        << " length_prunes=" << e.length_history_prunes
        << " pressure_prunes=" << e.pressure_history_prunes
        << " length_cap_hit=" << (e.length_history_cap_hit ? "yes" : "no")
        << " pressure_cap_hit="
        << (e.pressure_history_cap_hit ? "yes" : "no")
        << " timed_out="
        << (e.region_timed_out_observed ? "yes" : "no")
        << "\n";
  }
}

// Print the `summary:` line for the iterative phase, summarizing how
// the outer loop terminated. `indent` is the column for the line.
static void PrintIterativePhaseSummary(StringRef indent, int iterations_run,
                                       StringRef terminated_via,
                                       int final_target) {
  llvm::outs() << indent << "summary:  iterations=" << iterations_run
               << " terminated_via=" << terminated_via;
  if (terminated_via == "feasible") {
    llvm::outs() << " final_target=" << final_target;
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
constexpr bool kUseTargetFeasibilityIteration = true;

// Compile-time selector for the length-minimization policy used by
// both phases. When true, both phases use
// DfsMinimizeLengthRefineOccupancyPolicy: after finding a length-
// optimal schedule, the search continues exploring same-length
// completions to refine continuous register-occupancy score
// (giving RA more headroom and reducing spill risk). When false,
// DfsMinimizeLengthPolicy is used — pure length min, stops on
// first length-optimal schedule.
//
// Picked once and threaded through both helpers as a template
// argument. Switching to runtime-configurable (e.g., a misched.txt
// entry branching between the two template instantiations at the
// orchestrator) is a follow-up.
// Compile-time selector for which length-min policy to use. Three
// options:
//   kNone:            pure length-min, no refinement (base policy).
//   kRefineOccupancy: after length floor, continue exploring
//                     same-length completions to refine occupancy.
//   kRefineIlp:       after length floor, continue exploring
//                     same-length completions to refine ILP.
// Pick one; the corresponding policy class becomes LengthMinPolicy
// and is threaded through both length-min helpers via templates.
// Runtime selection (e.g., via misched.txt) is a follow-up.
enum class LengthMinPolicyChoice { kNone, kRefineOccupancy, kRefineIlp };
constexpr LengthMinPolicyChoice kLengthMinPolicyChoice =
    LengthMinPolicyChoice::kRefineIlp;

using LengthMinPolicy = std::conditional_t<
    kLengthMinPolicyChoice == LengthMinPolicyChoice::kRefineIlp,
    DfsMinimizeLengthRefineIlpPolicy,
    std::conditional_t<
        kLengthMinPolicyChoice == LengthMinPolicyChoice::kRefineOccupancy,
        DfsMinimizeLengthRefineOccupancyPolicy,
        DfsMinimizeLengthPolicy>>;

// Phase 1 helper: target-feasibility iteration. Walks target
// length from the static graph floor up to input_length-1 on a
// dedicated DfsSearch with its own per-region budget. The first
// iteration to find a complete schedule with length <= target
// updates `best_schedule_constructor` and exits the loop (targets
// tested in ascending order, so the first feasibility is the
// optimum). Loop also exits on region timeout or natural
// exhaustion of the target range, leaving best at whatever was
// captured (input on no feasibility found).
//
// No-op when the loop range is empty (input_length <= floor).
template <typename Policy>
static void RunIterativeLengthMinPhase(
    ScheduleGraph &graph, const GCNSubtarget &st,
    const MachineFunction &mf, const LiveIntervals &lis,
    const ScheduleConstructor &input_schedule_constructor,
    ScheduleConstructor &best_schedule_constructor,
    bool &any_timed_out) {
  const int floor = graph.GetGraphLengthFloor();
  const int input_length =
      input_schedule_constructor.GetLengthTracker().GetCurrentCycle();

  // form_subgraphs=false: the orchestrator (ScheduleRegionForMinimumLength)
  // ran formation once for the whole region before either phase
  // started. Re-running here would treat the existing subgraph
  // proxies as nested-subgraph candidates and fatal.
  DfsSearch<Policy> iter_search(graph, st, mf, lis,
                                /*form_subgraphs=*/false);

  // Default outcome before any iteration runs:
  //   - "input_optimal" when the loop range is empty (input_length
  //     already at the floor, search would have no room to improve).
  //   - "infeasible" otherwise — overridden when the loop breaks
  //     on feasibility or timeout, and stays as "infeasible" iff
  //     the loop walks the full target range without success and
  //     without timing out.
  StringRef terminated_via =
      (input_length <= floor) ? "input_optimal" : "infeasible";
  int final_target = -1;
  int iterations_run = 0;
  SmallVector<LengthPassIterationLog, 8> iter_logs;

  // Phase header at level 2 (\t\t). Iter / output: / summary: lines
  // below are level 3 (one tab deeper).
  llvm::outs() << "\t\ttarget_iteration:\n";

  for (int target = floor; target < input_length; ++target) {
    iter_search.ResetForReuse(target);
    ScheduleConstructor result = iter_search.Run();
    ++iterations_run;

    const int result_length =
        result.GetLengthTracker().GetCurrentCycle();
    iter_logs.push_back(
        CaptureIterationLog(target, result_length, iter_search));

    if (result_length <= target) {
      best_schedule_constructor = result;
      terminated_via = "feasible";
      final_target = target;
      break;
    }
    if (iter_search.RegionTimedOut()) {
      terminated_via = "timeout";
      break;
    }
  }

  bool iter_changed =
      input_schedule_constructor.GetScheduleOrder() !=
      best_schedule_constructor.GetScheduleOrder();

  MaybePrintIterationLogs(iter_logs, "\t\t\t");
  PrintPostScheduleInfo(graph, best_schedule_constructor, iter_search, st,
                        iter_changed, "\t\t\t");
  PrintIterativePhaseSummary("\t\t\t", iterations_run, terminated_via,
                             final_target);

  if (iter_search.RegionTimedOut()) {
    any_timed_out = true;
  }
}

// Phase 2 helper: plain min-search on a fresh DfsSearch with its
// own per-region budget. The target ceiling is the tighter of
// input_length and any improvement iteration just produced —
// i.e., the current outer best's length. Plain's policy bound
// then comes out at min(ceiling, plain_seed_best.length - 1),
// giving aggressive pruning right out of the gate when iteration
// improved.
//
// When iteration timed out (or walked the full range without
// finding feasibility), this gives plain an untouched per-region
// budget — the no-regression-vs-plain fallback. When iteration
// is off, this is the only search that runs. Updates outer best
// only on strict improvement so iteration's optimum isn't
// overwritten by an equivalent plain result.
template <typename Policy>
static void RunPlainLengthMinPhase(
    ScheduleGraph &graph, const GCNSubtarget &st,
    const MachineFunction &mf, const LiveIntervals &lis,
    const ScheduleConstructor &input_schedule_constructor,
    ScheduleConstructor &best_schedule_constructor,
    bool &any_timed_out) {
  // form_subgraphs=false: see RunIterativeLengthMinPhase comment.
  // Formation is a once-per-region mutation done by the orchestrator.
  DfsSearch<Policy> plain_search(graph, st, mf, lis,
                                 /*form_subgraphs=*/false);
  int plain_target =
      best_schedule_constructor.GetLengthTracker().GetCurrentCycle();
  plain_search.ResetForReuse(plain_target);
  ScheduleConstructor plain_result = plain_search.Run();
  if (plain_result.IsBetterThan(best_schedule_constructor,
                                 Policy::kMetric)) {
    best_schedule_constructor = plain_result;
  }

  bool changed =
      input_schedule_constructor.GetScheduleOrder() !=
      best_schedule_constructor.GetScheduleOrder();

  // Phase header at level 2 (\t\t). The output: block below is level 3.
  llvm::outs() << "\t\tplain_search:\n";
  PrintPostScheduleInfo(graph, best_schedule_constructor, plain_search, st,
                        changed, "\t\t\t");

  if (plain_search.RegionTimedOut()) {
    any_timed_out = true;
  }
}

// Per-region worker. Runs DFS with DfsMinimizeLengthPolicy in two
// phases — gated target-feasibility iteration, then always-on plain
// min-search. Output is no worse than the region's current MF order
// (each phase's DfsSearch seeds best with the input) and no worse
// than plain alone (the plain phase always runs with a fresh
// per-region budget). Returns per-region stats for the driver to
// aggregate into the PASS RESULT line.
ScheduleDAGHierarchicalScheduler::LengthRegionStats
ScheduleDAGHierarchicalScheduler::ScheduleRegionForMinimumLength(
    RegionInfo &region) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  LengthRegionStats stats;

  WithRegionGraph(region, [&](ScheduleGraph &graph) {
    const ScheduleConstructor &input_schedule_constructor =
        graph.GetInputScheduleConstructor();

    // Formation is a once-per-region mutation: it materializes
    // subgraph proxies into the graph. Both phases below construct
    // their DfsSearches with form_subgraphs=false and rely on this
    // call's side effect. Running formation twice on the same graph
    // would fatal in CheckNoNestedMembers (the existing proxies
    // would be treated as members of a new subgraph).
    FormSubgraphs(graph, LengthMinPolicy::MakeFormationPolicy());

    // input: block shared by both phases — printed once per region
    // at indent level 2 (\t\t), directly under the region heading.
    PrintPreScheduleInfo(graph, input_schedule_constructor, st, "\t\t");

    ScheduleConstructor best_schedule_constructor =
        input_schedule_constructor;
    bool any_timed_out = false;

    if constexpr (kUseTargetFeasibilityIteration) {
      RunIterativeLengthMinPhase<LengthMinPolicy>(
          graph, st, MF, *LIS, input_schedule_constructor,
          best_schedule_constructor, any_timed_out);
    }

    RunPlainLengthMinPhase<LengthMinPolicy>(
        graph, st, MF, *LIS, input_schedule_constructor,
        best_schedule_constructor, any_timed_out);

    ApplyScheduleOrder(region, best_schedule_constructor);

    stats.input_length =
        input_schedule_constructor.GetLengthTracker().GetCurrentCycle();
    stats.output_length =
        best_schedule_constructor.GetLengthTracker().GetCurrentCycle();
    stats.floor = graph.GetGraphLengthFloor();
    stats.timed_out = any_timed_out;
  });

  return stats;
}

// Main hierarchical scheduling path.
void ScheduleDAGHierarchicalScheduler::RunHierarchicalScheduler() {
  InitFunction();

  llvm::outs() << "\n=== HierarchicalScheduler === (regions="
               << regions_.size() << " target_occupancy="
               << mfi_->getOccupancy() << ")\n";

  // Shakedowns are validation harnesses: noisy and slow. Off by default;
  // opt in via the `RunShakedowns` option in misched.txt.
  if (MachineInstrSchedulerConfig::GetConfig().HasSchedulingOption(
          MachineInstrSchedulerConfig::SchedulerOption::RunShakedowns)) {
    RunAllShakedowns();
  }

  RunMaximizeOccupancyPass();
  RunMinimizeLengthPass();
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
