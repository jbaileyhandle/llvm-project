//===- ScheduleDAGHierarchicalScheduler.cpp - Hierarchical Scheduler ------===//
//
// Implementation of the hierarchical instruction scheduler for AMDGPU.
//
// Currently a no-op: schedule() records regions and finalizeSchedule()
// does nothing. The actual scheduling logic will be added later.
//
//===----------------------------------------------------------------------===//

#include "ScheduleDAGHierarchicalScheduler.h"
#include "BfsDpSearch.h"
#include "BfsDpSettings.h"
#include "BranchAndBoundSearch.h"
#include "DecomposeAndSchedule.h"
#include "DfsOccupancyDispatch.h"
#include "DfsSearch.h"
#include "OccupancySearchDispatch.h"
#include "GCNRegisterTracker.h"
#include "HierarchicalConfig.h"
#include "MaliciousScheduler.h"
#include "ScheduleConstructor.h"
#include "SearchPolicies.h"
#include "SubgraphDagDump.h"
#include "SearchOutcomeLog.h"
#include "SubgraphFormation.h"
#include "GCNSubtarget.h"
#include "OccupancyTargetUtil.h"
#include "SIMachineFunctionInfo.h"
#include "ScheduleGraph.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/Format.h"
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

// Sort `regions_` ascending by the integer occupancy implied by each region's
// original peak pressure, breaking ties so spill-regime regions come before
// non-spill regions at the same occupancy. The occupancy pass processes
// hardest-first and breaks at the first region that is at/above the kernel
// ceiling AND not in the spill regime; ordering spill-regime regions first at a
// given occupancy ensures the pass never breaks while a spill-regime region is
// still ahead of it in the list. See the header for the stable-sort rationale.
void ScheduleDAGHierarchicalScheduler::SortRegionsByOriginalRegisterOnlyOccupancyAscending() {
  std::stable_sort(regions_.begin(), regions_.end(),
                   [](const RegionInfo &a, const RegionInfo &b) {
                     if (a.GetOriginalRegisterOnlyOccupancy() !=
                         b.GetOriginalRegisterOnlyOccupancy()) {
                       return a.GetOriginalRegisterOnlyOccupancy() <
                              b.GetOriginalRegisterOnlyOccupancy();
                     }
                     // Tie on occupancy: spill-regime regions first.
                     return a.IsOriginalInSpillRegime() &&
                            !b.IsOriginalInSpillRegime();
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

  if (HierarchicalConfig::Get().malicious) {
    RunMaliciousScheduler();
  } else {
    RunHierarchicalScheduler();
  }

  FlushSearchOutcomes();

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
  WithRegionGraph(region, GetLatencyDivisorForScheduling(), callback);
}

void ScheduleDAGHierarchicalScheduler::WithRegionGraph(
    const RegionInfo &region, int latency_divisor,
    function_ref<void(ScheduleGraph &)> callback) {
  ProcessRegion(region, [&]() {
    buildSchedGraph(AA);

    const GCNSubtarget &st =
        static_cast<const GCNSubtarget &>(MF.getSubtarget());
    auto graph = ScheduleGraph::BuildFromSUnits(
        SUnits, st, MF, *LIS, MF.getRegInfo(), region, latency_divisor);

    callback(*graph);
  });
}

// The scheduling-view latency divisor for graph construction. Under the
// ScaleEdgeLatencies misched option, each edge's data latency is divided
// by the kernel's occupancy, modeling other waves on the same SIMD
// covering most of the memory latency at runtime; unset (the default) it
// stays 1 and edges pass through raw. MFI->getOccupancy() reflects the
// kernel-wide ceiling — initially the function default, lowered by the
// occupancy pass's per-region limitOccupancy() calls. This is the
// scheduler's own lens; the schedule-length analyzer computes its divisors
// independently, so the two views never interfere.
//
// The occupancy pass accepts/rejects on a pressure-only metric, so its
// outcome is latency-independent; a divisor that varies across regions
// while limitOccupancy tightens the ceiling is therefore irrelevant there.
// The length pass is where latency matters, and by then getOccupancy() has
// stabilized at the final kernel-wide ceiling.
int ScheduleDAGHierarchicalScheduler::GetLatencyDivisorForScheduling() const {
  if (!HierarchicalConfig::Get().scale_edge_latencies) {
    return 1;
  }
  return static_cast<int>(
      MF.getInfo<SIMachineFunctionInfo>()->getOccupancy());
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

// Apply a schedule from a completed ScheduleConstructor: extract the
// real-instruction order (GetInstrOrder skips synthetic entry/exit
// and proxies) and delegate to the MachineInstr* overload. The
// intermediate vector is negligible — a schedule is applied at most
// once per region per pass.
void ScheduleDAGHierarchicalScheduler::ApplyScheduleOrder(
    const RegionInfo &region,
    const ScheduleConstructor &sc) {
  ApplyScheduleOrder(region, sc.GetInstrOrder());
}

// Stub pass: schedule every region in topo order and apply. Exercises
// the full pipeline without any real search logic.
void ScheduleDAGHierarchicalScheduler::RunTopoPass() {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  for (auto &region : regions_) {
    WithRegionGraph(region, [&](ScheduleGraph &graph) {
      ScheduleConstructor sc(graph, st, MF);

      for (ScheduleNode *node : graph.GetTopoOrder()) {
        sc.Schedule(node);
      }

      ApplyScheduleOrder(region, sc);
    });
  }
}

// The per-kernel occupancy target -- the `max` of a misched
// `kernel <sig>/<min>,<max>` line -- for this function, if set. This is the
// Hierarchical scheduler's internal occupancy ceiling. Unlike an
// amdgpu-waves-per-eu attribute max, it is deliberately NOT written to the
// function, so it only lowers MFI's Occupancy (the scheduling target), never
// MFI's WavesPerEU.second (getMaxWavesPerEU) -- and it is the latter that
// AMDGPUAsmPrinter uses to pad reserved registers. So it steers scheduling
// without padding registers or cutting the waves the runtime launches; the
// final occupancy stays register-derived.
static std::optional<int>
GetPerKernelOccupancyTarget(const MachineFunction &mf) {
  const MachineInstrSchedulerConfig &cfg =
      MachineInstrSchedulerConfig::GetConfig();
  if (!cfg.HasConfig()) {
    return std::nullopt;
  }
  // Per-kernel `kernel .../<min>,<max>` max if present, else the global
  // all_kernels_occupancy max (mutually exclusive; at most one is ever set).
  return cfg.GetEffectiveMaxWavesPerEUForFunction(mf.getFunction());
}

// Compute the occupancy pass's starting ceiling and publish it to MFI. The
// starting point is MFI's current occupancy: the structural maximum
// (arch ∩ LDS ∩ launch bounds, ignoring registers) UNLESS a misched per-kernel
// `<min>,<max>` target lowered it at MFI construction. That per-kernel target is
// therefore already reflected in `ceiling` here and needs no separate handling.
// This additionally applies the occupancy.max_occ_above_input=X cap:
// min(current, input_occ + X), where input_occ is the min original register-only
// occupancy over all regions (the function's input occupancy). The final ceiling
// is clamped up to the launch floor (getMinWavesPerEU, which a misched `min` may
// have lowered) and written to MFI, so the occupancy pass and the length pass both
// see it. The DFS occupancy policy and decompose's outer DFS honor the lowered
// target (BFS-DP maximizes regardless). See OccupancyConfig.
static int ApplyOccupancyTargetCap(SIMachineFunctionInfo &mfi,
                                   const MachineFunction &mf,
                                   ArrayRef<RegionInfo> regions) {
  int ceiling = static_cast<int>(mfi.getOccupancy());
  const OccupancyConfig &occ_cfg = HierarchicalConfig::Get().occupancy;

  // Cap: occupancy.max_occ_above_input lowers the ceiling to input_occ + X.
  if (occ_cfg.max_occ_above_input.has_value() && !regions.empty()) {
    int input_occ = ceiling;
    for (const RegionInfo &r : regions) {
      input_occ = std::min(input_occ, r.GetOriginalRegisterOnlyOccupancy());
    }
    ceiling = std::min(ceiling, input_occ + *occ_cfg.max_occ_above_input);
    llvm::outs() << "\t(occupancy cap: input_occ=" << input_occ << " + "
                 << *occ_cfg.max_occ_above_input << " -> target " << ceiling
                 << ")\n";
  }

  // Cap: a misched per-kernel `max` lowers the ceiling to that absolute target.
  // With the MaxOccupancy pre-pass now honoring the same cap, MFI already sits at
  // this target by the time we get here, so this is a self-contained re-assertion
  // rather than the sole enforcement point.
  if (std::optional<int> per_kernel_target = GetPerKernelOccupancyTarget(mf)) {
    ceiling = std::min(ceiling, *per_kernel_target);
    llvm::outs() << "\t(per-kernel occupancy target: " << *per_kernel_target
                 << ")\n";
  }

  // Clamp up to the launch floor and publish to MFI. A no-op when no cap lowered
  // the ceiling below the current occupancy (limitOccupancy only ever lowers).
  // input_occ + X is always at or above the floor, so nothing here goes below it.
  LimitOccupancyAboveFloor(mfi, ceiling);
  return ceiling;
}

// Maximize-occupancy outer loop. See header for detail.
void ScheduleDAGHierarchicalScheduler::RunMaximizeOccupancyPass() {
  // Tag any DumpSubgraphDag output from this pass into the
  // "occupancy" subdir (no-op unless the option is set).
  SubgraphDagDumpPassScope dump_scope("occupancy");

  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  // Grab initial occupancy, ignoring registers
  // This should match the configured limit for the function,
  // which we reset previously
  int configured_limit = static_cast<int>(mfi_->getOccupancy());
  int non_register =
      GCNRegisterTracker::ComputeNonRegisterOccupancy(st, MF);
  // Normally this is the structural max. A per-kernel `<min>,<max>` target is
  // honored by the MaxOccupancy pre-pass that runs first and lowers MFI to it, so
  // the expected baseline is that target when one is set. Anything else means MFI
  // was lowered without a matching reset.
  std::optional<int> per_kernel_target = GetPerKernelOccupancyTarget(MF);
  int expected_limit = per_kernel_target.has_value()
                           ? std::min(non_register, *per_kernel_target)
                           : non_register;
  if (configured_limit != expected_limit) {
    report_fatal_error(
        "RunMaximizeOccupancyPass: MFI->getOccupancy() disagrees with "
        "ComputeNonRegisterOccupancy — something lowered the MFI value "
        "without a matching reset");
  }

  // TODO: Remove this temporary print once the pass is wired up.
  llvm::outs() << "\n=== Pass: MaximizeOccupancy === (configured_limit="
               << configured_limit << ")\n";

  // Effective starting ceiling: structural max, or capped to input_occ + X
  // when occupancy.max_occ_above_input is set. In
  // optimize_every_region_past_occupancy_target mode the ceiling is the
  // unreachable kAboveHardwareMaxOccupancy, so no region's original occupancy
  // is ever at/above it and the per-region skip below never fires.
  int kernel_occupancy_so_far =
      HierarchicalConfig::Get()
              .occupancy.optimize_every_region_past_occupancy_target
          ? static_cast<int>(kAboveHardwareMaxOccupancy)
          : ApplyOccupancyTargetCap(*mfi_, MF, regions_);

  // Per-pass counters. `attempted` is regions where DFS actually ran
  // (i.e., not short-circuited by the "already at kernel ceiling"
  // skip). The three termination buckets are mutually exclusive and
  // sum to `attempted`.
  int attempted_count = 0;
  int fully_explored_count = 0;
  int timed_out_count = 0;
  int policy_satisfied_count = 0;

  for (size_t i = 0; i < regions_.size(); ++i) {
    RegionInfo &region = regions_[i];
    int original_register_only_occupancy =
        region.GetOriginalRegisterOnlyOccupancy();

    // Stop when this region's original register-only occupancy is already at or
    // above the running kernel ceiling AND its input schedule is not in the
    // spill regime. Ascending iteration -- with spill-regime regions ordered
    // first at each occupancy (see the sort) -- guarantees every later region is
    // also at/above the ceiling and non-spilling, so none can raise the kernel
    // occupancy, and none has spill to relieve: no further work can help.
    //
    // A spill-regime region at/above the ceiling is deliberately NOT skipped:
    // the occupancy search may still cut its register pressure / spill area even
    // though its occupancy is pinned. The sort's spill-first tiebreak ensures
    // every such region is reached and processed before this break fires.
    //
    // On the first iteration, kernel_occupancy_so_far == configured_limit, so
    // this also catches the case where region 0's occupancy is already maxed out
    // by other factors (e.g. arch max / LDS / launch bounds).
    if (original_register_only_occupancy >= kernel_occupancy_so_far &&
        !region.IsOriginalInSpillRegime()) {
      int num_skipped = static_cast<int>(regions_.size() - i);
      int total = static_cast<int>(regions_.size());
      int percent_skipped = (num_skipped * 100) / total;
      llvm::outs() << "\tregion[" << i << "]: SKIPPED (orig_reg_only="
                   << original_register_only_occupancy
                   << " >= kernel_so_far=" << kernel_occupancy_so_far
                   << ", not in spill regime; " << num_skipped << "/" << total
                   << " remaining, " << percent_skipped << "%)\n";
      break;
    }

    // Region heading printed before the per-region work so input:/output:
    // blocks below nest under it visually.
    llvm::outs() << "\n\tregion[" << i << "]: instrs="
                 << region.GetNumInstrs()
                 << " orig_reg_occ=" << original_register_only_occupancy
                 << "\n";

    // Tag any DumpSubgraphDag output from this region with its sorted
    // region[i] index, so dump files join back to this heading.
    SubgraphDagDumpRegionScope region_scope(static_cast<int>(i));

    // Determine highest occupancy achievable for region.
    MaxOccupancyRegionResult region_result =
        ScheduleRegionForMaximumOccupancy(region);
    ++attempted_count;
    switch (region_result.termination_cause) {
      case SearchTerminationCause::kFullyExplored:
        ++fully_explored_count;
        break;
      case SearchTerminationCause::kTimedOut:
        ++timed_out_count;
        break;
      case SearchTerminationCause::kPolicySatisfied:
        ++policy_satisfied_count;
        break;
    }

    SearchOutcome occ_row;
    occ_row.function = MF.getName().str();
    occ_row.pass = "occ";
    occ_row.region = static_cast<int>(i);
    // A decompose region emits an "outer" summary row plus one "sub{n}"
    // row per subgraph; a plain region emits a single "region" row.
    occ_row.slot = region_result.subgraph_rows.empty() ? "region" : "outer";
    occ_row.nodes = region.GetNumInstrs();
    occ_row.term_cause = region_result.termination_cause;
    occ_row.winner = region_result.winner;
    occ_row.bfs_pct = region_result.bfs_pct;
    occ_row.bfs_ms = region_result.bfs_ms;
    occ_row.dfs_ms = region_result.dfs_ms;
    occ_row.bfs_steps = region_result.bfs_steps;
    occ_row.dfs_steps = region_result.dfs_steps;
    occ_row.orig_vgpr = region_result.orig_vgpr;
    occ_row.orig_sgpr = region_result.orig_sgpr;
    occ_row.fin_vgpr = region_result.fin_vgpr;
    occ_row.fin_sgpr = region_result.fin_sgpr;
    occ_row.orig_spill_area = region_result.orig_spill_area;
    occ_row.fin_spill_area = region_result.fin_spill_area;
    occ_row.improved = region_result.raw_all_factors_occupancy >
                       original_register_only_occupancy;
    RecordSearchOutcome(occ_row);

    // Per-subgraph rows (decompose only; empty otherwise). Per-search
    // outcome only — region-level pressure/improved totals stay on the
    // "outer" row above.
    for (size_t s = 0; s < region_result.subgraph_rows.size(); ++s) {
      const MaxOccupancyRegionResult::SubgraphOutcome &subgraph_row =
          region_result.subgraph_rows[s];
      SearchOutcome sub_row;
      sub_row.function = occ_row.function;
      sub_row.pass = "occ";
      sub_row.region = static_cast<int>(i);
      sub_row.slot = "sub" + std::to_string(s);
      sub_row.nodes = subgraph_row.nodes;
      sub_row.term_cause = subgraph_row.termination_cause;
      sub_row.winner = subgraph_row.winner;
      sub_row.bfs_pct = subgraph_row.bfs_pct;
      sub_row.bfs_ms = subgraph_row.bfs_ms;
      sub_row.dfs_ms = subgraph_row.dfs_ms;
      sub_row.bfs_steps = subgraph_row.bfs_steps;
      sub_row.dfs_steps = subgraph_row.dfs_steps;
      RecordSearchOutcome(sub_row);
    }

    // Update kernel_occupancy_so_far. Uses the RAW all-factors value
    // (can be below launch floor) so the cross-region running min is
    // on the same register-driven scale as the loop's bail check
    // against original_register_only_occupancy. Using the floor-
    // clamped value here would make the bail's threshold artificially
    // high (a spill-regime region would appear to be at the floor
    // rather than below it), so the bail would fail to fire and the
    // loop would do wasted work on subsequent regions that can't
    // raise the kernel's bottleneck. LimitOccupancyAboveFloor below
    // does its own clamp at the floor when updating MFI's target
    // occupancy -- so the compiler-side target value is never set
    // below the launch floor, even when this raw running min is.
    // In optimize_every_region_past_occupancy_target mode the running ceiling
    // must stay at the unreachable kAboveHardwareMaxOccupancy, so skip both the
    // running-min update and the MFI tighten — every region runs the full
    // search and none is skipped or short-circuited.
    if (!HierarchicalConfig::Get()
             .occupancy.optimize_every_region_past_occupancy_target) {
      int kernel_occupancy_after_region = std::min(
          kernel_occupancy_so_far, region_result.raw_all_factors_occupancy);
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
      LimitOccupancyAboveFloor(*mfi_, kernel_occupancy_so_far);
    }
  }

  int total_regions = static_cast<int>(regions_.size());
  int skipped_count = total_regions - attempted_count;
  llvm::outs() << "\n\tPASS RESULT: kernel_occupancy="
               << kernel_occupancy_so_far
               << " (MFI->Occupancy now " << mfi_->getOccupancy() << ")\n"
               << "\t\tregions: total=" << total_regions
               << " attempted=" << attempted_count
               << " skipped=" << skipped_count << "\n";
  if (attempted_count > 0) {
    // Percent-of-attempted for each termination bucket. timed_out
    // last because the user reads it as the "bad outcome" anchor.
    auto pct = [&](int count) {
      return (count * 100) / attempted_count;
    };
    llvm::outs() << "\t\tattempted breakdown: fully_explored="
                 << fully_explored_count << " ("
                 << pct(fully_explored_count) << "%)"
                 << " policy_satisfied=" << policy_satisfied_count
                 << " (" << pct(policy_satisfied_count) << "%)"
                 << " timed_out=" << timed_out_count
                 << " (" << pct(timed_out_count) << "%)\n";
  }
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
      << " spill_area="
      << input.GetPressureTracker().GetVGPRSpillArea()
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
static void PrintDfsPostScheduleInfo(const ScheduleGraph &graph,
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
      << " spill_area="
      << dfs_best.GetPressureTracker().GetVGPRSpillArea()
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
      << search.LengthHistoryPruneCount().lifetime
      << " pressure_prunes="
      << search.PressureHistoryPruneCount().lifetime
      << " complete_schedules=" << search.CompleteSchedulesCount()
      << " best_updates=" << search.BestUpdatesCount()
      << " llvm_verif_rejects=" << search.LlvmTrackerRejectionsCount()
      << "\n"
      << key_indent << "          timed_out="
      << (search.RegionTimedOut() ? "yes" : "no")
      << " length_cap_hit="
      << (search.LengthHistoryMemoryCapHit().lifetime ? "yes" : "no")
      << " pressure_cap_hit="
      << (search.PressureHistoryMemoryCapHit().lifetime ? "yes" : "no")
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

// Prints the flat-path BFS-DP log rows (output / rates / timeout-fraction) for
// one occupancy search. Passed as RunOccupancySearch's bfs_after_run on the flat
// path; decompose passes a no-op. winner/bfs_pct are stamped by the dispatcher.
static void PrintBfsDpPostScheduleInfo(const ScheduleGraph &graph,
                                       BfsDpSearch &search,
                                       const SearchResult &result) {
  llvm::outs() << "\t\toutput: (BFS-DP) found_improvement="
               << result.schedule.has_value() << "\n";

  // Symmetric with the DFS "rates:" line (PrintDfsPostScheduleInfo): surface
  // BFS-DP's own step count (VisitSuccessor probes) and wall-clock so the two
  // searches are directly comparable. bfs_steps/bfs_ms are stamped in Run().
  if (result.bfs_steps.has_value() && result.bfs_ms.has_value()) {
    int steps = *result.bfs_steps;
    int ms = *result.bfs_ms;
    llvm::outs() << "\t\t\trates:    ";
    if (ms > 0) {
      llvm::outs() << "steps/s=" << (steps / (ms / 1000.0));
    } else {
      llvm::outs() << "steps/s=-";
    }
    llvm::outs() << " elapsed_ms=" << ms << " steps=" << steps << "\n";
  }

  if (result.termination_cause == SearchTerminationCause::kTimedOut) {
    // The BFS expands one partition-dag layer per graph node, so the
    // all-scheduled sink sits at depth graph.Size(). Layers completed
    // before the budget fired, over that depth, is the approximate
    // fraction of the search that was explored. Floating-point on
    // purpose: a timeout typically fires only a few layers into a
    // large graph, where integer division would round to 0.
    int levels_explored = search.GetLevelsExplored();
    int total_levels = graph.Size();
    double percent_explored =
        total_levels > 0
            ? (100.0 * levels_explored) / total_levels
            : 0.0;
    // Fixed-point: raw_ostream's operator<<(double) prints %e
    // (scientific), unreadable for a percentage.
    llvm::outs() << "\t\t\ttimed out: explored ~"
                 << llvm::format("%.2f", percent_explored)
                 << "% of levels (" << levels_explored << "/"
                 << total_levels << ")\n";
  }
}

// Per-region DecomposeAndSchedule occupancy search. Runs the full
// form-schedule-lock-search pipeline via the DecomposeAndScheduleOptions::Make
// factory:
//   - formation: the occupancy pass's configured strategy + install
//                mode (validated to be a real strategy, never none)
//   - inner_search: BFS-DP continuous → DFS continuous fallback
//   - outer_search: selected by (occ.policy, occ.search); see Make for
//                   the per-(policy, search) dispatch and budgets. May return a SearchResult
// whose schedule is empty (only the outer BFS-DP can; its DFS
// fallback always populates one); the dispatcher's common tail keeps
// the input order in that case.
static SearchResult RunOccupancyRegionWithDecompose(
    ScheduleGraph &graph, const RegionInfo &region,
    const GCNSubtarget &st, const MachineFunction &mf,
    const LiveIntervals &lis) {
  const OccupancyConfig &occ = HierarchicalConfig::Get().occupancy;
  const int seed = region.GetOriginalRegisterOnlyOccupancy();
  // Outer search/budget = (occ.policy, occ.search, occ.search.timeout); inner
  // (within-subgraph make) = (occ.inner_search, occ.inner_search.timeout). In
  // recursive decompose these are the outermost level only; deeper levels use
  // the inner config (see RecursiveDecomposeAndSchedule).
  if (occ.decompose_recursive) {
    // Recursive: cap each level at decompose_max_parts subgraphs and recurse
    // to leaves. The install mode comes from the configured formation.
    FormationConfig formation = occ.formation;
    formation.min_cut.max_parts = occ.decompose_max_parts; // subgraphs/level cap
    SearchResult result = RecursiveDecomposeAndSchedule(
        graph, st, mf, lis, seed, formation, occ.policy, occ.search,
        occ.SearchTimeoutOrUnlimited(), occ.FallbackTimeoutOrUnlimited(),
        occ.inner_search, occ.InnerSearchTimeoutOrUnlimited(),
        occ.InnerFallbackTimeoutOrUnlimited());
    llvm::outs() << "\t\toutput: (Decompose-recursive) found_improvement="
                 << result.schedule.has_value() << "\n";
    return result;
  }
  DecomposeAndScheduleOptions opts = DecomposeAndScheduleOptions::Make(
      st, mf, lis, seed, occ.formation, occ.policy, occ.search,
      occ.SearchTimeoutOrUnlimited(), occ.FallbackTimeoutOrUnlimited(),
      occ.inner_search, occ.InnerSearchTimeoutOrUnlimited(),
      occ.InnerFallbackTimeoutOrUnlimited());
  SearchResult result = DecomposeAndSchedule(graph, st, mf, opts);
  llvm::outs() << "\t\toutput: (Decompose) found_improvement="
               << result.schedule.has_value() << "\n";
  return result;
}

// Schedules the region for maximum occupancy and applies the result.
// Dispatches to one of the per-strategy helpers (Decompose / BFS-DP /
// DFS) based on the misched.txt configuration, then applies the
// chosen schedule. When the strategy returns no schedule (BFS-DP and
// Decompose's outer BFS-DP can — when nothing beats the seed) the
// input order is kept; applying it is a no-op move-wise since every
// MI is already at its CurrentTop position.
ScheduleDAGHierarchicalScheduler::MaxOccupancyRegionResult
ScheduleDAGHierarchicalScheduler::ScheduleRegionForMaximumOccupancy(
    RegionInfo &region) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  MaxOccupancyRegionResult result{
      /*raw_all_factors_occupancy=*/0,
      /*launch_floor_clamped_all_factors_occupancy=*/0,
      /*in_spill_regime=*/false,
      /*termination_cause=*/SearchTerminationCause::kFullyExplored,
  };
  WithRegionGraph(region, [&](ScheduleGraph &graph) {
    const ScheduleConstructor &input_schedule_constructor =
        graph.GetInputScheduleConstructor();
    // Occupancy pass has no phases: input:/output: live directly under
    // region[N] at indent level 2 (\t\t).
    PrintPreScheduleInfo(graph, input_schedule_constructor, st, "\t\t");

    SearchResult search_result;
    const OccupancyConfig &occupancy_config =
        HierarchicalConfig::Get().occupancy;
    if (occupancy_config.decompose) {
      // Decompose is the higher-order strategy: it forms (per the configured
      // strategy) internally, schedules each subgraph in isolation, combines.
      search_result =
          RunOccupancyRegionWithDecompose(graph, region, st, MF, *LIS);
    } else {
      // Form per the configured strategy (a no-op for kNone), then run the
      // chosen flat search over the formed graph. Formation is uniform
      // across searches, so dfs / bfsdp / bfsdp+dfs all run over whatever
      // carving was chosen.
      //
      // BFS-DP (and bfsdp+dfs) are occupancy-only: their soundness rests on
      // scheduling each DAG partition independently for register pressure,
      // which does not hold for schedule length (length depends on
      // cross-partition critical-path timing). The length pass is DFS-only.
      const FormationConfig &subgraph_formation = occupancy_config.formation;
      FormSubgraphs(
          graph,
          SubgraphFormationPolicy::FromStrategy(subgraph_formation.strategy,
                                                subgraph_formation.min_cut),
          subgraph_formation.mode);
      search_result = RunOccupancySearch(
          /*kind=*/occupancy_config.search,
          /*policy=*/occupancy_config.policy, graph, st, /*mf=*/MF, /*lis=*/*LIS,
          // Per-instruction budget (region_size * rate) overrides the flat
          // primary timeout when set. The fallback is the BFS-DP DFS-rescue
          // budget, which never runs under a per-instruction budget (that
          // requires dfs), so it stays the flat value.
          /*primary_timeout_us=*/
          occupancy_config.EffectiveTimeout(
              graph.Size(), occupancy_config.SearchTimeoutOrUnlimited()),
          /*fallback_timeout_us=*/occupancy_config.FallbackTimeoutOrUnlimited(),
          /*seed_bfs=*/
          [&](BfsDpSearch &bfs) {
            bfs.SetInitialBestScore(region.GetOriginalRegisterOnlyOccupancy());
          },
          /*bfs_after_run=*/
          [&](BfsDpSearch &search, SearchResult &r) {
            PrintBfsDpPostScheduleInfo(graph, search, r);
          },
          /*dfs_after_run=*/
          [&](auto &search, SearchResult &r) {
            // DFS always populates schedule (best is seeded with input).
            bool changed = input_schedule_constructor.GetScheduleOrder() !=
                           r.schedule->GetScheduleOrder();
            PrintDfsPostScheduleInfo(graph, *r.schedule, search, st, changed,
                                     "\t\t");
          });
    }

    // Common tail. Apply the search's schedule, or keep the input
    // order when the search produced none (only BFS-DP can).
    const ScheduleConstructor &applied =
        search_result.schedule ? *search_result.schedule
                               : input_schedule_constructor;
    ApplyScheduleOrder(region, applied);
    const GCNRegisterTracker &applied_tracker = applied.GetPressureTracker();
    result.raw_all_factors_occupancy =
        applied_tracker.GetAllFactorsRegionOnlyOccupancy();
    result.launch_floor_clamped_all_factors_occupancy =
        applied_tracker.GetLaunchFloorClampedAllFactorsRegionOnlyOccupancy();
    result.in_spill_regime = applied_tracker.IsPeakInSpillRegime();
    result.orig_spill_area =
        input_schedule_constructor.GetPressureTracker().GetVGPRSpillArea();
    result.fin_spill_area = applied_tracker.GetVGPRSpillArea();
    result.termination_cause = search_result.termination_cause;
    result.winner = search_result.winner;
    result.bfs_pct = search_result.bfs_pct;
    result.bfs_ms = search_result.bfs_ms;
    result.dfs_ms = search_result.dfs_ms;
    result.bfs_steps = search_result.bfs_steps;
    result.dfs_steps = search_result.dfs_steps;
    const GCNRegPressure &in = input_schedule_constructor.GetPressureTracker()
                                   .GetPeakPressure();
    const GCNRegPressure &out = applied.GetPressureTracker().GetPeakPressure();
    result.orig_vgpr = in.getVGPRNum(st.hasGFX90AInsts());
    result.orig_sgpr = in.getSGPRNum();
    result.fin_vgpr = out.getVGPRNum(st.hasGFX90AInsts());
    result.fin_sgpr = out.getSGPRNum();

    // Capture per-subgraph outcomes for the search_outcomes.csv
    // "sub{n}" rows. Only decompose populates schedule_result (via
    // ScheduleSubgraph); a flat search over formed subgraphs leaves it
    // unset, so this stays empty and the region records a single
    // "region" row. Subgraphs are in GetSubgraphInfos() (formation)
    // order.
    for (const std::unique_ptr<SubgraphInfo> &info :
         graph.GetSubgraphInfos()) {
      if (!info->schedule_result.has_value()) {
        continue;
      }
      const SubgraphScheduleResult &subgraph_result = *info->schedule_result;
      result.subgraph_rows.push_back(
          {/*nodes=*/static_cast<int>(info->members.size()),
           subgraph_result.termination_cause, subgraph_result.winner,
           subgraph_result.bfs_pct, subgraph_result.bfs_ms,
           subgraph_result.dfs_ms, subgraph_result.bfs_steps,
           subgraph_result.dfs_steps});
    }
  });
  return result;
}

// Outer loop of the length pass. See header.
void ScheduleDAGHierarchicalScheduler::RunLengthPass() {
  // Tag any DumpSubgraphDag output from this pass into the "length"
  // subdir (no-op unless the option is set).
  SubgraphDagDumpPassScope dump_scope("length");

  // Resolve direction once at the top. The per-region dispatch reads
  // the same config policy again for its own switch — a redundant cheap
  // read; both see the same value (the config is static for the run).
  const bool is_max =
      HierarchicalConfig::Get().length.policy == LengthPolicy::kMax;
  StringRef pass_name = is_max ? "MaximizeLength" : "MinimizeLength";

  llvm::outs() << "\n=== Pass: " << pass_name << " === (" << regions_.size()
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
    SubgraphDagDumpRegionScope region_scope(static_cast<int>(i));
    LengthRegionStats stats = ScheduleRegionForLengthPass(regions_[i]);

    // Direction-aware "improved": min wants shorter, max wants longer.
    bool improved = is_max
                        ? (stats.output_length > stats.input_length)
                        : (stats.output_length < stats.input_length);
    SearchOutcome len_row;
    len_row.function = MF.getName().str();
    len_row.pass = "len";
    len_row.region = static_cast<int>(i);
    len_row.slot = "region";
    len_row.nodes = stats.nodes;
    len_row.term_cause = stats.timed_out ? SearchTerminationCause::kTimedOut
                                         : SearchTerminationCause::kFullyExplored;
    len_row.winner = "dfs";
    len_row.dfs_ms = stats.dfs_ms;
    len_row.dfs_steps = stats.dfs_steps;
    len_row.orig_vgpr = stats.orig_vgpr;
    len_row.orig_sgpr = stats.orig_sgpr;
    len_row.fin_vgpr = stats.fin_vgpr;
    len_row.fin_sgpr = stats.fin_sgpr;
    len_row.orig_spill_area = stats.orig_spill_area;
    len_row.fin_spill_area = stats.fin_spill_area;
    len_row.orig_len = stats.input_length;
    len_row.fin_len = stats.output_length;
    len_row.improved = improved;
    RecordSearchOutcome(len_row);
    if (improved) {
      ++regions_improved;
    } else {
      ++regions_unchanged;
    }
    // at_floor is a length-MIN concept (the graph length floor is a
    // lower bound; hitting it means the schedule is provably optimal).
    // Length-MAX has no comparable bound in use here, so skip the
    // counter in max mode.
    if (!is_max && stats.output_length == stats.floor) {
      ++regions_at_floor;
    }
    if (stats.timed_out) {
      ++regions_timed_out;
    }
  }

  llvm::outs() << "\n\tPASS RESULT: regions=" << regions_.size()
               << " improved=" << regions_improved
               << " unchanged=" << regions_unchanged;
  if (!is_max) {
    llvm::outs() << " at_floor=" << regions_at_floor;
  }
  llvm::outs() << " timed_out=" << regions_timed_out << "\n";
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
      search.LengthHistoryPruneCount().current_run;
  log.pressure_history_prunes =
      search.PressureHistoryPruneCount().current_run;
  log.length_history_cap_hit =
      search.LengthHistoryMemoryCapHit().current_run;
  log.pressure_history_cap_hit =
      search.PressureHistoryMemoryCapHit().current_run;
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
// length-min pass (does not apply to length-max — that path has its
// own single-phase worker). When false, the per-region length-min
// worker makes a single search.Run() with the policy's INT_MAX
// target — the plain-min-search path. When true, the search is
// driven by an outer loop walking target from the static graph
// floor up to input_length-1, calling search.Run() at each step.
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
    bool &any_timed_out, int64_t &dfs_ms, int64_t &dfs_steps) {
  const int floor = graph.GetGraphLengthFloor();
  const int input_length =
      input_schedule_constructor.GetLengthTracker().GetCurrentCycle();

  // Runs over the already-formed graph: formation happened once for the
  // whole region (in ScheduleRegionForLengthPass) before either phase
  // started; the searches never form. Budget: per-instruction (region_size *
  // rate) when set, else the flat length.search.timeout.
  const LengthConfig &length_config = HierarchicalConfig::Get().length;
  DfsSearch<Policy> iter_search(
      graph, st, mf, lis,
      length_config.EffectiveTimeout(graph.Size(),
                                     length_config.SearchTimeoutOrUnlimited()));

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
    // DfsSearch::Run always populates SearchResult::schedule, so the
    // deref is unconditional here.
    ScheduleConstructor result = std::move(*iter_search.Run().schedule);
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

  // Accumulate this phase's DFS throughput (lifetime totals over all
  // iterations). Guard on iterations_run: with an empty target range
  // iter_search never ran, so its stopwatch was never started.
  if (iterations_run > 0) {
    dfs_ms += iter_search.GetRegionElapsedMs();
    dfs_steps += iter_search.ScheduleCallCount().lifetime;
  }

  bool iter_changed =
      input_schedule_constructor.GetScheduleOrder() !=
      best_schedule_constructor.GetScheduleOrder();

  MaybePrintIterationLogs(iter_logs, "\t\t\t");
  PrintDfsPostScheduleInfo(graph, best_schedule_constructor, iter_search, st,
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
    bool &any_timed_out, int64_t &dfs_ms, int64_t &dfs_steps) {
  // Runs over the already-formed graph (formation done once per region
  // by ScheduleRegionForLengthPass). Budget: per-instruction when set, else
  // the flat length.search.timeout.
  const LengthConfig &length_config = HierarchicalConfig::Get().length;
  DfsSearch<Policy> plain_search(
      graph, st, mf, lis,
      length_config.EffectiveTimeout(graph.Size(),
                                     length_config.SearchTimeoutOrUnlimited()));
  int plain_target =
      best_schedule_constructor.GetLengthTracker().GetCurrentCycle();
  plain_search.ResetForReuse(plain_target);
  // DfsSearch::Run always populates SearchResult::schedule.
  ScheduleConstructor plain_result = std::move(*plain_search.Run().schedule);
  if (plain_result.IsBetterThan(best_schedule_constructor,
                                 Policy::kScoreRecipe)) {
    best_schedule_constructor = plain_result;
  }
  dfs_ms += plain_search.GetRegionElapsedMs();
  dfs_steps += plain_search.ScheduleCallCount().lifetime;

  bool changed =
      input_schedule_constructor.GetScheduleOrder() !=
      best_schedule_constructor.GetScheduleOrder();

  // Phase header at level 2 (\t\t). The output: block below is level 3.
  llvm::outs() << "\t\tplain_search:\n";
  PrintDfsPostScheduleInfo(graph, best_schedule_constructor, plain_search, st,
                        changed, "\t\t\t");

  if (plain_search.RegionTimedOut()) {
    any_timed_out = true;
  }
}

// Per-region per-policy worker. Templated on the length-min policy
// so we can instantiate one copy per length-min LengthPolicy
// variant and dispatch at runtime from ScheduleRegionForLengthPass.
// Runs DFS in two phases — gated target-feasibility iteration, then
// always-on plain min-search — and writes the chosen schedule into
// `best_schedule_constructor`. `any_timed_out` aggregates the
// per-region timeout from either phase. The length-max path uses a
// dedicated worker (RunMaximizeLengthForRegion) — not this one.
template <typename Policy>
static void RunMinimizeLengthForRegionWithPolicy(
    ScheduleGraph &graph, const GCNSubtarget &st,
    const MachineFunction &mf, const LiveIntervals &lis,
    const ScheduleConstructor &input_schedule_constructor,
    ScheduleConstructor &best_schedule_constructor,
    bool &any_timed_out, int64_t &dfs_ms, int64_t &dfs_steps) {
  // Subgraph formation already happened once for this region in
  // ScheduleRegionForLengthPass (per the configured strategy); both
  // phases below run over that formed graph.

  // input: block shared by both phases — printed once per region
  // at indent level 2 (\t\t), directly under the region heading.
  PrintPreScheduleInfo(graph, input_schedule_constructor, st, "\t\t");

  if constexpr (kUseTargetFeasibilityIteration) {
    RunIterativeLengthMinPhase<Policy>(
        graph, st, mf, lis, input_schedule_constructor,
        best_schedule_constructor, any_timed_out, dfs_ms, dfs_steps);
  }

  RunPlainLengthMinPhase<Policy>(
      graph, st, mf, lis, input_schedule_constructor,
      best_schedule_constructor, any_timed_out, dfs_ms, dfs_steps);
}

// Per-region worker for the length-MAX policy. Single-phase: no
// target-feasibility iteration (the iteration concept is length-min-
// specific — walking a target length up from the floor toward the
// input). Length-max just runs a plain DFS that tries to find a
// LONGER schedule than the input. Termination is wall-clock budget
// only (ShouldEndSearch is unconditionally false for length-max).
//
// Subgraph formation already happened once for this region in
// ScheduleRegionForLengthPass; this worker runs over the formed graph,
// like the length-min worker.
static void RunMaximizeLengthForRegion(
    ScheduleGraph &graph, const GCNSubtarget &st,
    const MachineFunction &mf, const LiveIntervals &lis,
    const ScheduleConstructor &input_schedule_constructor,
    ScheduleConstructor &best_schedule_constructor,
    bool &any_timed_out, int64_t &dfs_ms, int64_t &dfs_steps) {
  PrintPreScheduleInfo(graph, input_schedule_constructor, st, "\t\t");

  // Runs over the already-formed graph (formation done once per region
  // above). Budget: per-instruction when set, else the flat length.search.timeout.
  const LengthConfig &length_config = HierarchicalConfig::Get().length;
  DfsSearch<DfsMaximizeLengthPolicy> plain_search(
      graph, st, mf, lis,
      length_config.EffectiveTimeout(graph.Size(),
                                     length_config.SearchTimeoutOrUnlimited()));
  // Don't call ResetForReuse — the default requested_target_length_
  // (INT_MAX) is the right value for length-max (no upper bound on
  // achievable length, beyond what dominance + timeout enforce).
  // RecomputeWorkingMaxScheduleCycles will still set a max-acceptable
  // value on the working length tracker each Recurse, but length-
  // max's policy hooks don't consult it.
  // DfsSearch::Run always populates SearchResult::schedule.
  ScheduleConstructor plain_result = std::move(*plain_search.Run().schedule);
  if (plain_result.IsBetterThan(best_schedule_constructor,
                                DfsMaximizeLengthPolicy::kScoreRecipe)) {
    best_schedule_constructor = plain_result;
  }
  dfs_ms += plain_search.GetRegionElapsedMs();
  dfs_steps += plain_search.ScheduleCallCount().lifetime;

  bool changed = input_schedule_constructor.GetScheduleOrder() !=
                 best_schedule_constructor.GetScheduleOrder();

  // Reuse the plain_search label so downstream tools can rely on a
  // stable section name across policies; the per-region heading
  // already disambiguates which policy ran.
  llvm::outs() << "\t\tplain_search:\n";
  PrintDfsPostScheduleInfo(graph, best_schedule_constructor, plain_search, st,
                        changed, "\t\t\t");

  if (plain_search.RegionTimedOut()) {
    any_timed_out = true;
  }
}

// Per-region worker. Resolves the length-pass policy from misched.txt
// at the top of each region and dispatches accordingly — to one of
// the templated length-min instantiations, or to the dedicated
// length-MAX worker. Output is no worse than the region's current MF
// order (each path's DfsSearch seeds best with the input). Returns
// per-region stats for the driver to aggregate into the PASS RESULT
// line.
ScheduleDAGHierarchicalScheduler::LengthRegionStats
ScheduleDAGHierarchicalScheduler::ScheduleRegionForLengthPass(
    RegionInfo &region) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  LengthRegionStats stats;

  WithRegionGraph(region, [&](ScheduleGraph &graph) {
    const ScheduleConstructor &input_schedule_constructor =
        graph.GetInputScheduleConstructor();
    ScheduleConstructor best_schedule_constructor =
        input_schedule_constructor;
    bool any_timed_out = false;
    // Cumulative DFS throughput across this region's length search(es)
    // (iterative + plain phases, or the single length-max search).
    int64_t total_dfs_ms = 0;
    int64_t total_dfs_steps = 0;

    // Form per the length pass's configured strategy (a no-op for kNone),
    // once per region before the policy search. The DfsSearches in the
    // workers below run over this formed graph. Length is DFS-only
    // (BFS-DP is occupancy-only).
    const FormationConfig &subgraph_formation =
        HierarchicalConfig::Get().length.formation;
    FormSubgraphs(
        graph,
        SubgraphFormationPolicy::FromStrategy(subgraph_formation.strategy,
                                              subgraph_formation.min_cut),
        subgraph_formation.mode);

    switch (HierarchicalConfig::Get().length.policy) {
    case LengthPolicy::kMin:
      RunMinimizeLengthForRegionWithPolicy<DfsMinimizeLengthPolicy>(
          graph, st, MF, *LIS, input_schedule_constructor,
          best_schedule_constructor, any_timed_out, total_dfs_ms,
          total_dfs_steps);
      break;
    case LengthPolicy::kMinRefineIlp:
      RunMinimizeLengthForRegionWithPolicy<
          DfsMinimizeLengthRefineIlpPolicy>(
          graph, st, MF, *LIS, input_schedule_constructor,
          best_schedule_constructor, any_timed_out, total_dfs_ms,
          total_dfs_steps);
      break;
    case LengthPolicy::kMinRefineOccupancy:
      RunMinimizeLengthForRegionWithPolicy<
          DfsMinimizeLengthRefineOccupancyPolicy>(
          graph, st, MF, *LIS, input_schedule_constructor,
          best_schedule_constructor, any_timed_out, total_dfs_ms,
          total_dfs_steps);
      break;
    case LengthPolicy::kMinBoundedSpillSignals:
      RunMinimizeLengthForRegionWithPolicy<
          DfsMinimizeLengthBoundedSpillSignalsPolicy>(
          graph, st, MF, *LIS, input_schedule_constructor,
          best_schedule_constructor, any_timed_out, total_dfs_ms,
          total_dfs_steps);
      break;
    case LengthPolicy::kMax:
      RunMaximizeLengthForRegion(graph, st, MF, *LIS,
                                 input_schedule_constructor,
                                 best_schedule_constructor, any_timed_out,
                                 total_dfs_ms, total_dfs_steps);
      break;
    }

    ApplyScheduleOrder(region, best_schedule_constructor);

    stats.input_length =
        input_schedule_constructor.GetLengthTracker().GetCurrentCycle();
    stats.output_length =
        best_schedule_constructor.GetLengthTracker().GetCurrentCycle();
    stats.floor = graph.GetGraphLengthFloor();
    stats.timed_out = any_timed_out;
    stats.nodes = graph.Size();
    const GCNRegPressure &in = input_schedule_constructor.GetPressureTracker()
                                   .GetPeakPressure();
    const GCNRegPressure &out = best_schedule_constructor.GetPressureTracker()
                                    .GetPeakPressure();
    stats.orig_vgpr = in.getVGPRNum(st.hasGFX90AInsts());
    stats.orig_sgpr = in.getSGPRNum();
    stats.fin_vgpr = out.getVGPRNum(st.hasGFX90AInsts());
    stats.fin_sgpr = out.getSGPRNum();
    stats.orig_spill_area =
        input_schedule_constructor.GetPressureTracker().GetVGPRSpillArea();
    stats.fin_spill_area =
        best_schedule_constructor.GetPressureTracker().GetVGPRSpillArea();
    stats.dfs_ms = static_cast<int>(total_dfs_ms);
    stats.dfs_steps = static_cast<int>(total_dfs_steps);
  });

  return stats;
}

// Measure a buffered order under `latency_divisor`. See header. The
// returned length is in the divisor's lens because the length tracker
// counts cycles in the units of the graph's edge weights, and the graph
// is built here with exactly that divisor.
int ScheduleDAGHierarchicalScheduler::MeasureBufferedOrderLength(
    const RegionInfo &region, const std::vector<MachineInstr *> &order,
    int latency_divisor) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());
  int length = 0;
  WithRegionGraph(region, latency_divisor, [&](ScheduleGraph &graph) {
    length = graph.MakeConstructorForInstrOrder(order, st, MF)
                 .GetLengthTracker()
                 .GetCurrentCycle();
  });
  return length;
}

// Per-region worker for the tier sweep. See header. The length-min
// worker's input:/phase blocks print at \t\t, nesting under the region
// heading the caller prints.
ScheduleDAGHierarchicalScheduler::MinAdjustedLengthRegionSchedule
ScheduleDAGHierarchicalScheduler::SearchRegionAtOccupancyTier(
    RegionInfo &region) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());
  MinAdjustedLengthRegionSchedule result;
  WithRegionGraph(region, /*latency_divisor=*/1,
                  [&](ScheduleGraph &graph) {
    const ScheduleConstructor &input_sc =
        graph.GetInputScheduleConstructor();
    ScheduleConstructor best_sc = input_sc;
    bool timed_out = false;
    int64_t dfs_ms = 0;
    int64_t dfs_steps = 0;
    // Spill-BOUNDED length min, not plain min: spill instructions are
    // inserted by the register allocator after scheduling, so they are
    // not in the DAG and the length tracker cannot price them — an
    // unbounded min-length search "shortens" spill-regime schedules by
    // stretching live ranges into spill cost the score cannot see
    // (measured: 2-3.4x GRBM regressions with spill counts tripling,
    // 2026-08-24 sweep). Bounding VGPR spill area to the input
    // baseline's, exactly as the length pass's default policy does,
    // removes that blind spot; the bound never binds for non-spilling
    // regions.
    RunMinimizeLengthForRegionWithPolicy<
        DfsMinimizeLengthBoundedSpillSignalsPolicy>(
        graph, st, MF, *LIS, input_sc, best_sc, timed_out, dfs_ms,
        dfs_steps);
    result.order = best_sc.GetInstrOrder();
    result.raw_length = best_sc.GetLengthTracker().GetCurrentCycle();
    result.issue_slots = static_cast<int>(result.order.size());
    result.achieved_occupancy =
        best_sc.GetPressureTracker()
            .GetLaunchFloorClampedAllFactorsRegionOnlyOccupancy();
    result.timed_out = timed_out;
  });
  return result;
}

// See header. MLI is the ScheduleDAGInstrs member inherited through
// ScheduleDAGMILive/ScheduleDAGMI, filled from the MachineSchedContext
// by the MachineScheduler pass (which addRequired's MachineLoopInfo) —
// never null when this scheduler is driven by the pass, so a null here
// means a construction path that skipped the context, not a kernel
// without loops.
std::vector<int64_t>
ScheduleDAGHierarchicalScheduler::ComputeMinAdjustedLengthRegionWeights()
    const {
  std::vector<int64_t> region_weights(regions_.size(), 1);
  const MinAdjustedLengthConfig &config =
      HierarchicalConfig::Get().min_adjusted_length_config;
  if (config.region_weighting != RegionWeighting::kLoopDepth) {
    return region_weights;
  }
  if (!MLI) {
    report_fatal_error("ComputeMinAdjustedLengthRegionWeights: no "
                       "MachineLoopInfo (scheduler constructed without a "
                       "MachineSchedContext?)");
  }

  // Cap far below int64 overflow of the score sums; depths that deep
  // carry no additional ranking information anyway.
  constexpr int64_t kMaxRegionWeight = 1000000000000; // 10^12
  llvm::outs() << "\tregion_weights (loop_depth, base="
               << config.loop_weight_base << "):";
  for (size_t i = 0; i < regions_.size(); ++i) {
    const int depth =
        static_cast<int>(MLI->getLoopDepth(regions_[i].GetBlock()));
    int64_t weight = 1;
    for (int d = 0; d < depth && weight < kMaxRegionWeight; ++d) {
      weight *= config.loop_weight_base;
    }
    region_weights[i] = weight;
    llvm::outs() << " [" << i << "] depth=" << depth << " w=" << weight;
  }
  llvm::outs() << "\n";
  return region_weights;
}

// Per-tier worker. See header.
ScheduleDAGHierarchicalScheduler::MinAdjustedLengthCandidate
ScheduleDAGHierarchicalScheduler::ScheduleKernelForOccupancyTier(
    int tier, ArrayRef<int64_t> region_weights) {
  // budget(tier): the length-min DFS policies read their register
  // budget from MFI's occupancy target live, so setting the target to
  // `tier` is what makes every region search prune schedules that
  // would not fit at this tier. SetOccupancyTarget lands on `tier`
  // regardless of where the previous tier left the target, so the
  // sweep order doesn't matter.
  SetOccupancyTarget(*mfi_, MF, tier);

  MinAdjustedLengthCandidate candidate;
  candidate.searched_occupancy_tier = tier;
  candidate.actual_occupancy = tier;

  for (size_t i = 0; i < regions_.size(); ++i) {
    llvm::outs() << "\n\tregion[" << i
                 << "]: instrs=" << regions_[i].GetNumInstrs() << "\n";
    SubgraphDagDumpRegionScope region_scope(static_cast<int>(i));

    candidate.region_schedules.push_back(
        SearchRegionAtOccupancyTier(regions_[i]));
    const MinAdjustedLengthRegionSchedule &region_schedule =
        candidate.region_schedules.back();

    candidate.timed_out |= region_schedule.timed_out;

    // First region initializes the kernel-wide min (achieved occupancy
    // is never below `tier`, so starting at `tier` would overshoot in
    // the all-regions-above-tier case).
    candidate.actual_occupancy =
        (i == 0) ? region_schedule.achieved_occupancy
                 : std::min(candidate.actual_occupancy,
                            region_schedule.achieved_occupancy);
  }

  // Score at the ACTUAL occupancy — arithmetic on the recorded
  // numbers; raw length is occupancy-independent so nothing is
  // re-measured. Sum across regions FIRST, then take one max: a wave
  // flows through every region, and its stalls in one region are
  // covered by other waves issuing from whatever region they are in —
  // interleaved waves execute different regions at the same time, so
  // per-region max would deny that cross-region coverage (see the
  // scoring-granularity section of
  // docs/AMDGPUMinAdjustedLengthScheduler.md). Each region enters
  // both sums times its weight — the estimated executions per wave
  // (see ComputeMinAdjustedLengthRegionWeights). The two kernel-wide
  // floors on wave-completion spacing:
  //   issue_floor     — total instructions per wave: they all pass
  //                     through the shared port; concurrency cannot
  //                     reduce it;
  //   residency_floor — ceil(total raw length / o): a wave holds a
  //                     residency slot for its whole lifetime (summed
  //                     region raw lengths, stalls included), so o
  //                     slots finish at most one wave per lifetime/o
  //                     cycles. Rounded UP via the ceiling-division
  //                     idiom ceil(a/b) = (a+b-1)/b for positive ints
  //                     (integer division truncates; adding b-1 first
  //                     pushes any nonzero remainder over).
  // The larger floor is the score; occupancy only pays while the
  // residency floor binds, so the regime print below is the model
  // saying whether occupancy matters for this kernel at all.
  int64_t issue_floor = 0;
  int64_t wave_lifetime = 0;
  for (size_t i = 0; i < candidate.region_schedules.size(); ++i) {
    const MinAdjustedLengthRegionSchedule &region_schedule =
        candidate.region_schedules[i];
    issue_floor += region_weights[i] * region_schedule.issue_slots;
    wave_lifetime += region_weights[i] * region_schedule.raw_length;
  }
  const int64_t residency_floor =
      (wave_lifetime + candidate.actual_occupancy - 1) /
      candidate.actual_occupancy;
  candidate.score_sum = std::max(issue_floor, residency_floor);
  llvm::outs() << "\n\ttier " << tier << " scoring: issue_floor="
               << issue_floor << " residency_floor=" << residency_floor
               << " -> "
               << (issue_floor >= residency_floor ? "issue-bound"
                                                  : "latency-bound")
               << "\n";

  return candidate;
}

// The min-adjusted-length pass driver. See header.
void ScheduleDAGHierarchicalScheduler::RunMinimizeAdjustedLengthPass() {
  // Tag any DumpSubgraphDag output from this pass (no-op unless the
  // option is set).
  SubgraphDagDumpPassScope dump_scope("min_adjusted_length");

  const int top_tier = static_cast<int>(mfi_->getOccupancy());
  const int floor_tier = static_cast<int>(mfi_->getMinWavesPerEU());

  llvm::outs() << "\n=== Pass: MinimizeAdjustedLength === ("
               << regions_.size() << " regions, tiers " << top_tier
               << " down to " << floor_tier << ")\n";

  if (regions_.empty()) {
    llvm::outs() << "\tPASS RESULT: no regions, nothing to do\n";
    return;
  }

  const std::vector<int64_t> region_weights =
      ComputeMinAdjustedLengthRegionWeights();

  std::optional<MinAdjustedLengthCandidate> best;
  for (int tier = top_tier; tier >= floor_tier; --tier) {
    llvm::outs() << "\n\t=== occupancy tier " << tier << " ===\n";

    MinAdjustedLengthCandidate candidate =
        ScheduleKernelForOccupancyTier(tier, region_weights);
    llvm::outs() << "\n\ttier " << tier
                 << " result: actual_occupancy=" << candidate.actual_occupancy
                 << " score_sum=" << candidate.score_sum
                 << (candidate.timed_out ? " timed_out" : "") << "\n";

    // Smallest score wins. Ties go to the higher ACTUAL occupancy —
    // insurance against latency misestimates, since more waves dampen
    // a modeling error; ties are common because issue-bound regions
    // score issue_slots at every tier. Actual occupancy is not
    // monotone in sweep order (a looser-budget search can land lower
    // pressure by accident), so the tie-break compares explicitly;
    // remaining ties keep the earlier candidate (higher searched
    // tier) for determinism.
    if (!best.has_value() || candidate.score_sum < best->score_sum ||
        (candidate.score_sum == best->score_sum &&
         candidate.actual_occupancy > best->actual_occupancy)) {
      best = std::move(candidate);
    }
  }

  // Commit. Set MFI's occupancy target to the winner's actual
  // occupancy, independent of wherever the sweep left it. Downstream
  // consumers — the register allocator's pressure limits — read this
  // target, which is what pins the kernel at the chosen occupancy.
  SetOccupancyTarget(*mfi_, MF, best->actual_occupancy);
  for (size_t i = 0; i < regions_.size(); ++i) {
    ProcessRegion(regions_[i], [&]() {
      ApplyScheduleOrder(regions_[i], best->region_schedules[i].order);
    });
  }

  llvm::outs() << "\n\tPASS RESULT: winner searched_occupancy_tier="
               << best->searched_occupancy_tier
               << " actual_occupancy=" << best->actual_occupancy
               << " score_sum=" << best->score_sum
               << " (MFI->Occupancy now " << mfi_->getOccupancy() << ")\n";
}

// Main hierarchical scheduling path.
void ScheduleDAGHierarchicalScheduler::RunHierarchicalScheduler() {
  InitFunction();

  llvm::outs() << "\n=== HierarchicalScheduler === (regions="
               << regions_.size() << " target_occupancy="
               << mfi_->getOccupancy() << ")\n";

  // The HierarchicalScheduler's typed config (built once, cached). Call
  // sites below read HierarchicalConfig::Get(); print it for visibility.
  HierarchicalConfig::Get().DebugPrint();

  // Shakedowns are validation harnesses: noisy and slow. Off by default;
  // opt in via the `run_shakedowns` flag in misched.txt.
  if (HierarchicalConfig::Get().run_shakedowns) {
    RunAllShakedowns();
  }

  if (!HierarchicalConfig::Get().skip_occupancy_pass) {
    RunMaximizeOccupancyPass();
  }
  // The min-adjusted-length pass replaces the length pass: both
  // optimize the post-occupancy schedule, but min-adjusted-length also
  // owns the occupancy choice, so running the plain length pass too
  // would be redundant work under a stale lens.
  if (HierarchicalConfig::Get().min_adjusted_length_config.enabled) {
    RunMinimizeAdjustedLengthPass();
  } else if (!HierarchicalConfig::Get().skip_length_pass) {
    RunLengthPass();
  }
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
  std::vector<MachineInstr *> instr_order;
  instr_order.reserve(scheduled_units.size());
  for (SUnit *su : scheduled_units) {
    instr_order.push_back(su->getInstr());
  }
  ApplyScheduleOrder(region, instr_order);
}

void ScheduleDAGHierarchicalScheduler::ApplyScheduleOrder(
    const RegionInfo &region,
    const std::vector<MachineInstr *> &scheduled_instrs) {
  for (MachineInstr *mi : scheduled_instrs) {
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
