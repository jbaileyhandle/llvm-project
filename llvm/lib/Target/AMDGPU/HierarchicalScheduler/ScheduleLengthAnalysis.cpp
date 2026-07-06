//===- ScheduleLengthAnalysis.cpp - Post-schedule length/bubble analysis -===//
//
// See ScheduleLengthAnalysis.h for what this computes and why.
//
//===----------------------------------------------------------------------===//

#include "ScheduleLengthAnalysis.h"

#include "GCNRegPressure.h"
#include "GCNRegisterTracker.h"
#include "GCNSubtarget.h"
#include "RegionInfo.h"
#include "SIMachineFunctionInfo.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "ScheduleLengthTracker.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/Format.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <fstream>
#include <memory>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Per-lens statistics for one region's final schedule.
struct LensStats {
  int num_ops = 0;         // scheduling-unit (real instruction) count
  int critical_path = 0;   // longest latency-weighted path (this lens)
  int schedule_length = 0; // achieved cycles
  int lower_bound = 0;     // graph length floor: max(num_ops, critical_path+1)
  int total_bubbles = 0;   // stall cycles (schedule_length - num_ops)
  int bubbles_nothing_ready = 0;
  int bubbles_over_budget = 0;
  int bubbles_avoidable = 0;
};

// part as a percentage of whole; 0 when whole is 0.
double Percent(int part, int whole) {
  return whole > 0 ? 100.0 * part / whole : 0.0;
}

// Split a region's bubble cycles into the three causes by re-walking the
// final order through a fresh ScheduleConstructor. At each stall (the next
// instruction's ready cycle is beyond the current cycle) the ready frontier
// tells us what could have filled the slot, and a tentative schedule of each
// ready instruction tells us whether it would fit the occupancy budget.
//
// Within one stall the live-set is fixed (nothing issues), so a ready
// instruction stays ready and its budget-fit is constant. Readiness is
// therefore monotonic in the cycle, and the per-cause cycle counts follow
// analytically from two thresholds:
//   ready_any_time = earliest cycle ANY ready instruction exists
//   ready_fit_time = earliest cycle a ready instruction that FITS exists
// with cur <= ready_any_time <= ready_fit_time <= node_ready. These cut the
// half-open stall window [cur, node_ready) into three CONTIGUOUS, DISJOINT
// pieces:
//   nothing-ready = [cur,            ready_any_time)
//   over-budget   = [ready_any_time, ready_fit_time)
//   avoidable     = [ready_fit_time, node_ready)
// Their lengths telescope to (node_ready - cur), i.e. exactly this stall's
// bubble count — every bubble cycle is attributed once, no double counting.
// Stalls across nodes don't overlap either (the node issues AT node_ready,
// which the half-open interval excludes), so the three counters sum to
// total_bubbles over the region.
void ClassifyBubbles(const ScheduleGraph &graph, const GCNSubtarget &st,
                     const MachineFunction &mf, int occupancy,
                     LensStats &stats) {
  const int vgpr_budget = static_cast<int>(st.getMaxNumVGPRs(occupancy));
  const int sgpr_budget =
      static_cast<int>(st.getMaxNumSGPRs(occupancy, /*Addressable=*/true));
  const bool unified_vgpr = st.hasGFX90AInsts();

  // Fresh full-featured constructor (recipe nullopt -> length + pressure
  // trackers on). We replay the same fixed order the input schedule used, so
  // its per-node cycles match the input constructor's.
  ScheduleConstructor sc(graph, st, mf);

  SmallVector<const ScheduleNode *, 32> ready;
  for (const ScheduleNode *node :
       graph.GetInputScheduleConstructor().GetScheduleOrder()) {
    // Only real instructions occupy an issue slot / advance the cycle;
    // proxies and boundary nodes are scheduled but don't create bubbles.
    if (node->IsSchedulingUnit()) {
      const ScheduleLengthTracker &lt = sc.GetLengthTracker();
      const int cur = lt.GetCurrentCycle();
      const int node_ready = lt.GetMinScheduleCycle(node);
      if (node_ready > cur) {
        // Snapshot the ready frontier (stable across Schedule/Unschedule).
        ready.clear();
        sc.GetReadyListSnapshot(ready);

        // Defaults assume nothing else is ever ready in this window, so the
        // whole stall is nothing-ready.
        int ready_any_time = node_ready;
        int ready_fit_time = node_ready;
        for (const ScheduleNode *cand : ready) {
          if (!cand->IsSchedulingUnit() || cand == node) {
            continue;
          }
          const int cand_ready = lt.GetMinScheduleCycle(cand);
          ready_any_time = std::min(ready_any_time, cand_ready);

          // Would issuing cand now keep us within the occupancy budget?
          // Tentatively schedule it, read the resulting pressure, undo.
          sc.Schedule(cand);
          const GCNRegPressure &p =
              sc.GetPressureTracker().GetCurrentPressure();
          const bool fits =
              static_cast<int>(p.getVGPRNum(unified_vgpr)) <= vgpr_budget &&
              static_cast<int>(p.getSGPRNum()) <= sgpr_budget;
          sc.Unschedule();
          if (fits) {
            ready_fit_time = std::min(ready_fit_time, cand_ready);
          }
        }

        // Clamp the thresholds into the stall window. `fit >= any` is forced
        // so the three pieces stay disjoint (see the partition note above).
        const int any = std::max(cur, std::min(ready_any_time, node_ready));
        const int fit = std::max(any, std::min(ready_fit_time, node_ready));
        stats.bubbles_nothing_ready += any - cur;
        stats.bubbles_over_budget += fit - any;
        stats.bubbles_avoidable += node_ready - fit;
      }
    }
    sc.Schedule(node);
  }
}

LensStats ComputeLensStats(const ScheduleGraph &graph, const GCNSubtarget &st,
                           const MachineFunction &mf, int occupancy) {
  LensStats stats;
  const ScheduleLengthTracker &lt =
      graph.GetInputScheduleConstructor().GetLengthTracker();

  stats.schedule_length = lt.GetCurrentCycle();
  stats.total_bubbles = lt.GetTotalBubbles();
  stats.critical_path = graph.GetCriticalPathLength();
  stats.num_ops = graph.NumSchedulingUnits();
  stats.lower_bound = graph.GetGraphLengthFloor();

  ClassifyBubbles(graph, st, mf, occupancy, stats);
  return stats;
}

// One bubble-cause line: raw count, share of all bubbles, share of all cycles.
void PrintBubbleCause(raw_ostream &os, StringRef cause, int count,
                      const LensStats &s) {
  os << "\t\t\t" << cause << "=" << count
     << format(" (%.1f%% of bubbles, %.1f%% of cycles)\n",
               Percent(count, s.total_bubbles),
               Percent(count, s.schedule_length));
}

// One latency lens: the length/critical-path/floor summary plus the bubble
// breakdown by cause.
void PrintLensReport(raw_ostream &os, StringRef label, const LensStats &s) {
  const double efficiency =
      static_cast<double>(s.lower_bound) / s.schedule_length;
  os << "\t\t" << label << ": critpath=" << s.critical_path
     << " length=" << s.schedule_length << " floor=" << s.lower_bound
     << format(" efficiency=%.2f", efficiency) << " bubbles=" << s.total_bubbles
     << format(" (%.1f%% of cycles)\n",
               Percent(s.total_bubbles, s.schedule_length));
  PrintBubbleCause(os, "nothing-ready", s.bubbles_nothing_ready, s);
  PrintBubbleCause(os, "over-budget", s.bubbles_over_budget, s);
  PrintBubbleCause(os, "avoidable", s.bubbles_avoidable, s);
}

// One row per region per lens, appended to schedule_length_analysis.csv.
//
// -j NOTE: written like search_outcomes.csv — a plain ofstream that truncates
// on the first write of the process (so one compile yields one clean file)
// and appends afterward. This is NOT safe under `make -j`: parallel clang
// processes each truncate on their first write and clobber each other's rows.
// We knowingly accept that here and in FlushSearchOutcomes, which shares the
// same limitation. If it ever bites, switch both to the O_APPEND
// atomic-per-row scheme kernel_resource_usage.csv uses.
void WriteCsvRows(StringRef function, int region_index, int occupancy,
                  int peak_vgpr, int vgpr_budget, int peak_sgpr,
                  int sgpr_budget, const LensStats &raw,
                  const LensStats &adj) {
  static bool started = false;
  std::ofstream csv_file("schedule_length_analysis.csv",
                         started ? std::ios::app : std::ios::trunc);
  if (!csv_file) {
    return;
  }
  if (!started) {
    csv_file << "function,region,lens,num_ops,occupancy,peak_vgpr,vgpr_budget,"
                "peak_sgpr,sgpr_budget,critical_path,length,floor,efficiency,"
                "bubbles,nothing_ready,over_budget,avoidable\n";
    started = true;
  }
  auto write_row = [&](const char *lens, const LensStats &s) {
    csv_file << function.str() << ',' << region_index << ',' << lens << ','
             << s.num_ops << ',' << occupancy << ',' << peak_vgpr << ','
             << vgpr_budget << ',' << peak_sgpr << ',' << sgpr_budget << ','
             << s.critical_path << ',' << s.schedule_length << ','
             << s.lower_bound << ','
             << static_cast<double>(s.lower_bound) / s.schedule_length << ','
             << s.total_bubbles << ',' << s.bubbles_nothing_ready << ','
             << s.bubbles_over_budget << ',' << s.bubbles_avoidable << '\n';
  };
  write_row("raw", raw);
  write_row("adjusted", adj);
}

} // namespace

void ScheduleLengthAnalyzer::AnalyzeRegionFinalSchedule(
    MutableArrayRef<SUnit> sunits, const GCNSubtarget &st,
    const MachineFunction &mf, const LiveIntervals &lis,
    const MachineRegisterInfo &mri, const RegionInfo &region,
    int region_index) {
  // Nothing to analyze for a region with no real instructions.
  if (sunits.empty()) {
    return;
  }

  // Achieved kernel occupancy after scheduling: the wave count the hardware
  // runs, and the register-budget rung. Drives the adjusted lens's latency
  // divisor and the register-% report. (Analysis-owned; independent of the
  // scheduler's ScaleEdgeLatencies option.) A real kernel always runs >= 1
  // wave; 0 is a broken invariant (and a divide-by-zero below), so fail loud.
  const int occupancy =
      static_cast<int>(mf.getInfo<SIMachineFunctionInfo>()->getOccupancy());
  if (occupancy < 1) {
    report_fatal_error(
        "ScheduleLengthAnalyzer: kernel occupancy is < 1 after scheduling");
  }

  // Two lenses over the SAME final order: raw (divisor 1) and occupancy-
  // adjusted (divisor = achieved occupancy).
  std::unique_ptr<ScheduleGraph> raw_graph =
      ScheduleGraph::BuildFromSUnits(sunits, st, mf, lis, mri, region,
                                     /*latency_divisor=*/1);
  std::unique_ptr<ScheduleGraph> adj_graph =
      ScheduleGraph::BuildFromSUnits(sunits, st, mf, lis, mri, region,
                                     /*latency_divisor=*/occupancy);

  const LensStats raw = ComputeLensStats(*raw_graph, st, mf, occupancy);
  const LensStats adj = ComputeLensStats(*adj_graph, st, mf, occupancy);

  // Register pressure is a property of the ordering, not the latency lens, so
  // report it once (from the raw graph). Peak vs the occupancy budget: >100%
  // would drop a rung, which the hierarchical scheduler disallows.
  const GCNRegisterTracker &rt =
      raw_graph->GetInputScheduleConstructor().GetPressureTracker();
  const int peak_vgpr = static_cast<int>(rt.GetPeakVGPRNum());
  const int peak_sgpr = static_cast<int>(rt.GetPeakSGPRNum());
  const int vgpr_budget = static_cast<int>(st.getMaxNumVGPRs(occupancy));
  const int sgpr_budget =
      static_cast<int>(st.getMaxNumSGPRs(occupancy, /*Addressable=*/true));

  raw_ostream &os = outs();
  os << "\tregion[" << region_index
     << "] schedule-length analysis: ops=" << raw.num_ops
     << " occ=" << occupancy << "  peakVGPR=" << peak_vgpr << "/" << vgpr_budget
     << format(" (%.0f%%)", Percent(peak_vgpr, vgpr_budget))
     << "  peakSGPR=" << peak_sgpr << "/" << sgpr_budget
     << format(" (%.0f%%)", Percent(peak_sgpr, sgpr_budget)) << "\n";
  PrintLensReport(os, "raw     ", raw);
  PrintLensReport(os, "adjusted", adj);

  WriteCsvRows(mf.getName(), region_index, occupancy, peak_vgpr, vgpr_budget,
               peak_sgpr, sgpr_budget, raw, adj);
}
