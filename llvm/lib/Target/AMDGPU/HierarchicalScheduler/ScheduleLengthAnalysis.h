//===- ScheduleLengthAnalysis.h - Post-schedule length/bubble analysis ---===//
//
// Analyzes the FINAL schedule of a region — after all hierarchical passes
// have run and applied their orders to the MachineFunction — and prints
// per-region statistics to stdout. It is analysis only: it never changes
// the schedule.
//
// The region graph is rebuilt twice from the final MF order, under two
// latency "lenses":
//   - raw:      edge latencies unscaled (latency_divisor = 1). The pure
//               single-wave latency exposure of this ordering.
//   - adjusted: edge latencies divided by the kernel's achieved occupancy,
//               modeling other waves on the SIMD covering the latency. How
//               much of the raw stall the hardware actually hides.
// Both lenses time the SAME fixed ordering, so they are two views of one
// schedule, reported side by side.
//
// The analyzer computes its divisors itself and does not consult the
// scheduler's ScaleEdgeLatencies option, so the scheduling view and the
// analysis view never affect one another.
//
// Per lens it reports the critical-path length, the achieved schedule
// length, the lower bound max(critical_path, num_ops), and the stall
// (bubble) cycles split three ways by cause:
//   - nothing-ready: no unissued instruction had its operands ready — a
//                    pure critical-path wait (intrinsic).
//   - over-budget:   an instruction was ready, but issuing it would push
//                    register pressure past the occupancy budget (the
//                    ILP-vs-occupancy tradeoff).
//   - avoidable:     a ready instruction that also fit the budget existed,
//                    and the schedule idled anyway (a beatable schedule).
//
// Alongside the printed stats and schedule_length_analysis.csv, the analyzer
// returns a structured RegionViz per region and writes a per-function JSON
// (schedule_length_viz/<func>.json) that the standalone HTML viewer
// (viz/schedule_length_viewer.html) renders as a control-flow graph of basic
// blocks with a per-region cycle-accurate timeline.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHANALYSIS_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHANALYSIS_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include <map>
#include <string>
#include <vector>

namespace llvm {

class GCNSubtarget;
class LiveIntervals;
class MachineFunction;
class MachineRegisterInfo;
struct SUnit;

namespace hierarchical_scheduler {

class RegionInfo;

/// One issued instruction on a region's cycle-accurate timeline, together with
/// the idle cycles that immediately precede it. issue_cycle is the cycle this
/// instruction issues at under the lens's latency model (single-issue: exactly
/// one instruction per cycle, so a gap of N cycles before it means N bubbles).
struct TimelineEntry {
  /// A contiguous run of idle cycles before this instruction, split by cause.
  /// cause is one of "nothing-ready" / "over-budget" / "avoidable" (that fixed
  /// order); only nonzero segments are stored.
  struct BubbleSeg {
    const char *cause;
    int cycles;
  };
  int issue_cycle = 0;
  std::string text;          // the MachineInstr as one line
  bool on_crit_path = false; // zero-slack under this lens's latencies
  // The instruction's "latency shadow" length: the max latency over its
  // outgoing data edges (how many cycles until its result feeds a consumer).
  // Large for loads (memory latency), ~1 for ALU. Scaled by the lens's
  // latency divisor, so the adjusted lens shows the occupancy-shrunk shadow.
  int latency = 0;
  int vgpr = 0, sgpr = 0;    // live registers right after this instruction issues
  double vgpr_pct = 0.0;     // vgpr as a percentage of the occupancy budget
  double sgpr_pct = 0.0;     // sgpr as a percentage of the occupancy budget
  SmallVector<BubbleSeg, 3> bubble_before;
};

/// One latency lens over a region: the summary stats plus the timeline.
struct LensViz {
  int critical_path = 0;
  int length = 0; // achieved cycles
  int floor = 0;  // lower bound: max(num_ops, critical_path + 1)
  int bubbles = 0;
  int nothing_ready = 0;
  int over_budget = 0;
  int avoidable = 0;
  double efficiency = 0.0; // floor / length
  std::vector<TimelineEntry> timeline;
};

/// Everything the visualization needs for one scheduling region.
struct RegionViz {
  int index = 0;
  int num_ops = 0;
  int occupancy = 0;
  int peak_vgpr = 0, vgpr_budget = 0;
  int peak_sgpr = 0, sgpr_budget = 0;
  LensViz raw;
  LensViz adjusted;
};

class ScheduleLengthAnalyzer {
public:
  /// Analyze one region's final schedule under both latency lenses. Prints the
  /// stats block to stdout and appends to schedule_length_analysis.csv (as
  /// before), and returns the structured per-region data the visualization
  /// consumes. `sunits` must already be populated by buildSchedGraph for
  /// `region`; the analyzer builds its own two ScheduleGraphs (raw +
  /// occupancy-adjusted) and does not disturb the scheduler's state.
  static RegionViz AnalyzeRegionFinalSchedule(MutableArrayRef<SUnit> sunits,
                                              const GCNSubtarget &st,
                                              const MachineFunction &mf,
                                              const LiveIntervals &lis,
                                              const MachineRegisterInfo &mri,
                                              const RegionInfo &region,
                                              int region_index);

  /// Write schedule_length_viz/<func>.json for one function: a control-flow
  /// graph of its basic blocks (edges from each MBB's successors) with the
  /// per-region RegionViz attached to the block that contains it. Called once
  /// per function after all its regions have been analyzed.
  /// `regions_by_block` maps a MachineBasicBlock number to the regions in that
  /// block, in program order; blocks with no schedulable region still appear
  /// as nodes so the CFG is complete. `scheduler_name` labels which scheduler
  /// produced the schedule (one scheduler per compilation).
  static void WriteVizJson(
      const MachineFunction &mf, StringRef scheduler_name,
      const std::map<int, std::vector<RegionViz>> &regions_by_block);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHANALYSIS_H
