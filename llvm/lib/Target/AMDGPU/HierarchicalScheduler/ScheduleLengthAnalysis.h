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
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHANALYSIS_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHANALYSIS_H

#include "llvm/ADT/ArrayRef.h"

namespace llvm {

class GCNSubtarget;
class LiveIntervals;
class MachineFunction;
class MachineRegisterInfo;
struct SUnit;

namespace hierarchical_scheduler {

class RegionInfo;

class ScheduleLengthAnalyzer {
public:
  /// Analyze one region's final schedule under both latency lenses and print
  /// a stats block to stdout. `sunits` must already be populated by
  /// buildSchedGraph for `region`; the analyzer builds its own two
  /// ScheduleGraphs from them (raw + occupancy-adjusted) and does not
  /// disturb the scheduler's state.
  static void AnalyzeRegionFinalSchedule(MutableArrayRef<SUnit> sunits,
                                         const GCNSubtarget &st,
                                         const MachineFunction &mf,
                                         const LiveIntervals &lis,
                                         const MachineRegisterInfo &mri,
                                         const RegionInfo &region,
                                         int region_index);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif
