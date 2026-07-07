//===- ScheduleDAGLengthAnalyzer.h - Schedule-length analysis --*- C++ -*-===//
//
// A standalone pre-RA analysis pass that runs after whichever scheduler
// produced the code (MaxOcc, MaxIlp, OptSched, or the hierarchical scheduler)
// and emits per-region schedule-length / bubble statistics.
//
// It never reorders instructions. For each region the outer driver
// (scheduleRegions) hands it, schedule() builds the SUnit DAG from the
// region's final MachineInstr order and passes it to
// ScheduleLengthAnalyzer::AnalyzeRegionFinalSchedule. The MachineFunction is
// left untouched: buildSchedGraph only reads the MIR, and with no
// moveInstruction and no placeDebugValues, nothing is modified.
//
// It inherits ScheduleDAGInstrs (not ScheduleDAGMILive) deliberately: the
// analyzer neither reorders nor tracks register pressure, so it needs no
// MachineSchedStrategy. ScheduleDAGInstrs still initializes the scheduling
// model (latencies) in its constructor and provides buildSchedGraph, which is
// all the analysis requires. AA and LiveIntervals -- which ScheduleDAGMILive
// would have held -- are passed in and stored here.
//
// Being a separate pass -- rather than a mode of the hierarchical scheduler --
// keeps analysis orthogonal to scheduling: every scheduler is analyzed the
// same way through the same path, including the hierarchical scheduler's own
// output. The pass is inserted unconditionally (see AMDGPUTargetMachine), so
// bubble stats are collected no matter which scheduler is configured.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEDAGLENGTHANALYZER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEDAGLENGTHANALYZER_H

#include "llvm/CodeGen/ScheduleDAGInstrs.h"

namespace llvm {

class AAResults;
class LiveIntervals;
class MachineFunction;
class MachineLoopInfo;

namespace hierarchical_scheduler {

class ScheduleDAGLengthAnalyzer : public ScheduleDAGInstrs {
  // Held so schedule() can drive buildSchedGraph and the analyzer.
  // ScheduleDAGMILive would have owned these; ScheduleDAGInstrs does not.
  AAResults *aa_;
  LiveIntervals *lis_;
  // Program-order index of the next region, used only to label output.
  int region_index_ = 0;
  // Whether the per-function banner has been printed yet.
  bool printed_banner_ = false;

public:
  ScheduleDAGLengthAnalyzer(MachineFunction &mf, const MachineLoopInfo *mli,
                            AAResults *aa, LiveIntervals *lis);

  // Called per-region by the outer driver (scheduleRegions), after the region
  // has been entered. Builds the SUnit DAG for the region's final order and
  // hands it to ScheduleLengthAnalyzer. Reorders nothing.
  void schedule() override;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEDAGLENGTHANALYZER_H
