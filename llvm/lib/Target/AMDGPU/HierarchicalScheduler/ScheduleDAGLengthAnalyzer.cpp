//===- ScheduleDAGLengthAnalyzer.cpp - Schedule-length analysis ----------===//
//
// Implementation of the standalone schedule-length / bubble analysis pass.
// See ScheduleDAGLengthAnalyzer.h for the design rationale.
//
//===----------------------------------------------------------------------===//

#include "ScheduleDAGLengthAnalyzer.h"
#include "GCNSubtarget.h"
#include "RegionInfo.h"
#include "ScheduleLengthAnalysis.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "machine-scheduler"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

ScheduleDAGLengthAnalyzer::ScheduleDAGLengthAnalyzer(
    MachineFunction &mf, const MachineLoopInfo *mli, AAResults *aa,
    LiveIntervals *lis)
    : ScheduleDAGInstrs(mf, mli, /*RemoveKillFlags=*/false), aa_(aa),
      lis_(lis) {}

ScheduleDAGLengthAnalyzer::~ScheduleDAGLengthAnalyzer() {
  // Flush one JSON file per function for the HTML viewer. Skip functions with
  // no schedulable region (nothing to visualize).
  if (regions_by_block_.empty()) {
    return;
  }
  const std::string scheduler =
      MachineInstrSchedulerConfig::GetConfig().GetSchedulerAsString();
  ScheduleLengthAnalyzer::WriteVizJson(MF, scheduler, regions_by_block_);
}

void ScheduleDAGLengthAnalyzer::schedule() {
  // scheduleRegions has already called enterRegion, so RegionBegin/RegionEnd
  // bound this region's final instruction order (whatever the previous
  // scheduler pass produced). Build the SUnit DAG over that order and analyze
  // it.
  //
  // We deliberately do NOT reorder. The MachineFunction is left exactly as we
  // found it: buildSchedGraph only reads the MIR to construct SUnits, and
  // because we never call moveInstruction the instruction order is unchanged
  // and placeDebugValues is unnecessary (no DBG_VALUE needs repositioning).
  buildSchedGraph(aa_);

  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());

  if (!printed_banner_) {
    llvm::outs() << "\n=== Schedule-length analysis: " << MF.getName()
                 << " ===\n";
    printed_banner_ = true;
  }

  RegionInfo region(RegionBegin, RegionEnd, *lis_);
  RegionViz viz = ScheduleLengthAnalyzer::AnalyzeRegionFinalSchedule(
      SUnits, st, MF, *lis_, MF.getRegInfo(), region, region_index_++);
  regions_by_block_[BB->getNumber()].push_back(std::move(viz));
}
