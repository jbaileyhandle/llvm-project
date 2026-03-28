//===- ScheduleDAGHierarchicalScheduler.cpp - Hierarchical Scheduler ------===//
//
// Implementation of the hierarchical instruction scheduler for AMDGPU.
//
// Currently a no-op: schedule() records regions and finalizeSchedule()
// does nothing. The actual scheduling logic will be added later.
//
//===----------------------------------------------------------------------===//

#include "ScheduleDAGHierarchicalScheduler.h"
#include "MaliciousScheduler.h"
#include "ScheduleGraph.h"
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
  RegionInfo region(RegionBegin, RegionEnd);
  regions_.push_back(region);

  // TODO: Remove this temporary print once we've confirmed the pass runs.
  llvm::outs() << "HierarchicalScheduler: recorded region "
               << regions_.size() << " (" << region.GetNumInstrs()
               << " instrs)\n";
}

// Called once after all regions in all blocks have been visited.
// Checks the config and dispatches to the appropriate scheduler.
void ScheduleDAGHierarchicalScheduler::finalizeSchedule() {
  llvm::outs() << "HierarchicalScheduler: finalizeSchedule called with "
               << regions_.size() << " regions\n";

  const MachineInstrSchedulerConfig &config =
      MachineInstrSchedulerConfig::GetConfig();

  if (config.HasHierarchicalSchedulerOption(
          MachineInstrSchedulerConfig::HierarchicalSchedulerOption::
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

// Main hierarchical scheduling path. Builds the LLVM DAG and our
// ScheduleGraph for each region. Currently does not reorder instructions —
// this is where the hierarchical scheduling algorithm will be implemented.
void ScheduleDAGHierarchicalScheduler::RunHierarchicalScheduler() {
  llvm::outs() << "RunHierarchicalScheduler: processing " << regions_.size()
               << " regions\n";

  RunTestDAGShakedown();

  for (auto &region : regions_) {
    ProcessRegion(region, [&]() {
      buildSchedGraph(AA);
      ScheduleGraph graph =
          ScheduleGraph::BuildFromSUnits(SUnits, EntrySU, ExitSU);
      graph.ComputeTopologicalOrder();

      // TODO: Remove these temporary prints.
      llvm::outs() << "  Region: " << region.GetNumInstrs()
                   << " instrs, graph: " << graph.Size()
                   << " nodes (" << graph.LeafSize() << " leaves)"
                   << ", topo order size: " << graph.TopoOrder().size()
                   << "\n";
      // Print topo order for the first region only to avoid flooding output.
      if (&region == &regions_.front()) {
        llvm::outs() << "  First region topo order:\n";
        for (ScheduleNode *node : graph.TopoOrder()) {
          llvm::outs() << "    " << node->ToString() << "\n";
        }
      }
    });
  }
}

// Exercises graph algorithms on a synthetic test DAG with known structure.
// Will be extended as we add new algorithms (transitive reduction, dominator
// trees, etc.).
void ScheduleDAGHierarchicalScheduler::RunTestDAGShakedown() {
  ScheduleGraph test_graph = ScheduleGraph::BuildTestDAG();
  test_graph.ComputeTopologicalOrder();

  llvm::outs() << "  Test DAG topo order:";
  for (ScheduleNode *node : test_graph.TopoOrder()) {
    llvm::outs() << " " << node->ToString();
  }
  llvm::outs() << "\n";

  // Cycle detection verified: BuildTestDAGWithCycle() +
  // ComputeTopologicalOrder() fires report_fatal_error with graph ToString.
  // Uncomment to re-test:
  // ScheduleGraph cyclic = ScheduleGraph::BuildTestDAGWithCycle();
  // cyclic.ComputeTopologicalOrder();
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
