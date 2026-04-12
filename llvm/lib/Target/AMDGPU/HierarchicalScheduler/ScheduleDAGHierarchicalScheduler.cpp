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
#include "GCNRegisterTracker.h"
#include "ScheduleLengthTracker.h"
#include "GCNSubtarget.h"
#include "RegisterTracker.h"
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

// Run all shakedown / validation tests on a single region's graph.
// Exercises register trackers, schedule length tracker, and prints
// debug info about LLVM's EntrySU/ExitSU.
void ScheduleDAGHierarchicalScheduler::RunRegionShakedowns(
    ScheduleGraph &graph) {
  llvm::outs() << "  Topo order:\n";
  for (ScheduleNode *node : graph.TopoOrder()) {
    llvm::outs() << "    " << node->ToString() << "\n";
  }

  RunRegisterTrackerShakedown(graph);
  RunGCNRegisterTrackerShakedown(graph);

  SmallVector<ScheduleNode *> topo_nodes(graph.TopoOrder().begin(),
                                         graph.TopoOrder().end());
  VerifyGCNRegisterTracker(graph, topo_nodes);
  RunScheduleLengthTrackerShakedown(graph);

  // Dump EntrySU/ExitSU edges from the LLVM DAG.
  llvm::outs() << "  EntrySU succs (" << EntrySU.Succs.size() << "):";
  for (const SDep &dep : EntrySU.Succs) {
    llvm::outs() << " SU(" << dep.getSUnit()->NodeNum << ")";
  }
  llvm::outs() << "\n";
  llvm::outs() << "  EntrySU preds (" << EntrySU.Preds.size() << "):";
  for (const SDep &dep : EntrySU.Preds) {
    llvm::outs() << " SU(" << dep.getSUnit()->NodeNum << ")";
  }
  llvm::outs() << "\n";
  llvm::outs() << "  ExitSU succs (" << ExitSU.Succs.size() << "):";
  for (const SDep &dep : ExitSU.Succs) {
    llvm::outs() << " SU(" << dep.getSUnit()->NodeNum << ")";
  }
  llvm::outs() << "\n";
  llvm::outs() << "  ExitSU preds (" << ExitSU.Preds.size() << "):";
  for (const SDep &dep : ExitSU.Preds) {
    llvm::outs() << " SU(" << dep.getSUnit()->NodeNum << ")";
  }
  llvm::outs() << "\n";
}

// Run all shakedowns: synthetic test DAG first, then per-region
// shakedowns on the first region.
void ScheduleDAGHierarchicalScheduler::RunAllShakedowns() {
  llvm::outs() << "RunAllShakedowns:\n";

  RunTestDAGShakedown();

  for (auto &region : regions_) {
    ProcessRegion(region, [&]() {
      buildSchedGraph(AA);

      SlotIndex region_begin_idx = LIS->getInstructionIndex(*RegionBegin);
      SlotIndex region_end_idx = RegionEnd == BB->end()
          ? LIS->getMBBEndIdx(BB)
          : LIS->getInstructionIndex(*RegionEnd);

      ScheduleGraph graph = ScheduleGraph::BuildFromSUnits(
          SUnits, *LIS, MF.getRegInfo(), region_begin_idx, region_end_idx);
      graph.ComputeTopologicalOrder();

      llvm::outs() << "  Region: " << region.GetNumInstrs()
                   << " instrs, graph: " << graph.Size()
                   << " nodes (" << graph.LeafSize() << " leaves)"
                   << ", topo order size: " << graph.TopoOrder().size()
                   << "\n";

      // Run detailed shakedowns on the first region only.
      if (&region == &regions_.front()) {
        RunRegionShakedowns(graph);
      }
    });
  }
}

// Main hierarchical scheduling path. Currently runs shakedowns only —
// the actual scheduling algorithm will be implemented here.
void ScheduleDAGHierarchicalScheduler::RunHierarchicalScheduler() {
  llvm::outs() << "RunHierarchicalScheduler: processing " << regions_.size()
               << " regions\n";

  RunAllShakedowns();
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

  // Count original edges.
  int original_edge_count = 0;
  for (const ScheduleNode &node : test_graph.Nodes()) {
    original_edge_count += node.NumSuccs();
  }

  test_graph.ComputeTransitiveReduction();
  const ReducedGraph &reduced = test_graph.GetReducedGraph();

  // Count reduced edges.
  int reduced_edge_count = 0;
  for (int topo_idx = 0; topo_idx < reduced.size; ++topo_idx) {
    reduced_edge_count += static_cast<int>(reduced.succs[topo_idx].size());
  }

  llvm::outs() << "  Transitive reduction: " << original_edge_count
               << " edges -> " << reduced_edge_count << " edges\n";

  // Print the reduced edges using node names from the original graph.
  llvm::outs() << "  Reduced edges:";
  for (int topo_idx = 0; topo_idx < reduced.size; ++topo_idx) {
    ScheduleNode *from = test_graph.TopoOrder()[topo_idx];
    for (int succ_topo_idx : reduced.succs[topo_idx]) {
      ScheduleNode *to = test_graph.TopoOrder()[succ_topo_idx];
      llvm::outs() << " " << from->ToString() << "->" << to->ToString();
    }
  }
  llvm::outs() << "\n";

  // Build dominator tree.
  test_graph.ComputeDominatorTree();
  llvm::outs() << "  Dominator tree:\n" << test_graph.DominatorTreeToString();

  // Cycle detection verified: BuildTestDAGWithCycle() +
  // ComputeTopologicalOrder() fires report_fatal_error with graph ToString.
  // Uncomment to re-test:
  // ScheduleGraph cyclic = ScheduleGraph::BuildTestDAGWithCycle();
  // cyclic.ComputeTopologicalOrder();
}

// Tests RegisterTracker by scheduling the first region's instructions in
// topo order and printing pressure at each step.
void ScheduleDAGHierarchicalScheduler::RunRegisterTrackerShakedown(
    ScheduleGraph &graph) {
  SmallVector<ScheduleNode *> nodes(graph.TopoOrder().begin(),
                                    graph.TopoOrder().end());
  RegisterTracker tracker(nodes, MF.getRegInfo(),
                          *MF.getSubtarget().getRegisterInfo());

  llvm::outs() << "  Register pressure trace (topo order):\n";
  for (ScheduleNode *node : graph.TopoOrder()) {
    llvm::outs() << "    " << node->ToString() << "\n";
    tracker.Schedule(node);
    llvm::outs() << "      " << tracker.DescribeRegOps(node) << "\n";
    llvm::outs() << "      -> SGPR="
                 << tracker.GetCurrentRegisterPressure(RegType::kSGPR)
                 << " VGPR="
                 << tracker.GetCurrentRegisterPressure(RegType::kVGPR)
                 << "\n";
  }
  llvm::outs() << "  Peak: SGPR="
               << tracker.GetPeakRegisterPressure(RegType::kSGPR)
               << " VGPR="
               << tracker.GetPeakRegisterPressure(RegType::kVGPR) << "\n";
}

// Tests GCNRegisterTracker by scheduling the first region's instructions in
// topo order (printing pressure at each step), then unscheduling everything
// in reverse order (also printing pressure), and verifying that pressure
// returns to zero.
void ScheduleDAGHierarchicalScheduler::RunGCNRegisterTrackerShakedown(
    ScheduleGraph &graph) {
  SmallVector<ScheduleNode *> nodes(graph.TopoOrder().begin(),
                                    graph.TopoOrder().end());
  GCNRegisterTracker tracker(nodes, MF.getRegInfo(),
                             *MF.getSubtarget().getRegisterInfo(), *LIS);

  // --- Forward pass: schedule in topo order ---
  llvm::outs() << "  GCN register pressure trace (topo order):\n";
  for (ScheduleNode *node : graph.TopoOrder()) {
    tracker.Schedule(node);
    llvm::outs() << "    " << node->ToString() << "\n";
    llvm::outs() << "      " << tracker.DescribeRegOps(node) << "\n";
    llvm::outs() << "      " << tracker.DescribePressure() << "\n";
  }

  // --- Reverse pass: unschedule in reverse topo order ---
  llvm::outs() << "  GCN register pressure trace (unschedule):\n";
  for (int i = static_cast<int>(nodes.size()) - 1; i >= 0; --i) {
    tracker.Unschedule(nodes[i]);
    llvm::outs() << "    undo " << nodes[i]->ToString() << "\n";
    llvm::outs() << "      " << tracker.DescribePressure() << "\n";
  }

  // --- Verify round-trip ---
  const GCNRegPressure &final_pressure = tracker.GetCurrentPressure();
  const GCNRegPressure &final_peak = tracker.GetPeakPressure();
  bool pass = (final_pressure.getSGPRNum() == 0 &&
               final_pressure.getVGPRNum(false) == 0 &&
               final_peak.getSGPRNum() == 0 &&
               final_peak.getVGPRNum(false) == 0 &&
               tracker.GetLiveRegs().empty());
  llvm::outs() << "  Round-trip result: "
               << tracker.DescribePressure()
               << "  live_regs=" << tracker.GetLiveRegs().size()
               << (pass ? "  PASS" : "  FAIL") << "\n";
  if (!pass) {
    report_fatal_error("GCNRegisterTracker round-trip test failed: "
                       "state did not return to zero after full unschedule");
  }
}

// Cross-check our GCNRegisterTracker against LLVM's GCNUpwardRPTracker
// on the SAME instruction order. Both trackers process the same sequence
// of instructions; any peak difference is either:
//   - The known whole-register kill overestimate (ours >= LLVM's), or
//   - A bug.
//
// We use GCNUpwardRPTracker because its recede() works on arbitrary
// instruction order (not just BB order). We walk the order backwards
// with recede(), matching the GCNIterativeScheduler pattern.
//
// Entry/exit nodes have no MachineInstr, so they are skipped for LLVM's
// tracker. Our tracker still processes them (they carry live-in defs
// and live-out uses). The initial live set that GCNUpwardRPTracker gets
// from LiveIntervals at the region boundary is equivalent to our exit
// node's uses, so the peaks should still be comparable.
void ScheduleDAGHierarchicalScheduler::VerifyGCNRegisterTracker(
    ScheduleGraph &graph,
    ArrayRef<ScheduleNode *> order) {
  const MachineRegisterInfo &mri = MF.getRegInfo();

  // --- Collect MachineInstrs from the order, skipping entry/exit ---
  SmallVector<MachineInstr *, 32> mis;
  // Map from MachineInstr* to its index in order for labeling.
  DenseMap<MachineInstr *, ScheduleNode *> mi_to_node;
  for (ScheduleNode *node : order) {
    if (!node->IsLeaf()) {
      continue;
    }
    SUnit *su = node->GetSUnit();
    if (su && su->getInstr()) {
      mis.push_back(su->getInstr());
      mi_to_node[su->getInstr()] = node;
    }
  }

  if (mis.empty()) {
    return;
  }

  // --- LLVM's tracker: walk order backward, record pressure at each step ---
  // recede(MI) moves from "after MI" to "before MI". So after receding
  // MI, getLiveRegs() gives the state BEFORE MI — which is the same as
  // the state AFTER the previous instruction in forward order.
  //
  // To align with our forward tracker (which reports state AFTER each
  // instruction), we shift by one: LLVM's pressure after receding
  // mis[i] = our state after mis[i-1]. We record the pre-recede state
  // (the initial live set from reset) as the "after last instruction"
  // value.
  GCNUpwardRPTracker llvm_tracker(*LIS);
  llvm_tracker.reset(*mis.back());

  // llvm_pressures[i] = pressure after forward instruction mis[i].
  SmallVector<GCNRegPressure, 32> llvm_pressures(mis.size());

  // Initial state (after reset, before any recede) = state after the
  // last instruction.
  llvm_pressures[mis.size() - 1] =
      llvm::getRegPressure(mri, llvm_tracker.getLiveRegs());

  // Walk backward. After receding mis[i], the live set = state before
  // mis[i] = state after mis[i-1].
  for (int i = static_cast<int>(mis.size()) - 1; i >= 0; --i) {
    llvm_tracker.recede(*mis[i]);
    if (i > 0) {
      llvm_pressures[i - 1] =
          llvm::getRegPressure(mri, llvm_tracker.getLiveRegs());
    }
  }

  // --- Our tracker: walk order forward, record pressure at each step ---
  GCNRegisterTracker tracker(
      SmallVector<ScheduleNode *>(order.begin(), order.end()),
      mri, *MF.getSubtarget().getRegisterInfo(), *LIS);

  llvm::outs() << "  Cross-check per-instruction (same order):\n";
  llvm::outs() << "    " << std::string(60, '-') << "\n";

  int mi_idx = 0;
  GCNRegPressure our_peak;
  GCNRegPressure llvm_peak;
  for (ScheduleNode *node : order) {
    tracker.Schedule(node);

    SUnit *su = node->IsLeaf() ? node->GetSUnit() : nullptr;
    bool has_mi = su && su->getInstr();

    if (has_mi && mi_idx < static_cast<int>(llvm_pressures.size())) {
      const GCNRegPressure &ours = tracker.GetCurrentPressure();
      const GCNRegPressure &theirs = llvm_pressures[mi_idx];
      our_peak = max(our_peak, ours);
      llvm_peak = max(llvm_peak, theirs);

      bool sgpr_match = (ours.getSGPRNum() == theirs.getSGPRNum());
      bool vgpr_match = (ours.getVGPRNum(false) == theirs.getVGPRNum(false));

      llvm::outs() << "    " << node->ToString() << "\n"
                   << "      ours: SGPR=" << ours.getSGPRNum()
                   << " VGPR=" << ours.getVGPRNum(false)
                   << "  LLVM: SGPR=" << theirs.getSGPRNum()
                   << " VGPR=" << theirs.getVGPRNum(false);
      if (!sgpr_match || !vgpr_match) {
        llvm::outs() << "  <-- DIFF";
      }
      llvm::outs() << "\n";
      mi_idx++;
    }
  }

  llvm::outs() << "    " << std::string(60, '-') << "\n"
               << "    peak ours: SGPR=" << our_peak.getSGPRNum()
               << " VGPR=" << our_peak.getVGPRNum(false)
               << "  peak LLVM: SGPR=" << llvm_peak.getSGPRNum()
               << " VGPR=" << llvm_peak.getVGPRNum(false) << "\n";
}

// Tests ScheduleLengthTracker: schedules in topo order printing length/bubbles
// at each step, then unschedules everything and verifies state returns to zero.
void ScheduleDAGHierarchicalScheduler::RunScheduleLengthTrackerShakedown(
    ScheduleGraph &graph) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());
  ScheduleLengthTracker tracker(graph, st);

  // --- Forward pass: schedule in topo order ---
  llvm::outs() << "  Schedule length trace (topo order):\n";
  for (ScheduleNode *node : graph.TopoOrder()) {
    // Print the instruction.
    llvm::outs() << "    " << node->ToString() << "\n";

    // Print data dependency predecessors and their edge latencies.
    for (const ScheduleEdge &edge : node->Preds()) {
      if (!edge.IsDataEdge()) {
        continue;
      }
      llvm::outs() << "      pred " << edge.node_->ToString()
                   << "  latency=" << edge.latency_;
      if (tracker.IsScheduled(edge.node_)) {
        llvm::outs() << "  pred_cycle="
                     << tracker.GetScheduledCycle(edge.node_)
                     << "  ready_at="
                     << (tracker.GetScheduledCycle(edge.node_) +
                         edge.latency_);
      }
      llvm::outs() << "\n";
    }

    // Schedule and print resulting state.
    tracker.Schedule(node);
    llvm::outs() << "      -> cycle=" << tracker.GetScheduledCycle(node)
                 << "  " << tracker.Describe() << "\n";
  }

  // --- Reverse pass: unschedule everything ---
  llvm::outs() << "  Schedule length trace (unschedule):\n";
  for (int i = static_cast<int>(graph.TopoOrder().size()) - 1; i >= 0; --i) {
    tracker.Unschedule();
    llvm::outs() << "    undo  " << tracker.Describe() << "\n";
  }

  // --- Verify round-trip ---
  bool pass = (tracker.GetCurrentCycle() == 0 &&
               tracker.GetTotalBubbles() == 0 &&
               tracker.GetNumScheduled() == 0);
  llvm::outs() << "  Round-trip result: " << tracker.Describe()
               << (pass ? "  PASS" : "  FAIL") << "\n";
  if (!pass) {
    report_fatal_error("ScheduleLengthTracker round-trip test failed: "
                       "state did not return to zero after full unschedule");
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
