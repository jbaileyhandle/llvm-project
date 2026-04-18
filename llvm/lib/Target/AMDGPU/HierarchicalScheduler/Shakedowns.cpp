//===- Shakedowns.cpp - HierarchicalScheduler validation harnesses --------===//
//
// Shakedown / validation routines for the hierarchical scheduler's
// building blocks (ScheduleGraph, register trackers, length tracker,
// ScheduleConstructor, ScheduleMetric). These are invoked from
// RunHierarchicalScheduler via RunAllShakedowns and exercise each
// component against a real region's DAG plus a synthetic test DAG.
//
// Split out of ScheduleDAGHierarchicalScheduler.cpp so the main file
// stays focused on the scheduling pipeline (dispatch, region plumbing,
// pass drivers).
//
//===----------------------------------------------------------------------===//

#include "ScheduleDAGHierarchicalScheduler.h"
#include "GCNRegisterTracker.h"
#include "GCNSubtarget.h"
#include "RegisterTracker.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "ScheduleLengthTracker.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

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
  RunScheduleConstructorShakedown(graph);
  RunScheduleMetricShakedown(graph);

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
    WithRegionGraph(region, [&](ScheduleGraph &graph) {
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
  GCNRegisterTracker tracker(nodes, MF, *LIS);

  // --- Forward pass: schedule in topo order ---
  llvm::outs() << "  GCN register pressure trace (topo order):\n";
  for (ScheduleNode *node : graph.TopoOrder()) {
    tracker.Schedule(node);
    llvm::outs() << "    " << node->ToString() << "\n";
    llvm::outs() << "      " << tracker.DescribeRegOps(node) << "\n";
    llvm::outs() << "      " << tracker.DescribePressure() << "\n";
  }

  llvm::outs() << "  Occupancy: register_only="
               << tracker.GetRegisterOnlyOccupancy()
               << " all_factors_region_only="
               << tracker.GetAllFactorsRegionOnlyOccupancy()
               << "\n";

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
      MF, *LIS);

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

// Tests ScheduleConstructor: constructs a full schedule by always picking
// the first node from the ready list, then unschedules everything and
// verifies round-trip.
void ScheduleDAGHierarchicalScheduler::RunScheduleConstructorShakedown(
    ScheduleGraph &graph) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());
  ScheduleConstructor sc(graph, st, MF, *LIS);

  // --- Forward pass: always pick the first ready node ---
  llvm::outs() << "  ScheduleConstructor trace:\n";
  while (!sc.IsDone()) {
    const auto &ready = sc.GetReadyList();
    if (ready.empty()) {
      report_fatal_error("ScheduleConstructor: ready list empty before "
                         "all nodes scheduled");
    }
    const ScheduleNode *node = *ready.begin();
    sc.Schedule(node);
    llvm::outs() << "    " << node->ToString() << "\n"
                 << "      " << sc.Describe() << "\n";
  }

  llvm::outs() << "  ScheduleConstructor (arbitrary): "
               << sc.Describe() << "\n";

  // --- Reverse pass: unschedule everything ---
  int num_scheduled = sc.GetNumScheduled();
  for (int i = 0; i < num_scheduled; ++i) {
    sc.Unschedule();
  }

  // --- Verify round-trip ---
  bool pass = (sc.GetNumScheduled() == 0 &&
               sc.GetLengthTracker().GetCurrentCycle() == 0 &&
               sc.GetLengthTracker().GetTotalBubbles() == 0);
  llvm::outs() << "  ScheduleConstructor round-trip: "
               << sc.Describe()
               << (pass ? "  PASS" : "  FAIL") << "\n";
  if (!pass) {
    report_fatal_error("ScheduleConstructor round-trip test failed");
  }

  // --- Second pass: schedule in topo order for comparison ---
  ScheduleConstructor sc2(graph, st, MF, *LIS);
  for (ScheduleNode *node : graph.TopoOrder()) {
    sc2.Schedule(node);
  }
  llvm::outs() << "  ScheduleConstructor (topo order): "
               << sc2.Describe() << "\n";
}

void ScheduleDAGHierarchicalScheduler::RunScheduleMetricShakedown(
    ScheduleGraph &graph) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());
  // Local alias for the occupancy score multiplier. One integer
  // occupancy step is worth M points in the continuous score, so
  // each bracket occupies [occ*M, (occ+1)*M).
  constexpr int M = GCNRegisterTracker::kOccScoreMultiplier;
  int max_waves = static_cast<int>(st.getMaxWavesPerEU());

  llvm::outs() << "  ScheduleMetric shakedown:\n";

  // --- Part 1: sweep VGPR cliffs, SGPR held at 0 so VGPR dominates ---
  //
  // At each cliff `ceil`, the score should be exactly M*occ (within
  // = 0, top of bracket). One register above (`ceil + 1`), we've
  // dropped one integer occupancy level, so the score must lie in
  // [(occ-1)*M, occ*M) — the range of the next-lower bracket. That
  // single bound catches both "integer occ dropped by 1" and "didn't
  // overshoot into the bracket below that."
  llvm::outs() << "    VGPR cliff sweep (sgpr=0):\n";
  llvm::outs() << "      max_waves=" << max_waves << "\n";
  for (int occ = max_waves; occ >= 1; --occ) {
    unsigned ceil = st.getMaxNumVGPRs(occ);
    int score_at = GCNRegisterTracker::ComputeContinuousOccupancyScore(
        st, ceil, 0);
    int score_above = GCNRegisterTracker::ComputeContinuousOccupancyScore(
        st, ceil + 1, 0);
    int expected_at = M * occ;
    bool pass_at = (score_at == expected_at);
    bool pass_step = (score_above >= (occ - 1) * M) &&
                     (score_above < occ * M);
    llvm::outs() << "      occ=" << occ << " ceil=" << ceil
                 << " score@ceil=" << score_at
                 << " score@ceil+1=" << score_above
                 << (pass_at && pass_step ? "  PASS" : "  FAIL")
                 << "\n";
    if (!pass_at || !pass_step) {
      report_fatal_error("VGPR cliff sweep failed");
    }
  }

  // --- Part 2: sweep SGPR cliffs, VGPR held at 1 so SGPR dominates ---
  //
  // Uses our rolled-own GetMaxNumSGPRsForOcc (built from the same
  // classifier the scoring helper uses), so the ceil values here
  // agree with what the helper sees. Skips unreachable occupancies
  // (GetMaxNumSGPRsForOcc returns kNoSGPRCliff) — those have no
  // finite cliff to probe. Also skips the bottom bracket for the
  // same reason.
  llvm::outs() << "    SGPR cliff sweep (vgpr=1):\n";
  for (int occ = max_waves; occ >= 1; --occ) {
    unsigned ceil =
        GCNRegisterTracker::GetMaxNumSGPRsForOcc(st, occ);
    if (ceil >= GCNRegisterTracker::kNoSGPRCliff) {
      // Either unreachable or the unbounded bottom bracket. Skip.
      continue;
    }
    int score_at = GCNRegisterTracker::ComputeContinuousOccupancyScore(
        st, 1, ceil);
    int score_above = GCNRegisterTracker::ComputeContinuousOccupancyScore(
        st, 1, ceil + 1);
    int expected_at = M * occ;
    bool pass_at = (score_at == expected_at);
    bool pass_step = (score_above >= (occ - 1) * M) &&
                     (score_above < occ * M);
    llvm::outs() << "      occ=" << occ << " ceil=" << ceil
                 << " score@ceil=" << score_at
                 << " score@ceil+1=" << score_above
                 << (pass_at && pass_step ? "  PASS" : "  FAIL")
                 << "\n";
    if (!pass_at || !pass_step) {
      report_fatal_error("SGPR cliff sweep failed");
    }
  }

  // --- Part 3: IsBetterThan plumbing ---
  //
  // Schedule the full graph on sc_full, leave sc_empty empty. For
  // each metric, check that IsBetterThan's verdict matches a direct
  // comparison of the underlying getter — verifies metric dispatch,
  // comparison direction, and strict-vs-tie handling with real
  // (non-fabricated) values from the region.
  ScheduleConstructor sc_empty(graph, st, MF, *LIS);
  ScheduleConstructor sc_full(graph, st, MF, *LIS);
  for (ScheduleNode *node : graph.TopoOrder()) {
    sc_full.Schedule(node);
  }

  auto check_metric = [&](ScheduleMetric metric, const char *name,
                          int val_empty, int val_full, bool higher_is_better) {
    bool expect_empty_better = higher_is_better ? (val_empty > val_full)
                                                : (val_empty < val_full);
    bool expect_full_better = higher_is_better ? (val_full > val_empty)
                                               : (val_full < val_empty);
    bool got_empty_better = sc_empty.IsBetterThan(sc_full, metric);
    bool got_full_better = sc_full.IsBetterThan(sc_empty, metric);
    bool pass = (got_empty_better == expect_empty_better) &&
                (got_full_better == expect_full_better);
    llvm::outs() << "    " << name << ": empty=" << val_empty
                 << " full=" << val_full
                 << " empty_better=" << got_empty_better
                 << " full_better=" << got_full_better
                 << (pass ? "  PASS" : "  FAIL") << "\n";
    if (!pass) {
      report_fatal_error("ScheduleMetric plumbing test failed");
    }
  };

  check_metric(
      ScheduleMetric::kRegisterOccupancy, "reg_occ",
      sc_empty.GetPressureTracker().GetRegisterOnlyOccupancy(),
      sc_full.GetPressureTracker().GetRegisterOnlyOccupancy(),
      /*higher_is_better=*/true);
  check_metric(
      ScheduleMetric::kContinuousRegisterOccupancyScore, "cont_occ",
      sc_empty.GetPressureTracker().GetContinuousOccupancyScore(),
      sc_full.GetPressureTracker().GetContinuousOccupancyScore(),
      /*higher_is_better=*/true);
  check_metric(
      ScheduleMetric::kScheduleLength, "length",
      sc_empty.GetLengthTracker().GetCurrentCycle(),
      sc_full.GetLengthTracker().GetCurrentCycle(),
      /*higher_is_better=*/false);

  // --- Part 4: IsAtOccupancyCeiling observability ---
  //
  // Print the pieces so we can see them line up. No pass/fail since
  // whether the region is at the ceiling depends on actual pressure.
  llvm::outs() << "    initial ceiling check: reg_occ="
               << sc_empty.GetPressureTracker().GetRegisterOnlyOccupancy()
               << " fn_limit="
               << sc_empty.GetPressureTracker().GetConfiguredMachineFunctionOccupancyLimit()
               << " at_ceiling=" << sc_empty.IsAtOccupancyCeiling() << "\n";
  llvm::outs() << "    full ceiling check:    reg_occ="
               << sc_full.GetPressureTracker().GetRegisterOnlyOccupancy()
               << " fn_limit="
               << sc_full.GetPressureTracker().GetConfiguredMachineFunctionOccupancyLimit()
               << " at_ceiling=" << sc_full.IsAtOccupancyCeiling() << "\n";
}
