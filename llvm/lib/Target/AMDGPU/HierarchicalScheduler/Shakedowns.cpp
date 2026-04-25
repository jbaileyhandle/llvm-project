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
#include "SubgraphInfo.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <memory>
#include <set>
#include <vector>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Verifies transitive reduction on the test DAG: prints edge count
// before/after and the reduced edges themselves. Print-only.
void CheckTransitiveReduction(ScheduleGraph &graph) {
  int original_edge_count = 0;
  for (const ScheduleNode &node : graph.Nodes()) {
    original_edge_count += node.NumSuccessors();
  }

  graph.ComputeTransitiveReduction();
  const ReducedGraph &reduced = graph.GetReducedGraph();

  int reduced_edge_count = 0;
  for (int topo_idx = 0; topo_idx < reduced.size; ++topo_idx) {
    reduced_edge_count += static_cast<int>(reduced.successors_by_topo_index[topo_idx].size());
  }

  llvm::outs() << "  Transitive reduction: " << original_edge_count
               << " edges -> " << reduced_edge_count << " edges\n";

  llvm::outs() << "  Reduced edges:";
  for (int topo_idx = 0; topo_idx < reduced.size; ++topo_idx) {
    ScheduleNode *from = graph.GetTopoOrder()[topo_idx];
    for (int succ_topo_idx : reduced.successors_by_topo_index[topo_idx]) {
      ScheduleNode *to = graph.GetTopoOrder()[succ_topo_idx];
      llvm::outs() << " " << from->ToString() << "->" << to->ToString();
    }
  }
  llvm::outs() << "\n";
}

// Dumps the computed dominator tree on the test DAG. Print-only.
void CheckDominatorTree(ScheduleGraph &graph) {
  graph.ComputeDominatorTree();
  llvm::outs() << "  Dominator tree:\n" << graph.DominatorTreeToString();
}

// Inner helper: verifies GetLengthLowerBound on one graph against a
// hand-computed expected sequence. Exercises both the forward
// Schedule path (running max maintenance) and the reverse
// Unschedule path (restoration from undo records).
//
// `label` is used only for the PASS/FAIL print line. `expected_lb`
// must have size graph.Size()+1 (one entry per scheduling step,
// starting from the empty state). `expected_cp_length` and
// `expected_graph_length_floor` cross-check the graph-level scalars
// computed during ComputeCriticalPathFromExit.
//
// Requires graph.ComputeCriticalPathFromExit() and
// graph.ComputeTopologicalOrder() to have run already.
void CheckOneLengthLowerBoundRun(ScheduleGraph &graph,
                                 const GCNSubtarget &st,
                                 ArrayRef<int> expected_lb,
                                 int expected_cp_length,
                                 int expected_graph_length_floor,
                                 StringRef label) {
  ScheduleLengthTracker tracker(graph, st);

  if (static_cast<int>(expected_lb.size()) != graph.Size() + 1) {
    llvm::outs() << "  LB " << label << " expected array size mismatch: "
                 << expected_lb.size() << " vs graph.Size()+1="
                 << (graph.Size() + 1) << "  FAIL\n";
    return;
  }

  // Graph-level scalars.
  int got_cp = graph.GetCriticalPathLength();
  int got_floor = graph.GetGraphLengthFloor();
  llvm::outs() << "  Graph scalars on " << label
               << ": cp_length=" << got_cp
               << " (expected " << expected_cp_length << ")"
               << "  length_floor=" << got_floor
               << " (expected " << expected_graph_length_floor << ")  ";
  bool scalars_ok = (got_cp == expected_cp_length) &&
                    (got_floor == expected_graph_length_floor);
  llvm::outs() << (scalars_ok ? "PASS\n" : "FAIL\n");

  // Forward pass.
  int mismatches = 0;
  llvm::outs() << "  LB on " << label << " (forward):";
  int lb = tracker.GetLengthLowerBound();
  llvm::outs() << " " << lb;
  if (lb != expected_lb[0]) {
    ++mismatches;
  }
  for (int i = 0; i < graph.Size(); ++i) {
    tracker.Schedule(graph.GetTopoOrder()[i]);
    lb = tracker.GetLengthLowerBound();
    llvm::outs() << " " << lb;
    if (lb != expected_lb[i + 1]) {
      ++mismatches;
    }
  }
  llvm::outs() << (mismatches == 0 ? "  PASS\n" : "  FAIL\n");

  // Reverse pass: after undoing the k-th forward Schedule, LB
  // should match expected_lb[steps_remaining] = expected_lb[i].
  int reverse_mismatches = 0;
  llvm::outs() << "  LB on " << label << " (reverse):";
  for (int i = graph.Size() - 1; i >= 0; --i) {
    tracker.Unschedule(graph.GetTopoOrder()[i]);
    lb = tracker.GetLengthLowerBound();
    llvm::outs() << " " << lb;
    if (lb != expected_lb[i]) {
      ++reverse_mismatches;
    }
  }
  llvm::outs() << (reverse_mismatches == 0 ? "  PASS\n" : "  FAIL\n");
}


// Verifies critical-path-from-exit against the hand-computed values
// documented in BuildTestDAG's header docstring. Assumes nodes were
// emplaced in order A, C, D, E, F, G, H. PASS/FAIL based on exact
// match at every node.
void CheckCriticalPath(ScheduleGraph &graph) {
  graph.ComputeCriticalPathFromExit();
  struct ExpectedCp {
    const char *name;
    int expected;
  };
  const ExpectedCp expected_cps[] = {
      {"A", 9}, {"C", 7}, {"D", 4}, {"E", 3},
      {"F", 2}, {"G", 0}, {"H", 5},
  };
  int mismatches = 0;
  llvm::outs() << "  Critical path from exit:";
  for (int i = 0, n = graph.Size(); i < n; ++i) {
    const ScheduleNode &node = graph.Nodes()[i];
    int got = graph.GetCriticalPathFromExit(&node);
    int want = expected_cps[i].expected;
    llvm::outs() << " " << expected_cps[i].name << "=" << got;
    if (got != want) {
      llvm::outs() << "(expected " << want << ")";
      ++mismatches;
    }
  }
  llvm::outs() << (mismatches == 0 ? "  PASS\n" : "  FAIL\n");
}

// Exercises graph algorithms on a synthetic test DAG with known structure.
// Delegates each algorithm to a helper in this anonymous namespace.
void RunTestDAGShakedown() {
  auto test_graph = ScheduleGraph::BuildTestDAG();
  test_graph->ValidateAndComputeTopologicalOrder();

  llvm::outs() << "  Test DAG topo order:";
  for (ScheduleNode *node : test_graph->GetTopoOrder()) {
    llvm::outs() << " " << node->ToString();
  }
  llvm::outs() << "\n";

  CheckTransitiveReduction(*test_graph);
  CheckDominatorTree(*test_graph);
  CheckCriticalPath(*test_graph);

  // Cycle detection verified: BuildTestDAGWithCycle() +
  // ValidateAndComputeTopologicalOrder() fires report_fatal_error
  // with graph ToString. Uncomment to re-test:
  // auto cyclic = ScheduleGraph::BuildTestDAGWithCycle();
  // cyclic->ValidateAndComputeTopologicalOrder();
}

// Exercises ScheduleGraph::InsertSubgraphProxies on BuildTestDAG.
// BuildTestDAG layout (7 nodes, indexed in emplacement order):
//   0=A, 1=C, 2=D, 3=E, 4=F, 5=G, 6=H
// Edges: A→H, A→C, A→D, C→D, C→E, D→F, E→F, H→G, F→G.
//
// Subgraph S = {C, D, E, F}. Walking members' edges:
//   - Predecessors of members not in S: A (via A→C, A→D).
//   - Successors of members not in S: G (via F→G).
// Expect: ext_predecessors = [A], ext_successors = [G].
//
// After InsertSubgraphProxies:
//   - One new proxy node, IsSubgraphProxy(), parent_subgraph_proxy
//     == nullptr (top-level), GetSubgraphInfo()->debug_name == "S".
//   - Members C, D, E, F have parent_subgraph_proxy == proxy.
//   - Non-members A, G, H have null parent_subgraph_proxy.
//   - proxy's incoming edge set == {A}, all kSubgraphOrderEdge.
//   - proxy's outgoing edge set == {C, D, E, F}, all
//     kSubgraphOrderEdge.
//   - graph.Size() == 8 (was 7 + 1 proxy).
//   - graph.NumSchedulingUnits() == 7 (proxies excluded).
//   - Topological order is recomputed (cycle-free).
void RunInsertSubgraphProxiesShakedown() {
  auto graph = ScheduleGraph::BuildTestDAG();
  graph->ValidateAndComputeTopologicalOrder();

  // BuildTestDAG emplaces in order [A, C, D, E, F, G, H].
  ScheduleNode *a = &graph->Nodes()[0];
  ScheduleNode *c = &graph->Nodes()[1];
  ScheduleNode *d = &graph->Nodes()[2];
  ScheduleNode *e = &graph->Nodes()[3];
  ScheduleNode *f = &graph->Nodes()[4];
  ScheduleNode *g = &graph->Nodes()[5];
  ScheduleNode *h = &graph->Nodes()[6];

  // Construct the SubgraphInfo. The ctor walks members' edges to
  // populate ext_predecessors / ext_successors.
  SmallVector<ScheduleNode *, 4> members = {c, d, e, f};
  auto info = std::make_unique<SubgraphInfo>(members, "S");

  // ── Verify boundary computation BEFORE insertion ────────────────
  bool boundary_ok = info->ext_predecessors.size() == 1 &&
                     info->ext_predecessors[0] == a &&
                     info->ext_successors.size() == 1 &&
                     info->ext_successors[0] == g;
  llvm::outs() << "  SubgraphInfo boundary: ext_predecessors=["
               << info->ext_predecessors.size() << "] ext_successors=["
               << info->ext_successors.size() << "]  "
               << (boundary_ok ? "PASS" : "FAIL") << "\n";

  // ── Insert ─────────────────────────────────────────────────────
  std::vector<std::unique_ptr<SubgraphInfo>> infos;
  infos.push_back(std::move(info));
  graph->InsertSubgraphProxies(std::move(infos));

  // The proxy is the only IsSubgraphProxy node (and there should
  // be exactly one).
  ScheduleNode *proxy = nullptr;
  int proxy_count = 0;
  for (ScheduleNode &n : graph->Nodes()) {
    if (n.IsSubgraphProxy()) {
      ++proxy_count;
      proxy = &n;
    }
  }

  // ── Verify post-insertion structure ────────────────────────────
  bool size_ok =
      graph->Size() == 8 && graph->NumSchedulingUnits() == 7;
  llvm::outs() << "  Graph size: total=" << graph->Size()
               << " scheduling_units=" << graph->NumSchedulingUnits()
               << "  " << (size_ok ? "PASS" : "FAIL") << "\n";

  bool proxy_basic_ok =
      proxy_count == 1 && proxy != nullptr &&
      proxy->IsSubgraphProxy() &&
      proxy->GetParentSubgraphProxy() == nullptr &&
      proxy->GetSubgraphInfo()->debug_name == "S" &&
      proxy->GetSubgraphInfo()->subgraph_proxy == proxy;
  llvm::outs() << "  Proxy node: count=" << proxy_count
               << " name=\""
               << (proxy ? proxy->GetSubgraphInfo()->debug_name : "")
               << "\"  " << (proxy_basic_ok ? "PASS" : "FAIL") << "\n";

  bool member_parents_ok =
      c->GetParentSubgraphProxy() == proxy &&
      d->GetParentSubgraphProxy() == proxy &&
      e->GetParentSubgraphProxy() == proxy &&
      f->GetParentSubgraphProxy() == proxy;
  bool nonmember_parents_ok =
      a->GetParentSubgraphProxy() == nullptr &&
      g->GetParentSubgraphProxy() == nullptr &&
      h->GetParentSubgraphProxy() == nullptr;
  llvm::outs() << "  parent_subgraph_proxy: members="
               << (member_parents_ok ? "set" : "WRONG")
               << " non-members="
               << (nonmember_parents_ok ? "null" : "WRONG") << "  "
               << ((member_parents_ok && nonmember_parents_ok)
                       ? "PASS"
                       : "FAIL")
               << "\n";

  // Verify proxy's incoming edges: set == {A}, all
  // kSubgraphOrderEdge. The set-equality check catches missing or
  // extra targets; the NumPredecessors() == set size check catches
  // duplicate edges to the same target (they'd collapse to one set
  // entry but inflate NumPredecessors).
  std::set<ScheduleNode *> expected_preds = {a};
  std::set<ScheduleNode *> actual_preds;
  bool pred_kinds_ok = true;
  for (const ScheduleEdge &edge : proxy->Predecessors()) {
    if (edge.kind_ != ScheduleEdge::kSubgraphOrderEdge) {
      pred_kinds_ok = false;
    }
    actual_preds.insert(edge.node_);
  }
  bool preds_ok = pred_kinds_ok && actual_preds == expected_preds &&
                  proxy->NumPredecessors() ==
                      static_cast<int>(expected_preds.size());
  llvm::outs() << "  proxy predecessors: count="
               << proxy->NumPredecessors() << " expected_set={A}  "
               << (preds_ok ? "PASS" : "FAIL") << "\n";

  // Verify proxy's outgoing edges: set == {C,D,E,F}, all
  // kSubgraphOrderEdge.
  std::set<ScheduleNode *> expected_succs = {c, d, e, f};
  std::set<ScheduleNode *> actual_succs;
  bool succ_kinds_ok = true;
  for (const ScheduleEdge &edge : proxy->Successors()) {
    if (edge.kind_ != ScheduleEdge::kSubgraphOrderEdge) {
      succ_kinds_ok = false;
    }
    actual_succs.insert(edge.node_);
  }
  bool succs_ok = succ_kinds_ok && actual_succs == expected_succs &&
                  proxy->NumSuccessors() ==
                      static_cast<int>(expected_succs.size());
  llvm::outs() << "  proxy successors: count="
               << proxy->NumSuccessors()
               << " expected_set={C,D,E,F}  "
               << (succs_ok ? "PASS" : "FAIL") << "\n";

  // Topological order is recomputed by InsertSubgraphProxies.
  bool topo_ok = graph->IsTopoSorted();
  llvm::outs() << "  Topological order recomputed: "
               << (topo_ok ? "yes" : "no") << "  "
               << (topo_ok ? "PASS" : "FAIL") << "\n";
}

// Verifies ScheduleLengthTracker::GetLengthLowerBound against hand-
// computed expected sequences on two synthetic DAGs. Builds both
// graphs internally — this shakedown is self-contained and does not
// depend on any region-level graph.
void RunLengthLowerBoundShakedown(const GCNSubtarget &st) {
  // Primary test DAG (BuildTestDAG): topo order [A, H, C, D, E, F, G],
  // latencies per BuildTestDAG's header, giving expected LB sequence
  // {7, 10, 10, 10, 10, 10, 10, 10}. Single 7->10 transition at the
  // first Schedule (A's contribution 0+9+1=10 dominates everything
  // afterward; final length is also 10).
  // Graph scalars: cp_length=9 (cp[A]), length_floor=max(7, 10)=10.
  {
    auto graph = ScheduleGraph::BuildTestDAG();
    graph->ValidateAndComputeTopologicalOrder();
    graph->ComputeCriticalPathFromExit();
    const int expected_lb[] = {7, 10, 10, 10, 10, 10, 10, 10};
    CheckOneLengthLowerBoundRun(*graph, st, expected_lb,
                                /*expected_cp_length=*/9,
                                /*expected_graph_length_floor=*/10,
                                "BuildTestDAG");
  }

  // Dedicated-for-LB DAG: chosen so the LB transitions multiple
  // times via both terms of the formula. Topo order [N0..N5],
  // expected sequence {6, 10, 10, 10, 11, 12, 12}. Three transitions:
  // 6->10 (2nd term kicks in via N0), 10->11 (1st term overtakes due
  // to bubble at N3), 11->12 (2nd term jumps via N4).
  // Graph scalars: cp_length=9 (cp[N0]), length_floor=max(6, 10)=10.
  // (Floor is loose vs final length 12 because graph isn't a chain.)
  {
    auto graph = ScheduleGraph::BuildLengthLowerBoundTestDAG();
    graph->ValidateAndComputeTopologicalOrder();
    graph->ComputeCriticalPathFromExit();
    const int expected_lb[] = {6, 10, 10, 10, 11, 12, 12};
    CheckOneLengthLowerBoundRun(*graph, st, expected_lb,
                                /*expected_cp_length=*/9,
                                /*expected_graph_length_floor=*/10,
                                "BuildLengthLowerBoundTestDAG");
  }
}

// Tests RegisterTracker by scheduling the first region's instructions in
// topo order and printing pressure at each step.
void RunRegisterTrackerShakedown(ScheduleGraph &graph,
                                 const MachineFunction &mf) {
  SmallVector<ScheduleNode *> nodes(graph.GetTopoOrder().begin(),
                                    graph.GetTopoOrder().end());
  RegisterTracker tracker(nodes, mf.getRegInfo(),
                          *mf.getSubtarget().getRegisterInfo());

  llvm::outs() << "  Register pressure trace (topo order):\n";
  for (ScheduleNode *node : graph.GetTopoOrder()) {
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
void RunGCNRegisterTrackerShakedown(ScheduleGraph &graph,
                                    const MachineFunction &mf,
                                    const LiveIntervals &lis) {
  SmallVector<ScheduleNode *> nodes(graph.GetTopoOrder().begin(),
                                    graph.GetTopoOrder().end());
  GCNRegisterTracker tracker(graph, mf, lis);

  // --- Forward pass: schedule in topo order ---
  llvm::outs() << "  GCN register pressure trace (topo order):\n";
  for (ScheduleNode *node : graph.GetTopoOrder()) {
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
void VerifyGCNRegisterTracker(ScheduleGraph &graph,
                              ArrayRef<ScheduleNode *> order,
                              const MachineFunction &mf,
                              const LiveIntervals &lis) {
  const MachineRegisterInfo &mri = mf.getRegInfo();

  // --- Collect MachineInstrs from the order, skipping entry/exit ---
  SmallVector<MachineInstr *, 32> mis;
  // Map from MachineInstr* to its index in order for labeling.
  DenseMap<MachineInstr *, ScheduleNode *> mi_to_node;
  for (ScheduleNode *node : order) {
    if (!node->IsSchedulingUnit()) {
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
  GCNUpwardRPTracker llvm_tracker(lis);
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
  GCNRegisterTracker tracker(graph, mf, lis);

  llvm::outs() << "  Cross-check per-instruction (same order):\n";
  llvm::outs() << "    " << std::string(60, '-') << "\n";

  int mi_idx = 0;
  GCNRegPressure our_peak;
  GCNRegPressure llvm_peak;
  for (ScheduleNode *node : order) {
    tracker.Schedule(node);

    SUnit *su = node->IsSchedulingUnit() ? node->GetSUnit() : nullptr;
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
// Also exercises GetLengthLowerBound, both its monotonicity through Schedule
// and its correct restoration through Unschedule.
void RunScheduleLengthTrackerShakedown(ScheduleGraph &graph,
                                       const GCNSubtarget &st) {
  ScheduleLengthTracker tracker(graph, st);

  // lb_after[i] = GetLengthLowerBound() after i nodes have been
  // scheduled. Index 0 = empty state; index N = fully scheduled.
  std::vector<int> lb_after;
  lb_after.push_back(tracker.GetLengthLowerBound());

  // --- Forward pass: schedule in topo order ---
  llvm::outs() << "  Schedule length trace (topo order):\n";
  for (ScheduleNode *node : graph.GetTopoOrder()) {
    // Print the instruction.
    llvm::outs() << "    " << node->ToString() << "\n";

    // Print latency-carrying predecessors and their edge latencies.
    for (const ScheduleEdge &edge : node->Predecessors()) {
      if (!edge.IsLatencyEdge()) {
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
    int lb = tracker.GetLengthLowerBound();
    lb_after.push_back(lb);
    llvm::outs() << "      -> cycle=" << tracker.GetScheduledCycle(node)
                 << "  " << tracker.Describe() << "  lb=" << lb << "\n";
  }

  // --- Forward LB invariants ---
  // (a) LB at empty state == NumSchedulingUnits (current_cycle=0, no
  //     contribution from scheduled set).
  // (b) LB at fully scheduled state == final length (current_cycle_),
  //     i.e., tight bound at completion.
  // (c) LB is monotonically non-decreasing along the forward pass
  //     (first term is non-decreasing by IssueWidth=1; second term is
  //     a running max).
  int forward_violations = 0;
  if (lb_after.front() != graph.NumSchedulingUnits()) {
    llvm::outs() << "  LB-at-empty mismatch: got " << lb_after.front()
                 << ", expected " << graph.NumSchedulingUnits() << "\n";
    ++forward_violations;
  }
  if (lb_after.back() != tracker.GetCurrentCycle()) {
    llvm::outs() << "  LB-at-fully-scheduled mismatch: got "
                 << lb_after.back() << ", expected "
                 << tracker.GetCurrentCycle() << "\n";
    ++forward_violations;
  }
  for (int i = 1; i < static_cast<int>(lb_after.size()); ++i) {
    if (lb_after[i] < lb_after[i - 1]) {
      llvm::outs() << "  LB monotonicity violated at step " << i << ": "
                   << lb_after[i - 1] << " -> " << lb_after[i] << "\n";
      ++forward_violations;
    }
  }
  llvm::outs() << "  LB forward invariants:"
               << (forward_violations == 0 ? "  PASS\n" : "  FAIL\n");

  // --- Reverse pass: unschedule everything ---
  // After unscheduling node i (in reverse topo order), the tracker
  // state should equal the state just before that node was scheduled
  // forward, so LB should equal lb_after[steps_remaining].
  llvm::outs() << "  Schedule length trace (unschedule):\n";
  int reverse_violations = 0;
  for (int i = static_cast<int>(graph.GetTopoOrder().size()) - 1; i >= 0; --i) {
    tracker.Unschedule(graph.GetTopoOrder()[i]);
    int lb = tracker.GetLengthLowerBound();
    int expected = lb_after[i];
    llvm::outs() << "    undo  " << tracker.Describe()
                 << "  lb=" << lb << " (expected " << expected << ")\n";
    if (lb != expected) {
      ++reverse_violations;
    }
  }
  llvm::outs() << "  LB reverse (Unschedule restoration):"
               << (reverse_violations == 0 ? "  PASS\n" : "  FAIL\n");

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
void RunScheduleConstructorShakedown(ScheduleGraph &graph,
                                     const MachineFunction &mf,
                                     const LiveIntervals &lis) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(mf.getSubtarget());
  ScheduleConstructor sc(graph, st, mf, lis);

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
  ScheduleConstructor sc2(graph, st, mf, lis);
  for (ScheduleNode *node : graph.GetTopoOrder()) {
    sc2.Schedule(node);
  }
  llvm::outs() << "  ScheduleConstructor (topo order): "
               << sc2.Describe() << "\n";
}

void RunScheduleMetricShakedown(ScheduleGraph &graph,
                                const MachineFunction &mf,
                                const LiveIntervals &lis) {
  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(mf.getSubtarget());
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
  ScheduleConstructor sc_empty(graph, st, mf, lis);
  ScheduleConstructor sc_full(graph, st, mf, lis);
  for (ScheduleNode *node : graph.GetTopoOrder()) {
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
      ScheduleMetric::kMaximizeRegisterOccupancy, "reg_occ",
      sc_empty.GetPressureTracker().GetRegisterOnlyOccupancy(),
      sc_full.GetPressureTracker().GetRegisterOnlyOccupancy(),
      /*higher_is_better=*/true);
  check_metric(
      ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore, "cont_occ",
      sc_empty.GetPressureTracker().GetContinuousOccupancyScore(),
      sc_full.GetPressureTracker().GetContinuousOccupancyScore(),
      /*higher_is_better=*/true);
  check_metric(
      ScheduleMetric::kMinimizeScheduleLength, "length",
      sc_empty.GetLengthTracker().GetCurrentCycle(),
      sc_full.GetLengthTracker().GetCurrentCycle(),
      /*higher_is_better=*/false);

  // --- Part 4: IsAtOrAboveFunctionOccupancyCeiling observability ---
  //
  // Print the pieces so we can see them line up. No pass/fail since
  // whether the region is at the ceiling depends on actual pressure.
  llvm::outs() << "    initial ceiling check: reg_occ="
               << sc_empty.GetPressureTracker().GetRegisterOnlyOccupancy()
               << " fn_limit="
               << sc_empty.GetPressureTracker().GetConfiguredMachineFunctionOccupancyLimit()
               << " at_ceiling=" << sc_empty.IsAtOrAboveFunctionOccupancyCeiling() << "\n";
  llvm::outs() << "    full ceiling check:    reg_occ="
               << sc_full.GetPressureTracker().GetRegisterOnlyOccupancy()
               << " fn_limit="
               << sc_full.GetPressureTracker().GetConfiguredMachineFunctionOccupancyLimit()
               << " at_ceiling=" << sc_full.IsAtOrAboveFunctionOccupancyCeiling() << "\n";
}

// Sweep every entry of the precomputed continuous-occupancy-score
// lookup tables and verify each value matches what
// ComputeContinuousOccupancyScore returns for the corresponding
// register count. Compares vgpr_score_by_count[v] against
// ComputeContinuousOccupancyScore(st, v, 0) for all v in range,
// and similarly sgpr_score_by_count[s] against (st, 0, s).
void RunContinuousScoreTableSweepShakedown(const GCNSubtarget &st) {
  const auto &score_tables =
      GCNRegisterTracker::GetOrComputeContinuousOccupancyScoreTables(st);

  llvm::outs() << "  Continuous score table sweep:\n";

  int vgpr_mismatches = 0;
  for (size_t v = 0; v < GCNRegisterTracker::kContinuousScoreVGPRTableSize;
       ++v) {
    int formula = GCNRegisterTracker::ComputeContinuousOccupancyScore(
        st, /*num_vgpr=*/v, /*num_sgpr=*/0);
    int lookup = score_tables.vgpr_score_by_count[v];
    if (formula != lookup) {
      llvm::outs() << "    VGPR mismatch at vgpr=" << v
                   << ": formula=" << formula << " lookup=" << lookup << "\n";
      ++vgpr_mismatches;
    }
  }

  int sgpr_mismatches = 0;
  for (size_t s = 0; s < GCNRegisterTracker::kContinuousScoreSGPRTableSize;
       ++s) {
    int formula = GCNRegisterTracker::ComputeContinuousOccupancyScore(
        st, /*num_vgpr=*/0, /*num_sgpr=*/s);
    int lookup = score_tables.sgpr_score_by_count[s];
    if (formula != lookup) {
      llvm::outs() << "    SGPR mismatch at sgpr=" << s
                   << ": formula=" << formula << " lookup=" << lookup << "\n";
      ++sgpr_mismatches;
    }
  }

  bool pass = (vgpr_mismatches == 0) && (sgpr_mismatches == 0);
  llvm::outs() << "    swept "
               << GCNRegisterTracker::kContinuousScoreVGPRTableSize
               << " VGPR + "
               << GCNRegisterTracker::kContinuousScoreSGPRTableSize
               << " SGPR entries; "
               << vgpr_mismatches << " VGPR mismatches, "
               << sgpr_mismatches << " SGPR mismatches"
               << (pass ? "  PASS" : "  FAIL") << "\n";
  if (!pass) {
    report_fatal_error("ContinuousScoreTableSweep: lookup table disagrees "
                       "with formula on some entries");
  }
}

// Run all per-region shakedowns on one region's graph. Exercises
// register trackers, schedule-length tracker, ScheduleConstructor,
// ScheduleMetric, and prints the region's EntrySU/ExitSU edge info
// from the LLVM DAG.
void RunRegionShakedowns(ScheduleGraph &graph,
                         const MachineFunction &mf,
                         const LiveIntervals &lis,
                         const SUnit &entry_su,
                         const SUnit &exit_su) {
  llvm::outs() << "  Topo order:\n";
  for (ScheduleNode *node : graph.GetTopoOrder()) {
    llvm::outs() << "    " << node->ToString() << "\n";
  }

  RunRegisterTrackerShakedown(graph, mf);
  RunGCNRegisterTrackerShakedown(graph, mf, lis);

  SmallVector<ScheduleNode *> topo_nodes(graph.GetTopoOrder().begin(),
                                         graph.GetTopoOrder().end());
  VerifyGCNRegisterTracker(graph, topo_nodes, mf, lis);

  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(mf.getSubtarget());
  RunScheduleLengthTrackerShakedown(graph, st);
  RunScheduleConstructorShakedown(graph, mf, lis);
  RunScheduleMetricShakedown(graph, mf, lis);

  // Dump EntrySU/ExitSU edges from the LLVM DAG.
  llvm::outs() << "  EntrySU succs (" << entry_su.Succs.size() << "):";
  for (const SDep &dep : entry_su.Succs) {
    llvm::outs() << " SU(" << dep.getSUnit()->NodeNum << ")";
  }
  llvm::outs() << "\n";
  llvm::outs() << "  EntrySU preds (" << entry_su.Preds.size() << "):";
  for (const SDep &dep : entry_su.Preds) {
    llvm::outs() << " SU(" << dep.getSUnit()->NodeNum << ")";
  }
  llvm::outs() << "\n";
  llvm::outs() << "  ExitSU succs (" << exit_su.Succs.size() << "):";
  for (const SDep &dep : exit_su.Succs) {
    llvm::outs() << " SU(" << dep.getSUnit()->NodeNum << ")";
  }
  llvm::outs() << "\n";
  llvm::outs() << "  ExitSU preds (" << exit_su.Preds.size() << "):";
  for (const SDep &dep : exit_su.Preds) {
    llvm::outs() << " SU(" << dep.getSUnit()->NodeNum << ")";
  }
  llvm::outs() << "\n";
}

} // namespace

// The only class-member shakedown entry point. All the per-shakedown
// helpers live in the anonymous namespace above. Orchestrates the
// standalone shakedowns and the per-region batch.
void ScheduleDAGHierarchicalScheduler::RunAllShakedowns() {
  llvm::outs() << "RunAllShakedowns:\n";

  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(MF.getSubtarget());
  RunContinuousScoreTableSweepShakedown(st);
  RunTestDAGShakedown();
  RunInsertSubgraphProxiesShakedown();
  RunLengthLowerBoundShakedown(st);

  for (auto &region : regions_) {
    WithRegionGraph(region, [&](ScheduleGraph &graph) {
      llvm::outs() << "  Region: " << region.GetNumInstrs()
                   << " instrs, graph: " << graph.Size()
                   << " nodes (" << graph.NumSchedulingUnits()
                   << " scheduling units)"
                   << ", topo order size: " << graph.GetTopoOrder().size()
                   << "\n";

      // Run detailed shakedowns on the first region only.
      if (&region == &regions_.front()) {
        RunRegionShakedowns(graph, MF, *LIS, EntrySU, ExitSU);
      }
    });
  }
}
