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
#include "BfsDpSearch.h"
#include "BfsDpSettings.h"
#include "DecomposeAndSchedule.h"
#include "DfsSearch.h"
#include "DominatorTree.h"
#include "GCNRegisterTracker.h"
#include "GCNSubtarget.h"
#include "IlpTracker.h"
#include "LengthHistoryTracker.h"
#include "NodeRegInfo.h"
#include "OccupancyTargetUtil.h"
#include "PartitionDag.h"
#include "PressureHistoryTracker.h"
#include "RegisterTracker.h"
#include "SIMachineFunctionInfo.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "ScheduleLengthTracker.h"
#include "ScheduleSubgraph.h"
#include "ScheduledSetTracker.h"
#include "Score.h"
#include "SearchPolicies.h"
#include "SubgraphFormation.h"
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

  graph.ComputeTransitiveReductionAndReachability();
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
// computed during ComputeCriticalPathFromExit (now invoked via the
// combined ComputeCriticalPaths wrapper).
//
// Requires graph.ComputeCriticalPaths() and
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
  graph.ComputeCriticalPaths();
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

// Verifies ScheduleLengthTracker::SetMaxAcceptableScheduleLength's formula on
// BuildTestDAG. cp_from_exit values from CheckCriticalPath:
//   A=9, C=7, D=4, E=3, F=2, G=0, H=5.
// For target_length L, max_schedule_cycle[i] = L - 1 - cp_from_exit[i].
// Picking L = 12 to keep all expected values non-negative.
void CheckSetMaxAcceptableScheduleLength(ScheduleGraph &graph, const GCNSubtarget &st) {
  ScheduleLengthTracker tracker(graph, st);
  constexpr int kTargetLength = 12;
  tracker.SetMaxAcceptableScheduleLength(kTargetLength);

  if (!tracker.HasMaxAcceptableScheduleLength()) {
    llvm::outs() << "  SetMaxAcceptableScheduleLength formula: HasMaxAcceptableScheduleLength "
                    "returned false after SetMaxAcceptableScheduleLength  FAIL\n";
    return;
  }

  struct ExpectedMax {
    const char *name;
    int expected;
  };
  // L - 1 - cp_from_exit  (L=12)
  const ExpectedMax expected_max[] = {
      {"A", 2}, {"C", 4}, {"D", 7}, {"E", 8},
      {"F", 9}, {"G", 11}, {"H", 6},
  };
  int mismatches = 0;
  llvm::outs() << "  SetMaxAcceptableScheduleLength formula (L=" << kTargetLength
               << "):";
  for (int i = 0, n = graph.Size(); i < n; ++i) {
    const ScheduleNode &node = graph.Nodes()[i];
    int got = tracker.GetMaxScheduleCycle(&node);
    int want = expected_max[i].expected;
    llvm::outs() << " " << expected_max[i].name << "=" << got;
    if (got != want) {
      llvm::outs() << "(expected " << want << ")";
      ++mismatches;
    }
  }
  llvm::outs() << (mismatches == 0 ? "  PASS\n" : "  FAIL\n");
}

// Verifies the MaxScheduleCycleHeap inside ScheduleLengthTracker.
//
// BuildTestDAG cp_from_exit values (per CheckCriticalPath):
//   A=9, C=7, D=4, E=3, F=2, G=0, H=5
// Topo order assigned by Kahn's (per RunTestDAGShakedown's print):
//   A=0, H=1, C=2, D=3, E=4, F=5, G=6
// max_cycle = 12 - 1 - cp_from_exit:
//   A=2, H=6, C=4, D=7, E=8, F=9, G=11
//
// Sorted ascending by (max_cycle, topo_idx):
//   A(2,0), C(4,2), H(6,1), D(7,3), E(8,4), F(9,5), G(11,6)
//
// Tests:
//   1) Pre-Set: heap empty, predicate false.
//   2) After SetMax(12): heap fully populated, top=A(2),
//      predicate false (current_cycle=0).
//   3) Forward drain: Schedule each node in topo order, peeking
//      after each step to verify the heap's top advances to the
//      next-smallest unscheduled entry.
//   4) Reverse drain: Unschedule in stack order, peeking at each
//      step to verify the heap restores entries correctly. After
//      the full reverse, heap matches step (2)'s state.
//   5) Predicate behavior: with the post-(4) state, tighten
//      SetMax(2) so A.max_cycle becomes -8 (negative). With
//      current_cycle still 0, predicate fires (0 > -8).
void CheckMaxScheduleCycleHeap(ScheduleGraph &graph,
                               const GCNSubtarget &st) {
  ScheduleLengthTracker tracker(graph, st);
  const auto &heap = tracker.GetMaxScheduleCycleHeapForTest();

  // 1) Pre-Set: heap empty, predicate false.
  bool pre_ok = heap.Size() == 0 && !heap.Peek().has_value() &&
                !tracker.IsCurrentCycleBeyondEarliestMaxScheduleCycle();
  llvm::outs() << "  Heap pre-Set: size=" << heap.Size()
               << " peek="
               << (heap.Peek().has_value() ? "present" : "nullopt")
               << " predicate="
               << (tracker.IsCurrentCycleBeyondEarliestMaxScheduleCycle()
                       ? "true"
                       : "false")
               << "  " << (pre_ok ? "PASS\n" : "FAIL\n");

  // 2) SetMax(12): heap populated, top=A(2,0), predicate false.
  tracker.SetMaxAcceptableScheduleLength(12);
  auto top_after_set = heap.Peek();
  bool set_ok = heap.Size() == 7 && top_after_set.has_value() &&
                top_after_set->max_cycle == 2 &&
                top_after_set->topo_idx == 0 &&
                !tracker.IsCurrentCycleBeyondEarliestMaxScheduleCycle();
  llvm::outs() << "  Heap after SetMax(12): size=" << heap.Size()
               << " top.max_cycle="
               << (top_after_set.has_value() ? top_after_set->max_cycle : -1)
               << " top.topo_idx="
               << (top_after_set.has_value() ? top_after_set->topo_idx : -1)
               << " predicate="
               << (tracker.IsCurrentCycleBeyondEarliestMaxScheduleCycle()
                       ? "true"
                       : "false")
               << "  " << (set_ok ? "PASS\n" : "FAIL\n");

  // Resolve nodes by topo index for the drain steps.
  const ScheduleNode *nodes_by_topo[7] = {nullptr};
  for (const ScheduleNode &n : graph.Nodes()) {
    if (!n.IsSchedulingUnit()) {
      continue;
    }
    nodes_by_topo[n.GetTopoIndex()] = &n;
  }

  // 3) Forward drain.
  struct ForwardStep {
    int schedule_topo;
    std::optional<MaxScheduleCycleHeap::Entry> expected_top;
    const char *label;
  };
  const ForwardStep forward[] = {
      {0, MaxScheduleCycleHeap::Entry{4, 2}, "Schedule A -> top=C(4,2)"},
      {1, MaxScheduleCycleHeap::Entry{4, 2}, "Schedule H -> top=C(4,2)"},
      {2, MaxScheduleCycleHeap::Entry{7, 3}, "Schedule C -> top=D(7,3)"},
      {3, MaxScheduleCycleHeap::Entry{8, 4}, "Schedule D -> top=E(8,4)"},
      {4, MaxScheduleCycleHeap::Entry{9, 5}, "Schedule E -> top=F(9,5)"},
      {5, MaxScheduleCycleHeap::Entry{11, 6}, "Schedule F -> top=G(11,6)"},
      {6, std::nullopt, "Schedule G -> empty"},
  };
  int forward_fails = 0;
  llvm::outs() << "  Heap forward drain:\n";
  for (const ForwardStep &step : forward) {
    tracker.Schedule(nodes_by_topo[step.schedule_topo]);
    auto got = heap.Peek();
    bool ok = got.has_value() == step.expected_top.has_value() &&
              (!got.has_value() ||
               (got->max_cycle == step.expected_top->max_cycle &&
                got->topo_idx == step.expected_top->topo_idx));
    llvm::outs() << "    " << step.label << ": got "
                 << (got.has_value()
                         ? "(max=" + std::to_string(got->max_cycle) +
                               ",topo=" + std::to_string(got->topo_idx) + ")"
                         : "nullopt")
                 << "  " << (ok ? "PASS\n" : "FAIL\n");
    if (!ok) {
      ++forward_fails;
    }
  }

  // 4) Reverse drain.
  struct ReverseStep {
    int unschedule_topo;
    MaxScheduleCycleHeap::Entry expected_top;
    const char *label;
  };
  const ReverseStep reverse[] = {
      {6, {11, 6}, "Unschedule G -> top=G(11,6)"},
      {5, {9, 5}, "Unschedule F -> top=F(9,5)"},
      {4, {8, 4}, "Unschedule E -> top=E(8,4)"},
      {3, {7, 3}, "Unschedule D -> top=D(7,3)"},
      {2, {4, 2}, "Unschedule C -> top=C(4,2)"},
      {1, {4, 2}, "Unschedule H -> top=C(4,2)"},
      {0, {2, 0}, "Unschedule A -> top=A(2,0)"},
  };
  int reverse_fails = 0;
  llvm::outs() << "  Heap reverse drain:\n";
  for (const ReverseStep &step : reverse) {
    tracker.Unschedule(nodes_by_topo[step.unschedule_topo]);
    auto got = heap.Peek();
    bool ok = got.has_value() &&
              got->max_cycle == step.expected_top.max_cycle &&
              got->topo_idx == step.expected_top.topo_idx;
    llvm::outs() << "    " << step.label << ": got "
                 << (got.has_value()
                         ? "(max=" + std::to_string(got->max_cycle) +
                               ",topo=" + std::to_string(got->topo_idx) + ")"
                         : "nullopt")
                 << "  " << (ok ? "PASS\n" : "FAIL\n");
    if (!ok) {
      ++reverse_fails;
    }
  }

  // 5) Predicate behavior under tightened L. SetMax(2): rebuild
  // with max_cycle = 2 - 1 - cp_from_exit, all negative for any
  // cp_from_exit >= 2 (everything except G). Smallest entry's
  // max_cycle is A's = -8. current_cycle is still 0, so 0 > -8
  // and the predicate fires.
  tracker.SetMaxAcceptableScheduleLength(2);
  bool predicate_tight =
      tracker.IsCurrentCycleBeyondEarliestMaxScheduleCycle();
  llvm::outs() << "  Heap predicate after SetMax(2) (current_cycle=0, "
                  "smallest max_cycle=-8): "
               << (predicate_tight ? "true" : "false") << " (expected true)  "
               << (predicate_tight ? "PASS\n" : "FAIL\n");

  llvm::outs() << "  Heap drain summary: forward_fails=" << forward_fails
               << " reverse_fails=" << reverse_fails << "  "
               << ((forward_fails == 0 && reverse_fails == 0) ? "PASS\n"
                                                              : "FAIL\n");
}

// Verifies critical-path-from-entry against hand-computed values for
// BuildTestDAG. Node iteration order: A, C, D, E, F, G, H.
//
// Edge latencies (from BuildTestDAG header):
//   A->H=1  A->D=3  A->C=2
//   C->D=1  C->E=4
//   D->F=2  E->F=1
//   H->G=5  F->G=2
//
// Forward recurrence (base case A=0):
//   A = 0                                            (entry)
//   C = max(A->C=2 + A=0) = 2
//   D = max(A->D=3 + A=0, C->D=1 + C=2) = max(3,3) = 3
//   E = max(C->E=4 + C=2) = 6
//   F = max(D->F=2 + D=3, E->F=1 + E=6) = max(5,7) = 7
//   H = max(A->H=1 + A=0) = 1
//   G = max(H->G=5 + H=1, F->G=2 + F=7) = max(6,9) = 9
//
// Also asserts the directional sanity:
// cp_from_entry[exit] == cp_from_exit[entry] == CriticalPathLength.
// Both quantities measure the longest latency-weighted path
// through the whole DAG, so they must agree.
void CheckCriticalPathFromEntry(ScheduleGraph &graph) {
  graph.ComputeCriticalPathFromEntry();
  struct ExpectedCp {
    const char *name;
    int expected;
  };
  const ExpectedCp expected_cps[] = {
      {"A", 0}, {"C", 2}, {"D", 3}, {"E", 6},
      {"F", 7}, {"G", 9}, {"H", 1},
  };
  int mismatches = 0;
  llvm::outs() << "  Critical path from entry:";
  for (int i = 0, n = graph.Size(); i < n; ++i) {
    const ScheduleNode &node = graph.Nodes()[i];
    int got = graph.GetCriticalPathFromEntry(&node);
    int want = expected_cps[i].expected;
    llvm::outs() << " " << expected_cps[i].name << "=" << got;
    if (got != want) {
      llvm::outs() << "(expected " << want << ")";
      ++mismatches;
    }
  }
  llvm::outs() << (mismatches == 0 ? "  PASS\n" : "  FAIL\n");

  // Directional sanity: cp_from_entry at the exit node equals
  // cp_from_exit at the entry node equals CriticalPathLength.
  // Topo order in BuildTestDAG places A at index 0 (entry, single
  // root) and G at the last topologically-visited index (single
  // leaf — the only node with no successors). We look them up by
  // searching the node list rather than hard-coding an index, so
  // the check is robust to topo-iteration changes.
  const ScheduleNode *entry = nullptr;
  const ScheduleNode *exit = nullptr;
  for (const ScheduleNode &node : graph.Nodes()) {
    if (node.NumPredecessors() == 0) {
      entry = &node;
    }
    if (node.NumSuccessors() == 0) {
      exit = &node;
    }
  }
  int from_entry_at_exit = graph.GetCriticalPathFromEntry(exit);
  int from_exit_at_entry = graph.GetCriticalPathFromExit(entry);
  int cp_length = graph.GetCriticalPathLength();
  llvm::outs() << "  CP directional equality: cp_from_entry[exit]="
               << from_entry_at_exit
               << " cp_from_exit[entry]=" << from_exit_at_entry
               << " CriticalPathLength=" << cp_length << "  ";
  bool eq_ok = (from_entry_at_exit == from_exit_at_entry) &&
               (from_entry_at_exit == cp_length);
  llvm::outs() << (eq_ok ? "PASS\n" : "FAIL\n");
}

// Verifies min_schedule_cycle initialization, forward propagation,
// and undo on the BuildTestDAG. cp_from_entry values (per
// CheckCriticalPathFromEntry):
//   A=0, C=2, D=3, E=6, F=7, G=9, H=1
// cp_from_exit values (per CheckCriticalPath):
//   A=9, C=7, D=4, E=3, F=2, G=0, H=5
// Topo order: A=0, H=1, C=2, D=3, E=4, F=5, G=6.
//
// Tests:
//   1) Initial state: min == cp_from_entry for every node.
//   2) Bubble-inducing schedule. Order A, C, E, H — H lands at
//      cycle 7 (current_cycle has advanced past H's data-ready
//      cycle of 1 because of E's bubble). H's effective cycle
//      jumps from min[H]=1 to scheduled_cycle=7, so G's
//      contribution from H rises from 1+5=6 to 7+5=12, raising
//      min[G] from 9 to 12. The first three Schedules land at
//      their cp_from_entry values, so propagation finds no
//      raises in those steps.
//   3) Reverse drain: Unschedule in stack order, verify min[G]
//      restores at each step and the full empty state matches
//      the post-construction values.
//   4) Deadline detection: with max_acceptable=10, max[G] = 10 -
//      1 - cp_from_exit[G] = 9. Repeating the bubble schedule
//      raises min[G] to 12 > 9, so
//      IsAnyMinScheduleCycleBeyondMaxScheduleCycle fires after
//      Schedule(H). Unschedule(H) restores it to false.
void CheckMinScheduleCycle(ScheduleGraph &graph,
                           const GCNSubtarget &st) {
  ScheduleLengthTracker tracker(graph, st);

  // Resolve nodes by topo index. Every node in BuildTestDAG is a
  // scheduling unit (no proxies), so the IsSchedulingUnit guard
  // is for symmetry with the heap shakedown only.
  const ScheduleNode *nodes_by_topo[7] = {nullptr};
  for (const ScheduleNode &n : graph.Nodes()) {
    if (!n.IsSchedulingUnit()) {
      continue;
    }
    nodes_by_topo[n.GetTopoIndex()] = &n;
  }

  struct LabeledMin {
    const char *name;
    int topo_idx;
    int expected_min;
  };
  // Indexed by name for legibility; topo_idx for lookup.
  const LabeledMin initial[] = {
      {"A", 0, 0}, {"H", 1, 1}, {"C", 2, 2}, {"D", 3, 3},
      {"E", 4, 6}, {"F", 5, 7}, {"G", 6, 9},
  };

  // 1) Initial state.
  int initial_fails = 0;
  llvm::outs() << "  Initial min_schedule_cycle == cp_from_entry:";
  for (const LabeledMin &e : initial) {
    int got = tracker.GetMinScheduleCycleByTopoIndex(e.topo_idx);
    llvm::outs() << " " << e.name << "=" << got;
    if (got != e.expected_min) {
      llvm::outs() << "(expected " << e.expected_min << ")";
      ++initial_fails;
    }
  }
  llvm::outs() << (initial_fails == 0 ? "  PASS\n" : "  FAIL\n");

  // 2) Bubble-inducing schedule: A, C, E, H. Hand-traced expected
  // min[G] after each Schedule.
  struct ForwardStep {
    int schedule_topo;
    int expected_min_g;
    const char *label;
  };
  const ForwardStep forward[] = {
      {0, 9, "Schedule A -> min[G]=9 (no propagation)"},
      {2, 9, "Schedule C -> min[G]=9 (no propagation)"},
      {4, 9, "Schedule E -> min[G]=9 (no propagation)"},
      {1, 12, "Schedule H -> min[G]=12 (H@7 raises G via H->G=5)"},
  };
  int bubble_fails = 0;
  llvm::outs() << "  Bubble-inducing schedule (A,C,E,H):\n";
  for (const ForwardStep &step : forward) {
    tracker.Schedule(nodes_by_topo[step.schedule_topo]);
    int got = tracker.GetMinScheduleCycleByTopoIndex(/*G=*/6);
    bool ok = (got == step.expected_min_g);
    llvm::outs() << "    " << step.label << ": min[G]=" << got
                 << "  " << (ok ? "PASS\n" : "FAIL\n");
    if (!ok) {
      ++bubble_fails;
    }
  }

  // 3) Reverse drain. Unschedule in LIFO order; verify min[G]
  // restores along the way and final state matches initial.
  struct ReverseStep {
    int unschedule_topo;
    int expected_min_g;
    const char *label;
  };
  const ReverseStep reverse[] = {
      {1, 9, "Unschedule H -> min[G]=9 (restored)"},
      {4, 9, "Unschedule E -> min[G]=9"},
      {2, 9, "Unschedule C -> min[G]=9"},
      {0, 9, "Unschedule A -> min[G]=9"},
  };
  int undo_fails = 0;
  llvm::outs() << "  Reverse drain restores min:\n";
  for (const ReverseStep &step : reverse) {
    tracker.Unschedule(nodes_by_topo[step.unschedule_topo]);
    int got = tracker.GetMinScheduleCycleByTopoIndex(/*G=*/6);
    bool ok = (got == step.expected_min_g);
    llvm::outs() << "    " << step.label << ": min[G]=" << got
                 << "  " << (ok ? "PASS\n" : "FAIL\n");
    if (!ok) {
      ++undo_fails;
    }
  }
  // Full restoration: after the full undo, every node's min
  // should match cp_from_entry again.
  int restore_fails = 0;
  for (const LabeledMin &e : initial) {
    int got = tracker.GetMinScheduleCycleByTopoIndex(e.topo_idx);
    if (got != e.expected_min) {
      ++restore_fails;
    }
  }
  llvm::outs() << "    Full undo: min == cp_from_entry  "
               << (restore_fails == 0 ? "PASS\n" : "FAIL\n");

  // 4) Deadline detection. After SetMax(10), max[G] = 9; the
  // bubble schedule raises min[G] to 12 > 9, firing the bool.
  // Unschedule(H) restores it from the saved prior.
  struct DeadlineStep {
    enum Kind { kSchedule, kUnschedule } kind;
    int topo_idx;
    bool expected_bool;
    const char *label;
  };
  const DeadlineStep deadline[] = {
      {DeadlineStep::kSchedule, 0, false,
       "Schedule A -> bool=false"},
      {DeadlineStep::kSchedule, 2, false,
       "Schedule C -> bool=false"},
      {DeadlineStep::kSchedule, 4, false,
       "Schedule E -> bool=false"},
      {DeadlineStep::kSchedule, 1, true,
       "Schedule H -> bool=true (min[G]=12 > max[G]=9)"},
      {DeadlineStep::kUnschedule, 1, false,
       "Unschedule H -> bool=false (restored)"},
  };
  int deadline_fails = 0;
  llvm::outs() << "  Deadline detection (max_acceptable=10):\n";
  tracker.SetMaxAcceptableScheduleLength(10);
  if (tracker.IsAnyMinScheduleCycleBeyondMaxScheduleCycle()) {
    llvm::outs() << "    SetMax(10) on empty: bool=true  FAIL\n";
    ++deadline_fails;
  } else {
    llvm::outs() << "    SetMax(10) on empty: bool=false  PASS\n";
  }
  for (const DeadlineStep &step : deadline) {
    if (step.kind == DeadlineStep::kSchedule) {
      tracker.Schedule(nodes_by_topo[step.topo_idx]);
    } else {
      tracker.Unschedule(nodes_by_topo[step.topo_idx]);
    }
    bool got = tracker.IsAnyMinScheduleCycleBeyondMaxScheduleCycle();
    bool ok = (got == step.expected_bool);
    llvm::outs() << "    " << step.label << ": bool="
                 << (got ? "true" : "false") << "  "
                 << (ok ? "PASS\n" : "FAIL\n");
    if (!ok) {
      ++deadline_fails;
    }
  }
  // Drain back so the tracker is in a clean state if any caller
  // reuses it (defensive — current callers don't, but the heap
  // shakedown ends with an empty stack and we mirror that).
  tracker.Unschedule(nodes_by_topo[4]);
  tracker.Unschedule(nodes_by_topo[2]);
  tracker.Unschedule(nodes_by_topo[0]);

  llvm::outs() << "  Min schedule cycle summary: initial_fails="
               << initial_fails << " bubble_fails=" << bubble_fails
               << " undo_fails=" << undo_fails
               << " restore_fails=" << restore_fails
               << " deadline_fails=" << deadline_fails << "  "
               << ((initial_fails + bubble_fails + undo_fails +
                    restore_fails + deadline_fails) == 0
                       ? "PASS\n"
                       : "FAIL\n");
}

// Exercises graph algorithms on a synthetic test DAG with known structure.
// Delegates each algorithm to a helper in this anonymous namespace.
void RunTestDAGShakedown(const GCNSubtarget &st) {
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
  CheckCriticalPathFromEntry(*test_graph);
  CheckSetMaxAcceptableScheduleLength(*test_graph, st);
  CheckMaxScheduleCycleHeap(*test_graph, st);
  CheckMinScheduleCycle(*test_graph, st);

  // Cycle detection verified: BuildTestDAGWithCycle() +
  // ValidateAndComputeTopologicalOrder() fires report_fatal_error
  // with graph ToString. Uncomment to re-test:
  // auto cyclic = ScheduleGraph::BuildTestDAGWithCycle();
  // cyclic->ValidateAndComputeTopologicalOrder();
}

// Phase 1 of subgraph formation: exercises IsSubgraphSplitter, the
// retained reachability matrix on ScheduleGraph, and the GetIDom
// node-pointer overload on DominatorTree. Uses BuildTestDAG, whose
// edges/latencies are known statically:
//   A→H=1  A→C=2  A→D=3
//   C→D=1  C→E=4
//   D→F=2  E→F=1
//   H→G=5  F→G=2
//
// Splitter status (max outgoing latency-edge per node):
//   A=3, C=4, D=2, E=1, F=2, G=0 (no outgoing), H=5
// Predicate fires iff max > threshold:
//   threshold=4 → only H is a splitter.
//   threshold=3 → C and H.
//   threshold=2 → A, C, H.
//
// Reachability (reflexive + transitive closure of the directed graph):
//   A reaches everyone (A,C,D,E,F,G,H).
//   C reaches C,D,E,F,G.
//   D reaches D,F,G.
//   E reaches E,F,G.
//   F reaches F,G.
//   G reaches only G.
//   H reaches H,G.
void RunSubgraphFormationPhase1Shakedown() {
  llvm::outs() << "  RunSubgraphFormationPhase1Shakedown:\n";
  auto graph = ScheduleGraph::BuildTestDAG();
  graph->ValidateAndComputeTopologicalOrder();
  graph->ComputeTransitiveReductionAndReachability();
  graph->ComputeDominatorTree();

  // BuildTestDAG emplacement order: [A, C, D, E, F, G, H]
  ScheduleNode *a = &graph->Nodes()[0];
  ScheduleNode *c = &graph->Nodes()[1];
  ScheduleNode *d = &graph->Nodes()[2];
  ScheduleNode *e = &graph->Nodes()[3];
  ScheduleNode *f = &graph->Nodes()[4];
  ScheduleNode *g = &graph->Nodes()[5];
  ScheduleNode *h = &graph->Nodes()[6];

  // 1) IsSubgraphSplitter at three thresholds.
  struct SplitterCase {
    int threshold;
    // Bits in node order [A, C, D, E, F, G, H].
    bool expected[7];
  };
  const SplitterCase splitter_cases[] = {
      {4, {false, false, false, false, false, false, true}},
      {3, {false, true,  false, false, false, false, true}},
      {2, {true,  true,  false, false, false, false, true}},
  };
  ScheduleNode *nodes[] = {a, c, d, e, f, g, h};
  const char *names[] = {"A", "C", "D", "E", "F", "G", "H"};
  for (const SplitterCase &sc : splitter_cases) {
    int mismatches = 0;
    llvm::outs() << "    Splitter status (threshold=" << sc.threshold << "):";
    for (int i = 0; i < 7; ++i) {
      bool got = IsSubgraphSplitter(nodes[i], sc.threshold);
      llvm::outs() << " " << names[i] << "=" << (got ? "T" : "F");
      if (got != sc.expected[i]) {
        ++mismatches;
      }
    }
    llvm::outs() << (mismatches == 0 ? "  PASS\n" : "  FAIL\n");
  }

  // 2) IsReachableInDag for one row per source — every node tested as
  // target. The reachability matrix is reflexive + transitive over
  // the directed graph (computed and retained inside
  // ComputeTransitiveReductionAndReachability).
  struct ReachCase {
    ScheduleNode *from;
    const char *from_name;
    // Expected reachability to [A, C, D, E, F, G, H] in that order.
    bool expected[7];
  };
  const ReachCase reach_cases[] = {
      {a, "A", {true,  true,  true,  true,  true,  true,  true }},
      {c, "C", {false, true,  true,  true,  true,  true,  false}},
      {d, "D", {false, false, true,  false, true,  true,  false}},
      {e, "E", {false, false, false, true,  true,  true,  false}},
      {f, "F", {false, false, false, false, true,  true,  false}},
      {g, "G", {false, false, false, false, false, true,  false}},
      {h, "H", {false, false, false, false, false, true,  true }},
  };
  for (const ReachCase &rc : reach_cases) {
    int mismatches = 0;
    llvm::outs() << "    Reachable from " << rc.from_name << ":";
    for (int i = 0; i < 7; ++i) {
      bool got = graph->IsReachableInDag(rc.from, nodes[i]);
      if (got) {
        llvm::outs() << " " << names[i];
      }
      if (got != rc.expected[i]) {
        ++mismatches;
      }
    }
    llvm::outs() << (mismatches == 0 ? "  PASS\n" : "  FAIL\n");
  }

  // 3) GetIDom node-pointer overload agrees with the topo-index form
  // for every node.
  const DominatorTree &dom = graph->GetDominatorTree();
  int dom_mismatches = 0;
  for (int i = 0; i < 7; ++i) {
    int by_node = dom.GetIDom(nodes[i]);
    int by_idx = dom.GetIDomByTopoIndex(nodes[i]->GetTopoIndex());
    if (by_node != by_idx) {
      ++dom_mismatches;
    }
  }
  llvm::outs() << "    DominatorTree::GetIDom(node) matches "
               << "GetIDomByTopoIndex(): "
               << (dom_mismatches == 0 ? "PASS\n" : "FAIL\n");

  // 4) Reachability cache lifecycle: a graph mutation
  // (InsertSubgraphProxies) routes through InvalidateDerivedData,
  // which must clear the matrix. Re-running ComputeTransitiveReductionAndReachability
  // must repopulate it.
  bool initially_present = graph->HasReachability();
  llvm::outs() << "    HasReachability after build: "
               << (initially_present ? "PASS\n" : "FAIL\n");

  // Add one trivial subgraph to force invalidation (any AddEdge call
  // would also work, but InsertSubgraphProxies is the realistic
  // mutation here).
  std::vector<std::unique_ptr<SubgraphInfo>> infos;
  std::vector<ScheduleNode *> members = {c, d, e, f};
  infos.push_back(std::make_unique<SubgraphInfo>(members, "S"));
  graph->InsertSubgraphProxies(std::move(infos));
  bool cleared = !graph->HasReachability();
  llvm::outs() << "    HasReachability cleared after mutation: "
               << (cleared ? "PASS\n" : "FAIL\n");

  graph->ComputeTransitiveReductionAndReachability();
  bool repopulated = graph->HasReachability();
  llvm::outs() << "    HasReachability repopulated after recompute: "
               << (repopulated ? "PASS\n" : "FAIL\n");
}

// Phase 2 of subgraph formation: exercises SubgraphFormationTree on
// BuildTestDAG. Uses the dom tree dumped by RunTestDAGShakedown:
//   A is the root.
//     A's children: H, C, G.
//     C's children: D, E, F.
//   (idom mapping: H,C,G→A; D,E,F→C; A→ROOT.)
//
// At splitter threshold 4, only H is a splitter (max outgoing edge
// H→G=5; every other max ≤ 4). So:
//   - subtree_node_count: A=7, C=4, others=1
//   - subtree_splitter_count: A=1, H=1, others=0
//
// IsInSubtreeOf semantics tested across ancestor / sibling / self
// triples — cheaper than re-deriving expected dfs_pre/dfs_post,
// which depend on Kahn's FIFO order.
//
// RecordEmission test:
//   - Mark E. Then E.emitted, C.descendants_emitted,
//     A.descendants_emitted; D / F / H / G untouched.
//   - Re-mark E (idempotency). EmitPoints unchanged.
//   - Mark D. D.emitted, A.descendants_emitted (already true, walk
//     stops). EmitPoints == [E, D] (insertion order).
void RunSubgraphFormationPhase2Shakedown() {
  llvm::outs() << "  RunSubgraphFormationPhase2Shakedown:\n";
  auto graph = ScheduleGraph::BuildTestDAG();
  graph->ValidateAndComputeTopologicalOrder();
  graph->ComputeTransitiveReductionAndReachability();
  graph->ComputeDominatorTree();

  // Build with threshold 4 → only H is a splitter.
  auto is_splitter = [](const ScheduleNode *n) {
    return IsSubgraphSplitter(n, /*latency_threshold=*/4);
  };
  SubgraphFormationTree tree =
      SubgraphFormationTree::BuildFromDominatorTree(
          *graph, graph->GetDominatorTree(), is_splitter);

  // BuildTestDAG emplaces in order [A, C, D, E, F, G, H].
  ScheduleNode *a = &graph->Nodes()[0];
  ScheduleNode *c = &graph->Nodes()[1];
  ScheduleNode *d = &graph->Nodes()[2];
  ScheduleNode *e = &graph->Nodes()[3];
  ScheduleNode *f = &graph->Nodes()[4];
  ScheduleNode *g = &graph->Nodes()[5];
  ScheduleNode *h = &graph->Nodes()[6];

  SubgraphFormationTreeNode *tA = tree.GetNode(a);
  SubgraphFormationTreeNode *tC = tree.GetNode(c);
  SubgraphFormationTreeNode *tD = tree.GetNode(d);
  SubgraphFormationTreeNode *tE = tree.GetNode(e);
  SubgraphFormationTreeNode *tF = tree.GetNode(f);
  SubgraphFormationTreeNode *tG = tree.GetNode(g);
  SubgraphFormationTreeNode *tH = tree.GetNode(h);

  // 1) Tree shape: parent and child counts.
  bool root_ok = tree.Root() == tA && tA->parent == nullptr;
  llvm::outs() << "    Root is A with no parent: "
               << (root_ok ? "PASS\n" : "FAIL\n");
  bool a_kids = static_cast<int>(tA->children.size()) == 3;
  bool c_kids = static_cast<int>(tC->children.size()) == 3;
  bool h_kids = tH->children.empty() && tD->children.empty() &&
                tE->children.empty() && tF->children.empty() &&
                tG->children.empty();
  llvm::outs() << "    Children counts (A=3 C=3 leaves=0): "
               << (a_kids && c_kids && h_kids ? "PASS\n" : "FAIL\n");

  bool parents_ok = tH->parent == tA && tC->parent == tA &&
                    tG->parent == tA && tD->parent == tC &&
                    tE->parent == tC && tF->parent == tC;
  llvm::outs() << "    Parent links match dom tree: "
               << (parents_ok ? "PASS\n" : "FAIL\n");

  // 2) Subtree counts.
  bool counts_ok =
      tA->subtree_node_count == 7 && tA->subtree_splitter_count == 1 &&
      tC->subtree_node_count == 4 && tC->subtree_splitter_count == 0 &&
      tH->subtree_node_count == 1 && tH->subtree_splitter_count == 1 &&
      tD->subtree_node_count == 1 && tD->subtree_splitter_count == 0 &&
      tE->subtree_node_count == 1 && tE->subtree_splitter_count == 0 &&
      tF->subtree_node_count == 1 && tF->subtree_splitter_count == 0 &&
      tG->subtree_node_count == 1 && tG->subtree_splitter_count == 0;
  llvm::outs() << "    Subtree counts: "
               << (counts_ok ? "PASS\n" : "FAIL\n");

  // 3) is_splitter cached on tree nodes.
  bool splitter_cached_ok = !tA->is_splitter && !tC->is_splitter &&
                            !tD->is_splitter && !tE->is_splitter &&
                            !tF->is_splitter && !tG->is_splitter &&
                            tH->is_splitter;
  llvm::outs() << "    is_splitter cached on tree nodes (only H): "
               << (splitter_cached_ok ? "PASS\n" : "FAIL\n");

  // 4) IsInSubtreeOf semantics. Self-inclusive, transitive, asymmetric.
  bool subtree_ok =
      // Self.
      SubgraphFormationTree::IsInSubtreeOf(tA, tA) &&
      SubgraphFormationTree::IsInSubtreeOf(tE, tE) &&
      // Direct child.
      SubgraphFormationTree::IsInSubtreeOf(tH, tA) &&
      SubgraphFormationTree::IsInSubtreeOf(tE, tC) &&
      // Grandchild.
      SubgraphFormationTree::IsInSubtreeOf(tD, tA) &&
      SubgraphFormationTree::IsInSubtreeOf(tF, tA) &&
      // Siblings — neither contains the other.
      !SubgraphFormationTree::IsInSubtreeOf(tH, tC) &&
      !SubgraphFormationTree::IsInSubtreeOf(tC, tH) &&
      !SubgraphFormationTree::IsInSubtreeOf(tD, tE) &&
      !SubgraphFormationTree::IsInSubtreeOf(tE, tD) &&
      // Descendant->ancestor: false (ancestor is not in descendant's
      // subtree).
      !SubgraphFormationTree::IsInSubtreeOf(tA, tE) &&
      !SubgraphFormationTree::IsInSubtreeOf(tC, tE);
  llvm::outs() << "    IsInSubtreeOf semantics: "
               << (subtree_ok ? "PASS\n" : "FAIL\n");

  // 5) RecordEmission propagation + EmitPoints idempotency.
  tree.RecordEmission(tE);
  bool after_e_ok = tE->emitted && !tE->descendants_emitted &&
                    !tC->emitted && tC->descendants_emitted &&
                    !tA->emitted && tA->descendants_emitted &&
                    !tD->emitted && !tD->descendants_emitted &&
                    !tH->emitted && !tH->descendants_emitted &&
                    !tF->emitted && !tF->descendants_emitted &&
                    !tG->emitted && !tG->descendants_emitted &&
                    tree.EmitPoints().size() == 1 &&
                    tree.EmitPoints()[0] == tE;
  llvm::outs() << "    RecordEmission(E): flags + EmitPoints: "
               << (after_e_ok ? "PASS\n" : "FAIL\n");

  // Re-emit — must be a no-op.
  tree.RecordEmission(tE);
  bool idempotent_ok = tree.EmitPoints().size() == 1;
  llvm::outs() << "    RecordEmission idempotent: "
               << (idempotent_ok ? "PASS\n" : "FAIL\n");

  // Emit a sibling (D). A.descendants_emitted is already set so the
  // parent walk should short-circuit at A.
  tree.RecordEmission(tD);
  bool after_d_ok = tD->emitted &&
                    tree.EmitPoints().size() == 2 &&
                    tree.EmitPoints()[0] == tE &&
                    tree.EmitPoints()[1] == tD;
  llvm::outs() << "    RecordEmission(D): EmitPoints == [E, D]: "
               << (after_d_ok ? "PASS\n" : "FAIL\n");
}

// Helper: build a fresh dom tree + formation tree on BuildTestDAG at
// the given splitter threshold. Returns the graph (kept alive by the
// caller) and the tree, plus pointers to the seven named nodes A, C,
// D, E, F, G, H. Each Phase 3 sub-test wants a clean tree so the
// per-pass guards see no leftover state from a previous test.
struct TestDAGFormationTree {
  std::unique_ptr<ScheduleGraph> graph;
  SubgraphFormationTree tree;
  ScheduleNode *a;
  ScheduleNode *c;
  ScheduleNode *d;
  ScheduleNode *e;
  ScheduleNode *f;
  ScheduleNode *g;
  ScheduleNode *h;
};

TestDAGFormationTree BuildTestDAGFormationTree(int latency_threshold) {
  TestDAGFormationTree out;
  out.graph = ScheduleGraph::BuildTestDAG();
  out.graph->ValidateAndComputeTopologicalOrder();
  out.graph->ComputeTransitiveReductionAndReachability();
  out.graph->ComputeDominatorTree();
  auto is_splitter = [latency_threshold](const ScheduleNode *n) {
    return IsSubgraphSplitter(n, latency_threshold);
  };
  out.tree = SubgraphFormationTree::BuildFromDominatorTree(
      *out.graph, out.graph->GetDominatorTree(), is_splitter);
  // BuildTestDAG emplaces in order [A, C, D, E, F, G, H].
  out.a = &out.graph->Nodes()[0];
  out.c = &out.graph->Nodes()[1];
  out.d = &out.graph->Nodes()[2];
  out.e = &out.graph->Nodes()[3];
  out.f = &out.graph->Nodes()[4];
  out.g = &out.graph->Nodes()[5];
  out.h = &out.graph->Nodes()[6];
  return out;
}

// Phase 3: per-decision-rule passes on BuildTestDAG.
//
// Dom tree (verified by RunTestDAGShakedown's dump):
//   A → {H, C, G};   C → {D, E, F};   leaves H, D, E, F, G.
//
// Splitter status by latency threshold (max outgoing edge per node:
// A=3, C=4, D=2, E=1, F=2, G=0, H=5):
//   - threshold=4 → only H is a splitter.
//   - threshold=1 → A, C, D, F, H are splitters; E (max=1), G (max=0)
//     are not.
//
// At threshold=4:
//   subtree_node_count:     A=7, C=4, others=1
//   subtree_splitter_count: A=1, H=1, others=0
// At threshold=1:
//   subtree_splitter_count: A=5, C=3 (C, D, F), H=1, D=1, F=1,
//                           E=0, G=0
//
// Test outcomes (each assertion gets a fresh tree):
//   1. BottomUpSingleSplitterPass @ thr=4 → emit A only.
//      H has the splitter but its subtree_node_count==1, so we
//      ascend; A is the only qualifying ancestor.
//   2. TopDownSingleSplitterPass @ thr=4 → emit A only.
//      A is the highest qualifying node (rule fires immediately).
//   3. MultiSplitterRescuePass @ thr=1 → emit C only. Picked because
//      it exercises the descendant-emit guard meaningfully: C has
//      subtree_splitter_count==3 > 1 AND
//      subtree_node_count==4 > subtree_splitter_count==3 (E is the
//      bubble-filler material), so C qualifies first in post-order;
//      A inherits descendants_emitted from C and skips. With thr=0,
//      A would emit instead and the walk would terminate before
//      exercising any deeper rescue.
//   4. LargeSplitterFreeRescuePass @ thr=4, size=3 → emit C only.
//      C is splitter-free with subtree_node_count==4 > 3. (Note: A
//      has 1 splitter so it does NOT qualify here.)
//   5. LargeSplitterFreeRescuePass @ thr=4, size=4 → no emissions.
//      C's subtree_node_count==4 not > 4.
//   6. SiblingRescuePass @ thr=4, min_size=0, after manual
//      RecordEmission(C) → cascading trigger at A; H and G qualify
//      (clean siblings, subtree_node_count==1 > 0). Final EmitPoints
//      == [C, H, G] (insertion order).
//   7. Cross-pass composition: BottomUp @ thr=4 then TopDown @ thr=4
//      → second pass emits nothing (everything either emitted or
//      descendants_emitted).
void RunSubgraphFormationPhase3Shakedown() {
  llvm::outs() << "  RunSubgraphFormationPhase3Shakedown:\n";

  // 1. BottomUpSingleSplitterPass @ threshold=4.
  {
    auto t = BuildTestDAGFormationTree(/*latency_threshold=*/4);
    BottomUpSingleSplitterPass(t.tree);
    auto pts = t.tree.EmitPoints();
    bool ok = pts.size() == 1 && pts[0]->schedule_node == t.a;
    llvm::outs() << "    BottomUpSingleSplitterPass @ thr=4 → emit A: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // 2. TopDownSingleSplitterPass @ threshold=4.
  {
    auto t = BuildTestDAGFormationTree(/*latency_threshold=*/4);
    TopDownSingleSplitterPass(t.tree);
    auto pts = t.tree.EmitPoints();
    bool ok = pts.size() == 1 && pts[0]->schedule_node == t.a;
    llvm::outs() << "    TopDownSingleSplitterPass @ thr=4 → emit A: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // 3. MultiSplitterRescuePass @ threshold=1 → emit C (not A).
  {
    auto t = BuildTestDAGFormationTree(/*latency_threshold=*/1);
    MultiSplitterRescuePass(t.tree);
    auto pts = t.tree.EmitPoints();
    bool ok = pts.size() == 1 && pts[0]->schedule_node == t.c;
    llvm::outs() << "    MultiSplitterRescuePass @ thr=1 → emit C: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // 4. LargeSplitterFreeRescuePass @ thr=4, size=3 → emit C.
  {
    auto t = BuildTestDAGFormationTree(/*latency_threshold=*/4);
    LargeSplitterFreeRescuePass(t.tree, /*size_threshold=*/3);
    auto pts = t.tree.EmitPoints();
    bool ok = pts.size() == 1 && pts[0]->schedule_node == t.c;
    llvm::outs() << "    LargeSplitterFreeRescuePass size=3 → emit C: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // 5. LargeSplitterFreeRescuePass @ thr=4, size=4 → no emissions.
  {
    auto t = BuildTestDAGFormationTree(/*latency_threshold=*/4);
    LargeSplitterFreeRescuePass(t.tree, /*size_threshold=*/4);
    auto pts = t.tree.EmitPoints();
    bool ok = pts.empty();
    llvm::outs() << "    LargeSplitterFreeRescuePass size=4 → no emit: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // 6. SiblingRescuePass @ thr=4, min_size=0, after pre-emitting C.
  {
    auto t = BuildTestDAGFormationTree(/*latency_threshold=*/4);
    t.tree.RecordEmission(t.tree.GetNode(t.c));
    SiblingRescuePass(t.tree, /*min_size=*/0);
    auto pts = t.tree.EmitPoints();
    bool ok = pts.size() == 3 && pts[0]->schedule_node == t.c &&
              pts[1]->schedule_node == t.h && pts[2]->schedule_node == t.g;
    llvm::outs() << "    SiblingRescuePass after pre-emit C "
                    "→ EmitPoints == [C, H, G]: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // 7. Cross-pass composition: BottomUp then TopDown — second is no-op.
  {
    auto t = BuildTestDAGFormationTree(/*latency_threshold=*/4);
    BottomUpSingleSplitterPass(t.tree);
    int after_bottom_up = t.tree.EmitPoints().size();
    TopDownSingleSplitterPass(t.tree);
    int after_top_down = t.tree.EmitPoints().size();
    bool ok = after_bottom_up == 1 && after_top_down == 1;
    llvm::outs() << "    BottomUp then TopDown: TopDown is no-op: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }
}

// Helper: members of a SubgraphInfo as a sorted vector of debug
// names — makes set-equality comparisons readable in PASS / FAIL
// messages without depending on the order BuildSubgraphInfos
// discovered them.
std::vector<std::string> SortedMemberNames(const SubgraphInfo &info) {
  std::vector<std::string> out;
  out.reserve(info.members.size());
  for (ScheduleNode *m : info.members) {
    out.push_back(m->GetDebugName().str());
  }
  std::sort(out.begin(), out.end());
  return out;
}

// Phase 4: pipelines + BuildSubgraphInfos on the §8 worked-example
// DAG (BuildSubgraphFormationTestDAG). The DAG is designed so the
// single-splitter passes diverge meaningfully — BottomUp picks the
// deepest qualifying subtree (S, with 4 nodes), TopDown picks the
// highest (A, with all 10) — and so the partition policies produce
// different group counts.
//
// Expected outputs (member sets — order doesn't matter, but the
// per-emit-point order from BuildSubgraphInfos is deterministic):
//
// BottomUpDefault → 1 SubgraphInfo:
//   subgraph_0 = {D, E, F}
//   (Emit point S; split puts {D,E,F} in descendants_or_other;
//    ancestors_of_splitter is empty since S is the dom-root of
//    its own subtree.)
//
// TopDownAggressive default policy → 2 SubgraphInfos:
//   subgraph_0 = {A, P, Q}                           (ancestors)
//   subgraph_1 = {P2, Q2, D, E, F, Exit}             (descendants_or_other)
//
// TopDownAggressive alternate (kSplitDescendantsAndIndependents) → 3:
//   subgraph_0 = {A, P, Q}                           (ancestors)
//   subgraph_1 = {D, E, F, Exit}                     (descendants_of_splitter)
//   subgraph_2 = {P2, Q2}                            (independents)
//
// Singleton suppression check: synthetic emit point on a 1-node
// subtree (any leaf) routed through BuildSubgraphInfos must produce
// no SubgraphInfo (size-2 minimum filter).
void RunSubgraphFormationPhase4Shakedown() {
  llvm::outs() << "  RunSubgraphFormationPhase4Shakedown:\n";

  auto run_pipeline =
      [](const SubgraphFormationPolicy &policy)
      -> std::vector<std::unique_ptr<SubgraphInfo>> {
    auto graph = ScheduleGraph::BuildSubgraphFormationTestDAG();
    graph->ValidateAndComputeTopologicalOrder();
    graph->ComputeTransitiveReductionAndReachability();
    graph->ComputeDominatorTree();
    auto is_splitter = [&policy](const ScheduleNode *n) {
      return IsSubgraphSplitter(n, policy.latency_threshold);
    };
    SubgraphFormationTree tree =
        SubgraphFormationTree::BuildFromDominatorTree(
            *graph, graph->GetDominatorTree(), is_splitter);
    for (auto &pass : policy.pipeline.passes) {
      pass(tree);
    }
    return BuildSubgraphInfos(tree.EmitPoints(), *graph,
                              policy.splitter_partition);
  };

  auto check_subgraphs =
      [](StringRef label,
         ArrayRef<std::unique_ptr<SubgraphInfo>> got,
         ArrayRef<std::vector<std::string>> expected) {
    bool ok = got.size() == expected.size();
    if (ok) {
      for (int i = 0, n = got.size(); i < n; ++i) {
        if (SortedMemberNames(*got[i]) != expected[i]) {
          ok = false;
        }
      }
    }
    llvm::outs() << "    " << label << ": got " << got.size()
                 << " subgraph(s)";
    if (!ok) {
      llvm::outs() << " — members:";
      for (const auto &info : got) {
        llvm::outs() << " {";
        bool first = true;
        for (const std::string &n : SortedMemberNames(*info)) {
          if (!first) {
            llvm::outs() << ",";
          }
          llvm::outs() << n;
          first = false;
        }
        llvm::outs() << "}";
      }
    }
    llvm::outs() << (ok ? "  PASS\n" : "  FAIL\n");
  };

  // 1) BottomUpDefault → {D, E, F}
  {
    auto policy = SubgraphFormationPolicy::BottomUpDefault();
    auto infos = run_pipeline(policy);
    std::vector<std::vector<std::string>> expected = {{"D", "E", "F"}};
    check_subgraphs("BottomUpDefault", infos, expected);
  }

  // 2) TopDownAggressive default policy.
  {
    auto policy = SubgraphFormationPolicy::TopDownAggressive();
    auto infos = run_pipeline(policy);
    std::vector<std::vector<std::string>> expected = {
        {"A", "P", "Q"},
        {"D", "E", "Exit", "F", "P2", "Q2"},
    };
    check_subgraphs("TopDownAggressive bundle", infos, expected);
  }

  // 3) TopDownAggressive split policy.
  {
    auto policy = SubgraphFormationPolicy::TopDownAggressive();
    policy.splitter_partition =
        SplitterPartitionPolicy::kSplitDescendantsAndIndependents;
    auto infos = run_pipeline(policy);
    std::vector<std::vector<std::string>> expected = {
        {"A", "P", "Q"},
        {"D", "E", "Exit", "F"},
        {"P2", "Q2"},
    };
    check_subgraphs("TopDownAggressive split", infos, expected);
  }

  // 4) Singleton suppression: hand-craft an emit point on a leaf
  //    (subtree_node_count==1) and verify BuildSubgraphInfos drops it.
  {
    auto graph = ScheduleGraph::BuildSubgraphFormationTestDAG();
    graph->ValidateAndComputeTopologicalOrder();
    graph->ComputeTransitiveReductionAndReachability();
    graph->ComputeDominatorTree();
    auto is_splitter = [](const ScheduleNode *n) {
      return IsSubgraphSplitter(n, /*latency_threshold=*/32);
    };
    SubgraphFormationTree tree =
        SubgraphFormationTree::BuildFromDominatorTree(
            *graph, graph->GetDominatorTree(), is_splitter);
    // P2 is a leaf with subtree_node_count==1, no splitter — falls
    // into the "0 splitters" arm of BuildSubgraphInfos and gets
    // dropped by the size-2 floor.
    SubgraphFormationTreeNode *p2 = tree.GetNode(&graph->Nodes()[3]);
    tree.RecordEmission(p2);
    auto infos =
        BuildSubgraphInfos(tree.EmitPoints(), *graph,
                           SplitterPartitionPolicy::
                               kBundleDescendantsAndIndependents);
    bool ok = infos.empty();
    llvm::outs() << "    Singleton emit point suppressed: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }
}

// Phase 5 part 1: deterministic end-to-end check of FormSubgraphs
// on the §8 worked-example DAG. Verifies the full driver wiring —
// analyses run, pipeline picks the right emit point, materializer
// produces the right SubgraphInfo, InsertSubgraphProxies attaches
// proxies, and the post-mutation re-derive populates topo + cp.
//
// Expected output: BottomUpDefault → exactly one subgraph with
// members {D, E, F} (the §8 BottomUp result). Graph grows by 2
// nodes (start + end proxy).
void RunSubgraphFormationPhase5Shakedown() {
  llvm::outs() << "  RunSubgraphFormationPhase5Shakedown:\n";
  auto graph = ScheduleGraph::BuildSubgraphFormationTestDAG();
  int num_real_nodes_before = graph->Size();
  auto policy = SubgraphFormationPolicy::BottomUpDefault();
  FormSubgraphs(*graph, policy);

  // Locate the start proxies and pull their SubgraphInfos so we
  // can assert on actual member contents — the driver wiring is
  // what's being verified here, so going through ToString /
  // GetSubgraphInfo proves the proxies hold real data.
  std::vector<SubgraphInfo *> start_infos;
  int got_ends = 0;
  for (ScheduleNode &n : graph->Nodes()) {
    if (n.IsSubgraphStartProxy()) {
      start_infos.push_back(n.GetSubgraphInfo());
    }
    if (n.IsSubgraphEndProxy()) {
      ++got_ends;
    }
  }

  // Single subgraph {D, E, F} expected.
  std::vector<std::vector<std::string>> expected_member_sets = {
      {"D", "E", "F"},
  };
  bool counts_ok =
      static_cast<int>(start_infos.size()) ==
          static_cast<int>(expected_member_sets.size()) &&
      got_ends == static_cast<int>(start_infos.size());
  bool members_ok = counts_ok;
  if (counts_ok) {
    for (size_t i = 0; i < start_infos.size(); ++i) {
      if (SortedMemberNames(*start_infos[i]) != expected_member_sets[i]) {
        members_ok = false;
      }
    }
  }
  bool size_ok = graph->Size() ==
                 num_real_nodes_before + 2 * static_cast<int>(start_infos.size());
  bool topo_ok = graph->IsTopoSorted();
  bool cp_ok = graph->HasCriticalPathFromExit();
  bool ok = counts_ok && members_ok && size_ok && topo_ok && cp_ok;
  llvm::outs() << "    §8 DAG: starts=" << start_infos.size()
               << " ends=" << got_ends
               << " members=" << (members_ok ? "match" : "MISMATCH");
  if (!members_ok) {
    llvm::outs() << " got:";
    for (SubgraphInfo *info : start_infos) {
      llvm::outs() << " {";
      bool first = true;
      for (const std::string &n : SortedMemberNames(*info)) {
        if (!first) {
          llvm::outs() << ",";
        }
        llvm::outs() << n;
        first = false;
      }
      llvm::outs() << "}";
    }
  }
  llvm::outs() << " size=" << num_real_nodes_before << "→" << graph->Size()
               << " topo=" << (topo_ok ? "y" : "n")
               << " cp=" << (cp_ok ? "y" : "n")
               << "  " << (ok ? "PASS\n" : "FAIL\n");
}

// Umbrella for all SubgraphFormation shakedowns that don't need a
// real region. Phases 1–5 are independent and can run in any order;
// grouping them keeps RunAllShakedowns's body short.
void RunAllSubgraphFormationShakedowns() {
  RunSubgraphFormationPhase1Shakedown();
  RunSubgraphFormationPhase2Shakedown();
  RunSubgraphFormationPhase3Shakedown();
  RunSubgraphFormationPhase4Shakedown();
  RunSubgraphFormationPhase5Shakedown();
}

// Phase 5 part 2: smoke test on a real region's graph. Real DAGs
// vary by codegen — we can't assert specific subgraph contents, just
// that FormSubgraphs runs to completion and leaves the graph
// internally consistent (proxy counts match: 1 start + 1 end per
// emitted subgraph; size grew by exactly 2 * subgraphs_added; topo
// and cp are populated).
void RunFormSubgraphsRealRegionSmokeTest(ScheduleGraph &graph) {
  llvm::outs() << "  RunFormSubgraphsRealRegionSmokeTest:\n";
  int num_real_nodes_before = graph.Size();
  int starts_before = 0, ends_before = 0;
  for (const ScheduleNode &n : graph.Nodes()) {
    if (n.IsSubgraphStartProxy()) {
      ++starts_before;
    }
    if (n.IsSubgraphEndProxy()) {
      ++ends_before;
    }
  }
  auto policy = SubgraphFormationPolicy::BottomUpDefault();
  FormSubgraphs(graph, policy);

  int got_starts = 0, got_ends = 0;
  for (const ScheduleNode &n : graph.Nodes()) {
    if (n.IsSubgraphStartProxy()) {
      ++got_starts;
    }
    if (n.IsSubgraphEndProxy()) {
      ++got_ends;
    }
  }
  int subgraphs_added = got_starts - starts_before;
  bool counts_match = (got_ends - ends_before) == subgraphs_added;
  bool size_match = graph.Size() ==
                    num_real_nodes_before + 2 * subgraphs_added;
  bool topo_ok = graph.IsTopoSorted();
  bool cp_ok = graph.HasCriticalPathFromExit();
  bool ok = counts_match && size_match && topo_ok && cp_ok;
  llvm::outs() << "    Real region: N=" << num_real_nodes_before
               << " formed=" << subgraphs_added
               << " new_size=" << graph.Size()
               << " topo=" << (topo_ok ? "y" : "n")
               << " cp=" << (cp_ok ? "y" : "n")
               << "  " << (ok ? "PASS\n" : "FAIL\n");
}

// Helper: check edge set against expected, with kind kSubgraphOrderEdge.
// Uses set comparison + NumX() == set.size() to catch both missing /
// extra targets and duplicate edges to the same target.
bool CheckEdgeSet(ArrayRef<ScheduleEdge> edges, int num_edges,
                  const std::set<ScheduleNode *> &expected) {
  std::set<ScheduleNode *> actual;
  bool kinds_ok = true;
  for (const ScheduleEdge &edge : edges) {
    if (edge.kind_ != ScheduleEdge::kSubgraphOrderEdge) {
      kinds_ok = false;
    }
    actual.insert(edge.node_);
  }
  return kinds_ok && actual == expected &&
         num_edges == static_cast<int>(expected.size());
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
// After InsertSubgraphProxies (TWO proxies per subgraph):
//   - graph.Size() == 9 (was 7 + start + end).
//   - graph.NumSchedulingUnits() == 7 (proxies excluded).
//   - Exactly one IsSubgraphStartProxy and one IsSubgraphEndProxy.
//   - Start proxy: parent_subgraph_proxy == nullptr (top-level),
//       GetSubgraphInfo()->debug_name == "S",
//       SubgraphInfo->subgraph_proxy == start,
//       predecessors set == {A}, successors set == {C,D,E,F}.
//   - End proxy: parent_subgraph_proxy == start (lives in subgraph
//       scope), GetSubgraphInfo() returns the same info object,
//       SubgraphInfo->end_proxy == end,
//       predecessors set == {C,D,E,F}, successors set == {G}.
//   - Members C, D, E, F have parent_subgraph_proxy == start.
//   - Non-members A, G, H have null parent_subgraph_proxy.
//   - The member↔proxy edge directions are guaranteed by
//     AddSuccessor's edge-symmetry contract: start's successors
//     == {C,D,E,F} implies each member has start as a predecessor,
//     and end's predecessors == {C,D,E,F} implies each member has
//     end as a successor. No separate per-member edge counts needed.
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

  // Locate start and end proxies. There should be exactly one of
  // each.
  ScheduleNode *start = nullptr;
  ScheduleNode *end = nullptr;
  int start_count = 0;
  int end_count = 0;
  for (ScheduleNode &n : graph->Nodes()) {
    if (n.IsSubgraphStartProxy()) {
      ++start_count;
      start = &n;
    }
    if (n.IsSubgraphEndProxy()) {
      ++end_count;
      end = &n;
    }
  }

  // ── Verify post-insertion structure ────────────────────────────
  bool size_ok =
      graph->Size() == 9 && graph->NumSchedulingUnits() == 7;
  llvm::outs() << "  Graph size: total=" << graph->Size()
               << " scheduling_units=" << graph->NumSchedulingUnits()
               << "  " << (size_ok ? "PASS" : "FAIL") << "\n";

  bool counts_ok = start_count == 1 && end_count == 1 &&
                   start != nullptr && end != nullptr;
  llvm::outs() << "  Proxy counts: start=" << start_count
               << " end=" << end_count << "  "
               << (counts_ok ? "PASS" : "FAIL") << "\n";

  bool start_basic_ok =
      counts_ok && start->IsSubgraphStartProxy() &&
      start->GetParentSubgraphProxy() == nullptr &&
      start->GetSubgraphInfo()->debug_name == "S" &&
      start->GetSubgraphInfo()->subgraph_proxy == start;
  llvm::outs() << "  Start proxy: name=\""
               << (start ? start->GetSubgraphInfo()->debug_name : "")
               << "\" parent=" << (start_basic_ok ? "null" : "WRONG")
               << "  " << (start_basic_ok ? "PASS" : "FAIL") << "\n";

  bool end_basic_ok =
      counts_ok && end->IsSubgraphEndProxy() &&
      end->GetParentSubgraphProxy() == start &&
      end->GetSubgraphInfo() == start->GetSubgraphInfo() &&
      end->GetSubgraphInfo()->end_proxy == end;
  llvm::outs() << "  End proxy: parent=start info==start_info  "
               << (end_basic_ok ? "PASS" : "FAIL") << "\n";

  bool member_parents_ok =
      c->GetParentSubgraphProxy() == start &&
      d->GetParentSubgraphProxy() == start &&
      e->GetParentSubgraphProxy() == start &&
      f->GetParentSubgraphProxy() == start;
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

  // Start proxy edges: predecessors == {A}, successors == {C,D,E,F}.
  bool start_preds_ok =
      counts_ok && CheckEdgeSet(start->Predecessors(),
                                start->NumPredecessors(), {a});
  llvm::outs() << "  Start proxy predecessors: count="
               << (start ? start->NumPredecessors() : -1)
               << " expected_set={A}  "
               << (start_preds_ok ? "PASS" : "FAIL") << "\n";
  bool start_succs_ok =
      counts_ok && CheckEdgeSet(start->Successors(),
                                start->NumSuccessors(), {c, d, e, f});
  llvm::outs() << "  Start proxy successors: count="
               << (start ? start->NumSuccessors() : -1)
               << " expected_set={C,D,E,F}  "
               << (start_succs_ok ? "PASS" : "FAIL") << "\n";

  // End proxy edges: predecessors == {C,D,E,F}, successors == {G}.
  bool end_preds_ok =
      counts_ok && CheckEdgeSet(end->Predecessors(),
                                end->NumPredecessors(), {c, d, e, f});
  llvm::outs() << "  End proxy predecessors: count="
               << (end ? end->NumPredecessors() : -1)
               << " expected_set={C,D,E,F}  "
               << (end_preds_ok ? "PASS" : "FAIL") << "\n";
  bool end_succs_ok =
      counts_ok && CheckEdgeSet(end->Successors(),
                                end->NumSuccessors(), {g});
  llvm::outs() << "  End proxy successors: count="
               << (end ? end->NumSuccessors() : -1)
               << " expected_set={G}  "
               << (end_succs_ok ? "PASS" : "FAIL") << "\n";

  // Topological order is recomputed by InsertSubgraphProxies.
  bool topo_ok = graph->IsTopoSorted();
  llvm::outs() << "  Topological order recomputed: "
               << (topo_ok ? "yes" : "no") << "  "
               << (topo_ok ? "PASS" : "FAIL") << "\n";
}

// Demonstrates that wrapping {A, B, C} as a subgraph forces the
// chain to be scheduled contiguously, even when the unconstrained
// scheduler would have interleaved X and Y between A and B/C to
// fill the latency bubbles.
//
// See ScheduleGraph::BuildContiguityTestDAG for the DAG shape and
// the cycle-by-cycle reasoning for the expected lengths (11 cycles
// unclustered, 13 cycles clustered — the 2-cycle delta is the
// bubble that X+Y filled in the unclustered case).
//
// Uses the default topo-asc ready comparator; greedy "pick first
// ready" scheduling. Both checks are PASS/FAIL:
//   1. clustered length > unclustered length  (proves contiguity
//      changed scheduler behavior, not just that the proxy
//      machinery ran).
//   2. A, B, C appear at consecutive positions in the clustered
//      schedule_order (filtering out the proxies — see the
//      ApplyScheduleOrder filter for the same idea in production).
void RunSubgraphContiguityShakedown(const MachineFunction &mf,
                                    const LiveIntervals &lis) {
  llvm::outs() << "  Subgraph contiguity shakedown:\n";

  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(mf.getSubtarget());

  // Greedy schedule helper: runs Schedule(first ready) until done,
  // returns the constructor for length / schedule_order inspection.
  auto greedy_schedule = [&](ScheduleGraph &graph) {
    ScheduleConstructor sc(graph, st, mf);
    int safety = graph.Size() + 1;
    while (!sc.IsDone()) {
      if (--safety < 0) {
        report_fatal_error(
            "contiguity shakedown: greedy loop did not finish in "
            "graph.Size()+1 steps");
      }
      const auto &ready = sc.GetReadyList();
      if (ready.empty()) {
        report_fatal_error(
            "contiguity shakedown: ready list empty before IsDone");
      }
      sc.Schedule(ready.front());
    }
    return sc;
  };

  // ── Unclustered run ────────────────────────────────────────────
  auto unclustered = ScheduleGraph::BuildContiguityTestDAG();
  unclustered->ValidateAndComputeTopologicalOrder();
  unclustered->ComputeCriticalPaths();
  ScheduleConstructor sc_unclustered = greedy_schedule(*unclustered);
  int unclustered_length = sc_unclustered.GetLengthTracker().GetCurrentCycle();

  // ── Clustered run ──────────────────────────────────────────────
  auto clustered = ScheduleGraph::BuildContiguityTestDAG();
  clustered->ValidateAndComputeTopologicalOrder();
  // Identify members by their emplacement positions in
  // BuildContiguityTestDAG: [A, X, Y, B, C].
  ScheduleNode *a = &clustered->Nodes()[0];
  ScheduleNode *b = &clustered->Nodes()[3];
  ScheduleNode *c = &clustered->Nodes()[4];
  SmallVector<ScheduleNode *, 4> members = {a, b, c};
  auto info = std::make_unique<SubgraphInfo>(members, "ABC_chain");
  std::vector<std::unique_ptr<SubgraphInfo>> infos;
  infos.push_back(std::move(info));
  clustered->InsertSubgraphProxies(std::move(infos));
  // InsertSubgraphProxies recomputes critical path internally.
  ScheduleConstructor sc_clustered = greedy_schedule(*clustered);
  int clustered_length = sc_clustered.GetLengthTracker().GetCurrentCycle();

  // ── Check 1: clustered length must exceed unclustered length ──
  bool length_ok = clustered_length > unclustered_length;
  llvm::outs() << "    Length: unclustered=" << unclustered_length
               << " clustered=" << clustered_length
               << " (clustered > unclustered required)  "
               << (length_ok ? "PASS" : "FAIL") << "\n";

  // ── Check 2: A, B, C must appear consecutively in the
  //    clustered schedule_order (after filtering out proxies) ────
  ArrayRef<const ScheduleNode *> order = sc_clustered.GetScheduleOrder();
  // Refer to the same A, B, C pointers from the clustered graph.
  ScheduleNode *clustered_a = a;
  ScheduleNode *clustered_b = b;
  ScheduleNode *clustered_c = c;
  int idx_a = -1;
  int idx_b = -1;
  int idx_c = -1;
  int filtered_idx = 0;
  for (const ScheduleNode *n : order) {
    if (n->IsSubgraphProxy()) {
      continue;
    }
    if (n == clustered_a) {
      idx_a = filtered_idx;
    } else if (n == clustered_b) {
      idx_b = filtered_idx;
    } else if (n == clustered_c) {
      idx_c = filtered_idx;
    }
    ++filtered_idx;
  }
  bool consecutive_ok = idx_a >= 0 && idx_b == idx_a + 1 &&
                        idx_c == idx_b + 1;
  llvm::outs() << "    Chain positions in (proxy-filtered) order: "
               << "A=" << idx_a << " B=" << idx_b << " C=" << idx_c
               << " (consecutive required)  "
               << (consecutive_ok ? "PASS" : "FAIL") << "\n";

  if (!length_ok || !consecutive_ok) {
    report_fatal_error("RunSubgraphContiguityShakedown failed");
  }
}

// Verifies ScheduledSetTracker via several scenarios on synthetic
// graphs. Each subtest constructs a fresh graph + ScheduleLengthTracker
// + ScheduledSetTracker and drives Schedule/Unschedule sequences.
//
// BuildTestDAG layout (from BuildTestDAG()'s docstring), 7 nodes:
//   A → H (latency 1), A → C (latency 2), A → D (latency 3),
//   C → D (latency 1), C → E (latency 4),
//   D → F (latency 2), E → F (latency 1),
//   H → G (latency 5), F → G (latency 2)
// Topo order produced by Kahn's: [A, H, C, D, E, F, G].
void RunScheduledSetTrackerShakedown(const GCNSubtarget &st) {
  llvm::outs() << "  RunScheduledSetTrackerShakedown:\n";

  // --- Empty initial state on BuildTestDAG ---
  {
    auto graph = ScheduleGraph::BuildTestDAG();
    graph->ValidateAndComputeTopologicalOrder();
    graph->ComputeCriticalPaths();
    ScheduleLengthTracker length_tracker(*graph, st);
    ScheduledSetTracker scheduled_set_tracker(graph.get(), &length_tracker);
    bool ok = scheduled_set_tracker.GetPrefixSignature() == 0 &&
              scheduled_set_tracker.GetScheduledSet().none() &&
              scheduled_set_tracker.GetFrontier().empty();
    llvm::outs() << "    Empty initial state: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // --- Schedule(A) then Schedule(H): hand-checked frontier+LBs ---
  // After Schedule(A) at cycle 0:
  //   frontier = {H, C, D}
  //   LB(H) = 0 + max(1, A.IssueSlotsConsumed=1) = 1
  //   LB(C) = 0 + max(2, 1)                       = 2
  //   LB(D) = 0 + max(3, 1)                       = 3
  // After Schedule(H) at cycle 1:
  //   frontier = {C, D, G}    (H removed; H→G adds G)
  //   LB(G) = 1 + max(5, 1) = 6 ; LB(C), LB(D) unchanged.
  {
    auto graph = ScheduleGraph::BuildTestDAG();
    graph->ValidateAndComputeTopologicalOrder();
    graph->ComputeCriticalPaths();
    ArrayRef<ScheduleNode *> topo = graph->GetTopoOrder();
    ScheduleNode *a = topo[0];
    ScheduleNode *h = topo[1];
    ScheduleNode *c = topo[2];
    ScheduleNode *d = topo[3];
    ScheduleNode *g = topo[6];

    ScheduleLengthTracker length_tracker(*graph, st);
    ScheduledSetTracker scheduled_set_tracker(graph.get(), &length_tracker);

    length_tracker.Schedule(a);
    scheduled_set_tracker.Schedule(a);
    const auto &fr1 = scheduled_set_tracker.GetFrontier();
    bool a_ok = fr1.size() == 3 &&
                fr1.lookup(h->GetTopoIndex()).lower_bound == 1 &&
                fr1.lookup(c->GetTopoIndex()).lower_bound == 2 &&
                fr1.lookup(d->GetTopoIndex()).lower_bound == 3;
    llvm::outs() << "    Schedule(A): frontier {H=1,C=2,D=3}: "
                 << (a_ok ? "PASS\n" : "FAIL\n");

    length_tracker.Schedule(h);
    scheduled_set_tracker.Schedule(h);
    const auto &fr2 = scheduled_set_tracker.GetFrontier();
    bool h_ok = fr2.size() == 3 && fr2.count(h->GetTopoIndex()) == 0 &&
                fr2.lookup(c->GetTopoIndex()).lower_bound == 2 &&
                fr2.lookup(d->GetTopoIndex()).lower_bound == 3 &&
                fr2.lookup(g->GetTopoIndex()).lower_bound == 6;
    llvm::outs() << "    +Schedule(H): frontier {C=2,D=3,G=6}: "
                 << (h_ok ? "PASS\n" : "FAIL\n");
  }

  // --- Round-trip on BuildTestDAG ---
  // Schedule everything in topo order, Unschedule everything in
  // reverse — final state must match empty initial state.
  {
    auto graph = ScheduleGraph::BuildTestDAG();
    graph->ValidateAndComputeTopologicalOrder();
    graph->ComputeCriticalPaths();
    ScheduleLengthTracker length_tracker(*graph, st);
    ScheduledSetTracker scheduled_set_tracker(graph.get(), &length_tracker);
    ArrayRef<ScheduleNode *> topo = graph->GetTopoOrder();

    for (ScheduleNode *node : topo) {
      length_tracker.Schedule(node);
      scheduled_set_tracker.Schedule(node);
    }
    for (auto it = topo.rbegin(); it != topo.rend(); ++it) {
      scheduled_set_tracker.Unschedule(*it);
      length_tracker.Unschedule(*it);
    }
    bool ok = scheduled_set_tracker.GetPrefixSignature() == 0 &&
              scheduled_set_tracker.GetScheduledSet().none() &&
              scheduled_set_tracker.GetFrontier().empty();
    llvm::outs() << "    Round-trip on BuildTestDAG: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // --- Order invariance for {A, H, C} ---
  // XOR is commutative, so scheduling {A, H, C} as A→H→C should
  // give the same prefix signature and bitset as A→C→H.
  {
    auto build = [&]() {
      auto g = ScheduleGraph::BuildTestDAG();
      g->ValidateAndComputeTopologicalOrder();
      g->ComputeCriticalPaths();
      return g;
    };
    auto graph1 = build();
    auto graph2 = build();
    ArrayRef<ScheduleNode *> topo1 = graph1->GetTopoOrder();
    ArrayRef<ScheduleNode *> topo2 = graph2->GetTopoOrder();

    ScheduleLengthTracker lt1(*graph1, st);
    ScheduledSetTracker sub1(graph1.get(), &lt1);
    lt1.Schedule(topo1[0]); sub1.Schedule(topo1[0]); // A
    lt1.Schedule(topo1[1]); sub1.Schedule(topo1[1]); // H
    lt1.Schedule(topo1[2]); sub1.Schedule(topo1[2]); // C

    ScheduleLengthTracker lt2(*graph2, st);
    ScheduledSetTracker sub2(graph2.get(), &lt2);
    lt2.Schedule(topo2[0]); sub2.Schedule(topo2[0]); // A
    lt2.Schedule(topo2[2]); sub2.Schedule(topo2[2]); // C
    lt2.Schedule(topo2[1]); sub2.Schedule(topo2[1]); // H

    bool ok =
        sub1.GetPrefixSignature() == sub2.GetPrefixSignature() &&
        sub1.GetScheduledSet() == sub2.GetScheduledSet();
    llvm::outs() << "    Order invariance for {A,H,C}: "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // --- Multi-edge A → H on BuildTestDAG (existing A→H lat=1 plus
  //     a second added A→H lat=4) ---
  // The earlier "recompute overwrites count" bug was multi-edge-
  // specific. With the current "no stored count, always recompute"
  // design that bug class is gone, but exercise the path
  // explicitly: after Schedule(A), LB(H) should be max of the two
  // contributions = max(0+max(1,1), 0+max(4,1)) = 4. After the
  // Schedule(A)/Schedule(H)/Unschedule(H)/Unschedule(A) round-trip,
  // frontier must restore cleanly.
  {
    auto graph = ScheduleGraph::BuildTestDAG();
    // Indices in BuildTestDAG emplacement order: A=0, C=1, D=2,
    // E=3, F=4, G=5, H=6.
    ScheduleNode *a = &graph->Nodes()[0];
    ScheduleNode *h = &graph->Nodes()[6];
    graph->AddEdge(a, h, ScheduleEdge::kData, /*latency=*/4);
    graph->ValidateAndComputeTopologicalOrder();
    graph->ComputeCriticalPaths();
    ScheduleLengthTracker length_tracker(*graph, st);
    ScheduledSetTracker scheduled_set_tracker(graph.get(), &length_tracker);

    length_tracker.Schedule(a);
    scheduled_set_tracker.Schedule(a);
    bool lb_after_a =
        scheduled_set_tracker.GetFrontier().lookup(h->GetTopoIndex()).lower_bound == 4;

    length_tracker.Schedule(h);
    scheduled_set_tracker.Schedule(h);
    bool h_gone = scheduled_set_tracker.GetFrontier().count(h->GetTopoIndex()) == 0;

    scheduled_set_tracker.Unschedule(h);
    length_tracker.Unschedule(h);
    bool h_back_with_lb_4 =
        scheduled_set_tracker.GetFrontier().lookup(h->GetTopoIndex()).lower_bound == 4;

    scheduled_set_tracker.Unschedule(a);
    length_tracker.Unschedule(a);
    // After Unschedule(A), no real preds scheduled — entire frontier
    // empty (A was the only scheduled node).
    bool empty_after_a = scheduled_set_tracker.GetFrontier().empty();

    bool ok = lb_after_a && h_gone && h_back_with_lb_4 && empty_after_a;
    llvm::outs() << "    Multi-edge A→H (×2, lats 1 and 4): "
                 << (ok ? "PASS\n" : "FAIL\n");
  }

  // --- Subgraph proxies present ---
  // Run formation on a DAG that produces at least one subgraph.
  // Then drive the tracker across the whole graph and verify:
  //   - proxies appear in the bitset (proxy state is part of the
  //     search-state identity).
  //   - proxies NEVER appear in the frontier (proxy contributions
  //     to LB are 0; the tracker skips proxies from frontier
  //     mechanics by design).
  //   - round-trip restores cleanly.
  {
    auto graph = ScheduleGraph::BuildSubgraphFormationTestDAG();
    int n_real_before = graph->Size();
    auto policy = SubgraphFormationPolicy::TopDownSingleSplitterOnly();
    FormSubgraphs(*graph, policy);
    bool subgraph_formed = graph->Size() > n_real_before;

    ScheduleLengthTracker length_tracker(*graph, st);
    ScheduledSetTracker scheduled_set_tracker(graph.get(), &length_tracker);
    ArrayRef<ScheduleNode *> topo = graph->GetTopoOrder();

    bool proxy_in_frontier_ever = false;
    for (ScheduleNode *node : topo) {
      length_tracker.Schedule(node);
      scheduled_set_tracker.Schedule(node);
      for (const auto &kv : scheduled_set_tracker.GetFrontier()) {
        if (topo[kv.first]->IsSubgraphProxy()) {
          proxy_in_frontier_ever = true;
        }
      }
    }
    bool proxy_in_scheduled_set = false;
    for (int i = 0; i < graph->Size(); ++i) {
      if (topo[i]->IsSubgraphProxy() &&
          scheduled_set_tracker.GetScheduledSet().test(i)) {
        proxy_in_scheduled_set = true;
      }
    }
    bool all_scheduled =
        scheduled_set_tracker.GetScheduledSet().count() == graph->Size();

    for (auto it = topo.rbegin(); it != topo.rend(); ++it) {
      scheduled_set_tracker.Unschedule(*it);
      length_tracker.Unschedule(*it);
    }
    bool round_trip_ok = scheduled_set_tracker.GetPrefixSignature() == 0 &&
                         scheduled_set_tracker.GetScheduledSet().none() &&
                         scheduled_set_tracker.GetFrontier().empty();

    bool ok = subgraph_formed && all_scheduled &&
              proxy_in_scheduled_set && !proxy_in_frontier_ever &&
              round_trip_ok;
    llvm::outs() << "    Subgraphs present (proxies in bitset, "
                    "absent from frontier, round-trip): "
                 << (ok ? "PASS\n" : "FAIL\n");
  }
}

// Fixture for LengthHistoryTracker shakedowns. Owns the graph and
// the three trackers (length, scheduled-set, history) so each test
// gets a fresh, fully-wired stack. Heap allocation via unique_ptr
// keeps tracker pointers stable across move/return.
struct LengthHistoryTrackerFixture {
  std::unique_ptr<ScheduleGraph> graph;
  std::unique_ptr<ScheduleLengthTracker> length_tracker;
  std::unique_ptr<ScheduledSetTracker> scheduled_set_tracker;
  std::unique_ptr<LengthHistoryTracker> length_history_tracker;
  // Common BuildTestDAG node handles. Topo indices: A=0, H=1,
  // C=2, D=3.
  ScheduleNode *a;
  ScheduleNode *h;
  ScheduleNode *c;
  ScheduleNode *d;
};

static LengthHistoryTrackerFixture
BuildLengthHistoryTrackerFixture(const GCNSubtarget &st,
                                 bool include_pressure_dim = false,
                                 bool include_ilp_dim = false,
                                 bool length_max_mode = false) {
  LengthHistoryTrackerFixture fixture;
  fixture.graph = ScheduleGraph::BuildTestDAG();
  fixture.graph->ValidateAndComputeTopologicalOrder();
  fixture.graph->ComputeCriticalPaths();
  ArrayRef<ScheduleNode *> topo = fixture.graph->GetTopoOrder();
  fixture.a = topo[0];
  fixture.h = topo[1];
  fixture.c = topo[2];
  fixture.d = topo[3];
  fixture.length_tracker =
      std::make_unique<ScheduleLengthTracker>(*fixture.graph, st);
  fixture.scheduled_set_tracker = std::make_unique<ScheduledSetTracker>(
      fixture.graph.get(), fixture.length_tracker.get());
  // Score-source trackers are nullptr; tests stage Entry contents
  // directly via InsertEntryForTest rather than driving the
  // production query path.
  fixture.length_history_tracker = std::make_unique<LengthHistoryTracker>(
      fixture.scheduled_set_tracker.get(), fixture.length_tracker.get(),
      /*pressure_tracker=*/nullptr, /*ilp_tracker=*/nullptr,
      include_pressure_dim, include_ilp_dim, length_max_mode);
  return fixture;
}

// Drive both length and scheduled-set trackers in lockstep. Matches
// the production order in ScheduleConstructor::ScheduleByIndex.
static void ScheduleNodeOnFixture(LengthHistoryTrackerFixture &fixture,
                                  ScheduleNode *node) {
  fixture.length_tracker->Schedule(node);
  fixture.scheduled_set_tracker->Schedule(node);
}

static void UnscheduleNodeOnFixture(LengthHistoryTrackerFixture &fixture,
                                    ScheduleNode *node) {
  fixture.scheduled_set_tracker->Unschedule(node);
  fixture.length_tracker->Unschedule(node);
}

// Test 1: Empty-table behavior.
// After construction, no entries; IsDominated returns false even
// after some scheduling activity has set up a query state.
static void RunLengthHistoryEmptyShakedown(const GCNSubtarget &st) {
  auto fixture = BuildLengthHistoryTrackerFixture(st);
  bool initially_empty =
      fixture.length_history_tracker->GetTotalEntries() == 0;
  ScheduleNodeOnFixture(fixture, fixture.a);
  bool not_dominated_when_empty =
      !fixture.length_history_tracker->IsDominated();
  bool ok = initially_empty && not_dominated_when_empty;
  llvm::outs() << "    Empty-table behavior: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 2: GetFrontierLbsSnapshot matches the bound tracker's
// frontier, sorted by node_topo_idx. After Schedule(A) on
// BuildTestDAG: frontier {H@1, C@2, D@3}. Topo indices already in
// ascending order (H=1, C=2, D=3), so sorted snapshot order is
// (H, C, D).
static void RunLengthHistorySnapshotShakedown(const GCNSubtarget &st) {
  auto fixture = BuildLengthHistoryTrackerFixture(st);
  ScheduleNodeOnFixture(fixture, fixture.a);
  SmallVector<FrontierLb, 16> snapshot =
      fixture.length_history_tracker->GetFrontierLbsSnapshot();
  bool ok = snapshot.size() == 3 &&
            snapshot[0].node_topo_idx == fixture.h->GetTopoIndex() &&
            snapshot[0].lower_bound == 1 &&
            snapshot[1].node_topo_idx == fixture.c->GetTopoIndex() &&
            snapshot[1].lower_bound == 2 &&
            snapshot[2].node_topo_idx == fixture.d->GetTopoIndex() &&
            snapshot[2].lower_bound == 3;
  llvm::outs() << "    Snapshot matches frontier {H@1, C@2, D@3}: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 3: First IsDominatedElseInsert from empty inserts; second
// call is dominated by what we just inserted (equality dominates).
static void RunLengthHistoryFirstInsertAndSelfDominanceShakedown(
    const GCNSubtarget &st) {
  auto fixture = BuildLengthHistoryTrackerFixture(st);
  ScheduleNodeOnFixture(fixture, fixture.a);
  bool first_call_inserted =
      !fixture.length_history_tracker->IsDominatedElseInsert();
  bool count_one_after_first =
      fixture.length_history_tracker->GetTotalEntries() == 1;

  // Same bound state — second IsDominated should hit the entry we
  // just inserted (equal end_cycle, equal frontier_lbs → dominates).
  bool dominated_on_second_query =
      fixture.length_history_tracker->IsDominated();
  bool second_insert_pruned =
      fixture.length_history_tracker->IsDominatedElseInsert();
  bool count_unchanged_after_second =
      fixture.length_history_tracker->GetTotalEntries() == 1;

  bool ok = first_call_inserted && count_one_after_first &&
            dominated_on_second_query && second_insert_pruned &&
            count_unchanged_after_second;
  llvm::outs() << "    First insert + self-dominance: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 4: Strict dominator. Stage an entry strictly better than
// the query (smaller end_cycle, equal frontier LBs). IsDominated
// returns true; IsDominatedElseInsert returns true; bucket
// unchanged.
static void RunLengthHistoryStrictDominatorShakedown(
    const GCNSubtarget &st) {
  auto fixture = BuildLengthHistoryTrackerFixture(st);
  ScheduleNodeOnFixture(fixture, fixture.a);
  PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
  // Equal frontier_lbs to query (H=1, C=2, D=3 sorted by topo
  // idx); end_cycle=0 strictly beats query's end_cycle=1.
  LengthHistoryTracker::Entry strict_dominator;
  strict_dominator.end_cycle = 0;
  strict_dominator.frontier_lbs = {
      {fixture.h->GetTopoIndex(), 1},
      {fixture.c->GetTopoIndex(), 2},
      {fixture.d->GetTopoIndex(), 3},
  };
  fixture.length_history_tracker->InsertEntryForTest(key, strict_dominator);

  bool dominated = fixture.length_history_tracker->IsDominated();
  bool insert_pruned =
      fixture.length_history_tracker->IsDominatedElseInsert();
  bool count_unchanged =
      fixture.length_history_tracker->GetTotalEntries() == 1;
  bool bucket_unchanged =
      fixture.length_history_tracker->GetBucketForTest(key).size() == 1;

  bool ok = dominated && insert_pruned && count_unchanged &&
            bucket_unchanged;
  llvm::outs() << "    Strict dominator pruning: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 5: Pareto trim. Stage an entry strictly worse than the
// query. IsDominated returns false; IsDominatedElseInsert returns
// false, removes the dominated entry, and inserts the query.
// Bucket size stays at 1; total_entries stays at 1.
static void RunLengthHistoryParetoTrimShakedown(const GCNSubtarget &st) {
  auto fixture = BuildLengthHistoryTrackerFixture(st);
  ScheduleNodeOnFixture(fixture, fixture.a);
  PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
  // end_cycle=5 (worse than query's 1); frontier_lbs all equal.
  LengthHistoryTracker::Entry strictly_worse;
  strictly_worse.end_cycle = 5;
  strictly_worse.frontier_lbs = {
      {fixture.h->GetTopoIndex(), 1},
      {fixture.c->GetTopoIndex(), 2},
      {fixture.d->GetTopoIndex(), 3},
  };
  fixture.length_history_tracker->InsertEntryForTest(key, strictly_worse);

  bool not_dominated = !fixture.length_history_tracker->IsDominated();
  bool insert_succeeded =
      !fixture.length_history_tracker->IsDominatedElseInsert();
  // Trim removed strictly_worse; insert added query → count == 1.
  bool count_one =
      fixture.length_history_tracker->GetTotalEntries() == 1;
  ArrayRef<LengthHistoryTracker::Entry> bucket =
      fixture.length_history_tracker->GetBucketForTest(key);
  bool bucket_one = bucket.size() == 1;
  // The remaining entry should be the query (end_cycle=1), not the
  // strictly_worse one we trimmed (end_cycle=5).
  bool query_is_what_remains = bucket_one && bucket[0].end_cycle == 1;

  bool ok = not_dominated && insert_succeeded && count_one &&
            bucket_one && query_is_what_remains;
  llvm::outs() << "    Pareto trim: " << (ok ? "PASS\n" : "FAIL\n");
}

// Test 6: Incomparable entries co-exist. Stage an entry with a
// better end_cycle but a worse LB on H. Neither dominates the
// other; both stay in the bucket after IsDominatedElseInsert.
static void RunLengthHistoryIncomparableShakedown(const GCNSubtarget &st) {
  auto fixture = BuildLengthHistoryTrackerFixture(st);
  ScheduleNodeOnFixture(fixture, fixture.a);
  PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
  // end_cycle=0 (better than query's 1); H_lb=5 (worse than
  // query's 1). C and D LBs equal. Neither dominates the other.
  LengthHistoryTracker::Entry incomparable;
  incomparable.end_cycle = 0;
  incomparable.frontier_lbs = {
      {fixture.h->GetTopoIndex(), 5},
      {fixture.c->GetTopoIndex(), 2},
      {fixture.d->GetTopoIndex(), 3},
  };
  fixture.length_history_tracker->InsertEntryForTest(key, incomparable);

  bool not_dominated = !fixture.length_history_tracker->IsDominated();
  bool insert_succeeded =
      !fixture.length_history_tracker->IsDominatedElseInsert();
  bool count_two =
      fixture.length_history_tracker->GetTotalEntries() == 2;
  bool bucket_two =
      fixture.length_history_tracker->GetBucketForTest(key).size() == 2;

  bool ok = not_dominated && insert_succeeded && count_two && bucket_two;
  llvm::outs() << "    Incomparable entries co-exist: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 7: GetPartitionKey() integration with the table — distinct
// schedule states produce distinct PartitionKeys (different
// scheduled_sets) that the table holds as separate buckets.
//
// Uses {A} → {A, H} (both reachable via valid scheduling order),
// not {A} → {H}, because ScheduleLengthTracker requires scheduled
// latency-bearing predecessors when computing a node's ready
// cycle. H's only predecessor is A, so scheduling H without A
// first would violate that invariant.
static void RunLengthHistoryDistinctPartitionsShakedown(
    const GCNSubtarget &st) {
  auto fixture = BuildLengthHistoryTrackerFixture(st);
  // Partition 1: {A} scheduled.
  ScheduleNodeOnFixture(fixture, fixture.a);
  PartitionKey key_a = fixture.scheduled_set_tracker->GetPartitionKey();
  bool a_inserted =
      !fixture.length_history_tracker->IsDominatedElseInsert();

  // Partition 2: {A, H} scheduled.
  ScheduleNodeOnFixture(fixture, fixture.h);
  PartitionKey key_ah = fixture.scheduled_set_tracker->GetPartitionKey();
  bool ah_inserted =
      !fixture.length_history_tracker->IsDominatedElseInsert();

  bool count_two =
      fixture.length_history_tracker->GetTotalEntries() == 2;
  bool bucket_a_size_one =
      fixture.length_history_tracker->GetBucketForTest(key_a).size() == 1;
  bool bucket_ah_size_one =
      fixture.length_history_tracker->GetBucketForTest(key_ah).size() == 1;

  bool ok = a_inserted && ah_inserted && count_two &&
            bucket_a_size_one && bucket_ah_size_one;
  llvm::outs() << "    GetPartitionKey() integration across "
                  "distinct partitions: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 8: Hash collision across distinct partitions. Hand-craft
// two PartitionKeys with the same signature but different bitsets.
// DenseMap probing + isEqual must keep the buckets separate.
//
// This synthesizes the collision rather than waiting for one to
// occur naturally — natural collisions on the per-node signatures
// produced by BuildTestDAG are vanishingly rare and hard to
// reproduce.
static void RunLengthHistoryHashCollisionShakedown(const GCNSubtarget &st) {
  auto fixture = BuildLengthHistoryTrackerFixture(st);
  int n = fixture.graph->Size();

  // Two keys, same signature (42), different bitsets. Bitset
  // sizes match graph.Size() (n >= 2) so neither hits the
  // sentinel sizes (0 or 1).
  PartitionKey key1{42, BitVector(n)};
  key1.scheduled_set.set(0);
  PartitionKey key2{42, BitVector(n)};
  key2.scheduled_set.set(1);

  LengthHistoryTracker::Entry entry1;
  entry1.end_cycle = 100;
  LengthHistoryTracker::Entry entry2;
  entry2.end_cycle = 200;
  fixture.length_history_tracker->InsertEntryForTest(key1, entry1);
  fixture.length_history_tracker->InsertEntryForTest(key2, entry2);

  ArrayRef<LengthHistoryTracker::Entry> bucket1 =
      fixture.length_history_tracker->GetBucketForTest(key1);
  ArrayRef<LengthHistoryTracker::Entry> bucket2 =
      fixture.length_history_tracker->GetBucketForTest(key2);
  bool both_buckets_present =
      bucket1.size() == 1 && bucket2.size() == 1;
  bool entries_in_correct_buckets =
      both_buckets_present && bucket1[0].end_cycle == 100 &&
      bucket2[0].end_cycle == 200;
  bool count_two =
      fixture.length_history_tracker->GetTotalEntries() == 2;

  bool ok = both_buckets_present && entries_in_correct_buckets &&
            count_two;
  llvm::outs() << "    Hash collision across distinct partitions: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Pressure-score dimension on dominance. With
// include_pressure_dim=true, dominance also requires
// prior.continuous_occupancy_score >= query.continuous_occupancy_score
// (reversed direction — higher is better). With null
// pressure_tracker the query's score is read as 0; tests stage
// prior scores explicitly via InsertEntryForTest.
//
// Cases on a state where length dims would otherwise dominate:
//   prior.score = 5 (>= query.score = 0) → dominates → IsDominated true.
//   prior.score = -1 (< query.score = 0) → score check blocks → false.
//
// Also confirms the score dim is gated:
//   include_pressure_dim=false: prior.score = -1 still dominates
//     (length dims alone suffice).
static void RunLengthHistoryPressureDimShakedown(const GCNSubtarget &st) {
  // After ScheduleNodeOnFixture(a), query (current state) has
  // end_cycle=1 and frontier {H=1, C=2, D=3}.
  // The "length-equal" prior below uses end_cycle=1 with the same
  // frontier_lbs — it ties query on every length dim.
  auto make_length_equal_prior = [](const LengthHistoryTrackerFixture &f,
                                    int score) {
    LengthHistoryTracker::Entry prior;
    prior.end_cycle = 1;
    prior.frontier_lbs = {
        {f.h->GetTopoIndex(), 1},
        {f.c->GetTopoIndex(), 2},
        {f.d->GetTopoIndex(), 3},
    };
    prior.continuous_occupancy_score = score;
    return prior;
  };

  // Case 1: include_pressure_dim=true, prior.score=5 (better than
  // query.score=0). Length dims tie; score dominates → IsDominated true.
  {
    auto fixture =
        BuildLengthHistoryTrackerFixture(st, /*include_pressure_dim=*/true);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    fixture.length_history_tracker->InsertEntryForTest(
        key, make_length_equal_prior(fixture, /*score=*/5));
    bool dominated = fixture.length_history_tracker->IsDominated();
    llvm::outs() << "    pressure_dim=true, prior.score=5 (better): "
                    "dominated="
                 << (dominated ? "true" : "false")
                 << "  " << (dominated ? "PASS\n" : "FAIL\n");
  }

  // Case 2: include_pressure_dim=true, prior.score=-1 (worse than
  // query.score=0). Length dims tie; score check blocks dominance.
  {
    auto fixture =
        BuildLengthHistoryTrackerFixture(st, /*include_pressure_dim=*/true);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    fixture.length_history_tracker->InsertEntryForTest(
        key, make_length_equal_prior(fixture, /*score=*/-1));
    bool not_dominated = !fixture.length_history_tracker->IsDominated();
    llvm::outs() << "    pressure_dim=true, prior.score=-1 (worse): "
                    "not_dominated="
                 << (not_dominated ? "true" : "false")
                 << "  " << (not_dominated ? "PASS\n" : "FAIL\n");
  }

  // Case 3: include_pressure_dim=false, prior.score=-1 (worse).
  // Score field is ignored; length dims alone determine dominance.
  // Length dims tie → prior dominates regardless of score.
  {
    auto fixture =
        BuildLengthHistoryTrackerFixture(st, /*include_pressure_dim=*/false);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    fixture.length_history_tracker->InsertEntryForTest(
        key, make_length_equal_prior(fixture, /*score=*/-1));
    bool dominated = fixture.length_history_tracker->IsDominated();
    llvm::outs() << "    pressure_dim=false, prior.score=-1 (ignored): "
                    "dominated="
                 << (dominated ? "true" : "false")
                 << "  " << (dominated ? "PASS\n" : "FAIL\n");
  }
}

// ILP dimension on dominance. With include_ilp_dim=true, dominance
// also requires (both must hold for prior to dominate query):
//   1. For every open producer R,
//      prior.inst_count[R] <= query.inst_count[R].
//   2. prior.ilp_score >= query.ilp_score.
//
// Tests stage BOTH prior (via InsertEntryForTest) and query (via
// IsDominatedByEntryForTest). This is needed because the fixture's
// LengthHistoryTracker has a null IlpTracker, so the live-trackers
// path would produce a query with empty opens and ilp_score=0 —
// which can't exercise the per-producer walk or non-trivial ILP
// score checks. Staging the query directly lets us drive all the
// new logic.
//
// Cases:
//   A. ilp_dim=true, opens tied, prior.ilp > query.ilp → dominates.
//   B. ilp_dim=true, opens tied, prior.ilp < query.ilp → not.
//   C. ilp_dim=true, prior.inst_count[R] all < query's, ilp tied
//      → dominates.
//   D. ilp_dim=true, prior.inst_count Pareto-incomparable with
//      query (one R better in prior, another better in query)
//      → not dominated.
//   E. ilp_dim=false, all ILP fields conflicting → still dominates
//      because the gate is off and length dims alone tie.
static void RunLengthHistoryIlpDimShakedown(const GCNSubtarget &st) {
  // Build an Entry with the same length-state as the query post-
  // Schedule(a), plus caller-supplied ILP fields.
  auto make_entry = [](
      const LengthHistoryTrackerFixture &f, int ilp_score,
      SmallVector<IlpTracker::OpenProducerInstCount, 16> opens) {
    LengthHistoryTracker::Entry entry;
    entry.end_cycle = 1;
    entry.frontier_lbs = {
        {f.h->GetTopoIndex(), 1},
        {f.c->GetTopoIndex(), 2},
        {f.d->GetTopoIndex(), 3},
    };
    entry.continuous_occupancy_score = 0;
    entry.ilp_score = ilp_score;
    entry.open_producer_inst_counts = std::move(opens);
    return entry;
  };

  // Two synthetic open-producer vregs used in cases below. Reg
  // values are arbitrary but stable; same set in prior and query
  // for parallel-walk correctness.
  constexpr unsigned kRegX = 100;
  constexpr unsigned kRegY = 200;

  // Case A: opens tied, prior.ilp > query.ilp → dominates.
  {
    auto fixture = BuildLengthHistoryTrackerFixture(
        st, /*include_pressure_dim=*/false, /*include_ilp_dim=*/true);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    auto prior = make_entry(fixture, /*ilp=*/5,
                            {{kRegX, 0}, {kRegY, 1}});
    auto query = make_entry(fixture, /*ilp=*/0,
                            {{kRegX, 0}, {kRegY, 1}});
    fixture.length_history_tracker->InsertEntryForTest(key,
                                                       std::move(prior));
    bool dominated =
        fixture.length_history_tracker->IsDominatedByEntryForTest(key,
                                                                  query);
    llvm::outs()
        << "    A: opens tied, prior.ilp>query.ilp: dominated="
        << (dominated ? "true" : "false") << "  "
        << (dominated ? "PASS\n" : "FAIL\n");
  }

  // Case B: opens tied, prior.ilp < query.ilp → not dominated.
  {
    auto fixture = BuildLengthHistoryTrackerFixture(
        st, /*include_pressure_dim=*/false, /*include_ilp_dim=*/true);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    auto prior = make_entry(fixture, /*ilp=*/-1,
                            {{kRegX, 0}, {kRegY, 1}});
    auto query = make_entry(fixture, /*ilp=*/0,
                            {{kRegX, 0}, {kRegY, 1}});
    fixture.length_history_tracker->InsertEntryForTest(key,
                                                       std::move(prior));
    bool not_dominated =
        !fixture.length_history_tracker->IsDominatedByEntryForTest(key,
                                                                   query);
    llvm::outs()
        << "    B: opens tied, prior.ilp<query.ilp: not_dominated="
        << (not_dominated ? "true" : "false") << "  "
        << (not_dominated ? "PASS\n" : "FAIL\n");
  }

  // Case C: prior.inst_count strictly less on every open, ilp
  // tied → dominates (prior has more future cover everywhere).
  {
    auto fixture = BuildLengthHistoryTrackerFixture(
        st, /*include_pressure_dim=*/false, /*include_ilp_dim=*/true);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    auto prior = make_entry(fixture, /*ilp=*/0,
                            {{kRegX, 0}, {kRegY, 1}});
    auto query = make_entry(fixture, /*ilp=*/0,
                            {{kRegX, 2}, {kRegY, 3}});
    fixture.length_history_tracker->InsertEntryForTest(key,
                                                       std::move(prior));
    bool dominated =
        fixture.length_history_tracker->IsDominatedByEntryForTest(key,
                                                                  query);
    llvm::outs()
        << "    C: prior.opens earlier on every R: dominated="
        << (dominated ? "true" : "false") << "  "
        << (dominated ? "PASS\n" : "FAIL\n");
  }

  // Case D: inst_counts Pareto-incomparable (X earlier in prior,
  // Y earlier in query) → prior does NOT dominate.
  {
    auto fixture = BuildLengthHistoryTrackerFixture(
        st, /*include_pressure_dim=*/false, /*include_ilp_dim=*/true);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    auto prior = make_entry(fixture, /*ilp=*/5,
                            {{kRegX, 0}, {kRegY, 5}});
    auto query = make_entry(fixture, /*ilp=*/0,
                            {{kRegX, 2}, {kRegY, 1}});
    fixture.length_history_tracker->InsertEntryForTest(key,
                                                       std::move(prior));
    bool not_dominated =
        !fixture.length_history_tracker->IsDominatedByEntryForTest(key,
                                                                   query);
    llvm::outs()
        << "    D: opens Pareto-incomparable: not_dominated="
        << (not_dominated ? "true" : "false") << "  "
        << (not_dominated ? "PASS\n" : "FAIL\n");
  }

  // Case E: ilp_dim=false. Even with prior strictly worse on
  // every ILP field, the gate is off and dominance reduces to
  // length-only — which tie, so prior dominates.
  {
    auto fixture = BuildLengthHistoryTrackerFixture(
        st, /*include_pressure_dim=*/false, /*include_ilp_dim=*/false);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    auto prior = make_entry(fixture, /*ilp=*/-5,
                            {{kRegX, 9}, {kRegY, 9}});
    auto query = make_entry(fixture, /*ilp=*/0,
                            {{kRegX, 0}, {kRegY, 0}});
    fixture.length_history_tracker->InsertEntryForTest(key,
                                                       std::move(prior));
    bool dominated =
        fixture.length_history_tracker->IsDominatedByEntryForTest(key,
                                                                  query);
    llvm::outs() << "    E: ilp_dim=false (gate off): dominated="
                 << (dominated ? "true" : "false") << "  "
                 << (dominated ? "PASS\n" : "FAIL\n");
  }
}

// Length-max-mode dominance. The min-mode shakedowns above (Strict
// dominator, Pareto trim, Incomparable, etc.) cover the default
// lower-end_cycle-and-lower-LB-dominates direction. With
// length_max_mode=true the length axes flip: higher end_cycle and
// higher frontier LBs dominate. Verify by staging the same query
// state (after Schedule(a): end_cycle=1, frontier {H=1, C=2, D=3})
// and four priors:
//   A. Max-mode, prior strictly higher (end_cycle=5, LBs all higher)
//      → dominates.
//   B. Max-mode, prior strictly lower (end_cycle=0, LBs all lower)
//      → does NOT dominate.
//   C. Cross-check: the same "prior with higher end_cycle + higher
//      LBs" entry from (A) is fed into a MIN-mode fixture. Confirms
//      direction is actually flag-controlled (not a fluke of the
//      specific values) — in min mode, this same entry should NOT
//      dominate the query.
//   D. Max-mode, prior is mixed — higher end_cycle but LOWER LB on
//      one frontier node (Pareto-incomparable). Neither side wins
//      on every axis, so it should NOT dominate. Mirrors the min-
//      mode RunLengthHistoryIncomparableShakedown.
static void RunLengthHistoryLengthMaxModeShakedown(const GCNSubtarget &st) {
  auto make_prior_for_query_state = [](const LengthHistoryTrackerFixture &f,
                                       int end_cycle, int h_lb, int c_lb,
                                       int d_lb) {
    LengthHistoryTracker::Entry prior;
    prior.end_cycle = end_cycle;
    prior.frontier_lbs = {
        {f.h->GetTopoIndex(), h_lb},
        {f.c->GetTopoIndex(), c_lb},
        {f.d->GetTopoIndex(), d_lb},
    };
    return prior;
  };

  // Case A: max-mode, prior strictly higher on every length axis.
  // Should dominate.
  {
    auto fixture = BuildLengthHistoryTrackerFixture(
        st, /*include_pressure_dim=*/false, /*include_ilp_dim=*/false,
        /*length_max_mode=*/true);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    fixture.length_history_tracker->InsertEntryForTest(
        key, make_prior_for_query_state(fixture, /*end_cycle=*/5,
                                        /*h_lb=*/3, /*c_lb=*/4, /*d_lb=*/5));
    bool dominated = fixture.length_history_tracker->IsDominated();
    llvm::outs() << "    A: max-mode, prior strictly higher: dominated="
                 << (dominated ? "true" : "false") << "  "
                 << (dominated ? "PASS\n" : "FAIL\n");
  }

  // Case B: max-mode, prior strictly lower on every length axis.
  // Should NOT dominate.
  {
    auto fixture = BuildLengthHistoryTrackerFixture(
        st, /*include_pressure_dim=*/false, /*include_ilp_dim=*/false,
        /*length_max_mode=*/true);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    fixture.length_history_tracker->InsertEntryForTest(
        key, make_prior_for_query_state(fixture, /*end_cycle=*/0,
                                        /*h_lb=*/0, /*c_lb=*/1, /*d_lb=*/2));
    bool not_dominated = !fixture.length_history_tracker->IsDominated();
    llvm::outs() << "    B: max-mode, prior strictly lower: not_dominated="
                 << (not_dominated ? "true" : "false") << "  "
                 << (not_dominated ? "PASS\n" : "FAIL\n");
  }

  // Case C: same prior as (A) (higher end_cycle, higher LBs) under
  // MIN-mode. Should NOT dominate — confirms the direction flip is
  // actually flag-controlled.
  {
    auto fixture = BuildLengthHistoryTrackerFixture(
        st, /*include_pressure_dim=*/false, /*include_ilp_dim=*/false,
        /*length_max_mode=*/false);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    fixture.length_history_tracker->InsertEntryForTest(
        key, make_prior_for_query_state(fixture, /*end_cycle=*/5,
                                        /*h_lb=*/3, /*c_lb=*/4, /*d_lb=*/5));
    bool not_dominated = !fixture.length_history_tracker->IsDominated();
    llvm::outs() << "    C: min-mode, same prior: not_dominated="
                 << (not_dominated ? "true" : "false") << "  "
                 << (not_dominated ? "PASS\n" : "FAIL\n");
  }

  // Case D: max-mode, prior is Pareto-incomparable with query —
  // higher end_cycle (good in max) but lower H_lb (bad in max).
  // Should NOT dominate. Mirrors the min-mode
  // RunLengthHistoryIncomparableShakedown.
  {
    auto fixture = BuildLengthHistoryTrackerFixture(
        st, /*include_pressure_dim=*/false, /*include_ilp_dim=*/false,
        /*length_max_mode=*/true);
    ScheduleNodeOnFixture(fixture, fixture.a);
    PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
    // end_cycle=5 (better than query's 1 in max-mode); H_lb=0
    // (worse than query's 1 in max-mode). C and D LBs match.
    // Neither side wins on every axis.
    fixture.length_history_tracker->InsertEntryForTest(
        key, make_prior_for_query_state(fixture, /*end_cycle=*/5,
                                        /*h_lb=*/0, /*c_lb=*/2, /*d_lb=*/3));
    bool not_dominated = !fixture.length_history_tracker->IsDominated();
    llvm::outs() << "    D: max-mode, mixed (incomparable): not_dominated="
                 << (not_dominated ? "true" : "false") << "  "
                 << (not_dominated ? "PASS\n" : "FAIL\n");
  }
}

void RunLengthHistoryTrackerShakedown(const GCNSubtarget &st) {
  llvm::outs() << "  RunLengthHistoryTrackerShakedown:\n";
  RunLengthHistoryEmptyShakedown(st);
  RunLengthHistorySnapshotShakedown(st);
  RunLengthHistoryFirstInsertAndSelfDominanceShakedown(st);
  RunLengthHistoryStrictDominatorShakedown(st);
  RunLengthHistoryParetoTrimShakedown(st);
  RunLengthHistoryIncomparableShakedown(st);
  RunLengthHistoryDistinctPartitionsShakedown(st);
  RunLengthHistoryHashCollisionShakedown(st);
  RunLengthHistoryPressureDimShakedown(st);
  RunLengthHistoryIlpDimShakedown(st);
  RunLengthHistoryLengthMaxModeShakedown(st);
}

// =============================================================================
// PressureHistoryTracker shakedowns
// =============================================================================
//
// Standalone tests for PressureHistoryTracker's prefix-side logic.
// The tracker's explicit-Score overload of IsDominatedElseRecord
// is metric-agnostic -- it just compares Scores -- so these tests
// build Scores via the local helpers below and don't need a real
// ScheduleConstructor. The tracker is constructed with nullptr SC
// pointer; the no-arg overload would fatal-error in this
// configuration but isn't called here.

// Test helpers: build a 1-slot or 2-slot Score from raw ints, with
// the same Higher orientation production uses for peak / area.
static Score MakePeakOnlyTestScore(int peak) {
  return Score::Make(Score::Higher{peak});
}
static Score MakePeakAreaTestScore(int peak, int64_t area) {
  return Score::Make(Score::Higher{peak}, Score::Higher{area});
}

struct PressureHistoryTrackerFixture {
  std::unique_ptr<ScheduleGraph> graph;
  std::unique_ptr<ScheduleLengthTracker> length_tracker;
  std::unique_ptr<ScheduledSetTracker> scheduled_set_tracker;
  std::unique_ptr<PressureHistoryTracker> pressure_history_tracker;
  // Common BuildTestDAG node handles. Topo indices: A=0, H=1,
  // C=2, D=3.
  ScheduleNode *a;
  ScheduleNode *h;
  ScheduleNode *c;
  ScheduleNode *d;
};

static PressureHistoryTrackerFixture
BuildPressureHistoryTrackerFixture(const GCNSubtarget &st) {
  PressureHistoryTrackerFixture fixture;
  fixture.graph = ScheduleGraph::BuildTestDAG();
  fixture.graph->ValidateAndComputeTopologicalOrder();
  fixture.graph->ComputeCriticalPaths();
  ArrayRef<ScheduleNode *> topo = fixture.graph->GetTopoOrder();
  fixture.a = topo[0];
  fixture.h = topo[1];
  fixture.c = topo[2];
  fixture.d = topo[3];
  fixture.length_tracker =
      std::make_unique<ScheduleLengthTracker>(*fixture.graph, st);
  fixture.scheduled_set_tracker = std::make_unique<ScheduledSetTracker>(
      fixture.graph.get(), fixture.length_tracker.get());
  fixture.pressure_history_tracker = std::make_unique<PressureHistoryTracker>(
      fixture.scheduled_set_tracker.get(),
      /*working_schedule_constructor=*/nullptr,
      ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore);
  return fixture;
}

// Drive both length and scheduled-set trackers in lockstep, matching
// the production order in ScheduleConstructor::ScheduleByIndex.
static void
ScheduleNodeOnPressureFixture(PressureHistoryTrackerFixture &fixture,
                              ScheduleNode *node) {
  fixture.length_tracker->Schedule(node);
  fixture.scheduled_set_tracker->Schedule(node);
}

// Test 1: Empty-table behavior.
// After construction, total_entries == 0 and prune_count == 0.
static void RunPressureHistoryEmptyShakedown(const GCNSubtarget &st) {
  auto fixture = BuildPressureHistoryTrackerFixture(st);
  bool initially_empty =
      fixture.pressure_history_tracker->GetTotalEntries() == 0 &&
      fixture.pressure_history_tracker->PruneCount().lifetime == 0;
  llvm::outs() << "    Empty-table behavior: "
               << (initially_empty ? "PASS\n" : "FAIL\n");
}

// Test 2: First IsDominatedElseRecord from empty inserts; second
// call with the same scores is dominated by what we just inserted
// (equality dominates: prior >= current at equal scores).
static void RunPressureHistoryFirstInsertAndSelfDominanceShakedown(
    const GCNSubtarget &st) {
  auto fixture = BuildPressureHistoryTrackerFixture(st);
  ScheduleNodeOnPressureFixture(fixture, fixture.a);

  bool first_call_inserted =
      !fixture.pressure_history_tracker->IsDominatedElseRecord(
          MakePeakOnlyTestScore(100));
  bool count_one_after_first =
      fixture.pressure_history_tracker->GetTotalEntries() == 1;

  // Same partition, same Score -- prior dominates (lex equality).
  bool second_call_pruned =
      fixture.pressure_history_tracker->IsDominatedElseRecord(
          MakePeakOnlyTestScore(100));
  bool count_unchanged_after_second =
      fixture.pressure_history_tracker->GetTotalEntries() == 1;
  bool prune_count_one =
      fixture.pressure_history_tracker->PruneCount().lifetime == 1;

  bool ok = first_call_inserted && count_one_after_first &&
            second_call_pruned && count_unchanged_after_second &&
            prune_count_one;
  llvm::outs() << "    First insert + self-dominance: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 3: Strict prior dominator. Stage an entry with a strictly
// higher peak Score than the query. IsDominatedElseRecord returns
// true; entry unchanged; prune count incremented.
static void
RunPressureHistoryStrictPriorDominatorShakedown(const GCNSubtarget &st) {
  auto fixture = BuildPressureHistoryTrackerFixture(st);
  ScheduleNodeOnPressureFixture(fixture, fixture.a);
  PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();

  PressureHistoryTracker::Entry strict_dominator;
  strict_dominator.best_score = MakePeakOnlyTestScore(200);
  fixture.pressure_history_tracker->InsertEntryForTest(key, strict_dominator);

  // Query with strictly lower peak. Prior 200 dominates current 100
  // on the single populated slot -> prune.
  bool pruned = fixture.pressure_history_tracker->IsDominatedElseRecord(
      MakePeakOnlyTestScore(100));
  ArrayRef<PressureHistoryTracker::Entry> after =
      fixture.pressure_history_tracker->GetBucketForTest(key);
  bool bucket_unchanged =
      after.size() == 1 &&
      after[0].best_score == MakePeakOnlyTestScore(200);
  bool count_one =
      fixture.pressure_history_tracker->GetTotalEntries() == 1;
  bool prune_count_one =
      fixture.pressure_history_tracker->PruneCount().lifetime == 1;

  bool ok =
      pruned && bucket_unchanged && count_one && prune_count_one;
  llvm::outs() << "    Strict prior dominator pruning: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 4: Strict update when current is better. Stage an entry
// with a strictly lower peak Score than the query.
// IsDominatedElseRecord returns false; entry's Score is updated to
// the larger value; prune count unchanged.
static void RunPressureHistoryStrictCurrentBetterShakedown(
    const GCNSubtarget &st) {
  auto fixture = BuildPressureHistoryTrackerFixture(st);
  ScheduleNodeOnPressureFixture(fixture, fixture.a);
  PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();

  PressureHistoryTracker::Entry strictly_worse_prior;
  strictly_worse_prior.best_score = MakePeakOnlyTestScore(50);
  fixture.pressure_history_tracker->InsertEntryForTest(
      key, strictly_worse_prior);

  // Query with strictly higher current. Current 200 dominates prior
  // 50 -> not pruned; current inserted; Pareto trim removes prior;
  // bucket ends with only the new entry.
  bool not_pruned = !fixture.pressure_history_tracker->IsDominatedElseRecord(
      MakePeakOnlyTestScore(200));
  ArrayRef<PressureHistoryTracker::Entry> after =
      fixture.pressure_history_tracker->GetBucketForTest(key);
  bool bucket_replaced =
      after.size() == 1 &&
      after[0].best_score == MakePeakOnlyTestScore(200);
  bool count_one =
      fixture.pressure_history_tracker->GetTotalEntries() == 1;
  bool prune_count_zero =
      fixture.pressure_history_tracker->PruneCount().lifetime == 0;

  bool ok =
      not_pruned && bucket_replaced && count_one && prune_count_zero;
  llvm::outs() << "    Strict current-better trims prior + inserts: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 5: Distinct schedule states produce distinct PartitionKeys
// and the table holds them as separate entries. Uses {A} → {A, H}
// (both reachable via valid scheduling order) since H requires A
// scheduled first for ScheduleLengthTracker invariants.
static void
RunPressureHistoryDistinctPartitionsShakedown(const GCNSubtarget &st) {
  auto fixture = BuildPressureHistoryTrackerFixture(st);

  // Partition 1: {A} scheduled.
  ScheduleNodeOnPressureFixture(fixture, fixture.a);
  bool a_inserted =
      !fixture.pressure_history_tracker->IsDominatedElseRecord(
          MakePeakOnlyTestScore(100));

  // Partition 2: {A, H} scheduled.
  ScheduleNodeOnPressureFixture(fixture, fixture.h);
  bool ah_inserted =
      !fixture.pressure_history_tracker->IsDominatedElseRecord(
          MakePeakOnlyTestScore(200));

  bool count_two =
      fixture.pressure_history_tracker->GetTotalEntries() == 2;
  bool prune_count_zero =
      fixture.pressure_history_tracker->PruneCount().lifetime == 0;

  bool ok = a_inserted && ah_inserted && count_two && prune_count_zero;
  llvm::outs() << "    Distinct partitions get distinct entries: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Test 6: Hash collision across distinct partitions. Hand-craft
// two PartitionKeys with the same signature but different bitsets.
// DenseMap probing + DenseMapInfo<PartitionKey>::isEqual (which
// compares the full bitset) must keep the entries separate.
static void
RunPressureHistoryHashCollisionShakedown(const GCNSubtarget &st) {
  auto fixture = BuildPressureHistoryTrackerFixture(st);
  int n = fixture.graph->Size();

  // Two keys, same signature (42), different bitsets. Bitset sizes
  // match graph.Size() (n >= 2) so neither hits the sentinel sizes.
  PartitionKey key1{42, BitVector(n)};
  key1.scheduled_set.set(0);
  PartitionKey key2{42, BitVector(n)};
  key2.scheduled_set.set(1);

  PressureHistoryTracker::Entry entry1;
  entry1.best_score = MakePeakOnlyTestScore(100);
  PressureHistoryTracker::Entry entry2;
  entry2.best_score = MakePeakOnlyTestScore(200);
  fixture.pressure_history_tracker->InsertEntryForTest(key1, entry1);
  fixture.pressure_history_tracker->InsertEntryForTest(key2, entry2);

  ArrayRef<PressureHistoryTracker::Entry> got1 =
      fixture.pressure_history_tracker->GetBucketForTest(key1);
  ArrayRef<PressureHistoryTracker::Entry> got2 =
      fixture.pressure_history_tracker->GetBucketForTest(key2);
  bool both_present = got1.size() == 1 && got2.size() == 1;
  bool entries_in_correct_buckets =
      both_present && got1[0].best_score == MakePeakOnlyTestScore(100) &&
      got2[0].best_score == MakePeakOnlyTestScore(200);
  bool count_two =
      fixture.pressure_history_tracker->GetTotalEntries() == 2;

  bool ok = both_present && entries_in_correct_buckets && count_two;
  llvm::outs() << "    Hash collision across distinct partitions: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Area test 1: the (peak, area) overload records area on first insert,
// and the same (peak, area) self-dominates (lexicographic >= prunes on
// equality).
static void RunPressureHistoryAreaFirstInsertAndSelfDominanceShakedown(
    const GCNSubtarget &st) {
  auto fixture = BuildPressureHistoryTrackerFixture(st);
  ScheduleNodeOnPressureFixture(fixture, fixture.a);

  bool inserted = !fixture.pressure_history_tracker->IsDominatedElseRecord(
      MakePeakAreaTestScore(100, 500));
  PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();
  ArrayRef<PressureHistoryTracker::Entry> bucket =
      fixture.pressure_history_tracker->GetBucketForTest(key);
  bool area_recorded = bucket.size() == 1 &&
                       bucket[0].best_score ==
                           MakePeakAreaTestScore(100, 500);

  // Same (peak, area) -- prior dominates on every slot (>= on both)
  // so the query is Pareto-dominated.
  bool self_pruned = fixture.pressure_history_tracker->IsDominatedElseRecord(
      MakePeakAreaTestScore(100, 500));
  bool prune_count_one =
      fixture.pressure_history_tracker->PruneCount().lifetime == 1;

  bool ok = inserted && area_recorded && self_pruned && prune_count_one;
  llvm::outs() << "    Area first insert + self-dominance: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Area test 2: same-peak ties broken by area. Prior (100, 500). A
// lower-area query at the same peak is dominated (pruned, entry kept);
// a higher-area query at the same peak is not dominated and updates the
// recorded area (peak stays).
static void RunPressureHistoryAreaTiebreakShakedown(const GCNSubtarget &st) {
  auto fixture = BuildPressureHistoryTrackerFixture(st);
  ScheduleNodeOnPressureFixture(fixture, fixture.a);
  PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();

  PressureHistoryTracker::Entry prior;
  prior.best_score = MakePeakAreaTestScore(100, 500);
  fixture.pressure_history_tracker->InsertEntryForTest(key, prior);

  // Same peak, lower area (300): prior (100, 500) dominates (100,
  // 300) on every slot -> prune.
  bool lower_area_pruned =
      fixture.pressure_history_tracker->IsDominatedElseRecord(
          MakePeakAreaTestScore(100, 300));
  ArrayRef<PressureHistoryTracker::Entry> after_low =
      fixture.pressure_history_tracker->GetBucketForTest(key);
  bool unchanged =
      after_low.size() == 1 &&
      after_low[0].best_score == MakePeakAreaTestScore(100, 500);

  // Same peak, higher area (700): not dominated; (100, 700)
  // dominates prior (100, 500); Pareto trim drops prior; bucket
  // ends with only the new entry.
  bool higher_area_not_pruned =
      !fixture.pressure_history_tracker->IsDominatedElseRecord(
          MakePeakAreaTestScore(100, 700));
  ArrayRef<PressureHistoryTracker::Entry> after_high =
      fixture.pressure_history_tracker->GetBucketForTest(key);
  bool updated =
      after_high.size() == 1 &&
      after_high[0].best_score == MakePeakAreaTestScore(100, 700);

  bool ok = lower_area_pruned && unchanged && higher_area_not_pruned && updated;
  llvm::outs() << "    Area same-peak Pareto trim: "
               << (ok ? "PASS\n" : "FAIL\n");
}

// Area test 3: Pareto retention of incomparable entries. This is
// the scenario where the prior (lex-collapse) PHT was unsound -- it
// would have pruned the (lower-peak, higher-area) query even though
// neither it nor the (higher-peak, lower-area) prior dominates the
// other on every slot. Pareto keeps both.
//
// Then a third entry that strictly dominates the (200, 0) prior on
// every slot is inserted; Pareto trim removes (200, 0) but leaves
// (100, 9999) -- the third entry doesn't dominate it on area.
static void
RunPressureHistoryAreaIncomparableRetentionShakedown(
    const GCNSubtarget &st) {
  auto fixture = BuildPressureHistoryTrackerFixture(st);
  ScheduleNodeOnPressureFixture(fixture, fixture.a);
  PartitionKey key = fixture.scheduled_set_tracker->GetPartitionKey();

  PressureHistoryTracker::Entry prior;
  prior.best_score = MakePeakAreaTestScore(200, 0);
  fixture.pressure_history_tracker->InsertEntryForTest(key, prior);

  // (100, 9999) vs prior (200, 0): higher area, lower peak. Neither
  // Pareto-dominates the other (each loses on a different slot) --
  // both are kept on the frontier.
  bool incomparable_not_pruned =
      !fixture.pressure_history_tracker->IsDominatedElseRecord(
          MakePeakAreaTestScore(100, 9999));
  ArrayRef<PressureHistoryTracker::Entry> after_incomparable =
      fixture.pressure_history_tracker->GetBucketForTest(key);
  bool both_retained = after_incomparable.size() == 2;

  // (300, 0) vs both: dominates prior (200, 0) on every slot
  // (300>=200, 0>=0). Does NOT dominate (100, 9999) (0 < 9999 on
  // area). Pareto trim removes (200, 0), keeps (100, 9999), inserts
  // (300, 0). Final bucket size 2: {(100, 9999), (300, 0)}.
  bool higher_peak_not_pruned =
      !fixture.pressure_history_tracker->IsDominatedElseRecord(
          MakePeakAreaTestScore(300, 0));
  ArrayRef<PressureHistoryTracker::Entry> after_third =
      fixture.pressure_history_tracker->GetBucketForTest(key);
  bool size_two = after_third.size() == 2;
  bool contains_100_9999 = false;
  bool contains_300_0 = false;
  bool contains_200_0 = false;
  for (const auto &entry : after_third) {
    if (entry.best_score == MakePeakAreaTestScore(100, 9999)) {
      contains_100_9999 = true;
    }
    if (entry.best_score == MakePeakAreaTestScore(300, 0)) {
      contains_300_0 = true;
    }
    if (entry.best_score == MakePeakAreaTestScore(200, 0)) {
      contains_200_0 = true;
    }
  }
  bool pareto_correct = size_two && contains_100_9999 &&
                        contains_300_0 && !contains_200_0;

  bool ok = incomparable_not_pruned && both_retained &&
            higher_peak_not_pruned && pareto_correct;
  llvm::outs() << "    Area incomparable Pareto retention: "
               << (ok ? "PASS\n" : "FAIL\n");
}

void RunPressureHistoryTrackerShakedown(const GCNSubtarget &st) {
  llvm::outs() << "  RunPressureHistoryTrackerShakedown:\n";
  RunPressureHistoryEmptyShakedown(st);
  RunPressureHistoryFirstInsertAndSelfDominanceShakedown(st);
  RunPressureHistoryStrictPriorDominatorShakedown(st);
  RunPressureHistoryStrictCurrentBetterShakedown(st);
  RunPressureHistoryDistinctPartitionsShakedown(st);
  RunPressureHistoryHashCollisionShakedown(st);
  RunPressureHistoryAreaFirstInsertAndSelfDominanceShakedown(st);
  RunPressureHistoryAreaTiebreakShakedown(st);
  RunPressureHistoryAreaIncomparableRetentionShakedown(st);
}

// Test policies for the history-vs-no-history comparison shakedown.
// Both inherit DfsMinimizeLengthPolicy and override ShouldBoundSearch
// to skip the production LB + occupancy bounds entirely. The two
// variants differ only in whether ShouldBoundSearch consults the
// length history tracker. With LB + occupancy bounds disabled, the
// schedule_call_count gap between the variants is attributable to
// history pruning alone.
//
// ShouldEndSearch is inherited from DfsMinimizeLengthPolicy
// (terminates when best matches the length floor). The DAG used by
// the comparison (BuildHistoryPruneTestDAG) has optimum > floor, so
// end-search never fires and DFS exhausts — leaving room for
// history pruning to demonstrate value.
class TestLengthPolicyNoBoundsNoHistory : public DfsMinimizeLengthPolicy {
 public:
  static constexpr bool kUseLengthHistoryPruning = false;
  static bool ShouldBoundSearch(const ScheduleConstructor &,
                                const ScheduleConstructor &,
                                LengthHistoryTracker &,
                                PressureHistoryTracker &) {
    return false;
  }
};

class TestLengthPolicyNoBoundsWithHistory : public DfsMinimizeLengthPolicy {
 public:
  static constexpr bool kUseLengthHistoryPruning = true;
  static bool ShouldBoundSearch(const ScheduleConstructor &,
                                const ScheduleConstructor &,
                                LengthHistoryTracker &length_history,
                                PressureHistoryTracker &) {
    return length_history.IsDominatedElseInsert();
  }
};

// End-to-end comparison shakedown for length history-based
// domination. Runs DfsSearch on BuildHistoryPruneTestDAG twice —
// once with history pruning, once without — using test policies
// that disable LB and occupancy bounds so the difference between
// the runs isolates history pruning.
//
// Verifies:
//   - Both produce the same best schedule length (soundness — the
//     history prune doesn't lose optimal completions).
//   - The history tracker fired at least one prune (count > 0).
//   - When prunes fired, the history version made strictly fewer
//     Schedule calls (each prune fires at a non-leaf prefix and
//     skips at least one child Schedule call that the no-history
//     version takes).
//
// Neither run forms subgraphs, so the comparison stays stable across
// formation-policy changes.
void RunLengthHistoryDfsComparisonShakedown(const GCNSubtarget &st,
                                            const MachineFunction &mf,
                                            const LiveIntervals &lis) {
  llvm::outs() << "  RunLengthHistoryDfsComparisonShakedown:\n";

  auto graph = ScheduleGraph::BuildHistoryPruneTestDAG();
  graph->ValidateAndComputeTopologicalOrder();
  graph->ComputeCriticalPaths();
  graph->PopulateInputScheduleConstructorByTopoOrderForTest(st, mf);

  DfsSearch<TestLengthPolicyNoBoundsNoHistory> no_hist_search(
      *graph, st, mf, lis);
  ScheduleConstructor no_hist_best =
      std::move(*no_hist_search.Run().schedule);
  int no_hist_length =
      no_hist_best.GetLengthTracker().GetCurrentCycle();
  int64_t no_hist_calls = no_hist_search.ScheduleCallCount().lifetime;

  DfsSearch<TestLengthPolicyNoBoundsWithHistory> hist_search(
      *graph, st, mf, lis);
  ScheduleConstructor hist_best = std::move(*hist_search.Run().schedule);
  int hist_length = hist_best.GetLengthTracker().GetCurrentCycle();
  int64_t hist_calls = hist_search.ScheduleCallCount().lifetime;
  int hist_prunes =
      hist_search.GetLengthHistoryTracker().PruneCount().lifetime;

  llvm::outs() << "    no-history: length=" << no_hist_length
               << " schedule_calls=" << no_hist_calls << "\n";
  llvm::outs() << "    history:    length=" << hist_length
               << " schedule_calls=" << hist_calls
               << " prunes=" << hist_prunes << "\n";

  bool same_length = no_hist_length == hist_length;
  bool prunes_fired = hist_prunes > 0;
  // If prunes fired, calls must be strictly less.
  bool calls_strictly_less_when_pruning =
      !prunes_fired || hist_calls < no_hist_calls;

  llvm::outs() << "    Same best length (soundness): "
               << (same_length ? "PASS\n" : "FAIL\n");
  llvm::outs() << "    History pruning fired (count > 0): "
               << (prunes_fired ? "PASS\n" : "FAIL\n");
  llvm::outs() << "    History calls strictly less when pruning: "
               << (calls_strictly_less_when_pruning ? "PASS\n" : "FAIL\n");
}

// =============================================================================
// PressureHistoryTracker DFS comparison shakedown
// =============================================================================
//
// Test policies for pressure-history-vs-no-pressure-history. Both
// inherit DfsMaximizeOccupancyPolicy and override
// ShouldBoundSearch / ShouldEndSearch to disable production
// bounds. The two variants differ only in whether ShouldBoundSearch
// consults the pressure-history tracker. With production bounds
// off, the schedule_call_count gap between the variants is
// attributable to pressure-history pruning alone.

class TestPressurePolicyNoBoundsNoHistory
    : public DfsMaximizeOccupancyPolicy {
 public:
  static constexpr bool kUsePressureHistoryPruning = false;
  static bool ShouldBoundSearch(const ScheduleConstructor &,
                                const ScheduleConstructor &,
                                LengthHistoryTracker &,
                                PressureHistoryTracker &) {
    return false;
  }
  static bool ShouldEndSearch(const ScheduleConstructor &,
                              const ScheduleConstructor &) {
    return false;
  }
};

class TestPressurePolicyNoBoundsWithHistory
    : public DfsMaximizeOccupancyPolicy {
 public:
  static constexpr bool kUsePressureHistoryPruning = true;
  static bool ShouldBoundSearch(
      const ScheduleConstructor &,
      const ScheduleConstructor &,
      LengthHistoryTracker &,
      PressureHistoryTracker &pressure_history) {
    return pressure_history.IsDominatedElseRecord();
  }
  static bool ShouldEndSearch(const ScheduleConstructor &,
                              const ScheduleConstructor &) {
    return false;
  }
};

// End-to-end comparison shakedown for pressure history-based
// domination. Runs DfsSearch on BuildPressureHistoryPruneTestDAG
// twice — once with history pruning, once without — using test
// policies that inherit DfsMaximizeOccupancyPolicy (so kMetric =
// ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore) and
// override ShouldBoundSearch / ShouldEndSearch to disable production
// bounds + the IsAtOrAbove end-search. The difference between the
// runs is therefore attributable to pressure-history pruning alone.
// best_'s peak is seeded to a deliberately-bad value (255 VGPRs by
// default) so any synthetic working completion beats it on the
// first IsBetterThan, letting both runs exercise real best-update
// behavior.
//
// Synthetic VGPR pressure is driven via GCNRegisterTracker's
// test mode. Per-node deltas are indexed by topo idx
// (A=0, B=1, C=2, D=3, E=4, F=5):
//
//   A=+1, B=+1, C=+1, D=-1, E=-1, F=-1
//
// VGPR cumulative trace per ordering:
//   [A,B,C,D,E,F]: 1, 2, 3, 2, 1, 0  → peak 3
//   [A,B,C,E,D,F]: 1, 2, 3, 2, 1, 0  → peak 3
//   [A,B,D,C,E,F]: 1, 2, 1, 2, 1, 0  → peak 2
//   [A,C,B,D,E,F]: 1, 2, 3, 2, 1, 0  → peak 3
//   [A,C,B,E,D,F]: 1, 2, 3, 2, 1, 0  → peak 3
//   [A,C,E,B,D,F]: 1, 2, 1, 2, 1, 0  → peak 2
//
// Min peak across orderings = 2 (the optimum under
// kMaximizeContinuousRegisterOccupancyScore: lower peak → higher
// score). Both runs should converge to best_score(peak=2).
// Multiple orderings reach the same partition with different
// running peaks, so prefix-dominance pruning fires at several
// partitions during the with-history run.
//
// Verifies:
//   - Same best score in both runs (soundness — history pruning
//     doesn't lose the peak=2 optimum).
//   - The history tracker fires at least one prune.
//   - With history, schedule_call_count is strictly less than
//     no-history.
void RunPressureHistoryDfsComparisonShakedown(const GCNSubtarget &st,
                                              const MachineFunction &mf,
                                              const LiveIntervals &lis) {
  llvm::outs() << "  RunPressureHistoryDfsComparisonShakedown:\n";

  // kMetric is inherited from DfsMaximizeOccupancyPolicy on both
  // test policies; reference it explicitly here for the post-Run
  // score read.
  constexpr ScheduleMetric kPolicyMetric =
      ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore;

  auto graph = ScheduleGraph::BuildPressureHistoryPruneTestDAG();
  graph->ValidateAndComputeTopologicalOrder();
  graph->ComputeCriticalPaths();
  graph->PopulateInputScheduleConstructorByTopoOrderForTest(st, mf);

  std::vector<int> vgpr_deltas = {+1, +1, +1, -1, -1, -1};

  DfsSearch<TestPressurePolicyNoBoundsNoHistory> no_hist_search(
      *graph, st, mf, lis);
  no_hist_search.EnableTestModeForTest(vgpr_deltas);
  ScheduleConstructor no_hist_best =
      std::move(*no_hist_search.Run().schedule);
  int64_t no_hist_calls = no_hist_search.ScheduleCallCount().lifetime;
  int no_hist_score =
      no_hist_best.GetPressureTracker().GetMetricScore(kPolicyMetric);

  DfsSearch<TestPressurePolicyNoBoundsWithHistory> hist_search(
      *graph, st, mf, lis);
  hist_search.EnableTestModeForTest(vgpr_deltas);
  ScheduleConstructor hist_best = std::move(*hist_search.Run().schedule);
  int64_t hist_calls = hist_search.ScheduleCallCount().lifetime;
  int hist_score =
      hist_best.GetPressureTracker().GetMetricScore(kPolicyMetric);
  int hist_prunes =
      hist_search.GetPressureHistoryTracker().PruneCount().lifetime;

  llvm::outs() << "    no-history: best_score=" << no_hist_score
               << " schedule_calls=" << no_hist_calls << "\n";
  llvm::outs() << "    history:    best_score=" << hist_score
               << " schedule_calls=" << hist_calls
               << " prunes=" << hist_prunes << "\n";

  bool same_score = no_hist_score == hist_score;
  bool prunes_fired = hist_prunes > 0;
  bool calls_strictly_less_when_pruning =
      !prunes_fired || hist_calls < no_hist_calls;

  llvm::outs() << "    Same best score (soundness): "
               << (same_score ? "PASS\n" : "FAIL\n");
  llvm::outs() << "    History pruning fired (count > 0): "
               << (prunes_fired ? "PASS\n" : "FAIL\n");
  llvm::outs() << "    History calls strictly less when pruning: "
               << (calls_strictly_less_when_pruning ? "PASS\n" : "FAIL\n");
}

// DFS oracle policy for RunBfsDpVsDfsShakedown. Inherits the
// production occupancy policy but disables two early-exit conditions
// that would prevent the search from reaching the true optimum:
//
//   1. Per-region timeout (nullopt). The synthetic test DAG is tiny
//      so DFS finishes in well under a millisecond, but disabling
//      structurally rather than relying on a numerically-large value.
//   2. ShouldEndSearch's "best meets function occupancy target"
//      short-circuit. The synthetic VGPR deltas are tiny relative to
//      any plausible target, so the first complete schedule found
//      can easily meet target before the optimum is discovered.
//      We want the search to keep going until it has truly visited
//      every reachable partition.
//
// ShouldBoundSearch is left at the production default — its
// score-bound prune is sound and accelerates the search.
class BfsDpVsDfsShakedownOraclePolicy : public DfsMaximizeOccupancyPolicy {
 public:
  static bool ShouldEndSearch(const ScheduleConstructor &,
                              const ScheduleConstructor &) {
    return false;
  }
};

// Same as BfsDpVsDfsShakedownOraclePolicy but with pressure-history
// pruning disabled. Used to isolate how much work is being saved by
// the score-bound prune alone vs. how much depends on history-
// dominance. ShouldBoundSearch is re-derived (not inherited) so it
// doesn't consult the history tracker.
class BfsDpVsDfsShakedownOracleNoHistoryPolicy
    : public DfsMaximizeOccupancyPolicy {
 public:
  static constexpr bool kUsePressureHistoryPruning = false;
  static bool ShouldEndSearch(const ScheduleConstructor &,
                              const ScheduleConstructor &) {
    return false;
  }
  static bool ShouldBoundSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor,
      LengthHistoryTracker & /*length_history*/,
      PressureHistoryTracker & /*pressure_history*/) {
    constexpr ScheduleMetric kMetric =
        ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore;
    return schedule_constructor.GetPressureTracker().GetMetricScore(kMetric) <=
           best_schedule_constructor.GetPressureTracker().GetMetricScore(
               kMetric);
  }
};

// Per-graph driver for the BFS-DP comparison shakedown. Runs:
//   - DFS oracle (continuous metric) — gold-standard optimum.
//   - DFS no-history-prune variant — isolates score-bound prune
//     contribution.
//   - BFS-DP continuous unseeded — finds the optimum from scratch;
//     soundness anchor (must match DFS).
//   - BFS-DP continuous seeded with DFS_continuous - 1 — only the
//     optimum can strictly beat this seed, so BFS-DP must recover
//     exactly the optimum.
//   - BFS-DP continuous seeded with DFS_continuous — under <=
//     semantics, no path strictly beats the optimum; Build returns
//     false. Exercises the "no improvement" path.
//   - BFS-DP integer unseeded — finds the same integer optimum
//     (different metric, but both monotone in peak pressure).
//   - BFS-DP integer seeded with DFS_integer — also no improvement.
//
// Caveat: BFS-DP integer is only guaranteed to match DFS's integer
// score, NOT DFS's continuous score or peak VGPR — multiple
// distinct peaks can fall in the same integer occupancy bracket and
// BFS-DP integer treats them as equivalent.
//
// `graph` is mutated (topo computation + critical paths +
// input-schedule populate); caller retains ownership.
void RunBfsDpVsDfsComparisonOnGraph(StringRef case_name,
                                    ScheduleGraph &graph,
                                    const std::vector<int> &vgpr_deltas,
                                    const GCNSubtarget &st,
                                    const MachineFunction &mf,
                                    const LiveIntervals &lis) {
  llvm::outs() << "  RunBfsDpVsDfsShakedown[" << case_name << "]:\n";

  constexpr ScheduleMetric kContinuous =
      ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore;
  constexpr ScheduleMetric kInteger =
      ScheduleMetric::kMaximizeRegisterOccupancy;

  graph.ValidateAndComputeTopologicalOrder();
  graph.ComputeCriticalPaths();
  graph.PopulateInputScheduleConstructorByTopoOrderForTest(st, mf);

  // DFS oracle: production policy with timeout + early-exit disabled
  // (see BfsDpVsDfsShakedownOraclePolicy). Pruning stays on; it's
  // sound, so this finds the global optimum.
  DfsSearch<BfsDpVsDfsShakedownOraclePolicy> dfs_search(
      graph, st, mf, lis,
      /*timeout_ms=*/std::nullopt);
  dfs_search.EnableTestModeForTest(vgpr_deltas);
  ScheduleConstructor dfs_best = std::move(*dfs_search.Run().schedule);
  int dfs_continuous =
      dfs_best.GetPressureTracker().GetMetricScore(kContinuous);
  int dfs_integer =
      dfs_best.GetPressureTracker().GetMetricScore(kInteger);
  unsigned dfs_peak =
      dfs_best.GetPressureTracker().GetPeakPressure().getVGPRNum(
          st.hasGFX90AInsts());
  llvm::outs() << "    DFS oracle: peak_vgpr=" << dfs_peak
               << " continuous=" << dfs_continuous
               << " integer=" << dfs_integer
               << " schedule_calls=" << dfs_search.ScheduleCallCount().lifetime
               << " history_prunes="
               << dfs_search.GetPressureHistoryTracker().PruneCount().lifetime
               << "\n";

  // DFS variant with history pruning off — isolates score-bound prune.
  DfsSearch<BfsDpVsDfsShakedownOracleNoHistoryPolicy> dfs_no_history(
      graph, st, mf, lis,
      /*timeout_ms=*/std::nullopt);
  dfs_no_history.EnableTestModeForTest(vgpr_deltas);
  dfs_no_history.Run();
  llvm::outs() << "    DFS oracle (no history pruning): schedule_calls="
               << dfs_no_history.ScheduleCallCount().lifetime << "\n";

  // BFS-DP continuous, unseeded — soundness anchor: must match DFS.
  BfsDpSearch bfs_cont(&graph, &st, &mf, BfsDpSettings{kContinuous});
  bfs_cont.EnableTestModeForTest(vgpr_deltas);
  if (!bfs_cont.Run().schedule) {
    report_fatal_error("RunBfsDpVsDfsShakedown[" + case_name +
                       "]: unseeded BFS-DP continuous found no sink");
  }
  const PartitionDag &bfs_cont_dag = bfs_cont.GetDagForTest();
  int bfs_cont_score = bfs_cont_dag.GetSink()->best_path_bottleneck.score;
  llvm::outs() << "    BFS-DP continuous (unseeded): score="
               << bfs_cont_score
               << " peak_vgpr="
               << bfs_cont_dag.GetSink()
                      ->best_path_bottleneck.register_pressure
                      .getVGPRNum(st.hasGFX90AInsts())
               << " schedule_calls=" << bfs_cont_dag.GetScheduleCallCount()
               << " partitions=" << bfs_cont_dag.GetPartitionNodeCount()
               << " levels=" << bfs_cont_dag.GetCurrentLevel() << "\n";

  // BFS-DP continuous seeded just below the optimum. The optimum is
  // the only target that can strictly beat the seed; this demonstrates
  // pruning at the boundary AND that the optimum is recovered when
  // only it can survive.
  BfsDpSearch bfs_cont_seed_below(&graph, &st, &mf,
                                  BfsDpSettings{kContinuous});
  bfs_cont_seed_below.EnableTestModeForTest(vgpr_deltas);
  bfs_cont_seed_below.SetInitialBestScore(dfs_continuous - 1);
  bool bfs_cont_seed_below_found =
      bfs_cont_seed_below.Run().schedule.has_value();
  const PartitionDag &bfs_cont_seed_below_dag =
      bfs_cont_seed_below.GetDagForTest();
  llvm::outs() << "    BFS-DP continuous (seeded=" << (dfs_continuous - 1)
               << " [DFS optimum - 1]): found_improvement="
               << bfs_cont_seed_below_found;
  if (bfs_cont_seed_below_found) {
    llvm::outs() << " score="
                 << bfs_cont_seed_below_dag.GetSink()
                        ->best_path_bottleneck.score;
  }
  llvm::outs() << " schedule_calls="
               << bfs_cont_seed_below_dag.GetScheduleCallCount()
               << " partitions=" << bfs_cont_seed_below_dag.GetPartitionNodeCount()
               << " prunes=" << bfs_cont_seed_below_dag.GetPruneCount()
               << "\n";

  // BFS-DP continuous seeded AT the optimum. <= prunes the optimum
  // too; expect no sink.
  BfsDpSearch bfs_cont_seed_opt(&graph, &st, &mf,
                                BfsDpSettings{kContinuous});
  bfs_cont_seed_opt.EnableTestModeForTest(vgpr_deltas);
  bfs_cont_seed_opt.SetInitialBestScore(dfs_continuous);
  bool bfs_cont_seed_opt_found =
      bfs_cont_seed_opt.Run().schedule.has_value();
  llvm::outs() << "    BFS-DP continuous (seeded=" << dfs_continuous
               << " [DFS optimum]): found_improvement="
               << bfs_cont_seed_opt_found
               << " schedule_calls="
               << bfs_cont_seed_opt.GetDagForTest().GetScheduleCallCount()
               << " prunes="
               << bfs_cont_seed_opt.GetDagForTest().GetPruneCount() << "\n";

  // BFS-DP integer, unseeded — same integer optimum, different metric.
  BfsDpSearch bfs_int(&graph, &st, &mf, BfsDpSettings{kInteger});
  bfs_int.EnableTestModeForTest(vgpr_deltas);
  if (!bfs_int.Run().schedule) {
    report_fatal_error("RunBfsDpVsDfsShakedown[" + case_name +
                       "]: unseeded BFS-DP integer found no sink");
  }
  const PartitionDag &bfs_int_dag = bfs_int.GetDagForTest();
  int bfs_int_score = bfs_int_dag.GetSink()->best_path_bottleneck.score;
  unsigned bfs_int_peak =
      bfs_int_dag.GetSink()->best_path_bottleneck.register_pressure
          .getVGPRNum(st.hasGFX90AInsts());
  llvm::outs() << "    BFS-DP integer (unseeded): score=" << bfs_int_score
               << " peak_vgpr=" << bfs_int_peak
               << " schedule_calls=" << bfs_int_dag.GetScheduleCallCount()
               << " partitions=" << bfs_int_dag.GetPartitionNodeCount()
               << " levels=" << bfs_int_dag.GetCurrentLevel() << "\n";

  // BFS-DP integer seeded AT the DFS integer optimum. Expect no sink.
  BfsDpSearch bfs_int_seed_opt(&graph, &st, &mf,
                               BfsDpSettings{kInteger});
  bfs_int_seed_opt.EnableTestModeForTest(vgpr_deltas);
  bfs_int_seed_opt.SetInitialBestScore(dfs_integer);
  bool bfs_int_seed_opt_found =
      bfs_int_seed_opt.Run().schedule.has_value();
  llvm::outs() << "    BFS-DP integer (seeded=" << dfs_integer
               << " [DFS optimum]): found_improvement="
               << bfs_int_seed_opt_found
               << " schedule_calls="
               << bfs_int_seed_opt.GetDagForTest().GetScheduleCallCount()
               << " prunes="
               << bfs_int_seed_opt.GetDagForTest().GetPruneCount() << "\n";

  // Schedules for visual inspection.
  llvm::outs() << "    DFS schedule:               ";
  for (const ScheduleNode *n : dfs_best.GetScheduleOrder()) {
    llvm::outs() << " " << n->GetDebugName();
  }
  llvm::outs() << "\n";
  llvm::outs() << "    BFS-DP continuous schedule: ";
  for (const ScheduleNode *n : bfs_cont_dag.GetSchedule()) {
    llvm::outs() << " " << n->GetDebugName();
  }
  llvm::outs() << "\n";
  llvm::outs() << "    BFS-DP integer schedule:    ";
  for (const ScheduleNode *n : bfs_int_dag.GetSchedule()) {
    llvm::outs() << " " << n->GetDebugName();
  }
  llvm::outs() << "\n";

  // Sanity check: replay BFS-DP continuous's recovered schedule and
  // confirm the score matches the DP claim end-to-end.
  ScheduleConstructor replay(graph, st, mf);
  replay.GetPressureTrackerForTest().EnableTestModeForTest(vgpr_deltas);
  for (const ScheduleNode *n : bfs_cont_dag.GetSchedule()) {
    replay.Schedule(n);
  }
  int replay_score = replay.GetPressureTracker().GetMetricScore(kContinuous);
  bool replay_matches = replay_score == bfs_cont_score;
  llvm::outs() << "    BFS-DP continuous replayed: score=" << replay_score
               << " matches_claim=" << (replay_matches ? "PASS" : "FAIL")
               << "\n";

  // Soundness asserts.
  bool dfs_cont_matches = dfs_continuous == bfs_cont_score;
  bool dfs_int_matches = dfs_integer == bfs_int_score;
  bool seed_below_recovers_optimum =
      bfs_cont_seed_below_found &&
      bfs_cont_seed_below_dag.GetSink()->best_path_bottleneck.score ==
          dfs_continuous;
  llvm::outs() << "    DFS continuous == BFS-DP continuous (unseeded): "
               << (dfs_cont_matches ? "PASS\n" : "FAIL\n");
  llvm::outs() << "    DFS integer == BFS-DP integer (unseeded): "
               << (dfs_int_matches ? "PASS\n" : "FAIL\n");
  llvm::outs() << "    Seeded-with-(DFS optimum - 1) recovers optimum "
                  "(continuous): "
               << (seed_below_recovers_optimum ? "PASS\n" : "FAIL\n");
  llvm::outs() << "    Seeded-with-DFS-optimum found no improvement "
                  "(continuous): "
               << (!bfs_cont_seed_opt_found ? "PASS\n" : "FAIL\n");
  llvm::outs() << "    Seeded-with-DFS-optimum found no improvement "
                  "(integer): "
               << (!bfs_int_seed_opt_found ? "PASS\n" : "FAIL\n");

  if (!replay_matches) {
    report_fatal_error("RunBfsDpVsDfsShakedown[" + case_name +
                       "]: BFS-DP continuous claimed score does not "
                       "match replay");
  }
  if (!dfs_cont_matches) {
    report_fatal_error("RunBfsDpVsDfsShakedown[" + case_name +
                       "]: BFS-DP continuous score does not match DFS "
                       "oracle");
  }
  if (!dfs_int_matches) {
    report_fatal_error("RunBfsDpVsDfsShakedown[" + case_name +
                       "]: BFS-DP integer score does not match DFS "
                       "oracle");
  }
  if (!seed_below_recovers_optimum) {
    report_fatal_error("RunBfsDpVsDfsShakedown[" + case_name +
                       "]: BFS-DP continuous seeded with DFS optimum - 1 "
                       "failed to recover the optimum");
  }
  if (bfs_cont_seed_opt_found) {
    report_fatal_error("RunBfsDpVsDfsShakedown[" + case_name +
                       "]: BFS-DP continuous seeded with the DFS optimum "
                       "found a sink, but no strict improvement should "
                       "be possible");
  }
  if (bfs_int_seed_opt_found) {
    report_fatal_error("RunBfsDpVsDfsShakedown[" + case_name +
                       "]: BFS-DP integer seeded with the DFS integer "
                       "optimum found a sink");
  }
}

// Verifies that BFS-DP (PartitionDag) finds the same best continuous-
// occupancy score as the production DFS occupancy search on the same
// synthetic graph and synthetic VGPR deltas. Both algorithms aim at
// the same optimum: the schedule whose worst-pressure edge has the
// lowest pressure. DFS picks by max_pressure_ across the schedule;
// BFS-DP picks by min-along-path edge score. Since
// ComputeContinuousOccupancyScore is monotonically decreasing in
// pressure, "min score along the path" corresponds to "max pressure
// on the path's worst edge" — the same definition of peak — so the
// two algorithms must agree.
void RunBfsDpVsDfsShakedown(const GCNSubtarget &st,
                            const MachineFunction &mf,
                            const LiveIntervals &lis) {
  auto small_graph = ScheduleGraph::BuildPressureHistoryPruneTestDAG();
  RunBfsDpVsDfsComparisonOnGraph(
      "small", *small_graph,
      /*vgpr_deltas=*/{+1, +1, +1, -1, -1, -1}, st, mf, lis);

  // Wide case: 16 nodes, cross-edges + pure-parallel chains. Deltas
  // are +/-5 (not +/-1) so the peak VGPR range across orderings
  // (10 .. 35) crosses integer occupancy brackets — otherwise BFS-DP
  // integer would have no signal. Indexed by topo idx (matches
  // creation order in BuildBfsDpWideTestDAG): A=+5, B,C,D,E,M,O=+5,
  // F,G,H,I,N,P=-5, J=0, K=0, L=-5. Sum = 0.
  auto wide_graph = ScheduleGraph::BuildBfsDpWideTestDAG();
  RunBfsDpVsDfsComparisonOnGraph(
      "wide", *wide_graph,
      /*vgpr_deltas=*/{+5, +5, +5, +5, +5, +5, +5,
                       -5, -5, -5, -5, -5, -5,
                       0, 0, -5},
      st, mf, lis);
}

// End-to-end check that the occupancy-area tiebreak changes the DFS
// outcome. Runs two DFS searches on BuildAreaTiebreakTestDAG
// (A→B→M→{X,Y}→T) with deltas {+1,+2,0,-1,-2,0}:
//   - a peak-only occupancy policy (oracle variant, fully explored), and
//   - the area-tiebreak policy.
// Every order has VGPR peak 3 (both A,B live at M), so the two agree on
// the primary objective. After M, X-first lingers at 2 and Y-first
// drops to 1; X is created first, so the peak-only search settles on
// the lower-area X-first order while the area policy finds the higher-
// area Y-first order. Verifies same peak (no primary regression) and a
// strictly higher area from the tiebreak.
void RunDfsAreaTiebreakShakedown(const GCNSubtarget &st,
                                 const MachineFunction &mf,
                                 const LiveIntervals &lis) {
  llvm::outs() << "  RunDfsAreaTiebreakShakedown:\n";

  auto graph = ScheduleGraph::BuildAreaTiebreakTestDAG();
  graph->ValidateAndComputeTopologicalOrder();
  graph->ComputeCriticalPaths();
  graph->PopulateInputScheduleConstructorByTopoOrderForTest(st, mf);

  std::vector<int> vgpr_deltas = {+1, +2, 0, -1, -2, 0};

  // Peak-only baseline (BfsDpVsDfsShakedownOraclePolicy inherits the
  // occupancy policy with ShouldEndSearch disabled so it explores fully).
  DfsSearch<BfsDpVsDfsShakedownOraclePolicy> plain_search(*graph, st, mf, lis);
  plain_search.EnableTestModeForTest(vgpr_deltas);
  ScheduleConstructor plain_best = std::move(*plain_search.Run().schedule);
  int plain_peak =
      plain_best.GetPressureTracker().GetContinuousOccupancyScore();
  int64_t plain_area =
      plain_best.GetPressureTracker().GetContinuousOccupancyArea();

  DfsSearch<DfsMaximizeContinuousOccupancyThenAreaPolicy> area_search(
      *graph, st, mf, lis);
  area_search.EnableTestModeForTest(vgpr_deltas);
  ScheduleConstructor area_best = std::move(*area_search.Run().schedule);
  int area_peak =
      area_best.GetPressureTracker().GetContinuousOccupancyScore();
  int64_t area_area =
      area_best.GetPressureTracker().GetContinuousOccupancyArea();

  llvm::outs() << "    peak-only: peak_score=" << plain_peak
               << " area=" << plain_area << "\n";
  llvm::outs() << "    area:      peak_score=" << area_peak
               << " area=" << area_area << "\n";

  bool same_peak = plain_peak == area_peak;
  bool area_strictly_better = area_area > plain_area;
  llvm::outs() << "    Same optimal peak (no primary regression): "
               << (same_peak ? "PASS\n" : "FAIL\n");
  llvm::outs() << "    Area tiebreak strictly improves area at equal peak: "
               << (area_strictly_better ? "PASS\n" : "FAIL\n");
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
    graph->ComputeCriticalPaths();
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
    graph->ComputeCriticalPaths();
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
  // Opt into pressure history because this shakedown reads
  // GetPressureHistory() to cross-check against expected_history.
  GCNRegisterTracker tracker(graph, mf, /*track_pressure_history=*/true);

  // --- Forward pass: schedule in topo order ---
  // Capture cur_pressure_ after each Schedule call so we can
  // cross-check against pressure_history_ entry-by-entry below.
  SmallVector<GCNRegPressure> expected_history;
  llvm::outs() << "  GCN register pressure trace (topo order):\n";
  for (ScheduleNode *node : graph.GetTopoOrder()) {
    tracker.Schedule(node);
    expected_history.push_back(tracker.GetCurrentPressure());
    llvm::outs() << "    " << node->ToString() << "\n";
    llvm::outs() << "      " << tracker.DescribeRegOps(node) << "\n";
    llvm::outs() << "      " << tracker.DescribePressure() << "\n";
  }

  llvm::outs() << "  Occupancy: register_only="
               << tracker.GetRegisterOnlyOccupancy()
               << " all_factors_region_only="
               << tracker.GetAllFactorsRegionOnlyOccupancy()
               << "\n";

  // --- Verify pressure_history_ matches expected, entry by entry ---
  ArrayRef<GCNRegPressure> history = tracker.GetPressureHistory();
  bool history_match =
      history.size() == expected_history.size();
  for (size_t i = 0; history_match && i < history.size(); ++i) {
    if (!(history[i] == expected_history[i])) {
      history_match = false;
    }
  }
  llvm::outs() << "  pressure_history_ matches per-step cur_pressure_ "
                  "(length " << history.size() << "): "
               << (history_match ? "PASS\n" : "FAIL\n");

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
               tracker.GetLiveRegs().empty() &&
               tracker.GetPressureHistory().empty());
  llvm::outs() << "  Round-trip result: "
               << tracker.DescribePressure()
               << "  live_regs=" << tracker.GetLiveRegs().size()
               << "  pressure_history_size="
               << tracker.GetPressureHistory().size()
               << (pass ? "  PASS" : "  FAIL") << "\n";
  if (!pass) {
    report_fatal_error("GCNRegisterTracker round-trip test failed: "
                       "state did not return to zero after full unschedule");
  }
}

// Verifies GCNRegisterTracker's occupancy-area accumulation
// (GetContinuousOccupancyArea): occ_area_ is the running sum, over
// scheduling steps, of the continuous occupancy score of cur_pressure_,
// and Unschedule subtracts each step back out exactly (round-trips to
// 0). Independently recomputes each step's expected contribution from
// cur_pressure_ via the formula-based ComputeContinuousOccupancyScore,
// which also cross-checks the tracker's table-backed lookup against the
// formula (the tables are built from that formula).
void RunOccupancyAreaTrackingShakedown(ScheduleGraph &graph,
                                       const MachineFunction &mf) {
  llvm::outs() << "  Occupancy-area tracking shakedown:\n";
  const GCNSubtarget &st = mf.getSubtarget<GCNSubtarget>();
  GCNRegisterTracker tracker(graph, mf);

  SmallVector<ScheduleNode *> nodes(graph.GetTopoOrder().begin(),
                                    graph.GetTopoOrder().end());

  // Forward: schedule in topo order. After each step the tracker's
  // area must equal the running sum of per-step continuous scores.
  // Proxies don't accumulate (no pressure change), so contrib is 0
  // for them — matching the tracker's real-node-only accumulation.
  SmallVector<int> contrib;
  int64_t expected_area = 0;
  bool forward_ok = true;
  for (ScheduleNode *node : nodes) {
    tracker.Schedule(node);
    int step = 0;
    if (node->IsSchedulingUnit()) {
      const GCNRegPressure &cur = tracker.GetCurrentPressure();
      step = GCNRegisterTracker::ComputeContinuousOccupancyScore(
          st, cur.getVGPRNum(st.hasGFX90AInsts()), cur.getSGPRNum());
    }
    contrib.push_back(step);
    expected_area += step;
    if (tracker.GetContinuousOccupancyArea() != expected_area) {
      forward_ok = false;
    }
  }
  llvm::outs() << "    forward area == sum of per-step score (area="
               << tracker.GetContinuousOccupancyArea() << "): "
               << (forward_ok ? "PASS\n" : "FAIL\n");

  // Reverse: unschedule in reverse order. Each Unschedule subtracts
  // its step's contribution; area must track the prefix sum and reach
  // 0 once everything is unscheduled.
  bool reverse_ok = true;
  for (int i = static_cast<int>(nodes.size()) - 1; i >= 0; --i) {
    tracker.Unschedule(nodes[i]);
    expected_area -= contrib[i];
    if (tracker.GetContinuousOccupancyArea() != expected_area) {
      reverse_ok = false;
    }
  }
  bool back_to_zero = tracker.GetContinuousOccupancyArea() == 0;
  llvm::outs() << "    reverse area round-trips to 0: "
               << ((reverse_ok && back_to_zero) ? "PASS\n" : "FAIL\n");
  if (!(forward_ok && reverse_ok && back_to_zero)) {
    report_fatal_error("Occupancy-area tracking shakedown failed");
  }
}

// Verifies GCNRegisterTracker's VGPR-spill-area accumulation
// (GetVGPRSpillArea): vgpr_spill_area_ is the running sum, over
// scheduling steps, of GetCurVGPRCountAboveSpillCap (count of VGPRs
// over the floor's VGPR cap), and Unschedule subtracts each step
// back out exactly (round-trips to 0). Recomputes each step's
// expected contribution independently from the synthetic delta
// sequence and the floor-derived spill cap.
//
// Self-contained, parameter-independent:
//   - Builds BuildAreaTiebreakTestDAG (6 nodes, known shape).
//   - Drives pressure via GCNRegisterTracker's test mode with a
//     hand-designed delta sequence chosen to push the running VGPR
//     above the cap on step 2-3 and back below it for steps 4-6.
//   - Forces the spill cap via SetOccupancyFloorForTest so the
//     test doesn't depend on the live MF's launch attributes.
void RunVGPRSpillAreaAccumulatorShakedown(const MachineFunction &mf) {
  llvm::outs() << "  VGPR-spill-area accumulator shakedown:\n";
  const GCNSubtarget &st = mf.getSubtarget<GCNSubtarget>();

  auto graph = ScheduleGraph::BuildAreaTiebreakTestDAG();
  graph->ValidateAndComputeTopologicalOrder();

  GCNRegisterTracker tracker(*graph, mf);

  // Force floor=8 -> spill_cap=getMaxNumVGPRs(8)=32 on gfx9. The
  // override decouples this test from the live MF's launch
  // attributes; without it, floor=1 -> cap=256 and the deltas below
  // would never cross the cap.
  constexpr unsigned kTestFloor = 8;
  tracker.SetOccupancyFloorForTest(kTestFloor);
  const unsigned spill_cap = st.getMaxNumVGPRs(kTestFloor);
  llvm::outs() << "    floor=" << kTestFloor
               << " vgpr spill cap=" << spill_cap << "\n";

  // Deltas indexed by topo index (BuildAreaTiebreakTestDAG topo
  // matches creation order A,B,M,X,Y,T). Chosen to put the running
  // VGPR above the cap on steps B/M and back below on X/Y/T, so the
  // accumulator picks up a non-zero contribution on multiple steps
  // and then is "fixed" thereafter:
  //   A(+10):   10 -> above_cap=0,  acc=0
  //   B(+25):   35 -> above_cap=3,  acc=3
  //   M(+0):    35 -> above_cap=3,  acc=6
  //   X(-15):   20 -> above_cap=0,  acc=6
  //   Y(-10):   10 -> above_cap=0,  acc=6
  //   T(-10):    0 -> above_cap=0,  acc=6
  std::vector<int> vgpr_deltas = {+10, +25, 0, -15, -10, -10};
  tracker.EnableTestModeForTest(vgpr_deltas);

  SmallVector<ScheduleNode *> nodes(graph->GetTopoOrder().begin(),
                                    graph->GetTopoOrder().end());

  // Forward: Schedule each node in topo order. After each step,
  // tracker.GetVGPRSpillArea() must equal the running sum of
  // max(0, running_vgpr - spill_cap) over all steps so far.
  // running_vgpr is computed directly from the deltas -- independent
  // of the tracker.
  SmallVector<int> contrib;
  int64_t expected_area = 0;
  int running_vgpr = 0;
  bool forward_ok = true;
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    ScheduleNode *node = nodes[i];
    tracker.Schedule(node);
    running_vgpr += vgpr_deltas[node->GetTopoIndex()];
    int step =
        (running_vgpr > static_cast<int>(spill_cap))
            ? (running_vgpr - static_cast<int>(spill_cap))
            : 0;
    contrib.push_back(step);
    expected_area += step;
    if (tracker.GetVGPRSpillArea() != expected_area) {
      forward_ok = false;
    }
  }
  llvm::outs() << "    forward area == sum of per-step above-cap "
                  "count (area="
               << tracker.GetVGPRSpillArea() << "): "
               << (forward_ok ? "PASS\n" : "FAIL\n");

  // Reverse: Unschedule in reverse order. Each Unschedule subtracts
  // its step's contribution; area must reach 0.
  bool reverse_ok = true;
  for (int i = static_cast<int>(nodes.size()) - 1; i >= 0; --i) {
    tracker.Unschedule(nodes[i]);
    expected_area -= contrib[i];
    if (tracker.GetVGPRSpillArea() != expected_area) {
      reverse_ok = false;
    }
  }
  bool back_to_zero = tracker.GetVGPRSpillArea() == 0;
  llvm::outs() << "    reverse area round-trips to 0: "
               << ((reverse_ok && back_to_zero) ? "PASS\n" : "FAIL\n");

  tracker.ClearTargetAndFloorOverridesForTest();
  if (!(forward_ok && reverse_ok && back_to_zero)) {
    report_fatal_error("VGPR-spill-area accumulator shakedown failed");
  }
}

// Tests GCNRegisterTracker::NoHistoryClone:
//   1. Build a parent tracker with track_pressure_history=true,
//      schedule a prefix of nodes so it has non-trivial
//      cur_pressure_ / live_regs_ / max_pressure_.
//   2. Clone via NoHistoryClone.
//   3. Verify clone.GetCurrentPressure() == parent.GetCurrentPressure()
//      (cur_pressure_ copied).
//   4. Verify clone.GetLiveRegs() == parent.GetLiveRegs() (live set
//      copied — same size + same {reg, mask} entries).
//   5. Verify clone.GetPeakPressure() == default (max_pressure_
//      reset, NOT carried from parent — design choice per the
//      NoHistoryClone header comment).
//
// No assumptions about what scheduling does to pressure; the
// checks are relative ("matches parent at clone time" / "reset to
// default"). Deep-copy / non-aliasing follows from the field
// types being value types (DenseMap, std::vector, GCNRegPressure),
// not pointers — the compiler-generated init copies handle it.
void RunNoHistoryCloneShakedown(ScheduleGraph &graph,
                                const MachineFunction &mf) {
  llvm::outs() << "  NoHistoryClone shakedown:\n";

  // track_pressure_history=true on the parent so we can also
  // verify the clone resets the opt-in flag (via the fact that
  // pressure_history_ is empty on the clone — implicit; we don't
  // GetPressureHistory on the clone because that would fatal-error
  // on the reset flag and we can't catch fatal-errors from here).
  GCNRegisterTracker parent(graph, mf, /*track_pressure_history=*/true);

  // Schedule the first three nodes in topo order so parent has
  // non-trivial state. Picks 3 as a small number; the exact
  // contents don't matter for the clone-correctness checks.
  ArrayRef<ScheduleNode *> topo = graph.GetTopoOrder();
  int scheduled_count = 0;
  for (ScheduleNode *node : topo) {
    parent.Schedule(node);
    ++scheduled_count;
    if (scheduled_count >= 3) {
      break;
    }
  }

  GCNRegPressure parent_cur = parent.GetCurrentPressure();
  GCNRegisterTracker::LiveRegSet parent_live = parent.GetLiveRegs();

  GCNRegisterTracker clone = parent.NoHistoryClone();

  // Check 1: cur_pressure_ copied.
  bool cur_match = clone.GetCurrentPressure() == parent_cur;
  llvm::outs() << "    cur_pressure copied: "
               << (cur_match ? "PASS" : "FAIL") << "\n";
  if (!cur_match) {
    report_fatal_error("NoHistoryClone: cur_pressure_ mismatch");
  }

  // Check 2: live_regs_ copied.
  const GCNRegisterTracker::LiveRegSet &clone_live = clone.GetLiveRegs();
  bool live_size_match = clone_live.size() == parent_live.size();
  bool live_entries_match = live_size_match;
  if (live_entries_match) {
    for (const auto &[reg, mask] : parent_live) {
      auto it = clone_live.find(reg);
      if (it == clone_live.end() || it->second != mask) {
        live_entries_match = false;
        break;
      }
    }
  }
  llvm::outs() << "    live_regs copied (size=" << parent_live.size()
               << "): "
               << (live_entries_match ? "PASS" : "FAIL") << "\n";
  if (!live_entries_match) {
    report_fatal_error("NoHistoryClone: live_regs_ mismatch");
  }

  // Check 3: max_pressure_ reset.
  bool max_reset = clone.GetPeakPressure() == GCNRegPressure();
  llvm::outs() << "    max_pressure reset to default: "
               << (max_reset ? "PASS" : "FAIL") << "\n";
  if (!max_reset) {
    report_fatal_error("NoHistoryClone: max_pressure_ not reset");
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
  GCNRegisterTracker tracker(graph, mf);

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

// Tests IlpTracker: drives Schedule/Unschedule across all graph
// nodes in topo order, exercising the producer-to-first-consumer
// scoring (open/close, weight propagation, count bump). Verifies:
//   - Initial empty state.
//   - Op-type desirable_spacing buckets populated for real
//     instructions (informational stat — no PASS/FAIL).
//   - Forward GetIlpScore (locked-in) is monotone non-decreasing.
//   - At full schedule, the open-producer map is empty (the exit
//     sentinel must have closed all live-out producers).
//   - At completion, GetProvisionalIlpScore == GetIlpScore.
//   - Reverse Unschedule restores both scores to each forward-step
//     snapshot exactly.
//   - Round-trip returns to initial empty state.
//
// Drives the IlpTracker directly (no ScheduleConstructor) — the
// pressure tracker's NodeRegInfo is the only IlpTracker dependency
// that needs to be live, and topo order is a valid schedule order
// that satisfies all dependencies (defs before uses, members
// before exit sentinel).
void RunIlpTrackerShakedown(ScheduleGraph &graph,
                            const MachineFunction &mf,
                            const LiveIntervals &lis) {
  GCNRegisterTracker pressure_tracker(graph, mf);
  IlpTracker tracker(graph, pressure_tracker);

  // --- Initial empty state ---
  bool initial_ok = tracker.GetIlpScore() == 0 &&
                    tracker.GetProvisionalIlpScore() == 0 &&
                    tracker.GetInstructionsIssuedCount() == 0 &&
                    tracker.GetOpenProducerVregCount() == 0;
  llvm::outs() << "  Initial empty state: "
               << (initial_ok ? "PASS\n" : "FAIL\n");

  // --- Op-type desirable_spacing buckets (informational) ---
  // Walk all nodes, count how many got each saturation-cap value.
  // Only real MachineInstr-backed nodes get a non-zero value;
  // everything else (proxies, sentinels) reads 0.
  int real_count = 0;
  int vmem_flat_count = 0;
  int smem_count = 0;
  int ds_count = 0;
  int default_count = 0;
  for (const ScheduleNode &node : graph.Nodes()) {
    int ds_val = tracker.GetDesirableSpacing(&node);
    if (ds_val == 0) {
      continue;
    }
    ++real_count;
    if (ds_val == 32) {
      ++vmem_flat_count;
    } else if (ds_val == 16) {
      ++smem_count;
    } else if (ds_val == 8) {
      ++ds_count;
    } else {
      ++default_count;
    }
  }
  llvm::outs() << "  Op-type desirable_spacing buckets: real=" << real_count
               << " vmem/flat(32)=" << vmem_flat_count
               << " smem(16)=" << smem_count
               << " ds(8)=" << ds_count
               << " default(2)=" << default_count << "\n";

  // --- Forward pass: schedule in topo order ---
  // ilp_after[i] / provisional_after[i] = GetIlpScore() /
  // GetProvisionalIlpScore() after i nodes have been scheduled.
  // Index 0 = empty state; index N = fully scheduled.
  std::vector<int> ilp_after;
  std::vector<int> provisional_after;
  ilp_after.push_back(tracker.GetIlpScore());
  provisional_after.push_back(tracker.GetProvisionalIlpScore());

  // Per-step verification of GetOpenProducerInstCountsSnapshot:
  //   - Size matches GetOpenProducerVregCount.
  //   - Sorted strictly ascending by reg.
  //   - Every inst_count is in [0, GetInstructionsIssuedCount()).
  // Accumulate violations across all steps; report once after the
  // forward pass.
  int snapshot_violations = 0;
  bool any_step_with_nonempty_opens = false;

  llvm::outs() << "  ILP trace (topo order):\n";
  for (ScheduleNode *node : graph.GetTopoOrder()) {
    tracker.Schedule(node);
    ilp_after.push_back(tracker.GetIlpScore());
    provisional_after.push_back(tracker.GetProvisionalIlpScore());
    llvm::outs() << "    " << node->ToString() << "  ->  "
                 << tracker.Describe() << "\n";

    auto snapshot = tracker.GetOpenProducerInstCountsSnapshot();
    if (static_cast<int>(snapshot.size()) !=
        tracker.GetOpenProducerVregCount()) {
      ++snapshot_violations;
    }
    for (int i = 1; i < static_cast<int>(snapshot.size()); ++i) {
      if (snapshot[i - 1].reg >= snapshot[i].reg) {
        ++snapshot_violations;
      }
    }
    const int issued = tracker.GetInstructionsIssuedCount();
    for (const auto &producer : snapshot) {
      if (producer.inst_count < 0 || producer.inst_count >= issued) {
        ++snapshot_violations;
      }
    }
    if (!snapshot.empty()) {
      any_step_with_nonempty_opens = true;
    }
  }
  // Coverage flag: if no step had non-empty opens (degenerate
  // small region), the snapshot logic wasn't really exercised.
  // Don't FAIL on that — just tag the output.
  llvm::outs() << "  Snapshot invariants (size, sorted, in-range):"
               << (snapshot_violations == 0 ? "  PASS" : "  FAIL")
               << "    (nonempty-opens steps: "
               << (any_step_with_nonempty_opens ? "yes" : "no")
               << ")\n";

  // --- Forward invariants ---
  // (a) GetIlpScore (locked in) is monotone non-decreasing —
  // every close (real or re-def-implicit) only ever ADDS to
  // closed_ilp_score_, so this is a strict invariant.
  // GetProvisionalIlpScore can dip at re-def events, so we don't
  // check it here.
  int monotonicity_violations = 0;
  for (int i = 1; i < static_cast<int>(ilp_after.size()); ++i) {
    if (ilp_after[i] < ilp_after[i - 1]) {
      llvm::outs() << "  GetIlpScore monotonicity violated at step "
                   << i << ": " << ilp_after[i - 1] << " -> "
                   << ilp_after[i] << "\n";
      ++monotonicity_violations;
    }
  }
  llvm::outs() << "  GetIlpScore monotonicity:"
               << (monotonicity_violations == 0 ? "  PASS\n"
                                                : "  FAIL\n");

  // (b) After full schedule, open-producer count must be 0 — the
  // exit sentinel processes its uses (live-outs) and closes them.
  // A nonzero count indicates either a live-out reg never reached
  // the exit sentinel's NodeRegInfo.uses (graph-construction bug)
  // or the close-side guard rejected sentinels (regression in
  // IlpTracker::Schedule).
  bool completion_ok = tracker.GetOpenProducerVregCount() == 0;
  llvm::outs() << "  Open producers settled at completion: "
               << (completion_ok ? "PASS\n" : "FAIL\n")
               << "    (" << tracker.GetOpenProducerVregCount()
               << " left open)\n";

  // (c) At completion, GetProvisionalIlpScore == GetIlpScore (open
  // map is empty, so the pending portion is zero).
  bool completion_scores_match =
      tracker.GetProvisionalIlpScore() == tracker.GetIlpScore();
  llvm::outs()
      << "  Completion: GetProvisionalIlpScore == GetIlpScore: "
      << (completion_scores_match ? "PASS\n" : "FAIL\n");

  // --- Reverse pass: unschedule everything ---
  // After unscheduling node i (in reverse topo order), the tracker
  // state should equal the state just before that node was
  // scheduled forward — both scores should equal their forward
  // snapshots at index i.
  llvm::outs() << "  ILP trace (unschedule):\n";
  int reverse_violations = 0;
  for (int i = static_cast<int>(graph.GetTopoOrder().size()) - 1;
       i >= 0; --i) {
    tracker.Unschedule(graph.GetTopoOrder()[i]);
    int ilp = tracker.GetIlpScore();
    int provisional = tracker.GetProvisionalIlpScore();
    int expected_ilp = ilp_after[i];
    int expected_provisional = provisional_after[i];
    llvm::outs() << "    undo  " << tracker.Describe()
                 << "  ilp=" << ilp << " (expected " << expected_ilp
                 << ")  provisional=" << provisional << " (expected "
                 << expected_provisional << ")\n";
    if (ilp != expected_ilp || provisional != expected_provisional) {
      ++reverse_violations;
    }
  }
  llvm::outs() << "  Unschedule restoration:"
               << (reverse_violations == 0 ? "  PASS\n" : "  FAIL\n");

  // --- Round-trip ---
  bool roundtrip_ok = tracker.GetIlpScore() == 0 &&
                      tracker.GetProvisionalIlpScore() == 0 &&
                      tracker.GetInstructionsIssuedCount() == 0 &&
                      tracker.GetOpenProducerVregCount() == 0;
  llvm::outs() << "  Round-trip result: " << tracker.Describe()
               << (roundtrip_ok ? "  PASS" : "  FAIL") << "\n";
  if (!roundtrip_ok) {
    report_fatal_error("IlpTracker round-trip test failed: state did "
                       "not return to zero after full unschedule");
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
  ScheduleConstructor sc(graph, st, mf);

  // --- Forward pass: always pick the first ready node ---
  // Also tracks expected state of ScheduledSetTracker: after each
  // Schedule, the just-scheduled node's bit must be set and the
  // scheduled-set count must match the number of Schedule calls.
  // Catches a regression where ScheduleConstructor stops driving
  // the tracker (would not show up in pure round-trip checks).
  llvm::outs() << "  ScheduleConstructor trace:\n";
  int expected_scheduled_count = 0;
  bool tracker_state_ok = true;
  while (!sc.IsDone()) {
    const auto &ready = sc.GetReadyList();
    if (ready.empty()) {
      report_fatal_error("ScheduleConstructor: ready list empty before "
                         "all nodes scheduled");
    }
    const ScheduleNode *node = *ready.begin();
    sc.Schedule(node);
    ++expected_scheduled_count;
    const ScheduledSetTracker &scheduled_set_tracker =
        sc.GetScheduledSetTracker();
    if (!scheduled_set_tracker.GetScheduledSet().test(
            node->GetTopoIndex()) ||
        scheduled_set_tracker.GetScheduledSet().count() !=
            expected_scheduled_count) {
      tracker_state_ok = false;
    }
    llvm::outs() << "    " << node->ToString() << "\n"
                 << "      " << sc.Describe() << "\n";
  }
  llvm::outs() << "  ScheduledSetTracker driven by ScheduleConstructor: "
               << (tracker_state_ok ? "PASS\n" : "FAIL\n");
  if (!tracker_state_ok) {
    report_fatal_error("ScheduledSetTracker not properly driven by "
                       "ScheduleConstructor::Schedule");
  }

  llvm::outs() << "  ScheduleConstructor (arbitrary): "
               << sc.Describe() << "\n";

  // --- Reverse pass: unschedule everything ---
  int num_scheduled = sc.GetNumScheduled();
  for (int i = 0; i < num_scheduled; ++i) {
    sc.Unschedule();
  }

  // --- Verify round-trip ---
  const ScheduledSetTracker &final_scheduled_set_tracker =
      sc.GetScheduledSetTracker();
  bool pass =
      (sc.GetNumScheduled() == 0 &&
       sc.GetLengthTracker().GetCurrentCycle() == 0 &&
       sc.GetLengthTracker().GetTotalBubbles() == 0 &&
       final_scheduled_set_tracker.GetPrefixSignature() == 0 &&
       final_scheduled_set_tracker.GetScheduledSet().none() &&
       final_scheduled_set_tracker.GetFrontier().empty());
  llvm::outs() << "  ScheduleConstructor round-trip: "
               << sc.Describe()
               << (pass ? "  PASS" : "  FAIL") << "\n";
  if (!pass) {
    report_fatal_error("ScheduleConstructor round-trip test failed");
  }

  // --- Second pass: schedule in topo order for comparison ---
  ScheduleConstructor sc2(graph, st, mf);
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
  ScheduleConstructor sc_empty(graph, st, mf);
  ScheduleConstructor sc_full(graph, st, mf);
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
  check_metric(
      ScheduleMetric::kMaximizeScheduleLength, "max_length",
      sc_empty.GetLengthTracker().GetCurrentCycle(),
      sc_full.GetLengthTracker().GetCurrentCycle(),
      /*higher_is_better=*/true);

  // --- Part 4: RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget observability ---
  //
  // Print the pieces so we can see them line up. No pass/fail since
  // whether the region is at the ceiling depends on actual pressure.
  llvm::outs() << "    initial ceiling check: reg_occ="
               << sc_empty.GetPressureTracker().GetRegisterOnlyOccupancy()
               << " fn_limit="
               << sc_empty.GetPressureTracker().GetConfiguredMachineFunctionOccupancyLimit()
               << " at_ceiling=" << sc_empty.RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget() << "\n";
  llvm::outs() << "    full ceiling check:    reg_occ="
               << sc_full.GetPressureTracker().GetRegisterOnlyOccupancy()
               << " fn_limit="
               << sc_full.GetPressureTracker().GetConfiguredMachineFunctionOccupancyLimit()
               << " at_ceiling=" << sc_full.RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget() << "\n";
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

// Forward declaration: the body lives at the end of the anonymous
// namespace alongside the other recent-addition shakedowns
// (RunScoreShakedown, RunOccupancyTargetUtilShakedown), but
// RunRegionShakedowns below calls it.
void RunEffectiveAndTargetLimitHelpersShakedown(
    ScheduleGraph &graph, const MachineFunction &mf);

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
  RunNoHistoryCloneShakedown(graph, mf);
  RunOccupancyAreaTrackingShakedown(graph, mf);

  SmallVector<ScheduleNode *> topo_nodes(graph.GetTopoOrder().begin(),
                                         graph.GetTopoOrder().end());
  VerifyGCNRegisterTracker(graph, topo_nodes, mf, lis);

  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(mf.getSubtarget());
  RunScheduleLengthTrackerShakedown(graph, st);
  RunIlpTrackerShakedown(graph, mf, lis);
  RunScheduleConstructorShakedown(graph, mf, lis);
  RunEffectiveAndTargetLimitHelpersShakedown(graph, mf);
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

// Round-trip shakedown for ScheduleGraph::BuildFromNodeSubset. Takes
// BuildTestDAG (nodes A,C,D,E,F,G,H; edges A->H, A->C, A->D, C->D,
// C->E, D->F, E->F, H->G, F->G), stages a register pattern on it, and
// extracts members {C,D,E,F} as a standalone subgraph.
//
// Staged register pattern:
//   A def r0;  C use r0 def r1;  D use r1 def r2;  E use r1 def r3;
//   F use r2,r3 def r4;  G use r4;  H none.
// With members {C,D,E,F}: r0 is used by C but defined by no member,
// so it is live-in; r4 is defined by F and used by non-member G, so
// it is live-out. r1/r2/r3 are defined and used entirely within the
// subgraph — neither boundary.
void RunBuildFromNodeSubsetShakedown(const GCNSubtarget &st,
                                     const MachineFunction &mf) {
  llvm::outs() << "  RunBuildFromNodeSubsetShakedown:\n";

  auto parent_graph = ScheduleGraph::BuildTestDAG();
  // BuildTestDAG emplacement order: [A, C, D, E, F, G, H].
  ScheduleNode *pA = &parent_graph->Nodes()[0];
  ScheduleNode *pC = &parent_graph->Nodes()[1];
  ScheduleNode *pD = &parent_graph->Nodes()[2];
  ScheduleNode *pE = &parent_graph->Nodes()[3];
  ScheduleNode *pF = &parent_graph->Nodes()[4];
  ScheduleNode *pG = &parent_graph->Nodes()[5];

  // Stage the parent's register table to the pattern above.
  auto vreg = [](unsigned i) { return Register::index2VirtReg(i); };
  LaneBitmask all = LaneBitmask::getAll();
  NodeRegInfoTable parent_table(parent_graph->GetNumGraphLocalIds());
  parent_table.AddDef(pA, vreg(0).id(), all);
  parent_table.AddUse(pC, vreg(0).id(), all);
  parent_table.AddDef(pC, vreg(1).id(), all);
  parent_table.AddUse(pD, vreg(1).id(), all);
  parent_table.AddDef(pD, vreg(2).id(), all);
  parent_table.AddUse(pE, vreg(1).id(), all);
  parent_table.AddDef(pE, vreg(3).id(), all);
  parent_table.AddUse(pF, vreg(2).id(), all);
  parent_table.AddUse(pF, vreg(3).id(), all);
  parent_table.AddDef(pF, vreg(4).id(), all);
  parent_table.AddUse(pG, vreg(4).id(), all);
  parent_graph->SetNodeRegInfoTable(std::move(parent_table));

  // Finalize the parent so it carries the topo order, critical
  // paths, and input schedule BuildFromNodeSubset reads.
  parent_graph->ValidateAndComputeTopologicalOrder();
  parent_graph->ComputeCriticalPaths();
  parent_graph->PopulateInputScheduleConstructorByTopoOrderForTest(st, mf);

  std::vector<ScheduleNode *> members = {pC, pD, pE, pF};
  DenseMap<const ScheduleNode *, ScheduleNode *> subgraph_to_parent;
  auto subgraph_graph = ScheduleGraph::BuildFromNodeSubset(
      members, *parent_graph, st, mf, subgraph_to_parent);

  auto check = [](StringRef desc, bool ok) {
    llvm::outs() << "    " << desc << ": " << (ok ? "PASS" : "FAIL")
                 << "\n";
  };

  // Locate the subgraph nodes by debug name (members inherit their
  // member's name; the synthetic pair is SubgraphEntry/SubgraphExit).
  ScheduleNode *sC = nullptr, *sD = nullptr, *sE = nullptr,
               *sF = nullptr, *sEntry = nullptr, *sExit = nullptr;
  for (ScheduleNode &n : subgraph_graph->Nodes()) {
    StringRef name = n.GetDebugName();
    if (name == "C") {
      sC = &n;
    } else if (name == "D") {
      sD = &n;
    } else if (name == "E") {
      sE = &n;
    } else if (name == "F") {
      sF = &n;
    } else if (name == "SubgraphEntry") {
      sEntry = &n;
    } else if (name == "SubgraphExit") {
      sExit = &n;
    }
  }
  check("node count is 6 (4 members + entry + exit)",
        subgraph_graph->Size() == 6);
  if (!sC || !sD || !sE || !sF || !sEntry || !sExit) {
    check("all six expected nodes present", false);
    return;
  }

  // Members emplaced first in parent input order (topo: C,D,E,F),
  // then the synthetic entry/exit.
  check("layout: members in input order, then entry/exit",
        &subgraph_graph->Nodes()[0] == sC &&
            &subgraph_graph->Nodes()[1] == sD &&
            &subgraph_graph->Nodes()[2] == sE &&
            &subgraph_graph->Nodes()[3] == sF &&
            &subgraph_graph->Nodes()[4] == sEntry &&
            &subgraph_graph->Nodes()[5] == sExit);

  // Translation map: one entry per member, entry/exit absent.
  check("translation map maps members, omits entry/exit",
        subgraph_to_parent.size() == 4 &&
            subgraph_to_parent.lookup(sC) == pC &&
            subgraph_to_parent.lookup(sD) == pD &&
            subgraph_to_parent.lookup(sE) == pE &&
            subgraph_to_parent.lookup(sF) == pF &&
            !subgraph_to_parent.count(sEntry) &&
            !subgraph_to_parent.count(sExit));

  // Debug-name sets of a node's successors / predecessors, and a
  // check of a node's complete set of incident edges.
  auto succ_names = [](const ScheduleNode &n) {
    std::set<std::string> s;
    for (const ScheduleEdge &e : n.Successors()) {
      s.insert(e.node_->GetDebugName().str());
    }
    return s;
  };
  auto pred_names = [](const ScheduleNode &n) {
    std::set<std::string> s;
    for (const ScheduleEdge &e : n.Predecessors()) {
      s.insert(e.node_->GetDebugName().str());
    }
    return s;
  };
  using NameSet = std::set<std::string>;
  auto edges_ok = [&](const ScheduleNode &n, NameSet preds,
                      NameSet succs) {
    return pred_names(n) == preds && succ_names(n) == succs;
  };

  // Every node's complete incident edges. Intra-subgraph edges
  // (C->D, C->E, D->F, E->F) are copied; boundary edges (A->C, F->G)
  // are dropped; SubgraphEntry feeds the root C, SubgraphExit is fed
  // by the leaf F.
  check("C edges: pred {SubgraphEntry}, succ {D,E}",
        edges_ok(*sC, {"SubgraphEntry"}, {"D", "E"}));
  check("D edges: pred {C}, succ {F}", edges_ok(*sD, {"C"}, {"F"}));
  check("E edges: pred {C}, succ {F}", edges_ok(*sE, {"C"}, {"F"}));
  check("F edges: pred {D,E}, succ {SubgraphExit}",
        edges_ok(*sF, {"D", "E"}, {"SubgraphExit"}));
  check("SubgraphEntry edges: no pred, succ {C}",
        edges_ok(*sEntry, {}, {"C"}));
  check("SubgraphExit edges: pred {F}, no succ",
        edges_ok(*sExit, {"F"}, {}));

  // Register boundary: entry defines live-in {r0}, exit uses
  // live-out {r4}.
  check("entry defines live-in {r0}",
        sEntry->RegDefs().size() == 1 &&
            sEntry->RegDefs()[0].reg == vreg(0));
  check("exit uses live-out {r4}",
        sExit->RegUses().size() == 1 &&
            sExit->RegUses()[0].reg == vreg(4));

  // Each member's register-table entry is copied from the parent.
  auto reg_ids = [](ArrayRef<RegMask> masks) {
    std::set<unsigned> ids;
    for (const RegMask &m : masks) {
      ids.insert(m.reg);
    }
    return ids;
  };
  auto member_table_ok = [&](const ScheduleNode *member,
                             std::set<unsigned> uses,
                             std::set<unsigned> defs) {
    const NodeRegInfo &info =
        subgraph_graph->GetNodeRegInfoTable().GetForNode(member);
    return reg_ids(info.uses) == uses && reg_ids(info.defs) == defs;
  };
  check("member C reg table: use {r0}, def {r1}",
        member_table_ok(sC, {vreg(0).id()}, {vreg(1).id()}));
  check("member D reg table: use {r1}, def {r2}",
        member_table_ok(sD, {vreg(1).id()}, {vreg(2).id()}));
  check("member E reg table: use {r1}, def {r3}",
        member_table_ok(sE, {vreg(1).id()}, {vreg(3).id()}));
  check("member F reg table: use {r2,r3}, def {r4}",
        member_table_ok(sF, {vreg(2).id(), vreg(3).id()},
                        {vreg(4).id()}));

  // Input schedule: SubgraphEntry, the members in input order, then
  // SubgraphExit.
  std::vector<std::string> input_order;
  for (const ScheduleNode *n :
       subgraph_graph->GetInputScheduleConstructor().GetScheduleOrder()) {
    input_order.push_back(n->GetDebugName().str());
  }
  check("input schedule order: Entry, C, D, E, F, Exit",
        input_order == std::vector<std::string>{"SubgraphEntry", "C",
                                                "D", "E", "F",
                                                "SubgraphExit"});

  check("subgraph graph is topologically sorted",
        subgraph_graph->IsTopoSorted());
}

// Shakedown for ScheduleSubgraph. Extracts members {C,D,E,F} from
// BuildTestDAG (staged as in RunBuildFromNodeSubsetShakedown) and
// checks that ScheduleSubgraph records the search's schedule —
// translated to parent nodes — on the SubgraphInfo, falling back to
// the input order when the search produces none.
void RunScheduleSubgraphShakedown(const GCNSubtarget &st,
                                  const MachineFunction &mf) {
  llvm::outs() << "  RunScheduleSubgraphShakedown:\n";

  auto parent_graph = ScheduleGraph::BuildTestDAG();
  // BuildTestDAG emplacement order: [A, C, D, E, F, G, H].
  ScheduleNode *pA = &parent_graph->Nodes()[0];
  ScheduleNode *pC = &parent_graph->Nodes()[1];
  ScheduleNode *pD = &parent_graph->Nodes()[2];
  ScheduleNode *pE = &parent_graph->Nodes()[3];
  ScheduleNode *pF = &parent_graph->Nodes()[4];
  ScheduleNode *pG = &parent_graph->Nodes()[5];

  // Stage a register pattern (same as RunBuildFromNodeSubsetShakedown):
  //   A def r0;  C use r0 def r1;  D use r1 def r2;  E use r1 def r3;
  //   F use r2,r3 def r4;  G use r4.
  auto vreg = [](unsigned i) { return Register::index2VirtReg(i); };
  LaneBitmask all = LaneBitmask::getAll();
  NodeRegInfoTable parent_table(parent_graph->GetNumGraphLocalIds());
  parent_table.AddDef(pA, vreg(0).id(), all);
  parent_table.AddUse(pC, vreg(0).id(), all);
  parent_table.AddDef(pC, vreg(1).id(), all);
  parent_table.AddUse(pD, vreg(1).id(), all);
  parent_table.AddDef(pD, vreg(2).id(), all);
  parent_table.AddUse(pE, vreg(1).id(), all);
  parent_table.AddDef(pE, vreg(3).id(), all);
  parent_table.AddUse(pF, vreg(2).id(), all);
  parent_table.AddUse(pF, vreg(3).id(), all);
  parent_table.AddDef(pF, vreg(4).id(), all);
  parent_table.AddUse(pG, vreg(4).id(), all);
  parent_graph->SetNodeRegInfoTable(std::move(parent_table));
  parent_graph->ValidateAndComputeTopologicalOrder();
  parent_graph->ComputeCriticalPaths();
  parent_graph->PopulateInputScheduleConstructorByTopoOrderForTest(st, mf);

  auto check = [](StringRef desc, bool ok) {
    llvm::outs() << "    " << desc << ": " << (ok ? "PASS" : "FAIL")
                 << "\n";
  };
  // Debug-name list of a recorded schedule's order.
  auto order_names = [](const SubgraphScheduleResult &result) {
    std::vector<std::string> names;
    for (const ScheduleNode *node : result.order) {
      names.push_back(node->GetDebugName().str());
    }
    return names;
  };
  using NameList = std::vector<std::string>;

  // Case 1: the search produces no schedule — ScheduleSubgraph
  // records the subgraph's input order (C,D,E,F) and preserves the
  // search's termination cause.
  {
    SubgraphInfo info({pC, pD, pE, pF}, "S");
    const SubgraphScheduleResult &result = ScheduleSubgraph(
        info, *parent_graph, st, mf, [](const ScheduleGraph &) {
          return SearchResult{std::nullopt,
                              SearchTerminationCause::kTimedOut};
        });
    check("empty search: schedule_result is populated",
          info.schedule_result.has_value());
    check("empty search: input order recorded (C,D,E,F)",
          order_names(result) == NameList{"C", "D", "E", "F"});
    check("empty search: termination_cause is kTimedOut",
          result.termination_cause ==
              SearchTerminationCause::kTimedOut);
  }

  // Case 2: the search produces a schedule — ScheduleSubgraph records
  // *that* order, not the input order. The functor reorders the
  // members to C,E,D,F (D and E each depend only on C and each feed
  // F, so swapping them is valid), distinguishing it from the input
  // order C,D,E,F.
  {
    SubgraphInfo info({pC, pD, pE, pF}, "S");
    const SubgraphScheduleResult &result = ScheduleSubgraph(
        info, *parent_graph, st, mf,
        [&st, &mf](const ScheduleGraph &graph) {
          const ScheduleNode *c = nullptr, *d = nullptr, *e = nullptr,
                             *f = nullptr, *entry = nullptr,
                             *exit = nullptr;
          for (const ScheduleNode &node : graph.Nodes()) {
            StringRef name = node.GetDebugName();
            if (name == "C") {
              c = &node;
            } else if (name == "D") {
              d = &node;
            } else if (name == "E") {
              e = &node;
            } else if (name == "F") {
              f = &node;
            } else if (name == "SubgraphEntry") {
              entry = &node;
            } else if (name == "SubgraphExit") {
              exit = &node;
            }
          }
          ScheduleConstructor sc(graph, st, mf);
          sc.Schedule(entry);
          sc.Schedule(c);
          sc.Schedule(e);
          sc.Schedule(d);
          sc.Schedule(f);
          sc.Schedule(exit);
          return SearchResult{std::move(sc),
                              SearchTerminationCause::kFullyExplored};
        });
    check("search schedule: that order recorded (C,E,D,F)",
          order_names(result) == NameList{"C", "E", "D", "F"});
    check("search schedule: termination_cause is kFullyExplored",
          result.termination_cause ==
              SearchTerminationCause::kFullyExplored);
  }

  // Case 3: drive ScheduleSubgraph with a real search strategy —
  // BfsDpSearch — on the extracted graph. On this 6-node subgraph
  // BFS-DP runs to completion, so the recorded order must be one of
  // the two valid member linearizations (C,D,E,F or C,E,D,F: D and E
  // are interchangeable).
  {
    SubgraphInfo info({pC, pD, pE, pF}, "S");
    const SubgraphScheduleResult &result = ScheduleSubgraph(
        info, *parent_graph, st, mf,
        [&st, &mf](const ScheduleGraph &graph) {
          BfsDpSearch search(&graph, &st, &mf, BfsDpSettings{});
          return search.Run();
        });
    NameList order = order_names(result);
    check("BFS-DP: recorded order is a valid member linearization",
          order == NameList{"C", "D", "E", "F"} ||
              order == NameList{"C", "E", "D", "F"});
    check("BFS-DP: termination_cause is kFullyExplored",
          result.termination_cause ==
              SearchTerminationCause::kFullyExplored);
  }
}

// Exercises ScheduleGraph::AddSubgraphOrderEdges. On BuildTestDAG,
// wraps {C, D, E, F} as subgraph S and records a non-trivial interior
// order — C, E, D, F. D and E each depend only on C and each feed F,
// so swapping them is still a valid topo order, distinct from the
// input order C, D, E, F. Checks that:
//   - with no schedule_result, AddSubgraphOrderEdges adds no edges;
//   - with the order recorded, the order-edge chain forces an
//     adversarially-driven ScheduleConstructor over the proxied graph
//     to replay exactly C, E, D, F, never offering more than one
//     member of S as ready at a time.
void RunAddSubgraphOrderEdgesShakedown(const GCNSubtarget &st,
                                       const MachineFunction &mf) {
  llvm::outs() << "  RunAddSubgraphOrderEdgesShakedown:\n";

  auto check = [](StringRef desc, bool ok) {
    llvm::outs() << "    " << desc << ": " << (ok ? "PASS" : "FAIL")
                 << "\n";
  };

  // Build BuildTestDAG and wrap {C, D, E, F} as subgraph S.
  // BuildTestDAG emplaces in order [A, C, D, E, F, G, H].
  auto build_proxied_graph = [] {
    auto graph = ScheduleGraph::BuildTestDAG();
    graph->ValidateAndComputeTopologicalOrder();
    SmallVector<ScheduleNode *, 4> members = {
        &graph->Nodes()[1], &graph->Nodes()[2], &graph->Nodes()[3],
        &graph->Nodes()[4]};
    std::vector<std::unique_ptr<SubgraphInfo>> infos;
    infos.push_back(std::make_unique<SubgraphInfo>(members, "S"));
    graph->InsertSubgraphProxies(std::move(infos));
    return graph;
  };

  // Case 1: with no schedule_result, AddSubgraphOrderEdges adds
  // nothing — the subgraph is left free.
  {
    auto graph = build_proxied_graph();
    auto count_edges = [&] {
      int edges = 0;
      for (const ScheduleNode &n : graph->Nodes()) {
        edges += n.NumSuccessors();
      }
      return edges;
    };
    int edges_before = count_edges();
    graph->AddSubgraphOrderEdges();
    check("no schedule_result: no order edges added",
          count_edges() == edges_before);
  }

  // Case 2: with a recorded order, the order-edge chain enforces it.
  {
    auto graph = build_proxied_graph();
    ScheduleNode *c = &graph->Nodes()[1];
    ScheduleNode *d = &graph->Nodes()[2];
    ScheduleNode *e = &graph->Nodes()[3];
    ScheduleNode *f = &graph->Nodes()[4];

    // Record the non-trivial interior order C, E, D, F on subgraph S
    // (the lone subgraph, so GetSubgraphInfos()[0]).
    SubgraphInfo *info = graph->GetSubgraphInfos()[0].get();
    info->schedule_result = SubgraphScheduleResult{
        /*order=*/{c, e, d, f},
        /*peak_pressure=*/GCNRegPressure{},
        /*termination_cause=*/SearchTerminationCause::kFullyExplored};
    graph->AddSubgraphOrderEdges();

    // Drive a ScheduleConstructor over the proxied + chained graph,
    // adversarially picking the last ready node each step. A correct
    // chain leaves the search no choice inside S.
    auto is_member = [&](const ScheduleNode *n) {
      return n == c || n == d || n == e || n == f;
    };
    ScheduleConstructor sc(*graph, st, mf);
    bool at_most_one_member_ready = true;
    bool ready_always_nonempty = true;
    while (!sc.IsDone()) {
      ArrayRef<const ScheduleNode *> ready = sc.GetReadyList();
      if (ready.empty()) {
        ready_always_nonempty = false;
        break;
      }
      int members_ready = 0;
      for (const ScheduleNode *n : ready) {
        if (is_member(n)) {
          ++members_ready;
        }
      }
      if (members_ready > 1) {
        at_most_one_member_ready = false;
      }
      sc.Schedule(ready.back());
    }

    std::vector<const ScheduleNode *> member_order;
    for (const ScheduleNode *n : sc.GetScheduleOrder()) {
      if (is_member(n)) {
        member_order.push_back(n);
      }
    }
    check("members emerge in the recorded order C, E, D, F",
          member_order ==
              std::vector<const ScheduleNode *>{c, e, d, f});
    check("at most one member of S ready at any step",
          at_most_one_member_ready);
    check("ready list never empty before the schedule completes",
          ready_always_nonempty);
  }
}

// Exercises the interleaving install path (ScheduleGraph::InstallSubgraphsFlat
// + AddSubgraphOrderEdges) — the SubgraphScheduleMode::kInterleaved
// counterpart to RunAddSubgraphOrderEdgesShakedown. Same subgraph
// S = {C, D, E, F} on BuildTestDAG with recorded interior order C, E, D, F,
// but installed WITHOUT proxies. H (A→H→G) runs parallel to S.
//
// Drives the ScheduleConstructor *adversarially* (always the last ready
// node), so the order-edge chain — not the picker — is what shapes the
// result. Checks that:
//   - members still emerge as C, E, D, F and never more than one member is
//     ready at a time: the chain enforces the interior order even without
//     proxies (drop the chain and the adversarial picker could reorder the
//     independent D/E);
//   - the free non-member H is ready *while the subgraph is mid-flight* (a
//     member scheduled, not all drained): the interleaving signature, which
//     the serialized proxy scope would forbid. H is deferred by the picker
//     so it stays available to observe — its readiness is the constructor's
//     doing, not ours.
void RunInterleavedSubgraphShakedown(const GCNSubtarget &st,
                                     const MachineFunction &mf) {
  llvm::outs() << "  RunInterleavedSubgraphShakedown:\n";

  auto check = [](StringRef desc, bool ok) {
    llvm::outs() << "    " << desc << ": " << (ok ? "PASS" : "FAIL") << "\n";
  };

  // BuildTestDAG emplaces [A, C, D, E, F, G, H].
  auto graph = ScheduleGraph::BuildTestDAG();
  graph->ValidateAndComputeTopologicalOrder();
  ScheduleNode *c = &graph->Nodes()[1];
  ScheduleNode *d = &graph->Nodes()[2];
  ScheduleNode *e = &graph->Nodes()[3];
  ScheduleNode *f = &graph->Nodes()[4];
  ScheduleNode *h = &graph->Nodes()[6];

  // Install S = {C, D, E, F} WITHOUT proxies (interleaving mode), then lock
  // the interior order C, E, D, F.
  {
    SmallVector<ScheduleNode *, 4> members = {c, d, e, f};
    std::vector<std::unique_ptr<SubgraphInfo>> infos;
    infos.push_back(std::make_unique<SubgraphInfo>(members, "S"));
    graph->InstallSubgraphsFlat(std::move(infos));
  }
  SubgraphInfo *info = graph->GetSubgraphInfos()[0].get();
  info->schedule_result = SubgraphScheduleResult{
      /*order=*/{c, e, d, f},
      /*peak_pressure=*/GCNRegPressure{},
      /*termination_cause=*/SearchTerminationCause::kFullyExplored};
  graph->AddSubgraphOrderEdges();

  auto is_member = [&](const ScheduleNode *n) {
    return n == c || n == d || n == e || n == f;
  };

  ScheduleConstructor sc(*graph, st, mf);
  bool at_most_one_member_ready = true;
  bool ready_always_nonempty = true;
  bool nonmember_ready_mid_subgraph = false;
  int members_scheduled = 0;
  while (!sc.IsDone()) {
    ArrayRef<const ScheduleNode *> ready = sc.GetReadyList();
    if (ready.empty()) {
      ready_always_nonempty = false;
      break;
    }
    int members_ready = 0;
    bool h_ready = false;
    for (const ScheduleNode *n : ready) {
      if (is_member(n)) {
        ++members_ready;
      } else if (n == h) {
        h_ready = true;
      }
    }
    if (members_ready > 1) {
      at_most_one_member_ready = false;
    }
    // Mid-subgraph: a member scheduled, subgraph not yet drained. H offered
    // here is the interleaving the serialized scope would forbid.
    if (members_scheduled >= 1 && members_scheduled < 4 && h_ready) {
      nonmember_ready_mid_subgraph = true;
    }
    // Adversarial pick: last ready node, but defer H so it stays available
    // to observe mid-subgraph.
    const ScheduleNode *pick = nullptr;
    for (int i = static_cast<int>(ready.size()) - 1; i >= 0; --i) {
      if (ready[i] != h) {
        pick = ready[i];
        break;
      }
    }
    if (!pick) {
      pick = h; // only H left (e.g. G still waits on H→G)
    }
    if (is_member(pick)) {
      ++members_scheduled;
    }
    sc.Schedule(pick);
  }

  std::vector<const ScheduleNode *> member_order;
  for (const ScheduleNode *n : sc.GetScheduleOrder()) {
    if (is_member(n)) {
      member_order.push_back(n);
    }
  }

  check("order-edge chain forces members out as C, E, D, F (adversarial drive)",
        member_order == std::vector<const ScheduleNode *>{c, e, d, f});
  check("at most one member of S ready at any step",
        at_most_one_member_ready);
  check("free non-member H is ready mid-subgraph (interleaving permitted)",
        nonmember_ready_mid_subgraph);
  check("ready list never empty before the schedule completes",
        ready_always_nonempty);
}

// Exercises the end-to-end DecomposeAndSchedule pipeline on the
// subgraph-formation test DAG: drives FormSubgraphs through
// AddSubgraphOrderEdges and the outer search in one call.
//   - subgraph_functor: BfsDpSearch on each extracted subgraph.
//   - outer_search: a ScheduleConstructor driven greedy-adversarially
//     over the proxied + chained graph (the chain leaves no choice
//     inside subgraphs; topo-last picking is fine outside).
// Verifies the returned schedule is complete and that each formed
// subgraph's members appear in step 3's schedule in exactly the order
// BFS-DP chose for it.
void RunDecomposeAndScheduleShakedown(const GCNSubtarget &st,
                                      const MachineFunction &mf) {
  llvm::outs() << "  RunDecomposeAndScheduleShakedown:\n";

  auto check = [](StringRef desc, bool ok) {
    llvm::outs() << "    " << desc << ": " << (ok ? "PASS" : "FAIL")
                 << "\n";
  };

  // Use the formation-test DAG: purpose-built so BottomUpDefault
  // emits a subgraph.
  auto graph = ScheduleGraph::BuildSubgraphFormationTestDAG();
  graph->ValidateAndComputeTopologicalOrder();
  graph->ComputeCriticalPaths();
  graph->PopulateInputScheduleConstructorByTopoOrderForTest(st, mf);

  auto inner_search =
      [&st, &mf](ScheduleGraph &sub) -> SearchResult {
    BfsDpSearch search(&sub, &st, &mf, BfsDpSettings{});
    return search.Run();
  };

  auto outer_search = [&st, &mf](ScheduleGraph &g) -> SearchResult {
    ScheduleConstructor sc(g, st, mf);
    while (!sc.IsDone()) {
      ArrayRef<const ScheduleNode *> ready = sc.GetReadyList();
      if (ready.empty()) {
        report_fatal_error(
            "outer_search: ready list empty while not done");
      }
      sc.Schedule(ready.back());
    }
    return SearchResult{std::move(sc),
                        SearchTerminationCause::kFullyExplored};
  };

  DecomposeAndScheduleOptions opts{
      /*formation=*/SubgraphFormationPolicy::BottomUpDefault(),
      /*mode=*/SubgraphScheduleMode::kSerialized,
      /*inner_search=*/inner_search,
      /*outer_search=*/outer_search};

  SearchResult result = DecomposeAndSchedule(*graph, st, mf, opts);

  check("DecomposeAndSchedule returned a schedule",
        result.schedule.has_value());
  if (!result.schedule.has_value()) {
    return;
  }
  const ScheduleConstructor &sc = *result.schedule;
  check("schedule is complete (IsDone)", sc.IsDone());

  // At least one subgraph was formed — sanity that the pipeline
  // actually exercised the form-and-lock path.
  check("at least one subgraph was formed",
        !graph->GetSubgraphInfos().empty());

  // For each formed subgraph: members appear in step 3's schedule in
  // exactly schedule_result.order — proving the chain locked them.
  ArrayRef<const ScheduleNode *> scheduled_order = sc.GetScheduleOrder();
  for (const std::unique_ptr<SubgraphInfo> &info_ptr : graph->GetSubgraphInfos()) {
    SubgraphInfo *info = info_ptr.get();
    if (!info->schedule_result.has_value()) {
      continue;
    }
    ArrayRef<ScheduleNode *> locked = info->schedule_result->order;
    std::set<const ScheduleNode *> member_set;
    for (ScheduleNode *m : locked) {
      member_set.insert(m);
    }
    std::vector<const ScheduleNode *> members_in_schedule;
    for (const ScheduleNode *n : scheduled_order) {
      if (member_set.count(n)) {
        members_in_schedule.push_back(n);
      }
    }
    bool order_matches =
        members_in_schedule.size() == locked.size();
    if (order_matches) {
      int n = static_cast<int>(members_in_schedule.size());
      for (int i = 0; i < n; ++i) {
        if (members_in_schedule[i] != locked[i]) {
          order_matches = false;
          break;
        }
      }
    }
    std::string desc = "subgraph \"" + info->debug_name +
                       "\": members in schedule match locked order";
    check(desc, order_matches);
  }
}

// Exercises the DecomposeAndScheduleOptions::BfsDpWithDfsFallback
// factory end-to-end: builds the formation-test DAG, asks the factory
// for a fully-wired options bundle (formation +
// BFS-DP-with-DFS-fallback inner_search + BFS-DP-with-DFS-fallback
// outer_search), and runs DecomposeAndSchedule with it. Verifies the
// pipeline returns a complete schedule and that each formed
// subgraph's locked-order property holds. The factory drives the
// production wiring path (vs. the hand-wired one in
// RunDecomposeAndScheduleShakedown above, which tests the pipeline
// mechanics in isolation).
//
// `seed_occupancy` is 1 — permissive: any non-spilling schedule
// beats it, so the outer BFS-DP's score-bound prune doesn't squeeze
// the test DAG into the no-schedule path.
void RunDecomposeAndScheduleFactoryShakedown(const GCNSubtarget &st,
                                             const MachineFunction &mf,
                                             const LiveIntervals &lis) {
  llvm::outs() << "  RunDecomposeAndScheduleFactoryShakedown:\n";

  auto check = [](StringRef desc, bool ok) {
    llvm::outs() << "    " << desc << ": " << (ok ? "PASS" : "FAIL")
                 << "\n";
  };

  auto graph = ScheduleGraph::BuildSubgraphFormationTestDAG();
  graph->ValidateAndComputeTopologicalOrder();
  graph->ComputeCriticalPaths();
  graph->PopulateInputScheduleConstructorByTopoOrderForTest(st, mf);

  DecomposeAndScheduleOptions opts =
      DecomposeAndScheduleOptions::BfsDpWithDfsFallback(
          st, mf, lis, /*seed_occupancy=*/1,
          FormationConfig{SubgraphFormationStrategy::kDomTree});

  SearchResult result = DecomposeAndSchedule(*graph, st, mf, opts);

  check("DecomposeAndSchedule returned a schedule",
        result.schedule.has_value());
  if (!result.schedule.has_value()) {
    return;
  }
  const ScheduleConstructor &sc = *result.schedule;
  check("schedule is complete (IsDone)", sc.IsDone());

  // The test DAG is purpose-built for subgraph formation, and the
  // kDomTree strategy passed above (realized as
  // TopDownSingleSplitterOnly) emits at least one subgraph on it —
  // verified by the earlier formation shakedown. If this check ever
  // fails, either the DAG or the dom-tree formation has drifted.
  check("at least one subgraph was formed",
        !graph->GetSubgraphInfos().empty());

  // ScheduleSubgraph's contract is to always populate
  // schedule_result (BFS-DP success, DFS fallback, or input-order
  // fallback). The chain installed by AddSubgraphOrderEdges then
  // forces those members into the outer schedule in exactly the
  // recorded order.
  ArrayRef<const ScheduleNode *> scheduled_order = sc.GetScheduleOrder();
  for (const std::unique_ptr<SubgraphInfo> &info_ptr : graph->GetSubgraphInfos()) {
    SubgraphInfo *info = info_ptr.get();
    if (!info->schedule_result.has_value()) {
      report_fatal_error("RunDecomposeAndScheduleFactoryShakedown: "
                         "ScheduleSubgraph failed to populate "
                         "schedule_result — contract violation");
    }
    ArrayRef<ScheduleNode *> locked = info->schedule_result->order;
    std::set<const ScheduleNode *> member_set;
    for (ScheduleNode *m : locked) {
      member_set.insert(m);
    }
    std::vector<const ScheduleNode *> members_in_schedule;
    for (const ScheduleNode *n : scheduled_order) {
      if (member_set.count(n)) {
        members_in_schedule.push_back(n);
      }
    }
    bool order_matches =
        members_in_schedule.size() == locked.size();
    if (order_matches) {
      int n = static_cast<int>(members_in_schedule.size());
      for (int i = 0; i < n; ++i) {
        if (members_in_schedule[i] != locked[i]) {
          order_matches = false;
          break;
        }
      }
    }
    std::string desc = "subgraph \"" + info->debug_name +
                       "\": members in schedule match locked order";
    check(desc, order_matches);
  }
}

// ---- Score type: Higher/Lower wrappers, Make, comparison operators ----

void RunScoreShakedown() {
  llvm::outs() << "  Score shakedown:\n";

  using H = Score::Higher;
  using L = Score::Lower;

  auto check = [&](const char *desc, bool ok) {
    llvm::outs() << "    " << desc << (ok ? "  PASS" : "  FAIL") << "\n";
    if (!ok) {
      report_fatal_error("Score shakedown: failure");
    }
  };

  // Single-slot Higher: larger raw value scores higher.
  {
    Score a = Score::Make(H{10});
    Score b = Score::Make(H{5});
    check("single Higher{10} > Higher{5}", a > b);
    check("single Higher{5} < Higher{10}", b < a);
    check("single Higher{10} != Higher{5}", a != b);
    check("single Higher{5} <= Higher{10}", b <= a);
  }

  // Single-slot Lower: smaller raw value scores higher (Lower inverts).
  {
    Score a = Score::Make(L{5});   // canonical: -5
    Score b = Score::Make(L{10});  // canonical: -10
    check("single Lower{5} > Lower{10} (lower raw is better)", a > b);
    check("single Lower{10} < Lower{5}", b < a);
  }

  // Two-slot lex: primary dominates when it differs.
  {
    Score occ4 = Score::Make(H{4}, L{80});
    Score occ3 = Score::Make(H{3}, L{80});
    check("primary slot wins: (occ=4,len=80) > (occ=3,len=80)",
          occ4 > occ3);
  }

  // Two-slot lex: tiebreak by secondary when primary ties.
  {
    Score occ4_long = Score::Make(H{4}, L{120});
    Score occ4_short = Score::Make(H{4}, L{80});
    check("tiebreak by secondary: same occ, shorter len wins",
          occ4_short > occ4_long);
  }

  // Equality on identical slots.
  {
    Score a = Score::Make(H{4}, L{80});
    Score b = Score::Make(H{4}, L{80});
    check("identical Scores: ==", a == b);
    check("identical Scores: !(a < b)", !(a < b));
    check("identical Scores: !(a > b)", !(a > b));
    check("identical Scores: a <= b", a <= b);
    check("identical Scores: a >= b", a >= b);
  }

  // Trailing slots default to 0: Make(H{4}) == Make(H{4}, L{0}).
  {
    Score one_slot = Score::Make(H{4});
    Score with_trailing_zero = Score::Make(H{4}, L{0});
    check("Make(H{4}) == Make(H{4}, L{0}) (trailing zero canonical)",
          one_slot == with_trailing_zero);
  }

  // Three-slot lex: tertiary tiebreak.
  {
    Score a = Score::Make(L{100}, H{50}, H{1000});
    Score b = Score::Make(L{100}, H{50}, H{500});
    check("same primary + secondary: tertiary tiebreaks", a > b);
  }
}

// ---- OccupancyTargetUtil: LimitOccupancyAboveFloor wrapper ----

void RunOccupancyTargetUtilShakedown(const MachineFunction &mf) {
  llvm::outs() << "  OccupancyTargetUtil shakedown:\n";

  SIMachineFunctionInfo *mfi =
      const_cast<MachineFunction &>(mf).getInfo<SIMachineFunctionInfo>();
  const unsigned saved_occ = mfi->getOccupancy();
  const unsigned floor = mfi->getMinWavesPerEU();

  auto restore = [&]() { mfi->increaseOccupancy(mf, saved_occ); };

  auto check = [&](const char *desc, bool ok) {
    llvm::outs() << "    " << desc << (ok ? "  PASS" : "  FAIL") << "\n";
    if (!ok) {
      restore();
      report_fatal_error("OccupancyTargetUtil shakedown: failure");
    }
  };

  llvm::outs() << "    saved_occ=" << saved_occ << " floor=" << floor << "\n";

  // Case 1: limit > current -> no-op.
  hierarchical_scheduler::LimitOccupancyAboveFloor(
      *mfi, static_cast<int>(saved_occ) + 100);
  check("limit > current: target unchanged",
        mfi->getOccupancy() == saved_occ);

  // Case 2: floor < limit < current -> set to limit. Skip if no room.
  if (saved_occ > floor + 1) {
    const unsigned mid = floor + 1;
    hierarchical_scheduler::LimitOccupancyAboveFloor(
        *mfi, static_cast<int>(mid));
    check("floor < limit < current: target == limit",
          mfi->getOccupancy() == mid);
  } else {
    llvm::outs() << "    skipping mid-range case "
                    "(saved_occ <= floor + 1)\n";
  }

  // Case 3: limit < floor -> clamped to floor.
  hierarchical_scheduler::LimitOccupancyAboveFloor(*mfi, 0);
  check("limit < floor: target clamped to floor",
        mfi->getOccupancy() == floor);

  // Restore.
  restore();
  check("MFI->Occupancy restored to saved value",
        mfi->getOccupancy() == saved_occ);
}

// ---- GCNRegisterTracker: effective occupancy + per-track helpers ----

void RunEffectiveAndTargetLimitHelpersShakedown(
    ScheduleGraph &graph, const MachineFunction &mf) {
  llvm::outs() << "  EffectiveAndTargetLimitHelpers shakedown:\n";

  const GCNSubtarget &st =
      static_cast<const GCNSubtarget &>(mf.getSubtarget());
  const unsigned sgpr_addressable =
      AMDGPU::IsaInfo::getAddressableNumSGPRs(&st);

  ScheduleConstructor sc(graph, st, mf);
  auto &tr = sc.GetPressureTrackerForTest();

  auto check = [&](const char *desc, bool ok) {
    llvm::outs() << "      " << desc << (ok ? "  PASS" : "  FAIL") << "\n";
    if (!ok) {
      tr.ClearTargetAndFloorOverridesForTest();
      report_fatal_error(
          "EffectiveAndTargetLimitHelpers shakedown: failure");
    }
  };

  // Run all the boundary checks for a given (target, floor) pair,
  // driven by the tracker's test overrides so we don't depend on the
  // test MF's actual occupancy or launch-attribute state.
  auto run_scenario = [&](unsigned target, unsigned floor) {
    tr.SetTargetOccupancyForTest(target);
    tr.SetOccupancyFloorForTest(floor);

    const unsigned vgpr_limit_at_target = st.getMaxNumVGPRs(target);
    const unsigned sgpr_limit_at_target =
        st.getMaxNumSGPRs(target, /*Addressable=*/true);
    const unsigned vgpr_limit_at_floor = st.getMaxNumVGPRs(floor);
    const unsigned sgpr_limit_at_floor =
        st.getMaxNumSGPRs(floor, /*Addressable=*/true);
    const bool sgpr_cliff_at_target =
        sgpr_limit_at_target < sgpr_addressable;
    const bool sgpr_cliff_at_floor =
        sgpr_limit_at_floor < sgpr_addressable;

    llvm::outs() << "    scenario target=" << target << " floor=" << floor
                 << " vgpr@target=" << vgpr_limit_at_target
                 << " sgpr@target=" << sgpr_limit_at_target
                 << " vgpr@floor=" << vgpr_limit_at_floor
                 << " sgpr_cliff_at_{target,floor}={"
                 << (sgpr_cliff_at_target ? "y" : "n") << ","
                 << (sgpr_cliff_at_floor ? "y" : "n") << "}\n";

    // -- VGPR cur-pressure target-limit boundary --

    // Well-below limit.
    {
      const unsigned vgpr = 4;
      tr.SetCurPressureForTest(GCNRegPressure(vgpr, /*sgpr32=*/0));
      check("VGPR well-below: IsAtOrBelow=true",
            tr.IsCurVGPRCountAtOrBelowTargetLimit());
      check("VGPR well-below: IsAbove=false",
            !tr.IsCurVGPRCountAboveTargetLimit());
      check("VGPR well-below: count_below = limit - cur",
            tr.GetCurVGPRCountBelowTargetLimit() ==
                (vgpr_limit_at_target - vgpr));
      check("VGPR well-below: count_above = 0",
            tr.GetCurVGPRCountAboveTargetLimit() == 0);
    }
    // At limit.
    {
      tr.SetCurPressureForTest(
          GCNRegPressure(vgpr_limit_at_target, /*sgpr32=*/0));
      check("VGPR at-limit: IsAtOrBelow=true",
            tr.IsCurVGPRCountAtOrBelowTargetLimit());
      check("VGPR at-limit: IsAbove=false",
            !tr.IsCurVGPRCountAboveTargetLimit());
      check("VGPR at-limit: count_below=0",
            tr.GetCurVGPRCountBelowTargetLimit() == 0);
      check("VGPR at-limit: count_above=0",
            tr.GetCurVGPRCountAboveTargetLimit() == 0);
    }
    // Above limit by 7.
    {
      const unsigned vgpr = vgpr_limit_at_target + 7;
      tr.SetCurPressureForTest(GCNRegPressure(vgpr, /*sgpr32=*/0));
      check("VGPR above-by-7: IsAtOrBelow=false",
            !tr.IsCurVGPRCountAtOrBelowTargetLimit());
      check("VGPR above-by-7: IsAbove=true",
            tr.IsCurVGPRCountAboveTargetLimit());
      check("VGPR above-by-7: count_below=0",
            tr.GetCurVGPRCountBelowTargetLimit() == 0);
      check("VGPR above-by-7: count_above=7",
            tr.GetCurVGPRCountAboveTargetLimit() == 7);
    }

    // -- SGPR cur-pressure target-limit boundary (only when a real
    //    SGPR cliff exists at this target) --

    if (sgpr_cliff_at_target) {
      // Well-below.
      {
        const unsigned sgpr = 4;
        tr.SetCurPressureForTest(
            GCNRegPressure(/*vgpr32=*/0, /*sgpr32=*/sgpr));
        check("SGPR well-below: IsAtOrBelow=true",
              tr.IsCurSGPRCountAtOrBelowTargetLimit());
        check("SGPR well-below: IsAbove=false",
              !tr.IsCurSGPRCountAboveTargetLimit());
        check("SGPR well-below: count_below = limit - cur",
              tr.GetCurSGPRCountBelowTargetLimit() ==
                  (sgpr_limit_at_target - sgpr));
        check("SGPR well-below: count_above = 0",
              tr.GetCurSGPRCountAboveTargetLimit() == 0);
      }
      // At limit.
      {
        tr.SetCurPressureForTest(GCNRegPressure(
            /*vgpr32=*/0, /*sgpr32=*/sgpr_limit_at_target));
        check("SGPR at-limit: IsAtOrBelow=true",
              tr.IsCurSGPRCountAtOrBelowTargetLimit());
        check("SGPR at-limit: IsAbove=false",
              !tr.IsCurSGPRCountAboveTargetLimit());
        check("SGPR at-limit: count_below=0",
              tr.GetCurSGPRCountBelowTargetLimit() == 0);
        check("SGPR at-limit: count_above=0",
              tr.GetCurSGPRCountAboveTargetLimit() == 0);
      }
      // Above limit by 5.
      {
        const unsigned sgpr = sgpr_limit_at_target + 5;
        tr.SetCurPressureForTest(
            GCNRegPressure(/*vgpr32=*/0, /*sgpr32=*/sgpr));
        check("SGPR above-by-5: IsAtOrBelow=false",
              !tr.IsCurSGPRCountAtOrBelowTargetLimit());
        check("SGPR above-by-5: IsAbove=true",
              tr.IsCurSGPRCountAboveTargetLimit());
        check("SGPR above-by-5: count_below=0",
              tr.GetCurSGPRCountBelowTargetLimit() == 0);
        check("SGPR above-by-5: count_above=5",
              tr.GetCurSGPRCountAboveTargetLimit() == 5);
      }
    } else {
      llvm::outs() << "      skipping SGPR target-limit tests "
                      "(no cliff at this target)\n";
    }

    // -- Cur-pressure spill predicates --

    // VGPR cur spill: cur_pressure_'s VGPR one above the floor's
    // VGPR cap. Test at any floor (floor=1 -> vgpr@floor=256, test
    // value is 257).
    {
      tr.SetCurPressureForTest(
          GCNRegPressure(vgpr_limit_at_floor + 1, /*sgpr32=*/0));
      check("VGPR cur spill: IsCurVGPRInSpillRegime=true",
            tr.IsCurVGPRInSpillRegime());
      check("VGPR cur spill: !IsCurSGPRInSpillRegime",
            !tr.IsCurSGPRInSpillRegime());
      check("VGPR cur spill: IsCurInSpillRegime=true",
            tr.IsCurInSpillRegime());
    }
    // SGPR cur spill: cur_pressure_'s SGPR one above the floor's
    // SGPR cap. Skip when there's no SGPR cliff at this floor.
    if (sgpr_cliff_at_floor) {
      tr.SetCurPressureForTest(
          GCNRegPressure(/*vgpr32=*/0, sgpr_limit_at_floor + 1));
      check("SGPR cur spill: IsCurSGPRInSpillRegime=true",
            tr.IsCurSGPRInSpillRegime());
      check("SGPR cur spill: !IsCurVGPRInSpillRegime",
            !tr.IsCurVGPRInSpillRegime());
      check("SGPR cur spill: IsCurInSpillRegime=true",
            tr.IsCurInSpillRegime());
    } else {
      llvm::outs() << "      skipping SGPR cur spill (no cliff at floor)\n";
    }

    // -- VGPR cur-pressure spill-cap count boundary --

    // Well-below the spill cap.
    {
      const unsigned vgpr = 4;
      tr.SetCurPressureForTest(GCNRegPressure(vgpr, /*sgpr32=*/0));
      check("VGPR well-below-cap: count_below = cap - cur",
            tr.GetCurVGPRCountBelowSpillCap() ==
                (vgpr_limit_at_floor - vgpr));
      check("VGPR well-below-cap: count_above = 0",
            tr.GetCurVGPRCountAboveSpillCap() == 0);
    }
    // At the spill cap.
    {
      tr.SetCurPressureForTest(
          GCNRegPressure(vgpr_limit_at_floor, /*sgpr32=*/0));
      check("VGPR at-cap: count_below = 0",
            tr.GetCurVGPRCountBelowSpillCap() == 0);
      check("VGPR at-cap: count_above = 0",
            tr.GetCurVGPRCountAboveSpillCap() == 0);
    }
    // Above the spill cap by 9.
    {
      const unsigned vgpr = vgpr_limit_at_floor + 9;
      tr.SetCurPressureForTest(GCNRegPressure(vgpr, /*sgpr32=*/0));
      check("VGPR above-cap-by-9: count_below = 0",
            tr.GetCurVGPRCountBelowSpillCap() == 0);
      check("VGPR above-cap-by-9: count_above = 9",
            tr.GetCurVGPRCountAboveSpillCap() == 9);
    }

    // -- SGPR cur-pressure spill-cap count boundary (only when a
    //    real SGPR cliff exists at this floor) --

    if (sgpr_cliff_at_floor) {
      // Well-below.
      {
        const unsigned sgpr = 4;
        tr.SetCurPressureForTest(
            GCNRegPressure(/*vgpr32=*/0, /*sgpr32=*/sgpr));
        check("SGPR well-below-cap: count_below = cap - cur",
              tr.GetCurSGPRCountBelowSpillCap() ==
                  (sgpr_limit_at_floor - sgpr));
        check("SGPR well-below-cap: count_above = 0",
              tr.GetCurSGPRCountAboveSpillCap() == 0);
      }
      // At cap.
      {
        tr.SetCurPressureForTest(GCNRegPressure(
            /*vgpr32=*/0, /*sgpr32=*/sgpr_limit_at_floor));
        check("SGPR at-cap: count_below = 0",
              tr.GetCurSGPRCountBelowSpillCap() == 0);
        check("SGPR at-cap: count_above = 0",
              tr.GetCurSGPRCountAboveSpillCap() == 0);
      }
      // Above cap by 6.
      {
        const unsigned sgpr = sgpr_limit_at_floor + 6;
        tr.SetCurPressureForTest(
            GCNRegPressure(/*vgpr32=*/0, /*sgpr32=*/sgpr));
        check("SGPR above-cap-by-6: count_below = 0",
              tr.GetCurSGPRCountBelowSpillCap() == 0);
        check("SGPR above-cap-by-6: count_above = 6",
              tr.GetCurSGPRCountAboveSpillCap() == 6);
      }
    } else {
      llvm::outs() << "      skipping SGPR spill-cap count boundary "
                      "(no cliff at floor)\n";
    }

    // -- Peak-pressure: GetEffectiveOccupancy + spill predicates --

    // Healthy peak: max_pressure_ at target's VGPR limit
    // -> reg-only = target, effective = reg-only, no spill.
    {
      tr.SetMaxPressureForTest(
          GCNRegPressure(vgpr_limit_at_target, /*sgpr32=*/0));
      const unsigned reg_only = tr.GetRegisterOnlyOccupancy();
      const unsigned effective = tr.GetEffectiveOccupancy();
      check("healthy peak: !IsPeakVGPRInSpillRegime",
            !tr.IsPeakVGPRInSpillRegime());
      check("healthy peak: !IsPeakSGPRInSpillRegime",
            !tr.IsPeakSGPRInSpillRegime());
      check("healthy peak: !IsPeakInSpillRegime",
            !tr.IsPeakInSpillRegime());
      check("healthy peak: reg_only >= floor && effective == reg_only",
            reg_only >= floor && effective == reg_only);
    }

    // VGPR peak spill: max_pressure_'s VGPR one above floor's cap.
    {
      tr.SetMaxPressureForTest(
          GCNRegPressure(vgpr_limit_at_floor + 1, /*sgpr32=*/0));
      check("VGPR peak spill: IsPeakVGPRInSpillRegime=true",
            tr.IsPeakVGPRInSpillRegime());
      check("VGPR peak spill: !IsPeakSGPRInSpillRegime",
            !tr.IsPeakSGPRInSpillRegime());
      check("VGPR peak spill: IsPeakInSpillRegime=true",
            tr.IsPeakInSpillRegime());
      check("VGPR peak spill: effective == floor",
            tr.GetEffectiveOccupancy() == floor);
    }

    // SGPR peak spill: max_pressure_'s SGPR one above floor's cap.
    // Skip if no SGPR cliff at floor.
    if (sgpr_cliff_at_floor) {
      tr.SetMaxPressureForTest(
          GCNRegPressure(/*vgpr32=*/0, sgpr_limit_at_floor + 1));
      check("SGPR peak spill: IsPeakSGPRInSpillRegime=true",
            tr.IsPeakSGPRInSpillRegime());
      check("SGPR peak spill: !IsPeakVGPRInSpillRegime",
            !tr.IsPeakVGPRInSpillRegime());
      check("SGPR peak spill: IsPeakInSpillRegime=true",
            tr.IsPeakInSpillRegime());
    } else {
      llvm::outs() << "      skipping SGPR peak spill (no cliff at floor)\n";
    }
  };

  // Sweep three (target, floor) scenarios:
  //   target=10, floor=1 -> VGPR cliff at 24, SGPR cliff at 80,
  //                          VGPR spill at 257.
  //   target=8,  floor=4 -> VGPR cliff at 32, SGPR cliff at 100,
  //                          VGPR spill at 65.
  //   target=4,  floor=1 -> VGPR cliff at 64, no SGPR cliff at target,
  //                          VGPR spill at 257.
  run_scenario(10, 1);
  run_scenario(8, 4);
  run_scenario(4, 1);

  tr.ClearTargetAndFloorOverridesForTest();
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
  RunTestDAGShakedown(st);
  RunInsertSubgraphProxiesShakedown();
  RunSubgraphContiguityShakedown(MF, *LIS);
  RunBuildFromNodeSubsetShakedown(st, MF);
  RunScheduleSubgraphShakedown(st, MF);
  RunAddSubgraphOrderEdgesShakedown(st, MF);
  RunInterleavedSubgraphShakedown(st, MF);
  RunDecomposeAndScheduleShakedown(st, MF);
  RunDecomposeAndScheduleFactoryShakedown(st, MF, *LIS);
  RunLengthLowerBoundShakedown(st);
  RunScheduledSetTrackerShakedown(st);
  RunLengthHistoryTrackerShakedown(st);
  RunPressureHistoryTrackerShakedown(st);
  RunLengthHistoryDfsComparisonShakedown(st, MF, *LIS);
  RunPressureHistoryDfsComparisonShakedown(st, MF, *LIS);
  RunBfsDpVsDfsShakedown(st, MF, *LIS);
  RunDfsAreaTiebreakShakedown(st, MF, *LIS);
  RunAllSubgraphFormationShakedowns();
  RunScoreShakedown();
  RunOccupancyTargetUtilShakedown(MF);
  RunVGPRSpillAreaAccumulatorShakedown(MF);

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

  // FormSubgraphs smoke test on the first region's graph. Runs in
  // its own WithRegionGraph block so it gets a clean (no-proxy)
  // ScheduleGraph — RunRegionShakedowns above operates on the
  // unmutated graph it was originally written against, and we
  // don't want FormSubgraphs to mutate that out from under it.
  if (!regions_.empty()) {
    WithRegionGraph(regions_.front(), [&](ScheduleGraph &graph) {
      RunFormSubgraphsRealRegionSmokeTest(graph);
    });
  }
}
