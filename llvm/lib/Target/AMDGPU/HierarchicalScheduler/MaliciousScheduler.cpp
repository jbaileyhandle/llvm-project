//===- MaliciousScheduler.cpp - Pressure-maximizing scheduler -------------===//
//
// Implementation of the malicious scheduling heuristic. See
// MaliciousScheduler.h for the scoring rules.
//
//===----------------------------------------------------------------------===//

#include "MaliciousScheduler.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include <algorithm>
#include <vector>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Returns true if |su| is the last consumer of any of its input registers.
// "Last consumer" means: for some Data predecessor of |su|, |su| is the only
// remaining unscheduled Data successor of that predecessor. If so, scheduling
// |su| would end that register's live range.
//
// We only check Data edges (SDep::Data), not Anti or Output, because only
// Data edges represent true "producer writes register, consumer reads it"
// relationships that create live ranges. Anti (WAR) and Output (WAW) are
// ordering constraints, not value-flow dependencies. In pre-RA scheduling,
// Anti/Output edges mostly arise from physical register conflicts (e.g.,
// $vcc, $exec), not virtual registers.
bool IsLastConsumerOfAnyInput(const SUnit &su) {
  for (const SDep &pred : su.Preds) {
    if (pred.getKind() != SDep::Data) {
      continue;
    }
    const SUnit *producer = pred.getSUnit();

    int unscheduled_data_succs = 0;
    for (const SDep &succ : producer->Succs) {
      if (succ.getKind() == SDep::Data && !succ.getSUnit()->isScheduled) {
        ++unscheduled_data_succs;
      }
    }

    if (unscheduled_data_succs == 1) {
      return true;
    }
  }
  return false;
}

// Score a candidate instruction for the malicious heuristic.
// Higher score = more preferred (worse for performance).
int ScoreCandidate(const SUnit &candidate, const SUnit *last_scheduled) {
  int score = 0;

  // +2 if the just-scheduled instruction is a direct predecessor (producer)
  // of this candidate. Favors back-to-back dependent instructions, which
  // minimizes the gap between producer and consumer — bad for latency hiding.
  if (last_scheduled != nullptr) {
    for (const SDep &pred : candidate.Preds) {
      if (pred.getSUnit() == last_scheduled) {
        score += 2;
        break;
      }
    }
  }

  // +1 if this instruction is NOT the last consumer of any input register.
  // Favors keeping live ranges open longer, which increases register pressure.
  if (!IsLastConsumerOfAnyInput(candidate)) {
    score += 1;
  }

  return score;
}

// Build the initial ready list: all SUnits with no unscheduled strong
// predecessors (NumPredsLeft == 0).
//
// NumPredsLeft is initialized during buildSchedGraph — each call to
// SUnit::addPred increments it for strong edges. By the time the DAG is
// built, NumPredsLeft reflects the total number of strong predecessors.
//
// We iterate over the SUnits vector, which contains only real instructions.
// EntrySU and ExitSU are special boundary nodes that represent "everything
// before the region" and "everything after the region" respectively. They
// anchor dependency edges for live-in and live-out values but do not
// correspond to real instructions. They are separate members of ScheduleDAG
// (not in the SUnits vector), so they cannot accidentally enter the ready
// list.
std::vector<SUnit *> InitReadyList(std::vector<SUnit> &sunits) {
  std::vector<SUnit *> ready_list;
  for (SUnit &su : sunits) {
    if (su.NumPredsLeft == 0) {
      ready_list.push_back(&su);
    }
  }
  return ready_list;
}

// Find the highest-scoring candidate in the ready list.
SUnit *FindBestCandidate(const std::vector<SUnit *> &ready_list,
                         const SUnit *last_scheduled) {
  int best_score = -1;
  SUnit *best_candidate = nullptr;
  for (SUnit *su : ready_list) {
    int score = ScoreCandidate(*su, last_scheduled);
    if (score > best_score) {
      best_score = score;
      best_candidate = su;
    }
  }
  return best_candidate;
}

// Release the successors of a just-scheduled SUnit. For each strong
// successor, decrement NumPredsLeft. If it reaches zero, add to the
// ready list.
//
// We skip weak edges (Cluster, Weak) because they don't affect readiness —
// NumPredsLeft only tracks strong predecessors. Decrementing for weak edges
// would release instructions before their real dependencies are satisfied.
// This matches ScheduleDAGMI::releaseSucc behavior.
//
// We skip boundary nodes (EntrySU, ExitSU) because they are not real
// instructions and should never be scheduled. EntrySU represents the start
// of the region (predecessor for live-in values) and ExitSU represents the
// end (successor for live-out values). They can appear as successors in the
// DAG but must not be added to the ready list or have their counters
// decremented.
void ReleaseSuccessors(SUnit *scheduled_su,
                       std::vector<SUnit *> &ready_list) {
  for (SDep &succ_dep : scheduled_su->Succs) {
    if (succ_dep.isWeak()) {
      continue;
    }
    SUnit *succ = succ_dep.getSUnit();
    if (succ->isBoundaryNode()) {
      continue;
    }
    --succ->NumPredsLeft;
    if (succ->NumPredsLeft == 0 && !succ->isScheduled) {
      ready_list.push_back(succ);
    }
  }
}

// Mark an SUnit as scheduled, remove it from the ready list, and release
// its successors.
void ScheduleInstruction(SUnit *su, std::vector<SUnit *> &ready_list) {
  su->isScheduled = true;
  ready_list.erase(std::find(ready_list.begin(), ready_list.end(), su));
  ReleaseSuccessors(su, ready_list);
}

} // namespace

std::vector<SUnit *>
hierarchical_scheduler::ComputeMaliciousSchedule(std::vector<SUnit> &sunits) {
  std::vector<SUnit *> scheduled_order;
  scheduled_order.reserve(sunits.size());

  std::vector<SUnit *> ready_list = InitReadyList(sunits);
  SUnit *last_scheduled = nullptr;

  while (!ready_list.empty()) {
    SUnit *best = FindBestCandidate(ready_list, last_scheduled);
    ScheduleInstruction(best, ready_list);
    scheduled_order.push_back(best);
    last_scheduled = best;
  }

  return scheduled_order;
}
