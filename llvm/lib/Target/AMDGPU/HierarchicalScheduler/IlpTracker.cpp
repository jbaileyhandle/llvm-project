//===- IlpTracker.cpp - Producer-to-first-consumer ILP tracking -----------===//
//
// Implementation of producer-to-first-consumer ILP tracking.
//
// See IlpTracker.h for design rationale.
//
//===----------------------------------------------------------------------===//

#include "IlpTracker.h"
#include "GCNRegisterTracker.h"
#include "SIInstrInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Op-type weight for ILP. Higher = scheduling extra issue work
// during this op's latency window matters more. Numbers are
// ballpark, not derived from a precise latency table — the AMDGPU
// timing model is too coarse to justify finer granularity.
//
// VMEM / FLAT (memory loads): hundreds of cycles latency.
// DS (LDS): tens of cycles latency.
// Default (VALU / SALU): single-digit, often 1.
//
// Easy to tune as a unit later. The relative ordering is what
// drives the ranking heuristic; absolute magnitudes set the trade
// vs other scoring axes (length, occupancy).
int IlpWeightForOp(const MachineInstr *mi) {
  if (SIInstrInfo::isVMEM(*mi) || SIInstrInfo::isFLAT(*mi)) {
    return 8;
  }
  if (SIInstrInfo::isDS(*mi)) {
    return 4;
  }
  return 1;
}

// True iff `node` is a real, MachineInstr-backed scheduling unit
// (not a proxy, not an entry/exit sentinel). Sentinels are
// scheduling units (IsSchedulingUnit() true) but lack a backing
// MachineInstr, distinguished here via GetSUnit()->getInstr().
bool IsRealInstruction(const ScheduleNode *node) {
  if (!node->IsSchedulingUnit()) {
    return false;
  }
  SUnit *su = node->GetSUnit();
  return su != nullptr && su->getInstr() != nullptr;
}

}  // namespace

// ============================================================================
// Construction
// ============================================================================

IlpTracker::IlpTracker(const ScheduleGraph &graph,
                       const GCNRegisterTracker &pressure_tracker)
    : pressure_tracker_(&pressure_tracker),
      weight_by_topo_index_(graph.Size(), 0) {
  // Precompute per-node op-type weight. Only real MachineInstr-backed
  // nodes get a non-zero weight; everything else stays at 0 (the
  // initialized value).
  for (const ScheduleNode &node : graph.Nodes()) {
    if (!IsRealInstruction(&node)) {
      continue;
    }
    weight_by_topo_index_[node.GetTopoIndex()] =
        IlpWeightForOp(node.GetSUnit()->getInstr());
  }
}

// ============================================================================
// Schedule / Unschedule
// ============================================================================

void IlpTracker::Schedule(const ScheduleNode *node) {
  // Subgraph proxies are scope bookkeeping, not scheduling work.
  // Full no-op (don't push an undo record).
  if (!node->IsSchedulingUnit()) {
    return;
  }

  undo_stack_.emplace_back();
  UndoRecord &undo = undo_stack_.back();

  // Close side: applies to every scheduling-unit Schedule (real MI
  // OR sentinel). The exit sentinel's uses are live-out vregs; this
  // is how we capture live-out producers' contributions.
  ProcessUsesAsCloses(node, undo);

  if (!IsRealInstruction(node)) {
    // Sentinel: no defs to open, no count bump. Done.
    return;
  }

  // Open side + count bump: real instructions only.
  OpenDefs(node, undo);
  instructions_issued_count_ += 1;
}

void IlpTracker::Unschedule(const ScheduleNode *node) {
  if (!node->IsSchedulingUnit()) {
    return;
  }
  if (undo_stack_.empty()) {
    report_fatal_error(
        "IlpTracker::Unschedule called with empty undo stack");
  }

  // Reverse Schedule's order: count first, then opens, then closes.
  if (IsRealInstruction(node)) {
    instructions_issued_count_ -= 1;
  }

  UndoRecord undo = std::move(undo_stack_.back());
  undo_stack_.pop_back();
  EraseOpens(undo);
  ReinsertCloses(undo);
}

// ============================================================================
// Schedule / Unschedule helpers
// ============================================================================

void IlpTracker::CloseOneProducer(unsigned reg, int contribution,
                                  UndoRecord &undo) {
  auto it = open_producer_by_reg_.find(reg);
  if (it == open_producer_by_reg_.end()) {
    report_fatal_error(
        "IlpTracker::CloseOneProducer: reg " + Twine(reg) +
        " is not in the open map. Caller precondition violated.");
  }
  const OpenProducer &producer = it->second;
  closed_ilp_score_ += contribution;
  undo.closed.push_back(
      {reg, producer.inst_count, producer.weight, contribution});
  sum_open_weights_ -= producer.weight;
  sum_open_weighted_indices_ -= producer.weight * producer.inst_count;
  open_producer_by_reg_.erase(it);
}

void IlpTracker::ProcessUsesAsCloses(const ScheduleNode *node,
                                     UndoRecord &undo) {
  const GCNRegisterTracker::NodeRegInfo &info =
      pressure_tracker_->GetNodeRegInfo(node);
  for (const GCNRegisterTracker::RegMask &use : info.uses) {
    auto it = open_producer_by_reg_.find(use.reg);
    if (it == open_producer_by_reg_.end()) {
      continue;
    }
    const OpenProducer &producer = it->second;
    int contribution = producer.weight *
        (instructions_issued_count_ - producer.inst_count - 1);
    CloseOneProducer(use.reg, contribution, undo);
  }
}

void IlpTracker::OpenDefs(const ScheduleNode *node, UndoRecord &undo) {
  const GCNRegisterTracker::NodeRegInfo &info =
      pressure_tracker_->GetNodeRegInfo(node);
  const int weight = weight_by_topo_index_[node->GetTopoIndex()];
  for (const GCNRegisterTracker::RegMask &def : info.defs) {
    // Re-def with no intervening read: prior value is dead, nothing
    // stalled on it, no ILP credit. Read-modify-write was already
    // closed by ProcessUsesAsCloses (which runs first), so by here
    // the use-side has removed the reg from the open map.
    if (open_producer_by_reg_.find(def.reg) !=
        open_producer_by_reg_.end()) {
      CloseOneProducer(def.reg, /*contribution=*/0, undo);
    }
    open_producer_by_reg_.try_emplace(
        def.reg, OpenProducer{instructions_issued_count_, weight});
    undo.opened.push_back(def.reg);
    sum_open_weights_ += weight;
    sum_open_weighted_indices_ += weight * instructions_issued_count_;
  }
}

void IlpTracker::EraseOpens(const UndoRecord &undo) {
  for (unsigned reg : undo.opened) {
    auto it = open_producer_by_reg_.find(reg);
    if (it == open_producer_by_reg_.end()) {
      report_fatal_error(
          "IlpTracker::EraseOpens: opened-record reg " + Twine(reg) +
          " not present in open map. Mutators driven out of order?");
    }
    sum_open_weights_ -= it->second.weight;
    sum_open_weighted_indices_ -= it->second.weight * it->second.inst_count;
    open_producer_by_reg_.erase(it);
  }
}

void IlpTracker::ReinsertCloses(const UndoRecord &undo) {
  for (const UndoRecord::ClosedEntry &c : undo.closed) {
    auto [it, inserted] = open_producer_by_reg_.try_emplace(
        c.reg, OpenProducer{c.prior_inst_count, c.prior_weight});
    if (!inserted) {
      report_fatal_error(
          "IlpTracker::ReinsertCloses: re-inserting closed reg " +
          Twine(c.reg) +
          " but it is already present in the open map. Round-trip "
          "invariant violated.");
    }
    sum_open_weights_ += c.prior_weight;
    sum_open_weighted_indices_ += c.prior_weight * c.prior_inst_count;
    closed_ilp_score_ -= c.contribution;
  }
}

// ============================================================================
// Score query
// ============================================================================

int IlpTracker::GetIlpScore() const {
  // closed + Σ_{R ∈ open}  w_R * (c - i_R - 1)
  //       = closed + (c - 1) * sum_w - sum_(w*i)
  // O(1).
  return closed_ilp_score_ +
         (instructions_issued_count_ - 1) * sum_open_weights_ -
         sum_open_weighted_indices_;
}

// ============================================================================
// Describe
// ============================================================================

std::string IlpTracker::Describe() const {
  std::string out;
  out += "ilp: score=" + std::to_string(GetIlpScore());
  out += " closed=" + std::to_string(closed_ilp_score_);
  out += " issued=" + std::to_string(instructions_issued_count_);
  out += " open_vregs=" + std::to_string(GetOpenProducerVregCount());
  out += " sum_w=" + std::to_string(sum_open_weights_);
  return out;
}
