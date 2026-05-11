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
#include "SubgraphInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>
#include <climits>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Per-op desirable spacing — the saturation cap on each producer's
// contribution. Numbers are ballpark approximations of how many
// real instructions of cover an op of each class typically benefits
// from before further hiding stops mattering. The AMDGPU latency
// model is too crude to justify finer granularity.
//
// VMEM / FLAT (memory loads): hundreds of cycles latency — significant
//   cover meaningful, capped at 32 to bound the table-driven score.
// SMEM (scalar memory load): ~tens of cycles via the scalar cache.
// DS (LDS): ~tens of cycles, faster than global.
// Default (VALU / SALU): single-digit cycles, minimal cover needed.
//
// Easy to tune as a unit later. The relative ordering is what
// drives ranking decisions; absolute magnitudes set the trade
// against other scoring axes (length, occupancy).
int IlpDesirableSpacingForOp(const MachineInstr *mi) {
  if (SIInstrInfo::isVMEM(*mi) || SIInstrInfo::isFLAT(*mi)) {
    return 32;
  }
  if (SIInstrInfo::isSMRD(*mi)) {
    return 16;
  }
  if (SIInstrInfo::isDS(*mi)) {
    return 8;
  }
  return 2;
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
      desirable_spacing_by_topo_index_(graph.Size(), 0) {
  // Precompute per-node desirable_spacing. Only real MachineInstr-
  // backed nodes get a non-zero value; everything else stays at 0
  // (the initialized value).
  for (const ScheduleNode &node : graph.Nodes()) {
    if (!IsRealInstruction(&node)) {
      continue;
    }
    desirable_spacing_by_topo_index_[node.GetTopoIndex()] =
        IlpDesirableSpacingForOp(node.GetSUnit()->getInstr());
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
  undo.closed.push_back({reg, producer.inst_count,
                         producer.desirable_spacing, contribution});
  sum_open_inst_indices_ -= producer.inst_count;
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
    int spacing = instructions_issued_count_ - producer.inst_count - 1;
    int contribution = std::min(spacing, producer.desirable_spacing);
    CloseOneProducer(use.reg, contribution, undo);
  }
}

void IlpTracker::OpenDefs(const ScheduleNode *node, UndoRecord &undo) {
  const GCNRegisterTracker::NodeRegInfo &info =
      pressure_tracker_->GetNodeRegInfo(node);
  const int desirable_spacing =
      desirable_spacing_by_topo_index_[node->GetTopoIndex()];
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
        def.reg,
        OpenProducer{instructions_issued_count_, desirable_spacing});
    undo.opened.push_back(def.reg);
    sum_open_inst_indices_ += instructions_issued_count_;
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
    sum_open_inst_indices_ -= it->second.inst_count;
    open_producer_by_reg_.erase(it);
  }
}

void IlpTracker::ReinsertCloses(const UndoRecord &undo) {
  for (const UndoRecord::ClosedEntry &c : undo.closed) {
    auto [it, inserted] = open_producer_by_reg_.try_emplace(
        c.reg,
        OpenProducer{c.prior_inst_count, c.prior_desirable_spacing});
    if (!inserted) {
      report_fatal_error(
          "IlpTracker::ReinsertCloses: re-inserting closed reg " +
          Twine(c.reg) +
          " but it is already present in the open map. Round-trip "
          "invariant violated.");
    }
    sum_open_inst_indices_ += c.prior_inst_count;
    closed_ilp_score_ -= c.contribution;
  }
}

// ============================================================================
// Score query
// ============================================================================

int IlpTracker::GetProvisionalIlpScore() const {
  // pending = Σ_{R in open} (c - i_R - 1)  [unsaturated]
  //         = open_count * (c - 1) - sum_open_inst_indices_
  // O(1).
  const int open_count =
      static_cast<int>(open_producer_by_reg_.size());
  return closed_ilp_score_ +
         open_count * (instructions_issued_count_ - 1) -
         sum_open_inst_indices_;
}

// ============================================================================
// Open-producer snapshot
// ============================================================================

SmallVector<IlpTracker::OpenProducerInstCount, 16>
IlpTracker::GetOpenProducerInstCountsSnapshot() const {
  SmallVector<OpenProducerInstCount, 16> result;
  result.reserve(open_producer_by_reg_.size());
  for (const auto &reg_to_producer : open_producer_by_reg_) {
    result.push_back(
        {reg_to_producer.first, reg_to_producer.second.inst_count});
  }
  // Sort by reg ascending for stable parallel-walk comparison
  // across two snapshots from the same partition. Same-partition
  // open sets are identical, so the sorted-by-reg order aligns
  // entries one-to-one between any two snapshots.
  std::sort(result.begin(), result.end(),
            [](const OpenProducerInstCount &a,
               const OpenProducerInstCount &b) {
              return a.reg < b.reg;
            });
  return result;
}

// ============================================================================
// Heuristic helper
// ============================================================================

int IlpTracker::CloseCostForNode(const ScheduleNode *node) const {
  // Subgraph start proxy: scheduling the proxy commits to scheduling
  // its members next (subgraph members are contiguous). Score the
  // proxy as the cheapest-available opening move in the subgraph —
  // min over initial_members of CloseCostForNode(member). Optimistic:
  // assumes the search will pick the best first move once inside.
  // Returns 0 for an empty initial-member list (degenerate; shouldn't
  // happen for a well-formed subgraph but defensive).
  //
  // Recursive on members so a member that is itself a start proxy
  // (nested subgraphs, currently unsupported but future-proofed) is
  // handled the same way. End proxies don't reach this path because
  // FilterAndSortReadyList short-circuits on them (sole-entry
  // invariant); if one ever did, the !IsRealInstruction branch
  // returns 0 conservatively.
  if (node->IsSubgraphStartProxy()) {
    SubgraphInfo *info = node->GetSubgraphInfo();
    int min_cost = INT_MAX;
    for (const ScheduleNode *member : info->initial_members) {
      min_cost = std::min(min_cost, CloseCostForNode(member));
    }
    return (min_cost == INT_MAX) ? 0 : min_cost;
  }
  if (!IsRealInstruction(node)) {
    // End proxies, entry/exit sentinels: no real-instruction
    // semantics — no ILP cost for ranking purposes.
    return 0;
  }

  // Real instruction 
  const GCNRegisterTracker::NodeRegInfo &info =
      pressure_tracker_->GetNodeRegInfo(node);
  int cost = 0;
  for (const GCNRegisterTracker::RegMask &use : info.uses) {
    int desirable_spacing = GetDesirableSpacingForOpenProducer(use.reg);
    if (desirable_spacing == 0) {
      // Not an open producer — closing is a no-op, no ILP cost.
      continue;
    }
    int spacing = GetSpacingForOpenProducer(use.reg);
    int freshness_remaining = std::max(0, desirable_spacing - spacing);
    cost += freshness_remaining;
  }
  return cost;
}

// ============================================================================
// Describe
// ============================================================================

std::string IlpTracker::Describe() const {
  std::string out;
  out += "ilp: score=" + std::to_string(GetIlpScore());
  out += " provisional=" + std::to_string(GetProvisionalIlpScore());
  out += " issued=" + std::to_string(instructions_issued_count_);
  out += " open_vregs=" + std::to_string(GetOpenProducerVregCount());
  return out;
}
