//===- PipeStalenessTracker.cpp - per-pipe issue-spacing credit -----------===//
//
// Implementation of per-pipe issue-spacing credit tracking.
//
// See PipeStalenessTracker.h for the measure and design rationale.
//
//===----------------------------------------------------------------------===//

#include "PipeStalenessTracker.h"

#include <algorithm>

#include "SubgraphInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Table value for nodes that do not participate in pipe tracking.
// Proxies, sentinels, and invisible instructions all keep this
// default (the constructor only overwrites visible instructions),
// and every entry point (Schedule, Unschedule, the queries) no-ops
// or returns 0 on it.
constexpr int kNoPipe = -1;

}  // namespace

// ============================================================================
// Construction
// ============================================================================

PipeStalenessTracker::PipeStalenessTracker(const ScheduleGraph &graph,
                                           const Options &options)
    : options_(options),
      pipe_index_by_topo_index_(graph.Size(), kNoPipe) {
  if (options_.staleness_cap < 1) {
    report_fatal_error("PipeStalenessTracker: staleness_cap must be >= 1");
  }
  if (!graph.IsTopoSorted()) {
    report_fatal_error(
        "PipeStalenessTracker requires ScheduleGraph::"
        "ComputeTopologicalOrder to have been called before construction "
        "(topo indices key the per-node pipe table).");
  }
  last_issue_position_by_pipe_.fill(-1);
  for (const ScheduleNode &node : graph.Nodes()) {
    if (!node.IsRealInstruction()) {
      continue;
    }
    // IsRealInstruction => GetSUnit()->getInstr() is non-null:
    // nodes with a non-null SUnit are created only in
    // CreateLeafNodesFromSUnits, which skips boundary SUnits — the
    // only instruction-less kind.
    const MachineInstr *mi = node.GetSUnit()->getInstr();
    if (!IsPipeTrackingVisible(*mi)) {
      continue;
    }
    pipe_index_by_topo_index_[node.GetTopoIndex()] =
        static_cast<int>(ClassifyHwPipe(*mi));
  }
}

// ============================================================================
// Participation helpers
// ============================================================================

bool PipeStalenessTracker::IsCreditedPipeIndex(int pipe_index) const {
  if (pipe_index == static_cast<int>(HwPipe::kOther)) {
    return options_.track_other_pipe;
  }
  return true;
}

int PipeStalenessTracker::PipeIndexForNode(const ScheduleNode *node) const {
  // Every graph node — proxies and sentinels included — carries a
  // valid topo index once ComputeTopologicalOrder has run (checked
  // at construction). Non-participating nodes hold the kNoPipe
  // default.
  return pipe_index_by_topo_index_[node->GetTopoIndex()];
}

int PipeStalenessTracker::CreditForPipeIndex(int pipe_index) const {
  if (pipe_index == kNoPipe || !IsCreditedPipeIndex(pipe_index)) {
    return 0;
  }
  const int gap = visible_instructions_issued_count_ -
                  last_issue_position_by_pipe_[pipe_index];
  return std::min(gap, options_.staleness_cap);
}

// ============================================================================
// Schedule / Unschedule
// ============================================================================

void PipeStalenessTracker::Schedule(const ScheduleNode *node) {
  const int pipe_index = PipeIndexForNode(node);
  if (pipe_index == kNoPipe) {
    return;
  }

  UndoRecord undo;
  undo.pipe_index = pipe_index;
  undo.prior_last_issue_position = last_issue_position_by_pipe_[pipe_index];
  // 0 for an uncredited kOther instruction: it consumes a stream
  // position (the bump below) but changes no credit and no
  // last-issue entry, and its undo record records exactly that
  // nothing-change (credit 0, last-issue value already in the
  // table), keeping Unschedule branch-free.
  undo.credit = CreditForPipeIndex(pipe_index);

  if (IsCreditedPipeIndex(pipe_index)) {
    intermix_credit_ += undo.credit;
    last_issue_position_by_pipe_[pipe_index] =
        visible_instructions_issued_count_;
  }

  undo_stack_.push_back(undo);
  visible_instructions_issued_count_ += 1;
}

void PipeStalenessTracker::Unschedule(const ScheduleNode *node) {
  const int pipe_index = PipeIndexForNode(node);
  if (pipe_index == kNoPipe) {
    return;
  }
  if (undo_stack_.empty()) {
    report_fatal_error(
        "PipeStalenessTracker::Unschedule called with empty undo stack");
  }

  const UndoRecord undo = undo_stack_.back();
  undo_stack_.pop_back();
  if (undo.pipe_index != pipe_index) {
    report_fatal_error(
        "PipeStalenessTracker::Unschedule: node's pipe does not match the "
        "back-of-stack undo record. Mutators driven out of order?");
  }

  visible_instructions_issued_count_ -= 1;
  intermix_credit_ -= undo.credit;
  last_issue_position_by_pipe_[pipe_index] = undo.prior_last_issue_position;
}

// ============================================================================
// Queries
// ============================================================================

int PipeStalenessTracker::GetStaleness(HwPipe pipe) const {
  return CreditForPipeIndex(static_cast<int>(pipe));
}

int PipeStalenessTracker::MarginalCreditForNode(
    const ScheduleNode *node) const {
  // Start proxy: best case over the subgraph's initial members —
  // see the header comment. Recursive so a member that is itself a
  // start proxy (nested subgraphs, currently unsupported but
  // future-proofed) is handled the same way.
  if (node->IsSubgraphStartProxy()) {
    SubgraphInfo *info = node->GetSubgraphInfo();
    int max_credit = 0;
    for (const ScheduleNode *member : info->initial_members) {
      max_credit = std::max(max_credit, MarginalCreditForNode(member));
    }
    return max_credit;
  }
  return CreditForPipeIndex(PipeIndexForNode(node));
}

std::array<int, kNumHwPipes> PipeStalenessTracker::GetStalenessSnapshot()
    const {
  std::array<int, kNumHwPipes> snapshot;
  for (int pipe_index = 0; pipe_index < kNumHwPipes; ++pipe_index) {
    snapshot[pipe_index] = CreditForPipeIndex(pipe_index);
  }
  return snapshot;
}

// ============================================================================
// Describe
// ============================================================================

std::string PipeStalenessTracker::Describe() const {
  std::string out;
  out += "pipe_mix: credit=" + std::to_string(intermix_credit_);
  out += " visible_issued=" +
         std::to_string(visible_instructions_issued_count_);
  out += " staleness=[";
  for (int pipe_index = 0; pipe_index < kNumHwPipes; ++pipe_index) {
    if (pipe_index > 0) {
      out += " ";
    }
    out += HwPipeName(static_cast<HwPipe>(pipe_index)).str();
    out += ":" + std::to_string(CreditForPipeIndex(pipe_index));
  }
  out += "]";
  return out;
}
