//===- PipeStalenessTracker.cpp - per-pipe issue-spacing credit -----------===//
//
// Implementation of per-pipe issue-spacing credit tracking.
//
// See PipeStalenessTracker.h for the full scoring system and design
// rationale.
//
//===----------------------------------------------------------------------===//

#include "PipeStalenessTracker.h"

#include <algorithm>
#include <cmath>

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

// Build the credit table for one pipe: index = capped staleness g in
// [0, target_spacing], value = credit in [0, kFixedPointScale].
// Both curves rise monotonically from table[0] = 0 to
// table[target_spacing] = kFixedPointScale — pipes differ in where
// they saturate, never in what a saturated instruction earns (see
// THE SCORE in the header). Doubles appear only here, once per
// region at construction; everything the search touches is int.
std::vector<int> BuildCreditTable(int target_spacing,
                                  PipeStalenessTracker::CreditCurve curve) {
  std::vector<int> table(target_spacing + 1, 0);

  for (int gap = 1; gap <= target_spacing; ++gap) {
    const double fraction_of_target =
        static_cast<double>(gap) / static_cast<double>(target_spacing);

    double shape;
    if (curve == PipeStalenessTracker::CreditCurve::kSqrt) {
      shape = std::sqrt(fraction_of_target);
    } else {
      shape = fraction_of_target;
    }

    table[gap] = static_cast<int>(std::lround(
        static_cast<double>(PipeStalenessTracker::kFixedPointScale) * shape));
  }

  return table;
}

}  // namespace

// ============================================================================
// Construction
// ============================================================================

PipeStalenessTracker::PipeStalenessTracker(const ScheduleGraph &graph,
                                           const Options &options)
    : options_(options),
      pipe_index_by_topo_index_(graph.Size(), kNoPipe) {
  ValidateConstructionPreconditions(graph);

  last_issue_position_by_pipe_.fill(-1);

  const std::array<int, kNumHwPipes> visible_count_by_pipe =
      ClassifyVisibleInstructions(graph);

  DerivePerPipeScoring(visible_count_by_pipe);
}

void PipeStalenessTracker::ValidateConstructionPreconditions(
    const ScheduleGraph &graph) const {
  if (options_.spacing_ceiling < 1 ||
      options_.spacing_ceiling > kMaxSpacingCeiling) {
    report_fatal_error("PipeStalenessTracker: spacing_ceiling must be in [1, " +
                       Twine(kMaxSpacingCeiling) + "]");
  }

  if (!graph.IsTopoSorted()) {
    report_fatal_error(
        "PipeStalenessTracker requires ScheduleGraph::"
        "ComputeTopologicalOrder to have been called before construction "
        "(topo indices key the per-node pipe table).");
  }
}

std::array<int, kNumHwPipes>
PipeStalenessTracker::ClassifyVisibleInstructions(const ScheduleGraph &graph) {
  std::array<int, kNumHwPipes> visible_count_by_pipe;
  visible_count_by_pipe.fill(0);

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

    const int pipe_index = static_cast<int>(ClassifyHwPipe(*mi));
    pipe_index_by_topo_index_[node.GetTopoIndex()] = pipe_index;
    visible_count_by_pipe[pipe_index] += 1;
  }

  return visible_count_by_pipe;
}

void PipeStalenessTracker::DerivePerPipeScoring(
    const std::array<int, kNumHwPipes> &visible_count_by_pipe) {
  int total_visible_count = 0;
  for (int count : visible_count_by_pipe) {
    total_visible_count += count;
  }
  total_visible_count_ = total_visible_count;

  for (int pipe_index = 0; pipe_index < kNumHwPipes; ++pipe_index) {
    const int count = visible_count_by_pipe[pipe_index];

    // Target spacing: the even-spread gap N/n_p, clamped to
    // [1, ceiling]. Pipes absent from the region get the ceiling
    // (they never issue; the value only feeds staleness reporting).
    int target_spacing = options_.spacing_ceiling;
    if (count > 0) {
      target_spacing = std::clamp(total_visible_count / count, 1,
                                  options_.spacing_ceiling);
    }
    desirable_spacing_by_pipe_[pipe_index] = target_spacing;

    credit_table_by_pipe_[pipe_index] =
        BuildCreditTable(target_spacing, options_.curve);

    // Fixed-point overdueness weight (see kFixedPointScale's
    // declaration for why the scale keeps distinct spacings
    // distinct). Divided once here; RankKeyForNode is then a single
    // multiply.
    rank_weight_by_pipe_[pipe_index] = kFixedPointScale / target_spacing;
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

int PipeStalenessTracker::StalenessForPipeIndex(int pipe_index) const {
  if (pipe_index == kNoPipe || !IsCreditedPipeIndex(pipe_index)) {
    return 0;
  }

  const int gap = visible_instructions_issued_count_ -
                  last_issue_position_by_pipe_[pipe_index];
  return std::min(gap, desirable_spacing_by_pipe_[pipe_index]);
}

int PipeStalenessTracker::CreditForPipeIndex(int pipe_index) const {
  if (pipe_index == kNoPipe || !IsCreditedPipeIndex(pipe_index)) {
    return 0;
  }

  return credit_table_by_pipe_[pipe_index]
                              [StalenessForPipeIndex(pipe_index)];
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
  return StalenessForPipeIndex(static_cast<int>(pipe));
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

int64_t PipeStalenessTracker::RankKeyForNode(const ScheduleNode *node) const {
  // Proxy handling mirrors MarginalCreditForNode: best case over
  // the subgraph's initial members.
  if (node->IsSubgraphStartProxy()) {
    SubgraphInfo *info = node->GetSubgraphInfo();

    int64_t max_key = 0;
    for (const ScheduleNode *member : info->initial_members) {
      max_key = std::max(max_key, RankKeyForNode(member));
    }
    return max_key;
  }

  const int pipe_index = PipeIndexForNode(node);
  if (pipe_index == kNoPipe || !IsCreditedPipeIndex(pipe_index)) {
    return 0;
  }

  // RAW staleness (uncapped), so long-starved pipes still order
  // among themselves past saturation — see RANKING vs SCORING in
  // the header for why the cap must not apply here.
  const int64_t raw_staleness = visible_instructions_issued_count_ -
                                last_issue_position_by_pipe_[pipe_index];
  return raw_staleness * rank_weight_by_pipe_[pipe_index];
}

std::array<int, kNumHwPipes> PipeStalenessTracker::GetStalenessSnapshot()
    const {
  std::array<int, kNumHwPipes> snapshot;
  for (int pipe_index = 0; pipe_index < kNumHwPipes; ++pipe_index) {
    snapshot[pipe_index] = StalenessForPipeIndex(pipe_index);
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

  out += " staleness/spacing=[";
  for (int pipe_index = 0; pipe_index < kNumHwPipes; ++pipe_index) {
    if (pipe_index > 0) {
      out += " ";
    }
    out += HwPipeName(static_cast<HwPipe>(pipe_index)).str();
    out += ":" + std::to_string(StalenessForPipeIndex(pipe_index));
    out += "/" + std::to_string(desirable_spacing_by_pipe_[pipe_index]);
  }
  out += "]";

  return out;
}
