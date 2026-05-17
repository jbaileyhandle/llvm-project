//===- PartitionDag.cpp - BFS-DP partition graph implementation ----------===//
//
// Implementation. See PartitionDag.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "PartitionDag.h"
#include "GCNRegisterTracker.h"
#include "GCNSubtarget.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>

namespace llvm {
namespace hierarchical_scheduler {

PartitionDag::PartitionDag(const ScheduleGraph *graph,
                           const GCNSubtarget *st,
                           const MachineFunction *mf)
    : graph_(graph), st_(st), mf_(mf) {}

void PartitionDag::Build() {
  // Two rolling layers — current = the level we're expanding now,
  // next = nodes discovered during this expansion (the next level).
  // Swap-and-clear at the end of each level. No global level-index
  // structure: a PartitionNode's own membership in either layer
  // (transient) plus partition_node_by_key_ (persistent) is enough.
  std::vector<PartitionNode *> current_layer{CreateSourceNode()};
  std::vector<PartitionNode *> next_layer;
  while (!current_layer.empty()) {
    for (PartitionNode *src : current_layer) {
      ExpandSource(src, next_layer);
    }
    current_layer.swap(next_layer);
    next_layer.clear();
    ++current_level_;
  }

  if (sink_ == nullptr) {
    report_fatal_error(
        "PartitionDag::Build: BFS completed without finding the "
        "all-scheduled sink");
  }
  ReconstructSchedule();
}

PartitionNode *PartitionDag::CreateSourceNode() {
  // Source uses a fresh BfsDp-preset ScheduleConstructor (length +
  // ILP trackers off, schedule_order_ maintained so Schedule /
  // Unschedule probe cycles round-trip cleanly). Inserted into
  // partition_node_by_key_ directly rather than via FindOrInsert
  // because the source isn't on any layer's queue — it goes
  // straight into current_layer.
  nodes_.push_back(std::make_unique<PartitionNode>());
  source_ = nodes_.back().get();
  source_->schedule_state.emplace(*graph_, *st_, *mf_,
                                  ScheduleConstructorOptions::BfsDp());
  // Test-mode propagation: if EnableTestModeForTest was called on
  // this dag, apply the deltas to the source's tracker now (before
  // the first Schedule). GCNRegisterTracker::NoHistoryClone
  // preserves test_mode_ / test_vgpr_deltas_, so clones inherit them.
  if (!test_vgpr_deltas_.empty()) {
    source_->schedule_state->GetPressureTrackerForTest()
        .EnableTestModeForTest(test_vgpr_deltas_);
  }
  partition_node_by_key_[source_->schedule_state->GetScheduledSetTracker()
                             .GetPartitionKey()] = source_;
  return source_;
}

void PartitionDag::ExpandSource(PartitionNode *src,
                                std::vector<PartitionNode *> &next_layer) {
  // VisitSuccessor probes by Schedule/Unschedule directly on src's
  // state (no per-probe clone); the round-trip restores src's ready
  // list to identical contents and order, so the ArrayRef stays
  // valid across iterations.
  ArrayRef<const ScheduleNode *> ready =
      src->schedule_state->GetReadyList();
  for (const ScheduleNode *next : ready) {
    VisitSuccessor(src, next, next_layer);
  }
  // src has been fully expanded; drop the snapshot. Reconstruction
  // walks best_incoming_edge, not schedule_state.
  src->schedule_state.reset();
}

void PartitionDag::VisitSuccessor(
    PartitionNode *src, const ScheduleNode *next,
    std::vector<PartitionNode *> &next_layer) {
  // Probe-schedule `next` directly on src's state. Between this
  // Schedule and the matching Unschedule below, src->schedule_state
  // represents the successor partition — that's the state we want
  // FindOrInsert to read the PartitionKey from (and to NoHistoryClone
  // on a miss).
  ++schedule_call_count_;
  GCNRegPressure edge_peak = src->schedule_state->Schedule(next);

  // Convert this edge's peak pressure to a continuous occupancy
  // score (higher = better). The lookup tables inside the tracker
  // already key on (vgpr, sgpr); the static helper just reads them.
  int edge_score = GCNRegisterTracker::ComputeContinuousOccupancyScore(
      *st_, edge_peak.getVGPRNum(st_->hasGFX90AInsts()),
      edge_peak.getSGPRNum());

  // Running bottleneck along the path through src then via this
  // edge: whichever of {src's prior bottleneck, this edge} has the
  // lower score wins (the new bottleneck), and we carry its
  // register_pressure alongside. src=source has score = INT_MAX
  // (default), so any real edge wins — no special case needed.
  PathBottleneck path_bottleneck;
  if (src->best_path_bottleneck.continuous_occupancy_score < edge_score) {
    path_bottleneck = src->best_path_bottleneck;
  } else {
    path_bottleneck = {edge_score, edge_peak};
  }

  // Score-bound prune: any completion past this edge has bottleneck
  // min(path_bottleneck, future) <= path_bottleneck. If this edge
  // already drops the bottleneck strictly below the caller's
  // achievable-baseline seed, no completion through here can beat
  // the seed — don't even materialize the successor partition.
  // Sound: a later (better) path to the same partition will create
  // it on demand via FindOrInsert.
  if (path_bottleneck.continuous_occupancy_score < initial_best_score_) {
    ++prune_count_;
    src->schedule_state->Unschedule();
    return;
  }

  PartitionNode *succ = FindOrInsert(*src->schedule_state, next_layer);

  // First writer always wins (succ->best_incoming_edge unset signals
  // succ's score is still the meaningless INT_MAX default). Subsequent
  // writers update only on strict improvement; ties leave the prior
  // path alone (first-writer-wins).
  if (!succ->best_incoming_edge.has_value() ||
      path_bottleneck.continuous_occupancy_score >
          succ->best_path_bottleneck.continuous_occupancy_score) {
    succ->best_path_bottleneck = path_bottleneck;
    succ->best_incoming_edge = PartitionEdge{src, next};
  }

  // Restore src's state for the next ready-list iteration.
  src->schedule_state->Unschedule();
}

PartitionNode *PartitionDag::FindOrInsert(
    const ScheduleConstructor &probe_state,
    std::vector<PartitionNode *> &next_layer) {
  PartitionKey key =
      probe_state.GetScheduledSetTracker().GetPartitionKey();
  auto it = partition_node_by_key_.find(key);
  if (it != partition_node_by_key_.end()) {
    return it->second;
  }

  // Miss: NoHistoryClone probe_state into a new PartitionNode. The
  // clone is the only ScheduleConstructor copy we pay for — hits
  // skip it entirely. probe_state itself isn't mutated; the caller
  // will Unschedule its own copy after we return.
  nodes_.push_back(std::make_unique<PartitionNode>());
  PartitionNode *new_node = nodes_.back().get();
  new_node->schedule_state = probe_state.NoHistoryClone();
  partition_node_by_key_[key] = new_node;
  next_layer.push_back(new_node);

  // Sink: the unique all-scheduled partition. Only one PartitionNode
  // can have an all-ones scheduled-set (all complete schedules
  // produce the same set), so this assignment fires at most once.
  if (key.scheduled_set.all()) {
    sink_ = new_node;
  }
  return new_node;
}

void PartitionDag::ReconstructSchedule() {
  schedule_.clear();
  // Walk sink → source via best_incoming_edge, collecting the
  // scheduled instructions along the way. Source has nullopt for
  // best_incoming_edge — the loop terminates there.
  for (PartitionNode *cur = sink_; cur->best_incoming_edge.has_value();
       cur = cur->best_incoming_edge->source) {
    schedule_.push_back(cur->best_incoming_edge->scheduled);
  }
  std::reverse(schedule_.begin(), schedule_.end());

  // The reconstructed schedule must visit every node in the graph
  // exactly once (one back-pointer hop per scheduled instruction
  // from source to sink). A mismatch here means the dag is corrupt
  // — most likely a broken best_incoming_edge chain.
  if (static_cast<int>(schedule_.size()) != graph_->Size()) {
    report_fatal_error(
        "PartitionDag::ReconstructSchedule: schedule has " +
        Twine(schedule_.size()) + " nodes but graph has " +
        Twine(graph_->Size()));
  }
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
