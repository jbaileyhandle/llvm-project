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
#include "SubgraphInfo.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>

namespace llvm {
namespace hierarchical_scheduler {

PartitionDag::PartitionDag(const ScheduleGraph *graph,
                           const GCNSubtarget *st,
                           const MachineFunction *mf,
                           ScheduleMetric metric)
    : graph_(graph), st_(st), mf_(mf), metric_(metric) {
  // Only MAX-direction metrics make sense for BFS-DP's
  // max-over-paths of min-along-path bottleneck DP. Min-direction
  // metrics would invert the semantics (we'd want min-over-paths of
  // max-along-path); not implemented.
  if (metric_ != ScheduleMetric::kMaximizeRegisterOccupancy &&
      metric_ !=
          ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore) {
    report_fatal_error(
        "PartitionDag: only kMaximizeRegisterOccupancy and "
        "kMaximizeContinuousRegisterOccupancyScore are currently "
        "supported metrics");
  }
}

bool PartitionDag::Build() {
  if (source_ != nullptr) {
    report_fatal_error(
        "PartitionDag::Build called twice on the same instance; "
        "construct a new PartitionDag instead");
  }

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
    // Score-bound prune killed every path to the all-scheduled
    // partition. No schedule strictly beats the seed. schedule_
    // stays empty; caller falls back to the baseline.
    return false;
  }
  ReconstructSchedule();
  return true;
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

void PartitionDag::ExpandSource(PartitionNode *source_partition,
                                std::vector<PartitionNode *> &next_layer) {
  // VisitSuccessor probes by Schedule/Unschedule directly on
  // source_partition's state (no per-probe clone); the round-trip
  // restores its ready list to identical contents and order, so
  // the ArrayRef stays valid across iterations.
  ArrayRef<const ScheduleNode *> ready =
      source_partition->schedule_state->GetReadyList();
  for (const ScheduleNode *scheduled_node : ready) {
    VisitSuccessor(source_partition, scheduled_node, next_layer);
  }
  // source_partition has been fully expanded; drop the snapshot.
  // Reconstruction walks best_incoming_edge, not schedule_state.
  source_partition->schedule_state.reset();
}

void PartitionDag::VisitSuccessor(
    PartitionNode *source_partition, const ScheduleNode *scheduled_node,
    std::vector<PartitionNode *> &next_layer) {
  // Probe-schedule `scheduled_node` directly on source_partition's
  // state. Between this Schedule and the matching Unschedule below,
  // source_partition->schedule_state represents the successor
  // partition — that's the state we want FindOrInsert to read the
  // PartitionKey from (and to NoHistoryClone on a miss).
  ++schedule_call_count_;
  GCNRegPressure edge_peak =
      source_partition->schedule_state->Schedule(scheduled_node);
  int edge_score = ComputeScoreFromPressure(edge_peak);

  // Running bottleneck along the path through source_partition then
  // via this edge:
  //   - score: min-along-path (the actual bottleneck metric value).
  //     The dag source has score = INT_MAX, so
  //     min(INT_MAX, edge_score) = edge_score on the first edge —
  //     no special case needed.
  //   - register_pressure: element-wise max-along-path; reports the
  //     schedule's actual per-component peak.
  PathBottleneck path_bottleneck;
  path_bottleneck.score =
      std::min(source_partition->best_path_bottleneck.score, edge_score);
  path_bottleneck.register_pressure =
      max(source_partition->best_path_bottleneck.register_pressure,
          edge_peak);

  // Sanity check: score should be reproducible from register_pressure
  // (holds by construction for both metrics — score = min over per-
  // class monotone fns, so score(element-wise max) = min of scores).
  assert(ComputeScoreFromPressure(path_bottleneck.register_pressure) ==
             path_bottleneck.score &&
         "PartitionDag: bottleneck score and register_pressure "
         "disagree");

  // Score-bound prune: any completion past this edge has bottleneck
  // min(path_bottleneck, future) <= path_bottleneck. If this edge's
  // path bottleneck is already <= the caller's known-baseline seed,
  // no completion through here can STRICTLY beat the seed — and
  // matching the seed isn't an improvement, so we'd take the
  // baseline schedule anyway. Don't materialize the successor.
  // Sound: a later (better) path to the same partition will create
  // it on demand via FindOrInsert.
  if (path_bottleneck.score <= initial_best_score_) {
    ++prune_count_;
    source_partition->schedule_state->Unschedule();
    return;
  }

  PartitionNode *succ =
      FindOrInsert(*source_partition->schedule_state, next_layer);

  // DP merge:
  //   - First writer: always wins (succ->best_incoming_edge unset
  //     signals succ's bottleneck is still the default placeholder).
  //   - Strict score improvement: update.
  //   - Score tie: break by input-order index (SUnit::NodeNum) of
  //     the just-scheduled instruction — higher (= later in input
  //     order) wins. Biases reconstruction toward the input
  //     schedule's ordering. Not for register pressure (we already
  //     optimize that explicitly via the metric); it's about ILP /
  //     schedule length, which the input schedule has already been
  //     tuned for and we can inherit on ties for free. Matters most
  //     when many edges tie at the same metric score (e.g., the
  //     integer occupancy metric on schedules that fit in one
  //     bracket).
  //   - Score strictly worse: ignore.
  bool should_update = false;
  if (!succ->best_incoming_edge.has_value()) {
    should_update = true;
  } else if (path_bottleneck.score >
             succ->best_path_bottleneck.score) {
    should_update = true;
  } else if (path_bottleneck.score ==
             succ->best_path_bottleneck.score) {
    should_update =
        GetInputOrderIndex(scheduled_node) >
        GetInputOrderIndex(succ->best_incoming_edge->scheduled_node);
  }
  if (should_update) {
    succ->best_path_bottleneck = path_bottleneck;
    succ->best_incoming_edge =
        PartitionEdge{source_partition, scheduled_node};
  }

  // Restore source_partition's state for the next ready-list
  // iteration.
  source_partition->schedule_state->Unschedule();
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

int PartitionDag::GetInputOrderIndex(const ScheduleNode *node) {
  if (node->IsSchedulingUnit()) {
    const SUnit *su = node->GetSUnit();
    return su != nullptr ? static_cast<int>(su->NodeNum) : 0;
  }
  // Proxy: min over members' input orders. Matches
  // EffectiveNodeNum in SearchPolicies.cpp — proxy represents the
  // earliest-in-input-order member of its subgraph. Returns 0 if
  // the proxy has no members.
  SubgraphInfo *info = node->GetSubgraphInfo();
  int result = INT_MAX;
  for (ScheduleNode *member : info->members) {
    result = std::min(result, GetInputOrderIndex(member));
  }
  return (result == INT_MAX) ? 0 : result;
}

int PartitionDag::ComputeScoreFromPressure(
    const GCNRegPressure &pressure) const {
  switch (metric_) {
    case ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore:
      return GCNRegisterTracker::ComputeContinuousOccupancyScore(
          *st_, pressure.getVGPRNum(st_->hasGFX90AInsts()),
          pressure.getSGPRNum());
    case ScheduleMetric::kMaximizeRegisterOccupancy:
      return static_cast<int>(pressure.getOccupancy(*st_));
    default:
      // Constructor rejected all other values.
      llvm_unreachable(
          "PartitionDag: unsupported metric in ComputeScoreFromPressure "
          "— constructor validation lapse");
  }
}

void PartitionDag::ReconstructSchedule() {
  schedule_.clear();
  // Walk sink → source via best_incoming_edge, collecting the
  // scheduled instructions along the way. Source has nullopt for
  // best_incoming_edge — the loop terminates there.
  for (PartitionNode *cur = sink_; cur->best_incoming_edge.has_value();
       cur = cur->best_incoming_edge->source_partition) {
    schedule_.push_back(cur->best_incoming_edge->scheduled_node);
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
