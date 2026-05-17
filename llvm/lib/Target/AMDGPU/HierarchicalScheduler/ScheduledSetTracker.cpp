//===- ScheduledSetTracker.cpp - Track scheduled-set state ----------------===//
//
// Implementation. See header for the contract.
//
//===----------------------------------------------------------------------===//

#include "ScheduledSetTracker.h"
#include "ScheduleLengthTracker.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>
#include <random>

namespace llvm {
namespace hierarchical_scheduler {

namespace {

// Deterministic seed: signatures reproduce across runs for the same
// graph topology. Value chosen arbitrarily.
constexpr uint32_t kSignatureSeed = 0xCAFEF00DU;

} // namespace

ScheduledSetTracker::ScheduledSetTracker(
    const ScheduleGraph *graph,
    const ScheduleLengthTracker *length_tracker)
    : graph_(graph), length_tracker_(length_tracker) {
  if (graph_ == nullptr) {
    report_fatal_error("ScheduledSetTracker: graph must not be null");
  }
  // length_tracker_ is now nullable. When null, frontier entries'
  // lower_bound stays at 0 (the default) — callers that don't
  // consult lower_bound (e.g., BFS-DP, which uses scheduled_set_
  // + prefix_signature_ for PartitionKey but never reads frontier
  // LBs) can skip building a ScheduleLengthTracker entirely.

  int n = graph_->Size();
  if (n < 2) {
    // Bitset sizes 0 and 1 are reserved as PartitionKey sentinels in
    // DenseMapInfo<PartitionKey> (history trackers use PartitionKey
    // as a DenseMap key). Refuse here so the precondition is checked
    // once, centrally, instead of in every history tracker.
    report_fatal_error(
        "ScheduledSetTracker requires a graph with at least 2 nodes "
        "(scheduled-set bitset sizes 0 and 1 are reserved as "
        "PartitionKey sentinels)");
  }
  per_node_signatures_.resize(n);
  scheduled_set_.resize(n);

  std::mt19937 random_generator(kSignatureSeed);
  for (int i = 0; i < n; ++i) {
    per_node_signatures_[i] = random_generator();
  }
}

void ScheduledSetTracker::Schedule(const ScheduleNode *node) {
  int topo_idx = node->GetTopoIndex();

  // Toggle prefix-state. XOR-update the signature; set the bit.
  // Both apply to proxies too — proxy state is part of the search-
  // state identity for dominance.
  prefix_signature_ ^= per_node_signatures_[topo_idx];
  scheduled_set_.set(topo_idx);

  // Proxies don't participate in frontier mechanics — they're
  // never frontier members and they don't contribute cycle delta
  // to any LB. Early-return so the helpers below can assume
  // `node` is a real instruction.
  if (node->IsSubgraphProxy()) {
    return;
  }

  // Frontier maintenance is gated on having a length tracker:
  // the only readers of the frontier (LengthHistoryTracker
  // dominance, length-tracking shakedowns) all assume real LBs,
  // which require length info. Callers that opt out of length
  // tracking (e.g., BFS-DP) consume only scheduled_set_ +
  // prefix_signature_ for PartitionKey and skip the frontier
  // entirely — the work below would be wasted for them.
  if (length_tracker_ == nullptr) {
    return;
  }

  // Real node: was in the frontier (all its strong predecessors
  // had been scheduled, otherwise we couldn't have scheduled it).
  // Remove.
  frontier_.erase(topo_idx);

  UpdateSuccessorFrontierForSchedule(node);
}

void ScheduledSetTracker::Unschedule(const ScheduleNode *node) {
  int topo_idx = node->GetTopoIndex();

  // Toggle prefix-state first so the recomputes below correctly
  // skip `node` (which is no longer scheduled).
  prefix_signature_ ^= per_node_signatures_[topo_idx];
  scheduled_set_.reset(topo_idx);

  if (node->IsSubgraphProxy()) {
    return;
  }

  // See Schedule's matching comment — frontier maintenance is
  // gated on a length tracker being bound. Callers that opt out
  // (BFS-DP) never read the frontier; the recompute below would
  // be wasted.
  if (length_tracker_ == nullptr) {
    return;
  }

  UpdateSuccessorFrontierForUnschedule(node);
  MaybeAddSelfToFrontier(node);
}

void ScheduledSetTracker::UpdateSuccessorFrontierForSchedule(
    const ScheduleNode *node) {
  int cycle = length_tracker_->GetScheduledCycle(node);

  for (const ScheduleEdge &edge : node->Successors()) {
    // IsLatencyEdge filters out kSubgraphOrderEdge, which is the
    // only edge kind that involves proxies in the current encoding
    // (InsertSubgraphProxies emits only kSubgraphOrderEdge for
    // proxy-related edges). So the surviving successor is a real
    // instruction — no separate proxy check needed. (See the
    // proxy → proxy assert in ScheduleGraph::AddEdge.)
    if (!edge.IsLatencyEdge()) {
      continue;
    }
    int contribution =
        cycle + std::max(edge.Latency(), node->IssueSlotsConsumed());

    // Find or insert. operator[] default-constructs the entry
    // (lower_bound=0); then max-merge node's contribution.
    auto &entry = frontier_[edge.node_->GetTopoIndex()];
    entry.lower_bound = std::max(entry.lower_bound, contribution);
  }
}

void ScheduledSetTracker::UpdateSuccessorFrontierForUnschedule(
    const ScheduleNode *node) {
  for (const ScheduleEdge &edge : node->Successors()) {
    if (!edge.IsLatencyEdge()) {
      continue;
    }
    // Recompute the successor's entry from currently-scheduled
    // latency-bearing predecessors (`node`'s bit was reset at the
    // top of Unschedule, so it's correctly excluded). Erase if no
    // scheduled predecessors remain; otherwise update LB.
    auto [count, lower_bound] =
        ComputeCountAndLowerBoundFromPredecessors(edge.node_);
    int successor_topo = edge.node_->GetTopoIndex();
    if (count == 0) {
      frontier_.erase(successor_topo);
    } else {
      frontier_[successor_topo].lower_bound = lower_bound;
    }
  }
}

void ScheduledSetTracker::MaybeAddSelfToFrontier(
    const ScheduleNode *node) {
  auto [count, lower_bound] =
      ComputeCountAndLowerBoundFromPredecessors(node);
  if (count > 0) {
    frontier_[node->GetTopoIndex()].lower_bound = lower_bound;
  }
}

std::pair<int, int>
ScheduledSetTracker::ComputeCountAndLowerBoundFromPredecessors(
    const ScheduleNode *node) const {
  int count = 0;
  int lower_bound = 0;
  for (const ScheduleEdge &edge : node->Predecessors()) {
    if (!edge.IsLatencyEdge()) {
      continue;
    }
    const ScheduleNode *predecessor = edge.node_;
    if (!scheduled_set_.test(predecessor->GetTopoIndex())) {
      continue;
    }
    ++count;
    int predecessor_cycle =
        length_tracker_->GetScheduledCycle(predecessor);
    int contribution = predecessor_cycle +
                       std::max(edge.Latency(),
                                predecessor->IssueSlotsConsumed());
    lower_bound = std::max(lower_bound, contribution);
  }
  return {count, lower_bound};
}

} // namespace hierarchical_scheduler
} // namespace llvm
