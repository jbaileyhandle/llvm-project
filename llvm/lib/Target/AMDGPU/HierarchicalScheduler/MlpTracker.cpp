//===- MlpTracker.cpp - Memory-level parallelism tracking -----------------===//
//
// See MlpTracker.h and llvm/docs/AMDGPUMemoryGraphDesign.md, section 5.
//
//===----------------------------------------------------------------------===//

#include "MlpTracker.h"
#include "llvm/Support/ErrorHandling.h"
#include <utility>

namespace llvm::hierarchical_scheduler {

MlpTracker::MlpTracker(const MemoryGraph *graph, LatencySetting setting,
                       const TrackedReadTypes &tracked_read_types)
    : graph_(graph), setting_(setting),
      tracked_read_types_(tracked_read_types), queues_(tracked_read_types) {}

void MlpTracker::Schedule(const MemoryNode *node) {
  // The node's waits begin where the clock is now.
  const int issue_wait_begin_time = GetCurrentTime();

  // Cautious mode is read before the waits pop anything, as WaitFor itself
  // does, so the step records the mode its waits were decided under.
  const bool cautious = queues_.CautiousMode();

  // A barrier waits for every op in flight. Any other node waits for the
  // loads whose data it reads, if any of them are still in flight.
  NodeWait wait;
  if (node->GetKind() == MemoryNodeKind::kBarrier) {
    wait = queues_.DrainAll(issue_wait_begin_time);
  } else {
    wait = queues_.WaitFor(node->GetLoadsUsed(), issue_wait_begin_time);
  }

  // A load or write starts when the node issues, and its data is back after
  // its latency.
  if (node->GetKind() == MemoryNodeKind::kMemOp) {
    queues_.Issue(*node, wait.issue_time + node->GetLatency(setting_));
  }

  steps_.push_back(WalkStep{node, issue_wait_begin_time, cautious,
                            wait.issue_time, std::move(wait.popped)});
}

OrderScore MlpTracker::GetScore() const {
  if (steps_.empty() || steps_.back().node != graph_->GetSink()) {
    report_fatal_error("MlpTracker::GetScore: the sink must be scheduled, "
                       "and last");
  }

  // Where each op's use-based flight stops, indexed by topo index: when the
  // first node whose waits cover the op issues. That isn't when the hardware
  // pops the op. If C waits for A (back at 100) and B (back at 200), C issues
  // at 200, and A's flight stops at 200, not 100. A FLAT in both queues is
  // covered once per queue, maybe by different nodes, and stops at the later
  // one. Steps come in order of time, so a later node's time writes over an
  // earlier one's.
  std::vector<int> use_based_stop_time_by_topo_index(
      graph_->GetTopoOrder().size(), 0);
  for (const WalkStep &step : steps_) {
    for (const MemoryNode *popped : step.popped) {
      use_based_stop_time_by_topo_index[popped->GetTopoIndex()] =
          step.issue_time;
    }
  }

  // Each tracked read's flight, measured both ways. Every read gets a
  // use-based stop time, because the sink waits for every read.
  std::vector<MemOpFlight> use_based_flights;
  std::vector<MemOpFlight> latency_based_flights;
  for (const WalkStep &step : steps_) {
    const MemoryNode &node = *step.node;
    if (!tracked_read_types_.Tracks(node)) {
      continue;
    }
    use_based_flights.push_back(MemOpFlight{
        step.issue_time,
        use_based_stop_time_by_topo_index[node.GetTopoIndex()]});
    latency_based_flights.push_back(MemOpFlight{
        step.issue_time, step.issue_time + node.GetLatency(setting_)});
  }

  OrderScore score;
  score.use_based_mlp = MeasureMlp(std::move(use_based_flights));
  score.latency_based_mlp = MeasureMlp(std::move(latency_based_flights));
  score.elapsed_time = GetCurrentTime();

  const WaitQueue &vmcnt = queues_.GetVmcntQueue();
  const WaitQueue &lgkmcnt = queues_.GetLgkmcntQueue();
  score.peak_vmcnt_entries = vmcnt.GetPeakSize();
  score.peak_lgkmcnt_entries = lgkmcnt.GetPeakSize();
  score.peak_vmcnt_tracked_entries = vmcnt.GetPeakTrackedSize();
  score.peak_lgkmcnt_tracked_entries = lgkmcnt.GetPeakTrackedSize();
  return score;
}

} // namespace llvm::hierarchical_scheduler
