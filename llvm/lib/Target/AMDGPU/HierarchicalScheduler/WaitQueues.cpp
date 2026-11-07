//===- WaitQueues.cpp - The vmcnt and lgkmcnt wait queues -----------------===//
//
// See WaitQueues.h and llvm/docs/AMDGPUMemoryGraphDesign.md, section 5.3.
//
//===----------------------------------------------------------------------===//

#include "WaitQueues.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>
#include <iterator>
#include <utility>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

WaitEvent llvm::hierarchical_scheduler::LgkmcntEvent(MemType type) {
  switch (type) {
  case MemType::kLds:
  case MemType::kFlat:
    return WaitEvent::kLdsAccess;
  case MemType::kGds:
    return WaitEvent::kGdsAccess;
  case MemType::kScalar:
    return WaitEvent::kSmemAccess;
  case MemType::kVector:
    break;
  }
  report_fatal_error("LgkmcntEvent: a vector op doesn't go in lgkmcnt");
}

bool llvm::hierarchical_scheduler::IsFlatInBothQueues(const MemoryNode &node) {
  const MemOpInfo &info = *node.GetMemOpInfo();
  return info.type == MemType::kFlat && info.counts_in_vmcnt &&
         info.counts_in_lgkmcnt;
}

namespace {

// Classifies an op's WaitEvent in a queue of `type`. Fails hard if the op
// doesn't go in that queue.
WaitEvent ClassifyWaitEvent(WaitQueueType type, const MemOpInfo &info) {
  if (type == WaitQueueType::kVmcnt) {
    if (!info.counts_in_vmcnt) {
      report_fatal_error("WaitQueue::Push: the op doesn't go in vmcnt");
    }
    return WaitEvent::kVmemAccess;
  }

  if (!info.counts_in_lgkmcnt) {
    report_fatal_error("WaitQueue::Push: the op doesn't go in lgkmcnt");
  }
  return LgkmcntEvent(info.type);
}

} // namespace

void WaitQueue::Push(const MemoryNode &node, int raw_op_complete_time) {
  // What the entry records about the op in this queue.
  const WaitEvent event = ClassifyWaitEvent(type_, *node.GetMemOpInfo());
  const bool counts_in_tracked_peak = tracked_read_types_.Tracks(node);

  // The op can't be popped before the entry already at the back.
  int earliest_possible_pop_time = raw_op_complete_time;
  if (!entries_.empty()) {
    earliest_possible_pop_time =
        std::max(earliest_possible_pop_time,
                 entries_.back().earliest_possible_pop_time);
  }

  entries_.push_back(QueueEntry{&node, event, earliest_possible_pop_time,
                                counts_in_tracked_peak});

  // Both peaks can only rise on a push.
  peak_size_ = std::max(peak_size_, static_cast<int>(entries_.size()));
  RaiseTrackedSize(entries_.back());
}

std::vector<QueueEntry> WaitQueue::Wait(const MemoryNode &node) {
  auto waited_entry = llvm::find_if(entries_, [&](const QueueEntry &entry) {
    return entry.node == &node;
  });
  if (waited_entry == entries_.end()) {
    report_fatal_error("WaitQueue::Wait: the op isn't in this queue");
  }

  auto after_waited_entry = std::next(waited_entry);
  std::vector<QueueEntry> popped(entries_.begin(), after_waited_entry);
  entries_.erase(entries_.begin(), after_waited_entry);

  LowerTrackedSize(popped);
  return popped;
}

std::vector<QueueEntry> WaitQueue::Drain() {
  std::vector<QueueEntry> popped = std::move(entries_);
  entries_.clear();

  LowerTrackedSize(popped);
  return popped;
}

void WaitQueue::RaiseTrackedSize(const QueueEntry &pushed) {
  if (!pushed.counts_in_tracked_peak) {
    return;
  }
  ++tracked_size_;
  peak_tracked_size_ = std::max(peak_tracked_size_, tracked_size_);
}

void WaitQueue::LowerTrackedSize(ArrayRef<QueueEntry> popped) {
  for (const QueueEntry &entry : popped) {
    if (entry.counts_in_tracked_peak) {
      --tracked_size_;
    }
  }
}

bool WaitQueue::Holds(const MemoryNode &node) const {
  return llvm::any_of(entries_, [&](const QueueEntry &entry) {
    return entry.node == &node;
  });
}

bool WaitQueue::HoldsOutOfOrderKinds() const {
  for (const QueueEntry &entry : entries_) {
    if (entry.event == WaitEvent::kSmemAccess) {
      return true;
    }
    if (entry.event != entries_.front().event) {
      return true;
    }
  }
  return false;
}

bool WaitQueue::HoldsFlatInBothQueues() const {
  return llvm::any_of(entries_, [](const QueueEntry &entry) {
    return IsFlatInBothQueues(*entry.node);
  });
}

void WaitQueues::Issue(const MemoryNode &node, int raw_op_complete_time) {
  const MemOpInfo &info = *node.GetMemOpInfo();
  if (info.counts_in_vmcnt) {
    vmcnt_.Push(node, raw_op_complete_time);
  }
  if (info.counts_in_lgkmcnt) {
    lgkmcnt_.Push(node, raw_op_complete_time);
  }
}

NodeWait WaitQueues::WaitFor(ArrayRef<const MemoryNode *> loads_used,
                             int issue_wait_begin_time) {
  NodeWait wait;
  wait.issue_time = issue_wait_begin_time;

  // Decided once, before any of this node's waits pop anything.
  const bool cautious = CautiousMode();

  for (const MemoryNode *load : loads_used) {
    for (WaitQueue *queue : {&vmcnt_, &lgkmcnt_}) {
      if (!queue->Holds(*load)) {
        continue;
      }
      if (ShouldDrain(*queue, cautious)) {
        AddPopped(queue->Drain(), wait);
      } else {
        AddPopped(queue->Wait(*load), wait);
      }
    }
  }
  return wait;
}

NodeWait WaitQueues::DrainAll(int issue_wait_begin_time) {
  NodeWait wait;
  wait.issue_time = issue_wait_begin_time;
  AddPopped(vmcnt_.Drain(), wait);
  AddPopped(lgkmcnt_.Drain(), wait);
  return wait;
}

bool WaitQueues::CautiousMode() const {
  return vmcnt_.HoldsFlatInBothQueues() || lgkmcnt_.HoldsFlatInBothQueues();
}

bool WaitQueues::ShouldDrain(const WaitQueue &queue, bool cautious) {
  return cautious || queue.HoldsOutOfOrderKinds();
}

void WaitQueues::AddPopped(const std::vector<QueueEntry> &popped,
                           NodeWait &wait) {
  for (const QueueEntry &entry : popped) {
    wait.issue_time =
        std::max(wait.issue_time, entry.earliest_possible_pop_time);
    wait.popped.push_back(entry.node);
  }
}
