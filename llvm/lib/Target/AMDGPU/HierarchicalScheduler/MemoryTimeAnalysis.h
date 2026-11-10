//===- MemoryTimeAnalysis.h - Memory time of a final schedule ---*- C++ -*-===//
//
// Estimates how long each region's final schedule spends waiting for memory,
// next to the best any order of the region could do, and combines a function's
// regions into one score. Analysis only: it never changes the schedule.
//
// Memory time is elapsed time under the MemoryGraph model: how long a region
// takes when waiting for memory is the only thing that costs time, and issuing
// an instruction is free. It runs until the data of every read issued in the
// region is back.
//   - scheduled: the region's final order, walked through the wait queues
//                (MemoryGraph::ScoreSchedule).
//   - ideal:     every memory op started as early as the region's edges allow
//                (MemoryGraph::ScoreIdeal). No order is faster, so ideal is
//                never more than scheduled.
// Both use LLVM's latencies, or misched.txt's load-latency settings where set.
//
// A function's memory time is the sum of its regions' memory times, each
// weighted by its block's frequency: how many times the block is estimated to
// run per run of the function, from MachineBlockFrequencyInfo (the estimate
// LLVM's spill weights use). So it estimates the memory time of one wave's run
// through the kernel. Its efficiency is ideal / scheduled: 1 means the
// schedule's memory work is as fast as the ideal, lower means slower.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYTIMEANALYSIS_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYTIMEANALYSIS_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineBranchProbabilityInfo.h"
#include <vector>

namespace llvm {

class LiveIntervals;
class MachineFunction;
class MachineLoopInfo;
class SUnit;

namespace hierarchical_scheduler {

class RegionInfo;

/// The memory-time analysis of one function. The pass that walks the
/// function's regions makes one, hands it each region, then calls Report once.
class MemoryTimeAnalysis {
public:
  /// `mf`, `loop_info` and `lis` must not be null, and must outlive this
  /// object. Computes the function's block frequencies.
  MemoryTimeAnalysis(const MachineFunction *mf,
                     const MachineLoopInfo *loop_info,
                     const LiveIntervals *lis);

  /// Measures one region's final order. `sunits` must already be built by
  /// buildSchedGraph for `region`. `region_index` labels the region in the
  /// output. Fails hard if the scheduled memory time is below the ideal: no
  /// order beats the ideal, so that would be a bug.
  void AnalyzeRegion(MutableArrayRef<SUnit> sunits, const RegionInfo &region,
                     int region_index);

  /// Prints each region's memory times and the function's weighted totals,
  /// and appends the totals as one row to memory_time_analysis.csv in the
  /// compile's working directory. Call once, after the last region.
  void Report() const;

private:
  /// One region's memory times, and how often its block runs.
  struct RegionMemoryTime {
    int region_index = 0;
    int block_number = 0;
    int loop_depth = 0;
    double block_frequency = 0;  // runs per run of the function; entry is 1
    int scheduled_memory_time = 0;
    int ideal_memory_time = 0;
  };

  const MachineFunction *mf_;
  const MachineLoopInfo *loop_info_;
  const LiveIntervals *lis_;

  // The function's block frequencies, computed in the constructor from each
  // branch's probabilities.
  //
  // The probabilities aren't in branch_probability_info_. Each block stores
  // them itself, one per successor ("to block 6: 97%, to block 9: 3%"), set
  // when instruction selection created the blocks. branch_probability_info_
  // only looks them up: its getEdgeProbability(from, to) returns
  // from->getSuccProbability(to). So a default-constructed one is complete.
  // It's here because block_frequency_info_'s calculate() asks for one.
  //
  // block_frequency_info_ keeps a pointer to it, so it's declared first, which
  // makes it destroyed last.
  MachineBranchProbabilityInfo branch_probability_info_;
  MachineBlockFrequencyInfo block_frequency_info_;

  // Every region analyzed so far, in program order.
  std::vector<RegionMemoryTime> regions_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYTIMEANALYSIS_H
