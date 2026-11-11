//===- MemoryTimeAnalysis.cpp - Memory time of a final schedule -----------===//
//
// See MemoryTimeAnalysis.h.
//
//===----------------------------------------------------------------------===//

#include "MemoryTimeAnalysis.h"
#include "GCNSubtarget.h"
#include "MemoryGraph.h"
#include "RegionInfo.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/Format.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// A region's scheduled and ideal memory times.
struct MemoryTimes {
  int scheduled = 0;
  int ideal = 0;
};

// A function's memory times and whole schedule times, each region's weighted
// by how many times its block runs, and added up.
struct WeightedTotals {
  double scheduled_memory_time = 0;
  double ideal_memory_time = 0;

  // ideal / scheduled; none when scheduled is 0.
  std::optional<double> efficiency;

  // Time for all of the schedule's instructions, not just its memory waits:
  // the sum of the regions' schedule lengths. The ideal one is the same
  // schedule with only its memory work made ideal: each region's length minus
  // its memory time gap, floored (see MemoryTimeAnalysis.h).
  double scheduled_whole_schedule_time = 0;
  double ideal_whole_schedule_time = 0;

  // scheduled / ideal whole schedule time; none when the ideal is 0, which
  // happens only for a function with no regions.
  std::optional<double> whole_schedule_time_speedup;
};

// Builds the region's graphs from its final order, then scores that order and
// the ideal. `sunits` must already be built by buildSchedGraph for `region`.
MemoryTimes ScoreFinalOrderAndIdeal(MutableArrayRef<SUnit> sunits,
                                    const RegionInfo &region,
                                    const MachineFunction &mf,
                                    const LiveIntervals &lis) {
  // A region with no instructions has no memory work.
  MemoryTimes memory_times;
  if (sunits.empty()) {
    return memory_times;
  }

  // Build the graphs. The latency divisor only scales the schedule graph's
  // edge latencies, which the memory graph doesn't use: it takes each op's
  // latency from its SUnit. So any divisor gives the same memory times; 1
  // leaves the edges as LLVM gave them.
  const GCNSubtarget &st = mf.getSubtarget<GCNSubtarget>();
  std::unique_ptr<ScheduleGraph> graph = ScheduleGraph::BuildFromSUnits(
      sunits, st, mf, lis, mf.getRegInfo(), region, /*latency_divisor=*/1);
  std::unique_ptr<MemoryGraph> memory_graph = MemoryGraph::Build(*graph, st);

  // Score the final order, and the ideal. Their elapsed times count every
  // memory op whichever read types are tracked, so any tracking gives the
  // same times; this uses the default.
  const TrackedReadTypes tracked_read_types;
  const ArrayRef<const ScheduleNode *> final_order =
      graph->GetInputScheduleConstructor().GetScheduleOrder();
  memory_times.scheduled =
      memory_graph
          ->ScoreSchedule(final_order, LatencySetting::kLlvm,
                          tracked_read_types)
          .elapsed_time;
  memory_times.ideal =
      memory_graph->ScoreIdeal(LatencySetting::kLlvm, tracked_read_types)
          .elapsed_time;
  return memory_times;
}

// Prints a function's weighted totals as one line.
void PrintWeightedTotals(const WeightedTotals &totals) {
  llvm::outs() << format("\tweighted total: scheduled %.1f, ideal %.1f",
                         totals.scheduled_memory_time,
                         totals.ideal_memory_time);
  if (totals.efficiency) {
    llvm::outs() << format(", efficiency %.3f", *totals.efficiency);
  } else {
    llvm::outs() << ", efficiency none";
  }

  // The whole schedule times, on the same line.
  llvm::outs() << format("; whole schedule: scheduled %.1f, ideal %.1f",
                         totals.scheduled_whole_schedule_time,
                         totals.ideal_whole_schedule_time);
  if (totals.whole_schedule_time_speedup) {
    llvm::outs() << format(", speedup %.3f",
                           *totals.whole_schedule_time_speedup);
  } else {
    llvm::outs() << ", speedup none";
  }
  llvm::outs() << "\n";
}

// `field` as one CSV value: wrapped in double quotes, with any double quote
// inside it doubled, so a CSV reader keeps its commas as part of the value.
std::string QuoteCsvField(StringRef field) {
  std::string quoted = "\"";
  for (char c : field) {
    if (c == '"') {
      quoted += '"';
    }
    quoted += c;
  }
  quoted += '"';
  return quoted;
}

// Appends a function's weighted totals as one row of
// memory_time_analysis.csv:
//   function,scheduled_memory_time,ideal_memory_time,memory_time_efficiency,
//   scheduled_whole_schedule_time,ideal_whole_schedule_time,
//   whole_schedule_time_speedup
// (one line in the file). The function is the demangled signature
// kernel_resource_usage.csv uses, so the harness can match kernels across the
// two files. A signature has commas between its parameters, so it's quoted.
// The efficiency and speedup fields are empty when there is none.
//
// Each clang process of a build appends and never empties the file, so a
// benchmark with several source files keeps every kernel; the harness deletes
// the file before each build. There's no header row, since every process
// would write its own.
void AppendCsvRow(StringRef mangled_function_name,
                  const WeightedTotals &totals) {
  // Build the row: the function and its memory times.
  std::string row;
  raw_string_ostream row_stream(row);
  row_stream << QuoteCsvField(
                    MachineInstrSchedulerConfig::DemangleFunctionSignature(
                        mangled_function_name.str()))
             << format(",%.1f,%.1f,", totals.scheduled_memory_time,
                       totals.ideal_memory_time);
  if (totals.efficiency) {
    row_stream << format("%.4f", *totals.efficiency);
  }

  // Then its whole schedule times.
  row_stream << format(",%.1f,%.1f,", totals.scheduled_whole_schedule_time,
                       totals.ideal_whole_schedule_time);
  if (totals.whole_schedule_time_speedup) {
    row_stream << format("%.4f", *totals.whole_schedule_time_speedup);
  }
  row_stream << "\n";

  // Several clang processes of one build can append to the file at the same
  // time. Opened for appending, each write goes to the end of the file with
  // nothing else written in between (POSIX write() with O_APPEND), so sending
  // the whole row in one write keeps rows from mixing, with no lock.
  std::ofstream csv_file("memory_time_analysis.csv", std::ios::app);
  if (!csv_file) {
    return;
  }
  csv_file << row_stream.str();
}

} // namespace

MemoryTimeAnalysis::MemoryTimeAnalysis(const MachineFunction *mf,
                                       const MachineLoopInfo *loop_info,
                                       const LiveIntervals *lis)
    : mf_(mf), loop_info_(loop_info), lis_(lis) {
  // Turn the branch probabilities into block frequencies. Computed here rather
  // than asked of the pass manager, which has no up-to-date copy at this
  // point in the pipeline: its last one is from before AMDGPU lowers its
  // control flow.
  block_frequency_info_.calculate(*mf_, branch_probability_info_,
                                  *loop_info_);
}

void MemoryTimeAnalysis::AnalyzeRegion(MutableArrayRef<SUnit> sunits,
                                       const RegionInfo &region,
                                       int region_index, int schedule_length,
                                       int schedule_length_floor) {
  const MachineBasicBlock &block = *region.GetBlock();

  // Where the region's block sits: how many times it's estimated to run per
  // run of the function, and how deep in loops it is. The loop depth is only
  // printed, to check the frequency estimate by eye.
  RegionMemoryTime result;
  result.region_index = region_index;
  result.block_number = block.getNumber();
  result.loop_depth = static_cast<int>(loop_info_->getLoopDepth(&block));
  result.block_frequency =
      block_frequency_info_.getBlockFreqRelativeToEntryBlock(&block);

  // The region's memory times.
  const MemoryTimes memory_times =
      ScoreFinalOrderAndIdeal(sunits, region, *mf_, *lis_);
  result.scheduled_memory_time = memory_times.scheduled;
  result.ideal_memory_time = memory_times.ideal;

  // The region's schedule length, from the schedule-length analysis.
  result.schedule_length = schedule_length;
  result.schedule_length_floor = schedule_length_floor;

  // No order beats the ideal, so a scheduled time below it is a bug.
  if (result.scheduled_memory_time < result.ideal_memory_time) {
    report_fatal_error("MemoryTimeAnalysis: region " + Twine(region_index) +
                       " of " + mf_->getName() + ": scheduled memory time " +
                       Twine(result.scheduled_memory_time) +
                       " is below the ideal " +
                       Twine(result.ideal_memory_time));
  }

  // No order is shorter than the floor, so a length below it is a bug.
  if (result.schedule_length < result.schedule_length_floor) {
    report_fatal_error("MemoryTimeAnalysis: region " + Twine(region_index) +
                       " of " + mf_->getName() + ": schedule length " +
                       Twine(result.schedule_length) +
                       " is below its floor " +
                       Twine(result.schedule_length_floor));
  }

  regions_.push_back(result);
}

void MemoryTimeAnalysis::Report() const {
  llvm::outs() << "\n=== Memory-time analysis: " << mf_->getName() << " ===\n";

  // For each region: work out its whole schedule time with only its memory
  // work made ideal, print the region, and add its times to the function's,
  // each weighted by how many times its block runs.
  WeightedTotals totals;
  for (const RegionMemoryTime &region : regions_) {
    // The region's length minus its memory time gap, but never below its
    // floor. See MemoryTimeAnalysis.h for why the floor.
    const int memory_time_gap =
        region.scheduled_memory_time - region.ideal_memory_time;
    const int ideal_whole_schedule_time =
        std::max(region.schedule_length - memory_time_gap,
                 region.schedule_length_floor);

    // Print the region.
    llvm::outs() << "\tregion[" << region.region_index << "] block "
                 << region.block_number << ", loop depth " << region.loop_depth
                 << format(", frequency %.2f", region.block_frequency)
                 << ": scheduled " << region.scheduled_memory_time
                 << ", ideal " << region.ideal_memory_time
                 << "; whole schedule: length " << region.schedule_length
                 << ", floor " << region.schedule_length_floor << ", ideal "
                 << ideal_whole_schedule_time << "\n";

    // Add its weighted times.
    totals.scheduled_memory_time +=
        region.block_frequency * region.scheduled_memory_time;
    totals.ideal_memory_time +=
        region.block_frequency * region.ideal_memory_time;
    totals.scheduled_whole_schedule_time +=
        region.block_frequency * region.schedule_length;
    totals.ideal_whole_schedule_time +=
        region.block_frequency * ideal_whole_schedule_time;
  }

  // Efficiency is ideal / scheduled. A function with no reads has 0 for both,
  // and no efficiency.
  if (totals.scheduled_memory_time > 0) {
    totals.efficiency =
        totals.ideal_memory_time / totals.scheduled_memory_time;
  }

  // Speedup is scheduled / ideal whole schedule time. A function with no
  // regions has 0 for both, and no speedup.
  if (totals.ideal_whole_schedule_time > 0) {
    totals.whole_schedule_time_speedup =
        totals.scheduled_whole_schedule_time /
        totals.ideal_whole_schedule_time;
  }

  // Then the totals.
  PrintWeightedTotals(totals);

  // Write the totals to the CSV.
  AppendCsvRow(mf_->getName(), totals);
}
