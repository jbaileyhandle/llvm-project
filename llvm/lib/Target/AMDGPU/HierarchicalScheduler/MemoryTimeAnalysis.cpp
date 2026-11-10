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

// A function's memory times, each region's weighted by how many times its
// block runs, and added up.
struct WeightedTotals {
  double scheduled_memory_time = 0;
  double ideal_memory_time = 0;

  // ideal / scheduled; none when scheduled is 0.
  std::optional<double> efficiency;
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
//   function,scheduled_memory_time,ideal_memory_time,memory_time_efficiency
// The function is the demangled signature kernel_resource_usage.csv uses, so
// the harness can match kernels across the two files. A signature has commas
// between its parameters, so it's quoted. The efficiency field is empty when
// there is none.
//
// Each clang process of a build appends and never empties the file, so a
// benchmark with several source files keeps every kernel; the harness deletes
// the file before each build. There's no header row, since every process
// would write its own.
void AppendCsvRow(StringRef mangled_function_name,
                  const WeightedTotals &totals) {
  // Build the row.
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
                                       int region_index) {
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

  // No order beats the ideal, so a scheduled time below it is a bug.
  if (result.scheduled_memory_time < result.ideal_memory_time) {
    report_fatal_error("MemoryTimeAnalysis: region " + Twine(region_index) +
                       " of " + mf_->getName() + ": scheduled memory time " +
                       Twine(result.scheduled_memory_time) +
                       " is below the ideal " +
                       Twine(result.ideal_memory_time));
  }

  regions_.push_back(result);
}

void MemoryTimeAnalysis::Report() const {
  // Weight each region's memory times by how many times its block runs, and
  // add them up.
  WeightedTotals totals;
  for (const RegionMemoryTime &region : regions_) {
    totals.scheduled_memory_time +=
        region.block_frequency * region.scheduled_memory_time;
    totals.ideal_memory_time +=
        region.block_frequency * region.ideal_memory_time;
  }

  // Efficiency is ideal / scheduled. A function with no reads has 0 for both,
  // and no efficiency.
  if (totals.scheduled_memory_time > 0) {
    totals.efficiency =
        totals.ideal_memory_time / totals.scheduled_memory_time;
  }

  // Print one line per region, then the totals.
  llvm::outs() << "\n=== Memory-time analysis: " << mf_->getName() << " ===\n";
  for (const RegionMemoryTime &region : regions_) {
    llvm::outs() << "\tregion[" << region.region_index << "] block "
                 << region.block_number << ", loop depth " << region.loop_depth
                 << format(", frequency %.2f", region.block_frequency)
                 << ": scheduled " << region.scheduled_memory_time
                 << ", ideal " << region.ideal_memory_time << "\n";
  }
  PrintWeightedTotals(totals);

  // Write the totals to the CSV.
  AppendCsvRow(mf_->getName(), totals);
}
