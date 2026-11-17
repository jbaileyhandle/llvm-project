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

// A function's totals: each region's value weighted by how many times its
// block runs, and added up.
struct WeightedTotals {
  // Memory time, in the memory model's time.
  double memory_time = 0;
  double ideal_memory_time = 0;

  // ideal_memory_time / memory_time; none for a function with no reads.
  std::optional<double> memory_efficiency;

  // Latency: schedule length in cycles (raw lens), the time for all of the
  // regions' instructions, and the floor no order beats.
  double latency = 0;
  double latency_floor = 0;

  // latency_floor / latency; none for a function with no regions.
  std::optional<double> latency_efficiency;

  // The latency if only the memory work were ideal: each region's latency
  // minus its memory time gap, never below its floor (see
  // MemoryTimeAnalysis.h).
  double latency_with_ideal_memory = 0;

  // The speedups: latency / latency_with_ideal_memory, and latency /
  // latency_floor. None for a function with no regions.
  std::optional<double> speedup_with_ideal_memory;
  std::optional<double> speedup_with_ideal_latency;
};

// Adds one region's values to `totals`, each weighted by `frequency`. Returns
// the region's latency with ideal memory: its latency minus its memory time
// gap, never below its floor (see MemoryTimeAnalysis.h for why the floor).
int AddRegion(double frequency, int memory_time, int ideal_memory_time,
              int latency, int latency_floor, WeightedTotals &totals) {
  // The region's latency with ideal memory.
  const int memory_time_gap = memory_time - ideal_memory_time;
  const int latency_with_ideal_memory =
      std::max(latency - memory_time_gap, latency_floor);

  // Its weighted values.
  totals.memory_time += frequency * memory_time;
  totals.ideal_memory_time += frequency * ideal_memory_time;
  totals.latency += frequency * latency;
  totals.latency_floor += frequency * latency_floor;
  totals.latency_with_ideal_memory += frequency * latency_with_ideal_memory;
  return latency_with_ideal_memory;
}

// Fills in `totals`' efficiencies and speedups, once every region is added.
void ComputeRatios(WeightedTotals &totals) {
  // The efficiencies, ideal / actual. A function with no reads has no memory
  // time, and no memory efficiency; one with no regions has no latency.
  if (totals.memory_time > 0) {
    totals.memory_efficiency = totals.ideal_memory_time / totals.memory_time;
  }
  if (totals.latency > 0) {
    totals.latency_efficiency = totals.latency_floor / totals.latency;
  }

  // The speedups: the latency over the latency with ideal memory, and over
  // the floor. A function with no regions has neither.
  if (totals.latency_with_ideal_memory > 0) {
    totals.speedup_with_ideal_memory =
        totals.latency / totals.latency_with_ideal_memory;
  }
  if (totals.latency_floor > 0) {
    totals.speedup_with_ideal_latency = totals.latency / totals.latency_floor;
  }
}

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
      sunits, st, mf, lis, mf.getRegInfo(), region, /*latency_divisor=*/1,
      /*count_live_out_latency=*/false);
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

// Prints a ratio with three decimals, or "none".
void PrintRatio(const std::optional<double> &ratio) {
  if (ratio) {
    llvm::outs() << format("%.3f", *ratio);
  } else {
    llvm::outs() << "none";
  }
}

// Prints a function's weighted totals as one line.
void PrintWeightedTotals(const WeightedTotals &totals) {
  // Memory time.
  llvm::outs() << format("\tweighted total: memory time %.1f, ideal %.1f",
                         totals.memory_time, totals.ideal_memory_time)
               << ", efficiency ";
  PrintRatio(totals.memory_efficiency);

  // Latency.
  llvm::outs() << format("; latency %.1f, floor %.1f", totals.latency,
                         totals.latency_floor)
               << ", efficiency ";
  PrintRatio(totals.latency_efficiency);

  // The what-ifs.
  llvm::outs() << format("; with ideal memory: latency %.1f",
                         totals.latency_with_ideal_memory)
               << ", speedup ";
  PrintRatio(totals.speedup_with_ideal_memory);
  llvm::outs() << "; with ideal latency: speedup ";
  PrintRatio(totals.speedup_with_ideal_latency);
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

// Appends a function's weighted totals as one row of ideal_time_analysis.csv:
//   function,memory_time,ideal_memory_time,memory_efficiency,
//   latency,latency_floor,latency_efficiency,
//   latency_with_ideal_memory,speedup_with_ideal_memory,
//   speedup_with_ideal_latency
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
  std::string row;
  raw_string_ostream row_stream(row);

  // A ratio with four decimals, or an empty field when there's none.
  auto append_ratio = [&](const std::optional<double> &ratio) {
    if (ratio) {
      row_stream << format("%.4f", *ratio);
    }
  };

  // The function.
  row_stream << QuoteCsvField(
      MachineInstrSchedulerConfig::DemangleFunctionSignature(
          mangled_function_name.str()));

  // Its memory time.
  row_stream << format(",%.1f,%.1f,", totals.memory_time,
                       totals.ideal_memory_time);
  append_ratio(totals.memory_efficiency);

  // Its latency.
  row_stream << format(",%.1f,%.1f,", totals.latency, totals.latency_floor);
  append_ratio(totals.latency_efficiency);

  // The what-ifs.
  row_stream << format(",%.1f,", totals.latency_with_ideal_memory);
  append_ratio(totals.speedup_with_ideal_memory);
  row_stream << ",";
  append_ratio(totals.speedup_with_ideal_latency);
  row_stream << "\n";

  // Several clang processes of one build can append to the file at the same
  // time. Opened for appending, each write goes to the end of the file with
  // nothing else written in between (POSIX write() with O_APPEND), so sending
  // the whole row in one write keeps rows from mixing, with no lock.
  std::ofstream csv_file("ideal_time_analysis.csv", std::ios::app);
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

  // For each region: add its values to the function's, weighted by how many
  // times its block runs, and print it.
  WeightedTotals totals;
  for (const RegionMemoryTime &region : regions_) {
    const int latency_with_ideal_memory =
        AddRegion(region.block_frequency, region.scheduled_memory_time,
                  region.ideal_memory_time, region.schedule_length,
                  region.schedule_length_floor, totals);

    llvm::outs() << "\tregion[" << region.region_index << "] block "
                 << region.block_number << ", loop depth " << region.loop_depth
                 << format(", frequency %.2f", region.block_frequency)
                 << ": memory time " << region.scheduled_memory_time
                 << ", ideal " << region.ideal_memory_time << "; latency "
                 << region.schedule_length << ", floor "
                 << region.schedule_length_floor << ", with ideal memory "
                 << latency_with_ideal_memory << "\n";
  }

  // The efficiencies and speedups.
  ComputeRatios(totals);

  // Then the totals.
  PrintWeightedTotals(totals);

  // Write the totals to the CSV.
  AppendCsvRow(mf_->getName(), totals);
}
