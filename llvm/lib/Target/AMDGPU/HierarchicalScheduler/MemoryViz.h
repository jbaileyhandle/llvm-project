//===- MemoryViz.h - Memory-time visualization data -------------*- C++ -*-===//
//
// Writes one memory_viz/<function>.json per function for
// viz/memory_viewer.html: the function's blocks and regions, each region's
// share of the function's possible gain from ideal memory work, and, for every
// instruction of each region's final order, what the memory model does there.
//
// Per region it records:
//   - Every instruction in final order, with the memory model's clock when it
//     issues, and its VGPR and SGPR pressure. The pressure is split into the
//     registers holding load results, which of those the load hasn't filled
//     yet (in flight), and everything else.
//   - The virtual registers each instruction writes and reads, and those the
//     region leaves live, so the viewer can link definitions to uses.
//   - The rows each instruction must come after (its strong predecessors in
//     the schedule graph, with the edge's kind), so the viewer can show
//     where rows can move.
//   - Every memory op: its type, which wait queues it goes in, when it issues
//     and when its data is back, and the rows whose waits pop it. Barriers
//     are listed with the ops, so the tier view can place them.
//   - Every wait: what it waited for, what it popped, how long it stalled,
//     and whether it was a drain.
//   - For each load, whether it was stuck behind waits it didn't need: it
//     issued later than its own inputs allowed, and which waits moved the
//     clock past them. "Inputs" are its memory-graph predecessors, at their
//     times in this schedule, so the delay of a stuck load isn't charged
//     again to the loads after it.
//   - Tiers, in this schedule and in the ideal. A tier is a group of ops no
//     wait separates: in this schedule, the ops between two waits that stall
//     (the clock only moves there); in the ideal, the ops after the same
//     number of rounds of waiting for data. Within a tier the ops can go in
//     any order; no op can move to an earlier tier without changing the
//     waits. Every op's tier in this schedule is at least its ideal tier.
//
// The memory model is MemoryTimeAnalysis's (MemoryGraph, walked through the
// wait queues), on the same final order. This rebuilds its own graphs rather
// than sharing MemoryTimeAnalysis's, so that analysis is untouched.
//
// Like schedule_length_viz/, it's written on every compile the length analysis
// runs on.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYVIZ_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYVIZ_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineBranchProbabilityInfo.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include <map>
#include <string>
#include <vector>

namespace llvm {

class LiveIntervals;
class MachineFunction;
class MachineLoopInfo;

namespace hierarchical_scheduler {

class RegionInfo;

/// A row an instruction must come after, and why.
struct MemoryVizPredecessor {
  int row = 0;

  // The kind of edge: data, anti, output, barrier, may-alias, must-alias or
  // artificial.
  std::string kind;
};

/// One instruction of a region's final order.
struct MemoryVizRow {
  std::string text;

  // The memory model's clock when this instruction issues. It only moves at
  // waits, so an instruction that isn't a memory node carries the clock of
  // the last one before it.
  int memory_time = 0;

  // The cycle it issues at in the whole schedule time: the length analysis's
  // raw lens, replaying the same order.
  int schedule_cycle = 0;

  // Register pressure after this instruction issues, as the length analysis
  // reports it.
  int vgpr = 0;
  int sgpr = 0;

  // Of the live registers, those holding a load's result, and of those, the
  // ones whose load hasn't been popped by a wait yet (in flight). Counted in
  // 32-bit registers from liveness, so they can differ slightly from the
  // pressure above.
  int vgpr_load = 0;
  int vgpr_load_in_flight = 0;
  int sgpr_load = 0;
  int sgpr_load_in_flight = 0;

  // Index into the region's ops and waits, or -1.
  int op = -1;
  int wait = -1;

  // The virtual registers it writes and reads, by number (%85 is 85). A
  // write to part of a register that keeps the rest (a subregister def
  // without undef) also reads it.
  std::vector<int> defs;
  std::vector<int> uses;

  // The rows it must come after: its predecessors in the region's schedule
  // graph over strong edges, the ones the scheduler waits on. Edges from the
  // region's entry are left out.
  std::vector<MemoryVizPredecessor> predecessors;
};

/// A wait that moved the clock past a stuck load's inputs, and by how much.
struct MemoryVizStuckWait {
  int wait = 0;    // index into the region's waits
  int cycles = 0;
};

/// One memory op or barrier of a region.
struct MemoryVizOp {
  int row = 0;

  // The type's prefix and a count per type: V (vector), F (flat), S
  // (scalar), LDS, GDS, with a "w" on writes, and BAR for barriers.
  std::string label;
  std::string type;   // vector, flat, scalar, lds, gds or barrier
  bool read = false;
  bool vmcnt = false;
  bool lgkmcnt = false;
  int latency = 0;
  int issue_time = 0;
  int data_time = 0;  // issue_time + latency

  // The rows whose waits popped this op: one per queue it's in. A row equal
  // to the region's row count is the end of the region (the sink).
  std::vector<int> pop_rows;

  // The register it writes, as MIR prints it (%43), for naming what a wait
  // waited for. Empty for a write.
  std::string def;

  // Reads only: the earliest its own inputs allowed it to issue, and how much
  // later it issued (0 if not stuck). stuck_waits are the waits, its own
  // included, that moved the clock past inputs_ready_time; their cycles add
  // up to stuck.
  int inputs_ready_time = 0;
  int stuck = 0;
  std::vector<MemoryVizStuckWait> stuck_waits;

  // Its tier in this schedule and in the ideal, and its earliest start in the
  // ideal (MemoryGraph::ScoreIdeal).
  int tier = 0;
  int ideal_tier = 0;
  int ideal_start = 0;

  // The ops and barriers it waits on, directly or through a node before it,
  // as indices into the region's ops. Only the nearest: an input that's also
  // an input of another of its inputs is left out.
  std::vector<int> inputs;
};

/// One wait: a node of the walk whose waits popped something.
struct MemoryVizWait {
  int row = 0;          // the region's row count for the sink
  int begin_time = 0;   // the clock when the walk reached it
  int end_time = 0;     // when it issued, after its waits
  std::vector<int> waited_for;  // ops whose data it reads
  std::vector<int> popped;      // ops its waits popped
  bool drain = false;    // popped an op issued after everything it needed
  bool barrier = false;  // an s_barrier, which drains both queues
  bool cautious = false; // cautious mode was on (FLAT in both queues)
};

/// One tier of this schedule.
struct MemoryVizTier {
  int clock = 0;  // the memory clock throughout the tier
  int wait = -1;  // index of the wait that stalled to start it; -1 for the first
};

/// One region's visualization data.
struct MemoryVizRegion {
  int index = 0;

  // Its first instruction's position in its block, to keep a block's regions
  // in program order. Not written out.
  int position_in_block = 0;

  int scheduled_memory_time = 0;
  int ideal_memory_time = 0;
  int schedule_length = 0;
  int schedule_length_floor = 0;

  // The region's gain from ideal memory work: its length minus its ideal
  // whole schedule time, max(length - memory gap, floor). Weighted by the
  // block's frequency in the function's totals.
  int gain = 0;

  std::vector<MemoryVizRow> rows;
  std::vector<MemoryVizOp> ops;
  std::vector<MemoryVizWait> waits;

  // This schedule's tiers, in order. If the region end stalls, the last one
  // starts there and holds no op.
  std::vector<MemoryVizTier> tiers;

  // The virtual registers read after the region, by number.
  std::vector<int> live_out;
};

class MemoryViz {
public:
  /// All three must outlive this.
  MemoryViz(const MachineFunction *mf, const MachineLoopInfo *loop_info,
            const LiveIntervals *lis);

  /// Records one region, whose SUnits buildSchedGraph has built over its final
  /// order. `schedule_length` and `schedule_length_floor` are the length
  /// analysis's raw-lens ones.
  void AnalyzeRegion(MutableArrayRef<SUnit> sunits, const RegionInfo &region,
                     int region_index, int schedule_length,
                     int schedule_length_floor);

  /// Writes memory_viz/<function>.json. No-op if no region was recorded.
  void WriteJson(StringRef scheduler_name) const;

private:
  const MachineFunction *mf_;
  const MachineLoopInfo *loop_info_;
  const LiveIntervals *lis_;

  // Block frequencies, computed here the way MemoryTimeAnalysis does.
  MachineBranchProbabilityInfo branch_probability_info_;
  MachineBlockFrequencyInfo block_frequency_info_;

  // Recorded regions, by block number, in program order within each block.
  // The scheduling driver visits a block's regions bottom-up, so
  // AnalyzeRegion inserts each one in place.
  std::map<int, std::vector<MemoryVizRegion>> regions_by_block_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYVIZ_H
