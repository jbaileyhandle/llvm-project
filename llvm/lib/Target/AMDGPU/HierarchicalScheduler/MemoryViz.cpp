//===- MemoryViz.cpp - Memory-time visualization data ---------------------===//
//
// See MemoryViz.h.
//
//===----------------------------------------------------------------------===//

#include "MemoryViz.h"
#include "GCNRegPressure.h"
#include "GCNRegisterTracker.h"
#include "GCNSubtarget.h"
#include "InstrText.h"
#include "MemoryGraph.h"
#include "RegionInfo.h"
#include "SIMachineFunctionInfo.h"
#include "SIRegisterInfo.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/Hashing.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/JSON.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cctype>
#include <memory>
#include <set>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// `name` made safe for a filename. A copy of ScheduleLengthAnalysis.cpp's
// SanitizeForFilename, so both viewers' files for a function have the same
// name: [A-Za-z0-9._-] kept, everything else '_', and a long name cut short
// with a hash of the whole name appended.
std::string SanitizeForFilename(StringRef name) {
  std::string out;
  out.reserve(name.size());
  for (char c : name) {
    const bool keep = std::isalnum(static_cast<unsigned char>(c)) ||
                      c == '.' || c == '_' || c == '-';
    out += keep ? c : '_';
  }
  constexpr size_t kMaxBasenameLength = 200;
  if (out.size() > kMaxBasenameLength) {
    const std::string suffix =
        "_" + utohexstr(static_cast<uint64_t>(hash_value(name)),
                        /*LowerCase=*/true);
    out.resize(kMaxBasenameLength - suffix.size());
    out += suffix;
  }
  return out;
}

const char *MemTypeName(MemType type) {
  switch (type) {
  case MemType::kVector:
    return "vector";
  case MemType::kFlat:
    return "flat";
  case MemType::kScalar:
    return "scalar";
  case MemType::kLds:
    return "lds";
  case MemType::kGds:
    return "gds";
  }
  llvm_unreachable("unknown MemType");
}

// The label prefix for an op of `type`.
const char *MemTypePrefix(MemType type) {
  switch (type) {
  case MemType::kVector:
    return "V";
  case MemType::kFlat:
    return "F";
  case MemType::kScalar:
    return "S";
  case MemType::kLds:
    return "LDS";
  case MemType::kGds:
    return "GDS";
  }
  llvm_unreachable("unknown MemType");
}

// The first virtual register `mi` writes, or an invalid Register.
Register FirstVirtualDef(const MachineInstr &mi) {
  for (const MachineOperand &operand : mi.operands()) {
    if (operand.isReg() && operand.isDef() && operand.getReg().isVirtual()) {
      return operand.getReg();
    }
  }
  return Register();
}

// Whether any instruction writing `reg` is a load.
bool IsWrittenByLoad(Register reg, const MachineRegisterInfo &mri) {
  for (const MachineInstr &def : mri.def_instructions(reg)) {
    if (def.mayLoad()) {
      return true;
    }
  }
  return false;
}

// The viewer's name for a strong edge's kind.
const char *StrongEdgeKindName(ScheduleEdge::Kind kind) {
  switch (kind) {
  case ScheduleEdge::kData:
    return "data";
  case ScheduleEdge::kAnti:
    return "anti";
  case ScheduleEdge::kOutput:
    return "output";
  case ScheduleEdge::kBarrier:
    return "barrier";
  case ScheduleEdge::kMayAliasMem:
    return "may-alias";
  case ScheduleEdge::kMustAliasMem:
    return "must-alias";
  case ScheduleEdge::kArtificial:
    return "artificial";
  case ScheduleEdge::kSubgraphOrderEdge:
    return "subgraph-order";
  case ScheduleEdge::kCluster:
  case ScheduleEdge::kWeak:
    break;
  }
  llvm_unreachable("not a strong edge kind");
}

// Whether two ops share a wait queue.
bool ShareAQueue(const MemoryVizOp &a, const MemoryVizOp &b) {
  return (a.vmcnt && b.vmcnt) || (a.lgkmcnt && b.lgkmcnt);
}

// A live register holding a load's result, for the pressure split.
struct LoadResultRegister {
  const LiveInterval *interval;
  int width;      // in 32-bit registers
  bool is_vgpr;   // otherwise SGPR
  // The rows during which its load is in flight: from the load's row up to,
  // not including, the row of the wait that pops it. Both -1 when the load
  // isn't in this region.
  int in_flight_begin_row = -1;
  int in_flight_end_row = -1;
};

} // anonymous namespace

MemoryViz::MemoryViz(const MachineFunction *mf,
                     const MachineLoopInfo *loop_info,
                     const LiveIntervals *lis)
    : mf_(mf), loop_info_(loop_info), lis_(lis) {
  // Block frequencies from the branch probabilities, as MemoryTimeAnalysis
  // computes them (see its constructor for why not the pass manager's).
  block_frequency_info_.calculate(*mf_, branch_probability_info_,
                                  *loop_info_);
}

void MemoryViz::AnalyzeRegion(MutableArrayRef<SUnit> sunits,
                              const RegionInfo &region, int region_index,
                              int schedule_length,
                              int schedule_length_floor) {
  if (sunits.empty()) {
    return;
  }

  const MachineFunction &mf = *mf_;
  const GCNSubtarget &st = mf.getSubtarget<GCNSubtarget>();
  const MachineRegisterInfo &mri = mf.getRegInfo();
  const SIRegisterInfo *tri = st.getRegisterInfo();

  MemoryVizRegion result;
  result.index = region_index;
  result.schedule_length = schedule_length;
  result.schedule_length_floor = schedule_length_floor;

  // The graphs. The schedule graph is built like the length analysis's raw
  // lens, so each instruction's schedule cycle matches it. The memory graph
  // doesn't read edge latencies, so it's the same as MemoryTimeAnalysis's.
  std::unique_ptr<ScheduleGraph> graph = ScheduleGraph::BuildFromSUnits(
      sunits, st, mf, *lis_, mri, region, /*latency_divisor=*/1,
      /*count_live_out_latency=*/true);
  std::unique_ptr<MemoryGraph> memory_graph = MemoryGraph::Build(*graph, st);
  const ArrayRef<const ScheduleNode *> final_order =
      graph->GetInputScheduleConstructor().GetScheduleOrder();
  const ScheduleLengthTracker &length_tracker =
      graph->GetInputScheduleConstructor().GetLengthTracker();

  // One row per real instruction of the final order, with the register
  // pressure after it issues: replay the order, as the length analysis does.
  const bool unified_vgpr = st.hasGFX90AInsts();
  DenseMap<const ScheduleNode *, int> row_of_schedule_node;
  std::vector<const MachineInstr *> row_instrs;
  ScheduleConstructor replay(*graph, st, mf);
  for (const ScheduleNode *node : final_order) {
    replay.Schedule(node);
    if (!node->IsRealInstruction() || !node->GetSUnit()->getInstr()) {
      continue;
    }
    const MachineInstr &mi = *node->GetSUnit()->getInstr();
    const GCNRegPressure &pressure =
        replay.GetPressureTracker().GetCurrentPressure();

    MemoryVizRow row;
    row.text = InstrText(mi);
    row.schedule_cycle = length_tracker.GetScheduledCycle(node);
    row.vgpr = static_cast<int>(pressure.getVGPRNum(unified_vgpr));
    row.sgpr = static_cast<int>(pressure.getSGPRNum());

    // Its virtual registers, each once. readsReg also counts a subregister
    // def that keeps the rest of the register, and skips undef reads.
    for (const MachineOperand &operand : mi.operands()) {
      if (!operand.isReg() || !operand.getReg().isVirtual()) {
        continue;
      }
      const int reg_number = static_cast<int>(operand.getReg().virtRegIndex());
      if (operand.isDef() && !is_contained(row.defs, reg_number)) {
        row.defs.push_back(reg_number);
      }
      if (operand.readsReg() && !is_contained(row.uses, reg_number)) {
        row.uses.push_back(reg_number);
      }
    }

    // The rows it must come after. Its predecessors are earlier in the order,
    // so they already have rows; the entry node has none and is skipped.
    for (const ScheduleEdge &edge : node->Predecessors()) {
      if (!edge.IsStrongEdge()) {
        continue;
      }
      auto predecessor_row = row_of_schedule_node.find(edge.node_);
      if (predecessor_row == row_of_schedule_node.end()) {
        continue;
      }
      row.predecessors.push_back(
          {predecessor_row->second, StrongEdgeKindName(edge.kind_)});
    }

    row_of_schedule_node[node] = static_cast<int>(result.rows.size());
    row_instrs.push_back(&mi);
    result.rows.push_back(std::move(row));
  }
  const int num_rows = static_cast<int>(result.rows.size());
  if (num_rows == 0) {
    return;
  }

  // The virtual registers read after the region: the exit node uses them.
  for (const RegWithLaneMask &live_out : graph->GetExitNode()->RegUses()) {
    if (!live_out.reg.isVirtual()) {
      continue;
    }
    const int reg_number = static_cast<int>(live_out.reg.virtRegIndex());
    if (!is_contained(result.live_out, reg_number)) {
      result.live_out.push_back(reg_number);
    }
  }

  // Walk the final order through the wait queues, keeping every step, and
  // score the ideal. The gain is the region's length minus its ideal whole
  // schedule time (MemoryTimeAnalysis.h).
  const TrackedReadTypes tracked_read_types;
  std::vector<WalkStep> steps;
  result.scheduled_memory_time =
      memory_graph
          ->ScoreSchedule(final_order, LatencySetting::kLlvm,
                          tracked_read_types, &steps)
          .elapsed_time;
  result.ideal_memory_time =
      memory_graph->ScoreIdeal(LatencySetting::kLlvm, tracked_read_types)
          .elapsed_time;
  const int memory_time_gap =
      result.scheduled_memory_time - result.ideal_memory_time;
  result.gain = schedule_length -
                std::max(schedule_length - memory_time_gap,
                         schedule_length_floor);

  // A memory node's row: its instruction's, or num_rows for the sink, which
  // stands for the end of the region.
  auto row_of = [&](const MemoryNode *node) {
    if (node->GetKind() == MemoryNodeKind::kSink) {
      return num_rows;
    }
    return row_of_schedule_node.lookup(node->GetOriginalNode());
  };

  // When each memory node issued.
  DenseMap<const MemoryNode *, int> issue_time_of;
  for (const WalkStep &step : steps) {
    issue_time_of[step.node] = step.issue_time;
  }

  // The memory ops and barriers, in row order, labeled by type with a count
  // per type: V1, V2, ..., LDS1, ..., BAR1, ...
  DenseMap<const MemoryNode *, int> op_of_node;
  DenseMap<Register, int> op_of_result_register;
  std::map<std::string, int> count_of_prefix;
  for (const WalkStep &step : steps) {
    const MemoryNode &node = *step.node;
    const bool is_barrier = node.GetKind() == MemoryNodeKind::kBarrier;
    if (node.GetKind() != MemoryNodeKind::kMemOp && !is_barrier) {
      continue;
    }

    MemoryVizOp op;
    op.row = row_of(&node);
    op.issue_time = step.issue_time;
    op.data_time = step.issue_time;
    if (is_barrier) {
      op.type = "barrier";
      op.label = "BAR" + std::to_string(++count_of_prefix["BAR"]);
      const int op_index = static_cast<int>(result.ops.size());
      op_of_node[&node] = op_index;
      result.rows[op.row].op = op_index;
      result.ops.push_back(std::move(op));
      continue;
    }

    const MemOpInfo &info = *node.GetMemOpInfo();
    op.read = info.access == MemAccess::kRead;
    const std::string prefix = MemTypePrefix(info.type);
    op.label = prefix + std::to_string(++count_of_prefix[prefix]) +
               (op.read ? "" : "w");
    op.type = MemTypeName(info.type);
    op.vmcnt = info.counts_in_vmcnt;
    op.lgkmcnt = info.counts_in_lgkmcnt;
    op.latency = node.GetLatency(LatencySetting::kLlvm);
    op.data_time = step.issue_time + op.latency;

    // The register a read fills, to name what a wait waits for.
    if (op.read) {
      const Register def = FirstVirtualDef(*node.GetOriginalInstr());
      if (def.isValid()) {
        std::string name;
        raw_string_ostream os(name);
        os << printReg(def, tri);
        op.def = os.str();
        op_of_result_register[def] = static_cast<int>(result.ops.size());
      }
    }

    const int op_index = static_cast<int>(result.ops.size());
    op_of_node[&node] = op_index;
    result.rows[op.row].op = op_index;
    result.ops.push_back(std::move(op));
  }

  // The waits: every step whose waits popped something. The sink's is the
  // region end; it pops whatever is still in flight.
  DenseMap<const MemoryNode *, int> wait_of_node;
  for (const WalkStep &step : steps) {
    if (step.popped.empty()) {
      continue;
    }
    const MemoryNode &node = *step.node;
    const bool is_sink = node.GetKind() == MemoryNodeKind::kSink;

    MemoryVizWait wait;
    wait.row = row_of(&node);
    wait.begin_time = step.issue_wait_begin_time;
    wait.end_time = step.issue_time;
    wait.barrier = node.GetKind() == MemoryNodeKind::kBarrier;
    wait.cautious = step.cautious;
    for (const MemoryNode *load : node.GetLoadsUsed()) {
      wait.waited_for.push_back(op_of_node.lookup(load));
    }
    for (const MemoryNode *popped : step.popped) {
      const int popped_op = op_of_node.lookup(popped);
      wait.popped.push_back(popped_op);
      result.ops[popped_op].pop_rows.push_back(wait.row);
    }

    // A drain popped an op issued after everything this node needed from the
    // same queue: an in-order wait never would. The sink waits for
    // everything, so it isn't one.
    if (!is_sink) {
      for (int popped_op : wait.popped) {
        int newest_needed_row = -1;
        for (int needed_op : wait.waited_for) {
          if (ShareAQueue(result.ops[popped_op], result.ops[needed_op])) {
            newest_needed_row =
                std::max(newest_needed_row, result.ops[needed_op].row);
          }
        }
        if (result.ops[popped_op].row > newest_needed_row) {
          wait.drain = true;
        }
      }
    }

    wait_of_node[&node] = static_cast<int>(result.waits.size());
    if (!is_sink) {
      result.rows[wait.row].wait = static_cast<int>(result.waits.size());
    }
    result.waits.push_back(std::move(wait));
  }

  // The memory clock at every row: a memory node's issue time, carried down
  // through the rows that aren't memory nodes.
  int clock = 0;
  DenseMap<int, int> issue_time_of_row;
  for (const WalkStep &step : steps) {
    if (step.node->GetKind() != MemoryNodeKind::kSink) {
      issue_time_of_row[row_of(step.node)] = step.issue_time;
    }
  }
  for (int row = 0; row < num_rows; ++row) {
    auto it = issue_time_of_row.find(row);
    if (it != issue_time_of_row.end()) {
      clock = it->second;
    }
    result.rows[row].memory_time = clock;
  }

  // Stuck loads: issued later than their own inputs allowed. A read's inputs
  // are its memory-graph predecessors at their times in this schedule: a
  // load whose data it reads is ready at its issue time plus its latency, and
  // any other predecessor at its issue time.
  for (const WalkStep &step : steps) {
    auto op_it = op_of_node.find(step.node);
    if (op_it == op_of_node.end() || !result.ops[op_it->second].read) {
      continue;
    }
    MemoryVizOp &op = result.ops[op_it->second];

    int inputs_ready_time = 0;
    for (const MemoryEdge &edge : step.node->Predecessors()) {
      int ready = issue_time_of.lookup(edge.node);
      if (edge.waits_for_data) {
        ready += edge.node->GetLatency(LatencySetting::kLlvm);
      }
      inputs_ready_time = std::max(inputs_ready_time, ready);
    }
    op.inputs_ready_time = inputs_ready_time;
    op.stuck = std::max(0, op.issue_time - inputs_ready_time);
    if (op.stuck == 0) {
      continue;
    }

    // The waits up to and including the load's own that moved the clock past
    // its inputs, and by how much. The clock only moves forward, so these
    // add up to op.stuck.
    for (const WalkStep &earlier : steps) {
      const int moved_from =
          std::max(earlier.issue_wait_begin_time, inputs_ready_time);
      if (earlier.issue_time > moved_from) {
        auto wait_it = wait_of_node.find(earlier.node);
        op.stuck_waits.push_back(MemoryVizStuckWait{
            wait_it == wait_of_node.end() ? -1 : wait_it->second,
            earlier.issue_time - moved_from});
      }
      if (earlier.node == step.node) {
        break;
      }
    }
  }

  // This schedule's tiers. The clock only moves at a wait that stalls, so
  // each such wait starts a tier. A node whose own wait stalls is in the tier
  // that wait starts.
  result.tiers.push_back(MemoryVizTier{0, -1});
  for (const WalkStep &step : steps) {
    if (step.issue_time > step.issue_wait_begin_time) {
      auto wait_it = wait_of_node.find(step.node);
      result.tiers.push_back(MemoryVizTier{
          step.issue_time,
          wait_it == wait_of_node.end() ? -1 : wait_it->second});
    }
    auto op_it = op_of_node.find(step.node);
    if (op_it != op_of_node.end()) {
      result.ops[op_it->second].tier = static_cast<int>(result.tiers.size()) - 1;
    }
  }

  // The ideal tiers: how many rounds of waiting must come before each node,
  // in any order. An edge adds a round where the ideal's earliest start adds
  // the predecessor's latency (MemoryGraph::ScoreIdeal): it waits for data,
  // or it goes into a barrier, and the predecessor takes time.
  DenseMap<const MemoryNode *, int> ideal_tier_of;
  DenseMap<const MemoryNode *, int> ideal_start_of;
  for (const MemoryNode *node : memory_graph->GetTopoOrder()) {
    const bool is_barrier = node->GetKind() == MemoryNodeKind::kBarrier;
    int ideal_tier = 0;
    int ideal_start = 0;
    for (const MemoryEdge &edge : node->Predecessors()) {
      const int latency = edge.node->GetLatency(LatencySetting::kLlvm);
      const bool waits = (edge.waits_for_data || is_barrier) && latency > 0;
      ideal_tier =
          std::max(ideal_tier, ideal_tier_of.lookup(edge.node) + (waits ? 1 : 0));
      ideal_start = std::max(ideal_start, ideal_start_of.lookup(edge.node) +
                                              (waits ? latency : 0));
    }
    ideal_tier_of[node] = ideal_tier;
    ideal_start_of[node] = ideal_start;
    auto op_it = op_of_node.find(node);
    if (op_it != op_of_node.end()) {
      result.ops[op_it->second].ideal_tier = ideal_tier;
      result.ops[op_it->second].ideal_start = ideal_start;
    }
  }

  // Each op's inputs, for the tier view: the ops it or a node before it
  // waits on, and the barriers before it that wait on something. A barrier
  // waits on every op before it that takes time.
  auto add_waited_on = [&](const MemoryNode &node, std::set<int> &inputs) {
    if (node.GetKind() == MemoryNodeKind::kBarrier) {
      for (const MemoryEdge &edge : node.Predecessors()) {
        auto op_it = op_of_node.find(edge.node);
        if (op_it != op_of_node.end() &&
            edge.node->GetLatency(LatencySetting::kLlvm) > 0) {
          inputs.insert(op_it->second);
        }
      }
      return;
    }
    for (const MemoryNode *load : node.GetLoadsUsed()) {
      auto op_it = op_of_node.find(load);
      if (op_it != op_of_node.end()) {
        inputs.insert(op_it->second);
      }
    }
  };
  std::vector<std::set<int>> all_inputs(result.ops.size());
  for (const auto &[node, op_index] : op_of_node) {
    std::set<int> &inputs = all_inputs[op_index];
    add_waited_on(*node, inputs);
    for (const MemoryEdge &edge : node->Predecessors()) {
      if (edge.node->GetKind() != MemoryNodeKind::kBarrier) {
        add_waited_on(*edge.node, inputs);
        continue;
      }
      std::set<int> barrier_inputs;
      add_waited_on(*edge.node, barrier_inputs);
      if (!barrier_inputs.empty()) {
        inputs.insert(barrier_inputs.begin(), barrier_inputs.end());
        inputs.insert(op_of_node.lookup(edge.node));
      }
    }
  }
  // Every predecessor is an edge of the memory graph, so the inputs are
  // transitive: an input of an input is an input. Keep only the nearest.
  for (int op_index = 0; op_index < static_cast<int>(all_inputs.size());
       ++op_index) {
    const std::set<int> &inputs = all_inputs[op_index];
    for (int input : inputs) {
      const bool through_another = any_of(inputs, [&](int other) {
        return other != input && all_inputs[other].count(input) != 0;
      });
      if (!through_another) {
        result.ops[op_index].inputs.push_back(input);
      }
    }
  }

  // Registers holding load results. A candidate is a virtual register that a
  // load writes and that's live somewhere in the region; it counts at a row
  // when it's live just after that row's instruction.
  const SlotIndex region_begin = lis_->getInstructionIndex(*row_instrs.front());
  const SlotIndex region_end =
      lis_->getInstructionIndex(*row_instrs.back()).getDeadSlot();
  std::vector<LoadResultRegister> load_results;
  for (int i = 0, num_virt_regs = static_cast<int>(mri.getNumVirtRegs());
       i < num_virt_regs; ++i) {
    const Register reg = Register::index2VirtReg(i);
    if (!lis_->hasInterval(reg) || !IsWrittenByLoad(reg, mri)) {
      continue;
    }
    const LiveInterval &interval = lis_->getInterval(reg);
    if (!interval.overlaps(region_begin, region_end)) {
      continue;
    }
    const TargetRegisterClass *reg_class = mri.getRegClass(reg);
    const bool is_vgpr = SIRegisterInfo::isVGPRClass(reg_class);
    if (!is_vgpr && !SIRegisterInfo::isSGPRClass(reg_class)) {
      continue;
    }

    LoadResultRegister candidate;
    candidate.interval = &interval;
    candidate.width =
        std::max(1, static_cast<int>(tri->getRegSizeInBits(*reg_class)) / 32);
    candidate.is_vgpr = is_vgpr;
    auto op_it = op_of_result_register.find(reg);
    if (op_it != op_of_result_register.end()) {
      const MemoryVizOp &op = result.ops[op_it->second];
      candidate.in_flight_begin_row = op.row;
      candidate.in_flight_end_row =
          op.pop_rows.empty()
              ? num_rows
              : *std::min_element(op.pop_rows.begin(), op.pop_rows.end());
    }
    load_results.push_back(candidate);
  }
  for (int row = 0; row < num_rows; ++row) {
    const SlotIndex after =
        lis_->getInstructionIndex(*row_instrs[row]).getRegSlot();
    MemoryVizRow &viz_row = result.rows[row];
    for (const LoadResultRegister &candidate : load_results) {
      if (!candidate.interval->liveAt(after)) {
        continue;
      }
      const bool in_flight = row >= candidate.in_flight_begin_row &&
                             row < candidate.in_flight_end_row;
      if (candidate.is_vgpr) {
        viz_row.vgpr_load += candidate.width;
        viz_row.vgpr_load_in_flight += in_flight ? candidate.width : 0;
      } else {
        viz_row.sgpr_load += candidate.width;
        viz_row.sgpr_load_in_flight += in_flight ? candidate.width : 0;
      }
    }
  }

  regions_by_block_[region.GetBlock()->getNumber()].push_back(
      std::move(result));
}

void MemoryViz::WriteJson(StringRef scheduler_name) const {
  if (regions_by_block_.empty()) {
    return;
  }

  const GCNSubtarget &st = mf_->getSubtarget<GCNSubtarget>();
  const int occupancy =
      static_cast<int>(mf_->getInfo<SIMachineFunctionInfo>()->getOccupancy());

  // Each block's frequency, and the function's total gain: the sum of every
  // region's gain weighted by its block's frequency. Each region's share is
  // its weighted gain over that total.
  std::map<int, double> frequency_of_block;
  for (const MachineBasicBlock &block : *mf_) {
    frequency_of_block[block.getNumber()] =
        block_frequency_info_.getBlockFreqRelativeToEntryBlock(&block);
  }
  double total_weighted_gain = 0;
  for (const auto &[block_number, regions] : regions_by_block_) {
    for (const MemoryVizRegion &region : regions) {
      total_weighted_gain += frequency_of_block[block_number] * region.gain;
    }
  }

  const char *kDirectory = "memory_viz";
  if (std::error_code ec = sys::fs::create_directories(kDirectory)) {
    report_fatal_error(Twine("MemoryViz: cannot create '") + kDirectory +
                       "': " + ec.message());
  }
  const std::string path = std::string(kDirectory) + "/" +
                           SanitizeForFilename(mf_->getName()) + ".json";
  std::error_code ec;
  raw_fd_ostream os(path, ec, sys::fs::OF_Text);
  if (ec) {
    report_fatal_error(Twine("MemoryViz: cannot open '") + path +
                       "': " + ec.message());
  }

  json::OStream out(os);
  out.object([&] {
    out.attribute("function", mf_->getName());
    out.attribute("scheduler", scheduler_name);
    out.attribute("occupancy", occupancy);
    out.attribute("vgpr_budget",
                  static_cast<int64_t>(st.getMaxNumVGPRs(occupancy)));
    out.attribute("sgpr_budget", static_cast<int64_t>(st.getMaxNumSGPRs(
                                     occupancy, /*Addressable=*/true)));
    out.attribute("total_weighted_gain", total_weighted_gain);

    out.attributeArray("blocks", [&] {
      for (const MachineBasicBlock &block : *mf_) {
        const double frequency = frequency_of_block[block.getNumber()];
        out.object([&] {
          out.attribute("number", block.getNumber());
          out.attribute("name", block.getName());
          out.attributeArray("succs", [&] {
            for (const MachineBasicBlock *successor : block.successors()) {
              out.value(successor->getNumber());
            }
          });
          out.attribute("frequency", frequency);
          out.attribute("loop_depth", static_cast<int64_t>(
                                          loop_info_->getLoopDepth(&block)));

          out.attributeArray("regions", [&] {
            auto it = regions_by_block_.find(block.getNumber());
            if (it == regions_by_block_.end()) {
              return;
            }
            for (const MemoryVizRegion &region : it->second) {
              const double weighted_gain = frequency * region.gain;
              out.object([&] {
                out.attribute("index", region.index);
                out.attribute("scheduled_memory_time",
                              region.scheduled_memory_time);
                out.attribute("ideal_memory_time", region.ideal_memory_time);
                out.attribute("schedule_length", region.schedule_length);
                out.attribute("schedule_length_floor",
                              region.schedule_length_floor);
                out.attribute("gain", region.gain);
                out.attribute("weighted_gain", weighted_gain);
                out.attribute("share", total_weighted_gain > 0
                                           ? weighted_gain / total_weighted_gain
                                           : 0.0);

                out.attributeArray("rows", [&] {
                  for (const MemoryVizRow &row : region.rows) {
                    out.object([&] {
                      out.attribute("text", row.text);
                      out.attribute("memory_time", row.memory_time);
                      out.attribute("schedule_cycle", row.schedule_cycle);
                      out.attribute("vgpr", row.vgpr);
                      out.attribute("sgpr", row.sgpr);
                      out.attribute("vgpr_load", row.vgpr_load);
                      out.attribute("vgpr_load_in_flight",
                                    row.vgpr_load_in_flight);
                      out.attribute("sgpr_load", row.sgpr_load);
                      out.attribute("sgpr_load_in_flight",
                                    row.sgpr_load_in_flight);
                      out.attribute("op", row.op);
                      out.attribute("wait", row.wait);
                      out.attributeArray("defs", [&] {
                        for (int reg : row.defs) {
                          out.value(reg);
                        }
                      });
                      out.attributeArray("uses", [&] {
                        for (int reg : row.uses) {
                          out.value(reg);
                        }
                      });
                      out.attributeArray("predecessors", [&] {
                        for (const MemoryVizPredecessor &predecessor :
                             row.predecessors) {
                          out.object([&] {
                            out.attribute("row", predecessor.row);
                            out.attribute("kind", predecessor.kind);
                          });
                        }
                      });
                    });
                  }
                });

                out.attributeArray("ops", [&] {
                  for (const MemoryVizOp &op : region.ops) {
                    out.object([&] {
                      out.attribute("row", op.row);
                      out.attribute("label", op.label);
                      out.attribute("type", op.type);
                      out.attribute("read", op.read);
                      out.attribute("vmcnt", op.vmcnt);
                      out.attribute("lgkmcnt", op.lgkmcnt);
                      out.attribute("latency", op.latency);
                      out.attribute("issue_time", op.issue_time);
                      out.attribute("data_time", op.data_time);
                      out.attributeArray("pop_rows", [&] {
                        for (int row : op.pop_rows) {
                          out.value(row);
                        }
                      });
                      out.attribute("def", op.def);
                      out.attribute("inputs_ready_time", op.inputs_ready_time);
                      out.attribute("stuck", op.stuck);
                      out.attributeArray("stuck_waits", [&] {
                        for (const MemoryVizStuckWait &stuck_wait :
                             op.stuck_waits) {
                          out.object([&] {
                            out.attribute("wait", stuck_wait.wait);
                            out.attribute("cycles", stuck_wait.cycles);
                          });
                        }
                      });
                      out.attribute("tier", op.tier);
                      out.attribute("ideal_tier", op.ideal_tier);
                      out.attribute("ideal_start", op.ideal_start);
                      out.attributeArray("inputs", [&] {
                        for (int input : op.inputs) {
                          out.value(input);
                        }
                      });
                    });
                  }
                });

                out.attributeArray("waits", [&] {
                  for (const MemoryVizWait &wait : region.waits) {
                    out.object([&] {
                      out.attribute("row", wait.row);
                      out.attribute("begin_time", wait.begin_time);
                      out.attribute("end_time", wait.end_time);
                      out.attributeArray("waited_for", [&] {
                        for (int op : wait.waited_for) {
                          out.value(op);
                        }
                      });
                      out.attributeArray("popped", [&] {
                        for (int op : wait.popped) {
                          out.value(op);
                        }
                      });
                      out.attribute("drain", wait.drain);
                      out.attribute("barrier", wait.barrier);
                      out.attribute("cautious", wait.cautious);
                    });
                  }
                });

                out.attributeArray("tiers", [&] {
                  for (const MemoryVizTier &tier : region.tiers) {
                    out.object([&] {
                      out.attribute("clock", tier.clock);
                      out.attribute("wait", tier.wait);
                    });
                  }
                });

                out.attributeArray("live_out", [&] {
                  for (int reg : region.live_out) {
                    out.value(reg);
                  }
                });
              });
            }
          });
        });
      }
    });
  });
  os << "\n";
}
