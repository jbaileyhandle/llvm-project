//===- MemoryGraph.cpp - Memory-op view of a region for MLP ---------------===//
//
// See MemoryGraph.h and llvm/docs/AMDGPUMemoryGraphDesign.md.
//
//===----------------------------------------------------------------------===//

#include "MemoryGraph.h"
#include "AMDGPU.h"
#include "GCNSubtarget.h"
#include "HwPipeClass.h"
#include "MlpTracker.h"
#include "SIInstrInfo.h"
#include "SIMachineFunctionInfo.h"
#include "ScheduleGraph.h"
#include "Utils/AMDGPUBaseInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"

namespace llvm::hierarchical_scheduler {

// Whether a FLAT op may touch global memory, and so counts in vmcnt.
// Local copy of SIInsertWaitcnts::mayAccessVMEMThroughFlat: an op with no
// memory info may touch anything, and only info saying LDS rules it out.
static bool MayTouchGlobalMemory(const MachineInstr &mi) {
  if (mi.memoperands_empty()) {
    return true;
  }
  for (const MachineMemOperand *memop : mi.memoperands()) {
    if (memop->getAddrSpace() != AMDGPUAS::LOCAL_ADDRESS) {
      return true;
    }
  }
  return false;
}

// Whether a FLAT op may touch LDS, and so counts in lgkmcnt.
// Local copy of SIInsertWaitcnts::mayAccessLDSThroughFlat.
static bool MayTouchLds(const MachineInstr &mi, const GCNSubtarget &st) {
  // Only FLAT ops whose instruction definition uses lgkmcnt can reach LDS.
  // global_* and scratch_* don't, so they never count in lgkmcnt.
  if (!SIInstrInfo::usesLGKM_CNT(mi)) {
    return false;
  }
  // In tgsplit mode a FLAT op can't reach LDS.
  if (st.isTgSplitEnabled()) {
    return false;
  }
  // No memory info: it may touch anything, LDS included.
  if (mi.memoperands_empty()) {
    return true;
  }
  // Info saying LDS, or the generic flat address space (which includes LDS).
  for (const MachineMemOperand *memop : mi.memoperands()) {
    unsigned address_space = memop->getAddrSpace();
    if (address_space == AMDGPUAS::LOCAL_ADDRESS ||
        address_space == AMDGPUAS::FLAT_ADDRESS) {
      return true;
    }
  }
  return false;
}

// Read or write. A read is an op whose result later code can wait for, which
// means it writes a register. This is the compiler's own test:
// SIInsertWaitcnts::updateByEvent records a counted op against every register
// it defines, implicit ones included, and only those registers are ever
// waited on. So an atomic that returns a value is a read and one that doesn't
// is a write, with no special case.
static MemAccess AccessOf(const MachineInstr &mi) {
  for (const MachineOperand &operand : mi.operands()) {
    if (operand.isReg() && operand.isDef()) {
      return MemAccess::kRead;
    }
  }
  return MemAccess::kWrite;
}

// Mirrors SIInsertWaitcnts::updateEventWaitcntAfter, which decides which
// counters each instruction counts in: the same families, in the same order,
// with the same conditions. The comments below note where this differs.
// Branches of that function not handled here (s_sendmsg, exports, calls) are
// listed in the implementation plan's "Left out" table or don't occur in
// compute kernels.
std::optional<MemOpInfo> ClassifyMemOp(const MachineInstr &mi,
                                       const GCNSubtarget &st) {
  const SIInstrInfo &tii = *st.getInstrInfo();

  // DS: LDS and GDS ops (ds_read, ds_write, ds_add, ...). They count only in
  // lgkmcnt. GDS is told apart as the compiler does: an always-GDS opcode, or
  // the gds bit set. A DS op that doesn't use lgkmcnt matches none of the
  // compiler's other branches either, so it isn't counted.
  if (SIInstrInfo::isDS(mi)) {
    if (!SIInstrInfo::usesLGKM_CNT(mi)) {
      return std::nullopt;
    }
    const bool is_gds = tii.isAlwaysGDS(mi.getOpcode()) ||
                        tii.hasModifiersSet(mi, AMDGPU::OpName::gds);
    return MemOpInfo{AccessOf(mi), is_gds ? MemType::kGds : MemType::kLds,
                     /*counts_in_vmcnt=*/false, /*counts_in_lgkmcnt=*/true};
  }

  // FLAT encoding. The counters come from the two helpers above, exactly as
  // the compiler decides them. isFLAT is true for two kinds of instruction:
  //   - global_* and scratch_*: they share the FLAT instruction encoding, but
  //     can only touch global memory or scratch, never LDS, so the helpers give
  //     vmcnt only, like a buffer load. We label them vector ops;
  //     SIInsertWaitcnts::updateVMCntOnly also groups them with vector memory.
  //   - flat_*: may touch global memory or LDS, depending on its memory info.
  //     One that counts in both counters turns on cautious mode (design doc
  //     section 5.2).
  if (SIInstrInfo::isFLAT(mi)) {
    const MemType type =
        SIInstrInfo::isFLATGlobal(mi) || SIInstrInfo::isFLATScratch(mi)
            ? MemType::kVector
            : MemType::kFlat;
    return MemOpInfo{AccessOf(mi), type, MayTouchGlobalMemory(mi),
                     MayTouchLds(mi, st)};
  }

  // Vector memory other than FLAT: buffer and image ops (MUBUF, MTBUF, MIMG).
  // They count only in vmcnt. Buffer cache invalidates are excluded, as in the
  // compiler: they aren't loads or stores, and it doesn't count them.
  if (SIInstrInfo::isVMEM(mi) &&
      !AMDGPU::getMUBUFIsBufferInv(mi.getOpcode())) {
    return MemOpInfo{AccessOf(mi), MemType::kVector,
                     /*counts_in_vmcnt=*/true, /*counts_in_lgkmcnt=*/false};
  }

  // Scalar memory (s_load, s_buffer_load, ...). Counts only in lgkmcnt. This
  // family also includes s_memtime, a clock read that isn't a memory access;
  // it writes a register that later code waits on, so it's a read, as the
  // compiler treats it.
  if (SIInstrInfo::isSMRD(mi)) {
    return MemOpInfo{AccessOf(mi), MemType::kScalar,
                     /*counts_in_vmcnt=*/false, /*counts_in_lgkmcnt=*/true};
  }

  return std::nullopt;
}

bool TrackedReadTypes::Tracks(const MemoryNode &node) const {
  // Writes, users, barriers and the sink are never tracked. Only reads reach
  // the switch, so their MemOpInfo is set.
  if (!node.IsRead()) {
    return false;
  }

  switch (node.GetMemOpInfo()->type) {
    case MemType::kVector: {
      return vector;
    }
    case MemType::kFlat: {
      return flat;
    }
    case MemType::kScalar: {
      return scalar;
    }
    case MemType::kLds: {
      return lds;
    }
    case MemType::kGds: {
      return gds;
    }
  }
  llvm_unreachable("TrackedReadTypes::Tracks: unknown MemType");
}

MemoryNode &MemoryGraph::AddNode(MemoryNodeKind kind,
                                 const ScheduleNode *original_node,
                                 std::optional<MemOpInfo> mem_op_info,
                                 int llvm_latency) {
  if ((kind == MemoryNodeKind::kMemOp) != mem_op_info.has_value()) {
    report_fatal_error("MemoryGraph::AddNode: a node has MemOpInfo exactly "
                       "when its kind is kMemOp");
  }
  // Nodes point at each other, so nodes_ must never move. It's reserved
  // before the first node is added; growing it here would break every pointer.
  if (nodes_.size() == nodes_.capacity()) {
    report_fatal_error("MemoryGraph::AddNode would reallocate nodes_; reserve "
                       "more space before adding nodes");
  }
  nodes_.emplace_back();
  MemoryNode &node = nodes_.back();
  node.id_ = static_cast<int>(nodes_.size()) - 1;
  node.kind_ = kind;
  node.original_node_ = original_node;
  node.mem_op_info_ = mem_op_info;
  node.llvm_latency_ = mem_op_info ? llvm_latency : 0;
  if (original_node != nullptr) {
    if (original_node->IsRealInstruction()) {
      node.original_instr_ = original_node->GetSUnit()->getInstr();
    }
    original_node_to_memory_node_[original_node] = &node;
  }
  return node;
}

void MemoryGraph::AddEdge(MemoryNode &from, MemoryNode &to,
                          bool waits_for_data) {
  from.successors_.push_back(MemoryEdge{&to, waits_for_data});
  to.predecessors_.push_back(MemoryEdge{&from, waits_for_data});
}

std::unique_ptr<MemoryGraph> MemoryGraph::BuildWithRoleClassifier(
    ScheduleGraph &original,
    function_ref<NodeRole(const ScheduleNode &)> role_classifier) {
  PrepareOriginalGraph(original);
  std::unique_ptr<MemoryGraph> graph(new MemoryGraph());
  // Each memory node stands for at most one original node, plus the sink.
  graph->nodes_.reserve(original.GetTopoOrder().size() + 1);
  graph->AddMemOpAndBarrierNodes(original, role_classifier);
  graph->AddUsers(original);
  graph->AddEdges(original);
  graph->AddSink();
  graph->ComputeTopologicalOrder();
  return graph;
}

void MemoryGraph::PrepareOriginalGraph(ScheduleGraph &original) {
  // Reachability relies on the topological order, so that comes first.
  if (!original.IsTopoSorted()) {
    original.ValidateAndComputeTopologicalOrder();
  }
  if (!original.HasReachability()) {
    original.ComputeTransitiveReductionAndReachability();
  }
}

void MemoryGraph::AddMemOpAndBarrierNodes(
    const ScheduleGraph &original,
    function_ref<NodeRole(const ScheduleNode &)> role_classifier) {
  for (const ScheduleNode *node : original.GetTopoOrder()) {
    const NodeRole role = role_classifier(*node);
    if (role.mem_op) {
      AddNode(MemoryNodeKind::kMemOp, node, role.mem_op, role.llvm_latency);
    } else if (role.is_barrier) {
      AddNode(MemoryNodeKind::kBarrier, node);
    }
  }
}

void MemoryGraph::AddUsers(const ScheduleGraph &original) {
  SmallVector<MemoryNode *> loads;
  for (MemoryNode &node : nodes_) {
    if (node.IsRead()) {
      loads.push_back(&node);
    }
  }

  // A user reads a load's data through a kData edge in the original graph.
  // The original exit node isn't one: it stands for code after the region,
  // and the sink covers the region's end.
  const ScheduleNode *original_exit = original.GetExitNode();
  for (MemoryNode *load : loads) {
    for (const ScheduleEdge &edge : load->original_node_->Successors()) {
      if (!edge.IsDataEdge() || edge.node_ == original_exit) {
        continue;
      }
      auto it = original_node_to_memory_node_.find(edge.node_);
      MemoryNode &user = it != original_node_to_memory_node_.end()
                             ? *it->second
                             : AddNode(MemoryNodeKind::kUserOnly, edge.node_);
      // A user can read several registers from one load; list the load once.
      if (!is_contained(user.loads_used_, load)) {
        user.loads_used_.push_back(load);
      }
    }
  }
}

void MemoryGraph::AddEdges(const ScheduleGraph &original) {
  // The edge waits for data when the later node reads the earlier load's data
  // (design doc section 3).
  for (MemoryNode &from : nodes_) {
    for (MemoryNode &to : nodes_) {
      if (&from == &to ||
          !original.IsReachableInDag(from.original_node_, to.original_node_)) {
        continue;
      }
      const bool waits_for_data =
          from.IsRead() && is_contained(to.loads_used_, &from);
      AddEdge(from, to, waits_for_data);
    }
  }
}

void MemoryGraph::AddSink() {
  // The sink is an ordinary user of every load, so every read has
  // something that waits on it, and it comes after every node.
  MemoryNode &sink = AddNode(MemoryNodeKind::kSink, nullptr);
  sink_ = &sink;
  for (MemoryNode &node : nodes_) {
    if (&node == &sink) {
      continue;
    }
    if (node.IsRead()) {
      sink.loads_used_.push_back(&node);
    }
    AddEdge(node, sink, /*waits_for_data=*/node.IsRead());
  }
}

void MemoryGraph::ComputeTopologicalOrder() {
  // Every edge A -> B exists because B is reachable from A in the original
  // graph, so ordering the nodes by their original nodes' topological order is
  // a valid order here too. The sink has no original node and goes last.
  topo_order_.clear();
  for (MemoryNode &node : nodes_) {
    topo_order_.push_back(&node);
  }
  llvm::sort(topo_order_, [](const MemoryNode *a, const MemoryNode *b) {
    if (a->original_node_ == nullptr || b->original_node_ == nullptr) {
      // Only the sink has no original node: a comes first iff b is the sink.
      return b->original_node_ == nullptr && a->original_node_ != nullptr;
    }
    return a->original_node_->GetTopoIndex() <
           b->original_node_->GetTopoIndex();
  });
  for (int index = 0; index < static_cast<int>(topo_order_.size()); ++index) {
    topo_order_[index]->topo_index_ = index;
  }
}

// An op's latency (design doc section 3). misched.txt's load-latency settings
// (vmem_load_latency, smem_load_latency, lds_load_latency, and per-kernel
// kernel_latencies) are applied by LLVM only to dependence edges, in
// GCNSubtarget::adjustSchedDependency, so SUnit::Latency doesn't see them.
// This is a local copy of how that function picks a setting: by the op's
// family, a per-kernel value over the global one, and only for loads
// (mayLoad). So, as in the compiler, an op that isn't a load, such as the
// s_memtime clock read, keeps its timing-model latency. Without a setting, the
// latency is SUnit::Latency, which reads the active timing model.
static int LlvmLatencyOf(const MachineInstr &mi, const SUnit &su) {
  if (mi.mayLoad()) {
    const MachineInstrSchedulerConfig::GlobalSettings &settings =
        MachineInstrSchedulerConfig::GetConfig().GetGlobalSettings();
    const MachineInstrSchedulerConfig::KernelLatencies *kernel =
        mi.getMF()->getInfo<SIMachineFunctionInfo>()->getKernelLatencyOverrides();
    std::optional<int> setting;
    if (SIInstrInfo::isVMEM(mi) || SIInstrInfo::isFLAT(mi)) {
      setting = kernel && kernel->vmem_load_latency ? kernel->vmem_load_latency
                                                    : settings.vmem_load_latency;
    } else if (SIInstrInfo::isSMRD(mi)) {
      setting = kernel && kernel->smem_load_latency ? kernel->smem_load_latency
                                                    : settings.smem_load_latency;
    } else if (SIInstrInfo::isDS(mi)) {
      setting = kernel && kernel->lds_load_latency ? kernel->lds_load_latency
                                                   : settings.lds_load_latency;
    }
    if (setting) {
      return *setting;
    }
  }
  return static_cast<int>(su.Latency);
}

std::unique_ptr<MemoryGraph> MemoryGraph::Build(ScheduleGraph &original,
                                                const GCNSubtarget &st) {
  return BuildWithRoleClassifier(original, [&](const ScheduleNode &node) {
    NodeRole role;
    // The entry and exit nodes and subgraph proxies have no instruction, so
    // they play no role.
    if (!node.IsRealInstruction() || node.GetSUnit()->getInstr() == nullptr) {
      return role;
    }
    const MachineInstr &mi = *node.GetSUnit()->getInstr();
    const std::optional<MemOpInfo> info = ClassifyMemOp(mi, st);
    if (info) {
      role.mem_op = info;
      role.llvm_latency = LlvmLatencyOf(mi, *node.GetSUnit());
    } else if (IsWaveRendezvous(mi)) {
      role.is_barrier = true;
    }
    return role;
  });
}

const MemoryNode *
MemoryGraph::GetMemoryNode(const ScheduleNode *original_node) const {
  auto it = original_node_to_memory_node_.find(original_node);
  return it == original_node_to_memory_node_.end() ? nullptr : it->second;
}

std::vector<const MemoryNode *>
MemoryGraph::OrderFromSchedule(ArrayRef<const ScheduleNode *> schedule) const {
  std::vector<const MemoryNode *> order;
  for (const ScheduleNode *original_node : schedule) {
    if (const MemoryNode *node = GetMemoryNode(original_node)) {
      order.push_back(node);
    }
  }
  // The sink stands for the end of the region, so it comes last.
  order.push_back(sink_);
  return order;
}

// The MLP of a set of memory op flights (MLP doc, section 2.3):
//
//   summed_read_flight_time: every flight's length, added up. Time when two
//     flights overlap counts twice.
//   any_read_in_flight_time: the time during which at least one flight is in
//     flight. Time when two flights overlap counts once.
//   mlp: summed / any, the average number of flights in flight while any is.
//
// Example: flights [0, 100), [50, 150) and [200, 300) add up to 300. At least
// one is in flight during [0, 150) and [200, 300), which is 150 + 100 = 250,
// so mlp is 300 / 250 = 1.2.
MlpMeasure MeasureMlp(std::vector<MemOpFlight> flights) {
  MlpMeasure measure;

  // Step 1: every flight's length, added up.
  for (const MemOpFlight &flight : flights) {
    measure.summed_read_flight_time += flight.stop_time - flight.start_time;
  }

  // Step 2: the time during which at least one flight is in flight.
  //
  // Sort the flights by start time, then sweep through them while keeping one
  // "stretch": a span of time in which some flight is always in flight. Each
  // flight either overlaps the current stretch, or starts after it stops.
  llvm::sort(flights, [](const MemOpFlight &a, const MemOpFlight &b) {
    return a.start_time < b.start_time;
  });

  bool have_stretch = false;  // false until the first flight
  int stretch_start_time = 0;
  int stretch_stop_time = 0;

  for (const MemOpFlight &flight : flights) {
    // The flight starts before the stretch stops, so it overlaps the stretch.
    // The stretch now lasts until the later of the two stop times.
    if (have_stretch && flight.start_time <= stretch_stop_time) {
      stretch_stop_time = std::max(stretch_stop_time, flight.stop_time);
      continue;
    }

    // The flight starts after the stretch stops, so between them nothing is
    // in flight. The stretch is finished: count its length, then start a new
    // stretch with this flight.
    if (have_stretch) {
      measure.any_read_in_flight_time += stretch_stop_time - stretch_start_time;
    }
    have_stretch = true;
    stretch_start_time = flight.start_time;
    stretch_stop_time = flight.stop_time;
  }

  // No later flight finishes the last stretch, so count it here.
  if (have_stretch) {
    measure.any_read_in_flight_time += stretch_stop_time - stretch_start_time;
  }

  // Step 3: the ratio. With nothing in flight there's nothing to overlap, so
  // mlp stays 0 rather than dividing by zero.
  if (measure.any_read_in_flight_time > 0) {
    measure.mlp = static_cast<float>(measure.summed_read_flight_time) /
                  measure.any_read_in_flight_time;
  }

  return measure;
}

IdealScore
MemoryGraph::ScoreIdeal(LatencySetting setting,
                        const TrackedReadTypes &tracked_read_types) const {
  // The earliest time each node can run, indexed by topo index. Filled in
  // topological order, so a node's predecessors are always done before it.
  std::vector<int> earliest_start_by_topo_index(topo_order_.size(), 0);

  // Each tracked read's flight: from its earliest start until its data is
  // back.
  std::vector<MemOpFlight> tracked_read_flights;

  for (const MemoryNode *node : topo_order_) {
    // A barrier waits for every memory op issued before it to finish, reads
    // and writes: the s_waitcnt 0 before s_barrier, which the walk's drain
    // models. Here that means the ops with an edge into the barrier, which
    // every order must issue before it. An op with no ordering to the
    // barrier, such as an invariant load, could issue after it instead, so
    // the barrier doesn't wait for it.
    const bool is_barrier = node->GetKind() == MemoryNodeKind::kBarrier;

    // A node can start once every predecessor allows it.
    int earliest_start = 0;
    for (const MemoryEdge &edge : node->Predecessors()) {
      const MemoryNode &predecessor = *edge.node;
      const int predecessor_start =
          earliest_start_by_topo_index[predecessor.GetTopoIndex()];

      // An ordering-only edge into a node that isn't a barrier only keeps the
      // node from starting before the predecessor. An edge that waits for
      // data, or any edge into a barrier, also waits for the predecessor to
      // finish. For a barrier, every predecessor's latency is added without
      // checking that it's a memory op: one that isn't, like a user, has
      // latency 0, so it adds nothing.
      int allowed_from = predecessor_start;
      if (edge.waits_for_data || is_barrier) {
        allowed_from += predecessor.GetLatency(setting);
      }

      earliest_start = std::max(earliest_start, allowed_from);
    }
    earliest_start_by_topo_index[node->GetTopoIndex()] = earliest_start;

    if (tracked_read_types.Tracks(*node)) {
      tracked_read_flights.push_back(MemOpFlight{
          earliest_start, earliest_start + node->GetLatency(setting)});
    }
  }

  IdealScore score;

  // The sink waits for every load, tracked or not, so the earliest it can
  // start is when the last read's data is back.
  score.elapsed_time = earliest_start_by_topo_index[sink_->GetTopoIndex()];

  score.latency_based_mlp = MeasureMlp(std::move(tracked_read_flights));
  return score;
}

// Fails hard unless `order` holds every node of `graph` once, each after its
// predecessors. The sink comes after every other node, so this also puts it
// last.
static void ValidateOrder(const MemoryGraph &graph,
                          ArrayRef<const MemoryNode *> order) {
  const ArrayRef<MemoryNode *> topo_order = graph.GetTopoOrder();
  if (order.size() != topo_order.size()) {
    report_fatal_error("MemoryGraph::ScoreOrder: the order must hold every "
                       "node once");
  }

  // Whether each node has come up in the order yet, indexed by topo index.
  std::vector<bool> placed_by_topo_index(topo_order.size(), false);

  for (const MemoryNode *node : order) {
    // The node must be one of this graph's, and come up only once. With the
    // size check above, that means every node comes up.
    const int topo_index = node->GetTopoIndex();
    if (topo_index < 0 || topo_index >= static_cast<int>(topo_order.size()) ||
        topo_order[topo_index] != node) {
      report_fatal_error("MemoryGraph::ScoreOrder: the order holds a node "
                         "from another graph");
    }
    if (placed_by_topo_index[topo_index]) {
      report_fatal_error("MemoryGraph::ScoreOrder: the order holds a node "
                         "twice");
    }

    // Every predecessor must come before it.
    for (const MemoryEdge &edge : node->Predecessors()) {
      if (!placed_by_topo_index[edge.node->GetTopoIndex()]) {
        report_fatal_error("MemoryGraph::ScoreOrder: the order puts a node "
                           "before one of its predecessors");
      }
    }

    placed_by_topo_index[topo_index] = true;
  }
}

OrderScore
MemoryGraph::ScoreOrder(ArrayRef<const MemoryNode *> order,
                        LatencySetting setting,
                        const TrackedReadTypes &tracked_read_types,
                        std::vector<WalkStep> *steps) const {
  // The tracker doesn't check that each node comes after its predecessors,
  // so the whole order is checked here first.
  ValidateOrder(*this, order);

  MlpTracker tracker(this, setting, tracked_read_types);
  for (const MemoryNode *node : order) {
    tracker.Schedule(node);
  }

  if (steps != nullptr) {
    const ArrayRef<WalkStep> tracker_steps = tracker.GetSteps();
    steps->assign(tracker_steps.begin(), tracker_steps.end());
  }
  return tracker.GetScore();
}

OrderScore
MemoryGraph::ScoreSchedule(ArrayRef<const ScheduleNode *> schedule,
                           LatencySetting setting,
                           const TrackedReadTypes &tracked_read_types,
                           std::vector<WalkStep> *steps) const {
  return ScoreOrder(OrderFromSchedule(schedule), setting, tracked_read_types,
                    steps);
}

} // namespace llvm::hierarchical_scheduler
