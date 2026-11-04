//===- MemoryGraph.cpp - Memory-op view of a region for MLP ---------------===//
//
// See MemoryGraph.h and llvm/docs/AMDGPUMemoryGraphDesign.md.
//
//===----------------------------------------------------------------------===//

#include "MemoryGraph.h"
#include "AMDGPU.h"
#include "GCNSubtarget.h"
#include "HwPipeClass.h"
#include "SIInstrInfo.h"
#include "SIMachineFunctionInfo.h"
#include "ScheduleGraph.h"
#include "Utils/AMDGPUBaseInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Whether a FLAT op may touch global memory, and so counts in vmcnt.
// Local copy of SIInsertWaitcnts::mayAccessVMEMThroughFlat: an op with no
// memory info may touch anything, and only info saying LDS rules it out.
bool MayTouchGlobalMemory(const MachineInstr &mi) {
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
bool MayTouchLds(const MachineInstr &mi, const GCNSubtarget &st) {
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
MemAccess AccessOf(const MachineInstr &mi) {
  for (const MachineOperand &operand : mi.operands()) {
    if (operand.isReg() && operand.isDef()) {
      return MemAccess::kRead;
    }
  }
  return MemAccess::kWrite;
}

} // namespace

// Mirrors SIInsertWaitcnts::updateEventWaitcntAfter, which decides which
// counters each instruction counts in: the same families, in the same order,
// with the same conditions. The comments below note where this differs.
// Branches of that function not handled here (s_sendmsg, exports, calls) are
// listed in the implementation plan's "Left out" table or don't occur in
// compute kernels.
std::optional<MemOpInfo>
llvm::hierarchical_scheduler::ClassifyMemOp(const MachineInstr &mi,
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

bool MemoryGraphOptions::IsTracked(const MemOpInfo &info) const {
  if (info.access == MemAccess::kWrite && !track_writes) {
    return false;
  }
  switch (info.type) {
    case MemType::kVector: {
      return track_vector;
    }
    case MemType::kFlat: {
      return track_flat;
    }
    case MemType::kScalar: {
      return track_scalar;
    }
    case MemType::kLds: {
      return track_lds;
    }
    case MemType::kGds: {
      return track_gds;
    }
  }
  llvm_unreachable("MemoryGraphOptions::IsTracked: unknown MemType");
}

MemoryNode &MemoryGraph::AddNode(MemoryNodeKind kind,
                                 const ScheduleNode *original_node,
                                 std::optional<MemOpInfo> mem_op_info,
                                 int latency) {
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
  node.latency_ = mem_op_info ? latency : 0;
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
    if (role.tracked_mem_op) {
      AddNode(MemoryNodeKind::kMemOp, node, role.tracked_mem_op, role.latency);
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

  // A user reads a tracked load's data through a kData edge in the original
  // graph. The original exit node isn't one: it stands for code after the
  // region, and the sink covers the region's end.
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
  // The sink is an ordinary user of every tracked load, so every read has
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

namespace {

// An op's latency (design doc section 3). misched.txt's load-latency settings
// (vmem_load_latency, smem_load_latency, lds_load_latency, and per-kernel
// kernel_latencies) are applied by LLVM only to dependence edges, in
// GCNSubtarget::adjustSchedDependency, so SUnit::Latency doesn't see them.
// This is a local copy of how that function picks a setting: by the op's
// family, a per-kernel value over the global one, and only for loads
// (mayLoad). So, as in the compiler, an op that isn't a load, such as the
// s_memtime clock read, keeps its timing-model latency. Without a setting, the
// latency is SUnit::Latency, which reads the active timing model.
int LatencyOf(const MachineInstr &mi, const SUnit &su) {
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

} // namespace

std::unique_ptr<MemoryGraph>
MemoryGraph::Build(ScheduleGraph &original, const GCNSubtarget &st,
                   const MemoryGraphOptions &options) {
  return BuildWithRoleClassifier(original, [&](const ScheduleNode &node) {
    NodeRole role;
    // The entry and exit nodes and subgraph proxies have no instruction, so
    // they play no role.
    if (!node.IsRealInstruction() || node.GetSUnit()->getInstr() == nullptr) {
      return role;
    }
    const MachineInstr &mi = *node.GetSUnit()->getInstr();
    const std::optional<MemOpInfo> info = ClassifyMemOp(mi, st);
    if (info && options.IsTracked(*info)) {
      role.tracked_mem_op = info;
      role.latency = LatencyOf(mi, *node.GetSUnit());
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
