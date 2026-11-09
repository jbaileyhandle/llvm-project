//===- MemoryGraph.h - Memory-op view of a region for MLP -------*- C++ -*-===//
//
// A graph of a region's memory operations, for measuring and searching
// memory-level parallelism (MLP). See llvm/docs/AMDGPUMemoryGraphDesign.md.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYGRAPH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYGRAPH_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/STLFunctionalExtras.h"
#include "llvm/ADT/SmallVector.h"
#include <memory>
#include <optional>
#include <vector>

namespace llvm {

class GCNSubtarget;
class MachineInstr;

namespace hierarchical_scheduler {

class ScheduleGraph;
class ScheduleNode;

/// Whether a memory op reads (later code can wait on its data) or writes.
enum class MemAccess { kRead, kWrite };

/// The family a memory op belongs to. Atomics take the type of their family.
enum class MemType { kVector, kFlat, kScalar, kLds, kGds };

/// What the classifier knows about one memory op.
struct MemOpInfo {
  MemAccess access;
  MemType type;
  // Which hardware wait counters the op counts in. Stored rather than derived
  // from `type`, because FLAT ops don't all use the same counters.
  bool counts_in_vmcnt;
  bool counts_in_lgkmcnt;
};

/// Classifies `mi` as a memory op, or returns nullopt if it isn't one.
/// See design doc section 2.
std::optional<MemOpInfo> ClassifyMemOp(const MachineInstr &mi,
                                       const GCNSubtarget &st);

class MemoryNode;

/// The read types a score tracks: the reads counted in its MLPs and in the
/// wait queues' tracked peaks. Tracking changes no timing.
struct TrackedReadTypes {
  bool vector = true;
  bool flat = true;
  bool scalar = false;
  bool lds = false;
  bool gds = false;

  /// Every read type tracked.
  static TrackedReadTypes AllReadTypesTracked() {
    TrackedReadTypes types;
    types.vector = true;
    types.flat = true;
    types.scalar = true;
    types.lds = true;
    types.gds = true;
    return types;
  }

  /// Whether `node` is a read of one of the types above.
  bool Tracks(const MemoryNode &node) const;
};

/// What a node is. Being a user (reading some load's data) is separate from
/// the kind, because one instruction can be both a load and a user: a load
/// whose address comes from another load's data.
enum class MemoryNodeKind {
  kMemOp,     // a load or write; GetMemOpInfo says which
  kBarrier,   // s_barrier, which waits for everything
  kUserOnly,  // not a memory op, only reads a load's data
  kSink,      // the end of the region; waits for every load, no instruction
};

/// An edge from one memory node to another: the other node must come after
/// this one. Design doc section 3.
struct MemoryEdge {
  MemoryNode *node;     // the connected node
  bool waits_for_data;  // the later node reads the earlier load's data, so
                        // time passes between them; otherwise ordering only
};

/// Where each memory op's latency comes from (design doc section 5.1).
enum class LatencySetting {
  kUnit,  // 1 for every load and write
  kLlvm,  // the op's LLVM latency, stored at build (design doc section 3)
};

/// One node of a MemoryGraph. Organized like ScheduleNode: read-only
/// accessors, filled in by MemoryGraph.
class MemoryNode {
public:
  /// This node's position in the graph's node list.
  int GetId() const { return id_; }
  /// This node's position in the graph's topological order.
  int GetTopoIndex() const { return topo_index_; }
  MemoryNodeKind GetKind() const { return kind_; }

  /// The original graph's node and instruction. Null for the sink. The
  /// ScheduleNode is valid only while the original graph lives; the
  /// MachineInstr survives graph rebuilds.
  const ScheduleNode *GetOriginalNode() const { return original_node_; }
  const MachineInstr *GetOriginalInstr() const { return original_instr_; }

  /// Set when the kind is kMemOp.
  const std::optional<MemOpInfo> &GetMemOpInfo() const { return mem_op_info_; }
  bool IsRead() const {
    return mem_op_info_ && mem_op_info_->access == MemAccess::kRead;
  }

  /// This op's LLVM latency, stored at build (design doc section 3): its
  /// latency under LatencySetting::kLlvm. 0 unless it's a load or write.
  int GetLlvmLatency() const { return llvm_latency_; }
  /// This op's latency under `setting`. 0 unless it's a load or write.
  int GetLatency(LatencySetting setting) const {
    if (kind_ != MemoryNodeKind::kMemOp) {
      return 0;
    }
    return setting == LatencySetting::kUnit ? 1 : llvm_latency_;
  }

  /// The loads whose data this node reads. Non-empty exactly when
  /// this node is a user, whatever its kind: a load whose address comes from
  /// another load's data has kind kMemOp and a non-empty list.
  ArrayRef<MemoryNode *> GetLoadsUsed() const { return loads_used_; }

  ArrayRef<MemoryEdge> Successors() const { return successors_; }
  ArrayRef<MemoryEdge> Predecessors() const { return predecessors_; }

private:
  // Only MemoryGraph fills nodes in, while building the graph. Everything else
  // gets the read-only accessors above, so a node can't change once the graph
  // is built. ScheduleGraph does the same for ScheduleNode.
  friend class MemoryGraph;

  int id_ = -1;
  int topo_index_ = -1;
  MemoryNodeKind kind_ = MemoryNodeKind::kUserOnly;
  const ScheduleNode *original_node_ = nullptr;
  const MachineInstr *original_instr_ = nullptr;
  std::optional<MemOpInfo> mem_op_info_;
  int llvm_latency_ = 0;
  SmallVector<MemoryNode *, 4> loads_used_;
  SmallVector<MemoryEdge, 4> successors_;
  SmallVector<MemoryEdge, 4> predecessors_;
};

/// The role one node of the original graph plays in the memory graph: a
/// memory op (with its latency), a barrier, or neither. Whether it's a user
/// isn't part of this; the build works that out from the graph's edges.
struct NodeRole {
  std::optional<MemOpInfo> mem_op;  // set if a load or write
  int llvm_latency = 0;             // used when mem_op is set
  bool is_barrier = false;
};

/// One MLP with its parts (MLP doc, sections 2.3 and 2.4).
struct MlpMeasure {
  int summed_read_flight_time = 0;  // each tracked read's flight time, added up
  int any_read_in_flight_time = 0;  // time during which any tracked read is in
                                    // flight
  float mlp = 0;                    // summed / any; 0 when any is 0
};

/// When one memory op is in flight: from start_time until stop_time.
struct MemOpFlight {
  int start_time;
  int stop_time;
};

/// The MLP of a set of flights (MLP doc, section 2.3). Time when two flights
/// overlap counts twice in the summed time and once in the "any" time.
MlpMeasure MeasureMlp(std::vector<MemOpFlight> flights);

/// The region's memory work with every op started as early as the edges allow
/// (design doc section 4). No order finishes sooner than elapsed_time. Its MLP
/// is the highest any order can get only when every read in the graph is
/// tracked.
struct IdealScore {
  MlpMeasure latency_based_mlp;  // each tracked read in flight from its start
                                 // time until its start time plus its latency
  int elapsed_time = 0;          // when the last read's data is back
};

/// One order's memory work, walked through the wait queues (design doc
/// section 5). Its MLPs count only the tracked reads. Its timing counts every
/// op.
struct OrderScore {
  // Each tracked read in flight from its issue time until the first node
  // whose waits cover it issues. A FLAT in both queues is covered once per
  // queue, and is in flight until the later of the two.
  MlpMeasure use_based_mlp;

  // Each tracked read in flight from its issue time until its issue time plus
  // its latency. This is the one to compare with IdealScore's.
  MlpMeasure latency_based_mlp;

  // When the sink issues: once its waits for every read have ended.
  int elapsed_time = 0;

  // The most ops each queue held at once, and the most tracked reads.
  int peak_vmcnt_entries = 0;
  int peak_lgkmcnt_entries = 0;
  int peak_vmcnt_tracked_entries = 0;
  int peak_lgkmcnt_tracked_entries = 0;
};

/// What happened at one node of ScoreOrder's walk. For tests and debugging.
struct WalkStep {
  const MemoryNode *node = nullptr;

  // When the walk reached the node: the issue time of the node before it, or
  // 0 for the first node.
  int issue_wait_begin_time = 0;

  // Whether cautious mode was on when the node's waits were decided, which
  // makes every one of them a drain.
  bool cautious = false;

  // When the node issued, after its waits.
  int issue_time = 0;

  // The ops the node's waits popped from the queues. A FLAT in both queues
  // can be listed twice, once per queue.
  std::vector<const MemoryNode *> popped;
};

/// A graph of one region's memory ops (design doc section 3). Organized like
/// ScheduleGraph: nodes reserved up front so pointers into them stay valid,
/// edges hold node pointers, and each node stores its own id and topo index.
class MemoryGraph {
public:
  /// Builds the graph for a region, giving each node its role with
  /// ClassifyMemOp and the barrier check, so every memory op gets a node.
  /// Computes the original graph's reachability if it hasn't been yet, which
  /// is why `original` isn't const.
  static std::unique_ptr<MemoryGraph> Build(ScheduleGraph &original,
                                            const GCNSubtarget &st);

  /// The same build, using `role_classifier` to give each node its role. Build
  /// calls this; tests call it directly with their own roles, since test
  /// graphs have no real instructions for ClassifyMemOp to look at.
  static std::unique_ptr<MemoryGraph> BuildWithRoleClassifier(
      ScheduleGraph &original,
      function_ref<NodeRole(const ScheduleNode &)> role_classifier);

  /// Takes a full schedule of the original graph and returns its memory nodes
  /// in that order, ending with the sink. Used to score the real schedule.
  std::vector<const MemoryNode *>
  OrderFromSchedule(ArrayRef<const ScheduleNode *> schedule) const;

  /// The region's ideal under `setting`: the best its memory work could do,
  /// held back only by the edges (design doc section 4). Its MLP counts only
  /// the reads `tracked_read_types` picks; its elapsed time counts every read.
  IdealScore ScoreIdeal(LatencySetting setting,
                        const TrackedReadTypes &tracked_read_types) const;

  /// Walks `order` through the wait queues and scores it (design doc section
  /// 5). `order` must hold every node of this graph once, each after its
  /// predecessors, so the sink is last; this fails hard otherwise.
  /// OrderFromSchedule gives such an order from a full schedule. The MLPs
  /// count only the reads `tracked_read_types` picks. The timing counts every
  /// op. If `steps` isn't null, it's filled with one WalkStep per node of
  /// `order`.
  OrderScore ScoreOrder(ArrayRef<const MemoryNode *> order,
                        LatencySetting setting,
                        const TrackedReadTypes &tracked_read_types,
                        std::vector<WalkStep> *steps = nullptr) const;

  /// ScoreOrder on the memory nodes of `schedule`, a full schedule of the
  /// original graph this graph was built from (OrderFromSchedule). Used to
  /// score a region's schedule as it stands.
  OrderScore ScoreSchedule(ArrayRef<const ScheduleNode *> schedule,
                           LatencySetting setting,
                           const TrackedReadTypes &tracked_read_types,
                           std::vector<WalkStep> *steps = nullptr) const;

  ArrayRef<MemoryNode> Nodes() const { return nodes_; }
  ArrayRef<MemoryNode *> GetTopoOrder() const { return topo_order_; }
  const MemoryNode *GetSink() const { return sink_; }

  /// The memory node for an original-graph node, or null if it isn't in this
  /// graph.
  const MemoryNode *GetMemoryNode(const ScheduleNode *original_node) const;

  MemoryGraph(const MemoryGraph &) = delete;
  MemoryGraph &operator=(const MemoryGraph &) = delete;

private:
  MemoryGraph() = default;

  /// Adds a complete node. `mem_op_info` must be set exactly when the kind is
  /// kMemOp, and `llvm_latency` is used only then. `original_node` is null
  /// only for the sink. nodes_ is reserved up front and this fails hard if it
  /// would reallocate, because nodes point at each other.
  MemoryNode &AddNode(MemoryNodeKind kind, const ScheduleNode *original_node,
                      std::optional<MemOpInfo> mem_op_info = std::nullopt,
                      int llvm_latency = 0);
  void AddEdge(MemoryNode &from, MemoryNode &to, bool waits_for_data);

  // The build's steps, called in this order by BuildWithRoleClassifier.

  /// Computes the original graph's topological order and reachability if they
  /// haven't been yet. This graph's order and edges are taken from them.
  static void PrepareOriginalGraph(ScheduleGraph &original);
  /// Adds a node for each memory op and each barrier.
  void AddMemOpAndBarrierNodes(
      const ScheduleGraph &original,
      function_ref<NodeRole(const ScheduleNode &)> role_classifier);
  /// Adds the users: nodes that read a load's data.
  void AddUsers(const ScheduleGraph &original);
  /// Adds one edge per pair where the original graph says B must come after A.
  void AddEdges(const ScheduleGraph &original);
  /// Adds the sink, after every node and a user of every load.
  void AddSink();
  /// Sorts the nodes into topological order and sets each node's topo index.
  /// The order is the original nodes' topological order, with the sink last.
  void ComputeTopologicalOrder();

  std::vector<MemoryNode> nodes_;
  std::vector<MemoryNode *> topo_order_;
  MemoryNode *sink_ = nullptr;
  DenseMap<const ScheduleNode *, MemoryNode *> original_node_to_memory_node_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYGRAPH_H
