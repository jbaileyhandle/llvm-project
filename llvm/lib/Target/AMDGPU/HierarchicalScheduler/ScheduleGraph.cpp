//===- ScheduleGraph.cpp - Hierarchical scheduling graph ------------------===//
//
// Implementation of ScheduleGraph and ScheduleNode.
//
//===----------------------------------------------------------------------===//

#include "ScheduleGraph.h"
#include "DominatorTree.h"
#include "GCNRegPressure.h"
#include "HierarchicalConfig.h"
#include "NodeRegInfo.h"
#include "RegionInfo.h"
#include "ScheduleConstructor.h"
#include "SIMachineFunctionInfo.h"
#include "SubgraphInfo.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <queue>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

// Process-unique id counter shared by ScheduleNode, ScheduleGraph, and
// SubgraphInfo (declared in ScheduleGraph.h). Not atomic: codegen runs
// single-threaded per function here.
int64_t llvm::hierarchical_scheduler::GetAndIncrementScheduleId() {
  static int64_t next_id = 0;
  return next_id++;
}

namespace {

/// Map an SDep edge to our ScheduleEdge::Kind. Preserves the full
/// distinction between SDep::Kind and SDep::OrderKind so no information
/// is lost.
///
/// SDep::Kind has four values: Data, Anti, Output, Order.
/// When Kind == Order, SDep::OrderKind further distinguishes:
///   Barrier, MayAliasMem, MustAliasMem, Artificial, Weak, Cluster.
/// We map each combination to a unique ScheduleEdge::Kind.
ScheduleEdge::Kind MapSDepToEdgeKind(const SDep &dep) {
  switch (dep.getKind()) {
  case SDep::Data:
    return ScheduleEdge::kData;
  case SDep::Anti:
    return ScheduleEdge::kAnti;
  case SDep::Output:
    return ScheduleEdge::kOutput;
  case SDep::Order:
    // Order edges have a sub-kind (OrderKind). SDep doesn't expose
    // OrderKind directly, so we use the query methods. We check Cluster
    // before Weak because isWeak() returns true for both Weak and Cluster
    // (it checks OrdKind >= Weak, and Cluster comes after Weak in the
    // enum). Every OrderKind has an explicit check; if none match, we hit
    // llvm_unreachable.
    if (dep.isCluster()) {
      return ScheduleEdge::kCluster;
    }
    if (dep.isWeak()) {
      return ScheduleEdge::kWeak;
    }
    if (dep.isBarrier()) {
      return ScheduleEdge::kBarrier;
    }
    if (dep.isMustAlias()) {
      return ScheduleEdge::kMustAliasMem;
    }
    if (dep.isArtificial()) {
      return ScheduleEdge::kArtificial;
    }
    if (dep.isNormalMemory()) {
      return ScheduleEdge::kMayAliasMem;
    }
    llvm_unreachable("Unknown SDep::OrderKind");
  }
  llvm_unreachable("Unknown SDep::Kind");
}

// Verify a structural invariant of the graph: under the given edge
// filter, exactly one node has no predecessors (the source) and
// exactly one has no successors (the sink). Downstream consumers
// (e.g., GetCriticalPathLength reading topo_order_[0] as the unique
// source) rely on this for the strong-edge interpretation; checking
// the include-weak interpretation too catches accidental disconnection
// or stray weak-only ends. report_fatal_error if violated.
void CheckSingleSourceSingleSink(const ScheduleGraph &graph,
                                 bool include_weak_edges) {
  int source_count = 0;
  int sink_count = 0;
  for (const ScheduleNode &node : graph.Nodes()) {
    bool has_pred = false;
    for (const ScheduleEdge &pred : node.Predecessors()) {
      if (include_weak_edges || pred.IsStrongEdge()) {
        has_pred = true;
        break;
      }
    }
    if (!has_pred) {
      ++source_count;
    }
    bool has_succ = false;
    for (const ScheduleEdge &succ : node.Successors()) {
      if (include_weak_edges || !succ.IsWeakEdge()) {
        has_succ = true;
        break;
      }
    }
    if (!has_succ) {
      ++sink_count;
    }
  }
  if (source_count == 1 && sink_count == 1) {
    return;
  }
  std::string msg =
      graph.ToString() +
      ": expected exactly one source and one sink (include_weak_edges=" +
      (include_weak_edges ? "true" : "false") + "), got " +
      std::to_string(source_count) + " sources and " +
      std::to_string(sink_count) + " sinks";
  report_fatal_error(llvm::StringRef(msg));
}

} // anonymous namespace

// --- ScheduleNode ---

ScheduleNode::ScheduleNode(SUnit *su, ScheduleGraph *top_level_graph)
    : id_(GetAndIncrementScheduleId()),
      graph_local_id_(top_level_graph->GetAndIncrementGraphLocalId()),
      content_(su) {
  ExtractRegInfo();
}

ScheduleNode::ScheduleNode(SUnit *su, std::string debug_name,
                           ScheduleGraph *top_level_graph)
    : id_(GetAndIncrementScheduleId()),
      graph_local_id_(top_level_graph->GetAndIncrementGraphLocalId()),
      content_(su), debug_name_(std::move(debug_name)) {
  ExtractRegInfo();
}

ScheduleNode::ScheduleNode(SubgraphInfo *info,
                           ScheduleGraph *top_level_graph)
    : id_(GetAndIncrementScheduleId()),
      graph_local_id_(top_level_graph->GetAndIncrementGraphLocalId()),
      content_(info) {}

SubgraphInfo *ScheduleNode::GetSubgraphInfo() const {
  if (!IsSubgraphProxy()) {
    report_fatal_error(
        "GetSubgraphInfo called on scheduling-unit node " +
        Twine(id_));
  }
  // Both start and end proxies carry the same raw back-reference.
  return std::get<SubgraphInfo *>(content_);
}

void ScheduleNode::ExtractRegInfo() {
  if (!IsSchedulingUnit()) {
    return;
  }
  SUnit *su = GetSUnit();
  if (!su || !su->getInstr()) {
    return;
  }
  for (const MachineOperand &mo : su->getInstr()->operands()) {
    if (!mo.isReg() || !mo.getReg().isVirtual()) {
      continue;
    }
    // Full lane mask as default — the GCNRegisterTracker computes
    // accurate lane masks directly from the MachineInstr operands,
    // so these are only used by the old RegisterTracker.
    if (mo.isDef()) {
      reg_defs_.push_back({mo.getReg(), LaneBitmask::getAll()});
    } else if (mo.isUse() && !mo.isUndef()) {
      reg_uses_.push_back({mo.getReg(), LaneBitmask::getAll()});
    }
  }
}

std::string ScheduleNode::ToString() const {
  std::string result = "[" + std::to_string(id_);

  if (IsSubgraphProxy()) {
    SubgraphInfo *info = GetSubgraphInfo();
    const char *kind = IsSubgraphStartProxy() ? "start" : "end";
    result += ":proxy_" + std::string(kind) + "(" + info->debug_name +
              "," + std::to_string(info->members.size()) + ")]";
    return result;
  }

  if (!debug_name_.empty()) {
    result += ":" + debug_name_ + "]";
    return result;
  }

  SUnit *su = std::get<SUnit *>(content_);
  if (su && su->getInstr()) {
    std::string mi_str;
    raw_string_ostream os(mi_str);
    // IsStandalone=true: print without context (self-contained).
    // SkipOpers=false: include all operands.
    // SkipDebugLoc=true: omit source location noise.
    // AddNewLine=false: no trailing newline.
    su->getInstr()->print(os, /*IsStandalone=*/true, /*SkipOpers=*/false,
                          /*SkipDebugLoc=*/true, /*AddNewLine=*/false);
    // Trim leading whitespace from MachineInstr::print output.
    size_t start = mi_str.find_first_not_of(" \t\n");
    if (start != std::string::npos) {
      mi_str = mi_str.substr(start);
    }
    result += "] " + mi_str;
    return result;
  }

  result += "]";
  return result;
}

// --- ScheduleGraph ---

ScheduleGraph::ScheduleGraph()
    : id_(GetAndIncrementScheduleId()),
      graph_local_id_(GetAndIncrementGraphLocalId()) {}
ScheduleGraph::~ScheduleGraph() = default;

// std::unique_ptr's default constructor value-initializes the stored
// pointer to nullptr (C++ standard guarantee, [unique.ptr.single.ctor]),
// so node_reg_info_table_ is null until SetNodeRegInfoTable installs
// one. The null check below catches the "factory forgot to install"
// bug at first read rather than crashing on the dereference.
const NodeRegInfoTable &ScheduleGraph::GetNodeRegInfoTable() const {
  if (node_reg_info_table_ == nullptr) {
    report_fatal_error("ScheduleGraph::GetNodeRegInfoTable: table "
                       "not yet installed by the graph factory");
  }
  return *node_reg_info_table_;
}

void ScheduleGraph::SetNodeRegInfoTable(NodeRegInfoTable table) {
  node_reg_info_table_ =
      std::make_unique<NodeRegInfoTable>(std::move(table));
}

std::string ScheduleGraph::ToString() const {
  return "graph[" + std::to_string(id_) + "](" +
         std::to_string(Size()) + " nodes)";
}

int ScheduleGraph::NumSchedulingUnits() const {
  if (!cached_num_scheduling_units_.has_value()) {
    cached_num_scheduling_units_ =
        static_cast<int>(llvm::count_if(nodes_, [](const ScheduleNode &n) {
          return n.IsSchedulingUnit();
        }));
  }
  return *cached_num_scheduling_units_;
}

namespace {

/// Precondition for installing subgraphs (run by registerSubgraphs, so
/// it guards both InsertSubgraphProxies and InstallSubgraphsFlat): every
/// member of every SubgraphInfo is a scheduling unit. Nested subgraphs
/// are deferred to a future phase; until then a subgraph proxy appearing
/// as a member is treated as a malformed input.
void CheckNoNestedMembers(
    ArrayRef<std::unique_ptr<SubgraphInfo>> infos) {
  for (const auto &info : infos) {
    for (ScheduleNode *m : info->members) {
      if (!m->IsSchedulingUnit()) {
        report_fatal_error(
            "registerSubgraphs: SubgraphInfo \"" +
            Twine(info->debug_name) +
            "\" has a member that is itself a subgraph proxy "
            "(node " +
            Twine(m->GetId()) +
            "); nested subgraphs are not yet supported");
      }
    }
  }
}

/// Precondition for installing subgraphs (run by registerSubgraphs, so
/// it guards both installs): each node appears as a member of at most
/// one SubgraphInfo.
void CheckMembersDisjoint(
    ArrayRef<std::unique_ptr<SubgraphInfo>> infos) {
  SmallPtrSet<ScheduleNode *, 32> seen;
  for (const auto &info : infos) {
    for (ScheduleNode *m : info->members) {
      if (!seen.insert(m).second) {
        report_fatal_error(
            "registerSubgraphs: node " + Twine(m->GetId()) +
            " appears as a member of more than one SubgraphInfo");
      }
    }
  }
}

} // anonymous namespace

// Mutation step for one SubgraphInfo: emplace BOTH its start and
// end proxy nodes (each holds a raw back-pointer to `info`, which is
// owned by subgraph_infos_), set parent_subgraph_proxy on members and
// on the end proxy itself,
// and add four families of kSubgraphOrderEdge artificials:
//   - ext_predecessor → start_proxy  (gates start on external preds)
//   - start_proxy → member           (gates members on start being scheduled)
//   - member → end_proxy             (gates end on every member being scheduled)
//   - end_proxy → ext_successor      (gates external succs on subgraph exit)
//
// Original member↔external real edges are NOT touched — they keep
// their data latencies for the length tracker. The artificials are
// purely additive.
//
// Member function (rather than anonymous-namespace helper) because
// EmplaceNode is private — only ScheduleGraph members can call it.
//
// Note on parent_subgraph_proxy assignment under the flat (two-level)
// shape:
//   - start_proxy's own parent_subgraph_proxy_ stays at default
//     nullptr — every subgraph is top-level under flat Phase 2,
//     so the start proxy lives in the base scope.
//   - end_proxy's parent_subgraph_proxy_ is set to start_proxy so
//     the end proxy lives INSIDE the subgraph scope (it gets
//     released into that scope and is the node whose Schedule call
//     pops the scope).
// When nesting lands in a future phase, the enclosing subgraph's
// InsertSubgraphProxies pass will overwrite the start proxy's
// parent_subgraph_proxy if the start proxy appears as one of its
// members. The flat case never produces that situation
// (CheckNoNestedMembers rules it out).
void ScheduleGraph::EmplaceProxyAndWireEdges(SubgraphInfo *info) {
  // `info` is already owned by subgraph_infos_; both proxies hold a raw
  // back-reference to it (the object stays at a stable heap address).
  ScheduleNode &start_proxy = EmplaceNode(info, this);
  info->subgraph_proxy = &start_proxy;
  for (ScheduleNode *m : info->members) {
    m->SetParentSubgraphProxy(&start_proxy);
  }
  for (ScheduleNode *p : info->ext_predecessors) {
    AddEdge(p, &start_proxy, ScheduleEdge::kSubgraphOrderEdge,
            /*latency=*/0);
  }
  for (ScheduleNode *m : info->members) {
    AddEdge(&start_proxy, m, ScheduleEdge::kSubgraphOrderEdge,
            /*latency=*/0);
  }

  // End proxy: takes a raw SubgraphInfo* (ownership stays with the
  // start proxy). Lives inside the subgraph scope, so its
  // parent_subgraph_proxy is the start proxy.
  ScheduleNode &end_proxy = EmplaceNode(info, this);
  info->end_proxy = &end_proxy;
  end_proxy.SetParentSubgraphProxy(&start_proxy);
  for (ScheduleNode *m : info->members) {
    AddEdge(m, &end_proxy, ScheduleEdge::kSubgraphOrderEdge,
            /*latency=*/0);
  }
  for (ScheduleNode *s : info->ext_successors) {
    AddEdge(&end_proxy, s, ScheduleEdge::kSubgraphOrderEdge,
            /*latency=*/0);
  }
}

SmallVector<SubgraphInfo *> ScheduleGraph::registerSubgraphs(
    std::vector<std::unique_ptr<SubgraphInfo>> infos) {
  // Preconditions, shared by both installs and independent of proxies:
  // members are scheduling units, not nested subgraphs
  // (CheckNoNestedMembers), and disjoint across subgraphs
  // (CheckMembersDisjoint).
  CheckNoNestedMembers(infos);
  CheckMembersDisjoint(infos);

  subgraph_infos_.reserve(subgraph_infos_.size() + infos.size());
  SmallVector<SubgraphInfo *> added;
  added.reserve(infos.size());
  for (auto &info_ptr : infos) {
    // The subgraph_infos_ vector owns the info; everything else (proxies,
    // members' parent pointers) back-references it by raw pointer, which
    // stays valid for the graph's lifetime.
    added.push_back(info_ptr.get());
    subgraph_infos_.push_back(std::move(info_ptr));
  }

  // Sort by member count descending so consumers (telemetry, debug
  // dumps) see the largest subgraphs first. The returned raw pointers
  // stay valid across the sort — the SubgraphInfo objects don't move.
  std::sort(subgraph_infos_.begin(), subgraph_infos_.end(),
            [](const std::unique_ptr<SubgraphInfo> &a,
               const std::unique_ptr<SubgraphInfo> &b) {
              return a->members.size() > b->members.size();
            });
  return added;
}

void ScheduleGraph::InsertSubgraphProxies(
    std::vector<std::unique_ptr<SubgraphInfo>> infos) {
  if (infos.empty()) {
    return;
  }

  // Register the subgraphs (ownership + preconditions; see
  // registerSubgraphs), then emplace a start/end proxy pair per
  // newly-added subgraph.
  SmallVector<SubgraphInfo *> added = registerSubgraphs(std::move(infos));
  for (SubgraphInfo *info : added) {
    EmplaceProxyAndWireEdges(info);
  }

  // Keep the per-node register-info table aligned with the new graph
  // size: the proxies just emplaced have graph_local_ids past the
  // original table bounds. New entries are default-constructed (no
  // defs, no uses) — proxies have no register effect.
  if (node_reg_info_table_) {
    node_reg_info_table_->EnsureSize(GetNumGraphLocalIds());
  }

  // Proxies added nodes + edges; re-derive topo order and critical
  // paths. The topo recompute also re-runs cycle detection, our
  // re-entrancy catch for the just-inserted subgraphs.
  ValidateAndComputeTopologicalOrder();
  ComputeCriticalPaths();
}

void ScheduleGraph::InstallSubgraphsFlat(
    std::vector<std::unique_ptr<SubgraphInfo>> infos) {
  if (infos.empty()) {
    return;
  }
  // Register the subgraphs (ownership + preconditions, see
  // registerSubgraphs) without emplacing proxies. Members stay ordinary
  // nodes, so the graph's nodes and edges are unchanged — no
  // topo/critical-path recompute is needed here. The chosen interiors
  // are locked later by AddSubgraphOrderEdges (which does its own
  // re-derive). Interleaving mode; see §8 of
  // AMDGPUSubgraphSchedulingDesign.md.
  registerSubgraphs(std::move(infos));
}

void ScheduleGraph::AddSubgraphOrderEdges() {
  for (const std::unique_ptr<SubgraphInfo> &info_ptr : subgraph_infos_) {
    SubgraphInfo *info = info_ptr.get();
    // A subgraph with no recorded schedule is left free — its members
    // keep whatever ordering freedom the real edges allow.
    if (!info->schedule_result.has_value()) {
      continue;
    }
    ArrayRef<ScheduleNode *> order = info->schedule_result->order;
    assert(order.size() == info->members.size() &&
           "AddSubgraphOrderEdges: schedule_result.order must list "
           "all subgraph members");
    // Chain consecutive members. Each kSubgraphOrderEdge is the last
    // of order[i+1]'s predecessors to be satisfied (a valid topo
    // order places every other predecessor in order[0..i]), so
    // ReleaseSuccessors releases order[i+1] exactly when order[i] is
    // scheduled — one member ready at a time, in the chosen order.
    for (int i = 0; i + 1 < static_cast<int>(order.size()); ++i) {
      AddEdge(order[i], order[i + 1], ScheduleEdge::kSubgraphOrderEdge,
              /*latency=*/0);
    }
  }

  // AddEdge invalidated the topo order and critical paths; re-derive
  // them so the graph is queryable for the proxied-graph search. The
  // topo recompute also re-runs cycle detection: a recorded order
  // that contradicts the real edges surfaces here as a graph cycle.
  // Both calls are cache-aware — a no-op when no edge was added.
  ValidateAndComputeTopologicalOrder();
  ComputeCriticalPaths();
}

void ScheduleGraph::PrintSubgraphInfos(raw_ostream &os,
                                       StringRef indent) const {
  int total_members_covered = 0;
  for (const std::unique_ptr<SubgraphInfo> &info : subgraph_infos_) {
    total_members_covered += static_cast<int>(info->members.size());
  }
  os << indent << "subgraphs: count=" << subgraph_infos_.size()
     << " covered=" << total_members_covered
     << "/" << NumSchedulingUnits() << "\n";
  for (size_t i = 0; i < subgraph_infos_.size(); ++i) {
    const SubgraphInfo *info = subgraph_infos_[i].get();
    os << indent << "\t[" << i << "] members=" << info->members.size()
       << " name=" << info->debug_name << "\n";
  }
}

// Kahn's algorithm: iteratively remove nodes with no unmet predecessors.
void ScheduleGraph::ValidateAndComputeTopologicalOrder(
    bool include_weak_edges) {
  // Cache-aware: skip if a current order exists AND it was computed
  // with the same include_weak_edges mode. Cleared by any graph
  // mutation via InvalidateDerivedData.
  if (IsTopoSorted() &&
      topo_order_include_weak_edges_ == include_weak_edges) {
    return;
  }
  ComputeTopologicalOrder(include_weak_edges);
  topo_order_include_weak_edges_ = include_weak_edges;
  CheckSingleSourceSingleSink(*this, /*include_weak_edges=*/false);
  CheckSingleSourceSingleSink(*this, /*include_weak_edges=*/true);
}

// Produces a forward topological order (sources first, sinks last).
// If not all nodes are placed, the graph contains a cycle.
void ScheduleGraph::ComputeTopologicalOrder(bool include_weak_edges) {
  // Re-sorting (e.g., with a different `include_weak_edges`) would
  // produce a different topo_order_, silently invalidating any
  // downstream caches keyed by topo index. Clear them up front.
  InvalidateDerivedData();
  topo_order_.reserve(nodes_.size());

  // Count the number of relevant predecessors for each node. We use a
  // separate counter rather than modifying the graph.
  DenseMap<ScheduleNode *, int> in_degree;
  for (ScheduleNode &node : nodes_) {
    int count = 0;
    for (const ScheduleEdge &pred : node.Predecessors()) {
      if (include_weak_edges || pred.IsStrongEdge()) {
        ++count;
      }
    }
    in_degree[&node] = count;
  }

  // Seed the queue with all nodes that have no relevant predecessors.
  std::queue<ScheduleNode *> ready;
  for (ScheduleNode &node : nodes_) {
    if (in_degree[&node] == 0) {
      ready.push(&node);
    }
  }

  // Process: remove a ready node, decrement its successors' in-degrees,
  // and add any that reach zero to the queue.
  while (!ready.empty()) {
    ScheduleNode *node = ready.front();
    ready.pop();

    node->SetTopoIndex(static_cast<int>(topo_order_.size()));
    topo_order_.push_back(node);

    for (const ScheduleEdge &succ : node->Successors()) {
      if (!include_weak_edges && succ.IsWeakEdge()) {
        continue;
      }
      int &deg = in_degree[succ.node_];
      --deg;
      if (deg == 0) {
        ready.push(succ.node_);
      }
    }
  }

  if (static_cast<int>(topo_order_.size()) != Size()) {
    std::string msg = "Cycle detected in " + ToString();
    report_fatal_error(llvm::StringRef(msg));
  }

  // Sort each node's successor list by ascending topo index. Required by
  // the transitive reduction algorithm (process closest successors first).
  for (ScheduleNode &node : nodes_) {
    node.SortSuccessorsByTopoIndex();
  }
}

void ScheduleGraph::InvalidateDerivedData() {
  topo_order_.clear();
  topo_order_include_weak_edges_.reset();
  ClearCriticalPaths();
  reduced_graph_.reset();
  reachability_by_topo_index_.clear();
  dom_tree_.reset();
  cached_num_scheduling_units_.reset();
  // input_schedule_constructor_ is NOT cleared; it's a Phase-4 artifact
  // tied to the original SUnit emplacement order, not a structural
  // derivation that changes when the graph mutates.
}

void ScheduleGraph::ComputeCriticalPathFromExit() {
  // Cache-aware: skip if cp is already current. Cleared by any
  // graph mutation via InvalidateDerivedData (which also clears
  // topo, so the not-topo-sorted check below would re-trigger if
  // anything got invalidated).
  if (HasCriticalPathFromExit()) {
    return;
  }
  if (!IsTopoSorted()) {
    std::string msg = "ComputeCriticalPathFromExit called on " + ToString() +
                      " before ComputeTopologicalOrder";
    report_fatal_error(llvm::StringRef(msg));
  }

  // One slot per node, indexed by topo_index. Default-initialized to 0
  // so the exit node's base case is implicit.
  critical_path_from_exit_by_topo_index_.assign(Size(), 0);

  // Walk in reverse topological order: successors are visited before
  // predecessors, so cp_from_exit[succ] is ready when we compute
  // cp_from_exit[node].
  for (ScheduleNode *node : llvm::reverse(topo_order_)) {
    int max_cp = 0;
    for (const ScheduleEdge &edge : node->Successors()) {
      if (!edge.IsLatencyEdge()) {
        continue;
      }
      // Per-edge cycle delta: max of the edge's modeled latency
      // and the source's IssueWidth=1 issue-slot consumption.
      int weight = std::max(edge.Latency(), node->IssueSlotsConsumed());
      int cp = weight +
               critical_path_from_exit_by_topo_index_[edge.node_->GetTopoIndex()];
      if (cp > max_cp) {
        max_cp = cp;
      }
    }
    critical_path_from_exit_by_topo_index_[node->GetTopoIndex()] = max_cp;
  }
  // Single-source graphs: topo_index 0 is always the source (Kahn's
  // emits roots first, and we have exactly one root). cp at the source
  // is the longest latency-weighted path through the whole graph.
  int cp_at_source = critical_path_from_exit_by_topo_index_[0];
  critical_path_length_ = cp_at_source;
  graph_length_floor_ = std::max(NumSchedulingUnits(), cp_at_source + 1);
}

void ScheduleGraph::ComputeCriticalPathFromEntry() {
  // Cache-aware: skip if cp is already current. Cleared by any
  // graph mutation via InvalidateDerivedData.
  if (HasCriticalPathFromEntry()) {
    return;
  }
  if (!IsTopoSorted()) {
    std::string msg = "ComputeCriticalPathFromEntry called on " + ToString() +
                      " before ComputeTopologicalOrder";
    report_fatal_error(llvm::StringRef(msg));
  }

  // One slot per node, indexed by topo_index. Default-initialized to 0
  // so the entry node's base case is implicit.
  critical_path_from_entry_by_topo_index_.assign(Size(), 0);

  // Walk in forward topological order: predecessors are visited before
  // successors, so cp_from_entry[pred] is ready when we compute
  // cp_from_entry[node].
  for (ScheduleNode *node : topo_order_) {
    int max_cp = 0;
    for (const ScheduleEdge &edge : node->Predecessors()) {
      if (!edge.IsLatencyEdge()) {
        continue;
      }
      ScheduleNode *predecessor = edge.node_;
      // Per-edge cycle delta: max of the edge's modeled latency
      // and the predecessor's IssueWidth=1 issue-slot consumption.
      // Mirrors the weight rule in ComputeCriticalPathFromExit.
      int weight = std::max(edge.Latency(), predecessor->IssueSlotsConsumed());
      int cp = weight +
               critical_path_from_entry_by_topo_index_[predecessor->GetTopoIndex()];
      if (cp > max_cp) {
        max_cp = cp;
      }
    }
    critical_path_from_entry_by_topo_index_[node->GetTopoIndex()] = max_cp;
  }
}

void ScheduleGraph::ComputeTransitiveReductionAndReachability() {
  // Cache-aware: skip if reduced graph + reachability are already
  // current. They're populated together and cleared together in
  // InvalidateDerivedData, so HasReducedGraph is the right gate.
  if (HasReducedGraph()) {
    return;
  }
  if (!IsTopoSorted()) {
    std::string msg = "ComputeTransitiveReductionAndReachability called on " + ToString() +
                      " before ComputeTopologicalOrder";
    report_fatal_error(llvm::StringRef(msg));
  }

  int num_nodes = Size();

  // One BitVector per node, each of size num_nodes.
  // reachability_by_topo_index_[i] tracks which nodes are reachable
  // from the node at topo index i (including itself). Built up here
  // as a working set for redundancy detection, then kept as a graph
  // member (subgraph formation in particular consumes it; see
  // AMDGPUSubgraphFormationDesign.md §4.3).
  reachability_by_topo_index_.assign(num_nodes, BitVector(num_nodes, false));
  for (int topo_idx = 0; topo_idx < num_nodes; ++topo_idx) {
    reachability_by_topo_index_[topo_idx].set(topo_idx);
  }

  auto reduced = std::make_unique<ReducedGraph>(num_nodes);

  // Process nodes in reverse topo order (sinks first, sources last).
  // For each node, iterate its successors in ascending topo order (closest
  // first, guaranteed by SortSuccessorsByTopoIndex in ComputeTopologicalOrder).
  //
  // If a successor is already in our reachable set, it means we can reach
  // it through a closer successor we already processed — the direct edge
  // is redundant. Otherwise, the edge is essential: we keep it and merge
  // the successor's reachable set into ours.
  for (int curr_topo_idx = num_nodes - 1; curr_topo_idx >= 0;
       --curr_topo_idx) {
    ScheduleNode *curr_node = topo_order_[curr_topo_idx];

    for (const ScheduleEdge &succ_edge : curr_node->Successors()) {
      int succ_topo_idx = succ_edge.node_->GetTopoIndex();

      if (reachability_by_topo_index_[curr_topo_idx].test(succ_topo_idx)) {
        // Already reachable via a closer path — edge is redundant.
        continue;
      }

      // Essential edge — keep it in the reduced graph.
      reduced->successors_by_topo_index[curr_topo_idx].push_back(succ_topo_idx);
      reduced->predecessors_by_topo_index[succ_topo_idx].push_back(curr_topo_idx);

      // Merge the successor's reachable set into ours.
      reachability_by_topo_index_[curr_topo_idx] |=
          reachability_by_topo_index_[succ_topo_idx];
    }
  }

  reduced_graph_ = std::move(reduced);
}

void ScheduleGraph::ComputeDominatorTree() {
  // Cache-aware: skip if dom tree already current.
  if (HasDominatorTree()) {
    return;
  }
  if (!HasReducedGraph()) {
    std::string msg = "ComputeDominatorTree called on " + ToString() +
                      " before ComputeTransitiveReductionAndReachability";
    report_fatal_error(llvm::StringRef(msg));
  }

  dom_tree_ = std::make_unique<DominatorTree>(
      DominatorTree::Build(*reduced_graph_));
}

std::string ScheduleGraph::DominatorTreeToString() const {
  if (!HasDominatorTree()) {
    return "DominatorTree not computed\n";
  }
  return dom_tree_->ToString(*this);
}

std::unique_ptr<ScheduleGraph>
ScheduleGraph::BuildFromSUnits(MutableArrayRef<SUnit> sunits,
                               const GCNSubtarget &st,
                               const MachineFunction &mf,
                               const LiveIntervals &lis,
                               const MachineRegisterInfo &mri,
                               const RegionInfo &region,
                               int latency_divisor) {
  auto graph = std::make_unique<ScheduleGraph>();

  // ReserveNodes provides 2x SUnits + 2 entry/exit nodes of
  // capacity — the 2x covers the worst-case subgraph-proxy count
  // (one proxy per real instruction). See ReserveNodes for why
  // this matters (preventing nodes_ reallocation that would
  // invalidate stored ScheduleNode * pointers in ScheduleEdges).
  graph->ReserveNodes(static_cast<int>(sunits.size()));

  // Slot indices at the region's top and bottom. Used by Phase 3 to
  // query LiveIntervals for live-in/live-out registers at the
  // boundaries.
  SlotIndex region_begin_idx = lis.getInstructionIndex(*region.Begin());
  SlotIndex region_end_idx = region.End() == region.GetBlock()->end()
      ? lis.getMBBEndIdx(region.GetBlock())
      : lis.getInstructionIndex(*region.End());

  // Local across phases: maps each non-boundary SUnit to the
  // ScheduleNode created for it in Phase 1. Used by Phase 2 (edges).
  DenseMap<const SUnit *, ScheduleNode *> sunit_to_node;

  graph->CreateLeafNodesFromSUnits(sunits, sunit_to_node);

  // latency_divisor is a caller-owned policy (see the header): each
  // data-dependency edge latency becomes ceil(latency / divisor),
  // floored at 1. divisor == 1 leaves latencies raw. The builder does
  // not consult any config here — callers decide (the scheduler from
  // the ScaleEdgeLatencies option, the analyzer from its lens).
  graph->AddEdgesBetweenLeafNodes(sunit_to_node, latency_divisor);
  graph->CreateEntryAndExitNodes(lis, mri, region_begin_idx, region_end_idx);
  // Compute topo and both critical-path directions before Phase 4 so
  // the ScheduleLengthTracker inside input_schedule_constructor_
  // satisfies its precondition (cp_from_exit must be available at
  // tracker construction) and so window-based length feasibility (a
  // future tracker addition reading cp_from_entry) is also ready.
  // ValidateAndComputeTopologicalOrder also enforces single-source /
  // single-sink (entry / exit must each be unique by construction).
  graph->ValidateAndComputeTopologicalOrder();
  graph->ComputeCriticalPaths();

  // Install the per-node register-operand info table. Must come
  // AFTER ValidateAndComputeTopologicalOrder (BuildForGraph indexes
  // entries by topo_index, which the topo order assigns) and BEFORE
  // PopulateInputScheduleConstructor (the input constructor builds
  // a GCNRegisterTracker, which now reads from this table).
  graph->SetNodeRegInfoTable(
      NodeRegInfoTable::BuildForGraph(*graph, mf, lis));

  graph->PopulateInputScheduleConstructor(st, mf, region);

  return graph;
}

// Helpers below are defined in BuildFromNodeSubset's call order; the
// orchestrator itself follows them. See ScheduleGraph.h for the
// member/subgraph-node vocabulary and the per-helper contracts.

// Return the members in the order they appear in parent_graph's input
// schedule: scan that schedule once, keeping the nodes in member_set.
// No sort — the scan yields input order directly. Order is taken from
// the schedule (not graph-local-id order) so a member that is itself
// parent_graph's entry/exit still lands consistently with the
// subgraph's edges.
static SmallVector<ScheduleNode *> MembersInParentInputOrder(
    const SmallPtrSetImpl<const ScheduleNode *> &member_set,
    const ScheduleGraph &parent_graph) {
  SmallVector<ScheduleNode *> ordered;
  ordered.reserve(member_set.size());
  for (const ScheduleNode *node :
       parent_graph.GetInputScheduleConstructor().GetScheduleOrder()) {
    if (member_set.contains(node)) {
      // const_cast is sound: parent_graph's nodes are non-const
      // objects — the caller passed non-const member pointers to the
      // same objects; GetScheduleOrder only hands back a const view.
      ordered.push_back(const_cast<ScheduleNode *>(node));
    }
  }
  return ordered;
}

void ScheduleGraph::CreateLeafNodesFromMembers(
    ArrayRef<ScheduleNode *> members,
    DenseMap<const ScheduleNode *, ScheduleNode *>
        &parent_member_to_subgraph_node,
    DenseMap<const ScheduleNode *, ScheduleNode *>
        &subgraph_node_to_parent_member) {
  for (ScheduleNode *member : members) {
    if (!member->IsSchedulingUnit()) {
      report_fatal_error(
          "ScheduleGraph::BuildFromNodeSubset: a member is a subgraph "
          "proxy; only scheduling-unit members are supported");
    }
    ScheduleNode &subgraph_node = EmplaceNode(
        member->GetSUnit(), member->GetDebugName().str(), this);
    parent_member_to_subgraph_node[member] = &subgraph_node;
    subgraph_node_to_parent_member[&subgraph_node] = member;
  }
}

void ScheduleGraph::CopyIntraSubgraphEdges(
    ArrayRef<ScheduleNode *> members,
    const DenseMap<const ScheduleNode *, ScheduleNode *>
        &parent_member_to_subgraph_node) {
  for (ScheduleNode *member : members) {
    ScheduleNode *from = parent_member_to_subgraph_node.lookup(member);
    for (const ScheduleEdge &edge : member->Successors()) {
      auto it = parent_member_to_subgraph_node.find(edge.node_);
      if (it == parent_member_to_subgraph_node.end()) {
        continue;  // boundary edge — the successor is not a member
      }
      AddEdge(from, it->second, edge.kind_, edge.latency_);
    }
  }
}

// Compute the subgraph's register boundary from parent_graph's
// NodeRegInfoTable. Fills live_in (lanes the subgraph consumes from
// outside) and live_out (lanes it produces for outside). Per-lane
// masks make the set arithmetic exact — each (vreg, lane) has a
// single definition.
static void ComputeSubgraphRegisterBoundary(
    const SmallPtrSetImpl<const ScheduleNode *> &member_set,
    const ScheduleGraph &parent_graph,
    DenseMap<unsigned, LaneBitmask> &live_in,
    DenseMap<unsigned, LaneBitmask> &live_out) {
  const NodeRegInfoTable &parent_reg_table =
      parent_graph.GetNodeRegInfoTable();

  // Union the lanes defined and used across all member nodes.
  DenseMap<unsigned, LaneBitmask> member_defs;
  DenseMap<unsigned, LaneBitmask> member_uses;
  for (const ScheduleNode *member : member_set) {
    const NodeRegInfo &info = parent_reg_table.GetForNode(member);
    for (const RegMask &def : info.defs) {
      member_defs[def.reg] |= def.mask;
    }
    for (const RegMask &use : info.uses) {
      member_uses[use.reg] |= use.mask;
    }
  }

  // Union the lanes used by every non-member node of parent_graph.
  DenseMap<unsigned, LaneBitmask> nonmember_uses;
  for (const ScheduleNode &node : parent_graph.Nodes()) {
    if (member_set.contains(&node)) {
      continue;
    }
    for (const RegMask &use : parent_reg_table.GetForNode(&node).uses) {
      nonmember_uses[use.reg] |= use.mask;
    }
  }

  // live_in = member-used lanes minus member-defined lanes. When no
  // member defines `reg`, member_defs.lookup(reg) is a default
  // LaneBitmask — i.e. none — so ~it is all lanes and the whole
  // use_mask survives: a register no member produces is fully live
  // into the subgraph.
  for (const auto &[reg, use_mask] : member_uses) {
    LaneBitmask mask = use_mask & ~member_defs.lookup(reg);
    if (mask.any()) {
      live_in[reg] = mask;
    }
  }
  // live_out = member-defined lanes that some non-member uses. When
  // no non-member uses `reg`, nonmember_uses.lookup(reg) is none, the
  // masked result is empty, and the register is correctly omitted.
  for (const auto &[reg, def_mask] : member_defs) {
    LaneBitmask mask = def_mask & nonmember_uses.lookup(reg);
    if (mask.any()) {
      live_out[reg] = mask;
    }
  }
}

void ScheduleGraph::CreateSubgraphBoundaryNodes(
    const DenseMap<unsigned, LaneBitmask> &live_in,
    const DenseMap<unsigned, LaneBitmask> &live_out) {
  ScheduleNode &entry =
      EmplaceNode(static_cast<SUnit *>(nullptr), "SubgraphEntry", this);
  ScheduleNode &exit =
      EmplaceNode(static_cast<SUnit *>(nullptr), "SubgraphExit", this);
  for (const auto &[reg, mask] : live_in) {
    entry.AddRegDef(Register(reg), mask);
  }
  for (const auto &[reg, mask] : live_out) {
    exit.AddRegUse(Register(reg), mask);
  }
  for (ScheduleNode &node : nodes_) {
    if (&node == &entry || &node == &exit) {
      continue;
    }
    if (node.NumPredecessors() == 0) {
      AddEdge(&entry, &node, ScheduleEdge::kArtificial);
    }
    if (node.NumSuccessors() == 0) {
      AddEdge(&node, &exit, ScheduleEdge::kArtificial);
    }
  }
}

void ScheduleGraph::BuildSubgraphRegInfoTable(
    const ScheduleGraph &parent_graph,
    const DenseMap<const ScheduleNode *, ScheduleNode *>
        &subgraph_node_to_parent_member) {
  const NodeRegInfoTable &parent_reg_table =
      parent_graph.GetNodeRegInfoTable();
  NodeRegInfoTable table(GetNumGraphLocalIds());
  for (const ScheduleNode &node : nodes_) {
    auto it = subgraph_node_to_parent_member.find(&node);
    if (it != subgraph_node_to_parent_member.end()) {
      // A member node: register info is intrinsic to the
      // instruction, so copy the parent's entry rather than
      // re-deriving it from the MachineInstr.
      table.SetEntry(&node, parent_reg_table.GetForNode(it->second));
    } else {
      // SubgraphEntry / SubgraphExit: the boundary register info
      // CreateSubgraphBoundaryNodes installed on the node.
      for (const RegWithLaneMask &def : node.RegDefs()) {
        table.AddDef(&node, def.reg.id(), def.mask);
      }
      for (const RegWithLaneMask &use : node.RegUses()) {
        table.AddUse(&node, use.reg.id(), use.mask);
      }
    }
  }
  SetNodeRegInfoTable(std::move(table));
}

void ScheduleGraph::PopulateSubgraphInputScheduleConstructor(
    const GCNSubtarget &st, const MachineFunction &mf, int member_count) {
  // nodes_ layout: [0 .. member_count-1] subgraph nodes in input
  // order, [member_count] SubgraphEntry, [member_count+1] SubgraphExit.
  ScheduleNode *entry = &nodes_[member_count];
  ScheduleNode *exit = &nodes_[member_count + 1];
  input_schedule_constructor_ =
      std::make_unique<ScheduleConstructor>(*this, st, mf);
  input_schedule_constructor_->Schedule(entry);
  for (int i = 0; i < member_count; ++i) {
    input_schedule_constructor_->Schedule(&nodes_[i]);
  }
  input_schedule_constructor_->Schedule(exit);
}

std::unique_ptr<ScheduleGraph> ScheduleGraph::BuildFromNodeSubset(
    ArrayRef<ScheduleNode *> members, const ScheduleGraph &parent_graph,
    const GCNSubtarget &st, const MachineFunction &mf,
    DenseMap<const ScheduleNode *, ScheduleNode *>
        &subgraph_node_to_parent_member) {
  if (members.empty()) {
    report_fatal_error(
        "ScheduleGraph::BuildFromNodeSubset: members is empty");
  }
  // Membership set, built once and shared by the helpers below.
  SmallPtrSet<const ScheduleNode *, 32> member_set;
  for (const ScheduleNode *member : members) {
    member_set.insert(member);
  }
  SmallVector<ScheduleNode *> ordered_members =
      MembersInParentInputOrder(member_set, parent_graph);

  auto graph = std::make_unique<ScheduleGraph>();
  // ReserveNodes over-reserves (3N+2) for subgraph-proxy headroom this
  // graph will not use — harmless, and the sanctioned way to size
  // nodes_ before emplacing into a graph whose edges store raw node
  // pointers (a reallocation would invalidate them).
  graph->ReserveNodes(static_cast<int>(ordered_members.size()));

  DenseMap<const ScheduleNode *, ScheduleNode *>
      parent_member_to_subgraph_node;
  graph->CreateLeafNodesFromMembers(ordered_members,
                                    parent_member_to_subgraph_node,
                                    subgraph_node_to_parent_member);
  graph->CopyIntraSubgraphEdges(ordered_members,
                                parent_member_to_subgraph_node);

  DenseMap<unsigned, LaneBitmask> live_in;
  DenseMap<unsigned, LaneBitmask> live_out;
  ComputeSubgraphRegisterBoundary(member_set, parent_graph, live_in,
                                  live_out);
  graph->CreateSubgraphBoundaryNodes(live_in, live_out);

  // The input ScheduleConstructor's trackers read the critical paths
  // and the register table, so finalize both before it; the critical
  // paths in turn require the topological order.
  graph->ValidateAndComputeTopologicalOrder();
  graph->ComputeCriticalPaths();
  graph->BuildSubgraphRegInfoTable(parent_graph,
                                   subgraph_node_to_parent_member);
  graph->PopulateSubgraphInputScheduleConstructor(
      st, mf, static_cast<int>(ordered_members.size()));

  return graph;
}

void ScheduleGraph::CreateLeafNodesFromSUnits(
    MutableArrayRef<SUnit> sunits,
    DenseMap<const SUnit *, ScheduleNode *> &sunit_to_node) {
  // SUnits are populated by LLVM's buildSchedGraph in MachineFunction
  // instruction order. Emplacing nodes_ in the same iteration order
  // is what makes Phase 4's iteration-of-nodes_ approach work — Phase
  // 4 verifies the resulting MF order at runtime via SUnit::NodeNum
  // and ScheduleNode::id_ monotonicity.
  //
  // Register defs/uses are extracted automatically by the
  // ScheduleNode constructor.
  for (SUnit &su : sunits) {
    if (su.isBoundaryNode()) {
      continue;
    }
    ScheduleNode &n = EmplaceNode(&su, this);
    sunit_to_node[&su] = &n;
  }
}

void ScheduleGraph::AddEdgesBetweenLeafNodes(
    const DenseMap<const SUnit *, ScheduleNode *> &sunit_to_node,
    int latency_divisor) {
  // Defensive: <=0 is meaningless. Treat as no-scaling.
  if (latency_divisor < 1) {
    latency_divisor = 1;
  }
  for (ScheduleNode &node : nodes_) {
    SUnit *su = node.GetSUnit();
    if (!su) {
      continue;
    }
    for (const SDep &sdep : su->Succs) {
      const SUnit *succ_su = sdep.getSUnit();
      if (succ_su->isBoundaryNode()) {
        continue;
      }

      ScheduleNode *succ_node = sunit_to_node.lookup(succ_su);
      if (!succ_node) {
        continue;
      }

      ScheduleEdge::Kind kind = MapSDepToEdgeKind(sdep);
      int latency = static_cast<int>(sdep.getLatency());
      if (latency_divisor > 1) {
        // Wave-visible latency = ceil(SIMD-cycles / wave-issue-rate).
        // Round-up (vs round-to-nearest) is the conservative choice:
        // the load takes a fixed number of SIMD cycles, so the wave
        // wakes up no earlier than ceil(latency / divisor) wave-slots
        // later. Round-down would let the consumer issue before the
        // load's result is ready. Floor of 1 preserves the
        // IssueWidth=1 ordering invariant.
        float scaled = static_cast<float>(latency) /
                       static_cast<float>(latency_divisor);
        latency = std::max(1, static_cast<int>(std::ceil(scaled)));
      }
      AddEdge(&node, succ_node, kind, latency);
    }
  }
}

void ScheduleGraph::PopulateInputScheduleConstructorByTopoOrderForTest(
    const GCNSubtarget &st, const MachineFunction &mf) {
  input_schedule_constructor_ =
      std::make_unique<ScheduleConstructor>(*this, st, mf);
  for (ScheduleNode *node : topo_order_) {
    input_schedule_constructor_->Schedule(node);
  }
}

void ScheduleGraph::PopulateInputScheduleConstructor(
    const GCNSubtarget &st, const MachineFunction &mf,
    const RegionInfo &region) {
  input_schedule_constructor_ =
      std::make_unique<ScheduleConstructor>(*this, st, mf);

  // nodes_ layout after Phase 1 + Phase 3:
  //   [0 .. N-1] : real-instruction leaves (Phase 1 emplacement order
  //                = current MF order)
  //   [N]        : Entry  (Phase 3, null SUnit, has kArtificial edges
  //                TO root leaves — root leaves need entry scheduled
  //                first)
  //   [N+1]      : Exit   (Phase 3, null SUnit, has kArtificial edges
  //                FROM tail leaves — exit becomes ready once all its
  //                predecessor leaves are scheduled)
  //
  // For IsDone() (which requires every node in nodes_ to be
  // scheduled), the order is: Entry, then leaves in MF order, then
  // Exit.
  size_t n = nodes_.size();
  if (n < 2) {
    report_fatal_error("PopulateInputScheduleConstructor: nodes_.size() < 2 "
                       "— Phase 3 (entry/exit) did not run");
  }
  ScheduleNode *entry_node = &nodes_[n - 2];
  ScheduleNode *exit_node = &nodes_[n - 1];
  if (entry_node->GetSUnit() || exit_node->GetSUnit()) {
    report_fatal_error("PopulateInputScheduleConstructor: expected the last "
                       "two nodes_ entries to be Phase 3 synthetic "
                       "entry/exit (null SUnit)");
  }

  // Schedule entry to release the root leaves.
  input_schedule_constructor_->Schedule(entry_node);

  // Schedule leaves in MF order (Phase 1 emplacement order). Two
  // monotonicity checks verify the ordering at runtime against the
  // previous leaf node:
  //
  //   1. ScheduleNode::id_ — assigned from a global counter at
  //      construction. Consecutive Phase 1 emplacements get
  //      consecutive ids, so monotonicity confirms we're iterating
  //      nodes in their emplacement order.
  //
  //   2. SUnit::NodeNum — assigned by buildSchedGraph at the *current*
  //      call site. SUnits are transient: every WithRegionGraph call
  //      rebuilds them fresh, with NodeNums numbered 0..N-1 in
  //      current MF order (so any prior pass's reordering is already
  //      reflected). Monotonicity confirms Phase 1's emplacement
  //      order matches that current MF order — i.e., Phase 1 didn't
  //      sort/shuffle/filter sunits before emplacing.
  //
  // Schedule()'s ready-list check is a third, separate guard against
  // non-topological orderings (which would indicate an LLVM bug in
  // buildSchedGraph or a Phase 2 edge bug).
  const ScheduleNode *prev_node = nullptr;
  for (ScheduleNode &node : nodes_) {
    SUnit *su = node.GetSUnit();
    if (!su) {
      continue;
    }
    if (prev_node) {
      if (node.GetId() <= prev_node->GetId()) {
        report_fatal_error("PopulateInputScheduleConstructor: ScheduleNode "
                           "ids not strictly increasing — nodes_ is not in "
                           "Phase 1 emplacement order");
      }
      if (su->NodeNum <= prev_node->GetSUnit()->NodeNum) {
        report_fatal_error("PopulateInputScheduleConstructor: SUnit NodeNums "
                           "not strictly increasing — Phase 1 emplaced nodes "
                           "out of MachineFunction order");
      }
    }
    prev_node = &node;
    input_schedule_constructor_->Schedule(&node);
  }

  // Schedule exit last; all its predecessor leaves are now scheduled,
  // so its strong-pred count is 0 and it's in the ready list.
  input_schedule_constructor_->Schedule(exit_node);

  VerifyInputScheduleMatchesMFOrder(region);
}

void ScheduleGraph::VerifyInputScheduleMatchesMFOrder(
    const RegionInfo &region) const {
  // Walk the schedule we built (skipping entry/exit) and the
  // region's MF iterator range (skipping debug/pseudo MIs, which
  // have no SUnit) in lockstep, comparing MachineInstr pointers.
  MachineBasicBlock::iterator mf_iter = region.Begin();
  MachineBasicBlock::iterator mf_end = region.End();
  int position = 0;
  for (const ScheduleNode *node :
       input_schedule_constructor_->GetScheduleOrder()) {
    SUnit *su = node->GetSUnit();
    if (!su) {
      continue;  // entry / exit
    }

    while (mf_iter != mf_end && mf_iter->isDebugOrPseudoInstr()) {
      ++mf_iter;
    }
    if (mf_iter == mf_end) {
      report_fatal_error("VerifyInputScheduleMatchesMFOrder: schedule has "
                         "more leaves than MF iterator range");
    }
    if (su->getInstr() != &*mf_iter) {
      report_fatal_error("VerifyInputScheduleMatchesMFOrder: schedule order "
                         "diverges from MF order at leaf position " +
                         Twine(position));
    }
    ++mf_iter;
    ++position;
  }

  while (mf_iter != mf_end && mf_iter->isDebugOrPseudoInstr()) {
    ++mf_iter;
  }
  if (mf_iter != mf_end) {
    report_fatal_error("VerifyInputScheduleMatchesMFOrder: MF iterator range "
                       "has more real instructions than the schedule");
  }
}

void ScheduleGraph::CreateEntryAndExitNodes(const LiveIntervals &lis,
                                            const MachineRegisterInfo &mri,
                                            SlotIndex region_begin_idx,
                                            SlotIndex region_end_idx) {
  ScheduleNode &entry_node =
      EmplaceNode(static_cast<SUnit *>(nullptr), "Entry", this);
  ScheduleNode &exit_node =
      EmplaceNode(static_cast<SUnit *>(nullptr), "Exit", this);

  // Wire entry to all root nodes, exit from all leaf nodes.
  for (ScheduleNode &node : nodes_) {
    if (&node == &entry_node || &node == &exit_node) {
      continue;
    }
    if (node.NumPredecessors() == 0) {
      AddEdge(&entry_node, &node, ScheduleEdge::kArtificial);
    }
    if (node.NumSuccessors() == 0) {
      // TODO: LLVM's buildSchedGraph adds an artificial edge from
      // high-latency leaf instructions to ExitSU with latency =
      // SU->Latency - 1 (ScheduleDAGInstrs.cpp, line 877). This
      // ensures cross-region heuristics account for results that
      // won't be ready for many cycles (e.g., a VMEM load with
      // latency 80 at the end of a region). We currently use
      // latency 0, which is fine for within-region schedule length
      // but may underestimate costs for cross-region analysis.
      AddEdge(&node, &exit_node, ScheduleEdge::kArtificial);
    }
  }

  // Populate register defs/uses from LiveIntervals with per-lane
  // accuracy. getLiveLaneMask queries sub-range liveness to determine
  // exactly which lanes are live at the given slot index, rather than
  // treating the whole register as live whenever any lane is.
  for (int i = 0, num_virt_regs = mri.getNumVirtRegs(); i < num_virt_regs;
       ++i) {
    Register reg = Register::index2VirtReg(i);
    if (!lis.hasInterval(reg)) {
      continue;
    }

    LaneBitmask live_in_mask =
        getLiveLaneMask(reg, region_begin_idx, lis, mri);
    if (live_in_mask.any()) {
      entry_node.AddRegDef(reg, live_in_mask);
    }

    LaneBitmask live_out_mask =
        getLiveLaneMask(reg, region_end_idx, lis, mri);
    if (live_out_mask.any()) {
      exit_node.AddRegUse(reg, live_out_mask);
    }
  }
}

std::unique_ptr<ScheduleGraph> ScheduleGraph::BuildTestDAG() {
  auto graph = std::make_unique<ScheduleGraph>();

  // 7 nodes: A, C, D, E, F, G, H (leaves wrapping nullptr since we
  // have no real SUnits).
  graph->ReserveNodes(7);
  ScheduleNode &a = graph->EmplaceNode(nullptr, "A", graph.get());
  ScheduleNode &c = graph->EmplaceNode(nullptr, "C", graph.get());
  ScheduleNode &d = graph->EmplaceNode(nullptr, "D", graph.get());
  ScheduleNode &e = graph->EmplaceNode(nullptr, "E", graph.get());
  ScheduleNode &f = graph->EmplaceNode(nullptr, "F", graph.get());
  ScheduleNode &g = graph->EmplaceNode(nullptr, "G", graph.get());
  ScheduleNode &h = graph->EmplaceNode(nullptr, "H", graph.get());

  // Edges (all kData). Latencies are chosen so cp_from_exit exercises
  // 3-way max at A (1+5=6 vs 3+4=7 vs 2+7=9 → 9), 2-way max at C
  // (1+4=5 vs 4+3=7 → 7), and a unique critical path A→C→E→F→G of
  // length 9. See BuildTestDAG docstring for expected cp values.
  graph->AddEdge(&a, &h, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&a, &c, ScheduleEdge::kData, /*latency=*/2);
  graph->AddEdge(&a, &d, ScheduleEdge::kData, /*latency=*/3);
  graph->AddEdge(&c, &d, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&c, &e, ScheduleEdge::kData, /*latency=*/4);
  graph->AddEdge(&d, &f, ScheduleEdge::kData, /*latency=*/2);
  graph->AddEdge(&e, &f, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&h, &g, ScheduleEdge::kData, /*latency=*/5);
  graph->AddEdge(&f, &g, ScheduleEdge::kData, /*latency=*/2);

  // Empty reg-info table sized for the graph so any tracker
  // constructed over this DAG (test mode or otherwise) can hold a
  // valid pointer to the table. All entries default to no defs / no
  // uses, which matches the "synthetic node with no MachineInstr"
  // case the tracker already handles.
  graph->SetNodeRegInfoTable(NodeRegInfoTable(graph->GetNumGraphLocalIds()));

  return graph;
}

std::unique_ptr<ScheduleGraph> ScheduleGraph::BuildLengthLowerBoundTestDAG() {
  auto graph = std::make_unique<ScheduleGraph>();

  graph->ReserveNodes(6);
  ScheduleNode &n0 = graph->EmplaceNode(nullptr, "N0", graph.get());
  ScheduleNode &n1 = graph->EmplaceNode(nullptr, "N1", graph.get());
  ScheduleNode &n2 = graph->EmplaceNode(nullptr, "N2", graph.get());
  ScheduleNode &n3 = graph->EmplaceNode(nullptr, "N3", graph.get());
  ScheduleNode &n4 = graph->EmplaceNode(nullptr, "N4", graph.get());
  ScheduleNode &n5 = graph->EmplaceNode(nullptr, "N5", graph.get());

  // Two parallel chains converging at N5. N0->N1->N3->N5 is the
  // heavy chain; N0->N2->N4->N5 is lighter. The latency-5 edge
  // N1->N3 and the latency-2 edge N4->N5 force bubbles during
  // forward scheduling, which makes the LB transition multiple
  // times.
  graph->AddEdge(&n0, &n1, ScheduleEdge::kData, /*latency=*/3);
  graph->AddEdge(&n0, &n2, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&n1, &n3, ScheduleEdge::kData, /*latency=*/5);
  graph->AddEdge(&n2, &n4, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&n3, &n5, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&n4, &n5, ScheduleEdge::kData, /*latency=*/2);

  graph->SetNodeRegInfoTable(NodeRegInfoTable(graph->GetNumGraphLocalIds()));

  return graph;
}

std::unique_ptr<ScheduleGraph> ScheduleGraph::BuildContiguityTestDAG() {
  auto graph = std::make_unique<ScheduleGraph>();

  // 6 nodes: A, X, Y, B, C, E. E is a synthetic exit so the graph
  // has exactly one source (A) and one sink (E) —
  // ValidateAndComputeTopologicalOrder requires this. Emplacement
  // order does not affect the test outcome.
  graph->ReserveNodes(6);
  ScheduleNode &a = graph->EmplaceNode(nullptr, "A", graph.get());
  ScheduleNode &x = graph->EmplaceNode(nullptr, "X", graph.get());
  ScheduleNode &y = graph->EmplaceNode(nullptr, "Y", graph.get());
  ScheduleNode &b = graph->EmplaceNode(nullptr, "B", graph.get());
  ScheduleNode &c = graph->EmplaceNode(nullptr, "C", graph.get());
  ScheduleNode &e = graph->EmplaceNode(nullptr, "E", graph.get());

  // X, Y depend only on A (latency 0 — they become ready the cycle
  // after A is scheduled). The chain A→B→C carries latency 5
  // between each link. All non-chain leaves funnel into E with
  // latency 0 so the graph has a single sink.
  graph->AddEdge(&a, &x, ScheduleEdge::kData, /*latency=*/0);
  graph->AddEdge(&a, &y, ScheduleEdge::kData, /*latency=*/0);
  graph->AddEdge(&a, &b, ScheduleEdge::kData, /*latency=*/5);
  graph->AddEdge(&b, &c, ScheduleEdge::kData, /*latency=*/5);
  graph->AddEdge(&x, &e, ScheduleEdge::kData, /*latency=*/0);
  graph->AddEdge(&y, &e, ScheduleEdge::kData, /*latency=*/0);
  graph->AddEdge(&c, &e, ScheduleEdge::kData, /*latency=*/0);

  graph->SetNodeRegInfoTable(NodeRegInfoTable(graph->GetNumGraphLocalIds()));

  return graph;
}

std::unique_ptr<ScheduleGraph>
ScheduleGraph::BuildSubgraphFormationTestDAG() {
  auto graph = std::make_unique<ScheduleGraph>();

  // 10 nodes: A (source), P, Q, P2, Q2, S (splitter), D, E, F, Exit
  // (sink). Emplacement order is also the natural Kahn's-FIFO topo
  // order seed, but the test asserts on node identity (via pointer
  // lookup) rather than on topo index, so the emplacement order is
  // not load-bearing.
  graph->ReserveNodes(10);
  ScheduleNode &a = graph->EmplaceNode(nullptr, "A", graph.get());
  ScheduleNode &p = graph->EmplaceNode(nullptr, "P", graph.get());
  ScheduleNode &q = graph->EmplaceNode(nullptr, "Q", graph.get());
  ScheduleNode &p2 = graph->EmplaceNode(nullptr, "P2", graph.get());
  ScheduleNode &q2 = graph->EmplaceNode(nullptr, "Q2", graph.get());
  ScheduleNode &s = graph->EmplaceNode(nullptr, "S", graph.get());
  ScheduleNode &d = graph->EmplaceNode(nullptr, "D", graph.get());
  ScheduleNode &e = graph->EmplaceNode(nullptr, "E", graph.get());
  ScheduleNode &f = graph->EmplaceNode(nullptr, "F", graph.get());
  ScheduleNode &exit = graph->EmplaceNode(nullptr, "Exit", graph.get());

  // S→D has latency 50, exceeding the default formation threshold
  // of 32 — that single edge makes S the unique splitter on this DAG.
  // All other edges latency 1 so they fall well below threshold.
  graph->AddEdge(&a, &p, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&a, &q, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&p, &p2, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&p, &s, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&q, &q2, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&q, &s, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&s, &d, ScheduleEdge::kData, /*latency=*/50);
  graph->AddEdge(&s, &e, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&d, &f, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&e, &f, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&p2, &exit, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&q2, &exit, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&f, &exit, ScheduleEdge::kData, /*latency=*/1);

  graph->SetNodeRegInfoTable(NodeRegInfoTable(graph->GetNumGraphLocalIds()));

  return graph;
}

std::unique_ptr<ScheduleGraph> ScheduleGraph::BuildHistoryPruneTestDAG() {
  auto graph = std::make_unique<ScheduleGraph>();

  // 4 nodes: A, B, C, D. Diamond with both A→{B,C} edges weighted at
  // latency 5; both {B,C}→D edges at latency 1. See header docstring
  // for the design rationale (floor < optimum, two orderings of
  // {A,B,C} reach the same dominating state).
  graph->ReserveNodes(4);
  ScheduleNode &a = graph->EmplaceNode(nullptr, "A", graph.get());
  ScheduleNode &b = graph->EmplaceNode(nullptr, "B", graph.get());
  ScheduleNode &c = graph->EmplaceNode(nullptr, "C", graph.get());
  ScheduleNode &d = graph->EmplaceNode(nullptr, "D", graph.get());

  graph->AddEdge(&a, &b, ScheduleEdge::kData, /*latency=*/5);
  graph->AddEdge(&a, &c, ScheduleEdge::kData, /*latency=*/5);
  graph->AddEdge(&b, &d, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&c, &d, ScheduleEdge::kData, /*latency=*/1);

  graph->SetNodeRegInfoTable(NodeRegInfoTable(graph->GetNumGraphLocalIds()));

  return graph;
}

std::unique_ptr<ScheduleGraph> ScheduleGraph::BuildPressureHistoryPruneTestDAG() {
  auto graph = std::make_unique<ScheduleGraph>();

  // 6 nodes A, B, C, D, E, F. Two parallel two-step chains between
  // A and F: A→B→D→F and A→C→E→F. See header for the rationale.
  graph->ReserveNodes(6);
  ScheduleNode &a = graph->EmplaceNode(nullptr, "A", graph.get());
  ScheduleNode &b = graph->EmplaceNode(nullptr, "B", graph.get());
  ScheduleNode &c = graph->EmplaceNode(nullptr, "C", graph.get());
  ScheduleNode &d = graph->EmplaceNode(nullptr, "D", graph.get());
  ScheduleNode &e = graph->EmplaceNode(nullptr, "E", graph.get());
  ScheduleNode &f = graph->EmplaceNode(nullptr, "F", graph.get());

  graph->AddEdge(&a, &b, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&a, &c, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&b, &d, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&c, &e, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&d, &f, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&e, &f, ScheduleEdge::kData, /*latency=*/1);

  graph->SetNodeRegInfoTable(NodeRegInfoTable(graph->GetNumGraphLocalIds()));

  return graph;
}

std::unique_ptr<ScheduleGraph> ScheduleGraph::BuildAreaTiebreakTestDAG() {
  auto graph = std::make_unique<ScheduleGraph>();

  // 6 nodes A→B→M→{X,Y}→T. The A→B chain raises pressure to the peak;
  // M gates the frees X,Y, which drop below the peak in either order;
  // T is the single sink. See header for the rationale. Created in
  // topo order so creation index == topo index (the test deltas are
  // indexed by topo index); X before Y so a peak-only DFS explores the
  // lower-area order first.
  graph->ReserveNodes(6);
  ScheduleNode &a = graph->EmplaceNode(nullptr, "A", graph.get());
  ScheduleNode &b = graph->EmplaceNode(nullptr, "B", graph.get());
  ScheduleNode &m = graph->EmplaceNode(nullptr, "M", graph.get());
  ScheduleNode &x = graph->EmplaceNode(nullptr, "X", graph.get());
  ScheduleNode &y = graph->EmplaceNode(nullptr, "Y", graph.get());
  ScheduleNode &t = graph->EmplaceNode(nullptr, "T", graph.get());

  graph->AddEdge(&a, &b, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&b, &m, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&m, &x, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&m, &y, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&x, &t, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&y, &t, ScheduleEdge::kData, /*latency=*/1);

  graph->SetNodeRegInfoTable(NodeRegInfoTable(graph->GetNumGraphLocalIds()));

  return graph;
}

std::unique_ptr<ScheduleGraph> ScheduleGraph::BuildBfsDpWideTestDAG() {
  auto graph = std::make_unique<ScheduleGraph>();

  // Nodes listed in topo order so creation index == topo index (no
  // intra-level reordering needed; the test deltas are indexed by
  // topo index).
  graph->ReserveNodes(16);
  ScheduleNode &a = graph->EmplaceNode(nullptr, "A", graph.get());
  ScheduleNode &b = graph->EmplaceNode(nullptr, "B", graph.get());
  ScheduleNode &c = graph->EmplaceNode(nullptr, "C", graph.get());
  ScheduleNode &d = graph->EmplaceNode(nullptr, "D", graph.get());
  ScheduleNode &e = graph->EmplaceNode(nullptr, "E", graph.get());
  ScheduleNode &m = graph->EmplaceNode(nullptr, "M", graph.get());
  ScheduleNode &o = graph->EmplaceNode(nullptr, "O", graph.get());
  ScheduleNode &f = graph->EmplaceNode(nullptr, "F", graph.get());
  ScheduleNode &g = graph->EmplaceNode(nullptr, "G", graph.get());
  ScheduleNode &h = graph->EmplaceNode(nullptr, "H", graph.get());
  ScheduleNode &i = graph->EmplaceNode(nullptr, "I", graph.get());
  ScheduleNode &n = graph->EmplaceNode(nullptr, "N", graph.get());
  ScheduleNode &p = graph->EmplaceNode(nullptr, "P", graph.get());
  ScheduleNode &j = graph->EmplaceNode(nullptr, "J", graph.get());
  ScheduleNode &k = graph->EmplaceNode(nullptr, "K", graph.get());
  ScheduleNode &l = graph->EmplaceNode(nullptr, "L", graph.get());

  // A → 6 level-1 children.
  graph->AddEdge(&a, &b, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&a, &c, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&a, &d, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&a, &e, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&a, &m, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&a, &o, ScheduleEdge::kData, /*latency=*/1);

  // Cross-edge cluster: F:{B,C}, G:{C,D}, H:{D,E}, I:{E}.
  graph->AddEdge(&b, &f, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&c, &f, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&c, &g, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&d, &g, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&d, &h, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&e, &h, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&e, &i, ScheduleEdge::kData, /*latency=*/1);

  // Pure-parallel chains.
  graph->AddEdge(&m, &n, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&o, &p, ScheduleEdge::kData, /*latency=*/1);

  // Intermediate joins inside the cluster.
  graph->AddEdge(&f, &j, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&g, &j, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&h, &k, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&i, &k, ScheduleEdge::kData, /*latency=*/1);

  // 4-way sink: J, K, N, P all feed L.
  graph->AddEdge(&j, &l, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&k, &l, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&n, &l, ScheduleEdge::kData, /*latency=*/1);
  graph->AddEdge(&p, &l, ScheduleEdge::kData, /*latency=*/1);

  graph->SetNodeRegInfoTable(NodeRegInfoTable(graph->GetNumGraphLocalIds()));
  return graph;
}

std::unique_ptr<ScheduleGraph> ScheduleGraph::BuildTestDAGWithCycle() {
  auto graph = std::make_unique<ScheduleGraph>();

  // 3 nodes with a cycle: A → B → C → B
  graph->ReserveNodes(3);
  ScheduleNode &a = graph->EmplaceNode(nullptr, "A", graph.get());
  ScheduleNode &b = graph->EmplaceNode(nullptr, "B", graph.get());
  ScheduleNode &c = graph->EmplaceNode(nullptr, "C", graph.get());

  graph->AddEdge(&a, &b, ScheduleEdge::kData);
  graph->AddEdge(&b, &c, ScheduleEdge::kData);
  graph->AddEdge(&c, &b, ScheduleEdge::kData);  // cycle: C → B

  graph->SetNodeRegInfoTable(NodeRegInfoTable(graph->GetNumGraphLocalIds()));

  return graph;
}
