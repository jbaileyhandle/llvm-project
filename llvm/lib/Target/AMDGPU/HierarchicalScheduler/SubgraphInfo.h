//===- SubgraphInfo.h - Subgraph metadata record ----------------*- C++ -*-===//
//
// One SubgraphInfo holds the structural metadata for a single
// subgraph: its member ScheduleNodes, a debug name, the external
// predecessors and successors of the member set, and a backpointer
// to the proxy ScheduleNode that represents the subgraph in the
// surrounding ScheduleGraph.
//
// The constructor takes the member list and a name; it walks the
// members once to populate ext_predecessors and ext_successors. No
// graph mutation happens here. The proxy backpointer is filled in
// later by InsertSubgraphProxies when the proxy node is emplaced in
// the graph.
//
// Ownership: the SubgraphInfo is owned by the START proxy ScheduleNode
// (held inside that node's std::variant content_ as a
// std::unique_ptr<SubgraphInfo>). The END proxy holds a raw
// SubgraphInfo* back-reference (in its variant). Both proxy nodes
// are owned by the ScheduleGraph; therefore the SubgraphInfo's
// lifetime is the START proxy's lifetime, which is the graph's
// lifetime. Callers construct a SubgraphInfo via std::make_unique
// and transfer ownership to the start proxy at emplace time.
//
// See AMDGPUClusteringDesign.md (Approach B) for how SubgraphInfo
// and the proxy node fit into the single-graph-hybrid model.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHINFO_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHINFO_H

#include "GCNRegPressure.h"
#include "SearchTerminationCause.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include <optional>
#include <string>

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleNode;

/// The schedule ScheduleSubgraph chose for a subgraph, with snapshot
/// register-pressure metadata. Stored on
/// SubgraphInfo::schedule_result. Snapshot values rather than the
/// search's ScheduleConstructor: that constructor is bound to the
/// extracted subgraph graph, which ScheduleSubgraph frees on return.
struct SubgraphScheduleResult {
  /// The chosen schedule, as parent-graph member nodes in scheduled
  /// order.
  SmallVector<ScheduleNode *, 32> order;

  /// Peak register pressure of `order` scheduled in isolation — the
  /// subgraph's modeled register boundary, pass-through registers
  /// excluded (see ScheduleGraph::BuildFromNodeSubset).
  GCNRegPressure peak_pressure;

  /// How the search that produced `order` ended.
  SearchTerminationCause termination_cause;
};

struct SubgraphInfo {
  /// Process-unique id, assigned at construction from the shared
  /// scheduler id counter (GetAndIncrementScheduleId) — the same space
  /// ScheduleNode and ScheduleGraph draw from, so a subgraph id never
  /// collides with a node or graph id. Stable identity independent of
  /// position in any list.
  int64_t id;

  /// Members of this subgraph. Stable set; used as the membership
  /// predicate when computing ext_predecessors / ext_successors.
  SmallVector<ScheduleNode *, 32> members;

  /// Human-readable name (used in diagnostics / ToString).
  std::string debug_name;

  /// Predecessors of any member that are NOT themselves members.
  /// Computed by the constructor; deduped; preserves first-seen
  /// order across `members` × each member's predecessor list.
  SmallVector<ScheduleNode *, 16> ext_predecessors;

  /// Successors of any member that are NOT themselves members.
  /// Same shape as ext_predecessors.
  SmallVector<ScheduleNode *, 16> ext_successors;

  /// Members with no in-subgraph predecessors — i.e., members that
  /// would become ready immediately upon scheduling the start proxy
  /// (since their only predecessors are external, all of which are
  /// scheduled by the time the proxy is ready). These are the
  /// candidates for the FIRST move within this subgraph once it's
  /// entered.
  ///
  /// Used by IlpTracker::CloseCostForNode to score a start proxy as
  /// "min over initial_members of CloseCostForNode(member)" — the
  /// cheapest opening move if the subgraph is entered now.
  ///
  /// Computed by the constructor in the same pass that builds
  /// ext_predecessors / ext_successors. Order is stable (matches
  /// first-seen order in `members`).
  SmallVector<ScheduleNode *, 8> initial_members;

  /// The START proxy node that represents the entry into this
  /// subgraph in the outer scheduling graph. Pushes a scope on the
  /// scope stack when scheduled. Null until InsertSubgraphProxies
  /// runs. Names "subgraph_proxy" for backward compatibility with
  /// callers that predate the start/end split; semantically this
  /// is the start proxy.
  ScheduleNode *subgraph_proxy = nullptr;

  /// The END proxy node that marks the exit from this subgraph.
  /// Has every member of the subgraph as a strong predecessor (so
  /// it is only ready after every member has been scheduled), and
  /// every external successor of the subgraph as a successor.
  /// Pops the scope when scheduled. Null until InsertSubgraphProxies
  /// runs.
  ScheduleNode *end_proxy = nullptr;

  /// The schedule chosen for this subgraph, set by ScheduleSubgraph.
  /// nullopt until ScheduleSubgraph has run.
  std::optional<SubgraphScheduleResult> schedule_result;

  /// Build from a member list and a debug name. Walks `members` once
  /// to compute ext_predecessors (predecessors of any member that
  /// aren't members) and ext_successors (successors of any member
  /// that aren't members).
  SubgraphInfo(ArrayRef<ScheduleNode *> members, StringRef debug_name);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHINFO_H
