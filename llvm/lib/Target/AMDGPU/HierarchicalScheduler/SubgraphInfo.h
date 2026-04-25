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
// Ownership: the SubgraphInfo is owned by the proxy ScheduleNode
// (held inside the node's std::variant content_ as a
// std::unique_ptr<SubgraphInfo>). The proxy node is owned by the
// ScheduleGraph; therefore the SubgraphInfo's lifetime is the proxy
// node's lifetime, which is the graph's lifetime. Callers construct
// a SubgraphInfo via std::make_unique and transfer ownership to the
// proxy at emplace time.
//
// See AMDGPUClusteringDesign.md (Approach B) for how SubgraphInfo
// and the proxy node fit into the single-graph-hybrid model.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHINFO_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHINFO_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include <string>

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleNode;

struct SubgraphInfo {
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

  /// The proxy node that represents this subgraph in the outer
  /// scheduling graph. Null until InsertSubgraphProxies runs.
  ScheduleNode *subgraph_proxy = nullptr;

  /// Build from a member list and a debug name. Walks `members` once
  /// to compute ext_predecessors (predecessors of any member that
  /// aren't members) and ext_successors (successors of any member
  /// that aren't members).
  SubgraphInfo(ArrayRef<ScheduleNode *> members, StringRef debug_name);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHINFO_H
