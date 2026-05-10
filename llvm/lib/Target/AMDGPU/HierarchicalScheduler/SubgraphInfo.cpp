//===- SubgraphInfo.cpp - Subgraph metadata record ------------------------===//
//
// Constructor walks members once to compute the external-predecessor
// and external-successor sets.
//
//===----------------------------------------------------------------------===//

#include "SubgraphInfo.h"
#include "ScheduleGraph.h"
#include "llvm/ADT/SmallPtrSet.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

SubgraphInfo::SubgraphInfo(ArrayRef<ScheduleNode *> members_in,
                           StringRef debug_name_in)
    : members(members_in.begin(), members_in.end()),
      debug_name(debug_name_in.str()) {
  // Membership set for fast O(1) lookup during the boundary walk.
  // SmallPtrSet is LLVM's pointer-specialized hash set: N inline
  // slots so small sets don't heap-allocate; pointer identity (no
  // hashing of dereferenced content); faster than DenseSet for
  // pointer keys.
  SmallPtrSet<ScheduleNode *, 32> member_set(members.begin(),
                                             members.end());

  // Dedupe ext_predecessors / ext_successors as we discover them —
  // the same external node may have edges to multiple members. We
  // use the standard LLVM "set + parallel vector" dedup pattern:
  //
  //   - The SmallPtrSet (`*_seen`) tracks what we've already
  //     recorded. SmallPtrSet::insert returns
  //     std::pair<iterator, bool>; the bool is true on first
  //     insertion, false if already present. We use that bool to
  //     gate the push_back.
  //
  //   - The SmallVector (`ext_predecessors` / `ext_successors`) is
  //     the deduped output. We carry it alongside the set, rather
  //     than iterating the set later, because SmallPtrSet's
  //     iteration order is NOT stable — it depends on hash-table
  //     layout, which can vary across rebuilds, LLVM versions, and
  //     pointer addresses. Compiler determinism (same input →
  //     byte-identical output) requires that downstream code that
  //     iterates these sets (InsertSubgraphProxies adding
  //     artificial edges, ValidateSubgraphs recomputing for
  //     comparison) sees the same order on every run. The vector
  //     preserves first-seen order, which is determined by
  //     `members` iteration order × each member's edge list order
  //     — both stable.
  SmallPtrSet<ScheduleNode *, 16> ext_predecessors_seen;
  SmallPtrSet<ScheduleNode *, 16> ext_successors_seen;

  for (ScheduleNode *m : members) {
    bool has_in_subgraph_pred = false;
    for (const ScheduleEdge &edge : m->Predecessors()) {
      if (member_set.contains(edge.node_)) {
        // In-subgraph predecessor: this member is not initial.
        has_in_subgraph_pred = true;
      } else if (ext_predecessors_seen.insert(edge.node_).second) {
        ext_predecessors.push_back(edge.node_);
      }
    }
    if (!has_in_subgraph_pred) {
      initial_members.push_back(m);
    }
    for (const ScheduleEdge &edge : m->Successors()) {
      if (!member_set.contains(edge.node_) &&
          ext_successors_seen.insert(edge.node_).second) {
        ext_successors.push_back(edge.node_);
      }
    }
  }
}
