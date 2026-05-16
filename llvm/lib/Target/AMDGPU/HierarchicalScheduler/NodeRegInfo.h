//===- NodeRegInfo.h - Pre-extracted per-node register info ----*- C++ -*-===//
//
// jbaile
//
// Standalone container for the pre-extracted per-node register-operand
// info that drives GCNRegisterTracker. Holds one NodeRegInfo per
// scheduling-unit node, indexed by topo_index, plus a builder that
// computes the table once per graph from MachineInstrs.
//
// Lives in its own header so consumers that need only the table types
// (notably ScheduleGraph, which owns the per-region table) don't have
// to pull in the full GCNRegisterTracker header. Tracker construction
// then receives a const pointer to the graph's table instead of
// computing one itself, and NoHistoryClone in the tracker can share
// the table via pointer copy.
//
// Why a container class rather than a raw vector:
//   - Tests can construct an empty table for n nodes and populate it
//     with AddDef / AddUse / SetEntry without going through the
//     MachineInstr-driven production builder. Useful for staged DAGs
//     where the desired def/use pattern is what you want to control.
//   - Dedup logic (sub-register defs of the same vreg merge their
//     lane masks) is implemented once on the table's mutators rather
//     than duplicated in callers.
//   - Ownership is explicit: the table is a value, not a vector
//     somewhere on someone else's class.
//
// Every accessor and mutator that takes a topo_index also has a
// ScheduleNode* overload that forwards through node->GetTopoIndex().
// Callers usually have the node in hand and shouldn't have to remember
// to call GetTopoIndex().
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_NODEREGINFO_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_NODEREGINFO_H

#include "ScheduleGraph.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/LaneBitmask.h"
#include <vector>

namespace llvm {

class LiveIntervals;
class MachineFunction;

namespace hierarchical_scheduler {

/// A register + lane mask pair. Used for pre-extracted defs and uses.
struct RegMask {
  unsigned reg;
  LaneBitmask mask;
};

/// Pre-extracted register info for one node. Defs are deduplicated
/// per register (sub-register defs of the same register have their
/// masks ORed). Dead defs (isDead()) are excluded. Uses are similarly
/// deduplicated.
struct NodeRegInfo {
  SmallVector<RegMask, 4> defs;
  SmallVector<RegMask, 4> uses;
};

/// Per-region table of NodeRegInfo, indexed by graph_local_id.
/// graph_local_id is chosen over topo_index because it's stable —
/// assigned at node construction from the top-level graph's
/// dense counter and never reassigned. Topo order can be
/// recomputed (e.g., after InsertSubgraphProxies adds proxy
/// nodes) without invalidating table entries.
///
/// Built by BuildForGraph in production. Tests stage entries
/// manually via the empty constructor + AddDef / AddUse / SetEntry.
/// After post-construction graph mutations that add nodes, callers
/// must EnsureSize the table to graph.Size() so the new
/// graph_local_ids have valid slots.
///
/// Every graph_local_id-keyed method has a ScheduleNode* overload
/// that forwards through node->GetGraphLocalId().
class NodeRegInfoTable {
 public:
  /// Empty table sized for `n_nodes` (all entries default-constructed
  /// — no defs, no uses). Intended for tests that populate entries
  /// directly. Production callers should use BuildForGraph instead.
  explicit NodeRegInfoTable(int n_nodes) : entries_(n_nodes) {}

  /// Production builder: extract NodeRegInfo for every scheduling-unit
  /// node in `graph` from its MachineInstr (or RegDefs/RegUses for
  /// entry/exit nodes). Subgraph proxies leave their slot default-
  /// constructed (empty defs/uses) — they have no register effect.
  /// `mf` provides the MachineRegisterInfo + TargetRegisterInfo;
  /// `lis` is needed for accurate use-mask extraction on multi-lane
  /// registers.
  static NodeRegInfoTable BuildForGraph(const ScheduleGraph &graph,
                                        const MachineFunction &mf,
                                        const LiveIntervals &lis);

  int Size() const { return static_cast<int>(entries_.size()); }

  /// Grow the table by appending default-constructed entries until
  /// Size() == n. No-op if Size() >= n. Used by ScheduleGraph after
  /// post-construction node additions (e.g., InsertSubgraphProxies)
  /// to keep the table aligned with graph.Size(). Never shrinks —
  /// graphs only grow.
  void EnsureSize(int n) {
    if (Size() < n) {
      entries_.resize(n);
    }
  }

  const NodeRegInfo &GetForGraphLocalId(int graph_local_id) const {
    return entries_[graph_local_id];
  }
  const NodeRegInfo &GetForNode(const ScheduleNode *node) const {
    return GetForGraphLocalId(node->GetGraphLocalId());
  }

  /// Append a def to the entry. If the entry already has a def for the
  /// same `reg`, the lane masks are OR'd together (matches the
  /// production dedup behavior). Otherwise a new entry is appended.
  void AddDef(int graph_local_id, unsigned reg, LaneBitmask mask);
  void AddDef(const ScheduleNode *node, unsigned reg, LaneBitmask mask) {
    AddDef(node->GetGraphLocalId(), reg, mask);
  }

  /// Append a use to the entry. Same dedup semantics as AddDef.
  void AddUse(int graph_local_id, unsigned reg, LaneBitmask mask);
  void AddUse(const ScheduleNode *node, unsigned reg, LaneBitmask mask) {
    AddUse(node->GetGraphLocalId(), reg, mask);
  }

  /// Replace the entry wholesale. Useful for tests that have a
  /// complete NodeRegInfo prepared and don't want to add entries one
  /// at a time.
  void SetEntry(int graph_local_id, NodeRegInfo info) {
    entries_[graph_local_id] = std::move(info);
  }
  void SetEntry(const ScheduleNode *node, NodeRegInfo info) {
    SetEntry(node->GetGraphLocalId(), std::move(info));
  }

 private:
  /// Insert (reg, mask) into `reg_masks`, merging with an existing
  /// entry for the same `reg` by OR-ing the lane masks. If no entry
  /// for `reg` is present, append a new one. This is the shared
  /// dedup logic behind AddDef and AddUse — see their header comments
  /// for the motivating case (sub-register defs/uses of the same
  /// vreg, which should collapse to one entry with merged lane mask
  /// rather than several entries with the same `reg`).
  static void InsertOrMergeRegMask(SmallVectorImpl<RegMask> &reg_masks,
                                   unsigned reg, LaneBitmask mask);

  std::vector<NodeRegInfo> entries_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_NODEREGINFO_H
