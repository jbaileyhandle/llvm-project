//===- NodeRegInfo.cpp - Pre-extracted per-node register info -------------===//
//
// jbaile
//
// Implementation of the NodeRegInfoTable mutators and the production
// builder. The lane-mask extraction helpers (formerly local to
// GCNRegisterTracker.cpp) live here too — they're purely about
// extracting per-MachineOperand info, which conceptually belongs with
// the table rather than the tracker that consumes it.
//
//===----------------------------------------------------------------------===//

#include "NodeRegInfo.h"

#include "GCNRegPressure.h"
#include "SIRegisterInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// ============================================================================
// Lane mask helpers
// ============================================================================
// Moved from GCNRegisterTracker.cpp. Local reimplementations of the
// static helpers in GCNRegPressure.cpp (getDefRegMask, getUsedRegMask)
// — same logic, same cases. File-local statics in this TU.

LaneBitmask GetDefMask(const MachineOperand &mo,
                       const MachineRegisterInfo &mri) {
  assert(mo.isDef() && mo.isReg() && mo.getReg().isVirtual());
  // Don't rely on read-undef flag — it may not be set correctly for
  // tentative schedules. See comment in GCNRegPressure.cpp.
  if (mo.getSubReg() == 0) {
    return mri.getMaxLaneMaskForVReg(mo.getReg());
  }
  return mri.getTargetRegisterInfo()->getSubRegIndexLaneMask(mo.getSubReg());
}

LaneBitmask GetUseMask(const MachineOperand &mo,
                       const MachineRegisterInfo &mri,
                       const LiveIntervals &lis) {
  assert(mo.isUse() && mo.isReg() && mo.getReg().isVirtual());

  if (auto sub_reg = mo.getSubReg()) {
    return mri.getTargetRegisterInfo()->getSubRegIndexLaneMask(sub_reg);
  }

  auto max_mask = mri.getMaxLaneMaskForVReg(mo.getReg());
  if (SIRegisterInfo::getNumCoveredRegs(max_mask) > 1) {
    return max_mask;
  }

  // Single-lane register: query LiveIntervals for the actual live
  // mask at this instruction. Subreg defs can be reordered but all
  // must dominate uses, so the live lane mask is schedule-invariant.
  auto si = lis.getInstructionIndex(*mo.getParent()).getBaseIndex();
  return getLiveLaneMask(mo.getReg(), si, lis, mri);
}

// ============================================================================
// Per-node extraction
// ============================================================================

void ExtractFromMachineInstr(const ScheduleNode *node,
                             NodeRegInfoTable &table,
                             const MachineRegisterInfo &mri,
                             const LiveIntervals &lis) {
  const MachineInstr &mi = *node->GetSUnit()->getInstr();

  // Defs: skip non-virtual and dead.
  for (const MachineOperand &mo : mi.all_defs()) {
    if (!mo.getReg().isVirtual() || mo.isDead()) {
      continue;
    }
    table.AddDef(node, mo.getReg().id(), GetDefMask(mo, mri));
  }

  // Uses: skip non-virtual. Also skip operands that are syntactically
  // uses but don't actually read the register (undef flag — appears
  // as an implicit use to keep the live range alive for the register
  // allocator, but no hardware read occurs).
  for (const MachineOperand &mo : mi.operands()) {
    if (!mo.isReg() || !mo.getReg().isVirtual()) {
      continue;
    }
    if (!mo.isUse() || !mo.readsReg()) {
      continue;
    }
    table.AddUse(node, mo.getReg().id(), GetUseMask(mo, mri, lis));
  }
}

void ExtractFromNodeRegLists(const ScheduleNode *node,
                             NodeRegInfoTable &table) {
  for (const RegWithLaneMask &rm : node->RegDefs()) {
    if (!rm.reg.isVirtual()) {
      continue;
    }
    table.AddDef(node, rm.reg.id(), rm.mask);
  }
  for (const RegWithLaneMask &rm : node->RegUses()) {
    if (!rm.reg.isVirtual()) {
      continue;
    }
    table.AddUse(node, rm.reg.id(), rm.mask);
  }
}

} // anonymous namespace

// ============================================================================
// NodeRegInfoTable
// ============================================================================

void NodeRegInfoTable::InsertOrMergeRegMask(
    SmallVectorImpl<RegMask> &reg_masks, unsigned reg, LaneBitmask mask) {
  auto it = llvm::find_if(reg_masks, [reg](const RegMask &existing) {
    return existing.reg == reg;
  });
  if (it != reg_masks.end()) {
    it->mask |= mask;
  } else {
    reg_masks.push_back({reg, mask});
  }
}

void NodeRegInfoTable::AddDef(int graph_local_id, unsigned reg,
                              LaneBitmask mask) {
  InsertOrMergeRegMask(entries_[graph_local_id].defs, reg, mask);
}

void NodeRegInfoTable::AddUse(int graph_local_id, unsigned reg,
                              LaneBitmask mask) {
  InsertOrMergeRegMask(entries_[graph_local_id].uses, reg, mask);
}

NodeRegInfoTable NodeRegInfoTable::BuildForGraph(
    const ScheduleGraph &graph, const MachineFunction &mf,
    const LiveIntervals &lis) {
  // Size by graph_local_id count, not node count: the graph itself
  // consumes graph_local_id 0, then nodes take 1..N. Indexing
  // entries_[node.GetGraphLocalId()] for a top-level graph needs
  // capacity N+1 to cover the highest node id.
  NodeRegInfoTable table(graph.GetNumGraphLocalIds());
  const MachineRegisterInfo &mri = mf.getRegInfo();

  for (const ScheduleNode &node : graph.Nodes()) {
    if (!node.IsSchedulingUnit()) {
      // Subgraph proxies are synthetic and have no register effect.
      // Leave their slot default-constructed (empty NodeRegInfo).
      continue;
    }

    SUnit *su = node.GetSUnit();
    if (su && su->getInstr()) {
      ExtractFromMachineInstr(&node, table, mri, lis);
    } else {
      ExtractFromNodeRegLists(&node, table);
    }
  }

  return table;
}
