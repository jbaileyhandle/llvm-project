//===- MemoryGraph.h - Memory-op view of a region for MLP -------*- C++ -*-===//
//
// A graph of a region's memory operations, for measuring and searching
// memory-level parallelism (MLP). See llvm/docs/AMDGPUMemoryGraphDesign.md.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYGRAPH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYGRAPH_H

#include <optional>

namespace llvm {

class GCNSubtarget;
class MachineInstr;

namespace hierarchical_scheduler {

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

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MEMORYGRAPH_H
