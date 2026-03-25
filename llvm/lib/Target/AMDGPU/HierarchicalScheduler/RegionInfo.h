//===- RegionInfo.h - Scheduling Region Info -------------------*- C++ -*-===//
//
// A lightweight, read-only wrapper around a scheduling region's boundaries.
// Stores RegionBegin and RegionEnd iterators at construction time and
// provides accessors for the parent basic block and instruction count.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGIONINFO_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGIONINFO_H

#include "llvm/CodeGen/MachineBasicBlock.h"
#include <iterator>

namespace llvm {
namespace hierarchical_scheduler {

class RegionInfo {
public:
  RegionInfo(MachineBasicBlock::iterator begin,
             MachineBasicBlock::iterator end)
      : begin_(begin), end_(end) {}

  MachineBasicBlock::iterator Begin() const { return begin_; }
  MachineBasicBlock::iterator End() const { return end_; }

  // The parent basic block, derived from the begin iterator.
  MachineBasicBlock *GetBlock() const { return begin_->getParent(); }

  // Number of instructions in the region. Region size is exceedingly
  // unlikely to exceed int range.
  int GetNumInstrs() const {
    return static_cast<int>(std::distance(begin_, end_));
  }

private:
  MachineBasicBlock::iterator begin_;
  MachineBasicBlock::iterator end_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGIONINFO_H
