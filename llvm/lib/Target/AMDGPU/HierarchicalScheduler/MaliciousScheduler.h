//===- MaliciousScheduler.h - Pressure-maximizing scheduler ------*- C++ -*-===//
//
// A deliberately bad list scheduler that maximizes register pressure and
// minimizes latency hiding. Used as an experimental baseline to measure
// the impact of scheduling decisions.
//
// Scoring heuristic (higher = preferred):
//   +0  base
//   +2  if the just-scheduled instruction is a predecessor/producer of this
//       instruction (favor back-to-back dependencies)
//   +1  if this instruction is NOT the last consumer of one of its input
//       registers (favor instructions that keep live ranges open)
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MALICIOUSSCHEDULER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MALICIOUSSCHEDULER_H

#include "llvm/CodeGen/ScheduleDAG.h"
#include <vector>

namespace llvm {
namespace hierarchical_scheduler {

// Given a set of SUnits with dependency edges already built, compute a
// schedule order using the malicious heuristic. Returns pointers to SUnits
// in the order they should be scheduled.
//
// The caller is responsible for building the DAG beforehand and applying
// the returned order afterwards.
std::vector<SUnit *> ComputeMaliciousSchedule(std::vector<SUnit> &sunits);

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MALICIOUSSCHEDULER_H
