# HierarchicalScheduler source files. Compiled as part of AMDGPUCodeGen
# (not a separate library) so we can call functions from GCNRegPressure,
# SIRegisterInfo, etc. without circular link dependencies.
#
# The parent CMakeLists.txt does add_subdirectory(HierarchicalScheduler)
# before add_llvm_target(AMDGPUCodeGen ...) to pick up this variable,
# then includes ${HIERARCHICAL_SCHEDULER_SOURCES} in the source list.
#
# Paths are relative to the parent directory (lib/Target/AMDGPU/).

set(HIERARCHICAL_SCHEDULER_SOURCES
  HierarchicalScheduler/ScheduleDAGHierarchicalScheduler.cpp
  HierarchicalScheduler/Shakedowns.cpp
  HierarchicalScheduler/MaliciousScheduler.cpp
  HierarchicalScheduler/ScheduleGraph.cpp
  HierarchicalScheduler/DominatorTree.cpp
  HierarchicalScheduler/RegisterTracker.cpp
  HierarchicalScheduler/GCNRegisterTracker.cpp
  HierarchicalScheduler/ScheduleLengthTracker.cpp
  HierarchicalScheduler/ScheduleConstructor.cpp
  HierarchicalScheduler/RegionInfo.cpp
  HierarchicalScheduler/BranchAndBoundSearch.cpp
  HierarchicalScheduler/SearchPolicies.cpp
  HierarchicalScheduler/SubgraphInfo.cpp
  HierarchicalScheduler/SubgraphFormation.cpp
  HierarchicalScheduler/ScheduledSetTracker.cpp
  HierarchicalScheduler/LengthHistoryTracker.cpp
  HierarchicalScheduler/PressureHistoryTracker.cpp
  PARENT_SCOPE
)

# NOTE: We would like to keep asserts enabled in our code even in release
# builds (-UNDEBUG), but this causes ABI incompatibility with the rest of LLVM.
# Some LLVM classes have different layouts depending on NDEBUG (e.g., extra
# debug-only fields), so mixing assert-enabled and assert-disabled code in the
# same process causes memory corruption. We must match the rest of LLVM's
# NDEBUG setting.
