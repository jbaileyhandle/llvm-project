add_llvm_component_library(LLVMAMDGPUHierarchicalScheduler
  ScheduleDAGHierarchicalScheduler.cpp
  MaliciousScheduler.cpp
  ScheduleGraph.cpp
  DominatorTree.cpp
  RegisterTracker.cpp

  LINK_COMPONENTS
  Analysis
  CodeGen
  Support

  ADD_TO_COMPONENT
  AMDGPU
  )

# NOTE: We would like to keep asserts enabled in our library even in release
# builds (-UNDEBUG), but this causes ABI incompatibility with the rest of LLVM.
# Some LLVM classes have different layouts depending on NDEBUG (e.g., extra
# debug-only fields), so mixing assert-enabled and assert-disabled code in the
# same process causes memory corruption. We must match the rest of LLVM's
# NDEBUG setting.
# target_compile_options(LLVMAMDGPUHierarchicalScheduler PRIVATE -UNDEBUG)
