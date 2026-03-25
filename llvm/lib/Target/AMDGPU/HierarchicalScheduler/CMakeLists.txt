add_llvm_component_library(LLVMAMDGPUHierarchicalScheduler
  ScheduleDAGHierarchicalScheduler.cpp
  MaliciousScheduler.cpp

  LINK_COMPONENTS
  Analysis
  CodeGen
  Support

  ADD_TO_COMPONENT
  AMDGPU
  )
