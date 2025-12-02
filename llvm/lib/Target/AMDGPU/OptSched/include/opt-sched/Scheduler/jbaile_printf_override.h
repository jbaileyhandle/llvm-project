/*******************************************************************************
Overrides printf calls from GPU so that we can safely run on old motherboard w/ older ROCM
*******************************************************************************/

#ifndef JBAILE_PRINTF_OVERRIDE_H
#define JBAILE_PRINTF_OVERRIDE_H 

namespace llvm {
namespace opt_sched {

// Device pass only: printf returns 0 and evaluates nothing.
#if defined(__HIP_DEVICE_COMPILE__)
      #define printf(...) (0)
#endif

} // namespace opt_sched
} // namespace llvm

#endif // JBAILE_PRINTF_OVERRIDE_H 
