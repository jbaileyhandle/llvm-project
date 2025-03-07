//===----RTLs/amdgpu/src/rtl.cpp - Target RTLs Implementation ----- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// RTL NextGen for AMDGPU machine
//
//===----------------------------------------------------------------------===//

#include <atomic>
#include <cassert>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <sys/time.h>
#include <system_error>
#include <unistd.h>
#include <unordered_map>

#include "Debug.h"
#include "DeviceEnvironment.h"
#include "GlobalHandler.h"
#include "PluginInterface.h"
#include "Utilities.h"
#include "UtilitiesRTL.h"
#include "omptarget.h"

#include "print_tracing.h"

#include "memtype.h"

#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Frontend/OpenMP/OMPConstants.h"
#include "llvm/Frontend/OpenMP/OMPGridValues.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/Program.h"
#include "llvm/Support/raw_ostream.h"

#if defined(__has_include)
#if __has_include("hsa/hsa.h")
#include "hsa/hsa.h"
#include "hsa/hsa_ext_amd.h"
#elif __has_include("hsa.h")
#include "hsa.h"
#include "hsa_ext_amd.h"
#endif
#else
#include "hsa/hsa.h"
#include "hsa/hsa_ext_amd.h"
#endif

#ifdef OMPT_SUPPORT
#include <ompt_device_callbacks.h>
#define OMPT_IF_ENABLED(stmts)                                                 \
  do {                                                                         \
    if (ompt_device_callbacks.is_enabled()) {                                  \
      stmts                                                                    \
    }                                                                          \
  } while (0)
#define OMPT_IF_TRACING_ENABLED(stmts)                                         \
  do {                                                                         \
    if (ompt_device_callbacks.is_tracing_enabled()) {                          \
      stmts                                                                    \
    }                                                                          \
  } while (0)
#else
#define OMPT_IF_ENABLED(stmts)
#define OMPT_IF_TRACING_ENABLED(stmts)
#endif

#ifdef OMPT_SUPPORT
extern bool OmptEnabled;
extern void OmptCallbackInit();
extern void setOmptTimestamp(uint64_t Start, uint64_t End);
extern void setOmptHostToDeviceRate(double Slope, double Offset);

/// HSA system clock frequency
double TicksToTime = 1.0;

/// Enable/disable async copy profiling.
void setOmptAsyncCopyProfile(bool Enable) {
  hsa_status_t Status = hsa_amd_profiling_async_copy_enable(Enable);
  if (Status != HSA_STATUS_SUCCESS)
    DP("Error enabling async copy profiling\n");
}

/// Compute system timestamp conversion factor, modeled after ROCclr.
void setOmptTicksToTime() {
  uint64_t TicksFrequency = 1;
  hsa_status_t Status =
      hsa_system_get_info(HSA_SYSTEM_INFO_TIMESTAMP_FREQUENCY, &TicksFrequency);
  if (Status == HSA_STATUS_SUCCESS)
    TicksToTime = (double)1e9 / (double)TicksFrequency;
  else
    DP("Error calling hsa_system_get_info for timestamp frequency\n");
}

/// Get the current HSA-based device timestamp.
uint64_t getSystemTimestampInNs() {
  uint64_t TimeStamp = 0;
  hsa_status_t Status =
      hsa_system_get_info(HSA_SYSTEM_INFO_TIMESTAMP, &TimeStamp);
  if (Status != HSA_STATUS_SUCCESS)
    DP("Error calling hsa_system_get_info for timestamp\n");
  return TimeStamp * TicksToTime;
}

/// @brief Helper to get the host time
/// @return  CLOCK_REALTIME seconds as double
static double getTimeOfDay() {
  double TimeVal = .0;
  struct timeval tval;
  int rc = gettimeofday(&tval, NULL);
  if (rc) {
    // XXX: Error case: What to do?
  } else {
    TimeVal = static_cast<double>(tval.tv_sec) +
              1.0E-06 * static_cast<double>(tval.tv_usec);
  }
  return TimeVal;
}

/// Get the first timepoints on host and device.
void startH2DTimeRate(double *HTime, uint64_t *DTime) {
  *HTime = getTimeOfDay();
  *DTime = getSystemTimestampInNs();
}

/// Get the second timepoints on host and device and compute the rate
/// required for translating device time to host time.
void completeH2DTimeRate(double HostRef1, uint64_t DeviceRef1) {
  double HostRef2 = getTimeOfDay();
  uint64_t DeviceRef2 = getSystemTimestampInNs();
  // Assume host (h) timing is related to device (d) timing as
  // h = m.d + o, where m is the slope and o is the offset.
  // Calculate slope and offset from the two host and device timepoints.
  double HostDiff = HostRef2 - HostRef1;
  uint64_t DeviceDiff = DeviceRef2 - DeviceRef1;
  double Slope = DeviceDiff != 0 ? (HostDiff / DeviceDiff) : HostDiff;
  double Offset = HostRef1 - Slope * DeviceRef1;
  setOmptHostToDeviceRate(Slope, Offset);
  DP("OMPT: Translate time Slope: %f Offset: %f\n", Slope, Offset);
}

#endif

namespace llvm {
namespace omp {
namespace target {
namespace plugin {

extern "C" {
uint64_t hostrpc_assign_buffer(hsa_agent_t Agent, hsa_queue_t *ThisQ,
                               uint32_t DeviceId,
                               hsa_amd_memory_pool_t HostMemoryPool,
                               hsa_amd_memory_pool_t DevMemoryPool);
hsa_status_t hostrpc_terminate();
__attribute__((weak)) hsa_status_t hostrpc_terminate() {
  return HSA_STATUS_SUCCESS;
}
__attribute__((weak)) uint64_t
hostrpc_assign_buffer(hsa_agent_t, hsa_queue_t *, uint32_t DeviceId,
                      hsa_amd_memory_pool_t HostMemoryPool,
                      hsa_amd_memory_pool_t DevMemoryPool) {
  // FIXME:THIS SHOULD BE HARD FAIL
  DP("Warning: Attempting to assign hostrpc to device %u, but hostrpc library "
     "missing\n",
     DeviceId);
  return 0;
}
}

/// Forward declarations for all specialized data structures.
struct AMDGPUKernelTy;
struct AMDGPUDeviceTy;
struct AMDGPUPluginTy;
struct AMDGPUStreamTy;
struct AMDGPUEventTy;
struct AMDGPUStreamManagerTy;
struct AMDGPUEventManagerTy;
struct AMDGPUDeviceImageTy;
struct AMDGPUMemoryManagerTy;
struct AMDGPUMemoryPoolTy;

namespace utils {

/// Iterate elements using an HSA iterate function. Do not use this function
/// directly but the specialized ones below instead.
template <typename ElemTy, typename IterFuncTy, typename CallbackTy>
hsa_status_t iterate(IterFuncTy Func, CallbackTy Cb) {
  auto L = [](ElemTy Elem, void *Data) -> hsa_status_t {
    CallbackTy *Unwrapped = static_cast<CallbackTy *>(Data);
    return (*Unwrapped)(Elem);
  };
  return Func(L, static_cast<void *>(&Cb));
}

/// Iterate elements using an HSA iterate function passing a parameter. Do not
/// use this function directly but the specialized ones below instead.
template <typename ElemTy, typename IterFuncTy, typename IterFuncArgTy,
          typename CallbackTy>
hsa_status_t iterate(IterFuncTy Func, IterFuncArgTy FuncArg, CallbackTy Cb) {
  auto L = [](ElemTy Elem, void *Data) -> hsa_status_t {
    CallbackTy *Unwrapped = static_cast<CallbackTy *>(Data);
    return (*Unwrapped)(Elem);
  };
  return Func(FuncArg, L, static_cast<void *>(&Cb));
}

/// Iterate elements using an HSA iterate function passing a parameter. Do not
/// use this function directly but the specialized ones below instead.
template <typename Elem1Ty, typename Elem2Ty, typename IterFuncTy,
          typename IterFuncArgTy, typename CallbackTy>
hsa_status_t iterate(IterFuncTy Func, IterFuncArgTy FuncArg, CallbackTy Cb) {
  auto L = [](Elem1Ty Elem1, Elem2Ty Elem2, void *Data) -> hsa_status_t {
    CallbackTy *Unwrapped = static_cast<CallbackTy *>(Data);
    return (*Unwrapped)(Elem1, Elem2);
  };
  return Func(FuncArg, L, static_cast<void *>(&Cb));
}

/// Iterate agents.
template <typename CallbackTy> Error iterateAgents(CallbackTy Callback) {
  hsa_status_t Status = iterate<hsa_agent_t>(hsa_iterate_agents, Callback);
  return Plugin::check(Status, "Error in hsa_iterate_agents: %s");
}

/// Iterate ISAs of an agent.
template <typename CallbackTy>
Error iterateAgentISAs(hsa_agent_t Agent, CallbackTy Cb) {
  hsa_status_t Status = iterate<hsa_isa_t>(hsa_agent_iterate_isas, Agent, Cb);
  return Plugin::check(Status, "Error in hsa_agent_iterate_isas: %s");
}

/// Iterate memory pools of an agent.
template <typename CallbackTy>
Error iterateAgentMemoryPools(hsa_agent_t Agent, CallbackTy Cb) {
  hsa_status_t Status = iterate<hsa_amd_memory_pool_t>(
      hsa_amd_agent_iterate_memory_pools, Agent, Cb);
  return Plugin::check(Status,
                       "Error in hsa_amd_agent_iterate_memory_pools: %s");
}

extern "C" uint64_t hostrpc_assign_buffer(hsa_agent_t Agent, hsa_queue_t *ThisQ,
                                          uint32_t DeviceId,
                                          hsa_amd_memory_pool_t HostMemoryPool,
                                          hsa_amd_memory_pool_t DevMemoryPool);
extern "C" hsa_status_t hostrpc_terminate();
} // namespace utils

/// Utility class representing generic resource references to AMDGPU resources.
template <typename ResourceTy>
struct AMDGPUResourceRef : public GenericDeviceResourceRef {
  /// Create an empty reference to an invalid resource.
  AMDGPUResourceRef() : Resource(nullptr) {}

  /// Create a reference to an existing resource.
  AMDGPUResourceRef(ResourceTy *Resource) : Resource(Resource) {}

  virtual ~AMDGPUResourceRef() {}

  /// Create a new resource and save the reference. The reference must be empty
  /// before calling to this function.
  Error create(GenericDeviceTy &Device) override;

  /// Destroy the referenced resource and invalidate the reference. The
  /// reference must be to a valid event before calling to this function.
  Error destroy(GenericDeviceTy &Device) override {
    if (!Resource)
      return Plugin::error("Destroying an invalid resource");

    if (auto Err = Resource->deinit())
      return Err;

    delete Resource;

    Resource = nullptr;
    return Plugin::success();
  }

  /// Get the underlying AMDGPUSignalTy reference.
  operator ResourceTy *() const { return Resource; }

private:
  /// The reference to the actual resource.
  ResourceTy *Resource;
};

/// Class holding an HSA memory pool.
struct AMDGPUMemoryPoolTy {
  /// Create a memory pool from an HSA memory pool.
  AMDGPUMemoryPoolTy(hsa_amd_memory_pool_t MemoryPool)
      : MemoryPool(MemoryPool), GlobalFlags(0) {}

  /// Initialize the memory pool retrieving its properties.
  Error init() {
    if (auto Err = getAttr(HSA_AMD_MEMORY_POOL_INFO_SEGMENT, Segment))
      return Err;

    if (auto Err = getAttr(HSA_AMD_MEMORY_POOL_INFO_GLOBAL_FLAGS, GlobalFlags))
      return Err;

    return Plugin::success();
  }

  /// Getter of the HSA memory pool.
  hsa_amd_memory_pool_t get() const { return MemoryPool; }

  /// Indicate the segment which belongs to.
  bool isGlobal() const { return (Segment == HSA_AMD_SEGMENT_GLOBAL); }
  bool isReadOnly() const { return (Segment == HSA_AMD_SEGMENT_READONLY); }
  bool isPrivate() const { return (Segment == HSA_AMD_SEGMENT_PRIVATE); }
  bool isGroup() const { return (Segment == HSA_AMD_SEGMENT_GROUP); }

  /// Indicate if it is fine-grained memory. Valid only for global.
  bool isFineGrained() const {
    assert(isGlobal() && "Not global memory");
    return (GlobalFlags & HSA_AMD_MEMORY_POOL_GLOBAL_FLAG_FINE_GRAINED);
  }

  /// Indicate if it is coarse-grained memory. Valid only for global.
  bool isCoarseGrained() const {
    assert(isGlobal() && "Not global memory");
    return (GlobalFlags & HSA_AMD_MEMORY_POOL_GLOBAL_FLAG_COARSE_GRAINED);
  }

  /// Indicate if it supports storing kernel arguments. Valid only for global.
  bool supportsKernelArgs() const {
    assert(isGlobal() && "Not global memory");
    return (GlobalFlags & HSA_AMD_MEMORY_POOL_GLOBAL_FLAG_KERNARG_INIT);
  }

  /// Allocate memory on the memory pool.
  Error allocate(size_t Size, void **PtrStorage) {
    hsa_status_t Status =
        hsa_amd_memory_pool_allocate(MemoryPool, Size, 0, PtrStorage);
    return Plugin::check(Status, "Error in hsa_amd_memory_pool_allocate: %s");
  }

  /// Return memory to the memory pool.
  Error deallocate(void *Ptr) {
    hsa_status_t Status = hsa_amd_memory_pool_free(Ptr);
    return Plugin::check(Status, "Error in hsa_amd_memory_pool_free: %s");
  }

  /// Allow the device to access a specific allocation.
  Error enableAccess(void *Ptr, int64_t Size,
                     const llvm::SmallVector<hsa_agent_t> &Agents) const {
#ifdef OMPTARGET_DEBUG
    for (hsa_agent_t Agent : Agents) {
      hsa_amd_memory_pool_access_t Access;
      if (auto Err =
              getAttr(Agent, HSA_AMD_AGENT_MEMORY_POOL_INFO_ACCESS, Access))
        return Err;

      // The agent is not allowed to access the memory pool in any case. Do not
      // continue because otherwise it result in undefined behavior.
      if (Access == HSA_AMD_MEMORY_POOL_ACCESS_NEVER_ALLOWED)
        return Plugin::error("An agent is not allowed to access a memory pool");
    }
#endif

    // We can access but it is disabled by default. Enable the access then.
    hsa_status_t Status =
        hsa_amd_agents_allow_access(Agents.size(), Agents.data(), nullptr, Ptr);
    return Plugin::check(Status, "Error in hsa_amd_agents_allow_access: %s");
  }

  Error zeroInitializeMemory(void *Ptr, size_t Size) {
    uint64_t Rounded = sizeof(uint32_t) * ((Size + 3) / sizeof(uint32_t));
    hsa_status_t Status =
        hsa_amd_memory_fill(Ptr, 0, Rounded / sizeof(uint32_t));
    return Plugin::check(Status, "Error in hsa_amd_memory_fill: %s");
  }

  /// Get attribute from the memory pool.
  template <typename Ty>
  Error getAttr(hsa_amd_memory_pool_info_t Kind, Ty &Value) const {
    hsa_status_t Status;
    Status = hsa_amd_memory_pool_get_info(MemoryPool, Kind, &Value);
    return Plugin::check(Status, "Error in hsa_amd_memory_pool_get_info: %s");
  }

  template <typename Ty>
  hsa_status_t getAttrRaw(hsa_amd_memory_pool_info_t Kind, Ty &Value) const {
    return hsa_amd_memory_pool_get_info(MemoryPool, Kind, &Value);
  }

  /// Get attribute from the memory pool relating to an agent.
  template <typename Ty>
  Error getAttr(hsa_agent_t Agent, hsa_amd_agent_memory_pool_info_t Kind,
                Ty &Value) const {
    hsa_status_t Status;
    Status =
        hsa_amd_agent_memory_pool_get_info(Agent, MemoryPool, Kind, &Value);
    return Plugin::check(Status,
                         "Error in hsa_amd_agent_memory_pool_get_info: %s");
  }

private:
  /// The HSA memory pool.
  hsa_amd_memory_pool_t MemoryPool;

  /// The segment where the memory pool belongs to.
  hsa_amd_segment_t Segment;

  /// The global flags of memory pool. Only valid if the memory pool belongs to
  /// the global segment.
  uint32_t GlobalFlags;
};

/// Class that implements a memory manager that gets memory from a specific
/// memory pool.
struct AMDGPUMemoryManagerTy : public DeviceAllocatorTy {

  /// Create an empty memory manager.
  AMDGPUMemoryManagerTy() : MemoryPool(nullptr), MemoryManager(nullptr) {}

  /// Initialize the memory manager from a memory pool.
  Error init(AMDGPUMemoryPoolTy &MemoryPool) {
    const uint32_t Threshold = 1 << 30;
    this->MemoryManager = new MemoryManagerTy(*this, Threshold);
    this->MemoryPool = &MemoryPool;
    return Plugin::success();
  }

  /// Deinitialize the memory manager and free its allocations.
  Error deinit() {
    assert(MemoryManager && "Invalid memory manager");

    // Delete and invalidate the memory manager. At this point, the memory
    // manager will deallocate all its allocations.
    delete MemoryManager;
    MemoryManager = nullptr;

    return Plugin::success();
  }

  /// Reuse or allocate memory through the memory manager.
  Error allocate(size_t Size, void **PtrStorage) {
    assert(MemoryManager && "Invalid memory manager");
    assert(PtrStorage && "Invalid pointer storage");

    *PtrStorage = MemoryManager->allocate(Size, nullptr);
    if (*PtrStorage == nullptr)
      return Plugin::error("Failure to allocate from AMDGPU memory manager");

    return Plugin::success();
  }

  /// Release an allocation to be reused.
  Error deallocate(void *Ptr) {
    assert(Ptr && "Invalid pointer");

    if (MemoryManager->free(Ptr))
      return Plugin::error("Failure to deallocate from AMDGPU memory manager");

    return Plugin::success();
  }

private:
  /// Allocation callback that will be called once the memory manager does not
  /// have more previously allocated buffers.
  void *allocate(size_t Size, void *HstPtr, TargetAllocTy Kind) override;

  /// Deallocation callack that will be called by the memory manager.
  int free(void *TgtPtr, TargetAllocTy Kind) override {
    if (auto Err = MemoryPool->deallocate(TgtPtr)) {
      consumeError(std::move(Err));
      return OFFLOAD_FAIL;
    }
    return OFFLOAD_SUCCESS;
  }

  /// The memory pool used to allocate memory.
  AMDGPUMemoryPoolTy *MemoryPool;

  /// Reference to the actual memory manager.
  MemoryManagerTy *MemoryManager;
};

/// Class implementing the AMDGPU device images' properties.
struct AMDGPUDeviceImageTy : public DeviceImageTy {
  /// Create the AMDGPU image with the id and the target image pointer.
  AMDGPUDeviceImageTy(int32_t ImageId, const __tgt_device_image *TgtImage)
      : DeviceImageTy(ImageId, TgtImage) {}

  /// Prepare and load the executable corresponding to the image.
  Error loadExecutable(const AMDGPUDeviceTy &Device);

  /// Unload the executable.
  Error unloadExecutable() {
    hsa_status_t Status = hsa_executable_destroy(Executable);
    if (auto Err = Plugin::check(Status, "Error in hsa_executable_destroy: %s"))
      return Err;

    Status = hsa_code_object_destroy(CodeObject);
    return Plugin::check(Status, "Error in hsa_code_object_destroy: %s");
  }

  /// Get the executable.
  hsa_executable_t getExecutable() const { return Executable; }

  /// Get to Code Object Version of the ELF
  uint16_t getELFABIVersion() const { return ELFABIVersion; }

  /// Find an HSA device symbol by its name on the executable.
  Expected<hsa_executable_symbol_t>
  findDeviceSymbol(GenericDeviceTy &Device, StringRef SymbolName) const;

  /// Get additional info for kernel, e.g., register spill counts
  std::optional<utils::KernelMetaDataTy>
  getKernelInfo(StringRef Identifier) const {
    auto It = KernelInfoMap.find(Identifier);

    if (It == KernelInfoMap.end())
      return {};

    return It->second;
  }

  /// Does device image contain Symbol
  bool hasDeviceSymbol(GenericDeviceTy &Device, StringRef SymbolName) const;

private:
  /// The exectuable loaded on the agent.
  hsa_executable_t Executable;
  hsa_code_object_t CodeObject;
#if SANITIZER_AMDGPU
  hsa_code_object_reader_t CodeObjectReader;
#endif
  StringMap<utils::KernelMetaDataTy> KernelInfoMap;
  uint16_t ELFABIVersion;
};

/// Class implementing the AMDGPU kernel functionalities which derives from the
/// generic kernel class.
struct AMDGPUKernelTy : public GenericKernelTy {
  /// Create an AMDGPU kernel with a name and an execution mode.
  AMDGPUKernelTy(const char *Name, OMPTgtExecModeFlags ExecutionMode)
      : GenericKernelTy(Name, ExecutionMode),
        ServiceThreadDeviceBufferGlobal("service_thread_buf", sizeof(uint64_t)),
        HostServiceBufferHandler(Plugin::createGlobalHandler()) {}

  /// Initialize the AMDGPU kernel.
  Error initImpl(GenericDeviceTy &Device, DeviceImageTy &Image) override {
    AMDGPUDeviceImageTy &AMDImage = static_cast<AMDGPUDeviceImageTy &>(Image);

    // Kernel symbols have a ".kd" suffix.
    std::string KernelName(getName());
    KernelName += ".kd";

    // Find the symbol on the device executable.
    auto SymbolOrErr = AMDImage.findDeviceSymbol(Device, KernelName);
    if (!SymbolOrErr)
      return SymbolOrErr.takeError();

    hsa_executable_symbol_t Symbol = *SymbolOrErr;
    hsa_symbol_kind_t SymbolType;
    hsa_status_t Status;

    // Retrieve different properties of the kernel symbol.
    std::pair<hsa_executable_symbol_info_t, void *> RequiredInfos[] = {
        {HSA_EXECUTABLE_SYMBOL_INFO_TYPE, &SymbolType},
        {HSA_EXECUTABLE_SYMBOL_INFO_KERNEL_OBJECT, &KernelObject},
        {HSA_EXECUTABLE_SYMBOL_INFO_KERNEL_KERNARG_SEGMENT_SIZE, &ArgsSize},
        {HSA_EXECUTABLE_SYMBOL_INFO_KERNEL_GROUP_SEGMENT_SIZE, &GroupSize},
        {HSA_EXECUTABLE_SYMBOL_INFO_KERNEL_PRIVATE_SEGMENT_SIZE, &PrivateSize}};

    for (auto &Info : RequiredInfos) {
      Status = hsa_executable_symbol_get_info(Symbol, Info.first, Info.second);
      if (auto Err = Plugin::check(
              Status, "Error in hsa_executable_symbol_get_info: %s"))
        return Err;
    }

    // Make sure it is a kernel symbol.
    if (SymbolType != HSA_SYMBOL_KIND_KERNEL)
      return Plugin::error("Symbol %s is not a kernel function");

    // TODO: Read the kernel descriptor for the max threads per block. May be
    // read from the image.

    // Get ConstWGSize for kernel from image
    std::string WGSizeName(getName());
    WGSizeName += "_wg_size";
    GlobalTy HostConstWGSize(WGSizeName, sizeof(decltype(ConstWGSize)),
                             &ConstWGSize);
    GenericGlobalHandlerTy &GHandler = Plugin::get().getGlobalHandler();
    if (auto Err =
            GHandler.readGlobalFromImage(Device, AMDImage, HostConstWGSize)) {
      // In case it is not found, we simply stick with the defaults.
      // So we consume the error and print a debug message.
      DP("Could not load %s global from kernel image. Run with %u %u\n",
         WGSizeName.c_str(), PreferredNumThreads, MaxNumThreads);
      consumeError(std::move(Err));
      assert(PreferredNumThreads > 0 && "Prefer more than 0 threads");
      assert(MaxNumThreads > 0 && "MaxNumThreads more than 0 threads");
    } else {
      // Set the number of preferred and max threads to the ConstWGSize to get
      // the exact value for kernel launch. Exception: In generic-spmd mode, we
      // set it to the default blocksize since ConstWGSize may include the
      // master thread which is not required.
      PreferredNumThreads =
          getExecutionModeFlags() == OMP_TGT_EXEC_MODE_GENERIC_SPMD
              ? Device.getDefaultNumThreads()
              : ConstWGSize;
      MaxNumThreads = ConstWGSize;
    }

    ImplicitArgsSize =
        (AMDImage.getELFABIVersion() < llvm::ELF::ELFABIVERSION_AMDGPU_HSA_V5)
            ? utils::COV4_SIZE
            : utils::COV5_SIZE;
    DP("ELFABIVersion: %d\n", AMDImage.getELFABIVersion());

    // Get additional kernel info read from image
    KernelInfo = AMDImage.getKernelInfo(getName());
    if (!KernelInfo.has_value())
      INFO(OMP_INFOTYPE_PLUGIN_KERNEL, Device.getDeviceId(),
           "Could not read extra information for kernel %s.", getName());

    NeedsHostServices =
        AMDImage.hasDeviceSymbol(Device, "__needs_host_services");
    if (NeedsHostServices) {
      // GenericGlobalHandlerTy * GHandler = Plugin::createGlobalHandler();
      if (auto Err = HostServiceBufferHandler->getGlobalMetadataFromDevice(
              Device, AMDImage, ServiceThreadDeviceBufferGlobal))
        return Err;
    }

    return Plugin::success();
  }

  /// Launch the AMDGPU kernel function.
  Error launchImpl(GenericDeviceTy &GenericDevice, uint32_t NumThreads,
                   uint64_t NumBlocks, KernelArgsTy &KernelArgs, void *Args,
                   AsyncInfoWrapperTy &AsyncInfoWrapper) const override;

  /// Print more elaborate kernel launch info for AMDGPU
  Error printLaunchInfoDetails(GenericDeviceTy &GenericDevice,
                               KernelArgsTy &KernelArgs, uint32_t NumThreads,
                               uint64_t NumBlocks) const override;
  /// Print the "old" AMD KernelTrace single-line format
  void printAMDOneLineKernelTrace(GenericDeviceTy &GenericDevice,
                                  KernelArgsTy &KernelArgs, uint32_t NumThreads,
                                  uint64_t NumBlocks) const;

  /// The default number of blocks is common to the whole device.
  uint32_t getDefaultNumBlocks(GenericDeviceTy &GenericDevice) const override {
    return GenericDevice.getDefaultNumBlocks();
  }

  /// The default number of threads is common to the whole device.
  uint32_t getDefaultNumThreads(GenericDeviceTy &GenericDevice) const override {
    return GenericDevice.getDefaultNumThreads();
  }

  /// Get group and private segment kernel size.
  uint32_t getGroupSize() const { return GroupSize; }
  uint32_t getPrivateSize() const { return PrivateSize; }
  uint16_t getConstWGSize() const { return ConstWGSize; }

  /// Get the HSA kernel object representing the kernel function.
  uint64_t getKernelObject() const { return KernelObject; }

  /// Get the size of implicitargs based on the code object version
  /// @return 56 for cov4 and 256 for cov5
  uint32_t getImplicitArgsSize() const { return ImplicitArgsSize; }

private:
  /// The kernel object to execute.
  uint64_t KernelObject;

  /// The args, group and private segments sizes required by a kernel instance.
  uint32_t ArgsSize;
  uint32_t GroupSize;
  uint32_t PrivateSize;

  /// The size of implicit kernel arguments.
  uint32_t ImplicitArgsSize;

  /// Additional Info for the AMD GPU Kernel
  std::optional<utils::KernelMetaDataTy> KernelInfo;
  /// CodeGen generate WGSize
  uint16_t ConstWGSize;

  /// Indicate whether this Kernel requires host services
  bool NeedsHostServices;

  /// Global for host service device thread buffer
  GlobalTy ServiceThreadDeviceBufferGlobal;

  /// Global handler for hostservices buffer
  GenericGlobalHandlerTy *HostServiceBufferHandler;

  /// Lower number of threads if tripcount is low. This should produce
  /// a larger number of teams if allowed by other constraints.
  std::pair<bool, uint32_t> adjustNumThreadsForLowTripCount(
      GenericDeviceTy &GenericDevice, uint32_t BlockSize,
      uint64_t LoopTripCount, uint32_t ThreadLimitClause[3]) const override {
    uint32_t NumThreads = BlockSize;

    // If there is an override already, do nothing
    if (NumThreads != GenericDevice.getDefaultNumThreads())
      return std::make_pair(false, NumThreads);

    // If tripcount not set or not low, do nothing.
    if ((LoopTripCount == 0) || (LoopTripCount > GenericDevice.getOMPXLowTripCount()))
      return std::make_pair(false, NumThreads);

    // Environment variable present, do nothing.
    if (GenericDevice.getOMPTeamsThreadLimit() > 0)
      return std::make_pair(false, NumThreads);

    // num_threads clause present, do nothing.
    if ((ThreadLimitClause[0] > 0) && (ThreadLimitClause[0] != (uint32_t)-1))
      return std::make_pair(false, NumThreads);

    // If generic, generic-SPMD, or Xteam reduction kernel, do nothing.
    if (isGenericMode() || isGenericSPMDMode() || isXTeamReductionsMode())
      return std::make_pair(false, NumThreads);

    // Reduce the blocksize as long as it is above the tunable limit.
    while (NumThreads > GenericDevice.getOMPXSmallBlockSize())
      NumThreads >>= 1;
    return std::make_pair(true, NumThreads);
  }

  /// Get the number of threads and blocks for the kernel based on the
  /// user-defined threads and block clauses.
  uint32_t getNumThreads(GenericDeviceTy &GenericDevice,
                         uint32_t ThreadLimitClause[3]) const override {
    assert(ThreadLimitClause[1] == 0 && ThreadLimitClause[2] == 0 &&
           "Multi dimensional launch not supported yet.");

    // Honor OMP_TEAMS_THREAD_LIMIT environment variable and
    // num_threads/thread_limit clause for BigJumpLoop and NoLoop kernel types.
    int32_t TeamsThreadLimitEnvVar = GenericDevice.getOMPTeamsThreadLimit();
    if (isBigJumpLoopMode() || isNoLoopMode()) {
      if (TeamsThreadLimitEnvVar > 0)
        return std::min(static_cast<int32_t>(ConstWGSize),
                        TeamsThreadLimitEnvVar);

      if ((ThreadLimitClause[0] > 0) && (ThreadLimitClause[0] != (uint32_t)-1))
        return std::min(static_cast<uint32_t>(ConstWGSize),
                        ThreadLimitClause[0]);
    }

    if (isNoLoopMode() || isBigJumpLoopMode() || isXTeamReductionsMode())
      return ConstWGSize;

    if (ThreadLimitClause[0] > 0 && isGenericMode()) {
      if (ThreadLimitClause[0] == (uint32_t)-1)
        ThreadLimitClause[0] = PreferredNumThreads;
      else
        ThreadLimitClause[0] += GenericDevice.getWarpSize();
    }

    return std::min(MaxNumThreads, (ThreadLimitClause[0] > 0)
                                       ? ThreadLimitClause[0]
                                       : PreferredNumThreads);
  }
  uint64_t getNumBlocks(GenericDeviceTy &GenericDevice,
                        uint32_t NumTeamsClause[3], uint64_t LoopTripCount,
                        uint32_t NumThreads) const override {
    assert(NumTeamsClause[1] == 0 && NumTeamsClause[2] == 0 &&
           "Multi dimensional launch not supported yet.");

    const auto getNumGroupsFromThreadsAndTripCount =
        [](const uint64_t TripCount, const uint32_t NumThreads) {
          return ((TripCount - 1) / NumThreads) + 1;
        };
    uint64_t DeviceNumCUs = GenericDevice.getNumComputeUnits(); // FIXME

    if (isNoLoopMode()) {
      return LoopTripCount > 0 ? getNumGroupsFromThreadsAndTripCount(
                                     LoopTripCount, NumThreads)
                               : 1;
    }

    if (isBigJumpLoopMode()) {
      uint64_t NumGroups = 1;
      // Cannot assert a non-zero tripcount. Instead, launch with 1 team if the
      // tripcount is indeed zero.
      if (LoopTripCount > 0)
        NumGroups =
            getNumGroupsFromThreadsAndTripCount(LoopTripCount, NumThreads);

      // Honor OMP_NUM_TEAMS environment variable for BigJumpLoop kernel type.
      int32_t NumTeamsEnvVar = GenericDevice.getOMPNumTeams();
      if (NumTeamsEnvVar > 0 && NumTeamsEnvVar <= GenericDevice.getBlockLimit())
        NumGroups = std::min(static_cast<uint64_t>(NumTeamsEnvVar), NumGroups);
      // Honor num_teams clause but lower it if tripcount dictates to
      else if (NumTeamsClause[0] > 0 &&
               NumTeamsClause[0] <= GenericDevice.getBlockLimit()) {
        NumGroups =
            std::min(static_cast<uint64_t>(NumTeamsClause[0]), NumGroups);
      } else {
        // num_teams clause is not specified. Choose lower of tripcount-based
        // num-groups and a value that maximizes occupancy. At this point, aim
        // to have 16 wavefronts in a CU.
        // TODO: This logic needs to be moved to the AMDGPU plugin.
        uint64_t NumWavesInGroup = NumThreads / GenericDevice.getWarpSize();
        uint64_t MaxOccupancyFactor =
            NumWavesInGroup ? (16 / NumWavesInGroup) : 16;
        NumGroups = std::min(NumGroups, MaxOccupancyFactor * DeviceNumCUs);
      }
      return NumGroups;
    }

    if (isXTeamReductionsMode()) {
      uint64_t NumGroups = 0;
      // Honor OMP_NUM_TEAMS environment variable for XteamReduction kernel
      // type.
      int32_t NumTeamsEnvVar = GenericDevice.getOMPNumTeams();
      if (NumTeamsEnvVar > 0 && NumTeamsEnvVar <= GenericDevice.getBlockLimit())
        NumGroups = NumTeamsEnvVar;
      else if (NumTeamsClause[0] > 0 &&
               NumTeamsClause[0] <= GenericDevice.getBlockLimit()) {
        NumGroups = NumTeamsClause[0];
      } else {
        // If num_teams clause is not specified, we allow a max of 2*CU teams
        if (NumThreads > 0) {
          const uint64_t UIntTwo = 2;
          NumGroups =
              DeviceNumCUs *
              std::min(UIntTwo, static_cast<uint64_t>(1024 / NumThreads));
        } else {
          NumGroups = DeviceNumCUs;
        }
        // Ensure we don't have a large number of teams running if the tripcount
        // is low
        uint64_t NumGroupsFromTripCount = 1;
        if (LoopTripCount > 0)
          NumGroupsFromTripCount =
              getNumGroupsFromThreadsAndTripCount(LoopTripCount, NumThreads);
        NumGroups = std::min(NumGroups, NumGroupsFromTripCount);
      }
      // For now, we don't allow number of teams beyond 512.
      uint64_t fiveTwelve = 512;
      NumGroups = std::min(fiveTwelve, NumGroups);
      return NumGroups;
    }

    if (NumTeamsClause[0] > 0) {
      // TODO: We need to honor any value and consequently allow more than the
      // block limit. For this we might need to start multiple kernels or let
      // the blocks start again until the requested number has been started.
      return std::min(NumTeamsClause[0], GenericDevice.getBlockLimit());
    }

    uint64_t TripCountNumBlocks = std::numeric_limits<uint64_t>::max();
    if (LoopTripCount > 0) {
      if (isSPMDMode()) {
        // We have a combined construct, i.e. `target teams distribute
        // parallel for [simd]`. We launch so many teams so that each thread
        // will execute one iteration of the loop. round up to the nearest
        // integer
        TripCountNumBlocks = ((LoopTripCount - 1) / NumThreads) + 1;
      } else {
        assert((isGenericMode() || isGenericSPMDMode()) &&
               "Unexpected execution mode!");
        // If we reach this point, then we have a non-combined construct, i.e.
        // `teams distribute` with a nested `parallel for` and each team is
        // assigned one iteration of the `distribute` loop. E.g.:
        //
        // #pragma omp target teams distribute
        // for(...loop_tripcount...) {
        //   #pragma omp parallel for
        //   for(...) {}
        // }
        //
        // Threads within a team will execute the iterations of the `parallel`
        // loop.
        TripCountNumBlocks = LoopTripCount;
      }
    }
    // If the loops are long running we rather reuse blocks than spawn too many.
    uint32_t PreferredNumBlocks = std::min(uint32_t(TripCountNumBlocks),
                                           getDefaultNumBlocks(GenericDevice));
    return std::min(PreferredNumBlocks, GenericDevice.getBlockLimit());
  }
};

/// Class representing an HSA signal. Signals are used to define dependencies
/// between asynchronous operations: kernel launches and memory transfers.
struct AMDGPUSignalTy {
  /// Create an empty signal.
  AMDGPUSignalTy() : Signal({0}), UseCount() {}
  AMDGPUSignalTy(AMDGPUDeviceTy &Device) : Signal({0}), UseCount() {}

  /// Initialize the signal with an initial value.
  Error init(uint32_t InitialValue = 1) {
    hsa_status_t Status =
        hsa_amd_signal_create(InitialValue, 0, nullptr, 0, &Signal);
    return Plugin::check(Status, "Error in hsa_signal_create: %s");
  }

  /// Deinitialize the signal.
  Error deinit() {
    hsa_status_t Status = hsa_signal_destroy(Signal);
    return Plugin::check(Status, "Error in hsa_signal_destroy: %s");
  }

  /// Wait until the signal gets a zero value.
  Error wait(const uint64_t ActiveTimeout = 0) const {
    if (ActiveTimeout) {
      hsa_signal_value_t Got = 1;
      Got = hsa_signal_wait_scacquire(Signal, HSA_SIGNAL_CONDITION_EQ, 0,
                                      ActiveTimeout, HSA_WAIT_STATE_ACTIVE);
      if (Got == 0)
        return Plugin::success();
    }
    while (hsa_signal_wait_scacquire(Signal, HSA_SIGNAL_CONDITION_EQ, 0,
                                     UINT64_MAX, HSA_WAIT_STATE_BLOCKED) != 0)
      ;
    return Plugin::success();
  }

  /// Load the value on the signal.
  hsa_signal_value_t load() const { return hsa_signal_load_scacquire(Signal); }

  /// Signal decrementing by one.
  void signal() {
    assert(load() > 0 && "Invalid signal value");
    hsa_signal_subtract_screlease(Signal, 1);
  }

  /// Reset the signal value before reusing the signal. Do not call this
  /// function if the signal is being currently used by any watcher, such as a
  /// plugin thread or the HSA runtime.
  void reset() { hsa_signal_store_screlease(Signal, 1); }

  /// Increase the number of concurrent uses.
  void increaseUseCount() { UseCount.increase(); }

  /// Decrease the number of concurrent uses and return whether was the last.
  bool decreaseUseCount() { return UseCount.decrease(); }

  hsa_signal_t get() const { return Signal; }

private:
  /// The underlying HSA signal.
  hsa_signal_t Signal;

  /// Reference counter for tracking the concurrent use count. This is mainly
  /// used for knowing how many streams are using the signal.
  RefCountTy<> UseCount;
};

/// Classes for holding AMDGPU signals and managing signals.
using AMDGPUSignalRef = AMDGPUResourceRef<AMDGPUSignalTy>;
using AMDGPUSignalManagerTy = GenericDeviceResourceManagerTy<AMDGPUSignalRef>;

/// Class holding an HSA queue to submit kernel and barrier packets.
struct AMDGPUQueueTy {
  /// Create an empty queue.
  AMDGPUQueueTy() : Queue(nullptr), Mutex() {}

  /// Initialize a new queue belonging to a specific agent.
  Error init(hsa_agent_t Agent, int32_t QueueSize) {
    hsa_status_t Status =
        hsa_queue_create(Agent, QueueSize, HSA_QUEUE_TYPE_MULTI, callbackError,
                         nullptr, UINT32_MAX, UINT32_MAX, &Queue);
    OMPT_IF_TRACING_ENABLED(
        hsa_amd_profiling_set_profiler_enabled(Queue, /*Enable=*/1););
    return Plugin::check(Status, "Error in hsa_queue_create: %s");
  }

  /// Deinitialize the queue and destroy its resources.
  Error deinit() {
    if (!Queue)
      return Plugin::success();

    // Don't bother turning OFF profiling, the queue is going away anyways.
    hsa_status_t Status = hsa_queue_destroy(Queue);
    return Plugin::check(Status, "Error in hsa_queue_destroy: %s");
  }

  /// Returns if this queue is considered busy
  bool isBusy() { return Busy.load() > 0; }

  /// Returns if the underlying HSA queue is initialized
  bool isInitialized() { return Queue != nullptr; }

  /// Decrement busy count of the queue object
  void decBusy() { Busy.fetch_sub(1); }

  /// Increase busy count ob the queue object
  void incBusy() { Busy.fetch_add(1); }

  /// Push a kernel launch to the queue. The kernel launch requires an output
  /// signal and can define an optional input signal (nullptr if none).
  Error pushKernelLaunch(const AMDGPUKernelTy &Kernel, void *KernelArgs,
                         uint32_t NumThreads, uint64_t NumBlocks,
                         uint32_t GroupSize, AMDGPUSignalTy *OutputSignal,
                         AMDGPUSignalTy *InputSignal) {
    assert(OutputSignal && "Invalid kernel output signal");

    // Lock the queue during the packet publishing process. Notice this blocks
    // the addition of other packets to the queue. The following piece of code
    // should be lightweight; do not block the thread, allocate memory, etc.
    std::lock_guard<std::mutex> Lock(Mutex);

    // Avoid defining the input dependency if already satisfied.
    if (InputSignal && !InputSignal->load())
      InputSignal = nullptr;

    // Add a barrier packet before the kernel packet in case there is a pending
    // preceding operation. The barrier packet will delay the processing of
    // subsequent queue's packets until the barrier input signal are satisfied.
    // No need output signal needed because the dependency is already guaranteed
    // by the queue barrier itself.
    if (InputSignal)
      if (auto Err = pushBarrierImpl(nullptr, InputSignal))
        return Err;

    // Now prepare the kernel packet.
    uint64_t PacketId;
    hsa_kernel_dispatch_packet_t *Packet = acquirePacket(PacketId);
    assert(Packet && "Invalid packet");

    // The header of the packet is written in the last moment.
    Packet->setup = UINT16_C(1) << HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS;
    Packet->workgroup_size_x = NumThreads;
    Packet->workgroup_size_y = 1;
    Packet->workgroup_size_z = 1;
    Packet->reserved0 = 0;
    Packet->grid_size_x = NumBlocks * NumThreads;
    Packet->grid_size_y = 1;
    Packet->grid_size_z = 1;
    Packet->private_segment_size = Kernel.getPrivateSize();
    Packet->group_segment_size = GroupSize;
    Packet->kernel_object = Kernel.getKernelObject();
    Packet->kernarg_address = KernelArgs;
    Packet->reserved2 = 0;
    Packet->completion_signal = OutputSignal->get();

    // Publish the packet. Do not modify the packet after this point.
    publishKernelPacket(PacketId, Packet);

    return Plugin::success();
  }

  /// Push a barrier packet that will wait up to two input signals. All signals
  /// are optional (nullptr if none).
  Error pushBarrier(AMDGPUSignalTy *OutputSignal,
                    const AMDGPUSignalTy *InputSignal1,
                    const AMDGPUSignalTy *InputSignal2) {
    // Lock the queue during the packet publishing process.
    std::lock_guard<std::mutex> Lock(Mutex);

    // Push the barrier with the lock acquired.
    return pushBarrierImpl(OutputSignal, InputSignal1, InputSignal2);
  }

  /// Return the pointer to the underlying HSA queue
  hsa_queue_t *getHsaQueue() {
    assert(Queue && "HSA Queue initialized");
    return Queue;
  }

private:
  /// Push a barrier packet that will wait up to two input signals. Assumes the
  /// the queue lock is acquired.
  Error pushBarrierImpl(AMDGPUSignalTy *OutputSignal,
                        const AMDGPUSignalTy *InputSignal1,
                        const AMDGPUSignalTy *InputSignal2 = nullptr) {
    // Add a queue barrier waiting on both the other stream's operation and the
    // last operation on the current stream (if any).
    uint64_t PacketId;
    hsa_barrier_and_packet_t *Packet =
        (hsa_barrier_and_packet_t *)acquirePacket(PacketId);
    assert(Packet && "Invalid packet");

    Packet->reserved0 = 0;
    Packet->reserved1 = 0;
    Packet->dep_signal[0] = {0};
    Packet->dep_signal[1] = {0};
    Packet->dep_signal[2] = {0};
    Packet->dep_signal[3] = {0};
    Packet->dep_signal[4] = {0};
    Packet->reserved2 = 0;
    Packet->completion_signal = {0};

    // Set input and output dependencies if needed.
    if (OutputSignal)
      Packet->completion_signal = OutputSignal->get();
    if (InputSignal1)
      Packet->dep_signal[0] = InputSignal1->get();
    if (InputSignal2)
      Packet->dep_signal[1] = InputSignal2->get();

    // Publish the packet. Do not modify the packet after this point.
    publishBarrierPacket(PacketId, Packet);

    return Plugin::success();
  }

  /// Acquire a packet from the queue. This call may block the thread if there
  /// is no space in the underlying HSA queue. It may need to wait until the HSA
  /// runtime processes some packets. Assumes the queue lock is acquired.
  hsa_kernel_dispatch_packet_t *acquirePacket(uint64_t &PacketId) {
    // Increase the queue index with relaxed memory order. Notice this will need
    // another subsequent atomic operation with acquire order.
    PacketId = hsa_queue_add_write_index_relaxed(Queue, 1);

    // Wait for the package to be available. Notice the atomic operation uses
    // the acquire memory order.
    while (PacketId - hsa_queue_load_read_index_scacquire(Queue) >= Queue->size)
      ;

    // Return the packet reference.
    const uint32_t Mask = Queue->size - 1; // The size is a power of 2.
    return (hsa_kernel_dispatch_packet_t *)Queue->base_address +
           (PacketId & Mask);
  }

  /// Publish the kernel packet so that the HSA runtime can start processing
  /// the kernel launch. Do not modify the packet once this function is called.
  /// Assumes the queue lock is acquired.
  void publishKernelPacket(uint64_t PacketId,
                           hsa_kernel_dispatch_packet_t *Packet) {
    uint32_t *PacketPtr = reinterpret_cast<uint32_t *>(Packet);

    uint16_t Setup = Packet->setup;
    uint16_t Header = HSA_PACKET_TYPE_KERNEL_DISPATCH << HSA_PACKET_HEADER_TYPE;
    Header |= HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_ACQUIRE_FENCE_SCOPE;
    Header |= HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_RELEASE_FENCE_SCOPE;

    // Publish the packet. Do not modify the package after this point.
    __atomic_store_n(PacketPtr, Header | (Setup << 16), __ATOMIC_RELEASE);

    // Signal the doorbell about the published packet.
    hsa_signal_store_relaxed(Queue->doorbell_signal, PacketId);
  }

  /// Publish the barrier packet so that the HSA runtime can start processing
  /// the barrier. Next packets in the queue will not be processed until all
  /// barrier dependencies (signals) are satisfied. Assumes the queue is locked
  void publishBarrierPacket(uint64_t PacketId,
                            hsa_barrier_and_packet_t *Packet) {
    uint32_t *PacketPtr = reinterpret_cast<uint32_t *>(Packet);

    uint16_t Setup = 0;
    uint16_t Header = HSA_PACKET_TYPE_BARRIER_AND << HSA_PACKET_HEADER_TYPE;
    Header |= HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_ACQUIRE_FENCE_SCOPE;
    Header |= HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_RELEASE_FENCE_SCOPE;

    // Publish the packet. Do not modify the package after this point.
    __atomic_store_n(PacketPtr, Header | (Setup << 16), __ATOMIC_RELEASE);

    // Signal the doorbell about the published packet.
    hsa_signal_store_relaxed(Queue->doorbell_signal, PacketId);
  }

  /// Callack that will be called when an error is detected on the HSA queue.
  static void callbackError(hsa_status_t Status, hsa_queue_t *Source, void *) {
    auto Err = Plugin::check(Status, "Received error in queue %p: %s", Source);
    FATAL_MESSAGE(1, "%s", toString(std::move(Err)).data());
  }

  /// The HSA queue.
  hsa_queue_t *Queue;

  /// Mutex to protect the acquiring and publishing of packets. For the moment,
  /// we need this mutex to prevent publishing packets that are not ready to be
  /// published in a multi-thread scenario. Without a queue lock, a thread T1
  /// could acquire packet P and thread T2 acquire packet P+1. Thread T2 could
  /// publish its packet P+1 (signaling the queue's doorbell) before packet P
  /// from T1 is ready to be processed. That scenario should be invalid. Thus,
  /// we use the following mutex to make packet acquiring and publishing atomic.
  /// TODO: There are other more advanced approaches to avoid this mutex using
  /// atomic operations. We can further investigate it if this is a bottleneck.
  std::mutex Mutex;

  /// Indicates that the queue is busy when > 0
  std::atomic<int> Busy{0};
};

/// Struct that implements a stream of asynchronous operations for AMDGPU
/// devices. This class relies on signals to implement streams and define the
/// dependencies between asynchronous operations.
struct AMDGPUStreamTy {
private:
  /// Utility struct holding arguments for async H2H memory copies.
  struct MemcpyArgsTy {
    void *Dst;
    const void *Src;
    size_t Size;
  };

  /// Utility struct holding arguments for freeing buffers to memory managers.
  struct ReleaseBufferArgsTy {
    void *Buffer;
    AMDGPUMemoryManagerTy *MemoryManager;
  };

  /// Utility struct holding arguments for releasing signals to signal managers.
  struct ReleaseSignalArgsTy {
    AMDGPUSignalTy *Signal;
    AMDGPUSignalManagerTy *SignalManager;
  };

  /// Utility struct holding arguments for OMPT-based kernel timing.
  struct OmptKernelTimingArgsTy {
    hsa_agent_t Agent;
    AMDGPUSignalTy *Signal;
    double TicksToTime;
  };

  /// Utility struct holding arguments for HSA lazy queue handling
  struct HSABusyQueueTy {
    AMDGPUQueueTy *Q;
  };

  /// The stream is composed of N stream's slots. The struct below represents
  /// the fields of each slot. Each slot has a signal and an optional action
  /// function. When appending an HSA asynchronous operation to the stream, one
  /// slot is consumed and used to store the operation's information. The
  /// operation's output signal is set to the consumed slot's signal. If there
  /// is a previous asynchronous operation on the previous slot, the HSA async
  /// operation's input signal is set to the signal of the previous slot. This
  /// way, we obtain a chain of dependant async operations. The action is a
  /// function that will be executed eventually after the operation is
  /// completed, e.g., for releasing a buffer.
  struct StreamSlotTy {
    /// The output signal of the stream operation. May be used by the subsequent
    /// operation as input signal.
    AMDGPUSignalTy *Signal;

    /// The action that must be performed after the operation's completion. Set
    /// to nullptr when there is no action to perform.
    Error (*ActionFunction)(void *);

    /// The OMPT action that must be performed after the operation's completion.
    /// Set to nullptr when there is no action to perform.
    Error (*OmptActionFunction)(void *);

    /// Action function to unmark an HSA queue from being busy
    Error (*BusyQueueActionFunction)(void *);

    /// Space for the action's arguments. A pointer to these arguments is passed
    /// to the action function. Notice the space of arguments is limited.
    union {
      MemcpyArgsTy MemcpyArgs;
      ReleaseBufferArgsTy ReleaseBufferArgs;
      ReleaseSignalArgsTy ReleaseSignalArgs;
    } ActionArgs;

    /// Space for the OMPT action's arguments. A pointer to these arguments is
    /// passed to the action function.
    OmptKernelTimingArgsTy OmptKernelTimingArgs;

    /// Space for Busy queue acstion's arguments
    HSABusyQueueTy BusyQueueArgs;

    /// Create an empty slot.
    StreamSlotTy()
        : Signal(nullptr), ActionFunction(nullptr), OmptActionFunction(nullptr),
          BusyQueueActionFunction(nullptr) {}

    /// Schedule a host memory copy action on the slot.
    Error schedHostMemoryCopy(void *Dst, const void *Src, size_t Size) {
      ActionFunction = memcpyAction;
      ActionArgs.MemcpyArgs = MemcpyArgsTy{Dst, Src, Size};
      return Plugin::success();
    }

    /// Schedule a release buffer action on the slot.
    Error schedReleaseBuffer(void *Buffer, AMDGPUMemoryManagerTy &Manager) {
      ActionFunction = releaseBufferAction;
      ActionArgs.ReleaseBufferArgs = ReleaseBufferArgsTy{Buffer, &Manager};
      return Plugin::success();
    }

    /// Schedule a release buffer action on the slot.
    Error schedReleaseSignal(AMDGPUSignalTy *SignalToRelease,
                             AMDGPUSignalManagerTy *SignalManager) {
      ActionFunction = releaseSignalAction;
      ActionArgs.ReleaseSignalArgs =
          ReleaseSignalArgsTy{SignalToRelease, SignalManager};
      return Plugin::success();
    }

    /// Schedule OMPT kernel timing on the slot.
    Error schedOmptKernelTiming(hsa_agent_t Agent, AMDGPUSignalTy *Signal,
                                double TicksToTime) {
      OmptActionFunction = timeKernelInNs;
      OmptKernelTimingArgs = OmptKernelTimingArgsTy{Agent, Signal, TicksToTime};
      return Plugin::success();
    }

    Error schedDecrementQueueBusyCount(AMDGPUQueueTy *Q) {
      BusyQueueActionFunction = decrementBusyCounter;
      BusyQueueArgs = HSABusyQueueTy{Q};
      return Plugin::success();
    }

    // Perform the action if needed.
    Error performAction() {
      if (!ActionFunction && !BusyQueueActionFunction
#ifdef OMPT_SUPPORT
          && !OmptActionFunction
#endif
      )
        return Plugin::success();

      // Perform the action.
      if (ActionFunction == memcpyAction) {
        if (auto Err = memcpyAction(&ActionArgs))
          return Err;
      } else if (ActionFunction == releaseBufferAction) {
        if (auto Err = releaseBufferAction(&ActionArgs))
          return Err;
      } else if (ActionFunction == releaseSignalAction) {
        if (auto Err = releaseSignalAction(&ActionArgs))
          return Err;
      } else {
        return Plugin::error("Unknown action function!");
      }

      OMPT_IF_TRACING_ENABLED(
          if (OmptActionFunction == timeKernelInNs) {
            if (auto Err = timeKernelInNs(&OmptKernelTimingArgs))
              return Err;
          } else { return Plugin::error("Unknown ompt action function!"); });

      if (BusyQueueActionFunction == decrementBusyCounter) {
        if (auto Err = decrementBusyCounter(&BusyQueueArgs))
          return Err;
      }

      BusyQueueActionFunction = nullptr;

      // Invalidate the actions.
      ActionFunction = nullptr;

#ifdef OMPT_SUPPORT
      OmptActionFunction = nullptr;
#endif

      return Plugin::success();
    }
  };

  /// The device agent where the stream was created.
  hsa_agent_t Agent;

  /// The manager of signals to reuse signals.
  AMDGPUSignalManagerTy &SignalManager;

  /// Array of stream slots. Use std::deque because it can dynamically grow
  /// without invalidating the already inserted elements. For instance, the
  /// std::vector may invalidate the elements by reallocating the internal
  /// array if there is not enough space on new insertions.
  std::deque<StreamSlotTy> Slots;

  /// The next available slot on the queue. This is reset to zero each time the
  /// stream is synchronized. It also indicates the current number of consumed
  /// slots at a given time.
  uint32_t NextSlot;

  /// The synchronization id. This number is increased each time the stream is
  /// synchronized. It is useful to detect if an AMDGPUEventTy points to an
  /// operation that was already finalized in a previous stream sycnhronize.
  uint32_t SyncCycle;

  /// Mutex to protect stream's management.
  mutable std::mutex Mutex;

  /// Timeout hint for HSA actively waiting for signal value to change
  const uint64_t StreamBusyWaitMicroseconds;

  /// Return the current number of asychronous operations on the stream.
  uint32_t size() const { return NextSlot; }

  /// Return the last valid slot on the stream.
  uint32_t last() const { return size() - 1; }

  /// Consume one slot from the stream. Since the stream uses signals on demand
  /// and releases them once the slot is no longer used, the function requires
  /// an idle signal for the new consumed slot.
  std::pair<uint32_t, AMDGPUSignalTy *> consume(AMDGPUSignalTy *OutputSignal) {
    // Double the stream size if needed. Since we use std::deque, this operation
    // does not invalidate the already added slots.
    if (Slots.size() == NextSlot)
      Slots.resize(Slots.size() * 2);

    // Update the next available slot and the stream size.
    uint32_t Curr = NextSlot++;

    // Retrieve the input signal, if any, of the current operation.
    AMDGPUSignalTy *InputSignal = (Curr > 0) ? Slots[Curr - 1].Signal : nullptr;

    // Set the output signal of the current slot.
    Slots[Curr].Signal = OutputSignal;

    return std::make_pair(Curr, InputSignal);
  }

  /// Complete all pending post actions and reset the stream after synchronizing
  /// or positively querying the stream.
  Error complete() {
    for (uint32_t Slot = 0; Slot < NextSlot; ++Slot) {
      // Take the post action of the operation if any.
      if (auto Err = Slots[Slot].performAction())
        return Err;

      // Release the slot's signal if possible. Otherwise, another user will.
      if (Slots[Slot].Signal->decreaseUseCount())
        SignalManager.returnResource(Slots[Slot].Signal);

      Slots[Slot].Signal = nullptr;
    }

    // Reset the stream slots to zero.
    NextSlot = 0;

    // Increase the synchronization id since the stream completed a sync cycle.
    SyncCycle += 1;

    return Plugin::success();
  }

  /// Make the current stream wait on a specific operation of another stream.
  /// The idea is to make the current stream waiting on two signals: 1) the last
  /// signal of the current stream, and 2) the last signal of the other stream.
  /// Use a barrier packet with two input signals.
  Error waitOnStreamOperation(AMDGPUStreamTy &OtherStream, uint32_t Slot);

  /// Callback for running a specific asynchronous operation. This callback is
  /// used for hsa_amd_signal_async_handler. The argument is the operation that
  /// should be executed. Notice we use the post action mechanism to codify the
  /// asynchronous operation.
  static bool asyncActionCallback(hsa_signal_value_t Value, void *Args) {
    StreamSlotTy *Slot = reinterpret_cast<StreamSlotTy *>(Args);
    assert(Slot && "Invalid slot");
    assert(Slot->Signal && "Invalid signal");

    // This thread is outside the stream mutex. Make sure the thread sees the
    // changes on the slot.
    std::atomic_thread_fence(std::memory_order_acquire);

    // Peform the operation.
    if (auto Err = Slot->performAction())
      FATAL_MESSAGE(1, "Error peforming post action: %s",
                    toString(std::move(Err)).data());

    // Signal the output signal to notify the asycnhronous operation finalized.
    Slot->Signal->signal();

    // Unregister callback.
    return false;
  }

  // Callback for host-to-host memory copies.
  static Error memcpyAction(void *Data) {
    MemcpyArgsTy *Args = reinterpret_cast<MemcpyArgsTy *>(Data);
    assert(Args && "Invalid arguments");
    assert(Args->Dst && "Invalid destination buffer");
    assert(Args->Src && "Invalid source buffer");

    std::memcpy(Args->Dst, Args->Src, Args->Size);

    return Plugin::success();
  }

  // Callback for releasing a memory buffer to a memory manager.
  static Error releaseBufferAction(void *Data) {
    ReleaseBufferArgsTy *Args = reinterpret_cast<ReleaseBufferArgsTy *>(Data);
    assert(Args && "Invalid arguments");
    assert(Args->MemoryManager && "Invalid memory manager");
    assert(Args->Buffer && "Invalid buffer");

    // Release the allocation to the memory manager.
    return Args->MemoryManager->deallocate(Args->Buffer);
  }

  static Error releaseSignalAction(void *Data) {
    ReleaseSignalArgsTy *Args = reinterpret_cast<ReleaseSignalArgsTy *>(Data);
    assert(Args && "Invalid arguments");
    assert(Args->Signal && "Invalid signal");
    assert(Args->SignalManager && "Invalid signal manager");

    // Release the signal if needed.
    if (Args->Signal->decreaseUseCount())
      Args->SignalManager->returnResource(Args->Signal);

    return Plugin::success();
  }

  static Error timeKernelInNs(void *Data) {
    OmptKernelTimingArgsTy *Args =
        reinterpret_cast<OmptKernelTimingArgsTy *>(Data);
    assert(Args && "Invalid arguments");
    assert(Args->Signal && "Invalid signal");
    DP("Getting kernel dispatch timing for OMPT trace records\n");
    hsa_amd_profiling_dispatch_time_t TimeRec;
    hsa_status_t Status = hsa_amd_profiling_get_dispatch_time(
        Args->Agent, Args->Signal->get(), &TimeRec);
    ::setOmptTimestamp(TimeRec.start * Args->TicksToTime,
                       TimeRec.end * Args->TicksToTime);
    return Plugin::check(Status,
                         "Error in hsa_amd_profiling_get_dispatch_time");
  }

  static Error decrementBusyCounter(void *Data) {
    HSABusyQueueTy *Args = reinterpret_cast<HSABusyQueueTy *>(Data);
    assert(Args && "Valid arguments");
    Args->Q->decBusy();
    return Plugin::success();
  }

  AMDGPUDeviceTy &Device;

public:
  /// Create an empty stream associated with a specific device.
  AMDGPUStreamTy(AMDGPUDeviceTy &Device);

  /// Intialize the stream's signals.
  Error init() { return Plugin::success(); }

  /// Deinitialize the stream's signals.
  Error deinit() { return Plugin::success(); }

  hsa_queue_t *getHsaQueue();

  /// Push a asynchronous kernel to the stream. The kernel arguments must be
  /// placed in a special allocation for kernel args and must keep alive until
  /// the kernel finalizes. Once the kernel is finished, the stream will release
  /// the kernel args buffer to the specified memory manager.
  Error pushKernelLaunch(const AMDGPUKernelTy &Kernel, void *KernelArgs,
                         uint32_t NumThreads, uint64_t NumBlocks,
                         uint32_t GroupSize,
                         AMDGPUMemoryManagerTy &MemoryManager);

  /// Push an asynchronous memory copy between pinned memory buffers.
  Error pushPinnedMemoryCopyAsync(void *Dst, const void *Src,
                                  uint64_t CopySize) {
    // Retrieve an available signal for the operation's output.
    AMDGPUSignalTy *OutputSignal = SignalManager.getResource();
    OutputSignal->reset();
    OutputSignal->increaseUseCount();

    std::lock_guard<std::mutex> Lock(Mutex);

    // Consume stream slot and compute dependencies.
    auto [Curr, InputSignal] = consume(OutputSignal);

    // Avoid defining the input dependency if already satisfied.
    if (InputSignal && !InputSignal->load())
      InputSignal = nullptr;

    // Issue the async memory copy.
    hsa_status_t Status;
    if (InputSignal) {
      hsa_signal_t InputSignalRaw = InputSignal->get();
      Status = hsa_amd_memory_async_copy(Dst, Agent, Src, Agent, CopySize, 1,
                                         &InputSignalRaw, OutputSignal->get());
    } else
      Status = hsa_amd_memory_async_copy(Dst, Agent, Src, Agent, CopySize, 0,
                                         nullptr, OutputSignal->get());
    return Plugin::check(Status, "Error in hsa_amd_memory_async_copy: %s");
  }

  /// Push an asynchronous memory copy device-to-host involving an unpinned
  /// memory buffer. The operation consists of a two-step copy from the
  /// device buffer to an intermediate pinned host buffer, and then, to a
  /// unpinned host buffer. Both operations are asynchronous and dependant.
  /// The intermediate pinned buffer will be released to the specified memory
  /// manager once the operation completes.
  Error pushMemoryCopyD2HAsync(void *Dst, const void *Src, void *Inter,
                               uint64_t CopySize,
                               AMDGPUMemoryManagerTy &MemoryManager) {
    // TODO: Managers should define a function to retrieve multiple resources
    // in a single call.
    // Retrieve available signals for the operation's outputs.
    AMDGPUSignalTy *OutputSignal1 = SignalManager.getResource();
    AMDGPUSignalTy *OutputSignal2 = SignalManager.getResource();
    OutputSignal1->reset();
    OutputSignal2->reset();
    OutputSignal1->increaseUseCount();
    OutputSignal2->increaseUseCount();

    std::lock_guard<std::mutex> Lock(Mutex);

    // Consume stream slot and compute dependencies.
    auto [Curr, InputSignal] = consume(OutputSignal1);

    // Avoid defining the input dependency if already satisfied.
    if (InputSignal && !InputSignal->load())
      InputSignal = nullptr;

    // Setup the post action for releasing the intermediate buffer.
    if (auto Err = Slots[Curr].schedReleaseBuffer(Inter, MemoryManager))
      return Err;

    // Issue the first step: device to host transfer. Avoid defining the input
    // dependency if already satisfied.
    hsa_status_t Status;
    if (InputSignal) {
      hsa_signal_t InputSignalRaw = InputSignal->get();
      Status = hsa_amd_memory_async_copy(Inter, Agent, Src, Agent, CopySize, 1,
                                         &InputSignalRaw, OutputSignal1->get());
    } else {
      Status = hsa_amd_memory_async_copy(Inter, Agent, Src, Agent, CopySize, 0,
                                         nullptr, OutputSignal1->get());
    }

    if (auto Err =
            Plugin::check(Status, "Error in hsa_amd_memory_async_copy: %s"))
      return Err;

    // Consume another stream slot and compute dependencies.
    std::tie(Curr, InputSignal) = consume(OutputSignal2);
    assert(InputSignal && "Invalid input signal");

    // The std::memcpy is done asynchronously using an async handler. We store
    // the function's information in the action but it's not actually an action.
    if (auto Err = Slots[Curr].schedHostMemoryCopy(Dst, Inter, CopySize))
      return Err;

    // Make changes on this slot visible to the async handler's thread.
    std::atomic_thread_fence(std::memory_order_release);

    // Issue the second step: host to host transfer.
    Status = hsa_amd_signal_async_handler(
        InputSignal->get(), HSA_SIGNAL_CONDITION_EQ, 0, asyncActionCallback,
        (void *)&Slots[Curr]);

    return Plugin::check(Status, "Error in hsa_amd_signal_async_handler: %s");
  }

  /// Push an asynchronous memory copy host-to-device involving an unpinned
  /// memory buffer. The operation consists of a two-step copy from the
  /// unpinned host buffer to an intermediate pinned host buffer, and then, to
  /// the pinned host buffer. Both operations are asynchronous and dependant.
  /// The intermediate pinned buffer will be released to the specified memory
  /// manager once the operation completes.
  Error pushMemoryCopyH2DAsync(void *Dst, const void *Src, void *Inter,
                               uint64_t CopySize,
                               AMDGPUMemoryManagerTy &MemoryManager) {
    // Retrieve available signals for the operation's outputs.
    AMDGPUSignalTy *OutputSignal1 = SignalManager.getResource();
    AMDGPUSignalTy *OutputSignal2 = SignalManager.getResource();
    OutputSignal1->reset();
    OutputSignal2->reset();
    OutputSignal1->increaseUseCount();
    OutputSignal2->increaseUseCount();

    AMDGPUSignalTy *OutputSignal = OutputSignal1;

    std::lock_guard<std::mutex> Lock(Mutex);

    // Consume stream slot and compute dependencies.
    auto [Curr, InputSignal] = consume(OutputSignal);

    // Avoid defining the input dependency if already satisfied.
    if (InputSignal && !InputSignal->load())
      InputSignal = nullptr;

    // Issue the first step: host to host transfer.
    if (InputSignal) {
      // The std::memcpy is done asynchronously using an async handler. We store
      // the function's information in the action but it is not actually a
      // post action.
      if (auto Err = Slots[Curr].schedHostMemoryCopy(Inter, Src, CopySize))
        return Err;

      // Make changes on this slot visible to the async handler's thread.
      std::atomic_thread_fence(std::memory_order_release);

      hsa_status_t Status = hsa_amd_signal_async_handler(
          InputSignal->get(), HSA_SIGNAL_CONDITION_EQ, 0, asyncActionCallback,
          (void *)&Slots[Curr]);

      if (auto Err = Plugin::check(Status,
                                   "Error in hsa_amd_signal_async_handler: %s"))
        return Err;

      // Let's use now the second output signal.
      OutputSignal = OutputSignal2;

      // Consume another stream slot and compute dependencies.
      std::tie(Curr, InputSignal) = consume(OutputSignal);
    } else {
      // All preceding operations completed, copy the memory synchronously.
      std::memcpy(Inter, Src, CopySize);

      // Return the second signal because it will not be used.
      OutputSignal2->decreaseUseCount();
      SignalManager.returnResource(OutputSignal2);
    }

    // Setup the post action to release the intermediate pinned buffer.
    if (auto Err = Slots[Curr].schedReleaseBuffer(Inter, MemoryManager))
      return Err;

    // Issue the second step: host to device transfer. Avoid defining the input
    // dependency if already satisfied.
    hsa_status_t Status;
    if (InputSignal && InputSignal->load()) {
      hsa_signal_t InputSignalRaw = InputSignal->get();
      Status = hsa_amd_memory_async_copy(Dst, Agent, Inter, Agent, CopySize, 1,
                                         &InputSignalRaw, OutputSignal->get());
    } else
      Status = hsa_amd_memory_async_copy(Dst, Agent, Inter, Agent, CopySize, 0,
                                         nullptr, OutputSignal->get());

    return Plugin::check(Status, "Error in hsa_amd_memory_async_copy: %s");
  }

  /// Synchronize with the stream. The current thread waits until all operations
  /// are finalized and it performs the pending post actions (i.e., releasing
  /// intermediate buffers).
  Error synchronize() {
    std::lock_guard<std::mutex> Lock(Mutex);

    // No need to synchronize anything.
    if (size() == 0)
      return Plugin::success();

    // Wait until all previous operations on the stream have completed.
    if (auto Err = Slots[last()].Signal->wait(StreamBusyWaitMicroseconds))
      return Err;

    // Reset the stream and perform all pending post actions.
    return complete();
  }

  /// Query the stream and complete pending post actions if operations finished.
  /// Return whether all the operations completed. This operation does not block
  /// the calling thread.
  Expected<bool> query() {
    std::lock_guard<std::mutex> Lock(Mutex);

    // No need to query anything.
    if (size() == 0)
      return true;

    // The last operation did not complete yet. Return directly.
    if (Slots[last()].Signal->load())
      return false;

    // Reset the stream and perform all pending post actions.
    if (auto Err = complete())
      return std::move(Err);

    return true;
  }

  /// Record the state of the stream on an event.
  Error recordEvent(AMDGPUEventTy &Event) const;

  /// Make the stream wait on an event.
  Error waitEvent(const AMDGPUEventTy &Event);
};

/// Class representing an event on AMDGPU. The event basically stores some
/// information regarding the state of the recorded stream.
struct AMDGPUEventTy {
  /// Create an empty event.
  AMDGPUEventTy(AMDGPUDeviceTy &Device)
      : RecordedStream(nullptr), RecordedSlot(-1), RecordedSyncCycle(-1) {}

  /// Initialize and deinitialize.
  Error init() { return Plugin::success(); }
  Error deinit() { return Plugin::success(); }

  /// Record the state of a stream on the event.
  Error record(AMDGPUStreamTy &Stream) {
    std::lock_guard<std::mutex> Lock(Mutex);

    // Ignore the last recorded stream.
    RecordedStream = &Stream;

    return Stream.recordEvent(*this);
  }

  /// Make a stream wait on the current event.
  Error wait(AMDGPUStreamTy &Stream) {
    std::lock_guard<std::mutex> Lock(Mutex);

    if (!RecordedStream)
      return Plugin::error("Event does not have any recorded stream");

    // Synchronizing the same stream. Do nothing.
    if (RecordedStream == &Stream)
      return Plugin::success();

    // No need to wait anything, the recorded stream already finished the
    // corresponding operation.
    if (RecordedSlot < 0)
      return Plugin::success();

    return Stream.waitEvent(*this);
  }

protected:
  /// The stream registered in this event.
  AMDGPUStreamTy *RecordedStream;

  /// The recordered operation on the recorded stream.
  int64_t RecordedSlot;

  /// The sync cycle when the stream was recorded. Used to detect stale events.
  int64_t RecordedSyncCycle;

  /// Mutex to safely access event fields.
  mutable std::mutex Mutex;

  friend struct AMDGPUStreamTy;
};

Error AMDGPUStreamTy::recordEvent(AMDGPUEventTy &Event) const {
  std::lock_guard<std::mutex> Lock(Mutex);

  if (size() > 0) {
    // Record the synchronize identifier (to detect stale recordings) and
    // the last valid stream's operation.
    Event.RecordedSyncCycle = SyncCycle;
    Event.RecordedSlot = last();

    assert(Event.RecordedSyncCycle >= 0 && "Invalid recorded sync cycle");
    assert(Event.RecordedSlot >= 0 && "Invalid recorded slot");
  } else {
    // The stream is empty, everything already completed, record nothing.
    Event.RecordedSyncCycle = -1;
    Event.RecordedSlot = -1;
  }
  return Plugin::success();
}

Error AMDGPUStreamTy::waitEvent(const AMDGPUEventTy &Event) {
  // Retrieve the recorded stream on the event.
  AMDGPUStreamTy &RecordedStream = *Event.RecordedStream;

  std::scoped_lock<std::mutex, std::mutex> Lock(Mutex, RecordedStream.Mutex);

  // The recorded stream already completed the operation because the synchronize
  // identifier is already outdated.
  if (RecordedStream.SyncCycle != (uint32_t)Event.RecordedSyncCycle)
    return Plugin::success();

  // Again, the recorded stream already completed the operation, the last
  // operation's output signal is satisfied.
  if (!RecordedStream.Slots[Event.RecordedSlot].Signal->load())
    return Plugin::success();

  // Otherwise, make the current stream wait on the other stream's operation.
  return waitOnStreamOperation(RecordedStream, Event.RecordedSlot);
}

/// Abstract class that holds the common members of the actual kernel devices
/// and the host device. Both types should inherit from this class.
struct AMDGenericDeviceTy {
  AMDGenericDeviceTy() {}

  virtual ~AMDGenericDeviceTy() {}

  /// Create all memory pools which the device has access to and classify them.
  Error initMemoryPools() {
    // Retrieve all memory pools from the device agent(s).
    Error Err = retrieveAllMemoryPools();
    if (Err)
      return Err;

    for (AMDGPUMemoryPoolTy *MemoryPool : AllMemoryPools) {
      // Initialize the memory pool and retrieve some basic info.
      Error Err = MemoryPool->init();
      if (Err)
        return Err;

      if (!MemoryPool->isGlobal())
        continue;

      // Classify the memory pools depending on their properties.
      if (MemoryPool->isFineGrained()) {
        FineGrainedMemoryPools.push_back(MemoryPool);
        if (MemoryPool->supportsKernelArgs())
          ArgsMemoryPools.push_back(MemoryPool);
      } else if (MemoryPool->isCoarseGrained()) {
        CoarseGrainedMemoryPools.push_back(MemoryPool);
      }
    }
    return Plugin::success();
  }

  /// Destroy all memory pools.
  Error deinitMemoryPools() {
    for (AMDGPUMemoryPoolTy *Pool : AllMemoryPools)
      delete Pool;

    AllMemoryPools.clear();
    FineGrainedMemoryPools.clear();
    CoarseGrainedMemoryPools.clear();
    ArgsMemoryPools.clear();

    return Plugin::success();
  }
  AMDGPUMemoryPoolTy *getCoarseGrainedMemoryPool() {
    return CoarseGrainedMemoryPools[0];
  }

  /// Retrieve and construct all memory pools from the device agent(s).
  virtual Error retrieveAllMemoryPools() = 0;

  /// Get the device agent.
  virtual hsa_agent_t getAgent() const = 0;

protected:
  /// Array of all memory pools available to the host agents.
  llvm::SmallVector<AMDGPUMemoryPoolTy *> AllMemoryPools;

  /// Array of fine-grained memory pools available to the host agents.
  llvm::SmallVector<AMDGPUMemoryPoolTy *> FineGrainedMemoryPools;

  /// Array of coarse-grained memory pools available to the host agents.
  llvm::SmallVector<AMDGPUMemoryPoolTy *> CoarseGrainedMemoryPools;

  /// Array of kernel args memory pools available to the host agents.
  llvm::SmallVector<AMDGPUMemoryPoolTy *> ArgsMemoryPools;
};

/// Class representing the host device. This host device may have more than one
/// HSA host agent. We aggregate all its resources into the same instance.
struct AMDHostDeviceTy : public AMDGenericDeviceTy {
  /// Create a host device from an array of host agents.
  AMDHostDeviceTy(const llvm::SmallVector<hsa_agent_t> &HostAgents)
      : AMDGenericDeviceTy(), Agents(HostAgents), ArgsMemoryManager(),
        PinnedMemoryManager() {
    assert(HostAgents.size() && "No host agent found");
  }

  /// Initialize the host device memory pools and the memory managers for
  /// kernel args and host pinned memory allocations.
  Error init() {
    if (auto Err = initMemoryPools())
      return Err;

    if (auto Err = ArgsMemoryManager.init(getArgsMemoryPool()))
      return Err;

    if (auto Err = PinnedMemoryManager.init(getFineGrainedMemoryPool()))
      return Err;

    return Plugin::success();
  }

  /// Deinitialize memory pools and managers.
  Error deinit() {
    if (auto Err = deinitMemoryPools())
      return Err;

    if (auto Err = ArgsMemoryManager.deinit())
      return Err;

    if (auto Err = PinnedMemoryManager.deinit())
      return Err;

    return Plugin::success();
  }

  /// Retrieve and construct all memory pools from the host agents.
  Error retrieveAllMemoryPools() override {
    // Iterate through the available pools across the host agents.
    for (hsa_agent_t Agent : Agents) {
      Error Err = utils::iterateAgentMemoryPools(
          Agent, [&](hsa_amd_memory_pool_t HSAMemoryPool) {
            AMDGPUMemoryPoolTy *MemoryPool =
                new AMDGPUMemoryPoolTy(HSAMemoryPool);
            AllMemoryPools.push_back(MemoryPool);
            return HSA_STATUS_SUCCESS;
          });
      if (Err)
        return Err;
    }
    return Plugin::success();
  }

  /// Get one of the host agents. Return always the first agent.
  hsa_agent_t getAgent() const override { return Agents[0]; }

  /// Get a memory pool for fine-grained allocations.
  AMDGPUMemoryPoolTy &getFineGrainedMemoryPool() {
    assert(!FineGrainedMemoryPools.empty() && "No fine-grained mempool");
    // Retrive any memory pool.
    return *FineGrainedMemoryPools[0];
  }

  AMDGPUMemoryPoolTy &getCoarseGrainedMemoryPool() {
    assert(!CoarseGrainedMemoryPools.empty() && "No coarse-grained mempool");
    // Retrive any memory pool.
    return *CoarseGrainedMemoryPools[0];
  }

  /// Get a memory pool for kernel args allocations.
  AMDGPUMemoryPoolTy &getArgsMemoryPool() {
    assert(!ArgsMemoryPools.empty() && "No kernelargs mempool");
    // Retrieve any memory pool.
    return *ArgsMemoryPools[0];
  }

  /// Getters for kernel args and host pinned memory managers.
  AMDGPUMemoryManagerTy &getArgsMemoryManager() { return ArgsMemoryManager; }
  AMDGPUMemoryManagerTy &getPinnedMemoryManager() {
    return PinnedMemoryManager;
  }

private:
  /// Array of agents on the host side.
  const llvm::SmallVector<hsa_agent_t> Agents;

  // Memory manager for kernel arguments.
  AMDGPUMemoryManagerTy ArgsMemoryManager;

  // Memory manager for pinned memory.
  AMDGPUMemoryManagerTy PinnedMemoryManager;
};

/// Class implementing the AMDGPU device functionalities which derives from the
/// generic device class.
struct AMDGPUDeviceTy : public GenericDeviceTy, AMDGenericDeviceTy {
  // Create an AMDGPU device with a device id and default AMDGPU grid values.
  AMDGPUDeviceTy(int32_t DeviceId, int32_t NumDevices,
                 AMDHostDeviceTy &HostDevice, hsa_agent_t Agent)
      : GenericDeviceTy(DeviceId, NumDevices, {0}), AMDGenericDeviceTy(),
        OMPX_NumQueues("LIBOMPTARGET_AMDGPU_NUM_HSA_QUEUES", 4),
        OMPX_QueueSize("LIBOMPTARGET_AMDGPU_HSA_QUEUE_SIZE", 512),
        OMPX_DefaultTeamsPerCU("LIBOMPTARGET_AMDGPU_TEAMS_PER_CU", 6),
        OMPX_LowTripCount("LIBOMPTARGET_AMDGPU_LOW_TRIPCOUNT", 2000),
        OMPX_SmallBlockSize("LIBOMPTARGET_MIN_THREADS_FOR_LOW_TRIP_COUNT", 8),
        OMPX_MaxAsyncCopyBytes("LIBOMPTARGET_AMDGPU_MAX_ASYNC_COPY_BYTES",
                               1 * 1024 * 1024), // 1MB
        OMPX_InitialNumSignals("LIBOMPTARGET_AMDGPU_NUM_INITIAL_HSA_SIGNALS",
                               64),
        OMPX_ForceSyncRegions("OMPX_FORCE_SYNC_REGIONS", 0),
        OMPX_StreamBusyWait("LIBOMPTARGET_AMDGPU_STREAM_BUSYWAIT", 2000000),
        AMDGPUStreamManager(*this), AMDGPUEventManager(*this),
        AMDGPUSignalManager(*this), Agent(Agent), HostDevice(HostDevice),
        Queues() {}

  ~AMDGPUDeviceTy() {}

  /// Returns the maximum of HSA queues to create
  /// This reads a non-cached environment variable, don't call everywhere.
  uint32_t getMaxNumHsaQueues() const {
    // In case this environment variable is set: respect it and give it
    // precendence
    if (const char *GPUMaxHwQsEnv = getenv("GPU_MAX_HW_QUEUES")) {
      uint32_t MaxGPUHwQueues = std::atoi(GPUMaxHwQsEnv);
      if (MaxGPUHwQueues != OMPX_NumQueues)
        DP("Different numbers of maximum HSA queues specified. Using %u\n",
           MaxGPUHwQueues);

      return MaxGPUHwQueues;
    }
    // Otherwise use the regular environment variable
    return OMPX_NumQueues;
  }

  virtual uint32_t getOMPXLowTripCount() const override {
    return OMPX_LowTripCount;
  }
  virtual uint32_t getOMPXSmallBlockSize() const override {
    return OMPX_SmallBlockSize;
  }

  /// Initialize the device, its resources and get its properties.
  Error initImpl(GenericPluginTy &Plugin) override {
    // First setup all the memory pools.
    if (auto Err = initMemoryPools())
      return Err;

    OMPT_IF_ENABLED(::setOmptTicksToTime(););

#ifdef OMPT_SUPPORT
    // At init we capture two time points for host and device. The two
    // timepoints are spaced out to help smooth out their accuracy
    // differences.
    // libomp uses the CLOCK_REALTIME (via gettimeofday) to get
    // the value for omp_get_wtime. So we use the same clock here to calculate
    // the slope/offset and convert device time to omp_get_wtime via
    // translate_time.
    double HostRef1 = 0;
    uint64_t DeviceRef1 = 0;
#endif
    // Take the first timepoints.
    OMPT_IF_ENABLED(startH2DTimeRate(&HostRef1, &DeviceRef1););

    if (auto Err = preAllocateDeviceMemoryPool())
      return Err;

    char GPUName[64];
    if (auto Err = getDeviceAttr(HSA_AGENT_INFO_NAME, GPUName))
      return Err;
    ComputeUnitKind = GPUName;

    // Get the wavefront size.
    uint32_t WavefrontSize = 0;
    if (auto Err = getDeviceAttr(HSA_AGENT_INFO_WAVEFRONT_SIZE, WavefrontSize))
      return Err;
    GridValues.GV_Warp_Size = WavefrontSize;

    // Load the grid values dependending on the wavefront.
    if (WavefrontSize == 32)
      GridValues = getAMDGPUGridValues<32>();
    else if (WavefrontSize == 64)
      GridValues = getAMDGPUGridValues<64>();
    else
      return Plugin::error("Unexpected AMDGPU wavefront %d", WavefrontSize);

    // Get maximum number of workitems per workgroup.
    uint16_t WorkgroupMaxDim[3];
    if (auto Err =
            getDeviceAttr(HSA_AGENT_INFO_WORKGROUP_MAX_DIM, WorkgroupMaxDim))
      return Err;
    GridValues.GV_Max_WG_Size = WorkgroupMaxDim[0];

    // Get maximum number of workgroups.
    hsa_dim3_t GridMaxDim;
    if (auto Err = getDeviceAttr(HSA_AGENT_INFO_GRID_MAX_DIM, GridMaxDim))
      return Err;

    GridValues.GV_Max_Teams = GridMaxDim.x / GridValues.GV_Max_WG_Size;
    if (GridValues.GV_Max_Teams == 0)
      return Plugin::error("Maximum number of teams cannot be zero");

    // Compute the default number of teams.
    uint32_t ComputeUnits = 0;
    if (auto Err =
            getDeviceAttr(HSA_AMD_AGENT_INFO_COMPUTE_UNIT_COUNT, ComputeUnits))
      return Err;
    GridValues.GV_Default_Num_Teams = ComputeUnits * OMPX_DefaultTeamsPerCU;
    NumComputeUnits = ComputeUnits;

    // Get maximum size of any device queues and maximum number of queues.
    uint32_t MaxQueueSize;
    if (auto Err = getDeviceAttr(HSA_AGENT_INFO_QUEUE_MAX_SIZE, MaxQueueSize))
      return Err;

    uint32_t MaxQueues;
    if (auto Err = getDeviceAttr(HSA_AGENT_INFO_QUEUES_MAX, MaxQueues))
      return Err;

    // Compute the number of queues and their size.
    const uint32_t NumQueues = std::min(getMaxNumHsaQueues(), MaxQueues);
    const uint32_t QueueSize = std::min(OMPX_QueueSize.get(), MaxQueueSize);
    DP("Using a maximum of %u HSA queues\n", NumQueues);

    // Default-Construct each device queue (and initialize only the first) to
    // avoid unnecessary initialization overhead.
    Queues = std::vector<AMDGPUQueueTy>(NumQueues);
    // TODO: Handle errors here: abort? Gracefully? Ignore?
    if (auto Err = Queues.front().init(Agent, QueueSize))
      DP("LAZY_QUEUE: Error occurred during AMDGPUQueueTy init\n");
    ++NumInitQueues;

    // Initialize stream pool.
    if (auto Err = AMDGPUStreamManager.init(OMPX_InitialNumStreams))
      return Err;

    // Initialize event pool.
    if (auto Err = AMDGPUEventManager.init(OMPX_InitialNumEvents))
      return Err;

    // Initialize signal pool.
    if (auto Err = AMDGPUSignalManager.init(OMPX_InitialNumSignals))
      return Err;

    // Initialize memspace table to keep track of coarse grain memory regions
    // in USM mode
    if (Plugin::get().getRequiresFlags() & OMP_REQ_UNIFIED_SHARED_MEMORY) {
      // TODO: add framework for multiple systems supporting
      // unified_shared_memory
      coarse_grain_mem_tab = new AMDGPUMemTypeBitFieldTable(
          AMDGPU_X86_64_SystemConfiguration::max_addressable_byte +
              1, // memory size
          AMDGPU_X86_64_SystemConfiguration::page_size);
    }

    // Take the second timepoints and compute the required metadata.
    OMPT_IF_ENABLED(completeH2DTimeRate(HostRef1, DeviceRef1););

    return Plugin::success();
  }

  /// Deinitialize the device and release its resources.
  Error deinitImpl() override {
    // Deinitialize the stream and event pools.
    if (auto Err = AMDGPUStreamManager.deinit())
      return Err;

    if (auto Err = AMDGPUEventManager.deinit())
      return Err;

    if (auto Err = AMDGPUSignalManager.deinit())
      return Err;

    // Close modules if necessary.
    if (!LoadedImages.empty()) {
      // Each image has its own module.
      for (DeviceImageTy *Image : LoadedImages) {
        AMDGPUDeviceImageTy &AMDImage =
            static_cast<AMDGPUDeviceImageTy &>(*Image);

        // Unload the executable of the image.
        if (auto Err = AMDImage.unloadExecutable())
          return Err;
      }
    }

    for (AMDGPUQueueTy &Queue : Queues) {
      if (auto Err = Queue.deinit())
        return Err;
    }

    // Invalidate agent reference.
    Agent = {0};

    return Plugin::success();
  }

  const uint64_t getStreamBusyWaitMicroseconds() const {
    return OMPX_StreamBusyWait;
  }

  Expected<std::unique_ptr<MemoryBuffer>>
  doJITPostProcessing(std::unique_ptr<MemoryBuffer> MB) const override {

    // TODO: We should try to avoid materialization but there seems to be no
    // good linker interface w/o file i/o.
    SmallString<128> LinkerOutputFilePath;
    std::error_code EC = sys::fs::createTemporaryFile(
        "amdgpu-pre-link-jit", ".out", LinkerOutputFilePath);
    if (EC)
      return createStringError(EC,
                               "Failed to create temporary file for linker");

    SmallString<128> LinkerInputFilePath = LinkerOutputFilePath;
    LinkerInputFilePath.pop_back_n(2);

    auto FD = raw_fd_ostream(LinkerInputFilePath.data(), EC);
    if (EC)
      return createStringError(EC, "Failed to open temporary file for linker");
    FD.write(MB->getBufferStart(), MB->getBufferSize());
    FD.close();

    const auto &ErrorOrPath = sys::findProgramByName("lld");
    if (!ErrorOrPath)
      return createStringError(inconvertibleErrorCode(),
                               "Failed to find `lld` on the PATH.");

    std::string LLDPath = ErrorOrPath.get();
    INFO(OMP_INFOTYPE_PLUGIN_KERNEL, getDeviceId(),
         "Using `%s` to link JITed amdgcn ouput.", LLDPath.c_str());

    std::string MCPU = "-plugin-opt=mcpu=" + getComputeUnitKind();

    StringRef Args[] = {LLDPath,
                        "-flavor",
                        "gnu",
                        "--no-undefined",
                        "-shared",
                        MCPU,
                        "-o",
                        LinkerOutputFilePath.data(),
                        LinkerInputFilePath.data()};

    std::string Error;
    int RC = sys::ExecuteAndWait(LLDPath, Args, std::nullopt, {}, 0, 0, &Error);
    if (RC)
      return createStringError(inconvertibleErrorCode(),
                               "Linking optimized bitcode failed: %s",
                               Error.c_str());

    return std::move(
        MemoryBuffer::getFileOrSTDIN(LinkerOutputFilePath.data()).get());
  }

  /// See GenericDeviceTy::getComputeUnitKind().
  std::string getComputeUnitKind() const override { return ComputeUnitKind; }

  uint32_t getNumComputeUnits() const override { return NumComputeUnits; }

  /// Allocate and construct an AMDGPU kernel.
  Expected<GenericKernelTy *>
  constructKernelEntry(const __tgt_offload_entry &KernelEntry,
                       DeviceImageTy &Image) override {

    Expected<OMPTgtExecModeFlags> ExecModeOrErr =
        getExecutionModeForKernel(KernelEntry.name, Image);
    if (!ExecModeOrErr)
      return ExecModeOrErr.takeError();

    // Allocate and initialize the AMDGPU kernel.
    AMDGPUKernelTy *AMDKernel = Plugin::get().allocate<AMDGPUKernelTy>();
    new (AMDKernel) AMDGPUKernelTy(KernelEntry.name, ExecModeOrErr.get());

    return AMDKernel;
  }

  /// Set the current context to this device's context. Do nothing since the
  /// AMDGPU devices do not have the concept of contexts.
  Error setContext() override { return Plugin::success(); }

  /// Get the stream of the asynchronous info sructure or get a new one.
  AMDGPUStreamTy &getStream(AsyncInfoWrapperTy &AsyncInfoWrapper) {
    AMDGPUStreamTy *&Stream = AsyncInfoWrapper.getQueueAs<AMDGPUStreamTy *>();
    if (!Stream)
      Stream = AMDGPUStreamManager.getResource();
    return *Stream;
  }

  /// Load the binary image into the device and allocate an image object.
  Expected<DeviceImageTy *> loadBinaryImpl(const __tgt_device_image *TgtImage,
                                           int32_t ImageId) override {
    // Allocate and initialize the image object.
    AMDGPUDeviceImageTy *AMDImage =
        Plugin::get().allocate<AMDGPUDeviceImageTy>();
    new (AMDImage) AMDGPUDeviceImageTy(ImageId, TgtImage);

    // Load the HSA executable.
    if (Error Err = AMDImage->loadExecutable(*this))
      return std::move(Err);

    return AMDImage;
  }

  /// Allocate memory on the device or related to the device.
  void *allocate(size_t Size, void *, TargetAllocTy Kind) override;

  /// Deallocate memory on the device or related to the device.
  int free(void *TgtPtr, TargetAllocTy Kind) override {
    if (TgtPtr == nullptr)
      return OFFLOAD_SUCCESS;

    AMDGPUMemoryPoolTy *MemoryPool = nullptr;
    switch (Kind) {
    case TARGET_ALLOC_DEFAULT:
    case TARGET_ALLOC_DEVICE:
      MemoryPool = CoarseGrainedMemoryPools[0];
      break;
    case TARGET_ALLOC_HOST:
      MemoryPool = &HostDevice.getFineGrainedMemoryPool();
      break;
    case TARGET_ALLOC_SHARED:
      MemoryPool = &HostDevice.getFineGrainedMemoryPool();
      break;
    }

    if (!MemoryPool) {
      REPORT("No memory pool for the specified allocation kind\n");
      return OFFLOAD_FAIL;
    }

    if (Error Err = MemoryPool->deallocate(TgtPtr)) {
      REPORT("%s\n", toString(std::move(Err)).data());
      return OFFLOAD_FAIL;
    }

    return OFFLOAD_SUCCESS;
  }

  /// Synchronize current thread with the pending operations on the async info.
  Error synchronizeImpl(__tgt_async_info &AsyncInfo) override {
    AMDGPUStreamTy *Stream =
        reinterpret_cast<AMDGPUStreamTy *>(AsyncInfo.Queue);
    assert(Stream && "Invalid stream");

    if (auto Err = Stream->synchronize())
      return Err;

    // Once the stream is synchronized, return it to stream pool and reset
    // AsyncInfo. This is to make sure the synchronization only works for its
    // own tasks.
    AMDGPUStreamManager.returnResource(Stream);
    AsyncInfo.Queue = nullptr;

    return Plugin::success();
  }

  /// Query for the completion of the pending operations on the async info.
  Error queryAsyncImpl(__tgt_async_info &AsyncInfo) override {
    AMDGPUStreamTy *Stream =
        reinterpret_cast<AMDGPUStreamTy *>(AsyncInfo.Queue);
    assert(Stream && "Invalid stream");

    auto CompletedOrErr = Stream->query();
    if (!CompletedOrErr)
      return CompletedOrErr.takeError();

    // Return if it the stream did not complete yet.
    if (!(*CompletedOrErr))
      return Plugin::success();

    // Once the stream is completed, return it to stream pool and reset
    // AsyncInfo. This is to make sure the synchronization only works for its
    // own tasks.
    AMDGPUStreamManager.returnResource(Stream);
    AsyncInfo.Queue = nullptr;

    return Plugin::success();
  }

  /// Pin the host buffer and return the device pointer that should be used for
  /// device transfers.
  Expected<void *> dataLockImpl(void *HstPtr, int64_t Size) override {
    void *PinnedPtr = nullptr;

    hsa_status_t Status =
        hsa_amd_memory_lock(HstPtr, Size, nullptr, 0, &PinnedPtr);
    if (auto Err = Plugin::check(Status, "Error in hsa_amd_memory_lock: %s\n"))
      return std::move(Err);

    return PinnedPtr;
  }

  /// Unpin the host buffer.
  Error dataUnlockImpl(void *HstPtr) override {
    hsa_status_t Status = hsa_amd_memory_unlock(HstPtr);
    return Plugin::check(Status, "Error in hsa_amd_memory_unlock: %s\n");
  }

  /// Check through the HSA runtime whether the \p HstPtr buffer is pinned.
  Expected<bool> isPinnedPtrImpl(void *HstPtr, void *&BaseHstPtr,
                                 void *&BaseDevAccessiblePtr,
                                 size_t &BaseSize) const override {
    hsa_amd_pointer_info_t Info;
    Info.size = sizeof(hsa_amd_pointer_info_t);

    hsa_status_t Status =
        hsa_amd_pointer_info(HstPtr, &Info, /* Allocator */ nullptr,
                             /* Number of accessible agents (out) */ nullptr,
                             /* Accessible agents */ nullptr);
    if (auto Err = Plugin::check(Status, "Error in hsa_amd_pointer_info: %s"))
      return std::move(Err);

    // The buffer may be locked or allocated through HSA allocators. Assume that
    // the buffer is host pinned if the runtime reports a HSA type.
    if (Info.type != HSA_EXT_POINTER_TYPE_LOCKED &&
        Info.type != HSA_EXT_POINTER_TYPE_HSA)
      return false;

    assert(Info.hostBaseAddress && "Invalid host pinned address");
    assert(Info.agentBaseAddress && "Invalid agent pinned address");
    assert(Info.sizeInBytes > 0 && "Invalid pinned allocation size");

    // Save the allocation info in the output parameters.
    BaseHstPtr = Info.hostBaseAddress;
    BaseDevAccessiblePtr = Info.agentBaseAddress;
    BaseSize = Info.sizeInBytes;

    return true;
  }

  /// Submit data to the device (host to device transfer).
  Error dataSubmitImpl(void *TgtPtr, const void *HstPtr, int64_t Size,
                       AsyncInfoWrapperTy &AsyncInfoWrapper) override {
    // Use one-step asynchronous operation when host memory is already pinned.
    if (void *PinnedPtr =
            PinnedAllocs.getDeviceAccessiblePtrFromPinnedBuffer(HstPtr)) {
      AMDGPUStreamTy &Stream = getStream(AsyncInfoWrapper);
      return Stream.pushPinnedMemoryCopyAsync(TgtPtr, PinnedPtr, Size);
    }

    void *PinnedHstPtr = nullptr;

    // For large transfers use synchronous behavior.
    // If OMPT is enabled or synchronous behavior is explicitly requested:
    if (OmptEnabled || OMPX_ForceSyncRegions ||
        Size >= OMPX_MaxAsyncCopyBytes) {
      if (AsyncInfoWrapper.hasQueue())
        if (auto Err = synchronize(AsyncInfoWrapper))
          return Err;

      hsa_status_t Status;
      Status = hsa_amd_memory_lock(const_cast<void *>(HstPtr), Size, nullptr, 0,
                                   &PinnedHstPtr);
      if (auto Err =
              Plugin::check(Status, "Error in hsa_amd_memory_lock: %s\n"))
        return Err;

      AMDGPUSignalTy Signal;
      if (auto Err = Signal.init())
        return Err;

      Status = hsa_amd_memory_async_copy(TgtPtr, Agent, PinnedHstPtr, Agent,
                                         Size, 0, nullptr, Signal.get());
      if (auto Err =
              Plugin::check(Status, "Error in hsa_amd_memory_async_copy: %s"))
        return Err;

      if (auto Err = Signal.wait(getStreamBusyWaitMicroseconds()))
        return Err;

      OMPT_IF_TRACING_ENABLED(recordCopyTimingInNs(Signal.get()););

      if (auto Err = Signal.deinit())
        return Err;

      Status = hsa_amd_memory_unlock(const_cast<void *>(HstPtr));
      return Plugin::check(Status, "Error in hsa_amd_memory_unlock: %s\n");
    }

    // Otherwise, use two-step copy with an intermediate pinned host buffer.
    AMDGPUMemoryManagerTy &PinnedMemoryManager =
        HostDevice.getPinnedMemoryManager();
    if (auto Err = PinnedMemoryManager.allocate(Size, &PinnedHstPtr))
      return Err;

    AMDGPUStreamTy &Stream = getStream(AsyncInfoWrapper);
    return Stream.pushMemoryCopyH2DAsync(TgtPtr, HstPtr, PinnedHstPtr, Size,
                                         PinnedMemoryManager);
  }

  /// Retrieve data from the device (device to host transfer).
  Error dataRetrieveImpl(void *HstPtr, const void *TgtPtr, int64_t Size,
                         AsyncInfoWrapperTy &AsyncInfoWrapper) override {

    // Use one-step asynchronous operation when host memory is already pinned.
    if (void *PinnedPtr =
            PinnedAllocs.getDeviceAccessiblePtrFromPinnedBuffer(HstPtr)) {
      AMDGPUStreamTy &Stream = getStream(AsyncInfoWrapper);
      return Stream.pushPinnedMemoryCopyAsync(PinnedPtr, TgtPtr, Size);
    }

    void *PinnedHstPtr = nullptr;

    // For large transfers use synchronous behavior.
    // If OMPT is enabled or synchronous behavior is explicitly requested:
    if (OmptEnabled || OMPX_ForceSyncRegions ||
        Size >= OMPX_MaxAsyncCopyBytes) {
      if (AsyncInfoWrapper.hasQueue())
        if (auto Err = synchronize(AsyncInfoWrapper))
          return Err;

      hsa_status_t Status;
      Status = hsa_amd_memory_lock(const_cast<void *>(HstPtr), Size, nullptr, 0,
                                   &PinnedHstPtr);
      if (auto Err =
              Plugin::check(Status, "Error in hsa_amd_memory_lock: %s\n"))
        return Err;

      AMDGPUSignalTy Signal;
      if (auto Err = Signal.init())
        return Err;

      Status = hsa_amd_memory_async_copy(PinnedHstPtr, Agent, TgtPtr, Agent,
                                         Size, 0, nullptr, Signal.get());
      if (auto Err =
              Plugin::check(Status, "Error in hsa_amd_memory_async_copy: %s"))
        return Err;

      if (auto Err = Signal.wait(getStreamBusyWaitMicroseconds()))
        return Err;

      OMPT_IF_TRACING_ENABLED(recordCopyTimingInNs(Signal.get()););

      if (auto Err = Signal.deinit())
        return Err;

      Status = hsa_amd_memory_unlock(const_cast<void *>(HstPtr));
      return Plugin::check(Status, "Error in hsa_amd_memory_unlock: %s\n");
    }

    // Otherwise, use two-step copy with an intermediate pinned host buffer.
    AMDGPUMemoryManagerTy &PinnedMemoryManager =
        HostDevice.getPinnedMemoryManager();
    if (auto Err = PinnedMemoryManager.allocate(Size, &PinnedHstPtr))
      return Err;

    AMDGPUStreamTy &Stream = getStream(AsyncInfoWrapper);
    return Stream.pushMemoryCopyD2HAsync(HstPtr, TgtPtr, PinnedHstPtr, Size,
                                         PinnedMemoryManager);
  }

  /// Exchange data between two devices within the plugin. This function is not
  /// supported in this plugin.
  Error dataExchangeImpl(const void *SrcPtr, GenericDeviceTy &DstGenericDevice,
                         void *DstPtr, int64_t Size,
                         AsyncInfoWrapperTy &AsyncInfoWrapper) override {
    // This function should never be called because the function
    // AMDGPUPluginTy::isDataExchangable() returns false.
    return Plugin::error("dataExchangeImpl not supported");
  }

  /// Initialize the async info for interoperability purposes.
  Error initAsyncInfoImpl(AsyncInfoWrapperTy &AsyncInfoWrapper) override {
    // TODO: Implement this function.
    return Plugin::success();
  }

  /// Initialize the device info for interoperability purposes.
  Error initDeviceInfoImpl(__tgt_device_info *DeviceInfo) override {
    DeviceInfo->Context = nullptr;

    if (!DeviceInfo->Device)
      DeviceInfo->Device = reinterpret_cast<void *>(Agent.handle);

    return Plugin::success();
  }

  Error setCoarseGrainMemoryImpl(void *ptr, int64_t size) override final {
    // track coarse grain memory pages in local table
    coarse_grain_mem_tab->insert((const uintptr_t)ptr, size);

    // Instruct ROCr that the [ptr, ptr+size-1] pages are
    // coarse grain
    hsa_amd_svm_attribute_pair_t tt;
    tt.attribute = HSA_AMD_SVM_ATTRIB_GLOBAL_FLAG;
    tt.value = HSA_AMD_SVM_GLOBAL_FLAG_COARSE_GRAINED;
    hsa_status_t err = hsa_amd_svm_attributes_set(ptr, size, &tt, 1);
    if (err != HSA_STATUS_SUCCESS) {
      return Plugin::error("Failed to switch memotry to coarse grain mode.");
    }

    return Plugin::success();
  }

  uint32_t queryCoarseGrainMemoryImpl(const void *ptr,
                                      int64_t size) override final {

    // if the table is not yet allocated, it means we have not yet gone through
    // an OpenMP pragma or API that would provoke intialization of the RTL
    if (!coarse_grain_mem_tab)
      return 0;

    return coarse_grain_mem_tab->contains((const uintptr_t)ptr, size);
  }

  /// Create an event.
  Error createEventImpl(void **EventPtrStorage) override {
    AMDGPUEventTy **Event = reinterpret_cast<AMDGPUEventTy **>(EventPtrStorage);
    *Event = AMDGPUEventManager.getResource();
    return Plugin::success();
  }

  /// Destroy a previously created event.
  Error destroyEventImpl(void *EventPtr) override {
    AMDGPUEventTy *Event = reinterpret_cast<AMDGPUEventTy *>(EventPtr);
    AMDGPUEventManager.returnResource(Event);
    return Plugin::success();
  }

  /// Record the event.
  Error recordEventImpl(void *EventPtr,
                        AsyncInfoWrapperTy &AsyncInfoWrapper) override {
    AMDGPUEventTy *Event = reinterpret_cast<AMDGPUEventTy *>(EventPtr);
    assert(Event && "Invalid event");

    AMDGPUStreamTy &Stream = getStream(AsyncInfoWrapper);

    return Event->record(Stream);
  }

  /// Make the stream wait on the event.
  Error waitEventImpl(void *EventPtr,
                      AsyncInfoWrapperTy &AsyncInfoWrapper) override {
    AMDGPUEventTy *Event = reinterpret_cast<AMDGPUEventTy *>(EventPtr);

    AMDGPUStreamTy &Stream = getStream(AsyncInfoWrapper);

    return Event->wait(Stream);
  }

  /// Synchronize the current thread with the event.
  Error syncEventImpl(void *EventPtr) override {
    return Plugin::error("Synchronize event not implemented");
  }

  /// Print information about the device.
  Error obtainInfoImpl(InfoQueueTy &Info) override {
    char TmpChar[1000];
    const char *TmpCharPtr = "Unknown";
    uint16_t Major, Minor;
    uint32_t TmpUInt, TmpUInt2;
    uint32_t CacheSize[4];
    size_t TmpSt;
    bool TmpBool;
    uint16_t WorkgrpMaxDim[3];
    hsa_dim3_t GridMaxDim;
    hsa_status_t Status, Status2;

    Status = hsa_system_get_info(HSA_SYSTEM_INFO_VERSION_MAJOR, &Major);
    Status2 = hsa_system_get_info(HSA_SYSTEM_INFO_VERSION_MINOR, &Minor);
    if (Status == HSA_STATUS_SUCCESS && Status2 == HSA_STATUS_SUCCESS)
      Info.add("HSA Runtime Version",
               std::to_string(Major) + "." + std::to_string(Minor));

    Info.add("HSA OpenMP Device Number", DeviceId);

    Status = getDeviceAttrRaw(HSA_AMD_AGENT_INFO_PRODUCT_NAME, TmpChar);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Product Name", TmpChar);

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_NAME, TmpChar);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Device Name", TmpChar);

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_VENDOR_NAME, TmpChar);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Vendor Name", TmpChar);

    hsa_device_type_t DevType;
    Status = getDeviceAttrRaw(HSA_AGENT_INFO_DEVICE, DevType);
    if (Status == HSA_STATUS_SUCCESS) {
      switch (DevType) {
      case HSA_DEVICE_TYPE_CPU:
        TmpCharPtr = "CPU";
        break;
      case HSA_DEVICE_TYPE_GPU:
        TmpCharPtr = "GPU";
        break;
      case HSA_DEVICE_TYPE_DSP:
        TmpCharPtr = "DSP";
        break;
      }
      Info.add("Device Type", TmpCharPtr);
    }

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_QUEUES_MAX, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Max Queues", TmpUInt);

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_QUEUE_MIN_SIZE, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Queue Min Size", TmpUInt);

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_QUEUE_MAX_SIZE, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Queue Max Size", TmpUInt);

    // FIXME: This is deprecated according to HSA documentation. But using
    // hsa_agent_iterate_caches and hsa_cache_get_info breaks execution during
    // runtime.
    Status = getDeviceAttrRaw(HSA_AGENT_INFO_CACHE_SIZE, CacheSize);
    if (Status == HSA_STATUS_SUCCESS) {
      Info.add("Cache");

      for (int I = 0; I < 4; I++)
        if (CacheSize[I])
          Info.add<InfoLevel2>("L" + std::to_string(I), CacheSize[I]);
    }

    Status = getDeviceAttrRaw(HSA_AMD_AGENT_INFO_CACHELINE_SIZE, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Cacheline Size", TmpUInt);

    Status = getDeviceAttrRaw(HSA_AMD_AGENT_INFO_MAX_CLOCK_FREQUENCY, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Max Clock Freq", TmpUInt, "MHz");

    Status = getDeviceAttrRaw(HSA_AMD_AGENT_INFO_COMPUTE_UNIT_COUNT, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Compute Units", TmpUInt);

    Status = getDeviceAttrRaw(HSA_AMD_AGENT_INFO_NUM_SIMDS_PER_CU, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("SIMD per CU", TmpUInt);

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_FAST_F16_OPERATION, TmpBool);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Fast F16 Operation", TmpBool);

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_WAVEFRONT_SIZE, TmpUInt2);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Wavefront Size", TmpUInt2);

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_WORKGROUP_MAX_SIZE, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Workgroup Max Size", TmpUInt);

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_WORKGROUP_MAX_DIM, WorkgrpMaxDim);
    if (Status == HSA_STATUS_SUCCESS) {
      Info.add("Workgroup Max Size per Dimension");
      Info.add<InfoLevel2>("x", WorkgrpMaxDim[0]);
      Info.add<InfoLevel2>("y", WorkgrpMaxDim[1]);
      Info.add<InfoLevel2>("z", WorkgrpMaxDim[2]);
    }

    Status = getDeviceAttrRaw(
        (hsa_agent_info_t)HSA_AMD_AGENT_INFO_MAX_WAVES_PER_CU, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS) {
      Info.add("Max Waves Per CU", TmpUInt);
      Info.add("Max Work-item Per CU", TmpUInt * TmpUInt2);
    }

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_GRID_MAX_SIZE, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Grid Max Size", TmpUInt);

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_GRID_MAX_DIM, GridMaxDim);
    if (Status == HSA_STATUS_SUCCESS) {
      Info.add("Grid Max Size per Dimension");
      Info.add<InfoLevel2>("x", GridMaxDim.x);
      Info.add<InfoLevel2>("y", GridMaxDim.y);
      Info.add<InfoLevel2>("z", GridMaxDim.z);
    }

    Status = getDeviceAttrRaw(HSA_AGENT_INFO_FBARRIER_MAX_SIZE, TmpUInt);
    if (Status == HSA_STATUS_SUCCESS)
      Info.add("Max fbarriers/Workgrp", TmpUInt);

    Info.add("Memory Pools");
    for (AMDGPUMemoryPoolTy *Pool : AllMemoryPools) {
      std::string TmpStr, TmpStr2;

      if (Pool->isGlobal())
        TmpStr = "Global";
      else if (Pool->isReadOnly())
        TmpStr = "ReadOnly";
      else if (Pool->isPrivate())
        TmpStr = "Private";
      else if (Pool->isGroup())
        TmpStr = "Group";
      else
        TmpStr = "Unknown";

      Info.add<InfoLevel2>(std::string("Pool ") + TmpStr);

      if (Pool->isGlobal()) {
        if (Pool->isFineGrained())
          TmpStr2 += "Fine Grained ";
        if (Pool->isCoarseGrained())
          TmpStr2 += "Coarse Grained ";
        if (Pool->supportsKernelArgs())
          TmpStr2 += "Kernarg ";

        Info.add<InfoLevel3>("Flags", TmpStr2);
      }

      Status = Pool->getAttrRaw(HSA_AMD_MEMORY_POOL_INFO_SIZE, TmpSt);
      if (Status == HSA_STATUS_SUCCESS)
        Info.add<InfoLevel3>("Size", TmpSt, "bytes");

      Status = Pool->getAttrRaw(HSA_AMD_MEMORY_POOL_INFO_RUNTIME_ALLOC_ALLOWED,
                                TmpBool);
      if (Status == HSA_STATUS_SUCCESS)
        Info.add<InfoLevel3>("Allocatable", TmpBool);

      Status = Pool->getAttrRaw(HSA_AMD_MEMORY_POOL_INFO_RUNTIME_ALLOC_GRANULE,
                                TmpSt);
      if (Status == HSA_STATUS_SUCCESS)
        Info.add<InfoLevel3>("Runtime Alloc Granule", TmpSt, "bytes");

      Status = Pool->getAttrRaw(
          HSA_AMD_MEMORY_POOL_INFO_RUNTIME_ALLOC_ALIGNMENT, TmpSt);
      if (Status == HSA_STATUS_SUCCESS)
        Info.add<InfoLevel3>("Runtime Alloc Alignment", TmpSt, "bytes");

      Status =
          Pool->getAttrRaw(HSA_AMD_MEMORY_POOL_INFO_ACCESSIBLE_BY_ALL, TmpBool);
      if (Status == HSA_STATUS_SUCCESS)
        Info.add<InfoLevel3>("Accessable by all", TmpBool);
    }

    Info.add("ISAs");
    auto Err = utils::iterateAgentISAs(getAgent(), [&](hsa_isa_t ISA) {
      Status = hsa_isa_get_info_alt(ISA, HSA_ISA_INFO_NAME, TmpChar);
      if (Status == HSA_STATUS_SUCCESS)
        Info.add<InfoLevel2>("Name", TmpChar);

      return Status;
    });

    // Silently consume the error.
    if (Err)
      consumeError(std::move(Err));

    return Plugin::success();
  }

  /// Get the HSA system timestamps for the input signal associated with an
  /// async copy and pass the information to libomptarget
  void recordCopyTimingInNs(hsa_signal_t signal) {
    hsa_amd_profiling_async_copy_time_t time_rec;
    hsa_status_t Status =
        hsa_amd_profiling_get_async_copy_time(signal, &time_rec);
    if (Status != HSA_STATUS_SUCCESS) {
      DP("Error while getting async copy time\n");
      return;
    }
    ::setOmptTimestamp(time_rec.start * TicksToTime,
                       time_rec.end * TicksToTime);
  }

  /// Getters and setters for stack and heap sizes.
  Error getDeviceStackSize(uint64_t &Value) override {
    Value = 0;
    return Plugin::success();
  }
  Error setDeviceStackSize(uint64_t Value) override {
    return Plugin::success();
  }
  Error getDeviceHeapSize(uint64_t &Value) override {
    Value = 0;
    return Plugin::success();
  }
  Error setDeviceHeapSize(uint64_t Value) override { return Plugin::success(); }

  /// AMDGPU-specific function to get device attributes.
  template <typename Ty> Error getDeviceAttr(uint32_t Kind, Ty &Value) {
    hsa_status_t Status =
        hsa_agent_get_info(Agent, (hsa_agent_info_t)Kind, &Value);
    return Plugin::check(Status, "Error in hsa_agent_get_info: %s");
  }

  template <typename Ty>
  hsa_status_t getDeviceAttrRaw(uint32_t Kind, Ty &Value) {
    return hsa_agent_get_info(Agent, (hsa_agent_info_t)Kind, &Value);
  }

  /// Get the device agent.
  hsa_agent_t getAgent() const override { return Agent; }

  /// Get the signal manager.
  AMDGPUSignalManagerTy &getSignalManager() { return AMDGPUSignalManager; }

  /// Retrieve and construct all memory pools of the device agent.
  Error retrieveAllMemoryPools() override {
    // Iterate through the available pools of the device agent.
    return utils::iterateAgentMemoryPools(
        Agent, [&](hsa_amd_memory_pool_t HSAMemoryPool) {
          AMDGPUMemoryPoolTy *MemoryPool =
              Plugin::get().allocate<AMDGPUMemoryPoolTy>();
          new (MemoryPool) AMDGPUMemoryPoolTy(HSAMemoryPool);
          AllMemoryPools.push_back(MemoryPool);
          return HSA_STATUS_SUCCESS;
        });
  }

  /// Get the next queue depending on its status. Preferably an idle queue is
  /// returned. If no initialized queue is available, but more queues may be
  /// active, the next default-constructed queue is initialized. Otherwise, the
  /// queue is selected in a round-robin fashion.
  AMDGPUQueueTy &getNextQueue(bool shouldTrackBusy = false) {
    static std::atomic<uint32_t> NextQueue(0);
    // For now, simply use a lock.
    // TODO: Improve implementation and get rid of lock if possible
    std::lock_guard<std::mutex> LG(QueuesLock);

    // The size is the maximum number of queues
    int MaxNumQueues = Queues.size();

    // Determine queues that are busy right now
    // If an idle and already initialized queue is encountered, return it
    int NumBusyQueues = 0;
    for (auto &Q : Queues)
      if (Q.isBusy())
        NumBusyQueues++;
      else if (Q.isInitialized()) {
        if (shouldTrackBusy)
          Q.incBusy();
        return Q;
      }

    // For now we always take this code path, as no queue is initialized at the
    // beginning, so we need to execute here at least once.
    int QueueCount = NumInitQueues.load();
    if (QueueCount < MaxNumQueues && QueueCount <= NumBusyQueues) {
      DP("LAZY_QUEUE: Constructing new Queue: %i (Device %i)\n", QueueCount,
         getDeviceId());
      // TODO: Handle errors here: abort? Gracefully? Ignore?
      if (auto Err = Queues[QueueCount].init(getAgent(), OMPX_QueueSize))
        DP("LAZY_QUEUE: Error occurred during AMDGPUQueueTy init\n");
      // Actually an atomic pre-increment
      QueueCount = ++NumInitQueues;
    }
    QueueCount = (QueueCount == 0) ? 1 : QueueCount; // Circumvent divide by 0

    uint32_t Current = NextQueue.fetch_add(1, std::memory_order_relaxed);
    DP("LAZY_QUEUE: Busy: %i Current %i, QueueCount %i\n", NumBusyQueues,
       Current, QueueCount);
    // Now upper limit is number of init-ed queues
    Queues[Current % QueueCount].incBusy();
    return Queues[Current % QueueCount];
  }

  /// Enable/disable profiling of the HSA queues.
  void setOmptQueueProfile(int Enable) {
    for (auto &Q : Queues)
      if (Q.isInitialized())
        hsa_amd_profiling_set_profiler_enabled(Q.getHsaQueue(), Enable);
  }

  /// Get the address of pointer to the preallocated device memory pool.
  void **getPreAllocatedDeviceMemoryPool() {
    return &PreAllocatedDeviceMemoryPool;
  }

  /// Allocate and zero initialize a small memory pool from the coarse grained
  /// device memory of each device.
  Error preAllocateDeviceMemoryPool() {
    Error Err = retrieveAllMemoryPools();
    if (Err)
      return Plugin::error("Unable to retieve all memmory pools");

    void *DevPtr;
    for (AMDGPUMemoryPoolTy *MemoryPool : AllMemoryPools) {
      if (!MemoryPool->isGlobal())
        continue;

      if (MemoryPool->isCoarseGrained()) {
        DevPtr = nullptr;
        size_t PreAllocSize = utils::PER_DEVICE_PREALLOC_SIZE;

        Err = MemoryPool->allocate(PreAllocSize, &DevPtr);
        if (Err)
          return Plugin::error("Device memory pool preallocation failed");

        Err = MemoryPool->enableAccess(DevPtr, PreAllocSize, {getAgent()});
        if (Err)
          return Plugin::error("Preallocated device memory pool inaccessible");

        Err = MemoryPool->zeroInitializeMemory(DevPtr, PreAllocSize);
        if (Err)
          return Plugin::error(
              "Zero initialization of preallocated device memory pool failed");

        PreAllocatedDeviceMemoryPool = DevPtr;
      }
    }
    return Plugin::success();
  }

private:
  using AMDGPUStreamRef = AMDGPUResourceRef<AMDGPUStreamTy>;
  using AMDGPUEventRef = AMDGPUResourceRef<AMDGPUEventTy>;

  using AMDGPUStreamManagerTy = GenericDeviceResourceManagerTy<AMDGPUStreamRef>;
  using AMDGPUEventManagerTy = GenericDeviceResourceManagerTy<AMDGPUEventRef>;

  /// Envar for controlling the number of HSA queues per device. High number of
  /// queues may degrade performance.
  UInt32Envar OMPX_NumQueues;

  /// Envar for controlling the size of each HSA queue. The size is the number
  /// of HSA packets a queue is expected to hold. It is also the number of HSA
  /// packets that can be pushed into each queue without waiting the driver to
  /// process them.
  UInt32Envar OMPX_QueueSize;

  /// Envar for controlling the default number of teams relative to the number
  /// of compute units (CUs) the device has:
  ///   #default_teams = OMPX_DefaultTeamsPerCU * #CUs.
  UInt32Envar OMPX_DefaultTeamsPerCU;

  /// Envar specifying tripcount below which the blocksize should be adjusted.
  UInt32Envar OMPX_LowTripCount;

  /// Envar specifying a value till which the blocksize can be adjusted if the
  /// tripcount is low.
  UInt32Envar OMPX_SmallBlockSize;

  /// Envar specifying the maximum size in bytes where the memory copies are
  /// asynchronous operations. Up to this transfer size, the memory copies are
  /// asychronous operations pushed to the corresponding stream. For larger
  /// transfers, they are synchronous transfers.
  UInt32Envar OMPX_MaxAsyncCopyBytes;

  /// Envar controlling the initial number of HSA signals per device. There is
  /// one manager of signals per device managing several pre-allocated signals.
  /// These signals are mainly used by AMDGPU streams. If needed, more signals
  /// will be created.
  UInt32Envar OMPX_InitialNumSignals;

  /// Envar to force synchronous target regions. The default 0 uses an
  /// asynchronous implementation.
  UInt32Envar OMPX_ForceSyncRegions;
  /// switching to blocked state. The default 2000000 busywaits for 2 seconds
  /// before going into a blocking HSA wait state. The unit for these variables
  /// are microseconds.
  UInt32Envar OMPX_StreamBusyWait;

  /// Stream manager for AMDGPU streams.
  AMDGPUStreamManagerTy AMDGPUStreamManager;

  /// Event manager for AMDGPU events.
  AMDGPUEventManagerTy AMDGPUEventManager;

  /// Signal manager for AMDGPU signals.
  AMDGPUSignalManagerTy AMDGPUSignalManager;

  /// The agent handler corresponding to the device.
  hsa_agent_t Agent;

  /// The GPU architecture.
  std::string ComputeUnitKind;

  /// The number of CUs available in this device
  uint32_t NumComputeUnits;

  /// Reference to the host device.
  AMDHostDeviceTy &HostDevice;

  /// List of device packet queues.
  std::vector<AMDGPUQueueTy> Queues;

  /// Guarding the whole queue initialization
  std::mutex QueuesLock;

  /// Number of initialized (HSA) queues
  std::atomic<int> NumInitQueues{0};

  // Data structure used to keep track of coarse grain memory regions
  AMDGPUMemTypeBitFieldTable *coarse_grain_mem_tab = nullptr;

  /// Pointer to the preallocated device memory pool
  void *PreAllocatedDeviceMemoryPool;
};

Error AMDGPUDeviceImageTy::loadExecutable(const AMDGPUDeviceTy &Device) {
  hsa_status_t Status;
  Status = hsa_code_object_deserialize(getStart(), getSize(), "", &CodeObject);
  if (auto Err =
          Plugin::check(Status, "Error in hsa_code_object_deserialize: %s"))
    return Err;

  Status = hsa_executable_create_alt(
      HSA_PROFILE_FULL, HSA_DEFAULT_FLOAT_ROUNDING_MODE_ZERO, "", &Executable);
  if (auto Err =
          Plugin::check(Status, "Error in hsa_executable_create_alt: %s"))
    return Err;

#if SANITIZER_AMDGPU
  Status = hsa_code_object_reader_create_from_memory(getStart(), getSize(),
                                                     &CodeObjectReader);
  if (auto Err = Plugin::check(
          Status, "Error in hsa_code_object_reader_from_memory: %s"))
    return Err;

  Status = hsa_executable_load_agent_code_object(Executable, Device.getAgent(),
                                                 CodeObjectReader, "", nullptr);
  if (auto Err =
          Plugin::check(Status, "Error in hsa_executable_load_code_object: %s"))
    return Err;
#else
  Status = hsa_executable_load_code_object(Executable, Device.getAgent(),
                                           CodeObject, "");
  if (auto Err =
          Plugin::check(Status, "Error in hsa_executable_load_code_object: %s"))
    return Err;
#endif

  Status = hsa_executable_freeze(Executable, "");
  if (auto Err = Plugin::check(Status, "Error in hsa_executable_freeze: %s"))
    return Err;

  uint32_t Result;
  Status = hsa_executable_validate(Executable, &Result);
  if (auto Err = Plugin::check(Status, "Error in hsa_executable_validate: %s"))
    return Err;

  if (Result)
    return Plugin::error("Loaded HSA executable does not validate");

  if (auto Err = utils::readAMDGPUMetaDataFromImage(
          getMemoryBuffer(), KernelInfoMap, ELFABIVersion))
    return Err;

  return Plugin::success();
}

Expected<hsa_executable_symbol_t>
AMDGPUDeviceImageTy::findDeviceSymbol(GenericDeviceTy &Device,
                                      StringRef SymbolName) const {
  AMDGPUDeviceTy &AMDGPUDevice = static_cast<AMDGPUDeviceTy &>(Device);
  hsa_agent_t Agent = AMDGPUDevice.getAgent();

  hsa_executable_symbol_t Symbol;
  hsa_status_t Status = hsa_executable_get_symbol_by_name(
      Executable, SymbolName.data(), &Agent, &Symbol);
  if (auto Err = Plugin::check(
          Status, "Error in hsa_executable_get_symbol_by_name(%s): %s",
          SymbolName.data()))
    return std::move(Err);

  return Symbol;
}

bool AMDGPUDeviceImageTy::hasDeviceSymbol(GenericDeviceTy &Device,
                                          StringRef SymbolName) const {
  AMDGPUDeviceTy &AMDGPUDevice = static_cast<AMDGPUDeviceTy &>(Device);
  hsa_agent_t Agent = AMDGPUDevice.getAgent();
  hsa_executable_symbol_t Symbol;
  hsa_status_t Status = hsa_executable_get_symbol_by_name(
      Executable, SymbolName.data(), &Agent, &Symbol);
  return (Status == HSA_STATUS_SUCCESS);
}

template <typename ResourceTy>
Error AMDGPUResourceRef<ResourceTy>::create(GenericDeviceTy &Device) {
  if (Resource)
    return Plugin::error("Creating an existing resource");

  AMDGPUDeviceTy &AMDGPUDevice = static_cast<AMDGPUDeviceTy &>(Device);

  Resource = new ResourceTy(AMDGPUDevice);

  return Resource->init();
}

AMDGPUStreamTy::AMDGPUStreamTy(AMDGPUDeviceTy &Device)
    : Agent(Device.getAgent()), SignalManager(Device.getSignalManager()),
      // Initialize the std::deque with some empty positions.
      Slots(32), NextSlot(0), SyncCycle(0),
      StreamBusyWaitMicroseconds(Device.getStreamBusyWaitMicroseconds()),
      Device(Device) {}

/// Class implementing the AMDGPU-specific functionalities of the global
/// handler.
struct AMDGPUGlobalHandlerTy final : public GenericGlobalHandlerTy {
  /// Get the metadata of a global from the device. The name and size of the
  /// global is read from DeviceGlobal and the address of the global is written
  /// to DeviceGlobal.
  Error getGlobalMetadataFromDevice(GenericDeviceTy &Device,
                                    DeviceImageTy &Image,
                                    GlobalTy &DeviceGlobal) override {
    AMDGPUDeviceImageTy &AMDImage = static_cast<AMDGPUDeviceImageTy &>(Image);

    // Find the symbol on the device executable.
    auto SymbolOrErr =
        AMDImage.findDeviceSymbol(Device, DeviceGlobal.getName());
    if (!SymbolOrErr)
      return SymbolOrErr.takeError();

    hsa_executable_symbol_t Symbol = *SymbolOrErr;
    hsa_symbol_kind_t SymbolType;
    hsa_status_t Status;
    uint64_t SymbolAddr;
    uint32_t SymbolSize;

    // Retrieve the type, address and size of the symbol.
    std::pair<hsa_executable_symbol_info_t, void *> RequiredInfos[] = {
        {HSA_EXECUTABLE_SYMBOL_INFO_TYPE, &SymbolType},
        {HSA_EXECUTABLE_SYMBOL_INFO_VARIABLE_ADDRESS, &SymbolAddr},
        {HSA_EXECUTABLE_SYMBOL_INFO_VARIABLE_SIZE, &SymbolSize}};

    for (auto &Info : RequiredInfos) {
      Status = hsa_executable_symbol_get_info(Symbol, Info.first, Info.second);
      if (auto Err = Plugin::check(
              Status, "Error in hsa_executable_symbol_get_info: %s"))
        return Err;
    }

    // Check the size of the symbol.
    if (SymbolSize != DeviceGlobal.getSize())
      return Plugin::error(
          "Failed to load global '%s' due to size mismatch (%zu != %zu)",
          DeviceGlobal.getName().data(), SymbolSize,
          (size_t)DeviceGlobal.getSize());

    // Store the symbol address on the device global metadata.
    DeviceGlobal.setPtr(reinterpret_cast<void *>(SymbolAddr));

    return Plugin::success();
  }

private:
  /// Extract the global's information from the ELF image, section, and symbol.
  Error getGlobalMetadataFromELF(const DeviceImageTy &Image,
                                 const ELF64LE::Sym &Symbol,
                                 const ELF64LE::Shdr &Section,
                                 GlobalTy &ImageGlobal) override {
    // The global's address in AMDGPU is computed as the image begin + the ELF
    // symbol value. Notice we do not add the ELF section offset.
    ImageGlobal.setPtr(advanceVoidPtr(Image.getStart(), Symbol.st_value));

    // Set the global's size.
    ImageGlobal.setSize(Symbol.st_size);

    return Plugin::success();
  }
};

/// Class implementing the AMDGPU-specific functionalities of the plugin.
struct AMDGPUPluginTy final : public GenericPluginTy {
  /// Create an AMDGPU plugin and initialize the AMDGPU driver.
  AMDGPUPluginTy()
      : GenericPluginTy(getTripleArch()), Initialized(false),
        HostDevice(nullptr) {}

  /// This class should not be copied.
  AMDGPUPluginTy(const AMDGPUPluginTy &) = delete;
  AMDGPUPluginTy(AMDGPUPluginTy &&) = delete;

  /// Initialize the plugin and return the number of devices.
  Expected<int32_t> initImpl() override {
    hsa_status_t Status = hsa_init();
    if (Status != HSA_STATUS_SUCCESS) {
      // Cannot call hsa_success_string.
      DP("Failed to initialize AMDGPU's HSA library\n");
      return 0;
    }

    // The initialization of HSA was successful. It should be safe to call
    // HSA functions from now on, e.g., hsa_shut_down.
    Initialized = true;

    // This should probably be ASO-only
    UInt32Envar KernTrace("LIBOMPTARGET_KERNEL_TRACE", 0);
    llvm::omp::target::plugin::PrintKernelTrace = KernTrace.get();

    // Register event handler to detect memory errors on the devices.
    Status = hsa_amd_register_system_event_handler(eventHandler, nullptr);
    if (auto Err = Plugin::check(
            Status, "Error in hsa_amd_register_system_event_handler: %s"))
      return std::move(Err);

    // List of host (CPU) agents.
    llvm::SmallVector<hsa_agent_t> HostAgents;

    // Count the number of available agents.
    auto Err = utils::iterateAgents([&](hsa_agent_t Agent) {
      // Get the device type of the agent.
      hsa_device_type_t DeviceType;
      hsa_status_t Status =
          hsa_agent_get_info(Agent, HSA_AGENT_INFO_DEVICE, &DeviceType);
      if (Status != HSA_STATUS_SUCCESS)
        return Status;

      // Classify the agents into kernel (GPU) and host (CPU) kernels.
      if (DeviceType == HSA_DEVICE_TYPE_GPU) {
        // Ensure that the GPU agent supports kernel dispatch packets.
        hsa_agent_feature_t Features;
        Status = hsa_agent_get_info(Agent, HSA_AGENT_INFO_FEATURE, &Features);
        if (Features & HSA_AGENT_FEATURE_KERNEL_DISPATCH)
          KernelAgents.push_back(Agent);
      } else if (DeviceType == HSA_DEVICE_TYPE_CPU) {
        HostAgents.push_back(Agent);
      }
      return HSA_STATUS_SUCCESS;
    });

    if (Err)
      return std::move(Err);

    int32_t NumDevices = KernelAgents.size();
    if (NumDevices == 0) {
      // Do not initialize if there are no devices.
      DP("There are no devices supporting AMDGPU.\n");
      return 0;
    }

    // There are kernel agents but there is no host agent. That should be
    // treated as an error.
    if (HostAgents.empty())
      return Plugin::error("No AMDGPU host agents");

    // Initialize the host device using host agents.
    HostDevice = allocate<AMDHostDeviceTy>();
    new (HostDevice) AMDHostDeviceTy(HostAgents);

    // Setup the memory pools of available for the host.
    if (auto Err = HostDevice->init())
      return std::move(Err);

#ifdef OMPT_SUPPORT
    ::OmptCallbackInit();
#endif

    return NumDevices;
  }

  /// Deinitialize the plugin.
  Error deinitImpl() override {
    utils::hostrpc_terminate();
    // The HSA runtime was not initialized, so nothing from the plugin was
    // actually initialized.
    if (!Initialized)
      return Plugin::success();

    if (HostDevice)
      if (auto Err = HostDevice->deinit())
        return Err;

    // Finalize the HSA runtime.
    hsa_status_t Status = hsa_shut_down();
    return Plugin::check(Status, "Error in hsa_shut_down: %s");
  }

  Triple::ArchType getTripleArch() const override { return Triple::amdgcn; }

  /// Get the ELF code for recognizing the compatible image binary.
  uint16_t getMagicElfBits() const override { return ELF::EM_AMDGPU; }

  /// Check whether the image is compatible with an AMDGPU device.
  Expected<bool> isImageCompatible(__tgt_image_info *Info) const override {
    for (hsa_agent_t Agent : KernelAgents) {
      std::string Target;
      auto Err = utils::iterateAgentISAs(Agent, [&](hsa_isa_t ISA) {
        uint32_t Length;
        hsa_status_t Status;
        Status = hsa_isa_get_info_alt(ISA, HSA_ISA_INFO_NAME_LENGTH, &Length);
        if (Status != HSA_STATUS_SUCCESS)
          return Status;

        // TODO: This is not allowed by the standard.
        char ISAName[Length];
        Status = hsa_isa_get_info_alt(ISA, HSA_ISA_INFO_NAME, ISAName);
        if (Status != HSA_STATUS_SUCCESS)
          return Status;

        llvm::StringRef TripleTarget(ISAName);
        if (TripleTarget.consume_front("amdgcn-amd-amdhsa"))
          Target = TripleTarget.ltrim('-').str();
        return HSA_STATUS_SUCCESS;
      });
      if (Err)
        return std::move(Err);

      if (utils::isImageCompatibleWithEnv(Info, Target))
        return true;
    }
    return false;
  }

  /// This plugin does not support exchanging data between two devices.
  bool isDataExchangable(int32_t SrcDeviceId, int32_t DstDeviceId) override {
    return false;
  }

  /// Get the host device instance.
  AMDHostDeviceTy &getHostDevice() {
    assert(HostDevice && "Host device not initialized");
    return *HostDevice;
  }

  /// Get the kernel agent with the corresponding agent id.
  hsa_agent_t getKernelAgent(int32_t AgentId) const {
    assert((uint32_t)AgentId < KernelAgents.size() && "Invalid agent id");
    return KernelAgents[AgentId];
  }

  /// Get the list of the available kernel agents.
  const llvm::SmallVector<hsa_agent_t> &getKernelAgents() const {
    return KernelAgents;
  }

private:
  /// Event handler that will be called by ROCr if an event is detected.
  static hsa_status_t eventHandler(const hsa_amd_event_t *Event, void *) {
    if (Event->event_type != HSA_AMD_GPU_MEMORY_FAULT_EVENT)
      return HSA_STATUS_SUCCESS;

    SmallVector<std::string> Reasons;
    uint32_t ReasonsMask = Event->memory_fault.fault_reason_mask;
    if (ReasonsMask & HSA_AMD_MEMORY_FAULT_PAGE_NOT_PRESENT)
      Reasons.emplace_back("Page not present or supervisor privilege");
    if (ReasonsMask & HSA_AMD_MEMORY_FAULT_READ_ONLY)
      Reasons.emplace_back("Write access to a read-only page");
    if (ReasonsMask & HSA_AMD_MEMORY_FAULT_NX)
      Reasons.emplace_back("Execute access to a page marked NX");
    if (ReasonsMask & HSA_AMD_MEMORY_FAULT_HOST_ONLY)
      Reasons.emplace_back("GPU attempted access to a host only page");
    if (ReasonsMask & HSA_AMD_MEMORY_FAULT_DRAMECC)
      Reasons.emplace_back("DRAM ECC failure");
    if (ReasonsMask & HSA_AMD_MEMORY_FAULT_IMPRECISE)
      Reasons.emplace_back("Can't determine the exact fault address");
    if (ReasonsMask & HSA_AMD_MEMORY_FAULT_SRAMECC)
      Reasons.emplace_back("SRAM ECC failure (ie registers, no fault address)");
    if (ReasonsMask & HSA_AMD_MEMORY_FAULT_HANG)
      Reasons.emplace_back("GPU reset following unspecified hang");

    // If we do not know the reason, say so, otherwise remove the trailing comma
    // and space.
    if (Reasons.empty())
      Reasons.emplace_back("Unknown (" + std::to_string(ReasonsMask) + ")");

    uint32_t Node = -1;
    hsa_agent_get_info(Event->memory_fault.agent, HSA_AGENT_INFO_NODE, &Node);

    // Abort the execution since we do not recover from this error.
    FATAL_MESSAGE(1,
                  "Memory access fault by GPU %" PRIu32 " (agent 0x%" PRIx64
                  ") at virtual address %p. Reasons: %s",
                  Node, Event->memory_fault.agent.handle,
                  (void *)Event->memory_fault.virtual_address,
                  llvm::join(Reasons, ", ").c_str());

    return HSA_STATUS_ERROR;
  }

  /// Indicate whether the HSA runtime was correctly initialized. Even if there
  /// is no available devices this boolean will be true. It indicates whether
  /// we can safely call HSA functions (e.g., hsa_shut_down).
  bool Initialized;

  /// Arrays of the available GPU and CPU agents. These arrays of handles should
  /// not be here but in the AMDGPUDeviceTy structures directly. However, the
  /// HSA standard does not provide API functions to retirve agents directly,
  /// only iterating functions. We cache the agents here for convenience.
  llvm::SmallVector<hsa_agent_t> KernelAgents;

  /// The device representing all HSA host agents.
  AMDHostDeviceTy *HostDevice;
};

Error AMDGPUKernelTy::launchImpl(GenericDeviceTy &GenericDevice,
                                 uint32_t NumThreads, uint64_t NumBlocks,
                                 KernelArgsTy &KernelArgs, void *Args,
                                 AsyncInfoWrapperTy &AsyncInfoWrapper) const {
  const uint32_t KernelArgsSize = KernelArgs.NumArgs * sizeof(void *);

  if (ArgsSize < KernelArgsSize)
    return Plugin::error("Mismatch of kernel arguments size");

  // The args size reported by HSA may or may not contain the implicit args.
  // For now, assume that HSA does not consider the implicit arguments when
  // reporting the arguments of a kernel. In the worst case, we can waste
  // 56 bytes per allocation.
  uint32_t AllArgsSize = KernelArgsSize + ImplicitArgsSize;

  AMDHostDeviceTy &HostDevice = Plugin::get<AMDGPUPluginTy>().getHostDevice();
  AMDGPUMemoryManagerTy &ArgsMemoryManager = HostDevice.getArgsMemoryManager();

  void *AllArgs = nullptr;
  if (auto Err = ArgsMemoryManager.allocate(AllArgsSize, &AllArgs))
    return Err;

  // Account for user requested dynamic shared memory.
  uint32_t GroupSize = getGroupSize();
  if (uint32_t MaxDynCGroupMem = std::max(
          KernelArgs.DynCGroupMem, GenericDevice.getDynamicMemorySize())) {
    GroupSize += MaxDynCGroupMem;
  }

  // Initialize implicit arguments.
  uint8_t *ImplArgs =
      static_cast<uint8_t *>(advanceVoidPtr(AllArgs, KernelArgsSize));

  // Initialize the implicit arguments to zero.
  std::memset(ImplArgs, 0, ImplicitArgsSize);

  // Copy the explicit arguments.
  // TODO: We should expose the args memory manager alloc to the common part as
  // 	   alternative to copying them twice.
  if (KernelArgs.NumArgs)
    std::memcpy(AllArgs, *static_cast<void **>(Args),
                sizeof(void *) * KernelArgs.NumArgs);

  uint64_t Buffer = 0;
  AMDGPUDeviceTy &AMDGPUDevice = static_cast<AMDGPUDeviceTy &>(GenericDevice);
  AMDGPUStreamTy &Stream = AMDGPUDevice.getStream(AsyncInfoWrapper);
  if (NeedsHostServices) {
    int32_t DevID = AMDGPUDevice.getDeviceId();
    hsa_amd_memory_pool_t HostMemPool =
        HostDevice.getFineGrainedMemoryPool().get();
    hsa_amd_memory_pool_t DeviceMemPool =
        AMDGPUDevice.getCoarseGrainedMemoryPool()->get();
    hsa_queue_t *HsaQueue = Stream.getHsaQueue();
    Buffer = utils::hostrpc_assign_buffer(AMDGPUDevice.getAgent(), HsaQueue,
                                          DevID, HostMemPool, DeviceMemPool);
    GlobalTy ServiceThreadHostBufferGlobal("service_thread_buf",
                                           sizeof(uint64_t), &Buffer);
    if (auto Err = HostServiceBufferHandler->writeGlobalToDevice(
            AMDGPUDevice, ServiceThreadHostBufferGlobal,
            ServiceThreadDeviceBufferGlobal)) {
      DP("Missing symbol %s, continue execution anyway.\n",
         ServiceThreadHostBufferGlobal.getName().data());
      consumeError(std::move(Err));
    }
    DP("Hostrpc buffer allocated at %p and service thread started\n",
       (void *)Buffer);
  } else {
    DP("No hostrpc buffer or service thread required\n");
  }

  if (getImplicitArgsSize() < utils::COV5_SIZE) {
    DP("Setting fields of ImplicitArgs for COV4\n");
    memcpy(&ImplArgs[utils::COV4_HOSTCALL_PTR_OFFSET], &Buffer,
           utils::HOSTCALL_PTR_SIZE);
  } else {
    DP("Setting fields of ImplicitArgs for COV5\n");
    uint16_t Remainder = 0;
    uint16_t GridDims = 1;
    uint32_t NumThreadsYZ = 1;
    uint16_t NumBlocksYZ = 0;
    memcpy(&ImplArgs[utils::COV5_BLOCK_COUNT_X_OFFSET], &NumBlocks,
           utils::COV5_BLOCK_COUNT_X_SIZE);
    memcpy(&ImplArgs[utils::COV5_BLOCK_COUNT_Y_OFFSET], &NumBlocksYZ,
           utils::COV5_BLOCK_COUNT_Y_SIZE);
    memcpy(&ImplArgs[utils::COV5_BLOCK_COUNT_Z_OFFSET], &NumBlocksYZ,
           utils::COV5_BLOCK_COUNT_Z_SIZE);

    memcpy(&ImplArgs[utils::COV5_GROUP_SIZE_X_OFFSET], &NumThreads,
           utils::COV5_GROUP_SIZE_X_SIZE);
    memcpy(&ImplArgs[utils::COV5_GROUP_SIZE_Y_OFFSET], &NumThreadsYZ,
           utils::COV5_GROUP_SIZE_Y_SIZE);
    memcpy(&ImplArgs[utils::COV5_GROUP_SIZE_Z_OFFSET], &NumThreadsYZ,
           utils::COV5_GROUP_SIZE_Z_SIZE);

    memcpy(&ImplArgs[utils::COV5_REMAINDER_X_OFFSET], &Remainder,
           utils::COV5_REMAINDER_X_SIZE);
    memcpy(&ImplArgs[utils::COV5_REMAINDER_Y_OFFSET], &Remainder,
           utils::COV5_REMAINDER_Y_SIZE);
    memcpy(&ImplArgs[utils::COV5_REMAINDER_Z_OFFSET], &Remainder,
           utils::COV5_REMAINDER_Z_SIZE);

    memcpy(&ImplArgs[utils::COV5_GRID_DIMS_OFFSET], &GridDims,
           utils::COV5_GRID_DIMS_SIZE);

    memcpy(&ImplArgs[utils::COV5_HOSTCALL_PTR_OFFSET], &Buffer,
           utils::HOSTCALL_PTR_SIZE);

    memcpy(&ImplArgs[utils::COV5_HEAPV1_PTR_OFFSET],
           AMDGPUDevice.getPreAllocatedDeviceMemoryPool(),
           utils::COV5_HEAPV1_PTR_SIZE);
  }

  // Push the kernel launch into the stream.
  return Stream.pushKernelLaunch(*this, AllArgs, NumThreads, NumBlocks,
                                 GroupSize, ArgsMemoryManager);
}

void AMDGPUKernelTy::printAMDOneLineKernelTrace(GenericDeviceTy &GenericDevice,
                                                KernelArgsTy &KernelArgs,
                                                uint32_t NumThreads,
                                                uint64_t NumBlocks) const {
  auto GroupSegmentSize = (*KernelInfo).GroupSegmentList;
  auto SGPRCount = (*KernelInfo).SGPRCount;
  auto VGPRCount = (*KernelInfo).VGPRCount;
  auto SGPRSpillCount = (*KernelInfo).SGPRSpillCount;
  auto VGPRSpillCount = (*KernelInfo).VGPRSpillCount;
  // auto MaxFlatWorkgroupSize = (*KernelInfo).MaxFlatWorkgroupSize;

  // This line should print exactly as the one in the old plugin.
  fprintf(stderr,
          "DEVID: %2d SGN:%d ConstWGSize:%-4d args:%2d teamsXthrds:(%4luX%4d) "
          "reqd:(%4dX%4d) lds_usage:%uB sgpr_count:%u vgpr_count:%u "
          "sgpr_spill_count:%u vgpr_spill_count:%u tripcount:%lu rpc:%d n:%s\n",
          GenericDevice.getDeviceId(), getExecutionModeFlags(), ConstWGSize,
          KernelArgs.NumArgs, NumBlocks, NumThreads, 0, 0, GroupSegmentSize,
          SGPRCount, VGPRCount, SGPRSpillCount, VGPRSpillCount,
          KernelArgs.Tripcount, NeedsHostServices, getName());
}

Error AMDGPUKernelTy::printLaunchInfoDetails(GenericDeviceTy &GenericDevice,
                                             KernelArgsTy &KernelArgs,
                                             uint32_t NumThreads,
                                             uint64_t NumBlocks) const {
  // When LIBOMPTARGET_KERNEL_TRACE is set, print the single-line kernel trace
  // info present in the old ASO plugin, and continue with the upstream 2-line
  // info, should LIBOMPTARGET_INFO be a meaningful value, otherwise return.
  if (getInfoLevel() & OMP_INFOTYPE_AMD_KERNEL_TRACE)
    printAMDOneLineKernelTrace(GenericDevice, KernelArgs, NumThreads,
                               NumBlocks);

  // Only do all this when the output is requested
  if (!(getInfoLevel() & OMP_INFOTYPE_PLUGIN_KERNEL))
    return Plugin::success();

  // We don't have data to print additional info, but no hard error
  if (!KernelInfo.has_value())
    return Plugin::success();

  // General Info
  auto NumGroups = NumBlocks;
  auto ThreadsPerGroup = NumThreads;

  // Kernel Arguments Info
  auto ArgNum = KernelArgs.NumArgs;
  auto LoopTripCount = KernelArgs.Tripcount;

  // Details for AMDGPU kernels (read from image)
  // https://www.llvm.org/docs/AMDGPUUsage.html#code-object-v4-metadata
  auto GroupSegmentSize = (*KernelInfo).GroupSegmentList;
  auto SGPRCount = (*KernelInfo).SGPRCount;
  auto VGPRCount = (*KernelInfo).VGPRCount;
  auto SGPRSpillCount = (*KernelInfo).SGPRSpillCount;
  auto VGPRSpillCount = (*KernelInfo).VGPRSpillCount;
  auto MaxFlatWorkgroupSize = (*KernelInfo).MaxFlatWorkgroupSize;

  // Prints additional launch info that contains the following.
  // Num Args: The number of kernel arguments
  // Teams x Thrds: The number of teams and the number of threads actually
  // running.
  // MaxFlatWorkgroupSize: Maximum flat work-group size supported by the
  // kernel in work-items
  // LDS Usage: Amount of bytes used in LDS storage
  // S/VGPR Count: the number of S/V GPRs occupied by the kernel
  // S/VGPR Spill Count: how many S/VGPRs are spilled by the kernel
  // Tripcount: loop tripcount for the kernel
  INFO(OMP_INFOTYPE_PLUGIN_KERNEL, GenericDevice.getDeviceId(),
       "#Args: %d Teams x Thrds: %4lux%4u (MaxFlatWorkGroupSize: %u) LDS "
       "Usage: %uB #SGPRs/VGPRs: %u/%u #SGPR/VGPR Spills: %u/%u Tripcount: "
       "%lu\n",
       ArgNum, NumGroups, ThreadsPerGroup, MaxFlatWorkgroupSize,
       GroupSegmentSize, SGPRCount, VGPRCount, SGPRSpillCount, VGPRSpillCount,
       LoopTripCount);

  return Plugin::success();
}

GenericPluginTy *Plugin::createPlugin() { return new AMDGPUPluginTy(); }

GenericDeviceTy *Plugin::createDevice(int32_t DeviceId, int32_t NumDevices) {
  AMDGPUPluginTy &Plugin = get<AMDGPUPluginTy &>();
  return new AMDGPUDeviceTy(DeviceId, NumDevices, Plugin.getHostDevice(),
                            Plugin.getKernelAgent(DeviceId));
}

GenericGlobalHandlerTy *Plugin::createGlobalHandler() {
  return new AMDGPUGlobalHandlerTy();
}

template <typename... ArgsTy>
Error Plugin::check(int32_t Code, const char *ErrFmt, ArgsTy... Args) {
  hsa_status_t ResultCode = static_cast<hsa_status_t>(Code);
  if (ResultCode == HSA_STATUS_SUCCESS || ResultCode == HSA_STATUS_INFO_BREAK)
    return Error::success();

  const char *Desc = "Unknown error";
  hsa_status_t Ret = hsa_status_string(ResultCode, &Desc);
  if (Ret != HSA_STATUS_SUCCESS)
    REPORT("Unrecognized " GETNAME(TARGET_NAME) " error code %d\n", Code);

  return createStringError<ArgsTy..., const char *>(inconvertibleErrorCode(),
                                                    ErrFmt, Args..., Desc);
}

void *AMDGPUMemoryManagerTy::allocate(size_t Size, void *HstPtr,
                                      TargetAllocTy Kind) {
  // Allocate memory from the pool.
  void *Ptr = nullptr;
  if (auto Err = MemoryPool->allocate(Size, &Ptr)) {
    consumeError(std::move(Err));
    return nullptr;
  }
  assert(Ptr && "Invalid pointer");

  auto &KernelAgents = Plugin::get<AMDGPUPluginTy>().getKernelAgents();

  // Allow all kernel agents to access the allocation.
  if (auto Err = MemoryPool->enableAccess(Ptr, Size, KernelAgents)) {
    REPORT("%s\n", toString(std::move(Err)).data());
    return nullptr;
  }
  return Ptr;
}

void *AMDGPUDeviceTy::allocate(size_t Size, void *, TargetAllocTy Kind) {
  if (Size == 0)
    return nullptr;

  // Find the correct memory pool.
  AMDGPUMemoryPoolTy *MemoryPool = nullptr;
  switch (Kind) {
  case TARGET_ALLOC_DEFAULT:
  case TARGET_ALLOC_DEVICE:
    MemoryPool = CoarseGrainedMemoryPools[0];
    break;
  case TARGET_ALLOC_HOST:
    MemoryPool = &HostDevice.getFineGrainedMemoryPool();
    break;
  case TARGET_ALLOC_SHARED:
    MemoryPool = &HostDevice.getFineGrainedMemoryPool();
    break;
  }

  if (!MemoryPool) {
    REPORT("No memory pool for the specified allocation kind\n");
    return nullptr;
  }

  // Allocate from the corresponding memory pool.
  void *Alloc = nullptr;
  if (Error Err = MemoryPool->allocate(Size, &Alloc)) {
    REPORT("%s\n", toString(std::move(Err)).data());
    return nullptr;
  }

  if (Alloc && (Kind == TARGET_ALLOC_HOST || Kind == TARGET_ALLOC_SHARED)) {
    auto &KernelAgents = Plugin::get<AMDGPUPluginTy>().getKernelAgents();

    // Enable all kernel agents to access the buffer.
    if (auto Err = MemoryPool->enableAccess(Alloc, Size, KernelAgents)) {
      REPORT("%s\n", toString(std::move(Err)).data());
      return nullptr;
    }
  }

  return Alloc;
}

Error AMDGPUStreamTy::pushKernelLaunch(const AMDGPUKernelTy &Kernel,
                                       void *KernelArgs, uint32_t NumThreads,
                                       uint64_t NumBlocks, uint32_t GroupSize,
                                       AMDGPUMemoryManagerTy &MemoryManager) {
  // Retrieve an available signal for the operation's output.
  AMDGPUSignalTy *OutputSignal = SignalManager.getResource();
  OutputSignal->reset();
  OutputSignal->increaseUseCount();

  std::lock_guard<std::mutex> StreamLock(Mutex);

  // Consume stream slot and compute dependencies.
  auto [Curr, InputSignal] = consume(OutputSignal);

  // Setup the post action to release the kernel args buffer.
  if (auto Err = Slots[Curr].schedReleaseBuffer(KernelArgs, MemoryManager))
    return Err;

  // Setup the post action to collect kernel execution timing.
  OMPT_IF_TRACING_ENABLED(
      if (auto Err = Slots[Curr].schedOmptKernelTiming(
              Agent, OutputSignal, TicksToTime)) return Err;);

  // Push the kernel with the output signal and an input signal (optional)
  auto &Queue = Device.getNextQueue(true);
  if (auto Err = Slots[Curr].schedDecrementQueueBusyCount(&Queue))
    return Err;

  DP("Using Queue: %p with HSA Queue: %p\n", &Queue, Queue.getHsaQueue());
  return Queue.pushKernelLaunch(Kernel, KernelArgs, NumThreads, NumBlocks,
                                GroupSize, OutputSignal, InputSignal);
}

Error AMDGPUStreamTy::waitOnStreamOperation(AMDGPUStreamTy &OtherStream,
                                            uint32_t Slot) {
  /// The signal that we must wait from the other stream.
  AMDGPUSignalTy *OtherSignal = OtherStream.Slots[Slot].Signal;

  // Prevent the release of the other stream's signal.
  OtherSignal->increaseUseCount();

  // Retrieve an available signal for the operation's output.
  AMDGPUSignalTy *OutputSignal = SignalManager.getResource();
  OutputSignal->reset();
  OutputSignal->increaseUseCount();

  // Consume stream slot and compute dependencies.
  auto [Curr, InputSignal] = consume(OutputSignal);

  // Setup the post action to release the signal.
  if (auto Err = Slots[Curr].schedReleaseSignal(OtherSignal, &SignalManager))
    return Err;

  // Push a barrier into the queue with both input signals.
  auto &Queue = Device.getNextQueue(true);
  if (auto Err = Slots[Curr].schedDecrementQueueBusyCount(&Queue))
    return Err;
  DP("Using Queue: %p with HSA Queue: %p\n", &Queue, Queue.getHsaQueue());
  return Queue.pushBarrier(OutputSignal, InputSignal, OtherSignal);
}

hsa_queue_t *AMDGPUStreamTy::getHsaQueue() {
  auto &Queue = Device.getNextQueue();
  return Queue.getHsaQueue();
};

} // namespace plugin
} // namespace target
} // namespace omp
} // namespace llvm

#ifdef OMPT_SUPPORT
namespace llvm::omp::target::plugin {

/// Enable/disable kernel profiling for the given device.
void setOmptQueueProfile(int DeviceId, int Enable) {
  AMDGPUPluginTy &Plugin = Plugin::get<AMDGPUPluginTy>();
  static_cast<AMDGPUDeviceTy &>(Plugin.getDevice(DeviceId))
      .setOmptQueueProfile(Enable);
}

} // namespace llvm::omp::target::plugin

/// Enable/disable kernel profiling for the given device.
void setGlobalOmptKernelProfile(int DeviceId, int Enable) {
  llvm::omp::target::plugin::setOmptQueueProfile(DeviceId, Enable);
}

#endif
