//===--- amdgpu/impl/get_elf_mach_gfx_name.cpp -------------------- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
#include "get_elf_mach_gfx_name.h"

// This header conflicts with the system elf.h (macros vs enums of the same
// identifier) and contains more up to date values for the enum checked here.
// rtl.cpp uses the system elf.h.
#include "llvm/BinaryFormat/ELF.h"
using namespace llvm::ELF;

const char *get_elf_mach_gfx_name(uint32_t EFlags) {
  using namespace llvm::ELF;
  uint32_t Gfx = (EFlags & EF_AMDGPU_MACH);
  switch (Gfx) {
  case EF_AMDGPU_MACH_AMDGCN_GFX801:
    return "gfx801";
  case EF_AMDGPU_MACH_AMDGCN_GFX802:
    return "gfx802";
  case EF_AMDGPU_MACH_AMDGCN_GFX803:
    return "gfx803";
  case EF_AMDGPU_MACH_AMDGCN_GFX805:
    return "gfx805";
  case EF_AMDGPU_MACH_AMDGCN_GFX810:
    return "gfx810";
  case EF_AMDGPU_MACH_AMDGCN_GFX900:
    return "gfx900";
  case EF_AMDGPU_MACH_AMDGCN_GFX902:
    return "gfx902";
  case EF_AMDGPU_MACH_AMDGCN_GFX904:
    return "gfx904";
  case EF_AMDGPU_MACH_AMDGCN_GFX906:
    return "gfx906";
  case EF_AMDGPU_MACH_AMDGCN_GFX908:
    return "gfx908";
  case EF_AMDGPU_MACH_AMDGCN_GFX909:
    return "gfx909";
  case EF_AMDGPU_MACH_AMDGCN_GFX90A:
    return "gfx90a";
  case EF_AMDGPU_MACH_AMDGCN_GFX90C:
    return "gfx90c";
  case EF_AMDGPU_MACH_AMDGCN_GFX940:
    return "gfx940";
  case EF_AMDGPU_MACH_AMDGCN_GFX941:
    return "gfx941";
  case EF_AMDGPU_MACH_AMDGCN_GFX942:
    return "gfx942";
  case EF_AMDGPU_MACH_AMDGCN_GFX1010:
    return "gfx1010";
  case EF_AMDGPU_MACH_AMDGCN_GFX1011:
    return "gfx1011";
  case EF_AMDGPU_MACH_AMDGCN_GFX1012:
    return "gfx1012";
  case EF_AMDGPU_MACH_AMDGCN_GFX1013:
    return "gfx1013";
  case EF_AMDGPU_MACH_AMDGCN_GFX1030:
    return "gfx1030";
  case EF_AMDGPU_MACH_AMDGCN_GFX1031:
    return "gfx1031";
  case EF_AMDGPU_MACH_AMDGCN_GFX1032:
    return "gfx1032";
  case EF_AMDGPU_MACH_AMDGCN_GFX1033:
    return "gfx1033";
  case EF_AMDGPU_MACH_AMDGCN_GFX1034:
    return "gfx1034";
  case EF_AMDGPU_MACH_AMDGCN_GFX1035:
    return "gfx1035";
  case EF_AMDGPU_MACH_AMDGCN_GFX1036:
    return "gfx1036";
  case EF_AMDGPU_MACH_AMDGCN_GFX1100:
    return "gfx1100";
  case EF_AMDGPU_MACH_AMDGCN_GFX1101:
    return "gfx1101";
  case EF_AMDGPU_MACH_AMDGCN_GFX1102:
    return "gfx1102";
  case EF_AMDGPU_MACH_AMDGCN_GFX1103:
    return "gfx1103";
  default:
    return "--unknown gfx";
  }
}

const uint16_t implicitArgsSize(uint16_t Version) {
  return Version < ELFABIVERSION_AMDGPU_HSA_V5 ? IMPLICITARGS::COV4_SIZE
                                               : IMPLICITARGS::COV5_SIZE;
}
