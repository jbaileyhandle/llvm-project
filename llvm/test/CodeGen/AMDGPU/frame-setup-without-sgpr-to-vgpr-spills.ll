; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc -march=amdgcn -mcpu=gfx900 -verify-machineinstrs -amdgpu-spill-sgpr-to-vgpr=true -amdgpu-spill-cfi-saved-regs=false < %s | FileCheck -check-prefixes=NO-CFI-SAVES-SPILL-TO-VGPR %s
; RUN: llc -march=amdgcn -mcpu=gfx900 -verify-machineinstrs -amdgpu-spill-sgpr-to-vgpr=true -amdgpu-spill-cfi-saved-regs=true < %s | FileCheck -check-prefixes=CFI-SAVES-SPILL-TO-VGPR %s
; RUN: llc -march=amdgcn -mcpu=gfx900 -verify-machineinstrs -amdgpu-spill-sgpr-to-vgpr=false -amdgpu-spill-cfi-saved-regs=false < %s | FileCheck -check-prefixes=NO-CFI-SAVES-NO-SPILL-TO-VGPR %s
; RUN: llc -march=amdgcn -mcpu=gfx900 -verify-machineinstrs -amdgpu-spill-sgpr-to-vgpr=false -amdgpu-spill-cfi-saved-regs=true < %s | FileCheck -check-prefixes=CFI-SAVES-NO-SPILL-TO-VGPR %s

; Check frame setup where SGPR spills to VGPRs are disabled or enabled.

declare hidden void @external_void_func_void() #0

define void @callee_with_stack_and_call() #0 {
; NO-CFI-SAVES-SPILL-TO-VGPR-LABEL: callee_with_stack_and_call:
; NO-CFI-SAVES-SPILL-TO-VGPR:       ; %bb.0:
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0) expcnt(0) lgkmcnt(0)
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b32 s4, s33
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s32
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_or_saveexec_b64 s[8:9], -1
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    buffer_store_dword v40, off, s[0:3], s33 offset:4 ; 4-byte Folded Spill
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[8:9]
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, s4, 2
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, s30, 0
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0x400
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, s31, 1
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, 0
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_getpc_b64 s[4:5]
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_add_u32 s4, s4, external_void_func_void@rel32@lo+4
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_addc_u32 s5, s5, external_void_func_void@rel32@hi+12
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_swappc_b64 s[30:31], s[4:5]
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s30, v40, 0
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s31, v40, 1
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s4, v40, 2
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_or_saveexec_b64 s[6:7], -1
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    buffer_load_dword v40, off, s[0:3], s33 offset:4 ; 4-byte Folded Reload
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[6:7]
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0xfc00
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s4
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_setpc_b64 s[30:31]
;
; CFI-SAVES-SPILL-TO-VGPR-LABEL: callee_with_stack_and_call:
; CFI-SAVES-SPILL-TO-VGPR:       ; %bb.0:
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0) expcnt(0) lgkmcnt(0)
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b32 s4, s33
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s32
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_or_saveexec_b64 s[8:9], -1
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    buffer_store_dword v40, off, s[0:3], s33 offset:4 ; 4-byte Folded Spill
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[8:9]
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, exec_lo, 2
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, exec_hi, 3
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, s4, 4
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, s30, 0
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0x400
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, s31, 1
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, 0
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_getpc_b64 s[4:5]
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_add_u32 s4, s4, external_void_func_void@rel32@lo+4
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_addc_u32 s5, s5, external_void_func_void@rel32@hi+12
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_swappc_b64 s[30:31], s[4:5]
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s30, v40, 0
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s31, v40, 1
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s4, v40, 4
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_or_saveexec_b64 s[6:7], -1
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    buffer_load_dword v40, off, s[0:3], s33 offset:4 ; 4-byte Folded Reload
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[6:7]
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0xfc00
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s4
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; CFI-SAVES-SPILL-TO-VGPR-NEXT:    s_setpc_b64 s[30:31]
;
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-LABEL: callee_with_stack_and_call:
; NO-CFI-SAVES-NO-SPILL-TO-VGPR:       ; %bb.0:
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0) expcnt(0) lgkmcnt(0)
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b32 s4, s33
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s32
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, s4
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:12 ; 4-byte Folded Spill
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0x800
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 s[4:5], exec
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, 3
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:16
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v0, s30, 0
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v0, s31, 1
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:4 ; 4-byte Folded Spill
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:16
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[4:5]
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, 0
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_getpc_b64 s[4:5]
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_add_u32 s4, s4, external_void_func_void@rel32@lo+4
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_addc_u32 s5, s5, external_void_func_void@rel32@hi+12
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_swappc_b64 s[30:31], s[4:5]
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 s[4:5], exec
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, 3
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:16
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:4 ; 4-byte Folded Reload
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s30, v0, 0
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s31, v0, 1
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:16
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[4:5]
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:12 ; 4-byte Folded Reload
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0xf800
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_readfirstlane_b32 s4, v0
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s4
; NO-CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_setpc_b64 s[30:31]
;
; CFI-SAVES-NO-SPILL-TO-VGPR-LABEL: callee_with_stack_and_call:
; CFI-SAVES-NO-SPILL-TO-VGPR:       ; %bb.0:
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0) expcnt(0) lgkmcnt(0)
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b32 s4, s33
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s32
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, exec_lo
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:12 ; 4-byte Folded Spill
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, exec_hi
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:16 ; 4-byte Folded Spill
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, s4
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:20 ; 4-byte Folded Spill
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0x800
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 s[4:5], exec
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, 3
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:24
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v0, s30, 0
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v0, s31, 1
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:4 ; 4-byte Folded Spill
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:24
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[4:5]
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, 0
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_getpc_b64 s[4:5]
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_add_u32 s4, s4, external_void_func_void@rel32@lo+4
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_addc_u32 s5, s5, external_void_func_void@rel32@hi+12
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_swappc_b64 s[30:31], s[4:5]
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 s[4:5], exec
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, 3
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:24
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:4 ; 4-byte Folded Reload
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s30, v0, 0
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s31, v0, 1
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:24
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[4:5]
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:20 ; 4-byte Folded Reload
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0xf800
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    v_readfirstlane_b32 s4, v0
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s4
; CFI-SAVES-NO-SPILL-TO-VGPR-NEXT:    s_setpc_b64 s[30:31]
; SPILL-TO-VGPR-LABEL: callee_with_stack_and_call:
; SPILL-TO-VGPR:       ; %bb.0:
; SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0) expcnt(0) lgkmcnt(0)
; SPILL-TO-VGPR-NEXT:    s_mov_b32 s4, s33
; SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s32
; SPILL-TO-VGPR-NEXT:    s_or_saveexec_b64 s[8:9], -1
; SPILL-TO-VGPR-NEXT:    buffer_store_dword v40, off, s[0:3], s33 offset:4 ; 4-byte Folded Spill
; SPILL-TO-VGPR-NEXT:    buffer_store_dword v41, off, s[0:3], s33 offset:8 ; 4-byte Folded Spill
; SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[8:9]
; SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0x400
; SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, s30, 0
; SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, 0
; SPILL-TO-VGPR-NEXT:    v_writelane_b32 v41, s4, 0
; SPILL-TO-VGPR-NEXT:    v_writelane_b32 v40, s31, 1
; SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33
; SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; SPILL-TO-VGPR-NEXT:    s_getpc_b64 s[4:5]
; SPILL-TO-VGPR-NEXT:    s_add_u32 s4, s4, external_void_func_void@rel32@lo+4
; SPILL-TO-VGPR-NEXT:    s_addc_u32 s5, s5, external_void_func_void@rel32@hi+12
; SPILL-TO-VGPR-NEXT:    s_swappc_b64 s[30:31], s[4:5]
; SPILL-TO-VGPR-NEXT:    v_readlane_b32 s31, v40, 1
; SPILL-TO-VGPR-NEXT:    v_readlane_b32 s30, v40, 0
; SPILL-TO-VGPR-NEXT:    v_readlane_b32 s4, v41, 0
; SPILL-TO-VGPR-NEXT:    s_or_saveexec_b64 s[6:7], -1
; SPILL-TO-VGPR-NEXT:    buffer_load_dword v40, off, s[0:3], s33 offset:4 ; 4-byte Folded Reload
; SPILL-TO-VGPR-NEXT:    buffer_load_dword v41, off, s[0:3], s33 offset:8 ; 4-byte Folded Reload
; SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[6:7]
; SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0xfc00
; SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s4
; SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; SPILL-TO-VGPR-NEXT:    s_setpc_b64 s[30:31]
; NO-SPILL-TO-VGPR-LABEL: callee_with_stack_and_call:
; NO-SPILL-TO-VGPR:       ; %bb.0:
; NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0) expcnt(0) lgkmcnt(0)
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b32 s4, s33
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s32
; NO-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, s4
; NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:12 ; 4-byte Folded Spill
; NO-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0x800
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 s[4:5], exec
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, 1
; NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:16
; NO-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v0, s30, 0
; NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:4 ; 4-byte Folded Spill
; NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:16
; NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[4:5]
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 s[4:5], exec
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, 1
; NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:16
; NO-SPILL-TO-VGPR-NEXT:    v_writelane_b32 v0, s31, 0
; NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:8 ; 4-byte Folded Spill
; NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:16
; NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[4:5]
; NO-SPILL-TO-VGPR-NEXT:    v_mov_b32_e32 v0, 0
; NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33
; NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-SPILL-TO-VGPR-NEXT:    s_getpc_b64 s[4:5]
; NO-SPILL-TO-VGPR-NEXT:    s_add_u32 s4, s4, external_void_func_void@rel32@lo+4
; NO-SPILL-TO-VGPR-NEXT:    s_addc_u32 s5, s5, external_void_func_void@rel32@hi+12
; NO-SPILL-TO-VGPR-NEXT:    s_swappc_b64 s[30:31], s[4:5]
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 s[4:5], exec
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, 1
; NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:16
; NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:8 ; 4-byte Folded Reload
; NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s31, v0, 0
; NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:16
; NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[4:5]
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 s[4:5], exec
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, 1
; NO-SPILL-TO-VGPR-NEXT:    buffer_store_dword v0, off, s[0:3], s33 offset:16
; NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:4 ; 4-byte Folded Reload
; NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-SPILL-TO-VGPR-NEXT:    v_readlane_b32 s30, v0, 0
; NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:16
; NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b64 exec, s[4:5]
; NO-SPILL-TO-VGPR-NEXT:    buffer_load_dword v0, off, s[0:3], s33 offset:12 ; 4-byte Folded Reload
; NO-SPILL-TO-VGPR-NEXT:    s_addk_i32 s32, 0xf800
; NO-SPILL-TO-VGPR-NEXT:    s_waitcnt vmcnt(0)
; NO-SPILL-TO-VGPR-NEXT:    v_readfirstlane_b32 s4, v0
; NO-SPILL-TO-VGPR-NEXT:    s_mov_b32 s33, s4
; NO-SPILL-TO-VGPR-NEXT:    s_setpc_b64 s[30:31]
  %alloca = alloca i32, addrspace(5)
  store volatile i32 0, ptr addrspace(5) %alloca
  call void @external_void_func_void()
  ret void
}

attributes #0 = { nounwind "amdgpu-no-dispatch-id" "amdgpu-no-dispatch-ptr" "amdgpu-no-implicitarg-ptr" "amdgpu-no-workgroup-id-x" "amdgpu-no-workgroup-id-y" "amdgpu-no-workgroup-id-z" "amdgpu-no-workitem-id-x" "amdgpu-no-workitem-id-y" "amdgpu-no-workitem-id-z" }
