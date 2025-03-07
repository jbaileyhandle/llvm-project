; NOTE: Assertions have been autogenerated by utils/update_mir_test_checks.py
; RUN: llc -global-isel -mtriple=amdgcn-mesa-mesa3d -mcpu=gfx810 -stop-after=instruction-select -verify-machineinstrs -o - %s | FileCheck %s

; Natural mapping
define amdgpu_ps void @struct_buffer_store_f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset(float %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $vgpr0, $vgpr1, $vgpr2
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY1]], %subreg.sub0, [[PRED_COPY2]], %subreg.sub1, [[PRED_COPY3]], %subreg.sub2, [[PRED_COPY4]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY5]], %subreg.sub0, [[PRED_COPY6]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_DWORD_BOTHEN_exact [[PRED_COPY]], [[REG_SEQUENCE1]], [[REG_SEQUENCE]], [[PRED_COPY7]], 0, 0, 0, implicit $exec :: (dereferenceable store (s32), align 1, addrspace 8)
  ; CHECK-NEXT:   S_ENDPGM 0
  call void @llvm.amdgcn.struct.buffer.store.f32(float %val, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
  ret void
}

define amdgpu_ps void @struct_buffer_store_v2f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset(<2 x float> %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_v2f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $vgpr0, $vgpr1, $vgpr2, $vgpr3
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY]], %subreg.sub0, [[PRED_COPY1]], %subreg.sub1
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY2]], %subreg.sub0, [[PRED_COPY3]], %subreg.sub1, [[PRED_COPY4]], %subreg.sub2, [[PRED_COPY5]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr3
  ; CHECK-NEXT:   [[PRED_COPY8:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[REG_SEQUENCE2:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY6]], %subreg.sub0, [[PRED_COPY7]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_DWORDX2_BOTHEN_exact [[REG_SEQUENCE]], [[REG_SEQUENCE2]], [[REG_SEQUENCE1]], [[PRED_COPY8]], 0, 0, 0, implicit $exec :: (dereferenceable store (<2 x s32>), align 1, addrspace 8)
  ; CHECK-NEXT:   S_ENDPGM 0
  call void @llvm.amdgcn.struct.buffer.store.v2f32(<2 x float> %val, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
  ret void
}

define amdgpu_ps void @struct_buffer_store_v3f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset(<3 x float> %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_v3f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $vgpr0, $vgpr1, $vgpr2, $vgpr3, $vgpr4
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:vreg_96 = REG_SEQUENCE [[PRED_COPY]], %subreg.sub0, [[PRED_COPY1]], %subreg.sub1, [[PRED_COPY2]], %subreg.sub2
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY3]], %subreg.sub0, [[PRED_COPY4]], %subreg.sub1, [[PRED_COPY5]], %subreg.sub2, [[PRED_COPY6]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr3
  ; CHECK-NEXT:   [[PRED_COPY8:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr4
  ; CHECK-NEXT:   [[PRED_COPY9:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[REG_SEQUENCE2:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY7]], %subreg.sub0, [[PRED_COPY8]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_DWORDX3_BOTHEN_exact [[REG_SEQUENCE]], [[REG_SEQUENCE2]], [[REG_SEQUENCE1]], [[PRED_COPY9]], 0, 0, 0, implicit $exec :: (dereferenceable store (<3 x s32>), align 1, addrspace 8)
  ; CHECK-NEXT:   S_ENDPGM 0
  call void @llvm.amdgcn.struct.buffer.store.v3f32(<3 x float> %val, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
  ret void
}

define amdgpu_ps void @struct_buffer_store_v4f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset(<4 x float> %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_v4f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $vgpr0, $vgpr1, $vgpr2, $vgpr3, $vgpr4, $vgpr5
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr3
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:vreg_128 = REG_SEQUENCE [[PRED_COPY]], %subreg.sub0, [[PRED_COPY1]], %subreg.sub1, [[PRED_COPY2]], %subreg.sub2, [[PRED_COPY3]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY4]], %subreg.sub0, [[PRED_COPY5]], %subreg.sub1, [[PRED_COPY6]], %subreg.sub2, [[PRED_COPY7]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY8:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr4
  ; CHECK-NEXT:   [[PRED_COPY9:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr5
  ; CHECK-NEXT:   [[PRED_COPY10:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[REG_SEQUENCE2:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY8]], %subreg.sub0, [[PRED_COPY9]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_DWORDX4_BOTHEN_exact [[REG_SEQUENCE]], [[REG_SEQUENCE2]], [[REG_SEQUENCE1]], [[PRED_COPY10]], 0, 0, 0, implicit $exec :: (dereferenceable store (<4 x s32>), align 1, addrspace 8)
  ; CHECK-NEXT:   S_ENDPGM 0
  call void @llvm.amdgcn.struct.buffer.store.v4f32(<4 x float> %val, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
  ret void
}

define amdgpu_ps void @struct_buffer_store_v4f32_vgpr_rsrc__sgpr_val__sgpr_vindex__sgpr_voffset__vgpr_soffset(<4 x float> inreg %val, <4 x i32> %rsrc, i32 inreg %vindex, i32 inreg %voffset, i32 %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_v4f32_vgpr_rsrc__sgpr_val__sgpr_vindex__sgpr_voffset__vgpr_soffset
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   successors: %bb.2(0x80000000)
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $sgpr7, $vgpr0, $vgpr1, $vgpr2, $vgpr3, $vgpr4
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY]], %subreg.sub0, [[PRED_COPY1]], %subreg.sub1, [[PRED_COPY2]], %subreg.sub2, [[PRED_COPY3]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr3
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:vreg_128 = REG_SEQUENCE [[PRED_COPY4]], %subreg.sub0, [[PRED_COPY5]], %subreg.sub1, [[PRED_COPY6]], %subreg.sub2, [[PRED_COPY7]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY8:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[PRED_COPY9:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr7
  ; CHECK-NEXT:   [[PRED_COPY10:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr4
  ; CHECK-NEXT:   [[PRED_COPY11:%[0-9]+]]:vreg_128 = PRED_COPY [[REG_SEQUENCE]]
  ; CHECK-NEXT:   [[PRED_COPY12:%[0-9]+]]:vgpr_32 = PRED_COPY [[PRED_COPY8]]
  ; CHECK-NEXT:   [[PRED_COPY13:%[0-9]+]]:vgpr_32 = PRED_COPY [[PRED_COPY9]]
  ; CHECK-NEXT:   [[S_MOV_B64_:%[0-9]+]]:sreg_64_xexec = S_MOV_B64 $exec
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT: bb.2:
  ; CHECK-NEXT:   successors: %bb.3(0x80000000)
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[V_READFIRSTLANE_B32_:%[0-9]+]]:sreg_32 = V_READFIRSTLANE_B32 [[PRED_COPY4]], implicit $exec
  ; CHECK-NEXT:   [[V_READFIRSTLANE_B32_1:%[0-9]+]]:sreg_32 = V_READFIRSTLANE_B32 [[PRED_COPY5]], implicit $exec
  ; CHECK-NEXT:   [[V_READFIRSTLANE_B32_2:%[0-9]+]]:sreg_32 = V_READFIRSTLANE_B32 [[PRED_COPY6]], implicit $exec
  ; CHECK-NEXT:   [[V_READFIRSTLANE_B32_3:%[0-9]+]]:sreg_32 = V_READFIRSTLANE_B32 [[PRED_COPY7]], implicit $exec
  ; CHECK-NEXT:   [[REG_SEQUENCE2:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[V_READFIRSTLANE_B32_]], %subreg.sub0, [[V_READFIRSTLANE_B32_1]], %subreg.sub1, [[V_READFIRSTLANE_B32_2]], %subreg.sub2, [[V_READFIRSTLANE_B32_3]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY14:%[0-9]+]]:vreg_64 = PRED_COPY [[REG_SEQUENCE1]].sub0_sub1
  ; CHECK-NEXT:   [[PRED_COPY15:%[0-9]+]]:vreg_64 = PRED_COPY [[REG_SEQUENCE1]].sub2_sub3
  ; CHECK-NEXT:   [[PRED_COPY16:%[0-9]+]]:sreg_64 = PRED_COPY [[REG_SEQUENCE2]].sub0_sub1
  ; CHECK-NEXT:   [[PRED_COPY17:%[0-9]+]]:sreg_64 = PRED_COPY [[REG_SEQUENCE2]].sub2_sub3
  ; CHECK-NEXT:   [[V_CMP_EQ_U64_e64_:%[0-9]+]]:sreg_64_xexec = V_CMP_EQ_U64_e64 [[PRED_COPY16]], [[PRED_COPY14]], implicit $exec
  ; CHECK-NEXT:   [[V_CMP_EQ_U64_e64_1:%[0-9]+]]:sreg_64_xexec = V_CMP_EQ_U64_e64 [[PRED_COPY17]], [[PRED_COPY15]], implicit $exec
  ; CHECK-NEXT:   [[S_AND_B64_:%[0-9]+]]:sreg_64_xexec = S_AND_B64 [[V_CMP_EQ_U64_e64_]], [[V_CMP_EQ_U64_e64_1]], implicit-def $scc
  ; CHECK-NEXT:   [[V_READFIRSTLANE_B32_4:%[0-9]+]]:sreg_32 = V_READFIRSTLANE_B32 [[PRED_COPY10]], implicit $exec
  ; CHECK-NEXT:   [[V_CMP_EQ_U32_e64_:%[0-9]+]]:sreg_64_xexec = V_CMP_EQ_U32_e64 [[V_READFIRSTLANE_B32_4]], [[PRED_COPY10]], implicit $exec
  ; CHECK-NEXT:   [[S_AND_B64_1:%[0-9]+]]:sreg_64_xexec = S_AND_B64 [[S_AND_B64_]], [[V_CMP_EQ_U32_e64_]], implicit-def $scc
  ; CHECK-NEXT:   [[S_AND_SAVEEXEC_B64_:%[0-9]+]]:sreg_64_xexec = S_AND_SAVEEXEC_B64 killed [[S_AND_B64_1]], implicit-def $exec, implicit-def $scc, implicit $exec
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT: bb.3:
  ; CHECK-NEXT:   successors: %bb.4(0x40000000), %bb.2(0x40000000)
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[REG_SEQUENCE3:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY12]], %subreg.sub0, [[PRED_COPY13]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_DWORDX4_BOTHEN_exact [[PRED_COPY11]], [[REG_SEQUENCE3]], [[REG_SEQUENCE2]], [[V_READFIRSTLANE_B32_4]], 0, 0, 0, implicit $exec :: (dereferenceable store (<4 x s32>), align 1, addrspace 8)
  ; CHECK-NEXT:   $exec = S_XOR_B64_term $exec, [[S_AND_SAVEEXEC_B64_]], implicit-def $scc
  ; CHECK-NEXT:   SI_WATERFALL_LOOP %bb.2, implicit $exec
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT: bb.4:
  ; CHECK-NEXT:   successors: %bb.5(0x80000000)
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   $exec = S_MOV_B64_term [[S_MOV_B64_]]
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT: bb.5:
  ; CHECK-NEXT:   S_ENDPGM 0
  call void @llvm.amdgcn.struct.buffer.store.v4f32(<4 x float> %val, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
  ret void
}

define amdgpu_ps void @struct_buffer_store_i8_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset(i32 %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_i8_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $vgpr0, $vgpr1, $vgpr2
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY1]], %subreg.sub0, [[PRED_COPY2]], %subreg.sub1, [[PRED_COPY3]], %subreg.sub2, [[PRED_COPY4]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY5]], %subreg.sub0, [[PRED_COPY6]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_BYTE_BOTHEN_exact [[PRED_COPY]], [[REG_SEQUENCE1]], [[REG_SEQUENCE]], [[PRED_COPY7]], 0, 0, 0, implicit $exec :: (dereferenceable store (s8), addrspace 8)
  ; CHECK-NEXT:   S_ENDPGM 0
  %val.trunc = trunc i32 %val to i8
  call void @llvm.amdgcn.struct.buffer.store.i8(i8 %val.trunc, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
  ret void
}

define amdgpu_ps void @struct_buffer_store_i16_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset(i32 %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_i16_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $vgpr0, $vgpr1, $vgpr2
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY1]], %subreg.sub0, [[PRED_COPY2]], %subreg.sub1, [[PRED_COPY3]], %subreg.sub2, [[PRED_COPY4]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY5]], %subreg.sub0, [[PRED_COPY6]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_SHORT_BOTHEN_exact [[PRED_COPY]], [[REG_SEQUENCE1]], [[REG_SEQUENCE]], [[PRED_COPY7]], 0, 0, 0, implicit $exec :: (dereferenceable store (s16), align 1, addrspace 8)
  ; CHECK-NEXT:   S_ENDPGM 0
  %val.trunc = trunc i32 %val to i16
  call void @llvm.amdgcn.struct.buffer.store.i16(i16 %val.trunc, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
  ret void
}

define amdgpu_ps void @struct_buffer_store_f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset_glc(float %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_f32_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset_glc
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $vgpr0, $vgpr1, $vgpr2
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY1]], %subreg.sub0, [[PRED_COPY2]], %subreg.sub1, [[PRED_COPY3]], %subreg.sub2, [[PRED_COPY4]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY5]], %subreg.sub0, [[PRED_COPY6]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_DWORD_BOTHEN_exact [[PRED_COPY]], [[REG_SEQUENCE1]], [[REG_SEQUENCE]], [[PRED_COPY7]], 0, 1, 0, implicit $exec :: (dereferenceable store (s32), align 1, addrspace 8)
  ; CHECK-NEXT:   S_ENDPGM 0
  call void @llvm.amdgcn.struct.buffer.store.f32(float %val, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 1)
  ret void
}

define amdgpu_ps void @struct_buffer_store_v2f16_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset(<2 x half> %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_v2f16_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $vgpr0, $vgpr1, $vgpr2
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY1]], %subreg.sub0, [[PRED_COPY2]], %subreg.sub1, [[PRED_COPY3]], %subreg.sub2, [[PRED_COPY4]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY5]], %subreg.sub0, [[PRED_COPY6]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_DWORD_BOTHEN_exact [[PRED_COPY]], [[REG_SEQUENCE1]], [[REG_SEQUENCE]], [[PRED_COPY7]], 0, 0, 0, implicit $exec :: (dereferenceable store (<2 x s16>), align 1, addrspace 8)
  ; CHECK-NEXT:   S_ENDPGM 0
  call void @llvm.amdgcn.struct.buffer.store.v2f16(<2 x half> %val, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
  ret void
}

; FIXME:
; define amdgpu_ps void @struct_buffer_store_v3f16_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset(<3 x half> %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
;   call void @llvm.amdgcn.struct.buffer.store.v3f16(<3 x half> %val, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
;   ret void
; }

define amdgpu_ps void @struct_buffer_store_v4f16_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset(<4 x half> %val, <4 x i32> inreg %rsrc, i32 %vindex, i32 %voffset, i32 inreg %soffset) {
  ; CHECK-LABEL: name: struct_buffer_store_v4f16_sgpr_rsrc__vgpr_val__vgpr_vindex__vgpr_voffset__sgpr_soffset
  ; CHECK: bb.1 (%ir-block.0):
  ; CHECK-NEXT:   liveins: $sgpr2, $sgpr3, $sgpr4, $sgpr5, $sgpr6, $vgpr0, $vgpr1, $vgpr2, $vgpr3
  ; CHECK-NEXT: {{  $}}
  ; CHECK-NEXT:   [[PRED_COPY:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr0
  ; CHECK-NEXT:   [[PRED_COPY1:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr1
  ; CHECK-NEXT:   [[REG_SEQUENCE:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY]], %subreg.sub0, [[PRED_COPY1]], %subreg.sub1
  ; CHECK-NEXT:   [[PRED_COPY2:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr2
  ; CHECK-NEXT:   [[PRED_COPY3:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr3
  ; CHECK-NEXT:   [[PRED_COPY4:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr4
  ; CHECK-NEXT:   [[PRED_COPY5:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr5
  ; CHECK-NEXT:   [[REG_SEQUENCE1:%[0-9]+]]:sgpr_128 = REG_SEQUENCE [[PRED_COPY2]], %subreg.sub0, [[PRED_COPY3]], %subreg.sub1, [[PRED_COPY4]], %subreg.sub2, [[PRED_COPY5]], %subreg.sub3
  ; CHECK-NEXT:   [[PRED_COPY6:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr2
  ; CHECK-NEXT:   [[PRED_COPY7:%[0-9]+]]:vgpr_32 = PRED_COPY $vgpr3
  ; CHECK-NEXT:   [[PRED_COPY8:%[0-9]+]]:sreg_32 = PRED_COPY $sgpr6
  ; CHECK-NEXT:   [[REG_SEQUENCE2:%[0-9]+]]:vreg_64 = REG_SEQUENCE [[PRED_COPY6]], %subreg.sub0, [[PRED_COPY7]], %subreg.sub1
  ; CHECK-NEXT:   BUFFER_STORE_DWORDX2_BOTHEN_exact [[REG_SEQUENCE]], [[REG_SEQUENCE2]], [[REG_SEQUENCE1]], [[PRED_COPY8]], 0, 0, 0, implicit $exec :: (dereferenceable store (<4 x s16>), align 1, addrspace 8)
  ; CHECK-NEXT:   S_ENDPGM 0
  call void @llvm.amdgcn.struct.buffer.store.v4f16(<4 x half> %val, <4 x i32> %rsrc, i32 %vindex, i32 %voffset, i32 %soffset, i32 0)
  ret void
}

declare void @llvm.amdgcn.struct.buffer.store.i8(i8, <4 x i32>, i32, i32, i32, i32 immarg)
declare void @llvm.amdgcn.struct.buffer.store.i16(i16, <4 x i32>, i32, i32, i32, i32 immarg)

declare void @llvm.amdgcn.struct.buffer.store.f16(half, <4 x i32>, i32, i32, i32, i32 immarg)
declare void @llvm.amdgcn.struct.buffer.store.v2f16(<2 x half>, <4 x i32>, i32, i32, i32, i32 immarg)
declare void @llvm.amdgcn.struct.buffer.store.v3f16(<3 x half>, <4 x i32>, i32, i32, i32, i32 immarg)
declare void @llvm.amdgcn.struct.buffer.store.v4f16(<4 x half>, <4 x i32>, i32, i32, i32, i32 immarg)

declare void @llvm.amdgcn.struct.buffer.store.f32(float, <4 x i32>, i32, i32, i32, i32 immarg)
declare void @llvm.amdgcn.struct.buffer.store.v2f32(<2 x float>, <4 x i32>, i32, i32, i32, i32 immarg)
declare void @llvm.amdgcn.struct.buffer.store.v3f32(<3 x float>, <4 x i32>, i32, i32, i32, i32 immarg)
declare void @llvm.amdgcn.struct.buffer.store.v4f32(<4 x float>, <4 x i32>, i32, i32, i32, i32 immarg)
