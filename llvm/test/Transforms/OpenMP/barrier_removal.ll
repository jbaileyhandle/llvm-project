; NOTE: Assertions have been autogenerated by utils/update_test_checks.py UTC_ARGS: --function-signature --check-globals
; RUN: opt < %s -S -passes=openmp-opt | FileCheck %s --check-prefixes=CHECK,MODULE
; RUN: opt < %s -S -passes=openmp-opt-cgscc | FileCheck %s --check-prefixes=CHECK,CGSCC
target triple = "amdgcn-amd-amdhsa"

declare void @useI32(i32)
declare void @unknown()
declare void @aligned_barrier() "llvm.assume"="ompx_aligned_barrier"
declare void @llvm.nvvm.barrier0()
declare i32 @llvm.nvvm.barrier0.and(i32)
declare i32 @llvm.nvvm.barrier0.or(i32)
declare i32 @llvm.nvvm.barrier0.popc(i32)
declare void @llvm.amdgcn.s.barrier()
declare void @llvm.assume(i1)

;.
; CHECK: @[[GC1:[a-zA-Z0-9_$"\\.-]+]] = constant i32 42
; CHECK: @[[GC2:[a-zA-Z0-9_$"\\.-]+]] = addrspace(4) global i32 0
; CHECK: @[[GPTR4:[a-zA-Z0-9_$"\\.-]+]] = addrspace(4) global ptr addrspace(4) null
; CHECK: @[[G:[a-zA-Z0-9_$"\\.-]+]] = global i32 42
; CHECK: @[[GS:[a-zA-Z0-9_$"\\.-]+]] = addrspace(3) global i32 0
; CHECK: @[[GPTR:[a-zA-Z0-9_$"\\.-]+]] = global ptr null
; CHECK: @[[PG1:[a-zA-Z0-9_$"\\.-]+]] = thread_local global i32 42
; CHECK: @[[PG2:[a-zA-Z0-9_$"\\.-]+]] = addrspace(5) global i32 0
; CHECK: @[[GPTR5:[a-zA-Z0-9_$"\\.-]+]] = global ptr addrspace(5) null
; CHECK: @[[G1:[a-zA-Z0-9_$"\\.-]+]] = global i32 42
; CHECK: @[[G2:[a-zA-Z0-9_$"\\.-]+]] = addrspace(1) global i32 0
;.
define void @pos_empty_1() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_1
; CHECK-SAME: () #[[ATTR4:[0-9]+]] {
; CHECK-NEXT:    ret void
;
  call void @llvm.assume(i1 true)
  call void @unknown() "llvm.assume"="ompx_aligned_barrier"
  call void @llvm.assume(i1 true)
  ret void
}
define void @pos_empty_2() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_2
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    ret void
;
  call void @aligned_barrier()
  ret void
}
define void @pos_empty_3() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_3
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    ret void
;
  call void @llvm.nvvm.barrier0()
  ret void
}
define void @pos_empty_4() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_4
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    ret void
;
  call i32 @llvm.nvvm.barrier0.and(i32 0)
  ret void
}
define void @pos_empty_5() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_5
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    ret void
;
  call i32 @llvm.nvvm.barrier0.or(i32 0)
  ret void
}
define void @pos_empty_6() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_6
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    ret void
;
  call i32 @llvm.nvvm.barrier0.popc(i32 0)
  ret void
}
define void @pos_empty_7a() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_7a
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    call void @unknown()
; CHECK-NEXT:    ret void
;
  call void @llvm.amdgcn.s.barrier()
  call void @unknown()
  ret void
}
; FIXME: We should remove the barrier.
define void @pos_empty_7b() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_7b
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    call void @unknown() #[[ATTR5:[0-9]+]]
; CHECK-NEXT:    call void @llvm.amdgcn.s.barrier()
; CHECK-NEXT:    call void @unknown()
; CHECK-NEXT:    ret void
;
  call void @unknown() nosync readnone
  call void @llvm.amdgcn.s.barrier()
  call void @unknown()
  ret void
}
define void @pos_empty_8(i1 %c) "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_8
; CHECK-SAME: (i1 [[C:%.*]]) #[[ATTR4]] {
; CHECK-NEXT:    br i1 [[C]], label [[T:%.*]], label [[F:%.*]]
; CHECK:       t:
; CHECK-NEXT:    call void @llvm.amdgcn.s.barrier() #[[ATTR0:[0-9]+]]
; CHECK-NEXT:    br label [[F]]
; CHECK:       f:
; CHECK-NEXT:    ret void
;
  br i1 %c, label %t, label %f
t:
  fence release
  call void @llvm.amdgcn.s.barrier() "llvm.assume"="ompx_aligned_barrier"
  br label %f
f:
  ret void
}
define void @neg_empty_8() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@neg_empty_8
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    call void @unknown()
; CHECK-NEXT:    call void @llvm.amdgcn.s.barrier()
; CHECK-NEXT:    ret void
;
  call void @unknown()
  call void @llvm.amdgcn.s.barrier()
  ret void
}
define void @neg_empty_9(i1 %c) "kernel" {
; CHECK-LABEL: define {{[^@]+}}@neg_empty_9
; CHECK-SAME: (i1 [[C:%.*]]) #[[ATTR4]] {
; CHECK-NEXT:    br i1 [[C]], label [[T:%.*]], label [[F:%.*]]
; CHECK:       t:
; CHECK-NEXT:    call void @llvm.amdgcn.s.barrier()
; CHECK-NEXT:    fence release
; CHECK-NEXT:    br label [[M:%.*]]
; CHECK:       f:
; CHECK-NEXT:    call void @llvm.amdgcn.s.barrier()
; CHECK-NEXT:    fence release
; CHECK-NEXT:    br label [[M]]
; CHECK:       m:
; CHECK-NEXT:    fence release
; CHECK-NEXT:    call void @llvm.amdgcn.s.barrier()
; CHECK-NEXT:    fence release
; CHECK-NEXT:    ret void
;
  br i1 %c, label %t, label %f
t:
  fence release
  call void @llvm.amdgcn.s.barrier()
  fence release
  br label %m
f:
  fence release
  call void @llvm.amdgcn.s.barrier()
  fence release
  br label %m
m:
  fence release
  call void @llvm.amdgcn.s.barrier()
  fence release
  ret void
}
; FIXME: We should remove the barrier
define void @pos_empty_10() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_10
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    br label [[M:%.*]]
; CHECK:       m:
; CHECK-NEXT:    call void @llvm.amdgcn.s.barrier()
; CHECK-NEXT:    ret void
;
  br label %m
m:
  call void @llvm.amdgcn.s.barrier()
  ret void
}
define void @pos_empty_11() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_empty_11
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    br label [[M:%.*]]
; CHECK:       m:
; CHECK-NEXT:    ret void
;
  br label %m
m:
  call void @aligned_barrier()
  call void @llvm.amdgcn.s.barrier()
  ret void
}
define void @empty() {
; CHECK-LABEL: define {{[^@]+}}@empty() {
; CHECK-NEXT:    ret void
;
  ret void
}
; FIXME: We should remove the barrier in the end but not the first one.
define void @neg_empty_12(i1 %c) "kernel" {
; MODULE-LABEL: define {{[^@]+}}@neg_empty_12
; MODULE-SAME: (i1 [[C:%.*]]) #[[ATTR4]] {
; MODULE-NEXT:    br i1 [[C]], label [[T:%.*]], label [[F:%.*]]
; MODULE:       t:
; MODULE-NEXT:    call void @llvm.amdgcn.s.barrier()
; MODULE-NEXT:    br label [[M:%.*]]
; MODULE:       f:
; MODULE-NEXT:    br label [[M]]
; MODULE:       m:
; MODULE-NEXT:    call void @llvm.amdgcn.s.barrier()
; MODULE-NEXT:    ret void
;
; CGSCC-LABEL: define {{[^@]+}}@neg_empty_12
; CGSCC-SAME: (i1 [[C:%.*]]) #[[ATTR4]] {
; CGSCC-NEXT:    br i1 [[C]], label [[T:%.*]], label [[F:%.*]]
; CGSCC:       t:
; CGSCC-NEXT:    call void @empty()
; CGSCC-NEXT:    call void @llvm.amdgcn.s.barrier()
; CGSCC-NEXT:    br label [[M:%.*]]
; CGSCC:       f:
; CGSCC-NEXT:    call void @empty()
; CGSCC-NEXT:    br label [[M]]
; CGSCC:       m:
; CGSCC-NEXT:    call void @llvm.amdgcn.s.barrier()
; CGSCC-NEXT:    ret void
;
  br i1 %c, label %t, label %f
t:
  call void @empty()
  call void @llvm.amdgcn.s.barrier()
  br label %m
f:
  call void @empty()
  br label %m
m:
  call void @llvm.amdgcn.s.barrier()
  ret void
}
define void @neg_empty_1() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@neg_empty_1
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    call void @unknown()
; CHECK-NEXT:    ret void
;
  call void @unknown()
  ret void
}
define void @neg_empty_2() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@neg_empty_2
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    ret void
;
  call void @aligned_barrier()
  ret void
}

@GC1 = constant i32 42
@GC2 = addrspace(4) global i32 0
@GPtr4 = addrspace(4) global ptr addrspace(4) null
define void @pos_constant_loads() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_constant_loads
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    [[ARG:%.*]] = load ptr addrspace(4), ptr addrspacecast (ptr addrspace(4) @GPtr4 to ptr), align 8
; CHECK-NEXT:    [[B:%.*]] = load i32, ptr addrspacecast (ptr addrspace(4) @GC2 to ptr), align 4
; CHECK-NEXT:    [[ARGC:%.*]] = addrspacecast ptr addrspace(4) [[ARG]] to ptr
; CHECK-NEXT:    [[C:%.*]] = load i32, ptr [[ARGC]], align 4
; CHECK-NEXT:    [[D:%.*]] = add i32 42, [[B]]
; CHECK-NEXT:    [[E:%.*]] = add i32 [[D]], [[C]]
; CHECK-NEXT:    call void @useI32(i32 [[E]])
; CHECK-NEXT:    ret void
;
  %GPtr4c = addrspacecast ptr addrspace(4) @GPtr4 to ptr
  %arg = load ptr addrspace(4), ptr %GPtr4c
  %a = load i32, ptr @GC1
  call void @aligned_barrier()
  %GC2c = addrspacecast ptr addrspace(4) @GC2 to ptr
  %b = load i32, ptr %GC2c
  call void @aligned_barrier()
  %argc = addrspacecast ptr addrspace(4) %arg to ptr
  %c = load i32, ptr %argc
  call void @aligned_barrier()
  %d = add i32 %a, %b
  %e = add i32 %d, %c
  call void @useI32(i32 %e)
  ret void
}
@G = global i32 42
@GS = addrspace(3) global i32 0
@GPtr = global ptr null
; TODO: We could remove some of the barriers due to the lack of write effects.
define void @neg_loads() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@neg_loads
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    [[ARG:%.*]] = load ptr, ptr @GPtr, align 8
; CHECK-NEXT:    [[A:%.*]] = load i32, ptr @G, align 4
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    [[B:%.*]] = load i32, ptr addrspacecast (ptr addrspace(3) @GS to ptr), align 4
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    [[C:%.*]] = load i32, ptr [[ARG]], align 4
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    [[D:%.*]] = add i32 [[A]], [[B]]
; CHECK-NEXT:    [[E:%.*]] = add i32 [[D]], [[C]]
; CHECK-NEXT:    call void @useI32(i32 [[E]])
; CHECK-NEXT:    ret void
;
  %arg = load ptr, ptr @GPtr
  %a = load i32, ptr @G
  call void @aligned_barrier()
  %GSc = addrspacecast ptr addrspace(3) @GS to ptr
  %b = load i32, ptr %GSc
  call void @aligned_barrier()
  %c = load i32, ptr %arg
  call void @aligned_barrier()
  %d = add i32 %a, %b
  %e = add i32 %d, %c
  call void @useI32(i32 %e)
  ret void
}
@PG1 = thread_local global i32 42
@PG2 = addrspace(5) global i32 0
@GPtr5 = global ptr addrspace(5) null
define void @pos_priv_mem() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_priv_mem
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    [[ARG:%.*]] = load ptr addrspace(5), ptr @GPtr5, align 8
; CHECK-NEXT:    [[LOC:%.*]] = alloca i32, align 4
; CHECK-NEXT:    [[A:%.*]] = load i32, ptr @PG1, align 4
; CHECK-NEXT:    store i32 [[A]], ptr [[LOC]], align 4
; CHECK-NEXT:    [[B:%.*]] = load i32, ptr addrspacecast (ptr addrspace(5) @PG2 to ptr), align 4
; CHECK-NEXT:    [[ARGC:%.*]] = addrspacecast ptr addrspace(5) [[ARG]] to ptr
; CHECK-NEXT:    store i32 [[B]], ptr [[ARGC]], align 4
; CHECK-NEXT:    [[V:%.*]] = load i32, ptr [[LOC]], align 4
; CHECK-NEXT:    store i32 [[V]], ptr @PG1, align 4
; CHECK-NEXT:    ret void
;
  %arg = load ptr addrspace(5), ptr @GPtr5
  %loc = alloca i32
  %a = load i32, ptr @PG1
  call void @aligned_barrier()
  store i32 %a, ptr %loc
  %PG2c = addrspacecast ptr addrspace(5) @PG2 to ptr
  %b = load i32, ptr %PG2c
  call void @aligned_barrier()
  %argc = addrspacecast ptr addrspace(5) %arg to ptr
  store i32 %b, ptr %argc
  call void @aligned_barrier()
  %v = load i32, ptr %loc
  store i32 %v, ptr @PG1
  call void @aligned_barrier()
  ret void
}
@G1 = global i32 42
@G2 = addrspace(1) global i32 0
define void @neg_mem() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@neg_mem
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    [[ARG:%.*]] = load ptr, ptr @GPtr, align 8
; CHECK-NEXT:    [[A:%.*]] = load i32, ptr @G1, align 4
; CHECK-NEXT:    fence seq_cst
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    store i32 [[A]], ptr [[ARG]], align 4
; CHECK-NEXT:    fence release
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    [[B:%.*]] = load i32, ptr addrspacecast (ptr addrspace(1) @G2 to ptr), align 4
; CHECK-NEXT:    store i32 [[B]], ptr @G1, align 4
; CHECK-NEXT:    fence acquire
; CHECK-NEXT:    ret void
;
  %arg = load ptr, ptr @GPtr
  %a = load i32, ptr @G1
  fence seq_cst
  call void @aligned_barrier()
  store i32 %a, ptr %arg
  fence release
  call void @aligned_barrier()
  %G2c = addrspacecast ptr addrspace(1) @G2 to ptr
  %b = load i32, ptr %G2c
  store i32 %b, ptr @G1
  fence acquire
  call void @aligned_barrier()
  ret void
}

define void @pos_multiple() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@pos_multiple
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    ret void
;
  call void @llvm.nvvm.barrier0()
  call void @aligned_barrier()
  call void @aligned_barrier()
  call void @llvm.amdgcn.s.barrier()
  call void @aligned_barrier()
  call void @llvm.nvvm.barrier0()
  call void @aligned_barrier()
  call void @aligned_barrier()
  ret void
}

define void @multiple_blocks_kernel_1(i1 %c0, i1 %c1) "kernel" {
; CHECK-LABEL: define {{[^@]+}}@multiple_blocks_kernel_1
; CHECK-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]]) #[[ATTR4]] {
; CHECK-NEXT:    br i1 [[C0]], label [[T0:%.*]], label [[F0:%.*]]
; CHECK:       t0:
; CHECK-NEXT:    br label [[T0B:%.*]]
; CHECK:       t0b:
; CHECK-NEXT:    br label [[M:%.*]]
; CHECK:       f0:
; CHECK-NEXT:    br i1 [[C1]], label [[T1:%.*]], label [[F1:%.*]]
; CHECK:       t1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       f1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       m:
; CHECK-NEXT:    ret void
;
  fence acquire
  call void @llvm.nvvm.barrier0()
  fence release
  call void @aligned_barrier()
  fence seq_cst
  br i1 %c0, label %t0, label %f0
t0:
  fence seq_cst
  call void @aligned_barrier()
  fence seq_cst
  br label %t0b
t0b:
  fence seq_cst
  call void @aligned_barrier()
  fence seq_cst
  br label %m
f0:
  fence release
  call void @aligned_barrier()
  fence acquire
  call void @llvm.nvvm.barrier0()
  fence acquire
  br i1 %c1, label %t1, label %f1
t1:
  fence acquire
  call void @aligned_barrier()
  fence seq_cst
  br label %m
f1:
  fence seq_cst
  call void @aligned_barrier()
  fence acquire
  br label %m
m:
  fence seq_cst
  call void @aligned_barrier()
  fence seq_cst
  ret void
}

define void @multiple_blocks_kernel_2(i1 %c0, i1 %c1, i32* %p) "kernel" {
; CHECK-LABEL: define {{[^@]+}}@multiple_blocks_kernel_2
; CHECK-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]], ptr [[P:%.*]]) #[[ATTR4]] {
; CHECK-NEXT:    store i32 4, ptr [[P]], align 4
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    br i1 [[C0]], label [[T0:%.*]], label [[F0:%.*]]
; CHECK:       t0:
; CHECK-NEXT:    br label [[T0B:%.*]]
; CHECK:       t0b:
; CHECK-NEXT:    br label [[M:%.*]]
; CHECK:       f0:
; CHECK-NEXT:    store i32 4, ptr [[P]], align 4
; CHECK-NEXT:    call void @llvm.nvvm.barrier0()
; CHECK-NEXT:    br i1 [[C1]], label [[T1:%.*]], label [[F1:%.*]]
; CHECK:       t1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       f1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       m:
; CHECK-NEXT:    store i32 4, ptr [[P]], align 4
; CHECK-NEXT:    ret void
;
  call void @llvm.nvvm.barrier0()
  store i32 4, i32* %p
  call void @aligned_barrier()
  br i1 %c0, label %t0, label %f0
t0:
  call void @aligned_barrier()
  br label %t0b
t0b:
  call void @aligned_barrier()
  br label %m
f0:
  call void @aligned_barrier()
  store i32 4, i32* %p
  call void @llvm.nvvm.barrier0()
  br i1 %c1, label %t1, label %f1
t1:
  call void @aligned_barrier()
  br label %m
f1:
  call void @aligned_barrier()
  br label %m
m:
  store i32 4, i32* %p
  call void @aligned_barrier()
  ret void
}

define void @multiple_blocks_non_kernel_1(i1 %c0, i1 %c1) "kernel" {
; CHECK-LABEL: define {{[^@]+}}@multiple_blocks_non_kernel_1
; CHECK-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]]) #[[ATTR4]] {
; CHECK-NEXT:    br i1 [[C0]], label [[T0:%.*]], label [[F0:%.*]]
; CHECK:       t0:
; CHECK-NEXT:    br label [[T0B:%.*]]
; CHECK:       t0b:
; CHECK-NEXT:    br label [[M:%.*]]
; CHECK:       f0:
; CHECK-NEXT:    br i1 [[C1]], label [[T1:%.*]], label [[F1:%.*]]
; CHECK:       t1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       f1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       m:
; CHECK-NEXT:    ret void
;
  call void @llvm.nvvm.barrier0()
  call void @aligned_barrier()
  br i1 %c0, label %t0, label %f0
t0:
  call void @aligned_barrier()
  br label %t0b
t0b:
  call void @aligned_barrier()
  br label %m
f0:
  call void @aligned_barrier()
  call void @llvm.nvvm.barrier0()
  br i1 %c1, label %t1, label %f1
t1:
  call void @aligned_barrier()
  br label %m
f1:
  call void @aligned_barrier()
  br label %m
m:
  call void @aligned_barrier()
  ret void
}

define void @multiple_blocks_non_kernel_2(i1 %c0, i1 %c1) "kernel" {
; CHECK-LABEL: define {{[^@]+}}@multiple_blocks_non_kernel_2
; CHECK-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]]) #[[ATTR4]] {
; CHECK-NEXT:    br i1 [[C0]], label [[T0:%.*]], label [[F0:%.*]]
; CHECK:       t0:
; CHECK-NEXT:    br label [[T0B:%.*]]
; CHECK:       t0b:
; CHECK-NEXT:    br label [[M:%.*]]
; CHECK:       f0:
; CHECK-NEXT:    br i1 [[C1]], label [[T1:%.*]], label [[F1:%.*]]
; CHECK:       t1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       f1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       m:
; CHECK-NEXT:    ret void
;
  br i1 %c0, label %t0, label %f0
t0:
  call void @aligned_barrier()
  br label %t0b
t0b:
  call void @aligned_barrier()
  br label %m
f0:
  call void @aligned_barrier()
  call void @llvm.nvvm.barrier0()
  br i1 %c1, label %t1, label %f1
t1:
  call void @aligned_barrier()
  br label %m
f1:
  call void @aligned_barrier()
  br label %m
m:
  call void @aligned_barrier()
  ret void
}

define void @multiple_blocks_non_kernel_3(i1 %c0, i1 %c1) "kernel" {
; CHECK-LABEL: define {{[^@]+}}@multiple_blocks_non_kernel_3
; CHECK-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]]) #[[ATTR4]] {
; CHECK-NEXT:    br i1 [[C0]], label [[T0:%.*]], label [[F0:%.*]]
; CHECK:       t0:
; CHECK-NEXT:    br label [[T0B:%.*]]
; CHECK:       t0b:
; CHECK-NEXT:    br label [[M:%.*]]
; CHECK:       f0:
; CHECK-NEXT:    br i1 [[C1]], label [[T1:%.*]], label [[F1:%.*]]
; CHECK:       t1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       f1:
; CHECK-NEXT:    br label [[M]]
; CHECK:       m:
; CHECK-NEXT:    ret void
;
  br i1 %c0, label %t0, label %f0
t0:
  br label %t0b
t0b:
  br label %m
f0:
  call void @aligned_barrier()
  call void @llvm.nvvm.barrier0()
  br i1 %c1, label %t1, label %f1
t1:
  call void @aligned_barrier()
  br label %m
f1:
  call void @aligned_barrier()
  br label %m
m:
  call void @aligned_barrier()
  ret void
}

define void @multiple_blocks_non_kernel_effects_1(i1 %c0, i1 %c1, i32* %p) "kernel" {
; CHECK-LABEL: define {{[^@]+}}@multiple_blocks_non_kernel_effects_1
; CHECK-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]], ptr [[P:%.*]]) #[[ATTR4]] {
; CHECK-NEXT:    store i32 0, ptr [[P]], align 4
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    br i1 [[C0]], label [[T0:%.*]], label [[F0:%.*]]
; CHECK:       t0:
; CHECK-NEXT:    store i32 1, ptr [[P]], align 4
; CHECK-NEXT:    br label [[T0B:%.*]]
; CHECK:       t0b:
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    br label [[M:%.*]]
; CHECK:       f0:
; CHECK-NEXT:    store i32 2, ptr [[P]], align 4
; CHECK-NEXT:    br i1 [[C1]], label [[T1:%.*]], label [[F1:%.*]]
; CHECK:       t1:
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    br label [[M]]
; CHECK:       f1:
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    br label [[M]]
; CHECK:       m:
; CHECK-NEXT:    store i32 3, ptr [[P]], align 4
; CHECK-NEXT:    ret void
;
  call void @aligned_barrier()
  store i32 0, i32* %p
  call void @aligned_barrier()
  br i1 %c0, label %t0, label %f0
t0:
  call void @aligned_barrier()
  store i32 1, i32* %p
  br label %t0b
t0b:
  call void @aligned_barrier()
  br label %m
f0:
  call void @aligned_barrier()
  call void @llvm.nvvm.barrier0()
  store i32 2, i32* %p
  br i1 %c1, label %t1, label %f1
t1:
  call void @aligned_barrier()
  br label %m
f1:
  call void @aligned_barrier()
  br label %m
m:
  call void @aligned_barrier()
  store i32 3, i32* %p
  call void @aligned_barrier()
  ret void
}

define internal void @write_then_barrier0(i32* %p) {
; CHECK-LABEL: define {{[^@]+}}@write_then_barrier0
; CHECK-SAME: (ptr [[P:%.*]]) {
; CHECK-NEXT:    store i32 0, ptr [[P]], align 4
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    ret void
;
  store i32 0, i32* %p
  call void @aligned_barrier()
  ret void
}
define internal void @barrier_then_write0(i32* %p) {
; MODULE-LABEL: define {{[^@]+}}@barrier_then_write0
; MODULE-SAME: (ptr [[P:%.*]]) {
; MODULE-NEXT:    store i32 0, ptr [[P]], align 4
; MODULE-NEXT:    ret void
;
; CGSCC-LABEL: define {{[^@]+}}@barrier_then_write0
; CGSCC-SAME: (ptr [[P:%.*]]) {
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    store i32 0, ptr [[P]], align 4
; CGSCC-NEXT:    ret void
;
  call void @aligned_barrier()
  store i32 0, i32* %p
  ret void
}
define internal void @barrier_then_write_then_barrier0(i32* %p) {
; MODULE-LABEL: define {{[^@]+}}@barrier_then_write_then_barrier0
; MODULE-SAME: (ptr [[P:%.*]]) {
; MODULE-NEXT:    store i32 0, ptr [[P]], align 4
; MODULE-NEXT:    call void @aligned_barrier()
; MODULE-NEXT:    ret void
;
; CGSCC-LABEL: define {{[^@]+}}@barrier_then_write_then_barrier0
; CGSCC-SAME: (ptr [[P:%.*]]) {
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    store i32 0, ptr [[P]], align 4
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    ret void
;
  call void @aligned_barrier()
  store i32 0, i32* %p
  call void @aligned_barrier()
  ret void
}
define void @multiple_blocks_functions_kernel_effects_0(i1 %c0, i1 %c1, i32* %p) "kernel" {
; MODULE-LABEL: define {{[^@]+}}@multiple_blocks_functions_kernel_effects_0
; MODULE-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]], ptr [[P:%.*]]) #[[ATTR4]] {
; MODULE-NEXT:    call void @barrier_then_write_then_barrier0(ptr [[P]])
; MODULE-NEXT:    br i1 [[C0]], label [[T03:%.*]], label [[F03:%.*]]
; MODULE:       t03:
; MODULE-NEXT:    call void @barrier_then_write0(ptr [[P]])
; MODULE-NEXT:    br label [[T0B3:%.*]]
; MODULE:       t0b3:
; MODULE-NEXT:    br label [[M3:%.*]]
; MODULE:       f03:
; MODULE-NEXT:    call void @barrier_then_write0(ptr [[P]])
; MODULE-NEXT:    br i1 [[C1]], label [[T13:%.*]], label [[F13:%.*]]
; MODULE:       t13:
; MODULE-NEXT:    br label [[M3]]
; MODULE:       f13:
; MODULE-NEXT:    br label [[M3]]
; MODULE:       m3:
; MODULE-NEXT:    call void @write_then_barrier0(ptr [[P]])
; MODULE-NEXT:    ret void
;
; CGSCC-LABEL: define {{[^@]+}}@multiple_blocks_functions_kernel_effects_0
; CGSCC-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]], ptr [[P:%.*]]) #[[ATTR4]] {
; CGSCC-NEXT:    call void @barrier_then_write_then_barrier0(ptr [[P]])
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br i1 [[C0]], label [[T03:%.*]], label [[F03:%.*]]
; CGSCC:       t03:
; CGSCC-NEXT:    call void @barrier_then_write0(ptr [[P]])
; CGSCC-NEXT:    br label [[T0B3:%.*]]
; CGSCC:       t0b3:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br label [[M3:%.*]]
; CGSCC:       f03:
; CGSCC-NEXT:    call void @barrier_then_write0(ptr [[P]])
; CGSCC-NEXT:    br i1 [[C1]], label [[T13:%.*]], label [[F13:%.*]]
; CGSCC:       t13:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br label [[M3]]
; CGSCC:       f13:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br label [[M3]]
; CGSCC:       m3:
; CGSCC-NEXT:    call void @write_then_barrier0(ptr [[P]])
; CGSCC-NEXT:    ret void
;
  call void @barrier_then_write_then_barrier0(i32* %p)
  call void @aligned_barrier()
  br i1 %c0, label %t03, label %f03
t03:
  call void @barrier_then_write0(i32* %p)
  br label %t0b3
t0b3:
  call void @aligned_barrier()
  br label %m3
f03:
  call void @aligned_barrier()
  call void @barrier_then_write0(i32* %p)
  br i1 %c1, label %t13, label %f13
t13:
  call void @aligned_barrier()
  br label %m3
f13:
  call void @aligned_barrier()
  br label %m3
m3:
  call void @aligned_barrier()
  call void @write_then_barrier0(i32* %p)
  ret void
}
define internal void @write_then_barrier1(i32* %p) {
; CHECK-LABEL: define {{[^@]+}}@write_then_barrier1
; CHECK-SAME: (ptr [[P:%.*]]) {
; CHECK-NEXT:    store i32 0, ptr [[P]], align 4
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    ret void
;
  store i32 0, i32* %p
  call void @aligned_barrier()
  ret void
}
define internal void @barrier_then_write1(i32* %p) {
; MODULE-LABEL: define {{[^@]+}}@barrier_then_write1
; MODULE-SAME: (ptr [[P:%.*]]) {
; MODULE-NEXT:    store i32 0, ptr [[P]], align 4
; MODULE-NEXT:    ret void
;
; CGSCC-LABEL: define {{[^@]+}}@barrier_then_write1
; CGSCC-SAME: (ptr [[P:%.*]]) {
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    store i32 0, ptr [[P]], align 4
; CGSCC-NEXT:    ret void
;
  call void @aligned_barrier()
  store i32 0, i32* %p
  ret void
}
define internal void @barrier_then_write_then_barrier1(i32* %p) {
; CHECK-LABEL: define {{[^@]+}}@barrier_then_write_then_barrier1
; CHECK-SAME: (ptr [[P:%.*]]) {
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    store i32 0, ptr [[P]], align 4
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    ret void
;
  call void @aligned_barrier()
  store i32 0, i32* %p
  call void @aligned_barrier()
  ret void
}
define void @multiple_blocks_functions_non_kernel_effects_1(i1 %c0, i1 %c1, i32* %p) {
; MODULE-LABEL: define {{[^@]+}}@multiple_blocks_functions_non_kernel_effects_1
; MODULE-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]], ptr [[P:%.*]]) {
; MODULE-NEXT:    call void @barrier_then_write_then_barrier1(ptr [[P]])
; MODULE-NEXT:    br i1 [[C0]], label [[T03:%.*]], label [[F03:%.*]]
; MODULE:       t03:
; MODULE-NEXT:    call void @barrier_then_write1(ptr [[P]])
; MODULE-NEXT:    br label [[T0B3:%.*]]
; MODULE:       t0b3:
; MODULE-NEXT:    call void @aligned_barrier()
; MODULE-NEXT:    br label [[M3:%.*]]
; MODULE:       f03:
; MODULE-NEXT:    call void @barrier_then_write1(ptr [[P]])
; MODULE-NEXT:    br i1 [[C1]], label [[T13:%.*]], label [[F13:%.*]]
; MODULE:       t13:
; MODULE-NEXT:    call void @aligned_barrier()
; MODULE-NEXT:    br label [[M3]]
; MODULE:       f13:
; MODULE-NEXT:    call void @aligned_barrier()
; MODULE-NEXT:    br label [[M3]]
; MODULE:       m3:
; MODULE-NEXT:    call void @write_then_barrier1(ptr [[P]])
; MODULE-NEXT:    ret void
;
; CGSCC-LABEL: define {{[^@]+}}@multiple_blocks_functions_non_kernel_effects_1
; CGSCC-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]], ptr [[P:%.*]]) {
; CGSCC-NEXT:    call void @barrier_then_write_then_barrier1(ptr [[P]])
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br i1 [[C0]], label [[T03:%.*]], label [[F03:%.*]]
; CGSCC:       t03:
; CGSCC-NEXT:    call void @barrier_then_write1(ptr [[P]])
; CGSCC-NEXT:    br label [[T0B3:%.*]]
; CGSCC:       t0b3:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br label [[M3:%.*]]
; CGSCC:       f03:
; CGSCC-NEXT:    call void @barrier_then_write1(ptr [[P]])
; CGSCC-NEXT:    br i1 [[C1]], label [[T13:%.*]], label [[F13:%.*]]
; CGSCC:       t13:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br label [[M3]]
; CGSCC:       f13:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br label [[M3]]
; CGSCC:       m3:
; CGSCC-NEXT:    call void @write_then_barrier1(ptr [[P]])
; CGSCC-NEXT:    ret void
;
  call void @barrier_then_write_then_barrier1(i32* %p)
  call void @aligned_barrier()
  br i1 %c0, label %t03, label %f03
t03:
  call void @barrier_then_write1(i32* %p)
  br label %t0b3
t0b3:
  call void @aligned_barrier()
  br label %m3
f03:
  call void @aligned_barrier()
  call void @barrier_then_write1(i32* %p)
  br i1 %c1, label %t13, label %f13
t13:
  call void @aligned_barrier()
  br label %m3
f13:
  call void @aligned_barrier()
  br label %m3
m3:
  call void @aligned_barrier()
  call void @write_then_barrier1(i32* %p)
  ret void
}

define internal void @write_then_barrier2(i32* %p) {
; CHECK-LABEL: define {{[^@]+}}@write_then_barrier2
; CHECK-SAME: (ptr [[P:%.*]]) {
; CHECK-NEXT:    store i32 0, ptr [[P]], align 4
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    ret void
;
  store i32 0, i32* %p
  call void @aligned_barrier()
  ret void
}
define internal void @barrier_then_write2(i32* %p) {
; CHECK-LABEL: define {{[^@]+}}@barrier_then_write2
; CHECK-SAME: (ptr [[P:%.*]]) {
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    store i32 0, ptr [[P]], align 4
; CHECK-NEXT:    ret void
;
  call void @aligned_barrier()
  store i32 0, i32* %p
  ret void
}
define internal void @barrier_then_write_then_barrier2(i32* %p) {
; MODULE-LABEL: define {{[^@]+}}@barrier_then_write_then_barrier2
; MODULE-SAME: (ptr [[P:%.*]]) {
; MODULE-NEXT:    store i32 0, ptr [[P]], align 4
; MODULE-NEXT:    call void @aligned_barrier()
; MODULE-NEXT:    ret void
;
; CGSCC-LABEL: define {{[^@]+}}@barrier_then_write_then_barrier2
; CGSCC-SAME: (ptr [[P:%.*]]) {
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    store i32 0, ptr [[P]], align 4
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    ret void
;
  call void @aligned_barrier()
  store i32 0, i32* %p
  call void @aligned_barrier()
  ret void
}
define void @multiple_blocks_functions_non_kernel_effects_2(i1 %c0, i1 %c1, i32* %p) "kernel" {
; MODULE-LABEL: define {{[^@]+}}@multiple_blocks_functions_non_kernel_effects_2
; MODULE-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]], ptr [[P:%.*]]) #[[ATTR4]] {
; MODULE-NEXT:    call void @barrier_then_write_then_barrier2(ptr [[P]])
; MODULE-NEXT:    store i32 0, ptr [[P]], align 4
; MODULE-NEXT:    br i1 [[C0]], label [[T03:%.*]], label [[F03:%.*]]
; MODULE:       t03:
; MODULE-NEXT:    call void @barrier_then_write2(ptr [[P]])
; MODULE-NEXT:    br label [[T0B3:%.*]]
; MODULE:       t0b3:
; MODULE-NEXT:    call void @aligned_barrier()
; MODULE-NEXT:    br label [[M3:%.*]]
; MODULE:       f03:
; MODULE-NEXT:    call void @aligned_barrier()
; MODULE-NEXT:    call void @barrier_then_write2(ptr [[P]])
; MODULE-NEXT:    br i1 [[C1]], label [[T13:%.*]], label [[F13:%.*]]
; MODULE:       t13:
; MODULE-NEXT:    call void @aligned_barrier()
; MODULE-NEXT:    br label [[M3]]
; MODULE:       f13:
; MODULE-NEXT:    call void @aligned_barrier()
; MODULE-NEXT:    br label [[M3]]
; MODULE:       m3:
; MODULE-NEXT:    call void @write_then_barrier2(ptr [[P]])
; MODULE-NEXT:    store i32 0, ptr [[P]], align 4
; MODULE-NEXT:    ret void
;
; CGSCC-LABEL: define {{[^@]+}}@multiple_blocks_functions_non_kernel_effects_2
; CGSCC-SAME: (i1 [[C0:%.*]], i1 [[C1:%.*]], ptr [[P:%.*]]) #[[ATTR4]] {
; CGSCC-NEXT:    call void @barrier_then_write_then_barrier2(ptr [[P]])
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    store i32 0, ptr [[P]], align 4
; CGSCC-NEXT:    br i1 [[C0]], label [[T03:%.*]], label [[F03:%.*]]
; CGSCC:       t03:
; CGSCC-NEXT:    call void @barrier_then_write2(ptr [[P]])
; CGSCC-NEXT:    br label [[T0B3:%.*]]
; CGSCC:       t0b3:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br label [[M3:%.*]]
; CGSCC:       f03:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    call void @barrier_then_write2(ptr [[P]])
; CGSCC-NEXT:    br i1 [[C1]], label [[T13:%.*]], label [[F13:%.*]]
; CGSCC:       t13:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br label [[M3]]
; CGSCC:       f13:
; CGSCC-NEXT:    call void @aligned_barrier()
; CGSCC-NEXT:    br label [[M3]]
; CGSCC:       m3:
; CGSCC-NEXT:    call void @write_then_barrier2(ptr [[P]])
; CGSCC-NEXT:    store i32 0, ptr [[P]], align 4
; CGSCC-NEXT:    ret void
;
  call void @barrier_then_write_then_barrier2(i32* %p)
  call void @aligned_barrier()
  store i32 0, i32* %p
  br i1 %c0, label %t03, label %f03
t03:
  call void @barrier_then_write2(i32* %p)
  br label %t0b3
t0b3:
  call void @aligned_barrier()
  br label %m3
f03:
  call void @aligned_barrier()
  call void @barrier_then_write2(i32* %p)
  br i1 %c1, label %t13, label %f13
t13:
  call void @aligned_barrier()
  br label %m3
f13:
  call void @aligned_barrier()
  br label %m3
m3:
  call void @aligned_barrier()
  call void @write_then_barrier2(i32* %p)
  store i32 0, i32* %p
  ret void
}

; Verify we do not remove the barrier in the callee.
define internal void @callee_barrier() {
; CHECK-LABEL: define {{[^@]+}}@callee_barrier() {
; CHECK-NEXT:    call void @aligned_barrier()
; CHECK-NEXT:    ret void
;
  call void @aligned_barrier()
  ret void
}
define void @caller_barrier1() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@caller_barrier1
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    call void @callee_barrier()
; CHECK-NEXT:    ret void
;
  call void @aligned_barrier()
  call void @callee_barrier()
  call void @aligned_barrier()
  ret void
}
define void @caller_barrier2() "kernel" {
; CHECK-LABEL: define {{[^@]+}}@caller_barrier2
; CHECK-SAME: () #[[ATTR4]] {
; CHECK-NEXT:    call void @unknown()
; CHECK-NEXT:    call void @callee_barrier()
; CHECK-NEXT:    call void @unknown()
; CHECK-NEXT:    ret void
;
  call void @unknown()
  call void @callee_barrier()
  call void @unknown()
  ret void
}

!llvm.module.flags = !{!16,!15}
!nvvm.annotations = !{!0,!1,!2,!3,!4,!5,!6,!7,!8,!9,!10,!11,!12,!13,!14,!17,!18,!19,!20,!21,!22,!23,!24,!25}

!0 = !{void ()* @pos_empty_1, !"kernel", i32 1}
!1 = !{void ()* @pos_empty_2, !"kernel", i32 1}
!2 = !{void ()* @pos_empty_3, !"kernel", i32 1}
!3 = !{void ()* @pos_empty_4, !"kernel", i32 1}
!4 = !{void ()* @pos_empty_5, !"kernel", i32 1}
!5 = !{void ()* @pos_empty_6, !"kernel", i32 1}
!17 = !{void ()* @pos_empty_7a, !"kernel", i32 1}
!18 = !{void ()* @pos_empty_7b, !"kernel", i32 1}
!23 = !{void (i1)* @pos_empty_8, !"kernel", i32 1}
!24 = !{void ()* @caller_barrier1, !"kernel", i32 1}
!25 = !{void ()* @caller_barrier2, !"kernel", i32 1}
!6 = !{void ()* @neg_empty_8, !"kernel", i32 1}
!19 = !{void (i1)* @neg_empty_9, !"kernel", i32 1}
!20 = !{void ()* @pos_empty_10, !"kernel", i32 1}
!21 = !{void ()* @pos_empty_11, !"kernel", i32 1}
!22 = !{void (i1)* @neg_empty_12, !"kernel", i32 1}
!7 = !{void ()* @pos_constant_loads, !"kernel", i32 1}
!8 = !{void ()* @neg_loads, !"kernel", i32 1}
!9 = !{void ()* @pos_priv_mem, !"kernel", i32 1}
!10 = !{void ()* @neg_mem, !"kernel", i32 1}
!11 = !{void ()* @pos_multiple, !"kernel", i32 1}
!12 = !{void (i1,i1)* @multiple_blocks_kernel_1, !"kernel", i32 1}
!13 = !{void (i1,i1,i32*)* @multiple_blocks_kernel_2, !"kernel", i32 1}
!14 = !{void (i1,i1,i32*)* @multiple_blocks_functions_kernel_effects_0, !"kernel", i32 1}
!15 = !{i32 7, !"openmp", i32 50}
!16 = !{i32 7, !"openmp-device", i32 50}
;.
; CHECK: attributes #[[ATTR0]] = { "llvm.assume"="ompx_aligned_barrier" }
; CHECK: attributes #[[ATTR1:[0-9]+]] = { convergent nocallback nounwind }
; CHECK: attributes #[[ATTR2:[0-9]+]] = { convergent nocallback nofree nounwind willreturn }
; CHECK: attributes #[[ATTR3:[0-9]+]] = { nocallback nofree nosync nounwind willreturn memory(inaccessiblemem: readwrite) }
; CHECK: attributes #[[ATTR4]] = { "kernel" }
; CHECK: attributes #[[ATTR5]] = { nosync memory(none) }
;.
; CHECK: [[META0:![0-9]+]] = !{i32 7, !"openmp-device", i32 50}
; CHECK: [[META1:![0-9]+]] = !{i32 7, !"openmp", i32 50}
; CHECK: [[META2:![0-9]+]] = !{ptr @pos_empty_1, !"kernel", i32 1}
; CHECK: [[META3:![0-9]+]] = !{ptr @pos_empty_2, !"kernel", i32 1}
; CHECK: [[META4:![0-9]+]] = !{ptr @pos_empty_3, !"kernel", i32 1}
; CHECK: [[META5:![0-9]+]] = !{ptr @pos_empty_4, !"kernel", i32 1}
; CHECK: [[META6:![0-9]+]] = !{ptr @pos_empty_5, !"kernel", i32 1}
; CHECK: [[META7:![0-9]+]] = !{ptr @pos_empty_6, !"kernel", i32 1}
; CHECK: [[META8:![0-9]+]] = !{ptr @neg_empty_8, !"kernel", i32 1}
; CHECK: [[META9:![0-9]+]] = !{ptr @pos_constant_loads, !"kernel", i32 1}
; CHECK: [[META10:![0-9]+]] = !{ptr @neg_loads, !"kernel", i32 1}
; CHECK: [[META11:![0-9]+]] = !{ptr @pos_priv_mem, !"kernel", i32 1}
; CHECK: [[META12:![0-9]+]] = !{ptr @neg_mem, !"kernel", i32 1}
; CHECK: [[META13:![0-9]+]] = !{ptr @pos_multiple, !"kernel", i32 1}
; CHECK: [[META14:![0-9]+]] = !{ptr @multiple_blocks_kernel_1, !"kernel", i32 1}
; CHECK: [[META15:![0-9]+]] = !{ptr @multiple_blocks_kernel_2, !"kernel", i32 1}
; CHECK: [[META16:![0-9]+]] = !{ptr @multiple_blocks_functions_kernel_effects_0, !"kernel", i32 1}
; CHECK: [[META17:![0-9]+]] = !{ptr @pos_empty_7a, !"kernel", i32 1}
; CHECK: [[META18:![0-9]+]] = !{ptr @pos_empty_7b, !"kernel", i32 1}
; CHECK: [[META19:![0-9]+]] = !{ptr @neg_empty_9, !"kernel", i32 1}
; CHECK: [[META20:![0-9]+]] = !{ptr @pos_empty_10, !"kernel", i32 1}
; CHECK: [[META21:![0-9]+]] = !{ptr @pos_empty_11, !"kernel", i32 1}
; CHECK: [[META22:![0-9]+]] = !{ptr @neg_empty_12, !"kernel", i32 1}
; CHECK: [[META23:![0-9]+]] = !{ptr @pos_empty_8, !"kernel", i32 1}
; CHECK: [[META24:![0-9]+]] = !{ptr @caller_barrier1, !"kernel", i32 1}
; CHECK: [[META25:![0-9]+]] = !{ptr @caller_barrier2, !"kernel", i32 1}
;.
