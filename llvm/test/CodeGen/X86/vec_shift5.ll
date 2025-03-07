; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc < %s -mtriple=i686-unknown -mattr=+sse2 | FileCheck %s --check-prefixes=CHECK,X86
; RUN: llc < %s -mtriple=x86_64-unknown -mattr=+sse2 | FileCheck %s --check-prefixes=CHECK,X64

; Verify that we correctly fold target specific packed vector shifts by
; immediate count into a simple build_vector when the elements of the vector
; in input to the packed shift are all constants or undef.

define <8 x i16> @test1() {
; CHECK-LABEL: test1:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [8,16,32,64,8,16,32,64]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <8 x i16> @llvm.x86.sse2.pslli.w(<8 x i16> <i16 1, i16 2, i16 4, i16 8, i16 1, i16 2, i16 4, i16 8>, i32 3)
  ret <8 x i16> %1
}

define <8 x i16> @test2() {
; CHECK-LABEL: test2:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [0,1,2,4,0,1,2,4]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <8 x i16> @llvm.x86.sse2.psrli.w(<8 x i16> <i16 4, i16 8, i16 16, i16 32, i16 4, i16 8, i16 16, i16 32>, i32 3)
  ret <8 x i16> %1
}

define <8 x i16> @test3() {
; CHECK-LABEL: test3:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [0,1,2,4,0,1,2,4]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <8 x i16> @llvm.x86.sse2.psrai.w(<8 x i16> <i16 4, i16 8, i16 16, i16 32, i16 4, i16 8, i16 16, i16 32>, i32 3)
  ret <8 x i16> %1
}

define <4 x i32> @test4() {
; CHECK-LABEL: test4:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [8,16,32,64]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32> <i32 1, i32 2, i32 4, i32 8>, i32 3)
  ret <4 x i32> %1
}

define <4 x i32> @test5() {
; CHECK-LABEL: test5:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [0,1,2,4]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <4 x i32> @llvm.x86.sse2.psrli.d(<4 x i32> <i32 4, i32 8, i32 16, i32 32>, i32 3)
  ret <4 x i32> %1
}

define <4 x i32> @test6() {
; CHECK-LABEL: test6:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [0,1,2,4]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <4 x i32> @llvm.x86.sse2.psrai.d(<4 x i32> <i32 4, i32 8, i32 16, i32 32>, i32 3)
  ret <4 x i32> %1
}

define <2 x i64> @test7() {
; X86-LABEL: test7:
; X86:       # %bb.0:
; X86-NEXT:    movaps {{.*#+}} xmm0 = [8,0,16,0]
; X86-NEXT:    retl
;
; X64-LABEL: test7:
; X64:       # %bb.0:
; X64-NEXT:    movaps {{.*#+}} xmm0 = [8,16]
; X64-NEXT:    retq
  %1 = tail call <2 x i64> @llvm.x86.sse2.pslli.q(<2 x i64> <i64 1, i64 2>, i32 3)
  ret <2 x i64> %1
}

define <2 x i64> @test8() {
; X86-LABEL: test8:
; X86:       # %bb.0:
; X86-NEXT:    movaps {{.*#+}} xmm0 = [1,0,2,0]
; X86-NEXT:    retl
;
; X64-LABEL: test8:
; X64:       # %bb.0:
; X64-NEXT:    movaps {{.*#+}} xmm0 = [1,2]
; X64-NEXT:    retq
  %1 = tail call <2 x i64> @llvm.x86.sse2.psrli.q(<2 x i64> <i64 8, i64 16>, i32 3)
  ret <2 x i64> %1
}

define <8 x i16> @test9() {
; CHECK-LABEL: test9:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [1,1,0,0,3,0,8,16]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <8 x i16> @llvm.x86.sse2.psrai.w(<8 x i16> <i16 15, i16 8, i16 undef, i16 undef, i16 31, i16 undef, i16 64, i16 128>, i32 3)
  ret <8 x i16> %1
}

define <4 x i32> @test10() {
; CHECK-LABEL: test10:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [0,1,0,4]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <4 x i32> @llvm.x86.sse2.psrai.d(<4 x i32> <i32 undef, i32 8, i32 undef, i32 32>, i32 3)
  ret <4 x i32> %1
}

define <2 x i64> @test11() {
; X86-LABEL: test11:
; X86:       # %bb.0:
; X86-NEXT:    movaps {{.*#+}} xmm0 = [0,0,3,0]
; X86-NEXT:    retl
;
; X64-LABEL: test11:
; X64:       # %bb.0:
; X64-NEXT:    movaps {{.*#+}} xmm0 = [0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0]
; X64-NEXT:    retq
  %1 = tail call <2 x i64> @llvm.x86.sse2.psrli.q(<2 x i64> <i64 undef, i64 31>, i32 3)
  ret <2 x i64> %1
}

define <8 x i16> @test12() {
; CHECK-LABEL: test12:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [1,1,0,0,3,0,8,16]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <8 x i16> @llvm.x86.sse2.psrai.w(<8 x i16> <i16 15, i16 8, i16 undef, i16 undef, i16 31, i16 undef, i16 64, i16 128>, i32 3)
  ret <8 x i16> %1
}

define <4 x i32> @test13() {
; CHECK-LABEL: test13:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [0,1,0,4]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <4 x i32> @llvm.x86.sse2.psrli.d(<4 x i32> <i32 undef, i32 8, i32 undef, i32 32>, i32 3)
  ret <4 x i32> %1
}

define <8 x i16> @test14() {
; CHECK-LABEL: test14:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [1,1,0,0,3,0,8,16]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <8 x i16> @llvm.x86.sse2.psrli.w(<8 x i16> <i16 15, i16 8, i16 undef, i16 undef, i16 31, i16 undef, i16 64, i16 128>, i32 3)
  ret <8 x i16> %1
}

define <4 x i32> @test15() {
; CHECK-LABEL: test15:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movaps {{.*#+}} xmm0 = [0,64,0,256]
; CHECK-NEXT:    ret{{[l|q]}}
  %1 = tail call <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32> <i32 undef, i32 8, i32 undef, i32 32>, i32 3)
  ret <4 x i32> %1
}

define <2 x i64> @test16() {
; X86-LABEL: test16:
; X86:       # %bb.0:
; X86-NEXT:    movaps {{.*#+}} xmm0 = [0,0,248,0]
; X86-NEXT:    retl
;
; X64-LABEL: test16:
; X64:       # %bb.0:
; X64-NEXT:    movaps {{.*#+}} xmm0 = [0,0,0,0,0,0,0,0,248,0,0,0,0,0,0,0]
; X64-NEXT:    retq
  %1 = tail call <2 x i64> @llvm.x86.sse2.pslli.q(<2 x i64> <i64 undef, i64 31>, i32 3)
  ret <2 x i64> %1
}

; Make sure we fold fully undef input vectors. We previously folded only when
; undef had a single use so use 2 undefs.
define <4 x i32> @test17(<4 x i32> %a0, <4 x i32>* %dummy) {
; X86-LABEL: test17:
; X86:       # %bb.0:
; X86-NEXT:    movl {{[0-9]+}}(%esp), %eax
; X86-NEXT:    xorps %xmm0, %xmm0
; X86-NEXT:    movaps %xmm0, (%eax)
; X86-NEXT:    retl
;
; X64-LABEL: test17:
; X64:       # %bb.0:
; X64-NEXT:    xorps %xmm0, %xmm0
; X64-NEXT:    movaps %xmm0, (%rdi)
; X64-NEXT:    retq
  %a = call <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32> undef, i32 6)
  store <4 x i32> %a, <4 x i32>* %dummy
  %res = call <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32> undef, i32 7)
  ret <4 x i32> %res
}

define <4 x i32> @test18(<4 x i32> %a0, <4 x i32>* %dummy) {
; X86-LABEL: test18:
; X86:       # %bb.0:
; X86-NEXT:    movl {{[0-9]+}}(%esp), %eax
; X86-NEXT:    xorps %xmm0, %xmm0
; X86-NEXT:    movaps %xmm0, (%eax)
; X86-NEXT:    retl
;
; X64-LABEL: test18:
; X64:       # %bb.0:
; X64-NEXT:    xorps %xmm0, %xmm0
; X64-NEXT:    movaps %xmm0, (%rdi)
; X64-NEXT:    retq
  %a = call <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32> undef, i32 3)
  store <4 x i32> %a, <4 x i32>* %dummy
  %res = call <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32> undef, i32 1)
  ret <4 x i32> %res
}

; PR39482

define <4 x i32> @extelt0_sub_pslli_v4i32(<4 x i32> %x, <4 x i32> %y){
; CHECK-LABEL: extelt0_sub_pslli_v4i32:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movdqa {{.*#+}} xmm2 = [32,32,32,32]
; CHECK-NEXT:    psubd %xmm1, %xmm2
; CHECK-NEXT:    pxor %xmm1, %xmm1
; CHECK-NEXT:    movss {{.*#+}} xmm1 = xmm2[0],xmm1[1,2,3]
; CHECK-NEXT:    pslld %xmm1, %xmm0
; CHECK-NEXT:    ret{{[l|q]}}
  %ext = extractelement <4 x i32> %y, i64 0
  %bo = sub i32 32, %ext
  %r = tail call <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32> %x, i32 %bo)
  ret <4 x i32> %r
}

define <4 x i32> @extelt1_add_psrli_v4i32(<4 x i32> %x, <4 x i32> %y){
; X86-LABEL: extelt1_add_psrli_v4i32:
; X86:       # %bb.0:
; X86-NEXT:    pshufd {{.*#+}} xmm1 = xmm1[1,1,1,1]
; X86-NEXT:    paddd {{\.?LCPI[0-9]+_[0-9]+}}, %xmm1
; X86-NEXT:    xorps %xmm2, %xmm2
; X86-NEXT:    movss {{.*#+}} xmm2 = xmm1[0],xmm2[1,2,3]
; X86-NEXT:    psrld %xmm2, %xmm0
; X86-NEXT:    retl
;
; X64-LABEL: extelt1_add_psrli_v4i32:
; X64:       # %bb.0:
; X64-NEXT:    pshufd {{.*#+}} xmm1 = xmm1[1,1,1,1]
; X64-NEXT:    paddd {{\.?LCPI[0-9]+_[0-9]+}}(%rip), %xmm1
; X64-NEXT:    xorps %xmm2, %xmm2
; X64-NEXT:    movss {{.*#+}} xmm2 = xmm1[0],xmm2[1,2,3]
; X64-NEXT:    psrld %xmm2, %xmm0
; X64-NEXT:    retq
  %ext = extractelement <4 x i32> %y, i64 1
  %bo = add i32 %ext, 3
  %r = tail call <4 x i32> @llvm.x86.sse2.psrli.d(<4 x i32> %x, i32 %bo)
  ret <4 x i32> %r
}

define i32 @extelt1_add_psrai_v4i32_uses(<4 x i32> %x, <4 x i32> %y){
; CHECK-LABEL: extelt1_add_psrai_v4i32_uses:
; CHECK:       # %bb.0:
; CHECK-NEXT:    pshufd {{.*#+}} xmm1 = xmm1[1,1,1,1]
; CHECK-NEXT:    movd %xmm1, %ecx
; CHECK-NEXT:    addl $3, %ecx
; CHECK-NEXT:    movd %ecx, %xmm1
; CHECK-NEXT:    pshufd {{.*#+}} xmm0 = xmm0[3,3,3,3]
; CHECK-NEXT:    psrad %xmm1, %xmm0
; CHECK-NEXT:    movd %xmm0, %eax
; CHECK-NEXT:    imull %ecx, %eax
; CHECK-NEXT:    ret{{[l|q]}}
  %ext = extractelement <4 x i32> %y, i64 1
  %bo = add i32 %ext, 3
  %r = tail call <4 x i32> @llvm.x86.sse2.psrai.d(<4 x i32> %x, i32 %bo)
  %ext3 = extractelement <4 x i32> %r, i64 3
  %r2 = mul i32 %bo, %ext3
  ret i32 %r2
}

define <4 x i32> @extelt0_twice_sub_pslli_v4i32(<4 x i32> %x, <4 x i32> %y, <4 x i32> %z){
; CHECK-LABEL: extelt0_twice_sub_pslli_v4i32:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movd %xmm1, %eax
; CHECK-NEXT:    movd %xmm2, %ecx
; CHECK-NEXT:    subl %ecx, %eax
; CHECK-NEXT:    movd %eax, %xmm1
; CHECK-NEXT:    pslld %xmm1, %xmm0
; CHECK-NEXT:    ret{{[l|q]}}
  %ext1 = extractelement <4 x i32> %y, i64 0
  %ext2 = extractelement <4 x i32> %z, i64 0
  %bo = sub i32 %ext1, %ext2
  %r = tail call <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32> %x, i32 %bo)
  ret <4 x i32> %r
}

; This would crash because the scalar shift amount has a different type than the shift result.

define <2 x i8> @PR58661(<2 x i8> %a0) {
; CHECK-LABEL: PR58661:
; CHECK:       # %bb.0:
; CHECK-NEXT:    psrlw $8, %xmm0
; CHECK-NEXT:    movd %xmm0, %eax
; CHECK-NEXT:    shll $8, %eax
; CHECK-NEXT:    movd %eax, %xmm0
; CHECK-NEXT:    ret{{[l|q]}}
  %shuffle = shufflevector <2 x i8> %a0, <2 x i8> <i8 poison, i8 0>, <2 x i32> <i32 1, i32 3>
  %x = bitcast <2 x i8> %shuffle to i16
  %shl = shl nuw i16 %x, 8
  %y = bitcast i16 %shl to <2 x i8>
  ret <2 x i8> %y
}

declare <8 x i16> @llvm.x86.sse2.pslli.w(<8 x i16>, i32)
declare <8 x i16> @llvm.x86.sse2.psrli.w(<8 x i16>, i32)
declare <8 x i16> @llvm.x86.sse2.psrai.w(<8 x i16>, i32)
declare <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32>, i32)
declare <4 x i32> @llvm.x86.sse2.psrli.d(<4 x i32>, i32)
declare <4 x i32> @llvm.x86.sse2.psrai.d(<4 x i32>, i32)
declare <2 x i64> @llvm.x86.sse2.pslli.q(<2 x i64>, i32)
declare <2 x i64> @llvm.x86.sse2.psrli.q(<2 x i64>, i32)
