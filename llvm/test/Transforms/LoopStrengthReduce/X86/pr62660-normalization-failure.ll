; NOTE: Assertions have been autogenerated by utils/update_test_checks.py UTC_ARGS: --version 2
; RUN: opt -loop-reduce -verify-scev -S %s | FileCheck %s

; XFAIL: *
; REQUIRES: asserts

target datalayout = "e-m:e-p270:32:32-p271:32:32-p272:64:64-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-apple-macos"

define i64 @test_pr62660() {
; CHECK-LABEL: define i64 @test_pr62660() {
; CHECK-NEXT:  entry:
; CHECK-NEXT:    br label [[LOOP:%.*]]
; CHECK:       loop:
; CHECK-NEXT:    [[LSR_IV1:%.*]] = phi i32 [ [[LSR_IV_NEXT2:%.*]], [[LOOP]] ], [ 65533, [[ENTRY:%.*]] ]
; CHECK-NEXT:    [[LSR_IV:%.*]] = phi i64 [ [[LSR_IV_NEXT:%.*]], [[LOOP]] ], [ -1, [[ENTRY]] ]
; CHECK-NEXT:    [[LSR_IV_NEXT]] = add nsw i64 [[LSR_IV]], 1
; CHECK-NEXT:    [[LSR_IV_NEXT2]] = add nuw nsw i32 [[LSR_IV1]], 2
; CHECK-NEXT:    [[CMP:%.*]] = icmp sgt i32 [[LSR_IV_NEXT2]], 8
; CHECK-NEXT:    br i1 [[CMP]], label [[LOOP]], label [[EXIT:%.*]]
; CHECK:       exit:
; CHECK-NEXT:    ret i64 [[LSR_IV_NEXT]]
;
entry:
  br label %loop

loop:
  %iv = phi i32 [ 0, %entry ], [ %iv.next, %loop ]
  %conv1 = and i32 %iv, 65535
  %add = add nsw i32 %iv, -1
  %sub = add i32 %add, %conv1
  %cmp = icmp sgt i32 %sub, 8
  %iv.next = add nuw nsw i32 %iv, 1
  br i1 %cmp, label %loop, label %exit

exit:
  %conv5 = zext i32 %iv to i64
  ret i64 %conv5
}
