; Verify the emission of accelerator tables for nameTableKind: Apple
; REQUIRES: x86-registered-target

; RUN: llc -mtriple=x86_64-apple-darwin12 -filetype=obj < %S/Inputs/name-table-kind-apple-5.ll \
; RUN:   | llvm-readobj --sections - | FileCheck --check-prefix=DEBUG_NAMES %s

; RUN: llc -mtriple=x86_64-apple-darwin12 -filetype=obj < %S/Inputs/name-table-kind-apple-4.ll \
; RUN:   | llvm-readobj --sections - | FileCheck --check-prefix=APPLE %s

; APPLE-NOT: debug_names
; APPLE-NOT: debug{{.*}}pub
; APPLE: apple_names
; APPLE-NOT: debug_names
; APPLE-NOT: debug{{.*}}pub

; DEBUG_NAMES-NOT: apple_names
; DEBUG_NAMES-NOT: pubnames
; DEBUG_NAMES: debug_names
; DEBUG_NAMES-NOT: apple_names
; DEBUG_NAMES-NOT: pubnames

@var = thread_local global i32 0, align 4, !dbg !0

; Function Attrs: norecurse nounwind readnone uwtable
define void @_Z3funv() local_unnamed_addr #0 !dbg !11 {
  ret void, !dbg !14
}

; Function Attrs: norecurse uwtable
define weak_odr hidden ptr @_ZTW3var() local_unnamed_addr #1 {
  ret ptr @var
}

attributes #0 = { norecurse nounwind readnone uwtable }
attributes #1 = { norecurse uwtable }

!llvm.dbg.cu = !{!2}
!llvm.module.flags = !{!7, !8, !9}
!llvm.ident = !{!10}

!0 = !DIGlobalVariableExpression(var: !1, expr: !DIExpression())
!1 = distinct !DIGlobalVariable(name: "var", scope: !2, file: !3, line: 1, type: !6, isLocal: false, isDefinition: true)
!2 = distinct !DICompileUnit(language: DW_LANG_C_plus_plus, file: !3, producer: "clang version 7.0.0 (trunk 322268) (llvm/trunk 322267)", isOptimized: true, runtimeVersion: 0, emissionKind: FullDebug, enums: !4, globals: !5, nameTableKind: Apple)
!3 = !DIFile(filename: "debugger-tune.cpp", directory: "/tmp")
!4 = !{}
!5 = !{!0}
!6 = !DIBasicType(name: "int", size: 32, encoding: DW_ATE_signed)
!7 = !{i32 2, !"Dwarf Version", i32 4}
!8 = !{i32 2, !"Debug Info Version", i32 3}
!9 = !{i32 1, !"wchar_size", i32 4}
!10 = !{!"clang version 7.0.0 (trunk 322268) (llvm/trunk 322267)"}
!11 = distinct !DISubprogram(name: "fun", linkageName: "_Z3funv", scope: !3, file: !3, line: 2, type: !12, isLocal: false, isDefinition: true, scopeLine: 2, flags: DIFlagPrototyped, isOptimized: true, unit: !2, retainedNodes: !4)
!12 = !DISubroutineType(types: !13)
!13 = !{null}
!14 = !DILocation(line: 2, column: 13, scope: !11)

