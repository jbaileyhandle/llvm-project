; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; REQUIRES: asserts
; RUN: llc < %s -mtriple=x86_64-linux -mcpu=corei7 -relocation-model=static | FileCheck %s

; The register-pressure scheduler should be able to schedule this in a
; way that does not require spills.

@X = external dso_local global i64		; <i64*> [#uses=25]

define fastcc i64 @foo() nounwind {
; CHECK-LABEL: foo:
; CHECK:       # %bb.0:
; CHECK-NEXT:    pushq %r14
; CHECK-NEXT:    pushq %rbx
; CHECK-NEXT:    movq X(%rip), %rcx
; CHECK-NEXT:    movq X(%rip), %r9
; CHECK-NEXT:    movq X(%rip), %r8
; CHECK-NEXT:    movq X(%rip), %rdi
; CHECK-NEXT:    movq X(%rip), %rsi
; CHECK-NEXT:    movq X(%rip), %rdx
; CHECK-NEXT:    movq X(%rip), %rbx
; CHECK-NEXT:    movq X(%rip), %rax
; CHECK-NEXT:    addq %rsi, %rax
; CHECK-NEXT:    movq X(%rip), %r10
; CHECK-NEXT:    movq X(%rip), %r11
; CHECK-NEXT:    bswapq %r10
; CHECK-NEXT:    leaq (%rbx,%rdx), %r14
; CHECK-NEXT:    addq %rsi, %r14
; CHECK-NEXT:    addq %rax, %r14
; CHECK-NEXT:    addq %r10, %r14
; CHECK-NEXT:    leaq (%r9,%rcx), %rax
; CHECK-NEXT:    leaq (%rax,%r8), %r10
; CHECK-NEXT:    addq %r14, %rdi
; CHECK-NEXT:    addq %r10, %r10
; CHECK-NEXT:    bswapq %r11
; CHECK-NEXT:    addq %r14, %r10
; CHECK-NEXT:    addq %rbx, %r11
; CHECK-NEXT:    leaq (%rsi,%rdx), %rbx
; CHECK-NEXT:    addq %rdi, %rbx
; CHECK-NEXT:    addq %rdi, %r11
; CHECK-NEXT:    addq %rbx, %r11
; CHECK-NEXT:    addq %rax, %rax
; CHECK-NEXT:    addq %r10, %rax
; CHECK-NEXT:    addq %r11, %r8
; CHECK-NEXT:    addq %r10, %rax
; CHECK-NEXT:    addq %r11, %rax
; CHECK-NEXT:    movq X(%rip), %r11
; CHECK-NEXT:    bswapq %r11
; CHECK-NEXT:    addq %rdx, %r11
; CHECK-NEXT:    leaq (%rdi,%rsi), %rdx
; CHECK-NEXT:    addq %r8, %rdx
; CHECK-NEXT:    addq %r8, %r11
; CHECK-NEXT:    addq %rdx, %r11
; CHECK-NEXT:    leaq (%r10,%rcx), %rdx
; CHECK-NEXT:    addq %rdx, %rdx
; CHECK-NEXT:    addq %rax, %rdx
; CHECK-NEXT:    addq %r11, %r9
; CHECK-NEXT:    addq %rax, %rdx
; CHECK-NEXT:    addq %r11, %rdx
; CHECK-NEXT:    movq X(%rip), %r11
; CHECK-NEXT:    bswapq %r11
; CHECK-NEXT:    addq %rsi, %r11
; CHECK-NEXT:    leaq (%r8,%rdi), %rsi
; CHECK-NEXT:    addq %r9, %rsi
; CHECK-NEXT:    addq %r9, %r11
; CHECK-NEXT:    addq %rsi, %r11
; CHECK-NEXT:    leaq (%rax,%r10), %rsi
; CHECK-NEXT:    addq %rsi, %rsi
; CHECK-NEXT:    addq %rdx, %rsi
; CHECK-NEXT:    addq %r11, %rcx
; CHECK-NEXT:    addq %rdx, %rsi
; CHECK-NEXT:    addq %r11, %rsi
; CHECK-NEXT:    movq X(%rip), %r11
; CHECK-NEXT:    bswapq %r11
; CHECK-NEXT:    addq %rdi, %r11
; CHECK-NEXT:    leaq (%r9,%r8), %rdi
; CHECK-NEXT:    addq %rcx, %rdi
; CHECK-NEXT:    addq %rcx, %r11
; CHECK-NEXT:    addq %rdi, %r11
; CHECK-NEXT:    leaq (%rdx,%rax), %rdi
; CHECK-NEXT:    addq %rdi, %rdi
; CHECK-NEXT:    addq %rsi, %rdi
; CHECK-NEXT:    addq %r11, %r10
; CHECK-NEXT:    addq %rsi, %rdi
; CHECK-NEXT:    addq %r11, %rdi
; CHECK-NEXT:    movq X(%rip), %r11
; CHECK-NEXT:    bswapq %r11
; CHECK-NEXT:    addq %r8, %r11
; CHECK-NEXT:    leaq (%rcx,%r9), %r8
; CHECK-NEXT:    addq %r10, %r8
; CHECK-NEXT:    addq %r10, %r11
; CHECK-NEXT:    addq %r8, %r11
; CHECK-NEXT:    leaq (%rsi,%rdx), %r8
; CHECK-NEXT:    addq %r8, %r8
; CHECK-NEXT:    addq %rdi, %r8
; CHECK-NEXT:    addq %r11, %rax
; CHECK-NEXT:    addq %rdi, %r8
; CHECK-NEXT:    addq %r11, %r8
; CHECK-NEXT:    movq X(%rip), %r11
; CHECK-NEXT:    bswapq %r11
; CHECK-NEXT:    addq %r9, %r11
; CHECK-NEXT:    leaq (%r10,%rcx), %r9
; CHECK-NEXT:    addq %rax, %r9
; CHECK-NEXT:    addq %rax, %r11
; CHECK-NEXT:    addq %r9, %r11
; CHECK-NEXT:    leaq (%rdi,%rsi), %r9
; CHECK-NEXT:    addq %r9, %r9
; CHECK-NEXT:    addq %r8, %r9
; CHECK-NEXT:    addq %r11, %rdx
; CHECK-NEXT:    addq %r8, %r9
; CHECK-NEXT:    addq %r11, %r9
; CHECK-NEXT:    movq X(%rip), %r11
; CHECK-NEXT:    bswapq %r11
; CHECK-NEXT:    addq %rcx, %r11
; CHECK-NEXT:    leaq (%rax,%r10), %rcx
; CHECK-NEXT:    addq %rdx, %rcx
; CHECK-NEXT:    addq %rdx, %r11
; CHECK-NEXT:    addq %rcx, %r11
; CHECK-NEXT:    leaq (%r8,%rdi), %rcx
; CHECK-NEXT:    addq %rcx, %rcx
; CHECK-NEXT:    addq %r9, %rcx
; CHECK-NEXT:    addq %r11, %rsi
; CHECK-NEXT:    addq %r9, %rcx
; CHECK-NEXT:    addq %r11, %rcx
; CHECK-NEXT:    movq X(%rip), %r11
; CHECK-NEXT:    bswapq %r11
; CHECK-NEXT:    addq %r10, %r11
; CHECK-NEXT:    leaq (%rdx,%rax), %r10
; CHECK-NEXT:    addq %rsi, %r10
; CHECK-NEXT:    addq %rsi, %r11
; CHECK-NEXT:    addq %r10, %r11
; CHECK-NEXT:    leaq (%r9,%r8), %r10
; CHECK-NEXT:    addq %r10, %r10
; CHECK-NEXT:    addq %rcx, %r10
; CHECK-NEXT:    addq %r11, %rdi
; CHECK-NEXT:    addq %rcx, %r10
; CHECK-NEXT:    addq %r11, %r10
; CHECK-NEXT:    movq X(%rip), %rbx
; CHECK-NEXT:    bswapq %rbx
; CHECK-NEXT:    addq %rax, %rbx
; CHECK-NEXT:    leaq (%rsi,%rdx), %rax
; CHECK-NEXT:    addq %rdi, %rax
; CHECK-NEXT:    addq %rdi, %rbx
; CHECK-NEXT:    addq %rax, %rbx
; CHECK-NEXT:    leaq (%rcx,%r9), %r11
; CHECK-NEXT:    addq %r11, %r11
; CHECK-NEXT:    addq %r10, %r11
; CHECK-NEXT:    addq %rbx, %r8
; CHECK-NEXT:    addq %r10, %r11
; CHECK-NEXT:    addq %rbx, %r11
; CHECK-NEXT:    movq X(%rip), %rax
; CHECK-NEXT:    bswapq %rax
; CHECK-NEXT:    addq %rdx, %rax
; CHECK-NEXT:    leaq (%rdi,%rsi), %rdx
; CHECK-NEXT:    addq %r8, %rdx
; CHECK-NEXT:    addq %r8, %rax
; CHECK-NEXT:    addq %rdx, %rax
; CHECK-NEXT:    leaq (%r10,%rcx), %rdx
; CHECK-NEXT:    addq %rdx, %rdx
; CHECK-NEXT:    addq %r11, %rdx
; CHECK-NEXT:    addq %rax, %r9
; CHECK-NEXT:    addq %r11, %rdx
; CHECK-NEXT:    addq %rax, %rdx
; CHECK-NEXT:    movq X(%rip), %rbx
; CHECK-NEXT:    bswapq %rbx
; CHECK-NEXT:    addq %rsi, %rbx
; CHECK-NEXT:    leaq (%r8,%rdi), %rax
; CHECK-NEXT:    addq %r9, %rax
; CHECK-NEXT:    addq %r9, %rbx
; CHECK-NEXT:    addq %rax, %rbx
; CHECK-NEXT:    leaq (%r11,%r10), %rax
; CHECK-NEXT:    addq %rax, %rax
; CHECK-NEXT:    addq %rdx, %rax
; CHECK-NEXT:    addq %rbx, %rcx
; CHECK-NEXT:    addq %rdx, %rax
; CHECK-NEXT:    addq %rbx, %rax
; CHECK-NEXT:    movq X(%rip), %rbx
; CHECK-NEXT:    bswapq %rbx
; CHECK-NEXT:    addq %rdi, %rbx
; CHECK-NEXT:    leaq (%r9,%r8), %rsi
; CHECK-NEXT:    addq %rcx, %rsi
; CHECK-NEXT:    addq %rcx, %rbx
; CHECK-NEXT:    addq %rsi, %rbx
; CHECK-NEXT:    leaq (%rdx,%r11), %rsi
; CHECK-NEXT:    addq %rsi, %rsi
; CHECK-NEXT:    addq %rax, %rsi
; CHECK-NEXT:    addq %rbx, %r10
; CHECK-NEXT:    addq %rax, %rsi
; CHECK-NEXT:    addq %rbx, %rsi
; CHECK-NEXT:    movq X(%rip), %rbx
; CHECK-NEXT:    bswapq %rbx
; CHECK-NEXT:    addq %r8, %rbx
; CHECK-NEXT:    leaq (%rcx,%r9), %rdi
; CHECK-NEXT:    addq %r10, %rdi
; CHECK-NEXT:    addq %r10, %rbx
; CHECK-NEXT:    addq %rdi, %rbx
; CHECK-NEXT:    leaq (%rax,%rdx), %rdi
; CHECK-NEXT:    addq %rdi, %rdi
; CHECK-NEXT:    addq %rsi, %rdi
; CHECK-NEXT:    addq %rbx, %r11
; CHECK-NEXT:    addq %rsi, %rdi
; CHECK-NEXT:    addq %rbx, %rdi
; CHECK-NEXT:    movq X(%rip), %r8
; CHECK-NEXT:    bswapq %r8
; CHECK-NEXT:    addq %r9, %r8
; CHECK-NEXT:    leaq (%r10,%rcx), %r9
; CHECK-NEXT:    addq %r11, %r9
; CHECK-NEXT:    addq %r11, %r8
; CHECK-NEXT:    addq %r9, %r8
; CHECK-NEXT:    addq %rax, %rsi
; CHECK-NEXT:    addq %rsi, %rsi
; CHECK-NEXT:    addq %rdi, %rsi
; CHECK-NEXT:    addq %rdi, %rsi
; CHECK-NEXT:    addq %r8, %rdx
; CHECK-NEXT:    addq %r8, %rsi
; CHECK-NEXT:    movq X(%rip), %rax
; CHECK-NEXT:    bswapq %rax
; CHECK-NEXT:    addq %r10, %r11
; CHECK-NEXT:    movq %rax, X(%rip)
; CHECK-NEXT:    addq %rcx, %rax
; CHECK-NEXT:    addq %rdx, %r11
; CHECK-NEXT:    addq %rdx, %rax
; CHECK-NEXT:    addq %r11, %rax
; CHECK-NEXT:    addq %rsi, %rax
; CHECK-NEXT:    popq %rbx
; CHECK-NEXT:    popq %r14
; CHECK-NEXT:    retq
	%tmp = load volatile i64, i64* @X		; <i64> [#uses=7]
	%tmp1 = load volatile i64, i64* @X		; <i64> [#uses=5]
	%tmp2 = load volatile i64, i64* @X		; <i64> [#uses=3]
	%tmp3 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp4 = load volatile i64, i64* @X		; <i64> [#uses=5]
	%tmp5 = load volatile i64, i64* @X		; <i64> [#uses=3]
	%tmp6 = load volatile i64, i64* @X		; <i64> [#uses=2]
	%tmp7 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp8 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp9 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp10 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp11 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp12 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp13 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp14 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp15 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp16 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp17 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp18 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp19 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp20 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp21 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp22 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp23 = load volatile i64, i64* @X		; <i64> [#uses=1]
	%tmp24 = call i64 @llvm.bswap.i64(i64 %tmp8)		; <i64> [#uses=1]
	%tmp25 = add i64 %tmp6, %tmp5		; <i64> [#uses=1]
	%tmp26 = add i64 %tmp25, %tmp4		; <i64> [#uses=1]
	%tmp27 = add i64 %tmp7, %tmp4		; <i64> [#uses=1]
	%tmp28 = add i64 %tmp27, %tmp26		; <i64> [#uses=1]
	%tmp29 = add i64 %tmp28, %tmp24		; <i64> [#uses=2]
	%tmp30 = add i64 %tmp2, %tmp1		; <i64> [#uses=1]
	%tmp31 = add i64 %tmp30, %tmp		; <i64> [#uses=1]
	%tmp32 = add i64 %tmp2, %tmp1		; <i64> [#uses=1]
	%tmp33 = add i64 %tmp31, %tmp32		; <i64> [#uses=1]
	%tmp34 = add i64 %tmp29, %tmp3		; <i64> [#uses=5]
	%tmp35 = add i64 %tmp33, %tmp		; <i64> [#uses=1]
	%tmp36 = add i64 %tmp35, %tmp29		; <i64> [#uses=7]
	%tmp37 = call i64 @llvm.bswap.i64(i64 %tmp9)		; <i64> [#uses=1]
	%tmp38 = add i64 %tmp4, %tmp5		; <i64> [#uses=1]
	%tmp39 = add i64 %tmp38, %tmp34		; <i64> [#uses=1]
	%tmp40 = add i64 %tmp6, %tmp37		; <i64> [#uses=1]
	%tmp41 = add i64 %tmp40, %tmp39		; <i64> [#uses=1]
	%tmp42 = add i64 %tmp41, %tmp34		; <i64> [#uses=2]
	%tmp43 = add i64 %tmp1, %tmp		; <i64> [#uses=1]
	%tmp44 = add i64 %tmp36, %tmp43		; <i64> [#uses=1]
	%tmp45 = add i64 %tmp1, %tmp		; <i64> [#uses=1]
	%tmp46 = add i64 %tmp44, %tmp45		; <i64> [#uses=1]
	%tmp47 = add i64 %tmp42, %tmp2		; <i64> [#uses=5]
	%tmp48 = add i64 %tmp36, %tmp46		; <i64> [#uses=1]
	%tmp49 = add i64 %tmp48, %tmp42		; <i64> [#uses=7]
	%tmp50 = call i64 @llvm.bswap.i64(i64 %tmp10)		; <i64> [#uses=1]
	%tmp51 = add i64 %tmp34, %tmp4		; <i64> [#uses=1]
	%tmp52 = add i64 %tmp51, %tmp47		; <i64> [#uses=1]
	%tmp53 = add i64 %tmp5, %tmp50		; <i64> [#uses=1]
	%tmp54 = add i64 %tmp53, %tmp52		; <i64> [#uses=1]
	%tmp55 = add i64 %tmp54, %tmp47		; <i64> [#uses=2]
	%tmp56 = add i64 %tmp36, %tmp		; <i64> [#uses=1]
	%tmp57 = add i64 %tmp49, %tmp56		; <i64> [#uses=1]
	%tmp58 = add i64 %tmp36, %tmp		; <i64> [#uses=1]
	%tmp59 = add i64 %tmp57, %tmp58		; <i64> [#uses=1]
	%tmp60 = add i64 %tmp55, %tmp1		; <i64> [#uses=5]
	%tmp61 = add i64 %tmp49, %tmp59		; <i64> [#uses=1]
	%tmp62 = add i64 %tmp61, %tmp55		; <i64> [#uses=7]
	%tmp63 = call i64 @llvm.bswap.i64(i64 %tmp11)		; <i64> [#uses=1]
	%tmp64 = add i64 %tmp47, %tmp34		; <i64> [#uses=1]
	%tmp65 = add i64 %tmp64, %tmp60		; <i64> [#uses=1]
	%tmp66 = add i64 %tmp4, %tmp63		; <i64> [#uses=1]
	%tmp67 = add i64 %tmp66, %tmp65		; <i64> [#uses=1]
	%tmp68 = add i64 %tmp67, %tmp60		; <i64> [#uses=2]
	%tmp69 = add i64 %tmp49, %tmp36		; <i64> [#uses=1]
	%tmp70 = add i64 %tmp62, %tmp69		; <i64> [#uses=1]
	%tmp71 = add i64 %tmp49, %tmp36		; <i64> [#uses=1]
	%tmp72 = add i64 %tmp70, %tmp71		; <i64> [#uses=1]
	%tmp73 = add i64 %tmp68, %tmp		; <i64> [#uses=5]
	%tmp74 = add i64 %tmp62, %tmp72		; <i64> [#uses=1]
	%tmp75 = add i64 %tmp74, %tmp68		; <i64> [#uses=7]
	%tmp76 = call i64 @llvm.bswap.i64(i64 %tmp12)		; <i64> [#uses=1]
	%tmp77 = add i64 %tmp60, %tmp47		; <i64> [#uses=1]
	%tmp78 = add i64 %tmp77, %tmp73		; <i64> [#uses=1]
	%tmp79 = add i64 %tmp34, %tmp76		; <i64> [#uses=1]
	%tmp80 = add i64 %tmp79, %tmp78		; <i64> [#uses=1]
	%tmp81 = add i64 %tmp80, %tmp73		; <i64> [#uses=2]
	%tmp82 = add i64 %tmp62, %tmp49		; <i64> [#uses=1]
	%tmp83 = add i64 %tmp75, %tmp82		; <i64> [#uses=1]
	%tmp84 = add i64 %tmp62, %tmp49		; <i64> [#uses=1]
	%tmp85 = add i64 %tmp83, %tmp84		; <i64> [#uses=1]
	%tmp86 = add i64 %tmp81, %tmp36		; <i64> [#uses=5]
	%tmp87 = add i64 %tmp75, %tmp85		; <i64> [#uses=1]
	%tmp88 = add i64 %tmp87, %tmp81		; <i64> [#uses=7]
	%tmp89 = call i64 @llvm.bswap.i64(i64 %tmp13)		; <i64> [#uses=1]
	%tmp90 = add i64 %tmp73, %tmp60		; <i64> [#uses=1]
	%tmp91 = add i64 %tmp90, %tmp86		; <i64> [#uses=1]
	%tmp92 = add i64 %tmp47, %tmp89		; <i64> [#uses=1]
	%tmp93 = add i64 %tmp92, %tmp91		; <i64> [#uses=1]
	%tmp94 = add i64 %tmp93, %tmp86		; <i64> [#uses=2]
	%tmp95 = add i64 %tmp75, %tmp62		; <i64> [#uses=1]
	%tmp96 = add i64 %tmp88, %tmp95		; <i64> [#uses=1]
	%tmp97 = add i64 %tmp75, %tmp62		; <i64> [#uses=1]
	%tmp98 = add i64 %tmp96, %tmp97		; <i64> [#uses=1]
	%tmp99 = add i64 %tmp94, %tmp49		; <i64> [#uses=5]
	%tmp100 = add i64 %tmp88, %tmp98		; <i64> [#uses=1]
	%tmp101 = add i64 %tmp100, %tmp94		; <i64> [#uses=7]
	%tmp102 = call i64 @llvm.bswap.i64(i64 %tmp14)		; <i64> [#uses=1]
	%tmp103 = add i64 %tmp86, %tmp73		; <i64> [#uses=1]
	%tmp104 = add i64 %tmp103, %tmp99		; <i64> [#uses=1]
	%tmp105 = add i64 %tmp102, %tmp60		; <i64> [#uses=1]
	%tmp106 = add i64 %tmp105, %tmp104		; <i64> [#uses=1]
	%tmp107 = add i64 %tmp106, %tmp99		; <i64> [#uses=2]
	%tmp108 = add i64 %tmp88, %tmp75		; <i64> [#uses=1]
	%tmp109 = add i64 %tmp101, %tmp108		; <i64> [#uses=1]
	%tmp110 = add i64 %tmp88, %tmp75		; <i64> [#uses=1]
	%tmp111 = add i64 %tmp109, %tmp110		; <i64> [#uses=1]
	%tmp112 = add i64 %tmp107, %tmp62		; <i64> [#uses=5]
	%tmp113 = add i64 %tmp101, %tmp111		; <i64> [#uses=1]
	%tmp114 = add i64 %tmp113, %tmp107		; <i64> [#uses=7]
	%tmp115 = call i64 @llvm.bswap.i64(i64 %tmp15)		; <i64> [#uses=1]
	%tmp116 = add i64 %tmp99, %tmp86		; <i64> [#uses=1]
	%tmp117 = add i64 %tmp116, %tmp112		; <i64> [#uses=1]
	%tmp118 = add i64 %tmp115, %tmp73		; <i64> [#uses=1]
	%tmp119 = add i64 %tmp118, %tmp117		; <i64> [#uses=1]
	%tmp120 = add i64 %tmp119, %tmp112		; <i64> [#uses=2]
	%tmp121 = add i64 %tmp101, %tmp88		; <i64> [#uses=1]
	%tmp122 = add i64 %tmp114, %tmp121		; <i64> [#uses=1]
	%tmp123 = add i64 %tmp101, %tmp88		; <i64> [#uses=1]
	%tmp124 = add i64 %tmp122, %tmp123		; <i64> [#uses=1]
	%tmp125 = add i64 %tmp120, %tmp75		; <i64> [#uses=5]
	%tmp126 = add i64 %tmp114, %tmp124		; <i64> [#uses=1]
	%tmp127 = add i64 %tmp126, %tmp120		; <i64> [#uses=7]
	%tmp128 = call i64 @llvm.bswap.i64(i64 %tmp16)		; <i64> [#uses=1]
	%tmp129 = add i64 %tmp112, %tmp99		; <i64> [#uses=1]
	%tmp130 = add i64 %tmp129, %tmp125		; <i64> [#uses=1]
	%tmp131 = add i64 %tmp128, %tmp86		; <i64> [#uses=1]
	%tmp132 = add i64 %tmp131, %tmp130		; <i64> [#uses=1]
	%tmp133 = add i64 %tmp132, %tmp125		; <i64> [#uses=2]
	%tmp134 = add i64 %tmp114, %tmp101		; <i64> [#uses=1]
	%tmp135 = add i64 %tmp127, %tmp134		; <i64> [#uses=1]
	%tmp136 = add i64 %tmp114, %tmp101		; <i64> [#uses=1]
	%tmp137 = add i64 %tmp135, %tmp136		; <i64> [#uses=1]
	%tmp138 = add i64 %tmp133, %tmp88		; <i64> [#uses=5]
	%tmp139 = add i64 %tmp127, %tmp137		; <i64> [#uses=1]
	%tmp140 = add i64 %tmp139, %tmp133		; <i64> [#uses=7]
	%tmp141 = call i64 @llvm.bswap.i64(i64 %tmp17)		; <i64> [#uses=1]
	%tmp142 = add i64 %tmp125, %tmp112		; <i64> [#uses=1]
	%tmp143 = add i64 %tmp142, %tmp138		; <i64> [#uses=1]
	%tmp144 = add i64 %tmp141, %tmp99		; <i64> [#uses=1]
	%tmp145 = add i64 %tmp144, %tmp143		; <i64> [#uses=1]
	%tmp146 = add i64 %tmp145, %tmp138		; <i64> [#uses=2]
	%tmp147 = add i64 %tmp127, %tmp114		; <i64> [#uses=1]
	%tmp148 = add i64 %tmp140, %tmp147		; <i64> [#uses=1]
	%tmp149 = add i64 %tmp127, %tmp114		; <i64> [#uses=1]
	%tmp150 = add i64 %tmp148, %tmp149		; <i64> [#uses=1]
	%tmp151 = add i64 %tmp146, %tmp101		; <i64> [#uses=5]
	%tmp152 = add i64 %tmp140, %tmp150		; <i64> [#uses=1]
	%tmp153 = add i64 %tmp152, %tmp146		; <i64> [#uses=7]
	%tmp154 = call i64 @llvm.bswap.i64(i64 %tmp18)		; <i64> [#uses=1]
	%tmp155 = add i64 %tmp138, %tmp125		; <i64> [#uses=1]
	%tmp156 = add i64 %tmp155, %tmp151		; <i64> [#uses=1]
	%tmp157 = add i64 %tmp154, %tmp112		; <i64> [#uses=1]
	%tmp158 = add i64 %tmp157, %tmp156		; <i64> [#uses=1]
	%tmp159 = add i64 %tmp158, %tmp151		; <i64> [#uses=2]
	%tmp160 = add i64 %tmp140, %tmp127		; <i64> [#uses=1]
	%tmp161 = add i64 %tmp153, %tmp160		; <i64> [#uses=1]
	%tmp162 = add i64 %tmp140, %tmp127		; <i64> [#uses=1]
	%tmp163 = add i64 %tmp161, %tmp162		; <i64> [#uses=1]
	%tmp164 = add i64 %tmp159, %tmp114		; <i64> [#uses=5]
	%tmp165 = add i64 %tmp153, %tmp163		; <i64> [#uses=1]
	%tmp166 = add i64 %tmp165, %tmp159		; <i64> [#uses=7]
	%tmp167 = call i64 @llvm.bswap.i64(i64 %tmp19)		; <i64> [#uses=1]
	%tmp168 = add i64 %tmp151, %tmp138		; <i64> [#uses=1]
	%tmp169 = add i64 %tmp168, %tmp164		; <i64> [#uses=1]
	%tmp170 = add i64 %tmp167, %tmp125		; <i64> [#uses=1]
	%tmp171 = add i64 %tmp170, %tmp169		; <i64> [#uses=1]
	%tmp172 = add i64 %tmp171, %tmp164		; <i64> [#uses=2]
	%tmp173 = add i64 %tmp153, %tmp140		; <i64> [#uses=1]
	%tmp174 = add i64 %tmp166, %tmp173		; <i64> [#uses=1]
	%tmp175 = add i64 %tmp153, %tmp140		; <i64> [#uses=1]
	%tmp176 = add i64 %tmp174, %tmp175		; <i64> [#uses=1]
	%tmp177 = add i64 %tmp172, %tmp127		; <i64> [#uses=5]
	%tmp178 = add i64 %tmp166, %tmp176		; <i64> [#uses=1]
	%tmp179 = add i64 %tmp178, %tmp172		; <i64> [#uses=6]
	%tmp180 = call i64 @llvm.bswap.i64(i64 %tmp20)		; <i64> [#uses=1]
	%tmp181 = add i64 %tmp164, %tmp151		; <i64> [#uses=1]
	%tmp182 = add i64 %tmp181, %tmp177		; <i64> [#uses=1]
	%tmp183 = add i64 %tmp180, %tmp138		; <i64> [#uses=1]
	%tmp184 = add i64 %tmp183, %tmp182		; <i64> [#uses=1]
	%tmp185 = add i64 %tmp184, %tmp177		; <i64> [#uses=2]
	%tmp186 = add i64 %tmp166, %tmp153		; <i64> [#uses=1]
	%tmp187 = add i64 %tmp179, %tmp186		; <i64> [#uses=1]
	%tmp188 = add i64 %tmp166, %tmp153		; <i64> [#uses=1]
	%tmp189 = add i64 %tmp187, %tmp188		; <i64> [#uses=1]
	%tmp190 = add i64 %tmp185, %tmp140		; <i64> [#uses=4]
	%tmp191 = add i64 %tmp179, %tmp189		; <i64> [#uses=1]
	%tmp192 = add i64 %tmp191, %tmp185		; <i64> [#uses=4]
	%tmp193 = call i64 @llvm.bswap.i64(i64 %tmp21)		; <i64> [#uses=1]
	%tmp194 = add i64 %tmp177, %tmp164		; <i64> [#uses=1]
	%tmp195 = add i64 %tmp194, %tmp190		; <i64> [#uses=1]
	%tmp196 = add i64 %tmp193, %tmp151		; <i64> [#uses=1]
	%tmp197 = add i64 %tmp196, %tmp195		; <i64> [#uses=1]
	%tmp198 = add i64 %tmp197, %tmp190		; <i64> [#uses=2]
	%tmp199 = add i64 %tmp179, %tmp166		; <i64> [#uses=1]
	%tmp200 = add i64 %tmp192, %tmp199		; <i64> [#uses=1]
	%tmp201 = add i64 %tmp179, %tmp166		; <i64> [#uses=1]
	%tmp202 = add i64 %tmp200, %tmp201		; <i64> [#uses=1]
	%tmp203 = add i64 %tmp198, %tmp153		; <i64> [#uses=3]
	%tmp204 = add i64 %tmp192, %tmp202		; <i64> [#uses=1]
	%tmp205 = add i64 %tmp204, %tmp198		; <i64> [#uses=2]
	%tmp206 = call i64 @llvm.bswap.i64(i64 %tmp22)		; <i64> [#uses=1]
	%tmp207 = add i64 %tmp190, %tmp177		; <i64> [#uses=1]
	%tmp208 = add i64 %tmp207, %tmp203		; <i64> [#uses=1]
	%tmp209 = add i64 %tmp206, %tmp164		; <i64> [#uses=1]
	%tmp210 = add i64 %tmp209, %tmp208		; <i64> [#uses=1]
	%tmp211 = add i64 %tmp210, %tmp203		; <i64> [#uses=2]
	%tmp212 = add i64 %tmp192, %tmp179		; <i64> [#uses=1]
	%tmp213 = add i64 %tmp205, %tmp212		; <i64> [#uses=1]
	%tmp214 = add i64 %tmp192, %tmp179		; <i64> [#uses=1]
	%tmp215 = add i64 %tmp213, %tmp214		; <i64> [#uses=1]
	%tmp216 = add i64 %tmp211, %tmp166		; <i64> [#uses=2]
	%tmp217 = add i64 %tmp205, %tmp215		; <i64> [#uses=1]
	%tmp218 = add i64 %tmp217, %tmp211		; <i64> [#uses=1]
	%tmp219 = call i64 @llvm.bswap.i64(i64 %tmp23)		; <i64> [#uses=2]
	store volatile i64 %tmp219, i64* @X, align 8
	%tmp220 = add i64 %tmp203, %tmp190		; <i64> [#uses=1]
	%tmp221 = add i64 %tmp220, %tmp216		; <i64> [#uses=1]
	%tmp222 = add i64 %tmp219, %tmp177		; <i64> [#uses=1]
	%tmp223 = add i64 %tmp222, %tmp221		; <i64> [#uses=1]
	%tmp224 = add i64 %tmp223, %tmp216		; <i64> [#uses=1]
	%tmp225 = add i64 %tmp224, %tmp218		; <i64> [#uses=1]
	ret i64 %tmp225
}

declare i64 @llvm.bswap.i64(i64) nounwind readnone
