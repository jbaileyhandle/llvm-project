# Array-to-Register Promotion and Rematerialization on AMDGPU

Findings from a source dive (2026-08-26) prompted by an observed case where one
LLVM version kept a kernel's array in scratch memory and another promoted it to
registers — raising VGPR use, lowering occupancy, and *improving* performance.
This doc records which passes make that decision, the exact policy in **our
fork** (LLVM 17 snapshot, see "Version timeline"), how the policy changed
upstream since, and the related facts about rematerialization. Everything
cited to code was verified against this tree; upstream history was verified
against llvm/llvm-project on GitHub.

## 1. Which pass promotes an array to registers

Two promoters, different scopes:

- **SROA / mem2reg** (generic middle end): promotes scalars, and arrays only
  when *every* index is a compile-time constant — usually meaning full loop
  unrolling ran first. A version-to-version change in unrolling heuristics can
  therefore flip array promotion without any promotion-pass change.
- **`AMDGPUPromoteAlloca`**
  (`llvm/lib/Target/AMDGPU/AMDGPUPromoteAlloca.cpp`): the target-specific pass
  and the one that matters for private arrays. It is IR-level (runs long
  before ISel/scheduling/RA; added to the codegen pipeline in
  `AMDGPUTargetMachine.cpp`, plus an early to-vector-only variant in the opt
  pipeline). It rewrites a private (scratch) array alloca into an IR
  **vector**: GEP+load/store become extract/insertelement, and — unlike SROA —
  it handles *dynamic* indices, which become register indexing
  (`v_movrel`/lane-select) at selection. It can alternatively promote the
  array to LDS. Unpromoted private arrays live in scratch, which is swizzled
  global memory traffic on gfx906 — this is why the pass exists and why the
  win in the observed case was large.

Debug/experiment knobs: `-mllvm -debug-only=amdgpu-promote-alloca` logs each
alloca and the accept/reject reason; `-mllvm -disable-promote-alloca-to-vector`
is a clean A/B switch to reproduce the array-in-memory version deliberately.

## 2. The promotion policy in our fork

The budget chain is: **declared workgroup size → required minimum occupancy →
VGPR count at that occupancy → quarter of it, in bytes**.

- `getMaxVGPRs` (AMDGPUPromoteAlloca.cpp:160-174) evaluates
  `ST.getMaxNumVGPRs(ST.getWavesPerEU(F).first)`. The `.first` is the
  *minimum* waves/EU the function must sustain, and its default is not 1: it
  is derived from the max flat workgroup size (AMDGPUSubtarget.cpp,
  `getWavesPerEUForWorkGroup`). Default HIP kernel (workgroup bound 1024 = 16
  waves over gfx906's 4 SIMDs) → floor of 4 waves/SIMD → 64 VGPRs.
- The size gate (AMDGPUPromoteAlloca.cpp:340-348) rejects an alloca if
  `SizeBits * 4 > MaxVGPRs * 32`, i.e. alloca bytes > MaxVGPRs — which is a
  **quarter** of the register budget at the floor (MaxVGPRs registers hold
  MaxVGPRs*4 bytes/thread). Default kernel: max 64 bytes = 16 dwords.
  `__launch_bounds__(256)` drops the floor to 1 wave → 256 VGPRs → 256-byte
  cap. **Launch bounds silently change which arrays get promoted.**
- Hard cap of **16 vector elements** on top of the byte budget
  (AMDGPUPromoteAlloca.cpp:360).
- Non-entry, non-alwaysinline functions are clamped to 32 VGPRs
  (AMDGPUPromoteAlloca.cpp:167-172) — only 32 caller-saved VGPRs exist.
- **Per-array, not cumulative**: `MaxVGPRs` is computed once (line 233) and
  each alloca is checked independently (lines 249-254); nothing is decremented
  on success. Three 16-dword arrays all pass — 48 VGPRs from a "quarter" rule.
- **Open loop**: the pass runs where no register pressure exists to observe.
  The occupancy floor is a proxy; actual consequences surface hundreds of
  passes later in the scheduler/RA. By construction it can only spend
  headroom *above* the declared floor — it cannot push occupancy below the
  launch-bounds requirement, but within that band it is blind.
- Flag-units quirk: `-mllvm -amdgpu-promote-alloca-to-vector-limit=N` is
  nominally bytes, but the quarter multiplier still applies to the other side
  of the comparison, so the effective cap is **N/4 bytes** (to allow a 64-byte
  array, pass 256).

### Fork-specific interaction with misched.txt occupancy pins

In our fork, `AMDGPUSubtarget::getWavesPerEU` consults
`MachineInstrSchedulerConfig` (our marked block in AMDGPUSubtarget.cpp): a
`kernel <sig>/<min>,<max>` waves-per-EU pin is honored even below the
flat-workgroup-size floor. Since `AMDGPUPromoteAlloca` budgets off
`getWavesPerEU(F).first`, **an occupancy truth-pin changes the promotion
budget too** — e.g. pinning min-waves 2 on a default kernel raises MaxVGPRs
from 64 to 128 and doubles the promotable-array cap. Occupancy pins do not
only steer the scheduler; they can flip promote-alloca upstream of it.

## 3. Register cost arithmetic

A VGPR is per-thread storage: each thread owns one 32-bit slot of each VGPR
(the 64 lanes × 4 B = 256 B is the physical SRAM across the wave). So
per-thread bytes convert to VGPRs at **4 bytes per register**: a 64-entry ×
4-byte private array costs **64 VGPRs**, by itself capping occupancy at 4
waves/SIMD on gfx906. (In our fork such an array is never promoted — 16-element
cap — but newer compilers face exactly this arithmetic.)

Why promotion crossing an occupancy boundary is uncommon (inference, one
observed counterexample): (a) the budget caps the bite at 16 dwords here /
32 registers upstream-default; (b) the promoted vector only moves *peak*
pressure if its live span overlaps the peak; (c) the kernel must sit near a
bracket edge, and mid-range gfx906 brackets are 16-44+ VGPRs wide.

## 4. Version timeline of the promotion policy

Our fork: `amd-stg-open` last merged upstream main **2023-06-19** (merge
694ee8296786, upstream point 3350ec9b3e93), LLVM_VERSION_MAJOR 17 — ROCm 5.7
era. `AMDGPUPromoteAlloca.cpp` last upstream touch **2023-05-15**
(f104eb6e1550). The fork *does* contain the April 2023 structural refactor
(83ae2d3618c1) — the refactor changed no policy.

Policy changes upstream, all AFTER our snapshot (three separate changes, not
one rewrite):

| Date | Change | Ref |
|---|---|---|
| 2024-03-19 | Whole-function **cumulative** budget: per-function `VectorizationBudget` drawn down per promotion; allocas sorted by score (user count, loop users weighted 4x via `promote-alloca-vector-loop-user-weight`). 16-element cap still present. Pierre van Houtryve. LLVM 19. | [PR #84735](https://github.com/llvm/llvm-project/pull/84735) |
| 2025-03-12 | 16-element cap **replaced** by register-based limit `amdgpu-promote-alloca-to-vector-max-regs` (default 16 regs — same effective i32 size, more type/shape flexibility incl. multi-dim arrays) + tunable `amdgpu-promote-alloca-to-vector-vgpr-ratio`; both also function attributes. Carl Ritson. LLVM 21. | [PR #127973](https://github.com/llvm/llvm-project/pull/127973) |
| 2025-08-26 | Default max-regs raised 16 → **32** (128 B/thread). LLVM 22. | [commit 1f6648ccaaa6 / PR #155076](https://github.com/llvm/llvm-project/commit/1f6648ccaaa6) |

Current upstream main (verified by fetch): no literal 16; `MaxElements =
(MaxVectorRegs * 32) / ElementBits`; cumulative `VectorizationBudget -=
AllocaCost` over score-sorted allocas; the quarter-budget comment survives with
a `FIXME: Increase the limit for whole function budgets? Perhaps x2?`.

Implication for cross-compiler comparisons: even current upstream defaults cap
one promoted array at 32 VGPRs — a 64-dword array is still rejected by default
everywhere. A big-array promotion difference between compilers implies a
raised limit (flag/attribute/downstream fork) or an eligibility change
(unrolling), not just the timeline above.

## 5. Rematerialization: eligibility and practical reach

The public query is three gates (`TargetInstrInfo.h:132-137`):

```cpp
isTriviallyReMaterializable(MI) =
    IMPLICIT_DEF
 || ( MCID.isRematerializable()                        // gate 1: tablegen flag
      && ( SIInstrInfo::isReallyTriviallyReMaterializable(MI)   // gate 2: target hook
           || isReallyTriviallyReMaterializableGeneric(MI) ) ); // gate 3: generic
```

- **Gate 1** — opt-in per opcode in the .td files (`isReMaterializable = 1`):
  broad across VALU/SALU *arithmetic* (VOP1/2/3/3P, SOP moves/adds/shifts),
  never on branches/memory/waits. `s_branch` is SALU and would pass the hook;
  the flag is what stops it.
- **Gate 2** — `SIInstrInfo::isReallyTriviallyReMaterializable`
  (SIInstrInfo.cpp:110) *extends* the generic (OR, not replacement). For
  flagged VOP1/VOP2/VOP3/SDWA/SALU it requires: no implicit defs, no implicit
  operands beyond the descriptor (descriptor-level exec/mode reads are fine),
  no FP-exception risk. Its comment states the key liberalization: virtual
  register uses are ALLOWED (generic forbids them). Consequence of the
  implicit-def rule: e32 carry-writing ops (`v_add_co_u32_e32` implicitly
  defines VCC) are not remat-able — part of why VCC-threaded code is immovable.
  No mayLoad/mayStore check needed: those encodings structurally exclude
  memory ops.
- **Gate 3** — the generic fallback bans virtual-register uses outright and
  additionally admits invariant loads (immutable stack slots etc.).

Soundness of allowing register uses is enforced by the *clients* per remat
point: `LiveRangeEdit::allUsesAvailableAt` (RA spilling), and
`PreRARematStage` (GCNSchedStrategy.h:366) which only sinks single-def,
single-use-outside-block VGPR defs to claw back a wave.

Practical reach — "of X live VGPRs at the pressure peak, how many are remat
candidates" has no general answer; split the live set:

- **loaded data**: never remat-able (dominates the peaks in our spillers;
  rsbench's peak is live loaded doubles → remat fraction ~0);
- **loop-carried accumulators**: not remat-able;
- **addresses/constants**: mostly remat-able (a `v_add` off a still-live base
  qualifies under the AMDGPU rule) — tens of registers in unrolled
  address-heavy kernels.

So ~0% in data-dominated peaks, maybe 10-30% in address/ALU-heavy ones.
Measurable per kernel (compile-only): dump the live set at the peak region and
count defs passing the predicate with operands live across the span.

## 6. Catalog: optimizations that reduce memory accesses

Generic middle end: SROA/mem2reg; GVN/EarlyCSE/NewGVN (redundant-load
elimination, load PRE — lazy code motion, Knoop/Rüthing/Steffen PLDI 1992);
LICM with scalar promotion; DSE/MemCpyOpt/loop-idiom; unroll+SROA synergy;
vectorizers (fewer, wider transactions).

AMDGPU backend: AMDGPUPromoteAlloca (vector/LDS); LoadStoreVectorizer
(dwordx4 merging); uniform-load scalarization to SMEM (one fetch per wave;
fed by AMDGPUPromoteKernelArguments); AMDGPUAtomicOptimizer (per-lane global
atomics → one per-wave atomic via DPP reduction).

Literature beyond LLVM: scalar replacement of subscripted variables
(Callahan/Carr/Kennedy PLDI 1990; unroll-and-jam, Carr & Kennedy 1994);
register tiling/blocking — on GPUs Volkov & Demmel SC 2008 (lower occupancy +
more registers wins); shared-memory staging (Ryoo et al. PPoPP 2008);
wave-level register exchange via `ds_permute`/DPP; kernel fusion (Wahib &
Maruyama SC 2014); rematerialization (Briggs/Cooper/Torczon 1992).

Common thread: promotion, scalar replacement, register tiling, and remat are
the same trade — spend registers to remove memory traffic — and all of them
fight occupancy heuristics. The observed promoted-array case is that trade
paying off at reduced occupancy; rsbench is its failure mode (registers spent
that didn't exist, paid in spills).
