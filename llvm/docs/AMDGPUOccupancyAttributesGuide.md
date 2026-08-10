# AMDGPU Occupancy-Control Attributes: waves-per-eu (min/max) and flat-work-group-size

Three compile-time function attributes steer how the AMDGPU backend trades
registers, LDS, and occupancy. They are easy to conflate because they all sound
like "how many waves," but they drive *different* machinery, and one of them is
correctness-load-bearing. This guide describes each attribute and then, lever by
lever, exactly what each one affects, with a summary matrix at the end.

Scope: observations are for the HierarchicalScheduler fork (gfx906 primary), but
the mechanisms are generic AMDGPU. File/line references are indicative and may
drift; treat them as starting points.

> **A naming trap.** `getMaxWavesPerEU()` has two unrelated meanings:
> `AMDGPUSubtarget::getMaxWavesPerEU()` (no args) returns the *hardware* maximum
> (a subtarget constant, e.g. 10 on gfx906) and does **not** read any attribute;
> `SIMachineFunctionInfo::getMaxWavesPerEU()` returns `WavesPerEU.second`, the
> *attribute*-derived max. The attribute pair itself is read by
> `AMDGPUSubtarget::getWavesPerEU(F)`. Throughout this doc, "max waves-per-eu"
> means the attribute value unless it says "hardware max."

## The attributes

### `amdgpu-waves-per-eu` minimum

The occupancy *floor* the kernel is allowed to run at (waves per SIMD/EU). Its
sole structural job is to set the register allocator's per-wave register budget:
a lower min permits more registers per wave (a lower occupancy). It carries no
runtime/correctness contract -- it is a pure performance lever. When the
attribute is absent, the min defaults to the value implied by the workgroup size
(see below). A misched-injected min is honored even below the flat-work-group
floor (`getWavesPerEU`'s `honor_misched_min`, `AMDGPUSubtarget.cpp`).

### `amdgpu-waves-per-eu` maximum

The occupancy *ceiling*. Unlike the min, it does **not** enter the register
budget in the normal case; instead its dominant effect is that the AsmPrinter
pads the reported register count up to `getMinNumVGPRs(max)`, which forces the
runtime to allocate for that occupancy and caps launched waves there regardless
of real register use. It also bounds a couple of scheduler/occupancy-reset paths
and biases LDS promotion. Because of the padding, it is a blunt instrument: it is
*not* the way to give a scheduler an occupancy target without side effects.

### `amdgpu-flat-work-group-size`

The flat workgroup-size range (`min,max`); for occupancy purposes almost
everything reads the `max`. Sourced from HIP `__launch_bounds__(maxThreads, ...)`
(or OpenCL `reqd_work_group_size`); defaults to `(1, 1024)` for a kernel when
unset (`getDefaultFlatWorkGroupSize`). Its footprint spans several independent
subsystems: it *derives the waves-per-eu floor* when waves-per-eu is unset (so it
feeds the register budget indirectly, through the min), it drives the LDS-based
occupancy model, it sizes LDS promotion, and it bounds workitem IDs. It is also
the only one of the three that carries a runtime **correctness** contract: the
optimizer assumes thread IDs are `< max` and
that `s_barrier` can degrade to a single-wave barrier when `max <= wavefront`, so
the attribute may only ever be *tightened to the true maximum block a kernel is
launched with* -- setting it smaller than reality is a miscompile, not a slowdown.

## Levers and which attributes drive them

### 1. Register-allocator VGPR/SGPR budget
The per-wave register ceiling `getBaseMaxNum{V,S}GPRs = getMaxNum{V,S}GPRs(WavesPerEU.first)`
(`AMDGPUSubtarget.cpp`). Driven by the **min**. The **max** enters only to
validate an explicit `amdgpu-num-{v,s}gpr` request (`getMinNum*GPRs(max)`). The
**workgroup size** enters only indirectly, by deriving the default min when
waves-per-eu is absent.

### 2. Initial MFI occupancy (the scheduler's starting ceiling)
`SIMachineFunctionInfo` constructor: `Occupancy = computeOccupancy(F, LDS)`, which
is `min(hardware getMaxWavesPerEU(), getOccupancyWithLocalMemSize(LDS, F))`. The
waves-per-eu **attribute plays no part here** (it uses the *hardware* max). The
**workgroup size** does, through `getOccupancyWithLocalMemSize`. (The scheduler's
target can still be lowered afterward by `limitOccupancy`, but that is an explicit
call, not an attribute effect.)

### 3. Code-object register padding / runtime occupancy cap
`NumVGPRsForWavesPerEU = max(NumVGPR, getMinNumVGPRs(MFI->getMaxWavesPerEU()))`
(and the SGPR analogue), `AMDGPUAsmPrinter.cpp`. This padded count flows into
`VGPRBlocks`, the code-object register field, and the runtime `setNumUsedVgprs`.
Driven by the **max**. This is the padding/occupancy-cap effect and the main
reason writing the max is heavy-handed.

### 4. Scheduler high-register-pressure reschedule bound
`if (MFI.getMaxWavesPerEU() > MinOccupancy) increaseOccupancy(...)`
(`GCNSchedStrategy.cpp`): how high the unclustered-high-RP retry may push
occupancy. Driven by the **max**.

### 5. Scheduler occupancy-floor check
`WavesAfter <= MFI.getMinWavesPerEU()` (`GCNSchedStrategy.cpp`) guards
rematerialization/occupancy decisions. Driven by the **min** (workgroup size
indirectly, via the derived min).

### 6. `resetInitialOccupancy` upper bound
`SIMachineFunctionInfo::limitOccupancy(MF)` does `limitOccupancy(getMaxWavesPerEU())`
(`SIMachineFunctionInfo.cpp`), on the `resetInitialOccupancy`/`increaseOccupancy`
path. Driven by the **max**.

### 7. PromoteAlloca: alloca -> VGPR budget
`getMaxVGPRs = getMaxNumVGPRs(getWavesPerEU(F).first)` (`AMDGPUPromoteAlloca.cpp`)
caps how many allocas are promoted to registers. Driven by the **min** (workgroup
size indirectly). Note PromoteAlloca runs pre-ISel and is register-pressure
*blind* -- this budget is a static heuristic; the real arbiter is the register
allocator much later. (Non-entry functions are additionally capped at 32, but
device functions are inlined into the kernel before this pass runs.)

### 8. PromoteAlloca: alloca -> LDS budget
`LocalMemLimit = getMaxLocalMemSizeWithWaveCount(min(OccupancyHint, occ-from-current-LDS))`
where `OccupancyHint = getWavesPerEU(F).second` (`AMDGPUPromoteAlloca.cpp`) and
`getMaxLocalMemSizeWithWaveCount` divides LDS by workgroups-per-CU computed from
the workgroup size. Driven by **both the max and the workgroup size** (a lower
max ⇒ more LDS promotion). The min plays no part.

### 9. Occupancy-from-LDS (`getOccupancyWithLocalMemSize`)
Feeds lever 2 and lever 8. Computed from static LDS and the **workgroup size**
(`getFlatWorkGroupSizes(F).second`); waves-per-eu does not enter.

### 10. Workitem-ID range metadata and bounds (correctness)
`makeLIDRangeMetadata` tags `llvm.amdgcn.workitem.id.*` with `[0, max)` and
`getMaxWorkitemID` returns `max - 1` (`SIISelLowering.cpp`, `AMDGPUSubtarget.cpp`).
The optimizer then narrows/eliminates guards on thread IDs. Driven by the
**workgroup size** only. Correctness-relevant: wrong if the real block exceeds it.

### 11. Barrier lowering `s_barrier` -> `WAVE_BARRIER` (correctness)
If `flatWorkGroupSize.second <= wavefrontSize`, a full barrier degrades to a
cheap single-wave barrier (`SIISelLowering.cpp`). Driven by the **workgroup size**
only. Correctness-relevant: drops real cross-wave synchronization if the block is
actually larger than one wave.

### 12. Hierarchical scheduler occupancy target/floor
`GCNRegisterTracker` reads `mfi->getMinWavesPerEU()` as the occupancy floor and
clamps computed occupancy with `min(occ, mfi->getMaxWavesPerEU())`. Driven by the
**min** (floor) and the **max** (clamp).

## Summary matrix

Legend: **Y** = directly driven by this attribute; **~** = indirectly (the
workgroup size derives the waves-per-eu min when waves-per-eu is unset;
`amdgpu-num-*gpr` enables a max validation); blank = not involved.

| Lever / optimization | min waves-per-eu | max waves-per-eu | flat-work-group-size |
|---|:--:|:--:|:--:|
| 1. RA VGPR/SGPR budget | Y | ~ | ~ |
| 2. Initial MFI occupancy (sched ceiling) | | | Y |
| 3. Code-object reg padding / runtime cap | | Y | |
| 4. Scheduler high-RP reschedule bound | | Y | |
| 5. Scheduler occupancy-floor check | Y | | ~ |
| 6. resetInitialOccupancy upper bound | | Y | |
| 7. PromoteAlloca: alloca -> VGPR | Y | | ~ |
| 8. PromoteAlloca: alloca -> LDS | | Y | Y |
| 9. Occupancy-from-LDS | | | Y |
| 10. Workitem-ID range/bounds (correctness) | | | Y |
| 11. Barrier lowering (correctness) | | | Y |
| 12. Hier scheduler target/floor | Y | Y | ~ |

## Practical notes

- **To reclaim register slack (relax the RA) without changing runtime
  semantics, use the waves-per-eu min.** It drives levers 1, 5, 7, 12 and has no
  correctness contract, so it is safe for any launch configuration; the worst
  case is a suboptimal schedule. It does *not* touch the LDS-occupancy model, LDS
  promotion, or workitem bounds -- which for register reclamation is the intended
  surface, not a gap.
- **The waves-per-eu max is not a clean "occupancy target."** Its dominant effect
  (lever 3) pads registers and caps runtime occupancy. To give a scheduler an
  occupancy ceiling without that, lower `MFI.Occupancy` directly with
  `limitOccupancy` instead of writing the attribute.
- **The workgroup size is the richest input but is correctness-load-bearing**
  (levers 10, 11). It may only be tightened to the true maximum launched block.
  It is the right lever when the goal is to fix the compiler's whole occupancy
  *model* for a kernel whose block size is known and fixed (ideally set at the
  source via `__launch_bounds__`), not a safe retrofit for register tuning.
