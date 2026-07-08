# Instruction-Level Profiling on AMDGPU with rocprofv2 Thread Trace (ATT)

A practical, tested guide to getting **per-instruction hotspots** for a single GPU
kernel on **gfx906** (Radeon VII / MI50 / MI60, GCN5), using ROCm's **Advanced
Thread Trace (ATT)**, plus the interactive viewer that ships with it.

Everything here was worked out empirically on ROCm **5.7.1**, gfx906. The exact
commands, the env vars, and — especially — the pitfalls and interpretation
caveats are all things you will hit; they are documented inline.

---

## 1. What this is, and what it is not

There are two hardware mechanisms for "where in the code does time go" on AMD GPUs:

| | **PC sampling** | **ATT / SQTT thread trace** (this doc) |
|---|---|---|
| Mechanism | HW periodically records the program counter (statistical, like `perf`) | HW records *every* instruction of the *traced* waves |
| Overhead | Low | High (intercepts dispatches, large traces) |
| Scope | Whole app, all kernels at once | **One kernel, one CU, per run** |
| Output | "% of samples per line" (approximate) | Exact cycles/hits per ISA instruction |
| **gfx906 support** | **No** (needs gfx90a/MI200+, rocprofiler-sdk) | **Yes** |

So on gfx906 you cannot cheaply PC-sample a whole run. ATT is the available path,
and it is the opposite trade: precise per-instruction data, but you must isolate
one kernel, keep the run small, and trace a single compute unit.

Only **`rocprofv2`** has ATT. Plain `rocprof` (v1) has no thread-trace support.

---

## 2. One-time setup

**Confirm the ATT plugin is present** (it ships with ROCm):

```bash
ls $ROCM_PATH/lib/rocprofiler/libatt_plugin.so \
   $ROCM_PATH/lib/hsa-amd-aqlprofile/librocprofv2_att.so
ls $ROCM_PATH/libexec/rocprofiler/att/          # att.py, att_to_csv.py, ui/, ...
```

**Pin `websockets` to a legacy version.** The viewer (`att.py`) imports
`websockets` at load time and uses the *legacy* `websockets.serve()` API (called
outside a running event loop). Modern websockets (>= 11) removed that and the
viewer crashes with `RuntimeError: no running event loop`. Install a compatible
version once:

```bash
pip install 'websockets<11'      # 10.4 works; 15.x does NOT
```

Even the CSV path (`att.py --mode csv`) imports the viewer module, so this pin is
required for *any* ATT decode, not just the GUI.

---

## 3. The workflow at a glance

1. Build the target with debug info (`-g`) so ISA maps back to source.
2. Extract the kernel's gfx906 ISA disassembly (`roc-obj` + `llvm-objdump`).
3. Write a tiny ATT input file (which CU / shader engine to trace).
4. Filter the capture to your kernel with `KERNEL=<substring>` (see §7).
5. Capture: `rocprofv2 --plugin att … --mode csv <app>`.
6. Read the per-instruction CSV, and/or
7. Launch the interactive viewer (`--mode network`) at `localhost:8000`.

A complete worked example (the d3q19-bgk LBM `collide_and_stream` kernel) runs
through §4–§9.

---

## 4. Build with debug info

Build the app normally but add `-g` (keep your optimization level — `-O3 -g` is
fine and barely perturbs codegen). `-g` gives DWARF line tables so ATT can map
hot ISA lines back to source.

```bash
# example: a Makefile that takes EXTRA_CFLAGS, default -O3
make EXTRA_CFLAGS=-g
```

---

## 5. Extract the kernel ISA

ATT decodes the trace against a disassembly of the code object. Pull the gfx906
code object out of the executable and disassemble it:

```bash
roc-obj -o /tmp/co <app>                                  # extract bundles
CO=$(ls /tmp/co/*gfx906*)                                 # the gfx906 code object
llvm-objdump -d --mcpu=gfx906 "$CO" > isa.s               # the ISA ATT needs
grep -E '^[0-9a-f]+ <.*>:' isa.s                          # list kernels
```

One `isa.s` covering all kernels in the module is fine; the decoder picks the
traced kernel by name.

---

## 6. The ATT input file

ATT capture parameters go in a small text file passed with `-i`. The keys are
parsed by `librocprofiler_tool.so` (not the att plugin — look there if you need
to confirm the accepted set). The useful ones on ROCm 5.7.1:

```
att: TARGET_CU=1
SE_MASK=0x1
KERNEL=collide
```

- `TARGET_CU` — which compute unit to trace.
- `SE_MASK` — bitmask of shader engines. `0x1` = SE0 only.
- `SIMD_MASK` — bitmask of SIMDs within the CU.
- **`KERNEL`** — capture-time kernel filter (§7). This is the key that lets you
  target one kernel **without editing your program.**
- `DISPATCH` — filter by dispatch index.
- `BUFFER_SIZE` — trace buffer size (raise it to span more dispatches).

> **Note:** the parameter is `SIMD_MASK`, **not** `SIMD_SELECT`, and there is no
> `ISA_CAPTURE_MODE`. An unknown key prints `Error: Invalid parameter name: …`
> and is ignored (harmless) — check the spelling against the list above.

You only get data for the shader engine(s) in `SE_MASK`; the others come back as
`Error parsing …_seN.att` (empty 40-byte files) — **this is expected**, not a
failure.

---

## 7. Target a specific kernel with `KERNEL=` (do not edit your source)

An app usually runs setup kernels before the one you care about. SQTT traces the
dispatches that fit in the buffer, so a naive capture grabs the wrong (earlier)
kernel. **The fix is the `KERNEL` filter in the input file — not touching your
program.**

```
att: TARGET_CU=1
SE_MASK=0x1
KERNEL=collide
```

`KERNEL` does a **literal substring match** against the kernel name — `collide`
matches `collide_and_stream_g`. With it, you capture the **unmodified** app and
ATT walks past the setup dispatches to the first one whose name contains the
substring (you will see e.g. `Parsing kernel: collide_and_stream_g dispatch[6]`
— the 6th dispatch overall, setup skipped).

Two gotchas:

- **It is NOT a regex.** `KERNEL=.*collide.*` matches **nothing** (it searches
  for the literal characters `.*collide.*`), and even `KERNEL=.*` matches
  nothing. Use a plain substring: `KERNEL=collide`.
- Pick a substring **unique** to your kernel, or the filter is ambiguous.

(If you ever need coarser control instead: `DISPATCH=` selects by dispatch index,
and raising `BUFFER_SIZE` lets a single capture span more dispatches, which you
then separate at decode time with `att.py --att_kernel`.)

**The one requirement that remains: the kernel must be wave-rich.** SQTT traces a
single CU, so a kernel that launches only a handful of workgroups (e.g. a
1024-element toy: 8 workgroups ≈ 16 waves spread over 60 CUs) lands ~0 waves on
the traced CU and the trace comes back empty (`Error parsing` / `len(SIMD)==0`).
You need **thousands of waves** — real hot kernels qualify, toy demos do not. Run
at a problem size big enough for that; you can still keep the run short (a few
iterations) to bound the trace size.

---

## 8. Capture

```bash
rm -rf att_out && mkdir att_out
OUT_FILE_NAME=hotspots.csv \
  rocprofv2 -d att_out -i att_input.txt \
            --plugin att isa.s --mode csv \
            ./app <args>
```

- `--plugin att isa.s` — the ISA disassembly is the first positional after
  `--plugin att`.
- `--mode csv` — writes a per-instruction CSV (see §11). Other modes: `file`
  (dumps JSON for the viewer), `network` (launches the viewer, §10).
- `OUT_FILE_NAME` — names the CSV.
- Wrap with `timeout` — ATT overhead is high and a runaway trace can hang.

Success looks like:

```
--------------collecting data for shader_engine 0---------------
Parsing kernel: collide_and_stream_g dispatch[3] GPU[0]
Fully parsed 930 waves          <-- non-empty: good
Generating CSV file: hotspots.csv
```

`Fully parsed N waves` with N large is the signal it worked. `Error parsing` on
the non-target shader engines is normal (§6).

---

## 9. Decode an existing trace without re-running

The `.att` files in `att_out/` are the raw trace; you can re-decode them (e.g. to
try the viewer, or a different kernel filter) **without re-running the app**. Call
`att.py` directly — but it needs three environment variables that `rocprofv2`
normally sets for you:

```bash
export ROCPROFV2_ATT_LIB_PATH=$ROCM_PATH/lib/hsa-amd-aqlprofile/librocprofv2_att.so
export COUNTERS_PATH=$PWD/att_input.txt          # MUST be absolute
OUT_FILE_NAME=hotspots.csv \
  python3 $ROCM_PATH/libexec/rocprofiler/att/att.py \
          isa.s \
          --att_kernel att_out/<kernel>_v0_kernel.txt \
          --mode csv
```

- Missing `ROCPROFV2_ATT_LIB_PATH` → `ATT Lib path not set`.
- Missing / relative `COUNTERS_PATH` → `FileNotFoundError` on the input file.
- `--att_kernel` points at the `*_kernel.txt` the capture produced for the kernel
  you want.

---

## 10. The interactive viewer

```bash
export ROCPROFV2_ATT_LIB_PATH=$ROCM_PATH/lib/hsa-amd-aqlprofile/librocprofv2_att.so
export COUNTERS_PATH=$PWD/att_input.txt
python3 $ROCM_PATH/libexec/rocprofiler/att/att.py \
        isa.s --att_kernel att_out/<kernel>_v0_kernel.txt \
        --mode network --ports 8000,18000 &
```

Then open **`http://localhost:8000`** in a browser on the machine (it binds to
`127.0.0.1`). Two servers come up: HTTP on 8000 (the page) and a websocket on
18000 (the data). If the page loads but shows nothing, the websocket died — check
the `websockets` version (§2).

What the viewer shows:

- **Occupancy** / **Counters** tabs — waves-in-flight and counter plots over time.
- **ISA panel** — the disassembly with three numbers per instruction:
  `Count(times)`, `Issue to Inst(cycles)`, `Delay to Next Issue(cycles)`.
- **Wave timeline** — this is **not shown by default.** You must use the
  **"Shader:"** dropdown to pick the shader engine that has data (**SE0** with
  `SE_MASK=0x1`), then the **"Trace"** dropdown to pick a TraceID. Only then does
  the per-wave timeline canvas draw and the ISA panel populate. Blank dropdowns =
  nothing selected.

Honestly, the viewer is a clunky research tool; the CSV (§11) is easier for most
analysis. The viewer's value is *seeing* the wave timeline.

**Shutting it down.** The `&` backgrounds it, and `att.py` forks *separate* child
processes for the HTTP (8000) and websocket (18000) servers, which keep running
and holding those ports until killed. Stop them with:

```bash
pkill -f att.py                       # kills the att.py servers
```

If a port is still held (e.g. you relaunched and the page is stale or the bind
fails), find and kill whatever owns it:

```bash
ss -ltnp | grep -E ':8000|:18000'     # shows the pid holding each port
kill <pid>
```

Always kill an old instance before relaunching on the same ports — a lingering
server will either serve a stale trace or block the new bind.

---

## 11. Reading the output — and what the numbers really mean

The CSV columns are:

```
Line, Instruction, Hitcount, Cycles, Source Reference
```

- **Hitcount** — number of traced wave-executions of that ISA line.
- **Cycles** — the sum, over those wave-executions, of the per-instruction
  duration (the viewer's "Delay to Next Issue").

Sort by `Cycles` for hotspots. On a memory-latency-bound kernel the top line is
typically an `s_waitcnt vmcnt(...)` — the wave parked waiting on loads.

### What "Cycles" actually measures (read this before trusting it)

The per-instruction duration is, for one wave, the **wall-clock cycles from this
instruction's issue to that same wave's *next* issue**. That number contains
three things mixed together:

1. the instruction's own throughput latency (a VALU op occupies a GCN SIMD 4
   cycles regardless — 64 lanes / 16),
2. **cycles the SIMD spent issuing *other* resident waves** (round-robin
   multiplexing — not a stall of this wave at all), and
3. the genuine stall (next instruction not ready — waiting on a dependency/load).

Consequences:

- **It is a per-wave, summed quantity, not a hardware-idle measure.** Per-wave
  average stall ≈ `Cycles / Hitcount`.
- **It is contaminated by multiplexing, and the contamination grows with
  occupancy.** In a high-occupancy kernel, even a perfectly-pipelined instruction
  shows a large "delay" purely because other waves took issue slots in between.
  Treat per-instruction `Cycles` skeptically above occupancy 1.
- **It is cleanest at occupancy 1**, where there are no other waves to interleave,
  so the delay is genuinely this wave's own latency + real stalls. (This is why an
  occupancy-1 kernel's big `s_waitcnt` is a trustworthy exposed stall.)

### Occupancy is *residency*, not *issue*

The Occupancy tab counts **resident** waves — a wave parked on `s_waitcnt` still
counts. High occupancy across a stall means there were *candidates* to hide it,
**not** that any did: if all resident waves hit the same load in lockstep, the CU
idles despite high occupancy. Whether a stall was actually *hidden* (some other
wave issuing during it) is a cross-wave question this tooling does not expose —
the decoder stitches all waves into one aggregate timeline, so the individual
per-wave issue timelines needed to answer it are not readily recoverable.

### Aggregate corroboration via counters

For a whole-kernel sanity check on whether the SIMDs kept working (latency hidden)
or sat idle (exposed), use ordinary `rocprof` counters — e.g. `VALUBusy%`
(fraction of active time the VALU issued), `MemUnitBusy%`, `MemUnitStalled%`.
Low `VALUBusy%` on a kernel that is not bandwidth-saturated points to exposed
latency. These are coarse, whole-kernel proxies — there is **no** counter that
directly reports "% of load latency exposed," and `MemUnitStalled` is
memory-*subsystem* backpressure, not waves-waiting-for-loads.

---

## 12. Troubleshooting quick reference

| Symptom | Cause / fix |
|---|---|
| `RuntimeError: no running event loop` | websockets too new → `pip install 'websockets<11'` (§2) |
| `ModuleNotFoundError: websockets` | `pip install 'websockets<11'` |
| `ATT Lib path not set` | `export ROCPROFV2_ATT_LIB_PATH=…/librocprofv2_att.so` (§9) |
| `FileNotFoundError` on att input | `COUNTERS_PATH` unset or relative → use an **absolute** path (§9) |
| `Error: Invalid parameter name: SIMD_SELECT` | Misspelled key; the name is `SIMD_MASK` (§6) |
| `Error parsing …_se1/2/3.att` | Expected — only the SE in `SE_MASK` has data (§6) |
| `len(SIMD)==0` / empty traces | Kernel too small — needs thousands of waves on the traced CU (§7) |
| Captured the wrong kernel | Add `KERNEL=<substring>` to the input file (§7) |
| `KERNEL=…` captures nothing | It's a literal substring, not a regex — `KERNEL=collide`, not `.*collide.*` (§7) |
| Viewer page loads but is blank | Pick a Shader engine **and** a Trace from the dropdowns (§10); or dead websocket (§2) |

---

## 13. Limitations (gfx906 / ROCm 5.7.1)

- **No PC sampling** — statistical whole-app profiling is unavailable; ATT is the
  only instruction-level path (§1).
- **One kernel, one CU, per run** — not a whole-app profile.
- **Per-instruction `Cycles` is multiplexing-contaminated** above occupancy 1
  (§11); trustworthy mainly at low occupancy.
- **No omniperf / omnitrace** here, and their gfx906 support was always partial.
- The viewer is fragile (websockets version, manual dropdown selection); the CSV
  is the more dependable artifact.
