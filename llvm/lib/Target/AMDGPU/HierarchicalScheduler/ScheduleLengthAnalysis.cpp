//===- ScheduleLengthAnalysis.cpp - Post-schedule length/bubble analysis -===//
//
// See ScheduleLengthAnalysis.h for what this computes and why.
//
//===----------------------------------------------------------------------===//

#include "ScheduleLengthAnalysis.h"

#include "GCNRegPressure.h"
#include "GCNRegisterTracker.h"
#include "GCNSubtarget.h"
#include "RegionInfo.h"
#include "SIMachineFunctionInfo.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "ScheduleLengthTracker.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Format.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <fstream>
#include <memory>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// part as a percentage of whole; 0 when whole is 0.
double Percent(int part, int whole) {
  return whole > 0 ? 100.0 * part / whole : 0.0;
}

// One line of an instruction's text (no debug location, no trailing newline),
// with internal newlines/tabs flattened to single spaces so it stays on one
// timeline row.
std::string InstrText(const MachineInstr &mi) {
  std::string s;
  raw_string_ostream os(s);
  mi.print(os, /*IsStandalone=*/true, /*SkipOpers=*/false,
           /*SkipDebugLoc=*/true, /*AddNewLine=*/false);
  os.flush();
  for (char &c : s) {
    if (c == '\n' || c == '\t' || c == '\r') {
      c = ' ';
    }
  }
  // Trim leading/trailing spaces.
  size_t b = s.find_first_not_of(' ');
  size_t e = s.find_last_not_of(' ');
  return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
}

// Escape a string for embedding in a JSON double-quoted literal.
std::string JsonEscape(StringRef in) {
  std::string out;
  out.reserve(in.size() + 8);
  for (char c : in) {
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if (static_cast<unsigned char>(c) < 0x20) {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "\\u%04x", c);
        out += buf;
      } else {
        out += c;
      }
    }
  }
  return out;
}

// Make `s` safe to embed in a filename: keep [A-Za-z0-9._-], replace the rest
// with '_'.
std::string SanitizeForFilename(StringRef s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    out += (std::isalnum(static_cast<unsigned char>(c)) || c == '.' ||
            c == '_' || c == '-')
               ? c
               : '_';
  }
  return out;
}

// Walk the region's fixed final order once through a fresh ScheduleConstructor
// and produce both the per-lens summary stats and the per-instruction
// timeline. This is the schedule-length analysis' single source of truth for
// the bubble accounting (see the classification note below); the printed
// stats, the CSV, and the visualization all read the LensViz it returns.
//
// Bubble classification. At each stall (the next instruction's ready cycle is
// beyond the current cycle) the ready frontier tells us what could have filled
// the slot, and a tentative schedule of each ready instruction tells us
// whether it would fit the occupancy budget. Within one stall the live-set is
// fixed (nothing issues), so a ready instruction stays ready and its
// budget-fit is constant. Readiness is therefore monotonic in the cycle, and
// the per-cause cycle counts follow analytically from two thresholds:
//   ready_any_time = earliest cycle ANY ready instruction exists
//   ready_fit_time = earliest cycle a ready instruction that FITS exists
// with cur <= ready_any_time <= ready_fit_time <= node_ready. These cut the
// half-open stall window [cur, node_ready) into three CONTIGUOUS, DISJOINT
// pieces:
//   nothing-ready = [cur,            ready_any_time)
//   over-budget   = [ready_any_time, ready_fit_time)
//   avoidable     = [ready_fit_time, node_ready)
// Their lengths telescope to (node_ready - cur), i.e. exactly this stall's
// bubble count — every bubble cycle is attributed once, no double counting.
// Stalls across nodes don't overlap either (the node issues AT node_ready,
// which the half-open interval excludes), so the three counters sum to the
// region's total bubbles.
//
// `graph` must already have both critical-path directions computed (the caller
// does this) so on_crit_path can be read per node.
LensViz ComputeLensViz(const ScheduleGraph &graph, const GCNSubtarget &st,
                       const MachineFunction &mf, int occupancy) {
  LensViz v;
  const ScheduleLengthTracker &input_lt =
      graph.GetInputScheduleConstructor().GetLengthTracker();
  v.length = input_lt.GetCurrentCycle();
  v.bubbles = input_lt.GetTotalBubbles();
  v.critical_path = graph.GetCriticalPathLength();
  v.floor = graph.GetGraphLengthFloor();
  v.efficiency = v.length > 0 ? static_cast<double>(v.floor) / v.length : 0.0;

  const int cp_len = graph.GetCriticalPathLength();
  const bool have_cp =
      graph.HasCriticalPathFromEntry() && graph.HasCriticalPathFromExit();
  const int vgpr_budget = static_cast<int>(st.getMaxNumVGPRs(occupancy));
  const int sgpr_budget =
      static_cast<int>(st.getMaxNumSGPRs(occupancy, /*Addressable=*/true));
  const bool unified_vgpr = st.hasGFX90AInsts();

  // Fresh full-featured constructor (length + pressure trackers on). We replay
  // the same fixed order the input schedule used, so its per-node cycles match
  // the input constructor's.
  ScheduleConstructor sc(graph, st, mf);

  SmallVector<const ScheduleNode *, 32> ready;
  for (const ScheduleNode *node :
       graph.GetInputScheduleConstructor().GetScheduleOrder()) {
    const bool is_unit = node->IsSchedulingUnit();
    TimelineEntry entry;
    if (is_unit) {
      const ScheduleLengthTracker &lt = sc.GetLengthTracker();
      const int cur = lt.GetCurrentCycle();
      const int node_ready = lt.GetMinScheduleCycle(node);
      if (have_cp) {
        entry.on_crit_path = graph.GetCriticalPathFromEntry(node) +
                                 graph.GetCriticalPathFromExit(node) ==
                             cp_len;
      }
      // Latency-shadow length: the longest data-edge latency out of this node
      // (memory latency for a load), in this lens's cycles.
      for (const ScheduleEdge &se : node->Successors()) {
        if (se.IsLatencyEdge()) {
          entry.latency = std::max(entry.latency, se.Latency());
        }
      }
      if (node_ready > cur) {
        // Snapshot the ready frontier (stable across Schedule/Unschedule).
        ready.clear();
        sc.GetReadyListSnapshot(ready);

        // Defaults assume nothing else is ever ready in this window, so the
        // whole stall is nothing-ready.
        int ready_any_time = node_ready;
        int ready_fit_time = node_ready;
        for (const ScheduleNode *cand : ready) {
          if (!cand->IsSchedulingUnit() || cand == node) {
            continue;
          }
          const int cand_ready = lt.GetMinScheduleCycle(cand);
          ready_any_time = std::min(ready_any_time, cand_ready);

          // Would issuing cand now keep us within the occupancy budget?
          // Tentatively schedule it, read the resulting pressure, undo.
          sc.Schedule(cand);
          const GCNRegPressure &p =
              sc.GetPressureTracker().GetCurrentPressure();
          const bool fits =
              static_cast<int>(p.getVGPRNum(unified_vgpr)) <= vgpr_budget &&
              static_cast<int>(p.getSGPRNum()) <= sgpr_budget;
          sc.Unschedule();
          if (fits) {
            ready_fit_time = std::min(ready_fit_time, cand_ready);
          }
        }

        // Clamp the thresholds into the stall window. `fit >= any` is forced
        // so the three pieces stay disjoint (see the classification note).
        const int any = std::max(cur, std::min(ready_any_time, node_ready));
        const int fit = std::max(any, std::min(ready_fit_time, node_ready));
        const int nr = any - cur;
        const int ob = fit - any;
        const int av = node_ready - fit;
        v.nothing_ready += nr;
        v.over_budget += ob;
        v.avoidable += av;
        if (nr) {
          entry.bubble_before.push_back({"nothing-ready", nr});
        }
        if (ob) {
          entry.bubble_before.push_back({"over-budget", ob});
        }
        if (av) {
          entry.bubble_before.push_back({"avoidable", av});
        }
      }
    }

    sc.Schedule(node);

    if (is_unit) {
      // Register pressure after this instruction issues.
      const GCNRegPressure &p = sc.GetPressureTracker().GetCurrentPressure();
      entry.vgpr = static_cast<int>(p.getVGPRNum(unified_vgpr));
      entry.sgpr = static_cast<int>(p.getSGPRNum());
      entry.vgpr_pct = Percent(entry.vgpr, vgpr_budget);
      entry.sgpr_pct = Percent(entry.sgpr, sgpr_budget);
      // The actual cycle this instruction issues at (single-issue: max of the
      // current cycle and its ready cycle), not the earlier ready cycle. Read
      // after Schedule() so the tracker has recorded it.
      entry.issue_cycle = sc.GetLengthTracker().GetScheduledCycle(node);
      // A scheduling unit can have a null SUnit (synthetic boundary) or a
      // null MachineInstr; only real instructions carry text.
      const SUnit *su = node->GetSUnit();
      const MachineInstr *mi = su ? su->getInstr() : nullptr;
      entry.text = mi ? InstrText(*mi) : "";
      v.timeline.push_back(std::move(entry));
    }
  }
  return v;
}

// One bubble-cause line: raw count, share of all bubbles, share of all cycles.
void PrintBubbleCause(raw_ostream &os, StringRef cause, int count,
                      const LensViz &s) {
  os << "\t\t\t" << cause << "=" << count
     << format(" (%.1f%% of bubbles, %.1f%% of cycles)\n",
               Percent(count, s.bubbles), Percent(count, s.length));
}

// One latency lens: the length/critical-path/floor summary plus the bubble
// breakdown by cause.
void PrintLensReport(raw_ostream &os, StringRef label, const LensViz &s) {
  os << "\t\t" << label << ": critpath=" << s.critical_path
     << " length=" << s.length << " floor=" << s.floor
     << format(" efficiency=%.2f", s.efficiency) << " bubbles=" << s.bubbles
     << format(" (%.1f%% of cycles)\n", Percent(s.bubbles, s.length));
  PrintBubbleCause(os, "nothing-ready", s.nothing_ready, s);
  PrintBubbleCause(os, "over-budget", s.over_budget, s);
  PrintBubbleCause(os, "avoidable", s.avoidable, s);
}

// One row per region per lens, APPENDED to schedule_length_analysis.csv.
//
// Append-only (no truncate): every translation unit of a build accumulates into
// one file, so a multi-.cpp benchmark (e.g. lulesh) keeps ALL its kernels -- a
// truncating writer keeps only the last TU's, because each clang process starts
// fresh. The harness removes any stale copy before the build and renames the
// result per scheduler config, exactly as it does for kernel_resource_usage.csv.
// Each call reopens/closes, so its two rows flush together as one atomic append
// (they fit one buffer), which keeps rows from parallel clang processes from
// interleaving. No header row: the consumer (gpu2_benchmarks) knows the columns:
//   function,region,lens,num_ops,occupancy,peak_vgpr,vgpr_budget,peak_sgpr,
//   sgpr_budget,critical_path,length,floor,efficiency,bubbles,nothing_ready,
//   over_budget,avoidable
void WriteCsvRows(StringRef function, const RegionViz &r) {
  std::ofstream csv_file("schedule_length_analysis.csv", std::ios::app);
  if (!csv_file) {
    return;
  }
  // Demangle through the same helper kernel_resource_usage.csv uses, so this
  // file's kernel names match that file's (and the profiler's) exactly and the
  // consumer can join them by kernel. The demangled signature may contain
  // commas, so it stays FIRST in the row: the reader peels the 16 fixed
  // trailing fields off the right and keeps everything before as the name.
  std::string demangled =
      MachineInstrSchedulerConfig::DemangleFunctionSignature(function.str());
  auto write_row = [&](const char *lens, const LensViz &s) {
    csv_file << demangled << ',' << r.index << ',' << lens << ','
             << r.num_ops << ',' << r.occupancy << ',' << r.peak_vgpr << ','
             << r.vgpr_budget << ',' << r.peak_sgpr << ',' << r.sgpr_budget
             << ',' << s.critical_path << ',' << s.length << ',' << s.floor
             << ',' << s.efficiency << ',' << s.bubbles << ','
             << s.nothing_ready << ',' << s.over_budget << ',' << s.avoidable
             << '\n';
  };
  write_row("raw", r.raw);
  write_row("adjusted", r.adjusted);
}

// --- JSON serialization for the visualization ---------------------------

void WriteTimeline(raw_ostream &os, const std::vector<TimelineEntry> &tl,
                   StringRef indent) {
  os << "[";
  for (size_t i = 0; i < tl.size(); ++i) {
    const TimelineEntry &e = tl[i];
    os << (i ? ",\n" : "\n") << indent << "  {\"issue_cycle\": " << e.issue_cycle
       << ", \"latency\": " << e.latency
       << ", \"on_crit_path\": " << (e.on_crit_path ? "true" : "false")
       << ", \"vgpr\": " << e.vgpr << ", \"sgpr\": " << e.sgpr
       << format(", \"vgpr_pct\": %.1f, \"sgpr_pct\": %.1f", e.vgpr_pct,
                 e.sgpr_pct)
       << ", \"text\": \"" << JsonEscape(e.text) << "\", \"bubble_before\": [";
    for (size_t b = 0; b < e.bubble_before.size(); ++b) {
      os << (b ? ", " : "") << "{\"cause\": \"" << e.bubble_before[b].cause
         << "\", \"cycles\": " << e.bubble_before[b].cycles << "}";
    }
    os << "]}";
  }
  os << (tl.empty() ? "]" : (Twine("\n") + indent + "]").str());
}

void WriteLens(raw_ostream &os, const LensViz &s, StringRef indent) {
  os << "{\"critical_path\": " << s.critical_path << ", \"length\": " << s.length
     << ", \"floor\": " << s.floor
     << format(", \"efficiency\": %.4f", s.efficiency)
     << ", \"bubbles\": " << s.bubbles
     << ", \"nothing_ready\": " << s.nothing_ready
     << ", \"over_budget\": " << s.over_budget
     << ", \"avoidable\": " << s.avoidable << ", \"timeline\": ";
  WriteTimeline(os, s.timeline, indent);
  os << "}";
}

void WriteRegion(raw_ostream &os, const RegionViz &r, StringRef indent) {
  os << indent << "{\"index\": " << r.index << ", \"num_ops\": " << r.num_ops
     << ", \"occupancy\": " << r.occupancy << ", \"peak_vgpr\": " << r.peak_vgpr
     << ", \"vgpr_budget\": " << r.vgpr_budget
     << ", \"peak_sgpr\": " << r.peak_sgpr
     << ", \"sgpr_budget\": " << r.sgpr_budget << ",\n"
     << indent << " \"raw\": ";
  WriteLens(os, r.raw, (Twine(indent) + " ").str());
  os << ",\n" << indent << " \"adjusted\": ";
  WriteLens(os, r.adjusted, (Twine(indent) + " ").str());
  os << "}";
}

} // namespace

RegionViz ScheduleLengthAnalyzer::AnalyzeRegionFinalSchedule(
    MutableArrayRef<SUnit> sunits, const GCNSubtarget &st,
    const MachineFunction &mf, const LiveIntervals &lis,
    const MachineRegisterInfo &mri, const RegionInfo &region,
    int region_index) {
  RegionViz viz;
  viz.index = region_index;

  // Nothing to analyze for a region with no real instructions.
  if (sunits.empty()) {
    return viz;
  }

  // Achieved kernel occupancy after scheduling: the wave count the hardware
  // runs, and the register-budget rung. Drives the adjusted lens's latency
  // divisor and the register-% report. A real kernel always runs >= 1 wave; 0
  // is a broken invariant (and a divide-by-zero below), so fail loud.
  const int occupancy =
      static_cast<int>(mf.getInfo<SIMachineFunctionInfo>()->getOccupancy());
  if (occupancy < 1) {
    report_fatal_error(
        "ScheduleLengthAnalyzer: kernel occupancy is < 1 after scheduling");
  }

  // Two lenses over the SAME final order: raw (divisor 1) and occupancy-
  // adjusted (divisor = achieved occupancy).
  std::unique_ptr<ScheduleGraph> raw_graph =
      ScheduleGraph::BuildFromSUnits(sunits, st, mf, lis, mri, region,
                                     /*latency_divisor=*/1);
  std::unique_ptr<ScheduleGraph> adj_graph =
      ScheduleGraph::BuildFromSUnits(sunits, st, mf, lis, mri, region,
                                     /*latency_divisor=*/occupancy);

  // The from-exit direction is computed at build (GetCriticalPathLength);
  // compute from-entry too so per-node zero-slack (on_crit_path) is available.
  raw_graph->ComputeCriticalPathFromEntry();
  adj_graph->ComputeCriticalPathFromEntry();

  viz.num_ops = raw_graph->NumSchedulingUnits();
  viz.occupancy = occupancy;
  viz.raw = ComputeLensViz(*raw_graph, st, mf, occupancy);
  viz.adjusted = ComputeLensViz(*adj_graph, st, mf, occupancy);

  // Register pressure is a property of the ordering, not the latency lens, so
  // report it once (from the raw graph). Peak vs the occupancy budget: >100%
  // would drop a rung, which the hierarchical scheduler disallows.
  const GCNRegisterTracker &rt =
      raw_graph->GetInputScheduleConstructor().GetPressureTracker();
  viz.peak_vgpr = static_cast<int>(rt.GetPeakVGPRNum());
  viz.peak_sgpr = static_cast<int>(rt.GetPeakSGPRNum());
  viz.vgpr_budget = static_cast<int>(st.getMaxNumVGPRs(occupancy));
  viz.sgpr_budget =
      static_cast<int>(st.getMaxNumSGPRs(occupancy, /*Addressable=*/true));

  raw_ostream &os = outs();
  os << "\tregion[" << region_index
     << "] schedule-length analysis: ops=" << viz.num_ops
     << " occ=" << occupancy << "  peakVGPR=" << viz.peak_vgpr << "/"
     << viz.vgpr_budget << format(" (%.0f%%)", Percent(viz.peak_vgpr, viz.vgpr_budget))
     << "  peakSGPR=" << viz.peak_sgpr << "/" << viz.sgpr_budget
     << format(" (%.0f%%)", Percent(viz.peak_sgpr, viz.sgpr_budget)) << "\n";
  PrintLensReport(os, "raw     ", viz.raw);
  PrintLensReport(os, "adjusted", viz.adjusted);

  WriteCsvRows(mf.getName(), viz);
  return viz;
}

void ScheduleLengthAnalyzer::WriteVizJson(
    const MachineFunction &mf, StringRef scheduler_name,
    const std::map<int, std::vector<RegionViz>> &regions_by_block) {
  const char *kRootDir = "schedule_length_viz";
  if (std::error_code ec = sys::fs::create_directories(kRootDir)) {
    report_fatal_error(Twine("ScheduleLengthAnalyzer: cannot create '") +
                       kRootDir + "': " + ec.message());
  }
  std::string path =
      std::string(kRootDir) + "/" + SanitizeForFilename(mf.getName()) + ".json";
  std::error_code ec;
  raw_fd_ostream os(path, ec, sys::fs::OF_Text);
  if (ec) {
    report_fatal_error(Twine("ScheduleLengthAnalyzer: cannot open '") + path +
                       "': " + ec.message());
  }

  os << "{\n  \"function\": \"" << JsonEscape(mf.getName())
     << "\",\n  \"scheduler\": \"" << JsonEscape(scheduler_name)
     << "\",\n  \"blocks\": [\n";
  bool first_block = true;
  for (const MachineBasicBlock &mbb : mf) {
    if (!first_block) {
      os << ",\n";
    }
    first_block = false;
    os << "    {\"number\": " << mbb.getNumber() << ", \"name\": \""
       << JsonEscape(mbb.getName()) << "\", \"succs\": [";
    bool first_succ = true;
    for (const MachineBasicBlock *succ : mbb.successors()) {
      os << (first_succ ? "" : ", ") << succ->getNumber();
      first_succ = false;
    }
    os << "], \"regions\": [";
    auto it = regions_by_block.find(mbb.getNumber());
    if (it != regions_by_block.end() && !it->second.empty()) {
      for (size_t i = 0; i < it->second.size(); ++i) {
        os << (i ? ",\n" : "\n");
        WriteRegion(os, it->second[i], "      ");
      }
      os << "\n    ]}";
    } else {
      os << "]}";
    }
  }
  os << "\n  ]\n}\n";
}
