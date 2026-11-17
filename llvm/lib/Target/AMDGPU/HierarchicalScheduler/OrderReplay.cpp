//===- OrderReplay.cpp - Replay an instruction order from a file ---------===//
//
// Implementation of order file reading and matching. See OrderReplay.h for
// the file format.
//
//===----------------------------------------------------------------------===//

#include "OrderReplay.h"
#include "HierarchicalConfig.h"
#include "InstrText.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/Twine.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/JSON.h"
#include "llvm/Support/MemoryBuffer.h"
#include <memory>
#include <optional>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Stop the compile with an error about the order file at `path`.
[[noreturn]] void FailOrderFile(const std::string &path, const Twine &problem) {
  report_fatal_error(Twine("order file '") + path + "' " + problem);
}

// Read one entry of the "regions" array into `order_file`. Fatal if it isn't
// an object with an integer "index" not listed before, an integer "block",
// and an "instrs" array of strings.
void ReadRegion(const json::Value &value, OrderFile &order_file) {
  const json::Object *object = value.getAsObject();
  if (object == nullptr) {
    FailOrderFile(order_file.path, "has a region that isn't a JSON object");
  }

  // The region's fields.
  std::optional<int64_t> index = object->getInteger("index");
  std::optional<int64_t> block = object->getInteger("block");
  const json::Array *instrs = object->getArray("instrs");
  if (!index.has_value() || !block.has_value() || instrs == nullptr) {
    FailOrderFile(order_file.path,
                  "has a region without an integer \"index\", an integer "
                  "\"block\" and an \"instrs\" array");
  }

  // Its instruction texts, in the order to replay.
  OrderFileRegion region;
  region.block_number = static_cast<int>(*block);
  for (const json::Value &instr : *instrs) {
    std::optional<StringRef> text = instr.getAsString();
    if (!text.has_value()) {
      FailOrderFile(order_file.path, Twine("region ") + Twine(*index) +
                                         " has an instruction that isn't a "
                                         "string");
    }
    region.instr_texts.push_back(text->str());
  }

  // Store it under its index.
  const bool inserted =
      order_file.regions.emplace(static_cast<int>(*index), std::move(region))
          .second;
  if (!inserted) {
    FailOrderFile(order_file.path,
                  Twine("lists region ") + Twine(*index) + " twice");
  }
}

// Read one order file whole. Fatal if it can't be read, isn't valid JSON, or
// doesn't follow the format in OrderReplay.h.
OrderFile ReadOrderFile(const std::string &path) {
  OrderFile order_file;
  order_file.path = path;

  // Read the whole file.
  ErrorOr<std::unique_ptr<MemoryBuffer>> buffer = MemoryBuffer::getFile(path);
  if (!buffer) {
    FailOrderFile(path, Twine("can't be read: ") +
                            buffer.getError().message());
  }

  // Parse it as JSON.
  Expected<json::Value> parsed = json::parse((*buffer)->getBuffer());
  if (!parsed) {
    FailOrderFile(path, Twine("is not valid JSON: ") +
                            toString(parsed.takeError()));
  }

  // Take the top-level "function" string and "regions" array.
  std::optional<StringRef> function_name;
  const json::Array *regions = nullptr;
  const json::Object *top = parsed->getAsObject();
  if (top != nullptr) {
    function_name = top->getString("function");
    regions = top->getArray("regions");
  }
  if (!function_name.has_value()) {
    FailOrderFile(path, "has no \"function\" string");
  }
  if (regions == nullptr) {
    FailOrderFile(path, "has no \"regions\" array");
  }
  order_file.function_name = function_name->str();

  // Read each region.
  for (const json::Value &region : *regions) {
    ReadRegion(region, order_file);
  }

  return order_file;
}

// Map each function an order file names to that file. Fatal if two files name
// the same function.
StringMap<OrderFile> BuildOrderFilesByFunction() {
  StringMap<OrderFile> order_files_by_function;

  for (const std::string &path : HierarchicalConfig::Get().order_files) {
    OrderFile order_file = ReadOrderFile(path);

    auto [existing, inserted] = order_files_by_function.try_emplace(
        order_file.function_name, order_file);
    if (!inserted) {
      report_fatal_error(Twine("order files '") + existing->second.path +
                         "' and '" + path + "' both name function '" +
                         order_file.function_name + "'");
    }
  }

  return order_files_by_function;
}

// The map from BuildOrderFilesByFunction, built on first use and kept for the
// whole compile.
const StringMap<OrderFile> &OrderFilesByFunction() {
  static const StringMap<OrderFile> order_files_by_function =
      BuildOrderFilesByFunction();
  return order_files_by_function;
}

// Stop the compile if `order`, the file's order for region `region_index`,
// puts an instruction above one of its predecessors. `order` must hold each
// of the region's SUnits exactly once. The region's graph would reject such an
// order too, when the caller schedules it, but its error can't name the
// instructions.
void CheckOrderKeepsDependences(const OrderFile &order_file, int region_index,
                                const std::vector<SUnit *> &order) {
  // Each instruction's position in the order, indexed by NodeNum
  // (buildSchedGraph numbers a region's SUnits 0, 1, ...).
  std::vector<int> position_in_order(order.size());
  for (int position = 0; position < static_cast<int>(order.size());
       ++position) {
    position_in_order[order[position]->NodeNum] = position;
  }

  // Every instruction must come after its predecessors. Edges are skipped
  // like the graph skips them: weak edges, which the scheduler doesn't wait
  // on, and edges from the region's boundary.
  for (SUnit *sunit : order) {
    for (const SDep &edge : sunit->Preds) {
      const SUnit *predecessor = edge.getSUnit();
      if (edge.isWeak() || predecessor->isBoundaryNode()) {
        continue;
      }

      if (position_in_order[predecessor->NodeNum] >
          position_in_order[sunit->NodeNum]) {
        FailOrderFile(order_file.path,
                      Twine("region ") + Twine(region_index) + " puts '" +
                          InstrText(*sunit->getInstr()) + "' above '" +
                          InstrText(*predecessor->getInstr()) +
                          "', which it depends on");
      }
    }
  }
}

} // namespace

const OrderFile *
hierarchical_scheduler::FindOrderFile(StringRef function_name) {
  const StringMap<OrderFile> &order_files_by_function = OrderFilesByFunction();

  auto found = order_files_by_function.find(function_name);
  if (found == order_files_by_function.end()) {
    return nullptr;
  }
  return &found->second;
}

std::vector<SUnit *> hierarchical_scheduler::ComputeReplayOrder(
    const OrderFile &order_file, int region_index, int block_number,
    std::vector<SUnit> &sunits) {
  // The file's region with this index, which must be in the same block.
  auto found = order_file.regions.find(region_index);
  if (found == order_file.regions.end()) {
    FailOrderFile(order_file.path,
                  Twine("has no region ") + Twine(region_index));
  }
  const OrderFileRegion &file_region = found->second;
  if (file_region.block_number != block_number) {
    FailOrderFile(order_file.path,
                  Twine("puts region ") + Twine(region_index) + " in block " +
                      Twine(file_region.block_number) +
                      ", but the compiler's region " + Twine(region_index) +
                      " is in block " + Twine(block_number));
  }

  // The region's instructions grouped by text, each group in the region's
  // current order (buildSchedGraph numbers SUnits in that order).
  StringMap<std::vector<SUnit *>> sunits_by_text;
  for (SUnit &sunit : sunits) {
    sunits_by_text[InstrText(*sunit.getInstr())].push_back(&sunit);
  }

  // Pair each text in the file with the first unpaired instruction that has
  // that text. num_paired counts, per text, the instructions paired so far.
  StringMap<int> num_paired;
  std::vector<SUnit *> order;
  for (const std::string &text : file_region.instr_texts) {
    auto group = sunits_by_text.find(text);
    int &paired = num_paired[text];
    if (group == sunits_by_text.end() ||
        paired >= static_cast<int>(group->second.size())) {
      FailOrderFile(order_file.path,
                    Twine("region ") + Twine(region_index) +
                        " has an instruction the compiler's region doesn't, "
                        "or has it more times: " + text);
    }
    order.push_back(group->second[paired]);
    ++paired;
  }

  // Each text took a different instruction, so if the counts match, every
  // instruction is in the order exactly once.
  if (order.size() != sunits.size()) {
    FailOrderFile(order_file.path,
                  Twine("region ") + Twine(region_index) + " has " +
                      Twine(order.size()) +
                      " instructions, but the compiler's region has " +
                      Twine(sunits.size()));
  }

  // The order must keep every dependence.
  CheckOrderKeepsDependences(order_file, region_index, order);

  return order;
}
