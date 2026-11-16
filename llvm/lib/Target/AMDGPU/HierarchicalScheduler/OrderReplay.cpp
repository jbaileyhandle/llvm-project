//===- OrderReplay.cpp - Replay an instruction order from a file ---------===//
//
// Implementation of order file reading. See OrderReplay.h for the file
// format.
//
//===----------------------------------------------------------------------===//

#include "OrderReplay.h"
#include "HierarchicalConfig.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/JSON.h"
#include "llvm/Support/MemoryBuffer.h"
#include <memory>
#include <optional>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Read one order file and return the function it names. Fatal if the file
// can't be read, isn't valid JSON, or has no top-level "function" string.
std::string ReadFunctionName(const std::string &path) {
  // Read the whole file.
  ErrorOr<std::unique_ptr<MemoryBuffer>> buffer = MemoryBuffer::getFile(path);
  if (!buffer) {
    report_fatal_error(Twine("order file '") + path +
                       "' can't be read: " + buffer.getError().message());
  }

  // Parse it as JSON.
  Expected<json::Value> parsed = json::parse((*buffer)->getBuffer());
  if (!parsed) {
    report_fatal_error(Twine("order file '") + path +
                       "' is not valid JSON: " + toString(parsed.takeError()));
  }

  // Take the top-level "function" string.
  std::optional<StringRef> function_name;
  const json::Object *top = parsed->getAsObject();
  if (top != nullptr) {
    function_name = top->getString("function");
  }
  if (!function_name.has_value()) {
    report_fatal_error(Twine("order file '") + path +
                       "' has no \"function\" string");
  }

  return function_name->str();
}

// Map each function an order file names to that file's path. Fatal if two
// files name the same function.
StringMap<std::string> BuildOrderFilesByFunction() {
  StringMap<std::string> order_files_by_function;

  for (const std::string &path : HierarchicalConfig::Get().order_files) {
    std::string function_name = ReadFunctionName(path);

    auto [existing, inserted] =
        order_files_by_function.try_emplace(function_name, path);
    if (!inserted) {
      report_fatal_error(Twine("order files '") + existing->second +
                         "' and '" + path + "' both name function '" +
                         function_name + "'");
    }
  }

  return order_files_by_function;
}

// The map from BuildOrderFilesByFunction, built on first use and kept for the
// whole compile.
const StringMap<std::string> &OrderFilesByFunction() {
  static const StringMap<std::string> order_files_by_function =
      BuildOrderFilesByFunction();
  return order_files_by_function;
}

} // namespace

const std::string *
hierarchical_scheduler::FindOrderFile(StringRef function_name) {
  const StringMap<std::string> &order_files_by_function =
      OrderFilesByFunction();

  auto found = order_files_by_function.find(function_name);
  if (found == order_files_by_function.end()) {
    return nullptr;
  }
  return &found->second;
}
