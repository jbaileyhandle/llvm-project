//===- InstrText.cpp - One-line text of a MachineInstr --------------------===//
//
// See InstrText.h.
//
//===----------------------------------------------------------------------===//

#include "InstrText.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

std::string hierarchical_scheduler::InstrText(const MachineInstr &mi) {
  std::string text;
  raw_string_ostream os(text);
  mi.print(os, /*IsStandalone=*/true, /*SkipOpers=*/false,
           /*SkipDebugLoc=*/true, /*AddNewLine=*/false);
  os.flush();
  for (char &c : text) {
    if (c == '\n' || c == '\t' || c == '\r') {
      c = ' ';
    }
  }
  const size_t begin = text.find_first_not_of(' ');
  const size_t end = text.find_last_not_of(' ');
  if (begin == std::string::npos) {
    return std::string();
  }
  return text.substr(begin, end - begin + 1);
}
