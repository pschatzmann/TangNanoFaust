#pragma once
// Parser for Faust's textual interpreter bytecode (FBC), as written by
// `faust -lang interp`.
//
// An FBC file is a header followed by named blocks. Every block is
// `block_size N` followed by N instructions; kLoop, kIf, kSelectReal and
// kSelectInt are followed by their own sub-blocks (loop: init + body,
// if/select: then + else), which are not counted in the enclosing block's N.
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <utility>
#include <vector>

namespace tangnanofaust {
namespace compiler {

/// FBC instructions this compiler knows (everything else is rejected).
enum FbcOp : uint8_t {
  kRealValue, kInt32Value, kLoadReal, kLoadInt, kStoreReal, kStoreInt,
  kStoreRealValue, kStoreIntValue, kLoadIndexedReal, kLoadIndexedInt,
  kStoreIndexedReal, kStoreIndexedInt, kLoadInput, kStoreOutput, kCastReal,
  kCastInt, kBitcastInt, kBitcastReal, kAddReal, kAddInt, kSubReal, kSubInt,
  kMultReal, kMultInt, kDivReal, kDivInt, kRemReal, kRemInt, kLshInt, kARshInt,
  kLRshInt, kGTInt, kLTInt, kGEInt, kLEInt, kEQInt, kNEInt, kGTReal, kLTReal,
  kGEReal, kLEReal, kEQReal, kNEReal, kANDInt, kORInt, kXORInt, kAbs, kAbsf,
  kAcosf, kAcoshf, kAsinf, kAsinhf, kAtanf, kAtanhf, kCeilf, kCosf, kCoshf,
  kExpf, kFloorf, kLogf, kLog10f, kRintf, kRoundf, kSinf, kSinhf, kSqrtf, kTanf,
  kTanhf, kIsnanf, kIsinff, kAtan2f, kFmodf, kPowf, kMax, kMaxf, kMin, kMinf,
  kCopysignf, kLoop, kReturn, kIf, kSelectReal, kSelectInt, kCondBranch, kNop,
  // user interface
  kOpenVerticalBox, kOpenHorizontalBox, kOpenTabBox, kCloseBox, kAddButton,
  kAddCheckButton, kAddHorizontalSlider, kAddVerticalSlider, kAddNumEntry,
  kAddSoundfile, kAddHorizontalBargraph, kAddVerticalBargraph, kDeclare,
  kFbcOpCount
};

static const char *const kFbcOpNames[kFbcOpCount] = {
  "kRealValue", "kInt32Value", "kLoadReal", "kLoadInt", "kStoreReal", "kStoreInt",
  "kStoreRealValue", "kStoreIntValue", "kLoadIndexedReal", "kLoadIndexedInt",
  "kStoreIndexedReal", "kStoreIndexedInt", "kLoadInput", "kStoreOutput", "kCastReal",
  "kCastInt", "kBitcastInt", "kBitcastReal", "kAddReal", "kAddInt", "kSubReal", "kSubInt",
  "kMultReal", "kMultInt", "kDivReal", "kDivInt", "kRemReal", "kRemInt", "kLshInt", "kARshInt",
  "kLRshInt", "kGTInt", "kLTInt", "kGEInt", "kLEInt", "kEQInt", "kNEInt", "kGTReal", "kLTReal",
  "kGEReal", "kLEReal", "kEQReal", "kNEReal", "kANDInt", "kORInt", "kXORInt", "kAbs", "kAbsf",
  "kAcosf", "kAcoshf", "kAsinf", "kAsinhf", "kAtanf", "kAtanhf", "kCeilf", "kCosf", "kCoshf",
  "kExpf", "kFloorf", "kLogf", "kLog10f", "kRintf", "kRoundf", "kSinf", "kSinhf", "kSqrtf", "kTanf",
  "kTanhf", "kIsnanf", "kIsinff", "kAtan2f", "kFmodf", "kPowf", "kMax", "kMaxf", "kMin", "kMinf",
  "kCopysignf", "kLoop", "kReturn", "kIf", "kSelectReal", "kSelectInt", "kCondBranch", "kNop",
  "kOpenVerticalBox", "kOpenHorizontalBox", "kOpenTabBox", "kCloseBox", "kAddButton",
  "kAddCheckButton", "kAddHorizontalSlider", "kAddVerticalSlider", "kAddNumEntry",
  "kAddSoundfile", "kAddHorizontalBargraph", "kAddVerticalBargraph", "kDeclare",
};

struct Block;

struct Instr {
  FbcOp op = kNop;
  int32_t intValue = 0;
  double realValue = 0;
  int32_t offset1 = -1;
  int32_t offset2 = -1;
  std::vector<Block> branches;
};

struct Block {
  std::vector<Instr> instrs;
};

struct UIItem {
  FbcOp op;
  int32_t offset;
  std::string label, key, value;
  double init, min, max, step;
};

enum BlockId { kStaticInit, kConstants, kResetUI, kClear, kControl, kDsp, kBlockCount };

static const char *const kBlockNames[kBlockCount] = {
  "static_init_block", "constants_block", "reset_ui", "clear_block", "control_block", "dsp_block",
};

struct FbcProgram {
  std::string name = "dsp";
  int inputs = 0, outputs = 0;
  int intHeapSize = 0, realHeapSize = 0;
  int srOffset = 0, countOffset = 0, iotaOffset = -1;
  std::vector<UIItem> ui;
  Block blocks[kBlockCount];
  bool hasBlock[kBlockCount] = {false, false, false, false, false, false};
};

/// Parses FBC text. Returns false and sets `error` on failure.
class FbcParser {
 public:
  bool parse(const char *text, FbcProgram &out, std::string &error) {
    p_ = text;
    error_.clear();
    while (*p_ && error_.empty()) {
      std::string line = nextLine();
      if (line.empty()) continue;
      if (line == "meta_block") {
        int n = blockSize();
        for (int i = 0; i < n && error_.empty(); i++) nextLine();
      } else if (line == "user_interface_block") {
        parseUI(out);
      } else {
        bool isBlock = false;
        for (int b = 0; b < kBlockCount; b++) {
          if (line == kBlockNames[b]) {
            parseBlock(out.blocks[b]);
            out.hasBlock[b] = true;
            isBlock = true;
          }
        }
        if (!isBlock) header(line, out);
      }
    }
    if (error_.empty() && !out.hasBlock[kDsp]) error_ = "no dsp_block in FBC input";
    error = error_;
    return error_.empty();
  }

 protected:
  const char *p_ = nullptr;
  std::string error_;

  std::string nextLine() {
    const char *e = strchr(p_, '\n');
    if (!e) e = p_ + strlen(p_);
    std::string line(p_, e - p_);
    p_ = *e ? e + 1 : e;
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
    return line;
  }

  // Splits a line into tokens; "quoted strings" (with \ escapes) are one token.
  static std::vector<std::string> tokens(const std::string &line) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < line.size()) {
      while (i < line.size() && line[i] == ' ') i++;
      if (i >= line.size()) break;
      std::string tok;
      if (line[i] == '"') {
        i++;
        while (i < line.size() && line[i] != '"') {
          if (line[i] == '\\' && i + 1 < line.size()) i++;
          tok += line[i++];
        }
        i++;
      } else {
        while (i < line.size() && line[i] != ' ') tok += line[i++];
      }
      out.push_back(tok);
    }
    return out;
  }

  static int lookup(const std::string &name) {
    for (int i = 0; i < kFbcOpCount; i++)
      if (name == kFbcOpNames[i]) return i;
    return -1;
  }

  int blockSize() {
    std::string line = nextLine();
    if (line.compare(0, 11, "block_size ") != 0) {
      error_ = "expected block_size, got: " + line;
      return 0;
    }
    return atoi(line.c_str() + 11);
  }

  void parseBlock(Block &block) {
    int n = blockSize();
    block.instrs.reserve(n);
    for (int i = 0; i < n && error_.empty(); i++) {
      std::string line = nextLine();
      std::vector<std::string> t = tokens(line);
      // opcode N kName int I real R offset1 O1 offset2 O2 [name X]
      if (t.size() < 11 || t[0] != "opcode") {
        error_ = "cannot parse instruction: " + line;
        return;
      }
      int op = lookup(t[2]);
      if (op < 0) {
        error_ = "unsupported FBC instruction " + t[2];
        return;
      }
      Instr ins;
      ins.op = (FbcOp)op;
      ins.intValue = (int32_t)strtol(t[4].c_str(), nullptr, 10);
      ins.realValue = strtod(t[6].c_str(), nullptr);
      ins.offset1 = (int32_t)strtol(t[8].c_str(), nullptr, 10);
      ins.offset2 = (int32_t)strtol(t[10].c_str(), nullptr, 10);
      int subs = (op == kLoop || op == kIf || op == kSelectReal || op == kSelectInt) ? 2 : 0;
      ins.branches.resize(subs);
      for (int b = 0; b < subs && error_.empty(); b++) parseBlock(ins.branches[b]);
      block.instrs.push_back(std::move(ins));
    }
  }

  void parseUI(FbcProgram &out) {
    int n = blockSize();
    for (int i = 0; i < n && error_.empty(); i++) {
      std::vector<std::string> t = tokens(nextLine());
      // opcode N kName offset O label L key K value V init I min A max B step S
      if (t.size() < 3) continue;
      int op = lookup(t[2]);
      if (op < 0) {
        error_ = "unsupported UI element " + t[2];
        return;
      }
      UIItem item{(FbcOp)op, -1, "", "", "", 0, 0, 0, 0};
      for (size_t k = 3; k + 1 < t.size(); k += 2) {
        const std::string &key = t[k];
        const std::string &v = t[k + 1];
        if (key == "offset") item.offset = atoi(v.c_str());
        else if (key == "label") item.label = v;
        else if (key == "key") item.key = v;
        else if (key == "value") item.value = v;
        else if (key == "init") item.init = strtod(v.c_str(), nullptr);
        else if (key == "min") item.min = strtod(v.c_str(), nullptr);
        else if (key == "max") item.max = strtod(v.c_str(), nullptr);
        else if (key == "step") item.step = strtod(v.c_str(), nullptr);
      }
      out.ui.push_back(item);
    }
  }

  void header(const std::string &line, FbcProgram &out) {
    std::vector<std::string> t = tokens(line);
    if (t.empty()) return;
    if (t[0] == "name") {
      out.name = line.size() > 5 ? line.substr(5) : "dsp";
    } else if (t[0] == "inputs" && t.size() >= 4) {
      out.inputs = atoi(t[1].c_str());
      out.outputs = atoi(t[3].c_str());
    } else if (t[0] == "int_heap_size") {
      for (size_t i = 0; i + 1 < t.size(); i += 2) {
        int v = atoi(t[i + 1].c_str());
        if (t[i] == "int_heap_size") out.intHeapSize = v;
        else if (t[i] == "real_heap_size") out.realHeapSize = v;
        else if (t[i] == "sr_offset") out.srOffset = v;
        else if (t[i] == "count_offset") out.countOffset = v;
        else if (t[i] == "iota_offset") out.iotaOffset = v;
      }
    }
  }
};

}  // namespace compiler
}  // namespace tangnanofaust
