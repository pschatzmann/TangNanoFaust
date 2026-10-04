#pragma once
// FBC -> TangNanoFaust DSP core compiler, header-only C++ so the same code
// runs in the faust2tang command line tool (tools/) and on a
// microcontroller: the MCU compiles Faust interpreter bytecode (`faust
// -lang interp -double` output) and uploads the result to the FPGA with
// TangNanoFaust::load() (see TangNanoFaustCompiler.h).
//
// Memory: one 32-bit word address space; [0, fastWords) is block RAM, from
// kSdramBase on the board's SDRAM. Scalars and small arrays go to block
// RAM, arrays that don't fit go to SDRAM, smallest first.
//
// Program layout:
//   0          control block (re-run only after a parameter changed)
//   dspEntry   dsp block (one sample), HALT
//   bootEntry  sample rate + Faust's init blocks, HALT -- run once after reset
//   ...        math library routines (MathLib.h)
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "Fbc.h"
#include "IsaOpcodes.h"
#include "MathLib.h"

namespace tangnanofaust {
namespace compiler {

static const uint32_t kSdramBase = 0x00800000;
static const uint32_t kSdramWords = 0x00200000;  // 8MB
static const int kScratchReserve = 128;          // block RAM kept free for math scratch

// ------------------------------------------------------------------ floats

/// Round sign * (mant + sticky) * 2^exp2 to binary32 bits: round to nearest
/// even at 24 bits, then flush below-normal results to zero (fp32.round_pack).
inline uint32_t roundPack(bool sign, uint64_t mant, int exp2) {
  uint32_t s = sign ? 0x80000000u : 0;
  if (mant == 0) return s;
  int len = 64;
  while (!(mant >> (len - 1))) len--;
  int shift = len - 24;
  uint64_t keep;
  if (shift > 0) {
    keep = mant >> shift;
    bool guard = (mant >> (shift - 1)) & 1;
    bool sticky = shift > 1 && (mant & ((1ull << (shift - 1)) - 1)) != 0;
    if (guard && (sticky || (keep & 1))) {
      keep++;
      if (keep == (1ull << 24)) {
        keep >>= 1;
        shift++;
      }
    }
  } else {
    keep = mant << -shift;
  }
  int biased = exp2 + shift + 23 + 127;
  if (biased >= 255) return s | 0x7F800000u;
  if (biased <= 0) return s;
  return s | ((uint32_t)biased << 23) | (uint32_t)(keep & 0x7FFFFF);
}

/// double -> float32 bits with the core's rounding rule.
inline uint32_t fromFloat(double v) {
  if (v != v) return 0x7FC00000u;
  bool sign = signbit(v);
  if (isinf(v)) return (sign ? 0x80000000u : 0) | 0x7F800000u;
  int e;
  double m = frexp(fabs(v), &e);  // v = m * 2^e, 0.5 <= m < 1
  uint64_t mant = (uint64_t)ldexp(m, 53);
  return roundPack(sign, mant, e - 53);
}

// ------------------------------------------------------------------ results

struct Param {
  int index;
  std::string label, path, kind;
  uint32_t addr;
  double init, min, max, step;
  std::vector<std::pair<std::string, std::string>> meta;
};

/// Everything the FPGA needs to run a compiled Faust program.
struct ProgramImage {
  std::string name;
  int inputs = 0, outputs = 0;
  uint32_t sampleRate = 48000;
  uint32_t dspEntry = 0, bootEntry = 0;
  uint32_t fastWords = 0;   ///< block RAM words used
  uint32_t sdramWords = 0;  ///< SDRAM words used (0: no SDRAM needed)
  std::vector<uint64_t> program;  ///< 40-bit instructions
  std::string descriptor;         ///< parameter descriptor text
  std::vector<Param> params;
  std::vector<std::string> routines;  ///< math library routines linked in
  uint8_t minProtocol = 3;  ///< bitstream protocol version the program needs
};

struct CompileOptions {
  uint32_t sampleRate = 48000;
  uint32_t fastWords = 16384;  ///< block RAM words for the heap
  bool allowSdram = true;
  /// Fused instructions (TEE, operand forms of binary operators). Off only
  /// for debugging; constant array indices are always folded.
  bool fuse = true;
  /// Faster sin/cos (MathLib.h sin_fast/cos_fast): 1.9e-7 max error for
  /// moderate arguments, about 4x faster.
  bool fastMath = false;
};

// ------------------------------------------------------------------ assembler

class Asm {
 public:
  std::vector<AsmItem> items;

  std::string newLabel(const char *hint) { return std::string(hint) + std::to_string(++n_); }
  void label(const std::string &name) { items.push_back({true, 0, 0, 0, name}); }
  void op(uint8_t code, uint32_t imm = 0) { items.push_back({false, code, 0, imm, ""}); }
  void opLabel(uint8_t code, const std::string &l) { items.push_back({false, code, 1, 0, l}); }

 protected:
  int n_ = 0;
};

// ------------------------------------------------------------------ compiler

class Compiler {
 public:
  /// Compiles parsed FBC. Returns false and sets `error` on failure.
  bool compile(const FbcProgram &prog, const CompileOptions &opt, ProgramImage &out,
               std::string &error) {
    error_.clear();
    prog_ = &prog;
    opt_ = opt;
    buildHeapMap();
    if (!error_.empty()) return fail(error);
    if (sdramUsed_ && !opt.allowSdram) {
      error_ = "the DSP needs " + std::to_string(sdramUsed_) + " words beyond the " +
               std::to_string(opt.fastWords) + " words of block RAM; enable SDRAM or reduce "
               "table/delay sizes";
      return fail(error);
    }

    asm_ = Asm();
    for (int i = 0; i < mathlib::kRoutineCount; i++) mathUsed_[i] = false;
    block(prog.blocks[kControl], nullptr);
    asm_.label("dsp_entry");
    block(prog.blocks[kDsp], nullptr);
    asm_.op(OP_HALT);
    asm_.label("boot_entry");
    asm_.op(OP_PUSH, opt.sampleRate);
    asm_.op(OP_STORE, map(false, prog.srOffset));
    asm_.op(OP_PUSH, 1);
    asm_.op(OP_STORE, map(false, prog.countOffset));
    const BlockId init[] = {kStaticInit, kConstants, kResetUI, kClear};
    for (BlockId b : init)
      if (prog.hasBlock[b]) block(prog.blocks[b], nullptr);
    asm_.op(OP_HALT);
    if (!error_.empty()) return fail(error);
    addMath();
    peephole(opt.fuse);

    std::vector<std::pair<std::string, uint32_t>> labels, scratch;
    link(out.program, labels, scratch);
    if (!error_.empty()) return fail(error);
    uint32_t fastTotal = fastUsed_ + (uint32_t)scratch.size();
    if (fastTotal > opt.fastWords) {
      error_ = "block RAM overflow: " + std::to_string(fastTotal) + " > " +
               std::to_string(opt.fastWords) + " words";
      return fail(error);
    }

    out.name = prog.name;
    out.inputs = prog.inputs;
    out.outputs = prog.outputs;
    out.sampleRate = opt.sampleRate;
    out.dspEntry = find(labels, "dsp_entry");
    out.bootEntry = find(labels, "boot_entry");
    out.fastWords = fastTotal;
    out.sdramWords = sdramUsed_;
    params(out.params);
    if (!error_.empty()) return fail(error);
    out.descriptor = descriptor(out);
    out.minProtocol = 3;  // FFLOOR (and the fused instructions)
    out.routines.clear();
    for (int i = 0; i < mathlib::kRoutineCount; i++)
      if (mathUsed_[i]) out.routines.push_back(mathlib::kRoutines[i].name);
    return true;
  }

  /// Memory layout of the last compile: (what, fbc offset, size, address).
  struct Region {
    std::string what;
    int offset, size;
    uint32_t addr;
  };
  const std::vector<Region> &layout() const { return layout_; }

  /// Descriptor text read by TangNanoFaust::begin() (output.descriptor()).
  static std::string descriptor(const ProgramImage &c) {
    std::string s = "F|1|" + clean(c.name) + "|" + std::to_string(c.inputs) + "|" +
                    std::to_string(c.outputs) + "|" + std::to_string(c.sampleRate) + "|" +
                    std::to_string(c.params.size()) + "\n";
    for (const Param &p : c.params) {
      std::string meta;
      for (size_t i = 0; i < p.meta.size(); i++) {
        if (i) meta += ",";
        meta += clean(p.meta[i].first) + ":" + clean(p.meta[i].second);
      }
      s += "P|" + p.kind + "|" + std::to_string(p.addr) + "|" + num(p.init) + "|" +
           num(p.min) + "|" + num(p.max) + "|" + num(p.step) + "|" + clean(p.path) + "|" +
           meta + "\n";
    }
    return s;
  }

 protected:
  struct Array {
    bool isReal;
    int base, size;
    uint32_t addr;
  };

  const FbcProgram *prog_ = nullptr;
  CompileOptions opt_;
  std::string error_;
  std::vector<Array> arrays_;           // in placement order
  std::vector<int> scalars_[2];         // uncovered offsets per kind (int, real), sorted
  uint32_t scalarBase_[2] = {0, 0};
  uint32_t fastUsed_ = 0, sdramUsed_ = 0;
  std::vector<Region> layout_;
  Asm asm_;
  bool mathUsed_[64];

  bool fail(std::string &error) {
    error = error_;
    return false;
  }

  static std::string num(double x) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.9g", x);
    return buf;
  }

  static std::string clean(const std::string &s) {
    std::string r = s;
    for (char &c : r) {
      if (c == '|') c = '/';
      if (c == '\n') c = ' ';
    }
    return r;
  }

  static uint32_t find(const std::vector<std::pair<std::string, uint32_t>> &v,
                       const std::string &key) {
    for (auto &kv : v)
      if (kv.first == key) return kv.second;
    return 0;
  }

  // ---------------------------------------------------------------- heap map

  static bool isIndexed(FbcOp op, bool &isReal) {
    switch (op) {
      case kLoadIndexedReal: case kStoreIndexedReal: isReal = true; return true;
      case kLoadIndexedInt: case kStoreIndexedInt: isReal = false; return true;
      default: return false;
    }
  }

  void scan(const Block &b) {
    for (const Instr &ins : b.instrs) {
      bool isReal;
      if (isIndexed(ins.op, isReal) && ins.offset2 > 0) {
        bool found = false;
        for (Array &a : arrays_)
          if (a.isReal == isReal && a.base == ins.offset1) {
            a.size = std::max(a.size, (int)ins.offset2);
            found = true;
          }
        if (!found) arrays_.push_back({isReal, ins.offset1, ins.offset2, 0});
      }
      for (const Block &sub : ins.branches) scan(sub);
    }
  }

  bool covered(bool isReal, int off) const {
    for (const Array &a : arrays_)
      if (a.isReal == isReal && off >= a.base && off < a.base + a.size) return true;
    return false;
  }

  void buildHeapMap() {
    arrays_.clear();
    layout_.clear();
    for (int b = 0; b < kBlockCount; b++)
      if (prog_->hasBlock[b]) scan(prog_->blocks[b]);
    uint32_t fast = 0;
    const int sizes[2] = {prog_->intHeapSize, prog_->realHeapSize};
    for (int k = 0; k < 2; k++) {
      scalars_[k].clear();
      scalarBase_[k] = fast;
      for (int off = 0; off < sizes[k]; off++)
        if (!covered(k == 1, off)) scalars_[k].push_back(off);
      fast += (uint32_t)scalars_[k].size();
    }
    layout_.push_back({"scalars", 0, (int)fast, 0});
    std::stable_sort(arrays_.begin(), arrays_.end(),
                     [](const Array &a, const Array &b) { return a.size < b.size; });
    uint32_t sdram = 0;
    for (Array &a : arrays_) {
      if (fast + a.size + kScratchReserve <= opt_.fastWords) {
        a.addr = fast;
        fast += a.size;
      } else {
        a.addr = kSdramBase + sdram;
        sdram += a.size;
      }
      layout_.push_back({a.isReal ? "real array" : "int array", a.base, a.size, a.addr});
    }
    if (sdram > kSdramWords)
      error_ = "the DSP needs " + std::to_string(sdram) + " SDRAM words, more than the 8MB SDRAM";
    fastUsed_ = fast;
    sdramUsed_ = sdram;
  }

  /// Address of FBC heap cell (int/real, offset).
  uint32_t map(bool isReal, int off) {
    for (const Array &a : arrays_)  // first array (placement order) wins, as setdefault
      if (a.isReal == isReal && off >= a.base && off < a.base + a.size)
        return a.addr + (off - a.base);
    const std::vector<int> &s = scalars_[isReal ? 1 : 0];
    auto it = std::lower_bound(s.begin(), s.end(), off);
    if (it == s.end() || *it != off) {
      if (error_.empty())
        error_ = std::string(isReal ? "real" : "int") + " heap offset " + std::to_string(off) +
                 " outside the heap";
      return 0;
    }
    return scalarBase_[isReal ? 1 : 0] + (uint32_t)(it - s.begin());
  }

  // ---------------------------------------------------------------- codegen

  static int directOp(FbcOp op) {
    switch (op) {
      case kAddReal: return OP_FADD; case kSubReal: return OP_FSUB;
      case kMultReal: return OP_FMUL; case kDivReal: return OP_FDIV;
      case kAddInt: return OP_ADD; case kSubInt: return OP_SUB;
      case kMultInt: return OP_MUL; case kDivInt: return OP_DIV;
      case kRemInt: return OP_REM; case kLshInt: return OP_SHL;
      case kARshInt: return OP_ASR; case kLRshInt: return OP_LSR;
      case kGTInt: return OP_GT; case kLTInt: return OP_LT; case kGEInt: return OP_GE;
      case kLEInt: return OP_LE; case kEQInt: return OP_EQ; case kNEInt: return OP_NE;
      case kGTReal: return OP_FGT; case kLTReal: return OP_FLT; case kGEReal: return OP_FGE;
      case kLEReal: return OP_FLE; case kEQReal: return OP_FEQ; case kNEReal: return OP_FNE;
      case kANDInt: return OP_AND; case kORInt: return OP_OR; case kXORInt: return OP_XOR;
      case kMax: return OP_MAX; case kMin: return OP_MIN; case kMaxf: return OP_FMAX;
      case kMinf: return OP_FMIN; case kAbs: return OP_ABS; case kAbsf: return OP_FABS;
      case kSqrtf: return OP_FSQRT; case kCastReal: return OP_I2F; case kCastInt: return OP_F2I;
      case kFloorf: return OP_FFLOOR;
      default: return -1;
    }
  }

  const char *mathName(FbcOp op) const {
    if (opt_.fastMath && op == kSinf) return "sin_fast";
    if (opt_.fastMath && op == kCosf) return "cos_fast";
    switch (op) {
      case kFloorf: return "floor"; case kCeilf: return "ceil"; case kRintf: return "rint";
      case kRoundf: return "round"; case kSinf: return "sin"; case kCosf: return "cos";
      case kTanf: return "tan"; case kExpf: return "exp"; case kLogf: return "log";
      case kLog10f: return "log10"; case kPowf: return "pow"; case kAtanf: return "atan";
      case kAtan2f: return "atan2"; case kAsinf: return "asin"; case kAcosf: return "acos";
      case kSinhf: return "sinh"; case kCoshf: return "cosh"; case kTanhf: return "tanh";
      case kAsinhf: return "asinh"; case kAcoshf: return "acosh"; case kAtanhf: return "atanh";
      case kFmodf: return "fmod"; case kRemReal: return "remainder";
      default: return nullptr;
    }
  }

  void useMath(const char *name) {
    for (int i = 0; i < mathlib::kRoutineCount; i++) {
      if (strcmp(mathlib::kRoutines[i].name, name) == 0) {
        if (mathUsed_[i]) return;
        mathUsed_[i] = true;
        // dependencies (space separated)
        std::string deps = mathlib::kRoutines[i].deps;
        size_t p = 0;
        while (p < deps.size()) {
          size_t e = deps.find(' ', p);
          if (e == std::string::npos) e = deps.size();
          if (e > p) useMath(deps.substr(p, e - p).c_str());
          p = e + 1;
        }
        return;
      }
    }
  }

  void block(const Block &b, const std::string *loopLabel) {
    Asm &a = asm_;
    for (const Instr &ins : b.instrs) {
      if (!error_.empty()) return;
      FbcOp n = ins.op;
      int direct = directOp(n);
      const char *math = mathName(n);
      switch (n) {
        case kRealValue: a.op(OP_PUSH, fromFloat(ins.realValue)); break;
        case kInt32Value: a.op(OP_PUSH, (uint32_t)ins.intValue); break;
        case kLoadReal: a.op(OP_LOAD, map(true, ins.offset1)); break;
        case kLoadInt: a.op(OP_LOAD, map(false, ins.offset1)); break;
        case kStoreReal: a.op(OP_STORE, map(true, ins.offset1)); break;
        case kStoreInt: a.op(OP_STORE, map(false, ins.offset1)); break;
        case kStoreRealValue:
          a.op(OP_PUSH, fromFloat(ins.realValue));
          a.op(OP_STORE, map(true, ins.offset1));
          break;
        case kStoreIntValue:
          a.op(OP_PUSH, (uint32_t)ins.intValue);
          a.op(OP_STORE, map(false, ins.offset1));
          break;
        case kLoadIndexedReal: a.op(OP_LOADX, map(true, ins.offset1)); break;
        case kLoadIndexedInt: a.op(OP_LOADX, map(false, ins.offset1)); break;
        case kStoreIndexedReal: a.op(OP_STOREX, map(true, ins.offset1)); break;
        case kStoreIndexedInt: a.op(OP_STOREX, map(false, ins.offset1)); break;
        case kLoadInput: a.op(OP_IN, (uint32_t)ins.offset1); break;
        case kStoreOutput: a.op(OP_OUT, (uint32_t)ins.offset1); break;
        case kBitcastInt: case kBitcastReal: case kNop: break;
        case kCopysignf:  // v1 = magnitude (top), v2 = sign source
          a.op(OP_PUSH, 0x7FFFFFFF); a.op(OP_AND); a.op(OP_SWAP);
          a.op(OP_PUSH, 0x80000000); a.op(OP_AND); a.op(OP_OR);
          break;
        case kIsnanf:
          a.op(OP_PUSH, 0x7FFFFFFF); a.op(OP_AND); a.op(OP_PUSH, 0x7F800000);
          a.op(OP_SWAP); a.op(OP_GT);
          break;
        case kIsinff:
          a.op(OP_PUSH, 0x7FFFFFFF); a.op(OP_AND); a.op(OP_PUSH, 0x7F800000); a.op(OP_EQ);
          break;
        case kLoop: {
          block(ins.branches[0], nullptr);
          std::string body = a.newLabel("loop");
          a.label(body);
          block(ins.branches[1], &body);
          break;
        }
        case kIf: case kSelectReal: case kSelectInt: {
          std::string other = a.newLabel("else");
          std::string end = a.newLabel("endif");
          a.opLabel(OP_JZ, other);
          block(ins.branches[0], loopLabel);
          a.opLabel(OP_JMP, end);
          a.label(other);
          block(ins.branches[1], loopLabel);
          a.label(end);
          break;
        }
        case kCondBranch:
          if (!loopLabel) {
            error_ = "kCondBranch outside of a loop";
            return;
          }
          a.opLabel(OP_JNZ, *loopLabel);
          break;
        case kReturn: return;
        default:
          if (direct >= 0) {
            a.op((uint8_t)direct);
          } else if (math) {
            useMath(math);
            a.opLabel(OP_CALL, std::string("fn_") + math);
          } else {
            error_ = std::string("unsupported FBC instruction ") + kFbcOpNames[n];
            return;
          }
      }
    }
  }

  void addMath() {
    for (int i = 0; i < mathlib::kRoutineCount; i++) {
      if (!mathUsed_[i]) continue;
      mathlib::Routine r = mathlib::kRoutines[i].build();
      asm_.items.insert(asm_.items.end(), r.items.begin(), r.items.end());
    }
  }

  /// Combines an instruction with the one emitted before it:
  ///   PUSH k; LOADX b  -> LOAD b+k      PUSH k; STOREX b -> STORE b+k
  /// and with `fuse`:
  ///   STORE a; LOAD a  -> TEE a         PUSH k; op -> op+0x40 k
  ///   LOAD a; op       -> op+0x80 a
  /// A label between the two (a jump target) prevents it.
  void peephole(bool fuse) {
    std::vector<AsmItem> out;
    out.reserve(asm_.items.size());
    for (const AsmItem &it : asm_.items) {
      if (!it.isLabel && !out.empty() && !out.back().isLabel) {
        AsmItem &prev = out.back();
        if (prev.op == OP_PUSH && prev.argKind == 0 && it.argKind == 0 &&
            (it.op == OP_LOADX || it.op == OP_STOREX)) {
          prev.op = it.op == OP_LOADX ? OP_LOAD : OP_STORE;
          prev.imm = it.imm + prev.imm;
          continue;
        }
        if (fuse && prev.op == OP_STORE && it.op == OP_LOAD && prev.argKind == it.argKind &&
            prev.imm == it.imm && prev.sym == it.sym) {
          prev.op = OP_TEE;
          continue;
        }
        if (fuse && it.argKind == 0 && isFusableBinary(it.op) &&
            (prev.op == OP_PUSH || prev.op == OP_LOAD)) {
          prev.op = uint8_t(it.op + (prev.op == OP_PUSH ? kFuseImm : kFuseMem));
          continue;
        }
      }
      out.push_back(it);
    }
    asm_.items.swap(out);
  }

  void link(std::vector<uint64_t> &words, std::vector<std::pair<std::string, uint32_t>> &labels,
            std::vector<std::pair<std::string, uint32_t>> &scratch) {
    uint32_t pc = 0;
    for (const AsmItem &it : asm_.items) {
      if (it.isLabel) labels.push_back({it.sym, pc});
      else pc++;
    }
    words.clear();
    words.reserve(pc);
    for (const AsmItem &it : asm_.items) {
      if (it.isLabel) continue;
      uint32_t arg = it.imm;
      if (it.argKind == 1) {
        bool found = false;
        for (auto &l : labels)
          if (l.first == it.sym) {
            arg = l.second;
            found = true;
            break;
          }
        if (!found) {
          error_ = "undefined label " + it.sym;
          return;
        }
      } else if (it.argKind == 2) {
        bool found = false;
        for (auto &s : scratch)
          if (s.first == it.sym) {
            arg = s.second;
            found = true;
            break;
          }
        if (!found) {
          arg = fastUsed_ + (uint32_t)scratch.size();
          scratch.push_back({it.sym, arg});
        }
      }
      words.push_back(((uint64_t)it.op << 32) | arg);
    }
  }

  // ---------------------------------------------------------------- params

  static const char *widgetKind(FbcOp op) {
    switch (op) {
      case kAddButton: return "button";
      case kAddCheckButton: return "checkbox";
      case kAddHorizontalSlider: return "hslider";
      case kAddVerticalSlider: return "vslider";
      case kAddNumEntry: return "nentry";
      case kAddHorizontalBargraph: return "hbargraph";
      case kAddVerticalBargraph: return "vbargraph";
      default: return nullptr;
    }
  }

  void params(std::vector<Param> &out) {
    out.clear();
    std::vector<std::string> path;
    std::vector<std::pair<int, std::vector<std::pair<std::string, std::string>>>> meta;
    for (const UIItem &item : prog_->ui) {
      if (item.op == kOpenVerticalBox || item.op == kOpenHorizontalBox ||
          item.op == kOpenTabBox) {
        path.push_back(item.label);
      } else if (item.op == kCloseBox) {
        if (!path.empty()) path.pop_back();
      } else if (item.op == kDeclare) {
        if (item.offset < 0) continue;
        std::vector<std::pair<std::string, std::string>> *m = nullptr;
        for (auto &e : meta)
          if (e.first == item.offset) m = &e.second;
        if (!m) {
          meta.push_back({item.offset, {}});
          m = &meta.back().second;
        }
        bool replaced = false;
        for (auto &kv : *m)
          if (kv.first == item.key) {
            kv.second = item.value;
            replaced = true;
          }
        if (!replaced) m->push_back({item.key, item.value});
      } else if (const char *kind = widgetKind(item.op)) {
        Param p;
        p.index = (int)out.size();
        p.label = item.label;
        p.path = "";
        for (size_t i = 1; i < path.size(); i++) p.path += "/" + path[i];
        p.path += "/" + item.label;
        p.kind = kind;
        p.addr = map(true, item.offset);
        p.init = item.init;
        p.min = item.min;
        p.max = item.max;
        p.step = item.step;
        if (item.op == kAddButton || item.op == kAddCheckButton) {
          p.min = 0;
          p.max = 1;
        }
        for (auto &e : meta)
          if (e.first == item.offset) p.meta = e.second;
        out.push_back(p);
      } else if (item.op == kAddSoundfile) {
        error_ = "soundfile is not supported";
        return;
      }
    }
  }
};

/// Convenience: parse + compile FBC text in one call.
inline bool compileFbc(const char *fbcText, const CompileOptions &opt, ProgramImage &out,
                       std::string &error) {
  FbcProgram prog;
  FbcParser parser;
  if (!parser.parse(fbcText, prog, error)) return false;
  Compiler c;
  return c.compile(prog, opt, out, error);
}

}  // namespace compiler
}  // namespace tangnanofaust
