#pragma once
// Bit-exact simulator of the DSP core (gateware/rtl/dsp_core.v), including
// its cycle count: the core's `cycles` register reports exactly what
// Machine::run() returns, and gateware/tb/tb_dsp_core.v checks both values
// and cycles sample by sample. faust2tang uses it for the cycle budget.
#include <stdint.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "Fp32.h"
#include "IsaOpcodes.h"

namespace nanotangfaust {
namespace compiler {

static const uint32_t kSimSdramBase = 0x00800000;
/// Cycles an SDRAM access waits for its acknowledge (tb_dsp_core.v's model;
/// a typical estimate for the real controller, whose latency varies).
static const int kSdramReadWait = 8;
static const int kSdramWriteWait = 26;

/// Block RAM (zero at power-up) plus sparse SDRAM (undefined at power-up,
/// modelled as 0xDEADBEEF so reading it before writing shows up in tests).
class Memory {
 public:
  explicit Memory(uint32_t fastWords) : fast_(fastWords, 0) {}
  uint32_t read(uint32_t a) const {
    if (a < kSimSdramBase) return a < fast_.size() ? fast_[a] : 0;
    auto it = sdram_.find(a);
    return it == sdram_.end() ? 0xDEADBEEFu : it->second;
  }
  void write(uint32_t a, uint32_t v) {
    if (a < kSimSdramBase) {
      if (a < fast_.size()) fast_[a] = v;
    } else {
      sdram_[a] = v;
    }
  }

 protected:
  std::vector<uint32_t> fast_;
  std::unordered_map<uint32_t, uint32_t> sdram_;
};

class Machine {
 public:
  static const int kStackDepth = 32;
  static const int kReturnDepth = 8;

  std::vector<uint32_t> inputs, outputs;
  Memory heap;
  int maxStack = 0;
  std::string error;
  /// Optional: cycles spent per opcode (index = opcode), accumulated by run().
  std::vector<long> *profile = nullptr;
  /// Optional: (pc, cycles) of every executed instruction, appended by run().
  std::vector<std::pair<uint32_t, uint32_t>> *trace = nullptr;
  /// Prototype knobs: FPU latency of a normal (non-special) FADD/FSUB and FMUL
  /// result (0: the current fpu.v, 5 and 3).
  int faddLatency = 0, fmulLatency = 0;
  /// Instruction prefetch (dsp_core.v): a block RAM load or memory operand
  /// is read during the previous instruction's last cycle and saves a cycle,
  /// unless that instruction wrote block RAM (one port); taken
  /// jumps/calls/returns and the start of a run cost a cycle.
  bool prefetch = true;

  Machine(const std::vector<uint64_t> &program, uint32_t fastWords, int nIn, int nOut)
      : inputs(nIn > 0 ? nIn : 1, 0), outputs(nOut > 0 ? nOut : 1, 0), heap(fastWords),
        prog_(program) {}

  /// Runs from `entry` until HALT. Returns the cycle count, or -1 on error.
  long run(uint32_t entry, long maxSteps = 1000000) {
    uint32_t pc = entry;
    std::vector<uint32_t> st;
    std::vector<uint32_t> rs;
    long cycles = prefetch ? 2 : 1;
    bool prevWroteBram = false;  // the previous instruction wrote block RAM
    for (long step = 0; step < maxSteps; step++) {
      if (pc >= prog_.size()) return fail("pc out of range");
      uint8_t op = (uint8_t)(prog_[pc] >> 32);
      uint32_t arg = (uint32_t)prog_[pc];
      pc++;
      long before = cycles;
      if (isFusedImm(op) || isFusedMem(op)) {
        uint8_t base = fusedBase(op);
        if (st.empty()) return fail("stack underflow");
        uint32_t v1 = isFusedImm(op) ? arg : heap.read(arg);
        cycles += baseCycles(base) + (isFusedMem(op) ? 1 : 0);
        if (isFusedMem(op) && arg >= kSimSdramBase) cycles += kSdramReadWait - 1;
        else if (isFusedMem(op) && prefetch && !prevWroteBram) cycles -= 1;
        prevWroteBram = false;
        if (isFpu(base)) cycles += latency(base, v1, st.back());
        st.back() = binary(base, v1, st.back());
        if (!error.empty()) return -1;
        if (profile) {
          if (profile->size() < 256) profile->resize(256, 0);
          (*profile)[op] += cycles - before;
        }
        if (trace) trace->push_back({pc - 1, (uint32_t)(cycles - before)});
        continue;
      }
      cycles += baseCycles(op);
      if (op == OP_LOAD && arg < kSimSdramBase && prefetch && !prevWroteBram) cycles -= 1;
      {
        uint32_t waddr = arg + ((op == OP_STOREX && !st.empty()) ? st.back() : 0);
        prevWroteBram = (op == OP_STORE || op == OP_STOREX || op == OP_TEE) && waddr < kSimSdramBase;
      }
      if (isFpu(op)) {
        if (st.empty()) return fail("stack underflow");
        cycles += latency(op, st.back(), st.size() > 1 ? st[st.size() - 2] : 0);
      } else if (op == OP_LOAD || op == OP_LOADX || op == OP_STORE || op == OP_STOREX ||
                 op == OP_TEE) {
        uint32_t addr = arg;
        if (op == OP_LOADX || op == OP_STOREX) {
          if (st.empty()) return fail("stack underflow");
          addr = arg + st.back();
        }
        if (addr >= kSimSdramBase)
          cycles += (op == OP_LOAD || op == OP_LOADX) ? kSdramReadWait - 1 : kSdramWriteWait;
      }

      if (profile) {
        if (profile->size() < 256) profile->resize(256, 0);
        (*profile)[op] += cycles - before;
      }
      if (trace) trace->push_back({pc - 1, (uint32_t)(cycles - before)});
      size_t need = needs(op);
      if (st.size() < need) return fail("stack underflow");
      uint32_t a, b, i;
      switch (op) {
        case OP_NOP: break;
        case OP_PUSH: st.push_back(arg); break;
        case OP_LOAD: st.push_back(heap.read(arg)); break;
        case OP_STORE: heap.write(arg, pop(st)); break;
        case OP_TEE: heap.write(arg, st.back()); break;
        case OP_LOADX: st.back() = heap.read(arg + st.back()); break;
        case OP_STOREX:
          i = pop(st);
          heap.write(arg + i, pop(st));
          break;
        case OP_IN: st.back() = arg < inputs.size() ? inputs[arg] : 0; break;
        case OP_OUT:
          pop(st);
          a = pop(st);
          if (arg < outputs.size()) outputs[arg] = a;
          break;
        case OP_DUP: st.push_back(st.back()); break;
        case OP_DROP: st.pop_back(); break;
        case OP_SWAP: std::swap(st[st.size() - 1], st[st.size() - 2]); break;
        case OP_OVER: st.push_back(st[st.size() - 2]); break;
        case OP_JMP: pc = arg; cycles += prefetch; break;
        case OP_JZ: if (pop(st) == 0) { pc = arg; cycles += prefetch; } break;
        case OP_JNZ: if (pop(st) != 0) { pc = arg; cycles += prefetch; } break;
        case OP_CALL:
          rs.push_back(pc);
          if ((int)rs.size() > kReturnDepth) return fail("return stack overflow");
          pc = arg;
          cycles += prefetch;
          break;
        case OP_RET:
          if (rs.empty()) return fail("return stack underflow");
          pc = rs.back();
          rs.pop_back();
          cycles += prefetch;
          break;
        case OP_HALT:
          if (!st.empty()) return fail("stack not empty at HALT");
          return cycles;
        case OP_ABS: st.back() = (int32_t)st.back() < 0 ? 0u - st.back() : st.back(); break;
        case OP_FABS: st.back() = fp32::fabs_(st.back()); break;
        case OP_FNEG: st.back() = fp32::fneg(st.back()); break;
        case OP_I2F: st.back() = fp32::i2f(st.back()); break;
        case OP_F2I: st.back() = fp32::f2i(st.back()); break;
        case OP_FSQRT: st.back() = fp32::fsqrt(st.back()); break;
        case OP_FFLOOR: st.back() = fp32::ffloor(st.back()); break;
        default:
          a = pop(st);
          b = pop(st);
          st.push_back(binary(op, a, b));
          if (!error.empty()) return -1;
      }
      if ((int)st.size() > maxStack) {
        maxStack = (int)st.size();
        if (maxStack > kStackDepth) return fail("data stack overflow");
      }
    }
    return fail("program did not halt");
  }

  static int baseCycles(uint8_t op) {
    switch (op) {
      case OP_LOAD: case OP_LOADX: return 2;
      case OP_MUL: return 3;  // products registered (hmul.v)
      case OP_DIV: case OP_REM: return 35;
      case OP_HALT: return 0;
      default: return 1;
    }
  }

 protected:
  const std::vector<uint64_t> &prog_;

  long fail(const char *msg) {
    error = msg;
    return -1;
  }

  static uint32_t pop(std::vector<uint32_t> &st) {
    uint32_t v = st.back();
    st.pop_back();
    return v;
  }

  int latency(uint8_t op, uint32_t a, uint32_t b) const {
    int l = fp32::fpuLatency(op, a, b);
    if (faddLatency && (op == OP_FADD || op == OP_FSUB) && l == 5) l = faddLatency;
    if (fmulLatency && op == OP_FMUL && l == 3) l = fmulLatency;
    return l;
  }

  static bool isFpu(uint8_t op) {
    return op == OP_FADD || op == OP_FSUB || op == OP_FMUL || op == OP_FDIV || op == OP_FSQRT ||
           op == OP_I2F || op == OP_F2I || op == OP_FFLOOR;
  }

  static size_t needs(uint8_t op) {
    switch (op) {
      case OP_NOP: case OP_PUSH: case OP_LOAD: case OP_JMP: case OP_CALL: case OP_RET:
      case OP_HALT: return 0;
      case OP_STORE: case OP_TEE: case OP_LOADX: case OP_IN: case OP_DUP: case OP_DROP: case OP_JZ:
      case OP_JNZ: case OP_ABS: case OP_FABS: case OP_FNEG: case OP_I2F: case OP_F2I:
      case OP_FSQRT: case OP_FFLOOR: return 1;
      default: return 2;
    }
  }

  uint32_t binary(uint8_t op, uint32_t a, uint32_t b) {
    int32_t sa = (int32_t)a, sb = (int32_t)b;
    switch (op) {
      case OP_ADD: return a + b;
      case OP_SUB: return a - b;
      case OP_MUL: return a * b;
      case OP_DIV: {
        if (b == 0) return 0;
        uint32_t ma = sa < 0 ? 0u - a : a, mb = sb < 0 ? 0u - b : b;
        uint32_t q = ma / mb;
        return ((sa < 0) != (sb < 0)) ? 0u - q : q;
      }
      case OP_REM: {
        if (b == 0) return 0;
        uint32_t ma = sa < 0 ? 0u - a : a, mb = sb < 0 ? 0u - b : b;
        uint32_t r = ma % mb;
        return sa < 0 ? 0u - r : r;
      }
      case OP_SHL: return a << (b & 31);
      case OP_ASR: return (uint32_t)(sa >> (b & 31));
      case OP_LSR: return a >> (b & 31);
      case OP_GT: return sa > sb;
      case OP_LT: return sa < sb;
      case OP_GE: return sa >= sb;
      case OP_LE: return sa <= sb;
      case OP_EQ: return a == b;
      case OP_NE: return a != b;
      case OP_AND: return a & b;
      case OP_OR: return a | b;
      case OP_XOR: return a ^ b;
      case OP_MIN: return sb < sa ? b : a;
      case OP_MAX: return sa < sb ? b : a;
      case OP_FADD: return fp32::fadd(a, b);
      case OP_FSUB: return fp32::fsub(a, b);
      case OP_FMUL: return fp32::fmul(a, b);
      case OP_FDIV: return fp32::fdiv(a, b);
      case OP_FGT: return fp32::fgt(a, b);
      case OP_FLT: return fp32::flt(a, b);
      case OP_FGE: return fp32::fge(a, b);
      case OP_FLE: return fp32::fle(a, b);
      case OP_FEQ: return fp32::feq(a, b);
      case OP_FNE: return fp32::fne(a, b);
      case OP_FMIN: return fp32::fmin(a, b);
      case OP_FMAX: return fp32::fmax(a, b);
      default:
        error = "unknown opcode " + std::to_string(op);
        return 0;
    }
  }
};

}  // namespace compiler
}  // namespace nanotangfaust
