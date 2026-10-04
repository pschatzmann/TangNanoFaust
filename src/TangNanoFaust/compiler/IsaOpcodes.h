#pragma once
// Instruction set of the NanoTangFaust DSP core (gateware/rtl/dsp_core.v).
//
// This list is the single source of the opcodes: `faust2tang
// --verilog-opcodes` writes gateware/rtl/isa_opcodes.vh from it.
//
// The core is a stack machine on untyped 32-bit words. Every instruction is
// 40 bits: opcode[39:32], arg[31:0]. Binary operators take v1 = top of
// stack (T) and v2 = the word below it (N) and replace both with
// op(v1, v2) -- the same operand order as Faust's FBC.
#include <stdint.h>

// X(name, code, takes_arg, description)
#define NTF_CORE_OPCODES(X)                                                  \
  X(NOP, 0x00, 0, "no operation")                                            \
  X(PUSH, 0x01, 1, "push arg")                                               \
  X(LOAD, 0x02, 1, "push heap[arg]")                                         \
  X(STORE, 0x03, 1, "heap[arg] = pop")                                       \
  X(LOADX, 0x04, 1, "i = pop; push heap[arg + i]")                           \
  X(STOREX, 0x05, 1, "i = pop; v = pop; heap[arg + i] = v")                  \
  X(IN, 0x06, 1, "pop (frame index, ignored); push input[arg]")              \
  X(OUT, 0x07, 1, "pop (frame index, ignored); output[arg] = pop")           \
  X(DUP, 0x08, 0, "push T")                                                  \
  X(DROP, 0x09, 0, "pop")                                                    \
  X(SWAP, 0x0A, 0, "swap T and N")                                           \
  X(OVER, 0x0B, 0, "push N")                                                 \
  X(TEE, 0x0C, 1, "heap[arg] = T (no pop)")                                  \
  X(JMP, 0x10, 1, "pc = arg")                                                \
  X(JZ, 0x11, 1, "if pop == 0: pc = arg")                                    \
  X(JNZ, 0x12, 1, "if pop != 0: pc = arg")                                   \
  X(CALL, 0x13, 1, "push return address; pc = arg")                          \
  X(RET, 0x14, 0, "pc = return address")                                     \
  X(HALT, 0x15, 0, "end of this sample's computation")                       \
  X(ADD, 0x20, 0, "v1 + v2")                                                 \
  X(SUB, 0x21, 0, "v1 - v2")                                                 \
  X(MUL, 0x22, 0, "v1 * v2 (low 32 bits)")                                   \
  X(DIV, 0x23, 0, "v1 / v2 (truncating; x/0 = 0)")                           \
  X(REM, 0x24, 0, "v1 % v2 (sign of v1; x%0 = 0)")                           \
  X(SHL, 0x25, 0, "v1 << (v2 & 31)")                                         \
  X(ASR, 0x26, 0, "v1 >> (v2 & 31), arithmetic")                             \
  X(LSR, 0x27, 0, "v1 >> (v2 & 31), logical")                                \
  X(GT, 0x28, 0, "v1 > v2 (signed)")                                         \
  X(LT, 0x29, 0, "v1 < v2")                                                  \
  X(GE, 0x2A, 0, "v1 >= v2")                                                 \
  X(LE, 0x2B, 0, "v1 <= v2")                                                 \
  X(EQ, 0x2C, 0, "v1 == v2")                                                 \
  X(NE, 0x2D, 0, "v1 != v2")                                                 \
  X(AND, 0x2E, 0, "v1 & v2")                                                 \
  X(OR, 0x2F, 0, "v1 | v2")                                                  \
  X(XOR, 0x30, 0, "v1 ^ v2")                                                 \
  X(MIN, 0x31, 0, "std::min(v1, v2), signed")                                \
  X(MAX, 0x32, 0, "std::max(v1, v2), signed")                                \
  X(ABS, 0x33, 0, "T = |T| (unary)")                                         \
  X(FADD, 0x40, 0, "v1 + v2")                                                \
  X(FSUB, 0x41, 0, "v1 - v2")                                                \
  X(FMUL, 0x42, 0, "v1 * v2")                                                \
  X(FDIV, 0x43, 0, "v1 / v2")                                                \
  X(FGT, 0x44, 0, "v1 > v2")                                                 \
  X(FLT, 0x45, 0, "v1 < v2")                                                 \
  X(FGE, 0x46, 0, "v1 >= v2")                                                \
  X(FLE, 0x47, 0, "v1 <= v2")                                                \
  X(FEQ, 0x48, 0, "v1 == v2")                                                \
  X(FNE, 0x49, 0, "v1 != v2")                                                \
  X(FMIN, 0x4A, 0, "std::min(v1, v2)")                                       \
  X(FMAX, 0x4B, 0, "std::max(v1, v2)")                                       \
  X(FABS, 0x50, 0, "T = |T| (unary)")                                        \
  X(FNEG, 0x51, 0, "T = -T (unary)")                                         \
  X(I2F, 0x52, 0, "T = float(T) (unary)")                                    \
  X(F2I, 0x53, 0, "T = int(T), truncating (unary)")                          \
  X(FSQRT, 0x54, 0, "T = sqrt(T) (unary)")                                   \
  X(FFLOOR, 0x55, 0, "T = floor(T) (unary)")

namespace nanotangfaust {
namespace compiler {

enum CoreOp : uint8_t {
#define NTF_X(name, code, arg, desc) OP_##name = code,
  NTF_CORE_OPCODES(NTF_X)
#undef NTF_X
};

/// Fused operand forms of the binary operators (ADD..MAX, FADD..FMAX):
/// opcode + 0x40 takes v1 from the instruction (`arg` = immediate),
/// opcode + 0x80 from block RAM / SDRAM (`arg` = address); v2 is T and the
/// result replaces it. They replace `PUSH k; op` and `LOAD a; op`.
static const uint8_t kFuseImm = 0x40, kFuseMem = 0x80;
inline bool isFusableBinary(uint8_t c) { return (c >= 0x20 && c <= 0x32) || (c >= 0x40 && c <= 0x4B); }
inline bool isFusedImm(uint8_t c) { return c >= kFuseImm && isFusableBinary(uint8_t(c - kFuseImm)) && c >= 0x60; }
inline bool isFusedMem(uint8_t c) { return c >= kFuseMem && isFusableBinary(uint8_t(c - kFuseMem)); }
inline uint8_t fusedBase(uint8_t c) { return isFusedMem(c) ? uint8_t(c - kFuseMem) : uint8_t(c - kFuseImm); }

struct CoreOpInfo {
  const char *name;
  uint8_t code;
  bool takesArg;
  const char *description;
};

static const CoreOpInfo kCoreOps[] = {
#define NTF_X(name, code, arg, desc) {#name, code, arg != 0, desc},
    NTF_CORE_OPCODES(NTF_X)
#undef NTF_X
};
static const int kCoreOpCount = sizeof(kCoreOps) / sizeof(kCoreOps[0]);

}  // namespace compiler
}  // namespace nanotangfaust
