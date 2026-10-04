#pragma once
// Reference interpreter for Faust FBC programs in float32 (like a `-single`
// Faust build, with libm math), following Faust's own fbc_interpreter.hh.
// It is independent of the compiler and the DSP core, so the tests compare
// compiled programs (on the simulator and the RTL) against it.
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <string>
#include <vector>

#include "NanoTangFaust/compiler/Fbc.h"

namespace nanotangfaust {
namespace test {

using namespace compiler;

class FbcInterpreter {
 public:
  std::vector<int32_t> iheap;
  std::vector<float> rheap;
  std::vector<float> inputs, outputs;
  std::string error;

  FbcInterpreter(const FbcProgram &p, int sampleRate)
      : iheap(p.intHeapSize, 0), rheap(p.realHeapSize, 0.0f),
        inputs(p.inputs, 0.0f), outputs(p.outputs, 0.0f), prog_(p) {
    iheap[p.srOffset] = sampleRate;
  }

  /// Faust's init(sample_rate): classInit + instanceInit.
  void init() {
    const BlockId blocks[] = {kStaticInit, kConstants, kResetUI, kClear};
    for (BlockId b : blocks) run(b);
  }

  /// One sample: control + dsp with count = 1.
  void compute() {
    iheap[prog_.countOffset] = 1;
    run(kControl);
    run(kDsp);
  }

 protected:
  const FbcProgram &prog_;
  struct V {  // a stack value: int or float
    int32_t i;
    float f;
  };

  void run(BlockId b) {
    if (prog_.hasBlock[b]) {
      std::vector<V> st;
      exec(prog_.blocks[b], st);
    }
  }

  static V vi(int32_t i) { return V{i, 0}; }
  static V vf(float f) { return V{0, f}; }
  static V pop(std::vector<V> &st) {
    V v = st.back();
    st.pop_back();
    return v;
  }
  static int32_t f2i(float f) {
    if (f != f || f >= 2147483648.0f || f < -2147483648.0f) return INT32_MIN;
    return (int32_t)f;
  }
  static int32_t cdiv(int32_t a, int32_t b) {
    if (b == 0) return 0;
    int64_t q = (int64_t)a / b;
    return (int32_t)q;
  }
  static int32_t crem(int32_t a, int32_t b) {
    if (b == 0) return 0;
    return (int32_t)((int64_t)a % b);
  }

  /// Executes a block; returns true if it ended with a taken kCondBranch.
  bool exec(const Block &block, std::vector<V> &st) {
    std::vector<int32_t> &ih = iheap;
    std::vector<float> &rh = rheap;
    for (const Instr &ins : block.instrs) {
      if (!error.empty()) return false;
      V a, b;
      switch (ins.op) {
        case kRealValue: st.push_back(vf((float)ins.realValue)); break;
        case kInt32Value: st.push_back(vi(ins.intValue)); break;
        case kLoadReal: st.push_back(vf(rh[ins.offset1])); break;
        case kLoadInt: st.push_back(vi(ih[ins.offset1])); break;
        case kStoreReal: rh[ins.offset1] = pop(st).f; break;
        case kStoreInt: ih[ins.offset1] = pop(st).i; break;
        case kStoreRealValue: rh[ins.offset1] = (float)ins.realValue; break;
        case kStoreIntValue: ih[ins.offset1] = ins.intValue; break;
        case kLoadIndexedReal: st.push_back(vf(rh[ins.offset1 + pop(st).i])); break;
        case kLoadIndexedInt: st.push_back(vi(ih[ins.offset1 + pop(st).i])); break;
        case kStoreIndexedReal: a = pop(st); rh[ins.offset1 + a.i] = pop(st).f; break;
        case kStoreIndexedInt: a = pop(st); ih[ins.offset1 + a.i] = pop(st).i; break;
        case kLoadInput: pop(st); st.push_back(vf(inputs[ins.offset1])); break;
        case kStoreOutput: pop(st); outputs[ins.offset1] = pop(st).f; break;
        case kCastReal: st.push_back(vf((float)pop(st).i)); break;
        case kCastInt: st.push_back(vi(f2i(pop(st).f))); break;
        case kBitcastInt: { float f = pop(st).f; int32_t i; memcpy(&i, &f, 4); st.push_back(vi(i)); break; }
        case kBitcastReal: { int32_t i = pop(st).i; float f; memcpy(&f, &i, 4); st.push_back(vf(f)); break; }
        case kAddReal: a = pop(st); b = pop(st); st.push_back(vf(a.f + b.f)); break;
        case kSubReal: a = pop(st); b = pop(st); st.push_back(vf(a.f - b.f)); break;
        case kMultReal: a = pop(st); b = pop(st); st.push_back(vf(a.f * b.f)); break;
        case kDivReal: a = pop(st); b = pop(st); st.push_back(vf(a.f / b.f)); break;
        case kRemReal: a = pop(st); b = pop(st); st.push_back(vf(remainderf(a.f, b.f))); break;
        case kAtan2f: a = pop(st); b = pop(st); st.push_back(vf(atan2f(a.f, b.f))); break;
        case kFmodf: a = pop(st); b = pop(st); st.push_back(vf(fmodf(a.f, b.f))); break;
        case kPowf: a = pop(st); b = pop(st); st.push_back(vf(powf(a.f, b.f))); break;
        case kMaxf: a = pop(st); b = pop(st); st.push_back(vf(a.f < b.f ? b.f : a.f)); break;
        case kMinf: a = pop(st); b = pop(st); st.push_back(vf(b.f < a.f ? b.f : a.f)); break;
        case kCopysignf: a = pop(st); b = pop(st); st.push_back(vf(copysignf(a.f, b.f))); break;
        case kGTReal: a = pop(st); b = pop(st); st.push_back(vi(a.f > b.f)); break;
        case kLTReal: a = pop(st); b = pop(st); st.push_back(vi(a.f < b.f)); break;
        case kGEReal: a = pop(st); b = pop(st); st.push_back(vi(a.f >= b.f)); break;
        case kLEReal: a = pop(st); b = pop(st); st.push_back(vi(a.f <= b.f)); break;
        case kEQReal: a = pop(st); b = pop(st); st.push_back(vi(a.f == b.f)); break;
        case kNEReal: a = pop(st); b = pop(st); st.push_back(vi(a.f != b.f)); break;
        case kAddInt: a = pop(st); b = pop(st); st.push_back(vi((int32_t)((uint32_t)a.i + (uint32_t)b.i))); break;
        case kSubInt: a = pop(st); b = pop(st); st.push_back(vi((int32_t)((uint32_t)a.i - (uint32_t)b.i))); break;
        case kMultInt: a = pop(st); b = pop(st); st.push_back(vi((int32_t)((uint32_t)a.i * (uint32_t)b.i))); break;
        case kDivInt: a = pop(st); b = pop(st); st.push_back(vi(cdiv(a.i, b.i))); break;
        case kRemInt: a = pop(st); b = pop(st); st.push_back(vi(crem(a.i, b.i))); break;
        case kLshInt: a = pop(st); b = pop(st); st.push_back(vi((int32_t)((uint32_t)a.i << (b.i & 31)))); break;
        case kARshInt: a = pop(st); b = pop(st); st.push_back(vi(a.i >> (b.i & 31))); break;
        case kLRshInt: a = pop(st); b = pop(st); st.push_back(vi((int32_t)((uint32_t)a.i >> (b.i & 31)))); break;
        case kGTInt: a = pop(st); b = pop(st); st.push_back(vi(a.i > b.i)); break;
        case kLTInt: a = pop(st); b = pop(st); st.push_back(vi(a.i < b.i)); break;
        case kGEInt: a = pop(st); b = pop(st); st.push_back(vi(a.i >= b.i)); break;
        case kLEInt: a = pop(st); b = pop(st); st.push_back(vi(a.i <= b.i)); break;
        case kEQInt: a = pop(st); b = pop(st); st.push_back(vi(a.i == b.i)); break;
        case kNEInt: a = pop(st); b = pop(st); st.push_back(vi(a.i != b.i)); break;
        case kANDInt: a = pop(st); b = pop(st); st.push_back(vi(a.i & b.i)); break;
        case kORInt: a = pop(st); b = pop(st); st.push_back(vi(a.i | b.i)); break;
        case kXORInt: a = pop(st); b = pop(st); st.push_back(vi(a.i ^ b.i)); break;
        case kMax: a = pop(st); b = pop(st); st.push_back(vi(a.i < b.i ? b.i : a.i)); break;
        case kMin: a = pop(st); b = pop(st); st.push_back(vi(b.i < a.i ? b.i : a.i)); break;
        case kAbs: a = pop(st); st.push_back(vi(a.i < 0 ? (int32_t)(0u - (uint32_t)a.i) : a.i)); break;
        case kAbsf: st.push_back(vf(fabsf(pop(st).f))); break;
        case kAcosf: st.push_back(vf(acosf(pop(st).f))); break;
        case kAcoshf: st.push_back(vf(acoshf(pop(st).f))); break;
        case kAsinf: st.push_back(vf(asinf(pop(st).f))); break;
        case kAsinhf: st.push_back(vf(asinhf(pop(st).f))); break;
        case kAtanf: st.push_back(vf(atanf(pop(st).f))); break;
        case kAtanhf: st.push_back(vf(atanhf(pop(st).f))); break;
        case kCeilf: st.push_back(vf(ceilf(pop(st).f))); break;
        case kCosf: st.push_back(vf(cosf(pop(st).f))); break;
        case kCoshf: st.push_back(vf(coshf(pop(st).f))); break;
        case kExpf: st.push_back(vf(expf(pop(st).f))); break;
        case kFloorf: st.push_back(vf(floorf(pop(st).f))); break;
        case kLogf: st.push_back(vf(logf(pop(st).f))); break;
        case kLog10f: st.push_back(vf(log10f(pop(st).f))); break;
        case kRintf: st.push_back(vf(rintf(pop(st).f))); break;
        case kRoundf: st.push_back(vf(roundf(pop(st).f))); break;
        case kSinf: st.push_back(vf(sinf(pop(st).f))); break;
        case kSinhf: st.push_back(vf(sinhf(pop(st).f))); break;
        case kSqrtf: st.push_back(vf(sqrtf(pop(st).f))); break;
        case kTanf: st.push_back(vf(tanf(pop(st).f))); break;
        case kTanhf: st.push_back(vf(tanhf(pop(st).f))); break;
        case kIsnanf: st.push_back(vi(isnan(pop(st).f))); break;
        case kIsinff: st.push_back(vi(isinf(pop(st).f))); break;
        case kLoop:
          exec(ins.branches[0], st);
          while (exec(ins.branches[1], st)) {
          }
          break;
        case kIf: case kSelectReal: case kSelectInt:
          exec(ins.branches[pop(st).i ? 0 : 1], st);
          break;
        case kCondBranch:
          if (pop(st).i) return true;
          break;
        case kReturn: return false;
        case kNop: break;
        default:
          error = std::string("FbcInterpreter: unsupported ") + kFbcOpNames[ins.op];
          return false;
      }
    }
    return false;
  }
};

}  // namespace test
}  // namespace nanotangfaust
