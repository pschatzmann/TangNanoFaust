#pragma once
// Math library for the DSP core.
//
// The hardware FPU only does + - * / sqrt, comparisons and int<->float
// conversion; everything else Faust can call (sin, exp, log, pow, floor...)
// is a subroutine built here. Polynomials and range reductions follow
// Cephes' single-precision routines (sinf, expf, atanf) and the
// 2*atanh((m-1)/(m+1)) series for log, which keeps errors to a few float32
// ulps over the useful range (tests/mathlib_test.cpp measures it).
//
// The routines are written as formulas with a small expression builder
// (E, Fc(), V(), ...) that compiles them to stack code. Binary nodes
// evaluate to op(v1 = a, v2 = b) -- FBC's operand order: b is pushed first.
#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

#include "IsaOpcodes.h"

namespace tangnanofaust {
namespace compiler {

uint32_t fromFloat(double v);  // Compiler.h

/// Assembly item: a label, or an instruction whose argument is an
/// immediate, a label or a scratch variable placed in block RAM at link time.
struct AsmItem {
  bool isLabel;
  uint8_t op;
  uint8_t argKind;  // 0 immediate, 1 label, 2 scratch variable
  uint32_t imm;
  std::string sym;
};

namespace mathlib {

static const double kPi = 3.141592653589793;

// ------------------------------------------------------------------ expressions

struct Node;
using Expr = std::shared_ptr<const Node>;

struct Node {
  enum Kind { Const, Var, Bin, Un, Call } kind;
  uint32_t value = 0;     // Const
  std::string name;       // Var name / Call routine
  uint8_t op = 0;         // Bin / Un
  std::vector<Expr> args;
};

struct E {
  Expr e;
  E(Expr x) : e(std::move(x)) {}
};

inline E mk(Node::Kind k) {
  auto n = std::make_shared<Node>();
  n->kind = k;
  return E(n);
}

inline E Fc(double x) {
  E r = mk(Node::Const);
  std::const_pointer_cast<Node>(r.e)->value = fromFloat(x);
  return r;
}
inline E I(uint32_t x) {
  E r = mk(Node::Const);
  std::const_pointer_cast<Node>(r.e)->value = x;
  return r;
}
inline E V(const char *name) {
  E r = mk(Node::Var);
  std::const_pointer_cast<Node>(r.e)->name = name;
  return r;
}
inline E op2(uint8_t op, const E &a, const E &b) {
  E r = mk(Node::Bin);
  auto n = std::const_pointer_cast<Node>(r.e);
  n->op = op;
  n->args = {a.e, b.e};
  return r;
}
inline E op1(uint8_t op, const E &a) {
  E r = mk(Node::Un);
  auto n = std::const_pointer_cast<Node>(r.e);
  n->op = op;
  n->args = {a.e};
  return r;
}
inline E call(const char *routine, const E &a) {
  E r = mk(Node::Call);
  auto n = std::const_pointer_cast<Node>(r.e);
  n->name = routine;
  n->args = {a.e};
  return r;
}
inline E call(const char *routine, const E &a, const E &b) {
  E r = mk(Node::Call);
  auto n = std::const_pointer_cast<Node>(r.e);
  n->name = routine;
  n->args = {a.e, b.e};
  return r;
}

inline E operator+(const E &a, const E &b) { return op2(OP_FADD, a, b); }
inline E operator-(const E &a, const E &b) { return op2(OP_FSUB, a, b); }
inline E operator*(const E &a, const E &b) { return op2(OP_FMUL, a, b); }
inline E operator/(const E &a, const E &b) { return op2(OP_FDIV, a, b); }
inline E operator-(const E &a) { return op1(OP_FNEG, a); }
inline E lt(const E &a, const E &b) { return op2(OP_FLT, a, b); }
inline E gt(const E &a, const E &b) { return op2(OP_FGT, a, b); }
inline E le(const E &a, const E &b) { return op2(OP_FLE, a, b); }
inline E ge(const E &a, const E &b) { return op2(OP_FGE, a, b); }
inline E eq(const E &a, const E &b) { return op2(OP_FEQ, a, b); }
inline E ne(const E &a, const E &b) { return op2(OP_FNE, a, b); }

inline E iand(const E &a, const E &b) { return op2(OP_AND, a, b); }
inline E ior(const E &a, const E &b) { return op2(OP_OR, a, b); }
inline E fabs_(const E &a) { return op1(OP_FABS, a); }
inline E i2f(const E &a) { return op1(OP_I2F, a); }
inline E f2i(const E &a) { return op1(OP_F2I, a); }
inline E fsqrt(const E &a) { return op1(OP_FSQRT, a); }
inline E copysign(const E &mag, const E &sgn) {
  return ior(iand(mag, I(0x7FFFFFFF)), iand(sgn, I(0x80000000u)));
}
inline E not_(const E &cond) { return op2(OP_EQ, cond, I(0)); }

// ------------------------------------------------------------------ routines

/// Builder for one subroutine. Arguments arrive on the stack (first on
/// top) and are stored into named scratch variables on entry; ret() leaves
/// one value on the stack and returns.
class Routine {
 public:
  std::vector<AsmItem> items;

  Routine(const char *name, std::initializer_list<const char *> params) : name_(name) {
    label("fn_" + name_);
    for (const char *p : params) store(p);
  }

  void set(const char *var, const E &e) {
    emit(e);
    store(var);
  }
  void ret(const E &e) {
    emit(e);
    op(OP_RET);
  }
  void push(const E &e) { emit(e); }

  /// Skips to the returned label unless `cond` (int expression) is nonzero.
  std::string if_(const E &cond) {
    std::string end = newLabel(name_ + "_");
    emit(cond);
    items.push_back({false, OP_JZ, 1, 0, end});
    return end;
  }
  void end(const std::string &l) { label(l); }

  std::string newLabel(const std::string &hint) { return hint + std::to_string(++n_); }
  void label(const std::string &l) { items.push_back({true, 0, 0, 0, l}); }
  void op(uint8_t code, uint32_t imm = 0) { items.push_back({false, code, 0, imm, ""}); }
  void jump(uint8_t code, const std::string &l) { items.push_back({false, code, 1, 0, l}); }

 protected:
  std::string name_;
  int n_ = 0;

  void store(const char *var) { items.push_back({false, OP_STORE, 2, 0, name_ + "." + var}); }

  void emit(const E &x) { emit(x.e); }
  void emit(const Expr &n) {
    switch (n->kind) {
      case Node::Const: op(OP_PUSH, n->value); break;
      case Node::Var: items.push_back({false, OP_LOAD, 2, 0, name_ + "." + n->name}); break;
      case Node::Bin:
        emit(n->args[1]);
        emit(n->args[0]);
        op(n->op);
        break;
      case Node::Un:
        emit(n->args[0]);
        op(n->op);
        break;
      case Node::Call:
        for (size_t i = n->args.size(); i-- > 0;) emit(n->args[i]);
        items.push_back({false, OP_CALL, 1, 0, "fn_" + n->name});
        break;
    }
  }
};

inline Routine r_trunc() {
  Routine r("trunc", {"x"});
  E x = V("x");
  auto small = r.if_(lt(fabs_(x), Fc(8388608.0)));
  r.ret(copysign(i2f(f2i(x)), x));
  r.end(small);
  r.ret(x);  // large, inf or NaN: already integral
  return r;
}

inline Routine r_floor() {  // the FFLOOR instruction (kept as a routine for ceil/pow)
  Routine r("floor", {"x"});
  r.ret(op1(OP_FFLOOR, V("x")));
  return r;
}

inline Routine r_ceil() {
  Routine r("ceil", {"x"});
  r.ret(-call("floor", -V("x")));
  return r;
}

inline Routine r_rint() {
  // Round to nearest even using the FPU's own rounding: adding and removing
  // 2^23 drops the fraction bits.
  Routine r("rint", {"x"});
  E x = V("x");
  auto big = r.if_(not_(lt(fabs_(x), Fc(8388608.0))));
  r.ret(x);
  r.end(big);
  auto neg = r.if_(lt(x, Fc(0.0)));
  r.ret(copysign((x - Fc(8388608.0)) + Fc(8388608.0), x));
  r.end(neg);
  r.ret(copysign((x + Fc(8388608.0)) - Fc(8388608.0), x));
  return r;
}

inline Routine r_round() {
  // Half away from zero (std::round).
  Routine r("round", {"x"});
  E x = V("x");
  auto big = r.if_(not_(lt(fabs_(x), Fc(8388608.0))));
  r.ret(x);
  r.end(big);
  r.set("a", fabs_(x));
  r.set("t", i2f(f2i(V("a"))));
  auto up = r.if_(ge(V("a") - V("t"), Fc(0.5)));
  r.set("t", V("t") + Fc(1.0));
  r.end(up);
  r.ret(copysign(V("t"), x));
  return r;
}

inline Routine r_fmod() {
  // fmod(x, y) = x - y * trunc(x / y). Exact for the usual |x/y| < 2^23.
  Routine r("fmod", {"x", "y"});
  r.ret(V("x") - V("y") * call("trunc", V("x") / V("y")));
  return r;
}

inline Routine r_remainder() {
  Routine r("remainder", {"x", "y"});
  r.ret(V("x") - V("y") * call("rint", V("x") / V("y")));
  return r;
}

// Cephes sinf/cosf: Cody-Waite reduction by pi/2 in three parts.
inline void trigReduce(Routine &r, const E &x) {
  r.set("k", call("rint", x * Fc(2.0 / kPi)));
  E k = V("k");
  r.set("r", ((x - k * Fc(1.5703125)) - k * Fc(4.837512969970703125e-4)) -
                 k * Fc(7.54978995489188216e-8));
  r.set("q", iand(f2i(k), I(3)));
  r.set("z", V("r") * V("r"));
  E z = V("z"), rr = V("r");
  r.set("s", ((Fc(-1.9515295891e-4) * z + Fc(8.3321608736e-3)) * z - Fc(1.6666654611e-1)) * z * rr +
                 rr);
  r.set("c", (((Fc(2.443315711809948e-5) * z - Fc(1.388731625493765e-3)) * z +
               Fc(4.166664568298827e-2)) * z * z) - Fc(0.5) * z + Fc(1.0));
}

/// Returns values[q] for q = 0..3.
inline void quadrant(Routine &r, const E values[4]) {
  E q = V("q");
  for (int i = 0; i < 3; i++) {
    auto nxt = r.if_(op2(OP_EQ, q, I(i)));
    r.ret(values[i]);
    r.end(nxt);
  }
  r.ret(values[3]);
}

inline Routine r_sin() {
  Routine r("sin", {"x"});
  trigReduce(r, V("x"));
  const E v[4] = {V("s"), V("c"), -V("s"), -V("c")};
  quadrant(r, v);
  return r;
}

inline Routine r_cos() {
  Routine r("cos", {"x"});
  trigReduce(r, V("x"));
  const E v[4] = {V("c"), -V("s"), -V("c"), V("s")};
  quadrant(r, v);
  return r;
}

inline Routine r_tan() {
  Routine r("tan", {"x"});
  trigReduce(r, V("x"));
  auto odd = r.if_(iand(V("q"), I(1)));
  r.ret(-V("c") / V("s"));
  r.end(odd);
  r.ret(V("s") / V("c"));
  return r;
}

// Fast sine/cosine (--fast-math): reduce to turns with the FPU's own
// rounding, fold to w in [0, 0.5] without branches and evaluate an odd 9th
// order polynomial for sin(pi*w), fitted to 1.9e-7 max error in float32.
// Arguments with |x / 2pi| >= 2^22 fall back to the exact routine. The
// reduction is done in float, so very large arguments lose accuracy
// (about |x| * 6e-8): fine for oscillators and filter coefficients.
inline void fastSinTurns(Routine &r, const E &q) {
  r.set("q", q);
  auto big = r.if_(not_(lt(fabs_(V("q")), Fc(4194304.0))));
  r.ret(call("sin", V("x")));  // only reached for huge arguments
  r.end(big);
  r.set("u", Fc(2.0) * (V("q") - ((V("q") + Fc(12582912.0)) - Fc(12582912.0))));  // [-1, 1]
  r.set("v", fabs_(V("u")));
  r.set("w", op2(OP_FMIN, V("v"), Fc(1.0) - V("v")));  // sin(pi(1-v)) = sin(pi v)
  r.set("z", V("w") * V("w"));
  E z = V("z");
  E s = V("w") * (Fc(3.1415925800694131) + z * (Fc(-5.1677068801087476) + z * (Fc(2.5500313933443643) +
        z * (Fc(-0.59804525665628439) + z * Fc(0.077220271878849944)))));
  r.ret(copysign(s, V("u")));
}

inline Routine r_sin_fast() {
  Routine r("sin_fast", {"x"});
  fastSinTurns(r, V("x") * Fc(1.0 / (2 * kPi)));
  return r;
}

inline Routine r_cos_fast() {
  Routine r("cos_fast", {"x"});
  fastSinTurns(r, V("x") * Fc(1.0 / (2 * kPi)) + Fc(0.25));  // cos(x) = sin(x + pi/2)
  return r;
}

inline Routine r_exp() {
  Routine r("exp", {"x"});
  E x = V("x");
  auto nan = r.if_(ne(x, x));
  r.ret(x);
  r.end(nan);
  auto hi = r.if_(gt(x, Fc(88.72283935546875)));
  r.ret(I(0x7F800000));
  r.end(hi);
  auto lo = r.if_(lt(x, Fc(-87.33654475)));
  r.ret(Fc(0.0));
  r.end(lo);
  r.set("k", call("rint", x * Fc(1.44269504088896341)));
  E k = V("k");
  r.set("r", (x - k * Fc(0.693359375)) - k * Fc(-2.12194440e-4));
  E rr = V("r");
  E p = (((((Fc(1.9875691500e-4) * rr + Fc(1.3981999507e-3)) * rr + Fc(8.3334519073e-3)) * rr +
          Fc(4.1665795894e-2)) * rr + Fc(1.6666665459e-1)) * rr + Fc(5.0000001201e-1));
  r.set("p", p * (rr * rr) + rr + Fc(1.0));
  // Scale by 2^k as two halves so neither exponent overflows.
  r.set("ki", f2i(k));
  r.set("e1", op2(OP_ASR, V("ki"), I(1)));
  r.set("e2", op2(OP_SUB, V("ki"), V("e1")));
  E s1 = op2(OP_SHL, op2(OP_ADD, V("e1"), I(127)), I(23));
  E s2 = op2(OP_SHL, op2(OP_ADD, V("e2"), I(127)), I(23));
  r.ret(V("p") * s1 * s2);
  return r;
}

inline Routine r_log() {
  Routine r("log", {"x"});
  E x = V("x");
  auto bad = r.if_(op2(OP_OR, ne(x, x), lt(x, Fc(0.0))));
  r.ret(I(0x7FC00000));
  r.end(bad);
  auto zero = r.if_(eq(x, Fc(0.0)));
  r.ret(I(0xFF800000u));
  r.end(zero);
  auto inf = r.if_(op2(OP_EQ, x, I(0x7F800000)));
  r.ret(x);
  r.end(inf);
  r.set("e", op2(OP_SUB, iand(op2(OP_LSR, x, I(23)), I(0xFF)), I(127)));
  r.set("m", ior(iand(x, I(0x7FFFFF)), I(0x3F800000)));
  auto big = r.if_(gt(V("m"), Fc(1.41421356237)));
  r.set("m", V("m") * Fc(0.5));
  r.set("e", op2(OP_ADD, V("e"), I(1)));
  r.end(big);
  r.set("f", V("m") - Fc(1.0));
  r.set("s", V("f") / (V("f") + Fc(2.0)));
  r.set("z", V("s") * V("s"));
  E z = V("z");
  E series = Fc(1.0) + z * (Fc(1.0 / 3) + z * (Fc(1.0 / 5) + z * (Fc(1.0 / 7) + z * (Fc(1.0 / 9) +
                                                                               z * Fc(1.0 / 11)))));
  r.set("lm", Fc(2.0) * V("s") * series);
  r.set("ef", i2f(V("e")));
  r.ret(V("ef") * Fc(0.693145751953125) + (V("lm") + V("ef") * Fc(1.428606765330187045e-06)));
  return r;
}

inline Routine r_log10() {
  Routine r("log10", {"x"});
  r.ret(call("log", V("x")) * Fc(0.43429448190325176));
  return r;
}

inline Routine r_pow() {
  // pow(x, y), FBC order: x on top.
  Routine r("pow", {"x", "y"});
  E x = V("x"), y = V("y");
  auto one = r.if_(op2(OP_OR, eq(y, Fc(0.0)), eq(x, Fc(1.0))));
  r.ret(Fc(1.0));
  r.end(one);
  // Small integer exponent (Faust's x^3 etc.): square-and-multiply, which is
  // both much faster than exp(y*log(x)) and right for negative x.
  auto notint = r.if_(op2(OP_AND, le(fabs_(y), Fc(16.0)), eq(call("floor", y), y)));
  r.set("n", f2i(fabs_(y)));
  r.set("b", x);
  r.set("r", Fc(1.0));
  std::string loop = r.newLabel("pow_");
  r.label(loop);
  auto bit = r.if_(iand(V("n"), I(1)));
  r.set("r", V("r") * V("b"));
  r.end(bit);
  r.set("b", V("b") * V("b"));
  r.set("n", op2(OP_LSR, V("n"), I(1)));
  r.push(V("n"));
  r.jump(OP_JNZ, loop);
  auto inv = r.if_(lt(y, Fc(0.0)));
  r.ret(Fc(1.0) / V("r"));
  r.end(inv);
  r.ret(V("r"));
  r.end(notint);
  auto zero = r.if_(eq(x, Fc(0.0)));
  auto pos = r.if_(gt(y, Fc(0.0)));
  r.ret(Fc(0.0));
  r.end(pos);
  r.ret(I(0x7F800000));
  r.end(zero);
  auto neg = r.if_(lt(x, Fc(0.0)));
  auto nonint = r.if_(ne(call("floor", y), y));
  r.ret(I(0x7FC00000));
  r.end(nonint);
  r.set("r", call("exp", y * call("log", -x)));
  auto odd = r.if_(iand(f2i(call("fmod", y, Fc(2.0))), I(1)));
  r.ret(-V("r"));
  r.end(odd);
  r.ret(V("r"));
  r.end(neg);
  r.ret(call("exp", y * call("log", x)));
  return r;
}

inline Routine r_atan() {
  Routine r("atan", {"x"});
  E x = V("x");
  r.set("a", fabs_(x));
  r.set("y0", Fc(0.0));
  auto hi = r.if_(gt(V("a"), Fc(2.414213562373095)));
  r.set("y0", Fc(kPi / 2));
  r.set("a", Fc(-1.0) / V("a"));
  std::string midEnd = r.newLabel("atan_");
  r.jump(OP_JMP, midEnd);
  r.end(hi);
  auto mid = r.if_(gt(V("a"), Fc(0.4142135623730950)));
  r.set("y0", Fc(kPi / 4));
  r.set("a", (V("a") - Fc(1.0)) / (V("a") + Fc(1.0)));
  r.end(mid);
  r.label(midEnd);
  r.set("z", V("a") * V("a"));
  E z = V("z"), a = V("a");
  E y = V("y0") + ((((Fc(8.05374449538e-2) * z - Fc(1.38776856032e-1)) * z + Fc(1.99777106478e-1)) * z -
                    Fc(3.33329491539e-1)) * z * a + a);
  r.ret(copysign(y, x));
  return r;
}

inline Routine r_atan2() {
  // atan2(y, x), FBC order: y on top.
  Routine r("atan2", {"y", "x"});
  E y = V("y"), x = V("x");
  auto pos = r.if_(gt(x, Fc(0.0)));
  r.ret(call("atan", y / x));
  r.end(pos);
  auto neg = r.if_(lt(x, Fc(0.0)));
  auto up = r.if_(ge(y, Fc(0.0)));
  r.ret(call("atan", y / x) + Fc(kPi));
  r.end(up);
  r.ret(call("atan", y / x) - Fc(kPi));
  r.end(neg);
  auto above = r.if_(gt(y, Fc(0.0)));
  r.ret(Fc(kPi / 2));
  r.end(above);
  auto below = r.if_(lt(y, Fc(0.0)));
  r.ret(Fc(-kPi / 2));
  r.end(below);
  r.ret(Fc(0.0));
  return r;
}

inline Routine r_asin() {
  Routine r("asin", {"x"});
  E x = V("x");
  r.ret(call("atan2", x, fsqrt(Fc(1.0) - x * x)));
  return r;
}

inline Routine r_acos() {
  Routine r("acos", {"x"});
  E x = V("x");
  r.ret(call("atan2", fsqrt(Fc(1.0) - x * x), x));
  return r;
}

inline Routine r_sinh() {
  Routine r("sinh", {"x"});
  E x = V("x");
  auto small = r.if_(lt(fabs_(x), Fc(0.5)));
  r.set("z", x * x);
  E z = V("z");
  r.ret(x + x * z * (Fc(1.0 / 6) + z * (Fc(1.0 / 120) + z * Fc(1.0 / 5040))));
  r.end(small);
  r.set("e", call("exp", x));
  r.ret((V("e") - Fc(1.0) / V("e")) * Fc(0.5));
  return r;
}

inline Routine r_cosh() {
  Routine r("cosh", {"x"});
  r.set("e", call("exp", V("x")));
  r.ret((V("e") + Fc(1.0) / V("e")) * Fc(0.5));
  return r;
}

inline Routine r_tanh() {
  Routine r("tanh", {"x"});
  E x = V("x");
  auto sat = r.if_(gt(fabs_(x), Fc(9.0)));
  r.ret(copysign(Fc(1.0), x));
  r.end(sat);
  auto small = r.if_(lt(fabs_(x), Fc(0.0625)));
  r.set("z", x * x);
  E z = V("z");
  r.ret(x - x * z * (Fc(1.0 / 3) - z * (Fc(2.0 / 15) - z * Fc(17.0 / 315))));
  r.end(small);
  r.set("e", call("exp", Fc(2.0) * x));
  r.ret((V("e") - Fc(1.0)) / (V("e") + Fc(1.0)));
  return r;
}

inline Routine r_asinh() {
  Routine r("asinh", {"x"});
  E x = V("x");
  r.set("a", fabs_(x));
  auto small = r.if_(lt(V("a"), Fc(0.0625)));
  r.set("z", x * x);
  E z = V("z");
  r.ret(x - x * z * (Fc(1.0 / 6) - z * (Fc(3.0 / 40) - z * Fc(15.0 / 336))));
  r.end(small);
  E a = V("a");
  r.ret(copysign(call("log", a + fsqrt(a * a + Fc(1.0))), x));
  return r;
}

inline Routine r_acosh() {
  Routine r("acosh", {"x"});
  E x = V("x");
  r.ret(call("log", x + fsqrt(x * x - Fc(1.0))));
  return r;
}

inline Routine r_atanh() {
  Routine r("atanh", {"x"});
  E x = V("x");
  auto small = r.if_(lt(fabs_(x), Fc(0.0625)));
  r.set("z", x * x);
  E z = V("z");
  r.ret(x + x * z * (Fc(1.0 / 3) + z * (Fc(1.0 / 5) + z * Fc(1.0 / 7))));
  r.end(small);
  r.ret(Fc(0.5) * call("log", (Fc(1.0) + x) / (Fc(1.0) - x)));
  return r;
}

struct RoutineInfo {
  const char *name;
  Routine (*build)();
  const char *deps;  // space separated
};

/// All routines, sorted by name (the order they are linked in).
static const RoutineInfo kRoutines[] = {
    {"acos", r_acos, "atan2"},   {"acosh", r_acosh, "log"},    {"asin", r_asin, "atan2"},
    {"asinh", r_asinh, "log"},   {"atan", r_atan, ""},         {"atan2", r_atan2, "atan"},
    {"atanh", r_atanh, "log"},   {"ceil", r_ceil, "floor"},    {"cos", r_cos, "rint"},
    {"cos_fast", r_cos_fast, "sin"},
    {"cosh", r_cosh, "exp"},     {"exp", r_exp, "rint"},       {"floor", r_floor, ""},
    {"fmod", r_fmod, "trunc"},   {"log", r_log, ""},           {"log10", r_log10, "log"},
    {"pow", r_pow, "exp log floor fmod"},
    {"remainder", r_remainder, "rint"},
    {"rint", r_rint, ""},        {"round", r_round, ""},       {"sin", r_sin, "rint"},
    {"sin_fast", r_sin_fast, "sin"},
    {"sinh", r_sinh, "exp"},     {"tan", r_tan, "rint"},       {"tanh", r_tanh, "exp"},
    {"trunc", r_trunc, ""},
};
static const int kRoutineCount = sizeof(kRoutines) / sizeof(kRoutines[0]);

}  // namespace mathlib
}  // namespace compiler
}  // namespace tangnanofaust
