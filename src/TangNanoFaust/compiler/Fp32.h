#pragma once
// Bit-exact specification of the gateware FPU (gateware/rtl/fpu.v), used
// by the simulator (Isa.h) and the FPU test vectors.
//
// All values are binary32 bit patterns. Every operation is computed
// exactly with integers and then rounded by roundPack() (Compiler.h), the
// single rounding rule the hardware implements too:
// - round to nearest, ties to even, at 24 significant bits, with an
//   unbounded exponent; THEN
// - results below the normal range become a signed zero (flush to zero),
//   results above it infinity;
// - denormal inputs are treated as zero (DAZ);
// - any NaN result is the canonical quiet NaN 0x7FC00000;
// - float->int truncates toward zero; NaN/out-of-range give 0x80000000.
// For normal results this is exactly IEEE-754 round-to-nearest-even.
#include <stdint.h>
#include <string.h>

#include "Compiler.h"

namespace tangnanofaust {
namespace fp32 {

static const uint32_t QNAN = 0x7FC00000u, PINF = 0x7F800000u, SIGN = 0x80000000u;

using compiler::roundPack;

inline float toFloat(uint32_t b) {
  float f;
  memcpy(&f, &b, 4);
  return f;
}

inline uint32_t daz(uint32_t b) { return (b & 0x7F800000u) == 0 ? (b & SIGN) : b; }

/// Flush denormals to signed zero, canonicalize NaNs.
inline uint32_t canon(uint32_t b) {
  uint32_t e = b & 0x7F800000u;
  if (e == 0) return b & SIGN;
  if (e == 0x7F800000u && (b & 0x7FFFFF)) return QNAN;
  return b;
}

enum Kind { Zero, Num, Inf, NaN };

struct Unpacked {
  bool sign;
  Kind kind;
  uint64_t mant;  // value = mant * 2^exp2
  int exp2;
};

inline Unpacked unpack(uint32_t b) {
  b = daz(b);
  Unpacked u{(b >> 31) != 0, Num, 0, 0};
  uint32_t e = (b >> 23) & 0xFF, m = b & 0x7FFFFF;
  if (e == 0) u.kind = Zero;
  else if (e == 0xFF) u.kind = m ? NaN : Inf;
  else {
    u.mant = m | 0x800000u;
    u.exp2 = (int)e - 150;
  }
  return u;
}

inline uint32_t fadd(uint32_t a, uint32_t b) {
  Unpacked x = unpack(a), y = unpack(b);
  if (x.kind == NaN || y.kind == NaN) return QNAN;
  if (x.kind == Inf || y.kind == Inf) {
    if (x.kind == Inf && y.kind == Inf && x.sign != y.sign) return QNAN;
    return ((x.kind == Inf ? x.sign : y.sign) ? SIGN : 0) | PINF;
  }
  if (x.kind == Zero && y.kind == Zero) return (x.sign && y.sign) ? SIGN : 0;
  if (x.kind == Zero) return daz(b);
  if (y.kind == Zero) return daz(a);
  if (x.exp2 < y.exp2 || (x.exp2 == y.exp2 && x.mant < y.mant)) {
    Unpacked t = x;
    x = y;
    y = t;
  }
  // |x| >= |y|. Align y to x with up to 30 extra bits; further down y only
  // matters as a sticky bit (it is far below half an ulp of x).
  int d = x.exp2 - y.exp2;
  int64_t vx = (int64_t)(x.mant << 30);
  int64_t vy = d <= 30 ? (int64_t)(y.mant << (30 - d)) : 1;
  int64_t v = x.sign == y.sign ? vx + vy : vx - vy;
  if (v == 0) return 0;
  return roundPack(x.sign, (uint64_t)v, x.exp2 - 30);
}

inline uint32_t fsub(uint32_t a, uint32_t b) { return fadd(a, b ^ SIGN); }

inline uint32_t fmul(uint32_t a, uint32_t b) {
  Unpacked x = unpack(a), y = unpack(b);
  bool s = x.sign != y.sign;
  if (x.kind == NaN || y.kind == NaN) return QNAN;
  if (x.kind == Inf || y.kind == Inf) {
    if (x.kind == Zero || y.kind == Zero) return QNAN;
    return (s ? SIGN : 0) | PINF;
  }
  if (x.kind == Zero || y.kind == Zero) return s ? SIGN : 0;
  return roundPack(s, x.mant * y.mant, x.exp2 + y.exp2);
}

/// roundPack with a sticky bit for an inexact remainder below `mant`.
inline uint32_t roundPackSticky(bool sign, uint64_t mant, int exp2, bool sticky) {
  // Append the sticky as one extra low bit: below the guard bit whenever
  // mant has more than 24 bits, which div/sqrt guarantee.
  return roundPack(sign, (mant << 1) | (sticky ? 1 : 0), exp2 - 1);
}

inline uint32_t fdiv(uint32_t a, uint32_t b) {
  Unpacked x = unpack(a), y = unpack(b);
  bool s = x.sign != y.sign;
  if (x.kind == NaN || y.kind == NaN) return QNAN;
  if (x.kind == Inf) return y.kind == Inf ? QNAN : (s ? SIGN : 0) | PINF;
  if (y.kind == Inf) return s ? SIGN : 0;
  if (y.kind == Zero) return x.kind == Zero ? QNAN : (s ? SIGN : 0) | PINF;
  if (x.kind == Zero) return s ? SIGN : 0;
  uint64_t n = x.mant << 26;
  return roundPackSticky(s, n / y.mant, x.exp2 - y.exp2 - 26, n % y.mant != 0);
}

inline uint64_t isqrt64(uint64_t v) {
  uint64_t res = 0, one = 1ull << 62;
  while (one > v) one >>= 2;
  while (one) {
    if (v >= res + one) {
      v -= res + one;
      res = (res >> 1) + one;
    } else {
      res >>= 1;
    }
    one >>= 2;
  }
  return res;
}

inline uint32_t fsqrt(uint32_t a) {
  Unpacked x = unpack(a);
  if (x.kind == NaN) return QNAN;
  if (x.kind == Zero) return x.sign ? SIGN : 0;
  if (x.sign) return QNAN;
  if (x.kind == Inf) return PINF;
  uint64_t m = x.mant;
  int e = x.exp2;
  if (e & 1) {
    m <<= 1;
    e -= 1;
  }
  uint64_t v = m << 30;
  uint64_t r = isqrt64(v);
  return roundPackSticky(false, r, e / 2 - 15, r * r != v);
}

// comparisons: 1/0; NaN compares unordered (only FNE is true)
inline int cmp(uint32_t a, uint32_t b, bool &unordered) {
  Unpacked x = unpack(a), y = unpack(b);
  unordered = x.kind == NaN || y.kind == NaN;
  if (unordered) return 0;
  auto key = [](const Unpacked &u) -> int64_t {
    if (u.kind == Zero) return 0;
    int64_t mag = u.kind == Inf ? ((int64_t)1 << 40) : (int64_t)(u.exp2 + 150) << 23 |
                                                        (int64_t)(u.mant & 0x7FFFFF);
    return u.sign ? -mag : mag;
  };
  int64_t kx = key(x), ky = key(y);
  return kx < ky ? -1 : kx > ky ? 1 : 0;
}

inline uint32_t fgt(uint32_t a, uint32_t b) { bool u; int c = cmp(a, b, u); return !u && c > 0; }
inline uint32_t flt(uint32_t a, uint32_t b) { bool u; int c = cmp(a, b, u); return !u && c < 0; }
inline uint32_t fge(uint32_t a, uint32_t b) { bool u; int c = cmp(a, b, u); return !u && c >= 0; }
inline uint32_t fle(uint32_t a, uint32_t b) { bool u; int c = cmp(a, b, u); return !u && c <= 0; }
inline uint32_t feq(uint32_t a, uint32_t b) { bool u; int c = cmp(a, b, u); return !u && c == 0; }
inline uint32_t fne(uint32_t a, uint32_t b) { bool u; int c = cmp(a, b, u); return u || c != 0; }

/// std::min(v1, v2) = (v2 < v1) ? v2 : v1
inline uint32_t fmin(uint32_t a, uint32_t b) { return flt(b, a) ? canon(daz(b)) : canon(daz(a)); }
/// std::max(v1, v2) = (v1 < v2) ? v2 : v1
inline uint32_t fmax(uint32_t a, uint32_t b) { return flt(a, b) ? canon(daz(b)) : canon(daz(a)); }
inline uint32_t fabs_(uint32_t a) { return canon(a & 0x7FFFFFFFu); }
inline uint32_t fneg(uint32_t a) { return canon(a ^ SIGN); }

inline uint32_t i2f(uint32_t a) {
  if (a & SIGN) return roundPack(true, (uint32_t)(0u - a), 0);  // 0x80000000 -> 2^31
  return roundPack(false, a, 0);
}

/// floor(x), exact: integral values (|x| >= 2^23, inf) unchanged, -0 stays
/// -0, NaN canonical.
inline uint32_t ffloor(uint32_t a) {
  Unpacked x = unpack(a);
  if (x.kind == NaN) return QNAN;
  if (x.kind != Num || x.exp2 >= 0) return daz(a);
  uint64_t mag = x.exp2 <= -64 ? 0 : x.mant >> -x.exp2;
  bool frac = x.exp2 <= -64 ? true : (x.mant & ((1ull << -x.exp2) - 1)) != 0;
  if (x.sign && frac) mag += 1;
  if (mag == 0) return 0;  // floor of (0, 1) is +0
  return roundPack(x.sign, mag, 0);
}

inline uint32_t f2i(uint32_t a) {
  Unpacked x = unpack(a);
  if (x.kind == NaN || x.kind == Inf) return 0x80000000u;
  if (x.kind == Zero) return 0;
  uint64_t v = x.exp2 < 0 ? (x.exp2 <= -64 ? 0 : x.mant >> -x.exp2)
                          : (x.exp2 >= 40 ? (1ull << 40) : x.mant << x.exp2);
  if (v >= (1ull << 31)) return 0x80000000u;
  return x.sign ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

/// Cycles from start to done in fpu.v (its special cases finish early),
/// so the simulator reports exact cycle counts. `op` is the core opcode.
inline int fpuLatency(uint8_t op, uint32_t a, uint32_t b) {
  Unpacked x = unpack(a), y = unpack(b);
  bool special = x.kind != Num || y.kind != Num;
  switch (op) {
    case compiler::OP_FADD: case compiler::OP_FSUB: return special ? 1 : 3;
    case compiler::OP_FMUL: return special ? 1 : 3;  // products registered (hmul.v)
    case compiler::OP_FDIV: return special ? 1 : 29;
    case compiler::OP_FSQRT: return (x.kind != Num || x.sign) ? 1 : 31;
    case compiler::OP_I2F: return a == 0 ? 1 : 2;
    case compiler::OP_FFLOOR: {
      Unpacked f = unpack(a);
      if (f.kind != Num || f.exp2 >= 0) return 1;
      return ffloor(a) == 0 ? 1 : 2;
    }
    default: return 2;  // F2I (two stages: shift, negate)
  }
}

}  // namespace fp32
}  // namespace tangnanofaust
