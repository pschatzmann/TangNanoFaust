// FPU test vectors from the bit-exact specification (src/.../compiler/Fp32.h)
// for gateware/tb/tb_fpu.v.
//   fpu_vectors gen out.hex [n]   write n vectors per operation
//   fpu_vectors check in.hex      recompute the expected column of a file
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <random>
#include <vector>

#include "TangNanoFaust/compiler/Fp32.h"

using namespace tangnanofaust;

static uint32_t eval(uint32_t op, uint32_t a, uint32_t b) {
  switch (op) {
    case 0: return fp32::fadd(a, b);
    case 1: return fp32::fsub(a, b);
    case 2: return fp32::fmul(a, b);
    case 3: return fp32::fdiv(a, b);
    case 4: return fp32::fsqrt(a);
    case 5: return fp32::i2f(a);
    case 6: return fp32::f2i(a);
    default: return fp32::ffloor(a);
  }
}

static const uint32_t kSpecial[] = {
    0x00000000, 0x80000000, 0x3F800000, 0xBF800000, 0x7F800000, 0xFF800000, 0x7FC00000,
    0x7F800001, 0x00000001, 0x807FFFFF, 0x00800000, 0x80800000, 0x7F7FFFFF, 0xFF7FFFFF,
    0x4F000000, 0xCF000000, 0x4EFFFFFF, 0x3F000000, 0x34000000, 0x0C000000, 0x73000000,
    0x7FFFFFFF, 0x80000001};

static uint32_t bitsOf(float f) {
  uint32_t b;
  memcpy(&b, &f, 4);
  return b;
}

static uint32_t operand(std::mt19937 &rng) {
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  switch (rng() % 5) {
    case 0: return rng();                          // any bit pattern
    case 1: return bitsOf(u(rng) * 4.0f);          // similar magnitudes
    case 2: return bitsOf(u(rng) * 1e-36f);        // near the denormal range
    case 3: return bitsOf(u(rng) * 3e38f);         // near overflow
    default: return kSpecial[rng() % (sizeof(kSpecial) / 4)];
  }
}

int main(int argc, char **argv) {
  if (argc >= 3 && !strcmp(argv[1], "gen")) {
    int n = argc > 3 ? atoi(argv[3]) : 4000;
    std::mt19937 rng(42);
    FILE *f = fopen(argv[2], "w");
    for (uint32_t op = 0; op < 8; op++) {
      for (int i = 0; i < n; i++) {
        uint32_t a = op == 5 ? (uint32_t)rng() : operand(rng);
        uint32_t b = operand(rng);
        if (op <= 1 && i % 7 == 0) b = a ^ (op == 1 ? 0 : 0x80000000u);        // cancellation
        if (op <= 1 && i % 11 == 3) b = (a ^ (op == 1 ? 0 : 0x80000000u)) + 1;  // near it
        if (op == 5 && i < 4) a = (uint32_t[]){0, 0x80000000u, 0x7FFFFFFFu, 0xFFFFFFFFu}[i];
        fprintf(f, "%08x\n%08x\n%08x\n%08x\n", op, a, b, eval(op, a, b));
      }
    }
    fclose(f);
    printf("%d vectors -> %s\n", 8 * n, argv[2]);
    return 0;
  }
  if (argc >= 3 && !strcmp(argv[1], "check")) {
    FILE *f = fopen(argv[2], "r");
    unsigned op, a, b, e;
    long n = 0, bad = 0;
    while (fscanf(f, "%x %x %x %x", &op, &a, &b, &e) == 4) {
      uint32_t got = eval(op, a, b);
      if (got != e && ++bad <= 10)
        printf("op=%u a=%08x b=%08x file=%08x c++=%08x\n", op, a, b, e, got);
      n++;
    }
    printf("%ld vectors, %ld differ\n", n, bad);
    return bad ? 1 : 0;
  }
  fprintf(stderr, "usage: %s gen out.hex [n] | check in.hex\n", argv[0]);
  return 2;
}
