// Per-unit hardware test: each line below exercises one part of the DSP
// core on a pseudo-random sequence and folds the results into a checksum.
// At sample N the checksums are frozen in bargraphs; the simulator gives
// the expected values, so a unit that computes wrong results on the chip
// (e.g. too fast a clock) shows up as a mismatch. floor() has its own test,
// floor_test.dsp (with it here, Faust 2.70 stops with an internal assertion).
//   faust2tang tests/dsp/unit_test.dsp -o build/ut --port /dev/ttyUSB1 --load
//   faust2tang --port /dev/ttyUSB1 --get done --get alu --get mul ...
declare name "unit_test";
import("stdfaust.lib");

N = 100000;                                   // about 2 s at 48 kHz
n = (+(1) ~ _) - 1;                           // sample counter
r = n * 1103515245 + 12345;                   // pseudo-random int (MUL, ADD)
u = float(r & 65535) / 65536.0;               // 0 .. 1 (AND, I2F, FDIV by constant)
v = float((r >> 16) & 32767) / 4096.0 - 4.0;  // -4 .. 4 (ASR)

// integer checksums fold with xor and a rotate; float ones with a decay
ifold(x) = \(c).((((c << 1) | ((c >> 23) & 1)) xor x) & 16777215) ~ _;  // 24 bits: exact as float
ffold(x) = \(c).(c * 0.5 + x) ~ _;
freeze(x) = \(prev).(select2(n == N, prev, x)) ~ _;

alu   = (r xor (r >> 7)) + (r << 3) - (r & 255) : ifold;
mul   = r * (r >> 8) : ifold;
divi  = r / ((r & 1023) + 1) : ifold;
remi  = r % ((r & 1023) + 97) : ifold;
fadd  = u + v - (u - v * 0.25) : ffold;
fmul  = u * v * 1.0001 : ffold;
fdiv  = v / (u + 0.1) : ffold;
fsqrt = sqrt(u * 100.0) : ffold;
f2i   = int(v * 1000.0) : ifold;               // float -> int only
i2f   = float(r >> 9) : ffold;                 // int -> float only
cmp   = (u < 0.5) + (v >= 1.0) * 2 + (u == u) * 4 : ifold;
dly   = (u : @(37) : @(100)) : ffold;         // block RAM delay lines
sdr   = rwtable(65536, 0.0, n & 65535, u, (n + 1) & 65535) : ffold;  // SDRAM

process = 0
  : attach(_, (n >= N) : hbargraph("done", 0, 1))
  : attach(_, float(alu) : freeze : hbargraph("alu", -1e30, 1e30))
  : attach(_, float(mul) : freeze : hbargraph("mul", -1e30, 1e30))
  : attach(_, float(divi) : freeze : hbargraph("div", -1e30, 1e30))
  : attach(_, float(remi) : freeze : hbargraph("rem", -1e30, 1e30))
  : attach(_, fadd : freeze : hbargraph("fadd", -1e30, 1e30))
  : attach(_, fmul : freeze : hbargraph("fmul", -1e30, 1e30))
  : attach(_, fdiv : freeze : hbargraph("fdiv", -1e30, 1e30))
  : attach(_, fsqrt : freeze : hbargraph("fsqrt", -1e30, 1e30))
  : attach(_, float(f2i) : freeze : hbargraph("f2i", -1e30, 1e30))
  : attach(_, i2f : freeze : hbargraph("i2f", -1e30, 1e30))
  : attach(_, float(cmp) : freeze : hbargraph("cmp", -1e30, 1e30))
  : attach(_, dly : freeze : hbargraph("delay", -1e30, 1e30))
  : attach(_, sdr : freeze : hbargraph("sdram", -1e30, 1e30));
