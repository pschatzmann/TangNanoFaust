// floor() self-test on the board: floor (FFLOOR) against a reference from
// float/int conversions, over positive and negative values, plus the
// largest value a phasor (ma.frac) ever reaches, which must stay below 1.
//   faust2tang tests/dsp/floor_test.dsp -o build/flt --port /dev/ttyUSB1 --load
//   faust2tang --port /dev/ttyUSB1 --get bad --get checked --get first_x \
//              --get first_floor --get phase_max
declare name "floor_test";
import("stdfaust.lib");

hold(c, x) = \(prev).(select2(c, prev, x)) ~ _;
i = (+(1) ~ _) - 1;
x = float(i % 100003) * 0.000731 - 36.5;            // -36.5 .. 36.6, many fractions
ref(v) = float(int(v)) - (float(int(v)) > v);       // floor via truncation
bad = floor(x) != ref(x);
first = bad & ((bad : + ~ _) == 1);
phase = 440 / ma.SR : (+ : ma.frac) ~ _;

process = 0
  : attach(_, bad : + ~ _ : hbargraph("bad", 0, 1e9))
  : attach(_, 1 : + ~ _ : hbargraph("checked", 0, 1e9))
  : attach(_, hold(first, x) : hbargraph("first_x", -100, 100))
  : attach(_, hold(first, floor(x)) : hbargraph("first_floor", -100, 100))
  : attach(_, phase : max ~ _ : hbargraph("phase_max", 0, 1e9));
