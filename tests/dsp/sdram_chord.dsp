// SDRAM check with a real sound: the tutorial chord from a 64K sine table
// in SDRAM (as os.osc does). Each voice's table read is compared with the
// same value computed by sin() in the core, which is bit-identical when the
// read is right. Run it on the board and read the counters over USB:
//   faust2tang tests/dsp/sdram_chord.dsp -o build/sdc --port /dev/ttyUSB1 --load
//   faust2tang --port /dev/ttyUSB1 --get bad --get reads --get first_index \
//              --get first_got --get first_expected
// `bad` must stay 0; first_* describe the first bad read of voice 1.
declare name "sdram_chord";
import("stdfaust.lib");

ts = 65536;
root = hslider("root", 220, 50, 1000, 1);
volume = hslider("volume", 0.5, 0, 1, 0.01);
angle(k) = float(k) * (2.0 * ma.PI) / float(ts);
wave = ba.time : angle : sin;                       // table contents, filled at boot
phase(f) = f / ma.SR : (+ : ma.frac) ~ _;
index(f) = int(phase(f) * float(ts));
hold(c, x) = \(prev).(select2(c, prev, x)) ~ _;     // x from the last time c was 1

k1 = index(root);
k2 = index(root * 1.25);
k3 = index(root * 1.5);
t1 = rdtable(ts, wave, k1);
t2 = rdtable(ts, wave, k2);
t3 = rdtable(ts, wave, k3);
bad1 = t1 != (k1 : angle : sin);
bad = bad1 + (t2 != (k2 : angle : sin)) + (t3 != (k3 : angle : sin));
first = bad1 & ((bad1 : + ~ _) == 1);               // voice 1's first bad read

process = (t1 + t2 + t3) * 0.2 * volume
  : attach(_, bad : + ~ _ : hbargraph("bad", 0, 1e9))
  : attach(_, 3 : + ~ _ : hbargraph("reads", 0, 1e9))
  : attach(_, hold(first, k1) : hbargraph("first_index", 0, 1e9))
  : attach(_, hold(first, t1) : hbargraph("first_got", -2, 2))
  : attach(_, hold(first, k1 : angle : sin) : hbargraph("first_expected", -2, 2));
