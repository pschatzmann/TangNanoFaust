// 10 sine oscillators (a 4096-entry table each read with its own phase),
// each with frequency and level, mixed: a load test that fits 48 kHz.
// The default frequencies form a chord over two octaves (A major).
declare name "sines10";
import("stdfaust.lib");
freqs = (110, 220, 277.18, 329.63, 440, 554.37, 659.26, 880, 1108.73, 1318.51);
voice(i) = osc * level
with {
  f = hslider("v%i/freq", ba.take(i + 1, freqs), 20, 4000, 0.01);
  level = hslider("v%i/level", 1, 0, 1, 0.01);
  phase = f / ma.SR : (+ : ma.frac) ~ _;
  osc = rdtable(4096, sin(2 * ma.PI * float(ba.time) / 4096), int(4096 * phase));
};
process = par(i, 10, voice(i)) :> *(0.08) <: _, _;
