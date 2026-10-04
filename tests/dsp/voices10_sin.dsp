// 10 voices of (sin() oscillator -> ADSR -> gain), mixed: performance test
// for --fast-math.
declare name "voices10_sin";
import("stdfaust.lib");
voice(i) = osc * en.adsr(0.01, 0.1, 0.7, 0.3, gate) * gain
with {
  f = hslider("v%i/freq", 220 + 50*i, 20, 2000, 0.01);
  gain = hslider("v%i/gain", 0.5, 0, 1, 0.01);
  gate = button("v%i/gate");
  phase = f / ma.SR : (+ : ma.frac) ~ _;
  osc = sin(2*ma.PI*phase);
};
process = par(i, 10, voice(i)) :> *(0.1);
