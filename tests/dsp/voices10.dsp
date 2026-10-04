// 10 voices of (sine table oscillator -> ADSR -> gain), mixed. Performance
// test (run_regression.sh guards its cycles per sample).
declare name "voices10";
import("stdfaust.lib");
voice(i) = osc * en.adsr(0.01, 0.1, 0.7, 0.3, gate) * gain
with {
  f = hslider("v%i/freq", 220 + 50*i, 20, 2000, 0.01);
  gain = hslider("v%i/gain", 0.5, 0, 1, 0.01);
  gate = button("v%i/gate");
  phase = f / ma.SR : (+ : ma.frac) ~ _;
  osc = rdtable(4096, sin(2*ma.PI*float(ba.time)/4096), int(4096*phase));
};
process = par(i, 10, voice(i)) :> *(0.1);
