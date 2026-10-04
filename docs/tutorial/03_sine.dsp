// 3. A sine oscillator from scratch: a phasor (0..1 sawtooth) into sin().
declare name "sine";
import("stdfaust.lib");
freq  = hslider("freq [unit:Hz]", 440, 20, 2000, 1);
phase = freq / ma.SR : (+ : ma.frac) ~ _;   // add freq/SR every sample, wrap at 1
process = sin(2 * ma.PI * phase) * 0.3;
