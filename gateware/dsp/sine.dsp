// Sine oscillator computed with sin() on the core (no table, so it fits in
// block RAM and needs no SDRAM).
declare name "sine";
import("stdfaust.lib");

freq = hslider("freq [unit:Hz] [midi:ctrl 1]", 440, 20, 4000, 1) : si.smoo;
gain = hslider("gain [midi:ctrl 7]", 0.3, 0, 1, 0.01) : si.smoo;
phase = freq / ma.SR : (+ : ma.frac) ~ _;

process = sin(2 * ma.PI * phase) * gain;
