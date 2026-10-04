// Audio effect on the I2S input: lowpass + soft clipping distortion.
// The cutoff is deliberately not smoothed (si.smoo): a smoothed cutoff
// changes every sample, so Faust would recompute the filter coefficients
// (tan, pow) per sample -- 1400+ cycles, more than the 1000 available at
// 48 kHz. Unsmoothed, they are only recomputed when the parameter changes.
declare name "drive";
import("stdfaust.lib");

cutoff = hslider("cutoff [midi:ctrl 74]", 4000, 100, 15000, 1);
drive = hslider("drive [midi:ctrl 1]", 0.3, 0, 1, 0.01) : si.smoo;

process = fi.lowpass(2, cutoff) : ef.cubicnl(drive, 0);
