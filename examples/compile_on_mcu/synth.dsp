// Monophonic subtractive synth: sawtooth -> ADSR -> resonant lowpass.
// freq/gain/gate follow Faust's MIDI naming, so NanoTangFaust's MIDI
// support plays it from note on/off messages.
declare name "synth";
import("stdfaust.lib");

freq = hslider("freq", 220, 20, 2000, 0.01);
gain = hslider("gain", 0.5, 0, 1, 0.01);
gate = button("gate");
cutoff = hslider("cutoff [midi:ctrl 74]", 2000, 100, 8000, 1) : si.smoo;
res = hslider("resonance [midi:ctrl 71]", 2, 0.7, 10, 0.01) : si.smoo;

process = os.sawtooth(freq) * en.adsr(0.01, 0.1, 0.8, 0.3, gate) * gain
        : fi.resonlp(cutoff, res, 1);
