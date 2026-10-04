// 5. A playable synth: band-limited sawtooth, envelope, resonant filter.
// freq/gain/gate are Faust's MIDI names: FaustMidi plays it from a keyboard.
declare name "synth";
import("stdfaust.lib");
freq   = hslider("freq", 220, 20, 2000, 0.01);
gain   = hslider("gain", 0.5, 0, 1, 0.01);
gate   = button("gate");
cutoff = hslider("cutoff [midi:ctrl 74]", 2000, 100, 8000, 1);
process = os.sawtooth(freq) * en.adsr(0.01, 0.2, 0.6, 0.4, gate) * gain
        : fi.resonlp(cutoff, 3, 1);
