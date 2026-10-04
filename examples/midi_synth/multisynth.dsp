// Multi-timbral synthesizer: four instruments, each in its own group so
// that the midi_synth sketch can play each one from its own MIDI channel.
//   bass  (channel 1)   sawtooth -> resonant lowpass
//   lead  (channel 2)   two detuned sawtooths with vibrato (mod wheel) -> lowpass
//   pluck (channel 3)   Karplus-Strong string
//   drums (channel 10)  kick 36, snare 38, closed hi-hat 42, open hi-hat 46
// freq/gain/gate and [midi:ctrl N] follow Faust's MIDI conventions, and the
// drums use [midi:key N]; CC 7 sets each instrument's volume.
declare name "multisynth";
import("stdfaust.lib");

volume(v) = hslider("volume [midi:ctrl 7]", v, 0, 1, 0.01);

// Sine table in block RAM (shared by the kick and the vibrato), phase 0..1
sine(phase) = rdtable(4096, sin(2*ma.PI*float(ba.time)/4096), int(4096*phase));
phasor(f) = f/ma.SR : (+ : ma.frac) ~ _;

// Exponential decay with time constant tau, started by each new hit of the
// key parameter t (its value is the velocity).
decay(tau, t) = (t > t') * t : (max ~ *(ba.tau2pole(tau)));

// Envelope: rises with time constant a while gate is held, decays with
// time constant d to the sustain level s, falls with time constant r after
// the release. Cheaper than en.adsr(), which recomputes its segments after
// every parameter change.
adsr(a, d, s, r, gate) = gate * (s + (1 - s) * decay(d, gate)) : si.smooth(pole)
with {
  pole = ba.if(gate > 0, ba.tau2pole(a), ba.tau2pole(r));
};

// Resonant lowpass: state-variable filter (Zavalishin). Its coefficient
// tan(pi*fc/SR) comes from a table filled at boot (steps of SR/4096, up to
// SR/4), so a cutoff change costs no tan(). The table is indexed by fc/SR
// because ma.SR is not yet known when Faust fills tables.
svflp(fc, q) = (tick ~ (_, _)) : (!, !, _)
with {
  g = rdtable(1024, tan(ma.PI * float(ba.time) / 4096), min(1023, int(fc / ma.SR * 4096)));
  a1 = 1 / (1 + g * (g + 1/q));
  a2 = g * a1;
  a3 = g * a2;
  tick(ic1, ic2, x) = 2 * v1 - ic1, 2 * v2 - ic2, v2
  with {
    v3 = x - ic2;
    v1 = a1 * ic1 + a2 * v3;
    v2 = ic2 + a2 * ic1 + a3 * v3;
  };
};

// Karplus-Strong string: a delay line of one period with a two-point
// average (lowpass) and a loss factor in the loop.
string(freq, damping) = + ~ (de.fdelay(2048, ma.SR / freq - 1.5) : avg * loss)
with {
  avg(x) = (x + x') * 0.5;
  loss = 0.999 - 0.02 * damping;
};

bass = vgroup("bass", os.sawtooth(freq) * env * gain * volume(0.4)
                      : svflp(cutoff, res))
with {
  freq = hslider("freq", 110, 20, 1000, 0.01);
  gain = hslider("gain", 0.8, 0, 1, 0.01);
  gate = button("gate");
  cutoff = hslider("cutoff [midi:ctrl 74]", 800, 50, 5000, 1);
  res = hslider("resonance [midi:ctrl 71]", 2, 0.7, 8, 0.01);
  env = adsr(0.002, 0.2, 0.6, 0.05, gate);
};

lead = vgroup("lead", (os.sawtooth(f * 0.997) + os.sawtooth(f * 1.003))
                      * env * gain * volume(0.2) : svflp(cutoff, 0.7))
with {
  freq = hslider("freq", 440, 20, 4000, 0.01);
  gain = hslider("gain", 0.8, 0, 1, 0.01);
  gate = button("gate");
  vibrato = hslider("vibrato [midi:ctrl 1]", 0, 0, 1, 0.01);
  cutoff = hslider("cutoff [midi:ctrl 74]", 3000, 200, 8000, 1);
  f = freq * (1 + 0.02 * vibrato * sine(phasor(5.5)));
  env = adsr(0.01, 0.1, 0.8, 0.15, gate);
};

// Each rising edge of gate plucks the string with a short noise burst.
pluck = vgroup("pluck", burst : string(freq, damping) * volume(0.6))
with {
  freq = hslider("freq", 220, 30, 2000, 0.01);
  gain = hslider("gain", 0.8, 0, 1, 0.01);
  gate = button("gate");
  damping = hslider("damping [midi:ctrl 74]", 0.3, 0, 1, 0.01);
  burst = no.noise * decay(0.002, gate) * gain;
};

drums = vgroup("drums", (kick + snare + hat + openhat) * volume(0.6))
with {
  kickKey = button("kick [midi:key 36]");
  snareKey = button("snare [midi:key 38]");
  hatKey = button("hihat [midi:key 42]");
  openKey = button("open hihat [midi:key 46]");
  kick = sine(phasor(45 + 120 * decay(0.03, kickKey))) * decay(0.15, kickKey);
  snare = no.noise * decay(0.07, snareKey) * 0.6
        + sine(phasor(185)) * decay(0.04, snareKey) * 0.5;
  hiss = no.noise : fi.highpass(2, 7000);
  hat = hiss * decay(0.03, hatKey) * 0.5;
  openhat = hiss * decay(0.3, openKey) * 0.4;
};

// Soft clipping keeps the sum of all instruments within full scale.
softclip(x) = 1.5 * y - 0.5 * y * y * y with { y = max(-1, min(1, x)); };

process = bass + lead + pluck + drums : softclip;
