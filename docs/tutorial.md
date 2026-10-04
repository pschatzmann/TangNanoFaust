# Faust and DSP tutorial

This tutorial introduces the [Faust](https://faust.grame.fr) language and the
basic ideas of digital signal processing (DSP) by building sounds on the
Tang Nano 20K. Each step is a small program in
[docs/tutorial/](tutorial/), and all of them run on the FPGA. The cycle
counts are what `faust2tang` reports at 48 kHz, where each sample may use
1000 cycles.

You need the generic bitstream on the board ([getting started](getting-started.md))
and a speaker on its amplifier connector. To run a step from your PC over
the board's USB port:

```bash
tools/bin/faust2tang docs/tutorial/03_sine.dsp -o build/tut --port /dev/ttyUSB1 --load
tools/bin/faust2tang --port /dev/ttyUSB1 --set freq=330      # change a parameter
tools/bin/faust2tang --port /dev/ttyUSB1 --mute              # silence (--unmute)
```

Or from an Arduino sketch, see [Arduino library](arduino.md). A good
companion is the [Faust online IDE](https://faustide.grame.fr), which plays
the same programs in your browser and draws their block diagrams.

## How Faust thinks

A digital audio signal is a stream of numbers, the *samples*: 48000 per
second at 48 kHz, each between -1 and 1. A Faust program describes **one
computation that runs for every sample**. It's written as a block diagram,
where boxes have inputs and outputs and you wire them together with five
operators:

| Operator | Meaning | Example |
|---|---|---|
| `A : B` | sequence: A's outputs into B's inputs | `_ : *(0.5)` |
| `A , B` | parallel: side by side | `_ , _` (two channels) |
| `A <: B` | split: copy A's outputs to all of B's inputs | `_ <: _, _` (mono to stereo) |
| `A :> B` | merge: add A's outputs into B's inputs | `_, _ :> _` (mix to mono) |
| `A ~ B` | recursion: feed A's output back through B (one sample later) | `+ ~ _` |

`_` is a wire (the signal unchanged), and `!` cuts a signal. `process` is
the program's top-level diagram: its inputs come from the I2S input, and
its outputs go to the amplifier (outputs 0 and 1), and to TDM if the
bitstream has it.

## 1. A wire: `01_passthrough.dsp` (16 cycles)

```faust
process = _;
```

One input, one output: whatever arrives on the I2S input is played. It's
the "hello world" of audio. It also shows that the FPGA processes samples
one at a time, every 21 µs.

## 2. Gain and parameters: `02_gain.dsp` (21 cycles)

```faust
process = _ * hslider("gain", 0.5, 0, 1, 0.01);
```

`_ * x` multiplies every sample by `x`. That's volume. `hslider` (label,
default, min, max, step) declares a **parameter**: the FPGA lists it, and
you set it with `--set gain=0.2` or `faust.setParameter("gain", 0.2)`.
`button` and `checkbox` work the same way.

*DSP idea:* multiplying by 0.5 is −6 dB, and multiplying by 0.1 is −20 dB.
Loudness is perceived logarithmically, which is why audio uses decibels:
dB = 20·log₁₀(gain).

## 3. A sine oscillator: `03_sine.dsp` (222 cycles)

```faust
import("stdfaust.lib");
freq  = hslider("freq [unit:Hz]", 440, 20, 2000, 1);
phase = freq / ma.SR : (+ : ma.frac) ~ _;
process = sin(2 * ma.PI * phase) * 0.3;
```

`import("stdfaust.lib")` loads Faust's standard libraries, used with
prefixes: `ma.` for math, `os.` for oscillators, `fi.` for filters, `en.`
for envelopes, `de.` for delays, `re.` for reverbs and `si.` for signal tools.
`ma.SR` is the sample rate.

The interesting line is `phase`. `(+ : ma.frac) ~ _` adds its input to
its own previous output and keeps only the fractional part. With
`freq / ma.SR` as input, it climbs from 0 to 1 `freq` times per second: a
*phasor*. `sin(2π·phase)` turns that into a sine.

*DSP ideas:* this is how every oscillator works, as a phase that advances
by frequency/sample rate each sample. The highest frequency a sample rate
can represent is half of it, the *Nyquist frequency* (24 kHz at 48 kHz).

On this FPGA, `sin()` costs about 200 cycles. `os.osc(freq)` does the
same with a table (60 cycles), and `faust2tang --fast-math` makes `sin`
about 40% cheaper (see [Writing Faust programs](faust.md#oscillators)).

## 4. Recursion and filters: `04_onepole.dsp` (36 cycles)

```faust
a = hslider("smoothness", 0.9, 0, 0.999, 0.001);
process = *(1 - a) : + ~ *(a);
```

`+ ~ *(a)` is the recursion operator at work: the output is fed back,
multiplied by `a` and added to the next input, so y[n] = x[n] + a·y[n−1].
That's a **one-pole lowpass filter**: it averages, letting slow changes
(low frequencies) through and smoothing fast ones (high frequencies). The
closer `a` is to 1, the darker the sound. Play it on the I2S input, or put
it after the oscillator (`... : *(1 - a) : + ~ *(a)`).

*DSP idea:* every filter is built from delays, multiplications and
additions. The library has ready-made ones, such as `fi.lowpass(order,
cutoff)`, `fi.highpass`, `fi.resonlp(cutoff, q, gain)` and `fi.peak_eq`.
`si.smoo` is this one-pole filter with a = 1 − 44.1/SR (about 0.999); it
removes "zipper noise" when a parameter jumps.

## 5. A synthesizer: `05_synth.dsp` (183 cycles)

```faust
freq   = hslider("freq", 220, 20, 2000, 0.01);
gain   = hslider("gain", 0.5, 0, 1, 0.01);
gate   = button("gate");
cutoff = hslider("cutoff [midi:ctrl 74]", 2000, 100, 8000, 1);
process = os.sawtooth(freq) * en.adsr(0.01, 0.2, 0.6, 0.4, gate) * gain
        : fi.resonlp(cutoff, 3, 1);
```

This is subtractive synthesis:
1. **The oscillator.** `os.sawtooth` produces a bright tone rich in harmonics.
2. **The envelope.** `en.adsr(attack, decay, sustain, release, gate)` shapes
   the volume: it rises when `gate` goes to 1, falls to the sustain level,
   and fades out when `gate` returns to 0.
3. **The filter.** `fi.resonlp` removes the high harmonics above `cutoff`
   and emphasizes the frequencies around it.

Set `gate=1` and then `gate=0` to play a note.

The names `freq`, `gain` and `gate` are Faust's convention for
instruments. `FaustMidi` maps note on/off to them, and the metadata
`[midi:ctrl 74]` maps a MIDI controller (often the filter knob) to
`cutoff`. See [MIDI](arduino.md#midi).

*DSP idea:* a naive sawtooth (`phase * 2 - 1`) contains harmonics above
Nyquist that fold back as inharmonic noise, called *aliasing*.
`os.sawtooth` is band-limited to avoid that.

## 6. An effect: `06_echo.dsp` (92 cycles)

```faust
time     = hslider("time [unit:ms]", 300, 1, 1000, 1) * ma.SR / 1000;
feedback = hslider("feedback", 0.4, 0, 0.9, 0.01);
process  = _ <: _, (+ ~ (de.delay(65536, time) * feedback)) :> *(0.5);
```

The input is split (`<:`) into a dry path and an echo path, then mixed back
(`:>`). The echo path is a recursion around a delay line: every repeat is
`feedback` times quieter. `de.delay(maxsize, n)` delays by `n` samples.
Here that's up to 65536, more than a second, which needs 128K words of
memory, so `faust2tang` puts it in the board's SDRAM automatically.

*DSP idea:* delays are the building block of echoes, choruses, flangers,
comb filters and reverbs (`re.mono_freeverb`). Keep feedback below 1, or
the loop grows without bound.

## 7. Several voices: `07_chords.dsp` (149 cycles)

```faust
root = hslider("root", 220, 50, 1000, 1);
voice(ratio) = os.osc(root * ratio) * 0.2;
process = par(i, 3, voice(ba.take(i + 1, (1, 1.25, 1.5)))) :> _;
```

`voice(ratio)` is a *function*: Faust code is reusable like that.
`par(i, N, expr)` puts N copies side by side with `i` = 0…N−1, and `:> _`
mixes them. The ratios 1 : 1.25 : 1.5 make a major chord, and
`ba.take(k, list)` picks the k-th element.

On this FPGA, 10 table oscillators use less than half of the 1000 cycles
at 48 kHz (`tests/dsp/sines10.dsp`); with an ADSR each they need about
1150 and fit at 40 kHz (`tests/dsp/voices10.dsp`). Watch the "cycles per
sample" line in the `faust2tang` report as you add voices.

## 8. Stereo: `08_stereo.dsp` (133 cycles)

```faust
rate = hslider("rate [unit:Hz]", 0.5, 0.05, 5, 0.01);
pan  = (os.osc(rate) + 1) / 2;
process = os.osc(330) * 0.3 <: *(1 - pan), *(pan);
```

Two outputs mean stereo: output 0 is left and output 1 is right. A slow
oscillator (an *LFO*, low-frequency oscillator) moves `pan` between 0 and
1, and the tone moves between the speakers. With a TDM bitstream and a TDM
codec attached, up to 8 outputs are possible ([pinout](pinout.md#tdm-output)).

## Next steps

- Explore the [Faust libraries documentation](https://faustlibraries.grame.fr):
  oscillators, filters, effects, physical models (`pm.`), analyzers (`an.`).
- Read [Writing Faust programs](faust.md) for what costs how much on the
  FPGA, and how to stay within the cycle budget.
- More examples: `gateware/dsp/` and `tests/dsp/` (freeverb,
  Karplus-Strong plucked string, all math functions).
- The [Faust manual](https://faustdoc.grame.fr/manual/syntax/) covers the
  whole language: pattern matching, iterations (`seq`, `sum`, `prod`),
  `rdtable`/`rwtable`, and more.
