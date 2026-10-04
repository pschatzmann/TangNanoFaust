# Writing Faust programs for TangNanoFaust

Any Faust program that the Faust compiler turns into interpreter bytecode
(`faust -lang interp`) can be compiled. There are four practical limits:
the time available per sample, memory, the stereo I2S I/O, and soundfiles,
which aren't supported.

## Audio inputs and outputs

| Faust | Hardware |
|---|---|
| input 0 / 1 | I2S input, left / right (FPGA pin 73, data only) |
| output 0 / 1 | I2S output to the onboard MAX98357A, left / right (one output only: both channels) |
| output 0..7 | TDM output slots 0..7, on a TDM bitstream (`TDM=1`) with an external TDM master connected (see [pinout](pinout.md#tdm-output)) |

Samples are 24-bit, ±1.0 full scale, and are clipped there; anything
louder is heard as distortion or loud pops. `faust2tang` doesn't check the
level, so try new programs at low volume first. Programs can have at most
2 inputs, and 2 outputs (8 on a TDM bitstream); `TangNanoFaust::load()`
rejects others.

Without TDM, the FPGA generates the sample clock (the rate given to
`faust2tang --sample-rate`, 48 kHz by default). With an external TDM
master, *its* frame rate is the sample rate. Compile the program for that
rate, because Faust bakes the sample rate into its coefficients.

## Time per sample

The DSP core runs at 48 MHz, so at 48 kHz each sample has **1000 clock
cycles** (more at lower sample rates: 1500 at 32 kHz). `faust2tang`
simulates the program and reports how many it needs, here for
`gateware/dsp/synth.dsp`:

```
cycles per sample: 741 of 1000 available at 48 MHz (74%)
after a parameter change: 801 cycles (80%)
```

- If **cycles per sample** is over budget, the output glitches all the time.
  Lower the sample rate (`--sample-rate 32000`) or simplify the program.
- If **after a parameter change** is over budget, one sample is dropped each
  time a parameter changes. See [smoothing](#smoothing-and-the-control-block).

Both numbers are the worst case over each parameter's range (math
functions take longer for some arguments).

At runtime, `TangNanoFaust::info()` reports the measured worst case
(`cyclesMax`, `load()`) and whether a sample overran.

### What things cost

| Operation | Cycles |
|---|---|
| integer ops, comparisons, `min`/`max` | 1 |
| float `+ -` / `*` | 4 / 4 |
| `/`, `sqrt` | ~30 |
| reading a variable / delay line in block RAM | 1-2 |
| ... as the operand of `+ - *` etc. (fused, see below) | +0-1 |
| `floor` (so `ma.frac`, phasors) | 3 |
| `rint`, `round`, `fmod` | 30-75 |
| `sin`, `cos` | ~200 (~120 with `--fast-math`) |
| `tan`, `exp`, `log`, `log10` | ~230 |
| `atan`, `atan2`, `asin`, `acos` | 150-250 |
| `sinh`, `cosh`, `tanh` | ~290 |
| `pow(x, y)` with integer `y` up to 16 (e.g. Faust's `x^3`) | ~130 |
| `pow(x, y)` in general | ~530 |
| SDRAM read / write (large delay lines and tables) | ~8 / ~26 |

The compiler folds constant array indices into plain loads and stores and
fuses a constant or a variable with the operator that uses it, so
`x * 0.5` and `y + z` cost one instruction less. The core reads the next
instruction's operand early (prefetch). Together this saves 30-50% of the
cycles of typical programs.

### Oscillators

| 440 Hz sine | Cycles | |
|---|---|---|
| `os.osc`, `os.oscsin` | 60 | 64K-entry table in SDRAM, filled at boot (0.3 s) |
| `sin(2*ma.PI*phase)` with `--fast-math` | 131 | error 4e-7 |
| `sin(2*ma.PI*phase)` | 217 | exact |

For many voices, use table oscillators. Ten table sines with a level each
(`tests/dsp/sines10.dsp`) need 434 cycles; ten voices of table oscillator
and ADSR (`voices10.dsp`) up to 1153, so they fit at 40 kHz and below; ten
voices with `sin()` and `--fast-math` (`voices10_sin.dsp`) about 2000.

`faust2tang --fast-math` (and `FaustCompiler::compile(..., fastMath)`)
replaces `sin`/`cos` with a polynomial that is accurate to 1.9e-7 for
arguments up to a few hundred radians, which covers oscillators and filter
coefficients. A parabolic approximation in Faust code (about 70 cycles)
has only -60 dB of distortion; it's fine for LFOs, but not for audible
sines.

### Smoothing and the control block

Faust splits a program into a *control* part, computed from the parameters,
and a per-sample part. The core runs the control part only in the sample
after a parameter changed. Expensive functions of a parameter (filter
coefficients with `tan`, `pow` for dB or frequencies) are therefore cheap,
**unless the parameter is smoothed** (`si.smoo`, `si.smooth`). A smoothed
value changes every sample, so everything computed from it moves into the
per-sample part. Example: `fi.lowpass(2, cutoff : si.smoo)` needs 559
cycles per sample, `fi.lowpass(2, cutoff)` 62.

## Memory

| Memory | Size | Used for |
|---|---|---|
| block RAM | 16384 words (generic bitstream) | all scalars, arrays that fit (smallest first) |
| SDRAM | 8 MB (2M words) | the remaining arrays: long delay lines, big tables |

`faust2tang` places everything automatically and prints the layout.
Common SDRAM users are `os.osc`/`os.oscsin` (a 65536-entry sine table) and
delays longer than a few thousand samples. A program that uses SDRAM needs a
bitstream with the SDRAM controller; the generic bitstream has it.

The core fills tables and clears delay lines itself after a program is
loaded (its *boot program*). A 64K sine table takes about 0.4 s, and
`TangNanoFaust::load()` waits for this.

## Numbers

All computation is IEEE-754 single precision (float32), rounded to nearest,
like a `-single` Faust build on a computer, with three differences:
- denormal numbers are flushed to zero;
- the math functions come from the core's own math library, accurate to a
  few units in the last place (within 4e-7 of the C library in the test
  suite);
- `fmod(x, y)` and `remainder()` are exact only while `|x/y|` < 2^23.

## Parameters

Sliders, buttons, checkboxes and numeric entries become parameters the
microcontroller can set by label or path; bargraphs can be read. Their
metadata is kept, for example:

```faust
freq   = hslider("freq [unit:Hz]", 440, 20, 2000, 1);
cutoff = hslider("cutoff [midi:ctrl 74]", 2000, 100, 8000, 1);
```

For [MIDI](arduino.md#midi), name the pitch, velocity and key parameters
`freq`, `gain` and `gate`, Faust's convention for synths, tag continuous
controls with `[midi:ctrl N]` and parameters played by one key (drums) with
`[midi:key N]`. For several instruments in one program, put each in its own
group, as `examples/midi_synth/multisynth.dsp` does.

## Examples

`gateware/dsp/`: `sine.dsp`, `synth.dsp` (sawtooth, ADSR, resonant lowpass),
`drive.dsp` (input effect), `echo.dsp` (SDRAM delay), `pluck.dsp`
(Karplus-Strong string), `passthrough.dsp`. `tests/dsp/` has the test
programs: freeverb, all math functions, the on-board self-tests, and load
tests such as `sines10.dsp` and `voices10.dsp`.
`examples/midi_synth/multisynth.dsp` is a multi-timbral synth (bass, lead,
plucked string, drums) that fits at 32 kHz; it shows cheap replacements for
filters and envelopes whose coefficients would otherwise be recomputed after
every parameter change.

Physical models such as `pm.ks` need an *excitation*: start each note
with a short burst (noise or an impulse), as `pluck.dsp` does. Feeding a
`button` into the model directly kicks it with a step at both edges, which
pops loudly.
