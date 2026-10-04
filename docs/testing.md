# Testing and status

## How it is tested

Everything is checked against two references: a **bit-exact simulator** of
the DSP core (`src/TangNanoFaust/compiler/Isa.h`, with the FPU
specification in `Fp32.h`), and an independent **reference interpreter**
that executes Faust's bytecode the way Faust's own interpreter does
(`tests/FbcInterpreter.h`, float32 with the C math library).

```bash
make -C tests check           # compiled programs on the simulator vs. the interpreter
make -C gateware sim          # RTL regression (iverilog); FULL=1 adds the slow programs
```

| Test | Compares | Programs |
|---|---|---|
| `dsp_test check` | simulator vs reference interpreter, 2000 samples, random input; maximum error must be below 1e-4 | all of `tests/fbc/` |
| `dsp_test --max-cycles` | performance guard: `voices10` (10 × table oscillator + ADSR) within 1125 cycles, `voices10_sin` with `--fast-math` within 1900, so changes that cost cycles show up | voices10, voices10_sin |
| `tb_fpu.v` | `fpu.v` vs `Fp32.h`, bit for bit | 32,000 vectors: random, special values, cancellation, overflow, denormal range |
| `tb_dsp_core.v` | `dsp_core.v` vs the simulator, **outputs and cycle counts** of every sample, bit for bit | gain_offset, lowpass, lowpass_cubic, saw_adsr, karplus, delay (SDRAM), math (all 26 math functions); with FULL=1 also freeverb, osc tables (64K-entry SDRAM tables) |
| `tb_top.v` | the whole design through its pins: boot, PING/INFO, load another program over SPI, descriptor read-back, parameter write + read-back, then 48 I2S frames in and out, bit for bit vs. the simulator | passthrough → gain_offset |
| `tb_top.v` over the USB UART (FULL=1) | the same sequence with every command sent as a UART frame at 115200 baud through `uart_bridge.v` | passthrough → gain_offset |
| `tb_top.v` without TDM (NOTDM=1) | the same sequence on a bitstream without TDM: 2 outputs, INFO reports 2 in / 2 out | passthrough → gain_offset |
| `tb_top.v` with a TDM master | as above, plus an external TDM master at 48 kHz and at 44.1 kHz: TDM slots 0..3 and the onboard I2S, bit for bit, at a constant latency, which shows the DSP follows the external clock | passthrough → tdm_test (4 outputs) |

`tests/fbc/` holds the bytecode of `tests/dsp/` (Faust 2.70.3), so the tests
don't need Faust installed. The `tb_dsp_core.v` SDRAM is a model with fixed
latency; `+sd_jitter=N` adds 0..N random cycles per access, as refreshes do
on the chip.

## On the board

Simulation can't show whether the chip keeps up with the clock. These
programs check themselves on the board and report through bargraphs, read over USB with
`faust2tang --port /dev/ttyUSB1 --get <name>` (which also prints the raw
bits):

| Program | Checks | Pass |
|---|---|---|
| `tests/dsp/unit_test.dsp` | each unit (integer ALU, multiplier, remainder, FPU add/multiply/divide/sqrt, float↔int, comparisons, block RAM delay lines, SDRAM) folds 100,000 results into a checksum, frozen at sample 100,000 | every checksum equals the simulator's (`done` = 1) |
| `tests/dsp/floor_test.dsp` | `floor()` against a reference from integer conversion; a phasor's maximum | `bad` = 0, `phase_max` < 1 |
| `tests/dsp/sdram_test.dsp` | SDRAM written while running and read back a lap later; a table filled at boot | `errors` = `boot_errors` = 0 |
| `tests/dsp/sdram_chord.dsp` | a chord from a 64K sine table in SDRAM, every read compared with `sin()` | `bad` = 0 |
| `tests/build/midi_test` (`tests/host/`) | `FaustMidi` and `TangNanoFaust` compiled for the PC (with Arduino API stand-ins) drive the board over its USB port: a melody, a CC 74 filter sweep and a pitch bend into `synth.dsp` | parameters follow the MIDI; listen |
| `tests/dsp/sines10.dsp` | 10 sine oscillators with their own frequency and level, mixed | listen |

The expected `unit_test` checksums come from the simulator (`Isa.h`):
`tests/build/dsp_test params build/ut/dsp.fbc 100100`. A unit that fails
at one clock and passes at a lower one has a path that is too slow on the
chip.

Measured with these tests:
- the math library is within 3.6e-7 of the C library on `tests/dsp/math.dsp`;
- the FPU is IEEE-754 round-to-nearest-even for every result that isn't
  denormal;
- the compiler gives the same program for the same bytecode on every
  platform, because it is plain C++ with its own float rounding.

## Status

**Simulation:** everything in the first table passes.

**On a Tang Nano 20K**, with the generic bitstream at 48 MHz (board on USB
only, controlled with `faust2tang --port`):

- All self-tests pass: `unit_test`, `floor_test`, `sdram_test`,
  `sdram_chord`, also on the generic TDM bitstream (without a TDM master).
  At 54 MHz they fail, which is why 48 MHz is the highest `SYS_MHZ`.
- Programs load over USB one after another, and the cycle counts the
  board measures equal the simulator's.
- Listening tests sound as intended: `sine.dsp`, `synth.dsp` (attack and
  release), the tutorial chord (`os.osc`, a sine table in SDRAM),
  `pluck.dsp`, `tests/dsp/osc_noise_cubic.dsp` (tone and noise),
  `sines10.dsp` (10 sines at 48 kHz), `voices10.dsp` (10 voices with ADSR,
  at 32 kHz where it fits) and the MIDI test on `synth.dsp`.
- `tests/dsp/karplus.dsp` pops between notes, on the board as in the
  simulation: it feeds the gate step itself into the string (see
  `pluck.dsp` for the musical version).

**Compiled:** the Arduino library and all examples for ESP32 and RP2040
(`wifi_compile` for ESP32 and Pico W).

**Compile server:** `faust2tang --serve` returns programs that are
byte-identical to local compiles, and `FaustServerCompiler` runs against
it on a PC (with stubbed Arduino headers and a socket `Client` that writes
in short chunks): program, report, a Faust error, and an unreachable
server. `tests/build/blob_test` checks the program blob round trip for
every test program (`make -C tests check`).

**Not tested on hardware:** a microcontroller on SPI or the header UART,
TDM with a real codec, and a MIDI input circuit (the MIDI test runs the
library's MIDI and serial code on a PC).
