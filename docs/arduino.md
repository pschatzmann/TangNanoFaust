# Arduino library

`#include <NanoTangFaust.h>` provides `TangNanoFaust` (the FPGA over SPI
or a UART) and `FaustMidi`. `#include <NanoTangFaustCompiler.h>` adds
`FaustCompiler`, which compiles Faust bytecode on the microcontroller; it
needs the C++ standard library (ESP32, RP2040, ...).
`#include <NanoTangFaustServer.h>` adds `FaustServerCompiler`, which
compiles over the network. Everything is in the namespace `nanotangfaust`.

## TangNanoFaust

```cpp
TangNanoFaust faust;
faust.begin(SPI, CS_PIN);            // optional 3rd argument: SPI clock, default 2 MHz, max 3 MHz
// or over a UART: MCU TX -> FPGA pin 25, MCU RX <- FPGA pin 26
Serial1.begin(115200);
faust.begin(Serial1);
```

`begin()` checks that the FPGA answers and reads the list of parameters of
the program it is running. The FPGA describes its parameters itself, so a
sketch needs no header to use them.

| Method | |
|---|---|
| `load(program)` | upload a compiled program (a `ProgramData`), wait until it runs, re-read its parameters. Returns a `LoadResult`. |
| `parameterCount()`, `parameter(i)` | the parameters: `label()`, `path`, `kind`, `min`/`max`/`init`/`step`, `metaValue("midi", ...)` |
| `findParameter("freq")` | index by label or path, -1 if unknown |
| `setParameter(i or "label", value)` | clamped to the parameter's range; takes effect from the next sample |
| `getParameter(i or "label")` | current value, also of bargraphs (outputs of the DSP) |
| `info()` | `Info`: channels, sample rate, `booted`, `overrun`, `tdmActive`, `cyclesMax`/`load()` since the last call, capacities, `maxInputs`/`maxOutputs` and `tdm` (what the bitstream has) |
| `setMute(bool)`, `setAmplifier(bool)` | silence the output / switch the MAX98357A |
| `reset()` | restart the program: parameters back to their defaults, delay lines cleared |
| `name()`, `ping()` | program name, presence check |

### Loading programs

`faust2tang my.dsp -o build/` writes `build/my_program.h` with a
`ProgramData` named `my::program` (the namespace comes from the program
name, so `declare name "synth";` gives `synth::program`):

```cpp
#include "synth_program.h"
...
LoadResult r = faust.load(synth::program);
```

`load()` checks the program fits the bitstream: its instruction memory,
block RAM, descriptor size, whether SDRAM is needed, and the number of
inputs and outputs the bitstream has (2 and 2, or 2 and 8 with TDM). It
then stops the DSP, uploads, restarts and waits for the program's boot
(filling tables, at most a fraction of a second). The
generic bitstream accepts any program; a bitstream built for one program
only accepts programs that fit its sizes.

## Compiling on the microcontroller

```cpp
#include <NanoTangFaustCompiler.h>
#include "synth_fbc.h"            // faust2tang --sketch synth.dsp

FaustCompiler faustCompiler;
if (!faustCompiler.compileAndLoad(faust, synth_fbc))
  Serial.println(faustCompiler.error());
```

`compileAndLoad()` sizes the program for the connected FPGA, so it uses
the FPGA's block RAM and its SDRAM if it has one. `compile()` and
`programData()` split the two steps. `image()` gives the details: program
size, memory use and parameters.

The input is Faust *interpreter bytecode* (FBC): `faust -lang interp
-double`, which `faust2tang --sketch my.dsp` writes as a header. It could
also come from an SD card or the network (see below). The Faust compiler
itself only runs on a computer. It is a large program, about 7 MB of code, ~14 MB of
memory and 1.7 MB of library sources, while `FaustCompiler` takes about
100 KB of flash and some tens of KB of RAM while compiling.

## Compiling over the network

The Faust compiler can't run on the MCU, but it can run on a computer on
your network. `faust2tang --serve 8000` is a small compile server (see
[faust2tang](faust2tang.md#compile-server)), and `FaustServerCompiler`
sends it Faust source code and loads the answer:

```cpp
#include <NanoTangFaustServer.h>
#include <WiFi.h>

WiFiClient client;
FaustServerCompiler server(client, "http://192.168.1.10:8000");

// after WiFi and faust.begin():
if (!server.compileAndLoad(faust, "import(\"stdfaust.lib\"); process = os.osc(440) * 0.3;"))
  Serial.println(server.error());   // e.g. Faust's error message, with line number
```

The server compiles for the connected bitstream: `compileAndLoad()` sends
along its sample rate, block RAM size, SDRAM and clock from `info()`.

| Method | |
|---|---|
| `compileAndLoad(faust, source)` | the server compiles everything and returns the finished program (a few KB); the MCU only loads it. Needs no C++ standard library. |
| `compileAndLoad(faust, source, faustCompiler)` | the server only runs Faust and returns the bytecode; a `FaustCompiler` (`NanoTangFaustCompiler.h`) compiles it on the MCU |
| `compile(info, source, out)` | only fetch: `out` = `"program"`, `"fbc"` or `"report"`; the answer is in `data()`/`size()` |
| `error()`, `status()` | why the last call failed, and the HTTP status |
| `cycles()`, `controlCycles()` | cycles per sample of the compiled program, simulated by the server (normal / after a parameter change) |
| `setTimeout(ms)`, `setFastMath(on)` | default 30 s; faster `sin`/`cos` |

It speaks plain HTTP over any Arduino `Client`: `WiFiClient` on an ESP32
or Pico W, `EthernetClient`, and so on. A received program can also be
loaded directly: `parseProgramBlob(data, size, program)` fills a
`ProgramData` for `load()` from the server's `out=program` answer, for
example after storing it on an SD card.

## MIDI

`FaustMidi` plays the program from MIDI bytes read from any `Stream`:

```cpp
FaustMidi midi(faust);

void setup() {
  Serial1.begin(31250);        // DIN MIDI input circuit on RX1
  faust.begin(SPI, CS_PIN);
  midi.begin(Serial1);         // optional 2nd argument: channel 1-16, default all
}
void loop() { midi.update(); }
```

| MIDI | Faust parameter |
|---|---|
| note on/off | `freq` (Hz), `gain` (velocity / 127), `gate` (1 while a key is held) |
| control change N | every parameter with `[midi:ctrl N]`, scaled to its range |
| pitch bend | a parameter with `[midi:pitchwheel]`, otherwise bends `freq` (±2 semitones, `setBendRange()`) |
| CC 123 (all notes off) | `gate` = 0 |

It is monophonic with last-note priority. Playing a new note while another
is held changes the pitch without re-triggering the envelope (legato).
`noteOn()`, `noteOff()`, `controlChange()` and `pitchBend()` are public, so
events from another MIDI library (USB MIDI, BLE MIDI) can be passed in too.

Each parameter change is one short SPI transaction, which takes about 30 µs
at 2 MHz.

## Examples

- `basic`: list the parameters, sweep `freq`, print the DSP load
- `midi_serial`: MIDI over `Serial` (serial-MIDI bridge) or `Serial1` (DIN)
- `compile_on_mcu`: compile `synth.dsp`'s bytecode on the MCU and play it
- `serial_control`: control the FPGA over a UART (`begin(Serial1)`) instead of SPI
- `wifi_compile`: an ESP32 sends Faust source to `faust2tang --serve` and
  plays the result; paste new programs into the serial monitor
