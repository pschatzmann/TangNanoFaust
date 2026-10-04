# TangNano Faust

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-00979D?logo=arduino&logoColor=white)](docs/installation.md#1-install-the-arduino-library)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

Run [Faust](https://faust.grame.fr) DSP programs on a
[Sipeed Tang Nano 20K](https://wiki.sipeed.com/hardware/en/tang/tang-nano-20k/nano-20k.html)
FPGA, and control them from an Arduino or your computer.

The FPGA runs a floating-point DSP core that executes compiled Faust
programs and plays the audio on the board's I2S amplifier. Programs are
loaded and their parameters changed at runtime: from a microcontroller
over SPI or a UART (from code or MIDI), or from a PC over the board's USB
port.

```
 my.dsp --faust--> bytecode --faust2tang--> DSP core program --SPI/UART/USB--> Tang Nano 20K --I2S/TDM--> audio
                                (PC or MCU)
```

## Features

- Faust programs compile to a custom FPGA DSP core: binary32 floating point,
  all of Faust's math functions, delay lines and tables in block RAM or the
  8MB SDRAM.
- One generic bitstream runs any program. Programs load at runtime, so you
  don't need an FPGA rebuild per program.
- The `faust2tang` compiler is header-only C++ and also runs on the
  microcontroller (ESP32, RP2040). Or compile over WiFi: `faust2tang --serve`
  is a compile server for microcontrollers on your network.
- An Arduino library for parameters (discovered at runtime), status and DSP load,
  over SPI or a UART.
- Control from a PC over the board's own USB port: `faust2tang my.dsp --port
  /dev/ttyUSB1 --load`, then `--set freq=440`.
- MIDI through any Arduino `Stream` (`Serial`, DIN MIDI on `Serial1`, ...).
- Audio: the onboard I2S amplifier, I2S input, and optionally (`TDM=1`)
  8-channel TDM output for external DACs and codecs (the FPGA follows the
  external clock).
- 48 MHz DSP core: 1000 cycles per sample at 48 kHz, e.g. 10 sine voices
  use under half of that.
- Bitstreams are built with Gowin EDA (free Education edition).

## Quick start

```bash
make -C tools                                   # builds tools/bin/faust2tang
make -C gateware generic generic-flash          # generic bitstream -> FPGA
```

From your computer, over the board's USB port:

```bash
tools/bin/faust2tang my.dsp -o build/my --port /dev/ttyUSB1 --load
tools/bin/faust2tang --port /dev/ttyUSB1 --set freq=440
```

Or from an Arduino sketch, with the header `faust2tang my.dsp -o build/my`
writes (`build/my/my_program.h`):

```cpp
#include <TangNanoFaust.h>
#include "my_program.h"
using namespace tangnanofaust;

TangNanoFaust faust;

void setup() {
  faust.begin(SPI, 5);          // CS pin
  faust.load(my::program);      // upload the compiled Faust program
  faust.setParameter("freq", 440);
}
void loop() {}
```

See [examples/](examples/): `basic`, `midi_serial`, `midi_synth` (four
instruments, each on its own MIDI channel), `compile_on_mcu`
(Faust bytecode compiled on the microcontroller), `serial_control`
(control over a UART instead of SPI) and `wifi_compile` (an ESP32 sends Faust
source to a compile server, `faust2tang --serve`, and plays the result).

## Status

Verified in simulation (bit- and cycle-exact against the compiler's
model) and on a Tang Nano 20K controlled over its USB port: self-tests of
every core unit and the SDRAM, listening tests and MIDI. Not yet tested on
hardware: a microcontroller on SPI or the UART, and TDM with a codec. See
[testing](docs/testing.md#status).

## Documentation

- [Installation](docs/installation.md): Arduino library, FPGA tools, Faust
- [Getting started](docs/getting-started.md): bitstream, wiring, first sound
- [Faust and DSP tutorial](docs/tutorial.md): the language and DSP basics, step by step
- [Writing Faust programs](docs/faust.md): what's supported, performance, parameters
- [Arduino library](docs/arduino.md): `TangNanoFaust`, `FaustMidi`, `FaustCompiler`
- [faust2tang](docs/faust2tang.md): the compiler and its outputs
- [Gateware](docs/gateware.md): architecture, DSP core, building bitstreams
- [Protocol](docs/protocol.md) (SPI and serial) and [pinout](docs/pinout.md)
- [Testing and status](docs/testing.md): what's verified, and how
- [Licensing](docs/licensing.md)

