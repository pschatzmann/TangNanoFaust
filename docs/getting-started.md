# Getting started

Install the Arduino library and the tools first, see
[installation](installation.md).

## What you need

- A Sipeed Tang Nano 20K. Its onboard MAX98357A amplifier plays the audio, so
  you only need a small speaker on its speaker connector.
- Optional: a microcontroller with SPI or a UART. ESP32 and RP2040 are
  tested, and compiling Faust on the MCU needs one with the C++ standard
  library (not AVR). Without one, control the board from your computer
  over its USB port.
- On your computer:
  - [Faust](https://faust.grame.fr) 2.70 or newer (`faust` on the PATH, or
    `FAUST=/path/to/faust`). Only needed to turn `.dsp` files into bytecode.
  - A C++17 compiler (`g++`) for the `faust2tang` tool.
  - To build bitstreams: Gowin EDA (free Education edition) and
    `openFPGALoader`.
  - Optional, for the RTL tests: `iverilog`.

## 1. Build the tools

```bash
make -C tools          # tools/bin/faust2tang
```

## 2. Put the generic bitstream on the FPGA

The generic bitstream runs any Faust program you load later over SPI or
a UART. It boots with a stereo passthrough (I2S input to output).

```bash
make -C gateware generic                     # synthesize, place & route (~3 minutes)
make -C gateware generic-flash               # load it into the FPGA (until power-off)
make -C gateware generic-flash-persistent    # or: write it to the onboard flash
```

The onboard LEDs show the status, see [pinout](pinout.md#leds).

## 3a. Without an MCU: control from your computer

The board's second USB serial port talks to the FPGA:

```bash
tools/bin/faust2tang gateware/dsp/synth.dsp -o build/synth --port /dev/ttyUSB1 --load
tools/bin/faust2tang --port /dev/ttyUSB1 --set freq=220 --set gate=1
tools/bin/faust2tang --port /dev/ttyUSB1 --info      # sample rate, load, overruns
tools/bin/faust2tang --port /dev/ttyUSB1 --mute      # silence (--unmute)
```

## 3b. Or wire an MCU

| Signal | Tang Nano 20K FPGA pin |
|---|---|
| SCK  | 27 |
| MOSI | 28 |
| MISO | 29 |
| CS   | 30 |
| GND  | GND |

Find the pins on the board's headers with Sipeed's
[pinout diagram](https://wiki.sipeed.com/hardware/en/tang/tang-nano-20k/nano-20k.html).
Both boards use 3.3V logic. Keep SPI at 3 MHz or below (the default in the
library is 2 MHz). Details are in [pinout](pinout.md).

Instead of SPI, an MCU can use a UART at 115200 baud: its TX to FPGA pin
25, its RX to pin 26, and `faust.begin(Serial1)`; see
[examples/serial_control](../examples/serial_control/serial_control.ino).

## 4. Compile a Faust program and play it

```bash
tools/bin/faust2tang gateware/dsp/synth.dsp -o build/synth
```

This prints a report (memory, parameters, how much of each sample period
the program needs) and writes `build/synth/synth_program.h`. Copy that
header into your sketch:

```cpp
#include <NanoTangFaust.h>
#include "synth_program.h"
using namespace nanotangfaust;

TangNanoFaust faust;

void setup() {
  Serial.begin(115200);
  if (!faust.begin(SPI, 5)) Serial.println("FPGA not answering");
  if (faust.load(synth::program) != LoadResult::Ok) Serial.println("load failed");
  faust.setParameter("freq", 220);
  faust.setParameter("gate", 1);
}
void loop() {}
```

Or let the microcontroller compile the program itself, see
[examples/compile_on_mcu](../examples/compile_on_mcu/compile_on_mcu.ino) and
[Arduino library](arduino.md#compiling-on-the-microcontroller).

## Alternative: a bitstream for one program

`make -C gateware DSP=dsp/synth.dsp flash` builds a bitstream that starts
the synth on power-up, with no MCU needed for loading. It is sized for that
program (and has SDRAM only if the program needs it). You can still change
parameters over SPI or a UART.
