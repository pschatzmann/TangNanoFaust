# Installation

You need up to three things:

1. **The Arduino library**, on the computer you program the microcontroller from.
2. **The FPGA tools** (Gowin EDA, openFPGALoader), to build a bitstream and
   load it into the Tang Nano 20K.
   You only need these once: the generic bitstream runs any Faust program.
3. **Faust and the `faust2tang` tool**, to compile Faust programs.

## 1. Install the Arduino library

**Arduino IDE:** download the repository as a ZIP from
<https://github.com/pschatzmann/TangNanoFaust> (Code > Download ZIP), then
use *Sketch > Include Library > Add .ZIP Library...*.

**git** (recommended, since you also get the tools and gateware):

```bash
cd ~/Arduino/libraries          # your sketchbook's libraries folder
git clone https://github.com/pschatzmann/TangNanoFaust.git
```

**arduino-cli:**

```bash
arduino-cli config set library.enable_unsafe_install true
arduino-cli lib install --git-url https://github.com/pschatzmann/TangNanoFaust.git
```

The library has no dependencies. `TangNanoFaust` and `FaustMidi` need only
SPI and work on any Arduino board. `FaustCompiler` (compiling Faust on the
MCU) needs the C++ standard library: ESP32, RP2040 and similar boards,
but not AVR. Tested: ESP32 core 3.3, RP2040 (Earle Philhower) core 6.1.

## 2. Install the FPGA tools

| Tool | Used for |
|---|---|
| Gowin EDA (`gw_sh`) | synthesis, place and route, timing analysis, bitstream |
| `openFPGALoader` | loading the bitstream into the board |
| `iverilog` (optional) | the RTL tests (`make -C gateware sim`) |
| `make`, `g++` (C++17) | building the tools and running the flow |

```bash
sudo apt install build-essential openfpgaloader iverilog   # Debian/Ubuntu
```

### Gowin EDA

Download the **Education** edition for Linux from Gowin's website
(gowinsemi.com, Support > Download; it may ask you to register). It needs
no licence and supports the Tang Nano 20K's GW2AR-18. Unpack it, for
example to `~/gowin`:

```bash
mkdir -p ~/gowin && tar -xzf Gowin_V*_Education_linux.tar.gz -C ~/gowin
```

On current Linux distributions Gowin's bundled libraries clash with the
system's: its FreeType with fontconfig (`undefined symbol: FT_Done_MM_Var`)
and its OpenGL dispatch libraries with the system's libGL. Start Gowin
through two small wrapper scripts that set the workarounds only for Gowin.
Don't put them in `~/.profile` or `~/.bashrc`, where they break other
programs.

`~/.local/bin/gw_sh` (the command line, used by the build):

```sh
#!/bin/sh
L=/lib/x86_64-linux-gnu
export LD_LIBRARY_PATH="$HOME/gowin/IDE/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LD_PRELOAD="$L/libfreetype.so.6 $L/libGLdispatch.so.0 $L/libGLX.so.0"
export QT_PLUGIN_PATH="$HOME/gowin/IDE/plugins/qt"
export QT_QPA_PLATFORM=minimal
exec "$HOME/gowin/IDE/bin/gw_sh" "$@"
```

`~/.local/bin/gw_ide` (the IDE, optional): the same without the
`QT_QPA_PLATFORM` line, ending in `exec "$HOME/gowin/IDE/bin/gw_ide" "$@"`.

```bash
chmod +x ~/.local/bin/gw_sh ~/.local/bin/gw_ide
echo 'puts ok; exit' | gw_sh            # prints ok; ~/.local/bin must be on the PATH
```

If `gw_sh` isn't on the PATH, pass `GW_SH=/path/to/gw_sh` to `make`.

### USB access to the board

`openFPGALoader` talks to the board's onboard USB-JTAG. On Linux, install
openFPGALoader's udev rules once (see its
[documentation](https://trabucayre.github.io/openFPGALoader/guide/install.html#udev-rules))
so it works without root, then check:

```bash
openFPGALoader --detect -b tangnano20k
```

The board appears as two serial ports: the first (`/dev/ttyUSB0` on Linux)
is the JTAG interface, the second (`/dev/ttyUSB1`) is a UART into the
FPGA, which `faust2tang --port` uses to load programs and set parameters
(see [faust2tang](faust2tang.md#controlling-a-board-over-usb)). Your user
needs access to it, on Debian/Ubuntu by being in the `dialout` group.

### Build and load the generic bitstream

```bash
make -C tools                             # tools/bin/faust2tang
make -C gateware generic                  # ~3 minutes
make -C gateware generic-flash            # into the FPGA, until power-off
make -C gateware generic-flash-persistent # or: into its flash, survives power-off
```

The bitstream runs at 48 MHz (1000 cycles per sample at 48 kHz); see
[gateware](gateware.md#clock). The TDM output is optional: add `TDM=1`
(see [pinout](pinout.md#tdm-output)).

## 3. Install Faust and faust2tang

[Faust](https://faust.grame.fr) 2.70 or newer turns `.dsp` files into the
bytecode TangNanoFaust compiles:

```bash
sudo apt install faust            # Debian/Ubuntu (24.04 ships 2.70)
brew install faust                # macOS
```

Other systems: see <https://faust.grame.fr/downloads/>. Then:

```bash
make -C tools                     # builds tools/bin/faust2tang
tools/bin/faust2tang my.dsp -o build/my
```

If `faust` isn't on the PATH, use `--faust /path/to/faust` or set `FAUST`.

## Verified with

| | Version |
|---|---|
| Gowin EDA | V1.9.11.03 Education |
| openFPGALoader | 0.12.0 |
| Icarus Verilog | 12.0 |
| Faust | 2.70.3 |
| g++ | 13.3 |
| OS | Linux Mint 22.3 (Ubuntu 24.04 based) |
