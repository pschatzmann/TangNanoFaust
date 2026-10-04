# faust2tang

`faust2tang` compiles a Faust program for the NanoTangFaust DSP core. It is
a small C++ program (`tools/faust2tang.cpp`) around the header-only
compiler in `src/NanoTangFaust/compiler/`, which is the same code that
`FaustCompiler` runs on a microcontroller.

```bash
make -C tools                                   # -> tools/bin/faust2tang
tools/bin/faust2tang my.dsp -o build/my         # or a .fbc file
```

It runs `faust -lang interp -double` (set the executable with `--faust` or
`$FAUST`), compiles the bytecode and writes:

| File | |
|---|---|
| `my_program.h` | the compiled program for `TangNanoFaust::load(my::program)` |
| `my_params.h` | parameter indices as constants (optional; the FPGA describes its parameters itself) |
| `report.txt` | memory layout, parameters, cycles per sample, boot time (also printed) |
| `prog.hex`, `desc.hex`, `config.vh` | memory images and Verilog configuration for building a bitstream |
| `dsp.fbc` | the Faust bytecode |

| Option | |
|---|---|
| `--sample-rate N` | default 48000 |
| `--fast-words N` | block RAM words for data (default 16384) |
| `--no-sdram` | fail instead of using SDRAM |
| `--no-fuse` | don't use fused instructions (debugging) |
| `--fast-math` | faster `sin`/`cos` (1.9e-7 error for moderate arguments, ~40% fewer cycles) |
| `--strict-budget` | fail if a sample needs more cycles than available |
| `--clk-hz N` | core clock for the cycle report (48000000) |
| `-I DIR` | Faust library directory |
| `--faust-arg ARG` | pass an option to `faust` (repeatable); see below |
| `--generic` | size the bitstream configuration for runtime loading (used by `make generic`) |
| `--tdm` | build the TDM output into the bitstream; generic: 8 outputs instead of 2 (`make TDM=1`) |
| `--sketch` | only write `my_fbc.h` next to `my.dsp`: the bytecode as a C++ string, for `FaustCompiler` |
| `--verilog-opcodes FILE` | write `gateware/rtl/isa_opcodes.vh` from `IsaOpcodes.h` |

## Controlling a board over USB

The board's second USB serial port reaches the FPGA directly, so you can
develop without a microcontroller:

```bash
tools/bin/faust2tang synth.dsp -o build/synth --port /dev/ttyUSB1 --load   # compile + upload
tools/bin/faust2tang --port /dev/ttyUSB1 --list                            # parameters
tools/bin/faust2tang --port /dev/ttyUSB1 --set freq=440 --set gate=1
tools/bin/faust2tang --port /dev/ttyUSB1 --get freq --info
tools/bin/faust2tang --port /dev/ttyUSB1 --debug        # live state of the DSP core
tools/bin/faust2tang --port /dev/ttyUSB1 --mute         # silence, amplifier off
tools/bin/faust2tang --port /dev/ttyUSB1 --unmute
```

`--set` and `--get` take a label or a path (`freq`, `v3/level`); `--get`
also prints the value's raw bits. The program keeps running while muted.
`--load` unmutes.

The first USB serial port (`/dev/ttyUSB0`) is the JTAG interface
openFPGALoader uses.

## Compile server

```bash
tools/bin/faust2tang --serve 8000            # all interfaces; --bind ADDR to restrict
```

turns `faust2tang` into a small HTTP server that compiles Faust source code
for microcontrollers on your network (see
[Arduino library](arduino.md#compiling-over-the-network) and
`examples/wifi_compile`). `--faust`, `-I`, `--faust-arg` and the compile
options apply to every request.

`POST /compile` with the Faust source as the body. Query parameters:

| Parameter | |
|---|---|
| `out=program` | default: the finished program as a binary block (`src/NanoTangFaust/ProgramBlob.h`), for `parseProgramBlob()` + `TangNanoFaust::load()` |
| `out=fbc` | Faust's bytecode, for `FaustCompiler` on the MCU |
| `out=report` | the report `faust2tang` prints |
| `sr`, `fast_words`, `sdram`, `clk_hz` | the target bitstream: sample rate, block RAM words, SDRAM (0/1), clock (defaults 48000, 16384, 1, 48000000) |
| `fast_math=1` | faster `sin`/`cos` |
| `name` | program name if the source declares none (default `dsp`) |

The answer is status 200 with the result, or 400 with Faust's or the
compiler's error message as text. `X-NTF-Cycles` and
`X-NTF-Control-Cycles` give the simulated cycles per sample. Try it with
curl:

```bash
curl --data-binary @gateware/dsp/synth.dsp 'http://localhost:8000/compile?out=report'
```

It handles one request at a time and accepts sources up to 1 MB. Faust can
read any file the server's user can read (`import`, `library`), so run it
on trusted networks only.

## Faust options

`faust2tang` runs `faust -lang interp -double`. `-double` matters: the
bytecode then carries 16 significant digits per constant, and the compiler
rounds them to float32 itself. Other Faust options that change the
generated code can be passed with `--faust-arg`:

| Option | Effect here |
|---|---|
| `-mcd <n>` (default 16) | delays up to *n* samples use shift copies, longer ones ring buffers. The default is the fastest in our tests; `-mcd 0` costs 3-24% more cycles. |
| `-dlt <n>` | above *n* samples, select-based instead of mask-based ring buffers. Leave it at the default. |
| `-es 0` | multiply instead of "enable semantics" for `enable`/`control`. |
| `-ftz <n>` | adds flush-to-zero code. Not needed: the core flushes denormals in hardware. |

Options for other backends (`-vec`, `-omp`, `-os`, `-fx`, `-single`/`-quad`)
don't apply to the interpreter bytecode.

## How it compiles

Faust's own VHDL backend isn't usable for this. Even simple programs
produce invalid VHDL, and sliders, delays and tables aren't supported.
NanoTangFaust therefore starts from Faust's **interpreter bytecode** (FBC):
a well-defined stack machine program with an integer heap, a real heap,
init, control and per-sample blocks, and a description of the UI.

1. **Parse** the FBC text (`Fbc.h`).
2. **Lay out memory** (`Compiler.h`): every heap cell gets an address.
   Scalars and small arrays go to block RAM, arrays that don't fit go to
   SDRAM (smallest first, so hot state stays in fast memory).
3. **Generate code**. The DSP core is a stack machine with FBC's operand
   order, so most FBC instructions map one-to-one. Loops and selects become
   jumps, and math functions become calls into the math library
   (`MathLib.h`, polynomial approximations written as formulas).
4. **Peephole optimizations**: `PUSH k; LOADX b` becomes `LOAD b+k` (Faust
   indexes arrays mostly with constants), `PUSH k; STOREX b` becomes
   `STORE b+k`, and `STORE a; LOAD a` becomes `TEE a`. A constant or a
   variable followed by a binary operator becomes one fused instruction
   with an immediate or memory operand. This saves 20-35% of the cycles
   with bit-identical results.
5. **Program layout**: control block at address 0, the per-sample block at
   `dspEntry`, and the boot program at `bootEntry`. The boot program
   contains Faust's init blocks (tables, constants, slider defaults,
   clearing state) and runs on the core after loading.
6. **Simulate** (`Isa.h`): a bit-exact model of the core, including its
   cycle count. The report's numbers come from running the program on it,
   with every parameter set across its range (the worst case counts).

The instruction set is listed in `IsaOpcodes.h`; the core is described in
[gateware](gateware.md).
