# Gateware

The FPGA design is in `gateware/rtl/` (Verilog). Bitstreams are built with
Gowin EDA (`gw_sh`) and loaded with `openFPGALoader`, see
[installation](installation.md#2-install-the-fpga-tools).

```
   SPI (MCU)   USB UART (PC)   header UART (MCU)    external TDM master (TDM=1)
       |             \_______________/                 |  BCLK, FS
       |                uart_bridge.v                tdm_tx.v ----> TDM data, outputs 0..7
       v                     v                         v  sample clock when present
          spi_ctrl.v: params | program | status | debug
               |                                       |
          top_tangnano20k.v: sequencer, one run per sample
               |
          dsp_core.v  <-- fpu.v, hmul.v (hard multipliers)
          block RAM: program 4096 x 40, data 16384 x 32
          SDRAM (8MB) via sdram_bus.v / sdram.v
               |
          i2s_master.v --> MAX98357A (outputs 0/1), I2S input
```

The UART bridge runs on the 27 MHz oscillator, so its baud rate doesn't
depend on the system clock. It receives a framed SPI transaction, replays
it into `spi_ctrl.v` as if it came from the SPI pins (through the same
synchronizers), and sends the reply back (see
[protocol](protocol.md#over-a-serial-line)). The USB UART and the header
UART share the bridge: their RX lines are ANDed (both idle high) and its TX
drives both pins. SPI, the USB port and the header UART all speak the same
protocol, and only one of them may be used at a time.

## The DSP core

`dsp_core.v` is a 32-bit stack machine. The instruction set is listed in
`src/TangNanoFaust/compiler/IsaOpcodes.h`, and `isa_opcodes.vh` is generated
from it. Every instruction is 40 bits: an 8-bit opcode and a 32-bit
argument. Binary operations take the top of the stack as their first
operand, as Faust's bytecode does.

- **Timing and prefetch.** The program memory's output register holds the
  next instruction while the current one executes. Simple instructions
  (constants, stores, integer operations, comparisons) take 1 cycle. A
  block RAM load, or a memory operand, of the next instruction is read
  during the current instruction's last cycle and costs nothing extra,
  unless that instruction writes block RAM (single port); otherwise 1
  cycle. Taken jumps and the start of a run cost 1 extra cycle. FPU,
  multiply and divide instructions wait for their unit, SDRAM accesses for
  the controller.
- **FPU.** `fpu.v` is multi-cycle binary32: add/sub 3 cycles, multiply 3
  (four 12×12 products on hard `MULT18X18`s), divide 29, sqrt 31, floor,
  int→float and float→int 2. Comparisons, min/max, abs and neg run in the
  core in 1 cycle. Rounding is IEEE round-to-nearest-even, denormals are
  flushed to zero and NaNs are canonical. `src/.../compiler/Fp32.h` is the
  bit-exact specification.
- **Multipliers.** Integer multiply takes 3 cycles. In both multipliers
  the partial products are registered before they are added, so the
  multiplication, the sum and the FPU's rounding are in separate cycles.
- **Fused instructions.** Every binary operator also exists with an
  immediate operand (opcode + 0x40) or a memory operand (opcode + 0x80):
  v1 comes from the instruction or a read, v2 is T, and the result replaces
  T. A memory operand costs one extra cycle for the read. `TEE` stores
  without popping.
- **Data stack** 32 deep, **return stack** 8 deep (math routines call each other).
- **Cycle counter.** The `cycles` register holds the cost of the last run.
  The worst case since the last INFO is reported over SPI.
- **Robustness.** Unused FPU state codes end the operation with a NaN
  result, and if the FPU or the SDRAM doesn't answer within 255 cycles
  the core continues with the value it has. A wrong result is then
  counted by the self-tests instead of hanging the core.

`src/TangNanoFaust/compiler/Isa.h` simulates the core bit- and cycle-exactly.

## Sample flow

Once per sample frame, the sequencer starts the core. It starts at address
0 (control block + per-sample block) if a parameter changed, otherwise at
the per-sample block. Outputs are played one frame later. Between samples
it applies parameter writes from SPI and answers reads. After reset, or a
reboot over SPI, it first runs the program's boot code (tables, defaults,
clearing state). A sample that is still running when the next frame starts
is reported as an *overrun*.

The sample clock is the onboard I2S master (`i2s_master.v`: a fractional
divider from the system clock, so any rate is exact on average), or, while
an external TDM master sends frame syncs, that master. In TDM mode the
onboard I2S is clocked from the TDM bit clock, see
[pinout](pinout.md#tdm-output).

## Clock

A PLL makes the system clock from the 27 MHz oscillator, plus a 180° copy
for the SDRAM. The default is **48 MHz**, 1000 cycles per sample at
48 kHz. `SYS_MHZ` selects 40.5, 33.75 or 27 MHz instead (843, 703 or 562
cycles at 48 kHz). The board reports its clock in INFO, so hosts and
`faust2tang` adapt automatically.

Gowin's timing analysis reports about 44 MHz for the generic bitstream at
its worst-case corner (0.95 V, 85 °C); at room temperature the board runs
every self-test at 48 MHz (see [testing](testing.md#on-the-board)) and
fails at 54 MHz. The longest path (about 25 logic levels) runs from the
data block RAM through the operand selection into the FPU.

## Building

```bash
make -C gateware generic                 # the generic bitstream (any program at runtime)
make -C gateware generic TDM=1           # ... with the TDM output (8 outputs)
make -C gateware generic-flash           # load it (SRAM, until power-off)
make -C gateware generic-flash-persistent  # or write it to the onboard flash
make -C gateware DSP=dsp/synth.dsp       # a bitstream that boots one program
make -C gateware DSP=dsp/synth.dsp flash
make -C gateware DSP=dsp/synth.dsp report  # program numbers + FPGA utilization
make -C gateware sim                     # RTL regression (see testing.md)
```

Options: `SYS_MHZ`, `TDM=1`, `SAMPLE_RATE`, `FAUST=/path/to/faust`,
`GW_SH=/path/to/gw_sh`. A build takes about 3 minutes.

`gateware/build.sh` does the work: it runs `faust2tang` (memory images and
`config.vh`), copies the RTL next to them, writes the defines for the
clock and SDRAM and a timing constraint, and runs `gw_sh`. Each
configuration builds in its own directory, `build/<name>`, plus `_tdm`
with `TDM=1` and `_40.5mhz` etc. with a `SYS_MHZ` other than 48, for
example `build/passthrough_tdm_33.75mhz/`. Pass the same options to the
flash targets. In the build directory:

| File | |
|---|---|
| `<name>.fs` | the bitstream |
| `report.txt` | the program: memory, parameters, cycles per sample |
| `impl/pnr/tangnanofaust.tr.html` | Gowin's timing report |
| `impl/pnr/tangnanofaust.rpt.txt` | Gowin's utilization report |
| `gowin.log` | the Gowin run |

A bitstream built for one program is sized for it: program and data memory,
descriptor, number of outputs, and SDRAM only if the program needs it. The
generic one has the maximum sizes, 2 inputs, 2 outputs (8 with `TDM=1`)
and SDRAM.

The timing constraint declares the 27 MHz input and the PLL output, and
the paths between the UART bridge's 27 MHz domain and the system clock as
false paths (they only cross through synchronizers).

### Resources

The generic bitstream (`make generic`), without and with TDM:

| | Without TDM | With TDM | Available |
|---|---|---|---|
| Logic (LUT, ALU, RAM16) | 7409 (36%) | 8271 (40%) | 20736 |
| Registers | 2380 (15%) | 3431 (22%) | 15915 |
| BSRAM | 44 (96%) | 44 (96%) | 46 |
| MULT18X18 | 7 | 7 | 48 |
| Gowin's Fmax (worst case) | 44.0 MHz | 45.7 MHz | |

The block RAM is 32 blocks of data memory, 10 of program memory, 1 for
the descriptor and 1 for the UART frame buffer.
