# Protocol

The FPGA is an SPI slave: mode 0 (CPOL=0, CPHA=0), MSB first, at most
3 MHz (the FPGA samples it with its 48 MHz system clock). The same
commands also work over a serial line (see below). It is implemented in
`gateware/rtl/spi_ctrl.v`, and `TangNanoFaust`
(`src/TangNanoFaust/TangNanoFaust.h`) is the host side.
Every command is one transaction (CS low): an opcode byte, then its
arguments. Multi-byte values are little endian.

The bitstream reports its protocol version in PING (currently 4). A
compiled program records the version it needs (`ProgramData::minProtocol`,
currently 3), and `TangNanoFaust::load()` refuses to load it on a
bitstream with an older version.

**Reply bytes lag by one transfer.** The byte the FPGA sends during
transfer *n* was decided after transfer *n−1*. So a reply starts on the
transfer after the opcode (or after the last argument), and the host sends
dummy bytes (0x00) to clock it out.

## Over a serial line

The same commands also work over two UART ports at 115200 baud 8N1: the
board's own USB-UART bridge (the second USB serial port, e.g.
`/dev/ttyUSB1`) and a header UART for an MCU (see [pinout](pinout.md)).
Both share one receiver in the FPGA, so use one at a time (and not
together with SPI). A transaction becomes a frame:

| Direction | Bytes |
|---|---|
| host → FPGA | `0x7E`, length u16 LE (bit 15 set: no reply wanted), then the bytes an SPI host would send |
| FPGA → host | the bytes an SPI host would receive (same positions, same one-byte lag), or one `0x7E` if bit 15 was set |

Frames are at most 512 bytes, so streams are sent in chunks (program 100
instructions, descriptor 256 bytes). The FPGA answers only after the whole
frame has arrived. Keep the traffic strictly request → reply: the board's
USB bridge (BL616) loses FPGA → PC bytes while the PC is sending. A pause
of more than 10 ms inside a frame resets the receiver. One host at a time:
a UART frame waits while SPI chip select is active.

## Commands

| Op | Name | Host sends | FPGA replies |
|---|---|---|---|
| `0x01` | PING | — | `"FAUS"`, protocol version: 5 bytes |
| `0x02` | INFO | — | 23 bytes, see below. Also clears `overrun` and `cycles_max`. |
| `0x03` | READ_DESC | offset u16, 1 dummy byte | descriptor bytes from `offset`, one per transfer |
| `0x04` | DEBUG | — | live core state, 13 bytes: state, next PC u16, opcode, stack pointer, T u32, cycle counter u32 |
| `0x10` | WRITE_WORD | address u16, value u32 | — |
| `0x11` | READ_WORD | address u16 | `0x00`… until the value is ready, then `0xA5` and the value u32 |
| `0x20` | CONTROL | flags u8 | — |
| `0x30` | WRITE_PROG | address u16, then 5 bytes per instruction | — |
| `0x31` | WRITE_DESC | address u16, then bytes | — |
| `0x32` | SET_CONFIG | 17 bytes, see below | — |

**INFO reply**: inputs, outputs, parameters (u8 each), flags (u8),
sample rate (u32), descriptor length (u16), worst cycles per sample since
the last INFO (u32), core clock in Hz (u32), and log2 of the program
memory, data block RAM and descriptor capacities (u8 each), and the
number of input and output channels the bitstream has (u8 each).
Flags: bit 0 booted, bit 1 overrun, bit 2 muted, bit 3 SDRAM present,
bit 4 stopped, bit 5 an external TDM master provides the sample clock,
bit 6 the bitstream has TDM.
When TDM is active, the sample rate field still holds the configured
internal rate; the real rate is the master's.

**CONTROL flags**: bit 0 mute, bit 1 amplifier on (MAX98357A SD_MODE),
bit 2 reboot (pulse: re-run the boot program), bit 3 stop the DSP core.
After reset: amplifier on, everything else off.

**SET_CONFIG**: dsp entry u16, boot entry u16, inputs u8, outputs u8,
parameters u8, sample rate u32, I2S clock increment u32 (= sample rate ×
128 × 2³² / core clock), descriptor length u16.

## Parameters

Parameters are words in the core's block RAM. Their addresses come from the
descriptor. WRITE_WORD goes through a 16-entry FIFO and is applied between
two samples; the control block runs again in the next sample. READ_WORD is
answered between two samples too, which is why the host polls for `0xA5`.

## Descriptor

Text, one line per entry, fields separated by `|`:

```
F|1|<name>|<inputs>|<outputs>|<sample rate>|<parameter count>
P|<kind>|<address>|<init>|<min>|<max>|<step>|<path>|<key:value,...>
```

`kind` is one of `hslider`, `vslider`, `nentry`, `button`, `checkbox`,
`hbargraph` and `vbargraph`. The metadata holds Faust's `[key:value]`
annotations, such as `midi:ctrl 74` or `unit:Hz`.

## Loading a program

1. CONTROL with *stop* set. The current sample finishes, and the output is muted.
2. WRITE_PROG from address 0 (any chunk size; the address auto-increments).
3. WRITE_DESC from address 0.
4. SET_CONFIG.
5. CONTROL with *stop* cleared and *reboot* set. The core runs the boot program.
6. Poll INFO until *booted* is set, then read the descriptor.
