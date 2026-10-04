# Pinout

Pins of `gateware/rtl/top_tangnano20k.v`, as assigned in
`gateware/constraints/tangnano20k.cst`. Pin numbers are FPGA pins; find them
on the headers with Sipeed's
[pinout diagram](https://wiki.sipeed.com/hardware/en/tang/tang-nano-20k/nano-20k.html).
To use other header pins, edit the `.cst` file and rebuild.

| Signal | FPGA pin | Direction | |
|---|---|---|---|
| `spi_sclk` | 27 | in | SPI clock from the MCU |
| `spi_mosi` | 28 | in | |
| `spi_miso` | 29 | out | tri-stated while CS is high |
| `spi_cs_n` | 30 | in | active low |
| `i2s_rx_din` | 73 | in | optional I2S audio input (data only), pulled down |
| `i2s_bclk`, `i2s_ws`, `i2s_din` | 56, 55, 54 | out | to the onboard MAX98357A |
| `i2s_pa_en` | 51 | out | MAX98357A SD_MODE (amplifier on) |
| `tdm_bclk` | 74 | in | TDM bit clock from the external master, pulled down |
| `tdm_fs` | 75 | in | TDM frame sync from the external master, pulled down |
| `tdm_dout` | 77 | out | TDM data, slots 0..7 = Faust outputs 0..7 (TDM bitstreams only) |
| `usb_uart_tx`, `usb_uart_rx` | 69, 70 | out, in | command port on the onboard USB-UART bridge (no wiring needed) |
| `uart_rx`, `uart_tx` | 25, 26 | in, out | command port for an MCU's UART (FPGA RX, FPGA TX), 115200 8N1 |
| `clk_27m` | 4 | in | onboard 27 MHz oscillator |
| `reset_button` | 88 | in | S1, active high |
| `leds[5:0]` | 20-15 | out | onboard LEDs, active low |

The clock, button, LED and I2S pins are the ones arduino-tangnano20k uses
and has verified on hardware. The SPI pins are the ones NanoTangAI uses.
The SDRAM is inside the FPGA package; its pins need no constraints.

## I2S input

The FPGA is the I2S master: it generates BCLK (64 × sample rate) and WS on
pins 56/55. An external source (an I2S microphone, or an MCU as I2S slave
transmitter) shares these clocks and sends Philips-format data (24 or more
bits per slot) on pin 73.

## TDM output

For multichannel DACs and codecs. TDM is optional: build the bitstream
with `make ... TDM=1` (for example `make -C gateware generic TDM=1`). It
then has 8 outputs instead of 2, and `info().tdm` is true. Without it the
TDM pins are unused. The FPGA is the **slave**: the external
device (or an MCU) is the TDM master and drives BCLK and FS. The FPGA sends
8 slots of 32 bits per frame (BCLK = 256 × sample rate, 12.288 MHz at
48 kHz). Each slot holds a 24-bit sample, left-justified and MSB first.
Data changes on falling BCLK edges, and slot 0 starts one BCLK after the
rising edge at which FS is first high: the common "I2S-style" TDM format
with a 1-bit delay. FS may be one bit long or longer.

While frame syncs arrive, the master's frame rate is the sample clock of
the whole design. The DSP runs once per TDM frame, and the onboard I2S
amplifier (outputs 0/1) and the I2S input are clocked from the TDM bit
clock too (BCLK ÷ 4), so everything stays in sync. `TangNanoFaust::info()`
reports this as `tdmActive`. Without TDM clocks, the FPGA falls back to its
own sample clock within about 2.5 ms.

## LEDs

| LED | On when |
|---|---|
| 0 (pin 15) | the program has booted and is running |
| 1 (pin 16) | blinks with the sample clock (about 1.5 Hz at 48 kHz) |
| 2 (pin 17) | a sample overran its time since the last INFO |
| 3 (pin 18) | muted |
| 4 (pin 19) | SPI chip select active |
| 5 (pin 20) | PLL not locked |
