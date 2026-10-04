`timescale 1ns / 1ps
//
// Stand-alone I2S master (Philips format, 64 BCLK per frame = 2 x 32-bit
// slots, 24-bit samples MSB-first, one BCLK delay after each WS edge).
// Drives the Tang Nano 20K's onboard MAX98357A amplifier and optionally
// captures an external I2S source on `rx_din` (a microphone, or a host MCU
// streaming audio into the FPGA as an I2S slave transmitter).
//
// BCLK is generated with a 32-bit phase accumulator (the same fractional-N
// approach as arduino-tangnano20k's gateware/src/i2s.v): every `clk` cycle
// adds `bclk_inc`, and BCLK toggles on each overflow, so the average BCLK
// frequency is exactly sample_rate*64 even when the clock isn't an integer
// multiple of it (48kHz from 54MHz). bclk_inc = sample_rate * 128 * 2^32 /
// clock; it's a runtime input so a program loaded over SPI can change the
// sample rate. Individual BCLK edges jitter by at most
// one `clk` cycle, which the MAX98357A tolerates (it recovers its own clock
// from BCLK).
//
// External timing (`ext_en`, from tdm_tx.v): when an external TDM master
// provides the sample clock, BCLK toggles on `ext_tick` instead (every
// second TDM BCLK rising edge: 256 x fs / 2 = 128 toggles per frame, i.e.
// exactly 64 x fs) and `ext_sync` (each TDM frame sync) realigns the frame,
// so the onboard amplifier, the DSP and the TDM output share one sample
// clock.
//
// `frame_start` pulses for one `clk` cycle at the start of every frame. At
// that moment:
//   - tx_left/tx_right are latched for transmission in the frame that just
//     started (so they must already hold the next sample);
//   - rx_left/rx_right are updated with the samples captured during the
//     frame that just ended.
// top_tangnano20k.v uses `frame_start` as its once-per-sample compute trigger.
//
module i2s_master (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] bclk_inc,
    input  wire        ext_en,
    input  wire        ext_tick,
    input  wire        ext_sync,

    input  wire [23:0] tx_left,
    input  wire [23:0] tx_right,
    output reg  [23:0] rx_left,
    output reg  [23:0] rx_right,
    output reg         frame_start,

    output reg         bclk,
    output reg         ws,
    output reg         dout,
    input  wire        rx_din
);

  reg  [31:0] phase;
  wire [32:0] phase_sum = {1'b0, phase} + {1'b0, bclk_inc};
  reg         ext_div;  // ext_tick: use every second one
  wire        tick      = ext_en ? (ext_tick && ext_div) : phase_sum[32];

  reg  [5:0]  bit_cnt;     // index of the BCLK period being output (0..63)
  reg  [63:0] tx_shift;
  reg  [63:0] rx_shift;

  // One frame: {pad, L[23:0], 7 x pad, pad, R[23:0], 7 x pad}. The leading
  // pad bit of each slot is the Philips one-BCLK delay after the WS edge.
  wire [63:0] tx_frame = {1'b0, tx_left, 7'd0, 1'b0, tx_right, 7'd0};

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      phase       <= 32'd0;
      bclk        <= 1'b0;
      ws          <= 1'b0;
      dout        <= 1'b0;
      bit_cnt     <= 6'd63;
      tx_shift    <= 64'd0;
      rx_shift    <= 64'd0;
      rx_left     <= 24'd0;
      rx_right    <= 24'd0;
      frame_start <= 1'b0;
      ext_div     <= 1'b0;
    end else begin
      phase       <= phase_sum[31:0];
      frame_start <= 1'b0;
      if (ext_tick) ext_div <= ~ext_div;
      if (ext_en && ext_sync) begin
        // align: the next tick is the falling edge that starts a frame
        bclk    <= 1'b1;
        bit_cnt <= 6'd63;
        ext_div <= 1'b0;
      end else if (tick) begin
        bclk <= ~bclk;
        if (bclk) begin
          // Falling edge: advance to the next bit and drive WS/data.
          bit_cnt <= bit_cnt + 6'd1;
          ws      <= (bit_cnt + 6'd1) >= 6'd32;
          if (bit_cnt == 6'd63) begin
            tx_shift    <= {tx_frame[62:0], 1'b0};
            dout        <= tx_frame[63];
            rx_left     <= rx_shift[62:39];
            rx_right    <= rx_shift[30:7];
            frame_start <= 1'b1;
          end else begin
            tx_shift <= {tx_shift[62:0], 1'b0};
            dout     <= tx_shift[63];
          end
        end else begin
          // Rising edge: the receiver samples the bit driven on the last
          // falling edge.
          rx_shift <= {rx_shift[62:0], rx_din};
        end
      end
    end
  end

endmodule
