`timescale 1ns / 1ps
//
// TDM output, slave mode: an external master (codec, DAC, MCU) drives the
// bit clock and frame sync; the FPGA sends SLOTS x 32-bit slots, each with
// a 24-bit sample left-justified, MSB first. Data changes on falling BCLK
// edges and starts one BCLK after the rising edge at which FS is first seen
// high (the common I2S-style TDM format, "1 bit delay").
//
// The shift register runs in the external BCLK domain: sampling a
// 12.288 MHz bit clock with the system clock would leave too little
// margin to change the data in time. Samples cross from the system domain
// through a double buffer: `frame_we` writes the buffer that isn't
// published and then publishes it; the BCLK domain synchronizes the
// published-buffer bit and loads from it at the next frame start. A buffer
// is only rewritten a whole frame after it was published, long after the
// BCLK domain has switched to it.
//
// In the system domain this also reports the external timing: `active`
// while frame syncs arrive (the top level then derives the onboard I2S and
// the DSP's sample clock from it), `fs_tick` at each frame start and
// `bclk_tick` at every rising BCLK edge.
//
module tdm_tx #(
    parameter integer SLOTS = 8
) (
    input  wire                clk,
    input  wire                rst_n,

    // samples for the next frame (system domain)
    input  wire [SLOTS*24-1:0] frame,
    input  wire                frame_we,

    // external TDM master
    input  wire                tdm_bclk,
    input  wire                tdm_fs,
    output reg                 tdm_dout,

    // external timing, system domain
    output wire                active,
    output wire                fs_tick,
    output wire                bclk_tick
);

  // ---------------------------------------------------------------- system domain
  reg [SLOTS*24-1:0] buf0, buf1;
  reg                published;  // buffer the BCLK domain should send
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      published <= 1'b0;
    end else if (frame_we) begin
      if (published) buf0 <= frame;
      else buf1 <= frame;
      published <= ~published;
    end
  end

  reg [2:0]  bclk_s, fs_s;
  reg [16:0] idle;  // system clocks since the last frame sync (saturating)
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bclk_s <= 3'd0;
      fs_s   <= 3'd0;
      idle   <= 17'h1FFFF;
    end else begin
      bclk_s <= {bclk_s[1:0], tdm_bclk};
      fs_s   <= {fs_s[1:0], tdm_fs};
      if (fs_s[2:1] == 2'b01) idle <= 17'd0;
      else if (idle != 17'h1FFFF) idle <= idle + 17'd1;
    end
  end
  assign fs_tick   = (fs_s[2:1] == 2'b01);
  assign bclk_tick = (bclk_s[2:1] == 2'b01);
  // a frame sync within ~2.4 ms (1/48 kHz is 21 us; 1/8 kHz 125 us)
  assign active    = (idle != 17'h1FFFF);

  // ---------------------------------------------------------------- BCLK domain
  reg                fs_q = 1'b0, start = 1'b0;
  reg  [1:0]         pub_s = 2'b00;
  reg  [SLOTS*32-1:0] shift = {SLOTS*32{1'b0}};
  wire [SLOTS*24-1:0] cur = pub_s[1] ? buf1 : buf0;

  // one 32-bit slot per sample: {sample[23:0], 8'b0}
  reg  [SLOTS*32-1:0] slots;
  integer k;
  always @(*) begin
    for (k = 0; k < SLOTS; k = k + 1)
      slots[(SLOTS-1-k)*32 +: 32] = {cur[k*24 +: 24], 8'd0};
  end

  always @(posedge tdm_bclk) begin
    fs_q  <= tdm_fs;
    start <= tdm_fs & ~fs_q;
    pub_s <= {pub_s[0], published};
  end

  always @(negedge tdm_bclk) begin
    if (start) begin
      shift    <= {slots[SLOTS*32-2:0], 1'b0};
      tdm_dout <= slots[SLOTS*32-1];
    end else begin
      shift    <= {shift[SLOTS*32-2:0], 1'b0};
      tdm_dout <= shift[SLOTS*32-1];
    end
  end

endmodule
