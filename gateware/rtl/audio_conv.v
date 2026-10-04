`timescale 1ns / 1ps
//
// Sample format conversion between I2S (24-bit signed, full scale = 1.0)
// and the core's binary32 floats. Mirrored by tests/dsp_test.cpp (s24ToF32/f32ToS24).
//

// 24-bit signed -> float, exact (v / 2^23).
module s24_to_f32 (
    input  wire [23:0] v,
    output reg  [31:0] f
);
  reg [23:0] mag, norm;
  reg [4:0]  p;
  integer i;
  always @(*) begin
    mag = v[23] ? (~v + 24'd1) : v;
    p = 5'd0;
    for (i = 0; i < 24; i = i + 1)
      if (mag[i]) p = i[4:0];
    norm = mag << (5'd23 - p);  // leading one at bit 23
    if (v == 24'd0)
      f = 32'd0;
    else
      f = {v[23], 8'd104 + {3'd0, p}, norm[22:0]};
  end
endmodule

// float -> 24-bit signed: truncate toward zero, saturate at +-(2^23 - 1),
// NaN -> 0.
module f32_to_s24 (
    input  wire [31:0] f,
    output reg  [23:0] v
);
  wire [7:0]  e   = f[30:23];
  wire [23:0] man = {1'b1, f[22:0]};
  reg  [23:0] mag;
  always @(*) begin
    if (e == 8'hFF && f[22:0] != 0)
      mag = 24'd0;                       // NaN
    else if (e >= 8'd127)
      mag = 24'h7FFFFF;                  // |x| >= 1.0 (and inf): saturate
    else if (e < 8'd103)
      mag = 24'd0;                       // |x| < 2^-24 (and zero/denormal)
    else
      mag = man >> (8'd127 - e);         // man * 2^(e-127) * 2^23
    v = f[31] ? (~mag + 24'd1) : mag;
  end
endmodule
