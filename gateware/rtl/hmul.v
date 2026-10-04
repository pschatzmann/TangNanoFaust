`timescale 1ns / 1ps
//
// Hard-multiplier wrappers.
//
// The multipliers register their inputs, and mul24 and mul32lo register the
// partial products in the fabric before adding them: the product of the
// inputs presented at one clock edge is valid two cycles later. The
// multiplication and the following sum (and, in the FPU, normalize and
// round) are in separate cycles, which keeps the paths short.
//
// Under synthesis (SYNTHESIS defined) the Gowin MULT18X18 primitive is
// instantiated; simulation uses input registers and `*`, which behaves the
// same.
//

// 18x18 -> 36 unsigned, inputs registered.
module hmul18 (
    input  wire        clk,
    input  wire [17:0] a,
    input  wire [17:0] b,
    output wire [35:0] p
);
`ifdef SYNTHESIS
  MULT18X18 #(
      .AREG(1'b1), .BREG(1'b1), .OUT_REG(1'b0), .PIPE_REG(1'b0),
      .ASIGN_REG(1'b0), .BSIGN_REG(1'b0), .SOA_REG(1'b0),
      .MULT_RESET_MODE("SYNC")
  ) m (
      .A(a), .B(b), .ASIGN(1'b0), .BSIGN(1'b0), .CE(1'b1), .CLK(clk),
      .RESET(1'b0), .ASEL(1'b0), .BSEL(1'b0), .SIA(18'd0), .SIB(18'd0),
      .DOUT(p), .SOA(), .SOB()
  );
`else
  reg [17:0] ar = 18'd0, br = 18'd0;
  always @(posedge clk) begin
    ar <= a;
    br <= b;
  end
  assign p = ar * br;
`endif
endmodule

// 24x24 -> 48 unsigned (FPU significand product), from four 12x12 parts;
// valid two cycles after the inputs.
module mul24 (
    input  wire        clk,
    input  wire [23:0] a,
    input  wire [23:0] b,
    output wire [47:0] p
);
  wire [35:0] ll, lh, hl, hh;
  hmul18 m0 (.clk(clk), .a({6'd0, a[11:0]}),  .b({6'd0, b[11:0]}),  .p(ll));
  hmul18 m1 (.clk(clk), .a({6'd0, a[11:0]}),  .b({6'd0, b[23:12]}), .p(lh));
  hmul18 m2 (.clk(clk), .a({6'd0, a[23:12]}), .b({6'd0, b[11:0]}),  .p(hl));
  hmul18 m3 (.clk(clk), .a({6'd0, a[23:12]}), .b({6'd0, b[23:12]}), .p(hh));
  reg [23:0] ll_r, lh_r, hl_r, hh_r;
  always @(posedge clk) begin
    ll_r <= ll[23:0];
    lh_r <= lh[23:0];
    hl_r <= hl[23:0];
    hh_r <= hh[23:0];
  end
  assign p = {hh_r, 24'd0} + ({24'd0, lh_r} << 12) + ({24'd0, hl_r} << 12) + {24'd0, ll_r};
endmodule

// 32x32 -> low 32 bits (integer MUL; same for signed and unsigned), from
// three 16x16 parts.
module mul32lo (
    input  wire        clk,
    input  wire [31:0] a,
    input  wire [31:0] b,
    output wire [31:0] p
);
  wire [35:0] ll, lh, hl;
  hmul18 m0 (.clk(clk), .a({2'd0, a[15:0]}),  .b({2'd0, b[15:0]}),  .p(ll));
  hmul18 m1 (.clk(clk), .a({2'd0, a[15:0]}),  .b({2'd0, b[31:16]}), .p(lh));
  hmul18 m2 (.clk(clk), .a({2'd0, a[31:16]}), .b({2'd0, b[15:0]}),  .p(hl));
  reg [31:0] ll_r;
  reg [15:0] lh_r, hl_r;
  always @(posedge clk) begin
    ll_r <= ll[31:0];
    lh_r <= lh[15:0];
    hl_r <= hl[15:0];
  end
  assign p = ll_r + {lh_r, 16'd0} + {hl_r, 16'd0};
endmodule
