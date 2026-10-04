`timescale 1ns / 1ps
//
// Multi-cycle binary32 floating-point unit for dsp_core.v.
//
// Bit-exact with src/TangNanoFaust/compiler/Fp32.h (the specification, and
// the model the testbenches compare against):
//   - round to nearest even at 24 bits, then flush results below the normal
//     range to a signed zero (FTZ) and above it to infinity;
//   - denormal inputs are treated as zero (DAZ);
//   - NaN results are the canonical quiet NaN 0x7FC00000;
//   - F2I truncates; NaN/out-of-range give 0x80000000.
//
// Every operation hands a 27-bit significand (leading one at bit 26), a
// sticky bit and a biased exponent to the one shared rounding stage, so
// there is exactly one rounding rule in the design, as in roundPack().
//
// Latency (start -> done, see fp32::fpuLatency): ADD/SUB 3, MUL 3, DIV 29,
// SQRT 31, I2F 2, FLOOR 2, F2I 2; special cases (zero, inf, NaN operands,
// integral or zero floor results) 1. Add,
// multiply and int->float normalize and round in the cycle their result is
// ready; divide and sqrt go through the registered S_ROUND stage.
//
module fpu (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [2:0]  op,
    input  wire [31:0] a,       // v1 (top of stack)
    input  wire [31:0] b,       // v2
    output reg         done,
    output reg  [31:0] result
);

  localparam OP_ADD = 3'd0, OP_SUB = 3'd1, OP_MUL = 3'd2, OP_DIV = 3'd3,
             OP_SQRT = 3'd4, OP_I2F = 3'd5, OP_F2I = 3'd6, OP_FLOOR = 3'd7;

  localparam [31:0] QNAN = 32'h7FC00000;

  localparam S_IDLE = 4'd0, S_ADD_ALIGN = 4'd1, S_ADD_SUM = 4'd2,
             S_MUL = 4'd4, S_DIV = 4'd5, S_SQRT = 4'd6, S_I2F = 4'd7,
             S_ROUND = 4'd8, S_F2I = 4'd9, S_MULW = 4'd10;

  // Kept as encoded here (no FSM recoding) so that the default branch below
  // catches every unused code.
  (* fsm_encoding = "none" *) reg [3:0] state;

  // ---------------------------------------------------------------- unpack
  // DAZ: a zero exponent field means zero.
  wire        sa = a[31], sb = b[31] ^ (op == OP_SUB);
  wire [7:0]  ea = a[30:23], eb = b[30:23];
  wire        za = (ea == 8'd0), zb = (eb == 8'd0);
  wire        ia = (ea == 8'hFF) && (a[22:0] == 0), ib = (eb == 8'hFF) && (b[22:0] == 0);
  wire        na = (ea == 8'hFF) && (a[22:0] != 0), nb = (eb == 8'hFF) && (b[22:0] != 0);
  wire [23:0] ma = {1'b1, a[22:0]}, mb = {1'b1, b[22:0]};

  // ---------------------------------------------------------------- round
  reg         r_sign;
  reg  [26:0] r_sig;     // leading one at bit 26
  reg         r_sticky;
  reg  signed [11:0] r_exp;  // biased exponent of r_sig[26]

  // what gets rounded this cycle (q_*): the registered r_* for divide/sqrt,
  // or the add/multiply/int->float result directly (muxed below)
  reg         q_sign;
  reg  [26:0] q_sig;
  reg         q_sticky;
  reg  signed [11:0] q_exp;
  wire        r_guard = q_sig[2];
  wire        r_st    = q_sig[1] | q_sig[0] | q_sticky;
  wire [24:0] r_inc   = {1'b0, q_sig[26:3]} + 25'd1;
  wire        r_up    = r_guard & (r_st | q_sig[3]);
  wire [24:0] r_keep  = r_up ? r_inc : {1'b0, q_sig[26:3]};
  wire        r_carry = r_keep[24];
  wire signed [11:0] r_exp2 = q_exp + (r_carry ? 12'sd1 : 12'sd0);
  wire [22:0] r_frac  = r_carry ? r_keep[23:1] : r_keep[22:0];
  wire [31:0] r_bits  = (r_exp2 >= 12'sd255) ? {q_sign, 8'hFF, 23'd0} :
                        (r_exp2 <= 12'sd0)   ? {q_sign, 31'd0} :
                        {q_sign, r_exp2[7:0], r_frac};

  // ---------------------------------------------------------------- add
  reg         add_s1, add_s2;
  reg  [7:0]  add_e1;
  reg  [26:0] add_m1, add_m2;
  reg  [8:0]  add_d;
  reg  [27:0] add_sum;
  reg         add_neg;   // result sign

  wire        a_mag_ge = {ea, a[22:0]} >= {eb, b[22:0]};

  // aligned smaller operand, with sticky in bit 0
  wire [26:0] add_shift_in = add_m2;
  reg  [26:0] add_aligned;
  integer k;
  always @(*) begin
    if (add_d >= 9'd27) begin
      add_aligned = 27'd1;  // nonzero operand entirely in sticky
    end else begin
      add_aligned = add_shift_in >> add_d;
      for (k = 0; k < 27; k = k + 1)
        if (k < add_d && add_shift_in[k]) add_aligned[0] = 1'b1;
    end
  end

  // leading-zero count of the 28-bit sum (bit 27 is the carry position)
  function [4:0] lzc28(input [27:0] v);
    integer i;
    reg found;
    begin
      lzc28 = 5'd28;
      found = 1'b0;
      for (i = 27; i >= 0; i = i - 1)
        if (!found && v[i]) begin
          lzc28 = 27 - i;
          found = 1'b1;
        end
    end
  endfunction

  wire [4:0]  add_lz = lzc28(add_sum);

  // ---------------------------------------------------------------- mul
  wire [47:0] mul_p;
  mul24 u_mul (.clk(clk), .a(ma), .b(mb), .p(mul_p));  // start cycle's operands, 2 cycles later
  reg  signed [11:0] mul_e;

  // ---------------------------------------------------------------- div
  reg  [25:0] div_r;
  reg  [23:0] div_d;
  reg  [26:0] div_q;
  reg  [4:0]  div_n;
  reg  signed [11:0] div_e;
  wire [25:0] div_diff = div_r - {2'b00, div_d};
  wire        div_bit  = (div_r >= {2'b00, div_d});

  // ---------------------------------------------------------------- sqrt
  reg  [55:0] sq_op, sq_res, sq_one;
  reg  signed [11:0] sq_e;
  wire [55:0] sq_t = sq_res + sq_one;

  // ---------------------------------------------------------------- i2f
  reg  [31:0] i2f_v;
  reg         i2f_s;
  function [4:0] lzc32(input [31:0] v);
    integer i;
    reg found;
    begin
      lzc32 = 5'd31;
      found = 1'b0;
      for (i = 31; i >= 0; i = i - 1)
        if (!found && v[i]) begin
          lzc32 = 31 - i;
          found = 1'b1;
        end
    end
  endfunction
  wire [4:0]  i2f_lz = lzc32(i2f_v);
  wire [31:0] i2f_n  = i2f_v << i2f_lz;

  // ---------------------------------------------------------------- rounding input mux
  always @(*) begin
    case (state)
      S_ADD_SUM: begin  // normalize the sum
        q_sign = add_neg;
        if (add_sum[27]) begin
          q_sig    = add_sum[27:1];
          q_sticky = add_sum[0];
          q_exp    = $signed({4'd0, add_e1}) + 12'sd1;
        end else begin
          q_sig    = add_sum[26:0] << (add_lz - 5'd1);
          q_sticky = 1'b0;
          q_exp    = $signed({4'd0, add_e1}) - $signed({7'd0, add_lz}) + 12'sd1;
        end
      end
      S_MUL: begin
        q_sign = r_sign;
        if (mul_p[47]) begin
          q_sig    = mul_p[47:21];
          q_sticky = |mul_p[20:0];
          q_exp    = mul_e + 12'sd1;
        end else begin
          q_sig    = mul_p[46:20];
          q_sticky = |mul_p[19:0];
          q_exp    = mul_e;
        end
      end
      S_I2F: begin
        q_sign   = i2f_s;
        q_sig    = i2f_n[31:5];
        q_sticky = |i2f_n[4:0];
        q_exp    = 12'sd158 - $signed({7'd0, i2f_lz});
      end
      default: begin  // S_ROUND (divide, sqrt)
        q_sign   = r_sign;
        q_sig    = r_sig;
        q_sticky = r_sticky;
        q_exp    = r_exp;
      end
    endcase
  end

  // ---------------------------------------------------------------- floor
  // |x| < 2^23: integer part (+1 for negative non-integers), then the I2F
  // stage converts it back exactly
  wire [7:0]  fl_sh   = 8'd150 - ea;                       // fraction bits (1..)
  wire [23:0] fl_int  = (ea < 8'd127) ? 24'd0 : (ma >> fl_sh);
  wire [23:0] fl_mask = (ea < 8'd127) ? 24'hFFFFFF : ((24'd1 << fl_sh) - 24'd1);
  wire        fl_frac = (ma & fl_mask) != 24'd0;
  wire [23:0] fl_mag  = fl_int + ((sa && fl_frac) ? 24'd1 : 24'd0);

  // ---------------------------------------------------------------- f2i
  wire signed [9:0] f2i_e = {2'b00, ea} - 10'sd127;
  wire [31:0] f2i_mag = (f2i_e >= 10'sd23) ? ({8'd0, ma} << (f2i_e - 10'sd23))
                                           : ({8'd0, ma} >> (10'sd23 - f2i_e));
  // Two cycles: the shift here, the negation in S_F2I (keeps the path short).
  reg  [31:0] f2i_m;
  reg  [1:0]  f2i_k;    // 0: f2i_m, 1: -f2i_m, 2: 0, 3: 0x80000000
  wire [1:0]  f2i_kind = (ea == 8'hFF || f2i_e >= 10'sd31) ? 2'd3 :
                         (za || f2i_e < 0) ? 2'd2 : {1'b0, sa};

  // ---------------------------------------------------------------- control
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state  <= S_IDLE;
      done   <= 1'b0;
      result <= 32'd0;
    end else begin
      done <= 1'b0;
      case (state)
        S_IDLE: if (start) begin
          case (op)
            OP_ADD, OP_SUB: begin
              if (na || nb || (ia && ib && (sa != sb))) begin
                result <= QNAN; done <= 1'b1;
              end else if (ia || ib) begin
                result <= {ia ? sa : sb, 8'hFF, 23'd0}; done <= 1'b1;
              end else if (za && zb) begin
                result <= {sa & sb, 31'd0}; done <= 1'b1;
              end else if (za) begin
                result <= {sb, b[30:0]}; done <= 1'b1;
              end else if (zb) begin
                result <= a; done <= 1'b1;
              end else begin
                // order by magnitude: operand 1 is the larger one
                if (a_mag_ge) begin
                  add_s1 <= sa; add_e1 <= ea; add_m1 <= {ma, 3'b000};
                  add_s2 <= sb; add_m2 <= {mb, 3'b000}; add_d <= {1'b0, ea} - {1'b0, eb};
                end else begin
                  add_s1 <= sb; add_e1 <= eb; add_m1 <= {mb, 3'b000};
                  add_s2 <= sa; add_m2 <= {ma, 3'b000}; add_d <= {1'b0, eb} - {1'b0, ea};
                end
                state <= S_ADD_ALIGN;
              end
            end
            OP_MUL: begin
              if (na || nb || (ia && zb) || (ib && za)) begin
                result <= QNAN; done <= 1'b1;
              end else if (ia || ib) begin
                result <= {sa ^ sb, 8'hFF, 23'd0}; done <= 1'b1;
              end else if (za || zb) begin
                result <= {sa ^ sb, 31'd0}; done <= 1'b1;
              end else begin
                mul_e  <= $signed({4'd0, ea}) + $signed({4'd0, eb}) - 12'sd127;
                r_sign <= sa ^ sb;
                state  <= S_MULW;
              end
            end
            OP_DIV: begin
              if (na || nb || (ia && ib) || (za && zb)) begin
                result <= QNAN; done <= 1'b1;
              end else if (ia || zb) begin
                result <= {sa ^ sb, 8'hFF, 23'd0}; done <= 1'b1;
              end else if (ib || za) begin
                result <= {sa ^ sb, 31'd0}; done <= 1'b1;
              end else begin
                div_r  <= {2'b00, ma};
                div_d  <= mb;
                div_q  <= 27'd0;
                div_n  <= 5'd0;
                div_e  <= $signed({4'd0, ea}) - $signed({4'd0, eb}) + 12'sd127;
                r_sign <= sa ^ sb;
                state  <= S_DIV;
              end
            end
            OP_SQRT: begin
              if (na) begin
                result <= QNAN; done <= 1'b1;
              end else if (za) begin
                result <= {sa, 31'd0}; done <= 1'b1;
              end else if (sa) begin
                result <= QNAN; done <= 1'b1;
              end else if (ia) begin
                result <= 32'h7F800000; done <= 1'b1;
              end else begin
                // value = ma * 2^(ea-150); make the exponent even
                // (ea - 150 is odd exactly when ea is odd)
                sq_op  <= ea[0] ? ({32'd0, ma} << 31) : ({32'd0, ma} << 30);
                sq_e   <= ea[0] ? ($signed({4'd0, ea}) - 12'sd151) >>> 1
                                : ($signed({4'd0, ea}) - 12'sd150) >>> 1;
                sq_res <= 56'd0;
                sq_one <= 56'd1 << 54;
                r_sign <= 1'b0;
                state  <= S_SQRT;
              end
            end
            OP_I2F: begin
              if (a == 32'd0) begin
                result <= 32'd0; done <= 1'b1;
              end else begin
                i2f_s <= a[31];
                i2f_v <= a[31] ? (~a + 32'd1) : a;
                state <= S_I2F;
              end
            end
            OP_FLOOR: begin
              if (na) begin
                result <= QNAN; done <= 1'b1;
              end else if (za || ea == 8'hFF || ea >= 8'd150) begin
                result <= za ? {sa, 31'd0} : a;  // integral already
                done   <= 1'b1;
              end else if (fl_mag == 24'd0) begin
                result <= 32'd0; done <= 1'b1;
              end else begin
                i2f_s <= sa;
                i2f_v <= {8'd0, fl_mag};
                state <= S_I2F;
              end
            end
            default: begin  // OP_F2I
              f2i_m <= f2i_mag;
              f2i_k <= f2i_kind;
              state <= S_F2I;
            end
          endcase
        end

        S_F2I: begin
          result <= (f2i_k == 2'd3) ? 32'h80000000 : (f2i_k == 2'd2) ? 32'd0 :
                    f2i_k[0] ? (~f2i_m + 32'd1) : f2i_m;
          done   <= 1'b1;
          state  <= S_IDLE;
        end

        S_MULW: state <= S_MUL;  // partial products being registered (hmul.v)

        S_ADD_ALIGN: begin
          add_sum <= (add_s1 == add_s2) ? {1'b0, add_m1} + {1'b0, add_aligned}
                                        : {1'b0, add_m1} - {1'b0, add_aligned};
          add_neg <= add_s1;
          state   <= S_ADD_SUM;
        end

        S_ADD_SUM: begin  // normalize + round (q_*), or an exact zero
          result <= (add_sum == 28'd0) ? 32'd0 : r_bits;
          done   <= 1'b1;
          state  <= S_IDLE;
        end

        S_MUL: begin  // normalize + round the product (q_*)
          result <= r_bits;
          done   <= 1'b1;
          state  <= S_IDLE;
        end

        S_DIV: begin
          // one quotient bit per cycle, MSB (weight 2^26) first
          div_q <= {div_q[25:0], div_bit};
          div_r <= (div_bit ? div_diff : div_r) << 1;
          div_n <= div_n + 5'd1;
          if (div_n == 5'd26) begin
            if (div_q[25]) begin  // final q[26] = this div_q[25]
              r_sig    <= {div_q[25:0], div_bit};
              r_exp    <= div_e;
            end else begin
              r_sig    <= {div_q[24:0], div_bit, 1'b0};
              r_exp    <= div_e - 12'sd1;
            end
            r_sticky <= (div_bit ? div_diff : div_r) != 26'd0;
            state    <= S_ROUND;
          end
        end

        S_SQRT: begin
          if (sq_one == 56'd0) begin
            if (sq_res[27]) begin
              r_sig    <= sq_res[27:1];
              r_sticky <= sq_res[0] | (sq_op != 56'd0);
              r_exp    <= sq_e + 12'sd27 + 12'sd112;
            end else begin
              r_sig    <= sq_res[26:0];
              r_sticky <= (sq_op != 56'd0);
              r_exp    <= sq_e + 12'sd26 + 12'sd112;
            end
            state <= S_ROUND;
          end else begin
            if (sq_op >= sq_t) begin
              sq_op  <= sq_op - sq_t;
              sq_res <= (sq_res >> 1) + sq_one;
            end else begin
              sq_res <= sq_res >> 1;
            end
            sq_one <= sq_one >> 2;
          end
        end

        S_I2F: begin  // normalize + round (q_*)
          result <= r_bits;
          done   <= 1'b1;
          state  <= S_IDLE;
        end

        S_ROUND: begin
          result <= r_bits;
          done   <= 1'b1;
          state  <= S_IDLE;
        end

        // Unused state codes (only reachable through a timing error on the
        // chip): finish with NaN instead of leaving the core waiting forever.
        default: begin
          result <= QNAN;
          done   <= 1'b1;
          state  <= S_IDLE;
        end
      endcase
    end
  end

endmodule
