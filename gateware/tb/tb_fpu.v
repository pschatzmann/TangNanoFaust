`timescale 1ns / 1ps
// FPU vs. the bit-exact specification (tests/fpu_vectors.cpp, Fp32.h).
module tb_fpu;
  reg clk = 0, rst_n = 0, start = 0;
  reg [2:0] op;
  reg [31:0] a, b;
  wire done;
  wire [31:0] result;

  fpu dut (.clk(clk), .rst_n(rst_n), .start(start), .op(op), .a(a), .b(b),
           .done(done), .result(result));

  always #5 clk = ~clk;

  parameter MAXV = 200000;
  reg [31:0] vec [0:4*MAXV-1];
  integer i, n, errors, cycles, maxcyc [0:7];
  reg [8*512-1:0] path;

  initial begin
    if (!$value$plusargs("vectors=%s", path)) path = "fpu_vectors.hex";
    for (i = 0; i < 4*MAXV; i = i + 1) vec[i] = 32'hxxxxxxxx;
    $readmemh(path, vec);
    for (i = 0; i < 8; i = i + 1) maxcyc[i] = 0;
    errors = 0;
    n = 0;
    repeat (3) @(posedge clk);
    rst_n = 1;
    @(posedge clk);
    while (vec[4*n] !== 32'hxxxxxxxx && n < MAXV) begin
      #1;
      op = vec[4*n][2:0];
      a = vec[4*n+1];
      b = vec[4*n+2];
      start = 1;
      @(posedge clk);
      #1;
      start = 0;
      cycles = 1;
      while (!done) begin
        @(posedge clk);
        #1;
        cycles = cycles + 1;
      end
      if (cycles > maxcyc[op]) maxcyc[op] = cycles;
      if (result !== vec[4*n+3]) begin
        errors = errors + 1;
        if (errors <= 20)
          $display("MISMATCH op=%0d a=%h b=%h got=%h expected=%h", op, a, b, result, vec[4*n+3]);
      end
      n = n + 1;
      @(posedge clk);
    end
    $display("tb_fpu: %0d vectors, %0d errors; max cycles add=%0d sub=%0d mul=%0d div=%0d sqrt=%0d i2f=%0d f2i=%0d floor=%0d",
             n, errors, maxcyc[0], maxcyc[1], maxcyc[2], maxcyc[3], maxcyc[4], maxcyc[5], maxcyc[6], maxcyc[7]);
    if (errors == 0 && n > 0) $display("PASS"); else $display("FAIL");
    $finish;
  end
endmodule
