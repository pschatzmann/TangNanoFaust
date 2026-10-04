`timescale 1ns / 1ps
// dsp_core running a compiled Faust program, compared sample by sample
// (outputs AND cycle counts) against the simulator (src/TangNanoFaust/
// compiler/Isa.h) by run_core_test.sh. Expects config.vh / prog.hex /
// inputs.hex in the working directory (written by tests/dsp_test rtl).
`include "config.vh"

module tb_dsp_core;
  reg clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  reg                       run = 0;
  reg  [`NTF_PROG_AW-1:0]   entry = 0;
  wire                      busy, halted;
  wire [31:0]               cycles;
  reg  [`NTF_N_IN*32-1:0]   in_bus = 0;
  wire [`NTF_N_OUT*32-1:0]  out_bus;
  wire                      sd_req, sd_we;
  wire [20:0]               sd_addr;
  wire [31:0]               sd_wdata;
  reg  [31:0]               sd_rdata;
  reg                       sd_ack = 0;

  dsp_core #(
      .PROG_AW(`NTF_PROG_AW), .FAST_AW(`NTF_FAST_AW), .FAST_WORDS(`NTF_FAST_WORDS),
      .N_IN(`NTF_N_IN), .N_OUT(`NTF_N_OUT), .PROG_HEX("prog.hex")
  ) dut (
      .clk(clk), .rst_n(rst_n), .run(run), .entry(entry), .busy(busy),
      .idle(), .halted(halted), .cycles(cycles), .debug(), .in_bus(in_bus), .out_bus(out_bus),
      .host_req(1'b0), .host_we(1'b0), .host_addr({`NTF_FAST_AW{1'b0}}),
      .host_wdata(32'd0), .host_rdata(), .host_ack(),
      .prog_we(1'b0), .prog_waddr({`NTF_PROG_AW{1'b0}}), .prog_wdata(40'd0),
      .sd_req(sd_req), .sd_we(sd_we), .sd_addr(sd_addr), .sd_wdata(sd_wdata),
      .sd_rdata(sd_rdata), .sd_ack(sd_ack));

  // SDRAM model: acknowledges in the READ_WAIT-th / WRITE_WAIT-th cycle of
  // the request (Isa.h kSdramReadWait / kSdramWriteWait). Contents start
  // as 0xDEADBEEF like the simulator's Memory.
  // +sd_jitter=N adds 0..N random cycles per access, as refreshes do on the
  // real chip (outputs must still match; cycle counts then differ).
  parameter READ_WAIT = 8, WRITE_WAIT = 26;
  reg [31:0] sdram [0:(1 << 21)-1];
  integer    sd_cnt = 0, j, sd_jitter = 0, sd_extra = 0;
  initial if (!$value$plusargs("sd_jitter=%d", sd_jitter)) sd_jitter = 0;
  always @(posedge clk) begin
    sd_ack <= 1'b0;
    if (sd_req && !sd_ack) begin
      if (sd_cnt == 0 && sd_extra == 0 && sd_jitter > 0) sd_extra = 1 + ($urandom % (sd_jitter + 1));
      if (sd_extra > 1) begin
        sd_extra = sd_extra - 1;          // extra wait before the normal latency
      end else if (sd_cnt == (sd_we ? WRITE_WAIT : READ_WAIT) - 2) begin
        sd_extra = 0;
        sd_ack <= 1'b1;
        sd_cnt <= 0;
        if (sd_we) sdram[sd_addr] <= sd_wdata;
        else sd_rdata <= sdram[sd_addr];
      end else begin
        sd_cnt <= sd_cnt + 1;
      end
    end
  end

  parameter MAXS = 100000;
  reg [31:0] inputs [0:MAXS*`NTF_N_IN-1];
  integer nsamples, n, ch, fd, timeout;

  task run_from(input [`NTF_PROG_AW-1:0] e);
    begin
      #1;
      entry = e;
      run = 1;
      @(posedge clk);
      #1;
      run = 0;
      timeout = 0;
      while (!halted) begin
        @(posedge clk);
        #1;
        timeout = timeout + 1;
        if (timeout > 400000000) begin
          $display("TIMEOUT npc=%0d", dut.npc);
          $finish;
        end
      end
    end
  endtask

  initial begin
    if (!$value$plusargs("samples=%d", nsamples)) nsamples = 100;
    for (j = 0; j < (1 << 21); j = j + 1) sdram[j] = 32'hDEADBEEF;
    $readmemh("inputs.hex", inputs);
    fd = $fopen("rtl_out.txt", "w");
    repeat (3) @(posedge clk);
    rst_n = 1;
    @(posedge clk);
    run_from(`NTF_BOOT_ENTRY);
    $fdisplay(fd, "boot %0d", cycles);
    for (n = 0; n < nsamples; n = n + 1) begin
      for (ch = 0; ch < `NTF_N_IN; ch = ch + 1)
        in_bus[ch*32 +: 32] = inputs[n*`NTF_N_IN + ch];
      run_from(n == 0 ? {`NTF_PROG_AW{1'b0}} : `NTF_DSP_ENTRY);
      $fwrite(fd, "%0d", cycles);
      for (ch = 0; ch < `NTF_N_OUT; ch = ch + 1)
        $fwrite(fd, " %h", out_bus[ch*32 +: 32]);
      $fwrite(fd, "\n");
    end
    $fclose(fd);
    $finish;
  end
endmodule
