`timescale 1ns / 1ps
// System test of top_tangnano20k through its pins (run_top_test.sh):
// boots the program the bitstream was built with, loads another program
// over SPI (stop, program, descriptor, config, reboot), sets two
// parameters, reads one back, then feeds known samples into the I2S input
// and records what comes out of the I2S output.
`include "config.vh"

module tb_top;
  reg clk = 0;
  always #9.259 clk = ~clk;  // 54 MHz (pll_sys passes clk_27m through in simulation)

  reg  reset_button = 1;
  reg  sclk = 0, mosi = 0, cs_n = 1;
  wire miso;
  wire bclk, ws, dout, pa_en;
  reg  rx_din = 0;
  wire [5:0] leds;

  // TDM master model (+tdm=1): 12.288 MHz BCLK, 256 bits per frame, FS high
  // for one bit; driven on falling edges like a real master.
  reg  tdm_on = 0, tbclk = 0, tfs = 0;
  integer tbit = 255;
  real tdm_half = 40.69;  // ns; +tdm_half=44.29 for 44.1 kHz
  always begin
    #(tdm_half);
    if (tdm_on) tbclk = ~tbclk;
  end
  always @(negedge tbclk) begin
    tbit = (tbit + 1) % 256;
    tfs  = (tbit == 0);
  end
  wire tdm_dout;
  reg  urx = 1'b1;   // host -> FPGA (USB UART)
  wire utx;          // FPGA -> host

  top_tangnano20k dut (
      .clk_27m(clk), .reset_button(reset_button),
      .spi_sclk(sclk), .spi_mosi(mosi), .spi_miso(miso), .spi_cs_n(cs_n),
      .i2s_bclk(bclk), .i2s_ws(ws), .i2s_din(dout), .i2s_pa_en(pa_en),
      .i2s_rx_din(rx_din), .tdm_bclk(tbclk), .tdm_fs(tfs), .tdm_dout(tdm_dout),
      .usb_uart_rx(urx), .usb_uart_tx(utx), .uart_rx(1'b1), .uart_tx(),
      .leds(leds));

  // ------------------------------------------------------------ SPI master (mode 0, 4 MHz)
  reg [7:0] spi_in;
  task spi_byte(input [7:0] out);
    integer i;
    begin
      for (i = 7; i >= 0; i = i - 1) begin
        mosi = out[i];
        #125 sclk = 1;
        spi_in[i] = miso;
        #125 sclk = 0;
      end
    end
  endtask
  task cs_low;  begin #200 cs_n = 0; #200; end endtask
  task cs_high; begin #200 cs_n = 1; #400; end endtask

  // ------------------------------------------------------------ UART host (+uart), 115200 8N1
  localparam real UBIT = 1.0e9 / 115200;
  task uart_put(input [7:0] b);
    integer k;
    begin
      urx = 1'b0; #(UBIT);
      for (k = 0; k < 8; k = k + 1) begin urx = b[k]; #(UBIT); end
      urx = 1'b1; #(UBIT);
    end
  endtask
  task uart_get(output [7:0] b);
    integer k;
    begin
      @(negedge utx);           // start bit
      #(UBIT * 1.5);
      for (k = 0; k < 8; k = k + 1) begin b[k] = utx; #(UBIT); end
    end
  endtask

  // ------------------------------------------------------------ transactions
  // t_begin; t_byte(...)...; t_end(noreply): runs over SPI or, with +uart,
  // as a frame on the USB UART; the reply bytes end up in rbuf.
  reg        use_uart = 0;
  reg [7:0]  tbuf [0:1023];
  reg [7:0]  rbuf [0:1023];
  integer    tn;
  task t_begin; tn = 0; endtask
  task t_byte(input [7:0] b); begin tbuf[tn] = b; tn = tn + 1; end endtask
  task t_end(input noreply);
    integer k;
    reg [7:0] b;
    begin
      if (use_uart) begin
        uart_put(8'h7E);
        uart_put(tn[7:0]);
        uart_put({noreply, tn[14:8]});
        for (k = 0; k < tn; k = k + 1) uart_put(tbuf[k]);
        if (noreply) begin
          uart_get(b);
          if (b != 8'h7E) $display("UART: bad ack %h", b);
        end else begin
          for (k = 0; k < tn; k = k + 1) begin uart_get(b); rbuf[k] = b; end
        end
      end else begin
        cs_low;
        for (k = 0; k < tn; k = k + 1) begin spi_byte(tbuf[k]); rbuf[k] = spi_in; end
        cs_high;
      end
    end
  endtask

  reg [7:0] info [0:22];
  task get_info;
    integer i;
    begin
      t_begin; t_byte(8'h02);
      for (i = 0; i < 23; i = i + 1) t_byte(8'h00);
      t_end(0);
      for (i = 0; i < 23; i = i + 1) info[i] = rbuf[1 + i];
    end
  endtask

  task wait_booted;
    integer tries;
    begin
      tries = 0;
      get_info;
      while (!(info[3] & 8'h01)) begin
        #20000;
        get_info;
        tries = tries + 1;
        if (tries > 2000) begin $display("FAIL: boot timeout"); $finish; end
      end
    end
  endtask

  // ------------------------------------------------------------ program to load
  reg [7:0] prog_bytes [0:65535];
  reg [7:0] desc_bytes [0:4095];
  reg [7:0] cfg_bytes  [0:16];
  integer n_prog, n_desc, gain_addr, offset_addr;
  reg [31:0] gain_bits, offset_bits;

  // ------------------------------------------------------------ I2S input (tb transmits)
  parameter NIN = 64;
  reg [23:0] in_samples [0:NIN-1];
  integer frame = 0;
  always @(posedge clk) if (dut.u_i2s.frame_start) frame <= frame + 1;
  wire [63:0] in_frame = {1'b0, in_samples[frame % NIN], 7'd0, 32'd0};
  always @(*) rx_din = in_frame[63 - dut.u_i2s.bit_cnt];

  // ------------------------------------------------------------ I2S output decoder (pins)
  reg        bclk_d = 0;
  reg [63:0] out_sh = 0;
  reg        recording = 0;
  integer    fd, rec_count = 0;
  parameter  NREC = 48;
  always @(posedge clk) begin
    bclk_d <= bclk;
    if (bclk && !bclk_d) begin  // rising BCLK edge: sample dout
      out_sh <= {out_sh[62:0], dout};
      if (dut.u_i2s.bit_cnt == 6'd63 && recording && rec_count < NREC) begin
        // frame complete: left slot is bits 62:39 of the 64-bit frame
        // ({out_sh, dout} is the whole frame; its bits 62:39 = out_sh[61:38])
        $fdisplay(fd, "%0d %h", frame, out_sh[61:38]);
        rec_count <= rec_count + 1;
      end
    end
  end

  // TDM decoder: the slave sends slot 0's MSB in the bit after FS, so the
  // frame is complete at the rising edge where the next FS is seen.
  reg [255:0] tsh = 0;
  integer tdm_count = 0;
  always @(posedge tbclk) begin
    tsh <= {tsh[254:0], tdm_dout};
    if (tfs && recording && tdm_count < NREC) begin
      $fdisplay(fd, "tdm %0d %h %h %h %h", frame, tsh[254:231], tsh[222:199], tsh[190:167],
                tsh[158:135]);
      tdm_count = tdm_count + 1;
    end
  end

  integer i, j;
  reg [7:0] ping [0:4];
  reg [31:0] readback;
  initial begin
    $readmemh("load_prog.hex", prog_bytes);
    $readmemh("load_desc.hex", desc_bytes);
    $readmemh("load_cfg.hex", cfg_bytes);
    $readmemh("in_samples.hex", in_samples);
    if (!$value$plusargs("prog_bytes=%d", n_prog)) n_prog = 0;
    if (!$value$plusargs("desc_bytes=%d", n_desc)) n_desc = 0;
    if (!$value$plusargs("gain_addr=%d", gain_addr)) gain_addr = 0;
    if (!$value$plusargs("offset_addr=%d", offset_addr)) offset_addr = 0;
    if (!$value$plusargs("gain_bits=%h", gain_bits)) gain_bits = 0;
    if (!$value$plusargs("offset_bits=%h", offset_bits)) offset_bits = 0;
    if ($test$plusargs("tdm")) tdm_on = 1;
    if ($test$plusargs("uart")) use_uart = 1;
    if (!$value$plusargs("tdm_half=%f", tdm_half)) tdm_half = 40.69;
    fd = $fopen("tb_out.txt", "w");

    #500 reset_button = 0;
    wait_booted;

    // ping
    t_begin; t_byte(8'h01); for (i = 0; i < 5; i = i + 1) t_byte(8'h00); t_end(0);
    for (i = 0; i < 5; i = i + 1) ping[i] = rbuf[1 + i];
    $fdisplay(fd, "ping %c%c%c%c %0d", ping[0], ping[1], ping[2], ping[3], ping[4]);
    $fdisplay(fd, "info_boot in=%0d out=%0d params=%0d flags=%h caps=%0d,%0d,%0d channels=%0d,%0d",
              info[0], info[1], info[2], info[3], info[18], info[19], info[20], info[21], info[22]);

    // load: stop, program, descriptor, config, reboot (chunks fit UART frames)
    t_begin; t_byte(8'h20); t_byte(8'h0A); t_end(1);
    #5000;
    for (j = 0; j < n_prog; j = j + 500) begin  // 100 instructions per chunk
      t_begin; t_byte(8'h30); t_byte((j / 5) & 8'hFF); t_byte((j / 5) >> 8);
      for (i = j; i < j + 500 && i < n_prog; i = i + 1) t_byte(prog_bytes[i]);
      t_end(1);
    end
    for (j = 0; j < n_desc; j = j + 256) begin
      t_begin; t_byte(8'h31); t_byte(j & 8'hFF); t_byte(j >> 8);
      for (i = j; i < j + 256 && i < n_desc; i = i + 1) t_byte(desc_bytes[i]);
      t_end(1);
    end
    t_begin; t_byte(8'h32); for (i = 0; i < 17; i = i + 1) t_byte(cfg_bytes[i]); t_end(1);
    t_begin; t_byte(8'h20); t_byte(8'h06); t_end(1);
    wait_booted;
    $fdisplay(fd, "info_load in=%0d out=%0d params=%0d desc_len=%0d", info[0], info[1], info[2],
              info[8] | (info[9] << 8));

    // descriptor read back, in chunks
    $fwrite(fd, "desc ");
    for (j = 0; j < n_desc; j = j + 256) begin
      t_begin; t_byte(8'h03); t_byte(j & 8'hFF); t_byte(j >> 8); t_byte(8'h00);
      for (i = j; i < j + 256 && i < n_desc; i = i + 1) t_byte(8'h00);
      t_end(0);
      for (i = j; i < j + 256 && i < n_desc; i = i + 1) $fwrite(fd, "%h", rbuf[4 + i - j]);
    end
    $fwrite(fd, "\n");

    // parameters
    t_begin; t_byte(8'h10); t_byte(gain_addr[7:0]); t_byte(gain_addr[15:8]);
    for (i = 0; i < 4; i = i + 1) t_byte(gain_bits >> (8 * i));
    t_end(1);
    t_begin; t_byte(8'h10); t_byte(offset_addr[7:0]); t_byte(offset_addr[15:8]);
    for (i = 0; i < 4; i = i + 1) t_byte(offset_bits >> (8 * i));
    t_end(1);
    t_begin; t_byte(8'h11); t_byte(gain_addr[7:0]); t_byte(gain_addr[15:8]);
    for (i = 0; i < 64; i = i + 1) t_byte(8'h00);  // polls for READY, then the value
    t_end(0);
    j = 3;
    while (rbuf[j] != 8'hA5 && j < 3 + 59) j = j + 1;
    readback = {rbuf[j + 4], rbuf[j + 3], rbuf[j + 2], rbuf[j + 1]};
    $fdisplay(fd, "readback %h polls=%0d", readback, j - 3);

    // let the parameters reach the DSP, then record the output
    repeat (4) @(posedge dut.u_i2s.frame_start);
    recording = 1;
    wait (rec_count == NREC && (!tdm_on || tdm_count == NREC));
    get_info;
    $fdisplay(fd, "info_end flags=%h tdm_active=%0d cycles_max=%0d", info[3], (info[3] >> 5) & 1,
              info[10] | (info[11] << 8) | (info[12] << 16) | (info[13] << 24));
    $fclose(fd);
    $finish;
  end
endmodule
