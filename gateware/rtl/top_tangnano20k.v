`timescale 1ns / 1ps
//
// TangNanoFaust top level for the Sipeed Tang Nano 20K.
//
//   I2S master (onboard MAX98357A) --frame_start--> sequencer --> dsp_core
//   SPI (host MCU) --> spi_ctrl --> parameter writes/reads between samples
//
// With TDM built in (NTF_TDM, `make TDM=1`) and an external TDM master
// connected (tdm_*), its frame sync becomes the sample clock of everything:
// the onboard I2S follows it, and outputs 0..7 also go out in TDM slots
// 0..7. Without NTF_TDM the tdm_* pins are unused.
//
// Once per I2S frame the sequencer starts the core: from address 0 (control
// block + dsp block) when a parameter changed since the last sample,
// otherwise from the dsp block. Outputs are played one sample later. After
// reset (or a CONTROL reboot) it first runs the boot program, which fills
// tables, sets defaults and clears the state.
//
// Build-time configuration comes from the compiler's config.vh (memory
// sizes, and the program the bitstream boots with); NTF_SDRAM (Makefile)
// adds the SDRAM controller, NTF_TDM the TDM output. A different program can be loaded at runtime
// over SPI (stop, write program + descriptor, set config, reboot) -- see
// TangNanoFaust::load().
//
`include "config.vh"

module top_tangnano20k (
    input  wire       clk_27m,
    input  wire       reset_button,   // S1, active high

    // SPI from the host MCU
    input  wire       spi_sclk,
    input  wire       spi_mosi,
    output wire       spi_miso,
    input  wire       spi_cs_n,

    // onboard MAX98357A + optional external I2S input
    output wire       i2s_bclk,
    output wire       i2s_ws,
    output wire       i2s_din,
    output wire       i2s_pa_en,
    input  wire       i2s_rx_din,

    // UART command ports (115200 8N1): the onboard USB bridge (BL616) and a
    // header UART for an MCU
    input  wire       usb_uart_rx,
    output wire       usb_uart_tx,
    input  wire       uart_rx,
    output wire       uart_tx,

    // TDM output, slave: the external master drives BCLK and FS (NTF_TDM)
    input  wire       tdm_bclk,
    input  wire       tdm_fs,
    output wire       tdm_dout,

    output wire [5:0] leds            // active low

`ifdef NTF_SDRAM
    ,
    inout  wire [31:0] IO_sdram_dq,
    output wire [10:0] O_sdram_addr,
    output wire [1:0]  O_sdram_ba,
    output wire        O_sdram_cs_n,
    output wire        O_sdram_wen_n,
    output wire        O_sdram_ras_n,
    output wire        O_sdram_cas_n,
    output wire        O_sdram_clk,
    output wire        O_sdram_cke,
    output wire [3:0]  O_sdram_dqm
`endif
);

`ifndef NTF_CLK_HZ
`define NTF_CLK_HZ 54000000
`endif
  localparam integer CLK_HZ = `NTF_CLK_HZ;
`ifndef NTF_UART_BAUD
`define NTF_UART_BAUD 115200
`endif
// The UART bridge runs on the 27 MHz oscillator, so its baud rate doesn't
// depend on the system clock (simulation feeds clk_27m at another rate).
`ifndef NTF_REF_HZ
`define NTF_REF_HZ 27000000
`endif
// SDRAM controller timings are computed for this clock (default: the system
// clock); a higher value gives conservative timings at a lower real clock.
`ifndef NTF_SDRAM_HZ
`define NTF_SDRAM_HZ `NTF_CLK_HZ
`endif
`ifdef NTF_TDM
  localparam HAS_TDM = 1;
`else
  localparam HAS_TDM = 0;
`endif
  localparam [63:0]  BCLK_INC64 = ((64'd1 << 32) * `NTF_SAMPLE_RATE * 128) / CLK_HZ;

  // ---------------------------------------------------------------- clock/reset
  wire clk, clk_sdram, pll_locked;
  pll_sys u_pll (.clock_in(clk_27m), .clock_out(clk), .clock_p180(clk_sdram),
                 .locked(pll_locked));

  reg [3:0] rst_sync = 4'd0;
  always @(posedge clk or posedge reset_button)
    if (reset_button) rst_sync <= 4'd0;
    else rst_sync <= {rst_sync[2:0], pll_locked};
  wire rst_n = rst_sync[3];

  // reset for the 27 MHz domain (UART bridge): independent of the PLL
  reg [3:0] rst27_sync = 4'd0;
  always @(posedge clk_27m or posedge reset_button)
    if (reset_button) rst27_sync <= 4'd0;
    else rst27_sync <= {rst27_sync[2:0], 1'b1};
  wire rst27_n = rst27_sync[3];

  // ---------------------------------------------------------------- I2S
  wire [23:0] rx_left, rx_right;
  wire [23:0] tx_left, tx_right;
  wire        frame_start;
  wire [31:0] cfg_bclk_inc;
  wire        tdm_active, tdm_fs_tick, tdm_bclk_tick;
  i2s_master u_i2s (
      .clk(clk), .rst_n(rst_n), .bclk_inc(cfg_bclk_inc),
      .ext_en(tdm_active), .ext_tick(tdm_bclk_tick), .ext_sync(tdm_fs_tick),
      .tx_left(tx_left), .tx_right(tx_right),
      .rx_left(rx_left), .rx_right(rx_right), .frame_start(frame_start),
      .bclk(i2s_bclk), .ws(i2s_ws), .dout(i2s_din), .rx_din(i2s_rx_din));

  // ---------------------------------------------------------------- core
  wire                        core_busy, core_idle, core_halted;
  wire [31:0]                 core_cycles;
  wire [103:0]                core_debug;
  reg                         run;
  reg  [`NTF_PROG_AW-1:0]     entry;
  wire                        prog_we;
  wire [`NTF_PROG_AW-1:0]     prog_waddr;
  wire [39:0]                 prog_wdata;
  wire [`NTF_N_IN*32-1:0]     in_bus;
  wire [`NTF_N_OUT*32-1:0]    out_bus;
  reg                         host_req, host_we;
  reg  [`NTF_FAST_AW-1:0]     host_addr;
  reg  [31:0]                 host_wdata;
  wire [31:0]                 host_rdata;
  wire                        host_ack;
  wire                        sd_req, sd_we, sd_ack;
  wire [20:0]                 sd_addr;
  wire [31:0]                 sd_wdata, sd_rdata;

  dsp_core #(
      .PROG_AW(`NTF_PROG_AW), .FAST_AW(`NTF_FAST_AW), .FAST_WORDS(`NTF_FAST_WORDS),
      .N_IN(`NTF_N_IN), .N_OUT(`NTF_N_OUT), .PROG_HEX("prog.hex")
  ) u_core (
      .clk(clk), .rst_n(rst_n), .run(run), .entry(entry), .busy(core_busy),
      .idle(core_idle), .halted(core_halted), .cycles(core_cycles), .debug(core_debug),
      .in_bus(in_bus), .out_bus(out_bus),
      .host_req(host_req), .host_we(host_we), .host_addr(host_addr),
      .host_wdata(host_wdata), .host_rdata(host_rdata), .host_ack(host_ack),
      .prog_we(prog_we), .prog_waddr(prog_waddr), .prog_wdata(prog_wdata),
      .sd_req(sd_req), .sd_we(sd_we), .sd_addr(sd_addr), .sd_wdata(sd_wdata),
      .sd_rdata(sd_rdata), .sd_ack(sd_ack));

  // audio in: left -> input 0, right -> input 1
  wire [31:0] in_l, in_r;
  s24_to_f32 u_cin_l (.v(rx_left), .f(in_l));
  s24_to_f32 u_cin_r (.v(rx_right), .f(in_r));
  genvar gi;
  generate
    for (gi = 0; gi < `NTF_N_IN; gi = gi + 1) begin : g_in
      assign in_bus[gi*32 +: 32] = (gi == 0) ? in_l : (gi == 1) ? in_r : 32'd0;
    end
  endgenerate

  // audio out: output 0 -> left, output 1 (or 0 again if mono) -> right
  wire [23:0] out_l, out_r;
  wire [31:0] out_second;
  wire [7:0]  cfg_n_out;
  generate
    if (`NTF_N_OUT >= 2) begin : g_stereo
      assign out_second = (cfg_n_out >= 8'd2) ? out_bus[63:32] : out_bus[31:0];
    end else begin : g_mono
      assign out_second = out_bus[31:0];
    end
  endgenerate
  f32_to_s24 u_cout_l (.f(out_bus[31:0]), .v(out_l));
  f32_to_s24 u_cout_r (.f(out_second), .v(out_r));

  // ---------------------------------------------------------------- SPI
  wire                    wr_valid, rd_req, mute, amp_en, reboot, stop, stats_clear;
  wire [15:0]             cfg_dsp_entry, cfg_boot_entry, cfg_desc_len;
  wire [7:0]              cfg_n_in, cfg_n_params;
  wire [31:0]             cfg_sample_rate;
  wire [`NTF_FAST_AW-1:0] wr_addr, rd_addr;
  wire [31:0]             wr_data;
  reg                     wr_pop, rd_done;
  reg  [31:0]             rd_data;
  reg                     booted, overrun;
  reg  [31:0]             cycles_max;

  // SPI pins and the UART bridge share the command interface, one host at a
  // time: the bridge starts only while SPI chip select is idle. The bridge
  // runs in the 27 MHz domain; its virtual SPI pins enter spi_ctrl through
  // the same synchronizers as the real pins, and MISO and the chip select
  // it watches are synchronized into its domain here. The USB and
  // the header UART share one bridge (one block RAM): both lines idle high,
  // so their RX are ANDed and TX goes to both. Use one of them at a time.
  wire b_cs_n, b_sclk, b_mosi, b_active, b_tx;
  wire ctl_miso;
  wire v_sclk = b_active ? b_sclk : spi_sclk;
  wire v_mosi = b_active ? b_mosi : spi_mosi;
  wire v_cs_n = b_active ? b_cs_n : spi_cs_n;
  assign usb_uart_tx = b_tx;
  assign uart_tx     = b_tx;
  reg [1:0] b_miso_s = 2'b00, b_free_s = 2'b11;
  always @(posedge clk_27m) begin
    b_miso_s <= {b_miso_s[0], ctl_miso};
    b_free_s <= {b_free_s[0], spi_cs_n};
  end
  uart_bridge #(.CLK_HZ(`NTF_REF_HZ), .BAUD(`NTF_UART_BAUD)) u_uart (
      .clk(clk_27m), .rst_n(rst27_n), .rx(usb_uart_rx & uart_rx), .tx(b_tx),
      .bus_free(b_free_s[1]), .v_cs_n(b_cs_n), .v_sclk(b_sclk),
      .v_mosi(b_mosi), .v_miso(b_miso_s[1]), .active(b_active));

  spi_ctrl #(
      .FAST_AW(`NTF_FAST_AW), .PROG_AW(`NTF_PROG_AW), .DESC_AW(`NTF_DESC_AW),
      .DESC_HEX("desc.hex"), .DESC_LEN(`NTF_DESC_LEN), .DSP_ENTRY(`NTF_DSP_ENTRY),
      .BOOT_ENTRY(`NTF_BOOT_ENTRY), .N_IN(`NTF_DSP_INPUTS), .N_OUT(`NTF_DSP_OUTPUTS),
      .N_PARAMS(`NTF_N_PARAMS), .SAMPLE_RATE(`NTF_SAMPLE_RATE),
      .BCLK_INC(BCLK_INC64[31:0]), .CLK_HZ(CLK_HZ), .USE_SDRAM(`NTF_USE_SDRAM),
      .HW_IN(`NTF_N_IN), .HW_OUT(`NTF_N_OUT), .HAS_TDM(HAS_TDM)
  ) u_spi (
      .clk(clk), .rst_n(rst_n),
      .sclk_pin(v_sclk), .mosi_pin(v_mosi), .miso_pin(spi_miso), .cs_n_pin(v_cs_n),
      .miso_int(ctl_miso),
      .wr_valid(wr_valid), .wr_addr(wr_addr), .wr_data(wr_data), .wr_pop(wr_pop),
      .rd_req(rd_req), .rd_addr(rd_addr), .rd_done(rd_done), .rd_data(rd_data),
      .prog_we(prog_we), .prog_waddr(prog_waddr), .prog_wdata(prog_wdata),
      .cfg_dsp_entry(cfg_dsp_entry), .cfg_boot_entry(cfg_boot_entry),
      .cfg_n_in(cfg_n_in), .cfg_n_out(cfg_n_out), .cfg_n_params(cfg_n_params),
      .cfg_sample_rate(cfg_sample_rate), .cfg_bclk_inc(cfg_bclk_inc),
      .cfg_desc_len(cfg_desc_len),
      .booted(booted), .overrun(overrun), .cycles_max(cycles_max),
      .tdm_active(tdm_active), .core_debug(core_debug), .stats_clear(stats_clear), .mute(mute), .amp_en(amp_en), .stop(stop),
      .reboot(reboot));

  assign tx_left  = (mute || stop || !booted) ? 24'd0 : out_l;
  assign tx_right = (mute || stop || !booted) ? 24'd0 : out_r;
  assign i2s_pa_en = amp_en;

  // ---------------------------------------------------------------- sequencer
  localparam Q_IDLE = 2'd0, Q_RUN = 2'd1, Q_WRITE = 2'd2, Q_READ = 2'd3;
  reg [1:0] q;
  reg       boot_pending, booting, run_pending, dirty;
  reg       rd_busy;
  reg [15:0] heartbeat;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      q            <= Q_IDLE;
      run          <= 1'b0;
      entry        <= {`NTF_PROG_AW{1'b0}};
      host_req     <= 1'b0;
      host_we      <= 1'b0;
      wr_pop       <= 1'b0;
      rd_done      <= 1'b0;
      rd_busy      <= 1'b0;
      boot_pending <= 1'b1;
      booting      <= 1'b0;
      booted       <= 1'b0;
      run_pending  <= 1'b0;
      dirty        <= 1'b1;
      overrun      <= 1'b0;
      cycles_max   <= 32'd0;
      heartbeat    <= 16'd0;
    end else begin
      run      <= 1'b0;
      host_req <= 1'b0;
      wr_pop   <= 1'b0;
      rd_done  <= 1'b0;

      if (stats_clear) begin
        overrun    <= 1'b0;
        cycles_max <= 32'd0;
      end
      if (reboot) begin
        boot_pending <= 1'b1;
        booted       <= 1'b0;
      end
      if (stop) begin
        booted       <= 1'b0;   // a new program is being loaded
        run_pending  <= 1'b0;
      end
      if (frame_start && booted && !stop) begin
        heartbeat <= heartbeat + 16'd1;
        if (run_pending || core_busy || q == Q_RUN) overrun <= 1'b1;
        else run_pending <= 1'b1;
      end
      if (core_halted) begin
        if (booting) begin
          booting <= 1'b0;
          booted  <= 1'b1;
          dirty   <= 1'b1;
        end else if (core_cycles > cycles_max && !stats_clear) begin
          cycles_max <= core_cycles;
        end
      end
      if (!rd_req) rd_busy <= 1'b0;

      case (q)
        Q_IDLE: if (core_idle && !run && !host_req) begin
          if (boot_pending && !stop) begin
            run          <= 1'b1;
            entry        <= cfg_boot_entry[`NTF_PROG_AW-1:0];
            boot_pending <= 1'b0;
            booting      <= 1'b1;
            run_pending  <= 1'b0;
            q            <= Q_RUN;
          end else if (run_pending && booted) begin
            run         <= 1'b1;
            entry       <= dirty ? {`NTF_PROG_AW{1'b0}} : cfg_dsp_entry[`NTF_PROG_AW-1:0];
            dirty       <= 1'b0;
            run_pending <= 1'b0;
            q           <= Q_RUN;
          end else if (wr_valid) begin
            host_req   <= 1'b1;
            host_we    <= 1'b1;
            host_addr  <= wr_addr;
            host_wdata <= wr_data;
            q          <= Q_WRITE;
          end else if (rd_req && !rd_busy) begin
            host_req  <= 1'b1;
            host_we   <= 1'b0;
            host_addr <= rd_addr;
            q         <= Q_READ;
          end
        end
        Q_RUN: if (core_halted) q <= Q_IDLE;
        Q_WRITE: if (host_ack) begin
          wr_pop <= 1'b1;
          dirty  <= 1'b1;
          q      <= Q_IDLE;
        end
        Q_READ: if (host_ack) begin
          rd_data <= host_rdata;
          rd_done <= 1'b1;
          rd_busy <= 1'b1;
          q       <= Q_IDLE;
        end
      endcase
    end
  end

  // ---------------------------------------------------------------- TDM
`ifdef NTF_TDM
  // After every sample, convert the core's outputs (one shared converter,
  // one channel per clock) and hand them to the TDM transmitter: output k
  // goes to slot k. Unused slots, and everything while muted, are silent.
  localparam integer TDM_SLOTS = 8;
  reg  [TDM_SLOTS*24-1:0] tdm_frame;
  reg                     tdm_we;
  reg  [3:0]              tdm_ch;
  reg                     tdm_busy;
  wire [31:0]             tdm_f = (tdm_ch < `NTF_N_OUT && tdm_ch < cfg_n_out)
                                  ? out_bus[tdm_ch[2:0]*32 +: 32] : 32'd0;
  wire [23:0]             tdm_s;
  f32_to_s24 u_cout_tdm (.f(tdm_f), .v(tdm_s));
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tdm_we   <= 1'b0;
      tdm_busy <= 1'b0;
      tdm_ch   <= 4'd0;
    end else begin
      tdm_we <= 1'b0;
      if (core_halted && !booting) begin
        tdm_busy <= 1'b1;
        tdm_ch   <= 4'd0;
      end else if (tdm_busy) begin
        tdm_frame[tdm_ch[2:0]*24 +: 24] <= (mute || stop) ? 24'd0 : tdm_s;
        tdm_ch <= tdm_ch + 4'd1;
        if (tdm_ch == TDM_SLOTS - 1) begin
          tdm_busy <= 1'b0;
          tdm_we   <= 1'b1;
        end
      end
    end
  end

  tdm_tx #(.SLOTS(TDM_SLOTS)) u_tdm (
      .clk(clk), .rst_n(rst_n), .frame(tdm_frame), .frame_we(tdm_we),
      .tdm_bclk(tdm_bclk), .tdm_fs(tdm_fs), .tdm_dout(tdm_dout),
      .active(tdm_active), .fs_tick(tdm_fs_tick), .bclk_tick(tdm_bclk_tick));
`else
  // No TDM in this build: the onboard I2S master is always the sample clock.
  assign tdm_active    = 1'b0;
  assign tdm_fs_tick   = 1'b0;
  assign tdm_bclk_tick = 1'b0;
  assign tdm_dout      = 1'b0;
`endif

  // ---------------------------------------------------------------- SDRAM
`ifdef NTF_SDRAM
  wire sd_ready;
  sdram_bus #(.FREQ(`NTF_SDRAM_HZ)) u_sdram (
      .clk(clk), .clk_sdram(clk_sdram), .reset_n(rst_n),
      .sel(sd_req), .addr({sd_addr, 2'b00}), .wstrb(sd_we ? 4'hF : 4'h0),
      .wdata(sd_wdata), .ready(sd_ready), .rdata(sd_rdata),
      .dma_sel(1'b0), .dma_addr(23'd0), .dma_wstrb(4'h0), .dma_wdata(32'd0),
      .dma_ready(), .dma_rdata(),
      .SDRAM_DQ(IO_sdram_dq), .SDRAM_A(O_sdram_addr), .SDRAM_BA(O_sdram_ba),
      .SDRAM_nCS(O_sdram_cs_n), .SDRAM_nWE(O_sdram_wen_n), .SDRAM_nRAS(O_sdram_ras_n),
      .SDRAM_nCAS(O_sdram_cas_n), .SDRAM_CLK(O_sdram_clk), .SDRAM_CKE(O_sdram_cke),
      .SDRAM_DQM(O_sdram_dqm));
  assign sd_ack = sd_ready;
`else
  // No SDRAM in this build: the compiler never emits SDRAM addresses then.
  reg sd_ack_r;
  always @(posedge clk) sd_ack_r <= sd_req && !sd_ack_r;
  assign sd_ack   = sd_ack_r;
  assign sd_rdata = 32'd0;
`endif

  // ---------------------------------------------------------------- LEDs
  assign leds = ~{~pll_locked, ~spi_cs_n, mute, overrun, heartbeat[14], booted};

endmodule
