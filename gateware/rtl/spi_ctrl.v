`timescale 1ns / 1ps
//
// SPI slave (mode 0, MSB first) implementing docs/protocol.md: parameter
// access to the DSP core's block RAM, the parameter descriptor, status,
// control, and loading a new program. Framing follows NanoTangAI's
// spi_slave.v: the SPI pins are synchronized into `clk` (so SCLK must stay
// well below clk/8 -- 3 MHz is a safe maximum at 48 MHz) and every reply
// byte lags the request byte that produced it by one transfer.
//
// Block RAM accesses are handed to top_tangnano20k.v through a small write
// FIFO (wr_*) and a single read request (rd_*), because the core owns the
// RAM while it computes a sample; the top level services them between
// samples. Program words (prog_*) go straight to the core's program memory
// -- the host stops the core first (CONTROL stop bit).
//
// The cfg_* registers describe the loaded program. They reset to the
// program the bitstream was built with (parameters) and SET_CONFIG
// replaces them.
//
module spi_ctrl #(
    parameter integer FAST_AW  = 14,
    parameter integer PROG_AW  = 11,
    parameter integer DESC_AW  = 10,
    parameter         DESC_HEX = "desc.hex",
    parameter [15:0]  DESC_LEN = 1,
    parameter [15:0]  DSP_ENTRY = 0,
    parameter [15:0]  BOOT_ENTRY = 0,
    parameter [7:0]   N_IN     = 0,
    parameter [7:0]   N_OUT    = 1,
    parameter [7:0]   N_PARAMS = 0,
    parameter [31:0]  SAMPLE_RATE = 48000,
    parameter [31:0]  BCLK_INC = 0,
    parameter [31:0]  CLK_HZ   = 54000000,
    parameter         USE_SDRAM = 0,
    parameter [7:0]   HW_IN    = 2,   // channels this bitstream has (INFO)
    parameter [7:0]   HW_OUT   = 2,
    parameter         HAS_TDM  = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        sclk_pin,
    input  wire        mosi_pin,
    output wire        miso_pin,
    input  wire        cs_n_pin,
    output wire        miso_int,     // MISO before the tri-state (UART bridges)

    // block RAM write FIFO (towards the core's host port)
    output wire        wr_valid,
    output wire [FAST_AW-1:0] wr_addr,
    output wire [31:0] wr_data,
    input  wire        wr_pop,

    // block RAM read request
    output reg         rd_req,
    output reg  [FAST_AW-1:0] rd_addr,
    input  wire        rd_done,
    input  wire [31:0] rd_data,

    // program memory writes
    output reg         prog_we,
    output reg  [PROG_AW-1:0] prog_waddr,
    output reg  [39:0] prog_wdata,

    // configuration of the loaded program
    output reg  [15:0] cfg_dsp_entry,
    output reg  [15:0] cfg_boot_entry,
    output reg  [7:0]  cfg_n_in,
    output reg  [7:0]  cfg_n_out,
    output reg  [7:0]  cfg_n_params,
    output reg  [31:0] cfg_sample_rate,
    output reg  [31:0] cfg_bclk_inc,
    output reg  [15:0] cfg_desc_len,

    // status / control
    input  wire        booted,
    input  wire        overrun,     // a sample wasn't finished in time
    input  wire [31:0] cycles_max,  // worst cycles/sample since last INFO
    input  wire        tdm_active,  // an external TDM master provides the sample clock
    input  wire [103:0] core_debug, // live core state (DEBUG command)
    output reg         stats_clear, // pulse: INFO was read
    output reg         mute,
    output reg         amp_en,
    output reg         stop,        // keep the core stopped (while loading)
    output reg         reboot       // pulse
);

  localparam OP_PING = 8'h01, OP_INFO = 8'h02, OP_DESC = 8'h03, OP_DEBUG = 8'h04,
             OP_WRITE = 8'h10, OP_READ = 8'h11, OP_CONTROL = 8'h20,
             OP_WPROG = 8'h30, OP_WDESC = 8'h31, OP_CONFIG = 8'h32;
  localparam [7:0] VERSION = 8'h04, READY = 8'hA5;  // 3: fused instructions, 4: INFO channels

  // ---------------------------------------------------------------- pins
  reg [2:0] sclk_s, cs_s;
  reg [1:0] mosi_s;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sclk_s <= 3'b000;
      cs_s   <= 3'b111;
      mosi_s <= 2'b00;
    end else begin
      sclk_s <= {sclk_s[1:0], sclk_pin};
      cs_s   <= {cs_s[1:0], cs_n_pin};
      mosi_s <= {mosi_s[0], mosi_pin};
    end
  end
  wire sclk_rise = (sclk_s[2:1] == 2'b01);
  wire sclk_fall = (sclk_s[2:1] == 2'b10);
  wire active    = ~cs_s[2];

  reg [2:0] bit_cnt;
  reg [7:0] rx;
  reg       byte_done;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bit_cnt   <= 3'd0;
      rx        <= 8'd0;
      byte_done <= 1'b0;
    end else begin
      byte_done <= 1'b0;
      if (!active) begin
        bit_cnt <= 3'd0;
      end else if (sclk_rise) begin
        rx <= {rx[6:0], mosi_s[1]};
        bit_cnt <= bit_cnt + 3'd1;
        if (bit_cnt == 3'd7) byte_done <= 1'b1;
      end
    end
  end
  wire [7:0] rx_byte = rx;  // valid in the byte_done cycle

  reg [7:0] tx, next_tx;
  reg       reload;
  reg       miso_o;
  assign miso_pin = active ? miso_o : 1'bz;
  assign miso_int = miso_o;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx     <= 8'hFF;
      miso_o <= 1'b1;
      reload <= 1'b0;
    end else begin
      if (byte_done) reload <= 1'b1;
      if (!active) begin
        tx     <= 8'hFF;
        miso_o <= 1'b1;
        reload <= 1'b0;
      end else if (sclk_fall) begin
        if (bit_cnt == 3'd0 && reload) begin
          tx     <= {next_tx[6:0], 1'b0};
          miso_o <= next_tx[7];
          reload <= 1'b0;
        end else begin
          tx     <= {tx[6:0], 1'b0};
          miso_o <= tx[7];
        end
      end
    end
  end

  // ---------------------------------------------------------------- descriptor RAM
  reg [7:0]         desc [0:(1 << DESC_AW)-1];
  initial $readmemh(DESC_HEX, desc);
  reg [DESC_AW-1:0] desc_addr;
  reg [7:0]         desc_q;
  reg               desc_we;
  reg [7:0]         desc_wdata;
  always @(posedge clk) begin
    if (desc_we) desc[desc_addr] <= desc_wdata;
    desc_q <= desc[desc_addr];
  end

  // ---------------------------------------------------------------- write FIFO
  reg [FAST_AW+31:0] fifo [0:15];
  reg [3:0]          f_wp, f_rp;
  reg [4:0]          f_cnt;
  reg                f_push;
  reg [FAST_AW+31:0] f_in;
  assign wr_valid = (f_cnt != 5'd0);
  assign {wr_addr, wr_data} = fifo[f_rp];
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      f_wp  <= 4'd0;
      f_rp  <= 4'd0;
      f_cnt <= 5'd0;
    end else begin
      if (f_push && f_cnt != 5'd16) begin
        fifo[f_wp] <= f_in;
        f_wp <= f_wp + 4'd1;
      end
      if (wr_pop && wr_valid) f_rp <= f_rp + 4'd1;
      f_cnt <= f_cnt + ((f_push && f_cnt != 5'd16) ? 5'd1 : 5'd0)
                     - ((wr_pop && wr_valid) ? 5'd1 : 5'd0);
    end
  end

  // ---------------------------------------------------------------- protocol
  localparam P_OP = 3'd0, P_REPLY = 3'd1, P_ARGS = 3'd2, P_DESC = 3'd3,
             P_READ = 3'd4, P_IGNORE = 3'd5, P_STREAM = 3'd6;
  reg [2:0]   pstate;
  reg [7:0]   op;
  reg [4:0]   idx;
  reg [135:0] args;       // up to 17 argument bytes, newest in [135:128]
  reg [7:0]   reply [0:21];
  reg [4:0]   reply_len;
  reg         rd_have;
  reg [31:0]  rd_val;
  reg [2:0]   rd_sent;
  reg [2:0]   wb;         // byte within a program word
  reg [31:0]  wlo;        // program word low bytes collected so far

  wire [7:0]   flags = {1'b0, HAS_TDM ? 1'b1 : 1'b0, tdm_active, stop, USE_SDRAM ? 1'b1 : 1'b0, mute, overrun, booted};
  // argument bytes shift in from the top: after n bytes, the first one is
  // at bit 136-8n and the newest at [135:128]
  wire [135:0] a_next = {rx_byte, args[135:8]};

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pstate          <= P_OP;
      next_tx         <= 8'hFF;
      f_push          <= 1'b0;
      rd_req          <= 1'b0;
      rd_have         <= 1'b0;
      stats_clear     <= 1'b0;
      mute            <= 1'b0;
      amp_en          <= 1'b1;
      stop            <= 1'b0;
      reboot          <= 1'b0;
      prog_we         <= 1'b0;
      desc_we         <= 1'b0;
      desc_addr       <= {DESC_AW{1'b0}};
      cfg_dsp_entry   <= DSP_ENTRY;
      cfg_boot_entry  <= BOOT_ENTRY;
      cfg_n_in        <= N_IN;
      cfg_n_out       <= N_OUT;
      cfg_n_params    <= N_PARAMS;
      cfg_sample_rate <= SAMPLE_RATE;
      cfg_bclk_inc    <= BCLK_INC;
      cfg_desc_len    <= DESC_LEN;
    end else begin
      f_push      <= 1'b0;
      stats_clear <= 1'b0;
      reboot      <= 1'b0;
      prog_we     <= 1'b0;
      if (desc_we) desc_addr <= desc_addr + 1'b1;
      desc_we     <= 1'b0;
      if (prog_we) prog_waddr <= prog_waddr + 1'b1;
      if (rd_done) begin
        rd_req  <= 1'b0;
        rd_have <= 1'b1;
        rd_val  <= rd_data;
      end

      if (!active) begin
        pstate <= P_OP;
      end else if (byte_done) begin
        case (pstate)
          P_OP: begin
            op  <= rx_byte;
            idx <= 5'd0;
            next_tx <= 8'h00;
            case (rx_byte)
              OP_PING: begin
                reply[0] <= "A"; reply[1] <= "U"; reply[2] <= "S"; reply[3] <= VERSION;
                reply_len <= 5'd4;
                next_tx <= "F";
                pstate <= P_REPLY;
              end
              OP_INFO: begin
                reply[0]  <= cfg_n_out;                reply[1]  <= cfg_n_params;
                reply[2]  <= flags;
                reply[3]  <= cfg_sample_rate[7:0];     reply[4]  <= cfg_sample_rate[15:8];
                reply[5]  <= cfg_sample_rate[23:16];   reply[6]  <= cfg_sample_rate[31:24];
                reply[7]  <= cfg_desc_len[7:0];        reply[8]  <= cfg_desc_len[15:8];
                reply[9]  <= cycles_max[7:0];          reply[10] <= cycles_max[15:8];
                reply[11] <= cycles_max[23:16];        reply[12] <= cycles_max[31:24];
                reply[13] <= CLK_HZ[7:0];              reply[14] <= CLK_HZ[15:8];
                reply[15] <= CLK_HZ[23:16];            reply[16] <= CLK_HZ[31:24];
                reply[17] <= PROG_AW;                  reply[18] <= FAST_AW;
                reply[19] <= DESC_AW;
                reply[20] <= HW_IN;                    reply[21] <= HW_OUT;
                reply_len <= 5'd22;
                next_tx <= cfg_n_in;
                stats_clear <= 1'b1;
                pstate <= P_REPLY;
              end
              OP_DEBUG: begin  // live core state: state, npc u16, opcode, sp, T u32, cycles u32
                reply[0]  <= core_debug[15:8];  reply[1]  <= core_debug[23:16];
                reply[2]  <= core_debug[31:24]; reply[3]  <= core_debug[39:32];
                reply[4]  <= core_debug[47:40]; reply[5]  <= core_debug[55:48];
                reply[6]  <= core_debug[63:56]; reply[7]  <= core_debug[71:64];
                reply[8]  <= core_debug[79:72]; reply[9]  <= core_debug[87:80];
                reply[10] <= core_debug[95:88]; reply[11] <= core_debug[103:96];
                reply_len <= 5'd12;
                next_tx <= core_debug[7:0];
                pstate <= P_REPLY;
              end
              OP_DESC, OP_WRITE, OP_READ, OP_CONTROL, OP_WPROG, OP_WDESC, OP_CONFIG:
                pstate <= P_ARGS;
              default: pstate <= P_IGNORE;
            endcase
          end

          P_REPLY: begin
            next_tx <= (idx < reply_len && idx <= 5'd21) ? reply[idx] : 8'h00;
            idx <= idx + 5'd1;
          end

          P_ARGS: begin
            args <= a_next;
            idx  <= idx + 5'd1;
            next_tx <= 8'h00;
            case (op)
              OP_DESC: if (idx == 5'd1) begin
                // offset complete; one dummy reply byte follows
                desc_addr <= {rx_byte, args[135:128]};
                pstate    <= P_DESC;
              end
              OP_CONTROL: begin
                mute   <= rx_byte[0];
                amp_en <= rx_byte[1];
                reboot <= rx_byte[2];
                stop   <= rx_byte[3];
                pstate <= P_IGNORE;
              end
              OP_READ: if (idx == 5'd1) begin
                rd_addr <= {rx_byte, args[135:128]};
                rd_req  <= 1'b1;
                rd_have <= 1'b0;
                rd_sent <= 3'd0;
                pstate  <= P_READ;
              end
              OP_WPROG, OP_WDESC: if (idx == 5'd1) begin
                // start address complete; data bytes follow
                if (op == OP_WPROG) prog_waddr <= {rx_byte, args[135:128]};
                else desc_addr <= {rx_byte, args[135:128]};
                wb     <= 3'd0;
                pstate <= P_STREAM;
              end
              OP_CONFIG: if (idx == 5'd16) begin
                // dsp_entry u16, boot_entry u16, n_in, n_out, n_params,
                // sample_rate u32, bclk_inc u32, desc_len u16 (17 bytes)
                cfg_dsp_entry   <= a_next[15:0];
                cfg_boot_entry  <= a_next[31:16];
                cfg_n_in        <= a_next[39:32];
                cfg_n_out       <= a_next[47:40];
                cfg_n_params    <= a_next[55:48];
                cfg_sample_rate <= a_next[87:56];
                cfg_bclk_inc    <= a_next[119:88];
                cfg_desc_len    <= a_next[135:120];
                pstate          <= P_IGNORE;
              end
              default: if (idx == 5'd5) begin  // OP_WRITE: addr u16, value u32
                // a_next[135:88] = a0 a1 v0 v1 v2 v3 (oldest lowest)
                f_in   <= {a_next[103:88], a_next[135:104]};
                f_push <= 1'b1;
                pstate <= P_IGNORE;
              end
            endcase
          end

          P_STREAM: begin
            if (op == OP_WDESC) begin
              desc_wdata <= rx_byte;
              desc_we    <= 1'b1;
            end else begin
              // 5 bytes per instruction, little endian
              wb <= (wb == 3'd4) ? 3'd0 : wb + 3'd1;
              case (wb)
                3'd0: wlo[7:0]   <= rx_byte;
                3'd1: wlo[15:8]  <= rx_byte;
                3'd2: wlo[23:16] <= rx_byte;
                3'd3: wlo[31:24] <= rx_byte;
                default: begin
                  prog_wdata <= {rx_byte, wlo};
                  prog_we    <= 1'b1;
                end
              endcase
            end
          end

          P_DESC: begin
            next_tx   <= desc_q;
            desc_addr <= desc_addr + 1'b1;
          end

          P_READ: begin
            // 0x00 until the value is there, then READY and 4 bytes LE
            if (!rd_have && !rd_done) begin
              next_tx <= 8'h00;
            end else begin
              rd_sent <= rd_sent + 3'd1;
              case (rd_sent)
                3'd0: next_tx <= READY;
                3'd1: next_tx <= rd_val[7:0];
                3'd2: next_tx <= rd_val[15:8];
                3'd3: next_tx <= rd_val[23:16];
                3'd4: next_tx <= rd_val[31:24];
                default: next_tx <= 8'h00;
              endcase
            end
          end

          default: next_tx <= 8'h00;  // P_IGNORE
        endcase
      end
    end
  end

endmodule
