`timescale 1ns / 1ps
//
// UART command port: carries the SPI protocol (docs/protocol.md) over a
// serial line, so the FPGA can be controlled from a PC through the board's
// USB-UART bridge, or from an MCU's UART.
//
// Host -> FPGA frame:  0x7E, length u16 LE, then `length` bytes -- exactly
//                      the bytes an SPI host would send in one transaction
//                      (opcode, arguments, dummy bytes for replies).
//                      Bit 15 of the length: no reply wanted.
// FPGA -> host:        the `length` bytes an SPI host would have received
//                      (same one-byte lag), or a single 0x7E when bit 15
//                      was set.
//
// The bridge first receives the whole frame into its buffer, then replays
// it as an SPI transaction on the virtual v_* pins into spi_ctrl.v (each
// reply byte replaces the sent byte in place), and only then transmits.
// So traffic is strictly half-duplex: the board's BL616 USB bridge loses
// FPGA -> PC bytes while the PC is sending (see docs/protocol.md). A gap of
// more than ~10 ms inside a frame resets the receiver.
//
module uart_bridge #(
    parameter integer CLK_HZ = 54000000,
    parameter integer BAUD   = 115200,
    parameter integer BUF_AW = 9          // 512-byte frames
) (
    input  wire clk,
    input  wire rst_n,

    input  wire rx,
    output reg  tx,

    // virtual SPI master (mode 0) towards spi_ctrl.v
    input  wire bus_free,                 // no other master is active
    output reg  v_cs_n,
    output reg  v_sclk,
    output reg  v_mosi,
    input  wire v_miso,
    output wire active                    // owns the virtual SPI bus
);

  localparam integer DIV     = CLK_HZ / BAUD;
  localparam integer TIMEOUT = CLK_HZ / 100;  // 10 ms
  localparam integer HALF    = 8;              // SCLK half period: 27/16 = 1.7 MHz

  // ---------------------------------------------------------------- receiver
  reg [2:0]  rx_s = 3'b111;
  always @(posedge clk) rx_s <= {rx_s[1:0], rx};
  wire       rxd = rx_s[2];

  reg [15:0] rx_cnt;
  reg [3:0]  rx_bit;
  reg [7:0]  rx_sh;
  reg        rx_busy, rx_valid;
  reg [7:0]  rx_byte;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_busy  <= 1'b0;
      rx_valid <= 1'b0;
    end else begin
      rx_valid <= 1'b0;
      if (!rx_busy) begin
        if (!rxd) begin  // start bit: sample in the middle of each bit
          rx_busy <= 1'b1;
          rx_cnt  <= DIV / 2;
          rx_bit  <= 4'd0;
        end
      end else if (rx_cnt != 0) begin
        rx_cnt <= rx_cnt - 16'd1;
      end else begin
        rx_cnt <= DIV - 1;
        rx_bit <= rx_bit + 4'd1;
        if (rx_bit == 4'd0) begin
          if (rxd) rx_busy <= 1'b0;          // glitch, not a start bit
        end else if (rx_bit <= 4'd8) begin
          rx_sh <= {rxd, rx_sh[7:1]};         // LSB first
        end else begin                        // stop bit
          rx_busy  <= 1'b0;
          rx_valid <= rxd;
          rx_byte  <= rx_sh;
        end
      end
    end
  end

  // ---------------------------------------------------------------- transmitter
  reg [15:0] tx_cnt;
  reg [3:0]  tx_bit;
  reg [9:0]  tx_sh;
  reg        tx_busy, tx_start;
  reg [7:0]  tx_byte;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx      <= 1'b1;
      tx_busy <= 1'b0;
    end else if (!tx_busy) begin
      if (tx_start) begin
        tx_sh   <= {1'b1, tx_byte, 1'b0};    // stop, data (LSB first), start
        tx_busy <= 1'b1;
        tx_bit  <= 4'd0;
        tx_cnt  <= 16'd0;
      end
    end else if (tx_cnt != 0) begin
      tx_cnt <= tx_cnt - 16'd1;
    end else begin
      tx_cnt <= DIV - 1;
      if (tx_bit == 4'd10) begin
        tx_busy <= 1'b0;
      end else begin
        tx     <= tx_sh[0];
        tx_sh  <= {1'b1, tx_sh[9:1]};
        tx_bit <= tx_bit + 4'd1;
      end
    end
  end

  // ---------------------------------------------------------------- frame buffer
  reg [7:0]        buffer [0:(1 << BUF_AW)-1];
  reg [BUF_AW-1:0] b_addr;
  reg              b_we;
  reg [7:0]        b_wdata, b_q;
  always @(posedge clk) begin
    if (b_we) buffer[b_addr] <= b_wdata;
    b_q <= buffer[b_addr];
  end

  // ---------------------------------------------------------------- control
  localparam F_SYNC = 4'd0, F_LEN0 = 4'd1, F_LEN1 = 4'd2, F_RECV = 4'd3, F_BUS = 4'd4,
             F_LOAD = 4'd5, F_BITS = 4'd6, F_STORE = 4'd7, F_SEND = 4'd8, F_SENDW = 4'd9,
             F_ACK = 4'd10, F_DONE = 4'd11;
  reg [3:0]        f;
  reg [15:0]       len;      // bit 15: no reply
  reg [BUF_AW:0]   idx;
  reg [23:0]       idle;
  reg [7:0]        sh, rv;
  reg [3:0]        nbit;
  reg [4:0]        tick;
  reg              hi;       // second half of a bit (SCLK high)
  wire [BUF_AW:0]  n = len[BUF_AW:0];
  assign active = (f == F_LOAD) || (f == F_BITS) || (f == F_STORE) ||
                  ((f == F_BUS) && bus_free);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      f        <= F_SYNC;
      v_cs_n   <= 1'b1;
      v_sclk   <= 1'b0;
      v_mosi   <= 1'b0;
      b_we     <= 1'b0;
      tx_start <= 1'b0;
      idle     <= 24'd0;
    end else begin
      b_we     <= 1'b0;
      tx_start <= 1'b0;
      // receive timeout inside a frame
      if (rx_valid || f == F_SYNC || f >= F_BUS) idle <= 24'd0;
      else if (idle != TIMEOUT) idle <= idle + 24'd1;
      else f <= F_SYNC;

      case (f)
        F_SYNC: if (rx_valid && rx_byte == 8'h7E) f <= F_LEN0;
        F_LEN0: if (rx_valid) begin len[7:0] <= rx_byte; f <= F_LEN1; end
        F_LEN1: if (rx_valid) begin
          len[15:8] <= rx_byte;
          idx       <= 0;
          // empty or oversized frames are ignored
          f <= ({rx_byte[6:0], len[7:0]} == 15'd0 ||
                {rx_byte[6:0], len[7:0]} > (1 << BUF_AW)) ? F_SYNC : F_RECV;
        end
        F_RECV: if (rx_valid) begin
          b_addr  <= idx[BUF_AW-1:0];
          b_wdata <= rx_byte;
          b_we    <= 1'b1;
          idx     <= idx + 1'b1;
          if (idx + 1'b1 == n) f <= F_BUS;
        end

        // ---- replay as an SPI transaction (mode 0, MSB first)
        F_BUS: if (bus_free) begin
          v_cs_n <= 1'b0;
          idx    <= 0;
          b_addr <= 0;
          tick   <= 5'd0;
          f      <= F_LOAD;
        end
        F_LOAD: begin  // read buffer[idx] (after a pending write), then the first bit
          tick <= tick + 5'd1;
          if (tick == 5'd0) b_addr <= idx[BUF_AW-1:0];
          if (tick == HALF) begin
            sh     <= b_q;
            v_mosi <= b_q[7];
            nbit   <= 4'd0;
            hi     <= 1'b0;
            tick   <= 5'd0;
            f      <= F_BITS;
          end
        end
        F_BITS: begin
          tick <= tick + 5'd1;
          if (tick == HALF - 1) begin
            tick <= 5'd0;
            if (!hi) begin
              v_sclk <= 1'b1;                 // slave samples MOSI
              rv     <= {rv[6:0], v_miso};    // master samples MISO
              hi     <= 1'b1;
            end else begin
              v_sclk <= 1'b0;
              hi     <= 1'b0;
              nbit   <= nbit + 4'd1;
              if (nbit == 4'd7) begin
                f <= F_STORE;
              end else begin
                sh     <= {sh[6:0], 1'b0};
                v_mosi <= sh[6];
              end
            end
          end
        end
        F_STORE: begin  // reply byte replaces the sent one; next byte
          b_addr  <= idx[BUF_AW-1:0];
          b_wdata <= rv;
          b_we    <= 1'b1;
          idx     <= idx + 1'b1;
          tick    <= 5'd0;
          if (idx + 1'b1 == n) begin
            v_cs_n <= 1'b1;
            idx    <= 0;
            f      <= len[15] ? F_ACK : F_SEND;
          end else begin
            f <= F_LOAD;
          end
        end

        // ---- reply
        F_SEND: begin  // read buffer[idx] (after the last write)
          b_addr <= idx[BUF_AW-1:0];
          f      <= F_SENDW;
          tick   <= 5'd0;
        end
        F_SENDW: begin
          tick <= tick + 5'd1;
          if (tick == 5'd2 && !tx_busy && !tx_start) begin
            tx_byte  <= b_q;
            tx_start <= 1'b1;
            idx      <= idx + 1'b1;
            f        <= (idx + 1'b1 == n) ? F_DONE : F_SEND;
          end else if (tick == 5'd2) begin
            tick <= 5'd2;  // wait for the transmitter
          end
        end
        F_ACK: if (!tx_busy) begin
          tx_byte  <= 8'h7E;
          tx_start <= 1'b1;
          f        <= F_DONE;
        end
        F_DONE: if (!tx_busy && !tx_start) f <= F_SYNC;
        default: f <= F_SYNC;
      endcase
    end
  end

endmodule
