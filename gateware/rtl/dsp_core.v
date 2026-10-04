`timescale 1ns / 1ps
//
// TangNanoFaust DSP core: a 32-bit stack machine that runs programs
// compiled from Faust by faust2tang (instruction set: compiler/IsaOpcodes.h
// / isa_opcodes.vh). The simulator in src/TangNanoFaust/compiler/Isa.h is
// the bit-exact reference model, including the cycle count (`cycles`).
//
// Execution: `run` starts the program at `entry`; the core runs until HALT,
// then pulses `halted`. The top level does that once per audio sample
// (and once at boot). While the core is idle, the host port gives the SPI
// parameter interface access to block RAM.
//
// Timing: the next PC is computed combinationally and fed straight into
// the synchronous program ROM, so simple instructions -- including taken
// jumps -- take one cycle. Loads from block RAM take two; the FPU, the
// integer multiplier/divider and SDRAM accesses wait for their units.
//
// Memory map (word addresses): [0, FAST_WORDS) block RAM, from 0x800000
// SDRAM through the sd_* request port (tie sd_ack high-after-req in builds
// without SDRAM -- the compiler never emits SDRAM addresses for those).
//
module dsp_core #(
    parameter integer PROG_AW    = 11,
    parameter integer FAST_AW    = 14,
    parameter integer FAST_WORDS = 16384,
    parameter integer STACK_AW   = 5,
    parameter integer N_IN       = 2,
    parameter integer N_OUT      = 2,
    parameter         PROG_HEX   = "prog.hex"
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 run,
    input  wire [PROG_AW-1:0]   entry,
    output wire                 busy,
    output wire                 idle,       // may take `run` or a host request
    output reg                  halted,
    output reg  [31:0]          cycles,     // cycles used by the last run
    output wire [103:0]         debug,      // live state for the DEBUG command

    input  wire [N_IN*32-1:0]   in_bus,
    output wire [N_OUT*32-1:0]  out_bus,

    // block RAM access while idle (SPI parameters)
    input  wire                 host_req,
    input  wire                 host_we,
    input  wire [FAST_AW-1:0]   host_addr,
    input  wire [31:0]          host_wdata,
    output reg  [31:0]          host_rdata,
    output reg                  host_ack,

    // program memory write port (loading a program over SPI; only while
    // the top level keeps the core stopped)
    input  wire                 prog_we,
    input  wire [PROG_AW-1:0]   prog_waddr,
    input  wire [39:0]          prog_wdata,

    // SDRAM request port (word addresses relative to 0x800000)
    output reg                  sd_req,
    output reg                  sd_we,
    output reg  [20:0]          sd_addr,
    output reg  [31:0]          sd_wdata,
    input  wire [31:0]          sd_rdata,
    input  wire                 sd_ack
);

`include "isa_opcodes.vh"

  localparam S_IDLE = 3'd0, S_EXEC = 3'd1, S_MEMRD = 3'd2, S_SD = 3'd3,
             S_FPU = 3'd4, S_MUL = 3'd5, S_DIV = 3'd6, S_HOST = 3'd7;

  (* fsm_encoding = "none" *) reg [2:0] state;
  assign busy = (state != S_IDLE) && (state != S_HOST);
  assign idle = (state == S_IDLE);

  // ------------------------------------------------------------ program memory
  // Block RAM, initialized with the program the bitstream was built with
  // and rewritable over SPI (TangNanoFaust::load()). Single-ported: it is
  // only written while the top level keeps the core stopped, so writes and
  // instruction fetches share one address.
  reg [39:0] prog [0:(1 << PROG_AW)-1];
  initial $readmemh(PROG_HEX, prog);

  // Prefetch: the memory's output register `nir` holds the next instruction
  // while `ir` executes; `rom_en` (an instruction completes) moves nir into
  // ir and fetches the one after it, or the jump target. A taken jump, and
  // the start of a run, leave a bubble: ir is invalid and decodes as NOP.
  reg  [39:0]        nir;         // next instruction (block RAM output)
  reg  [PROG_AW-1:0] npc;         // its address
  reg  [39:0]        ir;          // executing instruction
  reg                ir_valid;
  reg                rom_en;
  reg  [PROG_AW-1:0] rom_addr;
  wire [PROG_AW-1:0] prog_addr = prog_we ? prog_waddr : rom_addr;
  always @(posedge clk) begin
    if (prog_we) prog[prog_addr] <= prog_wdata;
    if (rom_en) nir <= prog[prog_addr];
  end

  wire [7:0]  opc = ir_valid ? ir[39:32] : OP_NOP;
  wire [31:0] arg = ir[31:0];

  // ------------------------------------------------------------ data stack
  reg [31:0]         stk [0:(1 << STACK_AW)-1];
  reg [STACK_AW-1:0] sp;
  reg [31:0]         T;
  wire [31:0]        N  = stk[sp];
  wire [31:0]        NN = stk[sp - 1'b1];
  // The stack is written in the same cycle as sp moves: pushes store the
  // old T at sp+1, SWAP stores it at sp.
  reg                stk_we;
  wire [STACK_AW-1:0] stk_waddr = (opc == OP_SWAP) ? sp : sp + 1'b1;
  always @(posedge clk)
    if (stk_we) stk[stk_waddr] <= T;

  // return stack
  reg [PROG_AW-1:0] rstk [0:7];
  reg [2:0]         rsp;

  // ------------------------------------------------------------ block RAM
  reg  [31:0]        mem [0:FAST_WORDS-1];
  reg                mem_we;
  reg  [FAST_AW-1:0] mem_addr;
  reg  [31:0]        mem_wdata;
  reg  [31:0]        mem_q;
  always @(posedge clk) begin
    if (mem_we) mem[mem_addr] <= mem_wdata;
    mem_q <= mem[mem_addr];
  end

  // ------------------------------------------------------------ I/O
  reg [31:0] outs [0:N_OUT-1];
  genvar gi;
  generate
    for (gi = 0; gi < N_OUT; gi = gi + 1) begin : g_out
      assign out_bus[gi*32 +: 32] = outs[gi];
    end
  endgenerate
  wire [31:0] in_sel = in_bus[arg[2:0]*32 +: 32];

  // ------------------------------------------------------------ fused operand forms
  // opcode + 0x40: v1 = arg (immediate); opcode + 0x80: v1 = heap[arg]. v2 is
  // T and the result replaces it (see IsaOpcodes.h). Binary operators read
  // their operands as A (v1) and B (v2): normally T and N.
  function fusable(input [7:0] c);
    fusable = (c >= 8'h20 && c <= 8'h32) || (c >= 8'h40 && c <= 8'h4B);
  endfunction
  wire        fused_imm = fusable(opc - 8'h40);
  wire        fused_mem = fusable(opc - 8'h80);
  wire        fused     = fused_imm | fused_mem;
  wire [7:0]  bop       = fused_imm ? opc - 8'h40 : fused_mem ? opc - 8'h80 : opc;
  reg         pf;  // mem_q holds the operand of ir (read while it was in nir)
  wire [31:0] xval      = (state == S_SD) ? sd_rdata :
                          (state == S_MEMRD || (fused_mem && pf)) ? mem_q : arg;
  wire [31:0] A         = fused ? xval : T;
  wire [31:0] B         = fused ? T : N;

  // ------------------------------------------------------------ integer ALU
  wire signed [31:0] sA = A, sB = B;
  reg  [31:0] alu;
  reg         is_alu;
  always @(*) begin
    is_alu = 1'b1;
    alu    = 32'd0;
    case (bop)
      OP_ADD: alu = A + B;
      OP_SUB: alu = A - B;
      OP_SHL: alu = A << B[4:0];
      OP_ASR: alu = sA >>> B[4:0];
      OP_LSR: alu = A >> B[4:0];
      OP_GT:  alu = {31'd0, sA > sB};
      OP_LT:  alu = {31'd0, sA < sB};
      OP_GE:  alu = {31'd0, sA >= sB};
      OP_LE:  alu = {31'd0, sA <= sB};
      OP_EQ:  alu = {31'd0, A == B};
      OP_NE:  alu = {31'd0, A != B};
      OP_AND: alu = A & B;
      OP_OR:  alu = A | B;
      OP_XOR: alu = A ^ B;
      OP_MIN: alu = (sB < sA) ? B : A;
      OP_MAX: alu = (sA < sB) ? B : A;
      default: is_alu = 1'b0;
    endcase
  end

  // ------------------------------------------------------------ FP compare & co.
  // DAZ: zero exponent field = zero. NaN compares unordered.
  function [31:0] canon(input [31:0] x);
    canon = (x[30:23] == 8'd0) ? {x[31], 31'd0} :
            (x[30:23] == 8'hFF && x[22:0] != 0) ? 32'h7FC00000 : x;
  endfunction

  wire t_zero = (A[30:23] == 8'd0), n_zero = (B[30:23] == 8'd0);
  wire t_nan  = (A[30:23] == 8'hFF) && (A[22:0] != 0);
  wire n_nan  = (B[30:23] == 8'hFF) && (B[22:0] != 0);
  wire unord  = t_nan | n_nan;
  wire [30:0] t_mag = t_zero ? 31'd0 : A[30:0];
  wire [30:0] n_mag = n_zero ? 31'd0 : B[30:0];
  wire t_neg = A[31] && !t_zero, n_neg = B[31] && !n_zero;
  wire f_eq  = !unord && (t_mag == n_mag) && (t_neg == n_neg || t_mag == 0);
  // A < B
  wire f_lt  = !unord && !f_eq &&
               ((t_neg && !n_neg) ||
                (!t_neg && !n_neg && t_mag < n_mag) ||
                (t_neg && n_neg && t_mag > n_mag));
  wire f_gt  = !unord && !f_eq && !f_lt;

  reg [31:0] fsimple;
  reg        is_fsimple;
  always @(*) begin
    is_fsimple = 1'b1;
    fsimple    = 32'd0;
    case (bop)
      OP_FGT:  fsimple = {31'd0, f_gt};
      OP_FLT:  fsimple = {31'd0, f_lt};
      OP_FGE:  fsimple = {31'd0, f_gt | f_eq};
      OP_FLE:  fsimple = {31'd0, f_lt | f_eq};
      OP_FEQ:  fsimple = {31'd0, f_eq};
      OP_FNE:  fsimple = {31'd0, !f_eq};
      OP_FMIN: fsimple = f_gt ? canon(B) : canon(A);  // (v2 < v1) ? v2 : v1
      OP_FMAX: fsimple = f_lt ? canon(B) : canon(A);  // (v1 < v2) ? v2 : v1
      default: is_fsimple = 1'b0;
    endcase
  end

  // ------------------------------------------------------------ FPU
  reg  [2:0]  fpu_op;
  reg         is_fpu, fpu_unary;
  always @(*) begin
    is_fpu = 1'b1;
    fpu_unary = 1'b0;
    fpu_op = 3'd0;
    case (bop)
      OP_FADD:  fpu_op = 3'd0;
      OP_FSUB:  fpu_op = 3'd1;
      OP_FMUL:  fpu_op = 3'd2;
      OP_FDIV:  fpu_op = 3'd3;
      OP_FSQRT: begin fpu_op = 3'd4; fpu_unary = 1'b1; end
      OP_I2F:   begin fpu_op = 3'd5; fpu_unary = 1'b1; end
      OP_F2I:   begin fpu_op = 3'd6; fpu_unary = 1'b1; end
      OP_FFLOOR: begin fpu_op = 3'd7; fpu_unary = 1'b1; end
      default:  is_fpu = 1'b0;
    endcase
  end
  // an operator runs when its operands are ready: in EXEC, or for a fused
  // memory operand once the read data is there
  // Watchdog for the waits on the FPU and the SDRAM: if `done` / `ack` never
  // comes (only possible through a timing error on the chip), continue with
  // whatever value is there after 255 cycles instead of hanging. Normal waits
  // are at most ~35 cycles, so cycle counts don't change.
  reg  [7:0]  wait_n;
  wire        wd_fire = (wait_n == 8'd255);
  wire        sd_go   = sd_ack || wd_fire;
  wire        do_op = (state == S_EXEC && (!fused_mem || pf)) || (state == S_MEMRD && fused_mem) ||
                      (state == S_SD && sd_go && fused_mem);
  wire        fpu_start = do_op && is_fpu;
  wire        fpu_done;
  wire        fpu_go  = fpu_done || wd_fire;
  wire [31:0] fpu_result;
  reg         op_unary, op_fused;
  fpu u_fpu (.clk(clk), .rst_n(rst_n), .start(fpu_start), .op(fpu_op),
             .a(A), .b(B), .done(fpu_done), .result(fpu_result));

  // ------------------------------------------------------------ int mul/div
  wire [31:0] mul_p;
  mul32lo u_mul (.clk(clk), .a(A), .b(B), .p(mul_p));  // valid 2 cycles after do_op
  reg         mul_ph;  // S_MUL: 0 = products being registered, 1 = sum ready

  reg  [31:0] dv_q, dv_r, dv_d;
  reg  [5:0]  dv_n;
  reg         dv_rem, dv_qneg, dv_rneg, dv_zero;
  wire [32:0] dv_try = {dv_r[31:0], dv_q[31]} - {1'b0, dv_d};

  // ------------------------------------------------------------ address decode
  wire [31:0] ld_addr  = (opc == OP_LOADX || opc == OP_STOREX) ? arg + T : arg;
  wire        is_sd    = ld_addr[23];

  // ------------------------------------------------------------ main FSM
  reg [PROG_AW-1:0] target;
  reg               taken;
  reg [31:0]        cyc;

  always @(*) begin
    case (opc)
      OP_JMP, OP_CALL, OP_RET: taken = 1'b1;
      OP_JZ:   taken = (T == 32'd0);
      OP_JNZ:  taken = (T != 32'd0);
      default: taken = 1'b0;
    endcase
    target = (opc == OP_RET) ? rstk[rsp - 1'b1] : arg[PROG_AW-1:0];
  end

  // the next instruction is a block RAM load / memory operand: read it now
  wire [7:0] nop8     = nir[39:32];
  wire       nir_load = ((nop8 == OP_LOAD) || fusable(nop8 - 8'h80)) && (nir[31:23] == 9'd0);
  reg        pf_issue;

  // operators that need their own unit after the operands are ready
  wire op_multi   = is_fpu || (bop == OP_MUL) || (bop == OP_DIV) || (bop == OP_REM);
  // multi-cycle ops leave EXEC without fetching
  wire exec_multi = ((opc == OP_LOAD) && !pf) || (opc == OP_LOADX) || (fused_mem && !pf) ||
                    (((opc == OP_STORE) || (opc == OP_STOREX) || (opc == OP_TEE)) && is_sd) ||
                    op_multi || (opc == OP_HALT);

  always @(*) begin
    stk_we = ((state == S_EXEC) &&
              (opc == OP_PUSH || opc == OP_DUP || opc == OP_OVER || opc == OP_SWAP)) ||
             ((state == S_EXEC) && (opc == OP_LOAD) && pf) ||
             ((state == S_MEMRD) && (opc == OP_LOAD)) ||
             ((state == S_SD) && sd_go && (opc == OP_LOAD));
  end

  always @(*) begin
    rom_en   = 1'b0;
    rom_addr = npc + 1'b1;
    mem_we   = 1'b0;
    mem_addr = ld_addr[FAST_AW-1:0];
    mem_wdata = (opc == OP_STOREX) ? N : T;
    case (state)
      S_IDLE: begin
        rom_en   = run;
        rom_addr = entry;
        mem_addr = host_addr;
        mem_wdata = host_wdata;
        mem_we   = host_req && host_we && !run;
      end
      S_HOST: begin
        mem_addr = host_addr;
      end
      S_EXEC: begin
        if (!exec_multi) begin
          rom_en   = 1'b1;
          if (taken) rom_addr = target;
        end
        mem_we = ((opc == OP_STORE) || (opc == OP_STOREX) || (opc == OP_TEE)) && !is_sd;
      end
      S_MEMRD: begin
        rom_en = !(fused_mem && op_multi);
      end
      S_SD: begin
        rom_en = sd_go && !(fused_mem && op_multi);
      end
      S_FPU: begin
        rom_en = fpu_go;
      end
      S_MUL: begin
        rom_en = mul_ph;
      end
      S_DIV: begin
        rom_en = (dv_n == 6'd33);
      end
      default: ;
    endcase
    // prefetch the next instruction's operand when the port is free
    pf_issue = rom_en && (state != S_IDLE) && !(state == S_EXEC && taken) && nir_load && !mem_we;
    if (pf_issue) mem_addr = nir[FAST_AW-1:0];
  end

  // Runs the binary/FPU operator `bop` on A, B (do_op). Single-cycle results
  // replace T now (popping N unless fused); the others go to their unit.
  task exec_op;
    begin
      if (is_alu || is_fsimple) begin
        T <= is_alu ? alu : fsimple;
        if (!fused) sp <= sp - 1'b1;
      end else if (is_fpu) begin
        op_unary <= fpu_unary;
        op_fused <= fused;
        state    <= S_FPU;
      end else if (bop == OP_MUL) begin
        op_fused <= fused;
        mul_ph   <= 1'b0;
        state    <= S_MUL;
      end else if (bop == OP_DIV || bop == OP_REM) begin
        op_fused <= fused;
        dv_rem   <= (bop == OP_REM);
        dv_qneg  <= A[31] ^ B[31];
        dv_rneg  <= A[31];
        dv_zero  <= (B == 32'd0);
        dv_q     <= A[31] ? (~A + 32'd1) : A;
        dv_d     <= B[31] ? (~B + 32'd1) : B;
        dv_r     <= 32'd0;
        dv_n     <= 6'd0;
        state    <= S_DIV;
      end
    end
  endtask

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state    <= S_IDLE;
      npc      <= {PROG_AW{1'b0}};
      ir_valid <= 1'b0;
      pf       <= 1'b0;
      sp       <= {STACK_AW{1'b0}};
      rsp      <= 3'd0;
      T        <= 32'd0;
      halted   <= 1'b0;
      wait_n   <= 8'd0;
      cycles   <= 32'd0;
      cyc      <= 32'd0;
      host_ack <= 1'b0;
      sd_req   <= 1'b0;
      op_unary <= 1'b0;
      op_fused <= 1'b0;
    end else begin
      halted   <= 1'b0;
      host_ack <= 1'b0;
      if (busy) cyc <= cyc + 32'd1;
      wait_n <= (state == S_SD || state == S_FPU) && !wd_fire ? wait_n + 8'd1 : 8'd0;
      pf <= pf_issue;
      if (rom_en && state != S_IDLE) begin  // advance: nir -> ir
        ir <= nir;
        if (state == S_EXEC && taken) begin
          npc      <= target;
          ir_valid <= 1'b0;  // bubble while the target is fetched
        end else begin
          npc      <= npc + 1'b1;
          ir_valid <= 1'b1;
        end
      end

      case (state)
        S_IDLE: begin
          if (run) begin
            npc      <= entry;
            ir_valid <= 1'b0;   // bubble: the entry instruction arrives in nir
            sp    <= {STACK_AW{1'b0}};
            rsp   <= 3'd0;
            cyc   <= 32'd1;
            state <= S_EXEC;
          end else if (host_req) begin
            if (host_we) host_ack <= 1'b1;
            else state <= S_HOST;
          end
        end

        S_HOST: begin  // block RAM read data is valid now
          host_rdata <= mem_q;
          host_ack   <= 1'b1;
          state      <= S_IDLE;
        end

        S_EXEC: begin
          case (opc)
            OP_NOP: ;
            OP_PUSH, OP_DUP, OP_OVER: begin
              sp        <= sp + 1'b1;
              if (opc == OP_PUSH) T <= arg;
              if (opc == OP_OVER) T <= N;
            end
            OP_LOAD, OP_LOADX: begin
              if (opc == OP_LOAD && pf) begin  // prefetched: data is in mem_q
                sp <= sp + 1'b1;
                T  <= mem_q;
              end else if (is_sd) begin
                sd_req  <= 1'b1;
                sd_we   <= 1'b0;
                sd_addr <= ld_addr[20:0];
                state   <= S_SD;
              end else begin
                state <= S_MEMRD;
              end
            end
            OP_STORE: begin
              if (is_sd) begin
                sd_req   <= 1'b1;
                sd_we    <= 1'b1;
                sd_addr  <= ld_addr[20:0];
                sd_wdata <= T;
                state    <= S_SD;
              end
              T  <= N;
              sp <= sp - 1'b1;
            end
            OP_TEE: begin
              if (is_sd) begin
                sd_req   <= 1'b1;
                sd_we    <= 1'b1;
                sd_addr  <= ld_addr[20:0];
                sd_wdata <= T;
                state    <= S_SD;
              end
            end
            OP_STOREX: begin
              if (is_sd) begin
                sd_req   <= 1'b1;
                sd_we    <= 1'b1;
                sd_addr  <= ld_addr[20:0];
                sd_wdata <= N;
                state    <= S_SD;
              end
              T  <= NN;
              sp <= sp - 2'd2;
            end
            OP_IN: T <= in_sel;
            OP_OUT: begin
              outs[arg[2:0]] <= N;
              T  <= NN;
              sp <= sp - 2'd2;
            end
            OP_DROP, OP_JZ, OP_JNZ: begin
              T  <= N;
              sp <= sp - 1'b1;
            end
            OP_SWAP: T <= N;
            OP_JMP: ;
            OP_CALL: begin
              rstk[rsp] <= npc;  // the instruction after the CALL
              rsp       <= rsp + 1'b1;
            end
            OP_RET: rsp <= rsp - 1'b1;
            OP_HALT: begin
              cycles <= cyc;
              halted <= 1'b1;
              state  <= S_IDLE;
            end
            OP_ABS: T <= T[31] ? (~T + 32'd1) : T;
            OP_FABS: T <= canon({1'b0, T[30:0]});
            OP_FNEG: T <= canon({~T[31], T[30:0]});
            default: begin
              if (fused_mem && !pf) begin  // read the operand first
                if (is_sd) begin
                  sd_req  <= 1'b1;
                  sd_we   <= 1'b0;
                  sd_addr <= ld_addr[20:0];
                  state   <= S_SD;
                end else begin
                  state <= S_MEMRD;
                end
              end else begin
                exec_op;
              end
            end
          endcase
        end

        S_MEMRD: begin
          if (fused_mem) begin
            state <= S_EXEC;
            exec_op;
          end else begin
            if (opc == OP_LOAD) sp <= sp + 1'b1;
            T     <= mem_q;
            state <= S_EXEC;
          end
        end

        S_SD: begin
          if (sd_go) begin
            sd_req <= 1'b0;
            state  <= S_EXEC;
            if (fused_mem) begin
                exec_op;
            end else begin
              if (opc == OP_LOAD) sp <= sp + 1'b1;
              if (opc == OP_LOAD || opc == OP_LOADX) T <= sd_rdata;
            end
          end
        end

        S_FPU: begin
          if (fpu_go) begin
            T  <= fpu_result;
            if (!op_unary && !op_fused) sp <= sp - 1'b1;
            state <= S_EXEC;
          end
        end

        S_MUL: begin
          if (!mul_ph) begin
            mul_ph <= 1'b1;
          end else begin
            T     <= mul_p;
            if (!op_fused) sp <= sp - 1'b1;
            state <= S_EXEC;
          end
        end

        S_DIV: begin
          // restoring division of |T| by |N|, one bit per cycle
          if (dv_n == 6'd33) begin
            if (!op_fused) sp <= sp - 1'b1;
            T     <= dv_zero ? 32'd0 :
                     dv_rem  ? (dv_rneg ? (~dv_r + 32'd1) : dv_r)
                             : (dv_qneg ? (~dv_q + 32'd1) : dv_q);
            state <= S_EXEC;
          end else begin
            dv_n <= dv_n + 6'd1;
            if (dv_n != 6'd32) begin
              if (!dv_try[32]) begin
                dv_r <= dv_try[31:0];
                dv_q <= {dv_q[30:0], 1'b1};
              end else begin
                dv_r <= {dv_r[30:0], dv_q[31]};
                dv_q <= {dv_q[30:0], 1'b0};
              end
            end
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

  // {cycle counter, T, sp, opcode, npc (16 bits), state}
  assign debug = {cyc, T, {(8 - STACK_AW){1'b0}}, sp, opc, {(16 - PROG_AW){1'b0}}, npc, 5'd0, state};

endmodule
