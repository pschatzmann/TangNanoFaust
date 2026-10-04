// 27 MHz onboard oscillator -> system clock (build.sh passes the divider
// values for SYS_MHZ), plus the same clock shifted by 180 degrees for the
// SDRAM (sdram.v expects that on clk_sdram).
//
// Output = 27 MHz * (FBDIV + 1) / (IDIV + 1), VCO = output * ODIV:
//   48 MHz     IDIV 8 FBDIV 15 ODIV 16
//   40.5 MHz   IDIV 1 FBDIV 2  ODIV 16
//   33.75 MHz  IDIV 3 FBDIV 4  ODIV 16
//   27 MHz     IDIV 0 FBDIV 0  ODIV 32
// PSDA_SEL "1000" = 180 degree phase on CLKOUTP. Without the defines (the
// simulation testbenches) the PLL settings below are used; simulation
// passes clk_27m through.
`ifndef NTF_PLL_IDIV
`define NTF_PLL_IDIV 0
`define NTF_PLL_FBDIV 1
`define NTF_PLL_ODIV 16
`endif

module pll_sys (
    input  wire clock_in,    // 27 MHz
    output wire clock_out,   // system clock
    output wire clock_p180,  // system clock, 180 degrees
    output wire locked
);
`ifdef SYNTHESIS
  rPLL #(
      .FCLKIN("27.0"),
      .IDIV_SEL(`NTF_PLL_IDIV),
      .FBDIV_SEL(`NTF_PLL_FBDIV),
      .ODIV_SEL(`NTF_PLL_ODIV),
      .PSDA_SEL("1000"),
      .DUTYDA_SEL("1000"),
      .CLKOUT_FT_DIR(1'b1),
      .CLKOUTP_FT_DIR(1'b1),
      .CLKFB_SEL("internal"),
      .CLKOUT_BYPASS("false"),
      .CLKOUTP_BYPASS("false"),
      .DYN_DA_EN("false")
  ) pll (
      .CLKOUTD(), .CLKOUTD3(), .RESET(1'b0), .RESET_P(1'b0), .CLKFB(1'b0),
      .FBDSEL(6'b0), .IDSEL(6'b0), .ODSEL(6'b0), .PSDA(4'b0), .DUTYDA(4'b0),
      .FDLY(4'b0),
      .CLKIN(clock_in),
      .CLKOUT(clock_out),
      .CLKOUTP(clock_p180),
      .LOCK(locked)
  );
`else
  // Simulation: the testbench drives clock_in at the system clock rate.
  assign clock_out  = clock_in;
  assign clock_p180 = ~clock_in;
  assign locked     = 1'b1;
`endif
endmodule
