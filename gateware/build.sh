#!/bin/sh
# Builds a bitstream with Gowin EDA (gw_sh). Called by the Makefile:
#
#   build.sh OUTDIR PROGRAM SYS_MHZ [faust2tang options...]
#
# PROGRAM is a .dsp (compiled with $FAUST) or .fbc; the faust2tang options
# select the hardware (--generic, --tdm). Output: OUTDIR/<name>.fs, the
# timing report OUTDIR/impl/pnr/nanotangfaust.tr.html, the utilization
# report OUTDIR/impl/pnr/nanotangfaust.rpt.txt and faust2tang's report.txt.
set -e
GW=$(cd "$(dirname "$0")" && pwd)
OUT=$1
PROGRAM=$(cd "$(dirname "$2")" && pwd)/$(basename "$2")
MHZ=$3
shift 3
GW_SH=${GW_SH:-gw_sh}
FAUST=${FAUST:-faust}
NAME=$(basename "$PROGRAM"); NAME=${NAME%.*}

# PLL settings: 27 MHz * (FBDIV + 1) / (IDIV + 1), VCO = output * ODIV
case $MHZ in
  48)    PLL="48000000 8 15 16" ;;
  40.5)  PLL="40500000 1 2 16" ;;
  33.75) PLL="33750000 3 4 16" ;;
  27)    PLL="27000000 0 0 32" ;;
  *) echo "build.sh: SYS_MHZ must be 48, 40.5, 33.75 or 27" >&2; exit 2 ;;
esac
set -- $PLL "$@"
HZ=$1 IDIV=$2 FBDIV=$3 ODIV=$4
shift 4

mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
rm -rf "$OUT/impl"

# the program: memory images and config.vh (hardware sizes, boot program)
"$GW/../tools/bin/faust2tang" "$PROGRAM" -o "$OUT" --faust "$FAUST" --clk-hz "$HZ" "$@"

# sources side by side, so `include "config.vh"` and $readmemh("prog.hex") resolve
cp "$GW"/rtl/*.v "$GW"/rtl/*.vh "$GW"/rtl/sdram/*.v "$GW/constraints/tangnano20k.cst" "$OUT/"

SDRAM=$(grep -q 'NTF_USE_SDRAM   1' "$OUT/config.vh" && echo 1 || true)
{
  echo '`define SYNTHESIS'
  [ -n "$SDRAM" ] && echo '`define NTF_SDRAM'
  echo "\`define NTF_CLK_HZ $HZ"
  echo "\`define NTF_PLL_IDIV $IDIV"
  echo "\`define NTF_PLL_FBDIV $FBDIV"
  echo "\`define NTF_PLL_ODIV $ODIV"
} > "$OUT/ntf_defines.v"

# Timing: the 27 MHz oscillator and the PLL output. The UART bridge (27 MHz)
# and the system clock domain exchange signals only through synchronizers.
cat > "$OUT/nanotangfaust.sdc" <<EOF
create_clock -name clk_27m -period 37.037 [get_ports {clk_27m}]
create_generated_clock -name sys_clk -source [get_ports {clk_27m}] -master_clock clk_27m -multiply_by $((FBDIV + 1)) -divide_by $((IDIV + 1)) [get_pins {u_pll/pll/CLKOUT}]
set_false_path -from [get_clocks {clk_27m}] -to [get_clocks {sys_clk}]
set_false_path -from [get_clocks {sys_clk}] -to [get_clocks {clk_27m}]
EOF

{
  echo "set_device GW2AR-LV18QN88C8/I7 -name GW2AR-18C"
  echo "add_file ntf_defines.v"
  for f in top_tangnano20k dsp_core fpu hmul i2s_master spi_ctrl audio_conv pll_sys tdm_tx uart_bridge; do
    echo "add_file $f.v"
  done
  [ -n "$SDRAM" ] && echo "add_file sdram.v" && echo "add_file sdram_bus.v"
  echo "add_file tangnano20k.cst"
  echo "add_file nanotangfaust.sdc"
  echo "set_option -top_module top_tangnano20k"
  echo "set_option -output_base_name nanotangfaust"
  echo "set_option -use_mspi_as_gpio 1"
  echo "set_option -use_sspi_as_gpio 1"
  echo "run all"
  echo "exit"
} > "$OUT/build.tcl"

cd "$OUT"
"$GW_SH" build.tcl > gowin.log 2>&1 || { grep -E "ERROR" gowin.log >&2; exit 1; }
rm -f "$NAME.fs"   # Gowin writes its bitstream read-only
cp impl/pnr/nanotangfaust.fs "$NAME.fs"
chmod 644 "$NAME.fs"

# summary: maximum clock frequency (Gowin's worst case: 0.95 V, 85 C)
sed -e 's/<[^>]*>/ /g' impl/pnr/nanotangfaust_tr_content.html | tr -s ' \t\n' ' ' |
  grep -o 'sys_clk [0-9.]*(MHz) [0-9.]*(MHz)' | head -1 |
  awk '{printf "sys_clk: target %s, Fmax %s (worst case)\n", $2, $3}'
echo "Bitstream: $OUT/$NAME.fs"
