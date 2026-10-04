#!/bin/sh
# System test of top_tangnano20k.v through its pins: boots one program,
# loads another over SPI, sets/reads parameters and checks the I2S output.
#   run_top_test.sh [boot.fbc] [load.fbc]      TDM=1: with an external TDM master
#                                              NOTDM=1: bitstream without TDM (2 outputs)
#                                              UART=1: commands over the USB UART
#                                              (TDM_HALF=44.29: its BCLK half period in ns)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
TESTS="$HERE/../../tests"
BOOT=${1:-$TESTS/fbc/passthrough.fbc}
LOAD=${2:-$TESTS/fbc/gain_offset.fbc}
WORK="$HERE/../build/tb/top${TDM:+_tdm}${NOTDM:+_notdm}${UART:+_uart}"
RTL="$HERE/../rtl"
mkdir -p "$WORK"
make -s -C "$TESTS" build/dsp_test
"$TESTS/build/dsp_test" top "$BOOT" "$LOAD" "$WORK" ${NOTDM:+notdm}
iverilog -g2012 -DNTF_REF_HZ=54000000 -I "$WORK" -I "$RTL" -o "$WORK/tb_top" "$HERE/tb_top.v" \
  "$RTL/top_tangnano20k.v" "$RTL/dsp_core.v" "$RTL/fpu.v" "$RTL/hmul.v" \
  "$RTL/i2s_master.v" "$RTL/spi_ctrl.v" "$RTL/audio_conv.v" "$RTL/pll_sys.v" "$RTL/tdm_tx.v" "$RTL/uart_bridge.v"
(cd "$WORK" && vvp -n tb_top $(cat plusargs.txt) ${TDM:++tdm} ${TDM_HALF:++tdm_half=$TDM_HALF} ${UART:++uart} > vvp.log)
"$TESTS/build/dsp_test" compare-top "$WORK"
