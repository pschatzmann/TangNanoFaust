#!/bin/sh
# Runs a compiled Faust program on dsp_core.v (iverilog) and requires the
# same outputs and cycle counts as the simulator for every sample.
#   run_core_test.sh prog.fbc [samples]        DSP_TEST_OPTS=--fast-math to compile so
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
FBC=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
NAME=$(basename "$1" .fbc)
WORK="$HERE/../build/tb/$NAME"
RTL="$HERE/../rtl"
mkdir -p "$WORK"
make -s -C "$HERE/../../tests" build/dsp_test
"$HERE/../../tests/build/dsp_test" $DSP_TEST_OPTS rtl "$FBC" "$WORK" "${2:-200}"
iverilog -g2012 -I "$WORK" -I "$RTL" -o "$WORK/tb_dsp_core" "$HERE/tb_dsp_core.v" \
  "$RTL/fpu.v" "$RTL/hmul.v" "$RTL/dsp_core.v"
(cd "$WORK" && vvp -n tb_dsp_core +samples="${2:-200}" > /dev/null)
"$HERE/../../tests/build/dsp_test" compare-rtl "$WORK"
