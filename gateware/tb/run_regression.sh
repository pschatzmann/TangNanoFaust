#!/bin/sh
# RTL regression: FPU vs specification, the core vs the simulator on the
# test programs, and the whole design through its pins.
#   run_regression.sh          (FULL=1 also runs the slow table/SDRAM programs)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
TESTS="$HERE/../../tests"
WORK="$HERE/../build/tb"
mkdir -p "$WORK"
make -s -C "$TESTS"

echo "== simulator vs reference interpreter"
"$TESTS/build/dsp_test" check "$TESTS"/fbc/*.fbc

echo "== performance guard (voices10, voices10_sin)"
"$TESTS/build/dsp_test" --max-cycles 1125 check "$TESTS/fbc/voices10.fbc"
# with sin() oscillators and --fast-math they don't (yet): guard against regressions
"$TESTS/build/dsp_test" --fast-math --max-cycles 1900 check "$TESTS/fbc/voices10_sin.fbc"

echo "== FPU"
"$TESTS/build/fpu_vectors" gen "$WORK/fpu_vectors.hex" 4000
iverilog -g2012 -o "$WORK/tb_fpu" "$HERE/tb_fpu.v" "$HERE/../rtl/fpu.v" "$HERE/../rtl/hmul.v"
vvp -n "$WORK/tb_fpu" +vectors="$WORK/fpu_vectors.hex" | grep -E "tb_fpu|PASS|FAIL|MISMATCH"

echo "== core vs simulator"
for p in gain_offset lowpass lowpass_cubic saw_adsr karplus delay math voices10; do
  "$HERE/run_core_test.sh" "$TESTS/fbc/$p.fbc" 300
done
DSP_TEST_OPTS=--fast-math "$HERE/run_core_test.sh" "$TESTS/fbc/voices10_sin.fbc" 300
if [ -n "$FULL" ]; then
  for p in freeverb oscsin_gain osc_noise_cubic; do
    "$HERE/run_core_test.sh" "$TESTS/fbc/$p.fbc" 300
  done
fi

echo "== system (SPI load + I2S)"
"$HERE/run_top_test.sh"

echo "== system, bitstream without TDM (2 outputs)"
NOTDM=1 "$HERE/run_top_test.sh"

echo "== system with an external TDM master (48 kHz and 44.1 kHz)"
TDM=1 "$HERE/run_top_test.sh" "$TESTS/fbc/passthrough.fbc" "$TESTS/fbc/tdm_test.fbc"
TDM=1 TDM_HALF=44.29 "$HERE/run_top_test.sh" "$TESTS/fbc/passthrough.fbc" "$TESTS/fbc/tdm_test.fbc"

if [ -n "$FULL" ]; then  # ~10 minutes: real 115200 baud
  echo "== system over the USB UART"
  UART=1 "$HERE/run_top_test.sh"
fi
