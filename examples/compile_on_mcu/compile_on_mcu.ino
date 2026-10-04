// Compiles a Faust program on the microcontroller and runs it on the
// Tang Nano 20K -- no FPGA toolchain, no rebuilding the bitstream.
//
// The Faust program is synth.dsp in this sketch folder. The Faust compiler
// itself only runs on a computer, so after editing synth.dsp regenerate its
// bytecode once with
//     tools/bin/faust2tang --sketch examples/compile_on_mcu/synth.dsp
// which writes synth_fbc.h. This sketch then compiles that bytecode for
// the FPGA's DSP core on the MCU (FaustCompiler, the same compiler the
// faust2tang tool uses) and uploads it.
//
// Needs: the generic bitstream on the FPGA (`make -C gateware generic-flash`)
// and an MCU with the C++ standard library (ESP32, RP2040, ...).
// Wiring: SPI SCK/MOSI/MISO -> FPGA pins 27/28/29, CS_PIN -> pin 30, GND.
#include <TangNanoFaustCompiler.h>

#include "synth_fbc.h"

using namespace tangnanofaust;

const int CS_PIN = 5;

TangNanoFaust faust;
FaustCompiler faustCompiler;

void setup() {
  Serial.begin(115200);
  delay(2000);
  while (!faust.begin(SPI, CS_PIN)) {
    Serial.println("Tang Nano 20K not answering -- check wiring and bitstream");
    delay(1000);
  }

  uint32_t start = micros();
  if (!faustCompiler.compileAndLoad(faust, synth_fbc)) {
    Serial.print("Faust program not loaded: ");
    Serial.println(faustCompiler.error());
    while (true) delay(1000);
  }
  uint32_t ms = (micros() - start) / 1000;

  const compiler::ProgramImage &img = faustCompiler.image();
  Serial.printf("'%s' compiled and loaded in %lu ms: %u instructions, %u words of block RAM, "
                "%u words of SDRAM\n",
                img.name.c_str(), (unsigned long)ms, (unsigned)img.program.size(),
                (unsigned)img.fastWords, (unsigned)img.sdramWords);
  for (int i = 0; i < faust.parameterCount(); i++)
    Serial.printf("  %s [%g .. %g]\n", faust.parameter(i).path, faust.parameter(i).min,
                  faust.parameter(i).max);
}

void loop() {
  // a little melody on the synth's freq/gate parameters
  static const int notes[] = {57, 60, 64, 69, 67, 64, 60, 62};
  static int i = 0;
  faust.setParameter("freq", FaustMidi::noteToHz(notes[i]));
  faust.setParameter("gate", 1);
  delay(180);
  faust.setParameter("gate", 0);
  delay(70);
  i = (i + 1) % 8;

  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    Info info = faust.info();
    Serial.printf("DSP load %.0f%%%s\n", info.load(), info.overrun ? " (overrun)" : "");
  }
}
