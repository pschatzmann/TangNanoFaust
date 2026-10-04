// Lists the parameters of the Faust program running on the Tang Nano 20K
// and sweeps its "freq" parameter (e.g. gateware/dsp/sine.dsp).
//
// Wiring (see docs/pinout.md): MCU SPI SCK/MOSI/MISO -> FPGA pins 27/28/29,
// CS_PIN -> FPGA pin 30, and a common ground.
#include <TangNanoFaust.h>
using namespace tangnanofaust;

const int CS_PIN = 5;
TangNanoFaust faust;

void setup() {
  Serial.begin(115200);
  while (!faust.begin(SPI, CS_PIN)) {
    Serial.println("Tang Nano 20K not answering -- check wiring and bitstream");
    delay(1000);
  }
  Info info = faust.info();
  Serial.printf("Faust program '%s': %d in, %d out, %lu Hz\n", faust.name(), info.inputs,
                info.outputs, (unsigned long)info.sampleRate);
  for (int i = 0; i < faust.parameterCount(); i++) {
    const Parameter &p = faust.parameter(i);
    Serial.printf("  %d: %s [%g .. %g] init %g\n", i, p.path, p.min, p.max, p.init);
  }
}

void loop() {
  static float freq = 220;
  faust.setParameter("freq", freq);
  freq = freq < 880 ? freq * 1.05f : 220;
  delay(100);

  static unsigned long last = 0;
  if (millis() - last > 2000) {
    last = millis();
    Info info = faust.info();
    Serial.printf("DSP load %.0f%%%s\n", info.load(), info.overrun ? " (overrun!)" : "");
  }
}
