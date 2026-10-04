// Controls the Faust program on the Tang Nano 20K over a UART instead of
// SPI: 2 wires plus ground.
//
// Wiring: MCU TX -> FPGA pin 25, MCU RX <- FPGA pin 26, GND. 115200 baud.
// (The board's own USB port reaches the FPGA the same way, from a PC:
//  tools/bin/faust2tang --port /dev/ttyUSB1 --list)
#include <NanoTangFaust.h>
using namespace nanotangfaust;

TangNanoFaust faust;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // on ESP32: Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN)
  while (!faust.begin(Serial1)) {
    Serial.println("Tang Nano 20K not answering on Serial1");
    delay(1000);
  }
  Serial.printf("'%s' with %d parameters\n", faust.name(), faust.parameterCount());
}

void loop() {
  static float freq = 220;
  faust.setParameter("freq", freq);
  freq = freq < 880 ? freq * 1.05f : 220;
  delay(100);
}
