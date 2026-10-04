// Compiles Faust source code over WiFi and runs it on the Tang Nano 20K.
//
// The Faust compiler needs a computer, so this ESP32 sends the source to a
// compile server on your network:
//     tools/bin/faust2tang --serve 8000
// which compiles it and returns the finished program (a few KB); the ESP32
// loads it into the FPGA.
//
// At startup it plays the program below. Then paste another Faust program
// into the serial monitor (115200 baud, newline line ending) and finish it
// with a line containing only "." -- it is compiled and played at once.
//
// FaustServerCompiler (NanoTangFaustServer.h) does the work; it runs over
// any Arduino Client, so the same code works with Ethernet or a Pico W.
//
// Needs: the generic bitstream on the FPGA, an ESP32.
// Wiring: SPI SCK/MOSI/MISO -> FPGA pins 27/28/29, CS_PIN -> pin 30, GND.
#include <NanoTangFaustServer.h>
#include <WiFi.h>

#include <string>

using namespace nanotangfaust;

const char *WIFI_SSID = "your-ssid";
const char *WIFI_PASSWORD = "your-password";
const char *SERVER = "http://192.168.1.10:8000";  // the computer running faust2tang --serve
const int CS_PIN = 5;

static const char kStartProgram[] = R"FAUST(
import("stdfaust.lib");
freq = hslider("freq [unit:Hz]", 330, 50, 2000, 1);
gain = hslider("gain", 0.3, 0, 1, 0.01);
process = os.osc(freq) * gain;
)FAUST";

TangNanoFaust faust;
WiFiClient client;
FaustServerCompiler server(client, SERVER);

bool compileAndLoad(const char *source) {
  uint32_t start = millis();
  if (!server.compileAndLoad(faust, source)) {
    Serial.printf("not loaded: %s\n", server.error());  // e.g. Faust's error message
    return false;
  }
  Serial.printf("compiled and loaded in %lu ms, parameters:\n", (unsigned long)(millis() - start));
  for (int i = 0; i < faust.parameterCount(); i++)
    Serial.printf("  %s [%g .. %g]\n", faust.parameter(i).path, faust.parameter(i).min,
                  faust.parameter(i).max);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  while (!faust.begin(SPI, CS_PIN)) {
    Serial.println("Tang Nano 20K not answering -- check wiring and bitstream");
    delay(1000);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.printf(" connected, %s\n", WiFi.localIP().toString().c_str());
  compileAndLoad(kStartProgram);
  Serial.println("Paste a Faust program, end it with a line containing only '.'");
}

void loop() {
  static std::string source, line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      line += c;
      continue;
    }
    if (line == ".") {
      compileAndLoad(source.c_str());
      source.clear();
    } else {
      source += line + "\n";
    }
    line.clear();
  }
}
