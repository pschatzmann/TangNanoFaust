// Plays the Faust program on the Tang Nano 20K from MIDI received with the
// Arduino Serial API (e.g. gateware/dsp/synth.dsp: note on/off drive its
// freq/gain/gate parameters, CC 74/71 its [midi:ctrl] cutoff/resonance).
//
// MIDI_IN is any Stream:
//  - Serial1 at 31250 baud with a DIN MIDI input (optocoupler) circuit, or
//  - Serial at 115200 with a serial-to-MIDI bridge on the PC
//    (e.g. Hairless MIDI<->Serial), as configured below.
#include <TangNanoFaust.h>
using namespace tangnanofaust;

const int CS_PIN = 5;
#define MIDI_IN Serial
const long MIDI_BAUD = 115200;  // 31250 for a DIN MIDI port

TangNanoFaust faust;
FaustMidi midi(faust);

void setup() {
  MIDI_IN.begin(MIDI_BAUD);
  while (!faust.begin(SPI, CS_PIN)) delay(1000);
  midi.begin(MIDI_IN);  // all MIDI channels
}

void loop() { midi.update(); }
