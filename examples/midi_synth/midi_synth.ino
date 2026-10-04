// A multi-timbral MIDI synthesizer: four instruments on the Tang Nano 20K,
// each played from its own MIDI channel.
//   channel 1   bass   CC 74 cutoff, CC 71 resonance
//   channel 2   lead   CC 1 (mod wheel) vibrato, CC 74 cutoff
//   channel 3   pluck  CC 74 damping
//   channel 10  drums  kick 36, snare 38, closed hi-hat 42, open hi-hat 46
// CC 7 sets the volume of the instrument on that channel; pitch bend bends
// the bass, lead and pluck.
//
// The Faust program is multisynth.dsp in this sketch folder. The sketch
// uploads it to the FPGA at startup from multisynth_program.h, which was
// generated with
//     tools/bin/faust2tang --sample-rate 32000 examples/midi_synth/multisynth.dsp -o build
// (regenerate it the same way after editing multisynth.dsp). It runs at
// 32 kHz: four instruments need more DSP cycles than 48 kHz leaves.
//
// MIDI_IN is any Stream:
//  - Serial1 at 31250 baud with a DIN MIDI input (optocoupler) circuit, or
//  - Serial at 115200 with a serial-to-MIDI bridge on the PC
//    (e.g. Hairless MIDI<->Serial), as configured below.
//
// Needs: the generic bitstream on the FPGA (`make -C gateware generic-flash`).
// Wiring: SPI SCK/MOSI/MISO -> FPGA pins 27/28/29, CS_PIN -> pin 30, GND.
#include <TangNanoFaust.h>

#include "multisynth_program.h"

using namespace tangnanofaust;

const int CS_PIN = 5;
#define MIDI_IN Serial
const long MIDI_BAUD = 115200;  // 31250 for a DIN MIDI port

TangNanoFaust faust;
FaustMidi bass(faust), lead(faust), pluck(faust), drums(faust);
FaustMidi *instruments[] = {&bass, &lead, &pluck, &drums};

void setup() {
  MIDI_IN.begin(MIDI_BAUD);
  while (!faust.begin(SPI, CS_PIN)) delay(1000);
  while (faust.load(multisynth::program) != LoadResult::Ok) delay(1000);

  // each instrument is a group in multisynth.dsp
  bass.begin(1, "bass");
  lead.begin(2, "lead");
  pluck.begin(3, "pluck");
  drums.begin(10, "drums");
}

void loop() {
  // one MIDI input for all instruments: each picks its own channel
  while (MIDI_IN.available() > 0) {
    uint8_t b = MIDI_IN.read();
    for (FaustMidi *instrument : instruments) instrument->parse(b);
  }
}
