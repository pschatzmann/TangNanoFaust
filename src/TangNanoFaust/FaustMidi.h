#pragma once
#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "TangNanoFaust.h"

namespace tangnanofaust {

/**
 * @brief Plays the Faust program on the FPGA from MIDI received through the
 * Arduino Serial API: anything that is a `Stream` -- `Serial` (e.g. with a
 * serial-to-MIDI bridge on the PC), `Serial1`/`Serial2` at 31250 baud with a
 * DIN MIDI input circuit, or a USB-MIDI class that exposes a Stream.
 *
 * Mapping, following Faust's own MIDI conventions:
 * - note on/off -> the parameters labelled `freq` (Hz), `gain` (velocity,
 *   0..1) and `gate` (1 while a key is held). Monophonic with last-note
 *   priority; a new note while another is held glides there without
 *   re-triggering the envelope (legato).
 * - control change N -> every parameter with `[midi:ctrl N]` metadata,
 *   scaled from 0..127 to the parameter's range.
 * - pitch bend -> a parameter with `[midi:pitchwheel]` metadata if there is
 *   one, otherwise it bends `freq` by up to +-2 semitones (setBendRange()).
 * - note on/off of key N -> every parameter with `[midi:key N]` metadata
 *   (e.g. drums): the velocity scaled to its range, back to its minimum on
 *   note off.
 *
 * Several instruments in one Faust program: put each in its own group
 * (`vgroup("bass", ...)`) and give each FaustMidi a channel and a group.
 * It then only uses the parameters in that group, so every instrument can
 * have its own `freq`/`gain`/`gate` and `[midi:ctrl 7]`. One stream can
 * only be read once, so feed its bytes to all of them with parse():
 *
 * @code
 * FaustMidi bass(faust), drums(faust);
 * bass.begin(1, "bass");     // channel 1 plays /bass/...
 * drums.begin(10, "drums");  // channel 10 plays /drums/...
 * while (Serial1.available() > 0) {
 *   uint8_t b = Serial1.read();
 *   bass.parse(b);
 *   drums.parse(b);
 * }
 * @endcode
 *
 * @code
 * TangNanoFaust faust;
 * FaustMidi midi(faust);
 * void setup() {
 *   Serial1.begin(31250);
 *   faust.begin(SPI, 5);
 *   midi.begin(Serial1);
 * }
 * void loop() { midi.update(); }
 * @endcode
 *
 * noteOn()/noteOff()/controlChange()/pitchBend() are public, so events from
 * another MIDI library can be fed in directly as well.
 */
class FaustMidi {
 public:
  explicit FaustMidi(TangNanoFaust &faust) : faust_(faust) {}

  /// `in` must outlive this object. `channel` 1..16, or 0 for all channels.
  /// `group` (e.g. "bass"): use only the parameters in this Faust group,
  /// nullptr for all. Call after TangNanoFaust::begin() (it needs the
  /// parameter list).
  void begin(Stream &in, uint8_t channel = 0, const char *group = nullptr) {
    begin(channel, group);
    in_ = &in;
  }

  /// Like begin(Stream &, ...) without a stream: the sketch passes the MIDI
  /// bytes to parse() itself, e.g. to several FaustMidi on one input.
  void begin(uint8_t channel, const char *group = nullptr) {
    in_ = nullptr;
    channel_ = channel;
    group_[0] = 0;
    if (group && *group) {
      if (*group != '/') strcpy(group_, "/");
      strncat(group_, group, sizeof(group_) - 3);
      strcat(group_, "/");
    }
    freq_ = find("freq");
    gain_ = find("gain");
    gate_ = find("gate");
    wheel_ = -1;
    char buf[16];
    for (int i = 0; i < faust_.parameterCount(); i++)
      if (inGroup(i) && faust_.parameter(i).metaValue("midi", buf, sizeof(buf)) &&
          strcmp(buf, "pitchwheel") == 0)
        wheel_ = i;
    held_ = 0;
    status_ = 0;
  }

  /// Reads and handles all MIDI bytes available on the stream. Call often
  /// (from loop()).
  void update() {
    if (!in_) return;
    while (in_->available() > 0) parse((uint8_t)in_->read());
  }

  /// Feeds one MIDI byte (running status and real-time bytes handled).
  void parse(uint8_t b) {
    if (b >= 0xF8) return;            // real-time: clock, start, stop...
    if (b & 0x80) {
      if (b >= 0xF0) {                // system common/exclusive: skip data
        status_ = 0;
        return;
      }
      status_ = b;
      count_ = 0;
      return;
    }
    if (!status_) return;
    data_[count_++] = b;
    uint8_t type = status_ & 0xF0;
    uint8_t need = (type == 0xC0 || type == 0xD0) ? 1 : 2;
    if (count_ < need) return;
    count_ = 0;                       // running status: same status continues
    uint8_t ch = (status_ & 0x0F) + 1;
    if (channel_ && ch != channel_) return;
    switch (type) {
      case 0x90:
        if (data_[1]) noteOn(data_[0], data_[1]);
        else noteOff(data_[0]);
        break;
      case 0x80: noteOff(data_[0]); break;
      case 0xB0: controlChange(data_[0], data_[1]); break;
      case 0xE0: pitchBend((int)(data_[0] | (data_[1] << 7)) - 8192); break;
      default: break;
    }
  }

  void noteOn(uint8_t note, uint8_t velocity) {
    setKey(note, velocity);
    removeNote(note);
    if (held_ == kMaxHeld) removeAt(0);
    notes_[held_++] = note;
    if (gain_ >= 0) faust_.setParameter(gain_, velocity / 127.0f);
    updateFreq();
    if (gate_ >= 0 && held_ == 1) faust_.setParameter(gate_, 1.0f);
  }

  void noteOff(uint8_t note) {
    setKey(note, 0);
    bool wasCurrent = held_ > 0 && notes_[held_ - 1] == note;
    removeNote(note);
    if (held_ == 0) {
      if (gate_ >= 0) faust_.setParameter(gate_, 0.0f);
    } else if (wasCurrent) {
      updateFreq();
    }
  }

  void controlChange(uint8_t controller, uint8_t value) {
    if (controller == 123) {  // all notes off
      held_ = 0;
      if (gate_ >= 0) faust_.setParameter(gate_, 0.0f);
      setKey(-1, 0);
      return;
    }
    char buf[16];
    for (int i = 0; i < faust_.parameterCount(); i++) {
      const Parameter &p = faust_.parameter(i);
      if (inGroup(i) && p.metaValue("midi", buf, sizeof(buf)) && strncmp(buf, "ctrl", 4) == 0 &&
          atoi(buf + 4) == controller)
        faust_.setParameter(i, p.min + (p.max - p.min) * value / 127.0f);
    }
  }

  /// `bend` -8192..8191.
  void pitchBend(int bend) {
    if (wheel_ >= 0) {
      const Parameter &p = faust_.parameter(wheel_);
      faust_.setParameter(wheel_, p.min + (p.max - p.min) * (bend + 8192) / 16383.0f);
    } else {
      bend_ = bend;
      if (held_) updateFreq();
    }
  }

  /// Pitch bend range in semitones when bending `freq` (default 2).
  void setBendRange(float semitones) { bendRange_ = semitones; }

  /// Frequency of a MIDI note in Hz (A4 = note 69 = 440 Hz).
  static float noteToHz(float note) { return 440.0f * powf(2.0f, (note - 69.0f) / 12.0f); }

 protected:
  static constexpr int kMaxHeld = 16;
  TangNanoFaust &faust_;
  Stream *in_ = nullptr;
  uint8_t channel_ = 0;
  char group_[32] = "";  ///< "/bass/", or empty for all parameters
  uint8_t status_ = 0, count_ = 0, data_[2];
  int freq_ = -1, gain_ = -1, gate_ = -1, wheel_ = -1;
  uint8_t notes_[kMaxHeld];
  int held_ = 0;
  int bend_ = 0;
  float bendRange_ = 2.0f;

  bool inGroup(int i) const {
    return !group_[0] || strncmp(faust_.parameter(i).path, group_, strlen(group_)) == 0;
  }

  /// Index of the parameter with this label in the group, or -1.
  int find(const char *label) const {
    for (int i = 0; i < faust_.parameterCount(); i++)
      if (inGroup(i) && strcmp(faust_.parameter(i).label(), label) == 0) return i;
    return -1;
  }

  /// Sets the `[midi:key N]` parameters of `note` (-1: all of them) to the
  /// velocity, scaled to their range (0: their minimum).
  void setKey(int note, uint8_t velocity) {
    char buf[16];
    for (int i = 0; i < faust_.parameterCount(); i++) {
      const Parameter &p = faust_.parameter(i);
      if (inGroup(i) && p.metaValue("midi", buf, sizeof(buf)) && strncmp(buf, "key", 3) == 0 &&
          (buf[3] == ' ' || isdigit((unsigned char)buf[3])) && (note < 0 || atoi(buf + 3) == note))
        faust_.setParameter(i, p.min + (p.max - p.min) * velocity / 127.0f);
    }
  }

  void updateFreq() {
    if (freq_ < 0 || held_ == 0) return;
    float note = notes_[held_ - 1] + bendRange_ * bend_ / 8192.0f;
    faust_.setParameter(freq_, noteToHz(note));
  }

  void removeAt(int i) {
    for (int j = i; j < held_ - 1; j++) notes_[j] = notes_[j + 1];
    held_--;
  }

  void removeNote(uint8_t note) {
    for (int i = 0; i < held_; i++)
      if (notes_[i] == note) {
        removeAt(i);
        return;
      }
  }
};

}  // namespace tangnanofaust
