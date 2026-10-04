// MIDI test on the board without a microcontroller: runs FaustMidi and
// TangNanoFaust (the real library code) on a PC against the
// board -- the FPGA over its USB serial port (TangNanoFaust::begin(Stream&),
// the same protocol as an MCU on the header UART), MIDI bytes from a
// scripted Stream with real-time spacing: a melody, a CC 74 sweep and a
// pitch bend. Load a program with freq/gain/gate and [midi:ctrl 74] first:
//   faust2tang gateware/dsp/synth.dsp -o build/synth --port /dev/ttyUSB1 --load
//   tests/build/midi_test /dev/ttyUSB1
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <vector>
#include "NanoTangFaust.h"
using namespace nanotangfaust;

class SerialPort : public Stream {
  int fd_ = -1;
 public:
  bool open(const char *dev) {
    fd_ = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;
    termios t{}; tcgetattr(fd_, &t); cfmakeraw(&t); cfsetspeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD; tcsetattr(fd_, TCSANOW, &t); tcflush(fd_, TCIOFLUSH);
    return true;
  }
  int available() override { pollfd p{fd_, POLLIN, 0}; return poll(&p, 1, 0) > 0 ? 1 : 0; }
  int read() override { uint8_t c; return ::read(fd_, &c, 1) == 1 ? c : -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    size_t off = 0;
    while (off < n) { ssize_t w = ::write(fd_, b + off, n - off); if (w > 0) off += w; else usleep(100); }
    return n;
  }
  void flush() override { tcdrain(fd_); }
};

// MIDI bytes released at their time stamps (ms from start), like a serial line.
class MidiScript : public Stream {
  struct Ev { uint32_t at; std::vector<uint8_t> bytes; };
  std::vector<Ev> evs_;
  size_t ev_ = 0, pos_ = 0;
  uint32_t t0_ = 0;
 public:
  void add(uint32_t at, std::vector<uint8_t> b) { evs_.push_back({at, b}); }
  void start() { t0_ = millis(); }
  bool done() const { return ev_ >= evs_.size(); }
  uint32_t end() const { return evs_.empty() ? 0 : evs_.back().at; }
  int available() override { return !done() && millis() - t0_ >= evs_[ev_].at ? 1 : 0; }
  int read() override {
    if (!available()) return -1;
    int c = evs_[ev_].bytes[pos_++];
    if (pos_ == evs_[ev_].bytes.size()) { pos_ = 0; ev_++; }
    return c;
  }
  size_t write(uint8_t) override { return 0; }
};

int main(int argc, char **argv) {
  SerialPort port;
  if (!port.open(argc > 1 ? argv[1] : "/dev/ttyUSB1")) { printf("cannot open port\n"); return 1; }
  TangNanoFaust faust;
  if (!faust.begin(port)) { printf("FPGA not answering\n"); return 1; }
  printf("program '%s', %d parameters\n", faust.name(), faust.parameterCount());
  for (int i = 0; i < faust.parameterCount(); i++)
    printf("  %-12s meta '%s'\n", faust.parameter(i).path, faust.parameter(i).meta ? faust.parameter(i).meta : "");

  MidiScript m;
  uint32_t t = 0;
  // 1. a melody on channel 1 (running status on the note offs)
  int notes[] = {57, 60, 64, 69, 67, 64, 60, 62};
  for (int n : notes) { m.add(t, {0x90, (uint8_t)n, 100}); m.add(t + 220, {0x80, (uint8_t)n, 0}); t += 300; }
  t += 300;
  // 2. a held note while CC 74 (cutoff) sweeps up and down
  m.add(t, {0x90, 45, 110});
  for (int k = 0; k <= 40; k++) m.add(t + 50 + k * 40, {0xB0, 74, (uint8_t)(k <= 20 ? k * 6 : (40 - k) * 6)});
  t += 50 + 41 * 40;
  // 3. pitch bend up a whole tone and back, then note off
  for (int k = 0; k <= 20; k++) { int b = 8192 + (k <= 10 ? k : 20 - k) * 819; m.add(t + k * 40, {0xE0, (uint8_t)(b & 127), (uint8_t)(b >> 7)}); }
  t += 21 * 40;
  m.add(t + 100, {0x80, 45, 0});
  m.add(t + 150, {0xB0, 74, 64});   // filter back to the middle

  FaustMidi midi(faust);
  midi.begin(m);
  m.start();
  uint32_t lastPrint = 0, start = millis();
  while (!m.done() || millis() - start < m.end() + 800) {
    midi.update();
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      printf("%5.1f s  freq %7.2f  gate %.0f  cutoff %6.0f\n", (millis() - start) / 1000.0,
             faust.getParameter("freq"), faust.getParameter("gate"), faust.getParameter("cutoff"));
    }
    delay(1);
  }
  Info inf = faust.info();
  printf("done: load %.0f%%%s\n", inf.load(), inf.overrun ? ", OVERRUN" : "");
  return 0;
}
