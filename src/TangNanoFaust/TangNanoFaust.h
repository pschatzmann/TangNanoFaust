#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <stdlib.h>
#include <string.h>

#include "ProgramData.h"

namespace tangnanofaust {

/// Kind of a Faust UI element, as described by the FPGA.
enum class ParameterKind { Button, Checkbox, HSlider, VSlider, NumEntry, HBargraph, VBargraph, Unknown };

/**
 * @brief One Faust UI element (slider, button, bargraph...) of the program
 * running on the FPGA. Read from the FPGA's descriptor by
 * TangNanoFaust::begin(), so a sketch needs no generated header.
 */
struct Parameter {
  ParameterKind kind = ParameterKind::Unknown;
  uint32_t address = 0;  ///< word address in the DSP core's block RAM
  float init = 0, min = 0, max = 1, step = 0;
  char *path = nullptr;  ///< e.g. "/freq" (Faust's group path + label)
  char *meta = nullptr;  ///< "key:value,key:value" (e.g. "midi:ctrl 7,unit:Hz")

  /// The label (last path component).
  const char *label() const {
    const char *slash = path ? strrchr(path, '/') : nullptr;
    return slash ? slash + 1 : (path ? path : "");
  }

  /// Bargraphs are outputs of the DSP: read them with getParameter().
  bool isOutput() const {
    return kind == ParameterKind::HBargraph || kind == ParameterKind::VBargraph;
  }

  /// Copies the value of metadata `key` (e.g. "midi") into `out`. Returns
  /// false if the parameter has no such metadata.
  bool metaValue(const char *key, char *out, size_t len) const {
    if (!meta || len == 0) return false;
    size_t klen = strlen(key);
    const char *p = meta;
    while (*p) {
      const char *end = strchr(p, ',');
      if (!end) end = p + strlen(p);
      if ((size_t)(end - p) > klen && strncmp(p, key, klen) == 0 && p[klen] == ':') {
        size_t n = end - (p + klen + 1);
        if (n >= len) n = len - 1;
        memcpy(out, p + klen + 1, n);
        out[n] = 0;
        return true;
      }
      p = *end ? end + 1 : end;
    }
    return false;
  }
};

/// Why TangNanoFaust::load() failed.
enum class LoadResult { Ok, NoAnswer, ProgramTooLarge, DescriptorTooLarge, MemoryTooLarge,
                        NeedsSdram, TooManyChannels, BitstreamTooOld, BootTimeout,
                        BadDescriptor };

/// Status of the FPGA, see TangNanoFaust::info().
struct Info {
  uint8_t inputs = 0;
  uint8_t outputs = 0;
  uint8_t parameters = 0;
  bool booted = false;      ///< the boot program (tables, defaults) has finished
  bool overrun = false;     ///< a sample took longer than the sample period
  bool muted = false;
  bool sdram = false;       ///< this bitstream has the SDRAM controller
  bool stopped = false;     ///< the core is stopped (a program is being loaded)
  bool tdmActive = false;   ///< an external TDM master provides the sample clock
  bool tdm = false;         ///< this bitstream has the TDM output (`make TDM=1`)
  uint32_t sampleRate = 0;
  uint16_t descriptorLength = 0;
  uint32_t cyclesMax = 0;   ///< worst cycles per sample since the last info()
  uint32_t cyclesBudget = 0;///< cycles available per sample
  uint32_t clockHz = 0;     ///< DSP core clock
  uint32_t programCapacity = 0;    ///< max instructions
  uint32_t memoryCapacity = 0;     ///< block RAM words
  uint32_t descriptorCapacity = 0; ///< descriptor bytes
  uint8_t maxInputs = 2;           ///< channels this bitstream has
  uint8_t maxOutputs = 2;

  /// Worst-case load of the DSP core in percent since the last info().
  float load() const { return cyclesBudget ? 100.0f * cyclesMax / cyclesBudget : 0; }
};

/**
 * @brief Arduino driver for a Tang Nano 20K running a Faust program on the
 * TangNanoFaust DSP core (see docs/protocol.md). Connects over SPI or over a
 * serial line (`Stream`, 115200 baud) to the FPGA's header UART.
 *
 * The FPGA plays the audio itself (onboard MAX98357A amplifier, optional
 * I2S input); the MCU only changes parameters:
 * @code
 * TangNanoFaust faust;
 * faust.begin(SPI, 5);
 * faust.setParameter("freq", 440);
 * @endcode
 *
 * @author Phil Schatzmann
 * @copyright Apache-2.0
 */
class TangNanoFaust {
 public:
  ~TangNanoFaust() { clearParameters(); }

  /// Starts SPI, checks the FPGA answers and reads the parameter list.
  /// `spi` must outlive this object. Keep `sckHz` at 3 MHz or below (the
  /// FPGA samples SPI with its 48 MHz system clock).
  bool begin(SPIClass &spi, int csPin, uint32_t sckHz = 2000000) {
    spi_ = &spi;
    stream_ = nullptr;
    csPin_ = csPin;
    settings_ = SPISettings(sckHz, MSBFIRST, SPI_MODE0);
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);
    spi_->begin();
    if (!ping()) return false;
    return readParameters();
  }

  /// Connects over a serial line to the FPGA's header UART (pins 25 = FPGA
  /// RX, 26 = FPGA TX). Open the port at 115200 baud 8N1 first, e.g.
  /// `Serial1.begin(115200)`. `serial` must outlive this object.
  bool begin(Stream &serial) {
    stream_ = &serial;
    spi_ = nullptr;
    if (!ping()) return false;
    return readParameters();
  }

  /// True if the FPGA runs the TangNanoFaust gateware.
  bool ping() {
    tb_(kOpPing);
    for (int i = 0; i < 5; i++) tp_(0);
    if (!tx_()) return false;
    version_ = buf_[5];
    return memcmp(buf_ + 1, "FAUS", 4) == 0;
  }

  uint8_t protocolVersion() const { return version_; }

  /// Reads the status; this also resets the overrun flag and cyclesMax.
  Info info() {
    Info r;
    int n = version_ >= 4 ? 23 : 21;  // 4: + channel counts
    tb_(kOpInfo);
    for (int i = 0; i < n; i++) tp_(0);
    if (!tx_()) return r;
    const uint8_t *b = buf_ + 1;
    r.inputs = b[0];
    r.outputs = b[1];
    r.parameters = b[2];
    r.booted = b[3] & 1;
    r.overrun = b[3] & 2;
    r.muted = b[3] & 4;
    r.sdram = b[3] & 8;
    r.stopped = b[3] & 16;
    r.tdmActive = b[3] & 32;
    r.sampleRate = le32(b + 4);
    r.descriptorLength = b[8] | (b[9] << 8);
    r.cyclesMax = le32(b + 10);
    r.clockHz = le32(b + 14);
    r.cyclesBudget = r.sampleRate ? r.clockHz / r.sampleRate : 0;
    r.programCapacity = 1ul << b[18];
    r.memoryCapacity = 1ul << b[19];
    r.descriptorCapacity = 1ul << b[20];
    if (version_ >= 4) {
      r.tdm = b[3] & 64;
      r.maxInputs = b[21];
      r.maxOutputs = b[22];
    } else {  // older bitstreams always had TDM: 2 in, 8 out
      r.tdm = true;
      r.maxOutputs = 8;
    }
    return r;
  }

  /// Replaces the Faust program running on the FPGA (no FPGA rebuild):
  /// uploads the program and its descriptor, restarts the DSP core and
  /// waits until its boot program (tables, defaults) has run, then reads the
  /// new parameter list.
  LoadResult load(const ProgramData &p, uint32_t bootTimeoutMs = 3000) {
    if (!ping()) return LoadResult::NoAnswer;
    if (version_ < p.minProtocol) return LoadResult::BitstreamTooOld;
    Info inf = info();
    if (p.words > inf.programCapacity) return LoadResult::ProgramTooLarge;
    if (p.descriptorLength > inf.descriptorCapacity) return LoadResult::DescriptorTooLarge;
    if (p.fastWords > inf.memoryCapacity) return LoadResult::MemoryTooLarge;
    if (p.sdramWords && !inf.sdram) return LoadResult::NeedsSdram;
    if (p.inputs > inf.maxInputs || p.outputs > inf.maxOutputs) return LoadResult::TooManyChannels;

    stop_ = true;
    sendControl(false);
    delay(2);  // let a sample in progress finish

    // program: 5 bytes per instruction, in chunks of 100 instructions
    for (uint32_t w = 0; w < p.words; w += 100) {
      uint32_t n = p.words - w < 100 ? p.words - w : 100;
      tb_(kOpWriteProg);
      tp_(w & 0xFF);
      tp_((w >> 8) & 0xFF);
      for (uint32_t i = 0; i < n * 5; i++) tp_(p.code[w * 5 + i]);
      if (!tx_(true)) return LoadResult::NoAnswer;
    }
    // descriptor
    for (uint32_t o = 0; o < p.descriptorLength; o += 256) {
      uint32_t n = p.descriptorLength - o < 256 ? p.descriptorLength - o : 256;
      tb_(kOpWriteDesc);
      tp_(o & 0xFF);
      tp_((o >> 8) & 0xFF);
      for (uint32_t i = 0; i < n; i++) tp_((uint8_t)p.descriptor[o + i]);
      if (!tx_(true)) return LoadResult::NoAnswer;
    }
    // configuration
    uint32_t inc = (uint32_t)(((uint64_t)p.sampleRate * 128ull << 32) / inf.clockHz);
    uint8_t cfg[17] = {
        (uint8_t)p.dspEntry, (uint8_t)(p.dspEntry >> 8),
        (uint8_t)p.bootEntry, (uint8_t)(p.bootEntry >> 8),
        p.inputs, p.outputs, p.parameters,
        (uint8_t)p.sampleRate, (uint8_t)(p.sampleRate >> 8),
        (uint8_t)(p.sampleRate >> 16), (uint8_t)(p.sampleRate >> 24),
        (uint8_t)inc, (uint8_t)(inc >> 8), (uint8_t)(inc >> 16), (uint8_t)(inc >> 24),
        (uint8_t)p.descriptorLength, (uint8_t)(p.descriptorLength >> 8)};
    tb_(kOpConfig);
    for (int i = 0; i < 17; i++) tp_(cfg[i]);
    if (!tx_(true)) return LoadResult::NoAnswer;

    // restart: boot program first
    stop_ = false;
    sendControl(true);
    uint32_t start = millis();
    while (!info().booted) {
      if (millis() - start > bootTimeoutMs) return LoadResult::BootTimeout;
      delay(5);
    }
    return readParameters() ? LoadResult::Ok : LoadResult::BadDescriptor;
  }

  /// Faust program name (from `declare name`), after begin().
  const char *name() const { return name_ ? name_ : ""; }

  int parameterCount() const { return count_; }

  const Parameter &parameter(int index) const { return params_[index]; }

  /// Index of the parameter with this label or path, or -1.
  int findParameter(const char *labelOrPath) const {
    for (int i = 0; i < count_; i++) {
      if (strcmp(params_[i].path, labelOrPath) == 0 ||
          strcmp(params_[i].label(), labelOrPath) == 0)
        return i;
    }
    return -1;
  }

  /// Sets a parameter (clamped to its range). The DSP sees the new value
  /// from the next sample on.
  bool setParameter(int index, float value) {
    if (index < 0 || index >= count_ || params_[index].isOutput()) return false;
    const Parameter &p = params_[index];
    if (p.max > p.min) {
      if (value < p.min) value = p.min;
      if (value > p.max) value = p.max;
    }
    uint32_t bits;
    memcpy(&bits, &value, 4);
    writeWord(p.address, bits);
    return true;
  }

  bool setParameter(const char *labelOrPath, float value) {
    return setParameter(findParameter(labelOrPath), value);
  }

  /// Current value of a parameter or bargraph (NAN on error).
  float getParameter(int index) {
    if (index < 0 || index >= count_) return NAN;
    uint32_t bits;
    if (!readWord(params_[index].address, bits)) return NAN;
    float v;
    memcpy(&v, &bits, 4);
    return v;
  }

  float getParameter(const char *labelOrPath) {
    return getParameter(findParameter(labelOrPath));
  }

  /// Silences the output without stopping the DSP.
  void setMute(bool mute) {
    mute_ = mute;
    sendControl(false);
  }

  /// Switches the onboard amplifier (MAX98357A SD_MODE pin) on or off.
  void setAmplifier(bool on) {
    amp_ = on;
    sendControl(false);
  }

  /// Re-runs the boot program: all parameters back to their defaults and
  /// all DSP state (delay lines, envelopes) cleared.
  void reset() { sendControl(true); }

  /// Raw block RAM access (addresses as in Parameter::address).
  void writeWord(uint32_t address, uint32_t value) {
    tb_(kOpWrite);
    tp_(address & 0xFF);
    tp_((address >> 8) & 0xFF);
    for (int i = 0; i < 4; i++) tp_((value >> (8 * i)) & 0xFF);
    tx_(true);
  }

  bool readWord(uint32_t address, uint32_t &value) {
    // The core answers between two samples, so the reply is 0x00 bytes,
    // then the READY marker and the value: 64 bytes cover a sample period.
    tb_(kOpRead);
    tp_(address & 0xFF);
    tp_((address >> 8) & 0xFF);
    for (int i = 0; i < 64; i++) tp_(0);
    value = 0;
    if (!tx_()) return false;
    for (size_t i = 3; i + 4 < n_; i++) {
      if (buf_[i] == kReady) {
        for (int k = 0; k < 4; k++) value |= (uint32_t)buf_[i + 1 + k] << (8 * k);
        return true;
      }
    }
    return false;
  }

 protected:
  static constexpr uint8_t kOpPing = 0x01, kOpInfo = 0x02, kOpDesc = 0x03,
                           kOpWrite = 0x10, kOpRead = 0x11, kOpControl = 0x20,
                           kOpWriteProg = 0x30, kOpWriteDesc = 0x31, kOpConfig = 0x32;
  static constexpr uint8_t kReady = 0xA5;

  static constexpr size_t kMaxFrame = 512;  // UART frames (uart_bridge.v)
  SPIClass *spi_ = nullptr;
  Stream *stream_ = nullptr;
  uint8_t buf_[kMaxFrame];
  size_t n_ = 0;
  SPISettings settings_;
  int csPin_ = -1;
  uint8_t version_ = 0;
  bool mute_ = false, amp_ = true, stop_ = false;
  Parameter *params_ = nullptr;
  int count_ = 0;
  char *name_ = nullptr;

  // A command is one transaction: build it with tb_/tp_, run it with tx_.
  // Afterwards buf_ holds the reply bytes (same positions for both
  // transports; replies lag the request by one byte, see docs/protocol.md).
  void tb_(uint8_t op) {
    n_ = 0;
    tp_(op);
  }
  void tp_(uint8_t b) {
    if (n_ < kMaxFrame) buf_[n_++] = b;
  }

  bool tx_(bool noReply = false) {
    if (spi_) {
      spi_->beginTransaction(settings_);
      digitalWrite(csPin_, LOW);
      for (size_t i = 0; i < n_; i++) buf_[i] = spi_->transfer(buf_[i]);
      digitalWrite(csPin_, HIGH);
      spi_->endTransaction();
      return true;
    }
    if (!stream_) return false;
    // frame: 0x7E, length (bit 15: no reply), bytes; strictly half-duplex
    while (stream_->available() > 0) stream_->read();
    uint16_t len = (uint16_t)n_ | (noReply ? 0x8000 : 0);
    stream_->write((uint8_t)0x7E);
    stream_->write((uint8_t)(len & 0xFF));
    stream_->write((uint8_t)(len >> 8));
    stream_->write(buf_, n_);
    stream_->flush();
    stream_->setTimeout(200 + n_ / 8);  // ~87 us per byte at 115200
    if (noReply) {
      uint8_t ack = 0;
      return stream_->readBytes(&ack, 1) == 1 && ack == 0x7E;
    }
    return stream_->readBytes(buf_, n_) == n_;
  }

  static uint32_t le32(const uint8_t *b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
  }

  void sendControl(bool reboot) {
    tb_(kOpControl);
    tp_((mute_ ? 1 : 0) | (amp_ ? 2 : 0) | (reboot ? 4 : 0) | (stop_ ? 8 : 0));
    tx_(true);
  }

  void clearParameters() {
    for (int i = 0; i < count_; i++) {
      free(params_[i].path);
      free(params_[i].meta);
    }
    delete[] params_;
    params_ = nullptr;
    count_ = 0;
    free(name_);
    name_ = nullptr;
  }

  static char *dupRange(const char *from, const char *to) {
    size_t n = to - from;
    char *s = (char *)malloc(n + 1);
    if (s) {
      memcpy(s, from, n);
      s[n] = 0;
    }
    return s;
  }

  // Splits `line` at '|' into at most `max` fields (in place).
  static int split(char *line, char **fields, int max) {
    int n = 0;
    char *p = line;
    while (n < max) {
      fields[n++] = p;
      char *bar = strchr(p, '|');
      if (!bar) break;
      *bar = 0;
      p = bar + 1;
    }
    return n;
  }

  static ParameterKind kindOf(const char *s) {
    if (!strcmp(s, "button")) return ParameterKind::Button;
    if (!strcmp(s, "checkbox")) return ParameterKind::Checkbox;
    if (!strcmp(s, "hslider")) return ParameterKind::HSlider;
    if (!strcmp(s, "vslider")) return ParameterKind::VSlider;
    if (!strcmp(s, "nentry")) return ParameterKind::NumEntry;
    if (!strcmp(s, "hbargraph")) return ParameterKind::HBargraph;
    if (!strcmp(s, "vbargraph")) return ParameterKind::VBargraph;
    return ParameterKind::Unknown;
  }

  /// Reads and parses the descriptor (see compiler/Compiler.h, descriptor()).
  bool readParameters() {
    clearParameters();
    Info inf = info();
    uint16_t len = inf.descriptorLength;
    char *text = (char *)malloc(len + 1);
    if (!text) return false;
    for (uint16_t o = 0; o < len; o += 256) {  // in chunks that fit a UART frame
      uint16_t n = len - o < 256 ? len - o : 256;
      tb_(kOpDesc);
      tp_(o & 0xFF);
      tp_(o >> 8);
      tp_(0);  // dummy byte while the descriptor RAM is read
      for (uint16_t i = 0; i < n; i++) tp_(0);
      if (!tx_()) {
        free(text);
        return false;
      }
      memcpy(text + o, buf_ + 4, n);
    }
    text[len] = 0;

    char *fields[9];
    char *line = text;
    int expected = 0;
    bool ok = false;
    while (line && *line) {
      char *nl = strchr(line, '\n');
      if (nl) *nl = 0;
      int n = split(line, fields, 9);
      if (n >= 7 && fields[0][0] == 'F') {
        name_ = dupRange(fields[2], fields[2] + strlen(fields[2]));
        expected = atoi(fields[6]);
        params_ = new Parameter[expected > 0 ? expected : 1];
        ok = true;
      } else if (n >= 8 && fields[0][0] == 'P' && params_ && count_ < expected) {
        Parameter &p = params_[count_++];
        p.kind = kindOf(fields[1]);
        p.address = strtoul(fields[2], nullptr, 10);
        p.init = atof(fields[3]);
        p.min = atof(fields[4]);
        p.max = atof(fields[5]);
        p.step = atof(fields[6]);
        p.path = dupRange(fields[7], fields[7] + strlen(fields[7]));
        const char *m = n >= 9 ? fields[8] : "";
        p.meta = dupRange(m, m + strlen(m));
      }
      line = nl ? nl + 1 : nullptr;
    }
    free(text);
    return ok && count_ == expected;
  }
};

}  // namespace tangnanofaust
