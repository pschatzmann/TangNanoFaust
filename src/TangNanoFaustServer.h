#pragma once
/**
 * Compiling Faust source code over the network for NanoTangFaust.
 *
 * The Faust compiler needs a computer. FaustServerCompiler sends Faust
 * source code to a compile server on your network (`faust2tang --serve
 * 8000`, see docs/faust2tang.md#compile-server) over plain HTTP and loads
 * the answer into the FPGA. It works with any Arduino `Client`: WiFiClient
 * (ESP32, Pico W, ...), EthernetClient, ...
 *
 * Two ways:
 * - compileAndLoad(faust, source): the server compiles everything and
 *   returns the finished program (a few KB). Needs no C++ standard library.
 * - compileAndLoad(faust, source, faustCompiler): the server returns Faust's
 *   bytecode, and the MCU compiles it for the DSP core itself (include
 *   NanoTangFaustCompiler.h for FaustCompiler).
 *
 * @code
 * WiFiClient client;
 * FaustServerCompiler server(client, "http://192.168.1.10:8000");
 * if (!server.compileAndLoad(faust, "process = os.osc(440) * 0.3;"))
 *   Serial.println(server.error());   // e.g. Faust's error message
 * @endcode
 */
#include <Arduino.h>
#include <Client.h>
#include <stdlib.h>
#include <string.h>

#include "NanoTangFaust.h"

namespace nanotangfaust {

class FaustServerCompiler {
 public:
  /// `url`: the compile server, e.g. "http://192.168.1.10:8000" (http only).
  FaustServerCompiler(Client &client, const char *url) : client_(client) { setServer(url); }
  ~FaustServerCompiler() { free(buf_); }
  FaustServerCompiler(const FaustServerCompiler &) = delete;
  FaustServerCompiler &operator=(const FaustServerCompiler &) = delete;

  void setServer(const char *url) {
    if (strncmp(url, "http://", 7) == 0) url += 7;
    size_t n = strcspn(url, ":/");
    if (n >= sizeof(host_)) n = sizeof(host_) - 1;
    memcpy(host_, url, n);
    host_[n] = 0;
    url += strcspn(url, ":/");
    port_ = 80;
    if (*url == ':') port_ = (uint16_t)strtoul(url + 1, (char **)&url, 10);
    snprintf(path_, sizeof(path_), "%s", url);
    size_t len = strlen(path_);
    if (len && path_[len - 1] == '/') path_[len - 1] = 0;
  }

  /// How long to wait for the server (Faust needs a moment for large programs).
  void setTimeout(uint32_t ms) { timeoutMs_ = ms; }
  /// Faster sin/cos (see docs/faust.md).
  void setFastMath(bool on) { fastMath_ = on; }

  /// The server compiles `source` for the connected FPGA (its sample rate,
  /// memory and clock); the result is loaded. False: see error().
  bool compileAndLoad(TangNanoFaust &faust, const char *source) {
    if (!request(faust.info(), source, "program")) return false;
    ProgramData program;
    if (!parseProgramBlob(buf_, len_, program)) return fail("the server's answer is not a program");
    LoadResult r = faust.load(program);
    if (r != LoadResult::Ok) return fail(loadError(r));
    return true;
  }

  /// The server only runs Faust; `mcuCompiler` (a FaustCompiler) compiles
  /// the bytecode on the MCU and loads it. False: see error().
  template <class Compiler>
  bool compileAndLoad(TangNanoFaust &faust, const char *source, Compiler &mcuCompiler) {
    Info info = faust.info();
    if (!request(info, source, "fbc")) return false;
    if (!mcuCompiler.compileAndLoad(faust, (const char *)buf_, info.sampleRate, fastMath_))
      return fail(mcuCompiler.error());
    return true;
  }

  /// Only compiles: out = "program" (parseProgramBlob()), "fbc" (bytecode
  /// text) or "report" (faust2tang's report, text). The answer is in
  /// data()/size(), NUL-terminated. False: see error().
  bool compile(const Info &target, const char *source, const char *out = "program") {
    return request(target, source, out);
  }

  /// Why the last call failed: Faust's or the compiler's error message from
  /// the server, or what went wrong on the way.
  const char *error() const { return error_; }
  /// HTTP status of the last answer (200: ok; 0: no answer).
  int status() const { return status_; }
  /// Cycles per sample of the last compiled program, simulated by the server
  /// (normal / in the sample after a parameter change; 0 for out=fbc).
  uint32_t cycles() const { return cycles_; }
  uint32_t controlCycles() const { return controlCycles_; }
  /// The last answer from the server (valid until the next call).
  const uint8_t *data() const { return buf_; }
  size_t size() const { return len_; }

 protected:
  static const size_t kMaxAnswer = 256 * 1024;
  Client &client_;
  char host_[64] = "";
  char path_[64] = "";
  uint16_t port_ = 80;
  uint32_t timeoutMs_ = 30000;
  bool fastMath_ = false;
  uint8_t *buf_ = nullptr;
  size_t len_ = 0, cap_ = 0;
  int status_ = 0;
  uint32_t cycles_ = 0, controlCycles_ = 0;
  const char *error_ = "";

  bool fail(const char *why) {
    error_ = why;
    return false;
  }

  static const char *loadError(LoadResult r) {
    switch (r) {
      case LoadResult::Ok: return "ok";
      case LoadResult::NoAnswer: return "FPGA not answering";
      case LoadResult::ProgramTooLarge: return "program too large for this bitstream";
      case LoadResult::DescriptorTooLarge: return "too many parameters for this bitstream";
      case LoadResult::MemoryTooLarge: return "not enough block RAM in this bitstream";
      case LoadResult::NeedsSdram: return "program needs SDRAM, bitstream has none";
      case LoadResult::TooManyChannels: return "more inputs or outputs than this bitstream has (TDM: make TDM=1)";
      case LoadResult::BitstreamTooOld: return "bitstream too old for this program (rebuild it)";
      case LoadResult::BootTimeout: return "boot program did not finish";
      case LoadResult::BadDescriptor: return "descriptor could not be read back";
    }
    return "load failed";
  }

  bool append(const uint8_t *p, size_t n) {
    if (len_ + n + 1 > cap_) {
      size_t cap = cap_ ? cap_ : 4096;
      while (cap < len_ + n + 1) cap *= 2;
      if (cap > kMaxAnswer + 1) return false;
      uint8_t *b = (uint8_t *)realloc(buf_, cap);
      if (!b) return false;
      buf_ = b;
      cap_ = cap;
    }
    memcpy(buf_ + len_, p, n);
    len_ += n;
    buf_[len_] = 0;
    return true;
  }

  // Writes everything (clients may accept less than asked per call).
  bool writeAll(const char *p, size_t n) {
    uint32_t start = millis();
    while (n > 0) {
      size_t w = client_.write((const uint8_t *)p, n);
      if (w == 0) {
        if (!client_.connected() || millis() - start > timeoutMs_) return false;
        delay(1);
        continue;
      }
      p += w;
      n -= w;
    }
    return true;
  }

  // Reads one header line (without CR LF). False on timeout.
  bool readLine(char *line, size_t size, uint32_t start) {
    size_t n = 0;
    while (millis() - start < timeoutMs_) {
      if (!client_.available()) {
        if (!client_.connected()) break;
        delay(1);
        continue;
      }
      char c = (char)client_.read();
      if (c == '\n') {
        line[n] = 0;
        return true;
      }
      if (c != '\r' && n + 1 < size) line[n++] = c;
    }
    return false;
  }

  bool request(const Info &target, const char *source, const char *out) {
    len_ = 0;
    status_ = 0;
    cycles_ = controlCycles_ = 0;
    error_ = "";
    if (!client_.connect(host_, port_)) return fail("compile server not reachable");
    char head[384];
    size_t srcLen = strlen(source);
    snprintf(head, sizeof(head),
             "POST %s/compile?out=%s&sr=%lu&fast_words=%lu&sdram=%d&clk_hz=%lu&fast_math=%d "
             "HTTP/1.1\r\nHost: %s\r\nContent-Type: text/plain\r\nContent-Length: %lu\r\n"
             "Connection: close\r\n\r\n",
             path_, out, (unsigned long)target.sampleRate, (unsigned long)target.memoryCapacity,
             target.sdram ? 1 : 0, (unsigned long)target.clockHz, fastMath_ ? 1 : 0, host_,
             (unsigned long)srcLen);
    if (!writeAll(head, strlen(head)) || !writeAll(source, srcLen)) {
      client_.stop();
      return fail("sending to the compile server failed");
    }

    // status line and headers
    uint32_t start = millis();
    char line[128];
    long contentLength = -1;
    if (!readLine(line, sizeof(line), start) || strncmp(line, "HTTP/1.", 7) != 0) {
      client_.stop();
      return fail("no answer from the compile server");
    }
    status_ = atoi(line + 9);
    while (readLine(line, sizeof(line), start) && line[0]) {
      if (strncasecmp(line, "Content-Length:", 15) == 0) contentLength = atol(line + 15);
      else if (strncasecmp(line, "X-NTF-Cycles:", 13) == 0) cycles_ = strtoul(line + 13, nullptr, 10);
      else if (strncasecmp(line, "X-NTF-Control-Cycles:", 21) == 0)
        controlCycles_ = strtoul(line + 21, nullptr, 10);
    }

    // body
    uint8_t chunk[256];
    while ((contentLength < 0 || (long)len_ < contentLength) && millis() - start < timeoutMs_) {
      int n = client_.available();
      if (n <= 0) {
        if (!client_.connected()) break;
        delay(1);
        continue;
      }
      n = client_.read(chunk, n < (int)sizeof(chunk) ? n : (int)sizeof(chunk));
      if (n > 0 && !append(chunk, (size_t)n)) {
        client_.stop();
        return fail("the server's answer does not fit into memory");
      }
    }
    client_.stop();
    if (!buf_ && !append(chunk, 0)) return fail("out of memory");
    if (contentLength >= 0 && (long)len_ < contentLength)
      return fail("the compile server's answer is incomplete");
    if (status_ != 200) return fail(len_ ? (const char *)buf_ : "the compile server reported an error");
    return true;
  }
};

}  // namespace nanotangfaust
