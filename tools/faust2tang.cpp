// faust2tang: compile a Faust .dsp for the NanoTangFaust FPGA DSP core.
//
//   faust2tang my.dsp -o build/
//   faust2tang --serve 8000          compile server for microcontrollers
//
// Runs `faust -lang interp -double` (the double-precision bytecode carries
// 16 significant digits per constant; the compiler rounds them to float32
// itself), compiles the bytecode for the core with the header-only
// compiler in src/ (the same code that runs on a microcontroller), and
// writes the memory images, Verilog config, C++ headers and a report.
//
// Build: make -C tools   (g++ -std=c++17, no other dependencies)
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <errno.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "NanoTangFaust/compiler/Compiler.h"
#include "NanoTangFaust/compiler/Isa.h"
#include "NanoTangFaust/compiler/Output.h"

using namespace nanotangfaust::compiler;

// ---------------------------------------------------------------- USB serial
// The FPGA's UART command port (gateware/rtl/uart_bridge.v) on the board's
// USB-UART bridge: frames of 0x7E, length u16 (bit 15: no reply), the bytes
// an SPI host would send; the reply is the bytes it would receive.
class Board {
 public:
  bool open(const std::string &dev) {
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) return false;
    termios t{};
    tcgetattr(fd_, &t);
    cfmakeraw(&t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(CSTOPB | CRTSCTS);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 1;
    tcsetattr(fd_, TCSANOW, &t);
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  /// One transaction; `buf` is replaced by the reply.
  bool xfer(std::vector<uint8_t> &buf, bool noReply = false) {
    tcflush(fd_, TCIFLUSH);
    uint16_t len = (uint16_t)buf.size() | (noReply ? 0x8000 : 0);
    uint8_t hdr[3] = {0x7E, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    if (write(fd_, hdr, 3) != 3 || write(fd_, buf.data(), buf.size()) != (ssize_t)buf.size())
      return false;
    tcdrain(fd_);
    size_t want = noReply ? 1 : buf.size();
    std::vector<uint8_t> in;
    long deadline = nowMs() + 500 + (long)want / 8;
    while (in.size() < want && nowMs() < deadline) {
      uint8_t tmp[512];
      ssize_t n = read(fd_, tmp, std::min(sizeof(tmp), want - in.size()));
      if (n > 0) in.insert(in.end(), tmp, tmp + n);
    }
    if (in.size() < want) return false;
    if (noReply) return in[0] == 0x7E;
    buf = in;
    return true;
  }

  bool ping(int &version) {
    std::vector<uint8_t> b = {0x01, 0, 0, 0, 0, 0};
    if (!xfer(b) || memcmp(b.data() + 1, "FAUS", 4) != 0) return false;
    version = version_ = b[5];
    return true;
  }

  /// INFO, always 23 bytes: [21] [22] are the bitstream's input/output
  /// counts (protocol 4; older bitstreams had 2 in / 8 out).
  bool info(std::vector<uint8_t> &out) {
    std::vector<uint8_t> b(version_ >= 4 ? 24 : 22, 0);
    b[0] = 0x02;
    if (!xfer(b)) return false;
    out.assign(b.begin() + 1, b.end());
    if (version_ < 4) {
      out[3] |= 64;  // had TDM
      out.push_back(2);
      out.push_back(8);
    }
    return true;
  }

  /// Live core state: state, npc, opcode, sp, T, cycle counter.
  bool debug(std::vector<uint8_t> &out) {
    std::vector<uint8_t> b(14, 0);
    b[0] = 0x04;
    if (!xfer(b)) return false;
    out.assign(b.begin() + 1, b.end());
    return true;
  }

  bool control(uint8_t flags) {
    std::vector<uint8_t> b = {0x20, flags};
    return xfer(b, true);
  }

  bool descriptor(std::string &text) {
    std::vector<uint8_t> inf;
    if (!info(inf)) return false;
    int len = inf[8] | (inf[9] << 8);
    text.clear();
    for (int o = 0; o < len; o += 256) {
      int n = std::min(256, len - o);
      std::vector<uint8_t> b(4 + n, 0);
      b[0] = 0x03;
      b[1] = o & 0xFF;
      b[2] = o >> 8;
      if (!xfer(b)) return false;
      text.append((const char *)b.data() + 4, n);
    }
    return true;
  }

  bool writeWord(uint32_t addr, uint32_t v) {
    std::vector<uint8_t> b = {0x10, (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)v,
                              (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
    return xfer(b, true);
  }

  bool readWord(uint32_t addr, uint32_t &v) {
    std::vector<uint8_t> b(3 + 64, 0);
    b[0] = 0x11;
    b[1] = addr & 0xFF;
    b[2] = addr >> 8;
    if (!xfer(b)) return false;
    for (size_t i = 3; i + 4 < b.size(); i++)
      if (b[i] == 0xA5) {
        v = b[i + 1] | (b[i + 2] << 8) | (b[i + 3] << 16) | ((uint32_t)b[i + 4] << 24);
        return true;
      }
    return false;
  }

  /// Uploads a compiled program and waits until it has booted.
  bool load(const ProgramImage &img, std::string &error) {
    std::vector<uint8_t> inf;
    if (!info(inf)) return error = "no answer", false;
    // INFO: [0] in [1] out [2] params [3] flags [4..7] sample rate [8..9] descriptor length
    // [10..13] cycles [14..17] clock [18..20] log2 program/RAM/descriptor capacity
    // [21..22] input/output channels
    if (img.program.size() > (1u << inf[18])) return error = "program too large for the bitstream", false;
    if (img.fastWords > (1u << inf[19])) return error = "not enough block RAM in the bitstream", false;
    if (img.descriptor.size() > (1u << inf[20])) return error = "descriptor too large", false;
    if (img.sdramWords && !(inf[3] & 8)) return error = "the program needs SDRAM, the bitstream has none", false;
    if (img.inputs > inf[21] || img.outputs > inf[22])
      return error = "more inputs or outputs than the bitstream has (" + std::to_string(inf[21]) +
                     " in, " + std::to_string(inf[22]) + " out; TDM: make TDM=1)", false;
    uint32_t clk = inf[14] | (inf[15] << 8) | (inf[16] << 16) | ((uint32_t)inf[17] << 24);
    if (!control(0x0A)) return error = "stop failed", false;  // amp on + stop
    usleep(2000);
    for (size_t w = 0; w < img.program.size(); w += 100) {
      size_t n = std::min<size_t>(100, img.program.size() - w);
      std::vector<uint8_t> b = {0x30, (uint8_t)w, (uint8_t)(w >> 8)};
      for (size_t i = 0; i < n; i++)
        for (int k = 0; k < 5; k++) b.push_back((uint8_t)(img.program[w + i] >> (8 * k)));
      if (!xfer(b, true)) return error = "program upload failed", false;
    }
    for (size_t o = 0; o < img.descriptor.size(); o += 256) {
      size_t n = std::min<size_t>(256, img.descriptor.size() - o);
      std::vector<uint8_t> b = {0x31, (uint8_t)o, (uint8_t)(o >> 8)};
      b.insert(b.end(), img.descriptor.begin() + o, img.descriptor.begin() + o + n);
      if (!xfer(b, true)) return error = "descriptor upload failed", false;
    }
    uint32_t inc = (uint32_t)(((uint64_t)img.sampleRate * 128ull << 32) / clk);
    uint32_t sr = img.sampleRate, dl = (uint32_t)img.descriptor.size();
    std::vector<uint8_t> c = {0x32, (uint8_t)img.dspEntry, (uint8_t)(img.dspEntry >> 8),
                              (uint8_t)img.bootEntry, (uint8_t)(img.bootEntry >> 8),
                              (uint8_t)img.inputs, (uint8_t)img.outputs, (uint8_t)img.params.size(),
                              (uint8_t)sr, (uint8_t)(sr >> 8), (uint8_t)(sr >> 16), (uint8_t)(sr >> 24),
                              (uint8_t)inc, (uint8_t)(inc >> 8), (uint8_t)(inc >> 16), (uint8_t)(inc >> 24),
                              (uint8_t)dl, (uint8_t)(dl >> 8)};
    if (!xfer(c, true)) return error = "config failed", false;
    if (!control(0x06)) return error = "reboot failed", false;  // amp on + reboot
    long deadline = nowMs() + 3000;
    while (nowMs() < deadline) {
      if (info(inf) && (inf[3] & 1)) return true;
      usleep(5000);
    }
    return error = "boot program did not finish", false;
  }

 private:
  int fd_ = -1;
  int version_ = 0;
  static long nowMs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
  }
};

struct BoardParam {
  std::string kind, path, label, meta;
  uint32_t addr;
  double init, min, max;
};

static std::vector<BoardParam> parseDescriptor(const std::string &text, std::string &name) {
  std::vector<BoardParam> out;
  size_t p = 0;
  while (p < text.size()) {
    size_t e = text.find('\n', p);
    if (e == std::string::npos) e = text.size();
    std::string line = text.substr(p, e - p);
    p = e + 1;
    std::vector<std::string> f;
    size_t q = 0;
    while (true) {
      size_t bar = line.find('|', q);
      f.push_back(line.substr(q, bar == std::string::npos ? std::string::npos : bar - q));
      if (bar == std::string::npos) break;
      q = bar + 1;
    }
    if (f.size() >= 7 && f[0] == "F") name = f[2];
    if (f.size() >= 8 && f[0] == "P") {
      BoardParam bp;
      bp.kind = f[1];
      bp.addr = (uint32_t)strtoul(f[2].c_str(), nullptr, 10);
      bp.init = atof(f[3].c_str());
      bp.min = atof(f[4].c_str());
      bp.max = atof(f[5].c_str());
      bp.path = f[7];
      bp.label = bp.path.substr(bp.path.find_last_of('/') + 1);
      bp.meta = f.size() >= 9 ? f[8] : "";
      out.push_back(bp);
    }
  }
  return out;
}

static const char *kUsage =
    "usage: faust2tang [options] my.dsp|my.fbc\n"
    "  -o DIR              output directory (default: build)\n"
    "  --sample-rate N     audio sample rate (default 48000)\n"
    "  --clk-hz N          core clock for the cycle budget (default 48000000)\n"
    "  --fast-words N      block RAM words for the heap (default 16384)\n"
    "  --no-sdram          fail instead of placing large arrays in SDRAM\n"
    "  --no-fuse           no fused instructions (debugging)\n"
    "  --fast-math         faster sin/cos (1.9e-7 error for moderate arguments)\n"
    "  --faust PATH        faust executable (default: $FAUST or faust)\n"
    "  -I DIR              extra Faust library directory\n"
    "  --faust-arg ARG     pass ARG to faust (repeatable), e.g. --faust-arg -mcd --faust-arg 32\n"
    "  --strict-budget     fail if a sample needs more cycles than available\n"
    "  --generic           size the bitstream to load any program at runtime\n"
    "                      (4096 instructions, 2048 descriptor bytes, 2 in / 2 out, SDRAM)\n"
    "  --tdm               build the TDM output into the bitstream (generic: 8 outputs)\n"
    "  --sketch            only write <name>_fbc.h next to the .dsp: the bytecode for\n"
    "                      compiling on the microcontroller (FaustCompiler)\n"
    "  --verilog-opcodes F write the Verilog opcode header F and exit\n"
    "\n"
    "control a board over its USB serial port (115200 baud; no .dsp needed for these):\n"
    "  --port DEV          the board's second USB serial port, e.g. /dev/ttyUSB1\n"
    "  --load              upload the compiled program and start it\n"
    "  --info              show the board's status\n"
    "  --list              list the running program's parameters\n"
    "  --set LABEL=VALUE   set a parameter (repeatable)\n"
    "  --get LABEL         print a parameter's value (repeatable)\n"
    "  --debug             show the DSP core's live state (8 snapshots)\n"
    "  --mute              silence the output and switch the amplifier off\n"
    "  --unmute            output and amplifier on again\n"
    "\n"
    "compile server (e.g. for an ESP32 over WiFi; trusted networks only):\n"
    "  --serve PORT        answer POST /compile with the compiled program\n"
    "  --bind ADDR         listen address (default 0.0.0.0, all interfaces)\n";

static bool writeFile(const std::string &path, const std::string &text) {
  std::ofstream f(path, std::ios::binary);
  f << text;
  if (!f) {
    fprintf(stderr, "faust2tang: cannot write %s\n", path.c_str());
    return false;
  }
  return true;
}

static std::string quote(const std::string &s) {
  std::string r = "'";
  for (char c : s) r += c == '\'' ? std::string("'\\''") : std::string(1, c);
  return r + "'";
}

static bool endsWith(const std::string &s, const char *suffix) {
  size_t n = strlen(suffix);
  return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
}

// --port actions: load the compiled program, show status, list/set/get parameters.
static int boardActions(const std::string &port, const ProgramImage *load, bool info, bool list,
                        bool debug, int mute, const std::vector<std::string> &sets,
                        const std::vector<std::string> &gets) {
  Board b;
  int version = 0;
  if (!b.open(port)) {
    fprintf(stderr, "faust2tang: cannot open %s\n", port.c_str());
    return 1;
  }
  if (!b.ping(version)) {
    fprintf(stderr, "faust2tang: no NanoTangFaust bitstream answering on %s (it is the board's "
            "second USB serial port, 115200 baud)\n", port.c_str());
    return 1;
  }
  if (load) {
    std::string error;
    if (version < load->minProtocol) {
      fprintf(stderr, "faust2tang: the bitstream (protocol %d) is too old for this program\n", version);
      return 1;
    }
    if (!b.load(*load, error)) {
      fprintf(stderr, "faust2tang: load failed: %s\n", error.c_str());
      return 1;
    }
    printf("loaded '%s' on %s\n", load->name.c_str(), port.c_str());
  }
  if (mute) {  // CONTROL: bit 0 mute, bit 1 amplifier on
    if (!b.control(mute > 0 ? 0x01 : 0x02)) {
      fprintf(stderr, "faust2tang: CONTROL failed\n");
      return 1;
    }
    printf("%s\n", mute > 0 ? "muted, amplifier off" : "unmuted, amplifier on");
  }
  std::string desc, name;
  std::vector<BoardParam> params;
  if (list || !sets.empty() || !gets.empty()) {
    if (!b.descriptor(desc)) {
      fprintf(stderr, "faust2tang: cannot read the parameter list\n");
      return 1;
    }
    params = parseDescriptor(desc, name);
  }
  auto find = [&](const std::string &label) -> const BoardParam * {
    for (auto &p : params)
      if (p.label == label || p.path == label || p.path == "/" + label) return &p;
    return nullptr;
  };
  for (const std::string &kv : sets) {
    size_t eq = kv.find('=');
    const BoardParam *p = eq == std::string::npos ? nullptr : find(kv.substr(0, eq));
    if (!p) {
      fprintf(stderr, "faust2tang: unknown parameter in --set %s\n", kv.c_str());
      return 1;
    }
    double v = atof(kv.c_str() + eq + 1);
    if (p->max > p->min) v = std::max(p->min, std::min(p->max, v));
    float f = (float)v;
    uint32_t bits;
    memcpy(&bits, &f, 4);
    b.writeWord(p->addr, bits);
  }
  for (const std::string &label : gets) {
    const BoardParam *p = find(label);
    uint32_t bits;
    if (!p || !b.readWord(p->addr, bits)) {
      fprintf(stderr, "faust2tang: cannot read %s\n", label.c_str());
      return 1;
    }
    float f;
    memcpy(&f, &bits, 4);
    printf("%s = %.9g (0x%08x)\n", label.c_str(), f, bits);
  }
  if (list) {
    printf("program '%s':\n", name.c_str());
    for (auto &p : params)
      printf("  %-9s %-24s [%g .. %g] init %g %s\n", p.kind.c_str(), p.path.c_str(), p.min, p.max,
             p.init, p.meta.c_str());
  }
  if (debug) {
    static const char *states[] = {"IDLE", "EXEC", "MEMRD", "SD", "FPU", "MUL", "DIV", "HOST"};
    for (int k = 0; k < 8; k++) {
      std::vector<uint8_t> d;
      if (!b.debug(d)) break;
      // [0] state [1..2] npc [3] opcode [4] sp [5..8] T [9..12] cycle counter
      uint32_t T = d[5] | (d[6] << 8) | (d[7] << 16) | ((uint32_t)d[8] << 24);
      uint32_t cyc = d[9] | (d[10] << 8) | (d[11] << 16) | ((uint32_t)d[12] << 24);
      printf("state %-5s npc %4u opcode 0x%02x sp %2u T 0x%08x cycles %u\n", states[d[0] & 7],
             d[1] | (d[2] << 8), d[3], d[4], T, cyc);
    }
  }
  if (info) {
    std::vector<uint8_t> i;
    if (b.info(i)) {
      uint32_t sr = i[4] | (i[5] << 8) | (i[6] << 16) | ((uint32_t)i[7] << 24);
      uint32_t cyc = i[10] | (i[11] << 8) | (i[12] << 16) | ((uint32_t)i[13] << 24);
      uint32_t clk = i[14] | (i[15] << 8) | (i[16] << 16) | ((uint32_t)i[17] << 24);
      printf("protocol %d, %d in, %d out, %d parameters, %u Hz%s%s%s%s, load %.0f%% (%u of %u cycles)\n",
             version, i[0], i[1], i[2], sr, (i[3] & 1) ? ", running" : ", not booted",
             (i[3] & 2) ? ", OVERRUN" : "", (i[3] & 8) ? ", SDRAM" : "",
             (i[3] & 4) ? ", muted" : (i[3] & 32) ? ", TDM clock" : "",
             sr ? 100.0 * cyc / (clk / sr) : 0.0, cyc, sr ? clk / sr : 0);
      printf("bitstream: %g MHz, %u instructions, %u words block RAM, %d in / %d out%s\n",
             clk / 1e6, 1u << i[18], 1u << i[19], i[21], i[22], (i[3] & 64) ? ", TDM" : "");
    }
  }
  return 0;
}

// Boot and per-sample cycles on the simulator, the worst case over each
// parameter's range. False if the program faults.
static bool simulate(const ProgramImage &img, long &boot, long &control, long &worst,
                     std::string &error) {
  Machine m(img.program, img.fastWords, img.inputs, img.outputs);
  boot = m.run(img.bootEntry, 200000000);
  worst = control = 0;
  for (int n = 0; n < 256 && boot >= 0; n++) {
    for (auto &x : m.inputs) x = (n / 8) % 2 ? 0x3F000000u : 0xBF000000u;
    long c = m.run(n % 64 == 0 ? 0 : img.dspEntry);  // control block now and then
    if (c < 0) {
      boot = -1;
      break;
    }
    if (n % 64 == 0) {
      if (c > control) control = c;
    } else if (c > worst) {
      worst = c;
    }
  }
  // Math routines take longer for some arguments (range reduction), so try
  // every parameter across its range: the control block right after the
  // change, then the following samples (smoothed parameters keep changing).
  for (const Param &p : img.params) {
    if (boot < 0 || p.kind.find("bargraph") != std::string::npos || !(p.max > p.min)) continue;
    for (int k = 0; k <= 4 && boot >= 0; k++) {
      float v = (float)(p.min + (p.max - p.min) * k / 4.0);
      uint32_t bits;
      memcpy(&bits, &v, 4);
      m.heap.write(p.addr, bits);
      for (int n = 0; n < 33; n++) {
        long c = m.run(n == 0 ? 0 : img.dspEntry);
        if (c < 0) {
          boot = -1;
          break;
        }
        if (n == 0) control = std::max(control, c);
        else worst = std::max(worst, c);
      }
    }
    float init = (float)p.init;
    uint32_t bits;
    memcpy(&bits, &init, 4);
    m.heap.write(p.addr, bits);
  }
  if (boot < 0) error = "simulation failed: " + m.error;
  return boot >= 0;
}

static std::string readFile(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// ---------------------------------------------------------------- compile server
// A minimal HTTP/1.1 server, one request at a time:
//   POST /compile?out=program|fbc|report&sr=48000&fast_words=16384&sdram=1
//                &fast_math=0&clk_hz=48000000&name=dsp     body: Faust source
// out=program (default): the program blob (ProgramBlob.h) for
// TangNanoFaust::load(); out=fbc: the bytecode for FaustCompiler; out=report:
// faust2tang's report. Errors: status 400 with Faust's or the compiler's
// message as text. Headers X-NTF-Cycles / X-NTF-Control-Cycles carry the
// simulated cycles per sample (normal / after a parameter change).
struct ServerConfig {
  std::string faust;
  std::vector<std::string> includes, faustArgs;
  CompileOptions opt;
  uint32_t clkHz = 48000000;
};

static const char *kServerHelp =
    "NanoTangFaust compile server (faust2tang --serve)\n\n"
    "POST /compile with the Faust source as the body. Query parameters:\n"
    "  out=program|fbc|report  program blob for TangNanoFaust::load() (default),\n"
    "                          bytecode for FaustCompiler, or the text report\n"
    "  sr=48000                sample rate\n"
    "  fast_words=16384        block RAM words of the bitstream (Info::memoryCapacity)\n"
    "  sdram=1                 the bitstream has SDRAM (Info::sdram)\n"
    "  fast_math=0             faster sin/cos\n"
    "  clk_hz=48000000         FPGA clock, for the report\n"
    "  name=dsp                program name if the source declares none\n\n"
    "Example: curl --data-binary @synth.dsp 'http://localhost:8000/compile?out=report'\n";

static std::string queryValue(const std::string &query, const char *key, const char *def) {
  size_t p = 0, n = strlen(key);
  while (p < query.size()) {
    size_t e = query.find('&', p);
    if (e == std::string::npos) e = query.size();
    if (query.compare(p, n, key) == 0 && p + n < e && query[p + n] == '=')
      return query.substr(p + n + 1, e - p - n - 1);
    p = e + 1;
  }
  return def;
}

static bool sendAll(int fd, const std::string &data) {
  size_t off = 0;
  while (off < data.size()) {
    ssize_t n = send(fd, data.data() + off, data.size() - off, 0);
    if (n <= 0) return false;
    off += (size_t)n;
  }
  return true;
}

static void respond(int fd, int status, const std::string &type, const std::string &body,
                    const std::string &extraHeaders = "") {
  const char *reason = status == 200 ? "OK" : status == 400 ? "Bad Request"
                       : status == 404 ? "Not Found" : status == 413 ? "Payload Too Large"
                                                                     : "Error";
  char head[256];
  snprintf(head, sizeof(head), "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n",
           status, reason, type.c_str(), body.size());
  sendAll(fd, std::string(head) + extraHeaders + "Connection: close\r\n\r\n" + body);
}

// Compiles Faust source text. Returns the HTTP status; `body`/`type` is the answer.
static int serveCompile(const ServerConfig &cfg, const std::string &query, const std::string &src,
                        std::string &body, std::string &type, std::string &headers) {
  type = "text/plain; charset=utf-8";
  CompileOptions opt = cfg.opt;
  opt.sampleRate = (uint32_t)atol(queryValue(query, "sr", std::to_string(opt.sampleRate).c_str()).c_str());
  opt.fastWords = (uint32_t)atol(queryValue(query, "fast_words", std::to_string(opt.fastWords).c_str()).c_str());
  opt.allowSdram = queryValue(query, "sdram", opt.allowSdram ? "1" : "0") != "0";
  opt.fastMath = queryValue(query, "fast_math", opt.fastMath ? "1" : "0") != "0";
  uint32_t clkHz = (uint32_t)atol(queryValue(query, "clk_hz", std::to_string(cfg.clkHz).c_str()).c_str());
  std::string out = queryValue(query, "out", "program");
  std::string name = output::lower(output::ident(queryValue(query, "name", "dsp")));
  if (opt.sampleRate < 1000 || opt.sampleRate > 192000 || opt.fastWords < 256 || clkHz == 0) {
    body = "bad sr, fast_words or clk_hz\n";
    return 400;
  }
  if (out != "program" && out != "fbc" && out != "report") {
    body = "out must be program, fbc or report\n";
    return 400;
  }

  char dirTemplate[] = "/tmp/faust2tang-XXXXXX";
  if (!mkdtemp(dirTemplate)) {
    body = "cannot create a temporary directory\n";
    return 500;
  }
  std::string dir = dirTemplate, dspPath = dir + "/" + name + ".dsp";
  std::string fbcPath = dir + "/dsp.fbc", errPath = dir + "/faust.err";
  writeFile(dspPath, src);
  std::string cmd = quote(cfg.faust);
  for (auto &inc : cfg.includes) cmd += " -I " + quote(inc);
  for (auto &fa : cfg.faustArgs) cmd += " " + quote(fa);
  cmd += " -lang interp -double -o " + quote(fbcPath) + " " + quote(dspPath) + " 2> " + quote(errPath);
  int rc = system(cmd.c_str());
  std::string fbc = readFile(fbcPath), ferr = readFile(errPath);
  remove(dspPath.c_str());
  remove(fbcPath.c_str());
  remove(errPath.c_str());
  rmdir(dir.c_str());
  if (rc != 0 || fbc.empty()) {
    // Faust names the temporary file in its messages; show the name only.
    for (size_t p; (p = ferr.find(dir + "/")) != std::string::npos;) ferr.erase(p, dir.size() + 1);
    body = ferr.empty() ? "faust failed\n" : ferr;
    return 400;
  }
  if (out == "fbc") {
    body = output::stripMeta(fbc);
    return 200;
  }

  FbcProgram prog;
  FbcParser parser;
  ProgramImage img;
  Compiler compiler;
  std::string error;
  long boot, control, worst;
  if (!parser.parse(fbc.c_str(), prog, error) || !compiler.compile(prog, opt, img, error) ||
      !simulate(img, boot, control, worst, error)) {
    body = error + "\n";
    return 400;
  }
  headers = "X-NTF-Cycles: " + std::to_string(worst) + "\r\nX-NTF-Control-Cycles: " +
            std::to_string(control) + "\r\n";
  if (out == "report") {
    body = output::report(img, compiler.layout(), boot, control, worst, clkHz);
    return 200;
  }
  body = output::programBlob(img);
  type = "application/octet-stream";
  return 200;
}

static void serveConnection(const ServerConfig &cfg, int fd) {
  timeval tv{10, 0};  // a stalled client must not block the server
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  std::string req;
  char buf[4096];
  size_t headerEnd;
  while ((headerEnd = req.find("\r\n\r\n")) == std::string::npos) {
    ssize_t n = recv(fd, buf, sizeof(buf), 0);
    if (n <= 0 || req.size() > 16384) return;
    req.append(buf, (size_t)n);
  }
  std::string head = req.substr(0, headerEnd), lower = head;
  for (char &c : lower) c = (char)tolower((unsigned char)c);
  std::string body = req.substr(headerEnd + 4);
  size_t sp1 = head.find(' '), sp2 = head.find(' ', sp1 + 1);
  if (sp1 == std::string::npos || sp2 == std::string::npos) return;
  std::string method = head.substr(0, sp1), target = head.substr(sp1 + 1, sp2 - sp1 - 1);
  std::string path = target.substr(0, target.find('?'));
  std::string query = target.find('?') == std::string::npos ? "" : target.substr(target.find('?') + 1);
  size_t len = 0, cl = lower.find("\r\ncontent-length:");
  if (cl != std::string::npos) len = (size_t)atol(lower.c_str() + cl + 17);
  const size_t kMaxSource = 1 << 20;
  if (len > kMaxSource) {
    respond(fd, 413, "text/plain", "source larger than 1 MB\n");
    return;
  }
  if (lower.find("\r\nexpect: 100-continue") != std::string::npos)
    sendAll(fd, "HTTP/1.1 100 Continue\r\n\r\n");
  while (body.size() < len) {
    ssize_t n = recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) return;
    body.append(buf, (size_t)n);
  }
  body.resize(len);

  if (method == "GET" && (path == "/" || path == "/compile")) {
    respond(fd, 200, "text/plain; charset=utf-8", kServerHelp);
    return;
  }
  if (method != "POST" || path != "/compile") {
    respond(fd, 404, "text/plain", "not found; GET / for help\n");
    return;
  }
  std::string answer, type, headers;
  int status = serveCompile(cfg, query, body, answer, type, headers);
  respond(fd, status, type, answer, headers);
  printf("POST %s: %zu bytes of source -> %d, %zu bytes\n", target.c_str(), body.size(), status,
         answer.size());
  fflush(stdout);
}

static int serve(const ServerConfig &cfg, const std::string &bindAddr, int port) {
  signal(SIGPIPE, SIG_IGN);
  int s = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons((uint16_t)port);
  if (inet_pton(AF_INET, bindAddr.c_str(), &a.sin_addr) != 1) {
    fprintf(stderr, "faust2tang: bad --bind address %s\n", bindAddr.c_str());
    return 2;
  }
  if (s < 0 || bind(s, (sockaddr *)&a, sizeof(a)) != 0 || listen(s, 8) != 0) {
    fprintf(stderr, "faust2tang: cannot listen on %s:%d: %s\n", bindAddr.c_str(), port,
            strerror(errno));
    return 1;
  }
  printf("faust2tang: compile server on http://%s:%d/compile (Ctrl-C to stop)\n", bindAddr.c_str(),
         port);
  fflush(stdout);
  for (;;) {
    int c = accept(s, nullptr, nullptr);
    if (c < 0) continue;
    serveConnection(cfg, c);
    close(c);
  }
}

int main(int argc, char **argv) {
  std::string out = "build", dsp, faust = getenv("FAUST") ? getenv("FAUST") : "faust";
  std::vector<std::string> includes, faustArgs;
  CompileOptions opt;
  uint32_t clkHz = 48000000;
  bool strict = false, generic = false, sketch = false, tdm = false;
  std::string port;
  bool doLoad = false, doInfo = false, doList = false, doDebug = false;
  int doMute = 0;  // 1 mute, -1 unmute
  std::vector<std::string> sets, gets;
  int servePort = 0;
  std::string bindAddr = "0.0.0.0";

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        fprintf(stderr, "faust2tang: %s needs a value\n%s", a.c_str(), kUsage);
        exit(2);
      }
      return argv[++i];
    };
    if (a == "-o" || a == "--out") out = next();
    else if (a == "--sample-rate") opt.sampleRate = (uint32_t)atol(next().c_str());
    else if (a == "--clk-hz") clkHz = (uint32_t)atol(next().c_str());
    else if (a == "--fast-words") opt.fastWords = (uint32_t)atol(next().c_str());
    else if (a == "--no-sdram") opt.allowSdram = false;
    else if (a == "--no-fuse") opt.fuse = false;
    else if (a == "--fast-math") opt.fastMath = true;
    else if (a == "--faust") faust = next();
    else if (a == "-I") includes.push_back(next());
    else if (a == "--faust-arg") faustArgs.push_back(next());
    else if (a == "--strict-budget") strict = true;
    else if (a == "--generic") generic = true;
    else if (a == "--tdm") tdm = true;
    else if (a == "--sketch") sketch = true;
    else if (a == "--port") port = next();
    else if (a == "--load") doLoad = true;
    else if (a == "--info") doInfo = true;
    else if (a == "--list") doList = true;
    else if (a == "--debug") doDebug = true;
    else if (a == "--mute") doMute = 1;
    else if (a == "--unmute") doMute = -1;
    else if (a == "--set") sets.push_back(next());
    else if (a == "--get") gets.push_back(next());
    else if (a == "--serve") servePort = atoi(next().c_str());
    else if (a == "--bind") bindAddr = next();
    else if (a == "--verilog-opcodes") return writeFile(next(), output::verilogOpcodes()) ? 0 : 1;
    else if (a == "-h" || a == "--help") {
      printf("%s", kUsage);
      return 0;
    } else if (!a.empty() && a[0] == '-') {
      fprintf(stderr, "faust2tang: unknown option %s\n%s", a.c_str(), kUsage);
      return 2;
    } else dsp = a;
  }
  if (servePort > 0) {
    ServerConfig cfg;
    cfg.faust = faust;
    cfg.includes = includes;
    cfg.faustArgs = faustArgs;
    cfg.opt = opt;
    cfg.clkHz = clkHz;
    return serve(cfg, bindAddr, servePort);
  }
  bool boardOnly = dsp.empty() && !port.empty();
  if (dsp.empty() && !boardOnly) {
    fprintf(stderr, "%s", kUsage);
    return 2;
  }
  if (doLoad && dsp.empty()) {
    fprintf(stderr, "faust2tang: --load needs a .dsp or .fbc to compile\n");
    return 2;
  }
  if (!port.empty() && !doLoad && !doInfo && !doList && !doDebug && !doMute && sets.empty() && gets.empty())
    doInfo = true;
  ProgramImage img;

  if (!boardOnly) {
    if (sketch) {  // bytecode next to the .dsp, nothing else
      size_t slash = dsp.find_last_of('/');
      out = slash == std::string::npos ? "." : dsp.substr(0, slash);
    }
    if (system(("mkdir -p " + quote(out)).c_str()) != 0) {
      fprintf(stderr, "faust2tang: cannot create %s\n", out.c_str());
      return 1;
    }
    std::string fbcPath = dsp;
    if (!endsWith(dsp, ".fbc")) {
      fbcPath = out + "/dsp.fbc";
      std::string cmd = quote(faust);
      for (auto &inc : includes) cmd += " -I " + quote(inc);
      for (auto &fa : faustArgs) cmd += " " + quote(fa);
      cmd += " -lang interp -double -o " + quote(fbcPath) + " " + quote(dsp);
      if (system(cmd.c_str()) != 0) {
        fprintf(stderr, "faust2tang: '%s' failed -- is Faust (>= 2.70) installed? "
                "(--faust /path/to/faust)\n", cmd.c_str());
        return 1;
      }
    }
    std::ifstream in(fbcPath);
    if (!in) {
      fprintf(stderr, "faust2tang: cannot read %s\n", fbcPath.c_str());
      return 1;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    std::string text = ss.str();

    if (sketch) {
      std::ifstream src(dsp);
      std::stringstream sss;
      sss << src.rdbuf();
      std::string base = dsp.substr(dsp.find_last_of('/') + 1);
      base = base.substr(0, base.find_last_of('.'));
      std::string path = out + "/" + output::lower(output::ident(base)) + "_fbc.h";
      remove(fbcPath.c_str());
      if (!writeFile(path, output::fbcHeader(base, text, sss.str()))) return 1;
      printf("wrote %s (%u bytes of bytecode)\n", path.c_str(),
             (unsigned)output::stripMeta(text).size());
      return 0;
    }

    FbcProgram prog;
    FbcParser parser;
    std::string error;
    if (!parser.parse(text.c_str(), prog, error)) {
      fprintf(stderr, "faust2tang: %s\n", error.c_str());
      return 1;
    }
    Compiler compiler;
    if (!compiler.compile(prog, opt, img, error)) {
      fprintf(stderr, "faust2tang: %s\n", error.c_str());
      return 1;
    }

    long boot, control, worst;
    if (!simulate(img, boot, control, worst, error)) {
      fprintf(stderr, "faust2tang: %s\n", error.c_str());
      return 1;
    }

    output::Hardware hw;
    hw.tdm = tdm;
    if (generic) {
      hw.progAw = 12;
      hw.fastAw = output::aw(opt.fastWords);
      hw.descAw = 11;
      hw.nIn = 2;
      hw.nOut = tdm ? 8 : 2;  // onboard I2S: outputs 0/1, TDM: outputs 0..7
      hw.useSdram = 1;
    } else {
      hw.progAw = output::aw((uint32_t)img.program.size());
      hw.fastAw = output::aw(img.fastWords);
      hw.descAw = output::aw((uint32_t)img.descriptor.size());
    }
    if (img.program.size() > (1u << hw.progAw) || img.descriptor.size() > (1u << hw.descAw)) {
      fprintf(stderr, "faust2tang: program or descriptor too large for the bitstream\n");
      return 1;
    }
    std::string ns = output::lower(output::ident(img.name));
    std::string rep = output::report(img, compiler.layout(), boot, control, worst, clkHz);
    bool ok = writeFile(out + "/prog.hex", output::progHex(img.program, 1u << hw.progAw)) &&
              writeFile(out + "/desc.hex", output::descHex(img.descriptor, 1u << hw.descAw)) &&
              writeFile(out + "/config.vh", output::configVh(img, hw)) &&
              writeFile(out + "/" + ns + "_params.h", output::paramsHeader(img)) &&
              writeFile(out + "/" + ns + "_program.h", output::programHeader(img)) &&
              writeFile(out + "/report.txt", rep);
    if (!ok) return 1;
    printf("%s", rep.c_str());
    uint32_t budget = clkHz / opt.sampleRate;
    if (worst > (long)budget) {
      fprintf(stderr, "faust2tang: WARNING: a sample needs %ld cycles but only %u are available "
              "-- the output will glitch. Lower the sample rate or simplify the DSP.\n",
              worst, budget);
      if (strict) return 1;
    } else if (control > (long)budget) {
      fprintf(stderr, "faust2tang: note: the sample after a parameter change needs %ld cycles "
              "(%u available), so each change drops one sample. Avoid smoothing parameters "
              "that feed expensive functions, or lower the sample rate.\n", control, budget);
    }
  }
  if (port.empty()) return 0;
  return boardActions(port, doLoad ? &img : nullptr, doInfo, doList, doDebug, doMute, sets, gets);
}
