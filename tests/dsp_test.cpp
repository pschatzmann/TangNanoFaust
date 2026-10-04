// Test driver for compiled Faust programs.
//
//   dsp_test check a.fbc ...                  simulator vs reference interpreter
//   dsp_test rtl prog.fbc DIR [samples]       files for gateware/tb/tb_dsp_core.v
//   dsp_test top boot.fbc load.fbc DIR        files for gateware/tb/tb_top.v
//   dsp_test compare-rtl DIR                  check tb_dsp_core.v's rtl_out.txt
//   dsp_test compare-top DIR                  check tb_top.v's tb_out.txt
//
// Build: g++ -std=c++17 -O2 -I../src dsp_test.cpp -o build/dsp_test
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "FbcInterpreter.h"
#include "NanoTangFaust/compiler/Compiler.h"
#include "NanoTangFaust/compiler/Isa.h"
#include "NanoTangFaust/compiler/Output.h"

using namespace nanotangfaust;
using namespace nanotangfaust::compiler;

static std::string readFile(const std::string &path) {
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

static bool writeFile(const std::string &path, const std::string &text) {
  std::ofstream f(path);
  f << text;
  return (bool)f;
}

static bool gFastMath = false;
static uint32_t gFastWords = 16384;  // --fast-words: smaller pushes arrays into SDRAM
static long gMaxCycles = 0;  // check: fail if a sample needs more (0: no limit)

static bool load(const std::string &path, FbcProgram &prog, ProgramImage &img,
                 Compiler &compiler) {
  std::string text = readFile(path), error;
  FbcParser parser;
  CompileOptions opt;
  opt.fastMath = gFastMath;
  opt.fastWords = gFastWords;
  if (!parser.parse(text.c_str(), prog, error) ||
      !compiler.compile(prog, opt, img, error)) {
    printf("%s: %s\n", path.c_str(), error.c_str());
    return false;
  }
  return true;
}

static uint32_t bits(float f) {
  uint32_t b;
  memcpy(&b, &f, 4);
  return b;
}

// ------------------------------------------------------------------ audio model
// gateware/rtl/audio_conv.v

static uint32_t s24ToF32(uint32_t v) {
  v &= 0xFFFFFF;
  if (!v) return 0;
  bool neg = v & 0x800000;
  uint32_t mag = neg ? ((~v + 1) & 0xFFFFFF) : v;
  return roundPack(neg, mag, -23);
}

static uint32_t f32ToS24(uint32_t f) {
  uint32_t e = (f >> 23) & 0xFF, mag;
  if (e == 0xFF && (f & 0x7FFFFF)) mag = 0;
  else if (e >= 127) mag = 0x7FFFFF;
  else if (e < 103) mag = 0;
  else mag = (0x800000 | (f & 0x7FFFFF)) >> (127 - e);
  return (f & 0x80000000u) ? ((0u - mag) & 0xFFFFFF) : mag;
}

// ------------------------------------------------------------------ check

static int check(int argc, char **argv) {
  int failed = 0;
  for (int i = 2; i < argc; i++) {
    FbcProgram prog;
    ProgramImage img;
    Compiler compiler;
    if (!load(argv[i], prog, img, compiler)) {
      failed++;
      continue;
    }
    test::FbcInterpreter ref(prog, img.sampleRate);
    ref.init();
    Machine m(img.program, img.fastWords, img.inputs, img.outputs);
    long boot = m.run(img.bootEntry, 500000000);
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> u(-0.8f, 0.8f);
    double maxErr = 0;
    long worst = 0;
    bool ok = boot >= 0;
    for (int n = 0; n < 2000 && ok; n++) {
      for (int c = 0; c < img.inputs; c++) {
        float x = u(rng);
        ref.inputs[c] = x;
        m.inputs[c] = bits(x);
      }
      long cyc = m.run(n == 0 ? 0 : img.dspEntry);
      ref.compute();
      if (cyc < 0 || !ref.error.empty()) {
        ok = false;
        break;
      }
      if (n && cyc > worst) worst = cyc;
      for (int c = 0; c < img.outputs; c++) {
        double d = fabs((double)fp32::toFloat(m.outputs[c]) - ref.outputs[c]);
        if (d > maxErr || d != d) maxErr = d != d ? INFINITY : d;
      }
    }
    ok = ok && maxErr <= 1e-4 && (gMaxCycles == 0 || worst <= gMaxCycles);
    if (!ok) failed++;
    printf("%s %s: boot %ld cycles, worst %ld cycles/sample, max error %.3g%s%s\n",
           ok ? "PASS" : "FAIL", argv[i], boot, worst, maxErr,
           m.error.empty() ? "" : " -- ", m.error.c_str());
  }
  return failed ? 1 : 0;
}

// ------------------------------------------------------------------ rtl

// params prog.fbc N: every parameter / bargraph after N samples on the
// simulator -- the expected values of the on-board self-tests (unit_test.dsp).
static int params(int, char **argv) {
  FbcProgram prog;
  ProgramImage img;
  Compiler compiler;
  if (!load(argv[2], prog, img, compiler)) return 1;
  Machine m(img.program, img.fastWords, img.inputs, img.outputs);
  if (m.run(img.bootEntry, 2000000000L) < 0) {
    printf("boot failed: %s\n", m.error.c_str());
    return 1;
  }
  long n = atol(argv[3]);
  for (long k = 0; k < n; k++)
    if (m.run(k == 0 ? 0 : img.dspEntry) < 0) {
      printf("sample %ld failed: %s\n", k, m.error.c_str());
      return 1;
    }
  for (auto &p : img.params) {
    uint32_t bits = m.heap.read(p.addr);
    float f;
    memcpy(&f, &bits, 4);
    printf("%s = %.9g (0x%08x)\n", p.label.c_str(), f, bits);
  }
  return 0;
}

static int rtl(int argc, char **argv) {
  if (argc < 4) return 2;
  std::string dir = argv[3];
  int samples = argc > 4 ? atoi(argv[4]) : 200;
  FbcProgram prog;
  ProgramImage img;
  Compiler compiler;
  if (!load(argv[2], prog, img, compiler)) return 1;
  output::Hardware hw;
  hw.progAw = output::aw((uint32_t)img.program.size());
  hw.fastAw = output::aw(img.fastWords);
  hw.descAw = output::aw((uint32_t)img.descriptor.size());
  writeFile(dir + "/prog.hex", output::progHex(img.program, 1u << hw.progAw));
  writeFile(dir + "/config.vh", output::configVh(img, hw));

  int nIn = img.inputs > 0 ? img.inputs : 1;
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> u(-0.9f, 0.9f);
  std::vector<uint32_t> ins(samples * nIn);
  std::string inHex;
  for (auto &x : ins) {
    x = bits(u(rng));
    inHex += output::fmt("%08x\n", x);
  }
  writeFile(dir + "/inputs.hex", inHex);

  Machine m(img.program, img.fastWords, img.inputs, img.outputs);
  std::string exp = output::fmt("boot %ld\n", m.run(img.bootEntry, 500000000));
  for (int n = 0; n < samples; n++) {
    for (int c = 0; c < nIn; c++) m.inputs[c] = ins[n * nIn + c];
    exp += std::to_string(m.run(n == 0 ? 0 : img.dspEntry));
    for (uint32_t o : m.outputs) exp += output::fmt(" %08x", o);
    exp += "\n";
  }
  if (!m.error.empty()) {
    printf("simulation failed: %s\n", m.error.c_str());
    return 1;
  }
  writeFile(dir + "/expected.txt", exp);
  return 0;
}

static int compareLines(const std::string &dir, const char *a, const char *b) {
  std::istringstream e(readFile(dir + "/" + a)), g(readFile(dir + "/" + b));
  std::string le, lg;
  int line = 0, errors = 0;
  while (true) {
    bool he = (bool)std::getline(e, le), hg = (bool)std::getline(g, lg);
    if (!he && !hg) break;
    if (he != hg || le != lg) {
      if (++errors <= 10) printf("line %d: expected '%s' got '%s'\n", line, le.c_str(), lg.c_str());
    }
    line++;
  }
  printf("%s %s: %d lines, %d mismatching\n", errors ? "FAIL" : "PASS", dir.c_str(), line, errors);
  return errors ? 1 : 0;
}

// ------------------------------------------------------------------ top

static std::string bytesHex(const std::vector<uint8_t> &b) {
  std::string s;
  for (uint8_t x : b) s += output::fmt("%02x\n", x);
  return s.empty() ? "00\n" : s;
}

static int top(int argc, char **argv) {
  if (argc < 5) return 2;
  std::string dir = argv[4];
  FbcProgram bootProg, prog;
  ProgramImage bootImg, img;
  Compiler bootCompiler, compiler;
  if (!load(argv[2], bootProg, bootImg, bootCompiler) || !load(argv[3], prog, img, compiler))
    return 1;

  // the bitstream boots `boot`; generic sizes so `load` fits
  output::Hardware hw;
  hw.progAw = 12;
  hw.fastAw = 14;
  hw.descAw = 11;
  hw.nIn = 2;
  hw.tdm = !(argc > 5 && std::string(argv[5]) == "notdm");  // run_top_test.sh NOTDM=1
  hw.nOut = hw.tdm ? 8 : 2;
  hw.useSdram = 0;
  writeFile(dir + "/prog.hex", output::progHex(bootImg.program, 1u << hw.progAw));
  writeFile(dir + "/desc.hex", output::descHex(bootImg.descriptor, 1u << hw.descAw));
  writeFile(dir + "/config.vh", output::configVh(bootImg, hw));

  // what the host uploads
  std::vector<uint8_t> code, desc(img.descriptor.begin(), img.descriptor.end()), cfg;
  for (uint64_t w : img.program)
    for (int b = 0; b < 5; b++) code.push_back((uint8_t)(w >> (8 * b)));
  uint32_t inc = (uint32_t)(((uint64_t)img.sampleRate * 128ull << 32) / 54000000ull);
  auto le = [&](uint32_t v, int n) {
    for (int i = 0; i < n; i++) cfg.push_back((uint8_t)(v >> (8 * i)));
  };
  le(img.dspEntry, 2);
  le(img.bootEntry, 2);
  le(img.inputs, 1);
  le(img.outputs, 1);
  le((uint32_t)img.params.size(), 1);
  le(img.sampleRate, 4);
  le(inc, 4);
  le((uint32_t)img.descriptor.size(), 2);
  writeFile(dir + "/load_prog.hex", bytesHex(code));
  writeFile(dir + "/load_desc.hex", bytesHex(desc));
  writeFile(dir + "/load_cfg.hex", bytesHex(cfg));

  // parameters "gain" and "offset" of the loaded program
  const Param *gain = nullptr, *offset = nullptr;
  for (const Param &p : img.params) {
    if (p.label == "gain") gain = &p;
    if (p.label == "offset") offset = &p;
  }
  if (!gain || !offset) {
    printf("the loaded program needs 'gain' and 'offset' parameters\n");
    return 1;
  }
  float g = 0.75f, o = -0.125f;
  std::string args = output::fmt("+prog_bytes=%d +desc_bytes=%d +gain_addr=%u +offset_addr=%u "
                                 "+gain_bits=%08x +offset_bits=%08x\n",
                                 (int)code.size(), (int)desc.size(), gain->addr, offset->addr,
                                 bits(g), bits(o));
  writeFile(dir + "/plusargs.txt", args);

  // input samples (24-bit) and the expected output for each of them
  std::string inHex, expected, expectedTdm;
  Machine m(img.program, img.fastWords, img.inputs, img.outputs);
  m.run(img.bootEntry, 500000000);
  m.heap.write(gain->addr, bits(g));
  m.heap.write(offset->addr, bits(o));
  for (int k = 0; k < 64; k++) {
    int32_t v = (int32_t)((k * 2654435761u) >> 8) - 0x800000;  // spread over the range
    uint32_t s = (uint32_t)v & 0xFFFFFF;
    inHex += output::fmt("%06x\n", s);
    m.inputs[0] = s24ToF32(s);
    m.run(k == 0 ? 0 : img.dspEntry);
    expected += output::fmt("%06x\n", f32ToS24(m.outputs[0]));
    for (int c = 0; c < 4; c++)  // TDM slots 0..3
      expectedTdm += output::fmt(c ? " %06x" : "%06x",
                                 c < img.outputs ? f32ToS24(m.outputs[c]) : 0u);
    expectedTdm += "\n";
  }
  writeFile(dir + "/expected_tdm.txt", expectedTdm);
  writeFile(dir + "/in_samples.hex", inHex);
  writeFile(dir + "/expected_samples.txt", expected);
  writeFile(dir + "/expected_desc.txt", img.descriptor);
  writeFile(dir + "/expected_readback.txt", output::fmt("%08x\n", bits(g)));
  return 0;
}

// The DSP is stateless (gain/offset), so output frame f carries the result
// for input frame f - latency; find the latency and check every frame.
static int compareTop(const std::string &dir) {
  std::istringstream out(readFile(dir + "/tb_out.txt"));
  std::vector<uint32_t> expected;
  {
    std::istringstream e(readFile(dir + "/expected_samples.txt"));
    std::string l;
    while (std::getline(e, l)) expected.push_back((uint32_t)strtoul(l.c_str(), nullptr, 16));
  }
  std::string desc = readFile(dir + "/expected_desc.txt");
  std::string readback = readFile(dir + "/expected_readback.txt");
  readback = readback.substr(0, 8);
  int errors = 0;
  std::vector<std::pair<int, uint32_t>> frames;
  std::vector<std::pair<int, std::string>> tdm;
  std::string line;
  while (std::getline(out, line)) {
    if (line.rfind("ping ", 0) == 0) {
      if (line.compare(5, 4, "FAUS") != 0) {
        printf("bad ping reply: %s\n", line.c_str());
        errors++;
      }
    } else if (line.rfind("desc ", 0) == 0) {
      std::string hex;
      for (unsigned char ch : desc) hex += output::fmt("%02x", ch);
      if (line.substr(5) != hex) {
        printf("descriptor read back differs\n");
        errors++;
      }
    } else if (line.rfind("readback ", 0) == 0) {
      if (line.substr(9, 8) != readback) {
        printf("parameter read back %s, expected %s\n", line.c_str(), readback.c_str());
        errors++;
      }
    } else if (line.rfind("tdm ", 0) == 0) {
      int f;
      char rest[64];
      if (sscanf(line.c_str(), "tdm %d %63[0-9a-fx ]", &f, rest) == 2) tdm.push_back({f, rest});
    } else if (line.rfind("info", 0) == 0) {
      printf("%s\n", line.c_str());
    } else if (!line.empty() && isdigit((unsigned char)line[0])) {
      int f;
      unsigned v;
      if (sscanf(line.c_str(), "%d %x", &f, &v) == 2) frames.push_back({f, v});
    }
  }
  int latency = -1;
  for (int lat = 0; lat <= 4 && latency < 0; lat++) {
    bool all = !frames.empty();
    for (auto &fv : frames) {
      int k = ((fv.first - lat) % 64 + 64) % 64;
      if (expected[k] != fv.second) all = false;
    }
    if (all) latency = lat;
  }
  if (latency < 0) {
    printf("audio output doesn't match the model at any latency\n");
    for (size_t i = 0; i < frames.size() && i < 6; i++)
      printf("  frame %d: got %06x, expected (latency 2) %06x\n", frames[i].first,
             frames[i].second, expected[((frames[i].first - 2) % 64 + 64) % 64]);
    errors++;
  }
  if (!tdm.empty()) {
    std::vector<std::string> expTdm;
    std::istringstream e(readFile(dir + "/expected_tdm.txt"));
    std::string l;
    while (std::getline(e, l)) expTdm.push_back(l);
    int tlat = -1;
    for (int lat = 0; lat <= 4 && tlat < 0; lat++) {
      bool all = true;
      for (auto &fv : tdm)
        if (expTdm[((fv.first - lat) % 64 + 64) % 64] != fv.second) all = false;
      if (all) tlat = lat;
    }
    if (tlat < 0) {
      printf("TDM output doesn't match the model at any latency\n");
      for (size_t i = 0; i < tdm.size() && i < 4; i++)
        printf("  frame %d: got %s, expected (latency 2) %s\n", tdm[i].first, tdm[i].second.c_str(),
               expTdm[((tdm[i].first - 2) % 64 + 64) % 64].c_str());
      errors++;
    }
    printf("%s TDM: %d frames x 4 slots%s, latency %d frames\n", tlat < 0 ? "FAIL" : "PASS",
           (int)tdm.size(), tlat >= 0 ? " bit-exact" : "", tlat);
  }
  printf("%s system test: %d audio frames%s, latency %d frames\n", errors ? "FAIL" : "PASS",
         (int)frames.size(), latency >= 0 ? " bit-exact" : "", latency);
  return errors ? 1 : 0;
}

int main(int argc, char **argv) {
  while (argc >= 3 && argv[1][0] == '-' && argv[1][1] == '-') {  // options before the mode
    if (!strcmp(argv[1], "--fast-math")) gFastMath = true;
    else if (!strcmp(argv[1], "--fast-words") && argc >= 4) {
      gFastWords = (uint32_t)atol(argv[2]);
      argv++;
      argc--;
    } else if (!strcmp(argv[1], "--max-cycles") && argc >= 4) {
      gMaxCycles = atol(argv[2]);
      argv++;
      argc--;
    }
    argv++;
    argc--;
  }
  if (argc >= 3 && !strcmp(argv[1], "check")) return check(argc, argv);
  if (argc >= 4 && !strcmp(argv[1], "rtl")) return rtl(argc, argv);
  if (argc >= 4 && !strcmp(argv[1], "params")) return params(argc, argv);
  if (argc >= 5 && !strcmp(argv[1], "top")) return top(argc, argv);
  if (argc >= 3 && !strcmp(argv[1], "compare-rtl"))
    return compareLines(argv[2], "expected.txt", "rtl_out.txt");
  if (argc >= 3 && !strcmp(argv[1], "compare-top")) return compareTop(argv[2]);
  fprintf(stderr,
          "usage: dsp_test [--fast-math] [--fast-words N] [--max-cycles N] check a.fbc... | rtl prog.fbc DIR [samples] | "
          "top boot.fbc load.fbc DIR | params prog.fbc N | compare-rtl DIR | compare-top DIR\n");
  return 2;
}
