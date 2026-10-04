// Program blob round trip (Output.h programBlob -> ProgramBlob.h parseProgramBlob).
//   blob_test a.fbc b.fbc ...        compile, write the blob, parse it, compare
//   blob_test --server x.ntfp a.fbc  also compare with a blob from faust2tang --serve
#include <stdio.h>
#include <string.h>

#include <fstream>
#include <sstream>
#include <string>

#include "NanoTangFaust/ProgramBlob.h"
#include "NanoTangFaust/compiler/Compiler.h"
#include "NanoTangFaust/compiler/Output.h"

using namespace nanotangfaust;
using namespace nanotangfaust::compiler;

static std::string readFile(const char *path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

static bool check(const char *fbcPath, const std::string *server) {
  ProgramImage img;
  std::string error;
  if (!compileFbc(readFile(fbcPath).c_str(), CompileOptions(), img, error)) {
    printf("%s: %s\n", fbcPath, error.c_str());
    return false;
  }
  std::string blob = output::programBlob(img);
  ProgramData p;
  bool ok = parseProgramBlob((const uint8_t *)blob.data(), blob.size(), p) &&
            img.name == p.name && img.descriptor == std::string(p.descriptor, p.descriptorLength) &&
            img.program.size() == p.words && img.dspEntry == p.dspEntry &&
            img.bootEntry == p.bootEntry && img.inputs == p.inputs && img.outputs == p.outputs &&
            img.params.size() == p.parameters && img.sampleRate == p.sampleRate &&
            img.fastWords == p.fastWords && img.sdramWords == p.sdramWords &&
            img.minProtocol == p.minProtocol;
  for (size_t i = 0; ok && i < img.program.size(); i++)
    for (int b = 0; b < 5; b++)
      if (p.code[i * 5 + b] != (uint8_t)(img.program[i] >> (8 * b))) ok = false;
  // truncated or damaged blobs are rejected
  ProgramData q;
  if (parseProgramBlob((const uint8_t *)blob.data(), blob.size() - 1, q)) ok = false;
  std::string bad = blob;
  bad[4] = 99;
  if (parseProgramBlob((const uint8_t *)bad.data(), bad.size(), q)) ok = false;
  if (server && *server != blob) {
    printf("%s: server blob differs (%zu vs %zu bytes)\n", fbcPath, server->size(), blob.size());
    ok = false;
  }
  printf("%s: %s (%zu bytes)\n", fbcPath, ok ? "ok" : "FAILED", blob.size());
  return ok;
}

int main(int argc, char **argv) {
  std::string server;
  bool haveServer = false, ok = true;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--server") && i + 1 < argc) {
      server = readFile(argv[++i]);
      haveServer = true;
      continue;
    }
    ok &= check(argv[i], haveServer ? &server : nullptr);
  }
  return ok ? 0 : 1;
}
