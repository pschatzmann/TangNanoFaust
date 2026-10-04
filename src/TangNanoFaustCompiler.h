#pragma once
/**
 * On-MCU Faust compilation for TangNanoFaust.
 *
 * The Faust compiler itself only runs on a computer: `faust -lang interp
 * -double my.dsp -o my.fbc` turns a .dsp into interpreter bytecode (FBC,
 * plain text). Everything after that -- compiling the FBC for the FPGA's
 * DSP core, laying out its memory and uploading it -- can run on the
 * microcontroller with this header -- the same compiler the faust2tang
 * command line tool is built from. Needs the C++ standard library (ESP32,
 * RP2040, ...; not AVR). `faust2tang --sketch my.dsp` writes the bytecode
 * as a header (my_fbc.h) to include in a sketch.
 *
 * @code
 * TangNanoFaust faust;
 * FaustCompiler faustCompiler;
 * faust.begin(SPI, 5);
 * if (!faustCompiler.compileAndLoad(faust, fbcText)) Serial.println(faustCompiler.error());
 * @endcode
 */
#include <string>
#include <vector>

#include "TangNanoFaust.h"
#include "TangNanoFaust/compiler/Compiler.h"

namespace tangnanofaust {

class FaustCompiler {
 public:
  /// Compiles FBC text. `fastWords`: block RAM words of the target FPGA
  /// (TangNanoFaust::info().memoryCapacity); `allowSdram`: whether its
  /// bitstream has the SDRAM controller.
  bool compile(const char *fbcText, uint32_t sampleRate = 48000, uint32_t fastWords = 16384,
               bool allowSdram = true, bool fuse = true, bool fastMath = false) {
    compiler::CompileOptions opt;
    opt.sampleRate = sampleRate;
    opt.fastWords = fastWords;
    opt.allowSdram = allowSdram;
    opt.fuse = fuse;
    opt.fastMath = fastMath;
    error_.clear();
    if (!compiler::compileFbc(fbcText, opt, image_, error_)) return false;
    pack();
    return true;
  }

  /// Compiles for the connected FPGA (its memory sizes) and loads the
  /// result. TangNanoFaust::begin() must have succeeded.
  /// `fastMath`: faster sin/cos (see docs/faust.md).
  bool compileAndLoad(TangNanoFaust &faust, const char *fbcText, uint32_t sampleRate = 48000,
                      bool fastMath = false) {
    Info inf = faust.info();
    if (!compile(fbcText, sampleRate, inf.memoryCapacity, inf.sdram, true, fastMath)) return false;
    LoadResult r = faust.load(programData());
    if (r != LoadResult::Ok) {
      error_ = std::string("load failed: ") + loadResultText(r);
      return false;
    }
    return true;
  }

  /// The last compiled program, in the form TangNanoFaust::load() takes
  /// (valid until the next compile()).
  ProgramData programData() const {
    ProgramData p;
    p.name = image_.name.c_str();
    p.code = code_.data();
    p.words = (uint32_t)image_.program.size();
    p.descriptor = image_.descriptor.c_str();
    p.descriptorLength = (uint16_t)image_.descriptor.size();
    p.dspEntry = (uint16_t)image_.dspEntry;
    p.bootEntry = (uint16_t)image_.bootEntry;
    p.inputs = (uint8_t)image_.inputs;
    p.outputs = (uint8_t)image_.outputs;
    p.parameters = (uint8_t)image_.params.size();
    p.sampleRate = image_.sampleRate;
    p.fastWords = image_.fastWords;
    p.sdramWords = image_.sdramWords;
    p.minProtocol = image_.minProtocol;
    return p;
  }

  const compiler::ProgramImage &image() const { return image_; }
  const char *error() const { return error_.c_str(); }

  static const char *loadResultText(LoadResult r) {
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
    return "?";
  }

 protected:
  compiler::ProgramImage image_;
  std::vector<uint8_t> code_;
  std::string error_;

  void pack() {
    code_.resize(image_.program.size() * 5);
    for (size_t i = 0; i < image_.program.size(); i++)
      for (int b = 0; b < 5; b++) code_[i * 5 + b] = (uint8_t)(image_.program[i] >> (8 * b));
  }
};

}  // namespace tangnanofaust
