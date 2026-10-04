#pragma once
#include <stdint.h>

namespace tangnanofaust {

/**
 * @brief A compiled Faust program that TangNanoFaust::load() can upload to
 * the FPGA. faust2tang writes one as `<name>_program.h`; the on-MCU
 * compiler (TangNanoFaustCompiler.h) produces one at runtime.
 */
struct ProgramData {
  const char *name = "";
  const uint8_t *code = nullptr;  ///< instructions, 5 bytes each, little endian
  uint32_t words = 0;             ///< number of instructions
  const char *descriptor = "";    ///< parameter descriptor text
  uint16_t descriptorLength = 0;
  uint16_t dspEntry = 0, bootEntry = 0;
  uint8_t inputs = 0, outputs = 0, parameters = 0;
  uint32_t sampleRate = 48000;
  uint32_t fastWords = 0;   ///< block RAM words the program needs
  uint32_t sdramWords = 0;  ///< SDRAM words the program needs
  uint8_t minProtocol = 3;  ///< oldest bitstream protocol version that runs it
};

}  // namespace tangnanofaust
