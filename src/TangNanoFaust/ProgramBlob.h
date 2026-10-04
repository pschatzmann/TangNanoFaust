#pragma once
/**
 * Compiled programs as one binary block, e.g. received over the network
 * from `faust2tang --serve` (docs/faust2tang.md#compile-server).
 *
 * Layout (little endian):
 *   0  "NTFP", u8 format version (1), u8 minProtocol, u8 inputs, u8 outputs
 *   8  u8 parameters, u8 name length, u16 descriptor length
 *  12  u32 words, u16 dspEntry, u16 bootEntry
 *  20  u32 sampleRate, u32 fastWords, u32 sdramWords
 *  32  name + NUL, descriptor + NUL, code (5 bytes per instruction)
 *
 * parseProgramBlob() fills a ProgramData that points into the buffer, so
 * the buffer must stay valid while the ProgramData is used.
 */
#include <stddef.h>
#include <stdint.h>

#include "ProgramData.h"

namespace tangnanofaust {

static const uint8_t kProgramBlobVersion = 1;
static const size_t kProgramBlobHeader = 32;

namespace blob {
inline uint32_t rd(const uint8_t *p, int n) {
  uint32_t v = 0;
  for (int i = n - 1; i >= 0; i--) v = (v << 8) | p[i];
  return v;
}
}  // namespace blob

/// Fills `p` from a program blob. False if the data is not a complete
/// program blob of a known format version.
inline bool parseProgramBlob(const uint8_t *data, size_t size, ProgramData &p) {
  using blob::rd;
  if (!data || size < kProgramBlobHeader || data[0] != 'N' || data[1] != 'T' || data[2] != 'F' ||
      data[3] != 'P' || data[4] != kProgramBlobVersion)
    return false;
  size_t nameLen = data[9], descLen = rd(data + 10, 2);
  uint32_t words = rd(data + 12, 4);
  size_t need = kProgramBlobHeader + nameLen + 1 + descLen + 1 + (size_t)words * 5;
  if (size < need) return false;
  const uint8_t *name = data + kProgramBlobHeader;
  const uint8_t *desc = name + nameLen + 1;
  if (name[nameLen] != 0 || desc[descLen] != 0) return false;
  p.minProtocol = data[5];
  p.inputs = data[6];
  p.outputs = data[7];
  p.parameters = data[8];
  p.descriptorLength = (uint16_t)descLen;
  p.words = words;
  p.dspEntry = (uint16_t)rd(data + 16, 2);
  p.bootEntry = (uint16_t)rd(data + 18, 2);
  p.sampleRate = rd(data + 20, 4);
  p.fastWords = rd(data + 24, 4);
  p.sdramWords = rd(data + 28, 4);
  p.name = (const char *)name;
  p.descriptor = (const char *)desc;
  p.code = desc + descLen + 1;
  return true;
}

}  // namespace tangnanofaust
