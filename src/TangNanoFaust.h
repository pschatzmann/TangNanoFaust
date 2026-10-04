#pragma once
/**
 * TangNanoFaust: Faust DSP programs running on a Tang Nano 20K FPGA.
 *
 * The FPGA (gateware/) runs a Faust program compiled by tools/faust2tang on
 * its own floating-point DSP core and plays the audio on the board's
 * amplifier. This library is the MCU side: it changes the program's
 * parameters over SPI (TangNanoFaust) and can drive them from MIDI received
 * through any Arduino Stream such as Serial (FaustMidi).
 */
#include "TangNanoFaust/TangNanoFaust.h"
#include "TangNanoFaust/FaustMidi.h"
#include "TangNanoFaust/ProgramBlob.h"
