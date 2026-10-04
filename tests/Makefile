# C++ test tools (no dependencies besides a C++17 compiler):
#   build/dsp_test      compiled programs vs reference interpreter, RTL test files
#   build/fpu_vectors   FPU test vectors from the bit-exact specification
#   build/blob_test     program blob round trip (faust2tang --serve)
#   build/midi_test     FaustMidi + TangNanoFaust on a PC against the board
#                       (host/: Arduino API stand-ins)
#   make check          simulator vs reference interpreter for all tests/fbc/*.fbc
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
HEADERS  := $(wildcard ../src/TangNanoFaust/compiler/*.h) FbcInterpreter.h

all: build/dsp_test build/fpu_vectors build/blob_test

build/dsp_test: dsp_test.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -I../src -I. $< -o $@

build/fpu_vectors: fpu_vectors.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -I../src $< -o $@

build/blob_test: blob_test.cpp $(HEADERS) ../src/TangNanoFaust/ProgramBlob.h
	mkdir -p build
	$(CXX) $(CXXFLAGS) -I../src $< -o $@

build/midi_test: host/midi_test.cpp host/Arduino.h host/SPI.h $(wildcard ../src/TangNanoFaust/*.h)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -Ihost -I../src $< -o $@

check: build/dsp_test build/blob_test
	./build/dsp_test check fbc/*.fbc
	./build/blob_test fbc/*.fbc

clean:
	rm -rf build
