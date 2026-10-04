# Builds the command line tools (C++17, no dependencies):
#   bin/faust2tang     Faust -> DSP core compiler
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
HEADERS  := $(wildcard ../src/NanoTangFaust/compiler/*.h)

all: bin/faust2tang

bin/faust2tang: faust2tang.cpp $(HEADERS)
	mkdir -p bin
	$(CXX) $(CXXFLAGS) -I../src $< -o $@

clean:
	rm -rf bin
