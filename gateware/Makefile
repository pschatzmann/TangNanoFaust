# NanoTangFaust: Faust program -> Tang Nano 20K bitstream, built with Gowin EDA.
#
#   make generic                      bitstream that loads any program at runtime
#   make generic-flash                ... and load it into the FPGA (SRAM, until power-off)
#   make generic-flash-persistent     ... or write it to the onboard flash
#   make DSP=dsp/synth.dsp [flash]    bitstream that boots one program, sized for it
#   make sim                          RTL regression (FPU, core and system vs. the simulator)
#
# Options: SYS_MHZ=48 (default), 40.5, 33.75 or 27; TDM=1 (TDM output);
# SAMPLE_RATE=48000; FAUST=/path/to/faust; GW_SH=/path/to/gw_sh.
# Flow: faust -> faust2tang (tools/) -> Gowin synthesis, place & route, timing
# analysis and bitstream (gw_sh, see build.sh) -> openFPGALoader.

DSP      ?= dsp/sine.dsp
NAME     := $(basename $(notdir $(DSP)))
TDM      ?=
GENERIC  ?=
SYS_MHZ  ?= 48
SAMPLE_RATE ?= 48000
FAUST    ?= faust
GW_SH    ?= gw_sh
# one build directory per configuration, e.g. build/sine_tdm_40.5mhz
BUILD    := build/$(NAME)$(if $(TDM),_tdm,)$(if $(filter-out 48,$(SYS_MHZ)),_$(SYS_MHZ)mhz,)
FAUST2TANG := $(CURDIR)/../tools/bin/faust2tang
SOURCES  := $(wildcard rtl/*.v rtl/*.vh rtl/sdram/*.v) constraints/tangnano20k.cst build.sh

.PHONY: all bitstream flash flash-persistent report generic generic-flash \
        generic-flash-persistent sim clean

all: bitstream

$(FAUST2TANG): $(wildcard ../tools/*.cpp ../src/NanoTangFaust/compiler/*.h)
	$(MAKE) -C ../tools

$(BUILD)/$(NAME).fs: $(DSP) $(SOURCES) $(FAUST2TANG)
	FAUST=$(FAUST) GW_SH=$(GW_SH) ./build.sh $(BUILD) $(DSP) $(SYS_MHZ) \
		--sample-rate $(SAMPLE_RATE) $(if $(GENERIC),--generic,) $(if $(TDM),--tdm,)
	@head -5 $(BUILD)/report.txt

bitstream: $(BUILD)/$(NAME).fs

# the program's numbers, then Gowin's utilization summary
report: $(BUILD)/$(NAME).fs
	@cat $(BUILD)/report.txt
	@sed -n '/^3. Resource Usage Summary/,/MULT18X18/p' $(BUILD)/impl/pnr/nanotangfaust.rpt.txt

flash: $(BUILD)/$(NAME).fs
	openFPGALoader -b tangnano20k $<

flash-persistent: $(BUILD)/$(NAME).fs
	openFPGALoader -b tangnano20k -f $<

# Generic bitstream: boots the stereo passthrough and accepts any program
# over SPI or the UART (TangNanoFaust::load()), so programs need no rebuild.
generic:
	$(MAKE) DSP=dsp/passthrough.dsp GENERIC=1 bitstream

generic-flash:
	$(MAKE) DSP=dsp/passthrough.dsp GENERIC=1 flash

generic-flash-persistent:
	$(MAKE) DSP=dsp/passthrough.dsp GENERIC=1 flash-persistent

sim:
	tb/run_regression.sh

clean:
	rm -rf build
