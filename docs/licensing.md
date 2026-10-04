# Licensing

TangNanoFaust is licensed under Apache-2.0 (see `LICENSE`), with one exception:

- `gateware/rtl/sdram/sdram.v`, the SDRAM controller by nand2mario (from
  Sipeed's TangNano-20K examples, via arduino-tangnano20k), is **GPLv3**.
  `gateware/rtl/sdram/sdram_bus.v` comes from arduino-tangnano20k.

The SDRAM controller is only synthesized into bitstreams that use SDRAM:
the generic bitstream, and bitstreams built for a program that needs
SDRAM. Such a **bitstream is a GPLv3 derivative work**. Bitstreams without
SDRAM, and all other parts (the Arduino library, the compiler, the
`faust2tang` tool and the rest of the gateware), are Apache-2.0.

Bitstreams are built with Gowin EDA, Gowin's proprietary tools (the
Education edition is free); they are not part of this repository.

Faust programs you write are yours. The Faust libraries (`stdfaust.lib`
etc.) have their own licenses, mostly LGPL with an exception for generated
code. See their headers.
