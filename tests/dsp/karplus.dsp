// Compiler/RTL regression test (tests/fbc/karplus.fbc). Not for listening:
// the gate step itself excites the string, which pops and overloads; see
// gateware/dsp/pluck.dsp for the musical version.
import("stdfaust.lib");
process = pm.ks(hslider("len",100,10,400,1), 0.5, button("gate")) <: _,_;
