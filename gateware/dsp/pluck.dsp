// Plucked string (Karplus-Strong). Each rising edge of gate starts a note
// with a 5 ms noise burst. Feeding the gate step itself into the string,
// as tests/dsp/karplus.dsp does, kicks it with a jump at both edges: loud
// pops and an output far above full scale.
declare name "pluck";
import("stdfaust.lib");
len  = hslider("len [unit:samples]", 100, 10, 400, 1);  // pitch = SR / len
gate = button("gate");
trig = gate > gate';                                   // rising edge
burst = no.noise * en.ar(0.0005, 0.005, trig);         // short noise burst
process = pm.ks(len, 0.5, burst) * 0.3 <: _, _;
