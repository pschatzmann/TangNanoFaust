// Echo on the I2S input. The one-second delay line doesn't fit in block RAM,
// so this program uses the SDRAM (built in automatically).
declare name "echo";
import("stdfaust.lib");

time = hslider("time [unit:ms] [midi:ctrl 1]", 300, 1, 1000, 1) * ma.SR / 1000 : si.smoo;
feedback = hslider("feedback [midi:ctrl 7]", 0.4, 0, 0.95, 0.01);

process = _ <: _, (+ ~ (de.fdelay(65536, time) * feedback)) :> *(0.5);
