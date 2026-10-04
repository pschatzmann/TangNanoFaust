// 6. An effect on the I2S input: echo with feedback (the delay line lives in SDRAM).
declare name "echo";
import("stdfaust.lib");
time     = hslider("time [unit:ms]", 300, 1, 1000, 1) * ma.SR / 1000;
feedback = hslider("feedback", 0.4, 0, 0.9, 0.01);
process  = _ <: _, (+ ~ (de.delay(65536, time) * feedback)) :> *(0.5);
