import("stdfaust.lib");
process = _ * 0.5 : fi.lowpass(2, hslider("cutoff",1000,50,10000,1)) : ef.cubicnl(0.3, 0);
