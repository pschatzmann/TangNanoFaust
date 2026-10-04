import("stdfaust.lib");
process = fi.lowpass(2, hslider("cutoff",1000,50,10000,1));
