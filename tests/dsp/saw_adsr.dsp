import("stdfaust.lib");
freq = hslider("freq",440,20,2000,1); gate = button("gate"); gain = hslider("gain",0.5,0,1,0.01);
process = os.sawtooth(freq) * en.adsr(0.01,0.1,0.8,0.3,gate) * gain : fi.resonlp(2000,2,1);
