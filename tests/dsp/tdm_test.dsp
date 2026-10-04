// System test program with 4 outputs (onboard I2S: outputs 0/1, TDM: 0..3).
declare name "tdm_test";
g = hslider("gain", 0.5, 0, 2, 0.01);
o = hslider("offset", 0, -1, 1, 0.01);
process = _ <: *(g) + o, *(0 - g), *(0.5), *(0) + o;
