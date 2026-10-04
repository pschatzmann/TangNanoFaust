// 2. A volume control: multiply every sample by a slider value.
declare name "gain";
process = _ * hslider("gain", 0.5, 0, 1, 0.01);
