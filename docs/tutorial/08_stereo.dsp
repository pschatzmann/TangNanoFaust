// 8. Two outputs: an auto-panner moves a tone between left and right.
declare name "stereo";
import("stdfaust.lib");
rate = hslider("rate [unit:Hz]", 0.5, 0.05, 5, 0.01);
pan  = (os.osc(rate) + 1) / 2;               // 0..1, slowly
process = os.osc(330) * 0.3 <: *(1 - pan), *(pan);
