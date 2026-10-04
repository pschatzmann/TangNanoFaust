import("stdfaust.lib");
process = _ <: _, de.delay(48000, hslider("d",12000,1,47999,1)) * 0.5 :> _;
