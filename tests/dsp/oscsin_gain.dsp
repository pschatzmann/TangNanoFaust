import("stdfaust.lib");
process = os.oscsin(hslider("freq",440,20,2000,1)) * hslider("gain",0.5,0,1,0.01);
