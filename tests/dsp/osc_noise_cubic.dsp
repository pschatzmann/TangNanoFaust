import("stdfaust.lib");
process = os.oscsin(hslider("freq",440,20,2000,1)) * 0.3, no.noise * 0.1 :> _ : ef.cubicnl(0.5,0);
