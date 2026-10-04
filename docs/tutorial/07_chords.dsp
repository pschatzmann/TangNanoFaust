// 7. Several voices with par(): three oscillators for a chord, mixed with :>.
declare name "chords";
import("stdfaust.lib");
root = hslider("root", 220, 50, 1000, 1);
voice(ratio) = os.osc(root * ratio) * 0.2;
process = par(i, 3, voice(ba.take(i + 1, (1, 1.25, 1.5)))) :> _;
