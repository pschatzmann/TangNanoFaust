// 4. Recursion: a one-pole lowpass filter, y[n] = (1-a) x[n] + a y[n-1].
declare name "onepole";
a = hslider("smoothness", 0.9, 0, 0.999, 0.001);
process = *(1 - a) : + ~ *(a);
