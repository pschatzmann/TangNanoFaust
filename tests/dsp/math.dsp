// Exercises the math library on the input signal.
declare name "math";
import("stdfaust.lib");
x = _ * 0.9;
process = x <: sin(3 * x) + cos(x) + tan(x * 0.7) + exp(x) + log(2 + x) + log10(2 + x)
             + pow(1.5 + x, 2.3) + atan(4 * x) + atan2(x, 0.5) + asin(x) + acos(x)
             + ma.sinh(x) + ma.cosh(x) + ma.tanh(3 * x) + ma.asinh(x) + ma.acosh(2 + x) + ma.atanh(x)
             + floor(5 * x) + ceil(5 * x) + rint(5 * x) + round(5 * x) + fmod(5 * x, 1.3)
             + sqrt(1 + x) + abs(x) + max(x, 0.1) + min(x, -0.1) : *(0.05);
