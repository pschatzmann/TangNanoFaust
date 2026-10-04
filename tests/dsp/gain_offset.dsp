// Stateless test program for the system test (tb_top.v).
declare name "gain_offset";
process = _ * hslider("gain", 0.5, 0, 2, 0.01) + hslider("offset", 0, -1, 1, 0.01);
