// SDRAM self-test on the board, with tables too large for block RAM (so
// faust2tang places them in SDRAM):
// - a table written while running, one word per sample, and read back one
//   lap later (`errors`, `checked`);
// - a table filled by the boot program in a tight loop, as rdtable
//   waveforms are (os.osc), and read back while running (`boot_errors`).
// Silent; read the result over USB:
//   faust2tang tests/dsp/sdram_test.dsp -o build/sdt --port /dev/ttyUSB1 --load
//   faust2tang --port /dev/ttyUSB1 --get errors --get checked --get boot_errors
// `checked` counts compared words (48000 per second after the first lap);
// `errors` and `boot_errors` count mismatches and must stay 0. With N = 1024 the table fits in
// block RAM, which checks the test itself.
declare name "sdram_test";

N = 65536;
i = (+(1) ~ _) - 1;                           // sample counter 0, 1, 2, ...
pat(k) = k xor (k << 13) xor (k << 22);       // exercises all 32 bits
got = rwtable(N, 0, i & (N - 1), pat(i), (i + 1) & (N - 1));
valid = i >= N - 1;                           // read slot written one lap ago
bad = valid & (got != pat(i + 1 - N));
errors = bad : + ~ _;
checked = valid : + ~ _;

bootTable = rdtable(N, i : pat, i & (N - 1));  // filled at boot with pat(0..N-1)
bootErrors = (bootTable != pat(i & (N - 1))) : + ~ _;

process = 0 : attach(_, errors : hbargraph("errors", 0, 1e9))
            : attach(_, checked : hbargraph("checked", 0, 1e9))
            : attach(_, bootErrors : hbargraph("boot_errors", 0, 1e9));
