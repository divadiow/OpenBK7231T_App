# BL0937 acquisition regressions

Run from the repository root with Python 3, GCC and Clang installed:

```sh
python3 tests/bl0937/run_tests.py --source . --out /tmp/obk-bl0937-tests
python3 tests/bl0937/check_codegen.py --source . --out /tmp/obk-bl0937-codegen
```

Use `--quick` on the first command for one compiler/configuration. Nonzero exit means a failed compilation or assertion. Results and compiler output are written under `--out`. No SDK submodules or hardware are needed for these host tests.

The harness compiles the actual `drv_bl0937.c`, `drv_pwrCal.c` and `drv_bl_shared.c`. It stubs external clocks, GPIO, configuration, MQTT, persistence and the moving-average helper. It does not replace acquisition, calibration, shared energy integration or publication filtering with a second implementation. Each scenario is a separate process. Both SEL polarities are exercised.

The full matrix covers GCC/Clang with UndefinedBehaviorSanitizer, 100/1000 Hz ticks, unsigned 16/32-bit tick counters, Beken/non-Beken definitions, moving averages, twin-meter builds, native-word fallback, and extra 128/250 Hz rate-normalization checks. The code-generation check extracts the actual ISR and snapshot functions and compiles them for ARM9, Cortex-M0/M3/M4/M33 and RV32IM/RV32IMA, rejecting external atomic helper calls.

## Regression cases

The tests cover post-SEL contamination, nominal and delayed acquisition, short/zero callback intervals, snapshot preemption and retry exhaustion, delays before/after the physical SEL write, delays while publishing, tick and pulse-counter rollover, calibration isolation, legacy calibration compatibility, startup load changes, pin/inversion changes including a pending CF window, PowerMax, partial samples, moving averages, relay-zero policy, invalid intervals, twin-meter isolation and randomized timing/load sequences.

The old `a0991f62473e25d79e8669750ee8063cec3f7406` revision can be tested with `--source <old-checkout> --baseline --quick`. That mode intentionally checks the original preemption and startup-energy regressions and should fail. It compiles the old sources rather than a model of the old calculation.

## Acquisition design

CF and CF1 have separate cumulative 32-bit counters, each written only by its corresponding GPIO callback. The task never clears a live ISR counter. Snapshots read the RTOS tick on both sides of the counts, with ordering barriers, and accept only a same-tick bracket. Four interrupted attempts defer the read without moving any acquisition baseline. This bounds timestamp/count skew to one RTOS tick when the RTOS clock is advancing correctly.

Atomic word loads/stores are used only when the compiler guarantees lock freedom. Older single-core ports use their native aligned volatile word accesses. The ISR does not take locks, perform floating-point calculations, read time, or log. This relies on the existing HAL contract of one serialized writer per GPIO callback, not concurrent writers for the same counter. Acquisition processing and driver reconfiguration must remain serialized as in the existing main-loop driver dispatch.

CF1 waits at least 1000 ms after the actual SEL write. The post-switch window is discarded; a subsequent clean window of at least 500 ms updates only the measured quantity. Other voltage/current results are retained, initially NAN. A new mode starts after the actual pin write, not at a stale sample timestamp. Nominal one-second callbacks yield one voltage or current refresh per approximately four seconds.

CF power continues during settling. Its actual window duration is passed through the existing shared meter policies to energy integration. Existing meter APIs retain their previous implicit-timing/explicit-energy behavior. A complete pending CF window is flushed before pin reconfiguration; the physical interrupt-detach/reconfigure gap is not claimed to be observable.

## Limits

These are deterministic software regressions and representative compiler checks, not BL602/RTL emulators or physical BL0937 measurements. They cannot validate analogue settling time, electrical noise, maximum GPIO edge rate, flash-induced interrupt loss or SDK low-power clock behavior. Tick wrap tests assume acquisition resumes before a complete tick-counter wrap; missing hardware edges cannot be reconstructed from software counts. The 128/250 Hz tests check conversion, not a claim that a particular board uses those rates.

Real-device validation remains necessary with normal logging enabled, a changing load, both SEL polarities and representative legacy/new Beken, BL602 and RTL devices. The patch does not change SDK revisions, GPIO trigger selection or the logging implementation.
