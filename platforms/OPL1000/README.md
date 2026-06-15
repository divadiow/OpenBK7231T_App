# OpenOPL1000 real-port target

This is the first real OpenBeken/OpenOPL1000 integration target for the Opulinks
OPL1000 A2 SDK.

Unlike the earlier bring-up web-console experiments, this target does not provide
a separate fake OpenBeken-style HTTP server. It builds and starts the normal
OpenBeken core, command engine, config layer and web server, then supplies an
OPL1000 HAL underneath it.

Current scope:

- real OpenBeken `Main_Init()` / `Main_OnEverySecond()` path
- real OpenBeken HTTP server and UI handlers
- real OpenBeken command endpoint handling
- OPL1000 STA-only Wi-Fi backend using the vendor SDK
- hardcoded initial Wi-Fi credentials for bring-up:
  - SSID: `test`
  - password: `1234abcd`
- RAM-backed config/flash-var stubs
- GPIO/PWM/ADC/OTA stubs only

The Opulinks SDK must be present at:

```text
sdk/OpenOPL1000
```

The expected SDK root contains `Demo`, `FW_Pack` and `SDK` at its top level.
The intended SDK line for OPL1000 A2/Sonoff testing is:

```text
SDK Package: MP_2.21.004
Patch_Lib: 5753
Release Date: 2022/03/23
```

Build from the repository root with:

```sh
make OpenOPL1000
```

Important limitations:

- OPL1000 SoftAP is not implemented because Opulinks state OPL1000 supports STA
  mode only.
- Configuration persistence is currently RAM-only. Settings changed through the
  web UI will not survive reboot yet.
- GPIO/PWM/ADC are placeholders for now; do not expect real pin control yet.
- OTA route/command exists through the real OBK UI, but actual OPL1000 OTA flash
  writing is not wired yet.
