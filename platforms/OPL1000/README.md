# OpenOPL1000 first-stage target

This is a first-stage Opulinks OPL1000 A2 bring-up target for OpenBeken.

It does not implement the full OpenBeken platform HAL yet.  It builds a small
SDK-native M3 patch image named `OpenOPL1000` and uses the Opulinks station API
to connect to this fixed test AP:

- SSID: `test`
- Password: `1234abcd`

The OPL1000 SDK must be present as a submodule or directory at:

```text
sdk/OpenOPL1000
```

The expected SDK is the OPL1000 A2 SDK matching `MP_2.21.004` / `Patch_Lib 5753`.
A later private repo can be wired in with a normal submodule entry, for example:

```ini
[submodule "sdk/OpenOPL1000"]
	path = sdk/OpenOPL1000
	url = https://github.com/<owner>/OpenOPL1000.git
```

Build from the repository root with:

```sh
make OpenOPL1000
```

GitHub Actions uses `arm-none-eabi-gcc` release `8-2019-q3` for this target.
