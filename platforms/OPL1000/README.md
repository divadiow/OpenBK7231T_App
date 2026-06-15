# OpenOPL1000

First-stage OpenOPL1000 target for the Opulinks OPL1000 A2 SDK.

This target is still SDK-native rather than the full upstream OpenBeken runtime, but it now gives the device a proper OpenOPL1000 home page instead of just a proof/debug page.

## SDK layout

The build expects the A2 SDK submodule at:

```text
sdk/OpenOPL1000
```

The submodule root must contain the SDK root, not only `SDK/APS_PATCH`:

```text
sdk/OpenOPL1000/
├─ Demo/
├─ FW_Pack/
└─ SDK/
   ├─ APS/
   └─ APS_PATCH/
```

## Wi-Fi

This stage remains STA-only because the OPL1000 A2 SDK does not expose SoftAP support.

Default credentials are hardcoded at build time from `platforms/OPL1000/Makefile`:

```text
SSID: test
Password: 1234abcd
```

## Web routes

After DHCP, browse to the assigned IP address.

Implemented routes:

```text
/
/index
/index.html
/cfg
/cfg_wifi
/cfg_pins
/logs
/ota
/about
/cmd?cmnd=status
/cm?cmnd=status
/api/cmd?cmnd=status
/api/status
/status.json
```

The main page intentionally looks and routes more like a normal OpenBeken/OpenOPL1000 device page, but the deferred areas are clearly marked as not wired yet.

## Current scope

Working in this stage:

```text
STA Wi-Fi
DHCP
TCP listen socket
HTTP page routing
OpenOPL1000 home page
/cm?cmnd= command endpoint
JSON status
Wi-Fi scan/reconnect commands
basic GPIO diagnostics
```

Deferred until later:

```text
persistent flash config
pin role configuration
PWM/channel model
MQTT
OTA web update handler
full OpenBeken command registry
full OBK runtime loop
```
