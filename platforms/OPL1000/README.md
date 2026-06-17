# OpenOPL1000 platform bring-up

This is the first real-port OpenOPL1000 target. It starts the normal OpenBeken runtime rather than a separate OpenOPL1000 web-console shim.

Current scope:

- Opulinks OPL1000 A2 SDK patch entrypoint
- real OpenBeken `Main_Init()` task
- STA-only Wi-Fi bring-up through the OPL1000 HAL
- hardcoded test Wi-Fi defaults for bring-up: `test` / `1234abcd`
- normal OpenBeken HTTP/command path, trimmed to fit the A2 M3 patch image

Still intentionally incomplete:

- GPIO/PWM/ADC implementation
- persistent flash/config storage
- OTA write implementation
- SoftAP, because OPL1000 does not support SoftAP

## UART logging

For bring-up, v9 forces APS/debug UART to the same pins that show the ROM/bootloader text:

```text
IO0 = APS/debug UART TX
IO1 = APS/debug UART RX
115200 8N1
```

Expected early marker if the M3 patch reaches application init:

```text
[OpenOPL1000] Main_AppInit_patch reached; APS/debug UART is IO0/IO1 @115200
```

If you only see the ROM/bootloader text and never this marker, the M3 patch has not reached `Main_AppInit_patch()`.

## Opulinks pack-ready output

The GitHub build emits a pack-ready ZIP using the names expected by the Opulinks GUI packer:

```text
OpenOPL1000_pack_ready_<version>.zip
├─ PatchData.txt
├─ opl1000_m0.bin
└─ opl1000_app_m3.bin
```

Use those exact files in the OPL1000 Download Tool Pack tab:

```text
Script:  PatchData.txt
M0 Bin:  opl1000_m0.bin
M3 Bins: opl1000_app_m3.bin
```

v9 uses the root SDK `FW_Pack` M0/PatchData pairing, not the demo TCP_Client pairing. This matches the root SDK patch layout and includes the extra M3/M0 hardware patch words from `FW_Pack/PatchData.txt`.

## v11 Wi-Fi bring-up note

v11 keeps the v10 real OpenBeken runtime path and changes only the OPL1000 Wi-Fi glue.
The v10 log showed the scan table but no association, with only about 728 bytes free after Wi-Fi init. The SDK demo path allocates a full `wifi_scan_list_t`, which is too large under the real OBK runtime. v11 avoids that heap allocation by using a fixed small scan-record buffer, copies the matched BSSID/channel into the STA config, and removes the extra no-op OPL1000 Wi-Fi keeper task to recover heap.


## v12 Wi-Fi/lwIP sequencing note

v11 deliberately removed the extra lwIP keeper task, but that exposed an SDK sequencing problem: `lwip_net_ready()` blocks until Wi-Fi is associated and DHCP has completed. Calling it synchronously inside `HAL_ConnectToWiFi()` prevents `wifi_set_config()` and `wifi_start()` from running, so boot stops at `lwIP init start`.

v12 follows the Opulinks demos more closely: Wi-Fi init/config/start runs in the OBK path, while `lwip_network_init()` / `lwip_net_ready()` run in a small separate `opl_lwip` task. The v11 small scan buffer and BSSID/channel diagnostics are kept.

## v13 Wi-Fi bring-up note

v12 proved that the Opulinks Wi-Fi stack starts and the scan engine can see the configured SSID, but the real OBK runtime did not receive the expected `WIFI_EVENT_SCAN_COMPLETE` callback. The SDK still printed scan tables repeatedly, while no `[OpenOPL1000] scan complete event` or `wifi_connection_connect rc=...` marker appeared.

v13 therefore stops relying on the scan-complete event for association. A single `opl_wifi` worker task performs a blocking scan, reads the scan list synchronously with `wifi_scan_get_ap_list()`, matches the configured SSID, calls `wifi_connection_connect()`, then polls `wifi_sta_get_ap_info()` for association before starting lwIP/DHCP. This also avoids the previous dynamic scan-list allocation and keeps `bssid_present` cleared to match the vendor demo behaviour.

## v14 Wi-Fi bring-up note

v13 showed that `wifi_scan_start(..., true)` returns before the Opulinks scan cache has been populated in this real-port runtime. The worker read `ap_count=0`, then the vendor event-loop printed the real AP table a second or two later. v14 changes the worker to use the vendor-demo-style asynchronous scan, waits before reading `wifi_scan_get_ap_list()`, logs every delayed scan record, then calls `wifi_connection_connect()` only after the target SSID is present in the SDK scan cache. It also avoids calling `wifi_connection_disconnect_ap()` on a failed retry because v13 watchdog-reset in the SDK `opl_event_loop` shortly after the retry path.


## v15 note

v15 keeps the v9+ FW_Pack-style PatchData and pack_ready output.  The Wi-Fi worker now matches APs using the SDK's raw scan_report_t from wifi_get_scan_result(), because v14 showed the public wifi_scan_get_ap_list() result had RSSI/channel/auth populated but blank SSID fields while the SDK's own scan table printed the SSIDs correctly.

## v16 HTTP bring-up note

v15 reached STA association, DHCP and `Info:HTTP:TCP server listening`, but an incoming
browser connection failed because the generic OBK HTTP server attempted to create a
per-client `HTTP Client` task when only about 728 bytes of heap remained.

v16 keeps the same STA/raw-scan connection path, then terminates the temporary Wi-Fi
worker task after DHCP so its stack is returned.  The OPL1000 HTTP server also handles
one accepted connection synchronously on the TCP server task with small temporary
buffers instead of creating a per-client thread.


## v17 note

The v15/v16 logs proved that STA association, DHCP and the TCP listener are working, but only about 728 bytes of FreeRTOS heap remain once lwIP is up. v16 still tried to allocate request/reply buffers for an accepted browser socket and failed. v17 changes the temporary OPL1000 HTTP handling to a static-buffer micro endpoint so that a browser request can be answered without `os_malloc()` or an HTTP client task.

Temporary endpoints for bring-up:

```text
http://<device-ip>/
http://<device-ip>/cm?cmnd=status
http://<device-ip>/status
```

This is intentionally a bring-up path. It proves socket RX/TX from the real OpenBeken runtime, but the full OBK web UI remains disabled until more heap is recovered from the OPL1000 port.
