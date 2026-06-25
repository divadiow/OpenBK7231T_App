# OpenOPL1000 platform bring-up

This is the first real-port OpenOPL1000 target. It starts the normal OpenBeken runtime rather than a separate OpenOPL1000 web-console shim.

Current scope:

- Opulinks OPL1000 A2 SDK patch entrypoint
- real OpenBeken `Main_Init()` task
- STA-only Wi-Fi bring-up through the OPL1000 HAL
- hardcoded test Wi-Fi defaults for bring-up: `test` / `1234abcd`
- hardened OPL1000 HTTP transport with real OpenBeken `/cm?...` command path under test

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


## v18 notes

v18 keeps the v17 micro HTTP bring-up server but reduces TCP_server stack usage.
The v17 browser test proved that TCP accept/receive/send worked, but the module
then reset with `stack overflow: ... TCP_server`.  v18 removes local response
body buffers, makes the HTTP request probe/static accept storage global, and
reduces verbose per-request logging so the synchronous OPL1000 HTTP path can run
inside the existing TCP_server task stack.


## v19 notes - Wi-Fi-only service init

OpenOPL1000 v19 overrides the SDK service-initialisation hook with `Main_ServiceInitNoBle()`.
It recreates the normal Wi-Fi/lwIP/supplicant/controller/OTA initialisation sequence but deliberately skips `LeRtosTaskCreat()`.

The aim is to recover the BLE/LE task stack/heap for the Wi-Fi-only OpenBeken bring-up path. Expected boot log difference:

```text
[OpenOPL1000] Sys_ServiceInitNoBle: Wi-Fi services only; skipping BLE/LE task
wifiMac Task create successful
Supplicant task is created successfully!
controller_queue creates successful!
controller_queue_ble creates successful!
controller_task_create successful!
# no "LE Task create successful" line expected
```

The Opulinks pack-ready output remains:

```text
pack_ready/PatchData.txt
pack_ready/opl1000_m0.bin
pack_ready/opl1000_app_m3.bin
OpenOPL1000_pack_ready_<version>.zip
```

This is an experiment. If Wi-Fi fails earlier than v17/v18, revert to the normal SDK service-init path and continue with stack-size trimming instead.

## v20 note

v19 proved that skipping the BLE/LE task recovers a large amount of heap, but
WPA2-PSK then failed during association because the SDK security/HMAC resource
semaphores were not initialised:

```text
[scrt_res_lock_impl ...] sem is null
[nl_hmac_sha_1_impl ...] scrt_res_alloc fail
```

v20 keeps the BLE/LE task disabled, but explicitly calls `nl_scrt_Init()` in the
custom No-BLE service init before the WPA supplicant is started.  It also calls
`sys_cfg_init()` because replacing `Sys_ServiceInit_patch()` otherwise bypasses
that SDK post-service configuration step.

## v21 notes

v21 keeps the v20 Wi-Fi/BLE-disabled bring-up base, but changes the OPL1000
HTTP path from the tiny holding-page-only responder to a static-buffer,
synchronous real OpenBeken HTTP parser path.

Expected test URLs:

- `/` - real OpenBeken index route
- `/cm?cmnd=status` - real OpenBeken command route
- `/status` - tiny OPL1000 fallback JSON endpoint
- `/opl1000` - tiny OPL1000 fallback HTML endpoint

This is still deliberately single-client and synchronous on OPL1000. It avoids
heap allocation and per-client HTTP tasks because those were the source of the
pre-v20 browser crashes.

## v22 notes

v21 proved that the first `/status` browser request could be answered, but a
normal browser then opened another socket and closed it without sending a full
HTTP request.  The OPL1000 TCP server treated that client-level condition as a
server-level error and entered the HTTP restart path.

v22 is a transport-hardening build, not a full-GUI build.  It keeps the stable
v20 base and the v21 static/synchronous socket model, but changes the OPL1000
HTTP path as follows:

- empty/closed browser client sockets are treated as normal and simply closed
- transient `accept()` errors no longer enter the HTTP server restart path
- the receive timeout for accepted OPL1000 sockets is shortened to 3 seconds
- the listen backlog is raised from 0 to 1 for OPL1000
- `/`, `/status`, `/opl1000`, and `/favicon.ico` are handled by tiny static
  micro responses
- only `/cm?...` is routed through the real OpenBeken `HTTP_ProcessPacket()`
  command path
- `/cfg` and heavier GUI routes deliberately return a small diagnostic response

Initial test order:

```text
http://<device-ip>/
http://<device-ip>/status
http://<device-ip>/cm?cmnd=status
http://<device-ip>/cfg
```

The expected result is that repeated browser requests do not reset the TCP
server or the module.  If `/cm?cmnd=status` works repeatedly, the next step is to
recover more heap/stack and then enable selected lightweight OpenBeken pages.

## v24 notes

v22 was intended to harden browser/socket lifecycle handling, but the larger
real `HTTP_ProcessPacket()` command route made the build regress before DHCP on
some boots.  v24 deliberately returns to the v20-style micro HTTP transport and
keeps the v22 socket hardening, but does **not** route any request through
`HTTP_ProcessPacket()`.

Instead, `/cm?cmnd=...` is a minimal direct bridge into the already-running
OpenBeken command engine via `CMD_ExecuteCommand()`.  This is not yet the normal
Tasmota/OpenBeken JSON command response, but it proves command dispatch over
HTTP without pulling in the full web route/page machinery.

Expected test order:

```text
http://<device-ip>/
http://<device-ip>/status
http://<device-ip>/cm?cmnd=WifiState
http://<device-ip>/cm?cmnd=backlog%20led_enableAll%201;led_basecolor_rgb%20FF0000
http://<device-ip>/cfg
```

`/cfg` remains intentionally disabled.  The next goal after v24 is to confirm
that repeated requests and direct command dispatch are stable, then add only the
specific command/status response features needed for integration before trying
full GUI pages again.


## v24 micro UI notes

V24 keeps the stable v23 transport: BLE/LE remains disabled, `nl_scrt_Init()` remains enabled, HTTP stays single-client and synchronous, and the full OpenBeken HTTP/page router is still not used by default.

The root page now exposes a tiny form-based command UI. `/cm?cmnd=...` runs the already-registered OpenBeken command engine directly and returns compact JSON with the command return code. This deliberately does not capture full Tasmota JSON command output yet; it proves stable command dispatch without pulling the full HTTP router back into the constrained OPL1000 build.


## v25 note

v25 keeps the v24 micro-HTTP/direct-command approach but changes the Wi-Fi worker DHCP phase. It no longer blocks forever inside `lwip_net_ready()`. After `lwip_net_start(WIFI_MODE_STA)`, it polls the `st1` netif for a non-zero IPv4 address, logs the acquired address, then terminates the temporary Wi-Fi worker so its stack can be reclaimed. Expected heap after DHCP should be higher than v24's ~6648 bytes if the SDK releases the worker stack cleanly.

## v35 split-M3 high-tail 15 KB SHM probe

v35 is based on the stable v25 OpenOPL1000 line and deliberately does **not**
re-enable the full stock OpenBeken HTTP UI.

The only architectural experiment in v35 is the vendor `Expand_M3_RAM` style
split-M3 packaging path:

- normal M3 patch image is linked for `0x004164A0`
- a tiny probe function is linked into section `SHM_REGION` at `0x80000400`, the high 15 KB tail of the 16 KB SHM block
- the build emits two M3 binaries from one ELF:
  - `opl1000_app_m3_main.bin` loaded at `0x004164A0`
  - `opl1000_app_m3_shm.bin` loaded at `0x80000400`
- `pack/PatchData.txt` references both M3 binaries

Expected boot marker if the split-M3 binary was packed and loaded correctly:

```text
[OpenOPL1000] split-M3 v35-tail15k: shm_fn=0x80000401 result=0x........
```

If the marker does not appear, or the device resets immediately after the
`OpenBeken platform port` line, the second M3 image probably was not packed or
loaded correctly.

Use the generated pack-ready directory.  The pack-ready ZIP contains:

```text
PatchData.txt
opl1000_m0.bin
opl1000_app_m3_main.bin
opl1000_app_m3_shm.bin
```

In the Opulinks pack tool, use the `PatchData.txt` from the pack-ready folder
and include both M3 binary files.  This is intentionally different from the
single-M3 v25 pack-ready layout.


## v35 note

v29b proved the split-M3 pack path and executable SHM code at `0x80000000`, but repeated resets occurred immediately after `wifi_connection_connect rc=0`. v29c, v30, and v31 proved the high 1 KB, 2 KB, and 3 KB tails. v33 then jumped to `0x80001800` and proved the high 10 KB tail survives Wi-Fi association, lwIP, DHCP, worker termination, and TCP server startup.

v35 keeps the same v25 runtime base and split-M3 pack format but jumps down to `0x80000400` to test whether the high 15 KB tail is safe. This intentionally leaves only the low 1 KB of the 16 KB SHM block unused.

Expected marker:

```text
[OpenOPL1000] split-M3 v35-tail15k: shm_fn=0x80000401 result=0x........
```

If Wi-Fi connects and reaches DHCP, the high 15 KB SHM tail is potentially usable. If it resets after `wifi_connection_connect rc=0`, the boundary is somewhere between the proven high 10 KB tail and this 15 KB probe; continue with a binary-search style probe rather than 1 KB increments.


## v35 note

v34 proved that the high 14 KB SHM tail at `0x80000800-0x80004000` can execute split-M3 code and survive Wi-Fi association, lwIP, DHCP, worker termination, and TCP server startup.

v35 moves the split-M3 SHM region down to `0x80000400`, testing the high 15 KB tail and leaving only the low 1 KB of the 16 KB SHM block unused.

Expected marker:

```text
[OpenOPL1000] split-M3 v35-tail15k: shm_fn=0x80000401 result=0x........
```

If this survives to DHCP and TCP server startup, the remaining unsafe/owned area is likely below `0x80000400`. If it resets after `wifi_connection_connect rc=0`, use v34/high-14-KB as the proven-safe split-M3 boundary.


## v36 split-M3 HTTP-helper migration probe

v35 proved that the upper 15 KB of the 0x80000000-0x80004000 shared-memory window can execute split-M3 code while Wi-Fi STA, DHCP and the TCP server remain stable. v36 keeps the split-M3 base at 0x80000400 and moves selected OPL1000 micro-HTTP helper functions into `SHM_REGION`.

Expected boot marker:

```text
[OpenOPL1000] split-M3 v36-http-shm: shm_fn=0x80000401 result=0xb881be0b
```

Then test:

```text
http://172.16.62.218/
http://172.16.62.218/status
http://172.16.62.218/cm?cmnd=Status
http://172.16.62.218/cm?cmnd=Power%20Toggle
```

This is not expected to increase `xPortGetFreeHeapSize()` by itself. The success criterion is that real HTTP requests execute through relocated helper code without resetting or corrupting Wi-Fi.


## v37b split-M3 micro-UI and buffer migration probe

v36 proved that selected OPL1000 micro-HTTP helper functions can execute from the proven-safe split-M3 SHM tail at `0x80000400-0x80003fff` while Wi-Fi, DHCP and HTTP traffic remain stable.

v37b keeps the same split-M3 base and extends the experiment by moving the larger OPL1000 micro-UI working set into `SHM_REGION`:

```text
0x80000400-0x80003fff = split-M3 SHM tail used by OpenOPL1000
0x80000000-0x800003ff = deliberately avoided vendor/IPC-owned bottom area
```

Moved in v37b:

```text
- micro request buffer
- micro reply buffer
- status/page/command buffers
- HTTP status/content-type strings
- JSON response format strings
- homepage HTML format string
- selected request-prefix strings
- existing micro-HTTP helper functions from v36
```

Expected boot marker:

```text
[OpenOPL1000] split-M3 v37b-shm-ui: shm_fn=0x80000401 result=0xb881be0b
```

Expected test URLs after DHCP:

```text
http://<device-ip>/
http://<device-ip>/status
http://<device-ip>/cm?cmnd=Status
http://<device-ip>/cm?cmnd=Power%20Toggle
http://<device-ip>/cm?cmnd=Power%20On
http://<device-ip>/cm?cmnd=Power%20Off
```

This version still deliberately leaves full `HTTP_ProcessPacket()` disabled. v37b also fixes the GCC/LTO section-type conflict seen in v37 by using separate `.shm_text`, `.shm_rodata`, and `.shm_data` input sections that are all collected into the same split-M3 SHM output region. It tests whether meaningful UI code, constants and read/write HTTP buffers can live in the split-M3 SHM tail without destabilising Wi-Fi or the TCP server.


## v40 targeted multi-pass scan

v39 proved that the lower-level `wifi_scan_req_by_cfg()` call returns success on A2 but does not populate the custom expanded scan buffer in this runtime (`expanded_count=0`). The SDK still retained only the usual small all-SSID result set, so a weak `test` phone hotspot could be displaced by nearby stronger APs.

v40 returns to the known-good v37b micro-UI baseline and changes the scan strategy instead of trying to enlarge the all-SSID cache. The worker now performs several targeted active scans with `wifi_scan_config_t.ssid = "test"`, then one broad fallback scan. This should stop unrelated SSIDs from displacing the requested AP. If the scan cache still does not contain the target, v40 makes one final no-BSSID `wifi_connection_connect()` attempt so the vendor connection path can try its own all-channel search.

Expected marker:

```text
[OpenOPL1000] split-M3 v40-targeted-scan: shm_fn=0x80000401 result=0xb881be0b
```

Expected scan diagnostics:

```text
[OpenOPL1000] worker: targeted scan pass 1/4 for SSID 'test' ...
[OpenOPL1000] worker: wifi_scan_start(targeted) rc=0 ...
[OpenOPL1000] worker: raw scan report ptr=... count=...
```

Success criteria: the device should find the `test` AP even when it is not one of the strongest few APs in the broad scan list, then associate, get DHCP, start the TCP server, and keep the v37b-style steady heap.
