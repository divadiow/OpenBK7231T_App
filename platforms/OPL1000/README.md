# OpenOPL1000 platform bring-up

This is the first real-port OpenOPL1000 target. It starts the normal OpenBeken runtime rather than a separate OpenOPL1000 web-console shim.

Current scope:

- Opulinks OPL1000 A2 SDK patch entrypoint
- real OpenBeken `Main_Init()` task
- STA-only Wi-Fi bring-up through the OPL1000 HAL
- hardcoded test Wi-Fi defaults for bring-up: `OPL1000_AP` / `1234abcd`
- hardened OPL1000 HTTP transport with real OpenBeken `/cm?...` command path under test
- persistent OpenBeken main config via an OPL1000 FIM extension zone

Still intentionally incomplete:

- PWM/ADC implementation
- flash-vars persistence for boot/channel/energy retained values
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

v15 keeps the v9+ FW_Pack-style PatchData and pack_ready output. The Wi-Fi worker now matches APs using the SDK's raw scan_report_t from wifi_get_scan_result(), because v14 showed the public wifi_scan_get_ap_list() result had RSSI/channel/auth populated but blank SSID fields while the SDK's own scan table printed the SSIDs correctly.

## v16 HTTP bring-up note

v15 reached STA association, DHCP and `Info:HTTP:TCP server listening`, but an incoming
browser connection failed because the generic OBK HTTP server attempted to create a
per-client `HTTP Client` task when only about 728 bytes of heap remained.

v16 keeps the same STA/raw-scan connection path, then terminates the temporary Wi-Fi
worker task after DHCP so its stack is returned. The OPL1000 HTTP server also handles
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
then reset with `stack overflow: ... TCP_server`. v18 removes local response
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
custom No-BLE service init before the WPA supplicant is started. It also calls
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
HTTP request. The OPL1000 TCP server treated that client-level condition as a
server-level error and entered the HTTP restart path.

v22 is a transport-hardening build, not a full-GUI build. It keeps the stable
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
server or the module. If `/cm?cmnd=status` works repeatedly, the next step is to
recover more heap/stack and then enable selected lightweight OpenBeken pages.

## v24 notes

v22 was intended to harden browser/socket lifecycle handling, but the larger
real `HTTP_ProcessPacket()` command route made the build regress before DHCP on
some boots. v24 deliberately returns to the v20-style micro HTTP transport and
keeps the v22 socket hardening, but does **not** route any request through
`HTTP_ProcessPacket()`.

Instead, `/cm?cmnd=...` is a minimal direct bridge into the already-running
OpenBeken command engine via `CMD_ExecuteCommand()`. This is not yet the normal
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

`/cfg` remains intentionally disabled. The next goal after v24 is to confirm
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

Use the generated pack-ready directory. The pack-ready ZIP contains:

```text
PatchData.txt
opl1000_m0.bin
opl1000_app_m3_main.bin
opl1000_app_m3_shm.bin
```

In the Opulinks pack tool, use the `PatchData.txt` from the pack-ready folder
and include both M3 binary files. This is intentionally different from the
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

## v40b targeted multi-pass scan

v39 proved that the lower-level `wifi_scan_req_by_cfg()` call returns success on A2 but does not populate the custom expanded scan buffer in this runtime (`expanded_count=0`). The SDK still retained only the usual small all-SSID result set, so a weak `test` phone hotspot could be displaced by nearby stronger APs.

v40b returns to the known-good v37b micro-UI baseline and changes the scan strategy instead of trying to enlarge the all-SSID cache. The worker now performs four targeted active scans with `wifi_scan_config_t.ssid = "test"`, then one broad fallback scan. This should stop unrelated SSIDs from displacing the requested AP. If the scan cache still does not contain the target, v40b makes one final no-BSSID `wifi_connection_connect()` attempt so the vendor connection path can try its own all-channel search.

Expected marker:

```text
[OpenOPL1000] split-M3 v40b-targeted-scan: shm_fn=0x80000401 result=0xb881be0b
```

Expected scan diagnostics:

```text
[OpenOPL1000] worker: targeted scan pass 1/4 for SSID 'test' ...
[OpenOPL1000] worker: wifi_scan_start(targeted) rc=0 ...
[OpenOPL1000] worker: raw scan report ptr=... count=...
```

Success criteria: the device should find the `test` AP even when it is not one of the strongest few APs in the broad scan list, then associate, get DHCP, start the TCP server, and keep the v37b-style steady heap.

Build note: v40b also defines `OPENOPL1000_SCAN_PASS_COUNT`; v40 missed this macro and failed to compile.

## v41 direct-connect-first probe

v41 keeps the v40b split-M3 and micro-HTTP base, but changes the Wi-Fi association order. The worker now first calls the vendor connection path with only the configured SSID/password and `bssid_present = 0`, then waits briefly for association. If that does not associate, it falls back to the v40b targeted-scan passes and final broad scan.

The aim is to avoid depending on the SDK's small retained public scan list in crowded RF environments. If the vendor connection manager can perform its own internal scan, the `test` hotspot should connect even when it is not visible in the top few public scan records.

Expected marker:

```text
[OpenOPL1000] split-M3 v41-direct-first: shm_fn=0x80000401 result=0xb881be0b
```

Useful log sequence:

```text
[OpenOPL1000] worker: direct no-BSSID wifi_connection_connect rc=...
[OpenOPL1000] associated with 'test' ...
```

If direct connection does not associate, the fallback should still show the v40b targeted scan messages:

```text
[OpenOPL1000] worker: direct connect did not associate after 12 seconds, trying targeted scans ...
[OpenOPL1000] worker: targeted scan pass 1/5 for SSID 'test' ...
```

## v42 full OpenBeken HTTP probe

v42 changes the test Wi-Fi credentials to:

```text
SSID: test
Password: 1234abcd
```

It also removes the OPL1000 micro-only HTTP router from the active TCP server path. Accepted browser sockets are still handled synchronously and one-at-a-time on OPL1000, but the request now goes through the real OpenBeken `HTTP_ProcessPacket()` route using static RX/reply scratch buffers in the proven split-M3 SHM tail.

Expected marker:

```text
[OpenOPL1000] split-M3 v42-full-obk-http: shm_fn=0x80000401 result=0xb881be0b
```

Expected HTTP log marker:

```text
OPL1000 full OBK request len ...
OPL1000 full OBK reply len ...
```

The OPL1000 `OBK_OPL1000_MINIMAL_WEB` gate is removed for this probe, so stock routes like `/`, `/index`, `/cfg`, `/cfg_pins`, `/cmd_tool`, `/cm?cmnd=Status`, and `/ota` are routed through the normal OBK HTTP code again. Non-web services and drivers remain disabled in the OPL1000 profile.

The TCP server stack is raised from `0x800` to `0xC00` for this full-route probe. If this still stack-overflows, try `0x1000` next and compare the steady heap loss. If it does not overflow but pages are truncated or missing, inspect the `postany()` streaming path and the per-request `replymaxlen` before increasing heap allocations.

### v42 link result

Full classic OpenBeken HTTP now links in the split-M3 OPL1000 image. The build uses the normal `HTTP_ProcessPacket()` path, but keeps REST/webapp API registration disabled with `OBK_OPL1000_NO_REST` because the first full-classic-HTTP link was still over the M3 patch window with REST enabled.

The key memory changes were:

```text
OPL1000 log line buffer: 1024 -> 384 bytes
OPL1000 log ring:        4096 -> 1024 bytes
Moved to SHM tail: HTTP request/reply buffers, in-RAM config sector, g_cfg,
                   command tokenizer expansion scratch, command hash table,
                   and the OPL1000 log buffers.
```

Verified WSL build:

```text
make -C platforms/OPL1000 SDK_ROOT=../../sdk/OpenOPL1000 APP_NAME=OpenOPL1000 APP_VERSION=codex_v42 OBK_VARIANT=default -j 4

IRAM1: 169264 B / 170848 B (99.07%)
SHM:    13384 B / 15360 B  (87.14%)
```

The pack-ready output is written to:

```text
output/codex_v42/OpenOPL1000_pack_ready_codex_v42.zip
output/codex_v42/pack_ready/
```

The Makefile packaging step now falls back to `python3 -m zipfile` when the WSL image does not have `zip` installed.

## v43 SDK scan-cache Wi-Fi probe

v42 proved that the full classic OpenBeken HTTP path can run on OPL1000, but the attached serial log showed a weak Wi-Fi bring-up pattern:

```text
[OpenOPL1000] worker: direct no-BSSID wifi_connection_connect rc=-1 ...
[OpenOPL1000] worker: matched SSID 'test' ... wifi_connection_connect rc=-1
[DRV]:WPA: Ignore(1)
...
[OpenOPL1000] worker: direct no-BSSID wifi_connection_connect rc=0 ...
[CTRL_WIFI]:Connecting to 3c:37:86:42:32:23
```

The SDK station examples use the public event-driven scan/connect flow and do not force a BSSID into the station config. v43 follows that hint: it keeps direct no-BSSID connect as the real association path, uses mixed scans to warm the SDK scan cache, records the best seen BSSID/channel for diagnostics only, and stops calling `wifi_connection_connect()` with a manually injected BSSID.

Expected marker:

```text
[OpenOPL1000] split-M3 v43-sdk-cache-connect: shm_fn=0x80000401 result=0xb881be0b
```

Also changed in v43:

```text
Removed static wifi_scan_list_t scan buffer; scan diagnostics now use wifi_scan_get_ap_num().
HAL_GetWifiStrength() reads wifi_connection_get_rssi(), then wifi_sta_get_ap_info().
WiFI_GetMacAddress() rejects all-zero/all-FF SDK MAC reads before falling back.
```

### v43 result and v44 scan-first follow-up

The v43 hardware log confirmed the MAC/name and RSSI fixes:

```text
OpenOPL1000_18AFEE53
Wifi RSSI: Excellent (-43dBm)
Short name: opl100018AFEE53
```

It also showed the first direct connect still wasted about 18 seconds before the scan warmed the SDK's AP table. The successful sequence was:

```text
targeted scan pass 1/4 for SSID 'test'
best SSID 'test' bssid=3c:37:86:42:32:23 ch=6 rssi=-41
direct no-BSSID wifi_connection_connect rc=0
associated with 'test'
got IP 192.168.1.120
```

v44 makes that successful path the default: scan first, connect through the SDK no-BSSID path after the scan result, and only try a final direct connect if all scan passes fail.

Expected marker:

```text
[OpenOPL1000] split-M3 v44-scan-first-connect: shm_fn=0x80000401 result=0xb881be0b
```

### v44 result and v45 log trimming

The v44 hardware log confirmed scan-first is the right default for this SDK:

```text
Time 4:  targeted scan pass 1/4
Time 8:  best SSID 'test' bssid=3c:37:86:42:32:23 ch=6 rssi=-40
Time 10: wifi_connection_connect rc=0
Time 11: associated with 'test'
Time 13: got IP 192.168.1.120
Time 14: worker terminated, heap back to 7688
```

Compared with v43, this avoids the failed pre-scan direct connect and reaches HTTP roughly 18 seconds earlier on the same network. v45 keeps the v44 scan-first behavior, but compiles out raw per-AP scan dumps by default and suppresses duplicate association prints from the polling loop.

Expected marker:

```text
[OpenOPL1000] split-M3 v45-quiet-scan-first: shm_fn=0x80000401 result=0xb881be0b
```

### v45 result and v46 HTTP trace gate

The v45 hardware log kept the v44 timing and heap behavior:

```text
Time 4:  targeted scan pass 1/4
Time 10: wifi_connection_connect rc=0
Time 13: got IP 192.168.1.120
Time 14: worker terminated, heap back to 7688
```

Full classic HTTP continued to recover to the same steady heap after browser traffic. The remaining high-volume log output was the OPL1000 per-request HTTP trace:

```text
OPL1000 sync accepted ...
OPL1000 full OBK request len ...
OPL1000 full OBK reply len ...
```

v46 leaves the proven full OBK HTTP route in place, but gates those per-request trace lines behind `OPL1000_HTTP_TRACE` in `new_tcp_server.c`. The flag defaults to `0`; set it to `1` when debugging HTTP heap/request behavior again.

Expected marker:

```text
[OpenOPL1000] split-M3 v46-quiet-http: shm_fn=0x80000401 result=0xb881be0b
```

### v46 result and v47 netinfo trace gate

The v46 hardware log confirmed the HTTP trace gate worked: browser requests no longer emit the OPL1000 accept/request/reply lines, full HTTP still responds, and heap remains steady at 7688 bytes after the worker exits.

The remaining OPL1000-specific periodic trace was:

```text
[OpenOPL1000] IP=192.168.1.120 GW=192.168.1.1 MASK=255.255.255.0
```

v47 gates `HAL_PrintNetworkInfo()` behind `OPENOPL1000_NETINFO_TRACE`, defaulting to `0`. Set it to `1` if the periodic IP/GW/MASK line is useful during network debugging.

Expected marker:

```text
[OpenOPL1000] split-M3 v47-quiet-netinfo: shm_fn=0x80000401 result=0xb881be0b
```

## v48 GPIO baseline

v47 was the end of the one-change-at-a-time Wi-Fi/HTTP proof phase. It confirmed the quiet full-HTTP baseline: scan-first Wi-Fi comes up quickly, HTTP traffic no longer emits per-request OPL1000 trace lines, and the worker stack is returned with steady heap around 7688 bytes.

v48 starts the larger platform-functionality pass. The OPL1000 pin HAL no longer uses an in-RAM stub state table. It now routes safe user GPIOs through the SDK pin/GPIO APIs:

```text
GPIO-capable: IO2-IO11, IO16-IO23
Reserved for now: IO0/IO1 debug UART, IO12-IO15 SPI flash, other non-user GPIOs
```

Implemented:

```text
HAL_PIN_Setup_Output()
HAL_PIN_SetOutputValue()
HAL_PIN_Setup_Input()
HAL_PIN_Setup_Input_Pullup()
HAL_PIN_Setup_Input_Pulldown()
HAL_PIN_ReadDigitalInput()
```

This should make simple OBK pin roles like Relay, LED, Button, and digital input testable on OPL1000 without touching other platforms. PWM remains disabled for this build until the OPL1000 PWM channel mapping/allocation is wired deliberately.

Expected marker:

```text
[OpenOPL1000] split-M3 v48-gpio-baseline: shm_fn=0x80000401 result=0xb881be0b
```

## v49 persistent OBK config

v49 wires OpenBeken main config into the Opulinks FIM layer instead of keeping a 4 KB shadow in RAM/SHM. The SDK examples extend FIM by registering a project-owned zone, so the OPL1000 port now does the same:

```text
Zone 1 base: 0x00090000
Block size:  0x1000
Blocks:      3 (swap + group capacity)
File ID:     0x01010001
Record size: sizeof(mainConfig_t)
```

This is deliberately registered from `Main_FlashLayoutUpdate()` before FIM init completes, and the OpenBeken config backend now uses `MwFim_FileRead()` / `MwFim_FileWrite()`. First boot still falls through the normal OBK CRC/ident mismatch path and creates defaults; after the first save, changes should survive reboot.

Build result:

```text
v48: IRAM1 167752 / 170848, SHM 13384 / 15360
v49: IRAM1 167856 / 170848, SHM  9288 / 15360
```

So this adds persistent config with about 104 bytes more IRAM1 while freeing 4096 bytes of SHM.

## v50 persistent flash-vars

v50 keeps the proven 15 KB split-M3 SHM layout and adds the small OpenBeken
flash-vars block to the same OPL1000 FIM zone as the main config:

```text
Main config file ID: 0x01010001, size sizeof(mainConfig_t)
Flash-vars file ID: 0x01010002, size 64 bytes
FIM zone:            0x00090000, 3 x 4 KB blocks
```

This moves the OPL1000 `HAL_FlashVars_*()` implementation beyond the previous
RAM-only stub. Boot counters, retained channel values, LED state storage, and
energy totals now use `MwFim_FileRead()` / `MwFim_FileWrite()` on OPL1000 while
remaining isolated from other platforms.

The boot marker is now:

```text
[OpenOPL1000] split-M3 v50-persist-flashvars: shm_fn=0x80000401 result=0xb881be0b
```

The Makefile also now only passes `-no-pie` when the selected ARM GCC supports
it. This allows the older SDK-era `arm-none-eabi-gcc` 5.4 toolchain to build the
port; GCC 13 in MSYS linked larger newlib support code and overflowed the M3
patch window in this local test.

Build result with `C:\gcc-arm-none-eabi-5_4-2016q3`:

```text
IRAM1: 155764 / 170848
SHM:     9288 / 15360
```

Pack-ready output:

```text
output/codex_v50/OpenOPL1000_pack_ready_codex_v50.zip
```

## v51 SDK reset path

The v50 hardware log showed an OBK-requested reboot reaching the next bootloader
line and then sitting for several seconds before the full boot restarted. The
OPL1000 HAL had been using CMSIS `NVIC_SystemReset()`, but the Opulinks SDK
reset path and examples use `Hal_Sys_SwResetAll()`.

The SDK implementation first switches the APS clock back to XTAL, waits for that
clock switch to settle, and then asserts `SYS_SW_RESET_ALL`. A source comment in
the SDK says reset can hang if the APS clock is still on the high-speed clock
source, which matches the delayed/half-reset symptom.

v51 changes `HAL_RebootModule()` to call `Hal_Sys_SwResetAll()` first, with
`NVIC_SystemReset()` left only as a fallback if the SDK reset ever returns.

Expected marker:

```text
[OpenOPL1000] split-M3 v51-sdk-reset: shm_fn=0x80000401 result=0xb881be0b
```

## v52 RAM/reboot diagnostics

v51 fixed the delayed soft-reset symptom with the SDK reset path, but the
hardware log still did not show the usual OpenBeken reboot countdown. v52 keeps
the reset path unchanged and adds an OPL1000-only log inside
`RESET_ScheduleModuleReset()` so the next test can distinguish a normal
scheduled OBK reboot from a direct `HAL_RebootModule()` path.

v52 also prints a compact RAM/SHM layout at boot. The SDK Expand_M3_RAM demo
advertises the full 16 KB shared-memory block at `0x80000000..0x80003fff`, but
the SDK IPC headers also define M0/M3 IPC shared memory starting at
`0x80000000`. OpenOPL1000 therefore keeps the first 1 KB as a guard and loads
its split-M3 section at `0x80000400`.

Expected new markers:

```text
[OpenOPL1000] split-M3 v52-ram-reset-diag: shm_fn=0x80000401 result=0xb881be0b
[OpenOPL1000] RAM layout: bss_end=... iram_free_to_440000=... shm=0x80000400.....
[OpenOPL1000] IPC dyn addrs: bss=... dbg=... sta=... ps=...
[OpenOPL1000] SHM guard sample: 80000000=...
[OpenOPL1000] module reboot scheduled in 3 seconds
```

Build result with `C:\gcc-arm-none-eabi-5_4-2016q3`:

```text
IRAM1: 156308 / 170848
SHM:     9288 / 15360
```

## v54 flashvars FIM size fix

The latest first-boot hardware log showed main config being created correctly,
but retained flash variables still needed one OPL1000-specific fix. The SDK FIM
table was registering the flashvars record with the shared
`MAGIC_FLASHVARS_SIZE` value of 64 bytes, while the OPL1000 build's
`FLASH_VARS_STRUCTURE` is 80 bytes. `MwFim_FileWrite()` checks the registered
record size, so `HAL_FlashVars_*()` writes were rejected even though the FIM
file ID and sector layout were valid.

v54 changes only the OPL1000 FIM table size for flashvars to
`sizeof(FLASH_VARS_STRUCTURE)`. The main config FIM version stays unchanged, so
this does not deliberately invalidate existing `CFG_Holder` storage.

The confirming diagnostic build printed:

```text
[OpenOPL1000] flashvars size=80 fim_size=80 magic=64
[OpenOPL1000] flashvars FIM read miss, using defaults
```

The important result was that `flashvars FIM write failed` disappeared. A read
miss on a freshly erased device is expected until the first retained value is
written.

Expected marker:

```text
[OpenOPL1000] split-M3 v54-flashvars-size: shm_fn=0x80000401 result=0xb881be0b
```

## v55 webapp route

The classic index page exposes `Launch Web Application`, but OPL1000 still
returned `Not found.` for `/app`. This was not because `ENABLE_HTTP_WEBAPP` was
off; it is enabled in the shared config. The missing piece was the REST callback
registration. v42 deliberately kept `OBK_OPL1000_NO_REST` enabled to avoid the
first full-classic-HTTP image overflowing the M3 patch window, and that also
left the `/app` callback unregistered.

v55 keeps `OBK_OPL1000_NO_REST` in place and registers only the lightweight
`/app` callback on OPL1000. Other platforms still call `init_rest()` and get the
same `/api/` GET, `/api/` POST, and `/app` callbacks as before. OPL1000 gets
just the launcher page, which emits the configured remote webapp URL and should
not pull the heavier REST API surface into active use yet.

Expected marker:

```text
[OpenOPL1000] split-M3 v55-webapp-route: shm_fn=0x80000401 result=0xb881be0b
```

## v56 full REST registration

v55 proved that the `/app` launcher can open, but the remote OpenBeken webapp
expects the normal `/api/...` endpoints after it loads. v56 therefore removes
the OPL1000 `OBK_OPL1000_NO_REST` profile gate so OPL1000 follows the normal
`init_rest()` path.

This registers the standard REST callbacks:

```text
/api/ GET
/api/ POST
/app GET
```

The code for these handlers was already linked into the OPL1000 image via
`rest_interface.c`; v56 mainly adds the two `/api/` callback registrations at
runtime. This is the proper compatibility test for the hosted webapp, with the
main risk being the extra heap used by the callback table entries and whatever
the webapp requests through REST once loaded.

The first GCC 13 link overflowed IRAM by 4280 bytes. OPL1000 now excludes the
currently unusable REST flash/OTA endpoints (`/api/ota` and `/api/flash/...`)
and places the REST router text/strings into the already-proven split-M3 SHM
tail via the OPL1000 linker script.

Build result with WSL `arm-none-eabi-gcc` 13.2.1:

```text
IRAM1: 169704 / 170848
SHM:    14524 / 15360
```

Expected marker:

```text
[OpenOPL1000] split-M3 v56-full-rest: shm_fn=0x80000401 result=0xb881be0b
```

## v57 OPL1000-only OTA/httpclient prune

v56 proved the hosted webapp works with the normal REST callback registration,
but OPL1000 still had no implemented OTA flash writer. The REST flash/OTA
endpoints were already disabled for OPL1000, so v57 also removes the generic
HTTP client sources from the OPL1000 makefile and skips registering the
`ota_http` command on OPL1000 only.

This is deliberately scoped to the OPL1000 profile. Other platforms still build
the HTTP client and register `ota_http` as before.

Build result with WSL `arm-none-eabi-gcc` 13.2.1:

```text
IRAM1: 169616 / 170848
SHM:    14524 / 15360
```

Expected marker:

```text
[OpenOPL1000] split-M3 v57-ota-prune: shm_fn=0x80000401 result=0xb881be0b
```

## v58 OPL1000-only SDK initializer prune

The v57 image had the full classic HTTP UI and hosted `/app` webapp working, but
only about 1.2 KB of M3 patch-window headroom remained. v58 keeps the change
inside the OPL1000 platform and uses linker `--wrap` entries to skip vendor SDK
initializers that OpenBeken does not use at runtime:

```text
wpa_cli_func_init_patch
at_func_init_patch
Diag_PatchInit
le_ctrl_pre_patch_init
```

The OPL1000 makefile also drops `__AT_CMD_TASK__`, because the AT parser/task is
not exposed by the OpenBeken profile. ROM UART mode helpers remain available via
the SDK symbol file.

`LeHostPatchAssign` was tested as a possible BLE-side prune, but it produced no
link-size change, so it was not kept.

Build result with WSL `arm-none-eabi-gcc` 13.2.1:

```text
v57 IRAM1: 169616 / 170848
v58 IRAM1: 163444 / 170848
v58 SHM:    14524 / 15360
```

This gives back 6172 bytes of IRAM1 headroom. The boldest cut is
`le_ctrl_pre_patch_init`, so if hardware testing shows Wi-Fi association,
reboot, or early boot oddities, that wrapper is the first one to re-test.

Expected marker:

```text
[OpenOPL1000] split-M3 v58-sdk-prune: shm_fn=0x80000401 result=0xb881be0b
```

Hardware check from the first v58 boot log:

```text
[OpenOPL1000] split-M3 v58-sdk-prune: shm_fn=0x80001255 result=0xb881be0b
[OpenOPL1000] RAM layout: bss_end=0x0043e314 iram_free_to_440000=7404 shm=0x80000400..0x80003cbc used=14524 free=836
```

The SHM probe function address moved as the image layout changed; the important
part is that the probe still returned `0xb881be0b`. The unit connected to
`OPL1000_AP`, got DHCP address `192.168.1.120`, started the TCP server, and the
hosted webapp command path worked:

```text
Info:HTTP:TCP server listening
Info:CMD:[WebApp Cmd 'loglevel 3' Result] OK
```

The log still contains `[CLI]WPA: rssi=...` prints. Wrapping
`wpa_cli_func_init_patch` removed the vendor CLI command-table init, but this
RSSI print comes from another SDK WPA/status path and remains a separate cleanup
candidate.

## v59 OPL1000 pin labels and ping HTTP prune

v59 adds SDK-derived pin labels to the OPL1000 HAL so the config page shows the
main likely hardware function next to each IO. This is informational only: GPIO
still works as before, and PWM/ADC remain disabled/stubbed until they are wired
and tested properly.

The labels are based on the SDK project pinmux defaults and alternatives:
debug UART on IO0/IO1, UART/SPI/AUX labels through IO17, and PWM-capable labels
on IO18..IO23. IO20/IO21 also note their current ICE_M3 default functions.

v59 also disables the Ping Watchdog HTTP menu/page/router for OPL1000 only with
`#undef ENABLE_HTTP_PING`. The deeper ping watchdog feature was not enabled for
this profile, so this removes dead HTTP UI surface without changing other
platforms.

The OPL1000 makefile does not track header dependencies, so config-header
changes must be measured from a clean build.

Clean WSL build result with `arm-none-eabi-gcc` 13.2.1:

```text
v58 size: text 171815, data 2308, bss 3844, dec 177967
v59 size: text 170691, data 2312, bss 3836, dec 176839

v58 IRAM1: 163444 / 170848
v59 IRAM1: 162316 / 170848
v59 SHM:    14524 / 15360
```

Net v59 change versus v58 is 1128 bytes smaller overall and 1128 bytes less
IRAM1 use. SHM is unchanged.

A temporary clean `codex_v59_pingprobe` build left `ENABLE_HTTP_PING` enabled
to measure the saving from the HTTP ping prune:

```text
ping probe size: text 172339, data 2312, bss 3836, dec 178487
final v59 size:  text 170691, data 2312, bss 3836, dec 176839
```

So removing the unused ping HTTP UI saves 1648 bytes of text/IRAM in this build.

Expected marker:

```text
[OpenOPL1000] split-M3 v59-pin-labels-ping-prune: shm_fn=0x80001255 result=0xb881be0b
```

## v60 Wi-Fi config ownership

v60 makes the OPL1000 Wi-Fi path behave more like a normal OpenBeken STA build
while still acknowledging the current SDK limitation: no SoftAP fallback. The
compiled `OPL1000_AP` / `1234abcd` test network remains only as the first-boot
default seeded by the OPL1000 block in `CFG_SetDefaultConfig()`. After config
exists, `g_cfg` is the source of truth.

The OPL1000 Wi-Fi HAL no longer silently falls back to the compiled SSID or
password when it is called with empty values. An empty SSID now skips STA
connection and reports disconnected. A non-empty SSID with an empty password is
left as an open-network connection attempt.

The SoftAP/open-access path is now explicitly blocked for OPL1000:

```text
HAL_SetupWiFiOpenAccessPoint() reports unsupported and returns -1.
/cfg_wifi hides the "Convert to Open Access WiFi" form on OPL1000.
/cfg_wifi_set?open=1 refuses the request without clearing saved credentials.
```

This avoids the previous bad behaviour where requesting SoftAP could reconnect
to the compiled test AP and make the web UI look like credentials had changed
when they had not.

v60 also adds an OPL1000-only `/cfg_wifi` scan implementation using the SDK
`wifi_scan_start()` / `wifi_get_scan_result()` APIs. It prints SSID, channel,
RSSI, and capability bits. The scan is deliberately simple and blocks that HTTP
request for about four seconds while the SDK scan completes; the OPL1000 HTTP
server is still the simplified single worker path, so this should be treated as
a bring-up aid rather than a final polished UX.

Vendor AT remains disabled. UART provisioning through the OpenBeken command
line is the next likely step for first-time setup without requiring users to
stand up the default `OPL1000_AP`.

Clean WSL build result with `arm-none-eabi-gcc` 13.2.1:

```text
v59 size: text 170691, data 2312, bss 3836, dec 176839
v60 size: text 171287, data 2308, bss 3836, dec 177431

v59 IRAM1: 162316 / 170848
v60 IRAM1: 162916 / 170848
v60 SHM:    14516 / 15360
```

Net v60 change versus v59 is 592 bytes larger overall and 600 bytes more IRAM1
use. SHM is 8 bytes smaller because the SHM text layout moved slightly.

Expected marker:

```text
[OpenOPL1000] split-M3 v60-wifi-config: shm_fn=0x80001245 result=0xb881be0b
```

## v61 watchdog feed during warm reboot Wi-Fi reconnect

Hardware testing of v60 showed that saving new Wi-Fi credentials from the GUI
persisted correctly, but the immediate software reboot could reset again while
connecting to the newly saved AP. A full power cycle then booted and connected
normally with the new credentials, so the config/FIM save path was not the
problem.

The failing warm-boot log reset almost exactly at the SDK's 10 second watchdog
window, during the first scan/connect attempt and before the worker printed the
`wifi_connection_connect rc=...` line. The Opulinks SDK initializes the hardware
WDT with a 10 second timeout and its normal idle hook clears it, while the
OpenBeken OPL1000 HAL still had an empty `HAL_Run_WDT()`.

v61 wires OPL1000 `HAL_Run_WDT()` to `Hal_Wdt_Clear()` so OpenBeken's existing
once-per-second watchdog call feeds the SDK WDT. The Wi-Fi worker also clears
the WDT immediately around the long SDK scan, connect, and lwIP start calls.

This is intentionally OPL1000-only and does not change the generic reboot path
or Wi-Fi config behaviour. The test is to change Wi-Fi from `/cfg_wifi`, allow
the automatic reboot, and check whether the device reaches DHCP on the newly
saved AP without a second reset.

Clean WSL build result with `arm-none-eabi-gcc` 13.2.1:

```text
v60 size: text 171287, data 2308, bss 3836, dec 177431
v61 size: text 171331, data 2312, bss 3836, dec 177479

v60 IRAM1: 162916 / 170848
v61 IRAM1: 162964 / 170848
v61 SHM:    14516 / 15360
```

Net v61 change versus v60 is 48 bytes larger overall and 48 bytes more IRAM1
use. SHM is unchanged.

Expected marker:

```text
[OpenOPL1000] split-M3 v61-wdt-feed: shm_fn=0x80001245 result=0xb881be0b
```

## v62 default UART command line

OPL1000 has no SoftAP provisioning path in the vendor SDK, and the BLE/AT
provisioning stack remains disabled to keep the OpenBeken build small. v62
therefore enables the existing OpenBeken UART command line by default on
OPL1000 only.

Flag 31 (`OBK_FLAG_CMD_ACCEPT_UART_COMMANDS`) is now set in the OPL1000 default
config. The shared command-console runner is still limited to platforms with a
known backend, but the gate now includes OPL1000 as well as Beken. To keep heap
cost down, OPL1000 uses a 256 byte UART RX ring buffer instead of the 512 byte
Beken default.

The OPL1000 UART HAL now hooks the SDK debug-UART RX callback and appends bytes
to the OpenBeken UART command ring buffer. This uses the same IO0/IO1 debug
UART already used for boot and OpenBeken logs, so command input shares the log
serial port at 115200 baud. The vendor AT command system is not re-enabled.

The intended provisioning test after flashing is to send newline-terminated OBK
commands over the debug UART:

```text
SSID1 "YourWifiName"
Password1 "YourWifiPassword"
restart
```

Clean WSL build result with `arm-none-eabi-gcc` 13.2.1:

```text
v61 size: text 171331, data 2312, bss 3836, dec 177479
v62 size: text 171819, data 2336, bss 3836, dec 177991

v61 IRAM1: 162964 / 170848
v62 IRAM1: 163508 / 170848
v62 SHM:    14484 / 15360
```

Net v62 change versus v61 is 512 bytes larger overall and 544 bytes more IRAM1
use. SHM use decreased by 32 bytes. Runtime heap cost is the OPL1000 UART RX
ring buffer, currently 256 bytes, allocated only when the UART command flag is
enabled.

Expected marker:

```text
[OpenOPL1000] split-M3 v62-uart-cli: shm_fn=0x80001245 result=0xb881be0b
```

## v63 UART command line via polling

Hardware testing of v62 showed that replacing the SDK debug-UART RX callback
was too invasive. The device repeatedly reset during Wi-Fi association after
the targeted scan had found the AP and the worker had called
`wifi_connection_connect()`. The serial log also showed interleaved/truncated
output around the Wi-Fi bring-up window.

v63 keeps the OPL1000-only Flag 31 default and the 256 byte UART command RX
buffer, but removes the debug-UART callback replacement. Instead, OPL1000
disables debug-UART RX interrupts for the OBK console and polls
`Hal_DbgUart_DataRecvTimeOut()` from the existing OpenBeken UART command tick,
draining up to 64 bytes per pass into the normal command ring buffer.

This keeps UART command handling out of interrupt context and avoids taking
over the vendor debug/diag callback path. The provisioning command test remains
the same:

```text
SSID1 "YourWifiName"
Password1 "YourWifiPassword"
restart
```

Clean WSL build result with `arm-none-eabi-gcc` 13.2.1:

```text
v62 size: text 171819, data 2336, bss 3836, dec 177991
v63 size: text 171827, data 2336, bss 3836, dec 177999

v62 IRAM1: 163508 / 170848
v63 IRAM1: 163516 / 170848
v63 SHM:    14484 / 15360
```

Net v63 change versus v62 is 8 bytes larger overall and 8 bytes more IRAM1
use. SHM use is unchanged.

Expected marker:

```text
[OpenOPL1000] split-M3 v63-uart-poll: shm_fn=0x80001245 result=0xb881be0b
```

## v64 defer UART command line until Wi-Fi is online

Hardware testing of v63 showed the same reset pattern as v62: the device reset
during the first Wi-Fi association, after the targeted scan and
`direct no-BSSID connect set_config`, but before
`wifi_connection_connect rc=...` was printed.

v64 keeps the OPL1000-only UART command line code but stops initializing or
polling it during Wi-Fi association. On OPL1000, `CMD_InitCommands()` now skips
the immediate UART console init even when Flag 31 is set. The quick-tick UART
runner returns early until `Main_HasWiFiConnected()` is true; only after the
device is online does it lazily initialize the 256 byte command RX ring buffer
and start polling the debug UART.

This is deliberately a conservative probe. It does not solve first-boot
provisioning without the default AP, but it should preserve the known-good Wi-Fi
join path while still allowing UART credential changes after the device has
joined its current configured AP.

Clean WSL build result with `arm-none-eabi-gcc` 13.2.1:

```text
v63 size: text 171827, data 2336, bss 3836, dec 177999
v64 size: text 171823, data 2332, bss 3836, dec 177991

v63 IRAM1: 163516 / 170848
v64 IRAM1: 163508 / 170848
v64 SHM:    14484 / 15360
```

Net v64 change versus v63 is 8 bytes smaller overall and 8 bytes less IRAM1
use. SHM use is unchanged.

Expected marker:

```text
[OpenOPL1000] split-M3 v64-uart-after-wifi: shm_fn=0x80001245 result=0xb881be0b
```

## v65 rollback UART command-line experiment

Hardware testing of v64 still reset at the same Wi-Fi association point as v62
and v63, even though v64 deferred UART console init and polling until after
`Main_HasWiFiConnected()` became true. That means the OPL1000 image could not
tolerate this UART command-line experiment even when the runtime UART path was
dormant during association.

v65 rolls the UART command-line experiment out of the active OPL1000 build:

- Flag 31 is no longer enabled by default on OPL1000.
- `cmd_main.c` goes back to a Beken-only UART command console gate.
- `hal_uart_opl1000.c` goes back to the small TX-only stub.

The v62-v64 history is kept above because it is useful evidence: the obvious
OBK UART-console path is not safe enough for this tight OPL1000 image right now.
Future provisioning should use either a much smaller OPL1000-specific parser or
a lower-level SDK/FIM write path that does not pull the generic OBK UART command
console into the Wi-Fi association image.

Clean WSL build result with `arm-none-eabi-gcc` 13.2.1:

```text
v64 size: text 171823, data 2332, bss 3836, dec 177991
v65 size: text 171335, data 2308, bss 3836, dec 177479

v64 IRAM1: 163508 / 170848
v65 IRAM1: 162964 / 170848
v65 SHM:    14516 / 15360
```

Net v65 change versus v64 is 512 bytes smaller overall and 544 bytes less IRAM1
use. SHM use returns to the v61 baseline.

Expected marker:

```text
[OpenOPL1000] split-M3 v65-uart-rollback: shm_fn=0x80001245 result=0xb881be0b
```
