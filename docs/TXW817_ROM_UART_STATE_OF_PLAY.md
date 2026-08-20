# TXW817 / TXW81X ROM UART + EasyFlasher — State of Play

**Snapshot time:** 2026-08-20 07:15 BST  
**Purpose:** authoritative handoff for continuing this work in a fresh ChatGPT/Codex session without losing goals, evidence, constraints, or the exact current stopping point.

> **New-session instruction:** Read this file in full before doing anything. Treat the sections marked **LIVE-PROVEN** as authoritative. Do not ask the user to repeat the UART wiring, ROM token, latest command results, repositories, or overall objective. Do not revert to older PA8/PA9 assumptions found in earlier handoff files.

---

## 1. Executive status

The hard part of entry is solved.

We now have a custom minimal TXW817 application that:

1. boots reliably using the proven OpenTXW81X startup/link/runtime path;
2. uses the **same UART direction as the mask ROM**;
3. accepts our diagnostic commands over UART;
4. calls the genuine SDK `system_goto_boot()`;
5. watchdog-resets into the mask ROM;
6. sends exact raw `AT+BOOTL`;
7. receives the ROM acknowledgement `18 18 18`;
8. opens a framed ROM session using the exact key `@huge-ic`;
9. performs live framed read-only ROM commands.

The latest live run has additionally proven:

- ROM command `0x01` capability/info transfer.
- ROM command `0x05` range CRC operation.
- ROM command `0xFF` session close.
- The live `0x05` result for `(address=0x18000000, length=0x100)` is **`0xFEA8A821`**, which does **not** match the standard zlib CRC32 of the corresponding packaged application code bytes (`0x2720303C`). This discrepancy is unresolved and must be analysed rather than “corrected”.

The next primary work is **offline ROM decompilation**, especially commands `0x02` and `0x04`, plus resolving the `0x05` CRC semantics. Do not jump to destructive live commands.

---

## 2. Strategic end goal

The project goal is not merely to prove ROM entry. It is to add robust native TXW81X support to **BK7231 Easy Flasher / BK7231GUIFlashTool**.

Target end state:

1. Fully document the TXW817/TXW81X resident ROM UART protocol from the physical ROM dump and live hardware.
2. Build a deterministic native UART transport.
3. Support device identification and safe read/verify operations first.
4. Fully understand flash geometry and all status/error semantics.
5. Only then add erase/program/write/verify.
6. Integrate TXW81X cleanly into Easy Flasher without regressing existing platforms.
7. Avoid dependence on CK-Link or Taixin/C-SKY proprietary host software for normal flashing if the ROM supports all required operations.
8. Keep the CK-Link/C-SKY FlashProgrammer path as a recovery/reference route.

Older Easy Flasher exploration had a `TXW81X = 42` platform value in experimental work. Do **not** assume current `main` contains it; verify the target Easy Flasher branch before implementation.

Easy Flasher repositories:

- Upstream: `openshwprojects/BK7231GUIFlashTool`
- User fork: `divadiow/BK7231GUIFlashTool`

---

## 3. Hard safety / engineering constraints

During protocol discovery:

- **No erase.**
- **No flash programming.**
- **No arbitrary target-memory write.**
- **No RAM upload/execution.**
- No destructive command should be tried merely because its opcode is known.
- Resolve handlers statically first.
- When a live response is unexpected, stop rather than automatically continuing.
- Preserve every request/response byte and timing in `.tx.bin`, `.rx.bin`, and text logs.
- Use 3.3 V TTL only.
- Common ground required.
- Do not connect the USB-UART VCC pin when the target is powered normally.
- Disconnect CK-Link/debug drivers from potentially shared target pins before UART testing.
- Keep GitHub CI controlled: source edits should not cause queues of duplicate builds.

For deliverables:

- Windows-compatible scripts.
- CRLF line endings preferred.
- Versioned filenames.
- Minimal user test packages rather than build-report clutter.
- Never label an unverified linked/raw image as a flashable Taixin APP image.

---

## 4. Current physical target and UART

Target family:

- Taixin TXW817 / TXW81X
- Observed target ID: `0x8410`
- C-SKY CK803-class CPU family.

Latest working USB serial port:

- **COM45**
- Older handoff files mention COM47; that is historical. Current testing is COM45.

UART:

- 115200 baud
- 8 data bits
- no parity
- 1 stop bit
- DTR disabled
- RTS disabled

### Authoritative current wiring — LIVE-PROVEN

The diagnostic v0.4 application and TXW817 mask ROM both use:

```text
TXW817 PA9  TX  -> USB-TTL RX
TXW817 PA8  RX  <- USB-TTL TX
TXW817 GND      <-> USB-TTL GND
```

**This wiring is live-proven through both application UART and mask-ROM UART without a wire swap.**

### Important superseded information

Earlier handoff material says:

```text
PA8 = TX
PA9 = RX
```

That was the direction used by diagnostic v0.3 and is **superseded**.

ROM disassembly and live testing proved the resident ROM is:

```text
PA8 = ROM RX
PA9 = ROM TX
```

Diagnostic v0.4 was deliberately changed to match the ROM direction.

---

## 5. Canonical ROM evidence

Primary physical ROM dump available in the project/runtime:

- `txw81x-rom-2(1).bin`

Earlier physical reading established:

- a meaningful canonical first 64 KiB;
- the remainder of the larger physical read contains repeated/filler/open-bus-like regions;
- canonical 64 KiB SHA-256:

```text
9c03476fe29b6991971d9437188658416032c3b95f9873ac657c05eb6c762d56
```

The ROM contains:

- `AT+BOOTL`
- `AT+FWUPG`
- framed binary command dispatcher
- supported-command table
- response magic construction
- GPIO/UART initialization
- flash/memory operation handlers.

Treat the physical ROM as the primary truth for command semantics.

---

## 6. ROM entry — LIVE-PROVEN

Exact entry token:

```text
ASCII: AT+BOOTL
HEX:   41 54 2B 42 4F 4F 54 4C
```

Rules:

- exactly 8 bytes;
- **no CR**
- **no LF**
- after valid entry, ROM returns:

```text
18 18 18
```

v0.4 live trace proves the complete chain:

```text
custom diagnostic application
    ->
AT+DIAG=EXEC,<nonce>
    ->
genuine SDK system_goto_boot()
    ->
watchdog reset
    ->
TXW817 mask ROM
    ->
exact AT+BOOTL
    ->
18 18 18
```

No signal-wire swap is now required.

---

## 7. `system_goto_boot()` finding

Recovered from the genuine TXW81X SDK `libsysctrl` implementation.

High-level behaviour:

1. disable interrupts;
2. reset USB SIE;
3. reset USB PHY;
4. reset/configure PMU state;
5. set PMU boot-control flags;
6. configure short watchdog reset;
7. spin until watchdog reset.

Relevant PMUCON7 concepts found in SDK/ROM work:

- `deadcode_pending`
- `skip_master_boot_sign`
- `skip_test_mode_sign`
- `skip_psram_sign`

The crucial practical result is live-proven: calling the genuine function from our running application leaves the post-reset chip in the resident-ROM communication path rather than immediately booting the application again.

---

## 8. Current diagnostic firmware: v0.4

Repository:

- `divadiow/OpenBK7231T_App`

Branch:

- `txw81x-minimal-diag`

Primary source:

- `platforms/TXW81X/txw81x_diag_main.c`

Current source blob SHA:

```text
1f9e7db816ef7dfe2ff9fd7d406a204bc60d0854
```

Current firmware banner:

```text
TXWDIAG READY v=0.4 target=TXW817-810 PA9TX/PA8RX 115200
```

Diagnostic commands:

```text
AT+DIAG=PING
AT+DIAG=SNAPSHOT
AT+DIAG=ARM,<8-hex-digit nonce>
AT+DIAG=EXEC,<same nonce>
```

Typical live snapshot:

```text
mode=0x00000000
cp=0
ft=0
chip=0x00008410
pack=0x0000000A
bios=0x00000003
module=0x00000000
```

`EXEC` behaviour:

```text
validate armed nonce
snapshot
print TXWDIAG ROMGO
sleep ~100 ms
system_goto_boot()
```

### Important timing caveat

A v0.3 test with a human pause before EXEC failed to produce ROMGO after roughly three seconds of runtime, while immediate EXEC succeeded.

This suggests the stripped minimal application is not servicing some normal OpenTXW81X background/watchdog housekeeping.

**The exact cause has not been conclusively proven.**

Therefore current automated tests intentionally execute PING/ARM/EXEC quickly and do not wait for human confirmation between ARM and EXEC.

---

## 9. Why v0.4 uses OpenTXW81X rather than our earlier standalone image

The initial standalone diagnostic approach repeatedly produced images that did not boot.

Important failed/superseded stages:

### Standalone v0.1/v0.2

Problems included:

- incorrect/insufficient startup assumptions;
- packaging uncertainty;
- missing/incorrect Taixin `SYS_PARAM` placement in an early build;
- firmware accepted as “linked” without proving it matched the genuine TXW81X startup configuration.

Do not resurrect this approach.

### OpenTXW81X-based v0.3

This was the breakthrough:

- used the already-working OpenTXW81X startup/link/runtime path;
- gutted the OpenBeken application layer;
- inserted only the minimal diagnostic `main()`;
- successfully booted.

v0.3 initially used:

```text
PA8 = app TX
PA9 = app RX
```

while ROM used the reverse. A manual wire-swap test proved ROM entry.

### v0.4

Changed the diagnostic application itself to:

```text
PA9 = app TX
PA8 = app RX
```

matching the mask ROM.

This is the current baseline.

---

## 10. v0.4 build / CI state

Repository/branch:

```text
divadiow/OpenBK7231T_App
txw81x-minimal-diag
```

Workflow:

```text
.github/workflows/txw81x-minimal-diag.yml
```

Current CI discipline:

- workflow is triggered only by:
  - explicit `workflow_dispatch`, or
  - update of `.github/txw81x-diag-trigger.txt`;
- normal source edits do **not** trigger builds;
- concurrency uses one diagnostic build group;
- this prevents the earlier multi-build queue problem.

v0.4 successful workflow run:

```text
Run ID: 32336055483
Artifact: TXW817_810_DIAG_MINIMAL_v04
Artifact ID: 9394681457
```

Publish commit:

```text
b5cec053e02e08233f809e06f9db2542e94c534a
Publish TXW817 minimal diagnostic v0.4 build result
```

Downloaded test artifact:

```text
TXW817_810_DIAG_MINIMAL_v04.zip
```

Firmware member:

```text
TXW817_810_DIAG_MINIMAL_v04.bin
size: 115598 bytes
SHA-256:
8a4410e4140886859fa55ea5aab01531e731c6d63623decac65a2458a105ffe4
```

The image is built through the proven OpenTXW81X compile/link path and packaged with the native Python TXW81X packer recovered/validated in earlier work.

---

## 11. Taixin APP packaging findings

Relevant earlier repository:

- `divadiow/txw81xcam`
- branch `txw817-diag-build`

Useful vendor packaging files recovered there:

```text
project/BinScript.BinScript
project/BinScript.exe
project/makecode.exe
project/makecode.ini
project/parameter.bincfg
project/loader_compress.bin
project/CSKYFlashProgramerCfg
```

Key packaging configuration facts:

```text
CodeLoadToSramAddr = 0x18000000
CodeExeAddr        = 0x18000000
CodeAddrOffset     = 0xC00
ParamOffset        = 0x180
```

The vendor image includes metadata/header before code.

Do not rename a raw linked image to `APP.bin`.

The current native Python packer was previously checked against the Taixin tooling and is used in CI because it avoids the Windows/Wine packaging problem.

---

## 12. Framed ROM protocol — LIVE-PROVEN layout

After `AT+BOOTL` -> `18 18 18`, ROM accepts fixed binary requests.

### Request: 20 bytes

Live requests have the following form:

```text
offset  size  meaning
0x00    2     magic = 1A 2B
0x02    2     sequence, little-endian
0x04    4     total request length = 0x00000014
0x08    1     command
0x09    1     body/count field; current fixed requests use 0x0C
0x0A    1     checksum mode/selector; current requests use 0x00
0x0B    8     command payload
0x13    1     additive checksum
```

For the live-proven checksum mode:

```text
request checksum = sum(request[8..18]) & 0xFF
```

### Response: 16 bytes

```text
offset  size  meaning
0x00    2     magic = 1A 2B
0x02    2     echoed sequence
0x04    4     total response length = 0x00000010
0x08    1     echoed command
0x09    1     status
0x0A    5     command-specific response bytes
0x0F    1     additive checksum
```

Live-proven response checksum:

```text
response checksum = sum(response[8..14]) & 0xFF
```

This framing is now established by multiple valid live transactions.

---

## 13. Supported ROM command list — LIVE-PROVEN by command 0x01

The ROM's own live capability block returned:

```text
0F 00 01 02 04 05 06 07 08 09 0A FF 10 84 00 00
```

Interpretation:

```text
0x0F                block/length marker
supported commands: 00 01 02 04 05 06 07 08 09 0A FF
chip ID:            0x00008410
```

This supersedes older handoff files that mentioned `0x0E` as a possible command.

**0x0E is not in the live capability list.**

---

## 14. Command-status table at the current stopping point

### 0x00 — OPEN SESSION — LIVE-PROVEN

Request payload must contain the exact 8-byte key:

```text
ASCII: @huge-ic
HEX:   40 68 75 67 65 2D 69 63
```

Live request:

```text
1A 2B 01 00 14 00 00 00 00 0C 00 40 68 75 67 65 2D 69 63 EE
```

Live response:

```text
1A 2B 01 00 10 00 00 00 00 00 01 00 00 00 00 01
```

Status `0x00`.

---

### 0x01 — GET INFO / TARGET-TO-HOST TRANSFER — LIVE-PROVEN

Live request:

```text
1A 2B 02 00 14 00 00 00 01 0C 00 00 00 00 00 10 00 00 00 1D
```

Initial status response:

```text
1A 2B 02 00 10 00 00 00 01 00 01 00 00 00 00 02
```

Then exactly 16 raw data bytes:

```text
0F 00 01 02 04 05 06 07 08 09 0A FF 10 84 00 00
```

Then transfer-complete response:

```text
1A 2B 02 00 10 00 00 00 01 00 01 00 00 00 00 02
```

This proves command `0x01` has an extra raw target-to-host data phase between framed status responses.

---

### 0x02 — BULK HOST-TO-TARGET TRANSFER — STATIC ONLY / NOT LIVE TESTED

Current static conclusion:

- appears to implement a host-to-target bulk memory/data transfer;
- therefore write-capable/potentially destructive;
- likely highly relevant to eventual programming;
- **must be fully decompiled offline before any live use**.

No live test yet.

---

### 0x04 — FLASH/CONTROL OPERATION — STATIC PARTIAL / NOT LIVE TESTED

This is one of the most important unresolved handlers.

Current direction:

- appears flash/control related;
- potentially erase/program/destructive;
- do not probe live yet.

**Next major task: full offline decompilation of 0x04 and all callees.**

---

### 0x05 — CRC32 / RANGE CRC — LIVE-PROVEN, ALGORITHM DETAILS UNRESOLVED

Live request:

```text
command = 0x05
address = 0x18000000
length  = 0x00000100
```

Exact frame:

```text
1A 2B 03 00 14 00 00 00 05 0C 00 00 00 00 18 00 01 00 00 2A
```

Live response:

```text
1A 2B 03 00 10 00 00 00 05 00 21 A8 A8 FE 00 74
```

Decoded live result:

```text
CRC result = 0xFEA8A821
```

Important discrepancy:

- first 0x100 bytes of the v0.4 raw code image at APP package offset `0xC00` have ordinary zlib CRC32:

```text
0x2720303C
```

- ROM command `0x05` returned:

```text
0xFEA8A821
```

Do **not** assume either result is “wrong”.

Possible classes of explanation to investigate:

1. ROM CRC hardware is configured with different reflection/init/xor/endian semantics.
2. The effective bytes visible at `0x18000000` differ from the packaged raw code bytes.
3. The command's length unit/rounding is not simply bytes.
4. Address translation/XIP/cache/loader behaviour changes what is checksummed.
5. The response is a transformed/raw hardware CRC state rather than standard finalized CRC32.

This is an active unresolved item.

---

### 0x06 — AUTHENTICATED RESET/CONTROL — STATIC ONLY

Known to involve control/reset and authentication-style handling.

Do not live-test yet unless required and fully understood.

---

### 0x07 — DIRECT 32-BIT MMIO WRITE — STATIC-PROVEN / NEVER LIVE-TEST DURING DISCOVERY

Corresponding write operation.

Potentially dangerous.

Do not probe.

---

### 0x08 — DIRECT 32-BIT MMIO READ — LIVE-PROVEN

Static validation constrains its address space to the high peripheral/MMIO region.

Live-safe target:

```text
0x4002005C
```

Known chip-ID register.

Live request pass 1:

```text
1A 2B 02 00 14 00 00 00 08 0C 00 5C 00 02 40 00 00 00 00 B2
```

Live response:

```text
1A 2B 02 00 10 00 00 00 08 00 10 84 00 00 00 9C
```

Decoded:

```text
READ32 0x4002005C -> 0x00008410
```

Repeated a second time and returned the same value.

Therefore command `0x08`, frame construction, response framing, sequence echo and checksum are all hardware-proven.

It cannot be used as a generic low-address mask-ROM reader.

---

### 0x09 — UART / communications configuration — STATIC ONLY

Not live-tested.

Potentially changes communication state; leave alone until fully decoded.

---

### 0x0A — hardware/interface callback — STATIC PARTIAL

Not live-tested.

Need further decompilation if relevant.

---

### 0xFF — CLOSE SESSION — LIVE-PROVEN

Uses the same `@huge-ic` key.

Live request:

```text
1A 2B 04 00 14 00 00 00 FF 0C 00 40 68 75 67 65 2D 69 63 ED
```

Live response:

```text
1A 2B 04 00 10 00 00 00 FF 00 21 A8 A8 FE 00 6E
```

Status `0x00`.

Note that response bytes 10-13 retained the preceding CRC value (`0xFEA8A821`), which may reflect a persistent response/work buffer rather than semantic output of CLOSE. Do not assign meaning to those bytes yet.

---

## 15. Latest live evidence files

### v0.4 no-swap ROM-entry proof

Files:

```text
TXW817_DIAG_V04_20260820_070053.log
TXW817_DIAG_V04_20260820_070053.rx.bin
TXW817_DIAG_V04_20260820_070053.tx.bin
```

Key result:

```text
TXWDIAG READY v=0.4...
...
TXWDIAG ROMGO ...
...
AT+BOOTL
18 18 18
PASS: mask ROM replied 18 18 18 with no signal-wire swap.
```

### Framed session + 0x08 READ32 proof

Files:

```text
TXW817_ROM_READ32_SAFE_20260820_070220.log
TXW817_ROM_READ32_SAFE_20260820_070220.rx.bin
TXW817_ROM_READ32_SAFE_20260820_070220.tx.bin
```

Key results:

```text
0x00 session status = 0x00

READ32 0x4002005C -> 0x00008410
READ32 0x4002005C -> 0x00008410
```

### 0x01 INFO + 0x05 CRC + 0xFF CLOSE proof — CURRENT LATEST TEST

Files:

```text
TXW817_ROM_INFO_CRC_SAFE_20260820_071400.log
TXW817_ROM_INFO_CRC_SAFE_20260820_071400.rx.bin
TXW817_ROM_INFO_CRC_SAFE_20260820_071400.tx.bin
```

Key results:

```text
0x01 info:
0F 00 01 02 04 05 06 07 08 09 0A FF 10 84 00 00

0x05:
CRC32-like result for 0x18000000 + 0x100 = 0xFEA8A821

0xFF:
session closed with status 0x00
```

This is the exact current stopping point.

---

## 16. Immediate next work — DO THIS NEXT

### Priority 1 — resolve command 0x05 offline

Use the canonical ROM disassembly to trace the entire `0x05` handler:

- validation;
- address and length interpretation;
- CRC peripheral setup;
- polynomial/config registers;
- initialization;
- input width;
- endian/reflection handling;
- final XOR/transformation;
- result placement.

Then compare against the live value:

```text
0xFEA8A821
```

Do not discard the live value just because standard zlib CRC gives another result.

Useful controlled follow-up later, only after static understanding:

- `0x05` on very short known ranges;
- multiple lengths at same address;
- addresses whose live content is independently known.

---

### Priority 2 — fully decompile command 0x02 offline

Need exact answers:

- request payload fields;
- destination address ranges;
- transfer length;
- extra raw-data phase framing;
- chunking;
- acknowledgements;
- status codes;
- whether data goes to RAM, flash, or generic bus address;
- whether writes are direct or staged;
- bounds checks;
- relation to command `0x04`.

No live test until all of this is known.

---

### Priority 3 — fully decompile command 0x04 offline

This is likely central to actual flash operations.

Determine:

- subcommands/mode field;
- erase semantics;
- program semantics;
- flash addressing;
- sector/block sizes;
- alignment;
- range checks;
- protection/unlock;
- status values;
- interaction with 0x02;
- whether verify/CRC is expected;
- whether blank-check/readback exists.

Again: **no live destructive probe yet**.

---

### Priority 4 — map all ROM statuses

Build a table of:

```text
status byte -> exact condition / handler path
```

Known/provisional examples from older work included invalid-command/checksum statuses, but now that valid framing is proven these should be re-derived cleanly from the ROM and then live-checked only where safe.

---

### Priority 5 — determine read path for flash / program verification

`0x08` is MMIO read32, not a generic memory read.

Need identify whether:

- command `0x01` can transfer arbitrary memory;
- `0x04` has a flash-read mode;
- another handler exposes XIP/flash data;
- CRC-only verify is the intended design.

A practical Easy Flasher implementation needs either:

- actual flash readback, or
- sufficiently trustworthy identification + CRC verification.

---

### Priority 6 — only after the above, implement TXW81X Easy Flasher transport

Target design:

```text
enter ROM
open session
identify/capabilities
read/backup if supported
erase
transfer/program
verify
close/reset
```

Start with a read-only transport in the GUI.

Do not broadly refactor unrelated Easy Flasher code.

---

## 17. OpenTXW81X resources

Primary working SDK baseline:

- GitHub: `NonPIayerCharacter/OpenTXW81X`

Important source areas used:

```text
project/device.c
sdk/chip/txw81x/*
sdk/driver/uart/hguart_v2.c
sdk/driver/gpio/hggpio_v4.c
sdk/include/chip/txw81x/io_function.h
sdk/include/chip/txw81x/sysctrl.h
sdk/include/hal/uart.h
libs/libsysctrl.a / linked equivalents
project/BuildBIN.sh
project/BinScript.BinScript
project/makecode.ini
project/parameter.bincfg
```

The stock SDK normally opens UART0 during `device_init()` at a high console baud and configured project pins. Our diagnostic `main()` remaps the already-open UART rather than reopening it, specifically to avoid `uart_open()` reapplying stock pin configuration.

---

## 18. Other important uploaded / File Library resources

Useful historical handoffs and sources:

```text
TXW817-Codex-Prompt-20260812.txt
TXW817 Flashing Objective.json
TXW817 数据手册 V1.6_20241115165936.pdf
txw81x_app_gotoboot_probe_v0.1.0.py
ROM_First_Entry_Test_Plan.md
```

Caution:

- `TXW817-Codex-Prompt-20260812.txt` predates the v0.4 live results.
- Its PA8/PA9 UART direction and “not yet proven” protocol items are now partly superseded.
- Use this state-of-play document plus latest live logs as the authoritative update.

---

## 19. TX-Link-U / AU201 side investigation — deliberately not part of current TXW81X path

The AU201 programmer that was reverse-engineered is the older original:

```text
TX-Link-U / non-Lite
```

not the later TXLink-Lite.

Evidence indicates that generation is not the relevant programmer path for TXW81X Wi-Fi parts.

Therefore:

- do not use AU201/TX-Link-U firmware as primary evidence for TXW817 ROM protocol;
- do not let that side investigation distract from the physical TXW817 ROM and OpenTXW81X SDK;
- the AU201 work remains useful as a separate reverse-engineering project only.

---

## 20. Known earlier mistakes / traps not to repeat

1. **Do not claim a ZIP/binary exists unless it has actually been created and verified.**
2. Do not call a raw linked image a flashable APP image.
3. Do not use the failed standalone diagnostic as the build baseline.
4. Do not revert to `PA8 TX / PA9 RX`; current authoritative direction is `PA9 TX / PA8 RX`.
5. Do not spam `AT+BOOTL` after ROM acknowledges.
6. Do not reuse old malformed-frame conclusions now that valid framing is hardware-proven.
7. Do not treat `0x0E` as supported; live command `0x01` capability output excludes it.
8. Do not send `0x07` merely to “test” it; it is a direct write operation.
9. Do not test `0x02` or `0x04` until fully decompiled.
10. Do not assume standard zlib CRC semantics for command `0x05`.
11. Do not queue multiple GitHub builds. The diagnostic workflow is intentionally trigger-file/manual only.
12. Do not bring TX-Link-U/AU201 back into the TXW81X protocol path without independent TXW81X evidence.

---

## 21. Current confidence levels

### Hardware/live-proven

- v0.4 boots.
- PA9 TX / PA8 RX at 115200 works for application.
- Same wiring works for mask ROM.
- `system_goto_boot()` transition works.
- exact `AT+BOOTL` -> `18 18 18`.
- request magic/size framing.
- response magic/size framing.
- sequence echo.
- request additive checksum in current mode.
- response additive checksum.
- session key `@huge-ic`.
- command `0x00`.
- command `0x01`.
- command `0x05` produces live deterministic range-check result.
- command `0x08` MMIO read32.
- command `0xFF`.
- chip ID `0x8410`.
- live capability list.

### Strong static evidence, not yet live-tested

- `0x02` bulk host-to-target transfer.
- `0x04` flash/control.
- `0x06` reset/control.
- `0x07` MMIO write32.
- `0x09` communication/UART config.
- `0x0A` interface/hardware callback.

### Unresolved

- exact `0x05` CRC algorithm/byte stream.
- full flash geometry as exposed by ROM.
- complete `0x02` transfer semantics.
- complete `0x04` flash operation semantics.
- full status-code map.
- native ROM path for full flash readback/backup.
- whether mask ROM itself can be bulk-read over this protocol.
- exact `AT+FWUPG` role and relationship, if any, to the binary ROM protocol.
- minimal reliable timing bounds between application ROMGO, watchdog reset, ROM UART init and `AT+BOOTL` (500 ms is known-safe, not necessarily minimal).

---

## 22. What to upload into a new chat if context is lost

First upload this file:

```text
TXW817_ROM_UART_STATE_OF_PLAY_20260820_0715.md
```

If the new session cannot access the project File Library, also upload:

```text
txw81x-rom-2(1).bin

TXW817_DIAG_V04_20260820_070053.log
TXW817_DIAG_V04_20260820_070053.rx.bin
TXW817_DIAG_V04_20260820_070053.tx.bin

TXW817_ROM_READ32_SAFE_20260820_070220.log
TXW817_ROM_READ32_SAFE_20260820_070220.rx.bin
TXW817_ROM_READ32_SAFE_20260820_070220.tx.bin

TXW817_ROM_INFO_CRC_SAFE_20260820_071400.log
TXW817_ROM_INFO_CRC_SAFE_20260820_071400.rx.bin
TXW817_ROM_INFO_CRC_SAFE_20260820_071400.tx.bin

TXW817_810_DIAG_MINIMAL_v04.zip
```

The GitHub repositories can then be read directly.

---

## 23. Suggested first message in a replacement chat

Paste this after uploading the state file:

```text
Read TXW817_ROM_UART_STATE_OF_PLAY_20260820_0715.md in full and continue from its "Immediate next work" section. Do not re-derive or retest the live-proven ROM entry, UART direction, frame layout, session key, 0x01, 0x05, 0x08 or 0xFF results unless a new finding directly contradicts them. The current task is offline analysis of the canonical TXW817 ROM, especially the 0x05 CRC discrepancy and full decompilation of commands 0x02 and 0x04, with the eventual goal of safe native TXW81X support in BK7231 Easy Flasher.
```

---

## 24. Exact current continuation point

**Stop point at 2026-08-20 07:15 BST:**

The latest safe hardware test completed successfully.

It proved:

```text
0x00 OPEN SESSION             PASS
0x01 GET INFO                 PASS
0x05 CRC RANGE                PASS as a ROM transaction
                               live result = 0xFEA8A821
                               algorithm/content interpretation unresolved
0xFF CLOSE SESSION            PASS
```

No destructive command was sent.

### The next assistant should now:

1. return to the canonical ROM disassembly;
2. completely decode the `0x05` CRC implementation;
3. completely decode `0x02`;
4. completely decode `0x04`;
5. build a documented command/status/state-machine model;
6. only then propose the next live read-only test.

**Do not spend another round proving that ROM entry works. It works.**
