# Wemo compatibility and regression tests

## OTA compatibility contract

This remains the original single-switch, Tasmota-style OpenBeken Wemo emulator.
No configuration structure, stored setting, flash partition, driver startup command,
or SDK revision is changed by the Wemo revision.

Keep the original MAC-derived `Socket-1_0-201612...` UDN, the configured device name,
port 80, and the existing setup/control/service URLs. Set requests still execute
`POWER ON` or `POWER OFF` through OpenBeken's command dispatcher. Reads retain LED
engine priority and otherwise select the first eligible physical or Toggle channel
in numerical order, not all physical relays before virtual channels.

The original upstream driver exposed its current IP in `<serialNumber>`. Although
that is not a good permanent identifier, changing it on OTA would change the Home
Assistant Wemo entity unique ID. Preserve this legacy field; only the previously
incorrect presentation URL is corrected. DHCP-related changes to this legacy serial
remain an inherited limitation. The stable UDN is unchanged.

Normal SOAP get/set requests, headerless OpenBeken requests, default/inherited XML
namespaces, and the legacy redundant BinaryState argument in Get remain supported.
Malformed state arguments never change an output. Unsupported actions receive a
SOAP 401 fault; invalid arguments or incomplete bodies receive a SOAP 402 fault,
both using HTTP 500. The bounded SOAP reader is not a general XML implementation:
DTDs, entity expansion and CDATA are unsupported.

SSDP keeps the Belkin wildcard, rootdevice, ssdp:all and ssdpsearch:all compatibility
paths. Explicit controllee searches get one matching controllee response rather
than a second unrelated wildcard response. HUE and generic SSDP dispatch remain
separate. Start SSDP as before; Wemo does not recursively start or stop shared drivers.

GetMetaInfo returns the six pipe-separated fields understood by pywemo, identifying
OpenBeken rather than inventing Belkin firmware. GetExtMetaInfo is not advertised.
The four original HTTP endpoints take priority when registering callbacks; optional
metadata cannot prevent the old endpoints from starting if only four slots remain.

## Running tests

```
python3 tests/wemo/run_tests.py --sanitize
CC=clang python3 tests/wemo/run_tests.py --sanitize
```

These host tests compile the unchanged production driver against minimal API stubs,
with LED support both disabled and enabled. They cover target ordering, legacy
identity/name, get/set, invalid arguments, SOAP action consistency, namespace forms,
every truncated prefix of a request, metadata, stop/restart, callback registration
failure, allocation failure, SSDP reply selection and deterministic parser mutations.
The role and POWER behaviour in this harness are models, not the production GPIO
implementation. `src/selftest/selftest_wemo.c` separately exercises the real simulator
HTTP, command and channel paths in the existing simulator build and test workflow.

Neither suite is a physical Alexa/Belkin or over-the-air upgrade test. Subscription
and event delivery remain unsupported by the shared HTTP server. The HTTP transport
is not rewritten: an incomplete body is rejected, not assembled by this driver.
A real-client smoke test and an actual OTA on representative hardware remain needed
before asserting universal interoperability or absence of every possible regression.

## Reference implementations checked

- Tasmota Wemo emulation: https://github.com/arendst/Tasmota/blob/development/tasmota/tasmota_xdrv_driver/xdrv_21_wemo.ino
- pywemo metadata format: https://github.com/pywemo/pywemo/blob/main/pywemo/util.py
- Home Assistant Wemo entity identity: https://github.com/home-assistant/core/blob/dev/homeassistant/components/wemo/entity.py

These are behavioural references, not copied replacements for OpenBeken's transport
or channel APIs. The OTA baseline is upstream commit `e75bb9900b6225d1e971d13696914cc055c96c1d`.
