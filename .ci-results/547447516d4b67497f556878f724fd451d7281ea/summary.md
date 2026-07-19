# Remediation diagnostics: `547447516d4b67497f556878f724fd451d7281ea`

- Target set: **berry**
- Targets found: **7**
- Targets with build output: **6**
- Warning lines: **4233**
- Error lines: **2008**

## Known-warning fingerprints

| Fingerprint | Count |
| --- | ---: |
| `cjson_unicode_escape` | 1 |
| `nonvoid_return` | 8 |
| `implicit_allocator_declaration` | 8 |
| `maybe_uninitialized` | 1 |
| `shift_count_width` | 0 |
| `linker_symbol_size_change` | 0 |
| `format_truncation` | 2 |
| `incompatible_pointer` | 69 |

## Targets

| Platform | Variant | Output | Warnings | Errors | Largest firmware output |
| --- | --- | ---: | ---: | ---: | --- |
| OpenBK7231N | berry | yes | 102 | 0 | `output/agent_remediate-build-diagnostics_547447516d4b_berry/OpenBK7231M_QIO_agent_remediate-build-diagnostics_547447516d4b_berry.bin` (1,220,464 bytes) |
| OpenBK7238 | berry | yes | 177 | 0 | `output/agent_remediate-build-diagnostics_547447516d4b_berry/OpenBK7238_QIO_agent_remediate-build-diagnostics_547447516d4b_berry.bin` (1,253,376 bytes) |
| OpenBL602 | berry | yes | 309 | 0 | `output/agent_remediate-build-diagnostics_547447516d4b_berry/OpenBL602_agent_remediate-build-diagnostics_547447516d4b_berry_OTA.bin` (912,272 bytes) |
| OpenECR6600 | default | yes | 127 | 0 | `output/agent_remediate-build-diagnostics_547447516d4b/OpenECR6600_agent_remediate-build-diagnostics_547447516d4b.bin` (996,289 bytes) |
| OpenESP32C3 | 2M | yes | 1 | 0 | `output/agent_remediate-build-diagnostics_547447516d4b_2M/OpenESP32C3_agent_remediate-build-diagnostics_547447516d4b_2M.factory.bin` (1,041,456 bytes) |
| OpenLN882H | btproxy | yes | 85 | 0 | `output/agent_remediate-build-diagnostics_547447516d4b_btproxy/OpenLN882H_agent_remediate-build-diagnostics_547447516d4b_btproxy.bin` (980,984 bytes) |
| Simulator | sanitizers | no | 3432 | 2008 | none |
