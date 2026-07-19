# Remediation diagnostics: `1c091e22cd501e1a2dfa46b6c87803b8b04e288b`

- Target set: **shared**
- Targets found: **8**
- Targets with build output: **7**
- Warning lines: **4634**
- Error lines: **2008**

## Known-warning fingerprints

| Fingerprint | Count |
| --- | ---: |
| `cjson_unicode_escape` | 1 |
| `nonvoid_return` | 8 |
| `implicit_allocator_declaration` | 5 |
| `maybe_uninitialized` | 0 |
| `shift_count_width` | 0 |
| `linker_symbol_size_change` | 0 |
| `format_truncation` | 4 |
| `incompatible_pointer` | 89 |

## Targets

| Platform | Variant | Output | Warnings | Errors | Largest firmware output |
| --- | --- | ---: | ---: | ---: | --- |
| OpenBK7231N | default | yes | 100 | 0 | `output/agent_remediate-build-diagnostics_1c091e22cd50/OpenBK7231M_QIO_agent_remediate-build-diagnostics_1c091e22cd50.bin` (1,220,464 bytes) |
| OpenECR6600 | default | yes | 121 | 0 | `output/agent_remediate-build-diagnostics_1c091e22cd50/OpenECR6600_agent_remediate-build-diagnostics_1c091e22cd50.bin` (996,289 bytes) |
| OpenESP32C3 | 2M | yes | 1 | 0 | `output/agent_remediate-build-diagnostics_1c091e22cd50_2M/OpenESP32C3_agent_remediate-build-diagnostics_1c091e22cd50_2M.factory.bin` (1,041,456 bytes) |
| OpenGD32VW553 | default | yes | 200 | 0 | `output/agent_remediate-build-diagnostics_1c091e22cd50/OpenGD32VW553_agent_remediate-build-diagnostics_1c091e22cd50.bin` (869,573 bytes) |
| OpenRTL8720D | default | yes | 516 | 0 | `output/agent_remediate-build-diagnostics_1c091e22cd50/OpenRTL8720D_agent_remediate-build-diagnostics_1c091e22cd50.bin` (1,138,688 bytes) |
| OpenTXW81X | default | yes | 84 | 0 | `output/agent_remediate-build-diagnostics_1c091e22cd50/OpenTXW81X_agent_remediate-build-diagnostics_1c091e22cd50.bin` (607,248 bytes) |
| OpenW800 | default | yes | 180 | 0 | `output/agent_remediate-build-diagnostics_1c091e22cd50/OpenW800_agent_remediate-build-diagnostics_1c091e22cd50.fls` (708,956 bytes) |
| Simulator | sanitizers | no | 3432 | 2008 | none |
