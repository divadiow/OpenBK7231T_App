# Remediation diagnostics: `d92fff8f92a6586d6d98083a1ba6064bf67c9f97`

- Target set: **shared**
- Targets found: **8**
- Targets with build output: **7**
- Warning lines: **4682**
- Error lines: **2008**

## Known-warning fingerprints

| Fingerprint | Count |
| --- | ---: |
| `cjson_unicode_escape` | 3 |
| `nonvoid_return` | 8 |
| `implicit_allocator_declaration` | 25 |
| `maybe_uninitialized` | 0 |
| `shift_count_width` | 0 |
| `linker_symbol_size_change` | 1 |
| `format_truncation` | 14 |
| `incompatible_pointer` | 89 |

## Targets

| Platform | Variant | Output | Warnings | Errors | Largest firmware output |
| --- | --- | ---: | ---: | ---: | --- |
| OpenBK7231N | default | yes | 103 | 0 | `output/agent_remediate-build-diagnostics_d92fff8f92a6/OpenBK7231M_QIO_agent_remediate-build-diagnostics_d92fff8f92a6.bin` (1,220,464 bytes) |
| OpenECR6600 | default | yes | 139 | 0 | `output/agent_remediate-build-diagnostics_d92fff8f92a6/OpenECR6600_agent_remediate-build-diagnostics_d92fff8f92a6.bin` (998,041 bytes) |
| OpenESP32C3 | 2M | yes | 1 | 0 | `output/agent_remediate-build-diagnostics_d92fff8f92a6_2M/OpenESP32C3_agent_remediate-build-diagnostics_d92fff8f92a6_2M.factory.bin` (1,041,248 bytes) |
| OpenGD32VW553 | default | yes | 200 | 0 | `output/agent_remediate-build-diagnostics_d92fff8f92a6/OpenGD32VW553_agent_remediate-build-diagnostics_d92fff8f92a6.bin` (869,317 bytes) |
| OpenRTL8720D | default | yes | 518 | 0 | `output/agent_remediate-build-diagnostics_d92fff8f92a6/OpenRTL8720D_agent_remediate-build-diagnostics_d92fff8f92a6.bin` (1,138,688 bytes) |
| OpenTXW81X | default | yes | 85 | 0 | `output/agent_remediate-build-diagnostics_d92fff8f92a6/OpenTXW81X_agent_remediate-build-diagnostics_d92fff8f92a6.bin` (606,736 bytes) |
| OpenW800 | default | yes | 200 | 0 | `output/agent_remediate-build-diagnostics_d92fff8f92a6/OpenW800_agent_remediate-build-diagnostics_d92fff8f92a6.fls` (708,632 bytes) |
| Simulator | sanitizers | no | 3436 | 2008 | none |
