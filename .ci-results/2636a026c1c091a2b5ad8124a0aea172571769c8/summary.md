# Remediation diagnostics: `2636a026c1c091a2b5ad8124a0aea172571769c8`

- Target set: **lfs-hass**
- Targets found: **5**
- Targets with build output: **4**
- Warning lines: **4252**
- Error lines: **2008**

## Known-warning fingerprints

| Fingerprint | Count |
| --- | ---: |
| `cjson_unicode_escape` | 1 |
| `nonvoid_return` | 8 |
| `implicit_allocator_declaration` | 16 |
| `maybe_uninitialized` | 0 |
| `shift_count_width` | 0 |
| `linker_symbol_size_change` | 0 |
| `format_truncation` | 4 |
| `incompatible_pointer` | 61 |

## Targets

| Platform | Variant | Output | Warnings | Errors | Largest firmware output |
| --- | --- | ---: | ---: | ---: | --- |
| OpenBK7231N | default | yes | 103 | 0 | `output/agent_remediate-build-diagnostics_2636a026c1c0/OpenBK7231M_QIO_agent_remediate-build-diagnostics_2636a026c1c0.bin` (1,220,464 bytes) |
| OpenESP32C3 | 2M | yes | 1 | 0 | `output/agent_remediate-build-diagnostics_2636a026c1c0_2M/OpenESP32C3_agent_remediate-build-diagnostics_2636a026c1c0_2M.factory.bin` (1,041,456 bytes) |
| OpenRTL8720D | default | yes | 516 | 0 | `output/agent_remediate-build-diagnostics_2636a026c1c0/OpenRTL8720D_agent_remediate-build-diagnostics_2636a026c1c0.bin` (1,138,688 bytes) |
| OpenW800 | default | yes | 200 | 0 | `output/agent_remediate-build-diagnostics_2636a026c1c0/OpenW800_agent_remediate-build-diagnostics_2636a026c1c0.fls` (708,956 bytes) |
| Simulator | sanitizers | no | 3432 | 2008 | none |
