# Remediation diagnostics: `f1ca0495a8efffae49f463a973df54d34e582075`

- Target set: **lfs-hass**
- Targets found: **5**
- Targets with build output: **4**
- Warning lines: **4255**
- Error lines: **2008**

## Known-warning fingerprints

| Fingerprint | Count |
| --- | ---: |
| `cjson_unicode_escape` | 2 |
| `nonvoid_return` | 8 |
| `implicit_allocator_declaration` | 16 |
| `maybe_uninitialized` | 0 |
| `shift_count_width` | 0 |
| `linker_symbol_size_change` | 0 |
| `format_truncation` | 8 |
| `incompatible_pointer` | 61 |

## Targets

| Platform | Variant | Output | Warnings | Errors | Largest firmware output |
| --- | --- | ---: | ---: | ---: | --- |
| OpenBK7231N | default | yes | 103 | 0 | `output/agent_remediate-build-diagnostics_f1ca0495a8ef/OpenBK7231M_QIO_agent_remediate-build-diagnostics_f1ca0495a8ef.bin` (1,220,464 bytes) |
| OpenESP32C3 | 2M | yes | 1 | 0 | `output/agent_remediate-build-diagnostics_f1ca0495a8ef_2M/OpenESP32C3_agent_remediate-build-diagnostics_f1ca0495a8ef_2M.factory.bin` (1,041,440 bytes) |
| OpenRTL8720D | default | yes | 517 | 0 | `output/agent_remediate-build-diagnostics_f1ca0495a8ef/OpenRTL8720D_agent_remediate-build-diagnostics_f1ca0495a8ef.bin` (1,138,688 bytes) |
| OpenW800 | default | yes | 200 | 0 | `output/agent_remediate-build-diagnostics_f1ca0495a8ef/OpenW800_agent_remediate-build-diagnostics_f1ca0495a8ef.fls` (708,944 bytes) |
| Simulator | sanitizers | no | 3434 | 2008 | none |
