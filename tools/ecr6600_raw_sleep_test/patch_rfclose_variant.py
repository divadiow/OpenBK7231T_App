from __future__ import annotations

from pathlib import Path

candidates = sorted(Path(".").rglob("raw_sleep_test/main.c"))
if len(candidates) != 1:
    print("Candidate raw-sleep source files:")
    for candidate in candidates:
        print(f"  {candidate}")
    raise RuntimeError(f"Expected exactly one raw_sleep_test/main.c, found {len(candidates)}")

SOURCE = candidates[0]
text = SOURCE.read_text(encoding="utf-8")

extern_decl = "extern bool psm_config_rf_state(bool rf_state);\n"
if extern_decl not in text:
    include_marker = '#include "psm_user.h"\n'
    if include_marker not in text:
        include_marker = '#include "psm_system.h"\n'
    if include_marker not in text:
        raise RuntimeError("Could not find a PSM include insertion point")
    text = text.replace(include_marker, include_marker + extern_decl, 1)

rtc_call = "drv_rtc_set_alarm_relative("
rtc_pos = text.find(rtc_call)
if rtc_pos < 0:
    raise RuntimeError("Could not find drv_rtc_set_alarm_relative()")

line_start = text.rfind("\n", 0, rtc_pos) + 1
indent = text[line_start:]
indent = indent[: len(indent) - len(indent.lstrip(" \t"))]

insertion = (
    f'{indent}os_printf(LM_OS, LL_INFO, '
    '"RAWTEST:RF_CLOSE_BEGIN sta=%u ap=%u ble=%u\\n",\n'
    f'{indent}    (unsigned int)psm_check_single_device_idle(PSM_DEVICE_WIFI_STA),\n'
    f'{indent}    (unsigned int)psm_check_single_device_idle(PSM_DEVICE_WIFI_AP),\n'
    f'{indent}    (unsigned int)psm_check_single_device_idle(PSM_DEVICE_BLE));\n'
    f'{indent}psm_set_wifi_status(PSM_OFF_LINE);\n'
    f'{indent}psm_set_device_status(PSM_DEVICE_WIFI_STA, PSM_DEVICE_STATUS_IDLE);\n'
    f'{indent}psm_set_device_status(PSM_DEVICE_WIFI_AP, PSM_DEVICE_STATUS_IDLE);\n'
    f'{indent}psm_set_device_status(PSM_DEVICE_BLE, PSM_DEVICE_STATUS_IDLE);\n'
    f'{indent}bool rf_state_after_close = psm_config_rf_state(false);\n'
    f'{indent}os_printf(LM_OS, LL_INFO, '
    '"RAWTEST:RF_CLOSE_DONE state=%u sta=%u ap=%u ble=%u\\n",\n'
    f'{indent}    (unsigned int)rf_state_after_close,\n'
    f'{indent}    (unsigned int)psm_check_single_device_idle(PSM_DEVICE_WIFI_STA),\n'
    f'{indent}    (unsigned int)psm_check_single_device_idle(PSM_DEVICE_WIFI_AP),\n'
    f'{indent}    (unsigned int)psm_check_single_device_idle(PSM_DEVICE_BLE));\n'
    f'{indent}psm_clear_pcu_isr();\n'
)

if "RAWTEST:RF_CLOSE_BEGIN" not in text:
    text = text[:line_start] + insertion + text[line_start:]

SOURCE.write_text(text, encoding="utf-8", newline="\n")
print(f"Patched {SOURCE}")
