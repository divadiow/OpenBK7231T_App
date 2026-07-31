from __future__ import annotations

from pathlib import Path

main_path = Path("tools/ecr6600_raw_sleep_test/board/main.c")
build_path = Path("tools/ecr6600_raw_sleep_test/build_raw_sleep_tests.sh")

source = main_path.read_text(encoding="utf-8")

include_marker = '#include "psm_user.h"\n'
extern_decl = 'extern bool psm_config_rf_state(bool rf_state);\n'
if extern_decl not in source:
    if source.count(include_marker) != 1:
        raise RuntimeError("Could not locate psm_user.h include")
    source = source.replace(include_marker, include_marker + extern_decl, 1)

var_marker = "    unsigned int rtc_alarm;\n"
if "bool rf_state_after_close;" not in source:
    if source.count(var_marker) != 1:
        raise RuntimeError("Could not locate raw_enter_deep_sleep locals")
    source = source.replace(
        var_marker,
        var_marker + "    bool rf_state_after_close;\n",
        1,
    )

alarm_marker = "    rtc_alarm = drv_rtc_set_alarm_relative(rtc_ticks);\n"
rf_block = """    os_printf(LM_OS, LL_INFO,
              "RAWTEST:%s:RF_CLOSE_BEGIN sta=%u ap=%u ble=%u\\r\\n",
              raw_variant_name(),
              (unsigned int)psm_check_single_device_idle(PSM_DEVICE_WIFI_STA),
              (unsigned int)psm_check_single_device_idle(PSM_DEVICE_WIFI_AP),
              (unsigned int)psm_check_single_device_idle(PSM_DEVICE_BLE));

    psm_set_wifi_status(PSM_OFF_LINE);
    psm_set_device_status(PSM_DEVICE_WIFI_STA, PSM_DEVICE_STATUS_IDLE);
    psm_set_device_status(PSM_DEVICE_WIFI_AP, PSM_DEVICE_STATUS_IDLE);
    psm_set_device_status(PSM_DEVICE_BLE, PSM_DEVICE_STATUS_IDLE);

    rf_state_after_close = psm_config_rf_state(false);
    os_printf(LM_OS, LL_INFO,
              "RAWTEST:%s:RF_CLOSE_DONE state=%u sta=%u ap=%u ble=%u\\r\\n",
              raw_variant_name(), (unsigned int)rf_state_after_close,
              (unsigned int)psm_check_single_device_idle(PSM_DEVICE_WIFI_STA),
              (unsigned int)psm_check_single_device_idle(PSM_DEVICE_WIFI_AP),
              (unsigned int)psm_check_single_device_idle(PSM_DEVICE_BLE));

    psm_clear_pcu_isr();
"""
if "RF_CLOSE_BEGIN" not in source:
    if source.count(alarm_marker) != 1:
        raise RuntimeError("Could not locate RTC alarm call")
    source = source.replace(alarm_marker, rf_block + alarm_marker, 1)

main_path.write_text(source, encoding="utf-8", newline="\n")

build = build_path.read_text(encoding="utf-8")
old_tail = """build_variant 1 A PSM_RTC_ONLY
build_variant 2 B WIFI_INIT_UNASSOCIATED
build_variant 3 C STA_CONNECT_DISCONNECT
build_variant 4 D STA_DISCONNECT_WITH_WDT
"""
new_tail = """# This follow-up isolates whether RF must be explicitly closed before
# the exact SDK's direct RTC deep-sleep sequence. Build Variant B only.
build_variant 2 B WIFI_INIT_UNASSOCIATED_RF_CLOSED
"""
if old_tail not in build:
    raise RuntimeError("Could not locate build-variant list")
build = build.replace(old_tail, new_tail, 1)
build_path.write_text(build, encoding="utf-8", newline="\n")

print(f"Patched {main_path}")
print(f"Patched {build_path} to build Variant B only")
