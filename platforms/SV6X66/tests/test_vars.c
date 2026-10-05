#include "test_storage_env.h"
#include "../sv6x66_storage.h"
#include "../../../src/hal/hal_flashVars.h"
#include <assert.h>
#include <stdio.h>
void test_storage_fault(int);
int test_storage_writes(void);
int main(int argc, char **argv)
{
    FLASH_VARS_STRUCTURE original = {0}, retained;
    byte mode, rgb[3], enabled;
    short brightness, temperature;
    assert(argc == 2);
    original.boot_count = 7;
    original.boot_success_count = 6;
    original.savedValues[0] = 123;
    original.savedValues[MAX_RETAIN_CHANNELS - 4] = 1;
    original.savedValues[MAX_RETAIN_CHANNELS - 3] = 2;
    original.savedValues[MAX_RETAIN_CHANNELS - 2] = 300;
    original.savedValues[MAX_RETAIN_CHANNELS - 1] = 75;
    original.rgb[0] = 11; original.rgb[1] = 22; original.rgb[2] = 33;
    SV6X66_StorageInit();
    assert(SV6X66_StorageWriteRecord(SV6X66_STORAGE_VARS, &original, sizeof(original)) == sizeof(original));
    if (atoi(argv[1])) {
        ENERGY_METERING_DATA fallback;
        memset(&fallback, 0x33, sizeof(fallback));
        test_storage_fault(atoi(argv[1]));
        HAL_GetEnergyMeterStatus(&fallback);
        ENERGY_METERING_DATA zero = {0};
        assert(!memcmp(&fallback, &zero, sizeof(zero)));
        mode = enabled = 99; brightness = temperature = 99; memset(rgb, 99, sizeof(rgb));
        HAL_FlashVars_ReadLED(&mode, &brightness, &temperature, rgb, &enabled);
        assert(!mode && !enabled && !brightness && !temperature && !rgb[0] && !rgb[1] && !rgb[2]);
        HAL_FlashVars_IncreaseBootCount();
        assert(test_storage_writes() == 0);
        test_storage_fault(0);
        HAL_FlashVars_IncreaseBootCount();
        assert(HAL_FlashVars_GetBootCount() == 8 && HAL_FlashVars_GetChannelValue(0) == 123);
        HAL_SetEnergyMeterStatus(&fallback);
        HAL_FlashVars_SaveTotalConsumption(2.0f);
        HAL_FlashVars_SaveChannel(0, 456);
        HAL_FlashVars_SaveLED(3, 50, 400, 10, 20, 30, 1);
        assert(test_storage_writes() == 0);
        assert(SV6X66_StorageReadRecord(SV6X66_STORAGE_VARS, &retained, sizeof(retained)) == sizeof(retained));
        assert(!memcmp(&original, &retained, sizeof(retained)));
        puts("Failed startup read preserves retained state for the boot");
        return 0;
    }
    HAL_FlashVars_IncreaseBootCount();
    assert(HAL_FlashVars_GetBootCount() == 8 && HAL_FlashVars_GetBootFailures() == 2);
    assert(HAL_FlashVars_GetChannelValue(0) == 123);
    HAL_FlashVars_ReadLED(&mode, &brightness, &temperature, rgb, &enabled);
    assert(mode == 2 && brightness == 75 && temperature == 300 && enabled == 1 && !memcmp(rgb, original.rgb, 3));
    test_storage_fault(1);
    HAL_FlashVars_SaveChannel(0, 456);
    assert(test_storage_writes() == 0);
    test_storage_fault(0);
    HAL_FlashVars_SaveChannel(0, 456);
    assert(test_storage_writes() == 2);
    assert(SV6X66_StorageReadRecord(SV6X66_STORAGE_VARS, &retained, sizeof(retained)) == sizeof(retained));
    assert(retained.savedValues[0] == 456);
    test_storage_fault(1);
    HAL_FlashVars_SaveLED(3, 50, 400, 10, 20, 30, 1);
    test_storage_fault(0);
    HAL_FlashVars_SaveLED(3, 50, 400, 10, 20, 30, 1);
    assert(test_storage_writes() == 2);
    ENERGY_METERING_DATA energy = {0};
    energy.TotalConsumption = 1.25f;
    test_storage_fault(1);
    HAL_SetEnergyMeterStatus(&energy);
    test_storage_fault(0);
    HAL_SetEnergyMeterStatus(&energy);
    assert(test_storage_writes() == 2);
    puts("Retained-state startup failure and save retry checks passed");
    return 0;
}
