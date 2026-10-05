#include "../../new_common.h"
#include "../hal_flashVars.h"
#include "sv6x66_storage.h"

static FLASH_VARS_STRUCTURE vars;
static xSemaphoreHandle vars_mutex;
static int loaded, dirty, read_failed;
void SV6X66_FlashVarsInit(void) { vars_mutex = xSemaphoreCreateMutex(); }
static int lock_vars(void)
{
    if (!vars_mutex || xSemaphoreTake(vars_mutex, portMAX_DELAY) != pdTRUE) return 0;
    if (!loaded) {
        FLASH_VARS_STRUCTURE retained = {0};
        if (SV6X66_StorageReadRecord(SV6X66_STORAGE_VARS, &retained, sizeof(retained)) < 0) {
            read_failed = 1;
            xSemaphoreGive(vars_mutex);
            return 0;
        }
        vars = retained;
        loaded = 1;
    }
    return 1;
}
static void save_vars(void)
{
    // Callers may have initialized live state from defaults after a failed
    // read. Preserve the retained record until the next clean boot.
    if (read_failed) return;
    vars.len = sizeof(vars);
    dirty = SV6X66_StorageWriteRecord(SV6X66_STORAGE_VARS, &vars, sizeof(vars)) != sizeof(vars);
}
static void unlock_vars(void) { xSemaphoreGive(vars_mutex); }
void HAL_FlashVars_IncreaseBootCount(void)
{
    if (!lock_vars()) return;
    vars.boot_count++;
    save_vars();
    unlock_vars();
}
void HAL_FlashVars_SaveBootComplete(void)
{
    if (!lock_vars()) return;
    vars.boot_success_count = vars.boot_count;
    save_vars();
    unlock_vars();
}
int HAL_FlashVars_GetBootFailures(void)
{
    int result;
    if (!lock_vars()) return 0;
    result = (unsigned short)(vars.boot_count - vars.boot_success_count);
    unlock_vars();
    return result;
}
int HAL_FlashVars_GetBootCount(void)
{
    int result;
    if (!lock_vars()) return 0;
    result = vars.boot_count;
    unlock_vars();
    return result;
}
void HAL_FlashVars_SaveChannel(int index, int value)
{
    if (index < 0 || index >= MAX_RETAIN_CHANNELS || !lock_vars()) return;
    if (dirty || vars.savedValues[index] != (short)value) {
        vars.savedValues[index] = value;
        save_vars();
    }
    unlock_vars();
}
int HAL_FlashVars_GetChannelValue(int index)
{
    int result;
    if (index < 0 || index >= MAX_RETAIN_CHANNELS || !lock_vars()) return 0;
    result = vars.savedValues[index];
    unlock_vars();
    return result;
}
void HAL_FlashVars_ReadLED(byte *mode, short *brightness, short *temperature, byte *rgb, byte *enable_all)
{
    if (mode) *mode = 0;
    if (brightness) *brightness = 0;
    if (temperature) *temperature = 0;
    if (rgb) memset(rgb, 0, 3);
    if (enable_all) *enable_all = 0;
    if (!lock_vars()) return;
    if (enable_all) *enable_all = vars.savedValues[MAX_RETAIN_CHANNELS - 4];
    if (mode) *mode = vars.savedValues[MAX_RETAIN_CHANNELS - 3];
    if (temperature) *temperature = vars.savedValues[MAX_RETAIN_CHANNELS - 2];
    if (brightness) *brightness = vars.savedValues[MAX_RETAIN_CHANNELS - 1];
    if (rgb) memcpy(rgb, vars.rgb, 3);
    unlock_vars();
}
void HAL_FlashVars_SaveLED(byte mode, short brightness, short temperature, byte r, byte g, byte b, byte enable_all)
{
    FLASH_VARS_STRUCTURE old;
    if (!lock_vars()) return;
    old = vars;
    vars.savedValues[MAX_RETAIN_CHANNELS - 4] = enable_all;
    vars.savedValues[MAX_RETAIN_CHANNELS - 3] = mode;
    vars.savedValues[MAX_RETAIN_CHANNELS - 2] = temperature;
    vars.savedValues[MAX_RETAIN_CHANNELS - 1] = brightness;
    vars.rgb[0] = r; vars.rgb[1] = g; vars.rgb[2] = b;
    if (dirty || memcmp(&old, &vars, sizeof(vars))) save_vars();
    unlock_vars();
}
int HAL_GetEnergyMeterStatus(ENERGY_METERING_DATA *data)
{
    if (!data) return 0;
    memset(data, 0, sizeof(*data));
    if (!lock_vars()) return 0;
    *data = vars.emetering;
    unlock_vars();
    return 0;
}
int HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA *data)
{
    if (!data || !lock_vars()) return 0;
    if (dirty || memcmp(&vars.emetering, data, sizeof(*data))) {
        vars.emetering = *data;
        save_vars();
    }
    unlock_vars();
    return 0;
}
void HAL_FlashVars_SaveTotalConsumption(float value)
{
    if (!lock_vars()) return;
    vars.emetering.TotalConsumption = value;
    unlock_vars(); // Persist with the periodic energy status snapshot.
}
