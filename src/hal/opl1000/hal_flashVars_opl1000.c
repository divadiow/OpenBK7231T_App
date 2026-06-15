#if PLATFORM_OPL1000

#include <string.h>
#include "../hal_flashVars.h"

static FLASH_VARS_STRUCTURE s_flashVars;
static float s_energyExport;

void HAL_FlashVars_IncreaseBootCount(void)
{
    s_flashVars.boot_count++;
}

void HAL_FlashVars_SaveBootComplete(void)
{
    s_flashVars.boot_success_count = s_flashVars.boot_count;
}

int HAL_FlashVars_GetBootFailures(void)
{
    return s_flashVars.boot_count - s_flashVars.boot_success_count;
}

int HAL_FlashVars_GetBootCount(void)
{
    return s_flashVars.boot_count;
}

void HAL_FlashVars_SaveChannel(int index, int value)
{
    if (index >= 0 && index < MAX_RETAIN_CHANNELS)
    {
        s_flashVars.savedValues[index] = (short)value;
    }
}

int HAL_FlashVars_GetChannelValue(int ch)
{
    if (ch >= 0 && ch < MAX_RETAIN_CHANNELS)
    {
        return s_flashVars.savedValues[ch];
    }
    return 0;
}

void HAL_FlashVars_SaveLED(byte mode, short brightness, short temperature, byte r, byte g, byte b, byte bEnableAll)
{
    (void)mode;
    (void)brightness;
    (void)temperature;
    s_flashVars.rgb[0] = r;
    s_flashVars.rgb[1] = g;
    s_flashVars.rgb[2] = b;
    s_flashVars.len = bEnableAll;
}

void HAL_FlashVars_ReadLED(byte *mode, short *brightness, short *temperature, byte *rgb, byte *bEnableAll)
{
    if (mode) *mode = 0;
    if (brightness) *brightness = 100;
    if (temperature) *temperature = 0;
    if (rgb) memcpy(rgb, s_flashVars.rgb, 3);
    if (bEnableAll) *bEnableAll = s_flashVars.len;
}

int HAL_GetEnergyMeterStatus(ENERGY_METERING_DATA *data)
{
    if (data)
    {
        memcpy(data, &s_flashVars.emetering, sizeof(*data));
    }
    return 1;
}

int HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA *data)
{
    if (data)
    {
        memcpy(&s_flashVars.emetering, data, sizeof(*data));
    }
    return 1;
}

void HAL_FlashVars_SaveTotalConsumption(float total_consumption)
{
    s_flashVars.emetering.TotalConsumption = total_consumption;
}

void HAL_FlashVars_SaveEnergyExport(float f)
{
    s_energyExport = f;
}

float HAL_FlashVars_GetEnergyExport(void)
{
    return s_energyExport;
}

#ifdef ENABLE_DRIVER_HLW8112SPI
void HAL_FlashVars_SaveEnergy(ENERGY_DATA **data, int channel_count)
{
    (void)data;
    (void)channel_count;
}

void HAL_FlashVars_GetEnergy(ENERGY_DATA *data, ENERGY_CHANNEL channel)
{
    (void)channel;
    if (data)
    {
        data->Import = 0;
        data->Export = 0;
    }
}
#endif

#endif
