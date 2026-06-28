#if PLATFORM_OPL1000

#include <stdio.h>
#include <string.h>
#include "../hal_flashVars.h"
#include "mw_fim.h"
#include "hal_fim_config_opl1000.h"

static FLASH_VARS_STRUCTURE s_flashVars;
static float s_energyExport;
static int s_flashVarsLoaded;

static void OpenOPL1000_LoadFlashVars(void)
{
    if (s_flashVarsLoaded)
    {
        return;
    }

    memset(&s_flashVars, 0, sizeof(s_flashVars));
    if (MwFim_FileRead(OPENOPL1000_FIM_FLASHVARS_FILE_ID, 0, sizeof(s_flashVars), (uint8_t *)&s_flashVars) != MW_FIM_OK)
    {
        memset(&s_flashVars, 0, sizeof(s_flashVars));
        printf("FlashVars FIM read miss, using defaults\r\n");
    }

    s_flashVarsLoaded = 1;
}

static int OpenOPL1000_SaveFlashVars(void)
{
    if (MwFim_FileWrite(OPENOPL1000_FIM_FLASHVARS_FILE_ID, 0, sizeof(s_flashVars), (uint8_t *)&s_flashVars) != MW_FIM_OK)
    {
        printf("FlashVars FIM write failed\r\n");
        return 0;
    }

    return sizeof(s_flashVars);
}

void HAL_FlashVars_IncreaseBootCount(void)
{
    OpenOPL1000_LoadFlashVars();
    s_flashVars.boot_count++;
    OpenOPL1000_SaveFlashVars();
}

void HAL_FlashVars_SaveBootComplete(void)
{
    OpenOPL1000_LoadFlashVars();
    s_flashVars.boot_success_count = s_flashVars.boot_count;
    OpenOPL1000_SaveFlashVars();
}

int HAL_FlashVars_GetBootFailures(void)
{
    OpenOPL1000_LoadFlashVars();
    return s_flashVars.boot_count - s_flashVars.boot_success_count;
}

int HAL_FlashVars_GetBootCount(void)
{
    OpenOPL1000_LoadFlashVars();
    return s_flashVars.boot_count;
}

void HAL_FlashVars_SaveChannel(int index, int value)
{
    OpenOPL1000_LoadFlashVars();
    if (index >= 0 && index < MAX_RETAIN_CHANNELS)
    {
        if (s_flashVars.savedValues[index] != (short)value)
        {
            s_flashVars.savedValues[index] = (short)value;
            OpenOPL1000_SaveFlashVars();
        }
    }
}

int HAL_FlashVars_GetChannelValue(int ch)
{
    OpenOPL1000_LoadFlashVars();
    if (ch >= 0 && ch < MAX_RETAIN_CHANNELS)
    {
        return s_flashVars.savedValues[ch];
    }
    return 0;
}

void HAL_FlashVars_SaveLED(byte mode, short brightness, short temperature, byte r, byte g, byte b, byte bEnableAll)
{
    int changes = 0;

    OpenOPL1000_LoadFlashVars();
    if (s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 1] != brightness)
    {
        s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 1] = brightness;
        changes++;
    }
    if (s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 2] != temperature)
    {
        s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 2] = temperature;
        changes++;
    }
    if (s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 3] != mode)
    {
        s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 3] = mode;
        changes++;
    }
    if (s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 4] != bEnableAll)
    {
        s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 4] = bEnableAll;
        changes++;
    }
    if (s_flashVars.rgb[0] != r)
    {
        s_flashVars.rgb[0] = r;
        changes++;
    }
    if (s_flashVars.rgb[1] != g)
    {
        s_flashVars.rgb[1] = g;
        changes++;
    }
    if (s_flashVars.rgb[2] != b)
    {
        s_flashVars.rgb[2] = b;
        changes++;
    }

    if (changes)
    {
        OpenOPL1000_SaveFlashVars();
    }
}

void HAL_FlashVars_ReadLED(byte *mode, short *brightness, short *temperature, byte *rgb, byte *bEnableAll)
{
    OpenOPL1000_LoadFlashVars();
    if (mode) *mode = (byte)s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 3];
    if (brightness) *brightness = s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 1];
    if (temperature) *temperature = s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 2];
    if (rgb) memcpy(rgb, s_flashVars.rgb, 3);
    if (bEnableAll) *bEnableAll = (byte)s_flashVars.savedValues[MAX_RETAIN_CHANNELS - 4];
}

int HAL_GetEnergyMeterStatus(ENERGY_METERING_DATA *data)
{
    OpenOPL1000_LoadFlashVars();
    if (data)
    {
        memcpy(data, &s_flashVars.emetering, sizeof(*data));
    }
    return 1;
}

int HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA *data)
{
    OpenOPL1000_LoadFlashVars();
    if (data)
    {
        memcpy(&s_flashVars.emetering, data, sizeof(*data));
        OpenOPL1000_SaveFlashVars();
    }
    return 1;
}

void HAL_FlashVars_SaveTotalConsumption(float total_consumption)
{
    OpenOPL1000_LoadFlashVars();
    s_flashVars.emetering.TotalConsumption = total_consumption;
    OpenOPL1000_SaveFlashVars();
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
