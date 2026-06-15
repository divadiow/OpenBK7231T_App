#if PLATFORM_OPL1000

#include <string.h>
#include "../hal_flashConfig.h"

#define OPENOPL1000_CFG_RAM_SIZE 4096

static unsigned char s_cfgRam[OPENOPL1000_CFG_RAM_SIZE];
static int s_cfgRamValid;

int config_get_tableOffsets(int tableID, int *outStart, int *outLen)
{
    (void)tableID;
    if (outStart) *outStart = 0;
    if (outLen) *outLen = 0;
    return 0;
}

int beken_hal_flash_read(const unsigned int addr, void *dst, const unsigned int size)
{
    (void)addr;
    if (dst && size)
    {
        memset(dst, 0xFF, size);
    }
    return 0;
}

int HAL_Configuration_ReadConfigMemory(void *target, int dataLen)
{
    if (target == 0 || dataLen <= 0)
    {
        return 0;
    }

    if (dataLen > OPENOPL1000_CFG_RAM_SIZE)
    {
        dataLen = OPENOPL1000_CFG_RAM_SIZE;
    }

    if (!s_cfgRamValid)
    {
        memset(target, 0, dataLen);
        return 0;
    }

    memcpy(target, s_cfgRam, dataLen);
    return dataLen;
}

int HAL_Configuration_SaveConfigMemory(void *src, int dataLen)
{
    if (src == 0 || dataLen <= 0)
    {
        return 0;
    }

    if (dataLen > OPENOPL1000_CFG_RAM_SIZE)
    {
        dataLen = OPENOPL1000_CFG_RAM_SIZE;
    }

    memcpy(s_cfgRam, src, dataLen);
    s_cfgRamValid = 1;
    return 1;
}

#endif
