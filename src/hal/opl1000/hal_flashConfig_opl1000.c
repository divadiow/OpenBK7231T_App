#if PLATFORM_OPL1000

#include <string.h>
#include <stdio.h>
#include "../hal_flashConfig.h"
#include "mw_fim.h"
#include "hal_fim_config_opl1000.h"

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

    if (dataLen != (int)OPENOPL1000_FIM_CFG_SIZE)
    {
        memset(target, 0, dataLen);
        return 0;
    }

    if (MwFim_FileRead(OPENOPL1000_FIM_CFG_FILE_ID, 0, OPENOPL1000_FIM_CFG_SIZE, (uint8_t *)target) != MW_FIM_OK)
    {
        memset(target, 0, dataLen);
        printf("[OpenOPL1000] cfg FIM read miss, using defaults\r\n");
        return 0;
    }

    return dataLen;
}

int HAL_Configuration_SaveConfigMemory(void *src, int dataLen)
{
    if (src == 0 || dataLen <= 0)
    {
        return 0;
    }

    if (dataLen != (int)OPENOPL1000_FIM_CFG_SIZE)
    {
        return 0;
    }

    if (MwFim_FileWrite(OPENOPL1000_FIM_CFG_FILE_ID, 0, OPENOPL1000_FIM_CFG_SIZE, (uint8_t *)src) != MW_FIM_OK)
    {
        printf("[OpenOPL1000] cfg FIM write failed\r\n");
        return 0;
    }

    printf("[OpenOPL1000] cfg FIM saved %d bytes\r\n", dataLen);
    return dataLen;
}

#endif
