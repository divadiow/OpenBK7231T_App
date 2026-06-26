#if PLATFORM_OPL1000

#include <stddef.h>
#include <stdint.h>

#include "mw_fim_default.h"
#include "../hal_flashVars.h"
#include "hal_fim_config_opl1000.h"

static uint32_t s_OpenOPL1000CfgAddr[OPENOPL1000_FIM_CFG_RECORDS];
static uint32_t s_OpenOPL1000FlashVarsAddr[OPENOPL1000_FIM_FLASHVARS_RECORDS];

const T_MwFimFileInfo g_OpenOPL1000FimGroupTable11[] =
{
    {OPENOPL1000_FIM_CFG_FILE_ID, OPENOPL1000_FIM_CFG_RECORDS, OPENOPL1000_FIM_CFG_SIZE, NULL, s_OpenOPL1000CfgAddr},
    {OPENOPL1000_FIM_FLASHVARS_FILE_ID, OPENOPL1000_FIM_FLASHVARS_RECORDS, OPENOPL1000_FIM_FLASHVARS_SIZE, NULL, s_OpenOPL1000FlashVarsAddr},
    {0xFFFFFFFF, 0x00, 0x00, NULL, NULL}
};

void OpenOPL1000_FimRegister(void)
{
    T_MwFimZoneInfo zone =
    {
        OPENOPL1000_FIM_ZONE_BASE_ADDR,
        OPENOPL1000_FIM_ZONE_BLOCK_SIZE,
        OPENOPL1000_FIM_ZONE_BLOCK_NUM,
        g_ubaMwFimVersionTable[1]
    };

    MwFim_ZoneInfoUpdate(1, &zone);
    MwFim_GroupInfoUpdate(1, 1, (T_MwFimFileInfo *)g_OpenOPL1000FimGroupTable11);
    MwFim_GroupVersionUpdate(1, 1, OPENOPL1000_FIM_CFG_VERSION);
}

#endif
