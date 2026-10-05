#include "../../new_common.h"
#include "../hal_generic.h"
#include "wdt/drv_wdt.h"
void HAL_RebootModule(void) { drv_wdt_init(); drv_wdt_enable(SYS_WDT, 1); for (;;) {} }
void HAL_Delay_us(int delay) { if (delay > 0) OS_UsDelay(delay); }
int HAL_FlashRead(char *buffer, int len, int addr)
{
    if (!buffer || len < 0 || addr < 0 || (unsigned)addr > 0x200000U || (unsigned)len > 0x200000U - (unsigned)addr) return -1;
    memcpy(buffer, (const void *)(0x30000000U + (unsigned)addr), len);
    return 0;
}
int LWIP_GetMaxSockets(void) { return MEMP_NUM_NETCONN; }
int LWIP_GetActiveSockets(void) { return 0; }
