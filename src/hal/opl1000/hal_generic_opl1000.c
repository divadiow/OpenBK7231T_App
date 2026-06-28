#if PLATFORM_OPL1000

#include <stdint.h>
#include <stdio.h>

#include "core_cm3.h"
#include "hal_system.h"
#include "hal_wdt.h"
#include "../hal_generic.h"

void HAL_RebootModule(void)
{
    printf("Reboot requested\r\n");
    if (Hal_Sys_SwResetAll != NULL)
    {
        Hal_Sys_SwResetAll();
    }

    NVIC_SystemReset();
}

void HAL_Delay_us(int delay)
{
    volatile int loops = delay * 8;
    while (loops-- > 0)
    {
        __asm volatile("nop");
    }
}

void HAL_Configure_WDT(void)
{
}

void HAL_Run_WDT(void)
{
    if (Hal_Wdt_Clear != NULL)
    {
        Hal_Wdt_Clear();
    }
}

void HAL_RegisterPlatformSpecificCommands(void)
{
}

#endif
