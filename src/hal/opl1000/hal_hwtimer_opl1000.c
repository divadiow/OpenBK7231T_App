#if PLATFORM_OPL1000

#include "../hal_hwtimer.h"

int8_t HAL_RequestHWTimer(float requestPeriodUs, float *realPeriodUs, HWTimerCB callback, void *arg)
{
    (void)requestPeriodUs;
    (void)callback;
    (void)arg;
    if (realPeriodUs) *realPeriodUs = 0;
    return -1;
}

void HAL_HWTimerStart(int8_t timer)
{
    (void)timer;
}

void HAL_HWTimerStop(int8_t timer)
{
    (void)timer;
}

void HAL_HWTimerDeinit(int8_t timer)
{
    (void)timer;
}

#endif
