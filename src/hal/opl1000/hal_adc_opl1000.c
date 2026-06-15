#if PLATFORM_OPL1000

#include "../hal_adc.h"

void HAL_ADC_Init(int pinNumber)
{
    (void)pinNumber;
}

int HAL_ADC_Read(int pinNumber)
{
    (void)pinNumber;
    return 0;
}

void HAL_ADC_Deinit(int pinNumber)
{
    (void)pinNumber;
}

#endif
