#ifdef PLATFORM_BL602

#include "../hal_adc.h"

#include <stdint.h>
#include <bl_adc.h>
#include <hosal_adc.h>

#define BL602_ADC_COUNTS 4096

#ifdef CONF_ADC_ENABLE_TSEN
#define BL602_ADC_FULL_SCALE_MV 2000
#else
#define BL602_ADC_FULL_SCALE_MV 3200
#endif

static int BL602_ADC_GetChannel(int pinNumber)
{
    return bl_adc_get_channel_by_gpio(pinNumber);
}

void HAL_ADC_Init(int pinNumber)
{
    int channel = BL602_ADC_GetChannel(pinNumber);
    hosal_adc_dev_t *adc;

    if (channel < 0) {
        return;
    }

    /*
     * The BL602 startup code already owns a HOSAL ADC/DMA scanner for TSEN.
     * Reuse it here rather than creating another ADC instance.
     */
    adc = hosal_adc_device_get();
    if (adc == NULL) {
        return;
    }

    if (bl_adc_gpio_init(pinNumber) != 0) {
        return;
    }

    hosal_adc_add_channel(adc, (uint32_t)channel);
}

void HAL_ADC_Deinit(int pinNumber)
{
    int channel = BL602_ADC_GetChannel(pinNumber);
    hosal_adc_dev_t *adc;

    if (channel < 0) {
        return;
    }

    adc = hosal_adc_device_get();
    if (adc == NULL) {
        return;
    }

    hosal_adc_remove_channel(adc, (uint32_t)channel);
}

int HAL_ADC_Read(int pinNumber)
{
    int channel = BL602_ADC_GetChannel(pinNumber);
    int millivolts;
    int raw;
    hosal_adc_dev_t *adc;

    if (channel < 0) {
        return -1;
    }

    adc = hosal_adc_device_get();
    if (adc == NULL) {
        return -1;
    }

    millivolts = hosal_adc_value_get(adc, (uint32_t)channel, 20);
    if (millivolts < 0) {
        return millivolts;
    }

    /*
     * HOSAL exposes millivolts, while OpenBeken ADC consumers generally expect
     * a raw ADC-style value. Convert the BL602 result to a 12-bit range without
     * leaking BL602-specific units into shared code.
     */
    raw = (millivolts * BL602_ADC_COUNTS + BL602_ADC_FULL_SCALE_MV / 2) /
        BL602_ADC_FULL_SCALE_MV;

    if (raw >= BL602_ADC_COUNTS) {
        raw = BL602_ADC_COUNTS - 1;
    }

    return raw;
}

#endif // PLATFORM_BL602
