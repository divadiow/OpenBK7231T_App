#if PLATFORM_SV6X66
#include "../hal_pins.h"
#include <stdint.h>
#include <stddef.h>
#include <ctype.h>
#include "gpio/drv_gpio.h"
#include "pinmux/drv_pinmux.h"
#include "pwm/drv_pwm.h"

// P03/P04 and P21/P22 are UART routes; P14..P19 belong to XIP flash.
static int valid_pin(int pin)
{
    return pin >= 0 && pin < 23 && pin != 3 && pin != 4 &&
        !(pin >= 14 && pin <= 19) && pin != 21 && pin != 22;
}
static const char *const names[] = {
    "P00", "P01", "P02", "P03", "P04", "P05", "P06", "P07",
    "P08", "P09", "P10", "P11", "P12", "P13", "P14", "P15",
    "P16", "P17", "P18", "P19", "P20", "P21", "P22"
};
static uint32_t pwm_frequency[3];
static uint32_t pwm_duty[3];
const char *HAL_PIN_GetPinNameAlias(int pin)
{
    return pin >= 0 && pin < 23 ? names[pin] : "error";
}
int HAL_PIN_Find(const char *name)
{
    if (!name) return -1;
    for (int pin = 0; pin < 23; pin++) {
        const unsigned char *a = (const unsigned char *)name;
        const unsigned char *b = (const unsigned char *)names[pin];
        while (*a && *b && toupper(*a) == toupper(*b)) { a++; b++; }
        if (!*a && !*b && valid_pin(pin)) return pin;
    }
    return -1;
}
unsigned int HAL_GetGPIOPin(int pin) { return valid_pin(pin) ? (unsigned)pin : (unsigned)-1; }
void HAL_PIN_SetOutputValue(int pin, int value)
{
    if (valid_pin(pin)) drv_gpio_set_logic(pin, value ? GPIO_LOGIC_HIGH : GPIO_LOGIC_LOW);
}
int HAL_PIN_ReadDigitalInput(int pin)
{
    return valid_pin(pin) && drv_gpio_get_logic(pin) == GPIO_LOGIC_HIGH;
}
static void input(int pin, gpio_pull_t pull)
{
    if (!valid_pin(pin)) return;
    HAL_PIN_PWM_Stop(pin);
    drv_gpio_set_mode(pin, PIN_MODE_GPIO);
    drv_gpio_set_pull(pin, pull);
    drv_gpio_set_dir(pin, GPIO_DIR_IN);
}
void HAL_PIN_Setup_Input(int pin) { input(pin, GPIO_PULL_NONE); }
void HAL_PIN_Setup_Input_Pullup(int pin) { input(pin, GPIO_PULL_UP); }
void HAL_PIN_Setup_Input_Pulldown(int pin) { input(pin, GPIO_PULL_DOWN); }
void HAL_PIN_Setup_Output(int pin)
{
    if (!valid_pin(pin)) return;
    HAL_PIN_PWM_Stop(pin);
    drv_gpio_set_mode(pin, PIN_MODE_GPIO);
    drv_gpio_set_dir(pin, GPIO_DIR_OUT);
}
void HAL_PIN_Setup_Output_Initial(int pin, int value)
{
    if (!valid_pin(pin)) return;
    HAL_PIN_PWM_Stop(pin);
    HAL_PIN_SetOutputValue(pin, value);
    HAL_PIN_Setup_Output(pin);
}
int HAL_PIN_CanThisPinBePWM(int pin) { return pin >= 0 && pin < 3; }
int PIN_GetPWMIndexForPinIndex(int pin) { return HAL_PIN_CanThisPinBePWM(pin) ? pin : -1; }
void HAL_PIN_PWM_Start(int pin, int frequency)
{
    pinmux_fun_t function;
    if (!HAL_PIN_CanThisPinBePWM(pin) || frequency < 5 || frequency > 4000000) return;
    if (pwm_frequency[pin]) {
        if (pwm_frequency[pin] != (uint32_t)frequency &&
            drv_pwm_config(pin, frequency, pwm_duty[pin], 0) == 0)
            pwm_frequency[pin] = frequency;
        return;
    }
    function = (pinmux_fun_t)(SEL_PWM_0 + pin);
    if (drv_pwm_init(pin) != 0) return;
    if (drv_pwm_config(pin, frequency, 0, 0) != 0 ||
        drv_gpio_set_mode(pin, PIN_MODE_GPIO) != 0 ||
        drv_pinmux_manual_function_select_enable(function) != 0 ||
        drv_gpio_set_dir(pin, GPIO_DIR_OUT) != 0 || drv_pwm_enable(pin) != 0) {
        drv_pwm_disable(pin);
        drv_pinmux_manual_function_select_disable(function);
        pwm_frequency[pin] = 0;
        return;
    }
    pwm_frequency[pin] = frequency;
    pwm_duty[pin] = 0;
}
void HAL_PIN_PWM_Update(int pin, float percent)
{
    if (!HAL_PIN_CanThisPinBePWM(pin) || !pwm_frequency[pin]) return;
    if (!(percent >= 0)) percent = 0; // Includes NaN.
    if (percent > 100) percent = 100;
    uint32_t duty = (uint32_t)(percent * 4096.0f / 100.0f + 0.5f);
    if (drv_pwm_config(pin, pwm_frequency[pin], duty, 0) == 0)
        pwm_duty[pin] = duty;
}
void HAL_PIN_PWM_Stop(int pin)
{
    if (!HAL_PIN_CanThisPinBePWM(pin) || !pwm_frequency[pin]) return;
    drv_pwm_disable(pin);
    drv_pinmux_manual_function_select_disable((pinmux_fun_t)(SEL_PWM_0 + pin));
    pwm_frequency[pin] = 0;
    pwm_duty[pin] = 0;
}
// GPIO ISR argument semantics are undocumented in the binary driver.
void HAL_AttachInterrupt(int pin, OBKInterruptType mode, OBKInterruptHandler handler)
{
    (void)pin; (void)mode; (void)handler;
}
void HAL_DetachInterrupt(int pin) { (void)pin; }
#endif
