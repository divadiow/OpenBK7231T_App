#include <assert.h>
#include <math.h>
#include "gpio/drv_gpio.h"
#include "pinmux/drv_pinmux.h"
#include "pwm/drv_pwm.h"
#include "hal/hal_pins.h"

int PIN_GetPWMIndexForPinIndex(int);
static int gpio_calls, pwm_calls, fail_config, disable_calls;
static uint32_t last_duty, frequency[5];
static gpio_dir_t direction[23];
int8_t drv_gpio_set_mode(gpio_pin_t pin, pin_mode_t mode) { assert(mode == PIN_MODE_GPIO); (void)pin; gpio_calls++; return 0; }
int8_t drv_gpio_set_dir(gpio_pin_t pin, gpio_dir_t dir) { assert(dir == GPIO_DIR_IN || dir == GPIO_DIR_OUT); direction[pin] = dir; gpio_calls++; return 0; }
int8_t drv_gpio_set_pull(gpio_pin_t pin, gpio_pull_t pull) { (void)pull; (void)pin; gpio_calls++; return 0; }
int8_t drv_gpio_set_logic(gpio_pin_t pin, gpio_logic_t logic) { assert(logic == GPIO_LOGIC_HIGH || logic == GPIO_LOGIC_LOW); (void)pin; gpio_calls++; return 0; }
gpio_logic_t drv_gpio_get_logic(gpio_pin_t pin) { (void)pin; gpio_calls++; return GPIO_LOGIC_HIGH; }
int8_t drv_pinmux_manual_function_select_enable(pinmux_fun_t function) { assert(function >= SEL_PWM_0 && function <= SEL_PWM_2); return 0; }
int8_t drv_pinmux_manual_function_select_disable(pinmux_fun_t function) { assert(function >= SEL_PWM_0 && function <= SEL_PWM_2); return 0; }
int8_t drv_pwm_init(uint8_t id) { assert(id < 3); pwm_calls++; return 0; }
int8_t drv_pwm_config(uint8_t id, uint32_t hz, uint32_t duty, uint8_t invert)
{
    assert(id < 3 && duty <= 4096 && invert == 0);
    pwm_calls++; last_duty = duty; frequency[id] = hz;
    return fail_config ? -1 : 0;
}
int8_t drv_pwm_enable(uint8_t id) { assert(id < 3 && direction[id] == GPIO_DIR_OUT); pwm_calls++; return 0; }
int8_t drv_pwm_disable(uint8_t id) { assert(id < 3); disable_calls++; return 0; }
int main(void)
{
    int calls;
    assert(HAL_PIN_Find("p02") == 2);
    assert(PIN_GetPWMIndexForPinIndex(0) == 0 && PIN_GetPWMIndexForPinIndex(2) == 2);
    assert(PIN_GetPWMIndexForPinIndex(-1) == -1 && PIN_GetPWMIndexForPinIndex(3) == -1);
    assert(HAL_PIN_Find("P03") == -1);
    for (int pin = -5; pin < 30; pin++) {
        if (pin >= 0 && pin < 23 && pin != 3 && pin != 4 && !(pin >= 14 && pin <= 19) && pin != 21 && pin != 22) continue;
        calls = gpio_calls;
        HAL_PIN_Setup_Output(pin);
        HAL_PIN_SetOutputValue(pin, 1);
        HAL_PIN_ReadDigitalInput(pin);
        assert(calls == gpio_calls);
    }
    calls = pwm_calls;
    HAL_PIN_PWM_Start(3, 1000);
    HAL_PIN_PWM_Start(0, 0);
    HAL_PIN_PWM_Start(0, 4000001);
    assert(calls == pwm_calls);
    HAL_PIN_Setup_Input(0);
    HAL_PIN_PWM_Start(0, 1000);
    HAL_PIN_PWM_Start(1, 2000);
    HAL_PIN_PWM_Update(0, 50);
    assert(last_duty == 2048 && frequency[0] == 1000 && frequency[1] == 2000);
    HAL_PIN_PWM_Update(1, 120);
    assert(last_duty == 4096);
    HAL_PIN_PWM_Update(1, -1);
    assert(last_duty == 0);
    HAL_PIN_PWM_Update(1, NAN);
    assert(last_duty == 0);
    HAL_PIN_PWM_Stop(1);
    calls = pwm_calls;
    HAL_PIN_PWM_Update(1, 30);
    assert(calls == pwm_calls);
    HAL_PIN_Setup_Output(0);
    calls = pwm_calls;
    HAL_PIN_PWM_Update(0, 30);
    assert(calls == pwm_calls);
    fail_config = 1;
    HAL_PIN_PWM_Start(2, 1000);
    calls = pwm_calls;
    HAL_PIN_PWM_Update(2, 50);
    assert(calls == pwm_calls && disable_calls >= 3);
    return 0;
}
