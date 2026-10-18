#if PLATFORM_SV6X66

#include "new_common.h"
#include "new_pins.h"
#include "hal/hal_pins.h"
#include "gpio/drv_gpio.h"
#include "pwm/drv_pwm.h"
#include "pinmux/drv_pinmux.h"
#include "sys/sys_clock.h"

/* Datasheet alternatives describe silicon muxes, not enabled peripherals or
	* board wiring. Keep GPIO<n> first for the shared alias token parser. */
static const char *names[23] = {
	"GPIO0(PWM0)", "GPIO1(PWM1)", "GPIO2(PWM2)",
	"GPIO3(PWM3/RX0)", "GPIO4(PWM4/TX0)", "GPIO5(RTS1)",
	"GPIO6 (UART1 RX/I2S DI; recovery RX reserved)",
	"GPIO7 (UART1 TX/I2S DO; recovery TX/debug strap reserved)",
	"GPIO8(SPI CLK)", "GPIO9(CTS1)", "GPIO10(SDA)", "GPIO11",
	"GPIO12(SCL)", "GPIO13(CS)",
	"GPIO14 (flash DI/strap reserved)", "GPIO15 (flash CLK/strap reserved)",
	"GPIO16 (flash HD reserved)", "GPIO17 (flash CS reserved)",
	"GPIO18 (flash DO reserved)", "GPIO19 (flash WP reserved)",
	"GPIO20 (SIO20/I2S MCLK/SPI1 slave CLK; clock/strap reserved)",
	"GPIO21 (UART0 TX/I2C master-slave SCL; debug TX reserved)",
	"GPIO22 (UART0 RX/I2C master-slave SDA; debug RX reserved)"};
static int usable(int pin) {
	return pin >= 0 && pin <= 13 && pin != 6 && pin != 7;
}
int SV6X66_PinAvailable(int pin) {
	return usable(pin);
}
static void report_failure(int pin, int status) {
	if (status)
		SV6X66_Printf("GPIO%d unavailable or SDK operation failed (%d)\r\n", pin, status);
}
const char *HAL_PIN_GetPinNameAlias(int pin) {
	return pin >= 0 && pin < 23 ? names[pin] : "invalid";
}
unsigned int HAL_GetGPIOPin(int pin) {
	return usable(pin) ? (unsigned int)pin : UINT32_MAX;
}
int HAL_PIN_ReadDigitalInput(int pin) {
	if (!usable(pin)) {
		report_failure(pin, -1);
		return 0;
	}
	return drv_gpio_get_logic((gpio_pin_t)pin) == GPIO_LOGIC_HIGH;
}
void HAL_PIN_SetOutputValue(int pin, int value) {
	int result = usable(pin) ? drv_gpio_set_logic((gpio_pin_t)pin, value ? GPIO_LOGIC_HIGH : GPIO_LOGIC_LOW) : -1;
	report_failure(pin, result);
}
/* SV6X66_TEST test10: GPIO0-only manual strong-drive experiment before output
	* enable. Remove this helper and its two call sites after the pad-ownership
	* question is resolved. Do not generalize this override to other pins. */
#define SV6X66_TEST_GPIO0_STRONG_DRIVE 1
static int gpio0_test_strong_drive(int pin) {
#if SV6X66_TEST_GPIO0_STRONG_DRIVE
	int result;
	if (pin != 0)
		return 0;
	result = drv_gpio_set_driving_strengh(GPIO_00, GPIO_DRIVING_STRONG);
	return result;
#else
	(void)pin;
	return 0;
#endif
}
/* SDK PWM has five register entries; initial OBK support deliberately follows
	* the existing port's bounded GPIO0..2 mapping. No board defaults are assigned. */
typedef struct {
	uint32_t frequency, duty;
	int owned, ready;
} sv_pwm_state;
static sv_pwm_state pwm_state[5];
static int pwm_release(int pin) {
	int result;
	if (pin < 0 || pin > 4 || !pwm_state[pin].owned)
		return 0;
	pwm_state[pin].ready = 0;
	result = drv_pwm_disable((uint8_t)pin);
	if (!result)
		result = drv_pinmux_manual_function_select_disable((pinmux_fun_t)(SEL_PWM_0 + pin));
	if (!result) {
		pwm_state[pin].owned = 0;
		pwm_state[pin].frequency = 0;
		pwm_state[pin].duty = 0;
	}
	return result;
}
static int pwm_apply(int pin, uint32_t frequency, uint32_t duty) {
	int result = drv_pwm_config((uint8_t)pin, frequency, duty, 0);
	if (!result)
		result = drv_pwm_enable((uint8_t)pin);
	if (!result)
		result = drv_pinmux_manual_function_select_enable((pinmux_fun_t)(SEL_PWM_0 + pin));
	return result;
}
static int pwm_change(int pin, uint32_t frequency, uint32_t duty) {
	sv_pwm_state old = pwm_state[pin];
	int result;
	pwm_state[pin].ready = 0;
	pwm_state[pin].owned = 1;
	result = old.owned ? drv_pwm_disable((uint8_t)pin) : drv_pwm_init((uint8_t)pin);
	if (!old.owned) {
		/* Prepare the pad before selecting PWM; keep plain GPIO setup unchanged. */
		if (!result)
			result = drv_gpio_set_logic((gpio_pin_t)pin, GPIO_LOGIC_LOW);
		if (!result)
			result = drv_gpio_set_mode((gpio_pin_t)pin, PIN_MODE_GPIO);
		if (!result)
			result = gpio0_test_strong_drive(pin);
		if (!result)
			result = drv_gpio_set_dir((gpio_pin_t)pin, GPIO_DIR_OUT);
	}
	if (!result)
		result = pwm_apply(pin, frequency, duty);
	if (!result) {
		pwm_state[pin].frequency = frequency;
		pwm_state[pin].duty = duty;
		pwm_state[pin].owned = 1;
		pwm_state[pin].ready = 1;
		return 0;
	}
	/* Restore the last owned waveform if possible; otherwise leave it disabled.
		* Never disable the shared PWM clock while another channel may own it. */
	if (old.owned && old.ready && !drv_pwm_disable((uint8_t)pin) &&
		!pwm_apply(pin, old.frequency, old.duty)) {
		pwm_state[pin] = old;
	} else {
		int release = pwm_release(pin);
		if (release)
			SV6X66_Printf("PWM%d cleanup failed (%d); ownership retained\r\n", pin, release);
	}
	return result;
}
static void input(int pin, gpio_pull_t pull) {
	int result = -1;
	int release = pwm_release(pin);
	if (release) {
		report_failure(pin, release);
		return;
	}
	if (usable(pin)) {
		/* Mode resets native input-enable; establish input direction last. */
		result = drv_gpio_set_mode((gpio_pin_t)pin, PIN_MODE_GPIO);
		if (!result)
			result = drv_gpio_set_pull((gpio_pin_t)pin, pull);
		if (!result)
			result = drv_gpio_set_dir((gpio_pin_t)pin, GPIO_DIR_IN);
	}
	report_failure(pin, result);
}
void HAL_PIN_Setup_Input(int pin) {
	input(pin, GPIO_PULL_NONE);
}
void HAL_PIN_Setup_Input_Pullup(int pin) {
	input(pin, GPIO_PULL_UP);
}
void HAL_PIN_Setup_Input_Pulldown(int pin) {
	input(pin, GPIO_PULL_DOWN);
}
void HAL_PIN_Setup_Output_Initial(int pin, int value) {
	int result = -1;
	int release = pwm_release(pin);
	if (release) {
		report_failure(pin, release);
		return;
	}
	if (usable(pin)) {
		result = drv_gpio_set_logic((gpio_pin_t)pin, value ? GPIO_LOGIC_HIGH : GPIO_LOGIC_LOW);
		if (!result) {
			result = drv_gpio_set_mode((gpio_pin_t)pin, PIN_MODE_GPIO);
		}
		if (!result)
			result = gpio0_test_strong_drive(pin);
		if (!result) {
			result = drv_gpio_set_dir((gpio_pin_t)pin, GPIO_DIR_OUT);
		}
	}
	report_failure(pin, result);
}
void HAL_PIN_Setup_Output(int pin) {
	HAL_PIN_Setup_Output_Initial(pin, 0);
}
int HAL_PIN_CanThisPinBePWM(int pin) {
	return pin >= 0 && pin <= 4;
}
void HAL_PIN_PWM_Start(int pin, int frequency) {
	uint32_t bus = sys_bus_clock();
	int result;
	if (!HAL_PIN_CanThisPinBePWM(pin) || (bus != 40000000u && bus != 80000000u) ||
		frequency < 5 || (uint32_t)frequency > bus / 20u) {
		if (HAL_PIN_CanThisPinBePWM(pin))
			pwm_state[pin].ready = 0;
		report_failure(pin, -1);
		return;
	}
	result = pwm_change(pin, (uint32_t)frequency, pwm_state[pin].owned ? pwm_state[pin].duty : 0);
	report_failure(pin, result);
}
void HAL_PIN_PWM_Update(int pin, float value) {
	uint32_t duty;
	int result;
	if (!HAL_PIN_CanThisPinBePWM(pin) || !pwm_state[pin].owned || !pwm_state[pin].ready || value != value) {
		report_failure(pin, -1);
		return;
	}
	if (value < 0)
		value = 0;
	if (value > 100)
		value = 100;
	/* Common role code already inverts PWM_n. Native endpoints include4096. */
	duty = (uint32_t)(value * 4096.0f / 100.0f + 0.5f);
	result = pwm_change(pin, pwm_state[pin].frequency, duty);
	report_failure(pin, result);
}
void HAL_PIN_PWM_Stop(int pin) {
	report_failure(pin, pwm_release(pin));
}
void HAL_AttachInterrupt(int pin, OBKInterruptType mode, OBKInterruptHandler callback) {
	(void)mode;
	(void)callback;
	report_failure(pin, -1);
}
void HAL_DetachInterrupt(int pin) {
	report_failure(pin, -1);
}

#endif
