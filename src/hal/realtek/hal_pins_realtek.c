#ifdef PLATFORM_REALTEK

#include "../../new_common.h"
#include "../../logging/logging.h"
#include "../../new_cfg.h"
#include "../../new_pins.h"
#include "../hal_pins.h"
#include "hal_pinmap_realtek.h"
#if !PLATFORM_REALTEK_NEW
#include "gpio_ex_api.h"
#endif
#if !PLATFORM_RTL8710A && !PLATFORM_RTL8710B
#include "pwmout_ex_api.h"
#endif

#if PLATFORM_REALTEK_NEW

#include "pwmout_ex_api.h"
static int g_active_pwm = 0b0;

// RTL8720E/RTL8721DA currently expose eight application-selectable PWM
// channels. Return a signed result so exhaustion cannot wrap to channel 255.
#define OBK_REALTEK_PWM_CHANNEL_COUNT 8
static int HAL_RTK_GetFreeChannel(void)
{
	for (int channel = 0; channel < OBK_REALTEK_PWM_CHANNEL_COUNT; channel++) {
		if (((g_active_pwm >> channel) & 1) == 0) {
			g_active_pwm |= 1 << channel;
			return channel;
		}
	}
	return -1;
}

static void HAL_RTK_FreeChannel(int channel)
{
	if (channel >= 0 && channel < OBK_REALTEK_PWM_CHANNEL_COUNT)
		g_active_pwm &= ~(1 << channel);
}

#endif

#if PLATFORM_RTL8720D || PLATFORM_REALTEK_NEW
// Realtek-new drives all PWM compare channels from one period timer. RTL8720D
// has separate low-power and high-speed timer banks. IR must not silently
// retime a light or other PWM output sharing its period timer.
static int g_realtek_ir_pwm_pin = -1;

static bool Realtek_SharesPWMPeriodTimer(const int left, const int right)
{
#if PLATFORM_RTL8720D
	const uint32_t leftChannel = pwmout_pin2chan(g_pins[left].pin);
	const uint32_t rightChannel = pwmout_pin2chan(g_pins[right].pin);
	// RTL8720D encodes the high-speed timer bank in channel bit 7.
	return ((leftChannel ^ rightChannel) & 0x80U) == 0;
#else
	(void)left;
	(void)right;
	return true;
#endif
}

static bool Realtek_HasOtherPWMOwner(const int index)
{
	for (int pinIndex = 0; pinIndex < g_numPins; pinIndex++) {
		if (pinIndex != index && g_pins[pinIndex].pwm != NULL &&
			Realtek_SharesPWMPeriodTimer(index, pinIndex)) return true;
	}
	return false;
}
#endif


int PIN_GetPWMIndexForPinIndex(int index)
{
	if(index < 0 || index >= g_numPins)
		return -1;
	rtlPinMapping_t* pin = g_pins + index;
	if(pin->pwm != NULL) return pin->pwm->pwm_idx;
	else return HAL_PIN_CanThisPinBePWM(index);
}

const char* HAL_PIN_GetPinNameAlias(int index)
{
	if(index < 0 || index >= g_numPins)
		return "error";
	return g_pins[index].name;
}

void RTL_GPIO_Init(rtlPinMapping_t* pin)
{
	if(!pin || pin->gpio != NULL || pin->irq != NULL)
	{
		return;
	}
	pin->gpio = os_malloc(sizeof(gpio_t));
	if(pin->gpio == NULL)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "Realtek GPIO allocation failed");
		return;
	}
	memset(pin->gpio, 0, sizeof(gpio_t));
	gpio_init(pin->gpio, pin->pin);
}

void HAL_PIN_SetOutputValue(int index, int iVal)
{
	if(index < 0 || index >= g_numPins)
		return;
	rtlPinMapping_t* pin = g_pins + index;
	if(pin->gpio == NULL)
		return;
	gpio_write(pin->gpio, iVal ? 1 : 0);
}

int HAL_PIN_ReadDigitalInput(int index)
{
	if(index < 0 || index >= g_numPins)
		return 0;
	rtlPinMapping_t* pin = g_pins + index;
	if(pin->gpio == NULL)
		return 0;
	return gpio_read(pin->gpio);
}

void HAL_PIN_Setup_Input_Pullup(int index)
{
	if(index < 0 || index >= g_numPins)
		return;
	rtlPinMapping_t* pin = g_pins + index;
	RTL_GPIO_Init(pin);
	if(pin->gpio == NULL)
		return;
	gpio_dir(pin->gpio, PIN_INPUT);
	gpio_mode(pin->gpio, PullUp);
}

void HAL_PIN_Setup_Input_Pulldown(int index)
{
	if(index < 0 || index >= g_numPins)
		return;
	rtlPinMapping_t* pin = g_pins + index;
	RTL_GPIO_Init(pin);
	if(pin->gpio == NULL)
		return;
	gpio_dir(pin->gpio, PIN_INPUT);
	gpio_mode(pin->gpio, PullDown);
}

void HAL_PIN_Setup_Input(int index)
{
	if(index < 0 || index >= g_numPins)
		return;
	rtlPinMapping_t* pin = g_pins + index;
	RTL_GPIO_Init(pin);
	if(pin->gpio == NULL)
		return;
	gpio_dir(pin->gpio, PIN_INPUT);
	gpio_mode(pin->gpio, PullNone);
}

void HAL_PIN_Setup_Output(int index)
{
	if(index < 0 || index >= g_numPins)
		return;
	rtlPinMapping_t* pin = g_pins + index;
	RTL_GPIO_Init(pin);
	if(pin->gpio == NULL)
		return;
	gpio_dir(pin->gpio, PIN_OUTPUT);
	gpio_mode(pin->gpio, PullUp);
}

void HAL_PIN_PWM_Stop(int index)
{
	if(index < 0 || index >= g_numPins || !HAL_PIN_CanThisPinBePWM(index))
		return;
	rtlPinMapping_t* pin = g_pins + index;
	if(pin->pwm == NULL) return;
#if PLATFORM_REALTEK_NEW
	HAL_RTK_FreeChannel(pin->pwm->pwm_idx);
#endif
	pwmout_free(pin->pwm);
	os_free(pin->pwm);
	pin->pwm = NULL;
#if PLATFORM_RTL8720D || PLATFORM_REALTEK_NEW
	if (g_realtek_ir_pwm_pin == index) g_realtek_ir_pwm_pin = -1;
#endif
}

void HAL_PIN_PWM_Start(int index, int freq)
{
	if(index < 0 || index >= g_numPins || freq <= 0 ||
		!HAL_PIN_CanThisPinBePWM(index))
		return;
#if PLATFORM_RTL8720D || PLATFORM_REALTEK_NEW
	if (g_realtek_ir_pwm_pin >= 0 && g_realtek_ir_pwm_pin != index &&
		Realtek_SharesPWMPeriodTimer(index, g_realtek_ir_pwm_pin)) {
		ADDLOG_ERROR(LOG_FEATURE_DRV,
			"Realtek PWM pin %d rejected while IR owns the shared period timer",
			index);
		return;
	}
#endif
	rtlPinMapping_t* pin = g_pins + index;
	const uint32_t period_us = 1000000U / (uint32_t)freq;
	if(period_us == 0) return;
	if(pin->pwm != NULL)
	{
		pwmout_period_us(pin->pwm, period_us);
		return;
	}
	if(pin->gpio != NULL)
	{
		gpio_deinit(pin->gpio);
		os_free(pin->gpio);
		pin->gpio = NULL;
	}
	pin->pwm = os_malloc(sizeof(pwmout_t));
	if(pin->pwm == NULL)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV,
			"Realtek PWM allocation failed on pin %d", index);
		return;
	}
	memset(pin->pwm, 0, sizeof(pwmout_t));
#if PLATFORM_REALTEK_NEW
	const int channel = HAL_RTK_GetFreeChannel();
	if(channel < 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV,
			"Realtek PWM channels exhausted on pin %d", index);
		os_free(pin->pwm);
		pin->pwm = NULL;
		return;
	}
	pin->pwm->pwm_idx = (uint8_t)channel;
#endif
	pwmout_init(pin->pwm, pin->pin);
	pwmout_period_us(pin->pwm, period_us);
#ifndef PLATFORM_RTL8710A
	pwmout_start(pin->pwm);
#endif
}

void HAL_PIN_PWM_Update(int index, float value)
{
	if(index < 0 || index >= g_numPins || !HAL_PIN_CanThisPinBePWM(index))
		return;
	rtlPinMapping_t* pin = g_pins + index;
#ifdef PLATFORM_RTL87X0C
	if(pin->pwm == NULL || !pin->pwm->is_init) return;
#else
	if(pin->pwm == NULL) return;
#endif
	pwmout_write(pin->pwm, value / 100);
}

bool HAL_IR_PWM_Reserve(int index)
{
	if (index < 0 || index >= g_numPins || !HAL_PIN_CanThisPinBePWM(index))
		return false;
#if PLATFORM_RTL8720D || PLATFORM_REALTEK_NEW
	if (g_realtek_ir_pwm_pin >= 0) return g_realtek_ir_pwm_pin == index;
	if (g_pins[index].pwm != NULL || Realtek_HasOtherPWMOwner(index)) {
		ADDLOG_ERROR(LOG_FEATURE_DRV,
			"IR PWM pin %d cannot claim the Realtek shared period timer while another PWM is active",
			index);
		return false;
	}
	g_realtek_ir_pwm_pin = index;
#endif
	return true;
}

void HAL_IR_PWM_Release(int index)
{
#if PLATFORM_RTL8720D || PLATFORM_REALTEK_NEW
	if (g_realtek_ir_pwm_pin == index) g_realtek_ir_pwm_pin = -1;
#else
	(void)index;
#endif
}

void HAL_IR_PWM_Update(int index, float value)
{
	HAL_PIN_PWM_Update(index, value);
}

bool HAL_IR_PWM_IsActive(int index)
{
	if(index < 0 || index >= g_numPins) return false;
	rtlPinMapping_t* pin = g_pins + index;
	if(pin->pwm == NULL) return false;
#ifdef PLATFORM_RTL87X0C
	return pin->pwm->is_init;
#else
	return true;
#endif
}

unsigned int HAL_GetGPIOPin(int index)
{
	return index;
}

OBKInterruptHandler g_handlers[PLATFORM_GPIO_MAX];
OBKInterruptType g_modes[PLATFORM_GPIO_MAX];

#include "gpio_irq_api.h"

void Realtek_Interrupt(uint32_t obkPinNum, gpio_irq_event event)
{
	(void)event;
	if(obkPinNum < PLATFORM_GPIO_MAX && g_handlers[obkPinNum]) {
		g_handlers[obkPinNum](obkPinNum);
	}
}

void HAL_AttachInterrupt(int pinIndex, OBKInterruptType mode, OBKInterruptHandler function) {
	if(pinIndex < 0 || pinIndex >= g_numPins || function == NULL) return;
	rtlPinMapping_t *rtl_cf = g_pins + pinIndex;
#if PLATFORM_RTL87X0C
	if (rtl_cf->gpio != NULL)
	{
		hal_pinmux_unregister(rtl_cf->pin, PID_GPIO);
		os_free(rtl_cf->gpio);
		rtl_cf->gpio = NULL;
	}
#endif
	if(rtl_cf->irq != NULL)
	{
		gpio_irq_free(rtl_cf->irq);
		os_free(rtl_cf->irq);
		rtl_cf->irq = NULL;
	}
	rtl_cf->irq = os_malloc(sizeof(gpio_irq_t));
	if(rtl_cf->irq == NULL)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV,
			"Realtek IRQ allocation failed on pin %d", pinIndex);
		g_handlers[pinIndex] = NULL;
		return;
	}
	memset(rtl_cf->irq, 0, sizeof(gpio_irq_t));

	int rtl_mode;
	if (mode == INTERRUPT_RISING) {
		rtl_mode = IRQ_RISE;
	}
	else {
		rtl_mode = IRQ_FALL;
	}
	g_handlers[pinIndex] = function;
	gpio_irq_init(rtl_cf->irq, rtl_cf->pin, Realtek_Interrupt, pinIndex);
	gpio_irq_set(rtl_cf->irq, rtl_mode, 1);
	gpio_irq_enable(rtl_cf->irq);
}
void HAL_DetachInterrupt(int pinIndex) {
	if(pinIndex < 0 || pinIndex >= g_numPins || g_handlers[pinIndex] == 0)
		return;
	rtlPinMapping_t *rtl_cf = g_pins + pinIndex;
	if(rtl_cf->irq != NULL)
	{
		gpio_irq_free(rtl_cf->irq);
		os_free(rtl_cf->irq);
		rtl_cf->irq = NULL;
	}
	g_handlers[pinIndex] = 0;
}

#endif // PLATFORM_REALTEK
