#ifdef PLATFORM_BL602

#include "../../new_pins.h"
#include "../../new_common.h"
#include "../../logging/logging.h"
#include "../hal_pins.h"


#include "bl_gpio.h"
#include <bl_pwm.h>
#include <bl602_pwm.h>

// BL602 GPIOs are physically routed to one of five PWM channels in a repeating
// pattern. Track the owning pin so a second PWM user cannot silently reconfigure
// a channel that is already active on another pin.
static int8_t g_bl602_pwm_owner[5] = { -1, -1, -1, -1, -1 };

int BL_FindPWMForPin(int index){
	return index % 5;
}


void HAL_PIN_SetOutputValue(int index, int iVal) {
    bl_gpio_output_set(index, iVal ? 1 : 0);
}

const char *HAL_PIN_GetPinNameAlias(int index) {
	return 0;
}
// BL602 - any pin can be pwm
int HAL_PIN_CanThisPinBePWM(int index) {
	return 1;
}
int HAL_PIN_ReadDigitalInput(int index) {
	uint8_t iVal;
    bl_gpio_input_get(index, &iVal);
	return iVal;
}
void HAL_PIN_Setup_Input_Pulldown(int index) {
	bl_gpio_enable_input(index, 0, 1);
}
void HAL_PIN_Setup_Input_Pullup(int index) {
	// int bl_gpio_enable_input(uint8_t pin, uint8_t pullup, uint8_t pulldown);
	bl_gpio_enable_input(index, 1, 0);
}
void HAL_PIN_Setup_Input(int index) {
	// int bl_gpio_enable_input(uint8_t pin, uint8_t pullup, uint8_t pulldown);
	bl_gpio_enable_input(index, 0, 0);
}
void HAL_PIN_Setup_Output(int index) {
	bl_gpio_enable_output(index, 1,0);
	bl_gpio_output_set(index, 0);
}

void HAL_PIN_PWM_Stop(int index) {
	const int pwm = BL_FindPWMForPin(index);
	if (pwm < 0 || pwm >= 5 || g_bl602_pwm_owner[pwm] != index) return;
	PWM_SW_Mode((PWM_CH_ID_Type)pwm, DISABLE);
	bl_pwm_stop((uint8_t)pwm);
	g_bl602_pwm_owner[pwm] = -1;
}

void HAL_PIN_PWM_Start(int index, int freq) {
	const int pwm = BL_FindPWMForPin(index);
	if (pwm < 0 || pwm >= 5) return;
	if (g_bl602_pwm_owner[pwm] >= 0 &&
		g_bl602_pwm_owner[pwm] != index) {
		ADDLOG_ERROR(LOG_FEATURE_DRV,
			"BL602 PWM channel %d already belongs to pin %d; pin %d rejected",
			pwm, g_bl602_pwm_owner[pwm], index);
		return;
	}

	//addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,"HAL_PIN_PWM_Start: pin %i chose pwm %i",index,pwm);
    //  Frequency must be between 2000 and 800000
	if(freq < 2000) freq = 2000;
	// IR uses hardware software-force mode for exact static space levels. Clear
	// it before every ordinary/restarted PWM setup so state cannot leak between
	// owners or between consecutive IR transmissions.
	PWM_SW_Mode((PWM_CH_ID_Type)pwm, DISABLE);
	if (bl_pwm_init((uint8_t)pwm, (uint8_t)index, (uint32_t)freq) != 0 ||
		bl_pwm_start((uint8_t)pwm) != 0) {
		bl_pwm_stop((uint8_t)pwm);
		g_bl602_pwm_owner[pwm] = -1;
		ADDLOG_ERROR(LOG_FEATURE_DRV,
			"BL602 PWM start failed on pin %d channel %d", index, pwm);
		return;
	}
	g_bl602_pwm_owner[pwm] = (int8_t)index;
}

void HAL_PIN_PWM_Update(int index, float value) {
	const int pwm = BL_FindPWMForPin(index);
	if (pwm < 0 || pwm >= 5 || g_bl602_pwm_owner[pwm] != index) return;
	if(value < 0) value = 0;
	if(value > 100) value = 100;
	/* Ordinary PWM updates must leave any prior IR force mode. */
	PWM_SW_Mode((PWM_CH_ID_Type)pwm, DISABLE);
	bl_pwm_set_duty((uint8_t)pwm, value);
}

void HAL_IR_PWM_Update(int index, float value) {
	const int pwm = BL_FindPWMForPin(index);
	if (pwm < 0 || pwm >= 5 || g_bl602_pwm_owner[pwm] != index) return;
	const PWM_CH_ID_Type channel = (PWM_CH_ID_Type)pwm;
	if (value <= 0.0f) {
		PWM_SW_Force_Value(channel, 0);
		PWM_SW_Mode(channel, ENABLE);
		return;
	}
	if (value >= 100.0f) {
		PWM_SW_Force_Value(channel, 1);
		PWM_SW_Mode(channel, ENABLE);
		return;
	}
	PWM_SW_Mode(channel, DISABLE);
	bl_pwm_set_duty((uint8_t)pwm, value);
}

bool HAL_IR_PWM_IsActive(int index) {
	const int pwm = BL_FindPWMForPin(index);
	return pwm >= 0 && pwm < 5 && g_bl602_pwm_owner[pwm] == index;
}

unsigned int HAL_GetGPIOPin(int index) {
	return index;
}

OBKInterruptHandler g_handlers[PLATFORM_GPIO_MAX];
OBKInterruptType g_modes[PLATFORM_GPIO_MAX];

#include "hal_gpio.h"

void BL602_Interrupt(gpio_ctx_t* context) {
	int obkPinNum = (int)context->arg;
	if (g_handlers[obkPinNum]) {
		g_handlers[obkPinNum](obkPinNum);
	}
	bl_gpio_intmask(obkPinNum, 0);
}

void HAL_AttachInterrupt(int pinIndex, OBKInterruptType mode, OBKInterruptHandler function) {
	g_handlers[pinIndex] = function;
	int bl_mode;
	switch(mode)
	{
		case INTERRUPT_RISING: bl_mode = GPIO_INT_TRIG_POS_PULSE; break;
		case INTERRUPT_FALLING: bl_mode = GPIO_INT_TRIG_NEG_PULSE; break;
		default: bl_mode = GPIO_INT_TRIG_NEG_PULSE; break;
	}
	hal_gpio_register_handler(BL602_Interrupt, pinIndex,
		GPIO_INT_CONTROL_ASYNC, bl_mode, (void*)pinIndex);
}
void HAL_DetachInterrupt(int pinIndex) {
	if (g_handlers[pinIndex] == 0) {
		return; // already removed;
	}
	g_handlers[pinIndex] = 0;
}

#endif
