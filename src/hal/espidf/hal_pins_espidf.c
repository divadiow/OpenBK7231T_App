#if PLATFORM_ESPIDF || PLATFORM_ESP8266

#include "../../new_common.h"
#include "../../logging/logging.h"
#include "../../new_cfg.h"
#include "../../new_pins.h"
#include "hal_pinmap_espidf.h"
#if PLATFORM_ESPIDF
#include "driver/ledc.h"
#elif PLATFORM_ESP8266
#include "driver/pwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#endif
#include "../hal_pins.h"

#ifdef CONFIG_IDF_TARGET_ESP32C3

espPinMapping_t g_pins[] = {
	{ "IO0", GPIO_NUM_0, false },
	{ "IO1", GPIO_NUM_1, false },
	{ "IO2", GPIO_NUM_2, false },
	{ "IO3", GPIO_NUM_3, false },
	{ "IO4", GPIO_NUM_4, false },
	{ "IO5", GPIO_NUM_5, false },
	{ "IO6", GPIO_NUM_6, false },
	{ "IO7", GPIO_NUM_7, false },
	{ "IO8", GPIO_NUM_8, false },
	{ "IO9", GPIO_NUM_9, false },
	{ "IO10", GPIO_NUM_10, false },
	// SPI flash 11-17
	{ "IO11", GPIO_NUM_11, false },
	{ "IO12", GPIO_NUM_12, false },
	{ "IO13", GPIO_NUM_13, false },
	{ "IO14", GPIO_NUM_14, false },
	{ "IO15", GPIO_NUM_15, false },
	{ "IO16", GPIO_NUM_16, false },
	{ "IO17", GPIO_NUM_17, false },
	{ "IO18", GPIO_NUM_18, false },
	{ "IO19", GPIO_NUM_19, false },
	{ "IO20 (RX)", GPIO_NUM_20, false },
	{ "IO21 (TX)", GPIO_NUM_21, false },
};

#elif CONFIG_IDF_TARGET_ESP32C2

espPinMapping_t g_pins[] = {
	{ "IO0", GPIO_NUM_0, false },
	{ "IO1", GPIO_NUM_1, false },
	{ "IO2", GPIO_NUM_2, false },
	{ "IO3", GPIO_NUM_3, false },
	{ "IO4", GPIO_NUM_4, false },
	{ "IO5", GPIO_NUM_5, false },
	{ "IO6", GPIO_NUM_6, false },
	{ "IO7", GPIO_NUM_7, false },
	{ "IO8", GPIO_NUM_8, false },
	{ "IO9", GPIO_NUM_9, false },
	{ "IO10", GPIO_NUM_10, false },
	{ "IO11", GPIO_NUM_11, false },
	{ "IO12", GPIO_NUM_12, false },
	{ "IO13", GPIO_NUM_13, false },
	{ "IO14", GPIO_NUM_14, false },
	{ "IO15", GPIO_NUM_15, false },
	{ "IO16", GPIO_NUM_16, false },
	{ "IO17", GPIO_NUM_17, false },
	{ "IO18", GPIO_NUM_18, false },
	{ "IO19", GPIO_NUM_19, false },
	{ "IO20", GPIO_NUM_20, false },
};

#elif CONFIG_IDF_TARGET_ESP32C6

espPinMapping_t g_pins[] = {
	{ "IO0", GPIO_NUM_0, false },
	{ "IO1", GPIO_NUM_1, false },
	{ "IO2", GPIO_NUM_2, false },
	{ "IO3", GPIO_NUM_3, false },
	{ "IO4", GPIO_NUM_4, false },
	{ "IO5", GPIO_NUM_5, false },
	{ "IO6", GPIO_NUM_6, false },
	{ "IO7", GPIO_NUM_7, false },
	{ "IO8", GPIO_NUM_8, false },
	{ "IO9", GPIO_NUM_9, false },
	{ "IO10", GPIO_NUM_10, false },
	{ "IO11", GPIO_NUM_11, false },
	{ "IO12", GPIO_NUM_12, false },
	{ "IO13", GPIO_NUM_13, false },
	{ "IO14", GPIO_NUM_14, false },
	{ "IO15", GPIO_NUM_15, false },
	{ "IO16", GPIO_NUM_16, false },
	{ "IO17", GPIO_NUM_17, false },
	{ "IO18", GPIO_NUM_18, false },
	{ "IO19", GPIO_NUM_19, false },
	{ "IO20", GPIO_NUM_20, false },
	{ "IO21", GPIO_NUM_21, false },
	{ "IO22", GPIO_NUM_22, false },
	{ "IO23", GPIO_NUM_23, false },
	{ "IO24", GPIO_NUM_24, false },
	{ "IO25", GPIO_NUM_25, false },
	{ "IO26", GPIO_NUM_26, false },
	{ "IO27", GPIO_NUM_27, false },
	{ "IO28", GPIO_NUM_28, false },
	{ "IO29", GPIO_NUM_29, false },
	{ "IO30", GPIO_NUM_30, false },
};

#elif CONFIG_IDF_TARGET_ESP32S2

espPinMapping_t g_pins[] = {
	{ "IO0", GPIO_NUM_0, false },
	{ "IO1", GPIO_NUM_1, false },
	{ "IO2", GPIO_NUM_2, false },
	{ "IO3", GPIO_NUM_3, false },
	{ "IO4", GPIO_NUM_4, false },
	{ "IO5", GPIO_NUM_5, false },
	{ "IO6", GPIO_NUM_6, false },
	{ "IO7", GPIO_NUM_7, false },
	{ "IO8", GPIO_NUM_8, false },
	{ "IO9", GPIO_NUM_9, false },
	{ "IO10", GPIO_NUM_10, false },
	{ "IO11", GPIO_NUM_11, false },
	{ "IO12", GPIO_NUM_12, false },
	{ "IO13", GPIO_NUM_13, false },
	{ "IO14", GPIO_NUM_14, false },
	{ "IO15", GPIO_NUM_15, false },
	{ "IO16", GPIO_NUM_16, false },
	{ "IO17", GPIO_NUM_17, false },
	{ "IO18", GPIO_NUM_18, false },
	{ "IO19", GPIO_NUM_19, false },
	{ "IO20", GPIO_NUM_20, false },
	{ "IO21", GPIO_NUM_21, false },
	{ "NC", GPIO_NUM_NC, true },
	{ "NC", GPIO_NUM_NC, true },
	{ "NC", GPIO_NUM_NC, true },
	{ "NC", GPIO_NUM_NC, true },
	{ "IO26", GPIO_NUM_26, false },
	{ "IO27", GPIO_NUM_27, false },
	{ "IO28", GPIO_NUM_28, false },
	{ "IO29", GPIO_NUM_29, false },
	{ "IO30", GPIO_NUM_30, false },
	{ "IO31", GPIO_NUM_31, false },
	{ "IO32", GPIO_NUM_32, false },
	{ "IO33", GPIO_NUM_33, false },
	{ "IO34", GPIO_NUM_34, false },
	{ "IO35", GPIO_NUM_35, false },
	{ "IO36", GPIO_NUM_36, false },
	{ "IO37", GPIO_NUM_37, false },
	{ "IO38", GPIO_NUM_38, false },
	{ "IO39", GPIO_NUM_39, false },
	{ "IO40", GPIO_NUM_40, false },
	{ "IO41", GPIO_NUM_41, false },
	{ "IO42", GPIO_NUM_42, false },
	{ "IO43", GPIO_NUM_43, false },
	{ "IO44", GPIO_NUM_44, false },
	{ "IO45", GPIO_NUM_45, false },
	{ "IO46", GPIO_NUM_46, false },
};

#elif CONFIG_IDF_TARGET_ESP32S3

espPinMapping_t g_pins[] = {
	{ "IO0", GPIO_NUM_0, false },
	{ "IO1", GPIO_NUM_1, false },
	{ "IO2", GPIO_NUM_2, false },
	{ "IO3", GPIO_NUM_3, false },
	{ "IO4", GPIO_NUM_4, false },
	{ "IO5", GPIO_NUM_5, false },
	{ "IO6", GPIO_NUM_6, false },
	{ "IO7", GPIO_NUM_7, false },
	{ "IO8", GPIO_NUM_8, false },
	{ "IO9", GPIO_NUM_9, false },
	{ "IO10", GPIO_NUM_10, false },
	{ "IO11", GPIO_NUM_11, false },
	{ "IO12", GPIO_NUM_12, false },
	{ "IO13", GPIO_NUM_13, false },
	{ "IO14", GPIO_NUM_14, false },
	{ "IO15", GPIO_NUM_15, false },
	{ "IO16", GPIO_NUM_16, false },
	{ "IO17", GPIO_NUM_17, false },
	{ "IO18", GPIO_NUM_18, false },
	{ "IO19", GPIO_NUM_19, false },
	{ "IO20", GPIO_NUM_20, false },
	{ "IO21", GPIO_NUM_21, false },
	{ "NC", GPIO_NUM_NC, true },
	{ "NC", GPIO_NUM_NC, true },
	{ "NC", GPIO_NUM_NC, true },
	{ "NC", GPIO_NUM_NC, true },
	{ "IO26", GPIO_NUM_26, false },
	{ "IO27", GPIO_NUM_27, false },
	{ "IO28", GPIO_NUM_28, false },
	{ "IO29", GPIO_NUM_29, false },
	{ "IO30", GPIO_NUM_30, false },
	{ "IO31", GPIO_NUM_31, false },
	{ "IO32", GPIO_NUM_32, false },
	{ "IO33", GPIO_NUM_33, false },
	{ "IO34", GPIO_NUM_34, false },
	{ "IO35", GPIO_NUM_35, false },
	{ "IO36", GPIO_NUM_36, false },
	{ "IO37", GPIO_NUM_37, false },
	{ "IO38", GPIO_NUM_38, false },
	{ "IO39", GPIO_NUM_39, false },
	{ "IO40", GPIO_NUM_40, false },
	{ "IO41", GPIO_NUM_41, false },
	{ "IO42", GPIO_NUM_42, false },
	{ "IO43", GPIO_NUM_43, false },
	{ "IO44", GPIO_NUM_44, false },
	{ "IO45", GPIO_NUM_45, false },
	{ "IO46", GPIO_NUM_46, false },
	{ "IO47", GPIO_NUM_47, false },
	{ "IO48", GPIO_NUM_48, false },
};

#elif CONFIG_IDF_TARGET_ESP32

espPinMapping_t g_pins[] = {
	{ "IO0", GPIO_NUM_0, false },
	{ "IO1", GPIO_NUM_1, false },
	{ "IO2", GPIO_NUM_2, false },
	{ "IO3", GPIO_NUM_3, false },
	{ "IO4", GPIO_NUM_4, false },
	{ "IO5", GPIO_NUM_5, false },
	{ "IO6", GPIO_NUM_6, false },
	{ "IO7", GPIO_NUM_7, false },
	{ "IO8", GPIO_NUM_8, false },
	{ "IO9", GPIO_NUM_9, false },
	{ "IO10", GPIO_NUM_10, false },
	{ "IO11", GPIO_NUM_11, false },
	{ "IO12", GPIO_NUM_12, false },
	{ "IO13", GPIO_NUM_13, false },
	{ "IO14", GPIO_NUM_14, false },
	{ "IO15", GPIO_NUM_15, false },
	{ "IO16", GPIO_NUM_16, false },
	{ "IO17", GPIO_NUM_17, false },
	{ "IO18", GPIO_NUM_18, false },
	{ "IO19", GPIO_NUM_19, false },
	{ "IO20", GPIO_NUM_20, false },
	{ "IO21", GPIO_NUM_21, false },
	{ "IO22", GPIO_NUM_22, false },
	{ "IO23", GPIO_NUM_23, false },
	{ "NC", GPIO_NUM_NC, true },
	{ "IO25", GPIO_NUM_25, false },
	{ "IO26", GPIO_NUM_26, false },
	{ "IO27", GPIO_NUM_27, false },
	{ "IO28", GPIO_NUM_28, false },
	{ "IO29", GPIO_NUM_29, false },
	{ "IO30", GPIO_NUM_30, false },
	{ "IO31", GPIO_NUM_31, false },
	{ "IO32", GPIO_NUM_32, false },
	{ "IO33", GPIO_NUM_33, false },
	{ "IO34", GPIO_NUM_34, false },
	{ "IO35", GPIO_NUM_35, false },
	{ "IO36", GPIO_NUM_36, false },
	{ "IO37", GPIO_NUM_37, false },
	{ "IO38", GPIO_NUM_38, false },
	{ "IO39", GPIO_NUM_39, false },
};

#elif CONFIG_IDF_TARGET_ESP32C5

espPinMapping_t g_pins[] = {
	{ "IO0", GPIO_NUM_0, false },
	{ "IO1", GPIO_NUM_1, false },
	{ "IO2", GPIO_NUM_2, false },
	{ "IO3", GPIO_NUM_3, false },
	{ "IO4", GPIO_NUM_4, false },
	{ "IO5", GPIO_NUM_5, false },
	{ "IO6", GPIO_NUM_6, false },
	{ "IO7", GPIO_NUM_7, false },
	{ "IO8", GPIO_NUM_8, false },
	{ "IO9", GPIO_NUM_9, false },
	{ "IO10", GPIO_NUM_10, false },
	{ "IO11", GPIO_NUM_11, false },
	{ "IO12", GPIO_NUM_12, false },
	{ "IO13", GPIO_NUM_13, false },
	{ "IO14", GPIO_NUM_14, false },
	{ "IO15", GPIO_NUM_15, false },
	{ "IO16", GPIO_NUM_16, false },
	{ "IO17", GPIO_NUM_17, false },
	{ "IO18", GPIO_NUM_18, false },
	{ "IO19", GPIO_NUM_19, false },
	{ "IO20", GPIO_NUM_20, false },
	{ "IO21", GPIO_NUM_21, false },
	{ "IO22", GPIO_NUM_22, false },
	{ "IO23", GPIO_NUM_23, false },
	{ "IO24", GPIO_NUM_24, false },
	{ "IO25", GPIO_NUM_25, false },
	{ "IO26", GPIO_NUM_26, false },
	{ "IO27", GPIO_NUM_27, false },
	{ "IO28", GPIO_NUM_28, false },
};

#elif CONFIG_IDF_TARGET_ESP32C61

espPinMapping_t g_pins[] = {
	{ "IO0", GPIO_NUM_0, false },
	{ "IO1", GPIO_NUM_1, false },
	{ "IO2", GPIO_NUM_2, false },
	{ "IO3", GPIO_NUM_3, false },
	{ "IO4", GPIO_NUM_4, false },
	{ "IO5", GPIO_NUM_5, false },
	{ "IO6", GPIO_NUM_6, false },
	{ "IO7", GPIO_NUM_7, false },
	{ "IO8", GPIO_NUM_8, false },
	{ "IO9", GPIO_NUM_9, false },
	{ "IO10", GPIO_NUM_10, false },
	{ "IO11", GPIO_NUM_11, false },
	{ "IO12", GPIO_NUM_12, false },
	{ "IO13", GPIO_NUM_13, false },
	{ "IO14", GPIO_NUM_14, false },
	{ "IO15", GPIO_NUM_15, false },
	{ "IO16", GPIO_NUM_16, false },
	{ "IO17", GPIO_NUM_17, false },
	{ "IO18", GPIO_NUM_18, false },
	{ "IO19", GPIO_NUM_19, false },
	{ "IO20", GPIO_NUM_20, false },
	{ "IO21", GPIO_NUM_21, false },
};

#elif PLATFORM_ESP8266

espPinMapping_t g_pins[] = {
	{ "IO0", GPIO_NUM_0, false }, // 0
	{ "IO1", GPIO_NUM_1, false }, // 1
	{ "IO2", GPIO_NUM_2, false }, // 2
	{ "IO3", GPIO_NUM_3, false }, // 3
	{ "IO4", GPIO_NUM_4, false }, // 4
	{ "IO5", GPIO_NUM_5, false }, // 5
	{ "IO9", GPIO_NUM_9, false }, // 6
	{ "IO10", GPIO_NUM_10, false }, // 7
	{ "IO12", GPIO_NUM_12, false }, // 8
	{ "IO13", GPIO_NUM_13, false }, // 9
	{ "IO14", GPIO_NUM_14, false }, // 10
	{ "IO15", GPIO_NUM_15, false }, // 11
	{ "IO16", GPIO_NUM_16, false }, // 12
	// ADC only I guess (no GPIO)
	{ "ADC", GPIO_NUM_NC, false }, // 13
};


#else

espPinMapping_t g_pins[] = { };

#endif

#if PLATFORM_ESPIDF
#define LEDC_MAX_CH 6
#endif

int g_numPins = sizeof(g_pins) / sizeof(g_pins[0]);

const char* HAL_PIN_GetPinNameAlias(int index)
{
	if(index >= g_numPins)
		return "error";
	return g_pins[index].name;
}

void HAL_PIN_SetOutputValue(int index, int iVal)
{
	if(index >= g_numPins)
		return;
	espPinMapping_t* pin = g_pins + index;
	if(pin->pin == GPIO_NUM_NC) return;
	gpio_set_level(pin->pin, iVal ? 1 : 0);
}

int HAL_PIN_ReadDigitalInput(int index)
{
	if(index >= g_numPins)
		return 0;
	espPinMapping_t* pin = g_pins + index;
	if(pin->pin == GPIO_NUM_NC) return 0;
	return gpio_get_level(pin->pin);
}

void ESP_ConfigurePin(gpio_num_t pin, gpio_mode_t mode, bool pup, bool pdown, gpio_int_type_t intr)
{
	gpio_config_t conf = {};
	conf.pin_bit_mask = 1ULL << (uint32_t)pin;
	conf.mode = mode;
	conf.pull_up_en = pup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.pull_down_en = pdown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
	conf.intr_type = intr;
	gpio_config(&conf);
}

void HAL_PIN_Setup_Input_Pullup(int index)
{
	if(index >= g_numPins)
		return;
	espPinMapping_t* pin = g_pins + index;
	if(pin->pin == GPIO_NUM_NC) return;
	if(!pin->isConfigured)
	{
		pin->isConfigured = true;
		ESP_ConfigurePin(pin->pin, GPIO_MODE_INPUT, true, false, GPIO_INTR_DISABLE);
		return;
	}
	gpio_set_direction(pin->pin, GPIO_MODE_INPUT);
	gpio_set_pull_mode(pin->pin, GPIO_PULLUP_ONLY);
}

void HAL_PIN_Setup_Input_Pulldown(int index)
{
	if(index >= g_numPins)
		return;
	espPinMapping_t* pin = g_pins + index;
	if(pin->pin == GPIO_NUM_NC) return;
	if(!pin->isConfigured)
	{
		pin->isConfigured = true;
		ESP_ConfigurePin(pin->pin, GPIO_MODE_INPUT, false, true, GPIO_INTR_DISABLE);
		return;
	}
	gpio_set_direction(pin->pin, GPIO_MODE_INPUT);
	gpio_set_pull_mode(pin->pin, GPIO_PULLDOWN_ONLY);
}

void HAL_PIN_Setup_Input(int index)
{
	if(index >= g_numPins)
		return;
	espPinMapping_t* pin = g_pins + index;
	if(pin->pin == GPIO_NUM_NC) return;
	if(!pin->isConfigured)
	{
		pin->isConfigured = true;
		ESP_ConfigurePin(pin->pin, GPIO_MODE_INPUT, false, false, GPIO_INTR_DISABLE);
		return;
	}
	gpio_set_direction(pin->pin, GPIO_MODE_INPUT);
	gpio_set_pull_mode(pin->pin, GPIO_FLOATING);
}

void HAL_PIN_Setup_Output(int index)
{
	if(index >= g_numPins)
		return;
	espPinMapping_t* pin = g_pins + index;
	if(pin->pin == GPIO_NUM_NC) return;
	if(!pin->isConfigured)
	{
		pin->isConfigured = true;
		ESP_ConfigurePin(pin->pin, GPIO_MODE_OUTPUT, true, false, GPIO_INTR_DISABLE);
		return;
	}
	gpio_set_direction(pin->pin, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(pin->pin, GPIO_PULLUP_ONLY);
	gpio_set_level(pin->pin, 0);
}

#if PLATFORM_ESP8266

// ESP8266 has no independent LEDC channels. Its RTOS SDK LEDC layer is a
// compatibility wrapper around one grouped software-PWM engine. Keep the
// grouped topology and commit coalescing entirely inside this HAL so generic
// OpenBeken pin and LED code can retain the normal per-pin PWM API.

#define ESP8266_PWM_MAX_CH 8
#define ESP8266_PWM_MIN_FREQ_HZ 100
#define ESP8266_PWM_MAX_FREQ_HZ 20000
#define ESP8266_PWM_DEFAULT_FREQ_HZ 1000

static int esp8266_pwm_pin_index[ESP8266_PWM_MAX_CH];
static uint32_t esp8266_pwm_gpio[ESP8266_PWM_MAX_CH];
static float esp8266_pwm_value[ESP8266_PWM_MAX_CH];
static uint32_t esp8266_pwm_duty[ESP8266_PWM_MAX_CH];
static int esp8266_pwm_count = 0;
static uint32_t esp8266_pwm_period_us = 1000;
static bool esp8266_pwm_driver_initialized = false;
static bool esp8266_pwm_started = false;
static bool esp8266_pwm_topology_dirty = false;
static bool esp8266_pwm_duty_dirty = false;
static SemaphoreHandle_t esp8266_pwm_lock = NULL;
static TimerHandle_t esp8266_pwm_commit_timer = NULL;

static void ESP8266_PWM_CommitTimerCallback(TimerHandle_t timer);

static int ESP8266_GetPWMChannelForPinIndex(int index)
{
	for(int i = 0; i < esp8266_pwm_count; i++)
	{
		if(esp8266_pwm_pin_index[i] == index)
		{
			return i;
		}
	}
	return -1;
}

static int ESP8266_GetPWMChannelForGPIO(gpio_num_t gpio)
{
	for(int i = 0; i < esp8266_pwm_count; i++)
	{
		if(esp8266_pwm_gpio[i] == (uint32_t)gpio)
		{
			return i;
		}
	}
	return -1;
}

static uint32_t ESP8266_PWM_FrequencyToPeriodUS(int freq)
{
	if(freq < ESP8266_PWM_MIN_FREQ_HZ)
	{
		freq = ESP8266_PWM_MIN_FREQ_HZ;
	}
	else if(freq > ESP8266_PWM_MAX_FREQ_HZ)
	{
		freq = ESP8266_PWM_MAX_FREQ_HZ;
	}
	return 1000000UL / (uint32_t)freq;
}

static uint32_t ESP8266_PWMValueToDuty(float value)
{
	if(value <= 0.0f)
	{
		return 0;
	}
	if(value >= 100.0f)
	{
		return esp8266_pwm_period_us;
	}
	return (uint32_t)((value * (float)esp8266_pwm_period_us / 100.0f) + 0.5f);
}

static void ESP8266_PWM_RecalculateDutiesLocked(void)
{
	for(int i = 0; i < esp8266_pwm_count; i++)
	{
		esp8266_pwm_duty[i] = ESP8266_PWMValueToDuty(esp8266_pwm_value[i]);
	}
}

static bool ESP8266_PWM_EnsureCommitObjects(void)
{
	if(esp8266_pwm_lock == NULL)
	{
		esp8266_pwm_lock = xSemaphoreCreateMutex();
		if(esp8266_pwm_lock == NULL)
		{
			ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM mutex allocation failed");
			return false;
		}
	}

	if(esp8266_pwm_commit_timer == NULL)
	{
		esp8266_pwm_commit_timer = xTimerCreate("obkPWM", 1, pdFALSE, NULL, ESP8266_PWM_CommitTimerCallback);
		if(esp8266_pwm_commit_timer == NULL)
		{
			ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM timer allocation failed");
			return false;
		}
	}
	return true;
}

static TickType_t ESP8266_PWM_GetCommitDelayTicks(void)
{
	uint32_t period_ms = (esp8266_pwm_period_us + 999) / 1000;
	TickType_t ticks = (TickType_t)(period_ms / portTICK_PERIOD_MS) + 1;
	if(ticks < 1)
	{
		ticks = 1;
	}
	return ticks;
}

static bool ESP8266_PWM_ScheduleCommit(TickType_t wait_ticks)
{
	if(esp8266_pwm_commit_timer == NULL)
	{
		return false;
	}
	return xTimerChangePeriod(esp8266_pwm_commit_timer, ESP8266_PWM_GetCommitDelayTicks(), wait_ticks) == pdPASS;
}

static void ESP8266_PWM_DeinitLocked(void)
{
	if(esp8266_pwm_driver_initialized)
	{
		pwm_deinit();
		// pwm_deinit() calls pwm_stop(0xFF), which leaves every channel high.
		// Restore an inactive-low state before a topology rebuild or role handoff.
		for(int i = 0; i < esp8266_pwm_count; i++)
		{
			gpio_set_level((gpio_num_t)esp8266_pwm_gpio[i], 0);
		}
	}
	esp8266_pwm_driver_initialized = false;
	esp8266_pwm_started = false;
}

static bool ESP8266_PWM_RebuildLocked(void)
{
	ESP8266_PWM_DeinitLocked();

	if(esp8266_pwm_count <= 0)
	{
		return true;
	}

	ESP8266_PWM_RecalculateDutiesLocked();

	esp_err_t err = pwm_init(esp8266_pwm_period_us, esp8266_pwm_duty,
		(uint8_t)esp8266_pwm_count, esp8266_pwm_gpio);
	if(err != ESP_OK)
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM init failed: %i", err);
		return false;
	}
	esp8266_pwm_driver_initialized = true;

	float phase[ESP8266_PWM_MAX_CH];
	for(int i = 0; i < esp8266_pwm_count; i++)
	{
		phase[i] = 0.0f;
	}
	err = pwm_set_phases(phase);
	if(err != ESP_OK)
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM phase init failed: %i", err);
		ESP8266_PWM_DeinitLocked();
		return false;
	}

	err = pwm_start();
	if(err != ESP_OK)
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM start failed: %i", err);
		ESP8266_PWM_DeinitLocked();
		return false;
	}

	esp8266_pwm_started = true;
	ADDLOG_INFO(LOG_FEATURE_PINS, "ESP8266 PWM started with %i channels, period %u us",
		esp8266_pwm_count, (unsigned int)esp8266_pwm_period_us);
	return true;
}

static bool ESP8266_PWM_ApplyDutiesLocked(void)
{
	if(!esp8266_pwm_started)
	{
		return false;
	}

	ESP8266_PWM_RecalculateDutiesLocked();
	esp_err_t err = pwm_set_duties(esp8266_pwm_duty);
	if(err != ESP_OK)
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM set duties failed: %i", err);
		return false;
	}
	err = pwm_start();
	if(err != ESP_OK)
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM restart failed: %i", err);
		return false;
	}
	return true;
}

static void ESP8266_PWM_CommitTimerCallback(TimerHandle_t timer)
{
	(void)timer;

	if(esp8266_pwm_lock == NULL)
	{
		return;
	}

	// Timer callbacks run in the shared FreeRTOS timer task: never block it on
	// the PWM mutex. If a normal HAL call owns the state, try again next period.
	if(xSemaphoreTake(esp8266_pwm_lock, 0) != pdTRUE)
	{
		ESP8266_PWM_ScheduleCommit(0);
		return;
	}

	if(esp8266_pwm_topology_dirty)
	{
		if(ESP8266_PWM_RebuildLocked())
		{
			esp8266_pwm_topology_dirty = false;
			esp8266_pwm_duty_dirty = false;
		}
	}
	else if(esp8266_pwm_duty_dirty)
	{
		if(ESP8266_PWM_ApplyDutiesLocked())
		{
			esp8266_pwm_duty_dirty = false;
		}
	}

	xSemaphoreGive(esp8266_pwm_lock);
}

int PIN_GetPWMIndexForPinIndex(int index)
{
	if(index < 0 || index >= g_numPins)
	{
		return -1;
	}
	return ESP8266_GetPWMChannelForPinIndex(index);
}

int HAL_PIN_CanThisPinBePWM(int index)
{
	if(index < 0 || index >= g_numPins)
	{
		return 0;
	}
	espPinMapping_t* pin = g_pins + index;
	// GPIO0 can be used after boot, but assigning PWM there can leave the module
	// in a boot loop after reset.
	if(pin->pin == GPIO_NUM_0)
	{
		return 0;
	}
	return pin->pin != GPIO_NUM_NC;
}

void HAL_PIN_PWM_Stop(int index)
{
	if(index < 0 || index >= g_numPins)
	{
		return;
	}

	espPinMapping_t* pin = g_pins + index;
	int ch = ESP8266_GetPWMChannelForPinIndex(index);
	if(ch < 0)
	{
		return;
	}

	if(!ESP8266_PWM_EnsureCommitObjects())
	{
		return;
	}
	if(xSemaphoreTake(esp8266_pwm_lock, portMAX_DELAY) != pdTRUE)
	{
		return;
	}

	ch = ESP8266_GetPWMChannelForPinIndex(index);
	if(ch >= 0)
	{
		// The native driver still owns every pin in the old group. Detach it
		// synchronously before returning so generic role setup can safely reuse
		// this GPIO; the remaining group is rebuilt once by the deferred commit.
		ESP8266_PWM_DeinitLocked();

		for(int i = ch; i < esp8266_pwm_count - 1; i++)
		{
			esp8266_pwm_pin_index[i] = esp8266_pwm_pin_index[i + 1];
			esp8266_pwm_gpio[i] = esp8266_pwm_gpio[i + 1];
			esp8266_pwm_value[i] = esp8266_pwm_value[i + 1];
			esp8266_pwm_duty[i] = esp8266_pwm_duty[i + 1];
		}
		esp8266_pwm_count--;
		esp8266_pwm_topology_dirty = true;
		esp8266_pwm_duty_dirty = false;
	}
	pin->isConfigured = false;
	xSemaphoreGive(esp8266_pwm_lock);

	if(pin->pin != GPIO_NUM_NC)
	{
		gpio_set_level(pin->pin, 0);
	}
	if(!ESP8266_PWM_ScheduleCommit(portMAX_DELAY))
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM failed to schedule topology update");
	}
}

void HAL_PIN_PWM_Start(int index, int freq)
{
	if(index < 0 || index >= g_numPins)
	{
		return;
	}

	espPinMapping_t* pin = g_pins + index;
	if(pin->pin == GPIO_NUM_NC)
	{
		return;
	}
	if(pin->pin == GPIO_NUM_0)
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM rejected on GPIO0");
		return;
	}

	if(!ESP8266_PWM_EnsureCommitObjects())
	{
		return;
	}

	uint32_t new_period = ESP8266_PWM_FrequencyToPeriodUS(freq > 0 ? freq : ESP8266_PWM_DEFAULT_FREQ_HZ);
	bool needsCommit = false;

	if(xSemaphoreTake(esp8266_pwm_lock, portMAX_DELAY) != pdTRUE)
	{
		return;
	}

	if(new_period != esp8266_pwm_period_us)
	{
		esp8266_pwm_period_us = new_period;
		esp8266_pwm_topology_dirty = true;
	}

	int ch = ESP8266_GetPWMChannelForGPIO(pin->pin);
	if(ch < 0)
	{
		if(esp8266_pwm_count >= ESP8266_PWM_MAX_CH)
		{
			xSemaphoreGive(esp8266_pwm_lock);
			ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM_Start: no free channels for pin %i", pin->pin);
			return;
		}

		ch = esp8266_pwm_count++;
		esp8266_pwm_pin_index[ch] = index;
		esp8266_pwm_gpio[ch] = (uint32_t)pin->pin;
		esp8266_pwm_value[ch] = 0.0f;
		esp8266_pwm_duty[ch] = 0;
		esp8266_pwm_topology_dirty = true;
		esp8266_pwm_duty_dirty = false;
		ADDLOG_INFO(LOG_FEATURE_PINS, "ESP8266 PWM queued ch %i pin %i", ch, pin->pin);
	}
	else
	{
		esp8266_pwm_pin_index[ch] = index;
	}

	pin->isConfigured = true;
	needsCommit = esp8266_pwm_topology_dirty || esp8266_pwm_duty_dirty;
	xSemaphoreGive(esp8266_pwm_lock);

	ESP_ConfigurePin(pin->pin, GPIO_MODE_OUTPUT, false, false, GPIO_INTR_DISABLE);
	if(needsCommit && !ESP8266_PWM_ScheduleCommit(portMAX_DELAY))
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM failed to schedule start/rebuild");
	}
}

void HAL_PIN_PWM_Update(int index, float value)
{
	if(index < 0 || index >= g_numPins)
	{
		return;
	}
	if(!ESP8266_PWM_EnsureCommitObjects())
	{
		return;
	}

	if(value < 0.0f)
	{
		value = 0.0f;
	}
	else if(value > 100.0f)
	{
		value = 100.0f;
	}

	bool needsCommit = false;
	if(xSemaphoreTake(esp8266_pwm_lock, portMAX_DELAY) != pdTRUE)
	{
		return;
	}

	int ch = ESP8266_GetPWMChannelForPinIndex(index);
	if(ch >= 0)
	{
		if(esp8266_pwm_value[ch] != value)
		{
			esp8266_pwm_value[ch] = value;
			esp8266_pwm_duty_dirty = true;
		}
		needsCommit = esp8266_pwm_topology_dirty || esp8266_pwm_duty_dirty;
	}
	xSemaphoreGive(esp8266_pwm_lock);

	if(needsCommit && !ESP8266_PWM_ScheduleCommit(portMAX_DELAY))
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "ESP8266 PWM failed to schedule duty update");
	}
}

#elif PLATFORM_ESPIDF

static ledc_channel_config_t ledc_channel[LEDC_MAX_CH];
static float obk_ch_value[LEDC_MAX_CH];
static bool g_ledc_init = false;

void InitLEDC()
{
	if(!g_ledc_init)
	{
		ledc_timer_config_t ledc_timer =
		{
			.duty_resolution = LEDC_TIMER_13_BIT,
			.freq_hz = 1000,
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.timer_num = LEDC_TIMER_0,
			.clk_cfg = SOC_MOD_CLK_RC_FAST,
		};
		ledc_timer_config(&ledc_timer);
		for(int i = 0; i < LEDC_MAX_CH; i++)
		{
			ledc_channel[i].channel = i;
			ledc_channel[i].duty = 0;
			ledc_channel[i].gpio_num = GPIO_NUM_NC;
			ledc_channel[i].speed_mode = LEDC_LOW_SPEED_MODE;
			ledc_channel[i].hpoint = 0;
			ledc_channel[i].timer_sel = LEDC_TIMER_0;
			ledc_channel[i].intr_type = LEDC_INTR_DISABLE;
		}
		g_ledc_init = true;
	}
}

int GetAvailableLedcChannel()
{
	for(int i = 0; i < LEDC_MAX_CH; i++)
	{
		if(ledc_channel[i].gpio_num == GPIO_NUM_NC)
		{
			return ledc_channel[i].channel;
		}
	}
	return -1;
}

int GetLedcChannelForPin(gpio_num_t pin)
{
	for(int i = 0; i < LEDC_MAX_CH; i++)
	{
		if(ledc_channel[i].gpio_num == pin)
		{
			return ledc_channel[i].channel;
		}
	}
	return -1;
}

int PIN_GetPWMIndexForPinIndex(int index)
{
	if(index >= g_numPins)
		return -1;
	espPinMapping_t* pin = g_pins + index;
	int ch = GetLedcChannelForPin(pin->pin);
	if(ch >= 0)
	{
		return ch;
	}
	return -1;
}

int HAL_PIN_CanThisPinBePWM(int index)
{
	if(index >= g_numPins)
		return 0;
	espPinMapping_t* pin = g_pins + index;
	if(pin->pin != GPIO_NUM_NC) return 1;
	else return 0;
}

void HAL_PIN_PWM_Stop(int index)
{
	if(index >= g_numPins)
		return;
	espPinMapping_t* pin = g_pins + index;
	int ch = GetLedcChannelForPin(pin->pin);
	if(ch >= 0)
	{
		ledc_stop(LEDC_LOW_SPEED_MODE, ch, 0);
		gpio_reset_pin(pin->pin);
		pin->isConfigured = false;
		ledc_channel[ch].gpio_num = GPIO_NUM_NC;
	}
}

void HAL_PIN_PWM_Start(int index, int freq)
{
	if(index >= g_numPins)
		return;
	InitLEDC();
	espPinMapping_t* pin = g_pins + index;
	int freecha = GetAvailableLedcChannel();
	if(freecha >= 0)
	{
		ledc_channel[freecha].gpio_num = pin->pin;
		ledc_channel_config(&ledc_channel[freecha]);
		ADDLOG_INFO(LOG_FEATURE_PINS, "init ledc ch %i pin %i", freecha, pin->pin);
	}
	else
	{
		ADDLOG_ERROR(LOG_FEATURE_PINS, "PWM_Start: no free ledc channels for pin %i", pin->pin);
	}
}

void HAL_PIN_PWM_Update(int index, float value)
{
	if(index >= g_numPins)
		return;
	espPinMapping_t* pin = g_pins + index;
	int ch = GetLedcChannelForPin(pin->pin);
	if(ch >= 0)
	{
		uint32_t propduty = value * 81.91;
		if(value != obk_ch_value[ch]) 
		{ 
			obk_ch_value[ch] = value;
			ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, propduty);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
			if(value == 100.0f)
			{
				ledc_stop(LEDC_LOW_SPEED_MODE, ch, 1);
			}
			else if(value <= 0.01f)
			{
				ledc_stop(LEDC_LOW_SPEED_MODE, ch, 0);
			}
		}
	}
}

#endif

unsigned int HAL_GetGPIOPin(int index)
{
	return index;
}


OBKInterruptHandler g_handlers[PLATFORM_GPIO_MAX];
OBKInterruptType g_modes[PLATFORM_GPIO_MAX];

void ESP_Interrupt(void* context) {
	int obkPinNum = (int)context;
	if (g_handlers[obkPinNum]) {
		g_handlers[obkPinNum](obkPinNum);
	}
}

bool b_esp_ready = false;
void HAL_AttachInterrupt(int pinIndex, OBKInterruptType mode, OBKInterruptHandler function) {
	g_handlers[pinIndex] = function;

	if (b_esp_ready == false) {
		gpio_install_isr_service(0);
		b_esp_ready = true;
	}
	espPinMapping_t* esp_cf = g_pins + pinIndex;
	int esp_mode;
	if (mode == INTERRUPT_RISING) {
		esp_mode = GPIO_INTR_POSEDGE;
	}
	else if (mode == INTERRUPT_FALLING) {
		esp_mode = GPIO_INTR_NEGEDGE;
	}
	else {
		esp_mode = GPIO_INTR_ANYEDGE;
	}
	ESP_ConfigurePin(esp_cf->pin, GPIO_MODE_INPUT, true, false, esp_mode);
	gpio_isr_handler_add(esp_cf->pin, ESP_Interrupt, (void*)pinIndex);
}
void HAL_DetachInterrupt(int pinIndex) {
	if (g_handlers[pinIndex] == 0) {
		return; // already removed;
	}

	espPinMapping_t* esp_cf;
	esp_cf = g_pins + pinIndex;
	gpio_isr_handler_remove(esp_cf->pin);
	///gpio_uninstall_isr_service();
	g_handlers[pinIndex] = 0;
}

#endif // PLATFORM_ESPIDF
