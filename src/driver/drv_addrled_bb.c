#include "../new_cfg.h"
#include "../new_common.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "../hal/hal_generic.h"
#include "../hal/hal_pins.h"
#include "../logging/logging.h"

#include "drv_leds_shared.h"
#include "drv_local.h"

#if ENABLE_DRIVER_ADDRLED_BB && PLATFORM_BK7231N && !PLATFORM_BEKEN_NEW

#include "include.h"
#include "arm_arch.h"
#include "gpio_pub.h"
#include "gpio.h"
#include "drv_model_pub.h"
#include "sys_ctrl_pub.h"
#include "uart_pub.h"
#include "intc_pub.h"
#include "icu_pub.h"

#define ADDRLED_BB_DEFAULT_RESET_US      150
#define ADDRLED_BB_MAX_PIXELS            20
#define ADDRLED_BB_FAST_HIGH             0x02
#define ADDRLED_BB_FAST_LOW              0x00
#define ADDRLED_BB_ALWAYS_INLINE static inline __attribute__((always_inline))
#ifndef ADDRLED_BB_USE_ITCM
#define ADDRLED_BB_USE_ITCM 1
#endif
#if ADDRLED_BB_USE_ITCM
#define ADDRLED_BB_RAMFUNC __attribute__((section(".itcm"), noinline))
#else
#define ADDRLED_BB_RAMFUNC __attribute__((noinline))
#endif


/*
 * Fixed TX profiles.
 * Values are GCC assembler NOP repeat counts used inside the critical section.
 * The GPIO writes themselves also consume time; these are profile numbers, not ns/us.
 */
#define ADDRLED_BB_P0_T0H  0
#define ADDRLED_BB_P0_T1H  1
#define ADDRLED_BB_P0_T0L  1
#define ADDRLED_BB_P0_T1L  0

#define ADDRLED_BB_P1_T0H  1
#define ADDRLED_BB_P1_T1H  2
#define ADDRLED_BB_P1_T0L  2
#define ADDRLED_BB_P1_T1L  1

#define ADDRLED_BB_P2_T0H  1
#define ADDRLED_BB_P2_T1H  3
#define ADDRLED_BB_P2_T0L  3
#define ADDRLED_BB_P2_T1L  1

#define ADDRLED_BB_P3_T0H  2
#define ADDRLED_BB_P3_T1H  5
#define ADDRLED_BB_P3_T0L  5
#define ADDRLED_BB_P3_T1L  2

#define ADDRLED_BB_P4_T0H  3
#define ADDRLED_BB_P4_T1H  8
#define ADDRLED_BB_P4_T0L  8
#define ADDRLED_BB_P4_T1L  3

#define ADDRLED_BB_P5_T0H  5
#define ADDRLED_BB_P5_T1H  12
#define ADDRLED_BB_P5_T0L  12
#define ADDRLED_BB_P5_T1L  5

#define ADDRLED_BB_P6_T0H  8
#define ADDRLED_BB_P6_T1H  18
#define ADDRLED_BB_P6_T0L  18
#define ADDRLED_BB_P6_T1L  8


typedef enum addrledBBProtocol_e {
	ADDRLED_BB_PROTOCOL_WS2812B = 0,
} addrledBBProtocol_t;

typedef enum addrledBBGPIOMode_e {
	ADDRLED_BB_GPIO_RAW = 0,
	ADDRLED_BB_GPIO_PRESERVE = 1,
} addrledBBGPIOMode_t;

extern commandResult_t Strip_CMD_InitForLEDCount(const void *context, const char *cmd, const char *args, int flags);
extern commandResult_t Strip_CMD_setPixel(const void *context, const char *cmd, const char *args, int flags);
extern void Strip_Apply(void);

static byte *g_addrled_bb_buf;
static uint32_t g_addrled_bb_len;
static int g_addrled_bb_pin = -1;
static volatile unsigned int *g_addrled_bb_gpio_cfg;
static volatile unsigned int g_addrled_bb_reg_high = ADDRLED_BB_FAST_HIGH;
static volatile unsigned int g_addrled_bb_reg_low = ADDRLED_BB_FAST_LOW;
static int g_addrled_bb_reset_us = ADDRLED_BB_DEFAULT_RESET_US;
static int g_addrled_bb_profile = 2;
static int g_addrled_bb_custom_t0h = ADDRLED_BB_P2_T0H;
static int g_addrled_bb_custom_t1h = ADDRLED_BB_P2_T1H;
static int g_addrled_bb_custom_t0l = ADDRLED_BB_P2_T0L;
static int g_addrled_bb_custom_t1l = ADDRLED_BB_P2_T1L;
static addrledBBProtocol_t g_addrled_bb_protocol = ADDRLED_BB_PROTOCOL_WS2812B;
static addrledBBGPIOMode_t g_addrled_bb_gpio_mode = ADDRLED_BB_GPIO_RAW;

ADDRLED_BB_ALWAYS_INLINE void AddrLED_BB_NopDelayVariable(int nops) {
	while (nops-- > 0) {
		__asm__ __volatile__("nop");
	}
}

static void AddrLED_BB_UpdateGPIOValues(void) {
	if (!g_addrled_bb_gpio_cfg) {
		return;
	}
	if (g_addrled_bb_gpio_mode == ADDRLED_BB_GPIO_PRESERVE) {
		unsigned int reg_val = REG_READ(g_addrled_bb_gpio_cfg);
		g_addrled_bb_reg_high = (reg_val & ~GCFG_OUTPUT_BIT) | ((0x01 & 0x01) << GCFG_OUTPUT_POS);
		g_addrled_bb_reg_low = (reg_val & ~GCFG_OUTPUT_BIT) | ((0x00 & 0x01) << GCFG_OUTPUT_POS);
	}
	else {
		g_addrled_bb_reg_high = ADDRLED_BB_FAST_HIGH;
		g_addrled_bb_reg_low = ADDRLED_BB_FAST_LOW;
	}
}

static void AddrLED_BB_PrepareFastGPIO(int pin) {
	int id = pin;
#if (CFG_SOC_NAME != SOC_BK7231)
	if (id >= GPIO32) {
		id += 16;
	}
#endif
	g_addrled_bb_gpio_cfg = (volatile unsigned int *)(REG_GPIO_CFG_BASE_ADDR + id * 4);
	AddrLED_BB_UpdateGPIOValues();
}

ADDRLED_BB_ALWAYS_INLINE void AddrLED_BB_SetHighFast(void) {
	REG_WRITE(g_addrled_bb_gpio_cfg, g_addrled_bb_reg_high);
}

ADDRLED_BB_ALWAYS_INLINE void AddrLED_BB_SetLowFast(void) {
	REG_WRITE(g_addrled_bb_gpio_cfg, g_addrled_bb_reg_low);
}

static void ADDRLED_BB_RAMFUNC AddrLED_BB_SendWS2812BBytes(const byte *data, uint32_t len) {
	if (!data || len == 0 || !g_addrled_bb_gpio_cfg) {
		return;
	}

	volatile unsigned int *gpio_cfg_addr = g_addrled_bb_gpio_cfg;
	unsigned int reg_val_HIGH = g_addrled_bb_reg_high;
	unsigned int reg_val_LOW = g_addrled_bb_reg_low;
	int t0h = g_addrled_bb_custom_t0h;
	int t1h = g_addrled_bb_custom_t1h;
	int t0l = g_addrled_bb_custom_t0l;
	int t1l = g_addrled_bb_custom_t1l;

	switch (g_addrled_bb_profile) {
	case 0:
		t0h = ADDRLED_BB_P0_T0H; t1h = ADDRLED_BB_P0_T1H; t0l = ADDRLED_BB_P0_T0L; t1l = ADDRLED_BB_P0_T1L;
		break;
	case 1:
		t0h = ADDRLED_BB_P1_T0H; t1h = ADDRLED_BB_P1_T1H; t0l = ADDRLED_BB_P1_T0L; t1l = ADDRLED_BB_P1_T1L;
		break;
	case 2:
		t0h = ADDRLED_BB_P2_T0H; t1h = ADDRLED_BB_P2_T1H; t0l = ADDRLED_BB_P2_T0L; t1l = ADDRLED_BB_P2_T1L;
		break;
	case 3:
		t0h = ADDRLED_BB_P3_T0H; t1h = ADDRLED_BB_P3_T1H; t0l = ADDRLED_BB_P3_T0L; t1l = ADDRLED_BB_P3_T1L;
		break;
	case 4:
		t0h = ADDRLED_BB_P4_T0H; t1h = ADDRLED_BB_P4_T1H; t0l = ADDRLED_BB_P4_T0L; t1l = ADDRLED_BB_P4_T1L;
		break;
	case 5:
		t0h = ADDRLED_BB_P5_T0H; t1h = ADDRLED_BB_P5_T1H; t0l = ADDRLED_BB_P5_T0L; t1l = ADDRLED_BB_P5_T1L;
		break;
	case 6:
		t0h = ADDRLED_BB_P6_T0H; t1h = ADDRLED_BB_P6_T1H; t0l = ADDRLED_BB_P6_T0L; t1l = ADDRLED_BB_P6_T1L;
		break;
	default:
		break;
	}

	REG_WRITE(gpio_cfg_addr, reg_val_LOW);
	HAL_Delay_us(g_addrled_bb_reset_us);

	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	for (uint32_t i = 0; i < len; i++) {
		byte b = data[i];
		for (int bit = 7; bit >= 0; bit--) {
			if (b & (1 << bit)) {
				REG_WRITE(gpio_cfg_addr, reg_val_HIGH);
				AddrLED_BB_NopDelayVariable(t1h);
				REG_WRITE(gpio_cfg_addr, reg_val_LOW);
				AddrLED_BB_NopDelayVariable(t1l);
			}
			else {
				REG_WRITE(gpio_cfg_addr, reg_val_HIGH);
				AddrLED_BB_NopDelayVariable(t0h);
				REG_WRITE(gpio_cfg_addr, reg_val_LOW);
				AddrLED_BB_NopDelayVariable(t0l);
			}
		}
	}

	REG_WRITE(gpio_cfg_addr, reg_val_LOW);

	GLOBAL_INT_RESTORE();

	HAL_Delay_us(g_addrled_bb_reset_us);
}

static void AddrLED_BB_SendBytes(const byte *data, uint32_t len) {
	switch (g_addrled_bb_protocol) {
	case ADDRLED_BB_PROTOCOL_WS2812B:
	default:
		AddrLED_BB_SendWS2812BBytes(data, len);
		break;
	}
}

static void AddrLED_BB_Show(void) {
	AddrLED_BB_SendBytes(g_addrled_bb_buf, g_addrled_bb_len);
}

static byte AddrLED_BB_GetByte(uint32_t idx) {
	if (!g_addrled_bb_buf || idx >= g_addrled_bb_len) {
		return 0;
	}
	return g_addrled_bb_buf[idx];
}

static void AddrLED_BB_SetByte(uint32_t index, byte color) {
	if (!g_addrled_bb_buf || index >= g_addrled_bb_len) {
		return;
	}
	g_addrled_bb_buf[index] = color;
}

static void AddrLED_BB_SetLEDCount(int new_pixel_count, int new_pixel_size) {
	uint32_t new_len = new_pixel_count * new_pixel_size;
	if (g_addrled_bb_buf) {
		os_free(g_addrled_bb_buf);
		g_addrled_bb_buf = 0;
	}
	g_addrled_bb_len = 0;
	if (new_len == 0) {
		return;
	}
	g_addrled_bb_buf = (byte *)os_malloc(new_len);
	if (!g_addrled_bb_buf) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "AddrLED_BB: failed to allocate %u bytes", (unsigned int)new_len);
		return;
	}
	memset(g_addrled_bb_buf, 0, new_len);
	g_addrled_bb_len = new_len;
}

static bool AddrLED_BB_IsValidRGBOrder(const char *format) {
	bool seen_r = false;
	bool seen_g = false;
	bool seen_b = false;
	if (!format || strlen(format) != 3) {
		return false;
	}
	for (const char *sp = format; *sp; sp++) {
		switch (*sp) {
		case 'R':
			if (seen_r) return false;
			seen_r = true;
			break;
		case 'G':
			if (seen_g) return false;
			seen_g = true;
			break;
		case 'B':
			if (seen_b) return false;
			seen_b = true;
			break;
		default:
			return false;
		}
	}
	return seen_r && seen_g && seen_b;
}

static commandResult_t AddrLED_BB_CMD_InitForLEDCount(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);

	if (Tokenizer_GetArgsCount() == 0) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: AddrLED_BB_Init [LEDCount 1..%i] [ColorOrder, default GRB]", ADDRLED_BB_MAX_PIXELS);
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	int requested_pixel_count = Tokenizer_GetArgInteger(0);
	if (requested_pixel_count < 1 || requested_pixel_count > ADDRLED_BB_MAX_PIXELS) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB_Init: LED count must be 1..%i", ADDRLED_BB_MAX_PIXELS);
		return CMD_RES_BAD_ARGUMENT;
	}

	if (Tokenizer_GetArgsCount() == 1) {
		char default_args[16];
		sprintf(default_args, "%i GRB", requested_pixel_count);
		return Strip_CMD_InitForLEDCount(context, cmd, default_args, flags);
	}

	const char *format = Tokenizer_GetArg(1);
	if (!AddrLED_BB_IsValidRGBOrder(format)) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB_Init: WS2812B color order must be a 3-channel RGB permutation, e.g. GRB or RGB");
		return CMD_RES_BAD_ARGUMENT;
	}

	return Strip_CMD_InitForLEDCount(context, cmd, args, flags);
}

static commandResult_t AddrLED_BB_CMD_StartTX(const void *context, const char *cmd, const char *args, int flags) {
	Strip_Apply();
	return CMD_RES_OK;
}

static commandResult_t AddrLED_BB_CMD_SetRaw(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 3) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: AddrLED_BB_SetRaw [bUpdate] [byteOfs] [HexData]");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	if (!g_addrled_bb_buf || g_addrled_bb_len == 0) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB_SetRaw: call AddrLED_BB_Init first");
		return CMD_RES_ERROR;
	}

	int bPush = Tokenizer_GetArgInteger(0);
	int ofs = Tokenizer_GetArgInteger(1);
	const char *s = Tokenizer_GetArg(2);
	int hex_len = strlen(s);
	int byte_len = hex_len / 2;

	if ((hex_len & 1) || ofs < 0 || byte_len < 1 || (uint32_t)(ofs + byte_len) > g_addrled_bb_len) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB_SetRaw: range must fit current %u-byte LED buffer", (unsigned int)g_addrled_bb_len);
		return CMD_RES_BAD_ARGUMENT;
	}

	for (int i = 0; i < byte_len; i++) {
		g_addrled_bb_buf[ofs + i] = hexbyte(s + (i * 2));
	}
	if (bPush) {
		AddrLED_BB_Show();
	}
	return CMD_RES_OK;
}

static commandResult_t AddrLED_BB_CMD_Timing(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() == 0) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB custom timing: T0H=%i T1H=%i T0L=%i T1L=%i ResetUs=%i. Use AddrLED_BB_Profile for fixed-profile TX.", g_addrled_bb_custom_t0h, g_addrled_bb_custom_t1h, g_addrled_bb_custom_t0l, g_addrled_bb_custom_t1l, g_addrled_bb_reset_us);
		return CMD_RES_OK;
	}
	if (Tokenizer_GetArgsCount() < 5) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: AddrLED_BB_Timing [T0Hnops] [T1Hnops] [T0Lnops] [T1Lnops] [ResetUs]");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_addrled_bb_custom_t0h = Tokenizer_GetArgIntegerRange(0, 0, 200);
	g_addrled_bb_custom_t1h = Tokenizer_GetArgIntegerRange(1, 0, 200);
	g_addrled_bb_custom_t0l = Tokenizer_GetArgIntegerRange(2, 0, 200);
	g_addrled_bb_custom_t1l = Tokenizer_GetArgIntegerRange(3, 0, 200);
	g_addrled_bb_reset_us = Tokenizer_GetArgIntegerRange(4, 50, 500);
	g_addrled_bb_profile = -1;
	ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB custom timing set and selected: T0H=%i T1H=%i T0L=%i T1L=%i ResetUs=%i", g_addrled_bb_custom_t0h, g_addrled_bb_custom_t1h, g_addrled_bb_custom_t0l, g_addrled_bb_custom_t1l, g_addrled_bb_reset_us);
	return CMD_RES_OK;
}

static void AddrLED_BB_LogProfile(int profile) {
	switch (profile) {
	case 0: ADDLOG_INFO(LOG_FEATURE_CMD, "Profile 0: T0H=%i T1H=%i T0L=%i T1L=%i", ADDRLED_BB_P0_T0H, ADDRLED_BB_P0_T1H, ADDRLED_BB_P0_T0L, ADDRLED_BB_P0_T1L); break;
	case 1: ADDLOG_INFO(LOG_FEATURE_CMD, "Profile 1: T0H=%i T1H=%i T0L=%i T1L=%i", ADDRLED_BB_P1_T0H, ADDRLED_BB_P1_T1H, ADDRLED_BB_P1_T0L, ADDRLED_BB_P1_T1L); break;
	case 2: ADDLOG_INFO(LOG_FEATURE_CMD, "Profile 2: T0H=%i T1H=%i T0L=%i T1L=%i", ADDRLED_BB_P2_T0H, ADDRLED_BB_P2_T1H, ADDRLED_BB_P2_T0L, ADDRLED_BB_P2_T1L); break;
	case 3: ADDLOG_INFO(LOG_FEATURE_CMD, "Profile 3: T0H=%i T1H=%i T0L=%i T1L=%i", ADDRLED_BB_P3_T0H, ADDRLED_BB_P3_T1H, ADDRLED_BB_P3_T0L, ADDRLED_BB_P3_T1L); break;
	case 4: ADDLOG_INFO(LOG_FEATURE_CMD, "Profile 4: T0H=%i T1H=%i T0L=%i T1L=%i", ADDRLED_BB_P4_T0H, ADDRLED_BB_P4_T1H, ADDRLED_BB_P4_T0L, ADDRLED_BB_P4_T1L); break;
	case 5: ADDLOG_INFO(LOG_FEATURE_CMD, "Profile 5: T0H=%i T1H=%i T0L=%i T1L=%i", ADDRLED_BB_P5_T0H, ADDRLED_BB_P5_T1H, ADDRLED_BB_P5_T0L, ADDRLED_BB_P5_T1L); break;
	case 6: ADDLOG_INFO(LOG_FEATURE_CMD, "Profile 6: T0H=%i T1H=%i T0L=%i T1L=%i", ADDRLED_BB_P6_T0H, ADDRLED_BB_P6_T1H, ADDRLED_BB_P6_T0L, ADDRLED_BB_P6_T1L); break;
	default: ADDLOG_INFO(LOG_FEATURE_CMD, "Profile custom: T0H=%i T1H=%i T0L=%i T1L=%i", g_addrled_bb_custom_t0h, g_addrled_bb_custom_t1h, g_addrled_bb_custom_t0l, g_addrled_bb_custom_t1l); break;
	}
}

static commandResult_t AddrLED_BB_CMD_Profile(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() == 0) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB profile=%i resetUs=%i gpioMode=%s", g_addrled_bb_profile, g_addrled_bb_reset_us, g_addrled_bb_gpio_mode == ADDRLED_BB_GPIO_RAW ? "raw" : "preserve");
		for (int i = 0; i <= 6; i++) {
			AddrLED_BB_LogProfile(i);
		}
		return CMD_RES_OK;
	}
	int profile = Tokenizer_GetArgIntegerRange(0, 0, 6);
	g_addrled_bb_profile = profile;
	if (Tokenizer_GetArgsCount() > 1) {
		g_addrled_bb_reset_us = Tokenizer_GetArgIntegerRange(1, 50, 500);
	}
	ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB selected fixed profile %i resetUs=%i", g_addrled_bb_profile, g_addrled_bb_reset_us);
	AddrLED_BB_LogProfile(g_addrled_bb_profile);
	return CMD_RES_OK;
}

static commandResult_t AddrLED_BB_CMD_GPIOMode(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() == 0) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB GPIO mode: %s", g_addrled_bb_gpio_mode == ADDRLED_BB_GPIO_RAW ? "raw" : "preserve");
		return CMD_RES_OK;
	}
	const char *mode = Tokenizer_GetArg(0);
	if (!stricmp(mode, "raw")) {
		g_addrled_bb_gpio_mode = ADDRLED_BB_GPIO_RAW;
	}
	else if (!stricmp(mode, "preserve") || !stricmp(mode, "rmw")) {
		g_addrled_bb_gpio_mode = ADDRLED_BB_GPIO_PRESERVE;
	}
	else {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: AddrLED_BB_GPIOMode [raw|preserve]");
		return CMD_RES_BAD_ARGUMENT;
	}
	AddrLED_BB_UpdateGPIOValues();
	ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB GPIO mode set to %s high=0x%08x low=0x%08x", g_addrled_bb_gpio_mode == ADDRLED_BB_GPIO_RAW ? "raw" : "preserve", g_addrled_bb_reg_high, g_addrled_bb_reg_low);
	return CMD_RES_OK;
}

static commandResult_t AddrLED_BB_CMD_Protocol(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() == 0) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB protocol: WS2812B");
		return CMD_RES_OK;
	}
	const char *protocol = Tokenizer_GetArg(0);
	if (!stricmp(protocol, "WS2812B") || !stricmp(protocol, "WS2812")) {
		g_addrled_bb_protocol = ADDRLED_BB_PROTOCOL_WS2812B;
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB protocol set to WS2812B");
		return CMD_RES_OK;
	}
	ADDLOG_INFO(LOG_FEATURE_CMD, "Unsupported AddrLED_BB protocol '%s'. Supported: WS2812B", protocol);
	return CMD_RES_BAD_ARGUMENT;
}

static commandResult_t AddrLED_BB_CMD_SendRaw(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: AddrLED_BB_SendRaw [HexBytes]");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	const char *s = Tokenizer_GetArg(0);
	uint32_t hex_len = strlen(s);
	uint32_t len = hex_len / 2;
	if ((hex_len & 1) || len == 0 || len > (ADDRLED_BB_MAX_PIXELS * 3)) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB_SendRaw: byte count must be 1..%i", ADDRLED_BB_MAX_PIXELS * 3);
		return CMD_RES_BAD_ARGUMENT;
	}
	byte *tmp = (byte *)os_malloc(len);
	if (!tmp) {
		return CMD_RES_ERROR;
	}
	for (uint32_t i = 0; i < len; i++) {
		tmp[i] = hexbyte(s + (i * 2));
	}
	AddrLED_BB_SendBytes(tmp, len);
	os_free(tmp);
	return CMD_RES_OK;
}

static commandResult_t AddrLED_BB_CMD_PinHigh(const void *context, const char *cmd, const char *args, int flags) {
	if (g_addrled_bb_pin < 0 || !g_addrled_bb_gpio_cfg) {
		return CMD_RES_ERROR;
	}
	HAL_PIN_Setup_Output(g_addrled_bb_pin);
	AddrLED_BB_SetHighFast();
	ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB pin %i HIGH", g_addrled_bb_pin);
	return CMD_RES_OK;
}

static commandResult_t AddrLED_BB_CMD_PinLow(const void *context, const char *cmd, const char *args, int flags) {
	if (g_addrled_bb_pin < 0 || !g_addrled_bb_gpio_cfg) {
		return CMD_RES_ERROR;
	}
	HAL_PIN_Setup_Output(g_addrled_bb_pin);
	AddrLED_BB_SetLowFast();
	ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB pin %i LOW", g_addrled_bb_pin);
	return CMD_RES_OK;
}

static commandResult_t AddrLED_BB_CMD_TestPulse(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	int count = 10;
	int high_us = 10;
	int low_us = 10;
	if (Tokenizer_GetArgsCount() > 0) count = Tokenizer_GetArgIntegerRange(0, 1, 1000);
	if (Tokenizer_GetArgsCount() > 1) high_us = Tokenizer_GetArgIntegerRange(1, 1, 100000);
	if (Tokenizer_GetArgsCount() > 2) low_us = Tokenizer_GetArgIntegerRange(2, 1, 100000);
	for (int i = 0; i < count; i++) {
		AddrLED_BB_SetHighFast();
		HAL_Delay_us(high_us);
		AddrLED_BB_SetLowFast();
		HAL_Delay_us(low_us);
	}
	ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB pulse test complete: count=%i highUs=%i lowUs=%i", count, high_us, low_us);
	return CMD_RES_OK;
}

static void AddrLED_BB_FillPattern(byte *tmp, int count, const char *pattern) {
	memset(tmp, 0, count * 3);
	if (!stricmp(pattern, "ones") || !stricmp(pattern, "white")) {
		memset(tmp, 0xff, count * 3);
	}
	else if (!stricmp(pattern, "red")) {
		for (int i = 0; i < count; i++) {
			tmp[(i * 3) + 0] = 0x00;
			tmp[(i * 3) + 1] = 0xff;
			tmp[(i * 3) + 2] = 0x00;
		}
	}
	else if (!stricmp(pattern, "green")) {
		for (int i = 0; i < count; i++) {
			tmp[(i * 3) + 0] = 0xff;
			tmp[(i * 3) + 1] = 0x00;
			tmp[(i * 3) + 2] = 0x00;
		}
	}
	else if (!stricmp(pattern, "blue")) {
		for (int i = 0; i < count; i++) {
			tmp[(i * 3) + 0] = 0x00;
			tmp[(i * 3) + 1] = 0x00;
			tmp[(i * 3) + 2] = 0xff;
		}
	}
	else if (!stricmp(pattern, "single-red")) {
		tmp[0] = 0x00;
		tmp[1] = 0xff;
		tmp[2] = 0x00;
	}
	else if (!stricmp(pattern, "single-green")) {
		tmp[0] = 0xff;
		tmp[1] = 0x00;
		tmp[2] = 0x00;
	}
	else if (!stricmp(pattern, "single-blue")) {
		tmp[0] = 0x00;
		tmp[1] = 0x00;
		tmp[2] = 0xff;
	}
	else if (!stricmp(pattern, "rgb")) {
		if (count > 0) { tmp[0] = 0x00; tmp[1] = 0xff; tmp[2] = 0x00; }
		if (count > 1) { tmp[3] = 0xff; tmp[4] = 0x00; tmp[5] = 0x00; }
		if (count > 2) { tmp[6] = 0x00; tmp[7] = 0x00; tmp[8] = 0xff; }
	}
}

static commandResult_t AddrLED_BB_CMD_TestPattern(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: AddrLED_BB_TestPattern [zeros|ones|red|green|blue|single-red|single-green|single-blue|rgb] [count 1..20]");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	const char *pattern = Tokenizer_GetArg(0);
	int count = 1;
	if (Tokenizer_GetArgsCount() > 1) {
		count = Tokenizer_GetArgIntegerRange(1, 1, ADDRLED_BB_MAX_PIXELS);
	}
	if (stricmp(pattern, "zeros") && stricmp(pattern, "ones") && stricmp(pattern, "white") && stricmp(pattern, "red") && stricmp(pattern, "green") && stricmp(pattern, "blue") && stricmp(pattern, "single-red") && stricmp(pattern, "single-green") && stricmp(pattern, "single-blue") && stricmp(pattern, "rgb")) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB_TestPattern: unsupported pattern '%s'", pattern);
		return CMD_RES_BAD_ARGUMENT;
	}
	byte *tmp = (byte *)os_malloc(count * 3);
	if (!tmp) {
		return CMD_RES_ERROR;
	}
	AddrLED_BB_FillPattern(tmp, count, pattern);
	AddrLED_BB_SendBytes(tmp, count * 3);
	os_free(tmp);
	return CMD_RES_OK;
}

void AddrLED_BB_Init() {
	if (DRV_IsRunning("SM16703P")) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "AddrLED_BB: SM16703P is already running; stop it before starting AddrLED_BB");
		return;
	}

	g_addrled_bb_pin = PIN_FindPinIndexForRole(IOR_AddrLED_BB_DIN, -1);
	if (g_addrled_bb_pin < 0) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "AddrLED_BB: no AddrLED_BB_DIN pin role configured");
		return;
	}

	HAL_PIN_Setup_Output(g_addrled_bb_pin);
	HAL_PIN_SetOutputValue(g_addrled_bb_pin, 0);
	AddrLED_BB_PrepareFastGPIO(g_addrled_bb_pin);
	AddrLED_BB_SetLowFast();

	ledStrip_t addrled_bb_export;
	addrled_bb_export.apply = AddrLED_BB_Show;
	addrled_bb_export.getByte = AddrLED_BB_GetByte;
	addrled_bb_export.setByte = AddrLED_BB_SetByte;
	addrled_bb_export.setLEDCount = AddrLED_BB_SetLEDCount;

	LEDS_InitSharedBackendOnly(&addrled_bb_export);

	CMD_RegisterCommand("AddrLED_BB_Init", AddrLED_BB_CMD_InitForLEDCount, NULL);
	CMD_RegisterCommand("AddrLED_BB_Start", AddrLED_BB_CMD_StartTX, NULL);
	CMD_RegisterCommand("AddrLED_BB_SetPixel", Strip_CMD_setPixel, NULL);
	CMD_RegisterCommand("AddrLED_BB_SetRaw", AddrLED_BB_CMD_SetRaw, NULL);
	CMD_RegisterCommand("AddrLED_BB_Timing", AddrLED_BB_CMD_Timing, NULL);
	CMD_RegisterCommand("AddrLED_BB_Profile", AddrLED_BB_CMD_Profile, NULL);
	CMD_RegisterCommand("AddrLED_BB_GPIOMode", AddrLED_BB_CMD_GPIOMode, NULL);
	CMD_RegisterCommand("AddrLED_BB_Protocol", AddrLED_BB_CMD_Protocol, NULL);
	CMD_RegisterCommand("AddrLED_BB_SendRaw", AddrLED_BB_CMD_SendRaw, NULL);
	CMD_RegisterCommand("AddrLED_BB_PinHigh", AddrLED_BB_CMD_PinHigh, NULL);
	CMD_RegisterCommand("AddrLED_BB_PinLow", AddrLED_BB_CMD_PinLow, NULL);
	CMD_RegisterCommand("AddrLED_BB_TestPulse", AddrLED_BB_CMD_TestPulse, NULL);
	CMD_RegisterCommand("AddrLED_BB_TestPattern", AddrLED_BB_CMD_TestPattern, NULL);

	ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB v5 started on pin %i; protocol WS2812B; max LEDs %i; profile %i; default color order GRB", g_addrled_bb_pin, ADDRLED_BB_MAX_PIXELS, g_addrled_bb_profile);
}

void AddrLED_BB_Shutdown() {
	if (g_addrled_bb_buf) {
		os_free(g_addrled_bb_buf);
		g_addrled_bb_buf = 0;
	}
	g_addrled_bb_len = 0;
	g_addrled_bb_pin = -1;
	g_addrled_bb_gpio_cfg = 0;
	LEDS_ShutdownShared();
}

#endif
