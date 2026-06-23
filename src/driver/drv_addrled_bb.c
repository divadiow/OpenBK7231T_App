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

#define ADDRLED_BB_DEFAULT_T0H_NOPS      3
#define ADDRLED_BB_DEFAULT_T1H_NOPS      8
#define ADDRLED_BB_DEFAULT_T0L_NOPS      8
#define ADDRLED_BB_DEFAULT_T1L_NOPS      3
#define ADDRLED_BB_DEFAULT_RESET_US      80
#define ADDRLED_BB_MAX_PIXELS            20
#define ADDRLED_BB_FAST_HIGH             0x02
#define ADDRLED_BB_FAST_LOW              0x00
#define ADDRLED_BB_ALWAYS_INLINE static inline __attribute__((always_inline))

typedef enum addrledBBProtocol_e {
	ADDRLED_BB_PROTOCOL_WS2812B = 0,
} addrledBBProtocol_t;

extern commandResult_t Strip_CMD_InitForLEDCount(const void *context, const char *cmd, const char *args, int flags);
extern commandResult_t Strip_CMD_setPixel(const void *context, const char *cmd, const char *args, int flags);
extern void Strip_Apply(void);

static byte *g_addrled_bb_buf;
static uint32_t g_addrled_bb_len;
static int g_addrled_bb_pin = -1;
static volatile unsigned int *g_addrled_bb_gpio_cfg;
static int g_addrled_bb_t0h_nops = ADDRLED_BB_DEFAULT_T0H_NOPS;
static int g_addrled_bb_t1h_nops = ADDRLED_BB_DEFAULT_T1H_NOPS;
static int g_addrled_bb_t0l_nops = ADDRLED_BB_DEFAULT_T0L_NOPS;
static int g_addrled_bb_t1l_nops = ADDRLED_BB_DEFAULT_T1L_NOPS;
static int g_addrled_bb_reset_us = ADDRLED_BB_DEFAULT_RESET_US;
static addrledBBProtocol_t g_addrled_bb_protocol = ADDRLED_BB_PROTOCOL_WS2812B;

ADDRLED_BB_ALWAYS_INLINE void AddrLED_BB_NopDelay(int nops) {
	while (nops-- > 0) {
		__asm__ __volatile__("nop");
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
}

ADDRLED_BB_ALWAYS_INLINE void AddrLED_BB_SetHigh(void) {
	REG_WRITE(g_addrled_bb_gpio_cfg, ADDRLED_BB_FAST_HIGH);
}

ADDRLED_BB_ALWAYS_INLINE void AddrLED_BB_SetLow(void) {
	REG_WRITE(g_addrled_bb_gpio_cfg, ADDRLED_BB_FAST_LOW);
}

ADDRLED_BB_ALWAYS_INLINE void AddrLED_BB_SendWS2812BBit0(void) {
	AddrLED_BB_SetHigh();
	AddrLED_BB_NopDelay(g_addrled_bb_t0h_nops);
	AddrLED_BB_SetLow();
	AddrLED_BB_NopDelay(g_addrled_bb_t0l_nops);
}

ADDRLED_BB_ALWAYS_INLINE void AddrLED_BB_SendWS2812BBit1(void) {
	AddrLED_BB_SetHigh();
	AddrLED_BB_NopDelay(g_addrled_bb_t1h_nops);
	AddrLED_BB_SetLow();
	AddrLED_BB_NopDelay(g_addrled_bb_t1l_nops);
}

static void __attribute__((noinline)) AddrLED_BB_SendWS2812BBytes(const byte *data, uint32_t len) {
	if (!data || len == 0 || !g_addrled_bb_gpio_cfg) {
		return;
	}

	AddrLED_BB_SetLow();
	HAL_Delay_us(g_addrled_bb_reset_us);

	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	for (uint32_t i = 0; i < len; i++) {
		byte b = data[i];
		for (int bit = 7; bit >= 0; bit--) {
			if (b & (1 << bit)) {
				AddrLED_BB_SendWS2812BBit1();
			}
			else {
				AddrLED_BB_SendWS2812BBit0();
			}
		}
	}

	AddrLED_BB_SetLow();

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
		ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB timing: T0H=%i T1H=%i T0L=%i T1L=%i ResetUs=%i", g_addrled_bb_t0h_nops, g_addrled_bb_t1h_nops, g_addrled_bb_t0l_nops, g_addrled_bb_t1l_nops, g_addrled_bb_reset_us);
		return CMD_RES_OK;
	}
	if (Tokenizer_GetArgsCount() < 5) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: AddrLED_BB_Timing [T0Hnops] [T1Hnops] [T0Lnops] [T1Lnops] [ResetUs]");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_addrled_bb_t0h_nops = Tokenizer_GetArgIntegerRange(0, 0, 80);
	g_addrled_bb_t1h_nops = Tokenizer_GetArgIntegerRange(1, 0, 80);
	g_addrled_bb_t0l_nops = Tokenizer_GetArgIntegerRange(2, 0, 80);
	g_addrled_bb_t1l_nops = Tokenizer_GetArgIntegerRange(3, 0, 80);
	g_addrled_bb_reset_us = Tokenizer_GetArgIntegerRange(4, 50, 300);
	ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB timing set: T0H=%i T1H=%i T0L=%i T1L=%i ResetUs=%i", g_addrled_bb_t0h_nops, g_addrled_bb_t1h_nops, g_addrled_bb_t0l_nops, g_addrled_bb_t1l_nops, g_addrled_bb_reset_us);
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
	CMD_RegisterCommand("AddrLED_BB_Protocol", AddrLED_BB_CMD_Protocol, NULL);
	CMD_RegisterCommand("AddrLED_BB_SendRaw", AddrLED_BB_CMD_SendRaw, NULL);

	ADDLOG_INFO(LOG_FEATURE_CMD, "AddrLED_BB started on pin %i; protocol WS2812B; max LEDs %i; default color order GRB", g_addrled_bb_pin, ADDRLED_BB_MAX_PIXELS);
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
