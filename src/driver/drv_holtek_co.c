#include "../obk_config.h"

#if ENABLE_DRIVER_HOLTEKCO

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../new_common.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "../httpserver/new_http.h"
#include "drv_public.h"
#include "drv_uart.h"


#define HOLTEKCO_DRIVER_VERSION             "0.3.0"
#define HOLTEKCO_FRAME_LEN                  19
#define HOLTEKCO_RESPONSE_HEADER            0xAA
#define HOLTEKCO_RESPONSE_LEN_BYTE          0x13
#define HOLTEKCO_RESPONSE_END_MARKER        0x55
#define HOLTEKCO_DEFAULT_BAUD               9600
#define HOLTEKCO_DEFAULT_POLL_INTERVAL      5
#define HOLTEKCO_UART_BUFFER_SIZE           256

/*
 * HoltekCO v0.3.0
 *
 * Driver for CO detector designs where a Holtek BA45F6746-class CO MCU is
 * connected to the Wi-Fi module by UART. This is intentionally separate from
 * TuyaMCU: the factory CBU firmware speaks a short Holtek status protocol, not
 * the Tuya serial MCU protocol.
 *
 * Confirmed factory-side behaviour:
 *   Wi-Fi MCU -> Holtek query: 55 07 03 01 00 00 60
 *   The factory app sends this query three times before parsing a response.
 *   Holtek -> Wi-Fi MCU:      AA 13 ... 55 checksum
 *
 * The checksum is the low byte of sum(frame[0]..frame[17]); frame[18] is the
 * checksum. The 0x55 end marker is included in the checksum.
 *
 * Confirmed payload fields from live captures and factory app analysis:
 *   frame[3] bit 6  CO alarm/status bit
 *   frame[3] bit 5  fault bit
 *   frame[8]        CO ppm high byte
 *   frame[9]        CO ppm low byte
 *
 * Battery is deliberately not measured or owned by this driver. The original
 * firmware measures battery on the Wi-Fi MCU side using a local ADC input and
 * a switched sampling circuit; OpenBeken's Battery driver should be used for
 * that, with the correct board pin roles/configuration.
 */

static int g_holtekco_baudRate = HOLTEKCO_DEFAULT_BAUD;
static int g_holtekco_pollIntervalSeconds = HOLTEKCO_DEFAULT_POLL_INTERVAL;
static int g_holtekco_pollCountdownSeconds = 0;
static int g_holtekco_queryRepeats = 3;
static int g_holtekco_debug = 0;

static int g_holtekco_chCO = 1;
static int g_holtekco_chAlarm = 2;
static int g_holtekco_chFault = 3;

static int g_holtekco_lastCO = 0;
static int g_holtekco_lastAlarm = 0;
static int g_holtekco_lastFault = 0;
static int g_holtekco_validFrames = 0;
static int g_holtekco_checksumErrors = 0;
static int g_holtekco_badFrames = 0;
static int g_holtekco_badEndMarkerFrames = 0;
static int g_holtekco_garbageBytes = 0;
static int g_holtekco_secondsSinceValidFrame = -1;
static int g_holtekco_txQueries = 0;
static int g_holtekco_txBytes = 0;
static int g_holtekco_rxBytesConsumed = 0;
static int g_holtekco_rxFramesWithChangedState = 0;
static byte g_holtekco_lastFrame[HOLTEKCO_FRAME_LEN];

static const byte g_holtekco_query[] = {
	0x55, 0x07, 0x03, 0x01, 0x00, 0x00, 0x60
};

static bool HoltekCO_IsChannelValid(int ch) {
	return ch >= 0 && ch < CHANNEL_MAX;
}

static void HoltekCO_SetChannelIfChanged(int ch, int value) {
	if (!HoltekCO_IsChannelValid(ch)) {
		return;
	}
	if (CHANNEL_Get(ch) != value) {
		CHANNEL_Set(ch, value, 0);
	}
}

static void HoltekCO_ApplyChannelSetup(void) {
	if (HoltekCO_IsChannelValid(g_holtekco_chCO)) {
		CHANNEL_SetType(g_holtekco_chCO, ChType_ReadOnly);
		CHANNEL_SetLabel(g_holtekco_chCO, "CO ppm", 1);
	}
	if (HoltekCO_IsChannelValid(g_holtekco_chAlarm)) {
		CHANNEL_SetType(g_holtekco_chAlarm, ChType_ReadOnly);
		CHANNEL_SetLabel(g_holtekco_chAlarm, "CO alarm", 1);
	}
	if (HoltekCO_IsChannelValid(g_holtekco_chFault)) {
		CHANNEL_SetType(g_holtekco_chFault, ChType_ReadOnly);
		CHANNEL_SetLabel(g_holtekco_chFault, "CO fault", 1);
	}
}

static void HoltekCO_SendByteTracked(byte b) {
	UART_SendByte(b);
	g_holtekco_txBytes++;
}

static void HoltekCO_SendQueryOnce(void) {
	int i;
	for (i = 0; i < (int)sizeof(g_holtekco_query); i++) {
		HoltekCO_SendByteTracked(g_holtekco_query[i]);
	}
	g_holtekco_txQueries++;
}

static void HoltekCO_SendQueryRepeats(void) {
	int r;
	int repeats;
	repeats = g_holtekco_queryRepeats;
	if (repeats < 1) {
		repeats = 1;
	}

	for (r = 0; r < repeats; r++) {
		HoltekCO_SendQueryOnce();
	}

	if (g_holtekco_debug >= 1) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "HoltekCO: TX factory query repeats=%i bytes=%i totalQueries=%i uart=%i",
			repeats,
			repeats * (int)sizeof(g_holtekco_query),
			g_holtekco_txQueries,
			UART_GetSelectedPortIndex());
	}
}

static byte HoltekCO_CalcChecksum(const byte *frame) {
	int i;
	byte sum = 0;
	for (i = 0; i < HOLTEKCO_FRAME_LEN - 1; i++) {
		sum += frame[i];
	}
	return sum;
}

static void HoltekCO_FormatFrame(char *out, int outLen, const byte *frame) {
	int i;
	int pos = 0;
	if (outLen <= 0) {
		return;
	}
	out[0] = 0;
	for (i = 0; i < HOLTEKCO_FRAME_LEN; i++) {
		int written;
		if (pos >= outLen - 1) {
			break;
		}
		written = snprintf(out + pos, outLen - pos, "%s%02X", i ? " " : "", frame[i]);
		if (written < 0) {
			break;
		}
		pos += written;
	}
	out[outLen - 1] = 0;
}

static void HoltekCO_ProcessFrame(const byte *frame) {
	int co;
	int fault;
	int alarm;
	int changed;

	co = ((int)frame[8] << 8) | frame[9];
	alarm = (frame[3] & 0x40) ? 1 : 0;
	fault = (frame[3] & 0x20) ? 1 : 0;
	changed = (co != g_holtekco_lastCO) || (fault != g_holtekco_lastFault) || (alarm != g_holtekco_lastAlarm);

	memcpy(g_holtekco_lastFrame, frame, HOLTEKCO_FRAME_LEN);
	g_holtekco_validFrames++;
	g_holtekco_secondsSinceValidFrame = 0;

	if (changed) {
		g_holtekco_rxFramesWithChangedState++;
	}

	g_holtekco_lastCO = co;
	g_holtekco_lastFault = fault;
	g_holtekco_lastAlarm = alarm;

	HoltekCO_SetChannelIfChanged(g_holtekco_chCO, co);
	HoltekCO_SetChannelIfChanged(g_holtekco_chFault, fault);
	HoltekCO_SetChannelIfChanged(g_holtekco_chAlarm, alarm);

	if (g_holtekco_debug >= 2 || (g_holtekco_debug >= 1 && changed)) {
		char tmp[96];
		HoltekCO_FormatFrame(tmp, sizeof(tmp), frame);
		ADDLOG_INFO(LOG_FEATURE_DRV, "HoltekCO: RX valid frame=%s co=%i alarm=%i fault=%i status=0x%02X event=0x%02X changed=%i",
			tmp, co, alarm, fault, frame[3], frame[10], changed);
	}
}

static void HoltekCO_ConsumeBytesTracked(int count) {
	UART_ConsumeBytes(count);
	g_holtekco_rxBytesConsumed += count;
}

static int HoltekCO_TryGetNextFrame(byte *out) {
	int cs;
	int i;

	cs = UART_GetDataSize();
	if (cs < HOLTEKCO_FRAME_LEN) {
		return 0;
	}

	while (cs > 0 && UART_GetByte(0) != HOLTEKCO_RESPONSE_HEADER) {
		HoltekCO_ConsumeBytesTracked(1);
		g_holtekco_garbageBytes++;
		cs--;
	}

	if (cs < HOLTEKCO_FRAME_LEN) {
		return 0;
	}

	if (UART_GetByte(1) != HOLTEKCO_RESPONSE_LEN_BYTE) {
		HoltekCO_ConsumeBytesTracked(1);
		g_holtekco_badFrames++;
		return 0;
	}

	for (i = 0; i < HOLTEKCO_FRAME_LEN; i++) {
		out[i] = UART_GetByte(i);
	}

	if (out[17] != HOLTEKCO_RESPONSE_END_MARKER) {
		HoltekCO_ConsumeBytesTracked(1);
		g_holtekco_badFrames++;
		g_holtekco_badEndMarkerFrames++;
		if (g_holtekco_debug >= 2) {
			char tmp[96];
			HoltekCO_FormatFrame(tmp, sizeof(tmp), out);
			ADDLOG_INFO(LOG_FEATURE_DRV, "HoltekCO: rejected end marker frame=%s", tmp);
		}
		return 0;
	}

	if (HoltekCO_CalcChecksum(out) != out[18]) {
		HoltekCO_ConsumeBytesTracked(1);
		g_holtekco_checksumErrors++;
		if (g_holtekco_debug >= 2) {
			char tmp[96];
			HoltekCO_FormatFrame(tmp, sizeof(tmp), out);
			ADDLOG_INFO(LOG_FEATURE_DRV, "HoltekCO: rejected checksum calc=%02X frame=%s", HoltekCO_CalcChecksum(out), tmp);
		}
		return 0;
	}

	HoltekCO_ConsumeBytesTracked(HOLTEKCO_FRAME_LEN);
	return HOLTEKCO_FRAME_LEN;
}

static commandResult_t Cmd_HoltekCO_SetChannels(const void *context, const char *cmd, const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	g_holtekco_chCO = Tokenizer_GetArgInteger(0);
	g_holtekco_chAlarm = Tokenizer_GetArgInteger(1);
	g_holtekco_chFault = Tokenizer_GetArgInteger(2);
	HoltekCO_ApplyChannelSetup();

	ADDLOG_INFO(LOG_FEATURE_CMD, "HoltekCO: channels CO=%i Alarm=%i Fault=%i",
		g_holtekco_chCO, g_holtekco_chAlarm, g_holtekco_chFault);
	return CMD_RES_OK;
}

static commandResult_t Cmd_HoltekCO_SetPollInterval(const void *context, const char *cmd, const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_holtekco_pollIntervalSeconds = Tokenizer_GetArgInteger(0);
	if (g_holtekco_pollIntervalSeconds < 0) {
		g_holtekco_pollIntervalSeconds = 0;
	}
	g_holtekco_pollCountdownSeconds = 0;
	ADDLOG_INFO(LOG_FEATURE_CMD, "HoltekCO: poll interval=%i seconds", g_holtekco_pollIntervalSeconds);
	return CMD_RES_OK;
}

static commandResult_t Cmd_HoltekCO_SetQueryRepeats(const void *context, const char *cmd, const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_holtekco_queryRepeats = Tokenizer_GetArgInteger(0);
	if (g_holtekco_queryRepeats < 1) {
		g_holtekco_queryRepeats = 1;
	}
	if (g_holtekco_queryRepeats > 10) {
		g_holtekco_queryRepeats = 10;
	}
	ADDLOG_INFO(LOG_FEATURE_CMD, "HoltekCO: query repeats=%i", g_holtekco_queryRepeats);
	return CMD_RES_OK;
}

static commandResult_t Cmd_HoltekCO_SetDebug(const void *context, const char *cmd, const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_holtekco_debug = Tokenizer_GetArgInteger(0);
	if (g_holtekco_debug < 0) {
		g_holtekco_debug = 0;
	}
	if (g_holtekco_debug > 2) {
		g_holtekco_debug = 2;
	}
	ADDLOG_INFO(LOG_FEATURE_CMD, "HoltekCO: debug=%i", g_holtekco_debug);
	return CMD_RES_OK;
}

static commandResult_t Cmd_HoltekCO_SetBaudRate(const void *context, const char *cmd, const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_holtekco_baudRate = Tokenizer_GetArgInteger(0);
	if (g_holtekco_baudRate <= 0) {
		g_holtekco_baudRate = HOLTEKCO_DEFAULT_BAUD;
	}
	UART_InitUART(g_holtekco_baudRate, 0, false);
	ADDLOG_INFO(LOG_FEATURE_CMD, "HoltekCO: baud=%i uart=%i", g_holtekco_baudRate, UART_GetSelectedPortIndex());
	return CMD_RES_OK;
}

static commandResult_t Cmd_HoltekCO_SendQuery(const void *context, const char *cmd, const char *args, int cmdFlags) {
	HoltekCO_SendQueryRepeats();
	return CMD_RES_OK;
}

static commandResult_t Cmd_HoltekCO_Status(const void *context, const char *cmd, const char *args, int cmdFlags) {
	char tmp[96];
	HoltekCO_FormatFrame(tmp, sizeof(tmp), g_holtekco_lastFrame);
	ADDLOG_INFO(LOG_FEATURE_CMD,
		"HoltekCO v%s: CO=%i ppm alarm=%i fault=%i status=0x%02X event=0x%02X aux11=0x%02X aux12=0x%02X aux13=0x%02X valid=%i changed=%i bad=%i end=%i checksum=%i garbage=%i age=%i txQueries=%i txBytes=%i rxBytes=%i uart=%i repeats=%i frame=%s",
		HOLTEKCO_DRIVER_VERSION,
		g_holtekco_lastCO,
		g_holtekco_lastAlarm,
		g_holtekco_lastFault,
		g_holtekco_lastFrame[3],
		g_holtekco_lastFrame[10],
		g_holtekco_lastFrame[11],
		g_holtekco_lastFrame[12],
		g_holtekco_lastFrame[13],
		g_holtekco_validFrames,
		g_holtekco_rxFramesWithChangedState,
		g_holtekco_badFrames,
		g_holtekco_badEndMarkerFrames,
		g_holtekco_checksumErrors,
		g_holtekco_garbageBytes,
		g_holtekco_secondsSinceValidFrame,
		g_holtekco_txQueries,
		g_holtekco_txBytes,
		g_holtekco_rxBytesConsumed,
		UART_GetSelectedPortIndex(),
		g_holtekco_queryRepeats,
		tmp);
	return CMD_RES_OK;
}

void HoltekCO_RunFrame(void) {
	byte frame[HOLTEKCO_FRAME_LEN];
	int guard = 0;
	while (HoltekCO_TryGetNextFrame(frame) > 0) {
		HoltekCO_ProcessFrame(frame);
		guard++;
		if (guard > 8) {
			break;
		}
	}
}

void HoltekCO_RunEverySecond(void) {
	if (g_holtekco_secondsSinceValidFrame >= 0) {
		g_holtekco_secondsSinceValidFrame++;
	}


	if (g_holtekco_pollIntervalSeconds <= 0) {
		return;
	}
	if (g_holtekco_pollCountdownSeconds <= 0) {
		HoltekCO_SendQueryRepeats();
		g_holtekco_pollCountdownSeconds = g_holtekco_pollIntervalSeconds;
	}
	else {
		g_holtekco_pollCountdownSeconds--;
	}
}

void HoltekCO_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
	char tmp[96];
	if (bPreState) {
		return;
	}
	HoltekCO_FormatFrame(tmp, sizeof(tmp), g_holtekco_lastFrame);
	hprintf255(request, "<h2>HoltekCO v%s</h2>", HOLTEKCO_DRIVER_VERSION);
	hprintf255(request, "<p>CO: %i ppm, alarm: %i, fault: %i</p>",
		g_holtekco_lastCO, g_holtekco_lastAlarm, g_holtekco_lastFault);
	hprintf255(request, "<p>Status: byte=0x%02X alarmBit=%i faultBit=%i event=0x%02X aux11=0x%02X aux12=0x%02X aux13=0x%02X</p>",
		g_holtekco_lastFrame[3],
		(g_holtekco_lastFrame[3] & 0x40) ? 1 : 0,
		(g_holtekco_lastFrame[3] & 0x20) ? 1 : 0,
		g_holtekco_lastFrame[10],
		g_holtekco_lastFrame[11],
		g_holtekco_lastFrame[12],
		g_holtekco_lastFrame[13]);
	hprintf255(request, "<p>Frames: valid=%i changed=%i bad=%i end=%i checksum=%i garbage=%i age=%is</p>",
		g_holtekco_validFrames, g_holtekco_rxFramesWithChangedState,
		g_holtekco_badFrames, g_holtekco_badEndMarkerFrames,
		g_holtekco_checksumErrors, g_holtekco_garbageBytes,
		g_holtekco_secondsSinceValidFrame);
	hprintf255(request, "<p>UART: index=%i baud=%i, TX: queries=%i bytes=%i, RX consumed bytes=%i</p>",
		UART_GetSelectedPortIndex(), g_holtekco_baudRate,
		g_holtekco_txQueries, g_holtekco_txBytes, g_holtekco_rxBytesConsumed);
	hprintf255(request, "<p>Config: poll=%is, queryRepeats=%i, debug=%i</p>",
		g_holtekco_pollIntervalSeconds, g_holtekco_queryRepeats, g_holtekco_debug);
	hprintf255(request, "<p>Last frame: %s</p>", tmp);
}

void HoltekCO_Shutdown(void) {
	/* Nothing heap-backed in v0.3.0. */
}

void HoltekCO_Init(void) {
	g_holtekco_pollCountdownSeconds = 0;
	g_holtekco_secondsSinceValidFrame = -1;
	memset(g_holtekco_lastFrame, 0, sizeof(g_holtekco_lastFrame));

	UART_InitUART(g_holtekco_baudRate, 0, false);
	UART_InitReceiveRingBuffer(HOLTEKCO_UART_BUFFER_SIZE);
	HoltekCO_ApplyChannelSetup();

	//cmddetail:{"name":"HoltekCO_setChannels","args":"[coPpmChannel][alarmChannel][faultChannel]",
	//cmddetail:"descr":"Configures OpenBeken channels used by the Holtek CO detector UART driver.",
	//cmddetail:"fn":"Cmd_HoltekCO_SetChannels","file":"driver/drv_holtek_co.c","requires":"ENABLE_DRIVER_HOLTEKCO",
	//cmddetail:"examples":"HoltekCO_setChannels 1 2 3"}
	CMD_RegisterCommand("HoltekCO_setChannels", Cmd_HoltekCO_SetChannels, NULL);

	//cmddetail:{"name":"HoltekCO_setPollInterval","args":"[seconds]",
	//cmddetail:"descr":"Sets how often OpenBeken sends the known Holtek CO query frame. Use 0 to disable polling.",
	//cmddetail:"fn":"Cmd_HoltekCO_SetPollInterval","file":"driver/drv_holtek_co.c","requires":"ENABLE_DRIVER_HOLTEKCO",
	//cmddetail:"examples":"HoltekCO_setPollInterval 5"}
	CMD_RegisterCommand("HoltekCO_setPollInterval", Cmd_HoltekCO_SetPollInterval, NULL);

	//cmddetail:{"name":"HoltekCO_setQueryRepeats","args":"[count]",
	//cmddetail:"descr":"Sets how many short Holtek queries are sent each polling cycle. Default is 3.",
	//cmddetail:"fn":"Cmd_HoltekCO_SetQueryRepeats","file":"driver/drv_holtek_co.c","requires":"ENABLE_DRIVER_HOLTEKCO",
	//cmddetail:"examples":"HoltekCO_setQueryRepeats 3"}
	CMD_RegisterCommand("HoltekCO_setQueryRepeats", Cmd_HoltekCO_SetQueryRepeats, NULL);

	//cmddetail:{"name":"HoltekCO_setDebug","args":"[0/1/2]",
	//cmddetail:"descr":"Sets Holtek CO UART logging. 1 logs TX and changed values, 2 logs all valid frames and rejected candidates.",
	//cmddetail:"fn":"Cmd_HoltekCO_SetDebug","file":"driver/drv_holtek_co.c","requires":"ENABLE_DRIVER_HOLTEKCO",
	//cmddetail:"examples":"HoltekCO_setDebug 2"}
	CMD_RegisterCommand("HoltekCO_setDebug", Cmd_HoltekCO_SetDebug, NULL);

	//cmddetail:{"name":"HoltekCO_setBaudRate","args":"[baud]",
	//cmddetail:"descr":"Sets the UART baud rate used by the Holtek CO driver. Default is 9600.",
	//cmddetail:"fn":"Cmd_HoltekCO_SetBaudRate","file":"driver/drv_holtek_co.c","requires":"ENABLE_DRIVER_HOLTEKCO",
	//cmddetail:"examples":"HoltekCO_setBaudRate 9600"}
	CMD_RegisterCommand("HoltekCO_setBaudRate", Cmd_HoltekCO_SetBaudRate, NULL);

	//cmddetail:{"name":"HoltekCO_sendQuery","args":"",
	//cmddetail:"descr":"Sends the configured Holtek CO detector query immediately.",
	//cmddetail:"fn":"Cmd_HoltekCO_SendQuery","file":"driver/drv_holtek_co.c","requires":"ENABLE_DRIVER_HOLTEKCO",
	//cmddetail:"examples":"HoltekCO_sendQuery"}
	CMD_RegisterCommand("HoltekCO_sendQuery", Cmd_HoltekCO_SendQuery, NULL);

	//cmddetail:{"name":"HoltekCO_status","args":"",
	//cmddetail:"descr":"Prints the current Holtek CO driver state, counters and last received frame to the log.",
	//cmddetail:"fn":"Cmd_HoltekCO_Status","file":"driver/drv_holtek_co.c","requires":"ENABLE_DRIVER_HOLTEKCO",
	//cmddetail:"examples":"HoltekCO_status"}
	CMD_RegisterCommand("HoltekCO_status", Cmd_HoltekCO_Status, NULL);

	HoltekCO_SendQueryRepeats();
	ADDLOG_INFO(LOG_FEATURE_DRV, "HoltekCO v%s initialized baud=%i uart=%i channels=%i/%i/%i poll=%i repeats=%i",
		HOLTEKCO_DRIVER_VERSION,
		g_holtekco_baudRate,
		UART_GetSelectedPortIndex(),
		g_holtekco_chCO,
		g_holtekco_chAlarm,
		g_holtekco_chFault,
		g_holtekco_pollIntervalSeconds,
		g_holtekco_queryRepeats);
}

#endif /* ENABLE_DRIVER_HOLTEKCO */
