//
// Generic GirierMCU information
//

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../quicktick.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "../mqtt/new_mqtt.h"
#include "drv_girierMCU.h"
#include "drv_uart.h"

#define GIRIERMCU_BUFFER_SIZE 	256

typedef struct GirierMCUMapping_s {
	// internal Tuya variable index
	byte dpId;
	// data point type (one of the DP_TYPE_xxx defines)
	byte dpType;
	// true if it's supposed to be sent in dp cache
	byte obkFlags;
	// could be renamed to flags later?
	byte inv;
	// target channel
	short channel;
	// store last channel value to avoid sending it again
	int prevValue;
	// allow storing raw data for later usage
	byte *rawData;
	int rawBufferSize;
	int rawDataLen;
	// not really useful as long as we have integer channels
	float mult;
	float delta;
	float delta2;
	float delta3;
	// TODO
	//int mode;
	// list
	struct GirierMCUMapping_s* next;
} girierMCUMapping_t;

girierMCUMapping_t* g_girierMappings = 0;

/**
 * Dimmer range
 *
 * Map OpenBK7231T_App's dimmer range of 0..100 to the dimmer range used by GirierMCU (0..255).
 * 
 */
 // minimum dimmer value as reported by GirierMCU dimmer
static int g_dimmerRangeMin = 0;
// maximum dimmer value as reported by GirierMCU dimmer
static int g_dimmerRangeMax = 100;

// serial baud rate used to communicate with the GirierMCU
// common baud rates are 9600 bit/s and 115200 bit/s
static int g_baudRate = 9600;

// global mcu time
static int g_girierNextRequestDelay;

// ?? it's unused atm
//static char *prod_info = NULL;
static bool working_mode_valid = false;
static bool self_processing_mode = true;

static byte *g_GirierMCUpayloadBuffer = 0;
static int g_GirierMCUpayloadBufferSize = 0;

typedef struct GirierMCUPacket_s {
	byte *data;
	int size;
	int allocated;
	struct GirierMCUPacket_s *next;
} GirierMCUPacket_t;

GirierMCUPacket_t *gmcu_emptyPackets = 0;
GirierMCUPacket_t *gmcu_sendPackets = 0;

static inline int GirierMCU_ClampInt(int v, int lo, int hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static inline bool GirierMCU_IsBoolLike(int v) {
	return (v == 0) || (v == 1);
}

GirierMCUPacket_t *GirierMCU_AddToQueue(int len) {
	GirierMCUPacket_t *toUse;
	if (len < 0) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_AddToQueue: negative len=%d\n", len);
		return NULL;
	}
	const size_t want = (size_t)len;
	if (gmcu_emptyPackets) {
		toUse = gmcu_emptyPackets;
		gmcu_emptyPackets = toUse->next;

		// Make sure we don't keep stale links from the free-list.
		toUse->next = NULL;

		if (want > (size_t)toUse->allocated) {
			void *tmp = realloc(toUse->data, want);
			if (tmp == NULL) {
				addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_AddToQueue: realloc(%u) failed\n", (unsigned)want);
				// Put it back on the free-list so we don't lose it.
				toUse->next = gmcu_emptyPackets;
				gmcu_emptyPackets = toUse;
				return NULL;
			}
			toUse->data = (byte*)tmp;
			toUse->allocated = (int)want;
		}
	}
	else {
		toUse = (GirierMCUPacket_t*)malloc(sizeof(GirierMCUPacket_t));
		if (toUse == NULL) {
			addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_AddToQueue: malloc(packet) failed\n");
			return NULL;
		}
		memset(toUse, 0, sizeof(*toUse));
		int toAlloc = 128;
		if (len > toAlloc)
			toAlloc = len;
		toUse->allocated = toAlloc;
		toUse->data = (byte*)malloc((size_t)toUse->allocated);
		if (toUse->data == NULL) {
			addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_AddToQueue: malloc(%d) failed\n", toUse->allocated);
			free(toUse);
			return NULL;
		}
	}
	toUse->size = len;
	if (gmcu_sendPackets == 0) {
		gmcu_sendPackets = toUse;
	}
	else {
		GirierMCUPacket_t *p = gmcu_sendPackets;
		while (p->next) {
			p = p->next;
		}
		p->next = toUse;
	}
	toUse->next = NULL;
	return toUse;
}

bool GirierMCU_SendFromQueue(void) {
	GirierMCUPacket_t *toUse;
	if (gmcu_sendPackets == 0)
		return false;
	toUse = gmcu_sendPackets;
	gmcu_sendPackets = toUse->next;

	UART_SendByte(0x00);
	UART_SendByte(0x00);
	UART_SendByte(0xff);
	UART_SendByte(0x55);
	
	for (int i = 0; i < toUse->size; i++) {
		UART_SendByte(toUse->data[i]);
	}

	UART_SendByte(0x05);
	UART_SendByte(0xdc);
	UART_SendByte(0x0a);
	UART_SendByte(0x00);
	UART_SendByte(0x00);
	
	toUse->next = gmcu_emptyPackets;
	gmcu_emptyPackets = toUse;
	return true;
}

girierMCUMapping_t* GirierMCU_FindDefForID(int dpId) {
	addLogAdv(LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		"_FindDefForID(dpId=%d) called\n", dpId);
	girierMCUMapping_t* cur;

	cur = g_girierMappings;
	while (cur) {
		if (cur->dpId == dpId)
			return cur;
		cur = cur->next;
	}
	return 0;
}

girierMCUMapping_t* GirierMCU_FindDefForChannel(int channel) {
	addLogAdv(LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		"_FindDefForChannel(channel=%d) called\n", channel);
	girierMCUMapping_t* cur;

	cur = g_girierMappings;
	while (cur) {
		if (cur->channel == channel)
			return cur;
		cur = cur->next;
	}
	return 0;
}

girierMCUMapping_t* GirierMCU_MapIDToChannel(int dpId, int dpType, int channel, int obkFlags, float mul, int inv, float delta, float delta2, float delta3) {
	addLogAdv(
		LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		"_MapIDToChannel(dpId=%d, dpType=%d, channel=%d, obkFlags=%d, mul=%f, inv=%d, delta=%f, delta2=%f, delta3=%f) called", 
		dpId, dpType, channel, obkFlags, mul, inv, delta, delta2, delta3
	);
	girierMCUMapping_t* cur;

	cur = GirierMCU_FindDefForID(dpId);

	if (cur == 0) {
		cur = (girierMCUMapping_t*)malloc(sizeof(girierMCUMapping_t));
		if (cur == NULL) {
			addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_MapIDToChannel: malloc failed\n");
			return NULL;
		}
		memset(cur, 0, sizeof(*cur));
		cur->next = g_girierMappings;
		g_girierMappings = cur;
	}
	// Clamp/cast to on-wire / struct sizes to avoid silent truncation.
	cur->dpId = (byte)GirierMCU_ClampInt(dpId, 0, 255);
	cur->dpType = (byte)GirierMCU_ClampInt(dpType, 0, 255);
	cur->obkFlags = (byte)GirierMCU_ClampInt(obkFlags, 0, 255);
	cur->mult = mul;
	cur->delta = delta;
	cur->delta2 = delta2;
	cur->delta3 = delta3;
	cur->inv = (byte)GirierMCU_ClampInt(inv, 0, 255);
	cur->prevValue = 0;
	cur->channel = (short)GirierMCU_ClampInt(channel, -32768, 32767);
	return cur;
}

void GirierMCU_SendInit(void) {

	    byte girier_hello[] = {
			0x55, 0xaa, 0x00, 0xfe, 0x00, 0x05, 0x03, 0x03, 0xff, 0x03, 0xff, 0x09,
			0x00, 0x00, 0xff, 0x55, 0x01, 0x00, 0x79, 0x05, 0xdc, 0x0a, 0x00, 0x00,
			0x00, 0x00, 0xff, 0x55, 0x02, 0x79, 0x00, 0x05, 0xdc, 0x0a, 0x00, 0x00
		 };

		UART_InitUART(g_baudRate, 0, false);
		UART_SendByte(0x00);

		for (size_t i = 0; i < sizeof(girier_hello); i++) {
			UART_SendByte(girier_hello[i]);
		}     		
}


// append header, len, everything, checksum
void GirierMCU_SendCommandWithData(byte* data, int payload_len) {
	addLogAdv(
		LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		"_SendCommandWithData(data=%p, payload_len=%d) called\n",
		 data, payload_len
	);
	int i;
	if (payload_len < 0) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_SendCommandWithData: negative payload_len=%d\n", payload_len);
		return;
	}
	if ((payload_len > 0) && (data == NULL)) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_SendCommandWithData: NULL data with payload_len=%d\n", payload_len);
		return;
	}
	
	UART_InitUART(g_baudRate, 0, false);
	 if (CFG_HasFlag(OBK_FLAG_TUYAMCU_USE_QUEUE)) {
		GirierMCUPacket_t *p = GirierMCU_AddToQueue(payload_len);
		if (p == NULL) {
			addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_SendCommandWithData: queue alloc failed (len=%d)\n", payload_len);
			return;
		}
		if (payload_len > 0) {
			memcpy(p->data, data, (size_t)payload_len);
		}

	} else {
		UART_SendByte(0x00);
		UART_SendByte(0x00);
		UART_SendByte(0xff); 
		UART_SendByte(0x55);  
		     			
		for (i = 0; i < payload_len; i++) {
			byte b = data[i];
			UART_SendByte(b);
		}

		UART_SendByte(0x05);
		UART_SendByte(0xdc);
		UART_SendByte(0x0a);
		UART_SendByte(0x00);
		UART_SendByte(0x00);

	}
}

int GirierMCU_AppendStateInternal(byte *buffer, int bufferMax, int currentLen, uint8_t dpId, void* value, int dataLen) {
	addLogAdv(
		LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		"_AppendStateInternal(buffer=%p, bufferMax=%d, currentLen=%d, id=%d,value=%p, dataLen=%d) called\n", 
		buffer, bufferMax, currentLen, dpId, value, dataLen
	);

	if (buffer == NULL || bufferMax <= 0 || currentLen < 0 || currentLen > bufferMax || dataLen < 0) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_AppendStateInternal: bad args (buffer=%p bufferMax=%d currentLen=%d dataLen=%d)\n",
			buffer, bufferMax, currentLen, dataLen);
		return -1;
	}
	if ((dataLen > 0) && (value == NULL)) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_AppendStateInternal: NULL value with dataLen=%d\n", dataLen);
		return -1;
	}

	if (currentLen + 1 + dataLen > bufferMax) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_AppendStateInternal: buffer overflow (need=%d max=%d)\n",
			currentLen + 1 + dataLen, bufferMax);
		return -1;
	}
	buffer[currentLen + 0] = dpId;
	if (dataLen > 0) {
		memcpy(buffer + (currentLen + 1), value, (size_t)dataLen);
	}

	return currentLen + 1 + dataLen;
}

void GirierMCU_SendStateInternal(uint8_t dpId, void* value, int dataLen) {
	addLogAdv(
		LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		"_SendStateInternal(id=%d, value=%p, dataLen=%d) called\n", 
		dpId, value, dataLen
	);
	int payload_len = 0;
	if (g_GirierMCUpayloadBuffer == NULL || g_GirierMCUpayloadBufferSize <= 0) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU_SendStateInternal: payload buffer not initialised\n");
		return;
	}
	
	payload_len = GirierMCU_AppendStateInternal(
		g_GirierMCUpayloadBuffer,
		g_GirierMCUpayloadBufferSize,
		payload_len, 
		dpId, 
		value, 
		dataLen
	);

	if (payload_len <= 0) {
		return;
	}

	GirierMCU_SendCommandWithData(g_GirierMCUpayloadBuffer, payload_len);
}

void GirierMCU_SendValue(uint8_t dpId, int value) {
    uint8_t brightness = (uint8_t)(value & 0xFF);

    // two-byte payload (channel 1 + channel 2 brightness)
    uint8_t brightnessData[2] = { 0x00, 0x00 };

    if (dpId == 1) {
        brightnessData[0] = brightness;   // CH1 value in first byte
    } else if (dpId == 2) {
        brightnessData[1] = brightness;   // CH2 value in second byte
    } else {
        addLogAdv(LOG_WARN, LOG_FEATURE_TUYAMCU,
                  "Invalid dpId=%u (expected 1 or 2)", dpId);
        return;
    }

    addLogAdv(LOG_DEBUG, LOG_FEATURE_TUYAMCU,
              "dpId=%u, brightness=%u (data[0]=%u, data[1]=%u)",
              dpId, brightness, brightnessData[0], brightnessData[1]);

    // send both bytes (MCU might expect a fixed 2-byte frame)
    GirierMCU_SendStateInternal(dpId, brightnessData, sizeof(brightnessData));
}

commandResult_t GirierMCU_LinkGirierMCUOutputToChannel(const void* context, const char* cmd, const char* args, int cmdFlags) {
	addLogAdv(
		LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		"_LinkGirierMCUOutputToChannel(context=%p, cmd=%s, args=%s, cmdFlags=%d) called\n",
		context, cmd, args, cmdFlags
	);
	const char* dpTypeString;
	int dpId;
	byte dpType;
	int channelID;
	int argsCount;
	byte obkFlags;
	float mult, delta, delta2, delta3;
	byte inv;

	// linkGirierMCUOutputToChannel [dpId] [varType] [channelID] [obkFlags] [mult] [inv] [delta]
	// linkGirierMCUOutputToChannel 1 val 1
	Tokenizer_TokenizeString(args, 0);

	argsCount = Tokenizer_GetArgsCount();
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	dpId = Tokenizer_GetArgInteger(0);
	dpTypeString = Tokenizer_GetArg(1);

	if (argsCount < 3) {
		channelID = -999;
	}
	else {
		channelID = Tokenizer_GetArgInteger(2);
	}
	// Keep the same numeric codes as TuyaMCU for familiarity.
	// (This driver currently does not use dpType in its send path.)
	#define DP_TYPE_RAW    0x00
	#define DP_TYPE_BOOL   0x01
	#define DP_TYPE_VALUE  0x02
	#define DP_TYPE_STRING 0x03
	#define DP_TYPE_ENUM   0x04
	#define DP_TYPE_BITMAP 0x05

	if (dpTypeString == NULL) {
		dpType = DP_TYPE_VALUE;
	} else if (!stricmp(dpTypeString, "raw")) {
		dpType = DP_TYPE_RAW;
	} else if (!stricmp(dpTypeString, "bool")) {
		dpType = DP_TYPE_BOOL;
	} else if (!stricmp(dpTypeString, "val") || !stricmp(dpTypeString, "value")) {
		dpType = DP_TYPE_VALUE;
	} else if (!stricmp(dpTypeString, "str") || !stricmp(dpTypeString, "string")) {
		dpType = DP_TYPE_STRING;
	} else if (!stricmp(dpTypeString, "enum")) {
		dpType = DP_TYPE_ENUM;
	} else if (!stricmp(dpTypeString, "bitmap")) {
		dpType = DP_TYPE_BITMAP;
	} else {
		addLogAdv(LOG_WARN, LOG_FEATURE_TUYAMCU, "GirierMCU: unknown dpType '%s', defaulting to VALUE\n", dpTypeString);
		dpType = DP_TYPE_VALUE;
	}

	obkFlags = (byte)Tokenizer_GetArgIntegerDefault(3, 0);
	mult = Tokenizer_GetArgFloatDefault(4, 1.0f);
	inv = (byte)Tokenizer_GetArgIntegerDefault(5, 0);
	delta = Tokenizer_GetArgFloatDefault(6, 0.0f);
	delta2 = Tokenizer_GetArgFloatDefault(7, 0.0f);
	delta3 = Tokenizer_GetArgFloatDefault(8, 0.0f);

	GirierMCU_MapIDToChannel(dpId, dpType, channelID, obkFlags, mult, inv, delta, delta2, delta3);

	return CMD_RES_OK;
}

bool GirierMCU_IsChannelUsedByGirierMCU(int channel) {
	addLogAdv(LOG_DEBUG, LOG_FEATURE_TUYAMCU, "_IsChannelUsedByGirierMCU(channel=%d) called", channel);
	girierMCUMapping_t* mapping;

	// find mapping
	mapping = GirierMCU_FindDefForChannel(channel);

	if (mapping == 0) {
		return false;
	}
	return true;
}

void GirierMCU_OnChannelChanged(int channel, int iVal) {
	addLogAdv(
		LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		"_OnChannelChanged(channel=%d, iVal=%d) called", 
		channel, iVal
	);
	girierMCUMapping_t* mapping;

	// find mapping
	mapping = GirierMCU_FindDefForChannel(channel);

	if (mapping == 0) {
		return;
	}

	if (mapping->inv) {
		// Historically this flag behaved like a boolean NOT. Preserve that for boolean-like values,
		// but for dimmer-like values invert within the configured range.
		if (GirierMCU_IsBoolLike(iVal)) {
			iVal = !iVal;
		} else {
			iVal = (g_dimmerRangeMax + g_dimmerRangeMin) - iVal;
		}
	}

	const int range = (g_dimmerRangeMax - g_dimmerRangeMin);
	if (range <= 0) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_TUYAMCU, "GirierMCU: invalid dimmer range min=%d max=%d\n", g_dimmerRangeMin, g_dimmerRangeMax);
		return;
	}

	// Clamp to the configured OpenBK dimmer range before mapping.
	iVal = GirierMCU_ClampInt(iVal, g_dimmerRangeMin, g_dimmerRangeMax);

	int mappediVal;
	{
		const int64_t num = ((int64_t)(iVal - g_dimmerRangeMin) * 255LL);
		mappediVal = (int)(num / (int64_t)range);
	}
	mappediVal = GirierMCU_ClampInt(mappediVal, 0, 255);

	addLogAdv(
		LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		"g_dimmerRangeMax:%d, g_dimmerRangeMin:%d, mapped dp %i value %d to %d\n",
		g_dimmerRangeMax, g_dimmerRangeMin,  channel, iVal, mappediVal
	);

	// this might be a callback from CHANNEL_Set in GirierMCU_ApplyMapping. If we should set exactly the
	// same value, skip it
	if (mapping->prevValue == iVal) {
		return;
	}

	if (iVal != mappediVal) {
		addLogAdv(
			LOG_DEBUG, LOG_FEATURE_TUYAMCU,
			"OnChannelChanged: mapped value %d (OpenBK7231T_App range) to %d (GirierMCU range)\n",
			iVal,
			mappediVal
		);
	}
	mapping->prevValue = iVal;
	// send value to GirierMCU
	GirierMCU_SendValue(mapping->dpId, mappediVal);
		
}

#define CALIB_IF_NONZERO(x,d) if(x) { x += d; }

int girier_timer_send = 0;
void GirierMCU_RunFrame() {
	if (girier_timer_send > 0) {
		girier_timer_send -= g_deltaTimeMS;
	}
	else {
		if (GirierMCU_SendFromQueue()) {
			girier_timer_send = 100;
		}
	}
}

void GirierMCU_RunEverySecond() {

}

commandResult_t GirierMCU_SetBaudRate(const void* context, const char* cmd, const char* args, int cmdFlags) {
	addLogAdv(
		LOG_DEBUG, LOG_FEATURE_TUYAMCU,
		 "_SetBaudRate(context=%p, cmd=%s, args=%s, cmdFlags=%d) called",
		  context, cmd, args, cmdFlags
	);

	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	g_baudRate = Tokenizer_GetArgInteger(0);
	UART_InitUART(g_baudRate, 0, false);

	return CMD_RES_OK;
}

static SemaphoreHandle_t g_mutex = 0;

void GirierMCU_Shutdown() {
	addLogAdv(LOG_DEBUG, LOG_FEATURE_TUYAMCU, "_Shutdown() called");
	girierMCUMapping_t *tmp, *nxt;
	GirierMCUPacket_t *packet, *next_packet;

	// free the girierMCUMapping_t linked list
	tmp = g_girierMappings;
	while (tmp) {
		nxt = tmp->next;
		// free rawData if allocated
		if (tmp->rawData) {
			free(tmp->rawData);
			tmp->rawData = NULL;
			tmp->rawBufferSize = 0;
			tmp->rawDataLen = 0;
		}
		free(tmp);
		tmp = nxt;
	}
	g_girierMappings = NULL;

	// free the GirierMCUpayloadBuffer
	if (g_GirierMCUpayloadBuffer) {
		free(g_GirierMCUpayloadBuffer);
		g_GirierMCUpayloadBuffer = NULL;
		g_GirierMCUpayloadBufferSize = 0;
	}

	// free the gmcu_emptyPackets queue
	packet = gmcu_emptyPackets;
	while (packet) {
		next_packet = packet->next;
		if (packet->data) {
			free(packet->data);
			packet->data = NULL;
			packet->allocated = 0;
			packet->size = 0;
		}
		free(packet);
		packet = next_packet;
	}
	gmcu_emptyPackets = NULL;

	// free the gmcu_sendPackets queue
	packet = gmcu_sendPackets;
	while (packet) {
		next_packet = packet->next;
		if (packet->data) {
			free(packet->data);
			packet->data = NULL;
			packet->allocated = 0;
			packet->size = 0;
		}
		free(packet);
		packet = next_packet;
	}
	gmcu_sendPackets = NULL;

	// free the mutex
	if (g_mutex) {
		//vSemaphoreDelete(g_mutex);
		g_mutex = (SemaphoreHandle_t)0;
	}
}

void GirierMCU_ForcePublishChannelValues() {
	addLogAdv(LOG_DEBUG, LOG_FEATURE_TUYAMCU, "_ForcePublishChannelValues() called");
#if ENABLE_MQTT
	girierMCUMapping_t* cur;

	cur = g_girierMappings;
	while (cur) {
		MQTT_ChannelPublish(cur->channel, 0);
		cur = cur->next;
	}
#endif
}

void GirierMCU_Init() {
	addLogAdv(LOG_DEBUG, LOG_FEATURE_TUYAMCU, "_Init() called");
	g_girierNextRequestDelay = 1;
	if (g_GirierMCUpayloadBuffer == 0) {
		g_GirierMCUpayloadBufferSize = GIRIERMCU_BUFFER_SIZE;
		g_GirierMCUpayloadBuffer = (byte*)malloc(GIRIERMCU_BUFFER_SIZE);
	}

	UART_InitUART(g_baudRate, 0, false);
	UART_InitReceiveRingBuffer(1024);
	GirierMCU_SendInit();
	//cmddetail:{"name":"linkGirierMCUOutputToChannel","args":"TODO",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"GirierMCU_LinkGirierMCUOutputToChannel","file":"driver/drv_girierMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("linkGirierMCUOutputToChannel", GirierMCU_LinkGirierMCUOutputToChannel, NULL);
	//cmddetail:{"name":"GirierMCU_setBaudRate","args":"TODO",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"GirierMCU_SetBaudRate","file":"driver/drv_girierMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("GirierMCU_setBaudRate", GirierMCU_SetBaudRate, NULL);

}

