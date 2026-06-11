#include "../new_pins.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "drv_aht2x.h"

typedef enum {
	AHT_PROFILE_AUTO = 0,
	AHT_PROFILE_LEGACY_AHT2X,
	AHT_PROFILE_AHT1X,
	AHT_PROFILE_AHT2X,
	AHT_PROFILE_AHT3X
} ahtProfile_t;

typedef enum {
	AHT_PROTOCOL_NONE = 0,
	AHT_PROTOCOL_LEGACY_AHT2X,
	AHT_PROTOCOL_AHT1X_COMPAT,
	AHT_PROTOCOL_AHT2X_COMPAT,
	AHT_PROTOCOL_AHT3X_COMPAT
} ahtProtocol_t;

typedef enum {
	AHT_ERR_NONE = 0,
	AHT_ERR_I2C_PREINIT,
	AHT_ERR_NO_ACK,
	AHT_ERR_BUSY_TIMEOUT,
	AHT_ERR_NOT_CALIBRATED,
	AHT_ERR_BAD_DATA,
	AHT_ERR_CRC
} ahtError_t;

static int g_aht_secondsUntilNextMeasurement = 1;
static int g_aht_secondsBetweenMeasurements = 10;
static int channel_temp = -1;
static int channel_humid = -1;
static uint8_t g_aht_maxRetries = 20;
static uint8_t g_aht_lastStatus = 0;
static float g_temp = 0.0f, g_humid = 0.0f, g_calTemp = 0.0f, g_calHum = 0.0f;
static softI2C_t g_softI2C;
static bool isWorking = false;
static bool g_aht_crcEnabled = false;
static ahtProfile_t g_aht_requestedProfile = AHT_PROFILE_AUTO;
static ahtProtocol_t g_aht_activeProtocol = AHT_PROTOCOL_NONE;
static ahtError_t g_aht_lastError = AHT_ERR_NONE;

static const char *AHTXX_ProfileName(ahtProfile_t profile)
{
	switch(profile)
	{
	case AHT_PROFILE_AUTO: return "auto";
	case AHT_PROFILE_LEGACY_AHT2X: return "legacy-aht2x";
	case AHT_PROFILE_AHT1X: return "aht1x";
	case AHT_PROFILE_AHT2X: return "aht2x";
	case AHT_PROFILE_AHT3X: return "aht3x";
	default: return "unknown";
	}
}

static const char *AHTXX_ProtocolName(ahtProtocol_t protocol)
{
	switch(protocol)
	{
	case AHT_PROTOCOL_LEGACY_AHT2X: return "legacy AHT2X-compatible";
	case AHT_PROTOCOL_AHT1X_COMPAT: return "AHT1X-compatible";
	case AHT_PROTOCOL_AHT2X_COMPAT: return "AHT2X-compatible";
	case AHT_PROTOCOL_AHT3X_COMPAT: return "AHT3X-compatible";
	case AHT_PROTOCOL_NONE:
	default: return "none";
	}
}

static const char *AHTXX_ErrorName(ahtError_t error)
{
	switch(error)
	{
	case AHT_ERR_NONE: return "none";
	case AHT_ERR_I2C_PREINIT: return "i2c preinit failed";
	case AHT_ERR_NO_ACK: return "no i2c ack";
	case AHT_ERR_BUSY_TIMEOUT: return "busy timeout";
	case AHT_ERR_NOT_CALIBRATED: return "not calibrated";
	case AHT_ERR_BAD_DATA: return "bad data";
	case AHT_ERR_CRC: return "crc failed";
	default: return "unknown";
	}
}

static ahtProfile_t AHTXX_ParseProfile(const char *profile)
{
	if(profile == NULL || profile[0] == 0 || !stricmp(profile, "auto")) return AHT_PROFILE_AUTO;
	if(!stricmp(profile, "legacy")) return AHT_PROFILE_LEGACY_AHT2X;
	if(!stricmp(profile, "legacy-aht2x")) return AHT_PROFILE_LEGACY_AHT2X;
	if(!stricmp(profile, "aht1x")) return AHT_PROFILE_AHT1X;
	if(!stricmp(profile, "aht10")) return AHT_PROFILE_AHT1X;
	if(!stricmp(profile, "aht15")) return AHT_PROFILE_AHT1X;
	if(!stricmp(profile, "aht2x")) return AHT_PROFILE_AHT2X;
	if(!stricmp(profile, "aht20")) return AHT_PROFILE_AHT2X;
	if(!stricmp(profile, "aht21")) return AHT_PROFILE_AHT2X;
	if(!stricmp(profile, "aht21b")) return AHT_PROFILE_AHT2X;
	if(!stricmp(profile, "aht25")) return AHT_PROFILE_AHT2X;
	if(!stricmp(profile, "dht20")) return AHT_PROFILE_AHT2X;
	if(!stricmp(profile, "am2301b")) return AHT_PROFILE_AHT2X;
	if(!stricmp(profile, "aht3x")) return AHT_PROFILE_AHT3X;
	if(!stricmp(profile, "aht30")) return AHT_PROFILE_AHT3X;
	ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: Unknown profile '%s', using auto.", profile);
	return AHT_PROFILE_AUTO;
}

static bool AHTXX_WriteBytes(uint8_t cmd, uint8_t data1, uint8_t data2, int dataLen)
{
	bool ok;

	ok = Soft_I2C_Start(&g_softI2C, AHTXX_I2C_ADDR_WRITE);
	if(!ok)
	{
		Soft_I2C_Stop(&g_softI2C);
		g_aht_lastError = AHT_ERR_NO_ACK;
		return false;
	}
	ok = Soft_I2C_WriteByte(&g_softI2C, cmd);
	if(ok && dataLen > 0) ok = Soft_I2C_WriteByte(&g_softI2C, data1);
	if(ok && dataLen > 1) ok = Soft_I2C_WriteByte(&g_softI2C, data2);
	Soft_I2C_Stop(&g_softI2C);

	if(!ok) g_aht_lastError = AHT_ERR_NO_ACK;
	return ok;
}

static bool AHTXX_ReadBytes(uint8_t *data, int len)
{
	bool ok;

	ok = Soft_I2C_Start(&g_softI2C, AHTXX_I2C_ADDR_READ);
	if(!ok)
	{
		Soft_I2C_Stop(&g_softI2C);
		g_aht_lastError = AHT_ERR_NO_ACK;
		return false;
	}
	Soft_I2C_ReadBytes(&g_softI2C, data, len);
	Soft_I2C_Stop(&g_softI2C);
	return true;
}

static bool AHTXX_ReadStatus(uint8_t *status)
{
	if(!AHTXX_ReadBytes(status, 1)) return false;
	g_aht_lastStatus = *status;
	return true;
}

static bool AHTXX_WaitNotBusy(void)
{
	uint8_t status = AHTXX_STATUS_BUSY;
	uint8_t attempts = 0;

	while(status & AHTXX_STATUS_BUSY)
	{
		rtos_delay_milliseconds(20);
		if(!AHTXX_ReadStatus(&status)) return false;
		attempts++;
		if(attempts > g_aht_maxRetries)
		{
			g_aht_lastError = AHT_ERR_BUSY_TIMEOUT;
			return false;
		}
	}
	return true;
}

static uint8_t AHTXX_CalcCRC8(const uint8_t *data, int len)
{
	uint8_t crc = AHTXX_CRC8_INIT;
	int i, b;

	for(i = 0; i < len; i++)
	{
		crc ^= data[i];
		for(b = 0; b < 8; b++)
		{
			if(crc & 0x80)
			{
				crc = (uint8_t)((crc << 1) ^ AHTXX_CRC8_POLY);
			}
			else
			{
				crc <<= 1;
			}
		}
	}
	return crc;
}

void AHT2X_SoftReset(void)
{
	AHTXX_WriteBytes(AHTXX_CMD_SOFT_RESET, 0, 0, 0);
	rtos_delay_milliseconds(20);
}

static bool AHTXX_SendInitCommand(uint8_t initCmd)
{
	if(!AHTXX_WriteBytes(initCmd, AHTXX_DAT_INIT_1, AHTXX_DAT_INIT_2, 2)) return false;
	rtos_delay_milliseconds(10);
	return AHTXX_WaitNotBusy();
}

static bool AHTXX_StatusLooksReady(bool requireAht21Mask)
{
	uint8_t status;

	if(!AHTXX_ReadStatus(&status)) return false;
	if(requireAht21Mask)
	{
		if((status & AHTXX_STATUS_AHT21_READY_MASK) == AHTXX_STATUS_AHT21_READY_MASK) return true;
	}
	else if(status & AHTXX_STATUS_CALIBRATED)
	{
		return true;
	}
	g_aht_lastError = AHT_ERR_NOT_CALIBRATED;
	return false;
}

static bool AHTXX_TriggerMeasurement(void)
{
	if(!AHTXX_WriteBytes(AHTXX_CMD_TRIGGER_MEASUREMENT, AHTXX_DAT_TRIGGER_1, AHTXX_DAT_TRIGGER_2, 2)) return false;
	rtos_delay_milliseconds(80);
	return true;
}

static bool AHTXX_ReadAndDecodeMeasurement(bool updateChannels, bool logDetails)
{
	uint8_t data[AHTXX_MEASUREMENT_BYTES_CRC] = { 0, };
	int bytesToRead = g_aht_crcEnabled ? AHTXX_MEASUREMENT_BYTES_CRC : AHTXX_MEASUREMENT_BYTES;
	bool ready = false;
	uint8_t i;
	uint32_t raw_temperature;
	uint32_t raw_humidity;

	if(!AHTXX_TriggerMeasurement()) return false;

	for(i = 0; i < 10; i++)
	{
		if(!AHTXX_ReadBytes(data, bytesToRead)) return false;
		g_aht_lastStatus = data[0];
		if((data[0] & AHTXX_STATUS_BUSY) == 0)
		{
			ready = true;
			break;
		}
		if(logDetails)
		{
			ADDLOG_DEBUG(LOG_FEATURE_SENSOR, "AHTXX: Sensor is busy, waiting... (%ims)", i * 20);
		}
		rtos_delay_milliseconds(20);
	}

	if(!ready)
	{
		g_aht_lastError = AHT_ERR_BUSY_TIMEOUT;
		if(logDetails) ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: Measurement read timed out.");
		return false;
	}

	if(g_aht_crcEnabled && AHTXX_CalcCRC8(data, AHTXX_MEASUREMENT_BYTES) != data[6])
	{
		g_aht_lastError = AHT_ERR_CRC;
		if(logDetails) ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: CRC mismatch, ignoring measurement.");
		return false;
	}

	/* Preserve the old guard against all-zero humidity frames from floating/wrong I2C reads. */
	if(data[1] == 0x00 && data[2] == 0x00 && (data[3] >> 4) == 0x00)
	{
		g_aht_lastError = AHT_ERR_BAD_DATA;
		if(logDetails) ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: Unrealistic humidity, will not update values.");
		return false;
	}

	raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
	raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;

	g_humid = ((float)raw_humidity * 100.0f / 1048576.0f) + g_calHum;
	g_temp = (((200.0f * (float)raw_temperature) / 1048576.0f) - 50.0f) + g_calTemp;
	g_aht_lastError = AHT_ERR_NONE;

	if(updateChannels)
	{
		if(channel_temp >= 0)
		{
			CHANNEL_Set(channel_temp, (int)(g_temp * 10), 0);
		}
		if(channel_humid >= 0)
		{
			CHANNEL_Set(channel_humid, (int)g_humid, 0);
		}
	}

	if(logDetails)
	{
		ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: Temperature:%fC Humidity:%f%%", g_temp, g_humid);
	}
	return true;
}

static bool AHTXX_InitAsAHT1X(void)
{
	AHT2X_SoftReset();
	if(!AHTXX_SendInitCommand(AHTXX_CMD_INIT_AHT1X)) return false;
	if(!AHTXX_StatusLooksReady(false)) return false;
	g_aht_activeProtocol = AHT_PROTOCOL_AHT1X_COMPAT;
	return true;
}

static bool AHTXX_InitAsAHT2X(void)
{
	AHT2X_SoftReset();
	if(!AHTXX_SendInitCommand(AHTXX_CMD_INIT_AHT2X)) return false;
	if(!AHTXX_StatusLooksReady(false)) return false;
	g_aht_activeProtocol = AHT_PROTOCOL_AHT2X_COMPAT;
	return true;
}

static bool AHTXX_InitAsLegacyAHT2X(void)
{
	uint8_t status = AHTXX_STATUS_BUSY;

	AHT2X_SoftReset();
	if(!AHTXX_SendInitCommand(AHTXX_CMD_INIT_AHT2X)) return false;
	if(!AHTXX_ReadStatus(&status)) return false;
	/* Old OpenBeken check retained only for explicit 'legacy' mode. */
	if((status & 0x68) != 0x08)
	{
		g_aht_lastError = AHT_ERR_NOT_CALIBRATED;
		return false;
	}
	g_aht_activeProtocol = AHT_PROTOCOL_LEGACY_AHT2X;
	return true;
}

static bool AHTXX_InitAsAHT3X(void)
{
	AHT2X_SoftReset();
	/* AHT30 datasheet documents measurement/read/CRC sequence, not an AHT20-style 0xBE init. */
	if(!AHTXX_ReadAndDecodeMeasurement(false, false)) return false;
	g_aht_activeProtocol = AHT_PROTOCOL_AHT3X_COMPAT;
	return true;
}

static bool AHTXX_InitByProfile(ahtProfile_t profile)
{
	switch(profile)
	{
	case AHT_PROFILE_LEGACY_AHT2X: return AHTXX_InitAsLegacyAHT2X();
	case AHT_PROFILE_AHT1X: return AHTXX_InitAsAHT1X();
	case AHT_PROFILE_AHT2X: return AHTXX_InitAsAHT2X();
	case AHT_PROFILE_AHT3X: return AHTXX_InitAsAHT3X();
	case AHT_PROFILE_AUTO:
	default:
		if(AHTXX_InitAsAHT2X()) return true;
		if(AHTXX_InitAsAHT1X()) return true;
		return AHTXX_InitAsAHT3X();
	}
}

void AHT2X_Initialization(void)
{
	g_aht_activeProtocol = AHT_PROTOCOL_NONE;
	g_aht_lastError = AHT_ERR_NONE;
	isWorking = AHTXX_InitByProfile(g_aht_requestedProfile);
	if(isWorking)
	{
		ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: Initialization successful, requested=%s, active=%s, CRC=%s.",
			AHTXX_ProfileName(g_aht_requestedProfile), AHTXX_ProtocolName(g_aht_activeProtocol), g_aht_crcEnabled ? "on" : "off");
	}
	else
	{
		ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: Initialization failed, requested=%s, lastStatus=0x%02X, error=%s.",
			AHTXX_ProfileName(g_aht_requestedProfile), g_aht_lastStatus, AHTXX_ErrorName(g_aht_lastError));
	}
}

void AHT2X_StopDriver(void)
{
	AHT2X_SoftReset();
	isWorking = false;
	g_aht_activeProtocol = AHT_PROTOCOL_NONE;
}

void AHT2X_Measure(void)
{
	if(!AHTXX_ReadAndDecodeMeasurement(true, true))
	{
		isWorking = false;
		ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: Measurement failed, lastStatus=0x%02X, error=%s.",
			g_aht_lastStatus, AHTXX_ErrorName(g_aht_lastError));
	}
	else
	{
		isWorking = true;
	}
}

commandResult_t AHT2X_Calibrate(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	g_calTemp = Tokenizer_GetArgFloat(0);
	g_calHum = Tokenizer_GetArgFloatDefault(1, 0.0f);

	return CMD_RES_OK;
}

commandResult_t AHT2X_Cycle(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	int seconds = Tokenizer_GetArgInteger(0);
	if(seconds < 1)
	{
		ADDLOG_INFO(LOG_FEATURE_CMD, "AHTXX: You must have at least 1 second cycle.");
		return CMD_RES_BAD_ARGUMENT;
	}

	g_aht_secondsBetweenMeasurements = seconds;
	if(g_aht_secondsUntilNextMeasurement > g_aht_secondsBetweenMeasurements)
	{
		g_aht_secondsUntilNextMeasurement = g_aht_secondsBetweenMeasurements;
	}

	ADDLOG_INFO(LOG_FEATURE_CMD, "AHTXX: Measurement will run every %i seconds.", g_aht_secondsBetweenMeasurements);
	return CMD_RES_OK;
}

commandResult_t AHT2X_Force(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	g_aht_secondsUntilNextMeasurement = g_aht_secondsBetweenMeasurements;
	AHT2X_Measure();
	return CMD_RES_OK;
}

commandResult_t AHT2X_Reinit(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	g_aht_secondsUntilNextMeasurement = g_aht_secondsBetweenMeasurements;
	AHT2X_Initialization();
	return CMD_RES_OK;
}

commandResult_t AHT2X_Mode(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	g_aht_requestedProfile = AHTXX_ParseProfile(Tokenizer_GetArg(0));
	/* AHT30/AHT3X frames include CRC by spec; keep all other modes unchanged unless AHTXX_CRC is used. */
	g_aht_crcEnabled = (g_aht_requestedProfile == AHT_PROFILE_AHT3X);
	AHT2X_Initialization();
	return CMD_RES_OK;
}

commandResult_t AHT2X_CRC(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	g_aht_crcEnabled = Tokenizer_GetArgInteger(0) != 0;
	ADDLOG_INFO(LOG_FEATURE_CMD, "AHTXX: CRC checking is %s.", g_aht_crcEnabled ? "enabled" : "disabled");
	return CMD_RES_OK;
}

commandResult_t AHT2X_Info(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: requested=%s, active=%s, working=%i, crc=%i, lastStatus=0x%02X, lastError=%s, cycle=%i, tempChannel=%i, humidChannel=%i, calTemp=%f, calHum=%f.",
		AHTXX_ProfileName(g_aht_requestedProfile), AHTXX_ProtocolName(g_aht_activeProtocol), isWorking ? 1 : 0,
		g_aht_crcEnabled ? 1 : 0, g_aht_lastStatus, AHTXX_ErrorName(g_aht_lastError), g_aht_secondsBetweenMeasurements,
		channel_temp, channel_humid, g_calTemp, g_calHum);
	return CMD_RES_OK;
}

static void AHTXX_RegisterCommands(void)
{
	//cmddetail:{"name":"AHT2X_Calibrate","args":"[DeltaTemp][DeltaHumidity]",
	//cmddetail:"descr":"Calibrate the AHT-family sensor. Legacy AHT2X command name kept for compatibility.",
	//cmddetail:"fn":"AHT2X_Calibrate","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHT2X_Calibrate -4 10 <br /> meaning -4 on current temp reading and +10 on current humidity reading"}
	CMD_RegisterCommand("AHT2X_Calibrate", AHT2X_Calibrate, NULL);
	//cmddetail:{"name":"AHTXX_Calibrate","args":"[DeltaTemp][DeltaHumidity]",
	//cmddetail:"descr":"Calibrate the AHT-family sensor. Alias of AHT2X_Calibrate.",
	//cmddetail:"fn":"AHT2X_Calibrate","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHTXX_Calibrate -4 10"}
	CMD_RegisterCommand("AHTXX_Calibrate", AHT2X_Calibrate, NULL);
	//cmddetail:{"name":"AHT2X_Cycle","args":"[IntervalSeconds]",
	//cmddetail:"descr":"Set the AHT-family measurement interval in seconds. Legacy AHT2X command name kept for compatibility.",
	//cmddetail:"fn":"AHT2X_Cycle","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHT2X_Cycle 60 <br /> measurement is taken every 60 seconds"}
	CMD_RegisterCommand("AHT2X_Cycle", AHT2X_Cycle, NULL);
	//cmddetail:{"name":"AHTXX_Cycle","args":"[IntervalSeconds]",
	//cmddetail:"descr":"Set the AHT-family measurement interval in seconds. Alias of AHT2X_Cycle.",
	//cmddetail:"fn":"AHT2X_Cycle","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHTXX_Cycle 60"}
	CMD_RegisterCommand("AHTXX_Cycle", AHT2X_Cycle, NULL);
	//cmddetail:{"name":"AHT2X_Measure","args":"",
	//cmddetail:"descr":"Retrieve one-shot AHT-family measurement. Legacy AHT2X command name kept for compatibility.",
	//cmddetail:"fn":"AHT2X_Force","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHT2X_Measure"}
	CMD_RegisterCommand("AHT2X_Measure", AHT2X_Force, NULL);
	//cmddetail:{"name":"AHTXX_Measure","args":"",
	//cmddetail:"descr":"Retrieve one-shot AHT-family measurement. Alias of AHT2X_Measure.",
	//cmddetail:"fn":"AHT2X_Force","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHTXX_Measure"}
	CMD_RegisterCommand("AHTXX_Measure", AHT2X_Force, NULL);
	//cmddetail:{"name":"AHT2X_Reinit","args":"",
	//cmddetail:"descr":"Reinitialize the AHT-family sensor. Legacy AHT2X command name kept for compatibility.",
	//cmddetail:"fn":"AHT2X_Reinit","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHT2X_Reinit"}
	CMD_RegisterCommand("AHT2X_Reinit", AHT2X_Reinit, NULL);
	//cmddetail:{"name":"AHTXX_Reinit","args":"",
	//cmddetail:"descr":"Reinitialize the AHT-family sensor. Alias of AHT2X_Reinit.",
	//cmddetail:"fn":"AHT2X_Reinit","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHTXX_Reinit"}
	CMD_RegisterCommand("AHTXX_Reinit", AHT2X_Reinit, NULL);
	//cmddetail:{"name":"AHTXX_Mode","args":"[auto|legacy|aht1x|aht10|aht15|aht2x|aht20|aht21|aht21b|aht25|dht20|am2301b|aht3x|aht30]",
	//cmddetail:"descr":"Select and reinitialize the AHT-family compatibility profile. AHT2X_Mode is an alias.",
	//cmddetail:"fn":"AHT2X_Mode","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHTXX_Mode aht10 <br /> AHTXX_Mode auto"}
	CMD_RegisterCommand("AHTXX_Mode", AHT2X_Mode, NULL);
	CMD_RegisterCommand("AHT2X_Mode", AHT2X_Mode, NULL);
	//cmddetail:{"name":"AHTXX_CRC","args":"[0|1]",
	//cmddetail:"descr":"Enable or disable CRC8 checking for AHT-family seven-byte measurement frames. AHT2X_CRC is an alias.",
	//cmddetail:"fn":"AHT2X_CRC","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHTXX_CRC 1"}
	CMD_RegisterCommand("AHTXX_CRC", AHT2X_CRC, NULL);
	CMD_RegisterCommand("AHT2X_CRC", AHT2X_CRC, NULL);
	//cmddetail:{"name":"AHTXX_Info","args":"",
	//cmddetail:"descr":"Log AHT-family driver state, selected profile, active protocol, channels, calibration and last error. AHT2X_Info is an alias.",
	//cmddetail:"fn":"AHT2X_Info","file":"driver/drv_aht2x.c","requires":"",
	//cmddetail:"examples":"AHTXX_Info"}
	CMD_RegisterCommand("AHTXX_Info", AHT2X_Info, NULL);
	CMD_RegisterCommand("AHT2X_Info", AHT2X_Info, NULL);
}

// startDriver AHT2X 4 5 3 4
// startDriver AHTXX 4 5 3 4 [auto|aht1x|aht2x|aht3x|legacy]
void AHT2X_Init(void)
{
	g_softI2C.pin_clk = Tokenizer_GetPin(1, 9);
	g_softI2C.pin_data = Tokenizer_GetPin(2, 14);
	channel_temp = Tokenizer_GetArgIntegerDefault(3, -1);
	channel_humid = Tokenizer_GetArgIntegerDefault(4, -1);
	g_aht_requestedProfile = AHT_PROFILE_AUTO;
	if(Tokenizer_GetArgsCount() > 5)
	{
		g_aht_requestedProfile = AHTXX_ParseProfile(Tokenizer_GetArg(5));
	}
	g_aht_crcEnabled = (g_aht_requestedProfile == AHT_PROFILE_AHT3X);

	if(!Soft_I2C_PreInit(&g_softI2C))
	{
		g_aht_lastError = AHT_ERR_I2C_PREINIT;
		isWorking = false;
		ADDLOG_INFO(LOG_FEATURE_SENSOR, "AHTXX: I2C preinit failed, check SCL/SDA pins.");
		AHTXX_RegisterCommands();
		return;
	}
	rtos_delay_milliseconds(100);

	AHT2X_Initialization();
	AHTXX_RegisterCommands();
}

void AHT2X_OnEverySecond(void)
{
	if(g_aht_secondsUntilNextMeasurement <= 0)
	{
		if(isWorking)
		{
			AHT2X_Measure();
		}
		else
		{
			AHT2X_Initialization();
		}
		g_aht_secondsUntilNextMeasurement = g_aht_secondsBetweenMeasurements;
	}
	if(g_aht_secondsUntilNextMeasurement > 0)
	{
		g_aht_secondsUntilNextMeasurement--;
	}
}

void AHT2X_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState)
{
	if (bPreState)
		return;
	hprintf255(request, "<h2>AHTXX/AHT2X Temperature=%.1fC, Humidity=%.0f%% [%s]</h2>",
		g_temp, g_humid, AHTXX_ProtocolName(g_aht_activeProtocol));
	if(!isWorking)
	{
		hprintf255(request, "WARNING: AHT sensor is offline or failed initialization. Profile=%s, LastStatus=0x%02X, Error=%s. ",
			AHTXX_ProfileName(g_aht_requestedProfile), g_aht_lastStatus, AHTXX_ErrorName(g_aht_lastError));
	}
	if(channel_temp < 0 && channel_humid < 0)
	{
		hprintf255(request, "WARNING: AHT target channels are not configured; set temp and humidity channel indexes in Pins or startDriver arguments. ");
	}
	else if(channel_humid == channel_temp)
	{
		hprintf255(request, "WARNING: AHT temp and humidity target channels are the same; configure separate channel indexes. ");
	}
}
