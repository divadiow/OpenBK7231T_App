#ifdef WINDOWS

#include "selftest_local.h"

#if ENABLE_BL_SHARED

#include <math.h>
#include <string.h>

#include "../driver/drv_bl0942.h"
#include "../driver/drv_cse7766.h"
#include "../driver/drv_public.h"
#include "../driver/drv_test_drivers.h"
#include "../driver/drv_uart.h"

#define BATCH2_BL0942_PACKET_LEN 23
#define BATCH2_BL0942_READ_COMMAND 0x58
#define BATCH2_BL0942_UREF 15188.0f
#define BATCH2_BL0942_IREF 251210.0f
#define BATCH2_BL0942_PREF 598.0f

static const char *g_batch2CSEPacket70W240V = "555A02FCD800062F00413200D7F2537B18023E9F7171FEEC";
static const char *g_batch2CSEPacket70W240V_PartA = "555A02FCD800062F00413200";
static const char *g_batch2CSEPacket70W240V_PartB = "D7F2537B18023E9F7171FEEC";
static const char *g_batch2CSEPacket70W240V_BadChecksum = "555A02FCD800062F00413200D7F2537B18023E9F7171FEED";

static void Batch2_SetupMQTTPowerDevice(const char *clientName) {
	SIM_ClearOBK(0);
	SIM_ClearAndPrepareForMQTTTesting(clientName, "bekens");

	PIN_SetPinRoleForPinIndex(9, IOR_Relay);
	PIN_SetPinChannelForPinIndex(9, 1);
	SIM_ClearUART();
}

static void Batch2_AppendHexStringToUART(const char *hex) {
	while (*hex) {
		if (*hex == ' ' || *hex == '\t' || *hex == '\r' || *hex == '\n') {
			hex++;
			continue;
		}
		UART_AppendByteToReceiveRingBuffer(hexbyte(hex));
		hex += 2;
	}
}

#if ENABLE_DRIVER_CSE7766
static void Batch2_SetupCalibratedCSE7766(void) {
	Batch2_SetupMQTTPowerDevice("batch2CSE");

	SELFTEST_ASSERT_CMD_OK("startDriver CSE7766");
	SELFTEST_ASSERT_CMD_OK(va("uartFakeHex %s", g_batch2CSEPacket70W240V));
	CSE7766_RunEverySecond();
	SELFTEST_ASSERT(UART_GetDataSize() == 0);

	SELFTEST_ASSERT_CMD_OK("VoltageSet 240");
	SELFTEST_ASSERT_CMD_OK("CurrentSet 0.30");
	SELFTEST_ASSERT_CMD_OK("PowerSet 70");
	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 0 1");
	SIM_ClearMQTTHistory();
}

static void Test_Batch2_CSE7766_InvalidChecksum(void) {
	Batch2_SetupCalibratedCSE7766();

	float oldVoltage = CMD_EvaluateExpression("$voltage", 0);
	float oldCurrent = CMD_EvaluateExpression("$current", 0);
	float oldPower = CMD_EvaluateExpression("$power", 0);

	SELFTEST_ASSERT_CMD_OK(va("uartFakeHex %s", g_batch2CSEPacket70W240V_BadChecksum));
	CSE7766_RunEverySecond();

	SELFTEST_ASSERT(UART_GetDataSize() == 0);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$voltage", 0), oldVoltage, 0.001f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$current", 0), oldCurrent, 0.001f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$power", 0), oldPower, 0.001f);
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2CSE/voltage/get", 240.0f, false));
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2CSE/current/get", 0.30f, false));
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2CSE/power/get", 70.0f, false));

	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");
	SIM_ClearUART();
	SIM_ClearMQTTHistory();
}

static void Test_Batch2_CSE7766_GarbageBeforeFrame(void) {
	Batch2_SetupCalibratedCSE7766();

	SELFTEST_ASSERT_CMD_OK(va("uartFakeHex 00 FF AA 13 %s", g_batch2CSEPacket70W240V));
	CSE7766_RunEverySecond();

	SELFTEST_ASSERT(UART_GetDataSize() == 0);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$voltage", 0), 240.0f, 0.5f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$current", 0), 0.30f, 0.01f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$power", 0), 70.0f, 0.5f);

	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");
	SIM_ClearUART();
	SIM_ClearMQTTHistory();
}

static void Test_Batch2_CSE7766_PartialFrameRetention(void) {
	Batch2_SetupCalibratedCSE7766();

	SELFTEST_ASSERT_CMD_OK(va("uartFakeHex %s", g_batch2CSEPacket70W240V_PartA));
	CSE7766_RunEverySecond();
	SELFTEST_ASSERT(UART_GetDataSize() == 12);
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2CSE/voltage/get", 240.0f, false));
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2CSE/current/get", 0.30f, false));
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2CSE/power/get", 70.0f, false));

	SELFTEST_ASSERT_CMD_OK(va("uartFakeHex %s", g_batch2CSEPacket70W240V_PartB));
	CSE7766_RunEverySecond();
	SELFTEST_ASSERT(UART_GetDataSize() == 0);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$voltage", 0), 240.0f, 0.5f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$current", 0), 0.30f, 0.01f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$power", 0), 70.0f, 0.5f);

	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");
	SIM_ClearUART();
	SIM_ClearMQTTHistory();
}
#endif

#if ENABLE_DRIVER_BL0942
static void Batch2_BuildBL0942Packet(byte *data, float voltage, float current, float power, float frequency, unsigned int cfCnt, bool badChecksum) {
	int i;
	int blCurrent = (int)(BATCH2_BL0942_IREF * current);
	int blVoltage = (int)(BATCH2_BL0942_UREF * voltage);
	int blPower = (int)(BATCH2_BL0942_PREF * power);
	int blFrequency = (int)((2.0f * 500000.0f / frequency) + 0.5f);
	byte checksum = BATCH2_BL0942_READ_COMMAND;

	memset(data, 0, BATCH2_BL0942_PACKET_LEN);
	data[0] = 0x55;

	data[1] = (byte)(blCurrent);
	data[2] = (byte)(blCurrent >> 8);
	data[3] = (byte)(blCurrent >> 16);

	data[4] = (byte)(blVoltage);
	data[5] = (byte)(blVoltage >> 8);
	data[6] = (byte)(blVoltage >> 16);

	data[10] = (byte)(blPower);
	data[11] = (byte)(blPower >> 8);
	data[12] = (byte)(blPower >> 16);

	data[13] = (byte)(cfCnt);
	data[14] = (byte)(cfCnt >> 8);
	data[15] = (byte)(cfCnt >> 16);

	data[16] = (byte)(blFrequency);
	data[17] = (byte)(blFrequency >> 8);

	for (i = 0; i < BATCH2_BL0942_PACKET_LEN - 1; i++) {
		checksum += data[i];
	}
	checksum ^= 0xFF;
	if (badChecksum) {
		checksum ^= 0x01;
	}
	data[BATCH2_BL0942_PACKET_LEN - 1] = checksum;
}

static void Batch2_AppendBL0942Packet(float voltage, float current, float power, float frequency, unsigned int cfCnt, bool badChecksum) {
	byte data[BATCH2_BL0942_PACKET_LEN];
	Batch2_BuildBL0942Packet(data, voltage, current, power, frequency, cfCnt, badChecksum);
	for (int i = 0; i < BATCH2_BL0942_PACKET_LEN; i++) {
		UART_AppendByteToReceiveRingBuffer(data[i]);
	}
}

static void Batch2_AppendBL0942PacketRange(float voltage, float current, float power, float frequency, unsigned int cfCnt, int start, int count) {
	byte data[BATCH2_BL0942_PACKET_LEN];
	Batch2_BuildBL0942Packet(data, voltage, current, power, frequency, cfCnt, false);
	for (int i = 0; i < count; i++) {
		UART_AppendByteToReceiveRingBuffer(data[start + i]);
	}
}

static void Batch2_SetupBL0942(void) {
	Batch2_SetupMQTTPowerDevice("batch2BL");
	SELFTEST_ASSERT_CMD_OK("startDriver BL0942");
	SIM_ClearUART();
	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 0 1");
	SIM_ClearMQTTHistory();
}

static void Test_Batch2_BL0942_InvalidChecksum(void) {
	Batch2_SetupBL0942();

	Batch2_AppendBL0942Packet(230.0f, 0.250f, 57.5f, 50.0f, 10, false);
	BL0942_UART_RunEverySecond();
	SELFTEST_ASSERT(UART_GetDataSize() == 0);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$voltage", 0), 230.0f, 0.2f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$current", 0), 0.250f, 0.002f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$power", 0), 57.5f, 0.2f);
	SIM_ClearMQTTHistory();

	Batch2_AppendBL0942Packet(250.0f, 1.000f, 250.0f, 55.0f, 20, true);
	BL0942_UART_RunEverySecond();
	SELFTEST_ASSERT(UART_GetDataSize() == 0);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$voltage", 0), 230.0f, 0.2f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$current", 0), 0.250f, 0.002f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$power", 0), 57.5f, 0.2f);
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2BL/voltage/get", 250.0f, false));
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2BL/current/get", 1.000f, false));
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2BL/power/get", 250.0f, false));

	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");
	SIM_ClearUART();
	SIM_ClearMQTTHistory();
}

static void Test_Batch2_BL0942_GarbageAndRecovery(void) {
	Batch2_SetupBL0942();

	Batch2_AppendHexStringToUART("00 12 34 AA FE");
	Batch2_AppendBL0942Packet(231.0f, 0.321f, 74.2f, 49.8f, 11, false);
	BL0942_UART_RunEverySecond();

	SELFTEST_ASSERT(UART_GetDataSize() == 0);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$voltage", 0), 231.0f, 0.2f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$current", 0), 0.321f, 0.002f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$power", 0), 74.2f, 0.2f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$frequency", 0), 49.8f, 0.05f);

	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");
	SIM_ClearUART();
	SIM_ClearMQTTHistory();
}

static void Test_Batch2_BL0942_PartialFrameRetention(void) {
	Batch2_SetupBL0942();

	Batch2_AppendBL0942PacketRange(232.0f, 0.123f, 28.5f, 50.0f, 12, 0, 9);
	BL0942_UART_RunEverySecond();
	SELFTEST_ASSERT(UART_GetDataSize() == 9);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$voltage", 0), 0.0f, 0.001f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$current", 0), 0.0f, 0.001f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$power", 0), 0.0f, 0.001f);
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2BL/voltage/get", 232.0f, false));

	Batch2_AppendBL0942PacketRange(232.0f, 0.123f, 28.5f, 50.0f, 12, 9, BATCH2_BL0942_PACKET_LEN - 9);
	BL0942_UART_RunEverySecond();
	SELFTEST_ASSERT(UART_GetDataSize() == 0);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$voltage", 0), 232.0f, 0.2f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$current", 0), 0.123f, 0.002f);
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$power", 0), 28.5f, 0.2f);

	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");
	SIM_ClearUART();
	SIM_ClearMQTTHistory();
}

static void Test_Batch2_BL0942_Frequency(void) {
	Batch2_SetupBL0942();

	Batch2_AppendBL0942Packet(230.0f, 0.300f, 69.0f, 50.00f, 100, false);
	BL0942_UART_RunEverySecond();
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$frequency", 0), 50.00f, 0.05f);

	Batch2_AppendBL0942Packet(230.0f, 0.300f, 69.0f, 60.00f, 110, false);
	BL0942_UART_RunEverySecond();
	SELFTEST_ASSERT_FLOATCOMPAREEPSILON(CMD_EvaluateExpression("$frequency", 0), 60.00f, 0.05f);

	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");
	SIM_ClearUART();
	SIM_ClearMQTTHistory();
}

static void Test_Batch2_BL0942_EnergyCounterDeltaAndWrap(void) {
	Batch2_SetupBL0942();

	Batch2_AppendBL0942Packet(230.0f, 0.500f, 115.0f, 50.0f, 0xFFFFF0, false);
	BL0942_UART_RunEverySecond();
	float firstEnergy = DRV_GetReading(OBK_CONSUMPTION_TOTAL);

	Batch2_AppendBL0942Packet(230.0f, 0.500f, 115.0f, 50.0f, 0xFFFFF5, false);
	BL0942_UART_RunEverySecond();
	float secondEnergy = DRV_GetReading(OBK_CONSUMPTION_TOTAL);
	SELFTEST_ASSERT(secondEnergy > firstEnergy);

	Batch2_AppendBL0942Packet(230.0f, 0.500f, 115.0f, 50.0f, 0x000002, false);
	BL0942_UART_RunEverySecond();
	float wrappedEnergy = DRV_GetReading(OBK_CONSUMPTION_TOTAL);
	SELFTEST_ASSERT(wrappedEnergy > secondEnergy);
	SELFTEST_ASSERT(wrappedEnergy < secondEnergy + 10.0f);

	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");
	SIM_ClearUART();
	SIM_ClearMQTTHistory();
}
#endif

#if ENABLE_DRIVER_TESTPOWER
static void Test_Batch2_BLShared_PublishIntervals(void) {
	Batch2_SetupMQTTPowerDevice("batch2Pub");

	SELFTEST_ASSERT_CMD_OK("startDriver TESTPOWER");
	SELFTEST_ASSERT_CMD_OK("SetupTestPower 230 0.50 115 50.00 0");
	SELFTEST_ASSERT_CMD_OK("VCPPublishThreshold 0.25 0.002 0.25 0.1 0.02");
	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");

	Sim_RunSeconds(8, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_FLOAT("batch2Pub/voltage/get", 230.0f, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_FLOAT("batch2Pub/current/get", 0.50f, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_FLOAT("batch2Pub/power/get", 115.0f, false);

	SIM_ClearMQTTHistory();
	SELFTEST_ASSERT_CMD_OK("SetupTestPower 240 0.60 144 50.50 0");
	Test_Power_RunEverySecond();
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2Pub/voltage/get", 240.0f, false));
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2Pub/current/get", 0.60f, false));
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForFloat("batch2Pub/power/get", 144.0f, false));

	Sim_RunSeconds(6, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_FLOAT("batch2Pub/voltage/get", 240.0f, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_FLOAT("batch2Pub/current/get", 0.60f, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_FLOAT("batch2Pub/power/get", 144.0f, false);

	SELFTEST_ASSERT_CMD_OK("VCPPublishIntervals 5 60");
	SIM_ClearUART();
	SIM_ClearMQTTHistory();
}
#endif

void Test_Batch2(void) {
#if ENABLE_DRIVER_CSE7766
	Test_Batch2_CSE7766_InvalidChecksum();
	Test_Batch2_CSE7766_GarbageBeforeFrame();
	Test_Batch2_CSE7766_PartialFrameRetention();
#endif
#if ENABLE_DRIVER_BL0942
	Test_Batch2_BL0942_InvalidChecksum();
	Test_Batch2_BL0942_GarbageAndRecovery();
	Test_Batch2_BL0942_PartialFrameRetention();
	Test_Batch2_BL0942_Frequency();
	Test_Batch2_BL0942_EnergyCounterDeltaAndWrap();
#endif
#if ENABLE_DRIVER_TESTPOWER
	Test_Batch2_BLShared_PublishIntervals();
#endif
}

#endif
#endif
