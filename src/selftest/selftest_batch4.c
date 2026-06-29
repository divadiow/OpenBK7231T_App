#ifdef WINDOWS

#include "selftest_local.h"

#if ENABLE_DRIVER_DDP && ENABLE_DRIVER_SM16703P
bool Strip_VerifyPixel(uint32_t pixel, byte r, byte g, byte b);
void DDP_Parse(byte *data, int len);
#define BATCH4_ASSERT_PIXEL(index, r, g, b) SELFTEST_ASSERT(Strip_VerifyPixel(index, r, g, b))

static void Test_Batch4_DDP_ShortPacketIgnored(void) {
	byte ddpPacket[12];

	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("startDriver DMX");
	SELFTEST_ASSERT_CMD_OK("Strip_Init 3");
	SELFTEST_ASSERT_CMD_OK("SM16703P_SetPixel all 10 20 30");
	BATCH4_ASSERT_PIXEL(0, 10, 20, 30);
	BATCH4_ASSERT_PIXEL(1, 10, 20, 30);
	BATCH4_ASSERT_PIXEL(2, 10, 20, 30);

	memset(ddpPacket, 0, sizeof(ddpPacket));
	ddpPacket[10] = 255;
	ddpPacket[11] = 128;
	DDP_Parse(ddpPacket, sizeof(ddpPacket));

	BATCH4_ASSERT_PIXEL(0, 10, 20, 30);
	BATCH4_ASSERT_PIXEL(1, 10, 20, 30);
	BATCH4_ASSERT_PIXEL(2, 10, 20, 30);
}

static void Test_Batch4_DDP_ValidPacketAfterRejectedPacket(void) {
	byte shortPacket[12];
	byte validPacket[32];

	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("startDriver DMX");
	SELFTEST_ASSERT_CMD_OK("Strip_Init 3");
	SELFTEST_ASSERT_CMD_OK("SM16703P_SetPixel all 0 0 0");

	memset(shortPacket, 0, sizeof(shortPacket));
	shortPacket[10] = 255;
	DDP_Parse(shortPacket, sizeof(shortPacket));
	BATCH4_ASSERT_PIXEL(0, 0, 0, 0);

	memset(validPacket, 0, sizeof(validPacket));
	validPacket[10] = 1;
	validPacket[11] = 2;
	validPacket[12] = 3;
	validPacket[13] = 4;
	validPacket[14] = 5;
	validPacket[15] = 6;
	validPacket[16] = 7;
	validPacket[17] = 8;
	validPacket[18] = 9;
	DDP_Parse(validPacket, sizeof(validPacket));

	BATCH4_ASSERT_PIXEL(0, 1, 2, 3);
	BATCH4_ASSERT_PIXEL(1, 4, 5, 6);
	BATCH4_ASSERT_PIXEL(2, 7, 8, 9);
}
#endif

static void Test_Batch4_PWMToggler_DirectRelativeAndClamp(void) {
	SIM_ClearOBK(0);
	SIM_ClearAndPrepareForMQTTTesting("batch4Toggler", "bekens");
	SELFTEST_ASSERT_CMD_OK("startDriver PWMToggler");
	SELFTEST_ASSERT_CMD_OK("toggler_channel0 7");
	SELFTEST_ASSERT_CMD_OK("toggler_name0 Aux");

	SELFTEST_ASSERT_CMD_OK("toggler_set0 40");
	SELFTEST_ASSERT_CHANNEL(7, 0);
	SELFTEST_ASSERT_CMD_OK("toggler_enable0 1");
	SELFTEST_ASSERT_CHANNEL(7, 40);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_FLOAT("batch4Toggler/toggler_set0/get", 40.0f, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_FLOAT("batch4Toggler/toggler_enable0/get", 1.0f, false);

	SELFTEST_ASSERT_CMD_OK("toggler_set0 +80");
	SELFTEST_ASSERT_CHANNEL(7, 100);
	SELFTEST_ASSERT_CMD_OK("toggler_set0 -150");
	SELFTEST_ASSERT_CHANNEL(7, 0);
	SELFTEST_ASSERT_CMD_OK("toggler_set0 55");
	SELFTEST_ASSERT_CHANNEL(7, 55);
	SELFTEST_ASSERT_CMD_OK("toggler_enable0 0");
	SELFTEST_ASSERT_CHANNEL(7, 0);
	SELFTEST_ASSERT_CMD_OK("toggler_enable0 1");
	SELFTEST_ASSERT_CHANNEL(7, 55);
}

static void Test_Batch4_PWMToggler_ReinitClearsStaleState(void) {
	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("startDriver PWMToggler");
	SELFTEST_ASSERT_CMD_OK("toggler_channel0 8");
	SELFTEST_ASSERT_CMD_OK("toggler_set0 66");
	SELFTEST_ASSERT_CMD_OK("toggler_enable0 1");
	SELFTEST_ASSERT_CHANNEL(8, 66);

	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("startDriver PWMToggler");
	SELFTEST_ASSERT_CMD_OK("toggler_channel0 8");
	SELFTEST_ASSERT_CMD_OK("toggler_enable0 1");
	SELFTEST_ASSERT_CHANNEL(8, 0);
}

void Test_Batch4(void) {
#if ENABLE_DRIVER_DDP && ENABLE_DRIVER_SM16703P
	Test_Batch4_DDP_ShortPacketIgnored();
	Test_Batch4_DDP_ValidPacketAfterRejectedPacket();
#endif
	Test_Batch4_PWMToggler_DirectRelativeAndClamp();
	Test_Batch4_PWMToggler_ReinitClearsStaleState();
}

#endif
