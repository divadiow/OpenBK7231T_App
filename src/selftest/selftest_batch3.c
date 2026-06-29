#ifdef WINDOWS

#include "selftest_local.h"
#include "../driver/drv_local.h"
#include "../driver/drv_aht2x.h"

#if ENABLE_DRIVER_MCP9808
void MCP9808_Measure(void);
#endif

static void Batch3_AssertSoftI2CWriteSequence(const byte *expected, int count) {
	SELFTEST_ASSERT(SIM_SoftI2C_GetWriteCount() >= count);
	for (int i = 0; i < count; i++) {
		SELFTEST_ASSERT(SIM_SoftI2C_GetWriteByte(i) == expected[i]);
	}
}

static uint8_t Batch3_SHT3X_Crc(const byte *data) {
	uint8_t crc = 0xFF;
	for (int i = 0; i < 2; i++) {
		crc ^= data[i];
		for (int bit = 8; bit > 0; bit--) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31;
			}
			else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

static void Batch3_BuildSHT3XPacket(float temperature, float humidity, byte *out) {
	uint16_t rawTemperature = (uint16_t)(((temperature + 45.0f) * 65535.0f / 175.0f) + 0.5f);
	uint16_t rawHumidity = (uint16_t)((humidity * 65535.0f / 100.0f) + 0.5f);

	out[0] = (byte)(rawTemperature >> 8);
	out[1] = (byte)(rawTemperature);
	out[2] = Batch3_SHT3X_Crc(out);
	out[3] = (byte)(rawHumidity >> 8);
	out[4] = (byte)(rawHumidity);
	out[5] = Batch3_SHT3X_Crc(out + 3);
}

static int Batch3_SHT3X_TempChannelFromPacket(const byte *packet) {
	unsigned int raw = (packet[0] << 8) | packet[1];
	float temperature = 175.0f * (raw / 65535.0f) - 45.0f;
	temperature = (int)(temperature * 10.0f) / 10.0f;
	return (int)(temperature * 10.0f);
}

static int Batch3_SHT3X_HumChannelFromPacket(const byte *packet) {
	unsigned int raw = (packet[3] << 8) | packet[4];
	float humidity = 100.0f * (raw / 65535.0f);
	humidity = (int)humidity;
	return (int)humidity;
}

static void Test_Batch3_SoftI2C_FakeReadWriteQueue(void) {
	softI2C_t bus;
	byte queued[] = { 0x12, 0x34, 0x56 };
	byte readback[2];

	memset(&bus, 0, sizeof(bus));
	bus.pin_clk = 1;
	bus.pin_data = 2;

	SIM_SoftI2C_Reset();
	SIM_SoftI2C_Enable(1);

	SELFTEST_ASSERT(Soft_I2C_PreInit(&bus));
	SELFTEST_ASSERT(Soft_I2C_Start(&bus, 0x70));
	SELFTEST_ASSERT(Soft_I2C_WriteByte(&bus, 0xAA));
	SELFTEST_ASSERT(Soft_I2C_WriteByte(&bus, 0x55));
	Soft_I2C_Stop(&bus);

	SIM_SoftI2C_QueueReadBytes(queued, sizeof(queued));
	SELFTEST_ASSERT(Soft_I2C_Start(&bus, 0x71));
	SELFTEST_ASSERT(Soft_I2C_ReadByte(&bus, false) == 0x12);
	Soft_I2C_ReadBytes(&bus, readback, sizeof(readback));
	Soft_I2C_Stop(&bus);

	SELFTEST_ASSERT(readback[0] == 0x34);
	SELFTEST_ASSERT(readback[1] == 0x56);
	SELFTEST_ASSERT(SIM_SoftI2C_GetReadUnderflowCount() == 0);
	SELFTEST_ASSERT(SIM_SoftI2C_GetWriteCount() == 2);
	SELFTEST_ASSERT(SIM_SoftI2C_GetWriteByte(0) == 0xAA);
	SELFTEST_ASSERT(SIM_SoftI2C_GetWriteByte(1) == 0x55);
	SELFTEST_ASSERT(SIM_SoftI2C_GetStartCount() == 2);
	SELFTEST_ASSERT(SIM_SoftI2C_GetStartAddress(0) == 0x70);
	SELFTEST_ASSERT(SIM_SoftI2C_GetStartAddress(1) == 0x71);

	SIM_SoftI2C_Reset();
}

#if ENABLE_DRIVER_MCP9808
static void Test_Batch3_MCP9808_PositiveNegativeTemps(void) {
	byte positive25C[] = { 0x01, 0x90 };
	byte negative10p5C[] = { 0x1F, 0x58 };

	SIM_ClearOBK(0);
	SIM_SoftI2C_Reset();
	SIM_SoftI2C_Enable(1);

	SELFTEST_ASSERT_CMD_OK("setChannelType 5 Temperature_div10");
	SELFTEST_ASSERT_CMD_OK("startDriver MCP9808 26 24 5");

	SIM_SoftI2C_QueueReadBytes(positive25C, sizeof(positive25C));
	MCP9808_Measure();
	SELFTEST_ASSERT_CHANNEL(5, 250);
	SELFTEST_ASSERT(SIM_SoftI2C_GetReadUnderflowCount() == 0);
	SELFTEST_ASSERT(SIM_SoftI2C_GetWriteByte(SIM_SoftI2C_GetWriteCount() - 1) == 0x05);

	SIM_SoftI2C_QueueReadBytes(negative10p5C, sizeof(negative10p5C));
	MCP9808_Measure();
	SELFTEST_ASSERT_CHANNEL(5, -105);
	SELFTEST_ASSERT(SIM_SoftI2C_GetReadUnderflowCount() == 0);

	SIM_SoftI2C_Reset();
}

static void Test_Batch3_MCP9808_AlertRangeWriteSequence(void) {
	byte configRegZero[] = { 0x00, 0x00 };
	byte expectedWrites[] = {
		0x03, 0x01, 0x40,
		0x02, 0x01, 0xA0,
		0x04, 0x01, 0xA0,
		0x01,
		0x01, 0x00, 0x08
	};

	SIM_ClearOBK(0);
	SIM_SoftI2C_Reset();
	SIM_SoftI2C_Enable(1);

	SELFTEST_ASSERT_CMD_OK("startDriver MCP9808 26 24 5");
	SIM_SoftI2C_QueueReadBytes(configRegZero, sizeof(configRegZero));
	SELFTEST_ASSERT_CMD_OK("MCP9808_AlertRange 20 26 1");

	Batch3_AssertSoftI2CWriteSequence(expectedWrites, sizeof(expectedWrites));
	SELFTEST_ASSERT(SIM_SoftI2C_GetReadUnderflowCount() == 0);

	SIM_SoftI2C_Reset();
}
#endif

#if ENABLE_DRIVER_AHT2X
static void Test_Batch3_AHT2X_ValidAndUnrealisticHumidity(void) {
	byte initOk[] = { 0x08 };
	byte sample50C50H[] = { 0x00, 0x80, 0x00, 0x08, 0x00, 0x00 };
	byte unrealisticZeroHumidity[] = { 0x00, 0x00, 0x00, 0x08, 0x00, 0x00 };
	byte expectedPrefix[] = {
		AHT2X_CMD_RST,
		AHT2X_CMD_INI, AHT2X_DAT_INI1, AHT2X_DAT_INI2,
		AHT2X_CMD_TMS, AHT2X_DAT_TMS1, AHT2X_DAT_TMS2
	};

	SIM_ClearOBK(0);
	SIM_SoftI2C_Reset();
	SIM_SoftI2C_Enable(1);
	SIM_SoftI2C_QueueReadBytes(initOk, sizeof(initOk));

	SELFTEST_ASSERT_CMD_OK("startDriver AHT2X 4 5 6 7");

	SIM_SoftI2C_QueueReadBytes(sample50C50H, sizeof(sample50C50H));
	SELFTEST_ASSERT_CMD_OK("AHT2X_Measure");
	SELFTEST_ASSERT_CHANNEL(6, 500);
	SELFTEST_ASSERT_CHANNEL(7, 50);
	Batch3_AssertSoftI2CWriteSequence(expectedPrefix, sizeof(expectedPrefix));

	SIM_SoftI2C_QueueReadBytes(unrealisticZeroHumidity, sizeof(unrealisticZeroHumidity));
	SELFTEST_ASSERT_CMD_OK("AHT2X_Measure");
	SELFTEST_ASSERT_CHANNEL(6, 500);
	SELFTEST_ASSERT_CHANNEL(7, 50);
	SELFTEST_ASSERT(SIM_SoftI2C_GetReadUnderflowCount() == 0);

	SIM_SoftI2C_Reset();
}
#endif

#if ENABLE_DRIVER_SHT3X
static void Test_Batch3_SHT3X_ValidReadingAndBadCRC(void) {
	byte statusOk[] = { 0x00, 0x00 };
	byte sample[6];
	byte badSample[6];
	int expectedTemp;
	int expectedHum;

	Batch3_BuildSHT3XPacket(22.5f, 55.0f, sample);
	memcpy(badSample, sample, sizeof(sample));
	badSample[2] ^= 0x01;
	expectedTemp = Batch3_SHT3X_TempChannelFromPacket(sample);
	expectedHum = Batch3_SHT3X_HumChannelFromPacket(sample);

	SIM_ClearOBK(0);
	PIN_SetPinRoleForPinIndex(4, IOR_SHT3X_CLK);
	PIN_SetPinRoleForPinIndex(5, IOR_SHT3X_DAT);
	PIN_SetPinChannelForPinIndex(5, 9);
	PIN_SetPinChannel2ForPinIndex(5, 10);

	SIM_SoftI2C_Reset();
	SIM_SoftI2C_Enable(1);
	SIM_SoftI2C_QueueReadBytes(statusOk, sizeof(statusOk));
	SELFTEST_ASSERT_CMD_OK("startDriver SHT3X");

	SIM_SoftI2C_QueueReadBytes(sample, sizeof(sample));
	SELFTEST_ASSERT_CMD_OK("SHT_Measure");
	SELFTEST_ASSERT_CHANNEL(9, expectedTemp);
	SELFTEST_ASSERT_CHANNEL(10, expectedHum);

	SIM_SoftI2C_QueueReadBytes(badSample, sizeof(badSample));
	SELFTEST_ASSERT_CMD_OK("SHT_Measure");
	SELFTEST_ASSERT_CHANNEL(9, expectedTemp);
	SELFTEST_ASSERT_CHANNEL(10, expectedHum);
	SELFTEST_ASSERT(SIM_SoftI2C_GetReadUnderflowCount() == 0);

	SIM_SoftI2C_Reset();
}
#endif

void Test_Batch3(void) {
	Test_Batch3_SoftI2C_FakeReadWriteQueue();
#if ENABLE_DRIVER_MCP9808
	Test_Batch3_MCP9808_PositiveNegativeTemps();
	Test_Batch3_MCP9808_AlertRangeWriteSequence();
#endif
#if ENABLE_DRIVER_AHT2X
	Test_Batch3_AHT2X_ValidAndUnrealisticHumidity();
#endif
#if ENABLE_DRIVER_SHT3X
	Test_Batch3_SHT3X_ValidReadingAndBadCRC();
#endif
}

#endif
