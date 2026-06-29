#ifdef WINDOWS

#include "selftest_local.h"

#if ENABLE_DRIVER_TUYAMCU
static void Batch5_SetupTuyaValueChannel(void) {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	SELFTEST_ASSERT_CMD_OK("startDriver TuyaMCU");
	SELFTEST_ASSERT_CMD_OK("linkTuyaMCUOutputToChannel 2 val 15");
	SELFTEST_ASSERT_CHANNEL(15, 0);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

static void Test_Batch5_TuyaMCU_BadChecksumIgnoredThenRecovery(void) {
	Batch5_SetupTuyaValueChannel();

	SELFTEST_ASSERT_CMD_OK("uartFakeHex 55AA0307000802020004000000647E");
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(15, 0);

	SELFTEST_ASSERT_CMD_OK("uartFakeHex 55AA0307000802020004000000647D");
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(15, 100);
}

static void Test_Batch5_TuyaMCU_GarbageBeforeFrameRecovery(void) {
	Batch5_SetupTuyaValueChannel();

	SELFTEST_ASSERT_CMD_OK("uartFakeHex 00 FF 13 37 55AA03070008020200040000005A73");
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(15, 90);
}

static void Test_Batch5_TuyaMCU_PartialFrameRetention(void) {
	Batch5_SetupTuyaValueChannel();

	SELFTEST_ASSERT_CMD_OK("uartFakeHex 55AA0307000802020004");
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(15, 0);

	SELFTEST_ASSERT_CMD_OK("uartFakeHex 000000647D");
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(15, 100);
}
#endif

void Test_Batch5(void) {
#if ENABLE_DRIVER_TUYAMCU
	Test_Batch5_TuyaMCU_BadChecksumIgnoredThenRecovery();
	Test_Batch5_TuyaMCU_GarbageBeforeFrameRecovery();
	Test_Batch5_TuyaMCU_PartialFrameRetention();
#endif
}

#endif
