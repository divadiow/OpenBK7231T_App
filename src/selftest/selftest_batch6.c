#ifdef WINDOWS

#include "selftest_local.h"

#if ENABLE_DRIVER_HTTPBUTTONS
static void Test_Batch6_HTTPButtons_ValidDisabledAndInvalidAct(void) {
	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("startDriver httpButtons");
	SELFTEST_ASSERT_CMD_OK("setButtonEnabled 0 1");
	SELFTEST_ASSERT_CMD_OK("setButtonLabel 0 RunBatch6");
	SELFTEST_ASSERT_CMD_OK("setButtonCommand 0 \"setChannel 3 77\"");
	SELFTEST_ASSERT_CMD_OK("setButtonEnabled 1 0");
	SELFTEST_ASSERT_CMD_OK("setButtonLabel 1 DisabledBatch6");
	SELFTEST_ASSERT_CMD_OK("setButtonCommand 1 \"setChannel 3 99\"");

	Test_FakeHTTPClientPacket_GET("index?act=0");
	SELFTEST_ASSERT_CHANNEL(3, 77);
	SELFTEST_ASSERT_HTML_REPLY_CONTAINS("Will do action RunBatch6");

	SELFTEST_ASSERT_CMD_OK("setChannel 3 0");
	Test_FakeHTTPClientPacket_GET("index?act=1");
	SELFTEST_ASSERT_CHANNEL(3, 0);
	SELFTEST_ASSERT_HTML_REPLY_CONTAINS("No HTTP button action 1");

	Test_FakeHTTPClientPacket_GET("index?act=99");
	SELFTEST_ASSERT_CHANNEL(3, 0);
	SELFTEST_ASSERT_HTML_REPLY_CONTAINS("No HTTP button action 99");
}
#endif

static void Test_Batch6_PWMToggler_HTTPControlPath(void) {
	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("startDriver PWMToggler");
	SELFTEST_ASSERT_CMD_OK("toggler_channel0 6");
	SELFTEST_ASSERT_CMD_OK("toggler_name0 HttpAux");
	SELFTEST_ASSERT_CMD_OK("toggler_set0 70");

	Test_FakeHTTPClientPacket_GET("index?togglerOn=0");
	SELFTEST_ASSERT_CHANNEL(6, 70);
	SELFTEST_ASSERT_HTML_REPLY_CONTAINS("Toggled HttpAux");

	Test_FakeHTTPClientPacket_GET("index?togglerValueID=0&togglerValue=25");
	SELFTEST_ASSERT_CHANNEL(6, 25);
	SELFTEST_ASSERT_HTML_REPLY_CONTAINS("Set value 25 for HttpAux");

	Test_FakeHTTPClientPacket_GET("index?togglerOn=0");
	SELFTEST_ASSERT_CHANNEL(6, 0);
}

#if ENABLE_DRIVER_PINMUTEX
static void Test_Batch6_PinMutex_InvalidConfigRejected(void) {
	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("startDriver PinMutex");
	SELFTEST_ASSERT_CMD_ERROR("setMutex 9 0 50 10 11");
	SELFTEST_ASSERT_CMD_ERROR("setMutex 0 0 -1 10 11");
	SELFTEST_ASSERT_CMD_ERROR("setMutex 0 0 50 NotAPin 11");
	SELFTEST_ASSERT_CMD_OK("setMutex 0 0 25 10 11");
}
#endif

void Test_Batch6(void) {
#if ENABLE_DRIVER_HTTPBUTTONS
	Test_Batch6_HTTPButtons_ValidDisabledAndInvalidAct();
#endif
	Test_Batch6_PWMToggler_HTTPControlPath();
#if ENABLE_DRIVER_PINMUTEX
	Test_Batch6_PinMutex_InvalidConfigRejected();
#endif
}

#endif
