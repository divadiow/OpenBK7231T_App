#ifdef WINDOWS

#include "selftest_local.h"

void SIM_OnMQTTPublish(const char *topic, const char *value, int len, int qos, bool bRetain);

static void Test_Batch8_MQTTHistoryRingOverflowKeepsNewest(void) {
	char topic[64];
	char value[32];

	SIM_ClearOBK(0);
	SIM_ClearMQTTHistory();
	for (int i = 0; i < 300; i++) {
		snprintf(topic, sizeof(topic), "batch8/history/%i", i);
		snprintf(value, sizeof(value), "%i", i);
		SIM_OnMQTTPublish(topic, value, strlen(value), 0, false);
	}

	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForString("batch8/history/0", "0", false));
	SELFTEST_ASSERT(SIM_CheckMQTTHistoryForString("batch8/history/299", "299", false));
	SELFTEST_ASSERT(SIM_CheckMQTTHistoryForString("batch8/history/200", "200", false));
}

static void Test_Batch8_BacklogBoundedNestingFinalState(void) {
	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("backlog setChannel 1 1; backlog setChannel 1 2; setChannel 1 3");
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(1, 3);
}

static void Test_Batch8_TokenizerEscapedJSONStringCorpus(void) {
	Tokenizer_TokenizeString("SendPOST http://localhost:3000/ 3000 \"application/json\" \"{ \\\"a\\\":123, \\\"b\\\":77 }\" 0",
		TOKENIZER_ALLOW_QUOTES | TOKENIZER_ALLOW_ESCAPING_QUOTATIONS);

	SELFTEST_ASSERT_ARGUMENTS_COUNT(6);
	SELFTEST_ASSERT_ARGUMENT(0, "SendPOST");
	SELFTEST_ASSERT_ARGUMENT(1, "http://localhost:3000/");
	SELFTEST_ASSERT_ARGUMENT_INTEGER(2, 3000);
	SELFTEST_ASSERT_ARGUMENT(3, "application/json");
	SELFTEST_ASSERT_ARGUMENT(4, "{ \"a\":123, \"b\":77 }");
	SELFTEST_ASSERT_ARGUMENT_INTEGER(5, 0);
}

static void Test_Batch8_HTTPCommandLongInvalidInputRejected(void) {
	char cmd[512];

	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("setChannel 1 12");
	memset(cmd, 'X', sizeof(cmd));
	cmd[sizeof(cmd) - 1] = 0;
	Test_FakeHTTPClientPacket_POST_withJSONReply("api/cmnd", cmd);
	SELFTEST_ASSERT_CHANNEL(1, 12);
	SELFTEST_ASSERT_JSON_VALUE_INTEGER(0, "error", 501);
}

void Test_Batch8(void) {
	Test_Batch8_MQTTHistoryRingOverflowKeepsNewest();
	Test_Batch8_BacklogBoundedNestingFinalState();
	Test_Batch8_TokenizerEscapedJSONStringCorpus();
	Test_Batch8_HTTPCommandLongInvalidInputRejected();
}

#endif
