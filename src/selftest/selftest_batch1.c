#ifdef WINDOWS

#include "selftest_local.h"
#include "../bitmessage/bitmessage_public.h"
#include "../driver/drv_uart.h"
#include "../hal/hal_wifi.h"
#include "../httpclient/http_client.h"

static void Test_Batch1_UART_RingBuffer_Wraparound(void) {
	int i;
	byte expectedAfterWrap[] = { 0x14, 0x15, 0x20, 0x21, 0x22, 0x23 };
	byte expectedAfterOverflow[] = { 0x20, 0x21, 0x22, 0x23, 0x30, 0x31, 0x32 };

	SIM_ClearOBK(0);
	UART_InitReceiveRingBuffer(8);

	for (i = 0; i < 6; i++) {
		UART_AppendByteToReceiveRingBuffer(0x10 + i);
	}
	SELFTEST_ASSERT(UART_GetDataSize() == 6);
	for (i = 0; i < 6; i++) {
		SELFTEST_ASSERT(UART_GetByte(i) == (0x10 + i));
	}

	UART_ConsumeBytes(4);
	SELFTEST_ASSERT(UART_GetDataSize() == 2);
	SELFTEST_ASSERT(UART_GetByte(0) == 0x14);
	SELFTEST_ASSERT(UART_GetByte(1) == 0x15);

	for (i = 0; i < 4; i++) {
		UART_AppendByteToReceiveRingBuffer(0x20 + i);
	}
	SELFTEST_ASSERT(UART_GetDataSize() == 6);
	for (i = 0; i < (int)sizeof(expectedAfterWrap); i++) {
		SELFTEST_ASSERT(UART_GetByte(i) == expectedAfterWrap[i]);
	}

	for (i = 0; i < 3; i++) {
		UART_AppendByteToReceiveRingBuffer(0x30 + i);
	}
	SELFTEST_ASSERT(UART_GetDataSize() == 7);
	for (i = 0; i < (int)sizeof(expectedAfterOverflow); i++) {
		SELFTEST_ASSERT(UART_GetByte(i) == expectedAfterOverflow[i]);
	}

	UART_ConsumeBytes(UART_GetDataSize());
	SELFTEST_ASSERT(UART_GetDataSize() == 0);
}

static void Test_Batch1_UART_CommandFakeHex_PartialData(void) {
	SIM_ClearOBK(0);
	UART_InitReceiveRingBuffer(8);

	SELFTEST_ASSERT_CMD_OK("uartFakeHex A0 A1 A2");
	SELFTEST_ASSERT(UART_GetDataSize() == 3);
	SELFTEST_ASSERT(UART_GetByte(0) == 0xA0);
	SELFTEST_ASSERT(UART_GetByte(1) == 0xA1);
	SELFTEST_ASSERT(UART_GetByte(2) == 0xA2);

	SELFTEST_ASSERT_CMD_OK("uartFakeHex B0 B1");
	SELFTEST_ASSERT(UART_GetDataSize() == 5);
	SELFTEST_ASSERT(UART_GetByte(0) == 0xA0);
	SELFTEST_ASSERT(UART_GetByte(1) == 0xA1);
	SELFTEST_ASSERT(UART_GetByte(2) == 0xA2);
	SELFTEST_ASSERT(UART_GetByte(3) == 0xB0);
	SELFTEST_ASSERT(UART_GetByte(4) == 0xB1);

	UART_ConsumeBytes(2);
	SELFTEST_ASSERT(UART_GetDataSize() == 3);
	SELFTEST_ASSERT(UART_GetByte(0) == 0xA2);
	SELFTEST_ASSERT(UART_GetByte(1) == 0xB0);
	SELFTEST_ASSERT(UART_GetByte(2) == 0xB1);
}

static void Test_Batch1_SIM_UART_RingBuffer_Wraparound(void) {
	int i;
	byte expectedAfterWrap[] = { 0x44, 0x45, 0x50, 0x51, 0x52, 0x53 };

	SIM_UART_InitReceiveRingBuffer(8);
	for (i = 0; i < 6; i++) {
		SIM_AppendUARTByte(0x40 + i);
	}
	SELFTEST_ASSERT(SIM_UART_GetDataSize() == 6);
	for (i = 0; i < 6; i++) {
		SELFTEST_ASSERT(SIM_UART_GetByte(i) == (0x40 + i));
	}

	SIM_UART_ConsumeBytes(4);
	SELFTEST_ASSERT(SIM_UART_GetDataSize() == 2);
	SELFTEST_ASSERT(SIM_UART_GetByte(0) == 0x44);
	SELFTEST_ASSERT(SIM_UART_GetByte(1) == 0x45);

	for (i = 0; i < 4; i++) {
		SIM_AppendUARTByte(0x50 + i);
	}
	SELFTEST_ASSERT(SIM_UART_GetDataSize() == 6);
	for (i = 0; i < (int)sizeof(expectedAfterWrap); i++) {
		SELFTEST_ASSERT(SIM_UART_GetByte(i) == expectedAfterWrap[i]);
	}

	SIM_ClearUART();
	SELFTEST_ASSERT(SIM_UART_GetDataSize() == 0);
}

static void Test_Batch1_Command_ReturnContracts(void) {
	commandResult_t res;
	int oldRole;

	SIM_ClearOBK(0);
	SELFTEST_ASSERT(CMD_ExecuteCommand("", 0) == CMD_RES_EMPTY_STRING);
	SELFTEST_ASSERT(CMD_ExecuteCommand("   ", 0) == CMD_RES_EMPTY_STRING);
	SELFTEST_ASSERT(CMD_ExecuteCommand("NoSuchCommand_Batch1", 0) == CMD_RES_UNKNOWN_COMMAND);

	SELFTEST_ASSERT_CMD_OK("setChannel 4 10");
	SELFTEST_ASSERT_CHANNEL(4, 10);
	res = CMD_ExecuteCommand("setChannel", 0);
	SELFTEST_ASSERT(res == CMD_RES_NOT_ENOUGH_ARGUMENTS);
	SELFTEST_ASSERT_CHANNEL(4, 10);

	SELFTEST_ASSERT_CMD_OK("setPinRole 9 Relay");
	SELFTEST_ASSERT(PIN_GetPinRoleForPinIndex(9) == IOR_Relay);
	oldRole = PIN_GetPinRoleForPinIndex(9);
	res = CMD_ExecuteCommand("setPinRole 9 NotARealRole", 0);
	SELFTEST_ASSERT(res == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(PIN_GetPinRoleForPinIndex(9) == oldRole);
	res = CMD_ExecuteCommand("setPinRole", 0);
	SELFTEST_ASSERT(res == CMD_RES_NOT_ENOUGH_ARGUMENTS);
	SELFTEST_ASSERT(PIN_GetPinRoleForPinIndex(9) == oldRole);
}

static void Test_Batch1_MQTT_CommandTopic_Rejection(void) {
	SIM_ClearAndPrepareForMQTTTesting("batch1Device", "batch1Group");
	SELFTEST_ASSERT_CMD_OK("setChannel 1 0");
	SIM_ClearMQTTHistory();

	SIM_SendFakeMQTT("otherDevice/1/set", "1");
	SELFTEST_ASSERT_CHANNEL(1, 0);
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForString("batch1Device/1/get", "1", false));

	SIM_SendFakeMQTT("batch1Device/999/set", "1");
	SELFTEST_ASSERT_CHANNEL(1, 0);

	SELFTEST_ASSERT_CMD_OK("setChannel 0 0");
	SIM_SendFakeMQTT("batch1Device/notANumber/set", "1");
	SELFTEST_ASSERT_CHANNEL(0, 0);
	SELFTEST_ASSERT_CHANNEL(1, 0);
	SELFTEST_ASSERT_NO_MQTT_PUBLISH_STR("batch1Device/0/get", "1", false);

	SIM_SendFakeMQTT("batch1Device/1extra/set", "1");
	SELFTEST_ASSERT_CHANNEL(1, 0);

	SIM_SendFakeMQTT("batch1Device/1/notset", "1");
	SELFTEST_ASSERT_CHANNEL(1, 0);

	SIM_ClearMQTTHistory();
	SIM_SendFakeMQTT("batch1Device/1/get", "nonempty-payload");
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForString("batch1Device/1/get", "0", false));

	SIM_SendFakeMQTT("cmnd/otherDevice/setChannel", "1 1");
	SELFTEST_ASSERT_CHANNEL(1, 0);

	SIM_SendFakeMQTT("cmnd/batch1Device/setChannel", "1 1");
	SELFTEST_ASSERT_CHANNEL(1, 1);

	SELFTEST_ASSERT_CMD_OK("setChannel 1 0");
	SIM_SendFakeMQTT("cmnd/batch1Group/setChannel", "1 1");
	SELFTEST_ASSERT_CHANNEL(1, 1);
}

static void Test_Batch1_HTTP_Rest_InvalidInputs(void) {
	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("setChannel 1 77");
	PIN_SetPinRoleForPinIndex(9, IOR_Relay);

	Test_FakeHTTPClientPacket_POST_withJSONReply("api/cmnd", "setChannel");
	SELFTEST_ASSERT_CHANNEL(1, 77);
	SELFTEST_ASSERT_JSON_VALUE_INTEGER(0, "error", 400);

	Test_FakeHTTPClientPacket_POST_withJSONReply("api/cmnd", "NoSuchCommand_Batch1");
	SELFTEST_ASSERT_CHANNEL(1, 77);
	SELFTEST_ASSERT_JSON_VALUE_INTEGER(0, "error", 501);

	Test_FakeHTTPClientPacket_POST_withJSONReply("api/cmnd", "setPinRole 9 NotARealRole");
	SELFTEST_ASSERT(PIN_GetPinRoleForPinIndex(9) == IOR_Relay);
	SELFTEST_ASSERT_JSON_VALUE_INTEGER(0, "error", 400);

	Test_FakeHTTPClientPacket_POST_withJSONReply("api/cmnd", "setChannel 1 88");
	SELFTEST_ASSERT_CHANNEL(1, 88);
	SELFTEST_ASSERT_JSON_VALUE_INTEGER(0, "success", 200);
}


#if ENABLE_SEND_POSTANDGET
static void Test_Batch1_HTTP_Client_RequestContracts(void) {
	httprequest_t *request;
	int ret;

	SIM_ClearOBK(0);
	HTTPClient_Test_SetSkipAsyncThread(1);

	HTTPClient_Test_ClearLastRequest();
	ret = HTTPClient_Async_SendGet("http://127.0.0.1/batch1_get", "cmd", "SetChannel 9 19");
	request = HTTPClient_Test_GetLastRequest();
	SELFTEST_ASSERT(ret == 0);
	SELFTEST_ASSERT(request != 0);
	SELFTEST_ASSERT(request->method == HTTPCLIENT_GET);
	SELFTEST_ASSERT(request->port == 80);
	SELFTEST_ASSERT_STRING(request->url, "http://127.0.0.1/batch1_get");
	SELFTEST_ASSERT_STRING(request->targetFile, "cmd");
	SELFTEST_ASSERT_STRING(request->cmdToRun, "SetChannel 9 19");
	SELFTEST_ASSERT(request->client_data.response_buf != 0);
	SELFTEST_ASSERT(request->client_data.response_buf_len == 2048);
	HTTPClient_Test_FreeLastRequest();

	HTTPClient_Test_ClearLastRequest();
	ret = HTTPClient_Async_SendPost("http://127.0.0.1:8080/batch1_post", 8080, "application/json", "{\"a\":1}", "X-Test: 1\r\n");
	request = HTTPClient_Test_GetLastRequest();
	SELFTEST_ASSERT(ret == 0);
	SELFTEST_ASSERT(request != 0);
	SELFTEST_ASSERT(request->method == HTTPCLIENT_POST);
	SELFTEST_ASSERT(request->port == 8080);
	SELFTEST_ASSERT_STRING(request->url, "http://127.0.0.1:8080/batch1_post");
	SELFTEST_ASSERT_STRING(request->client_data.post_content_type, "application/json");
	SELFTEST_ASSERT_STRING(request->client_data.post_buf, "{\"a\":1}");
	SELFTEST_ASSERT(request->client_data.post_buf_len == strlen("{\"a\":1}"));
	SELFTEST_ASSERT_STRING(request->header, "X-Test: 1\r\n");
	HTTPClient_Test_FreeLastRequest();

	HTTPClient_Test_ClearLastRequest();
	HTTPClient_Test_SetSkipAsyncThread(0);
}
#endif

static void Test_Batch1_BitMessage_ReadWrite_Bounds(void) {
	bitMessage_t msg;
	byte buffer[8];
	byte patternA[] = { 0x10, 0x11, 0x12, 0x13 };
	byte patternB[] = { 0x20, 0x21, 0x22, 0x23 };
	char stringBuffer[5];

	memset(buffer, 0xEE, sizeof(buffer));
	MSG_BeginWriting(&msg, buffer, sizeof(buffer));
	SELFTEST_ASSERT(MSG_WriteBytes(&msg, patternA, sizeof(patternA)) == sizeof(patternA));
	SELFTEST_ASSERT(msg.position == 4);
	SELFTEST_ASSERT(MSG_WriteBytes(&msg, patternB, sizeof(patternB)) == sizeof(patternB));
	SELFTEST_ASSERT(msg.position == 8);
	SELFTEST_ASSERT(MSG_WriteByte(&msg, 0x99) == 0);
	SELFTEST_ASSERT(msg.position == 8);

	MSG_BeginReading(&msg, buffer, sizeof(buffer));
	SELFTEST_ASSERT(MSG_ReadByte(&msg) == 0x10);
	SELFTEST_ASSERT(MSG_ReadByte(&msg) == 0x11);
	SELFTEST_ASSERT(MSG_ReadByte(&msg) == 0x12);
	SELFTEST_ASSERT(MSG_ReadByte(&msg) == 0x13);
	SELFTEST_ASSERT(MSG_Read3Bytes(&msg) == 0x222120);
	SELFTEST_ASSERT(msg.position == 7);
	SELFTEST_ASSERT(MSG_ReadByte(&msg) == 0x23);
	SELFTEST_ASSERT(MSG_EOF(&msg) == 1);
	SELFTEST_ASSERT(MSG_ReadByte(&msg) == 0);
	SELFTEST_ASSERT(msg.position == 8);

	memset(buffer, 0, sizeof(buffer));
	MSG_BeginWriting(&msg, buffer, 5);
	SELFTEST_ASSERT(MSG_WriteString(&msg, "ABCD") == 5);
	SELFTEST_ASSERT(msg.position == 5);
	SELFTEST_ASSERT(MSG_WriteByte(&msg, 1) == 0);
	MSG_BeginReading(&msg, buffer, 5);
	SELFTEST_ASSERT(MSG_ReadString(&msg, stringBuffer, sizeof(stringBuffer)) == 4);
	SELFTEST_ASSERT_STRING(stringBuffer, "ABCD");
	SELFTEST_ASSERT(MSG_EOF(&msg) == 1);

	memset(buffer, 0, sizeof(buffer));
	MSG_BeginWriting(&msg, buffer, 2);
	SELFTEST_ASSERT(MSG_WriteU16(&msg, 0xBEEF) == 2);
	SELFTEST_ASSERT(msg.position == 2);
	SELFTEST_ASSERT(MSG_WriteByte(&msg, 0x99) == 0);
	SELFTEST_ASSERT(msg.position == 2);
	MSG_BeginReading(&msg, buffer, 2);
	SELFTEST_ASSERT(MSG_ReadU16(&msg) == 0xBEEF);
	SELFTEST_ASSERT(MSG_EOF(&msg) == 1);
}

static void Test_Batch1_Config_SaveLoad_CoreState(void) {
	const char *flashPath = "selftest_batch1_config.bin";

	remove(flashPath);
	SIM_ShutdownOBK();
	SIM_SetupEmptyFlashModeNoFile();
	SIM_SetupNewFlashFile(flashPath);
	SIM_StartOBK(flashPath);

	CFG_SetShortDeviceName("Batch1DeviceName");
	CFG_SetMQTTClientId("batch1ClientPersist");
	CFG_SetMQTTGroupTopic("batch1GroupPersist");
	CFG_SetMQTTHost("192.0.2.55");
	CFG_SetMQTTUserName("batch1User");
	CFG_SetWiFiSSID("Batch1SSID");
	CFG_SetShortStartupCommand("setChannel 8 88");
	CFG_SetChannelStartupValue(6, 77);
	CFG_SetFlag(3, true);
	PIN_SetPinRoleForPinIndex(9, IOR_Relay_n);
	PIN_SetPinChannelForPinIndex(9, 6);
	SELFTEST_ASSERT_CMD_OK("setPinRole 10 Button");
	SELFTEST_ASSERT_CMD_OK("setPinChannel 10 7");

	CFG_Save_IfThereArePendingChanges();
	SIM_SaveFlashData(flashPath);
	SIM_ClearOBK(flashPath);

	SELFTEST_ASSERT_STRING(CFG_GetShortDeviceName(), "Batch1DeviceName");
	SELFTEST_ASSERT_STRING(CFG_GetMQTTClientId(), "batch1ClientPersist");
	SELFTEST_ASSERT_STRING(CFG_GetMQTTGroupTopic(), "batch1GroupPersist");
	SELFTEST_ASSERT_STRING(CFG_GetMQTTHost(), "192.0.2.55");
	SELFTEST_ASSERT_STRING(CFG_GetMQTTUserName(), "batch1User");
	SELFTEST_ASSERT_STRING(CFG_GetWiFiSSID(), "Batch1SSID");
	SELFTEST_ASSERT_STRING(CFG_GetShortStartupCommand(), "setChannel 8 88");
	SELFTEST_ASSERT(CFG_GetChannelStartupValue(6) == 77);
	SELFTEST_ASSERT(CFG_HasFlag(3) == true);
	SELFTEST_ASSERT(PIN_GetPinRoleForPinIndex(9) == IOR_Relay_n);
	SELFTEST_ASSERT(PIN_GetPinChannelForPinIndex(9) == 6);
	SELFTEST_ASSERT(PIN_GetPinRoleForPinIndex(10) == IOR_Button);
	SELFTEST_ASSERT(PIN_GetPinChannelForPinIndex(10) == 7);

	remove(flashPath);
	SIM_ShutdownOBK();
	SIM_SetupEmptyFlashModeNoFile();
	SIM_StartOBK(0);
}

#if ENABLE_LITTLEFS
static void Test_Batch1_LFS_NegativeAndOverwrite(void) {
	SIM_ClearOBK(0);
	SELFTEST_ASSERT_CMD_OK("lfs_format");

	Test_FakeHTTPClientPacket_GET("api/lfs/batch1_missing.txt");
	SELFTEST_ASSERT_HTML_REPLY("{\"fname\":\"batch1_missing.txt\",\"error\":-2}");

	SELFTEST_ASSERT_CMD_OK("lfs_write batch1_file.txt ABCDEFG");
	Test_FakeHTTPClientPacket_GET("api/lfs/batch1_file.txt");
	SELFTEST_ASSERT_HTML_REPLY("ABCDEFG");

	SELFTEST_ASSERT_CMD_OK("lfs_write batch1_file.txt XY");
	Test_FakeHTTPClientPacket_GET("api/lfs/batch1_file.txt");
	SELFTEST_ASSERT_HTML_REPLY("XY");

	SELFTEST_ASSERT_CMD_OK("lfs_append batch1_file.txt Z");
	Test_FakeHTTPClientPacket_GET("api/lfs/batch1_file.txt");
	SELFTEST_ASSERT_HTML_REPLY("XYZ");

	SELFTEST_ASSERT_CMD_OK("lfs_remove batch1_file.txt");
	Test_FakeHTTPClientPacket_GET("api/lfs/batch1_file.txt");
	SELFTEST_ASSERT_HTML_REPLY("{\"fname\":\"batch1_file.txt\",\"error\":-2}");

	SELFTEST_ASSERT_CMD_OK("lfs_write batch1_file.txt recreated");
	Test_FakeHTTPClientPacket_GET("api/lfs/batch1_file.txt");
	SELFTEST_ASSERT_HTML_REPLY("recreated");
}
#endif

void Test_Batch1(void) {
	Test_Batch1_UART_RingBuffer_Wraparound();
	Test_Batch1_UART_CommandFakeHex_PartialData();
	Test_Batch1_SIM_UART_RingBuffer_Wraparound();
	Test_Batch1_Command_ReturnContracts();
	Test_Batch1_MQTT_CommandTopic_Rejection();
	Test_Batch1_HTTP_Rest_InvalidInputs();
#if ENABLE_SEND_POSTANDGET
	Test_Batch1_HTTP_Client_RequestContracts();
#endif
	Test_Batch1_BitMessage_ReadWrite_Bounds();
	Test_Batch1_Config_SaveLoad_CoreState();
#if ENABLE_LITTLEFS
	Test_Batch1_LFS_NegativeAndOverwrite();
#endif
}

#endif
