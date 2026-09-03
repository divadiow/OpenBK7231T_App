
#ifdef WINDOWS

#include "selftest_local.h"
#include "../hal/hal_wifi.h"
#include "../driver/drv_uart.h"

void Test_TuyaMCU_RawAccess() {
	// reset whole device
	SIM_ClearOBK(0);
	SIM_ClearAndPrepareForMQTTTesting("TuyaMCU", "bekens");

	SIM_UART_InitReceiveRingBuffer(2048);

	CMD_ExecuteCommand("startDriver TuyaMCU", 0);

	CFG_SetFlag(OBK_FLAG_TUYAMCU_STORE_RAW_DATA, 1);

	// This will map TuyaMCU dpID 2 of type Value to channel 15
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 val 15", 0);
	SELFTEST_ASSERT_CHANNEL(15, 0);
	// This packet sets dpID 2 of type Value to 100
	CMD_ExecuteCommand("uartFakeHex 55AA0307000802020004000000647D", 0);


	SIM_SendFakeMQTTAndRunSimFrame_CMND("Dp", "");
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("stat/TuyaMCU/DP", false);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "id", 2);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "type", 0x02);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "data", 100);

	SIM_ClearMQTTHistory();

	// This packet sets dpID 2 of type Value to 90
	CMD_ExecuteCommand("uartFakeHex 55AA03070008020200040000005A73", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);

	SIM_SendFakeMQTTAndRunSimFrame_CMND("Dp", "");
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("stat/TuyaMCU/DP", false);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "id", 2);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "type", 0x02);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "data", 90);

	SIM_ClearMQTTHistory();


	// This packet sets dpID 18 of type RAW
	// dpID 18
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 18 Raw", 0);
	CMD_ExecuteCommand("uartFakeHex 55AA030700101200000C0101003F030100FA040100AA25", 0);

	SIM_SendFakeMQTTAndRunSimFrame_CMND("Dp", "");
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("stat/TuyaMCU/DP", false);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "id", 18);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "type", 0x00);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_STR(0, "data", "0101003F030100FA040100AA");

	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "id", 2);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "type", 0x02);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "data", 90);

	SIM_ClearMQTTHistory();

	// This packet sets dpID 104 of type RAW
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 104 Raw", 0);
	CMD_ExecuteCommand("uartFakeHex 55AA03070008680200040000000180", 0);

	SIM_SendFakeMQTTAndRunSimFrame_CMND("Dp", "");
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("stat/TuyaMCU/DP", false);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "id", 104);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "type", 0x00);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_STR(0, "data", "00000001");

	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "id", 18);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "type", 0x00);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_STR(1, "data", "0101003F030100FA040100AA");

	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(2, "id", 2);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(2, "type", 0x02);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(2, "data", 90);

	SIM_ClearMQTTHistory();

}
void Test_TuyaMCU_Mult_Internal(float mul) {
	// reset whole device
	SIM_ClearOBK(0);
	SIM_ClearAndPrepareForMQTTTesting("TuyaMCU", "bekens");

	SIM_UART_InitReceiveRingBuffer(2048);

	CMD_ExecuteCommand("startDriver TuyaMCU", 0);

	CFG_SetFlag(OBK_FLAG_TUYAMCU_STORE_RAW_DATA, 1);

	char cmd[256];
	// This will map TuyaMCU dpID 2 of type Value to channel 15 and mult %f
	sprintf(cmd,"linkTuyaMCUOutputToChannel 2 val 15 0 %f",mul);
	CMD_ExecuteCommand(cmd, 0);
	SELFTEST_ASSERT_CHANNEL(15, 0);
	// This packet sets dpID 2 of type Value to 100
	CMD_ExecuteCommand("uartFakeHex 55AA0307000802020004000000647D", 0);


	SIM_SendFakeMQTTAndRunSimFrame_CMND("Dp", "");
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("stat/TuyaMCU/DP", false);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "id", 2);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "type", 0x02);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "data", 100 * mul);

	SIM_ClearMQTTHistory();

	// This packet sets dpID 2 of type Value to 90
	CMD_ExecuteCommand("uartFakeHex 55AA03070008020200040000005A73", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);

	SIM_SendFakeMQTTAndRunSimFrame_CMND("Dp", "");
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("stat/TuyaMCU/DP", false);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "id", 2);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "type", 0x02);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "data", 90 * mul);

	SIM_ClearMQTTHistory();


	// This packet sets dpID 18 of type RAW
	// dpID 18
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 18 Raw", 0);
	CMD_ExecuteCommand("uartFakeHex 55AA030700101200000C0101003F030100FA040100AA25", 0);

	SIM_SendFakeMQTTAndRunSimFrame_CMND("Dp", "");
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("stat/TuyaMCU/DP", false);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "id", 18);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "type", 0x00);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_STR(0, "data", "0101003F030100FA040100AA");

	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "id", 2);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "type", 0x02);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "data", 90 * mul);

	SIM_ClearMQTTHistory();

	// This packet sets dpID 104 of type RAW
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 104 Raw", 0);
	CMD_ExecuteCommand("uartFakeHex 55AA03070008680200040000000180", 0);

	SIM_SendFakeMQTTAndRunSimFrame_CMND("Dp", "");
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("stat/TuyaMCU/DP", false);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "id", 104);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(0, "type", 0x00);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_STR(0, "data", "00000001");

	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "id", 18);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(1, "type", 0x00);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_STR(1, "data", "0101003F030100FA040100AA");

	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(2, "id", 2);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(2, "type", 0x02);
	SELFTEST_ASSERT_HAS_MQTT_ARRAY_ITEM_INT(2, "data", 90 * mul);

	SIM_ClearMQTTHistory();

}
void Test_TuyaMCU_Mult() {
	Test_TuyaMCU_Mult_Internal(1.0f);
	Test_TuyaMCU_Mult_Internal(-1.0f);
	Test_TuyaMCU_Mult_Internal(-5.0f);
	Test_TuyaMCU_Mult_Internal(0.5f);
	Test_TuyaMCU_Mult_Internal(100.0f);
}
void Test_TuyaMCU_TH08() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("startDriver tmSensor", 0);

	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	01		00 00		00	");


	CMD_ExecuteCommand("uartFakeHex 55 AA	03	01		00 37	7B2270223A2269756E697661687039327A6C6C643064222C2276223A22312E302E30222C226D223A302C226E223A312C22736D223A307D	18	", 0);

	Sim_RunSeconds(3, false);
	//SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	03	02		00 00		04	");

}
void Test_TuyaMCU_Boolean() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 bool 2", 0);

	CMD_ExecuteCommand("setChannel 2 1", 0);
	/*
	55 AA    00    06        00 05    0201000101    0F
	HEADER    VER=00    SetDP        LEN    dpId=2 Bool V=1        CHK
	*/
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA    00    06        00 05    0201000101    0F");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	//CMD_ExecuteCommand("setChannel 2 0", 0);
	/*
	55 AA    00    06        00 05    0201000100    0E
	HEADER    VER=00    SetDP        LEN    dpId=2 Bool V=0        CHK
	*/
	//SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA    00    06        00 05    0201000100    0E");
	//SELFTEST_ASSERT_HAS_UART_EMPTY();

}
static void Test_TuyaMCU_RunUntilUARTData(int maxFrames) {
	int i;

	for (i = 0; i < maxFrames; i++) {
		if (SIM_UART_GetDataSize() != 0) {
			break;
		}
		Sim_RunFrames(1, false);
	}
	SELFTEST_ASSERT_HAS_SOME_DATA_IN_UART();
}
static int Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket() {
	int wifiState;
	int checksum;

	SELFTEST_ASSERT(SIM_UART_GetDataSize() >= 8);
	SELFTEST_ASSERT(SIM_UART_GetByte(0) == 0x55);
	SELFTEST_ASSERT(SIM_UART_GetByte(1) == 0xAA);
	SELFTEST_ASSERT(SIM_UART_GetByte(2) == 0x00);
	SELFTEST_ASSERT(SIM_UART_GetByte(3) == 0x03);
	SELFTEST_ASSERT(SIM_UART_GetByte(4) == 0x00);
	SELFTEST_ASSERT(SIM_UART_GetByte(5) == 0x01);
	wifiState = SIM_UART_GetByte(6);
	checksum = (0xFF + 0x03 + 0x01 + wifiState) & 0xFF;
	SELFTEST_ASSERT(SIM_UART_GetByte(7) == checksum);
	SIM_UART_ConsumeBytes(8);
	return wifiState;
}
static void Test_TuyaMCU_V3_RunToMCUConf() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode 0", 0);

	SELFTEST_ASSERT_HAS_UART_EMPTY();
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 00 00 00 FF");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA030000010003", 0);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 01 00 00 00");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100027B7DFD", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 02 00 00 01");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

static void Test_TuyaMCU_V3_NormalStartupCompatibility() {
	Test_TuyaMCU_V3_RunToMCUConf();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0302000004", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 08 00 00 07");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

static void Test_TuyaMCU_V3_LegacySelfProcessingStartupCompatibility() {
	Test_TuyaMCU_V3_RunToMCUConf();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA030200020C0D1F", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	// Preserve the second empty 0x03 frame emitted by the original state machine.
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 03 00 00 02");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

static void Test_TuyaMCU_V3_ManualProductQueryCompatibility() {
	Test_TuyaMCU_V3_NormalStartupCompatibility();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0307000009", 0);

	CMD_ExecuteCommand("tuyaMcu_sendProductInformation", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 01 00 00 00");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100027B7DFD", 0);

	// The normal heartbeat schedule continues, but the product response must not
	// restart initialization and request MCU configuration again.
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 00 00 00 FF");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030000010104", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

static void Test_TuyaMCU_V3_DisableHeartbeatNormalModeCompatibility() {
	Test_TuyaMCU_V3_NormalStartupCompatibility();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0307000009", 0);

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0325000027", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 25 00 00 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	// Disabling the legacy minimal mode must not undo the MCU's 0x25 request.
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode 0", 0);

	Sim_RunFrames(500, false);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

static void Test_TuyaMCU_V3_DisableHeartbeatLegacyMinimalModeCompatibility() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode", 0);

	SELFTEST_ASSERT_HAS_UART_EMPTY();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0325000027", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 25 00 00 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 01 00 00 00");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

static void Test_TuyaMCU_V3_LegacyMinimalModeStartupCompatibility() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode", 0);

	SELFTEST_ASSERT_HAS_UART_EMPTY();
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 01 00 00 00");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100027B7DFD", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0303000005", 0);
	Sim_RunFrames(250, false);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode 0", 0);
}

static void Test_TuyaMCU_V3_McuInitiatedWakeCompatibility() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode 0", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 5 val 5", 0);

	// A previous wake session asked the module to stop heartbeats before sleeping.
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0325000027", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 25 00 00 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100467B2270223A22726B6E7769306374626267687A676C61222C2276223A22312E302E30222C226D223A312C226D74223A312C226E223A302C226C6F77223A312C22736D223A307D6A", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 02 00 00 01");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0302000004", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// A compatibility script that explicitly disables the old minimal mode must
	// not cancel the automatic standard-v3 exchange already in progress.
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode 0", 0);
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0303000005", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 08 00 00 07");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA032200250F020004000000640502000400000138030200040000000009040001006502000400000371FC", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 23 00 01 01 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SELFTEST_ASSERT_CHANNEL(5, 312);

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0325000027", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 25 00 00 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100467B2270223A22726B6E7769306374626267687A676C61222C2276223A22312E302E30222C226D223A312C226D74223A312C226E223A302C226C6F77223A312C22736D223A307D6A", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 02 00 00 01");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// The alternate self-processing working-mode response still uses the same
	// immediate WiFi-ACK-to-QUERY_STATE wake ordering when sm=0/1.
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030200020C0D1F", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0303000005", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 08 00 00 07");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// Conversely, enabling the legacy minimal mode during a wake must take over
	// before the pending WiFi ACK can trigger the automatic 0x08 query.
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100467B2270223A22726B6E7769306374626267687A676C61222C2276223A22312E302E30222C226D223A312C226D74223A312C226E223A302C226C6F77223A312C22736D223A307D6A", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 02 00 00 01");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0302000004", 0);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode", 0);
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0303000005", 0);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode 0", 0);
}

static void Test_TuyaMCU_V3_LowPowerMissingMCUConfCompatibility() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode 0", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 5 val 5", 0);

	// This field capture identifies an sm=0 wake but never answers 0x02. The
	// standard query is still attempted first, then the driver must continue
	// with the legacy-compatible WiFi exchange instead of stalling forever.
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100467B2270223A22726B6E7769306374626267687A676C61222C2276223A22312E302E30222C226D223A312C226D74223A312C226E223A302C226C6F77223A312C22736D223A307D6A", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 02 00 00 01");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	Test_TuyaMCU_RunUntilUARTData(250);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0303000005", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 08 00 00 07");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA03220008050200040000013870", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 23 00 01 01 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SELFTEST_ASSERT_CHANNEL(5, 312);
}

static void Test_TuyaMCU_TH03Pro_V3PowerOffSession() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 102 val 21", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 103 val 22", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 104 enum 23", 0);

	// Exact TH03Pro FY sm=0 session recovered from NY8A054E ROM
	// EC1CBEE6224E2674437ECC18EB140ABFD1E8BFF8BE7762BD0A1C8BE4FAEAB162.
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 00 00 00 FF");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA030000010003", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 01 00 00 00");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100377B2270223A2278656167696D616E7462376437617062222C2276223A22312E302E30222C226D223A302C226E223A312C22736D223A307D24", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 02 00 00 01");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0302000004", 0);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0303000005", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 08 00 00 07");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0324000026", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 24 00 01 FF 23");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("fakeTuyaPacket 55AA0322001566020004000000E1670200040000002F680400010291", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 23 00 01 01 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SELFTEST_ASSERT_CHANNEL(21, 225);
	SELFTEST_ASSERT_CHANNEL(22, 47);
	SELFTEST_ASSERT_CHANNEL(23, 2);
}

static void Test_TuyaMCU_V3_EmptyCloudCacheCompatibility() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);

	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 25 val 25 1", 0);
	CMD_ExecuteCommand("setChannel 25 123456", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 26 enum 26 1", 0);
	CMD_ExecuteCommand("setChannel 26 2", 0);

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA039000020119AE", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 90 00 02 01 00 92");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA039000020319B0", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 90 00 02 01 00 92");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA03900002011AAF", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 90 00 02 01 00 92");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA039000010093", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 90 00 02 01 00 92");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// Legacy v0 command 0x10 still returns configured DPCache channel values.
	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA001000010010", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 10 00 0F 01 02 1A 04 00 01 02 19 02 00 04 00 01 E2 40 84");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// Preserve the old driver's version-header tolerance for legacy 0x10.
	// This remains distinct from the standard-v3 0x90 empty-cache response.
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0310000012", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 10 00 0F 01 02 1A 04 00 01 02 19 02 00 04 00 01 E2 40 84");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

static void Test_TuyaMCU_V3_FeatureSettingsCompatibility() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("tuyaMcu_batteryPoweredMode 0", 0);

	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 00 00 00 FF");
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030000010003", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 01 00 00 00");
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100027B7DFD", 0);

	// abv bit 3 is clear: optional reconnect/app-panel queries are disabled, but
	// Tuya still requires the initial status query after module initialization.
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0337000A007B22616276223A307D22", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 37 00 02 00 00 38");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 02 00 00 01");
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0302000004", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 08 00 00 07");
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0307000009", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 00 00 00 FF");
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030000010104", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0303000005", 0);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	Sim_RunSeconds(1.0f, false);
	// The simulator may establish MQTT during this second and legitimately send
	// one updated WiFi-state report. Acknowledge it, then ensure no 0x08 follows.
	if (SIM_UART_GetDataSize() != 0) {
		Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
		SELFTEST_ASSERT_HAS_UART_EMPTY();
		CMD_ExecuteCommand("fakeTuyaPacket 55AA0303000005", 0);
	}
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// A synchronous report remains accepted and acknowledged without a query.
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0322000024", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 23 00 01 01 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// On a later MCU restart, abv bit 3 now suppresses only the optional query.
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030000010003", 0);
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030100027B7DFD", 0);
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0337000A007B22616276223A307D22", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 37 00 02 00 00 38");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 02 00 00 01");
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0302000004", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 00 00 00 FF");
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030000010104", 0);
	Test_TuyaMCU_RunUntilUARTData(250);
	Test_TuyaMCU_ExpectAndConsumeWiFiStatePacket();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0303000005", 0);
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// Invalid feature JSON is explicitly rejected.
	CMD_ExecuteCommand("fakeTuyaPacket 55AA03370002007BB6", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 37 00 02 00 01 39");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

void Test_TuyaMCU_DP22() {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);

	// Preserve the legacy driver's tolerance for a v3 0x22 command carrying a
	// version-0 header. Existing devices must still receive the success ACK.
	CMD_ExecuteCommand("uartFakeHex 55AA0022000021", 0);
	Sim_RunFrames(20, false);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 23 00 01 01 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 1 bool 1", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 bool 2", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 3 bool 3", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 4 bool 4", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 5 bool 5", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 6 bool 6", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 101 bool 11", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 102 bool 12", 0);
	CMD_ExecuteCommand("setChannel 1 123", 0);
	CMD_ExecuteCommand("setChannel 2 234", 0);
	CMD_ExecuteCommand("setChannel 3 456", 0);
	CMD_ExecuteCommand("setChannel 4 567", 0);
	CMD_ExecuteCommand("setChannel 5 653", 0);
	CMD_ExecuteCommand("setChannel 6 777", 0);
	CMD_ExecuteCommand("setChannel 7 777", 0); // not set
	CMD_ExecuteCommand("setChannel 11 777", 0);
	CMD_ExecuteCommand("setChannel 12 777", 0);

	SIM_ClearUART();
	CMD_ExecuteCommand("uartFakeHex 55AA03220028010100010002010001000301000100040100010005010001000601000100650100010066010001003C", 0);
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 23 00 01 01 24");
	SIM_UART_ExpectAndConsumeHexStr("55 AA 00 00 00 00 FF");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SELFTEST_ASSERT_CHANNEL(1, 0);
	SELFTEST_ASSERT_CHANNEL(2, 0);
	SELFTEST_ASSERT_CHANNEL(3, 0);
	SELFTEST_ASSERT_CHANNEL(4, 0);
	SELFTEST_ASSERT_CHANNEL(5, 0);
	SELFTEST_ASSERT_CHANNEL(6, 0);
	SELFTEST_ASSERT_CHANNEL(7, 777);
	SELFTEST_ASSERT_CHANNEL(11, 0);
	SELFTEST_ASSERT_CHANNEL(12, 0);

	// TH03-style v3 sync report (dp102 temp*10, dp103 humidity, dp104 battery enum)
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 102 val 21", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 103 val 22", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 104 enum 23", 0);
	CMD_ExecuteCommand("setChannelType 21 Temperature_div10", 0);
	CMD_ExecuteCommand("setChannelType 22 Humidity", 0);
	CMD_ExecuteCommand("setChannelType 23 ReadOnlyLowMidHigh", 0);

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0322001566020004000000DC670200040000003A680400010297", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 23 00 01 01 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SELFTEST_ASSERT_CHANNEL(21, 220);
	SELFTEST_ASSERT_CHANNEL(22, 58);
	SELFTEST_ASSERT_CHANNEL(23, 2);

	// v3 command coverage needed by TH03/T1 devices
	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA03900003021718C6", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 90 00 02 01 00 92");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 23 bool 24 1", 0);
	CMD_ExecuteCommand("setChannel 24 1", 0);

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA039000020117AC", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 90 00 02 01 00 92");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA039000020118AD", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 90 00 02 01 00 92");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0337000A007B22616276223A387D2A", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 37 00 02 00 00 38");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030F000011", 0);
	{
		int i;
		int checksum = 0;
		uint32_t freeMemory;

		SELFTEST_ASSERT(SIM_UART_GetDataSize() == 11);
		SELFTEST_ASSERT(SIM_UART_GetByte(0) == 0x55);
		SELFTEST_ASSERT(SIM_UART_GetByte(1) == 0xAA);
		SELFTEST_ASSERT(SIM_UART_GetByte(2) == 0x00);
		SELFTEST_ASSERT(SIM_UART_GetByte(3) == 0x0F);
		SELFTEST_ASSERT(SIM_UART_GetByte(4) == 0x00);
		SELFTEST_ASSERT(SIM_UART_GetByte(5) == 0x04);
		freeMemory = ((uint32_t)SIM_UART_GetByte(6) << 24) |
			((uint32_t)SIM_UART_GetByte(7) << 16) |
			((uint32_t)SIM_UART_GetByte(8) << 8) |
			SIM_UART_GetByte(9);
		SELFTEST_ASSERT(freeMemory > 0);
		for (i = 0; i < 10; i++) {
			checksum += SIM_UART_GetByte(i);
		}
		SELFTEST_ASSERT(SIM_UART_GetByte(10) == (checksum & 0xFF));
		SIM_UART_ConsumeBytes(11);
	}
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0320000022", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 20 00 02 00 02 23");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA03340001033A", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 34 00 02 03 01 39");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA033400010B42", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 34 00 02 0B 03 43");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA030500010109", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 05 00 00 04");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0324000026", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 24 00 01 FF 23");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0325000027", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 25 00 00 24");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA032D00002F", 0);
	{
		byte expectedMac[6];
		byte expectedReply[14] = { 0x55, 0xAA, 0x00, 0x2D, 0x00, 0x07, 0x00, 0, 0, 0, 0, 0, 0, 0 };
		int i;
		int checksum = 0xFF;
		WiFI_GetMacAddress((char*)expectedMac);
		for (i = 0; i < 6; i++) {
			expectedReply[7 + i] = expectedMac[i];
		}
		// Tuya checksum for outgoing packet is 0xFF + command + lenHi + lenLo + payload bytes.
		for (i = 3; i < 13; i++) {
			checksum += expectedReply[i];
		}
		expectedReply[13] = checksum & 0xFF;
		SELFTEST_ASSERT(SIM_UART_GetDataSize() == 14);
		for (i = 0; i < 14; i++) {
			SELFTEST_ASSERT(SIM_UART_GetByte(i) == expectedReply[i]);
		}
		SIM_UART_ConsumeBytes(14);
	}
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	Test_TuyaMCU_V3_NormalStartupCompatibility();
	Test_TuyaMCU_V3_LegacySelfProcessingStartupCompatibility();
	Test_TuyaMCU_V3_ManualProductQueryCompatibility();
	Test_TuyaMCU_V3_DisableHeartbeatNormalModeCompatibility();
	Test_TuyaMCU_V3_DisableHeartbeatLegacyMinimalModeCompatibility();
	Test_TuyaMCU_V3_LegacyMinimalModeStartupCompatibility();
	Test_TuyaMCU_V3_McuInitiatedWakeCompatibility();
	Test_TuyaMCU_V3_LowPowerMissingMCUConfCompatibility();
	Test_TuyaMCU_TH03Pro_V3PowerOffSession();
	Test_TuyaMCU_V3_EmptyCloudCacheCompatibility();
	Test_TuyaMCU_V3_FeatureSettingsCompatibility();
}
void Test_TuyaMCU_Basic() {
	// reset whole device
	SIM_ClearOBK(0);

	SIM_UART_InitReceiveRingBuffer(2048);

	CMD_ExecuteCommand("startDriver TuyaMCU", 0);

	// This will map TuyaMCU dpID 2 of type Value to channel 15
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 val 15", 0);
	SELFTEST_ASSERT_CHANNEL(15, 0);
	// This packet sets dpID 2 of type Value to 100
	CMD_ExecuteCommand("uartFakeHex 55AA0307000802020004000000647D", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, channel 15 should be set to 100...
	SELFTEST_ASSERT_CHANNEL(15, 100);

	// This packet sets dpID 2 of type Value to 90
	CMD_ExecuteCommand("uartFakeHex 55AA03070008020200040000005A73", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, channel 15 should be set to 90...
	SELFTEST_ASSERT_CHANNEL(15, 90);

	// This packet sets dpID 2 of type Value to 110
	CMD_ExecuteCommand("uartFakeHex 55AA03070008020200040000006E87", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, channel 15 should be set to 110...
	SELFTEST_ASSERT_CHANNEL(15, 110);

	// This packet sets dpID 2 of type Value to 120
	CMD_ExecuteCommand("uartFakeHex 55AA03070008020200040000007891", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, channel 15 should be set to 120...
	SELFTEST_ASSERT_CHANNEL(15, 120);

	SIM_ClearUART();

	//
	// check sending from OBK to MCU
	//
	// OBK sends:   55 AA	00	06		00 05	1001000100	1C
	//HEADER	VER = 00	SetDP		LEN	dpId = 16 Bool V = 0	CHK
	CMD_ExecuteCommand("tuyaMcu_sendState 16 1 0", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 05	1001000100	1C");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// OBK sends:  55 AA	00	06		00 05	0101000101	0E
	// HEADER	VER = 00	Unk		LEN	dpId = 1 Bool V = 1	CHK
	CMD_ExecuteCommand("tuyaMcu_sendState 1 1 1", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 05	0101000101	0E");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// OBK sends:  55 AA	00	06		00 05	0101000100	0D
	// HEADER	VER = 00	Unk		LEN	dpId = 1 Bool V = 0	CHK
	CMD_ExecuteCommand("tuyaMcu_sendState 1 1 0", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 05	0101000100	0D");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	
	// OBK sends:  55 AA	00	06		00 05	6C01000101	79
	// HEADER	VER = 00	Unk		LEN	dpId = 108 Bool V = 1	CHK
	CMD_ExecuteCommand("tuyaMcu_sendState 108 bool 1", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 05	6C01000101	79");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// OBK sends:   55 AA	00	06		00 05	6D04000110	8C
	// HEADER	VER = 00	Unk		LEN	dpId = 109 Enum V = 16	CHK
	CMD_ExecuteCommand("tuyaMcu_sendState 109 4 16", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 05	6D04000110	8C");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// OBK sends:  55 AA	00	06		00 05	6D04000110	8C
	// HEADER	VER = 00	Unk		LEN	dpId = 109 Enum V = 16	CHK
	CMD_ExecuteCommand("tuyaMcu_sendState 109 enum 16", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 05	6D04000110	8C");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();


		

		


	// OBK sends: 
	// 55 AA	00	06		00 14	1100001001010050030100F5040100A008000032	64
	//HEADER	VER = 00	Unk		LEN	dpId = 17 Raw V = 01 01 00 50 03 01 00 F5 04 01 00 A0 08 00 00 32	CHK
	CMD_ExecuteCommand("tuyaMcu_sendState 17 0 01010050030100F5040100A008000032", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 14	1100001001010050030100F5040100A008000032	64");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// check channel as argument in raw
	CMD_ExecuteCommand("setChannel 10 1",0);
	CMD_ExecuteCommand("tuyaMcu_sendState 17 0 $CH10$$CH10$0050030100F5040100A008000032", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 14	1100001001010050030100F5040100A008000032	64");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// check channel as argument in raw
	CMD_ExecuteCommand("setChannel 10 1", 0);
	CMD_ExecuteCommand("setChannel 11 0", 0);
	CMD_ExecuteCommand("setChannel 2 0x50", 0);
	CMD_ExecuteCommand("setChannel 3 0x03", 0);
	CMD_ExecuteCommand("setChannel 4 0xF5", 0);
	CMD_ExecuteCommand("tuyaMcu_sendState 17 0 $CH10$$CH10$$CH11$$CH2$$CH3$0100$CH4$040100A008000032", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 14	1100001001010050030100F5040100A008000032	64");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// check channel as argument in raw
	CMD_ExecuteCommand("setChannel 10 1", 0);
	CMD_ExecuteCommand("setChannel 11 0", 0);
	CMD_ExecuteCommand("setChannel 2 0x50", 0);
	CMD_ExecuteCommand("setChannel 3 0x03", 0);
	CMD_ExecuteCommand("setChannel 4 0xF5", 0);
	CMD_ExecuteCommand("tuyaMcu_sendState 17 0 $CH10$ $CH10$ $CH11$ $CH2$ $CH3$ 01 00 $CH4$ 04 01 00 A0 08 00 00 32", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA	00	06		00 14	1100001001010050030100F5040100A008000032	64");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();


	SIM_ClearOBK(0);
	SIM_ClearAndPrepareForMQTTTesting("myTestDevice", "bekens");
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);

	// This packet sets dpID 2 of type Value to 120
	// linkTuyaMCUOutputToChannel dpId varType channelID
	// Special Syntax! is used to link it to MQTT
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 MQTT", 0);
	CMD_ExecuteCommand("uartFakeHex 55AA03070008020200040000007891", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, expect a certain MQTT packet to be published....
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("myTestDevice/tm/val/2", "120", false);
	// if assert has passed, we can clear SIM MQTT history, it's no longer needed
	SIM_ClearMQTTHistory();

	// This packet sets dpID 17 of type RAW
	// dpID 17: Leak protection
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 17 MQTT", 0);
	CMD_ExecuteCommand("uartFakeHex 55AA03070008110000040400001E48", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, expect a certain MQTT packet to be published....
	// NOTE: I just did hex to ascii on payload 
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("myTestDevice/tm/raw/17", "0400001E", false);
	// if assert has passed, we can clear SIM MQTT history, it's no longer needed
	SIM_ClearMQTTHistory();

	// This packet sets dpID 18 of type RAW
	// dpID 18
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 18 MQTT", 0);
	CMD_ExecuteCommand("uartFakeHex 55AA030700101200000C0101003F030100FA040100AA25",0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, expect a certain MQTT packet to be published....
	// NOTE: I just did hex to ascii on payload 
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("myTestDevice/tm/raw/18", "0101003F030100FA040100AA", false);
	// if assert has passed, we can clear SIM MQTT history, it's no longer needed
	SIM_ClearMQTTHistory();

	// This packet sets dpID 104 of type Val
	// dpID 104
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 104 MQTT", 0);
	CMD_ExecuteCommand("uartFakeHex 55AA03070008680200040000000180",0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, expect a certain MQTT packet to be published....
	// NOTE: I just did hex to ascii on payload 
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("myTestDevice/tm/val/104", "1", false);
	// if assert has passed, we can clear SIM MQTT history, it's no longer needed
	SIM_ClearMQTTHistory();

	SIM_ClearUART();

	//
	// uartSendHex tests
	//
	CMD_ExecuteCommand("uartSendHex FFAABB", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("FF AA	BB");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();

	CMD_ExecuteCommand("setChannel 12 0xCC", 0);
	CMD_ExecuteCommand("uartSendHex FF$CH12$BB", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("FF CC	BB");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();

	CMD_ExecuteCommand("setChannel 12 0xDD", 0);
	CMD_ExecuteCommand("uartSendHex FF$CH12$BB", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("FF DD	BB");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();

	CMD_ExecuteCommand("setChannel 13 0xEE", 0);
	CMD_ExecuteCommand("uartSendHex FF$CH12$$CH13$", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("FF DD	EE");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();


	CMD_ExecuteCommand("uartSendHex FF$CH12$00$CH13$00", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("FF DD 00 EE 00");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();

	// make sure that spaces won't break stuff
	CMD_ExecuteCommand("uartSendHex FF$CH12$00$CH13$00  ", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("FF DD 00 EE 00");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();

	// make sure that spaces won't break stuff
	CMD_ExecuteCommand("uartSendHex   FF  $CH12$  00  $CH13$   00  ", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("FF DD 00 EE 00");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();

	//
	// tuyaMcu_sendCmd tests
	//
	// This command will calculate checksum as well - 0x32
	CMD_ExecuteCommand("tuyaMcu_sendCmd 0x30 000000", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 30 00 03 00 00 00 32");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();
	// This command will calculate checksum as well - 0x38
	CMD_ExecuteCommand("tuyaMcu_sendCmd 0x30 000006", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 30 00 03 00 00 06 38");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();
	// This command will calculate checksum as well - 0x3
	// This will send wifi state 0x04
	CMD_ExecuteCommand("tuyaMcu_sendCmd 0x03 04", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 03 00 01 04 07");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();




	// check tuyamcu inversion

	CMD_ExecuteCommand("setChannel 15 0", 0);
	// This will map TuyaMCU dpID 2 of type Value to channel 15 with inverse
	// [dpId][varType][channelID][bDPCache-Optional][mult-optional][bInverse]
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 val 15 0 1 1", 0);
	SELFTEST_ASSERT_CHANNEL(15, 0);
	// This packet sets dpID 2 of type Value to 100
	CMD_ExecuteCommand("uartFakeHex 55AA0307000802020004000000647D", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, channel 15 should be set to negated 100. so 0
	SELFTEST_ASSERT_CHANNEL(15, 0);
	// and disable inversion
	CMD_ExecuteCommand("setChannel 15 0", 0);
	// This will map TuyaMCU dpID 2 of type Value to channel 15 with no inverse
	// [dpId][varType][channelID][bDPCache-Optional][mult-optional][bInverse]
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 val 15 0 1 0", 0);
	SELFTEST_ASSERT_CHANNEL(15, 0);
	// This packet sets dpID 2 of type Value to 100
	CMD_ExecuteCommand("uartFakeHex 55AA0307000802020004000000647D", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, channel 15 should be set to  100
	SELFTEST_ASSERT_CHANNEL(15, 100);

	// now try multiplier

	CMD_ExecuteCommand("setChannel 15 0", 0);
	// This will map TuyaMCU dpID 2 of type Value to channel 15 with no inverse
	// 10 is multiplier
	// [dpId][varType][channelID][bDPCache-Optional][mult-optional][bInverse]
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 val 15 0 10 0", 0);
	SELFTEST_ASSERT_CHANNEL(15, 0);
	// This packet sets dpID 2 of type Value to 100
	CMD_ExecuteCommand("uartFakeHex 55AA0307000802020004000000647D", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, channel 15 should be set to  100*10
	SELFTEST_ASSERT_CHANNEL(15, 100 * 10);



	SIM_ClearUART();
	// now try delta - value 5
	CMD_ExecuteCommand("setChannel 15 0", 0);
	// This will map TuyaMCU dpID 2 of type Value to channel 15 with no inverse
	// 10 is multiplier
	// 5 is delta
	// [dpId][varType][channelID][bDPCache-Optional][mult-optional][bInverse][delta]
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 val 15 0 10 0 5", 0);
	SELFTEST_ASSERT_CHANNEL(15, 0);
	// This packet sets dpID 2 of type Value to 100
	CMD_ExecuteCommand("uartFakeHex 55AA0307000802020004000000647D", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, channel 15 should be set to....
	SELFTEST_ASSERT_CHANNEL(15, (100+5) * 10);
	SIM_ClearUART();
	// now try delta - value -5
	CMD_ExecuteCommand("setChannel 15 0", 0);
	// This will map TuyaMCU dpID 2 of type Value to channel 15 with no inverse
	// 10 is multiplier
	// 5 is delta
	// [dpId][varType][channelID][bDPCache-Optional][mult-optional][bInverse][delta]
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 2 val 15 0 10 0 -5", 0);
	SELFTEST_ASSERT_CHANNEL(15, 0);
	// This packet sets dpID 2 of type Value to 100
	CMD_ExecuteCommand("uartFakeHex 55AA0307000802020004000000647D", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(1000, false);
	// Now, channel 15 should be set to....
	SELFTEST_ASSERT_CHANNEL(15, (100 + -5) * 10);
	SIM_ClearUART();


	CMD_ExecuteCommand("tuyaMcu_sendCmd 0x06 19030036303035413030303130303738303345383033453830303030303030303030303030313030303030303030303030303030303030303030", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 06 00 3A 19030036303035413030303130303738303345383033453830303030303030303030303030313030303030303030303030303030303030303030 18");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();

	// check to see if we can execute a file from within LFS
	Test_FakeHTTPClientPacket_POST("api/lfs/tuyamcu1.txt", "tuyaMcu_sendCmd 0x06 19030036303035413030303130303738303345383033453830303030303030303030303030313030303030303030303030303030303030303030");
	// exec will execute a script file in-place, all commands at once
	CMD_ExecuteCommand("exec tuyamcu1.txt", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 06 00 3A 19030036303035413030303130303738303345383033453830303030303030303030303030313030303030303030303030303030303030303030 18");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();

	CMD_ExecuteCommand("alias xyz123 tuyaMcu_sendCmd 0x06 19030036303035413030303130303738303345383033453830303030303030303030303030313030303030303030303030303030303030303030",0);
	CMD_ExecuteCommand("xyz123", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 06 00 3A 19030036303035413030303130303738303345383033453830303030303030303030303030313030303030303030303030303030303030303030 18");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	// twice
	CMD_ExecuteCommand("xyz123", 0);
	CMD_ExecuteCommand("xyz123", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 06 00 3A 19030036303035413030303130303738303345383033453830303030303030303030303030313030303030303030303030303030303030303030 18");
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 06 00 3A 19030036303035413030303130303738303345383033453830303030303030303030303030313030303030303030303030303030303030303030 18");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SIM_ClearMQTTHistory();
	// cause error
	//SELFTEST_ASSERT_CHANNEL(15, 666);



	// fake time request
	NTP_SetSimulatedTime(1732094000);
	// Simulate TuyaMCU sending 0x1C request to OBK
	CMD_ExecuteCommand("uartFakeHex 55 AA 03 1C 00 00 1E", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(100, false);
	// OBK will reply with 0x1C packet (TUYA_CMD_SET_TIME)
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55AA001C000801180B14090D140388");
	// skip optional heartbeat
	SIM_UART_ExpectAndConsumeHexStr("55AA00000000FF");
	// nothing is sent by OBK at that point
	SELFTEST_ASSERT_HAS_UART_EMPTY();


	
	SIM_ClearUART();


	// this will write to channel 40, 41, and 42
	// 232.6V 153mA 3.5W
	// 0.0153*232.6 = 3.55878W
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 6 RAW_TAC2121C_VCP 40", 0);
	CMD_ExecuteCommand("uartFakeHex 55AA03070014060000080916000099000023010200040000000310", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(40, 2326);// voltage
	SELFTEST_ASSERT_CHANNEL(41, 153);// current
	SELFTEST_ASSERT_CHANNEL(42, 35);// power

	// this will write to channel 45, 46 and 47
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 8 RAW_TAC2121C_VCP 45", 0);
	CMD_ExecuteCommand("uartFakeHex 55 AA 03 07 00 0C 08 00 00 08 09 01 00 01 2E 00 00 43 A1 ", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(45, 2305); // voltage
	SELFTEST_ASSERT_CHANNEL(46, 302);// current
	SELFTEST_ASSERT_CHANNEL(47, 67); // power

	SIM_ClearUART();
	SIM_ClearOBK(0);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);

	g_cfg.pins.channelTypes[3] = ChType_Voltage_div10;
	g_cfg.pins.channelTypes[7] = ChType_Current_div1000;
	g_cfg.pins.channelTypes[9] = ChType_Power;

	CMD_ExecuteCommand("setChannel 0 12345", 0);
	// this will write to auto-found channels
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 8 RAW_TAC2121C_VCP", 0);
	CMD_ExecuteCommand("uartFakeHex 55 AA 03 07 00 0C 08 00 00 08 09 01 00 01 2E 00 00 43 A1 ", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(100, false);

	SELFTEST_ASSERT_CHANNEL(0, 12345); // not changed
	SELFTEST_ASSERT_CHANNEL(3, 2305); // voltage
	SELFTEST_ASSERT_CHANNEL(7, 302);// current
	SELFTEST_ASSERT_CHANNEL(9, 67); // power


	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 115 RAW_VCPPfF", 0);
	CMD_ExecuteCommand("uartFakeHex 55 AA 03 07 00 13 73 00 00 0F 09 0F 00 00 BF 00 00 78 00 00 25 01 10 C3 32 18 ", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(100, false);

	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 113 RAW_VCPPfF", 0);
	CMD_ExecuteCommand("uartFakeHex 55 AA 03 07 00 13 71 00 00 0F 09 29 00 01 B7 00 03 FC 00 00 00 03 E8 C3 32 65", 0);
	// above command will just put into buffer - need at least a frame to parse it
	Sim_RunFrames(100, false);

	SIM_ClearUART();
}
void Test_TuyaMCU_Calib() {
	{
		// reset whole device
		SIM_ClearOBK(0);

		SIM_UART_InitReceiveRingBuffer(2048);

		CMD_ExecuteCommand("startDriver TuyaMCU", 0);

		g_cfg.pins.channelTypes[3] = ChType_Voltage_div10;
		g_cfg.pins.channelTypes[7] = ChType_Current_div1000;
		g_cfg.pins.channelTypes[9] = ChType_Power;

		CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 113 RAW_VCPPfF", 0);
		CMD_ExecuteCommand("uartFakeHex 55 AA 03 07 00 13 71 00 00 0F 09 29 00 01 B7 00 03 FC 00 00 00 03 E8 C3 32 65", 0);
		// above command will just put into buffer - need at least a frame to parse it
		Sim_RunFrames(100, false);

		SELFTEST_ASSERT_CHANNEL(3, 2345);
		SELFTEST_ASSERT_CHANNEL(7, 439);
		SELFTEST_ASSERT_CHANNEL(9, 1020);


		SIM_ClearUART();
	}
	{
		// reset whole device
		SIM_ClearOBK(0);

		SIM_UART_InitReceiveRingBuffer(2048);

		CMD_ExecuteCommand("startDriver TuyaMCU", 0);

		g_cfg.pins.channelTypes[3] = ChType_Voltage_div10;
		g_cfg.pins.channelTypes[7] = ChType_Current_div1000;
		g_cfg.pins.channelTypes[9] = ChType_Power;


		CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 113 RAW_VCPPfF -1 0 1 0 22 33 44", 0);
		CMD_ExecuteCommand("uartFakeHex 55 AA 03 07 00 13 71 00 00 0F 09 29 00 01 B7 00 03 FC 00 00 00 03 E8 C3 32 65", 0);
		// above command will just put into buffer - need at least a frame to parse it
		Sim_RunFrames(100, false);

		SELFTEST_ASSERT_CHANNEL(3, 2345+22);
		SELFTEST_ASSERT_CHANNEL(7, 439+33);
		SELFTEST_ASSERT_CHANNEL(9, 1020+44);


		SIM_ClearUART();
	}

}

// ---------------------------------------------------------------------------
// Test_TuyaMCU_Robustness
//
// Verifies that the truncation guards in TuyaMCU_ParseStateMessage() and
// TuyaMCU_V0_ParseRealTimeWithRecordStorage() reject malformed packets without
// crashing and without corrupting channel state.
//
// Each malformed packet below advertises sectorLen=4 for a VALUE datapoint but
// only carries 3 data bytes after the 4-byte sector header. That directly
// targets the 32-bit scalar decode path: on an unguarded build, the parser
// would attempt to read the missing fourth byte past the end of the payload.
//
// The test strategy is:
//   1. Set a channel to a known sentinel value.
//   2. Feed a truncated packet for the same dpId/channel mapping.
//   3. Assert the channel is unchanged.
//   4. Feed a valid packet on the same path and assert normal parsing still works.
//
// Note: the allocation-failure path in the rawData realloc guard cannot be
// reliably exercised under the Windows simulation (there is no OOM injection
// mechanism), so it is not tested here. The logic is statically correct by
// inspection: the temp-pointer pattern is the standard safe-realloc idiom.
// ---------------------------------------------------------------------------
void Test_TuyaMCU_Robustness() {
	// -----------------------------------------------------------------------
	// Part 1: Truncated STATE packet (v3, cmd 0x07)
	//
	// Malformed packet: 55AA030700060102000400172D
	//   version=3, cmd=STATE(0x07), payload_len=6
	//   payload: dpId=1 type=2(value) sectorLen=4, data bytes present=2
	//   remaining after header = 2, so 4 > 2 => guard fires, break.
	// -----------------------------------------------------------------------
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 1 val 20", 0);

	CMD_ExecuteCommand("setChannel 20 99", 0);
	SELFTEST_ASSERT_CHANNEL(20, 99);

	// inject the truncated STATE packet
	CMD_ExecuteCommand("uartFakeHex 55AA030700060102000400172D", 0);
	Sim_RunFrames(100, false);

	// channel must be untouched - guard prevented parsing
	SELFTEST_ASSERT_CHANNEL(20, 99);

	// Now inject a well-formed STATE packet setting dpId 1 value = 1.
	CMD_ExecuteCommand("uartFakeHex 55AA03070008010200040000000119", 0);
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(20, 1);

	// High-bit values must be decoded as defined signed 32-bit values.
	CMD_ExecuteCommand("uartFakeHex 55AA0307000801020004FFFFFFF60B", 0);
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(20, -10);

	// -----------------------------------------------------------------------
	// Part 2: Truncated V0 cmd 0x05 packet (no datetime prefix)
	//
	// Malformed packet: 55AA0005000601020004001223
	//   version=0, cmd=0x05(TUYA_CMD_WIFI_SELECT), payload_len=6
	//   payload: dpId=1 type=2(value) sectorLen=4, data bytes present=2
	//   remaining=2, guard fires.
	// -----------------------------------------------------------------------
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 1 val 21", 0);

	CMD_ExecuteCommand("setChannel 21 77", 0);
	SELFTEST_ASSERT_CHANNEL(21, 77);

	// inject truncated V0 no-date packet
	CMD_ExecuteCommand("uartFakeHex 55AA0005000601020004001223", 0);
	Sim_RunFrames(100, false);

	SELFTEST_ASSERT_CHANNEL(21, 77);

	// confirm the parser still works normally after seeing the bad packet
	CMD_ExecuteCommand("uartFakeHex 55AA00050008010200040000000114", 0);
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(21, 1);

	// -----------------------------------------------------------------------
	// Part 3: Truncated V0 cmd 0x08 packet (with 7-byte datetime prefix)
	//
	// Malformed packet: 55AA0008000D0000000000000001020004001C37
	//   version=0, cmd=0x08(TUYA_CMD_QUERY_STATE), payload_len=14
	//   7 datetime bytes (all zero) + dpId=1 type=2(value) sectorLen=4
	//   data bytes present=2, so remaining=2 and the guard fires.
	// -----------------------------------------------------------------------
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 1 val 22", 0);

	CMD_ExecuteCommand("setChannel 22 55", 0);
	SELFTEST_ASSERT_CHANNEL(22, 55);

	// inject truncated V0 with-date packet
	CMD_ExecuteCommand("uartFakeHex 55AA0008000D0000000000000001020004001C37", 0);
	Sim_RunFrames(100, false);

	SELFTEST_ASSERT_CHANNEL(22, 55);

	// confirm the parser still works normally after seeing the bad packet
	CMD_ExecuteCommand("uartFakeHex 55AA0008000F0000000000000001020004000000011E", 0);
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(22, 1);

	// Direct command injection must reject undersized and payload-less record frames.
	SIM_ClearUART();
	CMD_ExecuteCommand("fakeTuyaPacket 55AA", 0);
	CMD_ExecuteCommand("fakeTuyaPacket 55AA0334000036", 0);
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// A plausible but incomplete false header must not wedge a valid packet behind it.
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 1 val 23", 0);
	{
		byte falseHeader[] = { 0x55, 0xAA, 0x03, 0x07, 0x03, 0x00 };
		byte validPacket[] = { 0x55, 0xAA, 0x03, 0x07, 0x00, 0x08, 0x01, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x19 };
		int i;

		for (i = 0; i < (int)sizeof(falseHeader); i++) {
			UART_AppendByteToReceiveRingBuffer(falseHeader[i]);
		}
		for (i = 0; i < (int)sizeof(validPacket); i++) {
			UART_AppendByteToReceiveRingBuffer(validPacket[i]);
		}
	}
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(23, 1);

	// A complete bad-checksum frame is skipped without consuming the valid successor.
	{
		byte badPacket[] = { 0x55, 0xAA, 0x03, 0x07, 0x00, 0x00, 0x00 };
		byte validPacket[] = { 0x55, 0xAA, 0x03, 0x07, 0x00, 0x08, 0x01, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x02, 0x1A };
		int i;

		for (i = 0; i < (int)sizeof(badPacket); i++) {
			UART_AppendByteToReceiveRingBuffer(badPacket[i]);
		}
		for (i = 0; i < (int)sizeof(validPacket); i++) {
			UART_AppendByteToReceiveRingBuffer(validPacket[i]);
		}
	}
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(23, 2);

	// Exercise a valid 279-byte frame: both length bytes and receive-buffer growth matter.
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver TuyaMCU", 0);
	CMD_ExecuteCommand("linkTuyaMCUOutputToChannel 1 val 24", 0);
	{
		byte header[] = { 0x55, 0xAA, 0x03, 0x07, 0x01, 0x10 };
		byte rawHeader[] = { 0x63, 0x00, 0x01, 0x04 };
		byte valueDP[] = { 0x01, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x2A };
		byte checksum = 0;
		int i;

		for (i = 0; i < (int)sizeof(header); i++) {
			UART_AppendByteToReceiveRingBuffer(header[i]);
			checksum += header[i];
		}
		for (i = 0; i < (int)sizeof(rawHeader); i++) {
			UART_AppendByteToReceiveRingBuffer(rawHeader[i]);
			checksum += rawHeader[i];
		}
		for (i = 0; i < 260; i++) {
			UART_AppendByteToReceiveRingBuffer(0x00);
		}
		for (i = 0; i < (int)sizeof(valueDP); i++) {
			UART_AppendByteToReceiveRingBuffer(valueDP[i]);
			checksum += valueDP[i];
		}
		UART_AppendByteToReceiveRingBuffer(checksum);
	}
	Sim_RunFrames(100, false);
	SELFTEST_ASSERT_CHANNEL(24, 42);
}

#endif
