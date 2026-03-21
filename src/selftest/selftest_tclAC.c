#ifdef WINDOWS

#include "selftest_local.h"


void Test_Driver_TCL_AC() {
	// reset whole device
	SIM_ClearOBK(0);
	CMD_ExecuteCommand("lfs_format", 0);
	SIM_ClearUART();
	// Required to set up the receive ring buffer before feeding bytes with uartFakeHex
	SIM_UART_InitReceiveRingBuffer(256);

	CMD_ExecuteCommand("startDriver TCL", 0);

	// ACMode 1 = COOL mode: builds a 35-byte set command and flags it for sending
	CMD_ExecuteCommand("ACMode 1", 0);
	TCL_UART_RunEverySecond();
	// TCL set command is always 35 bytes — verify bytes were sent
	SELFTEST_ASSERT_HAS_SOME_DATA_IN_UART();
	SIM_ClearUART();

	// FANMode 1 = FAN_1: rebuilds and queues another 35-byte set command
	CMD_ExecuteCommand("FANMode 1", 0);
	TCL_UART_RunEverySecond();
	SELFTEST_ASSERT_HAS_SOME_DATA_IN_UART();
	SIM_ClearUART();

	// Feed a valid 49-byte get-response packet into the simulated UART receive buffer.
	// TCL_UART_RunEverySecond calls TCL_UART_TryToGetNextPacket which parses this packet
	// and updates MQTT state (ACMode/FANMode/temperature).
	CMD_ExecuteCommand("uartFakeHex BB 01 00 04 2D 04 00 00 00 00 00 00 FF 00 00 00 00 00 FF FF 00 00 00 00 00 00 F0 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 6A", 0);
	TCL_UART_RunEverySecond();
	// After parsing the incoming packet, the AC is reported as off (power=0x01 in packet -> COOL)
	// Verify by checking the MQTT state was published
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("obk0000/stat/ACMode", "cool", false);
}




