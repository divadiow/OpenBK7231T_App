#ifdef WINDOWS

#include "selftest_local.h"

// Dreo protocol packet format:
//   [55 AA] [ver=00] [seq] [cmd] [00] [lenH] [lenL] [payload] [checksum]
//   Checksum = (sum(seq + cmd + lenH + lenL + payload) - 1) & 0xFF
//
// DP payload inside status message (cmd 0x07 or 0x08):
//   [dpId] [0x01] [type] [lenH] [lenL] [value...]

static void Dreo_Test_ResetAndStart(void) {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(2048);
	CMD_ExecuteCommand("startDriver Dreo", 0);
	SIM_ClearUART();
}

static void Dreo_Test_FakeHexAndRun(const char *hex, int frames) {
	CMD_ExecuteCommand(va("uartFakeHex %s", hex), 0);
	if (frames > 0) {
		Sim_RunFrames(frames, false);
	}
}

// ---------------------------------------------------------------------------
// Test_Dreo_Basic
//
// Verifies:
//   1. Mapping a Dreo dpId to a channel via linkDreoOutputToChannel
//   2. Receiving Dreo status packets and setting channels
//   3. Bool, Value, and Enum DP types
//   4. Power on/off cycle
// ---------------------------------------------------------------------------
void Test_Dreo_Basic() {
	Dreo_Test_ResetAndStart();

	// Map dpIds to channels
	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);   // Power
	CMD_ExecuteCommand("linkDreoOutputToChannel 7 val 6", 0);    // Current Temp
	CMD_ExecuteCommand("linkDreoOutputToChannel 2 val 2", 0);    // Mode
	CMD_ExecuteCommand("linkDreoOutputToChannel 4 val 4", 0);    // Target Temp

	SELFTEST_ASSERT_CHANNEL(1, 0);
	SELFTEST_ASSERT_CHANNEL(2, 0);
	SELFTEST_ASSERT_CHANNEL(4, 0);
	SELFTEST_ASSERT_CHANNEL(6, 0);

	// =====================================================================
	// Test 1: dpId=1 (Power), bool value=1
	//
	// 55 AA 00 01 07 00 00 06 01 01 01 00 01 01 checksum
	// sum = 01+07+00+06 + 01+01+01+00+01+01 = 0x13
	// checksum = (0x13 - 1) & 0xFF = 0x12
	// =====================================================================
	Dreo_Test_FakeHexAndRun("55AA00010700000601010100010112", 100);
	SELFTEST_ASSERT_CHANNEL(1, 1);

	// =====================================================================
	// Test 2: dpId=7 (Current Temp), value=25 (0x19)
	//
	// 55 AA 00 02 07 00 00 09 07 01 02 00 04 00 00 00 19 checksum
	// sum = 00+02+07+00+00+09+07+01+02+00+04+00+00+00+19 = 0x39
	// checksum = (0x39 - 1) & 0xFF = 0x38
	// =====================================================================
	Dreo_Test_FakeHexAndRun("55AA00020700000907010200040000001938", 100);
	SELFTEST_ASSERT_CHANNEL(6, 25);

	// =====================================================================
	// Test 3: dpId=2 (Mode), enum value=2 (Eco)
	//
	// 55 AA 00 03 07 00 00 06 02 01 04 00 01 02 checksum
	// sum = 00+03+07+00+00+06+02+01+04+00+01+02 = 0x1A
	// checksum = (0x1A - 1) & 0xFF = 0x19
	// =====================================================================
	Dreo_Test_FakeHexAndRun("55AA00030700000602010400010219", 100);
	SELFTEST_ASSERT_CHANNEL(2, 2);

	// =====================================================================
	// Test 4: dpId=4 (Target Temp), value=30 (0x1E), cmd=0x08 (query state)
	//
	// 55 AA 00 04 08 00 00 09 04 01 02 00 04 00 00 00 1E checksum
	// sum = 00+04+08+00+00+09+04+01+02+00+04+00+00+00+1E = 0x3E
	// checksum = (0x3E - 1) & 0xFF = 0x3D
	// =====================================================================
	Dreo_Test_FakeHexAndRun("55AA00040800000904010200040000001E3D", 100);
	SELFTEST_ASSERT_CHANNEL(4, 30);

	// =====================================================================
	// Test 5: Previous channels still hold their values
	// =====================================================================
	SELFTEST_ASSERT_CHANNEL(1, 1);
	SELFTEST_ASSERT_CHANNEL(6, 25);
	SELFTEST_ASSERT_CHANNEL(2, 2);

	// =====================================================================
	// Test 6: Power off — dpId=1, bool value=0
	//
	// 55 AA 00 05 07 00 00 06 01 01 01 00 01 00 checksum
	// sum = 05+07+00+06 + 01+01+01+00+01+00 = 0x16
	// checksum = (0x16 - 1) & 0xFF = 0x15
	// =====================================================================
	Dreo_Test_FakeHexAndRun("55AA00050700000601010100010015", 100);
	SELFTEST_ASSERT_CHANNEL(1, 0);

	SIM_ClearUART();
}

// ---------------------------------------------------------------------------
// Test_Dreo_MultiDP
//
// Verifies receiving a status packet containing multiple DPs in one message.
// ---------------------------------------------------------------------------
void Test_Dreo_MultiDP() {
	Dreo_Test_ResetAndStart();

	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);
	CMD_ExecuteCommand("linkDreoOutputToChannel 2 val 2", 0);
	CMD_ExecuteCommand("linkDreoOutputToChannel 3 val 3", 0);

	SELFTEST_ASSERT_CHANNEL(1, 0);
	SELFTEST_ASSERT_CHANNEL(2, 0);
	SELFTEST_ASSERT_CHANNEL(3, 0);

	// =====================================================================
	// Multi-DP packet: dpId=1 power=1, dpId=2 mode=1, dpId=3 heatlevel=3
	//
	// DP1: 01 01 01 00 01 01  (dpId=1 proto=01 type=bool len=1 val=1)
	// DP2: 02 01 04 00 01 01  (dpId=2 proto=01 type=enum len=1 val=1)
	// DP3: 03 01 04 00 01 03  (dpId=3 proto=01 type=enum len=1 val=3)
	// Total payload: 18 bytes = 0x12
	//
	// Header: 55 AA 00 10 07 00 00 12
	// sum = 10+07+00+12
	//     + 01+01+01+00+01+01
	//     + 02+01+04+00+01+01
	//     + 03+01+04+00+01+03
	//   = 0x29 + 0x05 + 0x09 + 0x0C = 0x43
	// checksum = (0x43 - 1) & 0xFF = 0x42
	// =====================================================================
	Dreo_Test_FakeHexAndRun("55AA00100700001201010100010102010400010103010400010342", 100);

	SELFTEST_ASSERT_CHANNEL(1, 1);
	SELFTEST_ASSERT_CHANNEL(2, 1);
	SELFTEST_ASSERT_CHANNEL(3, 3);

	SIM_ClearUART();
}

// ---------------------------------------------------------------------------
// Test_Dreo_FragmentedPacket
//
// Verifies that partial UART arrival does not update channels early, and that
// a packet split across many chunks is parsed correctly once complete.
// ---------------------------------------------------------------------------
void Test_Dreo_FragmentedPacket() {
	const char *chunks[] = {
		"55",
		"AA0001",
		"07000006",
		"0101",
		"010001",
		"0112"
	};
	int i;

	Dreo_Test_ResetAndStart();
	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);

	for (i = 0; i < (int)(sizeof(chunks) / sizeof(chunks[0])) - 1; i++) {
		Dreo_Test_FakeHexAndRun(chunks[i], 1);
		SELFTEST_ASSERT_CHANNEL(1, 0);
		SELFTEST_ASSERT_HAS_UART_EMPTY();
	}

	Dreo_Test_FakeHexAndRun(chunks[i], 1);
	SELFTEST_ASSERT_CHANNEL(1, 1);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

// ---------------------------------------------------------------------------
// Test_Dreo_GarbagePrefixResync
//
// Verifies that garbage bytes before a valid header are skipped and the next
// packet still parses correctly.
// ---------------------------------------------------------------------------
void Test_Dreo_GarbagePrefixResync() {
	Dreo_Test_ResetAndStart();
	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);

	Dreo_Test_FakeHexAndRun("DEADBEEF55AA00010700000601010100010112", 1);
	SELFTEST_ASSERT_CHANNEL(1, 1);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

// ---------------------------------------------------------------------------
// Test_Dreo_ChecksumMismatchThenRecovery
//
// Verifies that a bad packet is ignored and that parsing recovers onto a
// later good packet already sitting in the same UART buffer.
//
// Current Dreo parser behavior on checksum failure is to consume only one byte
// and return. That means recovery requires a later driver pass to skip forward
// to the next 55 AA header and then parse the following valid packet.
// ---------------------------------------------------------------------------
void Test_Dreo_ChecksumMismatchThenRecovery() {
	Dreo_Test_ResetAndStart();
	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);
	CMD_ExecuteCommand("linkDreoOutputToChannel 4 val 4", 0);

	// First packet has a bad checksum (0x13 instead of 0x12). The second packet
	// is valid and should still be parsed after the next resync pass.
	Dreo_Test_FakeHexAndRun(
		"55AA00010700000601010100010113"
		"55AA00040800000904010200040000001E3D",
		0);

	// First pass should notice the checksum mismatch, consume a byte and stop.
	Sim_RunFrames(1, false);
	SELFTEST_ASSERT_CHANNEL(1, 0);
	SELFTEST_ASSERT_CHANNEL(4, 0);

	// Second pass should skip forward to the next header and parse the valid
	// packet that was already waiting in the buffer.
	Sim_RunFrames(1, false);
	SELFTEST_ASSERT_CHANNEL(1, 0);
	SELFTEST_ASSERT_CHANNEL(4, 30);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

// ---------------------------------------------------------------------------
// Test_Dreo_BackToBackPackets
//
// Verifies that two full packets arriving back-to-back in the UART buffer are
// both consumed during the same driver run.
// ---------------------------------------------------------------------------
void Test_Dreo_BackToBackPackets() {
	Dreo_Test_ResetAndStart();
	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);
	CMD_ExecuteCommand("linkDreoOutputToChannel 4 val 4", 0);

	Dreo_Test_FakeHexAndRun(
		"55AA00010700000601010100010112"
		"55AA00040800000904010200040000001E3D",
		1);

	SELFTEST_ASSERT_CHANNEL(1, 1);
	SELFTEST_ASSERT_CHANNEL(4, 30);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}


// ---------------------------------------------------------------------------
// Test_Dreo_FragmentedBackToBackPackets
//
// Verifies that two adjacent packets split across awkward UART chunk boundaries
// do not update channels early and are both parsed once the final bytes arrive.
// This mirrors the sort of short-frame + longer status-frame burst that was
// suspected in the ESP32/Dreo thread.
// ---------------------------------------------------------------------------
void Test_Dreo_FragmentedBackToBackPackets() {
	const char *chunks[] = {
		"55AA000107",
		"000006010101000101",
		"1255AA0004080000090401",
		"0200040000001E3D"
	};
	int i;

	Dreo_Test_ResetAndStart();
	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);
	CMD_ExecuteCommand("linkDreoOutputToChannel 4 val 4", 0);

	for (i = 0; i < (int)(sizeof(chunks) / sizeof(chunks[0])) - 1; i++) {
		Dreo_Test_FakeHexAndRun(chunks[i], 1);
		SELFTEST_ASSERT_CHANNEL(1, 0);
		SELFTEST_ASSERT_CHANNEL(4, 0);
		SELFTEST_ASSERT_HAS_UART_EMPTY();
	}

	Dreo_Test_FakeHexAndRun(chunks[i], 1);
	SELFTEST_ASSERT_CHANNEL(1, 1);
	SELFTEST_ASSERT_CHANNEL(4, 30);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

// ---------------------------------------------------------------------------
// Test_Dreo_ChannelToDP
//
// Verifies that changing a channel value sends the correct DP packet to MCU.
// Checks exact packet bytes including checksum.
// ---------------------------------------------------------------------------
void Test_Dreo_ChannelToDP() {
	Dreo_Test_ResetAndStart();

	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);

	SIM_ClearUART();

	// =====================================================================
	// Set channel 1 = 1 (power on) → sends SendBool(dpId=1, val=1)
	//
	// Dreo_SendDP builds payload: [01, 01, 01, 00, 01, 01]  (6 bytes)
	// Dreo_SendRaw(cmd=0x06, payload, 6):
	//   55 AA 00 [seq=00] 06 00 00 06 01 01 01 00 01 01 [checksum]
	//   sum = 00+06+00+06 + 01+01+01+00+01+01 = 0x11
	//   checksum = (0x11 - 1) & 0xFF = 0x10
	// =====================================================================
	CMD_ExecuteCommand("setChannel 1 1", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 00 06 00 00 06 01 01 01 00 01 01 10");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// =====================================================================
	// Set channel 1 = 0 (power off) → sends SendBool(dpId=1, val=0)
	//
	//   55 AA 00 [seq=01] 06 00 00 06 01 01 01 00 01 00 [checksum]
	//   sum = 01+06+00+06 + 01+01+01+00+01+00 = 0x11
	//   checksum = (0x11 - 1) & 0xFF = 0x10
	// =====================================================================
	CMD_ExecuteCommand("setChannel 1 0", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 01 06 00 00 06 01 01 01 00 01 00 10");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
}

// ---------------------------------------------------------------------------
// Test_Dreo_ChannelToDP_Value
//
// Verifies sending a Value-type DP (4-byte big-endian) when channel changes.
// ---------------------------------------------------------------------------
void Test_Dreo_ChannelToDP_Value() {
	Dreo_Test_ResetAndStart();

	CMD_ExecuteCommand("linkDreoOutputToChannel 4 val 4", 0);  // Target Temp

	SIM_ClearUART();

	// =====================================================================
	// Set channel 4 = 25 (0x19) → sends SendValue(dpId=4, val=25)
	//
	// Dreo_SendDP payload: [04, 01, 02, 00, 04, 00, 00, 00, 19]  (9 bytes)
	// Dreo_SendRaw(cmd=0x06, payload, 9):
	//   55 AA 00 [seq=00] 06 00 00 09 04 01 02 00 04 00 00 00 19 [checksum]
	//   sum = 00+06+00+09 + 04+01+02+00+04+00+00+00+19 = 0x33
	//   checksum = (0x33 - 1) & 0xFF = 0x32
	// =====================================================================
	CMD_ExecuteCommand("setChannel 4 25", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 00 06 00 00 09 04 01 02 00 04 00 00 00 19 32");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
}

// ---------------------------------------------------------------------------
// Test_Dreo_AutoStore
//
// Verifies that dpIDs not explicitly mapped are auto-stored without crash,
// and that mapping them later works correctly.
// ---------------------------------------------------------------------------
void Test_Dreo_AutoStore() {
	Dreo_Test_ResetAndStart();

	// Don't map dpId=19 — inject a packet for it
	// dpId=19 (0x13), bool, value=1
	//
	// 55 AA 00 20 07 00 00 06 13 01 01 00 01 01 checksum
	// sum = 20+07+00+06 + 13+01+01+00+01+01 = 0x44
	// checksum = (0x44 - 1) & 0xFF = 0x43
	Dreo_Test_FakeHexAndRun("55AA00200700000613010100010143", 100);

	// Channel 1 should still be 0 (dpId=19 is not mapped to any channel)
	SELFTEST_ASSERT_CHANNEL(1, 0);

	// Now map dpId=19 to channel 12 and inject again
	CMD_ExecuteCommand("linkDreoOutputToChannel 19 bool 12", 0);
	SELFTEST_ASSERT_CHANNEL(12, 0);

	// Inject dpId=19 again with value=1
	// 55 AA 00 21 07 00 00 06 13 01 01 00 01 01 checksum
	// sum = 21+07+00+06 + 13+01+01+00+01+01 = 0x45
	// checksum = (0x45 - 1) & 0xFF = 0x44
	Dreo_Test_FakeHexAndRun("55AA00210700000613010100010144", 100);

	// Now channel 12 should be 1
	SELFTEST_ASSERT_CHANNEL(12, 1);

	SIM_ClearUART();
}

// ---------------------------------------------------------------------------
// Test_Dreo_SendState
//
// Verifies the dreo_sendState command sends correct packets via UART.
// ---------------------------------------------------------------------------
void Test_Dreo_SendState() {
	Dreo_Test_ResetAndStart();

	SIM_ClearUART();

	// =====================================================================
	// dreo_sendState 16 bool 1  → SendBool(dpId=16(0x10), val=1)
	//
	// payload: [10, 01, 01, 00, 01, 01]  (6 bytes)
	// 55 AA 00 [seq=00] 06 00 00 06 10 01 01 00 01 01 [checksum]
	// sum = 00+06+00+06 + 10+01+01+00+01+01 = 0x20
	// checksum = (0x20 - 1) & 0xFF = 0x1F
	// =====================================================================
	CMD_ExecuteCommand("dreo_sendState 16 bool 1", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 00 06 00 00 06 10 01 01 00 01 01 1F");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();

	// =====================================================================
	// dreo_sendState 4 val 25  → SendValue(dpId=4, val=25=0x19)
	//
	// payload: [04, 01, 02, 00, 04, 00, 00, 00, 19]  (9 bytes)
	// 55 AA 00 [seq=01] 06 00 00 09 04 01 02 00 04 00 00 00 19 [checksum]
	// sum = 01+06+00+09 + 04+01+02+00+04+00+00+00+19 = 0x34
	// checksum = (0x34 - 1) & 0xFF = 0x33
	// =====================================================================
	CMD_ExecuteCommand("dreo_sendState 4 val 25", 0);
	SELFTEST_ASSERT_HAS_SENT_UART_STRING("55 AA 00 01 06 00 00 09 04 01 02 00 04 00 00 00 19 33");
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
}

// ---------------------------------------------------------------------------
// Test_Dreo_NoEcho
//
// Verifies that when a DP is received from MCU and sets a channel, the driver
// does not immediately echo the same value back to the MCU, and that setting
// the same channel value again still produces no packet.
// ---------------------------------------------------------------------------
void Test_Dreo_NoEcho() {
	Dreo_Test_ResetAndStart();
	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);

	SIM_ClearUART();

	// Inject dpId=1, power=1 from MCU
	// 55 AA 00 01 07 00 00 06 01 01 01 00 01 01 checksum
	// sum = 01+07+00+06+01+01+01+00+01+01 = 0x13
	// checksum = (0x13 - 1) = 0x12
	Dreo_Test_FakeHexAndRun("55AA00010700000601010100010112", 1);

	// Channel should be set, but no UART packet should have been sent.
	SELFTEST_ASSERT_CHANNEL(1, 1);
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	// Re-setting to the same value should still remain silent.
	CMD_ExecuteCommand("setChannel 1 1", 0);
	SELFTEST_ASSERT_HAS_UART_EMPTY();

	SIM_ClearUART();
}

// ---------------------------------------------------------------------------
// Test_Dreo_RealCapture
//
// Real captured packet from a Dreo heater.
// Source: https://www.elektroda.com/rtvforum/viewtopic.php?p=21856379#21856379
//
// 18 DPs in one status packet, 126-byte payload, checksum 0x49.
// Note: DP02 and DP03 were contiguous in the capture — the original listing
// showed "02 00 04 00 01 03 03" which is DP02 (mode=3) + first byte of DP03.
// DP03 (heat_level=3) was confirmed by checksum verification.
//
// Parsed DPs:
//   dpId=1  bool  val=0  (power off)
//   dpId=2  enum  val=3  (mode=Fan)
//   dpId=3  enum  val=3  (heat_level=3)
//   dpId=4  val   val=86 (target_temp=86F)
//   dpId=5  bool  val=0
//   dpId=6  bool  val=0  (sound=off)
//   dpId=7  val   val=77 (current_temp=77F)
//   dpId=8  bool  val=1  (display=on)
//   dpId=9  val   val=0  (timer=0)
//   dpId=17 val   val=0  (temp_unit_alias)
//   dpId=13 val   val=0
//   dpId=14 val   val=0
//   dpId=15 val   val=0  (calibration)
//   dpId=16 bool  val=0  (child_lock=off)
//   dpId=19 bool  val=0  (heating_status=off)
//   dpId=20 bool  val=0  (window_detection=off)
//   dpId=21 enum  val=0
//   dpId=22 enum  val=2  (temp_unit=C)
// ---------------------------------------------------------------------------
void Test_Dreo_RealCapture() {
	Dreo_Test_ResetAndStart();

	// Map all known dpIds to channels (same as suggested autoexec.bat)
	CMD_ExecuteCommand("linkDreoOutputToChannel 1 bool 1", 0);    // Power
	CMD_ExecuteCommand("linkDreoOutputToChannel 2 val 2", 0);     // Mode
	CMD_ExecuteCommand("linkDreoOutputToChannel 3 val 3", 0);     // Heat Level
	CMD_ExecuteCommand("linkDreoOutputToChannel 4 val 4", 0);     // Target Temp
	CMD_ExecuteCommand("linkDreoOutputToChannel 6 bool 5", 0);    // Sound
	CMD_ExecuteCommand("linkDreoOutputToChannel 7 val 6", 0);     // Current Temp
	CMD_ExecuteCommand("linkDreoOutputToChannel 8 bool 7", 0);    // Screen Display
	CMD_ExecuteCommand("linkDreoOutputToChannel 9 val 8", 0);     // Timer
	CMD_ExecuteCommand("linkDreoOutputToChannel 15 val 9", 0);    // Calibration
	CMD_ExecuteCommand("linkDreoOutputToChannel 16 bool 10", 0);  // Child Lock
	CMD_ExecuteCommand("linkDreoOutputToChannel 17 val 11", 0);   // Temp Unit Alias
	CMD_ExecuteCommand("linkDreoOutputToChannel 19 bool 12", 0);  // Heating Status
	CMD_ExecuteCommand("linkDreoOutputToChannel 20 bool 13", 0);  // Window Detection
	CMD_ExecuteCommand("linkDreoOutputToChannel 22 val 14", 0);   // Temp Unit

	// Full captured packet (checksum verified with Python):
	// Header: 55 AA 00 06 07 00 00 7E
	// 18 DP entries, 126 bytes payload
	// Checksum: 0x49
	Dreo_Test_FakeHexAndRun(
		"55AA00060700007E"
		"010001000100"     // dpId=1  bool  val=0
		"020004000103"     // dpId=2  enum  val=3
		"030004000103"     // dpId=3  enum  val=3
		"040002000156"     // dpId=4  val1  val=86
		"050001000100"     // dpId=5  bool  val=0
		"060001000100"     // dpId=6  bool  val=0
		"07000200040000004D"  // dpId=7  val4  val=77
		"080001000101"     // dpId=8  bool  val=1
		"090002000400000000"  // dpId=9  val4  val=0
		"110002000400000000"  // dpId=17 val4  val=0
		"0D0002000400000000"  // dpId=13 val4  val=0
		"0E0002000400000000"  // dpId=14 val4  val=0
		"0F0002000400000000"  // dpId=15 val4  val=0
		"100001000100"     // dpId=16 bool  val=0
		"130001000100"     // dpId=19 bool  val=0
		"140001000100"     // dpId=20 bool  val=0
		"150004000100"     // dpId=21 enum  val=0
		"160004000102"     // dpId=22 enum  val=2
		"49",
		100);

	// Verify mapped channels
	SELFTEST_ASSERT_CHANNEL(1, 0);    // dpId=1  power=off
	SELFTEST_ASSERT_CHANNEL(2, 3);    // dpId=2  mode=Fan
	SELFTEST_ASSERT_CHANNEL(3, 3);    // dpId=3  heat_level=3
	SELFTEST_ASSERT_CHANNEL(4, 86);   // dpId=4  target_temp=86 (F)
	SELFTEST_ASSERT_CHANNEL(5, 0);    // dpId=6  sound=off
	SELFTEST_ASSERT_CHANNEL(6, 77);   // dpId=7  current_temp=77 (F)
	SELFTEST_ASSERT_CHANNEL(7, 1);    // dpId=8  display=on
	SELFTEST_ASSERT_CHANNEL(8, 0);    // dpId=9  timer=0
	SELFTEST_ASSERT_CHANNEL(9, 0);    // dpId=15 calibration=0
	SELFTEST_ASSERT_CHANNEL(10, 0);   // dpId=16 child_lock=off
	SELFTEST_ASSERT_CHANNEL(11, 0);   // dpId=17 temp_unit_alias=0
	SELFTEST_ASSERT_CHANNEL(12, 0);   // dpId=19 heating_status=off
	SELFTEST_ASSERT_CHANNEL(13, 0);   // dpId=20 window_detection=off
	SELFTEST_ASSERT_CHANNEL(14, 2);   // dpId=22 temp_unit=C

	SIM_ClearUART();
}

// ---------------------------------------------------------------------------
// Test_Dreo — Main entry point, calls all sub-tests.
// ---------------------------------------------------------------------------
void Test_Dreo() {
	Test_Dreo_Basic();
	Test_Dreo_MultiDP();
	Test_Dreo_FragmentedPacket();
	Test_Dreo_GarbagePrefixResync();
	Test_Dreo_ChecksumMismatchThenRecovery();
	Test_Dreo_BackToBackPackets();
	Test_Dreo_FragmentedBackToBackPackets();
	Test_Dreo_ChannelToDP();
	Test_Dreo_ChannelToDP_Value();
	Test_Dreo_AutoStore();
	Test_Dreo_SendState();
	Test_Dreo_NoEcho();
	Test_Dreo_RealCapture();
}

#endif
