#ifdef WINDOWS

#include "selftest_local.h"

#if ENABLE_HA_DISCOVERY && ENABLE_DRIVER_HTTPBUTTONS
static void Test_Batch7_HassDiscovery_HTTPButtonConfigEntity(void) {
	SIM_ClearOBK("Batch7HA");
	SIM_ClearAndPrepareForMQTTTesting("batch7HaDevice", "bekens");
	CFG_SetShortDeviceName("Batch7HA");
	CFG_SetDeviceName("Batch7 HA Device");

	SELFTEST_ASSERT_CMD_OK("startDriver httpButtons");
	SELFTEST_ASSERT_CMD_OK("setButtonEnabled 0 1");
	SELFTEST_ASSERT_CMD_OK("setButtonLabel 0 \"Do Config Thing\"");
	SELFTEST_ASSERT_CMD_OK("setButtonCommand 0 \"setChannel 4 44\"");

	SIM_ClearMQTTHistory();
	SELFTEST_ASSERT_CMD_OK("scheduleHADiscovery 1");
	Sim_RunSeconds(5, false);

	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("homeassistant", true);
	SELFTEST_ASSERT_JSON_VALUE_STRING(NULL, "name", "Do Config Thing");
	SELFTEST_ASSERT_JSON_VALUE_STRING(NULL, "command_topic", "cmnd/batch7HaDevice/backlog");
	SELFTEST_ASSERT_JSON_VALUE_STRING(NULL, "payload_press", "setChannel 4 44");
	SELFTEST_ASSERT_JSON_VALUE_STRING_NOT_PRESENT(NULL, "entity_category");
}

static void Test_Batch7_HassDiscovery_HTTPButtonLinkOnlySkipped(void) {
	SIM_ClearOBK("Batch7HALink");
	SIM_ClearAndPrepareForMQTTTesting("batch7HaLink", "bekens");
	CFG_SetShortDeviceName("Batch7HALink");
	CFG_SetDeviceName("Batch7 HA Link");

	SELFTEST_ASSERT_CMD_OK("startDriver httpButtons");
	SELFTEST_ASSERT_CMD_OK("setButtonEnabled 0 1");
	SELFTEST_ASSERT_CMD_OK("setButtonLabel 0 LinkOnly");
	SELFTEST_ASSERT_CMD_OK("setButtonCommand 0 \"*/api/lfs/cfg.html\"");

	SIM_ClearMQTTHistory();
	SELFTEST_ASSERT_CMD_OK("scheduleHADiscovery 1");
	Sim_RunSeconds(5, false);

	SELFTEST_ASSERT_HAS_NOT_MQTT_JSON_SENT_ANY("homeassistant", true, 0, 0, "name", "LinkOnly");
}
#endif

void Test_Batch7(void) {
#if ENABLE_HA_DISCOVERY && ENABLE_DRIVER_HTTPBUTTONS
	Test_Batch7_HassDiscovery_HTTPButtonConfigEntity();
	Test_Batch7_HassDiscovery_HTTPButtonLinkOnlySkipped();
#endif
}

#endif
