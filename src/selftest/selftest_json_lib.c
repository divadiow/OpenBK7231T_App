#ifdef WINDOWS

#include "selftest_local.h"
#include "../cJSON/cJSON.h"

void Test_JSON_Lib() {
	int i;
	cJSON* root;
	cJSON* stats;
	char *msg;
	float dailyStats[4] = { 00000095.44071197,00000171.84954833,00000181.58737182,00000331.35061645 };

	root = cJSON_CreateObject();
	SELFTEST_ASSERT(root != 0);
	{
		stats = cJSON_CreateArray();
		SELFTEST_ASSERT(stats != 0);
		for (i = 0; i < 4; i++)
		{
			cJSON_AddItemToArray(stats, cJSON_CreateNumber(dailyStats[i]));
		}
		cJSON_AddItemToObject(root, "consumption_daily", stats);
	}

	msg = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);

	SELFTEST_ASSERT(msg != 0);
	SELFTEST_ASSERT(strstr(msg, "\"consumption_daily\"") != 0);
	{
		cJSON *parsed;
		cJSON *daily;

		parsed = cJSON_Parse(msg);
		SELFTEST_ASSERT(parsed != 0);
		daily = cJSON_GetObjectItem(parsed, "consumption_daily");
		SELFTEST_ASSERT(daily != 0);
		SELFTEST_ASSERT(cJSON_GetArraySize(daily) == 4);
		for (i = 0; i < 4; i++)
		{
			cJSON *item = cJSON_GetArrayItem(daily, i);
			SELFTEST_ASSERT(item != 0);
			SELFTEST_ASSERT_FLOATCOMPAREEPSILON((float)item->valuedouble, dailyStats[i], 0.001f);
		}
		cJSON_Delete(parsed);
	}
	free(msg);
}


#endif
