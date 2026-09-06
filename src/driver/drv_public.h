#ifndef __DRV_PUBLIC_H__
#define __DRV_PUBLIC_H__

#include "../httpserver/new_http.h"

typedef enum energySensor_e {
	OBK__FIRST = 0,
	OBK_VOLTAGE = OBK__FIRST, // must match order in cmd_public.h
	OBK_CURRENT,
	OBK_POWER,
	OBK_FREQUENCY,
	OBK_POWER_APPARENT,
	OBK_POWER_REACTIVE,
	OBK_POWER_FACTOR,
	OBK_CONSUMPTION_TOTAL,
	OBK__NUM_MEASUREMENTS = OBK_CONSUMPTION_TOTAL,

	// TODO OBK_CONSUMPTION_LAST_HOUR is actally "sum of consumption stats recording period"
	// and won't correspond to 'last hour' unless cmd SetupEnergyStats is enabled and configured to record one hour
	// e.g. 'SetupEnergyStats 1 60 60 0': 60 sec intervals, 60 samples
	OBK_CONSUMPTION_LAST_HOUR,	
	//OBK_CONSUMPTION_STATS, // represents a variable size array of energy samples, not a sensor
	// below here are sensors that are assumed to require NTP driver
	OBK_CONSUMPTION__DAILY_FIRST, //daily consumptions are assumed to be in chronological order
	OBK_CONSUMPTION_TODAY = OBK_CONSUMPTION__DAILY_FIRST, 
	OBK_CONSUMPTION_YESTERDAY,
	OBK_CONSUMPTION_2_DAYS_AGO,
	OBK_CONSUMPTION_3_DAYS_AGO,
	OBK_CONSUMPTION__DAILY_LAST = OBK_CONSUMPTION_3_DAYS_AGO,

	OBK_CONSUMPTION_CLEAR_DATE,
	OBK__LAST = OBK_CONSUMPTION_CLEAR_DATE,
	OBK__NUM_SENSORS,
} energySensor_t;

#if ENABLE_BL_TWIN
extern const int OBK_CONSUMPTION_STORED_LAST[2];
#else
extern const int OBK_CONSUMPTION_STORED_LAST[1];
#endif

typedef struct energySensorNames_s {
	const char* const hass_dev_class;
	const char* const units;
	const char* const name_friendly;
	const char* const name_mqtt;
	const char* const hass_uniq_id_suffix; //keep identifiers persistent in case OBK_ENERG_SENSOR changes
} energySensorNames_t;

extern int g_dhtsCount;

void DRV_Generic_Init();
void DRV_OnHassDiscovery(const char *topic);
void DRV_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState);
void DRV_OnEverySecond();
void DHT_OnEverySecond();
void DHT_OnPinsConfigChanged();
void DRV_RunQuickTick();
void DRV_StartDriver(const char* name);
void DRV_StopDriver(const char* name);
// right now only used by simulator
void DRV_ShutdownAllDrivers();
bool DRV_IsRunning(const char* name);
void DRV_SavePowerMeterDriverStatistics(void);
void DRV_OnChannelChanged(int channel, int iVal);
#if PLATFORM_BK7231N
void Strip_setMultiplePixel(uint32_t pixel, uint8_t *data, bool push);
#endif
void DRV_GosundSW2_Write(float* rgbcw);
void SM2135_Write(float* rgbcw);
void BP5758D_Write(float* rgbcw);
void BP1658CJ_Write(float* rgbcw);
void SM2235_Write(float* rgbcw);
void KP18058_Write(float *rgbcw);
void DRV_DGR_OnLedDimmerChange(int iVal);
void DRV_DGR_OnLedEnableAllChange(int iVal);
void DRV_DGR_OnLedFinalColorsChange(byte rgbcw[5]);

// OBK_POWER etc
float DRV_GetReading(energySensor_t type);
int BL_HasEnergySensorReadingEx(int asensdatasetix, energySensor_t type);
int BL_HasEnergySensorReading(energySensor_t type);
energySensorNames_t* DRV_GetEnergySensorNames(energySensor_t type);
energySensorNames_t* DRV_GetEnergySensorNamesEx(int asensdatasetix, energySensor_t type);
bool DRV_IsMeasuringPower();
bool DRV_IsMeasuringBattery();
bool DRV_IsSensor();

// TuyaMCU exports for LED
void TuyaMCU_OnRGBCWChange(const float *rgbcw, int bLightEnableAll, int iLightMode, float brightnessRange01, float temperatureRange01);
bool TuyaMCU_IsLEDRunning();

/*
 * CFG_CountLEDRemapChannels() describes the LED_Map used by physical LED
 * driver chips. TuyaMCU's LED bridge is logical/UART-backed and does not use
 * that map. http_fns.c historically uses the remap count for both its LED UI
 * and Home Assistant discovery, so adjust those callers without changing the
 * stored LED_Map or the physical-driver semantics.
 *
 * The wrapper is deliberately only enabled when new_cfg.h and new_pins.h have
 * already been included. At present the CFG_CountLEDRemapChannels() callers
 * that also include drv_public.h are in http_fns.c, where this is the intended
 * discovery/UI behaviour.
 */
#if ENABLE_DRIVER_TUYAMCU && defined(__NEW_CFG_H__) && defined(__NEW_PINS_H__)
static inline bool LED_IsPhysicalDriverChipRunningForDiscovery(void) {
#ifndef OBK_DISABLE_ALL_DRIVERS
	return DRV_IsRunning("SM2135") || DRV_IsRunning("BP5758D")
		|| DRV_IsRunning("TESTLED") || DRV_IsRunning("SM2235") || DRV_IsRunning("BP1658CJ")
		|| DRV_IsRunning("KP18058")
		|| DRV_IsRunning("SM16703P")
		|| DRV_IsRunning("SM15155E")
		|| DRV_IsRunning("DMX");
#else
	return false;
#endif
}

static inline int LED_GetEffectiveMappedChannelCount(void) {
	if (TuyaMCU_IsLEDRunning() && !LED_IsPhysicalDriverChipRunningForDiscovery()) {
		int pwmCount = 0;
		PIN_get_Relay_PWM_Count(0, &pwmCount, 0);
		// Local PWM configuration takes precedence in mixed local/Tuya setups.
		// With no local PWM, tuyaMcu_setupLED exposes the full logical RGBCW API.
		return pwmCount > 0 ? pwmCount : 5;
	}
	return (CFG_CountLEDRemapChannels)();
}

#define CFG_CountLEDRemapChannels() LED_GetEffectiveMappedChannelCount()
#endif

void Shutter_MoveByIndex(int index, float frac, bool bStopOnDuplicate);

#endif /* __DRV_PUBLIC_H__ */

