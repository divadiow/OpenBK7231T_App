#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#ifndef TICK_BITS
#define TICK_BITS 32
#endif
#if TICK_BITS == 16
typedef uint16_t portTickType;
#else
typedef uint32_t portTickType;
#endif
#ifndef TICK_MS
#define TICK_MS 1
#endif
#define portTICK_PERIOD_MS TICK_MS
#define ENABLE_DRIVER_BL0937 1
#define ENABLE_BL_SHARED 1
#define ENABLE_MQTT 1
#ifndef TICK_HZ
#define TICK_HZ (1000 / TICK_MS)
#endif
#define configTICK_RATE_HZ TICK_HZ
#define PLATFORM_GPIO_MAX 64
#ifndef PLATFORM_BEKEN
#define PLATFORM_BEKEN 0
#endif
#define GLOBAL_INT_DECLARATION() int saved_irq_depth
#define GLOBAL_INT_DISABLE() do { saved_irq_depth = mock_irq_depth; ++mock_irq_depth; } while (0)
#define GLOBAL_INT_RESTORE() mock_restore_irq(saved_irq_depth)
extern int mock_irq_depth;
void mock_restore_irq(int depth);
portTickType xTaskGetTickCount(void);
typedef int commandResult_t;
typedef commandResult_t (*CommandFn)(const void*,const char*,const char*,int);
enum { CMD_RES_OK=0, CMD_RES_ERROR=1, CMD_RES_NOT_ENOUGH_ARGUMENTS=2, CMD_RES_BAD_ARGUMENT=3 };
enum { CFG_OBK_VOLTAGE=0, CFG_OBK_CURRENT, CFG_OBK_POWER, CFG_OBK_POWER_MAX };
enum { IOR_BL0937_SEL=0, IOR_BL0937_SEL_n, IOR_BL0937_CF, IOR_BL0937_CF1, IOR_Relay, IOR_Relay_n, IOR_BridgeForward };
enum { LOG_INFO=0, LOG_FEATURE_ENERGYMETER=1, INTERRUPT_FALLING=0 };
#define ADDLOG_ERROR(...) ((void)0)
#define ADDLOG_INFO(...) ((void)0)
void addLogAdv(int level,int feature,const char*format,...);
int PIN_FindPinIndexForRole(int role,int fallback);
void HAL_PIN_Setup_Output(int pin);
void HAL_PIN_Setup_Input_Pullup(int pin);
void HAL_PIN_SetOutputValue(int pin,int value);
void HAL_AttachInterrupt(int pin,int mode,void(*callback)(int));
void HAL_DetachInterrupt(int pin);
void CMD_RegisterCommand(const char*name,CommandFn fn,const void*context);
void Tokenizer_TokenizeString(const char*str,int flags);
int Tokenizer_CheckArgsCountAndPrintWarning(const char*cmd,int count);
float Tokenizer_GetArgFloat(int index);
const char*CMD_GetResultString(int result);
float CFG_GetPowerMeasurementCalibrationFloat(int key,float default_value);
void CFG_SetPowerMeasurementCalibrationFloat(int key,float value);
void BL_Shared_Init(void);
void BL_ProcessUpdate(float voltage,float current,float power,float frequency,float energy_wh);

/* Minimal dependency declarations. The complete shared-meter C file is linked. */
typedef unsigned char byte;
typedef struct {const char *hass_dev_class,*units,*name_friendly,*name_mqtt,*hass_uniq_id_suffix;} energySensorNames_t;
typedef enum { OBK_VOLTAGE,OBK_CURRENT,OBK_POWER,OBK_FREQUENCY,OBK_POWER_APPARENT,OBK_POWER_REACTIVE,OBK_POWER_FACTOR,OBK_CONSUMPTION_TOTAL,OBK_CONSUMPTION_LAST_HOUR,OBK_CONSUMPTION_TODAY,OBK_CONSUMPTION_YESTERDAY,OBK_CONSUMPTION_2_DAYS_AGO,OBK_CONSUMPTION_3_DAYS_AGO,OBK_CONSUMPTION_CLEAR_DATE,OBK__NUM_SENSORS } energySensor_t;
#define OBK__FIRST OBK_VOLTAGE
#define OBK__LAST OBK_CONSUMPTION_CLEAR_DATE
#define OBK__NUM_MEASUREMENTS OBK_CONSUMPTION_TOTAL
#define OBK_CONSUMPTION__DAILY_FIRST OBK_CONSUMPTION_TODAY
#define OBK_CONSUMPTION__DAILY_LAST OBK_CONSUMPTION_3_DAYS_AGO
#define OBK_PUBLISH_FLAG_QOS_ZERO 0
#define TIME_FORMAT_LONG 0
#define TIME_FORMAT_ISO_8601 1
#define os_malloc malloc
#define os_free free
#define TS2STR(x,y) "2026-09-18T00:00"
typedef void http_request_t;
typedef struct {int dummy;} cJSON;
typedef struct {float TotalConsumption,TodayConsumpion,YesterdayConsumption,TotalConsumption_b,TodayConsumpion_b,ConsumptionHistory[2];int actual_mday;time_t ConsumptionResetTime;long save_counter;} ENERGY_METERING_DATA;
enum {OBK_FLAG_POWER_ALLOW_NEGATIVE,OBK_FLAG_POWER_FORCE_ZERO_IF_RELAYS_OPEN,OBK_FLAG_MQTT_ENERGY_IN_KWH};
enum EventCode {CMD_EVENT_NONE,CMD_EVENT_CHANGE_VOLTAGE,CMD_EVENT_CHANGE_CURRENT,CMD_EVENT_CHANGE_POWER,CMD_EVENT_CHANGE_FREQUENCY,CMD_EVENT_CHANGE_CONSUMPTION_TOTAL,CMD_EVENT_CHANGE_CONSUMPTION_LAST_HOUR};
struct MockConfig {struct {int roles[64],channels[64];} pins;};
extern struct MockConfig g_cfg;
extern int g_secondsElapsed;
void poststr(http_request_t*,const char*);
void hprintf255(http_request_t*,const char*,...);
int DRV_IsRunning(const char*);
float DRV_GetReading(energySensor_t);
int TIME_IsTimeSynced(void);
time_t TIME_GetCurrentTime(void);
int TIME_GetMDay(void);
int TIME_GetTimesZoneOfsSeconds(void);
int OTA_GetProgress(void);
int CHANNEL_Get(int);
int CFG_HasFlag(int);
void HAL_GetEnergyMeterStatus(ENERGY_METERING_DATA*);
void HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA*);
void HAL_FlashVars_SaveTotalConsumption(float);
int Tokenizer_GetArgsCount(void);
int Tokenizer_GetArgInteger(int);
float Tokenizer_GetArgFloatDefault(int,float);
const char* Tokenizer_GetArg(int);
void EventHandlers_ProcessVariableChange_Integer(enum EventCode,int,int);
int MQTT_IsReady(void);
void MQTT_PublishMain_StringFloat(const char*,float,int,int);
void MQTT_PublishMain_StringString(const char*,const char*,int);
cJSON *cJSON_CreateObject(void);
cJSON *cJSON_CreateArray(void);
cJSON *cJSON_CreateNumber(double);
void cJSON_AddNumberToObject(cJSON*,const char*,double);
void cJSON_AddStringToObject(cJSON*,const char*,const char*);
void cJSON_AddItemToArray(cJSON*,cJSON*);
void cJSON_AddItemToObject(cJSON*,const char*,cJSON*);
char *cJSON_PrintUnformatted(cJSON*);
void cJSON_Delete(cJSON*);
float XJ_MovingAverage_float(float,float);
#endif
