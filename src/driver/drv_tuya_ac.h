#ifndef __DRV_TUYA_AC_H__
#define __DRV_TUYA_AC_H__

#include "../new_common.h"

typedef enum {
    TUYA_AC_MODE_AUTO = 0,
    TUYA_AC_MODE_COOL = 1,
    TUYA_AC_MODE_DRY = 2,
    TUYA_AC_MODE_FAN = 3,
    TUYA_AC_MODE_HEAT = 4
} TuyaAC_Mode_e;

typedef enum {
    TUYA_AC_FAN_AUTO = 0,
    TUYA_AC_FAN_LOWEST = 1,
    TUYA_AC_FAN_LOW = 2,
    TUYA_AC_FAN_MID_LOW = 3,
    TUYA_AC_FAN_MID = 4,
    TUYA_AC_FAN_MID_HIGH = 5,
    TUYA_AC_FAN_HIGH = 6,
    TUYA_AC_FAN_TURBO = 7
} TuyaAC_FanMode_e;

typedef enum {
    TUYA_AC_HSWING_KEEP      = 0,
    TUYA_AC_HSWING_AUTO      = 1,
    TUYA_AC_HSWING_FLOW_LEFT = 2,
    TUYA_AC_HSWING_FLOW_MID  = 3,
    TUYA_AC_HSWING_FLOW_RIGHT = 4,
    // Values 5-8 are reserved/unused in the protocol
    TUYA_AC_HSWING_FIX_LEFT      = 9,
    TUYA_AC_HSWING_FIX_MID_LEFT  = 10,
    TUYA_AC_HSWING_FIX_MID       = 11,
    TUYA_AC_HSWING_FIX_MID_RIGHT = 12,
    TUYA_AC_HSWING_FIX_RIGHT     = 13
} TuyaAC_HSwing_e;

typedef enum {
    TUYA_AC_VSWING_KEEP     = 0,
    TUYA_AC_VSWING_AUTO     = 1,
    TUYA_AC_VSWING_FLOW_UP  = 2,
    TUYA_AC_VSWING_FLOW_DOWN = 3,
    // Values 4-8 are reserved/unused in the protocol
    TUYA_AC_VSWING_FIX_ABOVE    = 9,
    TUYA_AC_VSWING_FIX_MID_HIGH = 10,
    TUYA_AC_VSWING_FIX_MID      = 11,
    TUYA_AC_VSWING_FIX_MID_LOW  = 12,
    TUYA_AC_VSWING_FIX_DOWN     = 13
} TuyaAC_VSwing_e;

typedef enum {
    TUYA_AC_SLEEP_OFF = 0,
    TUYA_AC_SLEEP_STANDARD = 1,
    TUYA_AC_SLEEP_AGED = 2,
    TUYA_AC_SLEEP_CHILD = 3
} TuyaAC_Sleep_e;

typedef enum {
    TUYA_AC_GEN_OFF = 0,
    TUYA_AC_GEN_LEVEL_1 = 1,
    TUYA_AC_GEN_LEVEL_2 = 2,
    TUYA_AC_GEN_LEVEL_3 = 3
} TuyaAC_Generator_e;

typedef struct {
    int power;
    float target_temp;
    float current_temp;
    TuyaAC_Mode_e mode;
    TuyaAC_FanMode_e fan;
    TuyaAC_HSwing_e h_swing;
    TuyaAC_VSwing_e v_swing;
    int health;
    int display;
    TuyaAC_Sleep_e sleep;
    int buzzer;
    TuyaAC_Generator_e generator;
    int mute;
    int eco;
    int v_motor;
    int h_motor;
    uint32_t energy;
    uint32_t in_fan_rpm;
    uint32_t in_fan_percent;
    float out_temp;
    uint32_t out_fan_rpm;
    uint32_t runtime;
    uint32_t filter;
    uint32_t compressor_hz;
    int eight_degree;
    uint8_t seq;
} tuya_ac_state_t;

extern tuya_ac_state_t g_tuya_ac;

void TuyaAC_Init(void);
void TuyaAC_RunEverySecond(void);
void TuyaAC_RunFrame(void);
void TuyaAC_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);

#endif // __DRV_TUYA_AC_H__
