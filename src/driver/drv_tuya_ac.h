#ifndef __DRV_TUYA_AC_H__
#define __DRV_TUYA_AC_H__

#include "../new_common.h"

typedef struct {
    int power;
    float target_temp;
    float current_temp;
    int mode; // 0: Auto, 1: Cool, 2: Dry, 3: Fan, 4: Heat
    int fan; // 0: Auto, 1: Lowest, 2: Low, 3: Mid-Low, 4: Mid, 5: Mid-High, 6: High, 7: Turbo
    int h_swing;
    int v_swing;
    int health;
    int display;
    int sleep;
    int buzzer;
    int generator;
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
void TuyaAC_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);

#endif // __DRV_TUYA_AC_H__
