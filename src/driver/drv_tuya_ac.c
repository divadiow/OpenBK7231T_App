#include "../obk_config.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../httpserver/new_http.h"
#include "drv_uart.h"
#include "drv_tuya_ac.h"

// Always compile functions, but leave them empty if disabled to satisfy linker
#if ENABLE_DRIVER_TUYA_AC

#define TUYA_AC_BAUDRATE 115200
#define TUYA_AC_RX_BUFFER_SIZE 256
#define TUYA_AC_PUBLISH_INTERVAL_SEC 60

// A5/GFW frame constants verified against the RTL8720CF TCL dump.
#define TUYA_AC_A5_MAGIC           0xA5
#define TUYA_AC_FRAME_REQUEST      0x21
#define TUYA_AC_FRAME_RESPONSE     0x23
#define TUYA_AC_B1_DEFAULT         0x01
#define TUYA_AC_B2_CORE            0x00
#define TUYA_AC_B2_THING           0x01
#define TUYA_AC_RESP_OK            0x00

// Dirty flag bits — one per publishable parameter
#define DIRTY_POWER          (1UL << 0)
#define DIRTY_MODE           (1UL << 1)
#define DIRTY_FAN            (1UL << 2)
#define DIRTY_TARGET_TEMP    (1UL << 3)
#define DIRTY_CURRENT_TEMP   (1UL << 4)
#define DIRTY_H_SWING        (1UL << 5)
#define DIRTY_V_SWING        (1UL << 6)
#define DIRTY_HEALTH         (1UL << 7)
#define DIRTY_DISPLAY        (1UL << 8)
#define DIRTY_SLEEP          (1UL << 9)
#define DIRTY_BUZZER         (1UL << 10)
#define DIRTY_GENERATOR      (1UL << 11)
#define DIRTY_MUTE           (1UL << 12)
#define DIRTY_ECO            (1UL << 13)
#define DIRTY_V_MOTOR        (1UL << 14)
#define DIRTY_H_MOTOR        (1UL << 15)
#define DIRTY_ENERGY         (1UL << 16)
#define DIRTY_IN_FAN_RPM     (1UL << 17)
#define DIRTY_IN_FAN_PERCENT (1UL << 18)
#define DIRTY_OUT_TEMP       (1UL << 19)
#define DIRTY_OUT_FAN_RPM    (1UL << 20)
#define DIRTY_RUNTIME        (1UL << 21)
#define DIRTY_FILTER         (1UL << 22)
#define DIRTY_COMPRESSOR_HZ  (1UL << 23)
#define DIRTY_EIGHT_DEGREE   (1UL << 24)

#define DIRTY_INSTANT_ALWAYS 0xFFFFFFFF

static int g_tuya_ac_have_rx_state = 0;
static int g_tuya_ac_have_published_state = 0;
static int g_tuya_ac_prev_mqtt_connected = 0;
static int g_tuya_ac_snapshot_pending = 0;
static int g_tuya_ac_snapshot_index = 0;

static int g_tuya_ac_mqtt_batch_size = 8;
static int g_tuya_ac_mqtt_batch_cooldown = 1;

static int g_tuya_ac_snapshot_interval = 120;
static int g_tuya_ac_snapshot_timer = 0;

// Buffered publish state
static uint32_t g_tuya_ac_dirty_flags = 0;
static int g_tuya_ac_publish_countdown = TUYA_AC_PUBLISH_INTERVAL_SEC;

static uint32_t g_tuya_ac_instant_publish_mask = 0;
static int g_tuya_ac_instant_publish_timeout = 0;

static uint32_t TuyaAC_GetDirtyFlagForDP(uint16_t dp_id) {
    switch (dp_id) {
        case 0x0001: return DIRTY_POWER | DIRTY_MODE;
        case 0x0012: return DIRTY_MODE;
        case 0x0005: return DIRTY_FAN;
        case 0x0002: return DIRTY_TARGET_TEMP;
        case 0x0003: return DIRTY_CURRENT_TEMP;
        case 0x000E: return DIRTY_H_SWING;
        case 0x0011: return DIRTY_V_SWING;
        case 0x0015: return DIRTY_HEALTH;
        case 0x001E: return DIRTY_DISPLAY;
        case 0x0022: return DIRTY_SLEEP;
        case 0x0025: return DIRTY_BUZZER;
        case 0x002D: return DIRTY_GENERATOR;
        case 0x0073: return DIRTY_MUTE;
        case 0x00DF: return DIRTY_ECO;
        case 0x0147: return DIRTY_EIGHT_DEGREE;
        case 0x0072: return DIRTY_IN_FAN_PERCENT;
        default: return 0;
    }
}

static void TuyaAC_MarkDpConfigured(uint16_t dp_id) {
    g_tuya_ac_instant_publish_mask |= TuyaAC_GetDirtyFlagForDP(dp_id);
    g_tuya_ac_instant_publish_timeout = 5;
}

// AC State
tuya_ac_state_t g_tuya_ac = {
    .power = 0,
    .target_temp = 0.0f,
    .current_temp = 0.0f,
    .mode = TUYA_AC_MODE_AUTO,
    .fan = TUYA_AC_FAN_AUTO,  // auto = 7 per dongle capture
    .h_swing = TUYA_AC_HSWING_KEEP,
    .v_swing = TUYA_AC_VSWING_KEEP,
    .health = 0,
    .display = 0,
    .sleep = TUYA_AC_SLEEP_OFF,
    .buzzer = 0,
    .generator = TUYA_AC_GEN_OFF,
    .mute = 0,
    .eco = 0,
    .v_motor = 0,
    .h_motor = 0,
    .energy = 0,
    .in_fan_rpm = 0,
    .in_fan_percent = 0,
    .out_temp = 0.0f,
    .out_fan_rpm = 0,
    .runtime = 0,
    .filter = 0,
    .compressor_hz = 0,
    .eight_degree = 0,
    .seq = 1
};

static const struct {
    const char *name;
    TuyaAC_Mode_e mode;
} acModeMap[] = {
    {"auto", TUYA_AC_MODE_AUTO},
    {"cool", TUYA_AC_MODE_COOL},
    {"dry", TUYA_AC_MODE_DRY},
    {"fan", TUYA_AC_MODE_FAN},
    {"heat", TUYA_AC_MODE_HEAT}
};

static const struct {
    const char *name;
    TuyaAC_FanMode_e mode;
} fanModeMap[] = {
    {"auto", TUYA_AC_FAN_AUTO},
    {"turbo", TUYA_AC_FAN_TURBO},
    {"high", TUYA_AC_FAN_HIGH},
    {"mid-high", TUYA_AC_FAN_MID_HIGH},
    {"mid", TUYA_AC_FAN_MID},
    {"mid-low", TUYA_AC_FAN_MID_LOW},
    {"low", TUYA_AC_FAN_LOW},
    {"lowest", TUYA_AC_FAN_LOWEST}
};

static const struct {
    const char *name;
    TuyaAC_HSwing_e mode;
} hSwingMap[] = {
    {"Keep Position", TUYA_AC_HSWING_KEEP},
    {"Auto L/R", TUYA_AC_HSWING_AUTO},
    {"Flow Left", TUYA_AC_HSWING_FLOW_LEFT},
    {"Flow Middle", TUYA_AC_HSWING_FLOW_MID},
    {"Flow Right", TUYA_AC_HSWING_FLOW_RIGHT},
    {"Fix Left", TUYA_AC_HSWING_FIX_LEFT},
    {"Fix Mid-Left", TUYA_AC_HSWING_FIX_MID_LEFT},
    {"Fix Middle", TUYA_AC_HSWING_FIX_MID},
    {"Fix Mid-Right", TUYA_AC_HSWING_FIX_MID_RIGHT},
    {"Fix Right", TUYA_AC_HSWING_FIX_RIGHT}
};

static const struct {
    const char *name;
    TuyaAC_VSwing_e mode;
} vSwingMap[] = {
    {"Keep Position", TUYA_AC_VSWING_KEEP},
    {"Auto U/D", TUYA_AC_VSWING_AUTO},
    {"Flow Up", TUYA_AC_VSWING_FLOW_UP},
    {"Flow Down", TUYA_AC_VSWING_FLOW_DOWN},
    {"Fix Above", TUYA_AC_VSWING_FIX_ABOVE},
    {"Fix Mid-High", TUYA_AC_VSWING_FIX_MID_HIGH},
    {"Fix Middle", TUYA_AC_VSWING_FIX_MID},
    {"Fix Mid-Low", TUYA_AC_VSWING_FIX_MID_LOW},
    {"Fix Down", TUYA_AC_VSWING_FIX_DOWN}
};

static const struct {
    const char *name;
    TuyaAC_Sleep_e mode;
} sleepMap[] = {
    {"Off", TUYA_AC_SLEEP_OFF},
    {"Standard", TUYA_AC_SLEEP_STANDARD},
    {"Aged", TUYA_AC_SLEEP_AGED},
    {"Child", TUYA_AC_SLEEP_CHILD}
};

static const struct {
    const char *name;
    TuyaAC_Generator_e mode;
} generatorMap[] = {
    {"Off", TUYA_AC_GEN_OFF},
    {"Level 1", TUYA_AC_GEN_LEVEL_1},
    {"Level 2", TUYA_AC_GEN_LEVEL_2},
    {"Level 3", TUYA_AC_GEN_LEVEL_3}
};

static const char *get_ac_mode_str(TuyaAC_Mode_e mode) {
    for (int i = 0; i < sizeof(acModeMap)/sizeof(acModeMap[0]); i++) {
        if (acModeMap[i].mode == mode) return acModeMap[i].name;
    }
    return "Unknown";
}

static const char *get_ac_fan_str(TuyaAC_FanMode_e mode) {
    for (int i = 0; i < sizeof(fanModeMap)/sizeof(fanModeMap[0]); i++) {
        if (fanModeMap[i].mode == mode) return fanModeMap[i].name;
    }
    return "Unknown";
}

static const char *get_ac_fan_mute_str(void) {
    static char buf[32];
    if (g_tuya_ac.mute) {
        if (g_tuya_ac.fan == TUYA_AC_FAN_AUTO) {
            return "mute";
        } else {
            snprintf(buf, sizeof(buf), "mute_%s", get_ac_fan_str(g_tuya_ac.fan));
            return buf;
        }
    }
    return get_ac_fan_str(g_tuya_ac.fan);
}

static const char *get_ac_h_swing_str(TuyaAC_HSwing_e mode) {
    for (int i = 0; i < sizeof(hSwingMap)/sizeof(hSwingMap[0]); i++) {
        if (hSwingMap[i].mode == mode) return hSwingMap[i].name;
    }
    return "Unknown";
}

static const char *get_ac_v_swing_str(TuyaAC_VSwing_e mode) {
    for (int i = 0; i < sizeof(vSwingMap)/sizeof(vSwingMap[0]); i++) {
        if (vSwingMap[i].mode == mode) return vSwingMap[i].name;
    }
    return "Unknown";
}

static const char *get_ac_sleep_str(TuyaAC_Sleep_e mode) {
    for (int i = 0; i < sizeof(sleepMap)/sizeof(sleepMap[0]); i++) {
        if (sleepMap[i].mode == mode) return sleepMap[i].name;
    }
    return "Unknown";
}

static const char *get_ac_generator_str(TuyaAC_Generator_e mode) {
    for (int i = 0; i < sizeof(generatorMap)/sizeof(generatorMap[0]); i++) {
        if (generatorMap[i].mode == mode) return generatorMap[i].name;
    }
    return "Unknown";
}

static uint16_t calculate_crc16_xmodem(uint8_t *data, int len) {
    uint16_t crc = 0x0000;
    for (int i = 0; i < len; i++) {
        crc ^= (data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

static const char *TuyaAC_CommandSourceStr(int cmdFlags) {
    if (cmdFlags & COMMAND_FLAG_SOURCE_MQTT) return "MQTT";
    if (cmdFlags & COMMAND_FLAG_SOURCE_CONSOLE) return "CONSOLE";
    if (cmdFlags & COMMAND_FLAG_SOURCE_HTTP) return "HTTP";
    if (cmdFlags & COMMAND_FLAG_SOURCE_SCRIPT) return "SCRIPT";
    if (cmdFlags & COMMAND_FLAG_SOURCE_TCP) return "TCP";
    if (cmdFlags & COMMAND_FLAG_SOURCE_IR) return "IR";
    if (cmdFlags & COMMAND_FLAG_SOURCE_TELESENDER) return "TELE";
    return "UNKNOWN";
}

static void TuyaAC_LogCommand(const char *cmd, const char *args, int cmdFlags) {
    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "CMD %s SRC=%s ARGS='%s'", cmd, TuyaAC_CommandSourceStr(cmdFlags), args ? args : "");
}

static int TuyaAC_GetDpLength(uint16_t dp_id, const uint8_t *payload, int idx, int payload_len) {
    switch(dp_id) {
        case 0x0001: case 0x0013: case 0x000C: case 0x000D:
        case 0x000E: case 0x0011: case 0x0015: case 0x0017: case 0x001E:
        case 0x0020: case 0x0021: case 0x0022: case 0x0023: case 0x0024:
        case 0x0025: case 0x0027: case 0x002D: case 0x0035: case 0x0038:
        case 0x0040: case 0x0042: case 0x0046: case 0x0047: case 0x0048:
        case 0x00D5: case 0x00D6: case 0x00DF: case 0x0012:
        case 0x0147: case 0x0148: case 0x0220: case 0x0224: case 0x0225:
        case 0x0074: case 0x007B: case 0x0084: case 0x00A0: case 0x00C9:
        case 0x00D1: case 0x00D4: case 0x005E: case 0x0073: case 0x00A4:
            return 1;

        case 0x0005:
            return 2;

        case 0x0002: case 0x0003: case 0x003D: case 0x005C: case 0x0060:
        case 0x0064: case 0x0065: case 0x0072: case 0x008C: case 0x0095:
        case 0x00BD: case 0x00BE: case 0x00BF: case 0x00C0: case 0x00D7:
        case 0x0221: case 0x0222: case 0x0227:
            return 4;

        case 0x00FA: case 0x00FB: case 0x0223:
            if (idx + 2 <= payload_len) {
                return 2 + ((payload[idx] << 8) | payload[idx+1]);
            }
            return -1;

        case 0x0039:
            if (idx + 1 <= payload_len) {
                return 1 + payload[idx];
            }
            return -1;
    }
    return -1;
}

static int TuyaAC_IsKnownDpId(uint16_t dp_id) {
    switch (dp_id) {
        case 0x0001: case 0x0002: case 0x0003: case 0x0005: case 0x000C:
        case 0x000D: case 0x000E: case 0x0011: case 0x0012: case 0x0015:
        case 0x0013:
        case 0x0017: case 0x001E: case 0x0020: case 0x0021: case 0x0022:
        case 0x0023: case 0x0024: case 0x0025: case 0x0027: case 0x002D:
        case 0x0035: case 0x0038: case 0x0039: case 0x003D: case 0x0040:
        case 0x0042: case 0x0046: case 0x0047: case 0x0048: case 0x005C:
        case 0x005E: case 0x0060: case 0x0064: case 0x0065: case 0x0072:
        case 0x0073: case 0x0074: case 0x007B: case 0x0084: case 0x008C:
        case 0x0095: case 0x00A0: case 0x00A4: case 0x00BD: case 0x00BE:
        case 0x00BF: case 0x00C0: case 0x00C9: case 0x00D1: case 0x00D4:
        case 0x00D5: case 0x00D6: case 0x00D7: case 0x00DF: case 0x00FA:
        case 0x00FB: case 0x0147: case 0x0148: case 0x0220: case 0x0221:
        case 0x0222: case 0x0223: case 0x0224: case 0x0225: case 0x0227:
            return 1;
    }
    return 0;
}

static int TuyaAC_AdjustDpLengthHeuristic(uint16_t dp_id, const uint8_t *payload, int idx, int payload_len, int currentLen) {
    // Some AC reports encode fan auto in a compact way. Try both 1 and 2-byte fan lengths
    // and pick the one that keeps parser aligned with next known DP.
    if (dp_id == 0x0005) {
        int len1 = 1;
        int len2 = 2;
        int next1Known = 0;
        int next2Known = 0;

        if ((idx + len1 + 1) < payload_len) {
            uint16_t next1 = (payload[idx + len1] << 8) | payload[idx + len1 + 1];
            next1Known = TuyaAC_IsKnownDpId(next1);
        }
        if ((idx + len2 + 1) < payload_len) {
            uint16_t next2 = (payload[idx + len2] << 8) | payload[idx + len2 + 1];
            next2Known = TuyaAC_IsKnownDpId(next2);
        }

        if (!next2Known && next1Known) {
            return len1;
        }
        if (next2Known && !next1Known) {
            return len2;
        }
    }

    return currentLen;
}

static void TuyaAC_LogDpDecode(const char *dir, uint16_t dp_id, const uint8_t *dp, int dp_len) {
    uint32_t val32 = 0;
    uint16_t val16 = 0;
    if (dp_len >= 2) {
        val16 = (dp[0] << 8) | dp[1];
    }
    if (dp_len >= 4) {
        val32 = (dp[0] << 24) | (dp[1] << 16) | (dp[2] << 8) | dp[3];
    }
    switch (dp_id) {
        case 0x0001:
            ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] POWER=%s", dir, dp_id, dp[0] ? "ON" : "OFF");
            break;
        case 0x0013:
            ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] LOCK=%s", dir, dp_id, dp[0] ? "ON" : "OFF");
            break;
        case 0x0002:
            ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] TARGET=%.1fC", dir, dp_id, val32 / 100.0f);
            break;
        case 0x0005:
            ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] FAN=%s RAW=0x%04X", dir, dp_id, get_ac_fan_str((TuyaAC_FanMode_e)(val16 >> 8)), val16);
            break;
        case 0x0012:
            ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] MODE=%s RAW=0x%02X", dir, dp_id, get_ac_mode_str((TuyaAC_Mode_e)dp[0]), dp[0]);
            break;
        case 0x0073:
            ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] MUTE=%s", dir, dp_id, dp[0] ? "ON" : "OFF");
            break;
        default:
            if (dp_len == 1) {
                ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] V8=0x%02X", dir, dp_id, dp[0]);
            } else if (dp_len == 2) {
                ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] V16=0x%04X", dir, dp_id, val16);
            } else if (dp_len == 4) {
                ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] V32=%u", dir, dp_id, val32);
            } else {
                ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s DP[0x%04X] LEN=%d", dir, dp_id, dp_len);
            }
            break;
    }
}

static void TuyaAC_LogPayloadDp(const char *dir, const uint8_t *payload, int payload_len) {
    int idx = 0;
    while (idx + 2 <= payload_len) {
        uint16_t dp_id = (payload[idx] << 8) | payload[idx + 1];
        idx += 2;
        int dp_len = TuyaAC_GetDpLength(dp_id, payload, idx, payload_len);
        if (dp_len < 0 || (idx + dp_len) > payload_len) {
            ADDLOG_WARN(LOG_FEATURE_TUYA_AC, "%s DP decode stop at 0x%04X idx=%d len=%d", dir, dp_id, idx, payload_len);
            return;
        }
        TuyaAC_LogDpDecode(dir, dp_id, &payload[idx], dp_len);
        idx += dp_len;
    }
}

static void printHexToLog(const char* prefix, uint8_t *data, int len, uint16_t crc) {
    char hexstr[256] = {0};
    int limit = len;
    if (limit > 80) limit = 80;
    for(int i = 0; i < limit; i++) {
        snprintf(hexstr + (i*3), sizeof(hexstr) - (i*3), "%02X ", data[i]);
    }
    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s LEN=%d CRC=0x%04X %s", prefix, len, crc, hexstr);
}

static int TuyaAC_IsCompactRecordCommand(uint8_t b2, uint8_t cmd) {
    if (b2 != TUYA_AC_B2_THING) {
        return 0;
    }
    switch (cmd) {
        case 0x0A:
        case 0x0B:
        case 0x0C:
        case 0x0D:
        case 0x0E:
            return 1;
        default:
            return 0;
    }
}

static int TuyaAC_ShouldAckRequest(uint8_t b2, uint8_t cmd) {
    if (b2 == TUYA_AC_B2_THING) {
        switch (cmd) {
            case 0x0A: // property write
            case 0x0C: // compact report/notify
            case 0x0D: // compact report/notify
            case 0x0E: // secondary property bucket
                return 1;
            default:
                return 0;
        }
    }

    if (b2 == TUYA_AC_B2_CORE) {
        switch (cmd) {
            case 0x00: // observed MCU presence/core heartbeat probe
            case 0x11: // MCU init path from the decompiled GFW command table
                return 1;
            default:
                return 0;
        }
    }

    return 0;
}

static int TuyaAC_IsQueryAllPayload(const uint8_t *payload, int payload_len) {
    return payload_len == 2 && payload[0] == 0xFF && payload[1] == 0xFF;
}

static void TuyaAC_SendPacketEx(uint8_t b2, uint8_t frame_class, uint8_t seq, uint16_t cmd_flag, uint8_t *payload, uint16_t payload_len) {
    uint16_t packet_len = 12 + payload_len; // 12 bytes header/crc + payload
    uint8_t buffer[256];
    if (packet_len > sizeof(buffer)) return;

    buffer[0] = TUYA_AC_A5_MAGIC;
    buffer[1] = TUYA_AC_B1_DEFAULT;
    buffer[2] = b2;
    buffer[3] = frame_class;

    if (frame_class == TUYA_AC_FRAME_RESPONSE) {
        buffer[4] = 0x00;
        buffer[5] = seq;
    } else {
        buffer[4] = seq;
        buffer[5] = 0x00;
    }

    buffer[6] = (packet_len >> 8) & 0xFF;
    buffer[7] = packet_len & 0xFF;

    // placeholder for CRC
    buffer[8] = 0x00;
    buffer[9] = 0x00;

    buffer[10] = (cmd_flag >> 8) & 0xFF;
    buffer[11] = cmd_flag & 0xFF;

    if (payload_len > 0 && payload != NULL) {
        memcpy(&buffer[12], payload, payload_len);
    }

    // CRC is calculated over b[0..7] + b[10..end], excluding bytes 8/9.
    uint8_t data_to_check[256];
    memcpy(data_to_check, buffer, 8);
    memcpy(&data_to_check[8], &buffer[10], 2 + payload_len);

    uint16_t calc_crc = calculate_crc16_xmodem(data_to_check, 10 + payload_len);
    buffer[8] = (calc_crc >> 8) & 0xFF;
    buffer[9] = calc_crc & 0xFF;

    printHexToLog("TX UART:", buffer, packet_len, calc_crc);
    if (frame_class == TUYA_AC_FRAME_REQUEST && b2 == TUYA_AC_B2_THING && cmd_flag == 0x0A0A && payload != NULL && payload_len > 0) {
        TuyaAC_LogPayloadDp("TX", payload, payload_len);
    }

    for (int i = 0; i < packet_len; i++) {
        UART_SendByte(buffer[i]);
    }
}

static void TuyaAC_SendPacket(uint8_t frame_class, uint16_t cmd_flag, uint8_t *payload, uint16_t payload_len) {
    TuyaAC_SendPacketEx(TUYA_AC_B2_THING, frame_class, g_tuya_ac.seq++, cmd_flag, payload, payload_len);
}

static void TuyaAC_SendRequestOnChannel(uint8_t b2, uint8_t cmd, uint8_t *payload, uint16_t payload_len) {
    uint16_t cmd_flag = ((uint16_t)cmd << 8) | cmd;
    TuyaAC_SendPacketEx(b2, TUYA_AC_FRAME_REQUEST, g_tuya_ac.seq++, cmd_flag, payload, payload_len);
}

static void TuyaAC_SendAck(uint8_t b2, uint8_t request_seq, uint8_t cmd, uint8_t resp_flag) {
    uint16_t cmd_flag = ((uint16_t)(0x80 | (resp_flag & 0x7F)) << 8) | cmd;
    TuyaAC_SendPacketEx(b2, TUYA_AC_FRAME_RESPONSE, request_seq, cmd_flag, NULL, 0);
}

static void TuyaAC_SendDP(uint16_t dp_id, uint8_t *data, uint16_t len) {
    uint8_t payload[256];
    payload[0] = (dp_id >> 8) & 0xFF;
    payload[1] = dp_id & 0xFF;
    memcpy(&payload[2], data, len);
    
    TuyaAC_SendPacket(0x21, 0x0A0A, payload, 2 + len);
}

static void TuyaAC_AppendDpBool(uint8_t *payload, int *pLen, uint16_t dp_id, uint8_t val) {
    payload[(*pLen)++] = (dp_id >> 8) & 0xFF;
    payload[(*pLen)++] = dp_id & 0xFF;
    payload[(*pLen)++] = val;
}

static void TuyaAC_AppendDpEnum16(uint8_t *payload, int *pLen, uint16_t dp_id, uint8_t val) {
    payload[(*pLen)++] = (dp_id >> 8) & 0xFF;
    payload[(*pLen)++] = dp_id & 0xFF;
    payload[(*pLen)++] = val;
    payload[(*pLen)++] = 0x00;
}

static void TuyaAC_SendDP_Bool(uint16_t dp_id, uint8_t val) {
    TuyaAC_SendDP(dp_id, &val, 1);
}

static void TuyaAC_SendDP_Value(uint16_t dp_id, uint32_t val) {
    uint8_t data[4];
    data[0] = (val >> 24) & 0xFF;
    data[1] = (val >> 16) & 0xFF;
    data[2] = (val >> 8) & 0xFF;
    data[3] = val & 0xFF;
    TuyaAC_SendDP(dp_id, data, 4);
}

static void TuyaAC_SendDP_Enum(uint16_t dp_id, uint8_t val) {
    TuyaAC_SendDP(dp_id, &val, 1);
}

static void TuyaAC_SendDP_Enum16(uint16_t dp_id, uint8_t val) {
    uint8_t data[2];
    data[0] = val;
    data[1] = 0x00;
    TuyaAC_SendDP(dp_id, data, 2);
}

static void TuyaAC_RequestSnapshotPublish(void) {
    g_tuya_ac_snapshot_pending = 1;
    g_tuya_ac_snapshot_index = 0;
}

static int TuyaAC_PublishStateSnapshotStep(void) {
    switch (g_tuya_ac_snapshot_index) {
        case 0: MQTT_PublishMain_StringString("ac_power", g_tuya_ac.power ? "On" : "Off", 0); break;
        case 1: MQTT_PublishMain_StringString("ac_mode", g_tuya_ac.power ? get_ac_mode_str(g_tuya_ac.mode) : "off", 0); break;
        case 2: MQTT_PublishMain_StringString("ac_fan", get_ac_fan_mute_str(), 0); break;
        case 3: MQTT_PublishMain_StringFloat("ac_target_temp", g_tuya_ac.target_temp, 1, 0); break;
        case 4: MQTT_PublishMain_StringFloat("ac_current_temp", g_tuya_ac.current_temp, 1, 0); break;
        case 5: MQTT_PublishMain_StringString("ac_h_swing", get_ac_h_swing_str(g_tuya_ac.h_swing), 0); break;
        case 6: MQTT_PublishMain_StringString("ac_v_swing", get_ac_v_swing_str(g_tuya_ac.v_swing), 0); break;
        case 7: MQTT_PublishMain_StringString("ac_health", g_tuya_ac.health ? "On" : "Off", 0); break;
        case 8: MQTT_PublishMain_StringString("ac_display", g_tuya_ac.display ? "On" : "Off", 0); break;
        case 9: MQTT_PublishMain_StringString("ac_sleep", get_ac_sleep_str(g_tuya_ac.sleep), 0); break;
        case 10: MQTT_PublishMain_StringString("ac_buzzer", g_tuya_ac.buzzer ? "On" : "Off", 0); break;
        case 11: MQTT_PublishMain_StringString("ac_generator", get_ac_generator_str(g_tuya_ac.generator), 0); break;
        case 12: MQTT_PublishMain_StringString("ac_mute", g_tuya_ac.mute ? "On" : "Off", 0); break;
        case 13: MQTT_PublishMain_StringString("ac_eco", g_tuya_ac.eco ? "On" : "Off", 0); break;
        case 14: MQTT_PublishMain_StringString("ac_v_motor", g_tuya_ac.v_motor ? "Moving" : "Stopped", 0); break;
        case 15: MQTT_PublishMain_StringString("ac_h_motor", g_tuya_ac.h_motor ? "Moving" : "Stopped", 0); break;
        case 16: MQTT_PublishMain_StringInt("ac_energy", g_tuya_ac.energy, 0); break;
        case 17: MQTT_PublishMain_StringInt("ac_in_fan_rpm", g_tuya_ac.in_fan_rpm, 0); break;
        case 18: MQTT_PublishMain_StringInt("ac_in_fan_percent", g_tuya_ac.in_fan_percent, 0); break;
        case 19: MQTT_PublishMain_StringFloat("ac_out_temp", g_tuya_ac.out_temp, 1, 0); break;
        case 20: MQTT_PublishMain_StringInt("ac_out_fan_rpm", g_tuya_ac.out_fan_rpm, 0); break;
        case 21: MQTT_PublishMain_StringInt("ac_runtime", g_tuya_ac.runtime, 0); break;
        case 22: MQTT_PublishMain_StringInt("ac_filter", g_tuya_ac.filter, 0); break;
        case 23: MQTT_PublishMain_StringInt("ac_compressor_hz", g_tuya_ac.compressor_hz, 0); break;
        case 24: MQTT_PublishMain_StringString("ac_eight_degree", g_tuya_ac.eight_degree ? "On" : "Off", 0); break;
        default: return 0;
    }
    g_tuya_ac_snapshot_index++;
    return 1;
}

// Flush all dirty parameters to MQTT in one batch
static void TuyaAC_FlushDirtyToMQTT(void) {
    if (!g_tuya_ac_dirty_flags) return;

    int p = 0;
#define FLUSH(f, code) if (g_tuya_ac_dirty_flags & (f)) { code; g_tuya_ac_dirty_flags &= ~(f); if (++p >= g_tuya_ac_mqtt_batch_size) { g_tuya_ac_publish_countdown = g_tuya_ac_mqtt_batch_cooldown; return; } }

    if (g_tuya_ac_dirty_flags & DIRTY_POWER) {
        MQTT_PublishMain_StringString("ac_power", g_tuya_ac.power ? "On" : "Off", 0);
        g_tuya_ac_dirty_flags &= ~DIRTY_POWER;
        g_tuya_ac_dirty_flags |= DIRTY_MODE;
        if (++p >= g_tuya_ac_mqtt_batch_size) { g_tuya_ac_publish_countdown = g_tuya_ac_mqtt_batch_cooldown; return; }
    }
    FLUSH(DIRTY_MODE, MQTT_PublishMain_StringString("ac_mode", g_tuya_ac.power ? get_ac_mode_str(g_tuya_ac.mode) : "off", 0));

    if (g_tuya_ac_dirty_flags & DIRTY_MUTE) {
        MQTT_PublishMain_StringString("ac_mute", g_tuya_ac.mute ? "On" : "Off", 0);
        g_tuya_ac_dirty_flags &= ~DIRTY_MUTE;
        g_tuya_ac_dirty_flags |= DIRTY_FAN;
        if (++p >= g_tuya_ac_mqtt_batch_size) { g_tuya_ac_publish_countdown = g_tuya_ac_mqtt_batch_cooldown; return; }
    }
    FLUSH(DIRTY_FAN, MQTT_PublishMain_StringString("ac_fan", get_ac_fan_mute_str(), 0));

    FLUSH(DIRTY_TARGET_TEMP, MQTT_PublishMain_StringFloat("ac_target_temp", g_tuya_ac.target_temp, 1, 0));
    FLUSH(DIRTY_CURRENT_TEMP, MQTT_PublishMain_StringFloat("ac_current_temp", g_tuya_ac.current_temp, 1, 0));
    FLUSH(DIRTY_H_SWING, MQTT_PublishMain_StringString("ac_h_swing", get_ac_h_swing_str(g_tuya_ac.h_swing), 0));
    FLUSH(DIRTY_V_SWING, MQTT_PublishMain_StringString("ac_v_swing", get_ac_v_swing_str(g_tuya_ac.v_swing), 0));
    FLUSH(DIRTY_HEALTH, MQTT_PublishMain_StringString("ac_health", g_tuya_ac.health ? "On" : "Off", 0));
    FLUSH(DIRTY_DISPLAY, MQTT_PublishMain_StringString("ac_display", g_tuya_ac.display ? "On" : "Off", 0));
    FLUSH(DIRTY_SLEEP, MQTT_PublishMain_StringString("ac_sleep", get_ac_sleep_str(g_tuya_ac.sleep), 0));
    FLUSH(DIRTY_BUZZER, MQTT_PublishMain_StringString("ac_buzzer", g_tuya_ac.buzzer ? "On" : "Off", 0));
    FLUSH(DIRTY_GENERATOR, MQTT_PublishMain_StringString("ac_generator", get_ac_generator_str(g_tuya_ac.generator), 0));
    FLUSH(DIRTY_ECO, MQTT_PublishMain_StringString("ac_eco", g_tuya_ac.eco ? "On" : "Off", 0));
    FLUSH(DIRTY_V_MOTOR, MQTT_PublishMain_StringString("ac_v_motor", g_tuya_ac.v_motor ? "Moving" : "Stopped", 0));
    FLUSH(DIRTY_H_MOTOR, MQTT_PublishMain_StringString("ac_h_motor", g_tuya_ac.h_motor ? "Moving" : "Stopped", 0));
    FLUSH(DIRTY_ENERGY, MQTT_PublishMain_StringInt("ac_energy", g_tuya_ac.energy, 0));
    FLUSH(DIRTY_IN_FAN_RPM, MQTT_PublishMain_StringInt("ac_in_fan_rpm", g_tuya_ac.in_fan_rpm, 0));
    FLUSH(DIRTY_IN_FAN_PERCENT, MQTT_PublishMain_StringInt("ac_in_fan_percent", g_tuya_ac.in_fan_percent, 0));
    FLUSH(DIRTY_OUT_TEMP, MQTT_PublishMain_StringFloat("ac_out_temp", g_tuya_ac.out_temp, 1, 0));
    FLUSH(DIRTY_OUT_FAN_RPM, MQTT_PublishMain_StringInt("ac_out_fan_rpm", g_tuya_ac.out_fan_rpm, 0));
    FLUSH(DIRTY_RUNTIME, MQTT_PublishMain_StringInt("ac_runtime", g_tuya_ac.runtime, 0));
    FLUSH(DIRTY_FILTER, MQTT_PublishMain_StringInt("ac_filter", g_tuya_ac.filter, 0));
    FLUSH(DIRTY_COMPRESSOR_HZ, MQTT_PublishMain_StringInt("ac_compressor_hz", g_tuya_ac.compressor_hz, 0));
    FLUSH(DIRTY_EIGHT_DEGREE, MQTT_PublishMain_StringString("ac_eight_degree", g_tuya_ac.eight_degree ? "On" : "Off", 0));

    g_tuya_ac_publish_countdown = TUYA_AC_PUBLISH_INTERVAL_SEC;
#undef FLUSH
}

static TuyaAC_Mode_e parseACMode(const char *s) {
    for (int i = 0; i < sizeof(acModeMap)/sizeof(acModeMap[0]); i++) {
        if (!stricmp(s, acModeMap[i].name)) return acModeMap[i].mode;
    }
    return (TuyaAC_Mode_e)atoi(s);
}

static TuyaAC_FanMode_e parseFanMode(const char *s) {
    for (int i = 0; i < sizeof(fanModeMap)/sizeof(fanModeMap[0]); i++) {
        if (!stricmp(s, fanModeMap[i].name)) return fanModeMap[i].mode;
    }
    return (TuyaAC_FanMode_e)atoi(s);
}

static TuyaAC_HSwing_e parseHSwingMode(const char *s) {
    for (int i = 0; i < sizeof(hSwingMap)/sizeof(hSwingMap[0]); i++) {
        if (!stricmp(s, hSwingMap[i].name)) return hSwingMap[i].mode;
    }
    return (TuyaAC_HSwing_e)atoi(s);
}

static TuyaAC_VSwing_e parseVSwingMode(const char *s) {
    for (int i = 0; i < sizeof(vSwingMap)/sizeof(vSwingMap[0]); i++) {
        if (!stricmp(s, vSwingMap[i].name)) return vSwingMap[i].mode;
    }
    return (TuyaAC_VSwing_e)atoi(s);
}

static TuyaAC_Sleep_e parseSleepMode(const char *s) {
    for (int i = 0; i < sizeof(sleepMap)/sizeof(sleepMap[0]); i++) {
        if (!stricmp(s, sleepMap[i].name)) return sleepMap[i].mode;
    }
    return (TuyaAC_Sleep_e)atoi(s);
}

static TuyaAC_Generator_e parseGeneratorMode(const char *s) {
    for (int i = 0; i < sizeof(generatorMap)/sizeof(generatorMap[0]); i++) {
        if (!stricmp(s, generatorMap[i].name)) return generatorMap[i].mode;
    }
    return (TuyaAC_Generator_e)atoi(s);
}

static commandResult_t CMD_TuyaAC_Power(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.power = Tokenizer_GetArgInteger(0);
    
    // Send Lock DP 19 first, then Power
    TuyaAC_SendDP_Bool(0x01, g_tuya_ac.power ? 1 : 0);
    
    TuyaAC_MarkDpConfigured(0x01);
    return CMD_RES_OK;
}

static void TuyaAC_SendModePacket(TuyaAC_Mode_e mode) {
    uint8_t payload[8];
    int plen = 0;

    // Send Power On as part of the same packet if currently OFF
    if (g_tuya_ac.power == 0) {
        payload[plen++] = 0x00;
        payload[plen++] = 0x01;
        payload[plen++] = 0x01; // Power On
        TuyaAC_MarkDpConfigured(0x01);
    }

    // Send Mode
    payload[plen++] = 0x00;
    payload[plen++] = 0x12;
    payload[plen++] = (uint8_t)mode;

    TuyaAC_SendPacket(0x21, 0x0A0A, payload, plen);
}

static commandResult_t CMD_TuyaAC_Mode(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    const char *modeStr = Tokenizer_GetArgFrom(0);
    if (!modeStr) return CMD_RES_OK;

    if (!stricmp(modeStr, "off") || !strcmp(modeStr, "0")) {
        g_tuya_ac.power = 0;
        uint8_t payload[8];
        int plen = 0;
        payload[plen++] = 0x00;
        payload[plen++] = 0x01;
        payload[plen++] = 0x00;
        TuyaAC_SendPacket(0x21, 0x0A0A, payload, plen);
        
        TuyaAC_MarkDpConfigured(0x01);
        TuyaAC_MarkDpConfigured(0x12);
        return CMD_RES_OK;
    }

    g_tuya_ac.mode = parseACMode(modeStr);
    
    TuyaAC_SendModePacket(g_tuya_ac.mode);
    g_tuya_ac.power = 1;
    
    TuyaAC_MarkDpConfigured(0x12);
    TuyaAC_MarkDpConfigured(0x01);
    return CMD_RES_OK;
}

static void TuyaAC_SendFanMutePacket(TuyaAC_FanMode_e fan, int mute) {
    uint8_t payload[16];
    int plen = 0;

    if (mute) {
        // Mute ON: fan=0, mute=1 (00 05 00 00 73 01)
        payload[plen++] = 0x00;
        payload[plen++] = 0x05;
        payload[plen++] = 0x00;
        payload[plen++] = 0x00;
        payload[plen++] = 0x73;
        payload[plen++] = 0x01;
    } else {
        switch (fan) {
            case TUYA_AC_FAN_AUTO:
                // Auto: 00 05 00 00 73 01 (and forces mute=1)
                payload[plen++] = 0x00;
                payload[plen++] = 0x05;
                payload[plen++] = 0x00;
                payload[plen++] = 0x00;
                payload[plen++] = 0x73;
                payload[plen++] = 0x01;
                g_tuya_ac.mute = 1;
                break;
            case TUYA_AC_FAN_LOWEST:
                // Lowest: 00 05 01 00 73 00
                payload[plen++] = 0x00;
                payload[plen++] = 0x05;
                payload[plen++] = 0x01;
                payload[plen++] = 0x00;
                payload[plen++] = 0x73;
                payload[plen++] = 0x00;
                break;
            case TUYA_AC_FAN_LOW:
                // Low: 00 05 02 00 73 00
                payload[plen++] = 0x00;
                payload[plen++] = 0x05;
                payload[plen++] = 0x02;
                payload[plen++] = 0x00;
                payload[plen++] = 0x73;
                payload[plen++] = 0x00;
                break;
            case TUYA_AC_FAN_MID_LOW:
                // Mid-Low: 00 05 03 00 73 00
                payload[plen++] = 0x00;
                payload[plen++] = 0x05;
                payload[plen++] = 0x03;
                payload[plen++] = 0x00;
                payload[plen++] = 0x73;
                payload[plen++] = 0x00;
                break;
            case TUYA_AC_FAN_MID:
                // Mid: 00 05 04 00 73 00
                payload[plen++] = 0x00;
                payload[plen++] = 0x05;
                payload[plen++] = 0x04;
                payload[plen++] = 0x00;
                payload[plen++] = 0x73;
                payload[plen++] = 0x00;
                break;
            case TUYA_AC_FAN_MID_HIGH:
                // Mid-High: 00 05 05 00 73 00
                payload[plen++] = 0x00;
                payload[plen++] = 0x05;
                payload[plen++] = 0x05;
                payload[plen++] = 0x00;
                payload[plen++] = 0x73;
                payload[plen++] = 0x00;
                break;
            case TUYA_AC_FAN_HIGH:
                // High: 00 05 06 00 73 00
                payload[plen++] = 0x00;
                payload[plen++] = 0x05;
                payload[plen++] = 0x06;
                payload[plen++] = 0x00;
                payload[plen++] = 0x73;
                payload[plen++] = 0x00;
                break;
            case TUYA_AC_FAN_TURBO:
                // Turbo: 00 26 00 00 05 07 00 73 00
                payload[plen++] = 0x00;
                payload[plen++] = 0x26;
                payload[plen++] = 0x00;
                payload[plen++] = 0x00;
                payload[plen++] = 0x05;
                payload[plen++] = 0x07;
                payload[plen++] = 0x00;
                payload[plen++] = 0x73;
                payload[plen++] = 0x00;
                break;
            default:
                break;
        }
    }

    if (plen > 0) {
        TuyaAC_SendPacket(0x21, 0x0A0A, payload, plen);
    }
}

static commandResult_t CMD_TuyaAC_Fan(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    const char *str = Tokenizer_GetArgFrom(0);
    if (!str) return CMD_RES_OK;
    
    g_tuya_ac.fan = parseFanMode(str);
    if (g_tuya_ac.fan == TUYA_AC_FAN_AUTO) {
        g_tuya_ac.mute = 1;
    } else {
        g_tuya_ac.mute = 0;
    }
    
    TuyaAC_SendFanMutePacket(g_tuya_ac.fan, g_tuya_ac.mute);
    TuyaAC_MarkDpConfigured(0x05);
    TuyaAC_MarkDpConfigured(0x73);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_TargetTemp(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.target_temp = Tokenizer_GetArgFloat(0);
    uint32_t val = (uint32_t)(g_tuya_ac.target_temp * 100.0f);
    TuyaAC_SendDP_Value(0x02, val);
    TuyaAC_MarkDpConfigured(0x02);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_EightDegree(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.eight_degree = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x0147, g_tuya_ac.eight_degree ? 1 : 0);
    TuyaAC_MarkDpConfigured(0x0147);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_InFanPercent(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.in_fan_percent = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Value(0x0072, g_tuya_ac.in_fan_percent);
    TuyaAC_MarkDpConfigured(0x0072);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_HSwing(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    const char *str = Tokenizer_GetArgFrom(0);
    if (!str) return CMD_RES_OK;
    
    g_tuya_ac.h_swing = parseHSwingMode(str);
    TuyaAC_SendDP_Enum(0x0E, g_tuya_ac.h_swing);
    TuyaAC_MarkDpConfigured(0x0E);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_VSwing(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    const char *str = Tokenizer_GetArgFrom(0);
    if (!str) return CMD_RES_OK;
    
    g_tuya_ac.v_swing = parseVSwingMode(str);
    TuyaAC_SendDP_Enum(0x11, g_tuya_ac.v_swing);
    TuyaAC_MarkDpConfigured(0x11);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Sleep(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    const char *str = Tokenizer_GetArgFrom(0);
    if (!str) return CMD_RES_OK;
    
    g_tuya_ac.sleep = parseSleepMode(str);
    TuyaAC_SendDP_Enum(0x22, g_tuya_ac.sleep);
    TuyaAC_MarkDpConfigured(0x22);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Generator(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    const char *str = Tokenizer_GetArgFrom(0);
    if (!str) return CMD_RES_OK;
    
    g_tuya_ac.generator = parseGeneratorMode(str);
    TuyaAC_SendDP_Enum(0x2D, g_tuya_ac.generator);
    TuyaAC_MarkDpConfigured(0x2D);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Eco(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.eco = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0xDF, g_tuya_ac.eco ? 1 : 0);
    TuyaAC_MarkDpConfigured(0xDF);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Mute(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.mute = Tokenizer_GetArgInteger(0);
    if (g_tuya_ac.mute) {
        g_tuya_ac.fan = TUYA_AC_FAN_AUTO;
    }
    
    TuyaAC_SendFanMutePacket(g_tuya_ac.fan, g_tuya_ac.mute);
    TuyaAC_MarkDpConfigured(0x73);
    TuyaAC_MarkDpConfigured(0x05);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Buzzer(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.buzzer = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x25, g_tuya_ac.buzzer ? 1 : 0);
    TuyaAC_MarkDpConfigured(0x25);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Display(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.display = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x1E, g_tuya_ac.display ? 1 : 0);
    TuyaAC_MarkDpConfigured(0x1E);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Health(const void* context, const char* cmd, const char* args, int cmdFlags) {
    TuyaAC_LogCommand(cmd, args, cmdFlags);
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.health = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x15, g_tuya_ac.health ? 1 : 0);
    TuyaAC_MarkDpConfigured(0x15);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_MqttBatch(const void* context, const char* cmd, const char* args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_GetArgsCount() >= 1) {
        g_tuya_ac_mqtt_batch_size = Tokenizer_GetArgInteger(0);
    }
    if (Tokenizer_GetArgsCount() >= 2) {
        g_tuya_ac_mqtt_batch_cooldown = Tokenizer_GetArgInteger(1);
    }
    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "MQTT Batch limits: %d items per %d seconds", g_tuya_ac_mqtt_batch_size, g_tuya_ac_mqtt_batch_cooldown);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_MqttSnapshotInterval(const void* context, const char* cmd, const char* args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_GetArgsCount() >= 1) {
        g_tuya_ac_snapshot_interval = Tokenizer_GetArgInteger(0);
        g_tuya_ac_snapshot_timer = 0;
    }
    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "MQTT Snapshot interval: %d seconds", g_tuya_ac_snapshot_interval);
    return CMD_RES_OK;
}

void TuyaAC_Init(void) {
    UART_InitReceiveRingBuffer(TUYA_AC_RX_BUFFER_SIZE);
    UART_InitUART(TUYA_AC_BAUDRATE, 0, false); // Initialize UART without TX queue/buffers perhaps? port 0? parity 0!

    CMD_RegisterCommand("ACMqttSnapshotInterval", CMD_TuyaAC_MqttSnapshotInterval, NULL);
    CMD_RegisterCommand("ACMqttBatch", CMD_TuyaAC_MqttBatch, NULL);
    CMD_RegisterCommand("ACMode", CMD_TuyaAC_Mode, NULL);
    CMD_RegisterCommand("FANMode", CMD_TuyaAC_Fan, NULL);
    CMD_RegisterCommand("TargetTemperature", CMD_TuyaAC_TargetTemp, NULL);
    CMD_RegisterCommand("ACPower", CMD_TuyaAC_Power, NULL);
    CMD_RegisterCommand("ACEightDegree", CMD_TuyaAC_EightDegree, NULL);
    CMD_RegisterCommand("ACInFanPercent", CMD_TuyaAC_InFanPercent, NULL);
    CMD_RegisterCommand("ACHSwing", CMD_TuyaAC_HSwing, NULL);
    CMD_RegisterCommand("ACVSwing", CMD_TuyaAC_VSwing, NULL);
    CMD_RegisterCommand("ACSleep", CMD_TuyaAC_Sleep, NULL);
    CMD_RegisterCommand("ACGenerator", CMD_TuyaAC_Generator, NULL);
    CMD_RegisterCommand("ACEco", CMD_TuyaAC_Eco, NULL);
    CMD_RegisterCommand("ACMute", CMD_TuyaAC_Mute, NULL);
    CMD_RegisterCommand("ACBuzzer", CMD_TuyaAC_Buzzer, NULL);
    CMD_RegisterCommand("ACDisplay", CMD_TuyaAC_Display, NULL);
    CMD_RegisterCommand("ACHealth", CMD_TuyaAC_Health, NULL);

    // Send the core MCU presence probe observed in the original RTL8720CF TCL dump:
    // A5 01 00 21 <seq> 00 00 0D <crc> 00 00 01
    uint8_t q_init[1] = {0x01};
    TuyaAC_SendRequestOnChannel(TUYA_AC_B2_CORE, 0x00, q_init, 1);

    // Query all AC/thing properties.  The FF FF query-all form is the only query
    // payload currently backed by the first dump/recon notes, so avoid the
    // unproven 03 01 / 03 05 probes here.
    uint8_t query_payload[2] = {0xFF, 0xFF};
    TuyaAC_SendRequestOnChannel(TUYA_AC_B2_THING, 0x0B, query_payload, 2);
}

void TuyaAC_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
    if (!bPreState) {
        hprintf255(request, "<h3>Tuya AC Driver</h3>");
        hprintf255(request, "Power: %s<br>", g_tuya_ac.power ? "ON" : "OFF");
        hprintf255(request, "Mode: %s<br>", get_ac_mode_str(g_tuya_ac.mode));
        hprintf255(request, "Fan: %s<br>", get_ac_fan_mute_str());
        hprintf255(request, "Target Temp: %.1f &deg;C<br>", g_tuya_ac.target_temp);
        hprintf255(request, "Current Temp: %.1f &deg;C<br>", g_tuya_ac.current_temp);
        hprintf255(request, "Outdoor Temp: %.1f &deg;C<br>", g_tuya_ac.out_temp);
        hprintf255(request, "Indoor Fan: %u RPM<br>", g_tuya_ac.in_fan_rpm);
        hprintf255(request, "Outdoor Fan: %u RPM<br>", g_tuya_ac.out_fan_rpm);
        hprintf255(request, "Compressor: %u Hz<br>", g_tuya_ac.compressor_hz);
        hprintf255(request, "Energy/Elec: %u<br>", g_tuya_ac.energy);
        hprintf255(request, "Eight Degree Heat: %s<br>", g_tuya_ac.eight_degree ? "ON" : "OFF");
        hprintf255(request, "Runtime: %u mins<br>", g_tuya_ac.runtime);
        hprintf255(request, "Filter Health: %u%%<br>", g_tuya_ac.filter);
        hprintf255(request, "H-Swing: %s<br>", get_ac_h_swing_str(g_tuya_ac.h_swing));
        hprintf255(request, "V-Swing: %s<br>", get_ac_v_swing_str(g_tuya_ac.v_swing));
        hprintf255(request, "Display Light: %s<br>", g_tuya_ac.display ? "ON" : "OFF");
        hprintf255(request, "Eco Mode: %s<br>", g_tuya_ac.eco ? "ON" : "OFF");
        hprintf255(request, "Health/Ionizer: %s<br>", g_tuya_ac.health ? "ON" : "OFF");
        hprintf255(request, "Sleep Mode: %s<br>", get_ac_sleep_str(g_tuya_ac.sleep));
        hprintf255(request, "Buzzer: %s<br>", g_tuya_ac.buzzer ? "ON" : "OFF");
        hprintf255(request, "Generator: %s<br>", get_ac_generator_str(g_tuya_ac.generator));
        hprintf255(request, "Mute: %s<br>", g_tuya_ac.mute ? "ON" : "OFF");
    }
}
void TuyaAC_RunEverySecond(void) {
    int mqtt_connected = Main_HasMQTTConnected();

    if (!g_tuya_ac_prev_mqtt_connected && mqtt_connected && g_tuya_ac_have_rx_state) {
        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "MQTT reconnected, scheduling AC state snapshot republish");
        TuyaAC_RequestSnapshotPublish();
        g_tuya_ac_have_published_state = 1;
    }
    g_tuya_ac_prev_mqtt_connected = mqtt_connected;

    if (mqtt_connected) {
        g_tuya_ac_snapshot_timer++;
        if (g_tuya_ac_snapshot_interval > 0 && g_tuya_ac_snapshot_timer >= g_tuya_ac_snapshot_interval) {
            g_tuya_ac_snapshot_timer = 0;
            if (!g_tuya_ac_snapshot_pending && g_tuya_ac_have_rx_state) {
                TuyaAC_RequestSnapshotPublish();
            }
        }
    }

    if (mqtt_connected && g_tuya_ac_snapshot_pending) {
        for (int i = 0; i < 1; i++) {
            if (!TuyaAC_PublishStateSnapshotStep()) {
                g_tuya_ac_snapshot_pending = 0;
                ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "AC state snapshot republish complete");
                break;
            }
        }
    }

    if (g_tuya_ac_instant_publish_timeout > 0) {
        g_tuya_ac_instant_publish_timeout--;
        if (g_tuya_ac_instant_publish_timeout == 0) {
            g_tuya_ac_instant_publish_mask = 0;
        }
    }

    // Buffered publish: flush dirty parameters every TUYA_AC_PUBLISH_INTERVAL_SEC seconds
    if (mqtt_connected && g_tuya_ac_dirty_flags) {
        g_tuya_ac_publish_countdown--;
        if (g_tuya_ac_publish_countdown <= 0) {
            TuyaAC_FlushDirtyToMQTT();
        }
    }
}

void TuyaAC_RunFrame(void) {
    static uint8_t buffer[TUYA_AC_RX_BUFFER_SIZE];
    
    // Quick-tick logic for parsing incoming UART data
    while (UART_GetDataSize() > 0) {
        // Peek at start byte
        if (UART_GetByte(0) != 0xA5) {
            uint8_t garbage = UART_GetByte(0);
            ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "Garbage RX: %02X", garbage);
            UART_ConsumeBytes(1); // consume garbage
            continue;
        }

        int available = UART_GetDataSize();
        if (available < 12) {
            // Not enough bytes for a full A5 header yet
            break; 
        }

        uint8_t peek_frame_class = UART_GetByte(3);
        if (peek_frame_class != TUYA_AC_FRAME_REQUEST && peek_frame_class != TUYA_AC_FRAME_RESPONSE) {
            ADDLOG_WARN(LOG_FEATURE_TUYA_AC, "Discarding A5 frame with unsupported class 0x%02X", peek_frame_class);
            UART_ConsumeBytes(1);
            continue;
        }

        // We have at least 12 bytes, so we can read the length
        uint16_t len = (UART_GetByte(6) << 8) | UART_GetByte(7);

        // Sanity check length
        if (len < 12 || len > TUYA_AC_RX_BUFFER_SIZE) {
            // Corrupt or invalid length
            UART_ConsumeBytes(1);
            continue;
        }

        if (available < len) {
            // Wait for rest of packet
            break;
        }

        // Full packet is present, let's read it into buffer
        for (int i = 0; i < len; i++) {
            buffer[i] = UART_GetByte(0);
            UART_ConsumeBytes(1);
        }

        // Verify CRC
        uint8_t data_to_check[TUYA_AC_RX_BUFFER_SIZE];
        memcpy(data_to_check, buffer, 8);
        memcpy(&data_to_check[8], &buffer[10], len - 10);
        uint16_t calc_crc = calculate_crc16_xmodem(data_to_check, len - 2);
        uint16_t rx_crc = (buffer[8] << 8) | buffer[9];

        if (calc_crc != rx_crc) {
            ADDLOG_WARN(LOG_FEATURE_TUYA_AC, "Tuya AC CRC Error: expected %04X, got %04X", calc_crc, rx_crc);
            printHexToLog("RX UART BAD CRC:", buffer, len, rx_crc);
            continue;
        }

        printHexToLog("RX UART:", buffer, len, rx_crc);

        uint8_t b2 = buffer[2];
        uint8_t frame_class = buffer[3];
        uint8_t rx_seq = (frame_class == TUYA_AC_FRAME_RESPONSE) ? buffer[5] : buffer[4];
        uint8_t rx_cmd = (frame_class == TUYA_AC_FRAME_RESPONSE) ? buffer[11] : buffer[10];
        uint8_t rx_resp_flag = (frame_class == TUYA_AC_FRAME_RESPONSE) ? (buffer[10] & 0x7F) : 0;

        if (frame_class != TUYA_AC_FRAME_REQUEST && frame_class != TUYA_AC_FRAME_RESPONSE) {
            ADDLOG_WARN(LOG_FEATURE_TUYA_AC, "Tuya AC unsupported frame class 0x%02X", frame_class);
            continue;
        }

        if (frame_class == TUYA_AC_FRAME_RESPONSE && rx_resp_flag != TUYA_AC_RESP_OK) {
            ADDLOG_WARN(LOG_FEATURE_TUYA_AC, "Tuya AC response cmd 0x%02X returned flag 0x%02X", rx_cmd, rx_resp_flag);
        }

        if (frame_class == TUYA_AC_FRAME_REQUEST && TuyaAC_ShouldAckRequest(b2, rx_cmd)) {
            TuyaAC_SendAck(b2, rx_seq, rx_cmd, TUYA_AC_RESP_OK);
        }

        // Process compact AC/property records only on the proven b2=1 thing channel
        // and only for the GFW commands known to carry compact records.  Core b2=0
        // frames are gateway control, not DP/id_no records.
        int payload_len = len - 12;
        if (payload_len <= 0) {
            continue;
        }

        uint8_t *payload = &buffer[12];
        if (!TuyaAC_IsCompactRecordCommand(b2, rx_cmd)) {
            ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "RX non-property frame b2=0x%02X cmd=0x%02X payload_len=%d", b2, rx_cmd, payload_len);
            continue;
        }

        if (rx_cmd == 0x0B && TuyaAC_IsQueryAllPayload(payload, payload_len)) {
            ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "RX query-all request; not decoding FF FF as property records");
            continue;
        }

        {
            int idx = 0;
            int parseGuard = 0;
            while (idx + 2 <= payload_len) {
                if (parseGuard++ > (payload_len * 2)) {
                    ADDLOG_WARN(LOG_FEATURE_TUYA_AC, "Parser guard hit at idx %d/%d", idx, payload_len);
                    break;
                }

                uint16_t dp_id = (payload[idx] << 8) | payload[idx+1];
                idx += 2;
                
                int dp_len = TuyaAC_GetDpLength(dp_id, payload, idx, payload_len);
                dp_len = TuyaAC_AdjustDpLengthHeuristic(dp_id, payload, idx, payload_len, dp_len);
                
                if (dp_len < 0 || idx + dp_len > payload_len) {
                    ADDLOG_WARN(LOG_FEATURE_TUYA_AC, "Unknown/invalid DP %04X at idx %d, attempting resync", dp_id, idx);
                    // Resync by sliding one byte and trying again, so one bad/ambiguous
                    // field does not drop the entire remaining payload.
                    idx -= 1;
                    continue;
                }
                
                uint32_t val32 = 0;
                uint16_t val16 = 0;
                if (dp_len == 4) {
                    val32 = (payload[idx] << 24) | (payload[idx+1] << 16) | (payload[idx+2] << 8) | payload[idx+3];
                } else if (dp_len == 2) {
                    val16 = (payload[idx] << 8) | payload[idx+1];
                }
                
                if (dp_id == 0x0001) {
                    int new_power = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Power: %s", dp_id, new_power ? "On" : "Off");
                    if (g_tuya_ac.power != new_power || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.power = new_power;
                        g_tuya_ac_dirty_flags |= DIRTY_POWER | DIRTY_MODE;
                        if (!g_tuya_ac.power) {
                            g_tuya_ac.in_fan_rpm = 0;
                            g_tuya_ac.in_fan_percent = 0;
                            g_tuya_ac.out_fan_rpm = 0;
                            g_tuya_ac_dirty_flags |= DIRTY_IN_FAN_RPM | DIRTY_IN_FAN_PERCENT | DIRTY_OUT_FAN_RPM;
                        }
                    }
                } else if (dp_id == 0x0012) {
                    TuyaAC_Mode_e new_mode = (TuyaAC_Mode_e)payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] AC Mode: %s", dp_id, get_ac_mode_str(new_mode));
                    if (g_tuya_ac.mode != new_mode || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.mode = new_mode;
                        g_tuya_ac_dirty_flags |= DIRTY_MODE;
                    }
                } else if (dp_id == 0x0005) {
                    TuyaAC_FanMode_e new_fan = (TuyaAC_FanMode_e)((dp_len == 2) ? (val16 >> 8) : payload[idx]);
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Fan Speed Mode: %s", dp_id, get_ac_fan_str(new_fan));
                    if (g_tuya_ac.fan != new_fan || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.fan = new_fan;
                        g_tuya_ac_dirty_flags |= DIRTY_FAN;
                    }
                } else if (dp_id == 0x0002) {
                    float new_target = val32 / 100.0f;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Target Temp: %.1f C", dp_id, new_target);
                    if (g_tuya_ac.target_temp != new_target || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.target_temp = new_target;
                        g_tuya_ac_dirty_flags |= DIRTY_TARGET_TEMP;
                    }
                } else if (dp_id == 0x0003) {
                    float new_curr = val32 / 100.0f;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Current Temp: %.1f C", dp_id, new_curr);
                    if (g_tuya_ac.current_temp != new_curr || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.current_temp = new_curr;
                        g_tuya_ac_dirty_flags |= DIRTY_CURRENT_TEMP;
                    }
                } else if (dp_id == 0x000E) {
                    TuyaAC_HSwing_e new_h = (TuyaAC_HSwing_e)payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Horizontal Swing: %s", dp_id, get_ac_h_swing_str(new_h));
                    if (g_tuya_ac.h_swing != new_h || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.h_swing = new_h;
                        g_tuya_ac_dirty_flags |= DIRTY_H_SWING;
                    }
                } else if (dp_id == 0x0011) {
                    TuyaAC_VSwing_e new_v = (TuyaAC_VSwing_e)payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Vertical Swing: %s", dp_id, get_ac_v_swing_str(new_v));
                    if (g_tuya_ac.v_swing != new_v || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.v_swing = new_v;
                        g_tuya_ac_dirty_flags |= DIRTY_V_SWING;
                    }
                } else if (dp_id == 0x0015) {
                    int new_health = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Health/Ionizer Mode: %s", dp_id, new_health ? "On" : "Off");
                    if (g_tuya_ac.health != new_health || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.health = new_health;
                        g_tuya_ac_dirty_flags |= DIRTY_HEALTH;
                    }
                } else if (dp_id == 0x001E) {
                    int new_disp = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Display Light: %s", dp_id, new_disp ? "On" : "Off");
                    if (g_tuya_ac.display != new_disp || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.display = new_disp;
                        g_tuya_ac_dirty_flags |= DIRTY_DISPLAY;
                    }
                } else if (dp_id == 0x0022) {
                    TuyaAC_Sleep_e new_sleep = (TuyaAC_Sleep_e)payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Sleep Mode: %s", dp_id, get_ac_sleep_str(new_sleep));
                    if (g_tuya_ac.sleep != new_sleep || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.sleep = new_sleep;
                        g_tuya_ac_dirty_flags |= DIRTY_SLEEP;
                    }
                } else if (dp_id == 0x0025) {
                    int new_buzz = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Buzzer/Beep: %s", dp_id, new_buzz ? "On" : "Off");
                    if (g_tuya_ac.buzzer != new_buzz || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.buzzer = new_buzz;
                        g_tuya_ac_dirty_flags |= DIRTY_BUZZER;
                    }
                } else if (dp_id == 0x002D) {
                    TuyaAC_Generator_e new_gen = (TuyaAC_Generator_e)payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Generator Mode: %s", dp_id, get_ac_generator_str(new_gen));
                    if (g_tuya_ac.generator != new_gen || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.generator = new_gen;
                        g_tuya_ac_dirty_flags |= DIRTY_GENERATOR;
                    }
                } else if (dp_id == 0x0073) {
                    int new_mute = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Mute: %s", dp_id, new_mute ? "On" : "Off");
                    if (g_tuya_ac.mute != new_mute || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.mute = new_mute;
                        g_tuya_ac_dirty_flags |= DIRTY_MUTE;
                    }
                } else if (dp_id == 0x00DF) {
                    int new_eco = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Eco Mode: %s", dp_id, new_eco ? "On" : "Off");
                    if (g_tuya_ac.eco != new_eco || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.eco = new_eco;
                        g_tuya_ac_dirty_flags |= DIRTY_ECO;
                    }
                } else if (dp_id == 0x000C) {
                    int new_vmotor = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Vert Motor Status: %s", dp_id, new_vmotor ? "Moving" : "Stopped");
                    if (g_tuya_ac.v_motor != new_vmotor || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.v_motor = new_vmotor;
                        g_tuya_ac_dirty_flags |= DIRTY_V_MOTOR;
                    }
                } else if (dp_id == 0x000D) {
                    int new_hmotor = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Horiz Motor Status: %s", dp_id, new_hmotor ? "Moving" : "Stopped");
                    if (g_tuya_ac.h_motor != new_hmotor || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.h_motor = new_hmotor;
                        g_tuya_ac_dirty_flags |= DIRTY_H_MOTOR;
                    }
                } else if (dp_id == 0x003D) {
                    uint32_t new_nrg = val32;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Energy/Electricity: %u", dp_id, new_nrg);
                    if (g_tuya_ac.energy != new_nrg || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.energy = new_nrg;
                        g_tuya_ac_dirty_flags |= DIRTY_ENERGY;
                    }
                } else if (dp_id == 0x005C) {
                    uint32_t new_rpm = g_tuya_ac.power ? val32 : 0;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Indoor Fan Speed: %u RPM", dp_id, new_rpm);
                    if (g_tuya_ac.in_fan_rpm != new_rpm || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.in_fan_rpm = new_rpm;
                        g_tuya_ac_dirty_flags |= DIRTY_IN_FAN_RPM;
                    }
                } else if (dp_id == 0x0060) {
                    float new_out_t = val32 / 100.0f;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Outdoor Temp: %.1f C", dp_id, new_out_t);
                    if (g_tuya_ac.out_temp != new_out_t || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.out_temp = new_out_t;
                        g_tuya_ac_dirty_flags |= DIRTY_OUT_TEMP;
                    }
                } else if (dp_id == 0x0064) {
                    uint32_t new_out_rpm = g_tuya_ac.power ? val32 : 0;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Outdoor Fan Speed: %u RPM", dp_id, new_out_rpm);
                    if (g_tuya_ac.out_fan_rpm != new_out_rpm || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.out_fan_rpm = new_out_rpm;
                        g_tuya_ac_dirty_flags |= DIRTY_OUT_FAN_RPM;
                    }
                } else if (dp_id == 0x0065) {
                    uint32_t new_run = 100 - val32;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Compressor Frequency: %u mins", dp_id, new_run);
                    if (g_tuya_ac.runtime != new_run || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.runtime = new_run;
                        g_tuya_ac_dirty_flags |= DIRTY_RUNTIME;
                    }
                } else if (dp_id == 0x00A4) {
                    int new_filt = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Filter Health: %u%%", dp_id, new_filt);
                    if (g_tuya_ac.filter != new_filt || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.filter = new_filt;
                        g_tuya_ac_dirty_flags |= DIRTY_FILTER;
                    }
                } else if (dp_id == 0x00C0) {
                    uint32_t new_hz = g_tuya_ac.power ? val32 : 0;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Compressor: %u Hz", dp_id, new_hz);
                    if (g_tuya_ac.compressor_hz != new_hz || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.compressor_hz = new_hz;
                        g_tuya_ac_dirty_flags |= DIRTY_COMPRESSOR_HZ;
                    }
                } else if (dp_id == 0x0147) {
                    int new_eight = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Eight Degree Heat: %s", dp_id, new_eight ? "On" : "Off");
                    if (g_tuya_ac.eight_degree != new_eight || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.eight_degree = new_eight;
                        g_tuya_ac_dirty_flags |= DIRTY_EIGHT_DEGREE;
                    }
                } else if (dp_id == 0x0072) {
                    uint32_t new_pct = g_tuya_ac.power ? val32 : 0;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Indoor Fan Speed: %u%%", dp_id, new_pct);
                    if (g_tuya_ac.in_fan_percent != new_pct || g_tuya_ac.seq <= 2) {
                        g_tuya_ac.in_fan_percent = new_pct;
                        g_tuya_ac_dirty_flags |= DIRTY_IN_FAN_PERCENT;
                    }
                } else {
                    const char *name = "Unknown";
                    switch(dp_id) {
                        case 0x0220: name = "Access Card Insert"; break;
                        case 0x0224: name = "High Temperature Wind"; break;
                        case 0x0225: name = "Cool Feel Wind"; break;
                        case 0x007B: name = "Smart Windmode"; break;
                        case 0x0084: name = "Voice Switch"; break;
                        case 0x00A0: name = "Sound Location"; break;
                        case 0x00D1: name = "Voice Status"; break;
                        case 0x00D4: name = "Stores Mode"; break;
                        case 0x00D5: name = "Examine Mode"; break;
                        case 0x00D6: name = "Filter Blocknotify"; break;
                        case 0x00C9: name = "Regular Reporting"; break;
                        case 0x0074: name = "Internal or Reserved"; break;
                        case 0x0038: name = "Error Code Raw"; break;
                        case 0x008C: name = "Max Volume"; break;
                        case 0x0095: name = "Internal or Reserved"; break;
                        case 0x00BE: name = "Outdoor Fan Tarspeed"; break;
                        case 0x00BF: name = "Outdoor EEVTAR Opendegree"; break;
                        case 0x00D7: name = "Mic Distance"; break;
                        case 0x00FA: name = "Weektimer 1"; break;
                        case 0x00FB: name = "Weektimer 2"; break;
                        case 0x0223: name = "Specialtimer"; break;
                        case 0x0221: name = "Lower Temp Limit"; break;
                        case 0x0222: name = "Upper Temp Limit"; break;
                        case 0x0227: name = "Temp Set Fahrenheit"; break;
                    }

                    if (dp_id == 0x0221 || dp_id == 0x0222) {
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] %s: %.1f C", dp_id, name, val32 / 100.0f);
                    } else if (dp_len == 1) {
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] %s: %u", dp_id, name, payload[idx]);
                    } else if (dp_len == 4) {
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] %s: %u", dp_id, name, val32);
                    } else {
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] %s (%d bytes)", dp_id, name, dp_len);
                    }
                }
                
                uint32_t dp_dirty = TuyaAC_GetDirtyFlagForDP(dp_id);
                if (dp_dirty & g_tuya_ac_instant_publish_mask) {
                    g_tuya_ac_dirty_flags |= dp_dirty;
                }

                idx += dp_len;
            }

            uint32_t active_instant_flushed = g_tuya_ac_dirty_flags & (g_tuya_ac_instant_publish_mask | DIRTY_INSTANT_ALWAYS);
            if (active_instant_flushed) {
                TuyaAC_FlushDirtyToMQTT();
                g_tuya_ac_instant_publish_mask &= ~active_instant_flushed;
            }

            g_tuya_ac_have_rx_state = 1;
            if (!g_tuya_ac_have_published_state && Main_HasMQTTConnected()) {
                ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "Initial RX sync, scheduling AC state snapshot publish");
                TuyaAC_RequestSnapshotPublish();
                g_tuya_ac_have_published_state = 1;
            }
        }
    }
}

#else // ENABLE_DRIVER_TUYA_AC

void TuyaAC_Init(void) {}
void TuyaAC_RunEverySecond(void) {}
void TuyaAC_RunFrame(void) {}
void TuyaAC_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {}

#endif // ENABLE_DRIVER_TUYA_AC
