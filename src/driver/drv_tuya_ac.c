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

// AC State
tuya_ac_state_t g_tuya_ac = {
    .power = -1,
    .target_temp = -1.0f,
    .current_temp = -1.0f,
    .mode = -1,
    .fan = -1,
    .h_swing = -1,
    .v_swing = -1,
    .health = -1,
    .display = -1,
    .sleep = -1,
    .buzzer = -1,
    .generator = -1,
    .mute = -1,
    .eco = -1,
    .v_motor = -1,
    .h_motor = -1,
    .energy = 0x00000000,
    .in_fan_rpm = 0x00000000,
    .in_fan_percent = 0x00000000,
    .out_temp = -1.0f,
    .out_fan_rpm = 0x00000000,
    .runtime = 0x00000000,
    .filter = 0xFFFFFFFF,
    .compressor_hz = 0x00000000,
    .eight_degree = -1,
    .last_cmd_time = -10,
    .seq = 0
};

static const char *acModeOptions[] = { "auto", "cool", "dry", "fan_only", "heat" };
static const char *fanModeOptions[] = { "auto", "lowest", "low", "mid-low", "mid", "mid-high", "high", "turbo" };

static const char *get_ac_mode_str(int v) {
    if (v >= 0 && v < 5) return acModeOptions[v];
    return "Unknown";
}
static const char *get_ac_fan_str(int v) {
    if (v >= 0 && v < 8) return fanModeOptions[v];
    return "Unknown";
}
static const char *get_ac_h_swing_str(int v) {
    if (v == 0 || v == 8) return "Keep Position";
    if (v == 1) return "Auto L/R";
    if (v == 2) return "Flow Left";
    if (v == 3) return "Flow Middle";
    if (v == 4) return "Flow Right";
    if (v == 9) return "Fix Left";
    if (v == 10) return "Fix Mid-Left";
    if (v == 11) return "Fix Middle";
    if (v == 12) return "Fix Mid-Right";
    if (v == 13) return "Fix Right";
    return "Unknown";
}
static const char *get_ac_v_swing_str(int v) {
    if (v == 0 || v == 8) return "Keep Position";
    if (v == 1) return "Auto U/D";
    if (v == 2) return "Flow Up";
    if (v == 3) return "Flow Down";
    if (v == 9) return "Fix Above";
    if (v == 10) return "Fix Mid-High";
    if (v == 11) return "Fix Middle";
    if (v == 12) return "Fix Mid-Low";
    if (v == 13) return "Fix Down";
    return "Unknown";
}
static const char *get_ac_sleep_str(int v) {
    if (v == 0) return "Off";
    if (v == 1) return "Standard";
    if (v == 2) return "Aged";
    if (v == 3) return "Child";
    return "Unknown";
}
static const char *get_ac_generator_str(int v) {
    if (v == 0) return "Off";
    if (v == 1) return "Level 1";
    if (v == 2) return "Level 2";
    if (v == 3) return "Level 3";
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

static void printHexToLog(const char* prefix, uint8_t *data, int len) {
    char hexstr[256] = {0};
    int limit = len;
    if (limit > 80) limit = 80;
    for(int i = 0; i < limit; i++) {
        snprintf(hexstr + (i*3), sizeof(hexstr) - (i*3), "%02X ", data[i]);
    }
    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "%s %s", prefix, hexstr);
}

static void TuyaAC_SendPacket(uint8_t frame_class, uint16_t cmd_flag, uint8_t *payload, uint16_t payload_len) {
    uint16_t packet_len = 12 + payload_len; // 12 bytes header/crc + payload
    uint8_t buffer[256];
    if (packet_len > sizeof(buffer)) return;

    buffer[0] = 0xA5;
    buffer[1] = 0x01; // typically 01 01 or similar, maybe doesn't matter too much, but let's use 0x01
    buffer[2] = 0x01; // wait, typical header: A5 01 01 21...
    buffer[3] = frame_class;
    buffer[4] = g_tuya_ac.seq++; // seq
    buffer[5] = 0x00;
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

    // calculate CRC on everything except CRC bytes themselves
    // wait, CRC is calculated over data_to_check = b[:8] + b[10:]
    uint8_t data_to_check[256];
    memcpy(data_to_check, buffer, 8);
    memcpy(&data_to_check[8], &buffer[10], 2 + payload_len);
    
    uint16_t calc_crc = calculate_crc16_xmodem(data_to_check, 10 + payload_len);
    buffer[8] = (calc_crc >> 8) & 0xFF;
    buffer[9] = calc_crc & 0xFF;

    printHexToLog("TX:", buffer, packet_len);

    for (int i = 0; i < packet_len; i++) {
        UART_SendByte(buffer[i]);
    }
}

static void TuyaAC_SendDP(uint16_t dp_id, uint8_t *data, uint16_t len) {
    uint8_t payload[256];
    payload[0] = (dp_id >> 8) & 0xFF;
    payload[1] = dp_id & 0xFF;
    memcpy(&payload[2], data, len);
    
    TuyaAC_SendPacket(0x21, 0x0A0A, payload, 2 + len);
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

static void TuyaAC_SendDP_PowerOn() {
    uint8_t payload[6];
    payload[0] = 0x00; // DP 1
    payload[1] = 0x01;
    payload[2] = 0x01; // On
    payload[3] = 0x00; // DP 19
    payload[4] = 0x13;
    payload[5] = 0x00; // Value
    TuyaAC_SendPacket(0x21, 0x0A0A, payload, 6);
}

static void TuyaAC_SendDP_Enum(uint16_t dp_id, uint8_t val) {
    TuyaAC_SendDP(dp_id, &val, 1);
}

static int parseACMode(const char *s) {
    for (int i = 0; i < 5; i++) {
        if (!stricmp(s, acModeOptions[i])) return i;
    }
    return atoi(s);
}

static int parseFanMode(const char *s) {
    for (int i = 0; i < 8; i++) {
        if (!stricmp(s, fanModeOptions[i])) return i;
    }
    return atoi(s);
}

static int parseHSwingMode(const char *s) {
    if (!stricmp(s, "Keep Position")) return 8;
    if (!stricmp(s, "Auto L/R")) return 1;
    if (!stricmp(s, "Flow Left")) return 2;
    if (!stricmp(s, "Flow Middle")) return 3;
    if (!stricmp(s, "Flow Right")) return 4;
    if (!stricmp(s, "Fix Left")) return 9;
    if (!stricmp(s, "Fix Mid-Left")) return 10;
    if (!stricmp(s, "Fix Middle")) return 11;
    if (!stricmp(s, "Fix Mid-Right")) return 12;
    if (!stricmp(s, "Fix Right")) return 13;
    return atoi(s);
}

static int parseVSwingMode(const char *s) {
    if (!stricmp(s, "Keep Position")) return 8;
    if (!stricmp(s, "Auto U/D")) return 1;
    if (!stricmp(s, "Flow Up")) return 2;
    if (!stricmp(s, "Flow Down")) return 3;
    if (!stricmp(s, "Fix Above")) return 9;
    if (!stricmp(s, "Fix Mid-High")) return 10;
    if (!stricmp(s, "Fix Middle")) return 11;
    if (!stricmp(s, "Fix Mid-Low")) return 12;
    if (!stricmp(s, "Fix Down")) return 13;
    return atoi(s);
}

static int parseSleepMode(const char *s) {
    if (!stricmp(s, "Off")) return 0;
    if (!stricmp(s, "Standard")) return 1;
    if (!stricmp(s, "Aged")) return 2;
    if (!stricmp(s, "Child")) return 3;
    return atoi(s);
}

static int parseGeneratorMode(const char *s) {
    if (!stricmp(s, "Off")) return 0;
    if (!stricmp(s, "Level 1")) return 1;
    if (!stricmp(s, "Level 2")) return 2;
    if (!stricmp(s, "Level 3")) return 3;
    return atoi(s);
}

static commandResult_t CMD_TuyaAC_Power(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.power = Tokenizer_GetArgInteger(0);
    
    if (g_tuya_ac.power) {
        TuyaAC_SendDP_PowerOn();
    } else {
        TuyaAC_SendDP_Bool(0x01, 0);
    }
    
    MQTT_PublishMain_StringString("ac_power", g_tuya_ac.power ? "On" : "Off", 0);
    if (!g_tuya_ac.power) MQTT_PublishMain_StringString("ac_mode", "off", 0);
    else MQTT_PublishMain_StringString("ac_mode", get_ac_mode_str(g_tuya_ac.mode), 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Mode(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    const char *modeStr = Tokenizer_GetArg(0);
    
    if (!stricmp(modeStr, "off")) {
        g_tuya_ac.power = 0;
        TuyaAC_SendDP_Bool(0x01, 0);
        MQTT_PublishMain_StringString("ac_power", "Off", 0);
        MQTT_PublishMain_StringString("ac_mode", "off", 0);
    } else {
        g_tuya_ac.mode = parseACMode(modeStr);
        if (g_tuya_ac.power == 0) {
            g_tuya_ac.power = 1;
            // Bundle Power ON (DP 1), DP 19, and Mode (DP 12) into a single payload!
            uint8_t payload[9];
            payload[0] = 0x00; // DP 1
            payload[1] = 0x01;
            payload[2] = 0x01; // On
            payload[3] = 0x00; // DP 19
            payload[4] = 0x13;
            payload[5] = 0x00; // Value
            payload[6] = 0x00; // DP 12
            payload[7] = 0x12;
            payload[8] = g_tuya_ac.mode;
            TuyaAC_SendPacket(0x21, 0x0A0A, payload, 9);
            
            MQTT_PublishMain_StringString("ac_power", "On", 0);
        } else {
            TuyaAC_SendDP_Enum(0x12, g_tuya_ac.mode);
        }
        MQTT_PublishMain_StringString("ac_mode", get_ac_mode_str(g_tuya_ac.mode), 0);
    }
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Fan(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.fan = parseFanMode(Tokenizer_GetArg(0));
    TuyaAC_SendDP_Enum(0x05, g_tuya_ac.fan);
    MQTT_PublishMain_StringString("ac_fan", get_ac_fan_str(g_tuya_ac.fan), 0);
    if (g_tuya_ac.fan == 0) {
        TuyaAC_SendDP_Bool(0x0073, 1);
        g_tuya_ac.mute = 1;
        MQTT_PublishMain_StringString("ac_mute", "On", 0);
    } else {
        TuyaAC_SendDP_Bool(0x0073, 0);
        g_tuya_ac.mute = 0;
        MQTT_PublishMain_StringString("ac_mute", "Off", 0);
    }
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_TargetTemp(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.target_temp = Tokenizer_GetArgFloat(0);
    uint32_t val = (uint32_t)(g_tuya_ac.target_temp * 100.0f);
    TuyaAC_SendDP_Value(0x02, val);
    MQTT_PublishMain_StringFloat("ac_target_temp", g_tuya_ac.target_temp, 1, 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_EightDegree(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.eight_degree = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x0147, g_tuya_ac.eight_degree ? 1 : 0);
    MQTT_PublishMain_StringString("ac_eight_degree", g_tuya_ac.eight_degree ? "On" : "Off", 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_InFanPercent(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.in_fan_percent = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Value(0x0072, g_tuya_ac.in_fan_percent);
    MQTT_PublishMain_StringInt("ac_in_fan_percent", g_tuya_ac.in_fan_percent, 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_HSwing(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.h_swing = parseHSwingMode(Tokenizer_GetArg(0));
    TuyaAC_SendDP_Enum(0x0E, g_tuya_ac.h_swing);
    MQTT_PublishMain_StringString("ac_h_swing", get_ac_h_swing_str(g_tuya_ac.h_swing), 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_VSwing(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.v_swing = parseVSwingMode(Tokenizer_GetArg(0));
    TuyaAC_SendDP_Enum(0x11, g_tuya_ac.v_swing);
    MQTT_PublishMain_StringString("ac_v_swing", get_ac_v_swing_str(g_tuya_ac.v_swing), 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Sleep(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.sleep = parseSleepMode(Tokenizer_GetArg(0));
    TuyaAC_SendDP_Enum(0x22, g_tuya_ac.sleep);
    MQTT_PublishMain_StringString("ac_sleep", get_ac_sleep_str(g_tuya_ac.sleep), 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Generator(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.generator = parseGeneratorMode(Tokenizer_GetArg(0));
    TuyaAC_SendDP_Enum(0x2D, g_tuya_ac.generator);
    MQTT_PublishMain_StringString("ac_generator", get_ac_generator_str(g_tuya_ac.generator), 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Eco(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.eco = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0xDF, g_tuya_ac.eco ? 1 : 0);
    MQTT_PublishMain_StringString("ac_eco", g_tuya_ac.eco ? "On" : "Off", 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Mute(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.mute = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x73, g_tuya_ac.mute ? 1 : 0);
    MQTT_PublishMain_StringString("ac_mute", g_tuya_ac.mute ? "On" : "Off", 0);
    if (g_tuya_ac.mute) {
        g_tuya_ac.fan = 0; // Auto
        TuyaAC_SendDP_Enum(0x05, 0);
        MQTT_PublishMain_StringString("ac_fan", get_ac_fan_str(g_tuya_ac.fan), 0);
    }
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Buzzer(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.buzzer = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x25, g_tuya_ac.buzzer ? 1 : 0);
    MQTT_PublishMain_StringString("ac_buzzer", g_tuya_ac.buzzer ? "On" : "Off", 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Display(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.display = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x1E, g_tuya_ac.display ? 1 : 0);
    MQTT_PublishMain_StringString("ac_display", g_tuya_ac.display ? "On" : "Off", 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Health(const void* context, const char* cmd, const char* args, int cmdFlags) {
    extern int g_secondsElapsed;
    g_tuya_ac.last_cmd_time = g_secondsElapsed;
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac.health = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x15, g_tuya_ac.health ? 1 : 0);
    MQTT_PublishMain_StringString("ac_health", g_tuya_ac.health ? "On" : "Off", 0);
    return CMD_RES_OK;
}

void TuyaAC_Init(void) {
    UART_InitReceiveRingBuffer(TUYA_AC_RX_BUFFER_SIZE);
    UART_InitUART(TUYA_AC_BAUDRATE, 0, false); // Initialize UART without TX queue/buffers perhaps? port 0? parity 0!

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

    // Send initialization sequence just like the original dongle
    uint8_t q_init[1] = {0x01};
    TuyaAC_SendPacket(0x21, 0x0000, q_init, 1);

    uint8_t q_0301[2] = {0x03, 0x01};
    TuyaAC_SendPacket(0x21, 0x0B0B, q_0301, 2);

    uint8_t q_0305[2] = {0x03, 0x05};
    TuyaAC_SendPacket(0x21, 0x0B0B, q_0305, 2);

    uint8_t query_payload[2] = {0xFF, 0xFF};
    TuyaAC_SendPacket(0x21, 0x0B0B, query_payload, 2);
}

void TuyaAC_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
    if (!bPreState) {
        hprintf255(request, "<h3>Tuya AC Driver</h3>");
        hprintf255(request, "Power: %s<br>", g_tuya_ac.power ? "ON" : "OFF");
        hprintf255(request, "Mode: %s<br>", get_ac_mode_str(g_tuya_ac.mode));
        hprintf255(request, "Fan: %s<br>", get_ac_fan_str(g_tuya_ac.fan));
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
            printHexToLog("RX_BAD_CRC:", buffer, len);
            continue;
        }

        printHexToLog("RX:", buffer, len);

        extern int g_secondsElapsed;
        if (g_secondsElapsed - g_tuya_ac.last_cmd_time < 3) {
            // Ignore Tuya MCU echo for 3 seconds after an MQTT command
            continue;
        }

        // Process Payload
        int payload_len = len - 12;
        if (payload_len > 0) {
            uint8_t *payload = &buffer[12];
            int idx = 0;
            while (idx + 2 <= payload_len) {
                uint16_t dp_id = (payload[idx] << 8) | payload[idx+1];
                idx += 2;
                
                int dp_len = -1;
                // Determine length based on dp_id
                switch(dp_id) {
                    case 0x0001: case 0x0005: case 0x000C: case 0x000D: case 0x000E:
                    case 0x0011: case 0x0012: case 0x0015: case 0x0017: case 0x001E:
                    case 0x0020: case 0x0021: case 0x0022: case 0x0023: case 0x0024: 
                    case 0x0025: case 0x0027: case 0x002D: case 0x0035: case 0x0038: 
                    case 0x0040: case 0x0042: case 0x0046: case 0x0047: case 0x0048:
                    case 0x005E: case 0x0073: case 0x0074: case 0x007B: case 0x0084: 
                    case 0x00A0: case 0x00A4: case 0x00C9: case 0x00D1: case 0x00D4: 
                    case 0x00D5: case 0x00D6: case 0x00DF: 
                    case 0x0147: case 0x0148: case 0x0220: case 0x0224: case 0x0225:
                        dp_len = 1; break;
                    
                    case 0x0002: case 0x0003: case 0x003D: case 0x005C: case 0x0060:
                    case 0x0064: case 0x0065: case 0x0072: case 0x008C: case 0x0095:
                    case 0x00BD: case 0x00BE: case 0x00BF: case 0x00C0: case 0x00D7:
                    case 0x0221: case 0x0222: case 0x0227:
                        dp_len = 4; break;

                    case 0x00FA: case 0x00FB: case 0x0223: // String with 2 byte len
                        if (idx + 2 <= payload_len) {
                            dp_len = 2 + ((payload[idx] << 8) | payload[idx+1]);
                        } else dp_len = -1;
                        break;

                    case 0x0039: // Raw with 1 byte len
                        if (idx + 1 <= payload_len) {
                            dp_len = 1 + payload[idx];
                        } else dp_len = -1;
                        break;
                }
                
                if (dp_len < 0 || idx + dp_len > payload_len) {
                    ADDLOG_WARN(LOG_FEATURE_TUYA_AC, "Unknown/invalid DP %04X, stopping parse at idx %d", dp_id, idx);
                    break; // Abort remaining parse
                }
                
                uint32_t val32 = 0;
                if (dp_len == 4) {
                    val32 = (payload[idx] << 24) | (payload[idx+1] << 16) | (payload[idx+2] << 8) | payload[idx+3];
                }
                
                if (dp_id == 0x0001) {
                    g_tuya_ac.power = payload[idx];
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Power: %s", dp_id, g_tuya_ac.power ? "On" : "Off");
                    MQTT_PublishMain_StringString("ac_power", g_tuya_ac.power ? "On" : "Off", 0);
                    if (!g_tuya_ac.power) {
                        MQTT_PublishMain_StringString("ac_mode", "off", 0);
                        g_tuya_ac.in_fan_rpm = 0;
                        g_tuya_ac.in_fan_percent = 0;
                        g_tuya_ac.out_fan_rpm = 0;
                        MQTT_PublishMain_StringInt("ac_in_fan_rpm", 0, 0);
                        MQTT_PublishMain_StringInt("ac_in_fan_percent", 0, 0);
                        MQTT_PublishMain_StringInt("ac_out_fan_rpm", 0, 0);
                    }
                } else if (dp_id == 0x0012) {
                    if (g_tuya_ac.mode != payload[idx]) {
                        g_tuya_ac.mode = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] AC Mode: %s", dp_id, get_ac_mode_str(g_tuya_ac.mode));
                        if (g_tuya_ac.power > 0) {
                            MQTT_PublishMain_StringString("ac_mode", get_ac_mode_str(g_tuya_ac.mode), 0);
                        }
                    }
                } else if (dp_id == 0x0005) {
                    if (g_tuya_ac.fan != payload[idx]) {
                        g_tuya_ac.fan = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Fan Speed Mode: %s", dp_id, get_ac_fan_str(g_tuya_ac.fan));
                        MQTT_PublishMain_StringString("ac_fan", get_ac_fan_str(g_tuya_ac.fan), 0);
                    }
                } else if (dp_id == 0x0002) {
                    g_tuya_ac.target_temp = val32 / 100.0f;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Target Temp: %.1f C", dp_id, g_tuya_ac.target_temp);
                    MQTT_PublishMain_StringFloat("ac_target_temp", g_tuya_ac.target_temp, 1, 0);
                } else if (dp_id == 0x0003) {
                    g_tuya_ac.current_temp = val32 / 100.0f;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Current Temp: %.1f C", dp_id, g_tuya_ac.current_temp);
                    MQTT_PublishMain_StringFloat("ac_current_temp", g_tuya_ac.current_temp, 1, 0);
                } else if (dp_id == 0x000E) {
                    if (g_tuya_ac.h_swing != payload[idx]) {
                        g_tuya_ac.h_swing = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Horizontal Swing: %s", dp_id, get_ac_h_swing_str(g_tuya_ac.h_swing));
                        MQTT_PublishMain_StringString("ac_h_swing", get_ac_h_swing_str(g_tuya_ac.h_swing), 0);
                    }
                } else if (dp_id == 0x0011) {
                    if (g_tuya_ac.v_swing != payload[idx]) {
                        g_tuya_ac.v_swing = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Vertical Swing: %s", dp_id, get_ac_v_swing_str(g_tuya_ac.v_swing));
                        MQTT_PublishMain_StringString("ac_v_swing", get_ac_v_swing_str(g_tuya_ac.v_swing), 0);
                    }
                } else if (dp_id == 0x0015) {
                    if (g_tuya_ac.health != payload[idx]) {
                        g_tuya_ac.health = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Health/Ionizer Mode: %s", dp_id, g_tuya_ac.health ? "On" : "Off");
                        MQTT_PublishMain_StringString("ac_health", g_tuya_ac.health ? "On" : "Off", 0);
                    }
                } else if (dp_id == 0x001E) {
                    if (g_tuya_ac.display != payload[idx]) {
                        g_tuya_ac.display = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Display Light: %s", dp_id, g_tuya_ac.display ? "On" : "Off");
                        MQTT_PublishMain_StringString("ac_display", g_tuya_ac.display ? "On" : "Off", 0);
                    }
                } else if (dp_id == 0x0022) {
                    if (g_tuya_ac.sleep != payload[idx]) {
                        g_tuya_ac.sleep = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Sleep Mode: %s", dp_id, get_ac_sleep_str(g_tuya_ac.sleep));
                        MQTT_PublishMain_StringString("ac_sleep", get_ac_sleep_str(g_tuya_ac.sleep), 0);
                    }
                } else if (dp_id == 0x0025) {
                    if (g_tuya_ac.buzzer != payload[idx]) {
                        g_tuya_ac.buzzer = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Buzzer/Beep: %s", dp_id, g_tuya_ac.buzzer ? "On" : "Off");
                        MQTT_PublishMain_StringString("ac_buzzer", g_tuya_ac.buzzer ? "On" : "Off", 0);
                    }
                } else if (dp_id == 0x002D) {
                    if (g_tuya_ac.generator != payload[idx]) {
                        g_tuya_ac.generator = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Generator Mode: %s", dp_id, get_ac_generator_str(g_tuya_ac.generator));
                        MQTT_PublishMain_StringString("ac_generator", get_ac_generator_str(g_tuya_ac.generator), 0);
                    }
                } else if (dp_id == 0x0073) {
                    if (g_tuya_ac.mute != payload[idx]) {
                        g_tuya_ac.mute = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Mute: %s", dp_id, g_tuya_ac.mute ? "On" : "Off");
                        MQTT_PublishMain_StringString("ac_mute", g_tuya_ac.mute ? "On" : "Off", 0);
                    }
                } else if (dp_id == 0x00DF) {
                    if (g_tuya_ac.eco != payload[idx]) {
                        g_tuya_ac.eco = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Eco Mode: %s", dp_id, g_tuya_ac.eco ? "On" : "Off");
                        MQTT_PublishMain_StringString("ac_eco", g_tuya_ac.eco ? "On" : "Off", 0);
                    }
                } else if (dp_id == 0x000C) {
                    if (g_tuya_ac.v_motor != payload[idx]) {
                        g_tuya_ac.v_motor = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Vert Motor Status: %s", dp_id, g_tuya_ac.v_motor ? "Moving" : "Stopped");
                        MQTT_PublishMain_StringString("ac_v_motor", g_tuya_ac.v_motor ? "Moving" : "Stopped", 0);
                    }
                } else if (dp_id == 0x000D) {
                    if (g_tuya_ac.h_motor != payload[idx]) {
                        g_tuya_ac.h_motor = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Horiz Motor Status: %s", dp_id, g_tuya_ac.h_motor ? "Moving" : "Stopped");
                        MQTT_PublishMain_StringString("ac_h_motor", g_tuya_ac.h_motor ? "Moving" : "Stopped", 0);
                    }
                } else if (dp_id == 0x003D) {
                    if (g_tuya_ac.energy != val32) {
                        g_tuya_ac.energy = val32;
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Energy/Electricity: %u", dp_id, g_tuya_ac.energy);
                        MQTT_PublishMain_StringInt("ac_energy", g_tuya_ac.energy, 0);
                    }
                } else if (dp_id == 0x005C) {
                    g_tuya_ac.in_fan_rpm = g_tuya_ac.power ? val32 : 0;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Indoor Fan Speed: %u RPM", dp_id, g_tuya_ac.in_fan_rpm);
                    MQTT_PublishMain_StringInt("ac_in_fan_rpm", g_tuya_ac.in_fan_rpm, 0);
                } else if (dp_id == 0x0060) {
                    g_tuya_ac.out_temp = val32 / 100.0f;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Outdoor Temp: %.1f C", dp_id, g_tuya_ac.out_temp);
                    MQTT_PublishMain_StringFloat("ac_out_temp", g_tuya_ac.out_temp, 1, 0);
                } else if (dp_id == 0x0064) {
                    g_tuya_ac.out_fan_rpm = g_tuya_ac.power ? val32 : 0;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Outdoor Fan Speed: %u RPM", dp_id, g_tuya_ac.out_fan_rpm);
                    MQTT_PublishMain_StringInt("ac_out_fan_rpm", g_tuya_ac.out_fan_rpm, 0);
                } else if (dp_id == 0x0065) {
                    if (g_tuya_ac.runtime != val32) {
                        g_tuya_ac.runtime = val32;
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Runtime Counter: %u mins", dp_id, g_tuya_ac.runtime);
                        MQTT_PublishMain_StringInt("ac_runtime", g_tuya_ac.runtime, 0);
                    }
                } else if (dp_id == 0x00A4) {
                    if (g_tuya_ac.filter != payload[idx]) {
                        g_tuya_ac.filter = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Filter Health: %u%%", dp_id, g_tuya_ac.filter);
                        MQTT_PublishMain_StringInt("ac_filter", g_tuya_ac.filter, 0);
                    }
                } else if (dp_id == 0x00C0) {
                    g_tuya_ac.compressor_hz = g_tuya_ac.power ? val32 : 0;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Compressor: %u Hz", dp_id, g_tuya_ac.compressor_hz);
                    MQTT_PublishMain_StringInt("ac_compressor_hz", g_tuya_ac.compressor_hz, 0);
                } else if (dp_id == 0x0147) {
                    if (g_tuya_ac.eight_degree != payload[idx]) {
                        g_tuya_ac.eight_degree = payload[idx];
                        ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Eight Degree Heat: %s", dp_id, g_tuya_ac.eight_degree ? "On" : "Off");
                        MQTT_PublishMain_StringString("ac_eight_degree", g_tuya_ac.eight_degree ? "On" : "Off", 0);
                    }
                } else if (dp_id == 0x0072) {
                    g_tuya_ac.in_fan_percent = g_tuya_ac.power ? val32 : 0;
                    ADDLOG_INFO(LOG_FEATURE_TUYA_AC, "  - [0x%04X] Indoor Fan Speed: %u%%", dp_id, g_tuya_ac.in_fan_percent);
                    MQTT_PublishMain_StringInt("ac_in_fan_percent", g_tuya_ac.in_fan_percent, 0);
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
                
                idx += dp_len;
            }
        }
    }
}

#else // ENABLE_DRIVER_TUYA_AC

void TuyaAC_Init(void) {}
void TuyaAC_RunEverySecond(void) {}
void TuyaAC_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {}

#endif // ENABLE_DRIVER_TUYA_AC
