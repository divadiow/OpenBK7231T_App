#include "../obk_config.h"

#if ENABLE_DRIVER_TUYA_AC

#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../httpserver/new_http.h"
#include "drv_uart.h"
#include "drv_tuya_ac.h"

#define TUYA_AC_BAUDRATE 9600
#define TUYA_AC_RX_BUFFER_SIZE 256

// AC State
static int g_tuya_ac_power = 0;
static float g_tuya_ac_target_temp = 24.0f;
static float g_tuya_ac_current_temp = 0.0f;
static int g_tuya_ac_mode = 0; // 0: Auto, 1: Cool, 2: Dry, 3: Fan, 4: Heat
static int g_tuya_ac_fan = 0; // 0: Auto, 1: Lowest, 2: Low, 3: Mid-Low, 4: Mid, 5: Mid-High, 6: High, 7: Turbo

static uint8_t g_tuya_ac_seq = 0;

static const char *acModeOptions[] = { "auto", "cool", "dry", "fan", "heat" };
static const char *fanModeOptions[] = { "auto", "lowest", "low", "mid-low", "mid", "mid-high", "high", "turbo" };

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

static void TuyaAC_SendPacket(uint8_t frame_class, uint8_t *payload, uint16_t payload_len) {
    uint16_t packet_len = 12 + payload_len; // 12 bytes header/crc + payload
    uint8_t buffer[256];
    if (packet_len > sizeof(buffer)) return;

    buffer[0] = 0xA5;
    buffer[1] = 0x01; // typically 01 01 or similar, maybe doesn't matter too much, but let's use 0x01
    buffer[2] = 0x01; // wait, typical header: A5 01 01 21...
    buffer[3] = frame_class;
    buffer[4] = g_tuya_ac_seq++; // seq
    buffer[5] = 0x00;
    buffer[6] = (packet_len >> 8) & 0xFF;
    buffer[7] = packet_len & 0xFF;

    // placeholder for CRC
    buffer[8] = 0x00;
    buffer[9] = 0x00;

    buffer[10] = 0x0A; // Cmd flag? typically 0x0A 0x0A for dongle to AC commands
    buffer[11] = 0x0A;

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

    for (int i = 0; i < packet_len; i++) {
        UART_SendByte(buffer[i]);
    }
}

static void TuyaAC_SendDP(uint16_t dp_id, uint8_t *data, uint16_t len) {
    uint8_t payload[256];
    payload[0] = (dp_id >> 8) & 0xFF;
    payload[1] = dp_id & 0xFF;
    memcpy(&payload[2], data, len);
    
    TuyaAC_SendPacket(0x21, payload, 2 + len);
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

static commandResult_t CMD_TuyaAC_Power(const void* context, const char* cmd, const char* args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac_power = Tokenizer_GetArgInteger(0);
    TuyaAC_SendDP_Bool(0x01, g_tuya_ac_power ? 1 : 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Mode(const void* context, const char* cmd, const char* args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac_mode = parseACMode(Tokenizer_GetArg(0));
    TuyaAC_SendDP_Enum(0x12, g_tuya_ac_mode);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_Fan(const void* context, const char* cmd, const char* args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac_fan = parseFanMode(Tokenizer_GetArg(0));
    TuyaAC_SendDP_Enum(0x05, g_tuya_ac_fan);
    return CMD_RES_OK;
}

static commandResult_t CMD_TuyaAC_TargetTemp(const void* context, const char* cmd, const char* args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    g_tuya_ac_target_temp = Tokenizer_GetArgFloat(0);
    uint32_t val = (uint32_t)(g_tuya_ac_target_temp * 100.0f);
    TuyaAC_SendDP_Value(0x02, val);
    return CMD_RES_OK;
}

void TuyaAC_Init(void) {
    UART_InitUART(TUYA_AC_BAUDRATE, 2, false); // Initialize UART without TX queue/buffers perhaps? port 2? Beken uses port 2 (TX2/RX2) usually for TuyaMCU.
    UART_InitReceiveRingBuffer(TUYA_AC_RX_BUFFER_SIZE);

    CMD_RegisterCommand("ACMode", CMD_TuyaAC_Mode, NULL);
    CMD_RegisterCommand("FANMode", CMD_TuyaAC_Fan, NULL);
    CMD_RegisterCommand("TargetTemperature", CMD_TuyaAC_TargetTemp, NULL);
    CMD_RegisterCommand("ACPower", CMD_TuyaAC_Power, NULL);
}

void TuyaAC_AppendInformationToHTTPIndexPage(http_request_t *request) {
    hprintf255(request, "<h3>Tuya AC Driver</h3>");
    hprintf255(request, "<b>Power:</b> %s<br>", g_tuya_ac_power ? "ON" : "OFF");
    hprintf255(request, "<b>Mode:</b> %s<br>", acModeOptions[g_tuya_ac_mode % 5]);
    hprintf255(request, "<b>Fan:</b> %s<br>", fanModeOptions[g_tuya_ac_fan % 8]);
    hprintf255(request, "<b>Target Temp:</b> %.1f &deg;C<br>", g_tuya_ac_target_temp);
    hprintf255(request, "<b>Current Temp:</b> %.1f &deg;C<br>", g_tuya_ac_current_temp);
}

void TuyaAC_RunEverySecond(void) {
    static uint8_t buffer[TUYA_AC_RX_BUFFER_SIZE];
    
    // Quick-tick logic for parsing incoming UART data
    while (UART_GetDataSize() > 0) {
        // Peek at start byte
        if (UART_GetByte(0) != 0xA5) {
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
            ADDLOG_WARN(LOG_FEATURE_TUYAMCU, "Tuya AC CRC Error: expected %04X, got %04X", calc_crc, rx_crc);
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
                
                int dp_len = 0;
                
                if (dp_id == 0x0001 || dp_id == 0x0012 || dp_id == 0x0005) {
                    dp_len = 1;
                } else if (dp_id == 0x0002 || dp_id == 0x0003 || dp_id == 0x0018) {
                    dp_len = 4;
                } else if (dp_id == 0x0095 || dp_id == 0x0072) {
                    dp_len = 4;
                } else if (dp_id == 0x0074) {
                    dp_len = 1;
                } else {
                    // Unknown DP, breaking parsing since we don't know lengths for sure
                    break;
                }

                if (idx + dp_len > payload_len) break;
                
                if (dp_id == 0x0001) {
                    g_tuya_ac_power = payload[idx];
                } else if (dp_id == 0x0012) {
                    g_tuya_ac_mode = payload[idx];
                } else if (dp_id == 0x0005) {
                    g_tuya_ac_fan = payload[idx];
                } else if (dp_id == 0x0002) {
                    uint32_t val = (payload[idx] << 24) | (payload[idx+1] << 16) | (payload[idx+2] << 8) | payload[idx+3];
                    g_tuya_ac_target_temp = val / 100.0f;
                } else if (dp_id == 0x0003) {
                    uint32_t val = (payload[idx] << 24) | (payload[idx+1] << 16) | (payload[idx+2] << 8) | payload[idx+3];
                    g_tuya_ac_current_temp = val / 100.0f;
                }
                
                idx += dp_len;
            }
        }
    }
}

#endif // ENABLE_DRIVER_TUYA_AC
