#include "drv_bl0939.h"
#include "../obk_config.h"

#if ENABLE_DRIVER_BL0939SPI

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../cmnds/cmd_public.h"
#include "../hal/hal_flashVars.h"
#include "../hal/hal_ota.h"
#include "../httpserver/hass.h"
#include "../logging/logging.h"
#include "../mqtt/new_mqtt.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "drv_public.h"
#include "drv_spi.h"

#define BL0939_SPI_BAUD_RATE              800000
#define BL0939_SPI_CMD_READ               0x55
#define BL0939_SPI_CMD_WRITE              0xA5

#define BL0939_REG_IA_FAST_RMS            0x00
#define BL0939_REG_IA_RMS                 0x04
#define BL0939_REG_IB_RMS                 0x05
#define BL0939_REG_V_RMS                  0x06
#define BL0939_REG_IB_FAST_RMS            0x07
#define BL0939_REG_A_WATT                 0x08
#define BL0939_REG_B_WATT                 0x09
#define BL0939_REG_CFA_CNT                0x0A
#define BL0939_REG_CFB_CNT                0x0B
#define BL0939_REG_A_CORNER               0x0C
#define BL0939_REG_B_CORNER               0x0D
#define BL0939_REG_TPS1                   0x0E
#define BL0939_REG_TPS2                   0x0F
#define BL0939_REG_MODE                   0x18
#define BL0939_REG_SOFT_RESET             0x19
#define BL0939_REG_USR_WRPROT             0x1A
#define BL0939_USR_WRPROT_DISABLE         0x55

#define BL0939_DEFAULT_VOLTAGE_CAL        15188.0f
#define BL0939_DEFAULT_CURRENT_A_CAL      251210.0f
#define BL0939_DEFAULT_CURRENT_B_CAL      251210.0f
#define BL0939_DEFAULT_POWER_A_CAL        598.0f
#define BL0939_DEFAULT_POWER_B_CAL        598.0f

#define BL0939_SAVE_COUNTER               3600
#define BL0939_INVALID_CF_CNT             0xFFFFFFFFu

#define BL0939_CHANNEL_VOLTAGE            0
#define BL0939_CHANNEL_CURRENT_A          1
#define BL0939_CHANNEL_POWER_A            2
#define BL0939_CHANNEL_IMPORT_A           3
#define BL0939_CHANNEL_EXPORT_A           4
#define BL0939_CHANNEL_CURRENT_B          5
#define BL0939_CHANNEL_POWER_B            6
#define BL0939_CHANNEL_IMPORT_B           7
#define BL0939_CHANNEL_EXPORT_B           8
#define BL0939_CHANNEL_IMPORT_TOTAL       9
#define BL0939_CHANNEL_EXPORT_TOTAL       10
#define BL0939_CHANNEL_POWER_FACTOR_A     11
#define BL0939_CHANNEL_POWER_FACTOR_B     12

#define BL0939_CHANSET_FLAGS              (CHANNEL_SET_FLAG_SILENT)
#define BL0939_CHANSET_FLAGS_FORCE        (CHANNEL_SET_FLAG_SILENT | CHANNEL_SET_FLAG_FORCE)

typedef struct {
    uint32_t ia_rms;
    uint32_t ib_rms;
    uint32_t v_rms;
    int32_t a_watt;
    int32_t b_watt;
    uint32_t cfa_cnt;
    uint32_t cfb_cnt;
    uint32_t a_corner;
    uint32_t b_corner;
} BL0939_RawData_t;

typedef struct {
    float voltage;
    float current_a;
    float current_b;
    float power_a;
    float power_b;
    float apparent_a;
    float apparent_b;
    float pf_a;
    float pf_b;
} BL0939_UpdateData_t;

typedef enum {
    BL0939_CHANNEL_A = 0,
    BL0939_CHANNEL_B = 1,
} BL0939_PhaseChannel_t;

static BL0939_RawData_t last_raw;
static BL0939_UpdateData_t last_update;
static ENERGY_DATA energy_acc_a = { .Import = 0, .Export = 0 };
static ENERGY_DATA energy_acc_b = { .Import = 0, .Export = 0 };
static portTickType last_energy_tick;
static int save_count_down = BL0939_SAVE_COUNTER;
static uint32_t checksum_errors;
static uint32_t read_errors;
static uint32_t prev_cfa_cnt = BL0939_INVALID_CF_CNT;
static uint32_t prev_cfb_cnt = BL0939_INVALID_CF_CNT;

static float cal_voltage;
static float cal_current_a;
static float cal_current_b;
static float cal_power_a;
static float cal_power_b;

static int32_t BL0939_Int24ToInt32(uint32_t val) {
    val &= 0x00FFFFFF;
    if (val & 0x00800000) {
        val |= 0xFF000000;
    }
    return (int32_t)val;
}

static int32_t BL0939_CalcSignedCounterDelta(uint32_t now, uint32_t previous) {
    int32_t diff = (int32_t)((now & 0x00FFFFFF) - (previous & 0x00FFFFFF));
    if (diff > 0x007FFFFF) {
        diff -= 0x01000000;
    } else if (diff < -0x00800000) {
        diff += 0x01000000;
    }
    return diff;
}

static float BL0939_SafeDivide(float raw, float cal) {
    if (cal > -0.000001f && cal < 0.000001f) {
        return 0.0f;
    }
    return raw / cal;
}

static void BL0939_LoadCalibration(void) {
    cal_voltage = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_VOLTAGE, BL0939_DEFAULT_VOLTAGE_CAL);
    cal_current_a = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_CURRENT, BL0939_DEFAULT_CURRENT_A_CAL);
    cal_power_a = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_POWER, BL0939_DEFAULT_POWER_A_CAL);
    cal_current_b = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_RES_KIA, BL0939_DEFAULT_CURRENT_B_CAL);
    cal_power_b = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_RES_KIB, BL0939_DEFAULT_POWER_B_CAL);
}

static void BL0939_SaveStats(int force) {
    if (OTA_GetProgress() != -1) {
        ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "BL0939: OTA in progress, skipping energy save");
        return;
    }

    if (!force) {
        save_count_down--;
        if (save_count_down > 0) {
            return;
        }
    }

    save_count_down = BL0939_SAVE_COUNTER;
    ENERGY_DATA *data[2] = { &energy_acc_a, &energy_acc_b };
    HAL_FlashVars_SaveEnergy(data, 2);
}

static void BL0939_RestoreStats(void) {
    energy_acc_a.Import = 0;
    energy_acc_a.Export = 0;
    energy_acc_b.Import = 0;
    energy_acc_b.Export = 0;
    HAL_FlashVars_GetEnergy(&energy_acc_a, ENERGY_CHANNEL_A);
    HAL_FlashVars_GetEnergy(&energy_acc_b, ENERGY_CHANNEL_B);
}

static int BL0939_SPI_ReadReg(uint8_t reg, uint32_t *val) {
    uint8_t send[2];
    uint8_t recv[4];

    send[0] = BL0939_SPI_CMD_READ;
    send[1] = reg;
    memset(recv, 0, sizeof(recv));

    int result = SPI_Transmit(send, sizeof(send), recv, sizeof(recv));
    if (result < 0) {
        read_errors++;
        ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "BL0939: SPI read reg %02X failed result=%d", reg, result);
        return result;
    }

    uint8_t checksum = send[0] + send[1] + recv[0] + recv[1] + recv[2];
    checksum ^= 0xFF;
    if (recv[3] != checksum) {
        checksum_errors++;
        ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                    "BL0939: bad checksum reg=%02X got=%02X wanted=%02X bytes=%02X %02X %02X",
                    reg, recv[3], checksum, recv[0], recv[1], recv[2]);
        return -1;
    }

    *val = ((uint32_t)recv[0] << 16) | ((uint32_t)recv[1] << 8) | recv[2];
    return 0;
}

static int BL0939_SPI_WriteReg(uint8_t reg, uint32_t val) {
    uint8_t send[6];

    send[0] = BL0939_SPI_CMD_WRITE;
    send[1] = reg;
    send[2] = (val >> 16) & 0xFF;
    send[3] = (val >> 8) & 0xFF;
    send[4] = val & 0xFF;
    send[5] = (send[0] + send[1] + send[2] + send[3] + send[4]) ^ 0xFF;

    int result = SPI_WriteBytes(send, sizeof(send));
    if (result < 0) {
        ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "BL0939: SPI write reg %02X failed result=%d", reg, result);
    }
    return result;
}

static int BL0939_ReadRaw(BL0939_RawData_t *data) {
    uint32_t tmp;
    int errors = 0;

    if (BL0939_SPI_ReadReg(BL0939_REG_V_RMS, &data->v_rms) < 0) errors++;
    if (BL0939_SPI_ReadReg(BL0939_REG_IA_RMS, &data->ia_rms) < 0) errors++;
    if (BL0939_SPI_ReadReg(BL0939_REG_IB_RMS, &data->ib_rms) < 0) errors++;
    if (BL0939_SPI_ReadReg(BL0939_REG_A_WATT, &tmp) < 0) errors++; else data->a_watt = BL0939_Int24ToInt32(tmp);
    if (BL0939_SPI_ReadReg(BL0939_REG_B_WATT, &tmp) < 0) errors++; else data->b_watt = BL0939_Int24ToInt32(tmp);
    if (BL0939_SPI_ReadReg(BL0939_REG_CFA_CNT, &data->cfa_cnt) < 0) errors++;
    if (BL0939_SPI_ReadReg(BL0939_REG_CFB_CNT, &data->cfb_cnt) < 0) errors++;
    if (BL0939_SPI_ReadReg(BL0939_REG_A_CORNER, &data->a_corner) < 0) errors++;
    if (BL0939_SPI_ReadReg(BL0939_REG_B_CORNER, &data->b_corner) < 0) errors++;

    return errors ? -1 : 0;
}

static void BL0939_SetEnergyStat(BL0939_PhaseChannel_t channel, float import_kwh, float export_kwh) {
    ENERGY_DATA *data = channel == BL0939_CHANNEL_B ? &energy_acc_b : &energy_acc_a;
    data->Import = import_kwh;
    data->Export = export_kwh;
    BL0939_SaveStats(1);
}

static void BL0939_SetChannelTypes(void) {
    CHANNEL_SetType(BL0939_CHANNEL_VOLTAGE, ChType_Voltage_div100);
    CHANNEL_SetType(BL0939_CHANNEL_CURRENT_A, ChType_Current_div1000);
    CHANNEL_SetType(BL0939_CHANNEL_POWER_A, ChType_Power_div100);
    CHANNEL_SetType(BL0939_CHANNEL_IMPORT_A, ChType_EnergyImport_kWh_div1000);
    CHANNEL_SetType(BL0939_CHANNEL_EXPORT_A, ChType_EnergyExport_kWh_div1000);
    CHANNEL_SetType(BL0939_CHANNEL_CURRENT_B, ChType_Current_div1000);
    CHANNEL_SetType(BL0939_CHANNEL_POWER_B, ChType_Power_div100);
    CHANNEL_SetType(BL0939_CHANNEL_IMPORT_B, ChType_EnergyImport_kWh_div1000);
    CHANNEL_SetType(BL0939_CHANNEL_EXPORT_B, ChType_EnergyExport_kWh_div1000);
    CHANNEL_SetType(BL0939_CHANNEL_IMPORT_TOTAL, ChType_EnergyImport_kWh_div1000);
    CHANNEL_SetType(BL0939_CHANNEL_EXPORT_TOTAL, ChType_EnergyExport_kWh_div1000);
    CHANNEL_SetType(BL0939_CHANNEL_POWER_FACTOR_A, ChType_PowerFactor_div1000);
    CHANNEL_SetType(BL0939_CHANNEL_POWER_FACTOR_B, ChType_PowerFactor_div1000);

    CHANNEL_SetLabel(BL0939_CHANNEL_VOLTAGE, "Voltage", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_CURRENT_A, "Current A", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_POWER_A, "Power A", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_IMPORT_A, "Energy Import A", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_EXPORT_A, "Energy Export A", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_CURRENT_B, "Current B", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_POWER_B, "Power B", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_IMPORT_B, "Energy Import B", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_EXPORT_B, "Energy Export B", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_IMPORT_TOTAL, "Energy Import Total", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_EXPORT_TOTAL, "Energy Export Total", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_POWER_FACTOR_A, "Power Factor A", 1);
    CHANNEL_SetLabel(BL0939_CHANNEL_POWER_FACTOR_B, "Power Factor B", 1);
}

static void BL0939_PublishChannelsEx(int force) {
    int flags = force ? BL0939_CHANSET_FLAGS_FORCE : BL0939_CHANSET_FLAGS;
    CHANNEL_SetSmart(BL0939_CHANNEL_VOLTAGE, last_update.voltage, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_CURRENT_A, last_update.current_a, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_POWER_A, last_update.power_a, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_IMPORT_A, energy_acc_a.Import, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_EXPORT_A, energy_acc_a.Export, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_CURRENT_B, last_update.current_b, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_POWER_B, last_update.power_b, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_IMPORT_B, energy_acc_b.Import, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_EXPORT_B, energy_acc_b.Export, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_IMPORT_TOTAL, energy_acc_a.Import + energy_acc_b.Import, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_EXPORT_TOTAL, energy_acc_a.Export + energy_acc_b.Export, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_POWER_FACTOR_A, last_update.pf_a, flags);
    CHANNEL_SetSmart(BL0939_CHANNEL_POWER_FACTOR_B, last_update.pf_b, flags);
}

static void BL0939_PublishChannels(void) {
    BL0939_PublishChannelsEx(0);
}

static void BL0939_PublishChannelsForce(void) {
    BL0939_PublishChannelsEx(1);
}

static void BL0939_IntegrateEnergy(void) {
    portTickType now = xTaskGetTickCount();
    if (last_energy_tick == 0) {
        last_energy_tick = now;
        return;
    }

    int ticks = (int)(now - last_energy_tick);
    if (ticks <= 0) {
        ticks = 1;
    }
    last_energy_tick = now;

    float hours = (ticks * portTICK_PERIOD_MS) / 3600000.0f;
    float energy_a_kwh = fabsf(last_update.power_a) * hours / 1000.0f;
    float energy_b_kwh = fabsf(last_update.power_b) * hours / 1000.0f;

    if (last_update.power_a >= 0.0f) {
        energy_acc_a.Import += energy_a_kwh;
    } else {
        energy_acc_a.Export += energy_a_kwh;
    }

    if (last_update.power_b >= 0.0f) {
        energy_acc_b.Import += energy_b_kwh;
    } else {
        energy_acc_b.Export += energy_b_kwh;
    }
}

static void BL0939_ScaleAndUpdate(const BL0939_RawData_t *data) {
    last_update.voltage = BL0939_SafeDivide((float)data->v_rms, cal_voltage);
    last_update.current_a = BL0939_SafeDivide((float)data->ia_rms, cal_current_a);
    last_update.current_b = BL0939_SafeDivide((float)data->ib_rms, cal_current_b);
    last_update.power_a = BL0939_SafeDivide((float)data->a_watt, cal_power_a);
    last_update.power_b = BL0939_SafeDivide((float)data->b_watt, cal_power_b);

    last_update.apparent_a = last_update.voltage * last_update.current_a;
    last_update.apparent_b = last_update.voltage * last_update.current_b;
    last_update.pf_a = last_update.apparent_a <= 0.001f ? 1.0f : last_update.power_a / last_update.apparent_a;
    last_update.pf_b = last_update.apparent_b <= 0.001f ? 1.0f : last_update.power_b / last_update.apparent_b;

    if (last_update.pf_a > 1.0f) last_update.pf_a = 1.0f;
    if (last_update.pf_a < -1.0f) last_update.pf_a = -1.0f;
    if (last_update.pf_b > 1.0f) last_update.pf_b = 1.0f;
    if (last_update.pf_b < -1.0f) last_update.pf_b = -1.0f;

    if (prev_cfa_cnt != BL0939_INVALID_CF_CNT) {
        int32_t cfa_delta = BL0939_CalcSignedCounterDelta(data->cfa_cnt, prev_cfa_cnt);
        int32_t cfb_delta = BL0939_CalcSignedCounterDelta(data->cfb_cnt, prev_cfb_cnt);
        ADDLOG_DEBUG(LOG_FEATURE_ENERGYMETER, "BL0939: cf delta A=%ld B=%ld", (long)cfa_delta, (long)cfb_delta);
    }
    prev_cfa_cnt = data->cfa_cnt;
    prev_cfb_cnt = data->cfb_cnt;

    BL0939_IntegrateEnergy();
    BL0939_PublishChannels();
}

static commandResult_t BL0939_CalibrateFloat(const char *cmd, const char *args, float raw, int cfg_index, float *cal) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }

    float real = Tokenizer_GetArgFloat(0);
    if (real > -0.000001f && real < 0.000001f) {
        ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "BL0939: calibration value cannot be zero");
        return CMD_RES_BAD_ARGUMENT;
    }
    if (raw > -0.001f && raw < 0.001f) {
        ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "BL0939: raw calibration value is zero; connect load first");
        return CMD_RES_ERROR;
    }

    *cal = raw / real;
    CFG_SetPowerMeasurementCalibrationFloat(cfg_index, *cal);
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER, "BL0939: %s set calibration to %f", cmd, *cal);
    return CMD_RES_OK;
}

static commandResult_t BL0939_CalibrateVoltage(const void *context, const char *cmd, const char *args, int cmdFlags) {
    return BL0939_CalibrateFloat(cmd, args, (float)last_raw.v_rms, CFG_OBK_VOLTAGE, &cal_voltage);
}

static commandResult_t BL0939_CalibrateCurrentA(const void *context, const char *cmd, const char *args, int cmdFlags) {
    return BL0939_CalibrateFloat(cmd, args, (float)last_raw.ia_rms, CFG_OBK_CURRENT, &cal_current_a);
}

static commandResult_t BL0939_CalibrateCurrentB(const void *context, const char *cmd, const char *args, int cmdFlags) {
    return BL0939_CalibrateFloat(cmd, args, (float)last_raw.ib_rms, CFG_OBK_RES_KIA, &cal_current_b);
}

static commandResult_t BL0939_CalibratePowerA(const void *context, const char *cmd, const char *args, int cmdFlags) {
    return BL0939_CalibrateFloat(cmd, args, (float)last_raw.a_watt, CFG_OBK_POWER, &cal_power_a);
}

static commandResult_t BL0939_CalibratePowerB(const void *context, const char *cmd, const char *args, int cmdFlags) {
    return BL0939_CalibrateFloat(cmd, args, (float)last_raw.b_watt, CFG_OBK_RES_KIB, &cal_power_b);
}

static commandResult_t BL0939_SetEnergy(const void *context, const char *cmd, const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 4)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }

    BL0939_SetEnergyStat(BL0939_CHANNEL_A, Tokenizer_GetArgFloat(0), Tokenizer_GetArgFloat(1));
    BL0939_SetEnergyStat(BL0939_CHANNEL_B, Tokenizer_GetArgFloat(2), Tokenizer_GetArgFloat(3));
    BL0939_PublishChannelsForce();
    return CMD_RES_OK;
}

static commandResult_t BL0939_ClearEnergy(const void *context, const char *cmd, const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    const char *channel = Tokenizer_GetArgsCount() >= 1 ? Tokenizer_GetArg(0) : "all";

    if (!strcmp(channel, "a") || !strcmp(channel, "A") || !strcmp(channel, "channel_a")) {
        BL0939_SetEnergyStat(BL0939_CHANNEL_A, 0, 0);
    } else if (!strcmp(channel, "b") || !strcmp(channel, "B") || !strcmp(channel, "channel_b")) {
        BL0939_SetEnergyStat(BL0939_CHANNEL_B, 0, 0);
    } else {
        BL0939_SetEnergyStat(BL0939_CHANNEL_A, 0, 0);
        BL0939_SetEnergyStat(BL0939_CHANNEL_B, 0, 0);
    }
    BL0939_PublishChannelsForce();
    return CMD_RES_OK;
}

static commandResult_t BL0939_ReadRegCmd(const void *context, const char *cmd, const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }

    int reg = Tokenizer_GetArgInteger(0);
    if (reg < 0 || reg > 0xFF) {
        return CMD_RES_BAD_ARGUMENT;
    }
    uint32_t value = 0;
    int result = BL0939_SPI_ReadReg((uint8_t)reg, &value);
    ADDLOG_INFO(LOG_FEATURE_CMD, "BL0939_ReadReg result=%d reg=%02X value=%06lX", result, reg, (unsigned long)value);
    return result < 0 ? CMD_RES_ERROR : CMD_RES_OK;
}

static commandResult_t BL0939_WriteRegCmd(const void *context, const char *cmd, const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }

    int reg = Tokenizer_GetArgInteger(0);
    uint32_t value = (uint32_t)Tokenizer_GetArgInteger(1);
    if (reg < 0 || reg > 0xFF || value > 0xFFFFFF) {
        return CMD_RES_BAD_ARGUMENT;
    }
    int result = BL0939_SPI_WriteReg((uint8_t)reg, value);
    ADDLOG_INFO(LOG_FEATURE_CMD, "BL0939_WriteReg result=%d reg=%02X value=%06lX", result, reg, (unsigned long)value);
    return result < 0 ? CMD_RES_ERROR : CMD_RES_OK;
}

static commandResult_t BL0939_StatusCmd(const void *context, const char *cmd, const char *args, int cmdFlags) {
    ADDLOG_INFO(LOG_FEATURE_CMD,
                "BL0939 Status: V=%.2fV IA=%.3fA IB=%.3fA PA=%.2fW PB=%.2fW ImpA=%.4fkWh ExpA=%.4fkWh ImpB=%.4fkWh ExpB=%.4fkWh csum=%lu read=%lu",
                last_update.voltage,
                last_update.current_a,
                last_update.current_b,
                last_update.power_a,
                last_update.power_b,
                energy_acc_a.Import,
                energy_acc_a.Export,
                energy_acc_b.Import,
                energy_acc_b.Export,
                (unsigned long)checksum_errors,
                (unsigned long)read_errors);
    return CMD_RES_OK;
}

static void BL0939_AddCommands(void) {
    //cmddetail:{"name":"BL0939_Status","args":"","descr":"Print current BL0939 scaled readings, energy counters and SPI error counters.","fn":"BL0939_StatusCmd","file":"driver/drv_bl0939.c","requires":"ENABLE_DRIVER_BL0939SPI","examples":"BL0939_Status"}
    CMD_RegisterCommand("BL0939_Status", BL0939_StatusCmd, NULL);
    //cmddetail:{"name":"BL0939_SetVoltage","args":"voltage","descr":"Calibrate BL0939 voltage using the currently measured raw voltage and a known real voltage.","fn":"BL0939_CalibrateVoltage","file":"driver/drv_bl0939.c","requires":"ENABLE_DRIVER_BL0939SPI","examples":"BL0939_SetVoltage 230.0"}
    CMD_RegisterCommand("BL0939_SetVoltage", BL0939_CalibrateVoltage, NULL);
    //cmddetail:{"name":"BL0939_SetCurrentA","args":"current","descr":"Calibrate BL0939 channel A current using a known real current in amps.","fn":"BL0939_CalibrateCurrentA","file":"driver/drv_bl0939.c","requires":"ENABLE_DRIVER_BL0939SPI","examples":"BL0939_SetCurrentA 1.25"}
    CMD_RegisterCommand("BL0939_SetCurrentA", BL0939_CalibrateCurrentA, NULL);
    //cmddetail:{"name":"BL0939_SetCurrentB","args":"current","descr":"Calibrate BL0939 channel B current using a known real current in amps.","fn":"BL0939_CalibrateCurrentB","file":"driver/drv_bl0939.c","requires":"ENABLE_DRIVER_BL0939SPI","examples":"BL0939_SetCurrentB 1.25"}
    CMD_RegisterCommand("BL0939_SetCurrentB", BL0939_CalibrateCurrentB, NULL);
    //cmddetail:{"name":"BL0939_SetPowerA","args":"power","descr":"Calibrate BL0939 channel A active power using a known real power in watts.","fn":"BL0939_CalibratePowerA","file":"driver/drv_bl0939.c","requires":"ENABLE_DRIVER_BL0939SPI","examples":"BL0939_SetPowerA 100.0"}
    CMD_RegisterCommand("BL0939_SetPowerA", BL0939_CalibratePowerA, NULL);
    //cmddetail:{"name":"BL0939_SetPowerB","args":"power","descr":"Calibrate BL0939 channel B active power using a known real power in watts.","fn":"BL0939_CalibratePowerB","file":"driver/drv_bl0939.c","requires":"ENABLE_DRIVER_BL0939SPI","examples":"BL0939_SetPowerB 100.0"}
    CMD_RegisterCommand("BL0939_SetPowerB", BL0939_CalibratePowerB, NULL);
    //cmddetail:{"name":"BL0939_SetEnergy","args":"importA exportA importB exportB","descr":"Set BL0939 accumulated import/export kWh counters.","fn":"BL0939_SetEnergy","file":"driver/drv_bl0939.c","requires":"ENABLE_DRIVER_BL0939SPI","examples":"BL0939_SetEnergy 0 0 0 0"}
    CMD_RegisterCommand("BL0939_SetEnergy", BL0939_SetEnergy, NULL);
    //cmddetail:{"name":"BL0939_ClearEnergy","args":"[a|b|all]","descr":"Clear BL0939 accumulated import/export kWh counters.","fn":"BL0939_ClearEnergy","file":"driver/drv_bl0939.c","requires":"ENABLE_DRIVER_BL0939SPI","examples":"BL0939_ClearEnergy all"}
    CMD_RegisterCommand("BL0939_ClearEnergy", BL0939_ClearEnergy, NULL);
    CMD_RegisterCommand("BL0939_ReadReg", BL0939_ReadRegCmd, NULL);
    CMD_RegisterCommand("BL0939_WriteReg", BL0939_WriteRegCmd, NULL);
}

void BL0939_SPI_Init(void) {
    memset(&last_raw, 0, sizeof(last_raw));
    memset(&last_update, 0, sizeof(last_update));
    prev_cfa_cnt = BL0939_INVALID_CF_CNT;
    prev_cfb_cnt = BL0939_INVALID_CF_CNT;
    checksum_errors = 0;
    read_errors = 0;
    last_energy_tick = 0;

    BL0939_LoadCalibration();
    BL0939_RestoreStats();
    BL0939_SetChannelTypes();
    BL0939_AddCommands();

    SPI_DriverInit();
    spi_config_t cfg;
    cfg.role = SPI_ROLE_MASTER;
    cfg.bit_width = SPI_BIT_WIDTH_8BITS;
    cfg.polarity = SPI_POLARITY_LOW;
    cfg.phase = SPI_PHASE_2ND_EDGE;
    cfg.wire_mode = SPI_3WIRE_MODE;
    cfg.baud_rate = BL0939_SPI_BAUD_RATE;
    cfg.bit_order = SPI_MSB_FIRST;
    OBK_SPI_Init(&cfg);

    BL0939_SPI_WriteReg(BL0939_REG_USR_WRPROT, BL0939_USR_WRPROT_DISABLE);
    BL0939_PublishChannelsForce();
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER, "BL0939SPI initialized");
}

void BL0939_SPI_Stop(void) {
    BL0939_SaveStats(1);
    SPI_Deinit();
    SPI_DriverDeinit();
}

void BL0939_SPI_RunEverySecond(void) {
    BL0939_RawData_t data;
    memset(&data, 0, sizeof(data));

    if (BL0939_ReadRaw(&data) < 0) {
        return;
    }
    last_raw = data;
    BL0939_ScaleAndUpdate(&data);
    BL0939_SaveStats(0);
}

static void BL0939_AppendTableRow(http_request_t *request, const char *name, const char *unit, float value, int decimals) {
    hprintf255(request,
               "<tr><td><b>%s</b></td><td style='text-align:right;'>%.*f</td><td>%s</td></tr>",
               name, decimals, value, unit);
}

static void BL0939_AppendDualRow(http_request_t *request, const char *name, const char *unit, float value_a, float value_b, int decimals) {
    hprintf255(request,
               "<tr><td><b>%s</b></td><td style='text-align:right;'>%.*f</td><td>%s</td><td style='text-align:right;'>%.*f</td><td>%s</td></tr>",
               name, decimals, value_a, unit, decimals, value_b, unit);
}

void BL0939_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
    if (bPreState) {
        char channel[8];
        if (http_getArg(request->url, "bl0939_clear", channel, sizeof(channel))) {
            if (!strcmp(channel, "a")) {
                BL0939_SetEnergyStat(BL0939_CHANNEL_A, 0, 0);
            } else if (!strcmp(channel, "b")) {
                BL0939_SetEnergyStat(BL0939_CHANNEL_B, 0, 0);
            } else {
                BL0939_SetEnergyStat(BL0939_CHANNEL_A, 0, 0);
                BL0939_SetEnergyStat(BL0939_CHANNEL_B, 0, 0);
            }
            BL0939_PublishChannelsForce();
        }
        return;
    }

    poststr(request, "<hr><h2>BL0939SPI</h2>");
    poststr(request, "<table style='width:100%'>");
    BL0939_AppendTableRow(request, "Voltage", "V", last_update.voltage, 2);
    BL0939_AppendTableRow(request, "Total Import", "kWh", energy_acc_a.Import + energy_acc_b.Import, 4);
    BL0939_AppendTableRow(request, "Total Export", "kWh", energy_acc_a.Export + energy_acc_b.Export, 4);
    hprintf255(request, "<tr><td><b>Checksum Errors</b></td><td style='text-align:right;'>%lu</td><td></td></tr>", (unsigned long)checksum_errors);
    hprintf255(request, "<tr><td><b>Read Errors</b></td><td style='text-align:right;'>%lu</td><td></td></tr>", (unsigned long)read_errors);
    poststr(request, "</table>");

    poststr(request, "<hr><table style='width:100%'>");
    poststr(request, "<tr><th></th><th>Channel A</th><th></th><th>Channel B</th><th></th></tr>");
    BL0939_AppendDualRow(request, "Current", "A", last_update.current_a, last_update.current_b, 3);
    BL0939_AppendDualRow(request, "Power", "W", last_update.power_a, last_update.power_b, 2);
    BL0939_AppendDualRow(request, "Apparent", "VA", last_update.apparent_a, last_update.apparent_b, 2);
    BL0939_AppendDualRow(request, "Power Factor", "", last_update.pf_a, last_update.pf_b, 3);
    BL0939_AppendDualRow(request, "Import", "kWh", energy_acc_a.Import, energy_acc_b.Import, 4);
    BL0939_AppendDualRow(request, "Export", "kWh", energy_acc_a.Export, energy_acc_b.Export, 4);
    poststr(request,
            "<tr><td><b>Actions</b></td>"
            "<td style='text-align:right;'><button style='background-color:red;' onclick='location.href=\"?bl0939_clear=a\"'>Clear A</button></td><td></td>"
            "<td style='text-align:right;'><button style='background-color:red;' onclick='location.href=\"?bl0939_clear=b\"'>Clear B</button></td><td></td></tr>");
    poststr(request, "</table>");

    poststr(request, "<hr><table style='width:100%'>");
    hprintf255(request, "<tr><td><b>Raw V_RMS</b></td><td>%06lX</td></tr>", (unsigned long)last_raw.v_rms);
    hprintf255(request, "<tr><td><b>Raw IA_RMS</b></td><td>%06lX</td></tr>", (unsigned long)last_raw.ia_rms);
    hprintf255(request, "<tr><td><b>Raw IB_RMS</b></td><td>%06lX</td></tr>", (unsigned long)last_raw.ib_rms);
    hprintf255(request, "<tr><td><b>Raw A_WATT</b></td><td>%ld</td></tr>", (long)last_raw.a_watt);
    hprintf255(request, "<tr><td><b>Raw B_WATT</b></td><td>%ld</td></tr>", (long)last_raw.b_watt);
    hprintf255(request, "<tr><td><b>Raw CFA_CNT</b></td><td>%06lX</td></tr>", (unsigned long)last_raw.cfa_cnt);
    hprintf255(request, "<tr><td><b>Raw CFB_CNT</b></td><td>%06lX</td></tr>", (unsigned long)last_raw.cfb_cnt);
    poststr(request, "</table>");
}

static void BL0939_PublishHassButton(const char *topic, char *title, char *payload) {
    HassDeviceInfo *dev_info = hass_init_button_device_info(title, "BL0939_ClearEnergy", payload, HASS_CATEGORY_DIAGNOSTIC);
    if (dev_info == NULL) {
        return;
    }
    MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
    hass_free_device_info(dev_info);
}

void BL0939_OnHassDiscovery(const char *topic) {
    BL0939_PublishHassButton(topic, "Clear BL0939 Energy A", "a");
    BL0939_PublishHassButton(topic, "Clear BL0939 Energy B", "b");
    BL0939_PublishHassButton(topic, "Clear BL0939 Energy All", "all");
}

void BL0939_Save_Statistics(void) {
    if (DRV_IsRunning("BL0939SPI")) {
        BL0939_SaveStats(1);
    }
}

#endif
