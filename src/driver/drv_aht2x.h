#pragma once

/*
 * AHT-family command constants.
 * Datasheet anchors:
 * - AHT10/AHT15: init 1110'0001 (0xE1), trigger 1010'1100 (0xAC), soft reset 1011'1010 (0xBA).
 * - AHT20/AHT21-style parts: status read via I2C read address 0x71, optional init 0xBE 0x08 0x00,
 *   trigger 0xAC 0x33 0x00, wait >=80 ms, read six data bytes plus optional CRC.
 * - AHT30: trigger write sequence is 0x70 0xAC 0x33 0x00, then read Status + SRH[19:0] + ST[19:0] + CRC.
 */
#define AHTXX_I2C_ADDR_7BIT              0x38
#define AHTXX_I2C_ADDR_WRITE             (AHTXX_I2C_ADDR_7BIT << 1)
#define AHTXX_I2C_ADDR_READ              (AHTXX_I2C_ADDR_WRITE | 1)

#define AHTXX_CMD_INIT_AHT1X             0xE1
#define AHTXX_CMD_INIT_AHT2X             0xBE
#define AHTXX_CMD_TRIGGER_MEASUREMENT    0xAC
#define AHTXX_CMD_SOFT_RESET             0xBA

#define AHTXX_DAT_INIT_1                 0x08
#define AHTXX_DAT_INIT_2                 0x00
#define AHTXX_DAT_TRIGGER_1              0x33
#define AHTXX_DAT_TRIGGER_2              0x00

#define AHTXX_STATUS_BUSY                0x80
#define AHTXX_STATUS_CALIBRATED          0x08
#define AHTXX_STATUS_AHT21_READY_MASK    0x18

#define AHTXX_MEASUREMENT_BYTES          6
#define AHTXX_MEASUREMENT_BYTES_CRC      7
#define AHTXX_CRC8_INIT                  0xFF
#define AHTXX_CRC8_POLY                  0x31

/* Legacy AHT2X names kept for source and script compatibility. */
#define AHT2X_I2C_ADDR                   AHTXX_I2C_ADDR_WRITE
#define AHT2X_CMD_INI                    AHTXX_CMD_INIT_AHT2X
#define AHT2X_DAT_INI1                   AHTXX_DAT_INIT_1
#define AHT2X_DAT_INI2                   AHTXX_DAT_INIT_2
#define AHT2X_CMD_TMS                    AHTXX_CMD_TRIGGER_MEASUREMENT
#define AHT2X_DAT_TMS1                   AHTXX_DAT_TRIGGER_1
#define AHT2X_DAT_TMS2                   AHTXX_DAT_TRIGGER_2
#define AHT2X_CMD_RST                    AHTXX_CMD_SOFT_RESET
#define AHT2X_DAT_BUSY                   AHTXX_STATUS_BUSY
