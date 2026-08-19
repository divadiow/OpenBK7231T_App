// RN8209/RN8209C UART power-meter driver.
//
// The RN8209C reference implementation uses 4800 baud, 8 data bits,
// even parity and 1 stop bit. A register read is a one-byte register
// address followed by the register data (MSB first) and an inverted
// additive checksum over the address and data bytes.

#include "../obk_config.h"

#if ENABLE_DRIVER_RN8209

#include <math.h>
#include <stdint.h>

#include "../logging/logging.h"
#include "../new_pins.h"
#include "drv_bl_shared.h"
#include "drv_pwrCal.h"
#include "drv_uart.h"

#define RN8209_UART_BAUD_RATE             4800
#define RN8209_UART_PARITY_EVEN           2
#define RN8209_UART_RECEIVE_BUFFER_SIZE   64
#define RN8209_RESPONSE_TIMEOUT_MS        60
#define RN8209_RESPONSE_POLL_MS           5

#define RN8209_REG_CURRENT_A              0x22
#define RN8209_REG_VOLTAGE                0x24
#define RN8209_REG_ACTIVE_POWER_A         0x26
#define RN8209_REG_DEVICE_ID              0x7F

#define RN8209_CURRENT_SIZE               3
#define RN8209_VOLTAGE_SIZE               3
#define RN8209_ACTIVE_POWER_SIZE          4
#define RN8209_DEVICE_ID_SIZE             3

#define RN8209_DEVICE_ID                  0x820900U
#define RN8209_RMS_SIGN_BIT               0x800000U

#define DEFAULT_VOLTAGE_CAL               8810.0f
#define DEFAULT_CURRENT_CAL               57.0f
#define DEFAULT_POWER_CAL                 71572720.0f

typedef enum {
    RN8209_MEASURE_VOLTAGE = 0,
    RN8209_MEASURE_CURRENT,
    RN8209_MEASURE_POWER,
} rn8209_measurement_step_t;

static int rn8209_uart_port = -1;
static int rn8209_uart_init_counter = -1;
static unsigned int rn8209_read_error_count = 0;
static unsigned int rn8209_consecutive_read_errors = 0;

static rn8209_measurement_step_t rn8209_measurement_step = RN8209_MEASURE_VOLTAGE;
static bool rn8209_measurement_cycle_valid = true;
static uint32_t rn8209_raw_voltage = 0;
static uint32_t rn8209_raw_current = 0;
static uint32_t rn8209_raw_power = 0;

static void RN8209_ConfigureUART(void) {
    rn8209_uart_port = UART_GetSelectedPortIndex();
    UART_InitReceiveRingBuffer(RN8209_UART_RECEIVE_BUFFER_SIZE);
    rn8209_uart_init_counter =
        UART_InitUART(RN8209_UART_BAUD_RATE, RN8209_UART_PARITY_EVEN, false);
}

static void RN8209_EnsureUART(void) {
    int selected_port = UART_GetSelectedPortIndex();

    if (selected_port != rn8209_uart_port ||
        get_g_uart_init_counter() != rn8209_uart_init_counter) {
        RN8209_ConfigureUART();
        ADDLOG_INFO(LOG_FEATURE_ENERGYMETER,
                    "RN8209 UART%d restored at 4800 8E1",
                    rn8209_uart_port + 1);
    }
}

static void RN8209_RecordReadFailure(uint8_t reg, int received_bytes,
                                     bool checksum_failure) {
    rn8209_read_error_count++;
    rn8209_consecutive_read_errors++;

    // Keep a disconnected or misconfigured meter from flooding the log.
    if (rn8209_read_error_count <= 3 ||
        (rn8209_read_error_count % 30) == 0) {
        ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                    "RN8209 read 0x%02X failed: %s (%d bytes, error %u)",
                    reg,
                    checksum_failure ? "checksum" : "timeout",
                    received_bytes,
                    rn8209_read_error_count);
    }
}

static void RN8209_RecordReadSuccess(void) {
    if (rn8209_consecutive_read_errors >= 3) {
        ADDLOG_INFO(LOG_FEATURE_ENERGYMETER,
                    "RN8209 communication recovered after %u failed reads",
                    rn8209_consecutive_read_errors);
    }
    rn8209_consecutive_read_errors = 0;
}

static bool RN8209_ReadRegister(uint8_t reg, int data_size, uint32_t *value) {
    int available;
    int expected_size;
    int offset;
    int i;
    int waited_ms;

    if (value == NULL || data_size < 1 || data_size > 4) {
        return false;
    }

    RN8209_EnsureUART();

    // The reference SDK discards stale RX data before every transaction.
    available = UART_GetDataSize();
    if (available > 0) {
        UART_ConsumeBytes(available);
    }

    UART_SendByte(reg);

    expected_size = data_size + 1; // data plus checksum
    for (waited_ms = 0; waited_ms < RN8209_RESPONSE_TIMEOUT_MS;
         waited_ms += RN8209_RESPONSE_POLL_MS) {
        if (UART_GetDataSize() >= expected_size) {
            break;
        }
        rtos_delay_milliseconds(RN8209_RESPONSE_POLL_MS);
    }

    // If the minimum response length is present, allow one short grace period
    // before snapshotting the buffer. This keeps the parser tolerant of a
    // leading echo/noise byte that arrives before the final checksum byte.
    if (UART_GetDataSize() >= expected_size) {
        rtos_delay_milliseconds(RN8209_RESPONSE_POLL_MS);
    }

    available = UART_GetDataSize();
    if (available < expected_size) {
        RN8209_RecordReadFailure(reg, available, false);
        if (available > 0) {
            UART_ConsumeBytes(available);
        }
        return false;
    }

    // There is no packet header. If an echo/noise byte is present, scan the
    // short RX buffer for the first response window with a valid checksum.
    for (offset = 0; offset <= available - expected_size; offset++) {
        uint8_t checksum = reg;
        uint32_t result = 0;

        for (i = 0; i < data_size; i++) {
            checksum = (uint8_t)(checksum + UART_GetByte(offset + i));
        }
        checksum = (uint8_t)~checksum;

        if (checksum != UART_GetByte(offset + data_size)) {
            continue;
        }

        for (i = 0; i < data_size; i++) {
            result = (result << 8) | UART_GetByte(offset + i);
        }

        *value = result;
        UART_ConsumeBytes(available);
        RN8209_RecordReadSuccess();
        return true;
    }

    RN8209_RecordReadFailure(reg, available, true);
    UART_ConsumeBytes(available);
    return false;
}

static uint32_t RN8209_NormalizeRMS(uint32_t raw) {
    // The vendor SDK treats the 24-bit RMS sign bit as an invalid/negative
    // sample and reports zero for voltage/current in that case.
    return (raw & RN8209_RMS_SIGN_BIT) ? 0U : raw;
}

static int RN8209_NormalizeActivePower(uint32_t raw) {
    // The vendor SDK converts the signed 32-bit active-power register to
    // magnitude before applying its calibration. Preserve that behaviour;
    // OpenBeken's existing RN8209 integration is a single-direction meter.
    if (raw & 0x80000000U) {
        raw = (~raw) + 1U;
    }
    if (raw > 0x7FFFFFFFU) {
        raw = 0x7FFFFFFFU;
    }
    return (int)raw;
}

static void RN8209_PublishMeasurement(void) {
    float voltage;
    float current;
    float power;

    PwrCal_Scale((int)rn8209_raw_voltage,
                 (float)rn8209_raw_current,
                 RN8209_NormalizeActivePower(rn8209_raw_power),
                 &voltage, &current, &power);

    // NAN frequency and energy tell BL_Shared to hide unsupported frequency
    // and integrate calibrated power over elapsed time for energy accounting.
    BL_ProcessUpdate(voltage, current, power, NAN, NAN);
}

// Called by "startDriver RN8209".
void RN8209_Init(void) {
    uint32_t device_id = 0;

    rn8209_read_error_count = 0;
    rn8209_consecutive_read_errors = 0;
    rn8209_measurement_step = RN8209_MEASURE_VOLTAGE;
    rn8209_measurement_cycle_valid = true;
    rn8209_raw_voltage = 0;
    rn8209_raw_current = 0;
    rn8209_raw_power = 0;

    RN8209_ConfigureUART();

    BL_Shared_Init();
    PwrCal_Init(PWR_CAL_DIVIDE, DEFAULT_VOLTAGE_CAL, DEFAULT_CURRENT_CAL,
                DEFAULT_POWER_CAL);

    if (RN8209_ReadRegister(RN8209_REG_DEVICE_ID, RN8209_DEVICE_ID_SIZE,
                            &device_id)) {
        if (device_id == RN8209_DEVICE_ID) {
            ADDLOG_INFO(LOG_FEATURE_ENERGYMETER,
                        "RN8209 detected on UART%d (ID %06X, 4800 8E1)",
                        rn8209_uart_port + 1, (unsigned int)device_id);
        } else {
            ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                        "RN8209 unexpected device ID %06X on UART%d",
                        (unsigned int)device_id, rn8209_uart_port + 1);
        }
    }
}

// Name retained for the existing drv_main/drv_local integration.
void RN8029_RunEverySecond(void) {
    uint32_t value;
    bool ok;

    switch (rn8209_measurement_step) {
    case RN8209_MEASURE_VOLTAGE:
        rn8209_measurement_cycle_valid =
            RN8209_ReadRegister(RN8209_REG_VOLTAGE, RN8209_VOLTAGE_SIZE,
                                &value);
        if (rn8209_measurement_cycle_valid) {
            rn8209_raw_voltage = RN8209_NormalizeRMS(value);
        }
        rn8209_measurement_step = RN8209_MEASURE_CURRENT;
        break;

    case RN8209_MEASURE_CURRENT:
        ok = RN8209_ReadRegister(RN8209_REG_CURRENT_A, RN8209_CURRENT_SIZE,
                                 &value);
        if (ok) {
            rn8209_raw_current = RN8209_NormalizeRMS(value);
        } else {
            rn8209_measurement_cycle_valid = false;
        }
        rn8209_measurement_step = RN8209_MEASURE_POWER;
        break;

    case RN8209_MEASURE_POWER:
    default:
        ok = RN8209_ReadRegister(RN8209_REG_ACTIVE_POWER_A,
                                 RN8209_ACTIVE_POWER_SIZE, &value);
        if (ok) {
            rn8209_raw_power = value;
        } else {
            rn8209_measurement_cycle_valid = false;
        }

        if (rn8209_measurement_cycle_valid) {
            RN8209_PublishMeasurement();
        }

        rn8209_measurement_step = RN8209_MEASURE_VOLTAGE;
        break;
    }
}

#endif
