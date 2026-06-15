#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "hal_pin.h"
#include "hal_pin_def.h"
#include "hal_system.h"
#include "hal_vic.h"
#include "openopl1000_console.h"
#include "openopl1000_wifi_sta.h"

#define OPENOPL1000_CONSOLE_MAX_CMD 160

static int OpenOPL1000_StrCaseCmp(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = *a;
        char cb = *b;

        if ((ca >= 'A') && (ca <= 'Z'))
        {
            ca = (char)(ca - 'A' + 'a');
        }
        if ((cb >= 'A') && (cb <= 'Z'))
        {
            cb = (char)(cb - 'A' + 'a');
        }
        if (ca != cb)
        {
            return (int)((unsigned char)ca - (unsigned char)cb);
        }
        a++;
        b++;
    }

    return (int)((unsigned char)*a - (unsigned char)*b);
}

static int OpenOPL1000_StrNCaseCmp(const char *a, const char *b, size_t n)
{
    while (n--)
    {
        char ca = *a;
        char cb = *b;

        if ((ca >= 'A') && (ca <= 'Z'))
        {
            ca = (char)(ca - 'A' + 'a');
        }
        if ((cb >= 'A') && (cb <= 'Z'))
        {
            cb = (char)(cb - 'A' + 'a');
        }
        if (ca != cb)
        {
            return (int)((unsigned char)ca - (unsigned char)cb);
        }
        if (*a == '\0')
        {
            return 0;
        }
        a++;
        b++;
    }

    return 0;
}

static bool OpenOPL1000_IsSafeUserGpio(int pin)
{
    /* Avoid UART, flash-SPI and ICE/debug pins by default. The force variants
     * exist for board bring-up when the user has verified the pad externally. */
    if ((pin >= 2) && (pin <= 7))
    {
        return true;
    }
    if ((pin == 10) || (pin == 11))
    {
        return true;
    }
    if ((pin >= 16) && (pin <= 19))
    {
        return true;
    }
    if ((pin == 22) || (pin == 23))
    {
        return true;
    }

    return false;
}

static bool OpenOPL1000_PinInRange(int pin)
{
    return (pin >= 0) && (pin < GPIO_IDX_MAX);
}

static void OpenOPL1000_Trim(char *text)
{
    char *start;
    char *end;

    if (text == NULL)
    {
        return;
    }

    start = text;
    while ((*start == ' ') || (*start == '\t') || (*start == '\r') || (*start == '\n'))
    {
        start++;
    }

    if (start != text)
    {
        memmove(text, start, strlen(start) + 1);
    }

    end = text + strlen(text);
    while ((end > text) && ((end[-1] == ' ') || (end[-1] == '\t') || (end[-1] == '\r') || (end[-1] == '\n')))
    {
        *--end = '\0';
    }
}

static char *OpenOPL1000_NextToken(char **cursor)
{
    char *token;
    char *p;

    if ((cursor == NULL) || (*cursor == NULL))
    {
        return NULL;
    }

    p = *cursor;
    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }

    if (*p == '\0')
    {
        *cursor = p;
        return NULL;
    }

    token = p;
    while ((*p != '\0') && (*p != ' ') && (*p != '\t'))
    {
        p++;
    }

    if (*p != '\0')
    {
        *p++ = '\0';
    }

    *cursor = p;
    return token;
}

static bool OpenOPL1000_ParseInt(const char *text, int *value)
{
    char *endPtr;
    long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return false;
    }

    parsed = strtol(text, &endPtr, 0);
    if (*endPtr != '\0')
    {
        return false;
    }

    *value = (int)parsed;
    return true;
}

static void OpenOPL1000_AppendHelp(char *output, size_t outputLen)
{
    snprintf(output, outputLen,
             "OpenOPL1000 console commands:\r\n"
             "  help                       show this help\r\n"
             "  status                     show Wi-Fi/runtime status\r\n"
             "  scan                       start a Wi-Fi scan\r\n"
             "  scanresults                show cached scan results\r\n"
             "  reconnect                  disconnect and rescan/connect\r\n"
             "  gpio                       list GPIO command help\r\n"
             "  gpio_read <pin>            configure safe pin as input and read it\r\n"
             "  gpio_input <pin>           configure safe pin as input\r\n"
             "  gpio_output <pin> <0|1>    configure safe pin as output\r\n"
             "  gpio_force_read <pin>      read any valid pin, including reserved pins\r\n"
             "  gpio_force_output <pin> <0|1> output any valid pin, including reserved pins\r\n"
             "  reboot                     reboot using Hal_Sys_SwResetAll\r\n"
             "\r\nSafe GPIOs for default commands: 2-7, 10-11, 16-19, 22-23.\r\n"
             "Avoid flash-SPI, UART and ICE pins unless you know the board wiring.\r\n");
}

static void OpenOPL1000_AppendGpioHelp(char *output, size_t outputLen)
{
    snprintf(output, outputLen,
             "GPIO commands:\r\n"
             "  gpio_read <pin>\r\n"
             "  gpio_input <pin>\r\n"
             "  gpio_output <pin> <0|1>\r\n"
             "  gpio_force_read <pin>\r\n"
             "  gpio_force_input <pin>\r\n"
             "  gpio_force_output <pin> <0|1>\r\n"
             "\r\nDefault-safe pins: 2-7, 10-11, 16-19, 22-23.\r\n"
             "Reserved by this first-stage pinmux: 0/1 UART1, 8/9 APS UART, 12-15 SPI0 flash, 20/21 ICE.\r\n");
}

static void OpenOPL1000_GpioInputCommand(int pin, bool force, char *output, size_t outputLen, bool readAfter)
{
    E_GpioLevel_t level;

    if (!OpenOPL1000_PinInRange(pin))
    {
        snprintf(output, outputLen, "ERROR: pin %d is outside GPIO range 0-%d\r\n", pin, GPIO_IDX_MAX - 1);
        return;
    }

    if (!force && !OpenOPL1000_IsSafeUserGpio(pin))
    {
        snprintf(output, outputLen,
                 "ERROR: pin %d is not in the default-safe GPIO set. Use gpio_force_input/gpio_force_read only if board wiring is confirmed.\r\n",
                 pin);
        return;
    }

    Hal_Pin_ConfigSet((uint8_t)pin, PIN_TYPE_GPIO_INPUT, PIN_DRIVING_LOW);
    Hal_Vic_GpioDirection((E_GpioIdx_t)pin, GPIO_INPUT);
    level = Hal_Vic_GpioInput((E_GpioIdx_t)pin);

    snprintf(output, outputLen,
             readAfter ? "GPIO%d input level=%d\r\n" : "GPIO%d configured as input, level=%d\r\n",
             pin,
             (level == GPIO_LEVEL_HIGH) ? 1 : 0);
}

static void OpenOPL1000_GpioOutputCommand(int pin, int value, bool force, char *output, size_t outputLen)
{
    E_GpioLevel_t level = value ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;

    if (!OpenOPL1000_PinInRange(pin))
    {
        snprintf(output, outputLen, "ERROR: pin %d is outside GPIO range 0-%d\r\n", pin, GPIO_IDX_MAX - 1);
        return;
    }

    if ((value != 0) && (value != 1))
    {
        snprintf(output, outputLen, "ERROR: GPIO output value must be 0 or 1\r\n");
        return;
    }

    if (!force && !OpenOPL1000_IsSafeUserGpio(pin))
    {
        snprintf(output, outputLen,
                 "ERROR: pin %d is not in the default-safe GPIO set. Use gpio_force_output only if board wiring is confirmed.\r\n",
                 pin);
        return;
    }

    Hal_Pin_ConfigSet((uint8_t)pin,
                      value ? PIN_TYPE_GPIO_OUTPUT_HIGH : PIN_TYPE_GPIO_OUTPUT_LOW,
                      PIN_DRIVING_FLOAT);
    Hal_Vic_GpioDirection((E_GpioIdx_t)pin, GPIO_OUTPUT);
    Hal_Vic_GpioOutput((E_GpioIdx_t)pin, level);

    snprintf(output, outputLen, "GPIO%d output set to %d%s\r\n", pin, value, force ? " (forced)" : "");
}

static void OpenOPL1000_RebootTask(void *args)
{
    (void)args;
    osDelay(500);
    printf("[OpenOPL1000] reboot command executing\r\n");
    Hal_Sys_SwResetAll();
    while (1)
    {
        osDelay(1000);
    }
}

static void OpenOPL1000_ScheduleReboot(char *output, size_t outputLen)
{
    osThreadDef_t threadDef;
    osThreadId threadId;

    threadDef.name = "openopl1000_reboot";
    threadDef.stacksize = 512;
    threadDef.tpriority = OS_TASK_PRIORITY_APP;
    threadDef.instances = 0;
    threadDef.pthread = OpenOPL1000_RebootTask;

    threadId = osThreadCreate(&threadDef, NULL);
    if (threadId == NULL)
    {
        snprintf(output, outputLen, "ERROR: failed to create reboot task\r\n");
    }
    else
    {
        snprintf(output, outputLen, "Reboot scheduled.\r\n");
    }
}

void OpenOPL1000_ConsoleExecute(const char *command, char *output, size_t outputLen)
{
    char local[OPENOPL1000_CONSOLE_MAX_CMD];
    char *cursor;
    char *verb;
    char *pinText;
    char *valueText;
    int pin;
    int value;

    if ((output == NULL) || (outputLen == 0))
    {
        return;
    }

    output[0] = '\0';

    if (command == NULL)
    {
        OpenOPL1000_AppendHelp(output, outputLen);
        return;
    }

    snprintf(local, sizeof(local), "%s", command);
    OpenOPL1000_Trim(local);

    if (local[0] == '\0')
    {
        OpenOPL1000_AppendHelp(output, outputLen);
        return;
    }

    cursor = local;
    verb = OpenOPL1000_NextToken(&cursor);
    if (verb == NULL)
    {
        OpenOPL1000_AppendHelp(output, outputLen);
        return;
    }

    if ((OpenOPL1000_StrCaseCmp(verb, "help") == 0) || (OpenOPL1000_StrCaseCmp(verb, "?") == 0))
    {
        OpenOPL1000_AppendHelp(output, outputLen);
        return;
    }

    if ((OpenOPL1000_StrCaseCmp(verb, "status") == 0) || (OpenOPL1000_StrCaseCmp(verb, "state") == 0))
    {
        OpenOPL1000_WifiFormatStatus(output, outputLen);
        return;
    }

    if (OpenOPL1000_StrCaseCmp(verb, "scan") == 0)
    {
        OpenOPL1000_WifiRequestScan();
        snprintf(output, outputLen, "Wi-Fi scan requested. Run scanresults after the scan-complete UART event.\r\n");
        return;
    }

    if ((OpenOPL1000_StrCaseCmp(verb, "scanresults") == 0) || (OpenOPL1000_StrCaseCmp(verb, "scan_results") == 0))
    {
        OpenOPL1000_WifiFormatScanResults(output, outputLen);
        return;
    }

    if (OpenOPL1000_StrCaseCmp(verb, "reconnect") == 0)
    {
        OpenOPL1000_WifiRequestReconnect();
        snprintf(output, outputLen, "Reconnect requested.\r\n");
        return;
    }

    if ((OpenOPL1000_StrCaseCmp(verb, "version") == 0) || (OpenOPL1000_StrCaseCmp(verb, "about") == 0))
    {
        snprintf(output, outputLen,
                 "OpenOPL1000 web console build\r\n"
                 "SDK target: OPL1000 A2 MP_2.21.x line\r\n"
                 "SoftAP: unavailable on this SDK; STA mode only\r\n"
                 "Target SSID: %s\r\n",
                 OpenOPL1000_WifiGetTargetSsid());
        return;
    }

    if (OpenOPL1000_StrCaseCmp(verb, "gpio") == 0)
    {
        OpenOPL1000_AppendGpioHelp(output, outputLen);
        return;
    }

    if ((OpenOPL1000_StrNCaseCmp(verb, "gpio_read", 9) == 0) ||
        (OpenOPL1000_StrNCaseCmp(verb, "gpio_input", 10) == 0) ||
        (OpenOPL1000_StrNCaseCmp(verb, "gpio_force_read", 15) == 0) ||
        (OpenOPL1000_StrNCaseCmp(verb, "gpio_force_input", 16) == 0))
    {
        bool force = (OpenOPL1000_StrNCaseCmp(verb, "gpio_force", 10) == 0);
        bool readAfter = (strstr(verb, "read") != NULL);

        pinText = OpenOPL1000_NextToken(&cursor);
        if (!OpenOPL1000_ParseInt(pinText, &pin))
        {
            snprintf(output, outputLen, "ERROR: expected pin number\r\n");
            return;
        }

        OpenOPL1000_GpioInputCommand(pin, force, output, outputLen, readAfter);
        return;
    }

    if ((OpenOPL1000_StrNCaseCmp(verb, "gpio_output", 11) == 0) ||
        (OpenOPL1000_StrNCaseCmp(verb, "gpio_force_output", 17) == 0))
    {
        bool force = (OpenOPL1000_StrNCaseCmp(verb, "gpio_force", 10) == 0);

        pinText = OpenOPL1000_NextToken(&cursor);
        valueText = OpenOPL1000_NextToken(&cursor);
        if (!OpenOPL1000_ParseInt(pinText, &pin) || !OpenOPL1000_ParseInt(valueText, &value))
        {
            snprintf(output, outputLen, "ERROR: expected pin number and value, e.g. gpio_output 16 1\r\n");
            return;
        }

        OpenOPL1000_GpioOutputCommand(pin, value, force, output, outputLen);
        return;
    }

    if (OpenOPL1000_StrCaseCmp(verb, "reboot") == 0)
    {
        OpenOPL1000_ScheduleReboot(output, outputLen);
        return;
    }

    snprintf(output, outputLen, "ERROR: unknown command '%s'. Try help.\r\n", verb);
}
