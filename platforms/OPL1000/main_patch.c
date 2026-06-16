#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "at_cmd_common_patch.h"
#include "boot_sequence.h"
#include "cmsis_os.h"
#include "hal_dbg_uart.h"
#include "hal_pin.h"
#include "hal_pin_config_project.h"
#include "hal_pin_def.h"
#include "hal_system.h"
#include "hal_vic.h"
#include "mw_fim.h"
#include "sys_init.h"
#include "sys_init_patch.h"
#include "sys_os_config.h"

static E_IO01_UART_MODE s_io01UartMode;

void __Patch_EntryPoint(void) __attribute__((section("ENTRY_POINT")));
void __Patch_EntryPoint(void) __attribute__((used));

static void Main_PinMuxUpdate(void);
static void Main_FlashLayoutUpdate(void);
static void Main_MiscModulesInit(void);
static void Main_MiscDriverConfigSetup(void);
static void Main_AtUartDbgUartSwitch(void);
static void Main_ApsUartRxDectecConfig(void);
static void Main_ApsUartRxDectecCb(E_GpioIdx_t gpioIdx);
static void Main_AppInit_patch(void);
static void OpenOPL1000_OpenBekenTask(void *args);
static void OpenOPL1000_EarlyLog(const char *text);

extern void Main_Init(void);
extern void Main_OnEverySecond(void);

typedef void (*T_Main_AppInit_fp)(void);
extern T_Main_AppInit_fp Main_AppInit;

void __Patch_EntryPoint(void)
{
    SysInit_EntryPoint();

#ifdef SWITCH_TO_32K_RC
    Sys_SwitchTo32kRC();
#endif

    Hal_SysPinMuxAppInit = Main_PinMuxUpdate;
    MwFim_FlashLayoutUpdate = Main_FlashLayoutUpdate;
    Sys_MiscModulesInit = Main_MiscModulesInit;
    Sys_MiscDriverConfigSetup = Main_MiscDriverConfigSetup;
    at_cmd_switch_uart1_dbguart = Main_AtUartDbgUartSwitch;

    Sys_SetUnsuedSramEndBound(0x440000);
    Sys_AppInit = Main_AppInit_patch;
}

static void Main_PinMuxUpdate(void)
{
    Hal_Pin_ConfigSet(0, HAL_PIN_TYPE_IO_0, HAL_PIN_DRIVING_IO_0);
    Hal_Pin_ConfigSet(1, HAL_PIN_TYPE_IO_1, HAL_PIN_DRIVING_IO_1);
    Hal_Pin_ConfigSet(2, HAL_PIN_TYPE_IO_2, HAL_PIN_DRIVING_IO_2);
    Hal_Pin_ConfigSet(3, HAL_PIN_TYPE_IO_3, HAL_PIN_DRIVING_IO_3);
    Hal_Pin_ConfigSet(4, HAL_PIN_TYPE_IO_4, HAL_PIN_DRIVING_IO_4);
    Hal_Pin_ConfigSet(5, HAL_PIN_TYPE_IO_5, HAL_PIN_DRIVING_IO_5);
    Hal_Pin_ConfigSet(6, HAL_PIN_TYPE_IO_6, HAL_PIN_DRIVING_IO_6);
    Hal_Pin_ConfigSet(7, HAL_PIN_TYPE_IO_7, HAL_PIN_DRIVING_IO_7);
    Hal_Pin_ConfigSet(8, HAL_PIN_TYPE_IO_8, HAL_PIN_DRIVING_IO_8);
    Hal_Pin_ConfigSet(9, HAL_PIN_TYPE_IO_9, HAL_PIN_DRIVING_IO_9);
    Hal_Pin_ConfigSet(10, HAL_PIN_TYPE_IO_10, HAL_PIN_DRIVING_IO_10);
    Hal_Pin_ConfigSet(11, HAL_PIN_TYPE_IO_11, HAL_PIN_DRIVING_IO_11);
    Hal_Pin_ConfigSet(12, HAL_PIN_TYPE_IO_12, HAL_PIN_DRIVING_IO_12);
    Hal_Pin_ConfigSet(13, HAL_PIN_TYPE_IO_13, HAL_PIN_DRIVING_IO_13);
    Hal_Pin_ConfigSet(14, HAL_PIN_TYPE_IO_14, HAL_PIN_DRIVING_IO_14);
    Hal_Pin_ConfigSet(15, HAL_PIN_TYPE_IO_15, HAL_PIN_DRIVING_IO_15);
    Hal_Pin_ConfigSet(16, HAL_PIN_TYPE_IO_16, HAL_PIN_DRIVING_IO_16);
    Hal_Pin_ConfigSet(17, HAL_PIN_TYPE_IO_17, HAL_PIN_DRIVING_IO_17);
    Hal_Pin_ConfigSet(18, HAL_PIN_TYPE_IO_18, HAL_PIN_DRIVING_IO_18);
    Hal_Pin_ConfigSet(19, HAL_PIN_TYPE_IO_19, HAL_PIN_DRIVING_IO_19);
    Hal_Pin_ConfigSet(20, HAL_PIN_TYPE_IO_20, HAL_PIN_DRIVING_IO_20);
    Hal_Pin_ConfigSet(21, HAL_PIN_TYPE_IO_21, HAL_PIN_DRIVING_IO_21);
    Hal_Pin_ConfigSet(22, HAL_PIN_TYPE_IO_22, HAL_PIN_DRIVING_IO_22);
    Hal_Pin_ConfigSet(23, HAL_PIN_TYPE_IO_23, HAL_PIN_DRIVING_IO_23);

    s_io01UartMode = HAL_PIN_0_1_UART_MODE;
    at_io01_uart_mode_set(HAL_PIN_0_1_UART_MODE);
}

static void Main_FlashLayoutUpdate(void)
{
}

static void Main_MiscModulesInit(void)
{
}

static void Main_MiscDriverConfigSetup(void)
{
    if (Boot_CheckWarmBoot() == 0)
    {
        Hal_DbgUart_RxIntEn(0);

        if (HAL_PIN_TYPE_IO_1 == PIN_TYPE_UART_APS_RX)
        {
            Main_ApsUartRxDectecConfig();
        }
    }
}

static void Main_AtUartDbgUartSwitch(void)
{
    if (s_io01UartMode == IO01_UART_MODE_AT)
    {
        Hal_Pin_ConfigSet(0, PIN_TYPE_UART_APS_TX, PIN_DRIVING_FLOAT);
        Hal_Pin_ConfigSet(1, PIN_TYPE_UART_APS_RX, PIN_DRIVING_LOW);
        Hal_Pin_ConfigSet(8, PIN_TYPE_UART1_TX, PIN_DRIVING_FLOAT);
        Hal_Pin_ConfigSet(9, PIN_TYPE_UART1_RX, PIN_DRIVING_HIGH);
        Hal_DbgUart_RxIntEn(1);
    }
    else
    {
        Hal_DbgUart_RxIntEn(0);
        Hal_Pin_ConfigSet(0, PIN_TYPE_UART1_TX, PIN_DRIVING_FLOAT);
        Hal_Pin_ConfigSet(1, PIN_TYPE_UART1_RX, PIN_DRIVING_LOW);
        Hal_Pin_ConfigSet(8, PIN_TYPE_UART_APS_TX, PIN_DRIVING_FLOAT);
        Hal_Pin_ConfigSet(9, PIN_TYPE_UART_APS_RX, PIN_DRIVING_HIGH);
    }

    s_io01UartMode = (E_IO01_UART_MODE)!s_io01UartMode;
}

static void Main_ApsUartRxDectecConfig(void)
{
    E_GpioLevel_t gpioLevel;

    Hal_Pin_ConfigSet(1, PIN_TYPE_GPIO_INPUT, PIN_DRIVING_LOW);
    gpioLevel = Hal_Vic_GpioInput(GPIO_IDX_01);

    if (gpioLevel == GPIO_LEVEL_HIGH)
    {
        Hal_Pin_ConfigSet(1, HAL_PIN_TYPE_IO_1, HAL_PIN_DRIVING_IO_1);
        Hal_DbgUart_RxIntEn(1);
    }
    else
    {
        Hal_Vic_GpioCallBackFuncSet(GPIO_IDX_01, Main_ApsUartRxDectecCb);
        Hal_Vic_GpioIntTypeSel(GPIO_IDX_01, INT_TYPE_LEVEL);
        Hal_Vic_GpioIntInv(GPIO_IDX_01, 0);
        Hal_Vic_GpioIntMask(GPIO_IDX_01, 0);
        Hal_Vic_GpioIntEn(GPIO_IDX_01, 1);
    }
}

static void Main_ApsUartRxDectecCb(E_GpioIdx_t gpioIdx)
{
    (void)gpioIdx;

    Hal_Vic_GpioIntEn(GPIO_IDX_01, 0);
    Hal_Pin_ConfigSet(1, HAL_PIN_TYPE_IO_1, HAL_PIN_DRIVING_IO_1);
    Hal_DbgUart_RxIntEn(1);
}


static void OpenOPL1000_EarlyLog(const char *text)
{
    const char *p;

    if (text == NULL)
    {
        return;
    }

    printf("%s", text);

    if (Hal_DbgUart_DataSend == NULL)
    {
        return;
    }

    for (p = text; *p != '\0'; p++)
    {
        if (*p == '\n')
        {
            Hal_DbgUart_DataSend('\r');
        }
        Hal_DbgUart_DataSend((uint32_t)(uint8_t)*p);
    }
}

static void OpenOPL1000_OpenBekenTask(void *args)
{
    (void)args;

    OpenOPL1000_EarlyLog("\r\n[OpenOPL1000] OpenBeken task entered\r\n");
    OpenOPL1000_EarlyLog("[OpenOPL1000] starting real OpenBeken runtime\r\n");
    Main_Init();

    while (1)
    {
        osDelay(1000);
        Main_OnEverySecond();
    }
}

static void Main_AppInit_patch(void)
{
    osThreadDef_t threadDef;

    /* Keep early bring-up logging on IO0/IO1.
     * The vendor default is often UART1/AT on IO0/IO1 and APS debug on IO8/IO9;
     * for OpenOPL1000 bring-up we force APS/debug UART onto IO0/IO1 so the
     * same serial connection that shows the ROM/bootloader text also shows app text.
     */
    Hal_Pin_ConfigSet(0, PIN_TYPE_UART_APS_TX, PIN_DRIVING_FLOAT);
    Hal_Pin_ConfigSet(1, PIN_TYPE_UART_APS_RX, PIN_DRIVING_HIGH);
    Hal_Pin_ConfigSet(8, PIN_TYPE_UART1_TX, PIN_DRIVING_FLOAT);
    Hal_Pin_ConfigSet(9, PIN_TYPE_UART1_RX, PIN_DRIVING_HIGH);
    s_io01UartMode = IO01_UART_MODE_DBG;
    at_io01_uart_mode_set(IO01_UART_MODE_DBG);

    Hal_DbgUart_Init(115200);
    Hal_DbgUart_RxIntEn(1);
    OpenOPL1000_EarlyLog("\r\n[OpenOPL1000] Main_AppInit_patch reached; APS/debug UART is IO0/IO1 @115200\r\n");
    OpenOPL1000_EarlyLog("[OpenOPL1000] OpenBeken platform port\r\n");

    memset(&threadDef, 0, sizeof(threadDef));
    threadDef.name = "openbeken";
    threadDef.stacksize = OS_TASK_STACK_SIZE_APP * 2;
    threadDef.tpriority = OS_TASK_PRIORITY_APP;
    threadDef.instances = 0;
    threadDef.pthread = OpenOPL1000_OpenBekenTask;

    if (osThreadCreate(&threadDef, NULL) == NULL)
    {
        OpenOPL1000_EarlyLog("[OpenOPL1000] failed to create OpenBeken task\r\n");
    }
}
