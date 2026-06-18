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
#include "mw_ota.h"
#include "sys_init.h"
#include "sys_init_patch.h"
#include "sys_os_config.h"

static E_IO01_UART_MODE s_io01UartMode;

#define OPENOPL1000_SHM_CODE __attribute__((section("SHM_REGION"), noinline, used, long_call))

void __Patch_EntryPoint(void) __attribute__((section("ENTRY_POINT")));
void __Patch_EntryPoint(void) __attribute__((used));

static void Main_PinMuxUpdate(void);
static void Main_FlashLayoutUpdate(void);
static void Main_ServiceInitNoBle(void);
static void Main_MiscModulesInit(void);
static void Main_MiscDriverConfigSetup(void);
static void Main_AtUartDbgUartSwitch(void);
static void Main_ApsUartRxDectecConfig(void);
static void Main_ApsUartRxDectecCb(E_GpioIdx_t gpioIdx);
static void Main_AppInit_patch(void);
static void OpenOPL1000_OpenBekenTask(void *args);
static void OpenOPL1000_EarlyLog(const char *text);
static uint32_t OpenOPL1000_ShmProbeFn(uint32_t value) OPENOPL1000_SHM_CODE;

extern void Main_Init(void);
extern void Main_OnEverySecond(void);

typedef void (*T_Main_AppInit_fp)(void);
extern T_Main_AppInit_fp Main_AppInit;

/* SDK service-init hooks. Most of these symbols are ROM/SDK function-pointer
 * variables, not plain functions; declare them as callable pointers so we can
 * reproduce Sys_ServiceInit without creating the BLE/LE RTOS task.
 */
extern void (*wifi_mac_task_create)(void);
extern void lwip_task_create(void);
extern int (*do_supplicant_init)(void);
extern void (*controller_task_create)(void);
extern void (*ipc_init)(void);
extern void (*wifi_sta_info_init)(void);
extern void (*agent_init)(void);
extern void (*tracer_load)(void);
extern void (*tcpip_config_dhcp_arp_check_init)(void);
extern int (*nl_scrt_Init)(void);
extern void (*sys_cfg_init)(void);

static uint32_t OpenOPL1000_ShmProbeFn(uint32_t value)
{
    /* Tiny split-M3 probe.  Keep this deliberately self-contained: no strings,
     * no SDK calls, no static locals.  If it is actually loaded at 0x80000000,
     * calling it from normal patch RAM proves the second M3 image is present
     * and executable.
     */
    value ^= 0xA55A5AA5u;
    value += 0x12345678u;
    value ^= 0x0F0F0F0Fu;
    return value;
}

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
    Sys_ServiceInit = Main_ServiceInitNoBle;
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

static void Main_ServiceInitNoBle(void)
{
    T_MwOtaLayoutInfo tLayout;

    printf("[OpenOPL1000] Sys_ServiceInitNoBle: Wi-Fi services only; skipping BLE/LE task\r\n");

    if (wifi_mac_task_create != NULL)
    {
        wifi_mac_task_create();
    }

    lwip_task_create();

    /* WPA2-PSK association uses the SDK security/HMAC path even when BLE is
     * disabled.  The full SDK service init normally reaches this via the
     * BLE/security startup path or the HW-crypto define.  In the NoBle
     * service init we must initialise it explicitly; otherwise the supplicant
     * later fails with:
     *   [scrt_res_lock_impl] sem is null
     *   [nl_hmac_sha_1_impl] scrt_res_alloc fail
     */
    if (nl_scrt_Init != NULL)
    {
        int scrtRc = nl_scrt_Init();
        printf("[OpenOPL1000] nl_scrt_Init rc=%d\r\n", scrtRc);
    }
    else
    {
        printf("[OpenOPL1000] nl_scrt_Init pointer is NULL\r\n");
    }

    if (do_supplicant_init != NULL)
    {
        do_supplicant_init();
    }

    if (controller_task_create != NULL)
    {
        controller_task_create();
    }

    if (ipc_init != NULL)
    {
        ipc_init();
    }

    /* Intentionally do not call LeRtosTaskCreat().  This saves the BLE/LE task
     * stack/heap for the Wi-Fi-only OpenBeken bring-up path.
     */

    if (wifi_sta_info_init != NULL)
    {
        wifi_sta_info_init();
    }

    if (agent_init != NULL)
    {
        agent_init();
    }

    if (tracer_load != NULL)
    {
        tracer_load();
    }

    tLayout.ulaHeaderAddr[0] = MW_OTA_HEADER_ADDR_1;
    tLayout.ulaHeaderAddr[1] = MW_OTA_HEADER_ADDR_2;
    tLayout.ulaImageAddr[0] = MW_OTA_IMAGE_ADDR_1;
    tLayout.ulaImageAddr[1] = MW_OTA_IMAGE_ADDR_2;
    tLayout.ulImageSize = MW_OTA_IMAGE_SIZE;

    if (MwOta_Init != NULL)
    {
        MwOta_Init(&tLayout, 0);
    }

    if (tcpip_config_dhcp_arp_check_init != NULL)
    {
        tcpip_config_dhcp_arp_check_init();
    }

    /* Sys_ServiceInit_patch() normally calls sys_cfg_init() after
     * Sys_ServiceInit_impl().  Our replacement service init bypasses that
     * wrapper, so preserve the same post-service SDK configuration step.
     */
    if (sys_cfg_init != NULL)
    {
        sys_cfg_init();
    }
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

    if ((text == NULL) || (Hal_DbgUart_DataSend == NULL))
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

    {
        uint32_t shmProbe = OpenOPL1000_ShmProbeFn(0x00000029u);
        printf("[OpenOPL1000] split-M3 v29b: shm_fn=0x%08x result=0x%08x\r\n",
               (unsigned int)(uintptr_t)OpenOPL1000_ShmProbeFn,
               (unsigned int)shmProbe);
    }

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
