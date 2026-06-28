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
#include "hal_fim_config_opl1000.h"

static E_IO01_UART_MODE s_io01UartMode;

#define OPENOPL1000_SHM_CODE __attribute__((section(".shm_text"), noinline, used, long_call))

#ifndef OPENOPL1000_BOOT_TRACE
#define OPENOPL1000_BOOT_TRACE 0
#endif

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
#if OPENOPL1000_BOOT_TRACE
static void OpenOPL1000_PrintRamLayout(void);
#endif
static void OpenOPL1000_OpenBekenTask(void *args);
static void OpenOPL1000_EarlyLog(const char *text);
static uint32_t OpenOPL1000_ShmProbeFn(uint32_t value) OPENOPL1000_SHM_CODE;

void __wrap_wpa_cli_func_init_patch(void);
void __wrap_at_func_init_patch(void);
void __wrap_Diag_PatchInit(void);
void __wrap_le_ctrl_pre_patch_init(void);

extern void Main_Init(void);
extern void Main_OnEverySecond(void);
extern uint8_t __bss_end__;
extern uint8_t __shm_region_start__;
extern uint8_t __shm_region_end__;

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
extern uint32_t g_u32IpcWifiBssInfoAddr;
extern uint32_t g_u32IpcWifiDbgParamAddr;
extern uint32_t g_u32IpcWifiStaInfoAddr;
extern uint32_t g_u32IpcPsConfAddr;

static uint32_t OpenOPL1000_ShmProbeFn(uint32_t value)
{
    /* Tiny split-M3 probe.  Keep this deliberately self-contained: no strings,
     * no SDK calls, no static locals.  If it is actually loaded at the configured SHM tail address,
     * calling it from normal patch RAM proves the second M3 image is present
     * and executable.
     */
    value ^= 0xA55A5AA5u;
    value += 0x12345678u;
    value ^= 0x0F0F0F0Fu;
    return value;
}

void __wrap_wpa_cli_func_init_patch(void)
{
    /* OpenBeken uses the SDK Wi-Fi service/driver/event APIs directly.  The
     * vendor WPA CLI command table is not needed for the product runtime and
     * costs patch-window space on OPL1000.
     */
}

void __wrap_at_func_init_patch(void)
{
    /* The OpenOPL1000 runtime keeps the debug UART under our control and does
     * not expose the vendor AT command task/parser.  The ROM UART mode helpers
     * are still available through symdefs, so the AT command table init can be
     * skipped for the OpenBeken profile.
     */
}

void __wrap_Diag_PatchInit(void)
{
    /* The SDK diagnostic command task is not part of the OpenBeken runtime.
     * Serial logging still uses the debug UART directly.
     */
}

void __wrap_le_ctrl_pre_patch_init(void)
{
    /* Probe: BLE controller patch init may be unnecessary because the
     * OpenOPL1000 profile deliberately skips LeRtosTaskCreat().
     */
}

#if OPENOPL1000_BOOT_TRACE
static void OpenOPL1000_PrintRamLayout(void)
{
    uintptr_t bssEnd = (uintptr_t)&__bss_end__;
    uintptr_t shmStart = (uintptr_t)&__shm_region_start__;
    uintptr_t shmEnd = (uintptr_t)&__shm_region_end__;
    volatile uint32_t *shmBase = (volatile uint32_t *)0x80000000u;
    volatile uint32_t *shmGuardEnd = (volatile uint32_t *)0x800003F0u;

    printf("OPL1000 RAM layout: bss_end=0x%08x iram_free_to_440000=%u shm=0x%08x..0x%08x used=%u free=%u\r\n",
           (unsigned int)bssEnd,
           (unsigned int)(0x00440000u - bssEnd),
           (unsigned int)shmStart,
           (unsigned int)shmEnd,
           (unsigned int)(shmEnd - shmStart),
           (unsigned int)(0x80004000u - shmEnd));

    printf("OPL1000 IPC dyn addrs: bss=0x%08x dbg=0x%08x sta=0x%08x ps=0x%08x\r\n",
           (unsigned int)g_u32IpcWifiBssInfoAddr,
           (unsigned int)g_u32IpcWifiDbgParamAddr,
           (unsigned int)g_u32IpcWifiStaInfoAddr,
           (unsigned int)g_u32IpcPsConfAddr);

    printf("OPL1000 SHM guard sample: 80000000=%08x %08x %08x %08x / 800003f0=%08x %08x %08x %08x\r\n",
           (unsigned int)shmBase[0],
           (unsigned int)shmBase[1],
           (unsigned int)shmBase[2],
           (unsigned int)shmBase[3],
           (unsigned int)shmGuardEnd[0],
           (unsigned int)shmGuardEnd[1],
           (unsigned int)shmGuardEnd[2],
           (unsigned int)shmGuardEnd[3]);
}
#endif

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
    OpenOPL1000_FimRegister();
}

static void Main_ServiceInitNoBle(void)
{
    T_MwOtaLayoutInfo tLayout;

#if OPENOPL1000_BOOT_TRACE
    printf("Wi-Fi services only; skipping BLE/LE task\r\n");
#endif

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
#if OPENOPL1000_BOOT_TRACE
        printf("nl_scrt_Init rc=%d\r\n", scrtRc);
#endif
    }
#if OPENOPL1000_BOOT_TRACE
    else
    {
        printf("nl_scrt_Init pointer is NULL\r\n");
    }
#endif

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

#if OPENOPL1000_BOOT_TRACE
    OpenOPL1000_EarlyLog("\r\nOpenBeken task entered\r\n");
#endif
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
#if OPENOPL1000_BOOT_TRACE
    {
        uint32_t shmProbe = OpenOPL1000_ShmProbeFn(0x00000029u);
        printf("OPL1000 split-M3 v66-log-cleanup: shm_fn=0x%08x result=0x%08x\r\n",
               (unsigned int)(uintptr_t)OpenOPL1000_ShmProbeFn,
               (unsigned int)shmProbe);
    }

    OpenOPL1000_PrintRamLayout();
#endif

    memset(&threadDef, 0, sizeof(threadDef));
    threadDef.name = "openbeken";
    threadDef.stacksize = OS_TASK_STACK_SIZE_APP * 2;
    threadDef.tpriority = OS_TASK_PRIORITY_APP;
    threadDef.instances = 0;
    threadDef.pthread = OpenOPL1000_OpenBekenTask;

    if (osThreadCreate(&threadDef, NULL) == NULL)
    {
        OpenOPL1000_EarlyLog("Failed to create OpenBeken task\r\n");
    }
}
