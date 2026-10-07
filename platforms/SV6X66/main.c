#include "new_common.h"
#include "ota_stage.h"
#include "identity.h"
#include "cJSON/cJSON.h"
#include "sys/backtrace.h"
#include "sys/xip.h"
#include "sys/sys_clock.h"
#include "fsal.h"
#include "stock_mount.h"
#include "netstack.h"
#include "wifi_api.h"
#include "idmanage/pbuf.h"
#include "security/drv_security.h"
#include "uart/drv_uart.h"
#include "rf/rf_api.h"

SSV_FS fs_handle;
extern struct st_rf_table ssv_rf_table;
void SV6X66_StorageInit(void);
int SV6X66_OTAStartup(void);
void load_rf_table_from_flash(void);
static void obk_task(void *arg)
{
    if (fs_handle && SV6X66_OTAStartup()) printf("OTA marker could not be disarmed\n");
    if (!SV6X66_IdentityInit()) {
        printf("OpenSV6166F: no valid efuse MAC pair; radio startup stopped\n");
        OS_TaskDelete(NULL);
        return;
    }
    WIFI_INIT();
    netstack_init(NULL);
    configASSERT(SV6X66_NetInit());
    DUT_wifi_start(DUT_STA);
    Main_Init();
    for (;;) {
        OS_MsDelay(1000);
        Main_OnEverySecond();
    }
}
void APP_Init(void)
{
    xip_init();
    xip_enter();
    drv_uart_init();
    drv_uart_set_fifo(UART_INT_RXFIFO_TRGLVL_1, 0);
    drv_uart_set_format(115200, UART_WORD_LEN_8, UART_STOP_BIT_1, UART_PARITY_DISABLE);
    printf("OpenSV6166F: UART ready, bus=%lu Hz\n", (unsigned long)sys_bus_clock());
    OS_Init();
    OS_StatInit();
    OS_MemInit();
#if !defined(SV6X66_STOCK_CKW04)
    OS_PsramInit();
#endif
    cJSON_Hooks json_hooks = { OS_MemAlloc, OS_MemFree };
    cJSON_InitHooks(&json_hooks);
#if defined(SV6X66_STOCK_CKW04)
    // Factory RF layout differs. Use SDK defaults without touching factory data.
    build_default_rf_table(&ssv_rf_table);
#else
    load_rf_table_from_flash();
    if (ssv_rf_table.boot_flag == 0xFF) {
        build_default_rf_table(&ssv_rf_table);
    }
#endif
    load_rf_table_to_mac(&ssv_rf_table);
    // FS_init can format; allow it only with the installed OpenBeken layout.
#if defined(SV6X66_STOCK_CKW04)
    if (SV6X66_StockHeaderValid((const uint8_t *)0x30008000u)) fs_handle = SV6X66_StockMount();
    if (!fs_handle) printf("Stock filesystem unavailable; preserved without formatting\n");
#else
    if (ota_stage_layout_valid((const uint8_t *)0x30000000u)) fs_handle = FS_init();
    else printf("OpenBeken flash layout mismatch; storage disabled\n");
#endif
    printf("OpenSV6166F: storage %s\n", fs_handle ? "mounted" : "unavailable");
    SV6X66_StorageInit();
    // OS_TaskCreate uses a boolean result: 1 succeeds, 0 fails.
    if (OS_TaskCreate(obk_task, "OpenBeken", 2048, NULL, OS_TASK_PRIO1, NULL) == 0) {
        printf("OpenBeken task creation failed\n");
        for (;;) {}
    }
    OS_StartScheduler();
}
int lowpower_sleep_gpio_hook(void) { return 0; }
int lowpower_dormant_gpio_hook(void) { return 0; }
void lowpower_pre_sleep_hook(void) {}
void lowpower_post_sleep_hook(void) {}
void vAssertCalled(const char *func, int line)
{
    printf("Assert %s:%d\n", func, line);
    print_callstack();
    for (;;) {}
}
