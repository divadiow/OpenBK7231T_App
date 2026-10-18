#include "sv6x66_platform.h"
#include "flash.h"
#include "hsuart/drv_hsuart.h"
#include "uart/drv_uart.h"
#include "gpio/drv_gpio.h"
#include "pinmux/drv_pinmux.h"
#include "wdt/drv_wdt.h"
#include "sys/xip.h"

extern void Main_Init(void);
extern void Main_OnEverySecond(void);

static void application(void *unused) {
	uint32_t deadline;
	(void)unused;
	/* First task context: critical-section APIs and SDK SRAM are initialized.
		* Identification occurs before factory staging, filesystem or radio. */
	SV6X66_FlashIdentify();
	SV6X66_FactoryWindow();
	if (!SV6X66_ProfileValid()) {
		SV6X66_Printf("CKW04 selector/profile mismatch; refusing application startup\r\n");
		for (;;)
			OS_MsDelay(1000);
	}
	drv_hsuart_set_format(115200, HSUART_WLS_8, HSUART_STB_1, HSUART_PARITY_DISABLE);
	SV6X66_StorageInit();
	SV6X66_WifiInit();
	Main_Init();
	deadline = OS_GetSysTick();
	for (;;) {
		SV6X66_WifiPoll();
		if ((int32_t)(OS_GetSysTick() - deadline) >= 0) {
			deadline += OS_MS2TICK(1000);
			Main_OnEverySecond();
		}
		OS_MsDelay(50);
	}
}
void APP_Init(void) {
	xip_init();
	xip_enter();
	drv_uart_init();
	drv_uart_sw_rst();
	drv_uart_set_fifo(UART_INT_RXFIFO_TRGLVL_1, 0);
	if (drv_gpio_set_mode(UART0_TXD_I, PIN_MODE_GPIO) ||
		drv_gpio_set_mode(UART0_RXD_I, PIN_MODE_GPIO) ||
		drv_pinmux_manual_function_select_disable(SEL_UART0_II) ||
		drv_pinmux_manual_function_select_enable(SEL_UART0_I) ||
		SV6X66_LogUartSetBaud())
		for (;;) {
		}
	drv_wdt_init();
	drv_wdt_disable(MCU_WDT);
	drv_wdt_disable(SYS_WDT);
	/* Product factory UART is HSUART GPIO6RX/7TX, distinct from SDK UART0
		* debug console GPIO21/22. Keep both buses separate. */
	drv_pinmux_manual_function_select_enable(SEL_UART1);
	drv_hsuart_init();
	drv_hsuart_sw_rst();
	drv_hsuart_set_fifo(HSUART_INT_RX_FIFO_TRIG_LV_1);
	if (drv_hsuart_set_format(9600, HSUART_WLS_8, HSUART_STB_1, HSUART_PARITY_DISABLE))
		for (;;) {
		}
	OS_Init();
	OS_StatInit();
	if (!OS_MemInit())
		for (;;) {
		}
	SV6X66_FlashInit();
	if (rtos_create_thread(NULL, 2, "OpenSV6X66", application, 8192, NULL))
		for (;;) {
		}
	OS_StartScheduler();
	for (;;) {
	}
}
int lowpower_sleep_gpio_hook(void) {
	return 0;
}
int lowpower_dormant_gpio_hook(void) {
	return 0;
}
void lowpower_pre_sleep_hook(void) {
}
void lowpower_post_sleep_hook(void) {
}
void vAssertCalled(const char *function, int line) {
	SV6X66_Printf("RTOS assertion %s:%d\r\n", function, line);
	for (;;) {
	}
}
