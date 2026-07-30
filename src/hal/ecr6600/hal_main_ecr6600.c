#if PLATFORM_ECR6600

#include "oshal.h"
#include "cli.h"
#include "flash.h"
#include "sdk_version.h"
#include "psm_system.h"
#include "pmu_reg.h"
#include "chip_memmap.h"

#define DFE_VAR
#include "tx_power_config.h"

extern void vTaskStartScheduler(void);
extern void wifi_main();
extern void rf_platform_int();
extern void ke_init(void);
extern void amt_cal_info_obtain();
extern void wifi_conn_main();
extern int version_get(void);

extern void Main_Init();
extern void Main_OnEverySecond();

E_DRV_UART_NUM uart_num = E_UART_NUM_2;
TaskHandle_t g_sys_task_handle1;
extern uint8_t wmac[6];

#define ECR6600_BOOT_RTC_CNT_REG       (MEM_BASE_RTC + 0x04U)
#define ECR6600_BOOT_RTC_ALARM_REG     (MEM_BASE_RTC + 0x14U)
#define ECR6600_BOOT_RTC_CTRL_REG      (MEM_BASE_RTC + 0x18U)
#define ECR6600_BOOT_RTC_STATUS_REG    (MEM_BASE_RTC + 0x1CU)

typedef struct {
	unsigned int wake;
	unsigned int ena;
	unsigned int status;
	unsigned int fsm;
	unsigned int rtcCnt;
	unsigned int rtcAlarm;
	unsigned int rtcCtrl;
	unsigned int rtcStatus;
} ECR6600_SleepRegisterSnapshot;

static unsigned int ECR6600_BootReadRegister(unsigned int address)
{
	return *(volatile unsigned int*)address;
}

static void ECR6600_CaptureSleepRegisters(ECR6600_SleepRegisterSnapshot* snapshot)
{
	snapshot->wake = ECR6600_BootReadRegister(PCU_WAKEUP_CTRL_REG);
	snapshot->ena = ECR6600_BootReadRegister(PCU_INT_ENA_CTRL_REG);
	snapshot->status = ECR6600_BootReadRegister(PCU_INT_STATUS_CTRL_REG);
	snapshot->fsm = ECR6600_BootReadRegister(PCU_FSM_STATE_CTRL_REG);
	snapshot->rtcCnt = ECR6600_BootReadRegister(ECR6600_BOOT_RTC_CNT_REG);
	snapshot->rtcAlarm = ECR6600_BootReadRegister(ECR6600_BOOT_RTC_ALARM_REG);
	snapshot->rtcCtrl = ECR6600_BootReadRegister(ECR6600_BOOT_RTC_CTRL_REG);
	snapshot->rtcStatus = ECR6600_BootReadRegister(ECR6600_BOOT_RTC_STATUS_REG);
}

static void ECR6600_PrintSleepRegisters(
	const char* stage,
	const ECR6600_SleepRegisterSnapshot* snapshot)
{
	printf(
		"ECRSLPBOOT:%s wake=%08X ena=%08X status=%08X fsm=%08X "
		"rtc_cnt=%08X rtc_alarm=%08X rtc_ctrl=%08X rtc_st=%08X\n",
		stage,
		snapshot->wake,
		snapshot->ena,
		snapshot->status,
		snapshot->fsm,
		snapshot->rtcCnt,
		snapshot->rtcAlarm,
		snapshot->rtcCtrl,
		snapshot->rtcStatus);
}

static void ECR6600_CaptureAndPrintSleepRegisters(const char* stage)
{
	ECR6600_SleepRegisterSnapshot snapshot;
	ECR6600_CaptureSleepRegisters(&snapshot);
	ECR6600_PrintSleepRegisters(stage, &snapshot);
}

int _close_r()
{
	return 0;
}

void sys_task1(void* pvParameters)
{
	while(!wifi_is_ready())
	{
		sys_delay_ms(20);
		printf("wifi not ready!\n");
	}

	Main_Init();
	while(true)
	{
		sys_delay_ms(1000);
		Main_OnEverySecond();
	}
}

#undef os_malloc
void* os_malloc(size_t size)
{
	return pvPortMalloc(size);
}

int main(void)
{
	ECR6600_SleepRegisterSnapshot bootEntrySnapshot;

	// Capture before any SDK initialisation can acknowledge or rewrite the
	// wake source. UART is initialised immediately afterwards, then the saved
	// values are printed.
	ECR6600_CaptureSleepRegisters(&bootEntrySnapshot);
	component_cli_init(E_UART_NUM_2);
	ECR6600_PrintSleepRegisters("entry", &bootEntrySnapshot);

	printf("SDK version %s, Release version %s\n",
			  sdk_version, RELEASE_VERSION);

	rf_platform_int();
	ECR6600_CaptureAndPrintSleepRegisters("post-rf-init");

	int pinit = partion_init();
	if (pinit != 0)
	{
		printf("partition error %i\n", pinit);
	}
	if (easyflash_init() != 0)
	{
		printf("easyflash error\n");
	}

	ke_init();
	//rf_pta_config();

	drv_adc_init();
	get_volt_calibrate_data();
	time_check_temp();

	// disabling this allowed connecting to ap with -91 rssi
	amt_cal_info_obtain();

	wifi_main();

	time_check_temp();
	os_task_create("time_check_temp", SYSTEM_EVENT_LOOP_PRIORITY, 4096, time_check_temp, NULL);

	// disabling this will crash when getting ip
	ECR6600_CaptureAndPrintSleepRegisters("pre-psm-init");
	psm_wifi_ble_init();
	ECR6600_CaptureAndPrintSleepRegisters("post-psm-init");

	psm_boot_flag_dbg_op(true, 1);
	extern volatile int rtc_task_handle;
	extern void calculate_rtc_task();
	rtc_task_handle = os_task_create("calculate_rtc_task", SYSTEM_EVENT_LOOP_PRIORITY, 4096, calculate_rtc_task, NULL);
	if(rtc_task_handle)
	{
		os_printf(LM_OS, LL_INFO, "rtc calculate start!\r\n");
	}
	extern int health_mon_create_by_nv();
	health_mon_create_by_nv();
	//psm_wifi_ps_to_active();
	//psm_sleep_mode_ena_op(true, 0);
	//psm_set_psm_enable(0);
	//psm_pwr_mgt_ctrl(0);
	//psm_sleep_mode_ena_op(true, 0);
	//PSM_SLEEP_CLEAR(MODEM_SLEEP);
	//PSM_SLEEP_CLEAR(WFI_SLEEP);
	//psm_set_device_status(PSM_DEVICE_WIFI_STA, PSM_DEVICE_STATUS_ACTIVE);
	//psm_set_normal();
	//psm_set_sleep_mode(0, 0);
	//psm_boot_flag_dbg_op(true, 1);
	//AmtRfStart(1, "1");
	//int txgf = AmtRfGetTxGainFlag();
	//int txg = AmtRfGetTxGain();
	//printf("\r\nAmtRfGetTxGainFlag:%i AmtRfGetTxGain: %i\r\n", txgf, txg);
	// max txpower reported in console is 137 - 13.7dbm.
	//AmtRfSetApcIndex(1, "137");
	//uint8_t str[240];
	//amt_get_env_blob("txPower", &str, 240, NULL);
	//printf("txPower: %.*s\r\n", 240, str);
	//for(int i = 0; i < 240; i += 2)
	//{
	//	str[i] = 0xFF;
	//	str[i + 1] = 0x89;
	//}
	//amt_set_env_blob("txPower", str, 240);
	// efuse mac is not burnt on wg236p (or probably on anything, since writing it is unsupported in console)
	//drv_efuse_read(0x18, (unsigned int*)wmac, 6);

	xTaskCreate(
		sys_task1,
		"OpenBeken",
		8 * 256,
		NULL,
		6,
		&g_sys_task_handle1);

	vTaskStartScheduler();
	return 0;
}

#endif
