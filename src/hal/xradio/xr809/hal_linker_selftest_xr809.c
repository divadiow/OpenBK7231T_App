#if PLATFORM_XR809 && XR809_LINKER_SELFTEST

#include "../../../new_common.h"
#include "../../../cmnds/cmd_local.h"
#include "../../../logging/logging.h"
#include "../../hal_hwtimer.h"
#include "../../hal_ota.h"

#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/hal_wdg.h"
#include "image/flash.h"
#include "image/image.h"
#include "sys/ducc/ducc_app.h"

#define XRST_READ_SIZE              256U
#define XRST_QUICK_ITERATIONS       2000U
#define XRST_DEFAULT_ITERATIONS     20000U
#define XRST_DEFAULT_TIMER_US       1000U
#define XRST_FLASH_TASK_STACK       (768U)
#define XRST_XIP_TASK_STACK         (512U)
#define XRST_FLASH_TASK_PRIORITY    (5U)
#define XRST_XIP_TASK_PRIORITY      (6U)

typedef struct XR809LinkTestState {
	volatile uint32_t running;
	volatile uint32_t stopRequested;
	volatile uint32_t targetIterations;
	volatile uint32_t flashReads;
	volatile uint32_t flashReadFailures;
	volatile uint32_t flashMismatches;
	volatile uint32_t timerCallbacks;
	volatile uint32_t timerCanaryFailures;
	volatile uint32_t xipTaskLoops;
	volatile uint32_t allocationLoops;
	volatile uint32_t allocationFailures;
	volatile uint32_t placementFailures;
	volatile uint32_t minFreeHeap;
	volatile uint32_t activeAddress;
	volatile uint32_t referenceChecksum;
	TaskHandle_t flashTask;
	TaskHandle_t xipTask;
	int8_t hwTimer;
} XR809LinkTestState;

static XR809LinkTestState g_xrst;
static uint8_t g_xrstReference[XRST_READ_SIZE];
static uint8_t g_xrstReadBuffer[XRST_READ_SIZE];
static volatile uint32_t g_xrstCanaryA = 0x13579BDFU;
static volatile uint32_t g_xrstCanaryB = 0x2468ACE0U;

extern void TIMER0_IRQHandler(void);
extern void TIMER1_IRQHandler(void);

static uint32_t XRST_CodeAddress(const void *address)
{
	return ((uint32_t)(uintptr_t)address) & ~1U;
}

static int XRST_IsSRAMCode(const void *address)
{
	uint32_t value = XRST_CodeAddress(address);
	return value >= 0x00010000U && value < 0x00070000U;
}

static int XRST_IsXIPCode(const void *address)
{
	uint32_t value = XRST_CodeAddress(address);
	return value >= 0x10000000U && value < 0x10100000U;
}

static uint32_t XRST_Checksum(const uint8_t *data, uint32_t size)
{
	uint32_t hash = 2166136261U;
	uint32_t index;
	for (index = 0; index < size; index++) {
		hash ^= data[index];
		hash *= 16777619U;
	}
	return hash;
}

static void XRST_UpdateMinHeap(void)
{
	uint32_t freeHeap = (uint32_t)xPortGetFreeHeapSize();
	if (freeHeap < g_xrst.minFreeHeap) {
		g_xrst.minFreeHeap = freeHeap;
	}
}

/* This callback must remain in SRAM. It deliberately fires while flash XIP
 * is disabled, exercising the relocated timer IRQ path and its callback. */
void __attribute__((noinline)) XRST_TimerCallback(void *arg)
{
	(void)arg;
	g_xrst.timerCallbacks++;
	if (g_xrstCanaryA != 0x13579BDFU || g_xrstCanaryB != 0x2468ACE0U) {
		g_xrst.timerCanaryFailures++;
	}
}

/* This task is deliberately in XIP. The flash controller must suspend the
 * scheduler before XIP is disabled, so this task must never be dispatched
 * during a flash transaction. A scheduling leak will fault immediately. */
void __attribute__((section(".xip_text.xr809_linktest_task"), noinline))
XRST_XIPTask(void *arg)
{
	uint32_t state = 0x6D2B79F5U;
	(void)arg;

	while (!g_xrst.stopRequested) {
		uint32_t size;
		uint8_t *memory;
		state = state * 1664525U + 1013904223U;
		size = 32U + ((state >> 8) & 0x3FFU);
		memory = (uint8_t *)malloc(size);
		if (memory != NULL) {
			memory[0] = (uint8_t)state;
			memory[size - 1U] = (uint8_t)(state >> 8);
			if (memory[0] != (uint8_t)state ||
				memory[size - 1U] != (uint8_t)(state >> 8)) {
				g_xrst.timerCanaryFailures++;
			}
			free(memory);
			g_xrst.allocationLoops++;
		} else {
			g_xrst.allocationFailures++;
		}
		g_xrst.xipTaskLoops++;
		XRST_UpdateMinHeap();
		vTaskDelay(1);
	}

	g_xrst.xipTask = NULL;
	vTaskDelete(NULL);
}

static void XRST_StopTimer(void)
{
	int8_t timer = g_xrst.hwTimer;
	if (timer >= 0) {
		g_xrst.hwTimer = -1;
		HAL_HWTimerStop(timer);
		HAL_HWTimerDeinit(timer);
	}
}

static uint32_t XRST_TotalFailures(void)
{
	return g_xrst.flashReadFailures + g_xrst.flashMismatches +
		g_xrst.timerCanaryFailures + g_xrst.allocationFailures +
		g_xrst.placementFailures;
}

static void XRST_PrintStatus(const char *state)
{
	ADDLOG_INFO(LOG_FEATURE_CMD,
		"XRST:%s running=%u seq=%u addr=0x%x reads=%u readFail=%u mismatch=%u "
		"timer=%u xipLoops=%u alloc=%u allocFail=%u canaryFail=%u "
		"placementFail=%u minHeap=%u failures=%u",
		state,
		(unsigned int)g_xrst.running,
		(unsigned int)image_get_running_seq(),
		(unsigned int)g_xrst.activeAddress,
		(unsigned int)g_xrst.flashReads,
		(unsigned int)g_xrst.flashReadFailures,
		(unsigned int)g_xrst.flashMismatches,
		(unsigned int)g_xrst.timerCallbacks,
		(unsigned int)g_xrst.xipTaskLoops,
		(unsigned int)g_xrst.allocationLoops,
		(unsigned int)g_xrst.allocationFailures,
		(unsigned int)g_xrst.timerCanaryFailures,
		(unsigned int)g_xrst.placementFailures,
		(unsigned int)g_xrst.minFreeHeap,
		(unsigned int)XRST_TotalFailures());
}

void XRST_FlashTask(void *arg)
{
	(void)arg;
	while (!g_xrst.stopRequested &&
		(g_xrst.targetIterations == 0U ||
		 g_xrst.flashReads < g_xrst.targetIterations)) {
		int readResult = HAL_FlashRead((char *)g_xrstReadBuffer,
			XRST_READ_SIZE, g_xrst.activeAddress);
		if (readResult != (int)XRST_READ_SIZE) {
			g_xrst.flashReadFailures++;
		} else {
			uint32_t checksum = XRST_Checksum(g_xrstReadBuffer, XRST_READ_SIZE);
			if (checksum != g_xrst.referenceChecksum ||
				memcmp(g_xrstReadBuffer, g_xrstReference, XRST_READ_SIZE) != 0) {
				g_xrst.flashMismatches++;
			}
		}
		g_xrst.flashReads++;
		XRST_UpdateMinHeap();
		if ((g_xrst.flashReads % 1000U) == 0U) {
			XRST_PrintStatus("PROGRESS");
		}
		vTaskDelay(1);
	}

	g_xrst.stopRequested = 1;
	XRST_StopTimer();
	g_xrst.running = 0;
	g_xrst.flashTask = NULL;
	XRST_PrintStatus(XRST_TotalFailures() == 0U ? "PASS" : "FAIL");
	vTaskDelete(NULL);
}

static void XRST_CheckPlacementOne(const char *name, const void *address,
	int expectSRAM)
{
	int ok = expectSRAM ? XRST_IsSRAMCode(address) : XRST_IsXIPCode(address);
	ADDLOG_INFO(LOG_FEATURE_CMD, "XRST:PLACEMENT name=%s addr=0x%x expected=%s result=%s",
		name, (unsigned int)XRST_CodeAddress(address),
		expectSRAM ? "SRAM" : "XIP", ok ? "PASS" : "FAIL");
	if (!ok) {
		g_xrst.placementFailures++;
	}
}

static void XRST_CheckPlacements(void)
{
	g_xrst.placementFailures = 0;
	XRST_CheckPlacementOne("HAL_Flash_Open", HAL_Flash_Open, 1);
	XRST_CheckPlacementOne("HAL_Flash_Write", HAL_Flash_Write, 1);
	XRST_CheckPlacementOne("HAL_Flash_Erase", HAL_Flash_Erase, 1);
	XRST_CheckPlacementOne("HAL_Flashc_Xip_RawDisable", HAL_Flashc_Xip_RawDisable, 1);
	XRST_CheckPlacementOne("HAL_Flashc_Xip_RawEnable", HAL_Flashc_Xip_RawEnable, 1);
	XRST_CheckPlacementOne("flash_erase", flash_erase, 1);
	XRST_CheckPlacementOne("ducc_app_ioctl", ducc_app_ioctl, 1);
	XRST_CheckPlacementOne("TIMER0_IRQHandler", TIMER0_IRQHandler, 1);
	XRST_CheckPlacementOne("TIMER1_IRQHandler", TIMER1_IRQHandler, 1);
	XRST_CheckPlacementOne("HAL_WDG_Feed", HAL_WDG_Feed, 1);
	XRST_CheckPlacementOne("XRST_TimerCallback", XRST_TimerCallback, 1);
	XRST_CheckPlacementOne("XRST_FlashTask", XRST_FlashTask, 1);
	XRST_CheckPlacementOne("XRST_XIPTask", XRST_XIPTask, 0);
}

static int XRST_Start(uint32_t iterations, uint32_t timerUs)
{
	const image_ota_param_t *otaParam;
	image_seq_t runningSeq;
	float actualTimerUs = 0.0f;
	int readResult;

	if (g_xrst.running) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "XRST:START result=FAIL reason=already-running");
		return 0;
	}

	memset(&g_xrst, 0, sizeof(g_xrst));
	g_xrst.hwTimer = -1;
	g_xrst.targetIterations = iterations;
	g_xrst.minFreeHeap = (uint32_t)xPortGetFreeHeapSize();
	XRST_CheckPlacements();
	if (g_xrst.placementFailures != 0U) {
		XRST_PrintStatus("FAIL");
		return 0;
	}

	runningSeq = image_get_running_seq();
	otaParam = image_get_ota_param();
	if (otaParam == NULL || runningSeq >= IMAGE_SEQ_NUM) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "XRST:START result=FAIL reason=invalid-image-state");
		return 0;
	}
	g_xrst.activeAddress = otaParam->addr[runningSeq];
	readResult = HAL_FlashRead((char *)g_xrstReference,
		XRST_READ_SIZE, g_xrst.activeAddress);
	if (readResult != (int)XRST_READ_SIZE) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "XRST:START result=FAIL reason=reference-read result=%d",
			readResult);
		return 0;
	}
	g_xrst.referenceChecksum = XRST_Checksum(g_xrstReference, XRST_READ_SIZE);

	if (timerUs < 100U) {
		timerUs = 100U;
	}
	g_xrst.hwTimer = HAL_RequestHWTimer((float)timerUs, &actualTimerUs,
		XRST_TimerCallback, NULL);
	if (g_xrst.hwTimer < 0) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "XRST:START result=FAIL reason=no-hardware-timer");
		return 0;
	}
	HAL_HWTimerStart(g_xrst.hwTimer);

	g_xrst.running = 1;
	if (xTaskCreate(XRST_XIPTask, "XRST-XIP", XRST_XIP_TASK_STACK,
		NULL, XRST_XIP_TASK_PRIORITY, &g_xrst.xipTask) != pdPASS) {
		g_xrst.allocationFailures++;
		g_xrst.running = 0;
		XRST_StopTimer();
		ADDLOG_ERROR(LOG_FEATURE_CMD, "XRST:START result=FAIL reason=xip-task-create");
		return 0;
	}
	if (xTaskCreate(XRST_FlashTask, "XRST-Flash", XRST_FLASH_TASK_STACK,
		NULL, XRST_FLASH_TASK_PRIORITY, &g_xrst.flashTask) != pdPASS) {
		g_xrst.allocationFailures++;
		g_xrst.stopRequested = 1;
		g_xrst.running = 0;
		XRST_StopTimer();
		ADDLOG_ERROR(LOG_FEATURE_CMD, "XRST:START result=FAIL reason=flash-task-create");
		return 0;
	}

	ADDLOG_INFO(LOG_FEATURE_CMD,
		"XRST:START result=PASS mode=%s iterations=%u timerRequestedUs=%u "
		"timerActualUs=%u seq=%u addr=0x%x checksum=0x%x freeHeap=%u",
		iterations == 0U ? "OTA-SOAK" : "BOUNDED",
		(unsigned int)iterations,
		(unsigned int)timerUs,
		(unsigned int)actualTimerUs,
		(unsigned int)runningSeq,
		(unsigned int)g_xrst.activeAddress,
		(unsigned int)g_xrst.referenceChecksum,
		(unsigned int)g_xrst.minFreeHeap);
	return 1;
}

static commandResult_t XRST_Command(const void *context, const char *cmd,
	const char *args, int cmdFlags)
{
	const char *operation;
	uint32_t iterations;
	uint32_t timerUs;
	(void)context;
	(void)cmd;
	(void)cmdFlags;

	Tokenizer_TokenizeString(args, 0);
	operation = Tokenizer_GetArgsCount() > 0 ? Tokenizer_GetArg(0) : "status";

	if (!stricmp(operation, "status")) {
		XRST_PrintStatus("STATUS");
		return CMD_RES_OK;
	}
	if (!stricmp(operation, "placements")) {
		XRST_CheckPlacements();
		XRST_PrintStatus(g_xrst.placementFailures == 0U ? "PASS" : "FAIL");
		return g_xrst.placementFailures == 0U ? CMD_RES_OK : CMD_RES_ERROR;
	}
	if (!stricmp(operation, "stop")) {
		g_xrst.stopRequested = 1;
		ADDLOG_INFO(LOG_FEATURE_CMD, "XRST:STOP requested=1");
		return CMD_RES_OK;
	}
	if (!stricmp(operation, "quick")) {
		return XRST_Start(XRST_QUICK_ITERATIONS, XRST_DEFAULT_TIMER_US) ?
			CMD_RES_OK : CMD_RES_ERROR;
	}
	if (!stricmp(operation, "ota") || !stricmp(operation, "soak")) {
		timerUs = Tokenizer_GetArgIntegerDefault(1, XRST_DEFAULT_TIMER_US);
		return XRST_Start(0U, timerUs) ? CMD_RES_OK : CMD_RES_ERROR;
	}
	if (!stricmp(operation, "start")) {
		iterations = Tokenizer_GetArgIntegerDefault(1, XRST_DEFAULT_ITERATIONS);
		timerUs = Tokenizer_GetArgIntegerDefault(2, XRST_DEFAULT_TIMER_US);
		if (iterations == 0U) {
			iterations = XRST_DEFAULT_ITERATIONS;
		}
		return XRST_Start(iterations, timerUs) ? CMD_RES_OK : CMD_RES_ERROR;
	}

	ADDLOG_ERROR(LOG_FEATURE_CMD,
		"XRST:USAGE XR809LinkTest status|placements|quick|start [iterations] [timer_us]|ota [timer_us]|stop");
	return CMD_RES_BAD_ARGUMENT;
}

void XR809_LinkSelfTest_Init(void)
{
	memset(&g_xrst, 0, sizeof(g_xrst));
	g_xrst.hwTimer = -1;
	g_xrst.minFreeHeap = (uint32_t)xPortGetFreeHeapSize();
	CMD_RegisterCommand("XR809LinkTest", XRST_Command, NULL);
	ADDLOG_INFO(LOG_FEATURE_CMD,
		"XRST:READY command=XR809LinkTest warning=diagnostic-build-only freeHeap=%u",
		(unsigned int)g_xrst.minFreeHeap);
}

#endif /* PLATFORM_XR809 && XR809_LINKER_SELFTEST */
