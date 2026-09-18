// HLW8012 aka BL0937
#include "drv_bl0937.h"
#include "../obk_config.h"
#include "../hal/hal_pins.h"
#include "../new_pins.h"

#if ENABLE_DRIVER_BL0937

//dummy
#include <math.h>
#include <limits.h>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "drv_bl_shared.h"
#include "drv_pwrCal.h"
#include "drv_uart.h"

#define DEFAULT_VOLTAGE_CAL 0.13253012048f
#define DEFAULT_CURRENT_CAL 0.0118577075f
#define DEFAULT_POWER_CAL 1.5f
#define BL0937_MIN_WINDOW_MS 500U
#define BL0937_SETTLE_MS 1000U
#define BL0937_SNAPSHOT_ATTEMPTS 4

// The simulator's tick is in milliseconds, despite configTICK_RATE_HZ being 1.
#if WINDOWS
#define BL0937_TICK_HZ (1000U / portTICK_PERIOD_MS)
#else
#define BL0937_TICK_HZ ((uint32_t)configTICK_RATE_HZ)
#endif

// Those can be set by Web page pins configurator
// The below are default values for Mycket smart socket
int GPIO_HLW_SEL = 24; // pwm4
bool g_invertSEL = false;
int GPIO_HLW_CF = 7;
int GPIO_HLW_CF1 = 8;

bool g_sel = true;
float BL0937_PMAX = 3680.0f;
float last_p = 0.0f;

/* Each naturally aligned counter has exactly one writer: its GPIO ISR.
 * Never reset a live counter from task context. Unsigned snapshot differences
 * retain pulses across delayed/abandoned reads and counter rollover.
 *
 * Use relaxed atomic word accesses where the compiler guarantees lock freedom.
 * Older single-core ARM9/RISC-V ports use native aligned volatile word accesses.
 * They must not fall back to a libatomic lock from interrupt context.
 */
static volatile uint32_t g_vc_pulses;
static volatile uint32_t g_p_pulses;

#if defined(__GCC_ATOMIC_INT_LOCK_FREE) && __GCC_ATOMIC_INT_LOCK_FREE == 2 && UINT_MAX == UINT32_MAX
#define BL0937_COUNTER_READ(p) __atomic_load_n((p), __ATOMIC_RELAXED)
#define BL0937_COUNTER_WRITE(p, v) __atomic_store_n((p), (v), __ATOMIC_RELAXED)
#define BL0937_SNAPSHOT_BARRIER() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#else
#define BL0937_COUNTER_READ(p) (*(p))
#define BL0937_COUNTER_WRITE(p, v) (*(p) = (v))
#endif

#ifndef BL0937_SNAPSHOT_BARRIER
// Fallback ports are single-core; order compiler accesses around the ISR.
#if defined(_MSC_VER)
#define BL0937_SNAPSHOT_BARRIER() _ReadWriteBarrier()
#elif defined(__GNUC__)
#define BL0937_SNAPSHOT_BARRIER() __asm__ __volatile__("" ::: "memory")
#else
#error "BL0937 requires a compiler ordering barrier for counter snapshots"
#endif
#endif

typedef struct {
	portTickType tick;
	uint32_t cf;
	uint32_t cf1;
} bl0937_snapshot_t;

static bl0937_snapshot_t powerStart;
static bool havePowerStart;
static uint32_t cf1StartCount;
static portTickType cf1StartTick;
static bool settling;
static float final_v = NAN;
static float final_c = NAN;

void HlwCf1Interrupt(int pinNum)
{
	BL0937_COUNTER_WRITE(&g_vc_pulses, BL0937_COUNTER_READ(&g_vc_pulses) + 1U);
}
void HlwCfInterrupt(int pinNum)
{
	BL0937_COUNTER_WRITE(&g_p_pulses, BL0937_COUNTER_READ(&g_p_pulses) + 1U);
}

/* Bound the timestamp/count skew to a single RTOS tick. A preemption across
 * ticks retries the snapshot, not the acquisition window. No interrupts are
 * masked and no pulses are thrown away if all attempts are interrupted.
 */
static bool BL0937_Snapshot(bl0937_snapshot_t *sample)
{
	int attempt;
	for(attempt = 0; attempt < BL0937_SNAPSHOT_ATTEMPTS; attempt++)
	{
		portTickType before = xTaskGetTickCount();
		BL0937_SNAPSHOT_BARRIER();
		sample->cf = BL0937_COUNTER_READ(&g_p_pulses);
		sample->cf1 = BL0937_COUNTER_READ(&g_vc_pulses);
		BL0937_SNAPSHOT_BARRIER();
		sample->tick = xTaskGetTickCount();
		if(before == sample->tick)
			return true;
	}
	return false;
}

static uint32_t BL0937_Elapsed(portTickType now, portTickType then)
{
	uint32_t elapsed = (uint32_t)now - (uint32_t)then;
	// Also support older 16-bit tick ports; avoid signed arithmetic on Windows.
	return sizeof(portTickType) == 2 ? (uint16_t)elapsed : elapsed;
}

static uint32_t BL0937_WindowTicks(uint32_t ms)
{
	uint32_t ticks = (ms * BL0937_TICK_HZ + 999U) / 1000U;
	return ticks == 0 ? 1 : ticks;
}

static float BL0937_PulseRate(uint32_t pulses, uint32_t elapsed)
{
	return ((float)pulses * (float)BL0937_TICK_HZ) / (float)elapsed;
}

static void BL0937_StartSettling(void)
{
	HAL_PIN_SetOutputValue(GPIO_HLW_SEL, g_sel);
	// This timestamp must be after the physical pin write, not an old snapshot.
	cf1StartTick = xTaskGetTickCount();
	settling = true;
}

commandResult_t BL0937_PowerMax(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	float maxPower;

	if(args == 0 || *args == 0)
	{
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "This command needs one argument");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	maxPower = atof(args);
	if((maxPower > 200.0) && (maxPower < 7200.0f))
	{
		BL0937_PMAX = maxPower;
		// UPDATE: now they are automatically saved
		CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_POWER_MAX, BL0937_PMAX);
		{
			char dbg[128];
			snprintf(dbg, sizeof(dbg), "PowerMax: set max to %f\n", BL0937_PMAX);
			addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, dbg);
		}
	}
	return CMD_RES_OK;
}

void BL0937_Shutdown_Pins()
{
	HAL_DetachInterrupt(GPIO_HLW_CF);
	HAL_DetachInterrupt(GPIO_HLW_CF1);
}

void BL0937_Init_Pins()
{
	int tmp;

	// if not found, this will return the already set value
	tmp = PIN_FindPinIndexForRole(IOR_BL0937_SEL_n, -1);
	if(tmp != -1)
	{
		g_invertSEL = true;
		GPIO_HLW_SEL = tmp;
	}
	else
	{
		g_invertSEL = false;
		GPIO_HLW_SEL = PIN_FindPinIndexForRole(IOR_BL0937_SEL, GPIO_HLW_SEL);
	}
	GPIO_HLW_CF = PIN_FindPinIndexForRole(IOR_BL0937_CF, GPIO_HLW_CF);
	GPIO_HLW_CF1 = PIN_FindPinIndexForRole(IOR_BL0937_CF1, GPIO_HLW_CF1);

	BL0937_PMAX = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_POWER_MAX, BL0937_PMAX);

	g_sel = true;
	HAL_PIN_Setup_Output(GPIO_HLW_SEL);
	BL0937_StartSettling();

	HAL_PIN_Setup_Input_Pullup(GPIO_HLW_CF1);
	HAL_PIN_Setup_Input_Pullup(GPIO_HLW_CF);

	final_v = NAN;
	final_c = NAN;
	last_p = 0.0f;
	PwrCal_ScaleVoltage(0.0f);
	PwrCal_ScaleCurrent(0.0f);
	PwrCal_ScalePower(0.0f);

	HAL_AttachInterrupt(GPIO_HLW_CF, INTERRUPT_FALLING, HlwCfInterrupt);
	HAL_AttachInterrupt(GPIO_HLW_CF1, INTERRUPT_FALLING, HlwCf1Interrupt);
	havePowerStart = BL0937_Snapshot(&powerStart);
}

void BL0937_Init(void)
{
	BL_Shared_Init();

	PwrCal_Init(PWR_CAL_MULTIPLY, DEFAULT_VOLTAGE_CAL, DEFAULT_CURRENT_CAL,
		DEFAULT_POWER_CAL);

	//cmddetail:{"name":"PowerMax","args":"[MaxPowerInW]",
	//cmddetail:"descr":"Sets the maximum power limit for BL measurement used to filter incorrect values",
	//cmddetail:"fn":"BL0937_PowerMax","file":"driver/drv_bl0937.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("PowerMax", BL0937_PowerMax, NULL);

	BL0937_Init_Pins();
}

static void BL0937_PublishPower(float final_p, uint32_t powerTicks)
{
	/* Preserve PowerMax policy, but never let its logging alter sample time. */
	if(final_p > BL0937_PMAX)
	{
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER,
			"Power reading: %f exceeded MAX limit: %f, Last: %f\n", final_p, BL0937_PMAX, last_p);
		final_p = last_p;
	}
	else
	{
		last_p = final_p;
	}

	// CF power remains valid during CF1 settling, including startup. Missing
	// voltage/current values are NAN, not fictitious zero measurements.
	BL_ProcessUpdateWithInterval(final_v, final_c, final_p, NAN,
		(float)powerTicks / (float)BL0937_TICK_HZ);
}

void BL0937_RunEverySecond(void)
{
	bl0937_snapshot_t sample;
	uint32_t powerTicks;
	uint32_t cf1Ticks;
	float final_p;
	int inversePin = PIN_FindPinIndexForRole(IOR_BL0937_SEL_n, -1);
	bool inverse = inversePin != -1;
	int selPin = inverse ? inversePin : PIN_FindPinIndexForRole(IOR_BL0937_SEL, GPIO_HLW_SEL);

	if(inverse != g_invertSEL || selPin != GPIO_HLW_SEL
		|| GPIO_HLW_CF != PIN_FindPinIndexForRole(IOR_BL0937_CF, GPIO_HLW_CF)
		|| GPIO_HLW_CF1 != PIN_FindPinIndexForRole(IOR_BL0937_CF1, GPIO_HLW_CF1))
	{
		// Finish the pending CF window before rebasing onto the new pin setup.
		// A failed or too-short capture is not a valid frequency measurement.
		if(havePowerStart && BL0937_Snapshot(&sample))
		{
			powerTicks = BL0937_Elapsed(sample.tick, powerStart.tick);
			if(powerTicks >= BL0937_WindowTicks(BL0937_MIN_WINDOW_MS))
			{
				final_p = PwrCal_ScalePower(BL0937_PulseRate(sample.cf - powerStart.cf, powerTicks));
				powerStart = sample;
				BL0937_PublishPower(final_p, powerTicks);
			}
		}
		BL0937_Shutdown_Pins();
		BL0937_Init_Pins();
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "BL0937 pins have changed, reset the interrupts");
		return;
	}

	if(!BL0937_Snapshot(&sample))
		return;
	if(!havePowerStart)
	{
		// Initialization could not obtain a coherent starting snapshot.
		powerStart = sample;
		havePowerStart = true;
		return;
	}

	powerTicks = BL0937_Elapsed(sample.tick, powerStart.tick);
	if(powerTicks < BL0937_WindowTicks(BL0937_MIN_WINDOW_MS))
		return; // Keep both windows and counters intact during timer catch-up.

	final_p = PwrCal_ScalePower(BL0937_PulseRate(sample.cf - powerStart.cf, powerTicks));
	powerStart = sample;

	cf1Ticks = BL0937_Elapsed(sample.tick, cf1StartTick);
	if(settling)
	{
		if(cf1Ticks >= BL0937_WindowTicks(BL0937_SETTLE_MS))
		{
			// Discard the post-SEL interval. Start a clean window in this mode.
			cf1StartCount = sample.cf1;
			cf1StartTick = sample.tick;
			settling = false;
		}
	}
	else if(cf1Ticks >= BL0937_WindowTicks(BL0937_MIN_WINDOW_MS))
	{
		float rate = BL0937_PulseRate(sample.cf1 - cf1StartCount, cf1Ticks);
		if(g_sel != g_invertSEL)
			final_v = PwrCal_ScaleVoltage(rate);
		else
			final_c = PwrCal_ScaleCurrent(rate);
		g_sel = !g_sel;
		BL0937_StartSettling();
	}

	BL0937_PublishPower(final_p, powerTicks);
}

// close ENABLE_DRIVER_BL0937
#endif

