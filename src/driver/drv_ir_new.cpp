
#include "../obk_config.h"

#if ENABLE_DRIVER_IRREMOTEESP
// drv_ir_new.cpp - IRremoteESP8266

extern "C" {
	// these cause error: conflicting declaration of 'int bk_wlan_mcu_suppress_and_sleep(unsigned int)' with 'C' linkage
#include "../new_common.h"

#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "../obk_config.h"
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../hal/hal_generic.h"
#include "../hal/hal_hwtimer.h"

#if PLATFORM_BEKEN
#include "include.h"
#include "arm_arch.h"
#elif PLATFORM_LN882H || PLATFORM_LN8825
#define delay_ms OS_MsDelay
#elif PLATFORM_RTL8710B
	int __wrap_atoi(const char* str);
	char* _strncpy(char* dest, const char* src, size_t count);
	int _sscanf_patch(const char* buf, const char* fmt, ...);
//#undef sscanf
#endif

// why can;t I call this?
#include "../mqtt/new_mqtt.h"

	unsigned long ir_counter = 0;
	uint8_t gEnableIRSendWhilstReceive = 0;
	uint32_t gIRProtocolEnable = 0xFFFFFFFF;
	// 0 = normal PWM polarity, 1 = inverted PWM polarity.
	uint8_t gIRPinPolarity = 0;

	extern int my_strnicmp(const char* a, const char* b, int len);
	extern unsigned int g_timeMs;
#if ENABLE_DRIVER_TINYIR_NEC
	int TinyIR_NEC_IsReady(void);
#endif
}


#include "drv_ir.h"

//#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE 1
#undef read
#undef write
#undef send
//#define PROGMEM


//#define NO_LED_FEEDBACK_CODE 1

//typedef unsigned char uint_fast8_t;
typedef unsigned short uint16_t;

#define __FlashStringHelper char

// IRremoteESP8266 normally measures time spent in blocking mark()/space()
// calls. OpenBeken queues them, so expose queued duration while building a
// transaction.
static bool gIRUseVirtualMicros = false;
static uint32_t gIRVirtualMicros = 0;

static void IR_AdvanceVirtualMicros(const uint32_t usec) {
	if (gIRUseVirtualMicros) gIRVirtualMicros += usec;
}

#if defined(__GNUC__)
#define IR_COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")
#else
#define IR_COMPILER_BARRIER() do { } while (0)
#endif

// dummy functions
#if PLATFORM_BEKEN
void noInterrupts() { }
void interrupts() { }
void delay(int n) { }
void delayMicroseconds(int n) { }
unsigned long millis()
{
	if (gIRUseVirtualMicros) return gIRVirtualMicros / 1000;
	return 0;
}
unsigned long micros()
{
	if (gIRUseVirtualMicros) return gIRVirtualMicros;
	return 0;
}
#else
void noInterrupts() { taskENTER_CRITICAL(); }
void interrupts() { taskEXIT_CRITICAL(); }
void delay(int n) { delay_ms(n); }
void delayMicroseconds(int n) { HAL_Delay_us(n); }
unsigned long millis()
{
	if (gIRUseVirtualMicros) return gIRVirtualMicros / 1000;
	return g_timeMs;
}
unsigned long micros()
{
	if (gIRUseVirtualMicros) return gIRVirtualMicros;
	return g_timeMs * 1000;
}
#endif


class Print {
public:
	void println(const char *p) {
		return;
	}
	void print(...) {
		return;
	}
};

Print Serial;




#define EXTERNAL_IR_TIMER_ISR

//////////////////////////////////////////
// our external timer interrupt stuff
// this will have already been done
#define TIMER_RESET_INTR_PENDING


// #  if defined(ISR)
// #undef ISR
// #  endif
// #define ISR void IR_ISR

// THIS function is defined in src/libraries/IRremoteESP8266/src/IRrecv.cpp
extern "C" void DRV_IR_ISR(void* arg);
extern void IR_ISR(float period_us);
extern void IR_ISR_ResetClock(float period_us);

static int8_t ir_chan = -1;
static float ir_periodus = 50.0f;
static uint32_t ir_periodus_rounded = 50U;
static bool gIRDriverReady = false;
static bool gIRDeferredStart = false;

void timerConfigForReceive() {
	// OpenBeken owns the external timer.
}

static bool _timerConfigForReceive() {
	ir_counter = 0;
	ir_periodus = 50.0f;
	ir_periodus_rounded = 50U;

	ir_chan = HAL_RequestHWTimer(ir_periodus, &ir_periodus, DRV_IR_ISR, NULL);
	if (ir_chan < 0) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR hardware timer allocation failed");
		return false;
	}

	// Some timer HALs start the channel during allocation. Stop it until all
	// sender/receiver state has been initialised and published.
	HAL_HWTimerStop(ir_chan);
	ir_periodus_rounded = (uint32_t)(ir_periodus + 0.5f);
	if (!ir_periodus_rounded) ir_periodus_rounded = 1U;
	IR_ISR_ResetClock(ir_periodus);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer %d, %.2f us period",
		(int)ir_chan, ir_periodus);
	return true;
}

static void timer_enable() {
}
static void timer_disable() {
}
static void _timer_enable() {
	if (ir_chan < 0) return;
	HAL_HWTimerStart(ir_chan);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer enabled %d", (int)ir_chan);
}
static void _timer_disable() {
	if (ir_chan < 0) return;
	HAL_HWTimerStop(ir_chan);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer disabled %d", (int)ir_chan);
}

#define TIMER_ENABLE_RECEIVE_INTR timer_enable();
#define TIMER_DISABLE_RECEIVE_INTR timer_disable();

//////////////////////////////////////////

class SpoofIrReceiver {
public:
	static void restartAfterSend() {

	}
};

SpoofIrReceiver IrReceiver;

#include "../libraries/IRremoteESP8266/src/IRremoteESP8266.h"
#include "../libraries/IRremoteESP8266/src/IRsend.h"
#include "../libraries/IRremoteESP8266/src/IRrecv.h"
#include "../libraries/IRremoteESP8266/src/IRutils.h"
#ifdef ENABLE_IRAC
#include "../libraries/IRremoteESP8266/src/IRac.h"
#endif
#include "../libraries/IRremoteESP8266/src/IRproto.h"
#include "../libraries/IRremoteESP8266/src/digitalWriteFast.h"
#if SEND_ARGO
#include "../libraries/IRremoteESP8266/src/ir_Argo.h"
#endif

// OpenBeken's receiver adaptation keeps its capture state in IRrecv.cpp.
// Access it here so driver lifecycle and allocation failures can be handled
// without changing generic timer or GPIO behaviour.
namespace _IRrecv {
extern atomic_irparams_t params;
extern irparams_t *params_save;
}

// Declared here because lifecycle helpers precede the owning definition below.
extern IRrecv *ourReceiver;

static bool IR_ReceiverStorageReady(void) {
	return _IRrecv::params.rawbuf != NULL && _IRrecv::params.bufsize != 0;
}

static void IR_ClearReceiverStorageState(void) {
	_IRrecv::params.rcvstate = kStopState;
	_IRrecv::params.rawlen = 0;
	_IRrecv::params.overflow = false;
	_IRrecv::params.bufsize = 0;
	_IRrecv::params.rawbuf = NULL;
	_IRrecv::params_save = NULL;
}

// Synchronize the polling receiver with the actual idle pin level. The single
// direct IR_ISR() call updates the private previous-level state in IRrecv.cpp;
// the synthetic capture entry, if any, is then discarded. Only the IR-owned
// timer is stopped/restarted.
static bool IR_SyncReceiverInput(const bool restartTimer) {
	if (!ourReceiver || !IR_ReceiverStorageReady() || ir_chan < 0) return false;
	HAL_HWTimerStop(ir_chan);
	IR_ISR_ResetClock(ir_periodus);
	_IRrecv::params.rcvstate = kIdleState;
	_IRrecv::params.rawlen = 0;
	_IRrecv::params.overflow = false;
	IR_ISR(ir_periodus);
	_IRrecv::params.rcvstate = kIdleState;
	_IRrecv::params.rawlen = 0;
	_IRrecv::params.overflow = false;
	// Keep the newly sampled pin level, but restart duration accounting from
	// this synchronized baseline rather than from the synthetic sample above.
	IR_ISR_ResetClock(ir_periodus);
	if (restartTimer) HAL_HWTimerStart(ir_chan);
	return true;
}

// override aspects of sending for our own interrupt driven sends
// basically, IRsend calls mark(us) and space(us) to send.
// Build a complete waveform in a private transaction first, then expose it to
// the 50us ISR in one step. This prevents the ISR from transmitting a partial
// frame when a protocol exceeds the fixed queue capacity.
// Queue size for the interrupt-driven IR send buffer.
// Each IR frame is encoded as a sequence of mark/space durations; each
// duration occupies one slot.  A rough upper bound is 2 * bits + overhead:
//   - NEC 32-bit:         ~68 slots
//   - Fujitsu AC 128-bit: ~270 slots
//   - Daikin 312-bit:     ~640 slots
// 1024 slots gives comfortable headroom for one longest known protocol frame.
#define SEND_QUEUE_ITEMS 1024

#if PLATFORM_BL602 || PLATFORM_LN882H || PLATFORM_LN8825 || PLATFORM_REALTEK
extern "C" bool HAL_IR_PWM_IsActive(int index);
extern "C" void HAL_IR_PWM_Update(int index, float value);
#endif
#if PLATFORM_BL602 || PLATFORM_REALTEK
extern "C" bool HAL_IR_PWM_Reserve(int index);
extern "C" void HAL_IR_PWM_Release(int index);
#endif

static bool IR_PlatformPWMReserve(const int index) {
#if PLATFORM_BL602 || PLATFORM_REALTEK
	return HAL_IR_PWM_Reserve(index);
#else
	return index >= 0;
#endif
}

static void IR_PlatformPWMRelease(const int index) {
#if PLATFORM_BL602 || PLATFORM_REALTEK
	HAL_IR_PWM_Release(index);
#else
	(void)index;
#endif
}

static bool IR_PlatformPWMIsActive(const int index) {
#if PLATFORM_BL602 || PLATFORM_LN882H || PLATFORM_LN8825 || PLATFORM_REALTEK
	return HAL_IR_PWM_IsActive(index);
#else
	return index >= 0 && HAL_PIN_CanThisPinBePWM(index);
#endif
}

static void IR_PlatformPWMUpdate(const int index, const float duty) {
#if PLATFORM_BL602 || PLATFORM_LN882H || PLATFORM_LN8825 || PLATFORM_REALTEK
	HAL_IR_PWM_Update(index, duty);
#else
	HAL_PIN_PWM_Update(index, duty);
#endif
}

class myIRsend : public IRsend {
public:
	myIRsend(uint_fast8_t aSendPin) : IRsend(aSendPin) {
		sendPin = aSendPin;
		our_us = 0;
		our_ms = 0;
		pwmfrequency = 38000;
		pwmduty = 50;
		pwmInverted = gIRPinPolarity ? 1U : 0U;
		transactionFrequencyHz = pwmfrequency;
		transactionDuty = (uint8_t)pwmduty;
		transactionInverted = pwmInverted;
		carrierStarted = 0;
		carrierReleasePending = 0;
		lastCarrierMark = -1;
		lastCarrierDuty = -1;
		resetsendqueue();
	}
	~myIRsend() { }

	uint32_t millis() {
		return our_ms;
	}

	bool beginSendTransaction() {
		if (isBusy()) return false;
		transactionCount = 0;
		transactionRepeatCount = 0;
		transactionFailed = false;
		transactionFrequencyHz = pwmfrequency;
		transactionDuty = (uint8_t)pwmduty;
		transactionInverted = pwmInverted;
		overflows = 0;
		transactionBuilding = 1;
		gIRVirtualMicros = 0;
		gIRUseVirtualMicros = true;
		return true;
	}

	bool commitSendTransaction() {
		gIRUseVirtualMicros = false;
		if (!transactionBuilding || transactionFailed || transactionCount == 0) {
			abortSendTransaction();
			return false;
		}

		// Publish all queue metadata and carrier settings before exposing the
		// transaction to the timer ISR. The caller stops the IR-owned timer while
		// this method runs.
		timeout = 0;
		timein = transactionCount;
		transactionRepeatsRemaining = transactionRepeatCount;
		const uint32_t totalItems =
			(uint32_t)transactionCount * (transactionRepeatCount + 1U);
		timecount = (uint16_t)totalItems;
		timecounttotal = totalItems;
		pwmfrequency = transactionFrequencyHz;
		pwmduty = transactionDuty;
		pwmInverted = transactionInverted;

		if (!IR_PlatformPWMReserve(sendPin)) {
			abortSendTransaction();
			return false;
		}
		HAL_PIN_PWM_Start(sendPin, (int)pwmfrequency);
		if (!IR_PlatformPWMIsActive(sendPin)) {
			HAL_PIN_PWM_Stop(sendPin);
			IR_PlatformPWMRelease(sendPin);
			abortSendTransaction();
			return false;
		}
		carrierStarted = 1;
		carrierReleasePending = 0;
		lastCarrierMark = -1;
		lastCarrierDuty = -1;
		applyCarrierState(false, true);

		IR_COMPILER_BARRIER();
		transactionReady = 1;
		transactionBuilding = 0;
		transactionRepeatCount = 0;
		return true;
	}

	void abortSendTransaction() {
		gIRUseVirtualMicros = false;
		transactionReady = 0;
		IR_COMPILER_BARRIER();
		transactionBuilding = 0;
		transactionFailed = false;
		transactionCount = 0;
		transactionRepeatCount = 0;
		transactionRepeatsRemaining = 0;
		timein = timeout = 0;
		timecount = 0;
		currentsendtime = 0;
		currentbitval = 0;
		timecounttotal = 0;
		overflows = 0;
	}

	bool isBusy() const {
		return transactionBuilding || transactionReady || currentsendtime ||
			carrierReleasePending;
	}

	uint16_t getStagedItemCount() const {
		return transactionCount;
	}

	bool setTransactionRepeats(const uint16_t repeats) {
		if (!transactionBuilding || transactionFailed) return false;
		transactionRepeatCount = repeats;
		return true;
	}

	bool setInverted(const bool inverted) {
		if (isBusy()) return false;
		pwmInverted = inverted ? 1U : 0U;
		transactionInverted = pwmInverted;
		lastCarrierMark = -1;
		lastCarrierDuty = -1;
		HAL_PIN_Setup_Output(sendPin);
		HAL_PIN_SetOutputValue(sendPin, pwmInverted ? 1 : 0);
		return true;
	}

	void applyCarrierState(const bool mark, const bool force = false) {
		if (!carrierStarted) return;
		int duty;
		if (mark) {
			duty = pwmInverted ? 100 - (int)pwmduty : (int)pwmduty;
		} else {
			duty = pwmInverted ? 100 : 0;
		}
		if (duty < 0) duty = 0;
		if (duty > 100) duty = 100;
		if (!force && lastCarrierMark == (mark ? 1 : 0) &&
			lastCarrierDuty == duty) return;
		IR_PlatformPWMUpdate(sendPin, (float)duty);
		lastCarrierMark = mark ? 1 : 0;
		lastCarrierDuty = (int16_t)duty;
	}

	bool hasCarrierServicePending() const {
		return carrierReleasePending != 0;
	}

	void serviceCarrier() {
		if (!carrierReleasePending || transactionReady || currentsendtime) return;
		carrierReleasePending = 0;
		if (carrierStarted) {
			applyCarrierState(false, true);
			HAL_PIN_PWM_Stop(sendPin);
			IR_PlatformPWMRelease(sendPin);
			carrierStarted = 0;
		}
		HAL_PIN_Setup_Output(sendPin);
		HAL_PIN_SetOutputValue(sendPin, pwmInverted ? 1 : 0);
		lastCarrierMark = -1;
		lastCarrierDuty = -1;
	}

	void cancelActiveSend() {
		gIRUseVirtualMicros = false;
		transactionReady = 0;
		IR_COMPILER_BARRIER();
		transactionBuilding = 0;
		transactionFailed = false;
		transactionCount = 0;
		transactionRepeatCount = 0;
		transactionRepeatsRemaining = 0;
		timein = timeout = 0;
		timecount = 0;
		overflows = 0;
		currentsendtime = 0;
		currentbitval = 0;
		timecounttotal = 0;
		carrierReleasePending = 0;
		if (carrierStarted) {
			applyCarrierState(false, true);
			HAL_PIN_PWM_Stop(sendPin);
			IR_PlatformPWMRelease(sendPin);
			carrierStarted = 0;
		}
		HAL_PIN_Setup_Output(sendPin);
		HAL_PIN_SetOutputValue(sendPin, pwmInverted ? 1 : 0);
		lastCarrierMark = -1;
		lastCarrierDuty = -1;
	}

	void stopAndReleaseResources() {
		cancelActiveSend();
	}

	void delay(long int ms) {
		space((uint32_t)ms * 1000U);
	}

	uint16_t mark(uint16_t aMarkMicros) {
		if (!aMarkMicros) return 0;
		IR_AdvanceVirtualMicros(aMarkMicros);
		return appendDuration((uint32_t)aMarkMicros | 0x10000000U) ? 1 : 0;
	}

	void space(uint32_t aMarkMicros) {
		if (!aMarkMicros) return;
		IR_AdvanceVirtualMicros(aMarkMicros);
		appendDuration(aMarkMicros);
	}

	void enableIROut(uint32_t freq, uint8_t duty = 50) {
		if (freq < 1000U) freq *= 1000U;
		if (duty < 1U) duty = 1U;
		if (duty > 100U) duty = 100U;

		if (transactionBuilding) {
			transactionFrequencyHz = freq;
			transactionDuty = duty;
			return;
		}
		pwmfrequency = freq;
		pwmduty = duty;
	}

	void resetsendqueue() {
		gIRUseVirtualMicros = false;
		transactionReady = 0;
		IR_COMPILER_BARRIER();
		transactionBuilding = 0;
		transactionFailed = false;
		transactionCount = 0;
		transactionRepeatCount = 0;
		transactionRepeatsRemaining = 0;
		timein = timeout = 0;
		timecount = 0;
		overflows = 0;
		currentsendtime = 0;
		currentbitval = 0;
		timecounttotal = 0;
		carrierReleasePending = 0;
	}

	bool getsendqueue(int32_t *value) {
		if (!value || !transactionReady) return false;
		if (timeout >= timein) {
			if (transactionRepeatsRemaining) {
				transactionRepeatsRemaining--;
				timeout = 0;
			} else {
				transactionReady = 0;
				IR_COMPILER_BARRIER();
				timein = timeout = 0;
				timecount = 0;
				carrierReleasePending = 1;
				return false;
			}
		}
		*value = times[timeout++];
		if (timecount) timecount--;
		return true;
	}

	int32_t times[SEND_QUEUE_ITEMS];
	volatile unsigned short timein;
	volatile unsigned short timeout;
	volatile unsigned short timecount;
	unsigned short overflows;
	uint32_t timecounttotal;
	volatile int currentsendtime;
	volatile int currentbitval;

	uint8_t sendPin;
	uint32_t pwmduty;
	uint32_t pwmfrequency;
	uint32_t transactionFrequencyHz;
	uint8_t transactionDuty;
	uint8_t pwmInverted;
	uint8_t transactionInverted;
	volatile uint8_t carrierStarted;
	volatile uint8_t carrierReleasePending;
	volatile int8_t lastCarrierMark;
	volatile int16_t lastCarrierDuty;

	uint32_t our_ms;
	uint32_t our_us;

private:
	bool appendDuration(const uint32_t duration) {
		if (!transactionBuilding || transactionFailed) return false;
		if (transactionCount >= SEND_QUEUE_ITEMS) {
			transactionFailed = true;
			overflows++;
			return false;
		}
		times[transactionCount++] = duration;
		return true;
	}

	volatile uint8_t transactionReady;
	volatile uint8_t transactionBuilding;
	bool transactionFailed;
	uint16_t transactionCount;
	uint16_t transactionRepeatCount;
	volatile uint16_t transactionRepeatsRemaining;
};

// our send/receive instances
myIRsend *pIRsend = NULL;
IRrecv *ourReceiver = NULL;

// this is our ISR.
// it is called every 50us, so we need to work on making it as efficient as possible.
extern "C" void DRV_IR_ISR(void* arg)
{
	(void)arg;
	int sending = 0;
	if (pIRsend) {
		pIRsend->our_us += ir_periodus_rounded;
		if (pIRsend->our_us >= 1000U) {
			pIRsend->our_ms += pIRsend->our_us / 1000U;
			pIRsend->our_us %= 1000U;
		}

		if (pIRsend->currentsendtime) {
			sending = 1;
			pIRsend->currentsendtime -= (int)ir_periodus_rounded;
			if (pIRsend->currentsendtime <= 0) {
				const int32_t remains = pIRsend->currentsendtime;
				int32_t newtime = 0;
				if (!pIRsend->getsendqueue(&newtime)) {
					pIRsend->currentsendtime = 0;
					pIRsend->currentbitval = 0;
				} else {
					pIRsend->currentbitval =
						(newtime & 0x10000000U) ? 1 : 0;
					pIRsend->currentsendtime = newtime & 0x0FFFFFFF;
					pIRsend->currentsendtime += remains;
				}
			}
		} else {
			int32_t newtime = 0;
			if (!pIRsend->getsendqueue(&newtime)) {
				pIRsend->currentsendtime = 0;
				pIRsend->currentbitval = 0;
			} else {
				sending = 1;
				pIRsend->currentsendtime = newtime & 0x0FFFFFFF;
				pIRsend->currentbitval =
					(newtime & 0x10000000U) ? 1 : 0;
			}
		}
		pIRsend->applyCarrierState(pIRsend->currentbitval != 0);
	}

	if (gEnableIRSendWhilstReceive) sending = 0;
	if (ourReceiver && IR_ReceiverStorageReady() && !sending) IR_ISR(ir_periodus);
	ir_counter++;
}


static int hexNibbleValue(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
	if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
	return -1;
}

static const uint16_t kIRSendMaxStateBytes = 64;
static const uint16_t kIRSendMaxBits = kIRSendMaxStateBytes * 8;
static const uint8_t kIRSendMaxRepeats = 10;

static bool parseBoundedDecimal(const char *text, const uint32_t minValue,
	const uint32_t maxValue, uint32_t *result) {
	if (!text || !text[0] || !result || minValue > maxValue) return false;
	uint32_t value = 0;
	for (const char *cursor = text; *cursor; cursor++) {
		if (*cursor < '0' || *cursor > '9') return false;
		const uint32_t digit = (uint32_t)(*cursor - '0');
		if (value > maxValue / 10U ||
			(value == maxValue / 10U && digit > maxValue % 10U)) return false;
		value = value * 10U + digit;
	}
	if (value < minValue) return false;
	*result = value;
	return true;
}

static bool parseBoundedHex(const char *text, const uint32_t minValue,
	const uint32_t maxValue, uint32_t *result) {
	if (!text || !text[0] || !result || minValue > maxValue) return false;
	if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) text += 2;
	if (!text[0]) return false;
	uint32_t value = 0;
	for (const char *cursor = text; *cursor; cursor++) {
		const int nibble = hexNibbleValue(*cursor);
		if (nibble < 0) return false;
		const uint32_t digit = (uint32_t)nibble;
		if (value > maxValue / 16U ||
			(value == maxValue / 16U && digit > maxValue % 16U)) return false;
		value = value * 16U + digit;
	}
	if (value < minValue) return false;
	*result = value;
	return true;
}

static uint8_t IR_SplitWords(char *text, char **words, const uint8_t maxWords) {
	if (!text || !words || !maxWords) return 0;
	uint8_t count = 0;
	char *cursor = text;
	while (*cursor) {
		while (*cursor == ' ' || *cursor == '\t') cursor++;
		if (!*cursor) break;
		if (count >= maxWords) return (uint8_t)(maxWords + 1U);
		words[count++] = cursor;
		while (*cursor && *cursor != ' ' && *cursor != '\t') cursor++;
		if (*cursor) *cursor++ = '\0';
	}
	return count;
}

static uint8_t IR_SplitClassicFields(char *text, char **fields,
	const uint8_t maxFields) {
	if (!text || !fields || !maxFields) return 0;
	if (!strchr(text, '-')) return IR_SplitWords(text, fields, maxFields);

	uint8_t count = 0;
	char *fieldStart = text;
	for (char *cursor = text; ; cursor++) {
		if (*cursor == ' ' || *cursor == '\t')
			return (uint8_t)(maxFields + 1U);
		if (*cursor == '-' || *cursor == '\0') {
			if (cursor == fieldStart || count >= maxFields)
				return (uint8_t)(maxFields + 1U);
			fields[count++] = fieldStart;
			if (*cursor == '\0') break;
			*cursor = '\0';
			fieldStart = cursor + 1;
		}
	}
	return count;
}

static bool IR_EqualsIgnoreCase(const char *left, const char *right) {
	if (!left || !right) return false;
	const size_t leftLength = strlen(left);
	const size_t rightLength = strlen(right);
	return leftLength == rightLength &&
		my_strnicmp(left, right, (int)leftLength) == 0;
}

static bool IR_ProtocolUsesStatePayload(const decode_type_t protocol) {
	return hasACState(protocol) || protocol == decode_type_t::MWM;
}

static bool IR_ProtocolUsesACState(const decode_type_t protocol) {
	return hasACState(protocol);
}

static decode_type_t IR_ParseProtocol(const char *text) {
	uint32_t protocol = 0;
	if (parseBoundedDecimal(text, 1U, (uint32_t)kLastDecodeType, &protocol))
		return (decode_type_t)protocol;
	for (int value = 1; value <= (int)kLastDecodeType; value++) {
		const decode_type_t candidate = (decode_type_t)value;
		if (IR_EqualsIgnoreCase(text, typeToString(candidate).c_str()))
			return candidate;
	}
	return decode_type_t::UNKNOWN;
}

// The OpenBeken transmitter advances edges on the IR-owned periodic timer.
// Reject protocols whose shortest symbols cannot be kept within their upstream
// tolerance at the actual timer period. This does not change any shared timer
// cadence; it only prevents knowingly malformed transmissions.
static bool IR_ProtocolTxTimingSupported(const decode_type_t protocol) {
	if (protocol == decode_type_t::RCMM) return ir_periodus <= 16.0f;
	if (protocol == decode_type_t::LEGOPF) return ir_periodus <= 39.0f;
	return true;
}


#if SEND_ARGO
// IRremoteESP8266 2.9.0 keeps its WREM-3 preamble constant private to
// ir_Argo.cpp. Keep the port's selector local rather than depending on a
// non-public upstream symbol.
static const uint8_t kIRArgoWrem3Preamble = 0x0BU;

static bool IR_IsArgoWrem3Payload(const uint8_t *state,
	const uint16_t nbytes) {
	if (!state || !nbytes || (state[0] & 0x0FU) != kIRArgoWrem3Preamble)
		return false;
	const uint8_t messageType = state[0] >> 6U;
	if (messageType == (uint8_t)argoIrMessageType_t::AC_CONTROL)
		return nbytes == kArgo3AcControlStateLength;
	if (messageType == (uint8_t)argoIrMessageType_t::IFEEL_TEMP_REPORT)
		return nbytes == kArgo3iFeelReportStateLength;
	if (messageType == (uint8_t)argoIrMessageType_t::TIMER_COMMAND)
		return nbytes == kArgo3TimerStateLength;
	if (messageType == (uint8_t)argoIrMessageType_t::CONFIG_PARAM_SET)
		return nbytes == kArgo3ConfigStateLength;
	return false;
}

static bool IR_IsValidArgoPayload(const uint8_t *state,
	const uint16_t nbytes) {
	if (!state || nbytes < 2U) return false;
	if ((state[0] & 0x0FU) == kIRArgoWrem3Preamble)
		return IR_IsArgoWrem3Payload(state, nbytes);
	if (nbytes != kArgoStateLength && nbytes != kArgoShortStateLength)
		return false;
	return state[0] == kArgoPreamble1 && state[1] == kArgoPreamble2;
}
#endif  // SEND_ARGO

static bool isValidStatePayloadLength(const decode_type_t protocol,
	const uint16_t bits, const uint16_t nbytes) {
	if (!nbytes || nbytes != (uint16_t)((bits + 7U) / 8U)) return false;
	if (protocol == decode_type_t::CARRIER_AC84)
		return bits == kCarrierAc84Bits;
	if (bits & 7U) return false;
	if (protocol == decode_type_t::MWM)
		return bits >= 24U && bits <= kIRSendMaxBits;
	if (protocol == decode_type_t::ARGO)
		return bits == 16U || bits == 32U || bits == 48U ||
			bits == 72U || bits == 96U;
	if (protocol == decode_type_t::CORONA_AC)
		return bits == 56U || bits == 168U;
	if (protocol == decode_type_t::DAIKIN)
		return bits == 216U || bits == 280U;
	if (protocol == decode_type_t::FUJITSU_AC)
		return bits == 48U || bits == 56U || bits == 120U || bits == 128U;
	if (protocol == decode_type_t::HITACHI_AC3)
		return bits == 120U || bits == 136U || bits == 168U ||
			bits == 184U || bits == 216U;
	if (protocol == decode_type_t::PANASONIC_AC)
		return bits == 128U || bits == 216U;
	if (protocol == decode_type_t::SAMSUNG_AC)
		return bits == 112U || bits == 168U;
	if (protocol == decode_type_t::TOSHIBA_AC)
		return bits == 56U || bits == 72U || bits == 80U;
	const uint16_t expectedBits = IRsend::defaultBits(protocol);
	return expectedBits && bits == expectedBits;
}

static bool parseHexStateBytes(const char *hexIn, uint16_t bits, uint8_t *out,
	uint16_t outSize, uint16_t *outBytes, const char **endPtr) {
	if (!hexIn || !out || !outBytes) return false;
	const char *hex = hexIn;
	if (hex[0] == '0' && (hex[1] == 'x' || hex[1] == 'X')) hex += 2;
	const uint16_t nbytes = (uint16_t)((bits + 7U) / 8U);
	if (!nbytes || nbytes > outSize) return false;
	memset(out, 0, nbytes);
	uint16_t nibbleCount = 0;
	while (hex[nibbleCount] && hex[nibbleCount] != ',') {
		if (hexNibbleValue(hex[nibbleCount]) < 0) return false;
		nibbleCount++;
	}
	if (!nibbleCount) return false;
	const uint16_t maxNibbles = nbytes * 2U;
	while (nibbleCount > maxNibbles && hex[0] == '0') {
		hex++;
		nibbleCount--;
	}
	if (nibbleCount > maxNibbles) return false;
	const uint16_t nibbleOffset = maxNibbles - nibbleCount;
	for (uint16_t index = 0; index < nibbleCount; index++) {
		const int value = hexNibbleValue(hex[index]);
		const uint16_t position = nibbleOffset + index;
		const uint16_t byteIndex = position / 2U;
		if ((position & 1U) == 0)
			out[byteIndex] |= (uint8_t)(value << 4);
		else
			out[byteIndex] |= (uint8_t)value;
	}
	const uint8_t unusedHighBits = (uint8_t)(nbytes * 8U - bits);
	if (unusedHighBits) {
		const uint8_t unusedMask =
			(uint8_t)(0xFFU << (8U - unusedHighBits));
		if (out[0] & unusedMask) return false;
	}
	*outBytes = nbytes;
	if (endPtr) *endPtr = hex + nibbleCount;
	return true;
}

static bool IR_ValidateClassicFields(const decode_type_t protocol,
	const uint32_t address, const uint32_t command) {
	if (protocol == decode_type_t::RC5)
		return address <= 0x1FU && command <= 0x3FU;
	if (protocol == decode_type_t::RC5X)
		return address <= 0x1FU && command <= 0x7FU;
	if (protocol == decode_type_t::RC6)
		return address <= 0xFFFU && command <= 0xFFU;
	if (protocol == decode_type_t::NEC)
		return address <= 0xFFFFU && command <= 0xFFU;
	if (protocol == decode_type_t::PANASONIC)
		return address <= 0xFFFFU;
	if (protocol == decode_type_t::JVC)
		return address <= 0xFFU && command <= 0xFFU;
	if (protocol == decode_type_t::SAMSUNG)
		return address <= 0xFFU && command <= 0xFFU;
	if (protocol == decode_type_t::LG)
		return address <= 0xFFU && command <= 0xFFFFU;
	return false;
}

static void IR_ServicePendingCarrier(void) {
	if (!pIRsend || !pIRsend->hasCarrierServicePending()) return;
	const bool restartTimer = gIRDriverReady && ir_chan >= 0;
	if (ir_chan >= 0) HAL_HWTimerStop(ir_chan);
	pIRsend->serviceCarrier();
	if (restartTimer) HAL_HWTimerStart(ir_chan);
}

static bool IR_CommitSendTransaction(void) {
	if (!pIRsend || ir_chan < 0) return false;
	HAL_HWTimerStop(ir_chan);
	const bool committed = pIRsend->commitSendTransaction();
	if (gIRDriverReady) HAL_HWTimerStart(ir_chan);
	return committed;
}

static commandResult_t IR_SendCommaCommand(char *args) {
	char *protocolEnd = strchr(args, ',');
	if (!protocolEnd) return CMD_RES_BAD_ARGUMENT;
	*protocolEnd = '\0';
	const decode_type_t protocol = IR_ParseProtocol(args);
	if (protocol == decode_type_t::UNKNOWN) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend unknown protocol '%s'", args);
		return CMD_RES_BAD_ARGUMENT;
	}
	if (!IR_ProtocolTxTimingSupported(protocol)) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend protocol %s is unsupported at %.2fus timer resolution",
			args, ir_periodus);
		return CMD_RES_ERROR;
	}
	char *bitsText = protocolEnd + 1;
	char *bitsEnd = strchr(bitsText, ',');
	if (!bitsEnd) return CMD_RES_BAD_ARGUMENT;
	*bitsEnd = '\0';
	uint32_t bitValue = 0;
	if (!parseBoundedDecimal(bitsText, 1, kIRSendMaxBits, &bitValue)) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend invalid bit count '%s' (expected 1-%u)",
			bitsText, (unsigned int)kIRSendMaxBits);
		return CMD_RES_BAD_ARGUMENT;
	}
	const uint16_t bits = (uint16_t)bitValue;
	uint8_t state[kIRSendMaxStateBytes];
	uint16_t nbytes = 0;
	const char *payloadEnd = NULL;
	if (!parseHexStateBytes(bitsEnd + 1, bits, state, sizeof(state),
		&nbytes, &payloadEnd)) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend invalid payload for %s (bits=%d)", args, (int)bits);
		return CMD_RES_BAD_ARGUMENT;
	}
	uint32_t repeats = 0;
	if (payloadEnd && *payloadEnd == ',') {
		if (!parseBoundedDecimal(payloadEnd + 1, 0, kIRSendMaxRepeats,
			&repeats)) {
			ADDLOG_ERROR(LOG_FEATURE_IR,
				(char *)"IRSend invalid repeat count '%s' (expected 0-%u)",
				payloadEnd + 1, (unsigned int)kIRSendMaxRepeats);
			return CMD_RES_BAD_ARGUMENT;
		}
	} else if (payloadEnd && *payloadEnd) {
		return CMD_RES_BAD_ARGUMENT;
	}
	if (!gIRDriverReady || !pIRsend) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend: transmitter is not active");
		return CMD_RES_ERROR;
	}
	const bool statePayload = IR_ProtocolUsesStatePayload(protocol);
	if (statePayload && !isValidStatePayloadLength(protocol, bits, nbytes)) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend invalid state length for %s: bits %d bytes %d",
			args, (int)bits, (int)nbytes);
		return CMD_RES_BAD_ARGUMENT;
	}
	if (!statePayload && bits > 64U) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend scalar protocol %s cannot use %u bits",
			args, (unsigned int)bits);
		return CMD_RES_BAD_ARGUMENT;
	}
	if (!pIRsend->beginSendTransaction()) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IR send busy; previous transmission has not completed");
		return CMD_RES_ERROR;
	}
	const uint16_t stagedBefore = pIRsend->getStagedItemCount();
	bool sent = false;
	if (statePayload) {
		if (protocol == decode_type_t::ARGO) {
#if SEND_ARGO
			if (!IR_IsValidArgoPayload(state, nbytes)) {
				pIRsend->abortSendTransaction();
				ADDLOG_ERROR(LOG_FEATURE_IR,
					(char *)"IRSend invalid ARGO message type/length");
				return CMD_RES_BAD_ARGUMENT;
			}
			if (IR_IsArgoWrem3Payload(state, nbytes))
				pIRsend->sendArgoWREM3(state, nbytes, 0);
			else
				pIRsend->sendArgo(state, nbytes, 0);
			sent = pIRsend->getStagedItemCount() > stagedBefore;
#else
			sent = false;
#endif
		} else {
			sent = pIRsend->send(protocol, state, nbytes) &&
				pIRsend->getStagedItemCount() > stagedBefore;
		}
		if (sent) {
			pIRsend->delay(100);
			sent = pIRsend->setTransactionRepeats((uint16_t)repeats);
		}
	} else {
		uint64_t data = 0;
		for (uint16_t index = 0; index < nbytes; index++)
			data = (data << 8) | state[index];
		sent = pIRsend->send(protocol, data, bits, (uint16_t)repeats) &&
			pIRsend->getStagedItemCount() > stagedBefore;
		if (sent) pIRsend->delay(100);
	}
	if (!sent || !IR_CommitSendTransaction()) {
		pIRsend->abortSendTransaction();
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IR can't queue complete send %s: protocol %d bits %d repeats %d",
			args, (int)protocol, (int)bits, (int)repeats);
		return CMD_RES_ERROR;
	}
	ADDLOG_INFO(LOG_FEATURE_IR,
		(char *)"IR send %s: protocol %d bits %d bytes %d repeats %d",
		args, (int)protocol, (int)bits, (int)nbytes, (int)repeats);
	return CMD_RES_OK;
}

static commandResult_t IR_SendClassicCommand(char *args) {
	char *fields[4] = { NULL, NULL, NULL, NULL };
	const uint8_t count = IR_SplitClassicFields(args, fields, 4);
	if (count < 3 || count > 4) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend classic form expects PROTOCOL ADDRESS COMMAND [REPEAT]");
		return CMD_RES_BAD_ARGUMENT;
	}
	const decode_type_t protocol = IR_ParseProtocol(fields[0]);
	if (protocol == decode_type_t::UNKNOWN || IR_ProtocolUsesStatePayload(protocol)) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend classic protocol '%s' is unsupported", fields[0]);
		return CMD_RES_BAD_ARGUMENT;
	}
	if (!IR_ProtocolTxTimingSupported(protocol)) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend protocol %s is unsupported at %.2fus timer resolution",
			fields[0], ir_periodus);
		return CMD_RES_ERROR;
	}
	uint32_t address = 0;
	uint32_t command = 0;
	uint32_t repeats = 0;
	if (!parseBoundedHex(fields[1], 0, UINT32_MAX, &address) ||
		!parseBoundedHex(fields[2], 0, UINT32_MAX, &command) ||
		(count == 4 && !parseBoundedHex(fields[3], 0, kIRSendMaxRepeats,
			&repeats))) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend invalid classic hexadecimal field");
		return CMD_RES_BAD_ARGUMENT;
	}
	if (!IR_ValidateClassicFields(protocol, address, command)) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend address/command out of range for %s", fields[0]);
		return CMD_RES_BAD_ARGUMENT;
	}
	if (!gIRDriverReady || !pIRsend) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend: transmitter is not active");
		return CMD_RES_ERROR;
	}
	if (!pIRsend->beginSendTransaction()) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IR send busy; previous transmission has not completed");
		return CMD_RES_ERROR;
	}
	const uint16_t stagedBefore = pIRsend->getStagedItemCount();
	if (protocol == decode_type_t::RC5) {
		pIRsend->sendRC5((uint64_t)pIRsend->encodeRC5(address, command),
			kRC5Bits, (uint16_t)repeats);
	} else if (protocol == decode_type_t::RC5X) {
		pIRsend->sendRC5((uint64_t)pIRsend->encodeRC5X(address, command),
			kRC5XBits, (uint16_t)repeats);
	} else if (protocol == decode_type_t::RC6) {
		pIRsend->sendRC6((uint64_t)pIRsend->encodeRC6(address, command),
			kRC6Mode0Bits, (uint16_t)repeats);
	} else if (protocol == decode_type_t::NEC) {
		pIRsend->sendNEC((uint64_t)pIRsend->encodeNEC(address, command),
			kNECBits, (uint16_t)repeats);
	} else if (protocol == decode_type_t::PANASONIC) {
		pIRsend->sendPanasonic((uint16_t)address, command,
			kPanasonicBits, (uint16_t)repeats);
	} else if (protocol == decode_type_t::JVC) {
		pIRsend->sendJVC((uint64_t)pIRsend->encodeJVC(address, command),
			kJvcBits, (uint16_t)repeats);
	} else if (protocol == decode_type_t::SAMSUNG) {
		pIRsend->sendSAMSUNG((uint64_t)pIRsend->encodeSAMSUNG(address, command),
			kSamsungBits, (uint16_t)repeats);
	} else if (protocol == decode_type_t::LG) {
		pIRsend->sendLG((uint64_t)pIRsend->encodeLG(address, command),
			kLgBits, (uint16_t)repeats);
	} else {
		pIRsend->abortSendTransaction();
		return CMD_RES_BAD_ARGUMENT;
	}
	if (pIRsend->getStagedItemCount() <= stagedBefore) {
		pIRsend->abortSendTransaction();
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSend protocol %s produced no waveform", fields[0]);
		return CMD_RES_ERROR;
	}
	pIRsend->delay(100);
	if (!IR_CommitSendTransaction()) {
		pIRsend->abortSendTransaction();
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IR can't queue complete send %s", fields[0]);
		return CMD_RES_ERROR;
	}
	ADDLOG_INFO(LOG_FEATURE_IR,
		(char *)"IR send %s addr 0x%X cmd 0x%X repeats %u",
		fields[0], (unsigned int)address, (unsigned int)command,
		(unsigned int)repeats);
	return CMD_RES_OK;
}

#if PLATFORM_BEKEN && defined(__GNUC__)
#define IR_SEND_CMD_OPT __attribute__((optimize("no-jump-tables")))
#else
#define IR_SEND_CMD_OPT
#endif

extern "C" IR_SEND_CMD_OPT commandResult_t IR_Send_Cmd(const void *context,
	const char *cmd, const char *args_in, int cmdFlags) {
	(void)context;
	(void)cmd;
	(void)cmdFlags;
	if (!args_in || !args_in[0]) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	IR_ServicePendingCarrier();
	char args[384];
	if (strlen(args_in) >= sizeof(args)) return CMD_RES_BAD_ARGUMENT;
	strncpy(args, args_in, sizeof(args) - 1U);
	args[sizeof(args) - 1U] = '\0';
	if (strchr(args, ',')) return IR_SendCommaCommand(args);
	return IR_SendClassicCommand(args);
}

extern "C" commandResult_t IR_Enable(const void *context, const char *cmd,
	const char *args_in, int cmdFlags) {
	(void)context;
	(void)cmd;
	(void)cmdFlags;
	if (!args_in || !args_in[0]) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	IR_ServicePendingCarrier();
	char args[64];
	if (strlen(args_in) >= sizeof(args)) return CMD_RES_BAD_ARGUMENT;
	strncpy(args, args_in, sizeof(args) - 1U);
	args[sizeof(args) - 1U] = '\0';
	char *words[2] = { NULL, NULL };
	const uint8_t count = IR_SplitWords(args, words, 2);
	if (!count || count > 2) return CMD_RES_BAD_ARGUMENT;
	uint32_t enabled = 0;
	if (count == 2 && !parseBoundedDecimal(words[1], 0, 1, &enabled))
		return CMD_RES_BAD_ARGUMENT;
	if (IR_EqualsIgnoreCase(words[0], "RXTX")) {
		if (count == 1) enabled = 1;
		if (pIRsend && pIRsend->isBusy()) return CMD_RES_ERROR;
		gEnableIRSendWhilstReceive = (uint8_t)enabled;
		ADDLOG_INFO(LOG_FEATURE_IR,
			(char *)"IREnable RX whilst TX set %u", (unsigned int)enabled);
		return CMD_RES_OK;
	}
	if (IR_EqualsIgnoreCase(words[0], "invert")) {
		if (count == 1) enabled = 0;
		if (pIRsend && !pIRsend->setInverted(enabled != 0U)) {
			ADDLOG_ERROR(LOG_FEATURE_IR,
				(char *)"IREnable invert rejected while a send is active");
			return CMD_RES_ERROR;
		}
		gIRPinPolarity = (uint8_t)enabled;
		ADDLOG_INFO(LOG_FEATURE_IR,
			(char *)"IREnable invert set %u", (unsigned int)enabled);
		return CMD_RES_OK;
	}
	ADDLOG_ERROR(LOG_FEATURE_IR,
		(char *)"IREnable protocol masks are not implemented; use RXTX or invert");
	return CMD_RES_BAD_ARGUMENT;
}

extern "C" commandResult_t IR_Param(const void *context, const char *cmd,
	const char *args_in, int cmdFlags) {
	(void)context;
	(void)cmd;
	(void)cmdFlags;
	if (!args_in || !args_in[0]) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	if (!ourReceiver || !IR_ReceiverStorageReady()) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRParam: receiver is not active");
		return CMD_RES_ERROR;
	}
	char args[64];
	if (strlen(args_in) >= sizeof(args)) return CMD_RES_BAD_ARGUMENT;
	strncpy(args, args_in, sizeof(args) - 1U);
	args[sizeof(args) - 1U] = '\0';
	char *words[2] = { NULL, NULL };
	if (IR_SplitWords(args, words, 2) != 2) return CMD_RES_BAD_ARGUMENT;
	uint32_t minimumSize = 0;
	uint32_t tolerance = 0;
	const uint32_t maximumSize = ourReceiver->getBufSize();
	if (!parseBoundedDecimal(words[0], 1, maximumSize, &minimumSize) ||
		!parseBoundedDecimal(words[1], 0, 100, &tolerance)) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRParam expects MinSize 1-%u and tolerance 0-100",
			(unsigned int)maximumSize);
		return CMD_RES_BAD_ARGUMENT;
	}
	ourReceiver->setUnknownThreshold((uint16_t)minimumSize);
	ourReceiver->setTolerance((uint8_t)tolerance);
	ADDLOG_INFO(LOG_FEATURE_IR,
		(char *)"IRParam MinUnknownSize: %u tolerance: %u%%",
		(unsigned int)minimumSize, (unsigned int)tolerance);
	return CMD_RES_OK;
}

#ifdef ENABLE_IRAC
extern "C" commandResult_t IR_AC_Cmd(const void *context, const char *cmd,
	const char *args_in, int cmdFlags) {
	(void)context;
	(void)cmd;
	(void)args_in;
	(void)cmdFlags;
	ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRAC is not implemented");
	return CMD_RES_ERROR;
}
#endif

static void IR_RegisterCommands(void) {
	static bool registered = false;
	if (registered) return;
	//cmddetail:{"name":"IRSend","args":"[PROT-ADDR-CMD-REP] or [PROT,bits,0xDATA[,repeat]]",
	//cmddetail:"descr":"Sends IR commands either in the classic form PROT-ADDR-CMD-REP, e.g. NEC-1-1A-0, or in the raw-data form PROT,bits,0xDATA[,repeat] for long payloads such as A/C state frames",
	//cmddetail:"fn":"IR_Send_Cmd","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("IRSend", IR_Send_Cmd, NULL);
	//cmddetail:{"name":"IREnable","args":"[RXTX|invert] [0|1]",
	//cmddetail:"descr":"Enable or disable receiving while transmitting, or invert the IR transmit output",
	//cmddetail:"fn":"IR_Enable","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("IREnable", IR_Enable, NULL);
	//cmddetail:{"name":"IRParam","args":"[MinSize] [Tolerance]",
	//cmddetail:"descr":"Set the minimum received-message size and matching tolerance percentage",
	//cmddetail:"fn":"IR_Param","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("IRParam", IR_Param, NULL);
#ifdef ENABLE_IRAC
	//cmddetail:{"name":"IRAC","args":"[TODO]",
	//cmddetail:"descr":"Sends IR commands for HVAC control (not implemented)",
	//cmddetail:"fn":"IR_AC_Cmd","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("IRAC", IR_AC_Cmd, NULL);
#endif
	registered = true;
}

extern "C" void DRV_IR_Deinit(void) {
	gIRDriverReady = false;
	gIRDeferredStart = false;
	if (ir_chan >= 0) HAL_HWTimerStop(ir_chan);

	if (pIRsend) {
		myIRsend *sender = pIRsend;
		pIRsend = NULL;
		sender->stopAndReleaseResources();
		delete sender;
	}
	if (ourReceiver) {
		IRrecv *receiver = ourReceiver;
		ourReceiver = NULL;
		delete receiver;
	}
	IR_ClearReceiverStorageState();

	if (ir_chan >= 0) {
		HAL_HWTimerDeinit(ir_chan);
		ir_chan = -1;
	}
	ir_periodus = 50.0f;
	ir_periodus_rounded = 50U;
	gIRUseVirtualMicros = false;
}

extern "C" void DRV_IR_Init(void) {
	IR_RegisterCommands();
	DRV_IR_Deinit();

#if ENABLE_DRIVER_TINYIR_NEC
	if (TinyIR_NEC_IsReady()) {
		gIRDeferredStart = true;
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRremoteESP8266 deferred while TinyIR_NEC is running");
		return;
	}
#endif

	int receivePin = PIN_FindPinIndexForRole(IOR_IRRecv, -1);
	bool pullup = true;
	if (receivePin < 0) {
		receivePin = PIN_FindPinIndexForRole(IOR_IRRecv_nPup, -1);
		pullup = false;
	}
	const int transmitPin = PIN_FindPinIndexForRole(IOR_IRSend, -1);
	if (receivePin < 0 && transmitPin < 0) {
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR driver has no configured pins");
		return;
	}

	if (receivePin >= 0) {
		IRrecv *receiver = new IRrecv((uint16_t)receivePin);
		if (!receiver || !IR_ReceiverStorageReady()) {
			delete receiver;
			IR_ClearReceiverStorageState();
			ADDLOG_ERROR(LOG_FEATURE_IR,
				(char *)"IR receiver allocation failed on pin %d", receivePin);
		} else {
			ourReceiver = receiver;
			ourReceiver->enableIRIn(pullup);
		}
	}

	if (transmitPin >= 0) {
		if (!HAL_PIN_CanThisPinBePWM(transmitPin)) {
			ADDLOG_ERROR(LOG_FEATURE_IR,
				(char *)"IR transmit pin %d is not PWM-capable", transmitPin);
		} else {
			myIRsend *sender = new myIRsend((uint_fast8_t)transmitPin);
			if (!sender) {
				ADDLOG_ERROR(LOG_FEATURE_IR,
					(char *)"IR transmitter allocation failed on pin %d", transmitPin);
			} else {
				pIRsend = sender;
				pIRsend->enableIROut(38000, 50);
				pIRsend->setInverted(gIRPinPolarity != 0);
			}
		}
	}

	if (!ourReceiver && !pIRsend) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR driver failed to initialize");
		return;
	}
	if (!_timerConfigForReceive()) {
		DRV_IR_Deinit();
		return;
	}
	if (ourReceiver && !IR_SyncReceiverInput(false)) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR receiver synchronization failed");
		DRV_IR_Deinit();
		return;
	}
	delay_ms(10);
	_timer_enable();
	gIRDriverReady = true;
	ADDLOG_INFO(LOG_FEATURE_IR,
		(char *)"IR ready: RX pin %d TX pin %d timer %d period %.2fus",
		receivePin, transmitPin, (int)ir_chan, ir_periodus);
	if (pIRsend && (!IR_ProtocolTxTimingSupported(decode_type_t::RCMM) ||
		!IR_ProtocolTxTimingSupported(decode_type_t::LEGOPF))) {
		ADDLOG_INFO(LOG_FEATURE_IR,
			(char *)"IR TX timer resolution disables RCMM and/or LEGOPF");
	}
}

extern "C" int DRV_IR_IsReady(void) {
	return gIRDriverReady ? 1 : 0;
}

extern "C" int DRV_IR_IsDeferred(void) {
	return gIRDeferredStart ? 1 : 0;
}


void dump(decode_results *results) {
	// Dumps out the decode_results structure.
	// Call this after IRrecv::decode()
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"%s",
		resultToHumanReadableBasic(results).c_str());

	#ifdef ENABLE_IRAC
	if (hasACState(results->decode_type))
	{
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"%s",
			IRAcUtils::resultAcToString(results).c_str());
	}
	#endif
}



////////////////////////////////////////////////////
// this polls the IR receive to see if there was any IR received
extern "C" void DRV_IR_RunFrame() {
	if (!gIRDriverReady) {
#if ENABLE_DRIVER_TINYIR_NEC
		if (gIRDeferredStart && !TinyIR_NEC_IsReady()) {
			gIRDeferredStart = false;
			DRV_IR_Init();
		}
#endif
		if (!gIRDriverReady) return;
	}
	IR_ServicePendingCarrier();
	if (pIRsend && pIRsend->overflows) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IR send queue overflowed; transmission aborted");
		if (ir_chan >= 0) HAL_HWTimerStop(ir_chan);
		pIRsend->cancelActiveSend();
		if (gIRDriverReady && ir_chan >= 0) HAL_HWTimerStart(ir_chan);
	}
	if (!ourReceiver || !IR_ReceiverStorageReady()) return;

	decode_results results;
	if (!ourReceiver->decode(&results)) return;

	const String protocolName = typeToString(results.decode_type, results.repeat);
	const bool stateResult = IR_ProtocolUsesStatePayload(results.decode_type);
	const bool acStateResult = IR_ProtocolUsesACState(results.decode_type);
	const String dataText = resultToHexidecimal(&results);
	String lastIrReceived = String((int)results.decode_type, 16) + "," + dataText;
	// Preserve the established MQTT form: AC states omit Bits, all other
	// protocols (including MWM) include it.
	if (!acStateResult)
		lastIrReceived += "," + String((int)results.bits);

	char logText[256] = { 0 };
	const int repeat = results.repeat ? 1 : 0;
	const bool allowed = results.decode_type != decode_type_t::UNKNOWN ||
		CFG_HasFlag(OBK_FLAG_IR_ALLOW_UNKNOWN);
	if (allowed) {
		if (results.decode_type == decode_type_t::UNKNOWN) {
			snprintf(logText, sizeof(logText), "IR Unknown %s",
				lastIrReceived.c_str());
		} else if (stateResult) {
			snprintf(logText, sizeof(logText), "IR %s,%u,%s",
				protocolName.c_str(), (unsigned int)results.bits,
				dataText.c_str());
#ifdef ENABLE_IRAC
			if (acStateResult) {
				const String description = IRAcUtils::resultAcToString(&results);
				ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IRAC %s",
					description.c_str());
			}
#endif
		} else {
			snprintf(logText, sizeof(logText), "IR %s %lX %lX %d",
				protocolName.c_str(), (long int)results.address,
				(long int)results.command, repeat);
		}
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"%s", logText);
		if (!stateResult && results.decode_type != decode_type_t::UNKNOWN) {
			ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR %s,%u,%s",
				protocolName.c_str(), (unsigned int)results.bits,
				dataText.c_str());
		}

		if (CFG_HasFlag(OBK_FLAG_IR_PUBLISH_RECEIVED)) {
			const uint32_t before = ir_counter;
			MQTT_PublishMain_StringString("ir", lastIrReceived.c_str(), 0);
			const uint32_t duration = (uint32_t)(
				((float)(ir_counter - before) * ir_periodus) / 1000.0f);
			ADDLOG_INFO(LOG_FEATURE_IR,
				(char *)"IR MQTT publish %s took %ums", logText,
				(unsigned int)duration);
		}
		if (CFG_HasFlag(OBK_FLAG_IR_PUBLISH_RECEIVED_IN_JSON)) {
			snprintf(logText, sizeof(logText),
				"{\"IrReceived\":{\"Protocol\":\"%s\",\"Bits\":%u,\"Data\":\"%s\"}}",
				protocolName.c_str(), (unsigned int)results.bits,
				dataText.c_str());
			MQTT_PublishMain_StringString("RESULT", logText,
				OBK_PUBLISH_FLAG_FORCE_REMOVE_GET);
		}

		int eventType = 0;
		if (!stateResult) {
			if (results.decode_type == decode_type_t::NEC)
				eventType = CMD_EVENT_IR_NEC;
			else if (results.decode_type == decode_type_t::SAMSUNG)
				eventType = CMD_EVENT_IR_SAMSUNG;
			else if (results.decode_type == decode_type_t::SHARP)
				eventType = CMD_EVENT_IR_SHARP;
			else if (results.decode_type == decode_type_t::RC5)
				eventType = CMD_EVENT_IR_RC5;
			else if (results.decode_type == decode_type_t::RC6)
				eventType = CMD_EVENT_IR_RC6;
			else if (results.decode_type == decode_type_t::SONY)
				eventType = CMD_EVENT_IR_SONY;
		}
		if (eventType) {
			const uint32_t before = ir_counter;
			EventHandlers_FireEvent2(eventType, results.address, results.command);
			const uint32_t duration = (uint32_t)(
				((float)(ir_counter - before) * ir_periodus) / 1000.0f);
			ADDLOG_DEBUG(LOG_FEATURE_IR,
				(char *)"IR fire event took %ums", (unsigned int)duration);
		}
	} else {
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Received Unknown IR");
	}

	ourReceiver->resume();
	IR_SyncReceiverInput(true);
}


#ifdef TEST_CPP
// routines to test C++
class cpptest2 {
public:
	int initialised;
	cpptest2() {
		// remove else static class may kill us!!!ADDLOG_INFO(LOG_FEATURE_IR, "Log from Class constructor");
		initialised = 42;
	};
	~cpptest2() {
		initialised = 24;
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from Class destructor");
	}

	void print() {
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from Class %d", initialised);
	}
};

cpptest2 staticclass;

void cpptest() {
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from CPP");
	cpptest2 test;
	test.print();
	cpptest2 *test2 = new cpptest2();
	test2->print();
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from static class (is it initialised?):");
	staticclass.print();
}
#endif

#endif

