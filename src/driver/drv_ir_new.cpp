
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
	// 0 == active low.  1 = active hi
	uint8_t gIRPinPolarity = 0;

	extern int my_strnicmp(const char* a, const char* b, int len);
	extern unsigned int g_timeMs;
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

// dummy functions
#if PLATFORM_BEKEN
void noInterrupts() { }
void interrupts() { }
void delay(int n) { }
void delayMicroseconds(int n) { }
unsigned long millis()
{
	return 0;
}
unsigned long micros()
{
	return 0;
}
#else
void noInterrupts() { taskENTER_CRITICAL(); }
void interrupts() { taskEXIT_CRITICAL(); }
void delay(int n) { delay_ms(n); }
void delayMicroseconds(int n) { HAL_Delay_us(n); }
unsigned long millis()
{
	return g_timeMs;
}
unsigned long micros()
{
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

static int8_t ir_chan = -1;
static float ir_periodus = 50;

void timerConfigForReceive() {
	// nothing here`
}

void _timerConfigForReceive() {
	ir_counter = 0;

	ir_chan = HAL_RequestHWTimer(ir_periodus, &ir_periodus, DRV_IR_ISR, NULL);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer %u, %.2f us period", ir_chan, ir_periodus);
}

static void timer_enable() {
}
static void timer_disable() {
}
static void _timer_enable() {
	HAL_HWTimerStart(ir_chan);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer enabled %u", ir_chan);
}
static void _timer_disable() {
	HAL_HWTimerStop(ir_chan);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer disabled %u", ir_chan);
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

class myIRsend : public IRsend {
public:
	myIRsend(uint_fast8_t aSendPin) :IRsend(aSendPin) {
		sendPin = aSendPin;
		our_us = 0;
		our_ms = 0;
		resetsendqueue();
	}
	~myIRsend() { }


	uint32_t millis() {
		return our_ms;
	}

	bool beginSendTransaction() {
		if (isBusy()) {
			return false;
		}
		transactionCount = 0;
		transactionFailed = false;
		overflows = 0;
		transactionBuilding = 1;
		return true;
	}

	bool commitSendTransaction() {
		if (!transactionBuilding || transactionFailed || transactionCount == 0) {
			abortSendTransaction();
			return false;
		}

		// The ISR only looks at the queue after transactionReady is set. Publish
		// all queue metadata first, then flip that single-byte flag last.
		timeout = 0;
		timein = transactionCount;
		timecount = transactionCount;
		timecounttotal = transactionCount;
		transactionReady = 1;
		transactionBuilding = 0;
		return true;
	}

	void abortSendTransaction() {
		transactionBuilding = 0;
		transactionFailed = false;
		transactionCount = 0;
	}

	bool isBusy() const {
		return transactionBuilding || transactionReady || currentsendtime;
	}

	uint16_t getStagedItemCount() const {
		return transactionCount;
	}

	void delay(long int ms) {
		// add a pure delay to our queue
		space(ms * 1000);
	}

	uint16_t mark(uint16_t aMarkMicros) {
		// store mark bits in highest +ve bit of count
		return appendDuration(aMarkMicros | 0x10000000) ? 1 : 0;
	}

	void space(uint32_t aMarkMicros) {
		appendDuration(aMarkMicros);
	}

	void enableIROut(uint32_t freq, uint8_t duty=50) {
		//uint_fast8_t aFrequencyKHz
		if (freq < 1000)  // Were we given kHz? Supports the old call usage.
			freq *= 1000;
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"enableIROut %d freq %d duty",(int)freq, (int)duty);
		if(duty<1)
			duty=1;
		pwmduty = duty;

		HAL_PIN_PWM_Start(sendPin, freq);
		//HAL_PIN_PWM_Update(sendPin, duty);
	}

	void resetsendqueue() {
		// Hide the queue from the ISR before resetting shared metadata.
		transactionReady = 0;
		transactionBuilding = 0;
		transactionFailed = false;
		transactionCount = 0;
		timein = timeout = 0;
		timecount = 0;
		overflows = 0;
		currentsendtime = 0;
		currentbitval = 0;
		timecounttotal = 0;
	}

	bool getsendqueue(int32_t *value) {
		if (!value || !transactionReady) {
			return false;
		}
		if (timeout >= timein) {
			transactionReady = 0;
			timein = timeout = 0;
			timecount = 0;
			return false;
		}

		*value = times[timeout++];
		if (timecount) {
			timecount--;
		}
		return true;
	}

	int32_t times[SEND_QUEUE_ITEMS]; // committed mark/space transaction
	volatile unsigned short timein;
	volatile unsigned short timeout;
	volatile unsigned short timecount;
	unsigned short overflows;
	uint32_t timecounttotal;
	volatile int currentsendtime;
	volatile int currentbitval;

	uint8_t sendPin;
	uint32_t pwmduty;

	uint32_t our_ms;
	float our_us;

private:
	bool appendDuration(const uint32_t duration) {
		if (!transactionBuilding || transactionFailed) {
			return false;
		}
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
};


// our send/receive instances
myIRsend *pIRsend = NULL;
IRrecv *ourReceiver = NULL;

// this is our ISR.
// it is called every 50us, so we need to work on making it as efficient as possible.
extern "C" void DRV_IR_ISR(void* arg)
{
	int sending = 0;
	if (pIRsend) {
		pIRsend->our_us += ir_periodus;
		if (pIRsend->our_us > 1000) {
			pIRsend->our_ms++;
			pIRsend->our_us -= 1000;
		}

		int pinval = 0;
		if (pIRsend->currentsendtime) {
			sending = 1;
			pIRsend->currentsendtime -= ir_periodus;
			if (pIRsend->currentsendtime <= 0) {
				int32_t remains = pIRsend->currentsendtime;
				int32_t newtime = 0;
				if (!pIRsend->getsendqueue(&newtime)) {
					// if it was the last one
					pIRsend->currentsendtime = 0;
					pIRsend->currentbitval = 0;
				}
				else {
					// we got a new time
					// store mark bits in highest +ve bit of count
					pIRsend->currentbitval = (newtime & 0x10000000) ? 1 : 0;
					pIRsend->currentsendtime = (newtime & 0xfffffff);
					// adjust the us value to keep the running accuracy
					// and avoid a running error?
					// note remains is -ve
					pIRsend->currentsendtime += remains;
				}
			}
		}
		else {
			int32_t newtime = 0;
			if (!pIRsend->getsendqueue(&newtime)) {
				pIRsend->currentsendtime = 0;
				pIRsend->currentbitval = 0;
			}
			else {
				sending = 1;
				pIRsend->currentsendtime = (newtime & 0xfffffff);
				pIRsend->currentbitval = (newtime & 0x10000000) ? 1 : 0;
			}
		}
		pinval = pIRsend->currentbitval;

		uint32_t duty = pIRsend->pwmduty;
		if (!pinval) {
			if (gIRPinPolarity) {
				duty = 50;
			}
			else {
				duty = 0;
			}
		}
		HAL_PIN_PWM_Update(pIRsend->sendPin, duty);
	}

	// is someone really wants rx and TX at the same time, then allow it.
	if (gEnableIRSendWhilstReceive) {
		sending = 0;
	}

	// don't receive if we are currently sending
	if (ourReceiver && !sending){
		IR_ISR(ir_periodus);
	}
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
		if (value > maxValue / 10 ||
			(value == maxValue / 10 && digit > maxValue % 10)) {
			return false;
		}
		value = value * 10 + digit;
	}
	if (value < minValue) return false;

	*result = value;
	return true;
}

static bool isValidStatePayloadLength(const decode_type_t protocol,
	const uint16_t bits, const uint16_t nbytes) {
	if (!nbytes || nbytes != (bits + 7) / 8) return false;

	// Carrier AC84 carries four data bits in its first byte, followed by ten
	// complete bytes. It is the only supported state protocol that is not byte
	// aligned.
	if (protocol == decode_type_t::CARRIER_AC84)
		return bits == kCarrierAc84Bits;
	if (bits & 7) return false;

	// Protocols with more than one valid state frame size.
	if (protocol == decode_type_t::ARGO)
		return bits == 32 || bits == 96;
	if (protocol == decode_type_t::CORONA_AC)
		return bits == 56 || bits == 168;
	if (protocol == decode_type_t::DAIKIN)
		return bits == 216 || bits == 280;
	if (protocol == decode_type_t::FUJITSU_AC)
		return bits == 48 || bits == 56 || bits == 120 || bits == 128;
	if (protocol == decode_type_t::HITACHI_AC3)
		return bits == 120 || bits == 136 || bits == 168 ||
			bits == 184 || bits == 216;
	if (protocol == decode_type_t::PANASONIC_AC)
		return bits == 128 || bits == 216;
	if (protocol == decode_type_t::SAMSUNG_AC)
		return bits == 112 || bits == 168;
	if (protocol == decode_type_t::TOSHIBA_AC)
		return bits >= 56 && bits <= 80;

	// MWM is intentionally variable length. The per-frame staging check below
	// still rejects lengths for which the upstream sender emits nothing.
	if (protocol == decode_type_t::MWM) return true;

	const uint16_t expectedBits = IRsend::defaultBits(protocol);
	return expectedBits && bits == expectedBits;
}


static bool parseHexStateBytes(const char *hexIn, uint16_t bits, uint8_t *out, uint16_t outSize, uint16_t *outBytes, const char **endPtr) {
	if (!hexIn || !out || !outBytes) return false;
	const char *hex = hexIn;
	if (hex[0] == '0' && (hex[1] == 'x' || hex[1] == 'X')) hex += 2;
	uint16_t nbytes = (bits + 7) / 8;
	if (!nbytes || nbytes > outSize) return false;
	for (uint16_t i = 0; i < nbytes; i++) out[i] = 0;
	uint16_t nibbleCount = 0;
	while (hex[nibbleCount] && hex[nibbleCount] != ',') {
		if (hexNibbleValue(hex[nibbleCount]) < 0) return false;
		nibbleCount++;
	}
	if (!nibbleCount) return false;
	uint16_t maxNibbles = nbytes * 2;
	// Allow extra leading zeros beyond the requested bit-length.
	// This keeps older command strings working (e.g. bits=56 but payload has 16 nibbles with leading 00).
	while (nibbleCount > maxNibbles && hex[0] == '0') { hex++; nibbleCount--; }
	if (nibbleCount > maxNibbles) return false;
	uint16_t nibbleOffset = maxNibbles - nibbleCount;
	for (uint16_t i = 0; i < nibbleCount; i++) {
		int v = hexNibbleValue(hex[i]);
		uint16_t pos = nibbleOffset + i;
		uint16_t byteIndex = pos / 2;
		if ((pos & 1) == 0)
			out[byteIndex] |= (uint8_t)(v << 4);
		else
			out[byteIndex] |= (uint8_t)v;
	}

	// State bytes are right-aligned. Reject any supplied high bits that
	// fall outside the requested width rather than silently discarding them.
	const uint8_t unusedHighBits = (uint8_t)(nbytes * 8 - bits);
	if (unusedHighBits) {
		const uint8_t unusedMask =
			(uint8_t)(0xFFU << (8 - unusedHighBits));
		if (out[0] & unusedMask) return false;
	}

	*outBytes = nbytes;
	if (endPtr) *endPtr = hex + nibbleCount;
	return true;
}

#if PLATFORM_BEKEN && defined(__GNUC__)
#define IR_SEND_CMD_OPT __attribute__((optimize("no-jump-tables")))
#else
#define IR_SEND_CMD_OPT
#endif

extern "C" IR_SEND_CMD_OPT commandResult_t IR_Send_Cmd(const void *context, const char *cmd, const char *args_in, int cmdFlags) {
	if (!args_in) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	char args[384];
	strncpy(args, args_in, sizeof(args) - 1);
	args[sizeof(args) - 1] = 0;

	// split arg at hyphen;
	char *p = args;
	while (*p && (*p != '-') && (*p != ' ')) {
		p++;
	}

	if ((*p != '-') && (*p != ' ')) {
		// try to decode "new" format, separated by comma
		// the format is PROT,bits,0xDATA[,repeat]
		char *p = args;
		while (*p && (*p != ',')) {
			p++;
		}
		if(*p==',')
		{
			*p='\0';
			decode_type_t protocol = strToDecodeType(args);
			p++;
			char *_bits=p;
			while (*p && (*p != ',')) {
				p++;
			}
			if(*p==',')
			{
				*p='\0';
				uint32_t bitsValue = 0;
				if (!parseBoundedDecimal(_bits, 1, kIRSendMaxBits, &bitsValue)) {
					ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend invalid bit count '%s' (expected 1-%u)", _bits, (unsigned int)kIRSendMaxBits);
					return CMD_RES_BAD_ARGUMENT;
				}
				const uint16_t bits = (uint16_t)bitsValue;
				p++;
				if(protocol!=decode_type_t::UNKNOWN && pIRsend)
				{
					int repeats=0;
					char *_data=p;
					uint8_t state[kIRSendMaxStateBytes];
					uint16_t nbytes = 0;
					const char *payloadEnd = NULL;
					if (!parseHexStateBytes(_data, bits, state, sizeof(state), &nbytes, &payloadEnd)) {
						ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend invalid payload for %s (bits=%d)", args, (int)bits);
						return CMD_RES_BAD_ARGUMENT;
					}
					if(payloadEnd && *payloadEnd==',') {
						uint32_t repeatValue = 0;
						if (!parseBoundedDecimal(payloadEnd + 1, 0, kIRSendMaxRepeats, &repeatValue)) {
							ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend invalid repeat count '%s' (expected 0-%u)", payloadEnd + 1, (unsigned int)kIRSendMaxRepeats);
							return CMD_RES_BAD_ARGUMENT;
						}
						repeats = (int)repeatValue;
					}
					const bool statePayload = bits > 64 || hasACState(protocol);
					if (statePayload && !isValidStatePayloadLength(protocol, bits, nbytes)) {
						ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend invalid state length for %s: bits %d bytes %d", args, (int)bits, (int)nbytes);
						return CMD_RES_BAD_ARGUMENT;
					}
					if (!pIRsend->beginSendTransaction()) {
						ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR send busy; previous transmission has not completed");
						return CMD_RES_ERROR;
					}
					if (statePayload)
					{
						bool sent = true;
						for (int repeatIndex = 0; repeatIndex <= repeats; repeatIndex++) {
							const uint16_t stagedBefore = pIRsend->getStagedItemCount();
							if (!pIRsend->send(protocol,state,nbytes) ||
								pIRsend->getStagedItemCount() == stagedBefore) {
								sent = false;
								break;
							}
							if (repeatIndex < repeats) {
								pIRsend->delay(100);
							}
						}
						if (sent) {
							pIRsend->delay(100);
						}
						if (sent && pIRsend->commitSendTransaction())
						{
							ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR send %s: protocol %d bits %d bytes %d repeats %d", args, (int)protocol, (int)bits, (int)nbytes, (int)repeats);
							return CMD_RES_OK;
						}
						pIRsend->abortSendTransaction();
						ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR can't queue complete send %s: protocol %d bits %d bytes %d repeats %d", args, (int)protocol, (int)bits, (int)nbytes, (int)repeats);
						return CMD_RES_BAD_ARGUMENT;
					}
					else
					{
						uint64_t data = 0;
						for (uint16_t bi = 0; bi < nbytes; bi++) {
							data = (data << 8) | state[bi];
						}
						bool sent = pIRsend->send(protocol,data,bits,repeats);
						if (sent) {
							pIRsend->delay(100);
						}
						if (sent && pIRsend->commitSendTransaction())
						{
							ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR send %s: protocol %d bits %d data 0x%llX repeats %d", args, (int)protocol, (int)bits, (long long int)data, (int)repeats);
							return CMD_RES_OK;
						}
						pIRsend->abortSendTransaction();
						ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR can't queue complete send %s: protocol %d bits %d data 0x%llX repeats %d", args, (int)protocol, (int)bits, (long long int)data, (int)repeats);
						return CMD_RES_BAD_ARGUMENT;
					}
				}
			} 
		}
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend cmnd not valid [%s] not like [NEC-0-1A] or [NEC 0 1A 1] or [NEC,bits,0xDATA,[repeat]]", args);
 		return CMD_RES_BAD_ARGUMENT;
	}

	*p='\0';
	decode_type_t protocol = strToDecodeType(args);
	if(hasACState(protocol))
	{
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend can't send AC commands", args);
		return CMD_RES_BAD_ARGUMENT;
	}
	p++;
	int addr = strtol(p, &p, 16);
	if ((*p != '-') && (*p != ' ')) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend cmnd not valid [%s] not like [NEC-0-1A] or [NEC 0 1A 1].", args);
		return CMD_RES_BAD_ARGUMENT;
	}
	p++;
	int command = strtol(p, &p, 16);

	int repeats = 0;

	if ((*p == '-') || (*p == ' ')) {
		p++;
		repeats = strtol(p, &p, 16);
	}

	if (pIRsend) {
		if (!pIRsend->beginSendTransaction()) {
			ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR send busy; previous transmission has not completed");
			return CMD_RES_ERROR;
		}

		// BK7238 is built for Thumb-1 without the libgcc switch-table helper.
		// Keep this dispatch as comparisons so GCC cannot emit __gnu_thumb1_case_si.
		if (protocol == decode_type_t::RC5) {
			pIRsend->sendRC5((uint64_t)pIRsend->encodeRC5(addr,command));
		} else if (protocol == decode_type_t::RC6) {
			pIRsend->sendRC6((uint64_t)pIRsend->encodeRC6(addr,command));
		} else if (protocol == decode_type_t::NEC) {
			pIRsend->sendNEC((uint64_t)pIRsend->encodeNEC(addr,command));
		} else if (protocol == decode_type_t::PANASONIC) {
			pIRsend->sendPanasonic((uint16_t)addr,(uint32_t)command);
		} else if (protocol == decode_type_t::JVC) {
			pIRsend->sendJVC((uint64_t)pIRsend->encodeJVC(addr,command));
		} else if (protocol == decode_type_t::SAMSUNG) {
			pIRsend->sendSAMSUNG((uint64_t)pIRsend->encodeSAMSUNG(addr,command));
		} else if (protocol == decode_type_t::LG) {
			pIRsend->sendLG((uint64_t)pIRsend->encodeLG(addr,command));
		} else {
			pIRsend->abortSendTransaction();
			ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR send %s protocol not supported", args);
			return CMD_RES_ERROR;
		}

		// add a 100ms delay after command
		// NOTE: this is NOT a delay here.  it adds 100ms 'space' in the TX queue
		pIRsend->delay(100);
		if (!pIRsend->commitSendTransaction()) {
			pIRsend->abortSendTransaction();
			ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR can't queue complete send %s", args);
			return CMD_RES_ERROR;
		}

		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR send %s protocol %d addr 0x%X cmd 0x%X repeats %d", args, (int)protocol, (int)addr, (int)command, (int)repeats);
		return CMD_RES_OK;
	}
	else {
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR NOT send (no IRsend running) %s protocol %d addr 0x%X cmd 0x%X repeats %d", args, (int)protocol, (int)addr, (int)command, (int)repeats);
	}
	return CMD_RES_ERROR;
}

extern "C" commandResult_t IR_Enable(const void *context, const char *cmd, const char *args_in, int cmdFlags) {
	if (!args_in || !args_in[0]) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IREnable expects arguments");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	char args[384];
	strncpy(args, args_in, sizeof(args)-1);
	args[sizeof(args)-1] = 0;
	char *p = args;
	int enable = 1;
	if (!my_strnicmp(p, "RXTX", 4)) {
		p += 4;
		if (*p == ' ') {
			p++;
			if (*p) {
				enable = atoi(p);
			}
		}
		gEnableIRSendWhilstReceive = enable;
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable RX whilst TX enable set %d", enable);
		return CMD_RES_OK;
	}

	if (!my_strnicmp(p, "invert", 6)) {
		// default normal.
		enable = 0;
		p += 6;
		if (*p == ' ') {
			p++;
			if (*p) {
				enable = atoi(p);
			}
		}
		gIRPinPolarity = enable;
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable invert set %d", enable);
		return CMD_RES_OK;
	}


	// find length of first arg.
	while (*p && (*p != ' ')) {
		p++;
	}

	//int numProtocols = sizeof(ProtocolNames)/sizeof(*ProtocolNames);
	#if 0 // number of protocols now is 125 
	// TODO: reimpleemnt this using bigger mask
	int numProtocols = 0;
	int ournamelen = (p - args);
	int protocol = -1;
	for (int i = 0; i < numProtocols; i++) {
		const char *name = "Unknown"; //= ProtocolNames[i];
		int namelen = strlen(name);
		if (!my_strnicmp(name, args, namelen) && (ournamelen == namelen)) {
			protocol = i;
			break;
		}
	}
	if (*p == ' ') {
		p++;
		if (*p) {
			enable = atoi(p);
		}
	}

	uint32_t thisbit = (1 << protocol);
	if (protocol < 0) {
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable invalid protocol %s", args);
		return CMD_RES_BAD_ARGUMENT;
	}
	else {
		//ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable found protocol %s(%d), enable %d from %s, bitmask 0x%08X", ProtocolNames[protocol], protocol, enable, p, thisbit);
	}
	if (enable) {
		gIRProtocolEnable = gIRProtocolEnable | thisbit;
	}
	else {
		gIRProtocolEnable = gIRProtocolEnable & (~thisbit);
	}
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable Protocol mask now 0x%08X", gIRProtocolEnable);
	#endif //TODO
	return CMD_RES_OK;
}


extern "C" commandResult_t IR_Param(const void *context, const char *cmd, const char *args_in, int cmdFlags) {
	if (!args_in || !args_in[0]) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRParam expects two arguments");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	if(!ourReceiver)
	{
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRParam: IR receiver disabled");
		return CMD_RES_BAD_ARGUMENT;
	}

	// Set higher if you get lots of random short UNKNOWN messages when nothing
	// should be sending a message.
	// Set lower if you are sure your setup is working, but it doesn't see messages
	// from your device. (e.g. Other IR remotes work.)
	// NOTE: Set this value very high to effectively turn off UNKNOWN detection.	
	int kMinUnknownSize = 12;

	// How much percentage lee way do we give to incoming signals in order to match
	// it?
	// e.g. +/- 25% (default) to an expected value of 500 would mean matching a
	//      value between 375 & 625 inclusive.
	// Note: Default is 25(%). Going to a value >= 50(%) will cause some protocols
	//       to no longer match correctly. In normal situations you probably do not
	//       need to adjust this value. Typically that's when the library detects
	//       your remote's message some of the time, but not all of the time.
	int kTolerancePercentage = 25;  // kTolerance is normally 25%

	int res = sscanf(args_in, "%d %d", &kMinUnknownSize, &kTolerancePercentage);

	if(res!=2)
	{
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRParam invalid parameters %s", args_in);
		return CMD_RES_BAD_ARGUMENT;
	}
	ourReceiver->setUnknownThreshold(kMinUnknownSize);
	ourReceiver->setTolerance(kTolerancePercentage);

	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IRParam MinUnknownSize: %d  Noice tolerance: %d%%", kMinUnknownSize,kTolerancePercentage);
	return CMD_RES_OK;
}



#ifdef ENABLE_IRAC
extern "C" commandResult_t IR_AC_Cmd(const void *context, const char *cmd, const char *args_in, int cmdFlags) {
	if (!args_in) return CMD_RES_NOT_ENOUGH_ARGUMENTS;

	char args[64];
	strncpy(args, args_in, sizeof(args) - 1);
	args[sizeof(args) - 1] = 0;

	// split arg at hyphen;
	char *p = args;
	while (*p && (*p != '-') && (*p != ' ')) {
		p++;
	}
	int ournamelen = (p - args);
	if ((*p != '-') && (*p != ' ')) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRAC cmnd not valid [%s] ", args);
		return CMD_RES_BAD_ARGUMENT;
	}
	//	decode_type_t protocol = strToDecodeType(args);

	ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRAC cmnd not implemented yet", args);

	return CMD_RES_OK;
}
#endif //ENABLE_IRAC


// test routine to start IR RX and TX
// currently fixed pins for testing.
extern "C" void DRV_IR_Init() {
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from extern C CPP");

	int pin = -1; //9;// PWM3/25
	int txpin = -1; //24;// PWM3/25
	bool pup = true;

	// allow user to change them
	pin = PIN_FindPinIndexForRole(IOR_IRRecv, pin);
	if(pin == -1)
	{
		pin = PIN_FindPinIndexForRole(IOR_IRRecv_nPup, pin);
		if(pin >= 0) pup = false;
	}
	txpin = PIN_FindPinIndexForRole(IOR_IRSend, txpin);

	if (ourReceiver){
	     IRrecv *temp = ourReceiver;
	     ourReceiver = NULL;
	     delete temp;
	 }
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"DRV_IR_Init: recv pin %i", pin);
	if ((pin >= 0) || (txpin >= 0)) {
	}
	else {
		_timer_disable();
	}

	if (pin >= 0) {
		// setup IRrecv pin as input
		//bk_gpio_config_input_pup((GPIO_INDEX)pin); // enabled by enableIRIn

		//TODO: we should specify buffer size (now set to 1024), timeout (now 90ms) and tolerance 
		 ourReceiver = new IRrecv(pin);
		 ourReceiver->enableIRIn(pup);
	}

	if (pIRsend) {
		myIRsend *pIRsendTemp = pIRsend;
		pIRsend = NULL;
		delete pIRsendTemp;
	}

	if (txpin > 0) {
		// is this pin capable of PWM?
		if (HAL_PIN_CanThisPinBePWM(txpin)) {
			uint32_t pwmfrequency = 38000;
			myIRsend *pIRsendTemp = new myIRsend((uint_fast8_t)txpin);
			pIRsendTemp->resetsendqueue();
			pIRsendTemp->enableIROut(pwmfrequency, 50);

			pIRsend = pIRsendTemp;

			//cmddetail:{"name":"IRSend","args":"[PROT-ADDR-CMD-REP] or [PROT,bits,0xDATA[,repeat]]",
			//cmddetail:"descr":"Sends IR commands either in the classic form PROT-ADDR-CMD-REP, e.g. NEC-1-1A-0, or in the raw-data form PROT,bits,0xDATA[,repeat] for long payloads such as A/C state frames",
			//cmddetail:"fn":"IR_Send_Cmd","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
			//cmddetail:"examples":""}
			CMD_RegisterCommand("IRSend", IR_Send_Cmd, NULL);
			//cmddetail:{"name":"IRAC","args":"[TODO]",
			//cmddetail:"descr":"Sends IR commands for HVAC control (TODO)",
			//cmddetail:"fn":"IR_AC_Cmd","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
			//cmddetail:"examples":""}
			#ifdef ENABLE_IRAC
			CMD_RegisterCommand("IRAC", IR_AC_Cmd, NULL);
			#endif //ENABLE_IRAC
			//cmddetail:{"name":"IREnable","args":"[Str][1or0]",
			//cmddetail:"descr":"Enable/disable aspects of IR.  IREnable RXTX 0/1 - enable Rx whilst Tx.  IREnable [protocolname] 0/1 - enable/disable a specified protocol",
			//cmddetail:"fn":"IR_Enable","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
			//cmddetail:"examples":""}
			CMD_RegisterCommand("IREnable",IR_Enable, NULL);
			//cmddetail:{"name":"IRParam","args":"[MinSize] [Noise Threshold]",
			//cmddetail:"descr":"Set minimal size of the message and noise threshold",
			//cmddetail:"fn":"IR_Param","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
			//cmddetail:"examples":""}
			CMD_RegisterCommand("IRParam",IR_Param, NULL);
		}
	}
	if ((pin >= 0) || (txpin >= 0)) {
		// both tx and rx need the interrupt
		_timerConfigForReceive();
		delay_ms(10);
		_timer_enable();
	}
}

extern "C" void DRV_IR_Deinit()
{
	_timer_disable();
	HAL_HWTimerDeinit(ir_chan);
}

void dump(decode_results *results) {
	// Dumps out the decode_results structure.
	// Call this after IRrecv::decode()
	ADDLOG_INFO(LOG_FEATURE_IR, resultToHumanReadableBasic(results).c_str());

	#ifdef ENABLE_IRAC
	if (hasACState(results->decode_type))
	{
		ADDLOG_INFO(LOG_FEATURE_IR, IRAcUtils::resultAcToString(results).c_str());
	}
	#endif
}



////////////////////////////////////////////////////
// this polls the IR receive to see if there was any IR received
extern "C" void DRV_IR_RunFrame() {
	// Debug-only check to see if the timer interrupt is running
	if (ir_counter) {
		//ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR counter: %u", ir_counter);
	}
	if (pIRsend) {
		if (pIRsend->overflows) {
			ADDLOG_DEBUG(LOG_FEATURE_IR, (char *)"##### IR send overflows %d", (int)pIRsend->overflows);
			pIRsend->resetsendqueue();
		}
		else {
			//ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR send count %d remains %d currentus %d", (int)pIRsend->timecounttotal, (int)pIRsend->timecount, (int)pIRsend->currentsendtime);
		}
	}


	if (ourReceiver) {
		decode_results results;
		if (ourReceiver->decode(&results)) {
			// TODO: find a better way?
			String proto_name = typeToString(results.decode_type, results.repeat).c_str();

			#if 0 // TODO: implement different masking
			if (!(gIRProtocolEnable & (1 << (int)results.decode_type))) {
				ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR decode ignore masked protocol %s (%d) - mask 0x%08X", proto_name.c_str(), (int)results.decode_type, gIRProtocolEnable);
			}
			#endif

			//dump(&results);
			// 'UNKNOWN' protocol is by default disabled in flags
			// This is because I am getting a lot of 'UNKNOWN' spam with no IR signals in room
			if (((results.decode_type != decode_type_t::UNKNOWN) ||
				(results.decode_type == decode_type_t::UNKNOWN && CFG_HasFlag(OBK_FLAG_IR_ALLOW_UNKNOWN))) //&&
				// only process if this protocol is enabled.  all by default.
				//(gIRProtocolEnable & (1 << (int)results.decode_type)
				) {
				String lastIrReceived = String((int)results.decode_type, 16) + "," + resultToHexidecimal(&results);

				if (!hasACState(results.decode_type))
					lastIrReceived += "," + String((int)results.bits);
				else
					ADDLOG_INFO(LOG_FEATURE_IR, "Received AC code:%s",proto_name.c_str());

				char out[128];

				int repeat = results.repeat?0:1; // not sure how to deal with this

				if (results.decode_type == decode_type_t::UNKNOWN) {
					//snprintf(out, sizeof(out), "IR_RAW 0x%lX %d", (unsigned long)results.decodedRawData, repeat);
					snprintf(out, sizeof(out), "IR %s %s", "Unknown", lastIrReceived.c_str());
					ADDLOG_INFO(LOG_FEATURE_IR, (char *)out);
				}
				else if (!hasACState(results.decode_type)) {
					snprintf(out, sizeof(out), "IR %s %lX %lX %d", proto_name.c_str(), (long int)results.address, (long int)results.command, repeat);
					ADDLOG_INFO(LOG_FEATURE_IR, (char *)out);
					// show new format too
					snprintf(out, sizeof(out), "IR %s,%d,%s", proto_name.c_str(), (int)results.bits, resultToHexidecimal(&results).c_str());
					ADDLOG_INFO(LOG_FEATURE_IR, (char *)out);
				} else {
					#ifdef ENABLE_IRAC
					String description = IRAcUtils::resultAcToString(&results);
					ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IRAC %s", description.c_str());
					#endif //ENABLE_IRAC
				}
				// if user wants us to publish every received IR data, do it now
				if (CFG_HasFlag(OBK_FLAG_IR_PUBLISH_RECEIVED)) {

					// another flag required?
					int publishrepeats = 1;

					if (publishrepeats || !repeat) {
						//ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR MQTT publish %s", out);

						uint32_t counter_in = ir_counter;
						MQTT_PublishMain_StringString("ir", lastIrReceived.c_str(), 0);
						uint32_t counter_dur = ((ir_counter - counter_in) * 50) / 1000;
						ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR MQTT publish %s took %dms", out, counter_dur);
					}
					else {
						ADDLOG_INFO(LOG_FEATURE_IR, (char *)out);
					}
				}

				if (CFG_HasFlag(OBK_FLAG_IR_PUBLISH_RECEIVED_IN_JSON)) {
					// {"IrReceived":{"Protocol":"RC_5","Bits":0x1,"Data":"0xC"}}
					//
					String _data=resultToHexidecimal(&results);
					snprintf(out, sizeof(out), "{\"IrReceived\":{\"Protocol\":\"%s\",\"Bits\":%i,\"Data\":\"%s\"}}",
						proto_name.c_str(), (int)results.bits, _data.c_str());
					MQTT_PublishMain_StringString("RESULT", out, OBK_PUBLISH_FLAG_FORCE_REMOVE_GET);
				}

				if (results.decode_type != decode_type_t::UNKNOWN) {
					snprintf(out, sizeof(out), "%X", results.command);
					int tgType = 0;
					switch (results.decode_type)
					{
					case decode_type_t::NEC:
						tgType = CMD_EVENT_IR_NEC;
						break;
					case decode_type_t::SAMSUNG:
						tgType = CMD_EVENT_IR_SAMSUNG;
						break;
					case decode_type_t::SHARP:
						tgType = CMD_EVENT_IR_SHARP;
						break;
					case decode_type_t::RC5:
						tgType = CMD_EVENT_IR_RC5;
						break;
					case decode_type_t::RC6:
						tgType = CMD_EVENT_IR_RC6;
						break;
					case decode_type_t::SONY:
						tgType = CMD_EVENT_IR_SONY;
						break;
					default:
						break;
					}

					// we should include repeat here?
					// e.g. on/off button should not toggle on repeats, but up/down probably should eat them.
					uint32_t counter_in = ir_counter;
					EventHandlers_FireEvent2(tgType, results.address, results.command);
					uint32_t counter_dur = ((ir_counter - counter_in) * 50) / 1000;
					ADDLOG_DEBUG(LOG_FEATURE_IR, (char *)"IR fire event took %dms", counter_dur);
				}
			} else {
				ADDLOG_INFO(LOG_FEATURE_IR, "Received Unknown IR ");
			}
			/*
			* !!!Important!!! Enable receiving of the next value,
			* since receiving has stopped after the end of the current received data packet.
			*/
			ourReceiver->resume(); // Enable receiving of the next value
		}
	}
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

