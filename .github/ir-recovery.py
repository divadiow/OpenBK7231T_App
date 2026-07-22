from __future__ import annotations

import subprocess
import textwrap
from pathlib import Path

DRIVER = Path("src/driver/drv_ir_new.cpp")
RECEIVER = Path("src/libraries/IRremoteESP8266/src/IRrecv.cpp")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1 and label in ("repeat abort cleanup", "repeat reset cleanup"):
        signature = ("\tvoid abortSendTransaction()" if label == "repeat abort cleanup"
                     else "\tvoid resetsendqueue()")
        start, _, end = function_span(text, signature)
        block = text[start:end]
        if block.count(old) == 1:
            return text[:start] + block.replace(old, new, 1) + text[end:]
    if count != 1:
        raise RuntimeError(f"{label}: expected one match, found {count}")
    return text.replace(old, new, 1)


def function_span(text: str, signature: str) -> tuple[int, int, int]:
    start = text.index(signature)
    brace = text.index("{", start)
    depth = 0
    for index in range(brace, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return start, brace, index + 1
    raise RuntimeError(f"unterminated function: {signature}")


def replace_function(text: str, signature: str, replacement: str) -> str:
    start, _, end = function_span(text, signature)
    return text[:start] + replacement + text[end:]


def insert_after_brace(text: str, signature: str, code: str) -> str:
    _, brace, _ = function_span(text, signature)
    return text[: brace + 1] + code + text[brace + 1 :]


def insert_before_last(text: str, signature: str, needle: str, code: str) -> str:
    _, brace, end = function_span(text, signature)
    position = text.rfind(needle, brace, end)
    if position < 0:
        raise RuntimeError(f"missing insertion point in {signature}: {needle!r}")
    return text[:position] + code + text[position:]


def insert_after_function(text: str, signature: str, code: str) -> str:
    _, _, end = function_span(text, signature)
    return text[:end] + code + text[end:]


def write_and_commit(paths: list[Path], message: str) -> None:
    subprocess.run(["git", "diff", "--check"], check=True)
    subprocess.run(["git", "add", "--", *(str(path) for path in paths)], check=True)
    subprocess.run(["git", "commit", "-m", message], check=True)


# 1. Reject non-zero payload bits outside a requested partial byte.
driver = DRIVER.read_text()
if "unusedHighBits" not in driver:
    old = """\tfor (uint16_t i = 0; i < nibbleCount; i++) {
\t\tint v = hexNibbleValue(hex[i]);
\t\tuint16_t pos = nibbleOffset + i;
\t\tuint16_t byteIndex = pos / 2;
\t\tif ((pos & 1) == 0)
\t\t\tout[byteIndex] |= (uint8_t)(v << 4);
\t\telse
\t\t\tout[byteIndex] |= (uint8_t)v;
\t}
\t*outBytes = nbytes;
"""
    new = """\tfor (uint16_t i = 0; i < nibbleCount; i++) {
\t\tint v = hexNibbleValue(hex[i]);
\t\tuint16_t pos = nibbleOffset + i;
\t\tuint16_t byteIndex = pos / 2;
\t\tif ((pos & 1) == 0)
\t\t\tout[byteIndex] |= (uint8_t)(v << 4);
\t\telse
\t\t\tout[byteIndex] |= (uint8_t)v;
\t}

\t// State bytes are right-aligned. Reject any supplied high bits that
\t// fall outside the requested width rather than silently discarding them.
\tconst uint8_t unusedHighBits = (uint8_t)(nbytes * 8 - bits);
\tif (unusedHighBits) {
\t\tconst uint8_t unusedMask =
\t\t\t(uint8_t)(0xFFU << (8 - unusedHighBits));
\t\tif (out[0] & unusedMask) return false;
\t}

\t*outBytes = nbytes;
"""
    driver = replace_once(driver, old, new, "partial-byte payload validation")
    DRIVER.write_text(driver)
    write_and_commit([DRIVER], "ir: reject payload bits outside requested width")


# 2. Make IRtimer observe queued waveform time while constructing a send.
driver = DRIVER.read_text()
if "gIRUseVirtualMicros" not in driver:
    driver = replace_once(
        driver,
        "#define __FlashStringHelper char\n\n",
        """#define __FlashStringHelper char

// IRremoteESP8266 normally measures time spent in blocking mark()/space()
// calls. OpenBeken queues them, so expose queued duration while building a
// transaction.
static bool gIRUseVirtualMicros = false;
static uint32_t gIRVirtualMicros = 0;

static void IR_AdvanceVirtualMicros(const uint32_t usec) {
\tif (gIRUseVirtualMicros) gIRVirtualMicros += usec;
}

""",
        "virtual clock declarations",
    )

    driver = replace_once(
        driver,
        """unsigned long millis()
{
\treturn 0;
}
unsigned long micros()
{
\treturn 0;
}
""",
        """unsigned long millis()
{
\tif (gIRUseVirtualMicros) return gIRVirtualMicros / 1000;
\treturn 0;
}
unsigned long micros()
{
\tif (gIRUseVirtualMicros) return gIRVirtualMicros;
\treturn 0;
}
""",
        "Beken virtual clock",
    )

    driver = replace_once(
        driver,
        """unsigned long millis()
{
\treturn g_timeMs;
}
unsigned long micros()
{
\treturn g_timeMs * 1000;
}
""",
        """unsigned long millis()
{
\tif (gIRUseVirtualMicros) return gIRVirtualMicros / 1000;
\treturn g_timeMs;
}
unsigned long micros()
{
\tif (gIRUseVirtualMicros) return gIRVirtualMicros;
\treturn g_timeMs * 1000;
}
""",
        "non-Beken virtual clock",
    )

    driver = insert_before_last(
        driver,
        "\tbool beginSendTransaction()",
        "\t\treturn true;",
        "\t\tgIRVirtualMicros = 0;\n\t\tgIRUseVirtualMicros = true;\n",
    )
    driver = insert_after_brace(
        driver, "\tbool commitSendTransaction()", "\n\t\tgIRUseVirtualMicros = false;"
    )
    driver = insert_after_brace(
        driver, "\tvoid abortSendTransaction()", "\n\t\tgIRUseVirtualMicros = false;"
    )
    driver = insert_after_brace(
        driver, "\tvoid resetsendqueue()", "\n\t\tgIRUseVirtualMicros = false;"
    )
    driver = insert_after_brace(
        driver,
        "\tuint16_t mark(uint16_t aMarkMicros)",
        "\n\t\tIR_AdvanceVirtualMicros(aMarkMicros);",
    )
    driver = insert_after_brace(
        driver,
        "\tvoid space(uint32_t aMarkMicros)",
        "\n\t\tIR_AdvanceVirtualMicros(aMarkMicros);",
    )
    DRIVER.write_text(driver)
    write_and_commit([DRIVER], "ir: use queued time for frame duration")


# 3. Replace the unbounded floating-point receive clock.
driver = DRIVER.read_text()
receiver = RECEIVER.read_text()
if "IR_ISR_ResetClock" not in driver:
    driver = replace_once(
        driver,
        """extern "C" void DRV_IR_ISR(void* arg);
extern void IR_ISR(float period_us);
""",
        """extern "C" void DRV_IR_ISR(void* arg);
extern void IR_ISR(float period_us);
extern void IR_ISR_ResetClock(float period_us);
""",
        "RX reset declaration",
    )
    driver = replace_once(
        driver,
        """\tir_chan = HAL_RequestHWTimer(ir_periodus, &ir_periodus, DRV_IR_ISR, NULL);
\tADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer %u, %.2f us period", ir_chan, ir_periodus);
""",
        """\tir_chan = HAL_RequestHWTimer(ir_periodus, &ir_periodus, DRV_IR_ISR, NULL);
\tIR_ISR_ResetClock(ir_periodus);
\tADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer %u, %.2f us period", ir_chan, ir_periodus);
""",
        "RX reset call",
    )

    start = receiver.index("static float        ir_now = 0;")
    _, _, end = function_span(receiver, "void IR_ISR(float period_us)")
    replacement = """static uint32_t ir_sample_count = 0;
static uint32_t ir_edge_sample = 0;
static uint_fast8_t ir_old = 0;
static uint32_t ir_period_us_q16 = 0;
static uint32_t ir_timeout_samples = 1;

static uint32_t IR_PeriodToQ16(float period_us) {
  if (period_us < 1.0f) period_us = 1.0f;
  return (uint32_t)(period_us * 65536.0f + 0.5f);
}

void IR_ISR_ResetClock(float period_us) {
  ir_sample_count = 0;
  ir_edge_sample = 0;
  ir_period_us_q16 = IR_PeriodToQ16(period_us);
  const uint64_t timeout_q16 =
      (uint64_t)params.timeout * 1000U * 65536U;
  ir_timeout_samples = (uint32_t)(
      (timeout_q16 + ir_period_us_q16 - 1) / ir_period_us_q16);
  if (!ir_timeout_samples) ir_timeout_samples = 1;
}

void IR_ISR(float period_us) {
  if (!ir_period_us_q16) IR_ISR_ResetClock(period_us);
  ir_sample_count++;
  if (params.rcvstate == kStopState) return;

  const uint_fast8_t tIRInputLevel =
      (uint_fast8_t)digitalReadFast(params.recvpin);
  const uint32_t elapsed_samples = ir_sample_count - ir_edge_sample;

  if (elapsed_samples >= ir_timeout_samples && params.rawlen) {
    params.rcvstate = kStopState;
    return;
  }

  if (tIRInputLevel == ir_old) return;

  ir_old = tIRInputLevel;
  const uint16_t rawlen = params.rawlen;
  if (rawlen >= params.bufsize) {
    params.overflow = true;
    params.rcvstate = kStopState;
    return;
  }

  if (params.rcvstate == kIdleState) {
    params.rcvstate = kMarkState;
    params.rawbuf[rawlen] = 1;
  } else {
    const uint64_t elapsed_q16 =
        (uint64_t)elapsed_samples * ir_period_us_q16;
    uint32_t raw_ticks = (uint32_t)(
        elapsed_q16 / ((uint64_t)kRawTick * 65536U));
    if (!raw_ticks) raw_ticks = 1;
    if (raw_ticks > UINT16_MAX) raw_ticks = UINT16_MAX;
    params.rawbuf[rawlen] = (uint16_t)raw_ticks;
  }
  params.rawlen++;
  ir_edge_sample = ir_sample_count;
}"""
    receiver = receiver[:start] + replacement + receiver[end:]

    DRIVER.write_text(driver)
    RECEIVER.write_text(receiver)
    write_and_commit([DRIVER, RECEIVER], "ir: use integer ticks for receive timing")


# 4. Stage carrier settings with the complete waveform and suppress zero
# durations before they enter timing or queue logic.
driver = DRIVER.read_text()
if "transactionFrequencyHz" not in driver:
    driver = replace_once(
        driver,
        """\tmyIRsend(uint_fast8_t aSendPin) :IRsend(aSendPin) {
\t\tsendPin = aSendPin;
\t\tour_us = 0;
\t\tour_ms = 0;
\t\tresetsendqueue();
\t}
""",
        """\tmyIRsend(uint_fast8_t aSendPin) :IRsend(aSendPin) {
\t\tsendPin = aSendPin;
\t\tour_us = 0;
\t\tour_ms = 0;
\t\tpwmfrequency = 38000;
\t\tpwmduty = 50;
\t\ttransactionFrequencyHz = pwmfrequency;
\t\ttransactionDuty = (uint8_t)pwmduty;
\t\tresetsendqueue();
\t}
""",
        "sender constructor",
    )

    driver = replace_once(
        driver,
        """\t\ttransactionCount = 0;
\t\ttransactionFailed = false;
\t\toverflows = 0;
""",
        """\t\ttransactionCount = 0;
\t\ttransactionFailed = false;
\t\ttransactionFrequencyHz = pwmfrequency;
\t\ttransactionDuty = (uint8_t)pwmduty;
\t\toverflows = 0;
""",
        "carrier snapshot",
    )

    driver = replace_once(
        driver,
        """\tuint16_t mark(uint16_t aMarkMicros) {
\t\tIR_AdvanceVirtualMicros(aMarkMicros);
""",
        """\tuint16_t mark(uint16_t aMarkMicros) {
\t\tif (!aMarkMicros) return 0;
\t\tIR_AdvanceVirtualMicros(aMarkMicros);
""",
        "zero mark suppression",
    )
    driver = replace_once(
        driver,
        """\tvoid space(uint32_t aMarkMicros) {
\t\tIR_AdvanceVirtualMicros(aMarkMicros);
""",
        """\tvoid space(uint32_t aMarkMicros) {
\t\tif (!aMarkMicros) return;
\t\tIR_AdvanceVirtualMicros(aMarkMicros);
""",
        "zero space suppression",
    )

    carrier = """\tvoid enableIROut(uint32_t freq, uint8_t duty=50) {
\t\tif (freq < 1000)  // Were we given kHz? Supports old call usage.
\t\t\tfreq *= 1000;
\t\tADDLOG_INFO(LOG_FEATURE_IR, (char *)"enableIROut %d freq %d duty",
\t\t\t(int)freq, (int)duty);
\t\tif (duty < 1) duty = 1;
\t\tif (duty > 100) duty = 100;

\t\tif (transactionBuilding) {
\t\t\ttransactionFrequencyHz = freq;
\t\t\ttransactionDuty = duty;
\t\t\treturn;
\t\t}

\t\tpwmfrequency = freq;
\t\tpwmduty = duty;
\t\tHAL_PIN_PWM_Start(sendPin, pwmfrequency);
\t}"""
    driver = replace_function(
        driver, "\tvoid enableIROut(uint32_t freq, uint8_t duty=50)", carrier
    )

    driver = replace_once(
        driver,
        """\t\ttimecounttotal = transactionCount;
\t\ttransactionReady = 1;
""",
        """\t\ttimecounttotal = transactionCount;
\t\tpwmfrequency = transactionFrequencyHz;
\t\tpwmduty = transactionDuty;
\t\tHAL_PIN_PWM_Start(sendPin, pwmfrequency);
\t\ttransactionReady = 1;
""",
        "carrier commit",
    )

    driver = replace_once(
        driver,
        """\tuint8_t sendPin;
\tuint32_t pwmduty;
""",
        """\tuint8_t sendPin;
\tuint32_t pwmduty;
\tuint32_t pwmfrequency;
\tuint32_t transactionFrequencyHz;
\tuint8_t transactionDuty;
""",
        "carrier members",
    )

    DRIVER.write_text(driver)
    write_and_commit([DRIVER], "ir: stage carrier settings with send transaction")


# 5. Replay one staged state waveform for requested repeats, and honour
# the repeat argument in the classic command form.
driver = DRIVER.read_text()
if "transactionRepeatsRemaining" not in driver:
    hex_parser = """

static bool parseBoundedHex(const char *text, const uint32_t minValue,
\tconst uint32_t maxValue, uint32_t *result) {
\tif (!text || !text[0] || !result || minValue > maxValue) return false;

\tuint32_t value = 0;
\tfor (const char *cursor = text; *cursor; cursor++) {
\t\tuint32_t digit;
\t\tif (*cursor >= '0' && *cursor <= '9')
\t\t\tdigit = (uint32_t)(*cursor - '0');
\t\telse if (*cursor >= 'a' && *cursor <= 'f')
\t\t\tdigit = 10U + (uint32_t)(*cursor - 'a');
\t\telse if (*cursor >= 'A' && *cursor <= 'F')
\t\t\tdigit = 10U + (uint32_t)(*cursor - 'A');
\t\telse
\t\t\treturn false;

\t\tif (value > maxValue / 16 ||
\t\t\t(value == maxValue / 16 && digit > maxValue % 16)) {
\t\t\treturn false;
\t\t}
\t\tvalue = value * 16 + digit;
\t}
\tif (value < minValue) return false;

\t*result = value;
\treturn true;
}
"""
    driver = insert_after_function(driver, "static bool parseBoundedDecimal", hex_parser)

    driver = replace_once(
        driver,
        """\t\ttransactionCount = 0;
\t\ttransactionFailed = false;
\t\ttransactionFrequencyHz = pwmfrequency;
""",
        """\t\ttransactionCount = 0;
\t\ttransactionRepeatCount = 0;
\t\ttransactionFailed = false;
\t\ttransactionFrequencyHz = pwmfrequency;
""",
        "repeat count reset on begin",
    )

    repeat_method = """

\tbool setTransactionRepeats(const uint16_t repeats) {
\t\tif (!transactionBuilding || transactionFailed) return false;
\t\ttransactionRepeatCount = repeats;
\t\treturn true;
\t}"""
    driver = insert_after_function(
        driver, "\tuint16_t getStagedItemCount() const", repeat_method
    )

    driver = replace_once(
        driver,
        """\t\ttimeout = 0;
\t\ttimein = transactionCount;
\t\ttimecount = transactionCount;
\t\ttimecounttotal = transactionCount;
""",
        """\t\ttimeout = 0;
\t\ttimein = transactionCount;
\t\ttransactionRepeatsRemaining = transactionRepeatCount;
\t\tconst uint32_t totalItems =
\t\t\t(uint32_t)transactionCount * (transactionRepeatCount + 1U);
\t\ttimecount = (uint16_t)totalItems;
\t\ttimecounttotal = totalItems;
""",
        "repeat publication",
    )

    driver = replace_once(
        driver,
        """\t\ttransactionReady = 1;
\t\ttransactionBuilding = 0;
\t\treturn true;
""",
        """\t\ttransactionReady = 1;
\t\ttransactionBuilding = 0;
\t\ttransactionRepeatCount = 0;
\t\treturn true;
""",
        "repeat commit cleanup",
    )

    driver = replace_once(
        driver,
        """\t\ttransactionBuilding = 0;
\t\ttransactionFailed = false;
\t\ttransactionCount = 0;
""",
        """\t\ttransactionBuilding = 0;
\t\ttransactionFailed = false;
\t\ttransactionCount = 0;
\t\ttransactionRepeatCount = 0;
\t\ttransactionRepeatsRemaining = 0;
""",
        "repeat abort cleanup",
    )

    driver = replace_once(
        driver,
        """\t\ttransactionFailed = false;
\t\ttransactionCount = 0;
\t\ttimein = timeout = 0;
""",
        """\t\ttransactionFailed = false;
\t\ttransactionCount = 0;
\t\ttransactionRepeatCount = 0;
\t\ttransactionRepeatsRemaining = 0;
\t\ttimein = timeout = 0;
""",
        "repeat reset cleanup",
    )

    dequeue = """\tbool getsendqueue(int32_t *value) {
\t\tif (!value || !transactionReady) return false;

\t\tif (timeout >= timein) {
\t\t\tif (transactionRepeatsRemaining) {
\t\t\t\ttransactionRepeatsRemaining--;
\t\t\t\ttimeout = 0;
\t\t\t} else {
\t\t\t\ttransactionReady = 0;
\t\t\t\ttimein = timeout = 0;
\t\t\t\ttimecount = 0;
\t\t\t\treturn false;
\t\t\t}
\t\t}

\t\t*value = times[timeout++];
\t\tif (timecount) timecount--;
\t\treturn true;
\t}"""
    driver = replace_function(driver, "\tbool getsendqueue(int32_t *value)", dequeue)

    driver = replace_once(
        driver,
        """\tbool transactionFailed;
\tuint16_t transactionCount;
""",
        """\tbool transactionFailed;
\tuint16_t transactionCount;
\tuint16_t transactionRepeatCount;
\tvolatile uint16_t transactionRepeatsRemaining;
""",
        "repeat members",
    )

    old_state_send = """\t\t\t\t\t\tbool sent = true;
\t\t\t\t\t\tfor (int repeatIndex = 0; repeatIndex <= repeats; repeatIndex++) {
\t\t\t\t\t\t\tconst uint16_t stagedBefore = pIRsend->getStagedItemCount();
\t\t\t\t\t\t\tif (!pIRsend->send(protocol,state,nbytes) ||
\t\t\t\t\t\t\t\tpIRsend->getStagedItemCount() == stagedBefore) {
\t\t\t\t\t\t\t\tsent = false;
\t\t\t\t\t\t\t\tbreak;
\t\t\t\t\t\t\t}
\t\t\t\t\t\t\tif (repeatIndex < repeats) {
\t\t\t\t\t\t\t\tpIRsend->delay(100);
\t\t\t\t\t\t\t}
\t\t\t\t\t\t}
\t\t\t\t\t\tif (sent) {
\t\t\t\t\t\t\tpIRsend->delay(100);
\t\t\t\t\t\t}
"""
    new_state_send = """\t\t\t\t\t\tconst uint16_t stagedBefore =
\t\t\t\t\t\t\tpIRsend->getStagedItemCount();
\t\t\t\t\t\tbool sent = pIRsend->send(protocol, state, nbytes) &&
\t\t\t\t\t\t\tpIRsend->getStagedItemCount() > stagedBefore;
\t\t\t\t\t\tif (sent) {
\t\t\t\t\t\t\tpIRsend->delay(100);
\t\t\t\t\t\t\tsent = pIRsend->setTransactionRepeats(
\t\t\t\t\t\t\t\t(uint16_t)repeats);
\t\t\t\t\t\t}
"""
    driver = replace_once(driver, old_state_send, new_state_send, "state transaction replay")

    old_repeat_parse = """\tint repeats = 0;

\tif ((*p == '-') || (*p == ' ')) {
\t\tp++;
\t\trepeats = strtol(p, &p, 16);
\t}
"""
    new_repeat_parse = """\tint repeats = 0;

\tif (*p) {
\t\tif ((*p != '-') && (*p != ' ')) {
\t\t\tADDLOG_ERROR(LOG_FEATURE_IR,
\t\t\t\t(char *)"IRSend invalid classic repeat syntax in %s", args);
\t\t\treturn CMD_RES_BAD_ARGUMENT;
\t\t}
\t\tp++;
\t\tuint32_t repeatValue = 0;
\t\tif (!parseBoundedHex(p, 0, kIRSendMaxRepeats, &repeatValue)) {
\t\t\tADDLOG_ERROR(LOG_FEATURE_IR,
\t\t\t\t(char *)"IRSend invalid repeat count '%s' (expected 0-%X)",
\t\t\t\tp, (unsigned int)kIRSendMaxRepeats);
\t\t\treturn CMD_RES_BAD_ARGUMENT;
\t\t}
\t\trepeats = (int)repeatValue;
\t}
"""
    driver = replace_once(driver, old_repeat_parse, new_repeat_parse, "classic repeat parser")

    calls = {
        "pIRsend->sendRC5((uint64_t)pIRsend->encodeRC5(addr,command));":
            "pIRsend->sendRC5((uint64_t)pIRsend->encodeRC5(addr,command), kRC5XBits, repeats);",
        "pIRsend->sendRC6((uint64_t)pIRsend->encodeRC6(addr,command));":
            "pIRsend->sendRC6((uint64_t)pIRsend->encodeRC6(addr,command), kRC6Mode0Bits, repeats);",
        "pIRsend->sendNEC((uint64_t)pIRsend->encodeNEC(addr,command));":
            "pIRsend->sendNEC((uint64_t)pIRsend->encodeNEC(addr,command), kNECBits, repeats);",
        "pIRsend->sendPanasonic((uint16_t)addr,(uint32_t)command);":
            "pIRsend->sendPanasonic((uint16_t)addr,(uint32_t)command, kPanasonicBits, repeats);",
        "pIRsend->sendJVC((uint64_t)pIRsend->encodeJVC(addr,command));":
            "pIRsend->sendJVC((uint64_t)pIRsend->encodeJVC(addr,command), kJvcBits, repeats);",
        "pIRsend->sendSAMSUNG((uint64_t)pIRsend->encodeSAMSUNG(addr,command));":
            "pIRsend->sendSAMSUNG((uint64_t)pIRsend->encodeSAMSUNG(addr,command), kSamsungBits, repeats);",
        "pIRsend->sendLG((uint64_t)pIRsend->encodeLG(addr,command));":
            "pIRsend->sendLG((uint64_t)pIRsend->encodeLG(addr,command), kLgBits, repeats);",
    }
    for old, new in calls.items():
        driver = replace_once(driver, old, new, f"classic repeat call {old}")

    DRIVER.write_text(driver)
    write_and_commit([DRIVER], "ir: replay state transactions and honor classic repeats")


# 6. Add focused adapter regression coverage.
test_path = Path("tests/ir_adapter_regression.py")
workflow_path = Path(".github/workflows/ir-adapter-regression.yml")
if not test_path.exists():
    test_path.parent.mkdir(parents=True, exist_ok=True)
    workflow_path.parent.mkdir(parents=True, exist_ok=True)

    test_source = r'''#!/usr/bin/env python3
"""Focused behavioural and source-contract tests for the OpenBeken IR adapter."""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DRIVER_PATH = ROOT / "src/driver/drv_ir_new.cpp"
RECV_PATH = ROOT / "src/libraries/IRremoteESP8266/src/IRrecv.cpp"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def function_block(source: str, signature: str) -> str:
    start = source.find(signature)
    require(start >= 0, f"missing function signature: {signature}")
    brace = source.find("{", start)
    require(brace >= 0, f"missing function body: {signature}")
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def compile_and_run_parser_tests(driver: str) -> None:
    compiler = shutil.which("c++") or shutil.which("g++")
    require(compiler is not None, "a C++ compiler is required")

    functions = "\n\n".join(
        function_block(driver, signature)
        for signature in (
            "static int hexNibbleValue",
            "static bool parseBoundedDecimal",
            "static bool parseBoundedHex",
            "static bool parseHexStateBytes",
        )
    )
    harness = f"""
#include <cassert>
#include <cstdint>
#include <cstring>

{functions}

int main() {{
  uint32_t value = 0;
  assert(parseBoundedDecimal("1", 1, 512, &value) && value == 1);
  assert(parseBoundedDecimal("512", 1, 512, &value) && value == 512);
  assert(parseBoundedDecimal("0", 0, 10, &value) && value == 0);
  assert(parseBoundedDecimal("10", 0, 10, &value) && value == 10);
  assert(!parseBoundedDecimal("", 1, 512, &value));
  assert(!parseBoundedDecimal("513", 1, 512, &value));
  assert(!parseBoundedDecimal("-1", 0, 10, &value));
  assert(!parseBoundedDecimal("32junk", 1, 512, &value));

  assert(parseBoundedHex("A", 0, 10, &value) && value == 10);
  assert(!parseBoundedHex("10", 0, 10, &value));
  assert(!parseBoundedHex("-1", 0, 10, &value));
  assert(!parseBoundedHex("Ajunk", 0, 10, &value));

  uint8_t bytes[64] = {{0}};
  uint16_t nbytes = 0;
  const char *end = nullptr;

  assert(parseHexStateBytes("F", 4, bytes, sizeof(bytes), &nbytes, &end));
  assert(nbytes == 1 && bytes[0] == 0x0F && *end == '\\0');

  std::memset(bytes, 0, sizeof(bytes));
  assert(!parseHexStateBytes("F0", 4, bytes, sizeof(bytes), &nbytes, &end));

  std::memset(bytes, 0, sizeof(bytes));
  assert(parseHexStateBytes("ABC", 12, bytes, sizeof(bytes), &nbytes, &end));
  assert(nbytes == 2 && bytes[0] == 0x0A && bytes[1] == 0xBC);

  std::memset(bytes, 0, sizeof(bytes));
  assert(!parseHexStateBytes("FABC", 12, bytes, sizeof(bytes), &nbytes, &end));

  std::memset(bytes, 0, sizeof(bytes));
  assert(parseHexStateBytes(
      "FFFFFFFFFFFFFFFFFFFFF", 84, bytes, sizeof(bytes), &nbytes, &end));
  assert(nbytes == 11 && bytes[0] == 0x0F);

  std::memset(bytes, 0, sizeof(bytes));
  assert(!parseHexStateBytes(
      "FFFFFFFFFFFFFFFFFFFFFF", 84, bytes, sizeof(bytes), &nbytes, &end));

  std::memset(bytes, 0, sizeof(bytes));
  assert(parseHexStateBytes(
      "0011223344556677", 56, bytes, sizeof(bytes), &nbytes, &end));
  const uint8_t expected[] = {{0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77}};
  assert(nbytes == sizeof(expected));
  assert(std::memcmp(bytes, expected, sizeof(expected)) == 0);

  assert(!parseHexStateBytes("12G4", 16, bytes, sizeof(bytes), &nbytes, &end));
  assert(!parseHexStateBytes("0", 0, bytes, sizeof(bytes), &nbytes, &end));
  assert(!parseHexStateBytes("0", 513, bytes, sizeof(bytes), &nbytes, &end));
  return 0;
}}
"""

    with tempfile.TemporaryDirectory(prefix="ir-adapter-test-") as temp_dir:
        temp = Path(temp_dir)
        source_path = temp / "parser_test.cpp"
        binary_path = temp / "parser_test"
        source_path.write_text(harness)
        subprocess.run(
            [
                compiler,
                "-std=c++11",
                "-Wall",
                "-Wextra",
                "-Werror",
                str(source_path),
                "-o",
                str(binary_path),
            ],
            check=True,
        )
        subprocess.run([str(binary_path)], check=True)


def check_transaction_contracts(driver: str) -> None:
    begin = function_block(driver, "bool beginSendTransaction()")
    commit = function_block(driver, "bool commitSendTransaction()")
    abort = function_block(driver, "void abortSendTransaction()")
    mark = function_block(driver, "uint16_t mark(uint16_t aMarkMicros)")
    space = function_block(driver, "void space(uint32_t aMarkMicros)")
    carrier = function_block(
        driver, "void enableIROut(uint32_t freq, uint8_t duty=50)"
    )
    dequeue = function_block(driver, "bool getsendqueue(int32_t *value)")
    repeat_setter = function_block(driver, "bool setTransactionRepeats")

    for token in (
        "transactionBuilding",
        "transactionReady",
        "currentsendtime",
        "transactionRepeatCount = 0",
        "transactionFrequencyHz = pwmfrequency",
        "transactionDuty = (uint8_t)pwmduty",
        "gIRUseVirtualMicros = true",
    ):
        require(token in begin, f"begin transaction contract missing: {token}")

    require("gIRUseVirtualMicros = false" in commit, "commit leaves virtual time enabled")
    require("gIRUseVirtualMicros = false" in abort, "abort leaves virtual time enabled")
    require("transactionRepeatsRemaining = transactionRepeatCount" in commit,
            "repeat count is not published with the waveform")
    require("HAL_PIN_PWM_Start(sendPin, pwmfrequency)" in commit,
            "commit does not apply the staged carrier")
    require(
        commit.index("HAL_PIN_PWM_Start(sendPin, pwmfrequency)")
        < commit.index("transactionReady = 1"),
        "waveform is visible before the carrier is applied",
    )

    require(mark.index("if (!aMarkMicros) return 0") <
            mark.index("IR_AdvanceVirtualMicros"),
            "zero marks enter timing or queue logic")
    require(space.index("if (!aMarkMicros) return") <
            space.index("IR_AdvanceVirtualMicros"),
            "zero spaces enter timing or queue logic")

    require("if (transactionBuilding)" in carrier,
            "carrier changes are not staged")
    require("transactionFrequencyHz = freq" in carrier,
            "staged frequency is missing")
    require("transactionDuty = duty" in carrier,
            "staged duty is missing")
    require("transactionRepeatCount = repeats" in repeat_setter,
            "repeat setter does not stage the count")

    require("if (transactionRepeatsRemaining)" in dequeue,
            "dequeue does not replay staged state frames")
    require("transactionRepeatsRemaining--" in dequeue,
            "dequeue does not consume the repeat count")
    require("*value = times[timeout++]" in dequeue,
            "dequeue does not return the queued duration")


def check_validation_contracts(driver: str) -> None:
    validator = function_block(driver, "static bool isValidStatePayloadLength")
    for protocol in (
        "CARRIER_AC84",
        "ARGO",
        "CORONA_AC",
        "DAIKIN",
        "FUJITSU_AC",
        "HITACHI_AC3",
        "PANASONIC_AC",
        "SAMSUNG_AC",
        "TOSHIBA_AC",
        "MWM",
    ):
        require(protocol in validator, f"missing state-length rule for {protocol}")
    require("IRsend::defaultBits" in validator, "fixed-length fallback is missing")
    require("nbytes != (bits + 7) / 8" in validator,
            "bit/byte consistency check is missing")

    command = function_block(
        driver, 'extern "C" IR_SEND_CMD_OPT commandResult_t IR_Send_Cmd'
    )
    require("parseBoundedDecimal(_bits, 1, kIRSendMaxBits" in command,
            "bit count is not strictly bounded")
    require("parseBoundedDecimal(payloadEnd + 1, 0, kIRSendMaxRepeats" in command,
            "comma repeat count is not strictly bounded")
    require("parseBoundedHex(p, 0, kIRSendMaxRepeats" in command,
            "classic repeat count is not strictly bounded")
    require("getStagedItemCount() > stagedBefore" in command,
            "upstream no-op state senders can report success")
    require("setTransactionRepeats" in command,
            "state repeats are not delegated to queue replay")
    for token in (
        "sendRC5",
        "sendRC6",
        "sendNEC",
        "sendPanasonic",
        "sendJVC",
        "sendSAMSUNG",
        "sendLG",
    ):
        line = next(line for line in command.splitlines() if token in line)
        require("repeats" in line, f"classic repeat is ignored by {token}")


def check_receive_clock_contracts(driver: str, receiver: str) -> None:
    require("IR_ISR_ResetClock(ir_periodus)" in driver,
            "receiver clock is not reset from the actual timer period")
    require("static float        ir_now" not in receiver,
            "unbounded floating-point receive clock returned")
    for token in (
        "ir_sample_count",
        "ir_edge_sample",
        "ir_period_us_q16",
        "ir_timeout_samples",
        "ir_sample_count - ir_edge_sample",
        "void IR_ISR_ResetClock(float period_us)",
    ):
        require(token in receiver, f"missing integer receive-clock element: {token}")


def main() -> None:
    driver = DRIVER_PATH.read_text()
    receiver = RECV_PATH.read_text()
    compile_and_run_parser_tests(driver)
    check_transaction_contracts(driver)
    check_validation_contracts(driver)
    check_receive_clock_contracts(driver, receiver)
    print("IR adapter regression tests passed")


if __name__ == "__main__":
    main()
'''
    workflow_source = '''name: IR Adapter Regression

on:
  push:
    branches:
      - irremotesp8266_290
    paths:
      - src/driver/drv_ir_new.cpp
      - src/libraries/IRremoteESP8266/src/IRrecv.cpp
      - tests/ir_adapter_regression.py
      - .github/workflows/ir-adapter-regression.yml
  workflow_dispatch:

permissions:
  contents: read

jobs:
  regression:
    runs-on: ubuntu-latest
    steps:
      - name: Check out source
        uses: actions/checkout@v4
      - name: Run IR adapter regression tests
        run: python3 tests/ir_adapter_regression.py
'''
    test_path.write_text(textwrap.dedent(test_source))
    workflow_path.write_text(textwrap.dedent(workflow_source))
    write_and_commit(
        [test_path, workflow_path],
        "test: cover IR adapter parsing and transaction contracts",
    )


subprocess.run(["python3", str(test_path)], check=True)
subprocess.run(["git", "diff", "--check"], check=True)
subprocess.run(
    [
        "git",
        "log",
        "--oneline",
        "--reverse",
        "c7dbf61ed8ae2680ce311e20189a22d034697896..HEAD",
    ],
    check=True,
)
