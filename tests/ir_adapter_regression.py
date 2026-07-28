#!/usr/bin/env python3
"""Focused behavioural and source-contract tests for OpenBeken's IR adapter."""
from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FILES = {
    "driver": ROOT / "src/driver/drv_ir_new.cpp",
    "recv": ROOT / "src/libraries/IRremoteESP8266/src/IRrecv.cpp",
    "digital": ROOT / "src/libraries/IRremoteESP8266/src/digitalWriteFast.cpp",
    "tiny": ROOT / "src/driver/drv_tinyir_nec.c",
    "bl602": ROOT / "src/hal/bl602/hal_pins_bl602.c",
    "ln": ROOT / "src/hal/ln882h/hal_pins_ln882h.c",
    "realtek": ROOT / "src/hal/realtek/hal_pins_realtek.c",
}


def require(ok: bool, message: str) -> None:
    if not ok:
        raise AssertionError(message)


def block(source: str, signature: str) -> str:
    start = source.find(signature)
    require(start >= 0, f"missing function: {signature}")
    brace = source.find("{", start)
    require(brace >= 0, f"missing function body: {signature}")
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def flat(source: str) -> str:
    return re.sub(r"\s+", " ", source)


def contains(source: str, *tokens: str) -> None:
    for token in tokens:
        require(token in source, f"missing contract token: {token}")


def parser_tests(driver: str) -> None:
    compiler = shutil.which("c++") or shutil.which("g++")
    require(compiler is not None, "C++ compiler unavailable")
    functions = "\n\n".join(block(driver, signature) for signature in (
        "static int hexNibbleValue",
        "static bool parseBoundedDecimal",
        "static bool parseBoundedHex",
        "static bool parseHexStateBytes",
    ))
    harness = f'''#include <cassert>\n#include <cstdint>\n#include <cstring>\n{functions}\n
int main() {{
  uint32_t value = 0;
  assert(parseBoundedDecimal("1", 1, 512, &value) && value == 1);
  assert(parseBoundedDecimal("512", 1, 512, &value) && value == 512);
  assert(parseBoundedDecimal("10", 0, 10, &value) && value == 10);
  assert(!parseBoundedDecimal("-1", 0, 10, &value));
  assert(!parseBoundedDecimal("32junk", 1, 512, &value));
  assert(parseBoundedHex("0xA", 0, 10, &value) && value == 10);
  assert(!parseBoundedHex("0x", 0, 10, &value));
  assert(!parseBoundedHex("10", 0, 10, &value));
  uint8_t bytes[64] = {{0}};
  uint16_t count = 0;
  const char *end = nullptr;
  assert(parseHexStateBytes("F", 4, bytes, sizeof(bytes), &count, &end));
  assert(count == 1 && bytes[0] == 0x0F && *end == '\\0');
  assert(!parseHexStateBytes("F0", 4, bytes, sizeof(bytes), &count, &end));
  std::memset(bytes, 0, sizeof(bytes));
  assert(parseHexStateBytes("ABC", 12, bytes, sizeof(bytes), &count, &end));
  assert(count == 2 && bytes[0] == 0x0A && bytes[1] == 0xBC);
  assert(!parseHexStateBytes("FABC", 12, bytes, sizeof(bytes), &count, &end));
  std::memset(bytes, 0, sizeof(bytes));
  assert(parseHexStateBytes("0011223344556677", 56, bytes, sizeof(bytes), &count, &end));
  const uint8_t expected[] = {{0x11,0x22,0x33,0x44,0x55,0x66,0x77}};
  assert(count == sizeof(expected));
  assert(std::memcmp(bytes, expected, sizeof(expected)) == 0);
  assert(!parseHexStateBytes("12G4", 16, bytes, sizeof(bytes), &count, &end));
  assert(!parseHexStateBytes("0", 513, bytes, sizeof(bytes), &count, &end));
}}\n'''
    with tempfile.TemporaryDirectory(prefix="obk-ir-test-") as directory:
        src = Path(directory) / "parser.cpp"
        exe = Path(directory) / "parser"
        src.write_text(harness)
        subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
                        str(src), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


def source_contracts(s: dict[str, str]) -> None:
    d = s["driver"]
    contains(block(d, "bool beginSendTransaction()"),
             "if (isBusy())", "transactionInverted = pwmInverted",
             "gIRUseVirtualMicros = true")
    commit = block(d, "bool commitSendTransaction()")
    contains(commit, "IR_PlatformPWMReserve(sendPin)",
             "IR_PlatformPWMIsActive(sendPin)", "applyCarrierState(false, true)",
             "IR_COMPILER_BARRIER()", "transactionReady = 1")
    require(commit.index("IR_COMPILER_BARRIER()") < commit.index("transactionReady = 1"),
            "queue ready is published before the barrier")
    contains(block(d, "bool isBusy() const"), "carrierReleasePending")
    carrier = block(d, "void applyCarrierState")
    contains(carrier, "pwmInverted ? 100 - (int)pwmduty",
             "pwmInverted ? 100 : 0", "lastCarrierMark", "lastCarrierDuty",
             "IR_PlatformPWMUpdate")
    contains(block(d, "void serviceCarrier()"),
             "HAL_PIN_PWM_Stop(sendPin)", "IR_PlatformPWMRelease(sendPin)")
    isr = block(d, 'extern "C" void DRV_IR_ISR(void* arg)\n{')
    contains(isr, "applyCarrierState")
    require("HAL_PIN_PWM_Update" not in isr, "ISR bypasses IR PWM abstraction")

    validator = flat(block(d, "static bool isValidStatePayloadLength"))
    contains(validator, "CARRIER_AC84", "ARGO", "CORONA_AC", "DAIKIN",
             "FUJITSU_AC", "HITACHI_AC3", "PANASONIC_AC", "SAMSUNG_AC",
             "TOSHIBA_AC", "MWM", "IRsend::defaultBits")
    contains(validator, "bits == 16U || bits == 32U || bits == 48U",
             "bits == 56U || bits == 72U || bits == 80U",
             "bits >= 24U && bits <= kIRSendMaxBits")

    contains(d, "kIRArgoWrem3Preamble = 0x0BU")
    require("kArgoWrem3Preamble" not in d,
            "driver depends on a private ir_Argo.cpp constant")

    comma = block(d, "static commandResult_t IR_SendCommaCommand")
    contains(comma, "parseBoundedDecimal(bitsText, 1, kIRSendMaxBits",
             "parseBoundedDecimal(payloadEnd + 1, 0, kIRSendMaxRepeats",
             "IR_IsValidArgoPayload", "sendArgoWREM3",
             "getStagedItemCount() > stagedBefore", "setTransactionRepeats")
    classic = block(d, "static commandResult_t IR_SendClassicCommand")
    contains(classic, "IR_ValidateClassicFields", "parseBoundedHex(fields[1]",
             "parseBoundedHex(fields[2]", "sendRC5", "sendRC6", "sendNEC",
             "sendPanasonic", "sendJVC", "sendSAMSUNG", "sendLG")
    contains(block(d, "static bool IR_ProtocolTxTimingSupported"), "RCMM", "LEGOPF")

    init = block(d, 'extern "C" void DRV_IR_Init')
    contains(init, "IR_RegisterCommands()", "transmitPin >= 0",
             "new (std::nothrow) IRrecv", "new (std::nothrow) myIRsend",
             "IR_ReceiverStorageReady()", "IR_SyncReceiverInput(false)",
             "gIRDriverReady = true")
    deinit = block(d, 'extern "C" void DRV_IR_Deinit')
    contains(deinit, "gIRDriverReady = false", "sender->stopAndReleaseResources()", "delete sender",
             "delete receiver", "HAL_HWTimerDeinit(ir_chan)", "ir_chan = -1")
    run = block(d, 'extern "C" void DRV_IR_RunFrame')
    contains(run, "char logText[256] = { 0 }", "const bool stateResult",
             "if (eventType)", "IR_SyncReceiverInput(true)", "* ir_periodus")
    contains(block(d, 'extern "C" commandResult_t IR_Enable'),
             "parseBoundedDecimal(words[1], 0, 1", "protocol masks are not implemented")
    contains(block(d, 'extern "C" commandResult_t IR_Param'),
             "parseBoundedDecimal(words[1], 0, 100")
    contains(block(d, 'extern "C" commandResult_t IR_AC_Cmd'), "return CMD_RES_ERROR;")

    recv = s["recv"]
    require("static float        ir_now" not in recv, "floating absolute RX clock returned")
    contains(recv, "ir_sample_count", "ir_edge_sample", "ir_period_us_q16",
             "ir_timeout_samples", "ir_sample_count - ir_edge_sample")

    pinmode = flat(block(s["digital"], "void pinModeFast"))
    contains(pinmode, "INPUT_PULLUP: HAL_PIN_Setup_Input_Pullup",
             "INPUT_PULLDOWN: HAL_PIN_Setup_Input_Pulldown")
    contains(s["tiny"], "TinyIR_NEC_IsReady", "DRV_IR_IsReady()",
             "gTinyIRDeferredStart", "if(ir_chan < 0)")
    contains(s["bl602"], "g_bl602_pwm_owner", "g_bl602_pwm_owner[pwm] != index",
             "PWM_SW_Force_Value", "PWM_SW_Mode", "HAL_IR_PWM_IsActive")
    contains(s["ln"], "const uint8_t channel", "BIT_CLEAR(g_active_pwm, channel)",
             "if(freecha >= PWM_CH_MAX)",
             "LL_PWM_Compare_Set(pin->pwm_cha, g_pwm_load[pin->pwm_cha])",
             "LL_PWM_Compare_Set(pin->pwm_cha, 0)")
    allocator = block(s["realtek"], "static int HAL_RTK_GetFreeChannel")
    contains(allocator, "OBK_REALTEK_PWM_CHANNEL_COUNT", "return -1")
    contains(s["realtek"], "if(pin->gpio == NULL)", "if(pin->pwm == NULL)",
             "if(rtl_cf->irq == NULL)", "HAL_IR_PWM_Reserve",
             "Realtek_SharesPWMPeriodTimer", "Realtek_HasOtherPWMOwner",
             "g_realtek_ir_pwm_pin")


def main() -> None:
    sources = {name: path.read_text() for name, path in FILES.items()}
    parser_tests(sources["driver"])
    source_contracts(sources)
    print("IR adapter regression tests passed")


if __name__ == "__main__":
    main()
