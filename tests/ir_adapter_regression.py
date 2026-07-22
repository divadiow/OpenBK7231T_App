#!/usr/bin/env python3
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
        "if (isBusy())",
        "transactionRepeatCount = 0",
        "transactionFrequencyHz = pwmfrequency",
        "transactionDuty = (uint8_t)pwmduty",
        "gIRUseVirtualMicros = true",
    ):
        require(token in begin, f"begin transaction contract missing: {token}")
    require(
        "return transactionBuilding || transactionReady || currentsendtime;"
        in driver,
        "busy state does not cover building, queued, and active sends",
    )

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
