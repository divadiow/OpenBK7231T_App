#!/usr/bin/env python3
"""Fail a diagnostic XR809 build if its flash/XIP safety invariants regress."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


RAM_START = 0x00010000
RAM_END = 0x00070000
XIP_START = 0x10000000
XIP_END = 0x10100000
OTA_IMAGE_LIMIT = 950272

SRAM_SYMBOLS = (
    "HAL_Flash_Open",
    "HAL_Flash_Write",
    "HAL_Flash_Erase",
    "HAL_Flashc_Xip_RawDisable",
    "HAL_Flashc_Xip_RawEnable",
    "flash_erase",
    "ducc_app_ioctl",
    "TIMER0_IRQHandler",
    "TIMER1_IRQHandler",
    "HAL_WDG_Feed",
    "XRST_TimerCallback",
    "XRST_FlashTask",
)

XIP_SYMBOLS = ("XRST_XIPTask",)


def parse_number(value: str) -> int:
    return int(value, 16 if value.lower().startswith("0x") else 10)


def find_section(text: str, name: str) -> tuple[int, int]:
    match = re.search(
        rf"(?m)^\.{re.escape(name)}\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)",
        text,
    )
    if not match:
        raise ValueError(f"section .{name} was not found")
    return parse_number(match.group(1)), parse_number(match.group(2))


def find_symbol(text: str, symbol: str) -> int | None:
    matches = re.findall(
        rf"(?m)^\s*(0x[0-9a-fA-F]+)\s+{re.escape(symbol)}\s*$", text
    )
    values = [parse_number(value) for value in matches if parse_number(value) != 0]
    return values[0] if values else None


def find_assignment(text: str, symbol: str) -> int | None:
    match = re.search(
        rf"(?m)^\s*(0x[0-9a-fA-F]+)\s+{re.escape(symbol)}\s+=", text
    )
    return parse_number(match.group(1)) if match else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_file", type=Path)
    parser.add_argument("image_file", type=Path)
    parser.add_argument("--json", dest="json_file", type=Path)
    args = parser.parse_args()

    text = args.map_file.read_text(encoding="utf-8", errors="replace")
    failures: list[str] = []

    xip_start, xip_size = find_section(text, "xip")
    text_start, text_size = find_section(text, "text")
    data_start, data_size = find_section(text, "data")
    bss_start, bss_size = find_section(text, "bss")

    xip_block_match = re.search(r"(?ms)^\.xip\s+.*?(?=^\.text\s+)", text)
    if not xip_block_match:
        failures.append("unable to isolate .xip map block")
        forbidden_xip_lines: list[str] = []
    else:
        forbidden_xip_lines = [
            line.strip()
            for line in xip_block_match.group(0).splitlines()
            if re.search(r"0x0*10[0-9a-fA-F]+", line)
            and ("libxrsys.a" in line or "libchip.a" in line)
        ]
        if forbidden_xip_lines:
            failures.append("libxrsys/libchip has live content in .xip")

    symbols: dict[str, dict[str, object]] = {}
    for symbol in SRAM_SYMBOLS:
        address = find_symbol(text, symbol)
        ok = address is not None and RAM_START <= address < RAM_END
        symbols[symbol] = {
            "address": None if address is None else f"0x{address:08x}",
            "expected": "SRAM",
            "ok": ok,
        }
        if not ok:
            failures.append(f"{symbol} is missing or not in SRAM")

    for symbol in XIP_SYMBOLS:
        address = find_symbol(text, symbol)
        ok = address is not None and XIP_START <= address < XIP_END
        symbols[symbol] = {
            "address": None if address is None else f"0x{address:08x}",
            "expected": "XIP",
            "ok": ok,
        }
        if not ok:
            failures.append(f"{symbol} is missing or not in XIP")

    end_address = find_assignment(text, "__end__")
    stack_limit = find_assignment(text, "__StackLimit")
    if end_address is None or stack_limit is None:
        failures.append("unable to calculate SRAM heap/stack gap")
        ram_gap = None
    else:
        ram_gap = stack_limit - end_address
        if ram_gap < 0:
            failures.append("static SRAM overlaps the stack limit")

    image_size = args.image_file.stat().st_size
    image_headroom = OTA_IMAGE_LIMIT - image_size
    if image_headroom < 0:
        failures.append("image exceeds the XR809 OTA image limit")

    report = {
        "result": "PASS" if not failures else "FAIL",
        "map": str(args.map_file),
        "image": str(args.image_file),
        "sections": {
            "xip": {"start": f"0x{xip_start:08x}", "size": xip_size},
            "text": {"start": f"0x{text_start:08x}", "size": text_size},
            "data": {"start": f"0x{data_start:08x}", "size": data_size},
            "bss": {"start": f"0x{bss_start:08x}", "size": bss_size},
        },
        "static_ram_end": None if end_address is None else f"0x{end_address:08x}",
        "stack_limit": None if stack_limit is None else f"0x{stack_limit:08x}",
        "heap_stack_gap": ram_gap,
        "image_size": image_size,
        "image_limit": OTA_IMAGE_LIMIT,
        "image_headroom": image_headroom,
        "forbidden_xip_lines": forbidden_xip_lines,
        "symbols": symbols,
        "failures": failures,
    }

    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.json_file:
        args.json_file.write_text(rendered + "\n", encoding="utf-8")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
