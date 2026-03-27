#!/usr/bin/env python3
"""
Summarize PulseView/Sigrok UART CSV captures for the Dreo heater work.

The tool focuses on the MCU->ESP traffic that OBK needs to parse, and can:
- group UART bytes into bursts using a configurable time gap
- validate Dreo packets with the known 55 AA framing/checksum
- print replay-ready `uartFakeHex ...` commands for simulator use
- warn when a capture appears to be an older non-LSB export

Example:
    python scripts/dreo_capture_tool.py --emit-obk "f:\\Downloads\\LSB_On_H3_H2_H1_H2_H3_Fanonly_Off (1).csv"
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import re
import zipfile
from dataclasses import dataclass
from typing import Iterable, List, Sequence


DATA_ROWS = ("RX data", "TX data")
NOISE_ROWS = (
    "RX warning",
    "TX warning",
    "RX warnings",
    "TX warnings",
    "RX break",
    "TX break",
    "RX breaks",
    "TX breaks",
)
DEFAULT_DIRECTION = "TX data"


@dataclass
class CaptureEvent:
    time_s: float
    ann_row: str
    value: str


@dataclass
class PacketSummary:
    offset: int
    seq: int
    cmd: int
    payload_len: int
    packet_len: int
    status: str
    data: bytes


def parse_time_to_seconds(raw: str) -> float:
    match = re.match(r"([0-9.]+)\s*([a-zA-Z]+)?", (raw or "").strip())
    if not match:
        return 0.0
    value = float(match.group(1))
    unit = (match.group(2) or "s").lower()
    if unit.startswith("ms"):
        return value / 1000.0
    if unit.startswith("us"):
        return value / 1_000_000.0
    return value


def read_capture(path: pathlib.Path) -> list[CaptureEvent]:
    events: list[CaptureEvent] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            ann_row = (row.get("Ann Row") or "").strip()
            if ann_row not in DATA_ROWS and ann_row not in NOISE_ROWS:
                continue
            events.append(
                CaptureEvent(
                    time_s=parse_time_to_seconds(row.get("Time") or ""),
                    ann_row=ann_row,
                    value=(row.get("Value") or "").strip(),
                )
            )
    return events


def burst_events(events: Sequence[CaptureEvent], direction: str, gap_s: float) -> list[list[CaptureEvent]]:
    data = [event for event in events if event.ann_row == direction]
    if not data:
        return []
    bursts: list[list[CaptureEvent]] = [[data[0]]]
    for event in data[1:]:
        if event.time_s - bursts[-1][-1].time_s > gap_s:
            bursts.append([event])
        else:
            bursts[-1].append(event)
    return bursts


def burst_bytes(burst: Sequence[CaptureEvent]) -> bytes:
    return bytes(int(event.value, 16) for event in burst if event.value)


def detect_header_mode(bursts: Sequence[Sequence[CaptureEvent]]) -> str:
    count_55aa = 0
    count_aa55 = 0
    for burst in bursts:
        data = burst_bytes(burst)
        if len(data) < 2:
            continue
        if data[0] == 0x55 and data[1] == 0xAA:
            count_55aa += 1
        elif data[0] == 0xAA and data[1] == 0x55:
            count_aa55 += 1
    if count_55aa == 0 and count_aa55 == 0:
        return "unknown"
    if count_55aa >= count_aa55:
        return "55AA"
    return "AA55"


def parse_dreo_packets(data: bytes) -> list[PacketSummary]:
    packets: list[PacketSummary] = []
    index = 0
    while index + 9 <= len(data):
        if data[index] != 0x55 or data[index + 1] != 0xAA:
            index += 1
            continue
        payload_len = (data[index + 6] << 8) | data[index + 7]
        packet_len = 8 + payload_len + 1
        if index + packet_len > len(data):
            packets.append(
                PacketSummary(
                    offset=index,
                    seq=data[index + 3],
                    cmd=data[index + 4],
                    payload_len=payload_len,
                    packet_len=packet_len,
                    status="partial",
                    data=data[index:],
                )
            )
            break
        packet = data[index : index + packet_len]
        expected = (sum(packet[2:-1]) - 1) & 0xFF
        status = "ok" if expected == packet[-1] else "bad-checksum"
        packets.append(
            PacketSummary(
                offset=index,
                seq=packet[3],
                cmd=packet[4],
                payload_len=payload_len,
                packet_len=packet_len,
                status=status,
                data=packet,
            )
        )
        index += packet_len if status == "ok" else 1
    return packets


def format_hex(data: Iterable[int]) -> str:
    return " ".join(f"{value:02X}" for value in data)


def print_summary(path: pathlib.Path, events: Sequence[CaptureEvent], direction: str, gap_s: float, emit_obk: bool) -> None:
    bursts = burst_events(events, direction, gap_s)
    noise = [event for event in events if event.ann_row in NOISE_ROWS]
    header_mode = detect_header_mode(bursts)

    print(f"\n=== {path.name} ===")
    print(f"direction: {direction}")
    print(f"bursts: {len(bursts)}")
    print(f"warnings/breaks: {len(noise)}")
    print(f"detected-header: {header_mode}")

    if header_mode == "AA55":
        print("warning: this looks like an older non-LSB export; packet parsing below is not reliable.")

    for event in noise[:8]:
        print(f"noise {event.time_s:9.6f}s {event.ann_row}: {event.value}")

    preview_count = min(8, len(bursts))
    for burst_index, burst in enumerate(bursts[:preview_count]):
        data = burst_bytes(burst)
        packets = parse_dreo_packets(data) if header_mode == "55AA" else []
        print(
            f"burst {burst_index:02d} t={burst[0].time_s:9.6f}s len={len(data):3d} "
            f"hex={format_hex(data[:24])}"
        )
        for packet in packets[:4]:
            print(
                f"  packet ofs={packet.offset:3d} seq=0x{packet.seq:02X} "
                f"cmd=0x{packet.cmd:02X} payload={packet.payload_len:3d} status={packet.status}"
            )

    if emit_obk:
        if header_mode != "55AA":
            print("\n# replay skipped: capture is not a clean 55 AA / LSB-decoded export")
            return
        print("\n# replay-ready MCU->ESP packets")
        for burst in bursts:
            packets = parse_dreo_packets(burst_bytes(burst))
            for packet in packets:
                if packet.status != "ok":
                    continue
                print(f"# t={burst[0].time_s:9.6f}s cmd=0x{packet.cmd:02X} seq=0x{packet.seq:02X}")
                print(f"uartFakeHex {format_hex(packet.data)}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", nargs="+", help="PulseView CSV capture paths")
    parser.add_argument(
        "--direction",
        default=DEFAULT_DIRECTION,
        choices=DATA_ROWS,
        help="Which direction to summarize or emit for replay",
    )
    parser.add_argument(
        "--gap-ms",
        type=float,
        default=3.0,
        help="Gap in milliseconds used to split bursts",
    )
    parser.add_argument(
        "--emit-obk",
        action="store_true",
        help="Emit replay-ready uartFakeHex commands for valid packets",
    )
    args = parser.parse_args()

    for capture in args.captures:
        path = pathlib.Path(capture)
        if not path.exists():
            print(f"\n=== {path.name} ===")
            print("error: file not found")
            continue
        if path.suffix.lower() == ".zip":
            print(f"\n=== {path.name} ===")
            print("note: zip archives are not parsed directly yet.")
            try:
                with zipfile.ZipFile(path) as archive:
                    names = [entry.filename for entry in archive.infolist()]
                for name in names:
                    print(f"zip-entry: {name}")
            except zipfile.BadZipFile:
                print("error: invalid zip archive")
            continue
        events = read_capture(path)
        print_summary(path, events, args.direction, args.gap_ms / 1000.0, args.emit_obk)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
