#!/usr/bin/env python3
"""Build an Opulinks VEN-compatible OPL1000 packed flash image."""

import argparse
import os
from pathlib import Path
import struct
import sys


PATCH_MAGIC = 0x50544348
FUNCTION_TYPES = {"HW": 0x1, "SW": 0x2, "CODE": 0x4}
APPLY_TYPES = {"COLD": 0x10, "WARM": 0x20, "MASK": 0xF0}
CPU_TYPES = {"M3": 0x100, "M0": 0x200, "MASK": 0xF00}
WRITE_TYPES = {1: 0x00010000, 2: 0x00020000, 4: 0x00040000}


class PackError(ValueError):
    pass


def parse_number(value, context):
    try:
        number = int(value, 0)
    except ValueError as exc:
        raise PackError(f"{context}: invalid integer {value!r}") from exc
    if not 0 <= number <= 0xFFFFFFFF:
        raise PackError(f"{context}: value {value!r} is outside uint32 range")
    return number


def parse_patch_data(path):
    blocks = []
    current = None

    with path.open("r", encoding="utf-8-sig") as source:
        for line_number, raw_line in enumerate(source, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            fields = [field.strip() for field in line.split(",")]
            if len(fields) == 4:
                try:
                    width = int(fields[0], 0)
                except ValueError as exc:
                    raise PackError(
                        f"{path}:{line_number}: invalid write width {fields[0]!r}"
                    ) from exc
                function, apply, cpu = (field.upper() for field in fields[1:])
                if width not in WRITE_TYPES:
                    raise PackError(f"{path}:{line_number}: unsupported write width {width}")
                if function not in FUNCTION_TYPES:
                    raise PackError(f"{path}:{line_number}: unsupported function {function!r}")
                if apply not in APPLY_TYPES:
                    raise PackError(f"{path}:{line_number}: unsupported apply mode {apply!r}")
                if cpu not in CPU_TYPES:
                    raise PackError(f"{path}:{line_number}: unsupported CPU {cpu!r}")
                current = {
                    "width": width,
                    "function": function,
                    "apply": apply,
                    "cpu": cpu,
                    "entries": [],
                    "line": line_number,
                }
                blocks.append(current)
            elif len(fields) == 2:
                if current is None:
                    raise PackError(f"{path}:{line_number}: data appears before a block header")
                current["entries"].append((fields[0], fields[1], line_number))
            else:
                raise PackError(
                    f"{path}:{line_number}: expected a four-field header or two-field data line"
                )

    if not blocks:
        raise PackError(f"{path}: no patch blocks found")
    for block in blocks:
        if not block["entries"]:
            raise PackError(f"{path}:{block['line']}: block contains no data")
    return blocks


def read_m0_container(path):
    data = path.read_bytes()
    if not data:
        raise PackError(f"{path}: empty M0 container")

    segment_count = data[0]
    offset = 1
    segments = []
    for index in range(segment_count):
        if offset + 4 > len(data):
            raise PackError(f"{path}: truncated length for M0 segment {index + 1}")
        length = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        end = offset + length
        if end > len(data):
            raise PackError(f"{path}: truncated data for M0 segment {index + 1}")
        segments.append(data[offset:end])
        offset = end

    if offset != len(data):
        raise PackError(f"{path}: {len(data) - offset} trailing bytes after M0 segments")
    return segments


def resolve_script_path(script_path, patch_data_path):
    normalized = script_path.replace("\\", os.sep).replace("/", os.sep)
    candidate = Path(normalized)
    if not candidate.is_absolute():
        candidate = patch_data_path.parent / candidate
    return candidate.resolve()


def make_record(block_type, payload):
    header = struct.pack(">III", PATCH_MAGIC, block_type, len(payload))
    return header + struct.pack(">I", sum(header) & 0xFFFFFFFF) + payload + struct.pack(
        ">I", sum(payload) & 0xFFFFFFFF
    )


def build_image(blocks, patch_data_path, m0_segments):
    records = []
    m0_index = 0

    for block in blocks:
        block_type = (
            WRITE_TYPES[block["width"]]
            | FUNCTION_TYPES[block["function"]]
            | APPLY_TYPES[block["apply"]]
            | CPU_TYPES[block["cpu"]]
        )

        if block["function"] == "CODE":
            if len(block["entries"]) != 1:
                raise PackError(
                    f"{patch_data_path}:{block['line']}: CODE blocks require exactly one entry"
                )
            address_text, filename, line_number = block["entries"][0]
            address = parse_number(address_text, f"{patch_data_path}:{line_number}")
            if block["cpu"] == "M0" and m0_segments is not None:
                if m0_index >= len(m0_segments):
                    raise PackError(
                        f"{patch_data_path}:{line_number}: no M0 segment remains for this CODE block"
                    )
                code = m0_segments[m0_index]
                m0_index += 1
            else:
                code_path = resolve_script_path(filename, patch_data_path)
                try:
                    code = code_path.read_bytes()
                except OSError as exc:
                    raise PackError(
                        f"{patch_data_path}:{line_number}: cannot read {code_path}: {exc}"
                    ) from exc
            payload = struct.pack(">I", address) + code
        else:
            payload_parts = []
            for address_text, value_text, line_number in block["entries"]:
                address = parse_number(address_text, f"{patch_data_path}:{line_number}")
                value = parse_number(value_text, f"{patch_data_path}:{line_number}")
                payload_parts.append(struct.pack(">II", address, value))
            payload = b"".join(payload_parts)

        records.append((block, payload, make_record(block_type, payload)))

    if m0_segments is not None and m0_index != len(m0_segments):
        raise PackError(
            f"M0 container has {len(m0_segments)} segments but PatchData consumes {m0_index}"
        )
    return records


def main():
    parser = argparse.ArgumentParser(
        description="Pack OPL1000 PatchData, M3 images, and an M0 container into one flash image."
    )
    parser.add_argument("--patch-data", required=True, type=Path)
    parser.add_argument("--m0-container", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    try:
        patch_data_path = args.patch_data.resolve()
        blocks = parse_patch_data(patch_data_path)
        m0_segments = read_m0_container(args.m0_container.resolve()) if args.m0_container else None
        records = build_image(blocks, patch_data_path, m0_segments)

        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_name(args.output.name + ".tmp")
        temporary.write_bytes(b"".join(record for _, _, record in records))
        temporary.replace(args.output)
    except (OSError, PackError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    for index, (block, payload, _) in enumerate(records, 1):
        print(
            f"  {index}: {block['function']}/{block['cpu']}/{block['apply']} "
            f"payload={len(payload)} bytes"
        )
    print(f"Packed {len(records)} records into {args.output} ({args.output.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
