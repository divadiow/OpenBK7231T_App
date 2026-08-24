#!/usr/bin/env python3
"""Create a Tuya T1 PCZL full-image OTA from a BK7238 raw application.

This clean-room implementation uses only Python's standard library and does
not require Tuya's proprietary diff2ya executable.
"""

from __future__ import annotations

import argparse
import hashlib
import lzma
from pathlib import Path
import struct
import sys
import zlib


PCZL_MAGIC = b"MMM\0PCZL"
XZ_MAGIC = b"\xFD7zXZ\x00"
PRIMARY_HEADER_SIZE = 0x40
EXTENDED_HEADER_SIZE = 0x90
XZ_OFFSET = PRIMARY_HEADER_SIZE + EXTENDED_HEADER_SIZE

# These limits describe the preserved Tuya T1 profile. The normal package is
# staged at 0x132000 and must end before the first OTA journal sector at
# 0x1E1000. Keep the raw image within the shared standard-Beken application
# capacity so one canonical OpenBK7238 build can use either OTA envelope.
MAX_RAW_SIZE = 0x110000
MAX_PACKAGE_SIZE = 0x1E1000 - 0x132000


class PackageError(ValueError):
    """Raised when an input or generated package violates the T1 contract."""


def _u32(blob: bytes, offset: int) -> int:
    return struct.unpack_from("<I", blob, offset)[0]


def _compress(raw: bytes) -> bytes:
    """Match the deterministic XZ stream emitted by Tuya's full-image tool."""
    filters = [{
        "id": lzma.FILTER_LZMA2,
        "dict_size": 4096,
        "lc": 3,
        "lp": 0,
        "pb": 2,
        "mode": lzma.MODE_NORMAL,
        "nice_len": 273,
        "mf": lzma.MF_BT4,
        "depth": 0,
    }]
    return lzma.compress(
        raw,
        format=lzma.FORMAT_XZ,
        check=lzma.CHECK_CRC32,
        filters=filters,
    )


def build_package(raw: bytes) -> bytes:
    if not raw:
        raise PackageError("raw application is empty")
    if len(raw) > MAX_RAW_SIZE:
        raise PackageError(
            f"raw application is 0x{len(raw):X} bytes; maximum is 0x{MAX_RAW_SIZE:X}"
        )

    raw_crc = zlib.crc32(raw) & 0xFFFFFFFF
    # OpenBK has no Tuya segment B. B_START is deliberately the raw image end,
    # so the suffix is empty and its CRC32 is zero.
    b_start = len(raw)
    part_b_crc = zlib.crc32(raw[b_start:]) & 0xFFFFFFFF
    xz_stream = _compress(raw)

    extended_header = bytearray(EXTENDED_HEADER_SIZE)
    struct.pack_into(
        "<IIII",
        extended_header,
        0x04,
        PRIMARY_HEADER_SIZE,
        len(raw),
        len(raw),
        part_b_crc,
    )
    region = bytes(extended_header) + xz_stream

    primary_header = bytearray(PRIMARY_HEADER_SIZE)
    primary_header[:len(PCZL_MAGIC)] = PCZL_MAGIC
    struct.pack_into(
        "<IIII",
        primary_header,
        0x08,
        raw_crc,
        raw_crc,
        len(raw),
        len(raw),
    )
    struct.pack_into("<I", primary_header, 0x18, PRIMARY_HEADER_SIZE)
    struct.pack_into(
        "<II",
        primary_header,
        0x20,
        len(region),
        zlib.crc32(region) & 0xFFFFFFFF,
    )
    struct.pack_into("<II", primary_header, 0x2C, 0x10000, 0x20000)
    struct.pack_into("<I", primary_header, 0x3C, EXTENDED_HEADER_SIZE)

    package = bytes(primary_header) + region
    if len(package) > MAX_PACKAGE_SIZE:
        raise PackageError(
            f"PCZL package is 0x{len(package):X} bytes; staging limit is "
            f"0x{MAX_PACKAGE_SIZE:X}"
        )
    return package


def validate_package(package: bytes, expected_raw: bytes) -> None:
    if len(package) < XZ_OFFSET:
        raise PackageError("PCZL package is shorter than its headers")
    if package[:8] != PCZL_MAGIC:
        raise PackageError("bad PCZL magic")
    if package[XZ_OFFSET:XZ_OFFSET + len(XZ_MAGIC)] != XZ_MAGIC:
        raise PackageError("XZ stream is not at offset 0xD0")

    raw_crc = zlib.crc32(expected_raw) & 0xFFFFFFFF
    expected_fields = {
        0x08: raw_crc,
        0x0C: raw_crc,
        0x10: len(expected_raw),
        0x14: len(expected_raw),
        0x18: PRIMARY_HEADER_SIZE,
        0x20: len(package) - PRIMARY_HEADER_SIZE,
        0x24: zlib.crc32(package[PRIMARY_HEADER_SIZE:]) & 0xFFFFFFFF,
        0x2C: 0x10000,
        0x30: 0x20000,
        0x3C: EXTENDED_HEADER_SIZE,
        0x44: PRIMARY_HEADER_SIZE,
        0x48: len(expected_raw),
        0x4C: len(expected_raw),
        0x50: 0,
    }
    for offset, expected in expected_fields.items():
        actual = _u32(package, offset)
        if actual != expected:
            raise PackageError(
                f"field 0x{offset:02X} is 0x{actual:X}; expected 0x{expected:X}"
            )

    decompressor = lzma.LZMADecompressor(format=lzma.FORMAT_XZ)
    decoded = decompressor.decompress(package[XZ_OFFSET:])
    if not decompressor.eof or decompressor.unused_data:
        raise PackageError("XZ stream is incomplete or has trailing data")
    if decompressor.check != lzma.CHECK_CRC32:
        raise PackageError("XZ stream does not use CRC32")
    if decoded != expected_raw:
        raise PackageError("decoded application does not match the input")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="SDK out/bsp.bin raw application")
    parser.add_argument("output", type=Path, help="output Tuya T1 PCZL file")
    args = parser.parse_args()

    try:
        raw = args.input.read_bytes()
        package = build_package(raw)
        validate_package(package, raw)

        # A second independent build makes nondeterministic compressor or
        # packaging behaviour a hard build failure rather than a release drift.
        if build_package(raw) != package:
            raise PackageError("PCZL generation is not deterministic")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(package)
    except (OSError, PackageError, lzma.LZMAError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    print(f"[INFO] PCZL input:   {args.input}")
    print(f"[INFO] Raw size:     {len(raw)} (0x{len(raw):X})")
    print(f"[INFO] Raw SHA-256:  {hashlib.sha256(raw).hexdigest()}")
    print(f"[INFO] B_START:      {len(raw)} (empty segment B)")
    print(f"[INFO] PCZL output:  {args.output}")
    print(f"[INFO] Package size: {len(package)} (0x{len(package):X})")
    print(f"[INFO] PCZL SHA-256: {hashlib.sha256(package).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
