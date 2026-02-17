#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TR6260 OTA v1 (Tuya-style) FotaPKG builder.

This generates FotaPKG v1 OTA "diff" packages like Tuya TR6260 *.ug_*.bin files:
- Magic: "FotaPKG"
- patch_version: 0x01
- CRC32 at 0x0C is CRC32 of bytes from 0x10 to EOF (same as Tuya packages)
- Entry table at 0x40 with N records of 0x70 bytes each:
    name[16] + 8x u32 fields + old_sha256[32] + new_sha256[32]
- Each entry points to a standard BSDIFF40 patch blob (produced by the `bsdiff` tool).

It is intended to replace the ECR6600-derived ota_tool output (patch_version=3)
which TR6260 bootloaders often reject or ignore.

Requires:
- `bsdiff` present in PATH (Ubuntu: apt-get install bsdiff)
"""

import argparse
import hashlib
import math
import os
import struct
import subprocess
import sys
import tempfile
import zlib
from typing import Optional, Tuple


MAGIC = b"FotaPKG"
PATCH_VERSION_V1 = 0x01
REC_SIZE = 0x70
HDR_SIZE = 0x40


def _u32(x: int) -> bytes:
    return struct.pack("<I", x & 0xFFFFFFFF)


def _crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def _sha256(data: bytes) -> bytes:
    return hashlib.sha256(data).digest()


def _clamp_ascii_16(s: str) -> bytes:
    # Tuya uses 16-byte NUL padded ASCII; keep it simple.
    b = s.encode("ascii", "ignore")
    if len(b) > 16:
        b = b[:16]
    return b.ljust(16, b"\x00")


def _parse_hex_int(s: str) -> int:
    s = s.strip().lower()
    if s.startswith("0x"):
        return int(s, 16)
    return int(s, 10)


def _read_file(path: str) -> bytes:
    with open(path, "rb") as f:
        return f.read()


def _write_file(path: str, data: bytes) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)


def _have_bsdiff() -> bool:
    from shutil import which
    return which("bsdiff") is not None


def _run_bsdiff(old_bytes: bytes, new_bytes: bytes) -> bytes:
    """
    Produce a standard BSDIFF40 patch by calling the external `bsdiff` binary.
    """
    if not _have_bsdiff():
        raise RuntimeError("bsdiff not found in PATH. Install it (Ubuntu: apt-get install bsdiff).")

    with tempfile.TemporaryDirectory(prefix="tr6260_ota_v1_") as td:
        old_path = os.path.join(td, "old.bin")
        new_path = os.path.join(td, "new.bin")
        patch_path = os.path.join(td, "patch.bin")

        _write_file(old_path, old_bytes)
        _write_file(new_path, new_bytes)

        # bsdiff old new patch
        proc = subprocess.run(["bsdiff", old_path, new_path, patch_path], capture_output=True, text=True)
        if proc.returncode != 0:
            raise RuntimeError(f"bsdiff failed ({proc.returncode}):\n{proc.stderr.strip()}")

        patch = _read_file(patch_path)
        if not patch.startswith(b"BSDIFF40"):
            raise RuntimeError("bsdiff output does not start with BSDIFF40 (unexpected).")
        return patch


def _chunk_bounds(size: int, chunk_size: int, idx: int) -> Tuple[int, int]:
    start = idx * chunk_size
    if start >= size:
        return start, 0
    end = min(size, start + chunk_size)
    return start, end - start


def build_fotapkg_v1(
    current_path: str,
    new_path: str,
    out_path: str,
    out_path_alt: Optional[str],
    base_addr: int,
    chunk_size: int,
    src_ver: str,
    tgt_ver: str,
    fill_sizes: bool,
) -> None:
    current = _read_file(current_path)
    new = _read_file(new_path)

    old_size = len(current)
    new_size = len(new)

    # Use the larger size to decide chunk count, like Tuya packages.
    total_bytes = max(old_size, new_size)
    count = int(math.ceil(total_bytes / float(chunk_size))) if total_bytes else 0

    if count <= 0:
        raise RuntimeError("Nothing to patch (both inputs empty?).")

    # Build patches first, so we can compute offsets/lengths.
    patches = []
    entries = []

    for i in range(count):
        old_off, old_len = _chunk_bounds(old_size, chunk_size, i)
        new_off, new_len = _chunk_bounds(new_size, chunk_size, i)

        old_chunk = current[old_off:old_off + old_len]
        new_chunk = new[new_off:new_off + new_len]

        # If both are empty, skip (shouldn't normally happen with count derived from max size).
        if old_len == 0 and new_len == 0:
            continue

        patch = _run_bsdiff(old_chunk, new_chunk)

        # Flags pattern observed in Tuya ug packages:
        # - 0x101 for normal chunk update
        # - 0x100 for "create" (old_len==0)
        flags1 = 0x100 if old_len == 0 else 0x101
        flags2 = 0x20  # hash length (32)

        h_old = (b"\x00" * 32) if old_len == 0 else _sha256(old_chunk)
        h_new = (b"\x00" * 32) if new_len == 0 else _sha256(new_chunk)

        entry = {
            "name": b"cpu1",
            "old_addr": base_addr + i * chunk_size,
            "old_len": old_len,
            "new_addr": base_addr + i * chunk_size,
            "new_len": new_len,
            "patch_len": len(patch),
            "flags1": flags1,
            "flags2": flags2,
            "h_old": h_old,
            "h_new": h_new,
        }

        patches.append(patch)
        entries.append(entry)

    # Update count to actual entries produced (in case we skipped any tail empties)
    count = len(entries)
    table_size = HDR_SIZE + count * REC_SIZE

    # Assign patch offsets sequentially.
    cur_off = table_size
    for entry, patch in zip(entries, patches):
        entry["patch_off"] = cur_off
        cur_off += len(patch)

    # Build header (0x40).
    # Layout (v1):
    # 0x00: "FotaPKG"
    # 0x07: patch_version (0x01)
    # 0x08: total_size (u32)
    # 0x0C: crc32 of bytes 0x10..EOF (u32)
    # 0x10: entry_count (u32)
    # 0x14: old_size (u32) [seen used on some packages]
    # 0x18: new_size (u32)
    # 0x1C: reserved (u32)
    # 0x20: src_ver[16]
    # 0x30: tgt_ver[16]
    hdr = bytearray(HDR_SIZE)
    hdr[0:7] = MAGIC
    hdr[7] = PATCH_VERSION_V1
    # size + crc later
    hdr[0x10:0x14] = _u32(count)
    if fill_sizes:
        hdr[0x14:0x18] = _u32(old_size)
        hdr[0x18:0x1C] = _u32(new_size)
    else:
        hdr[0x14:0x18] = _u32(0)
        hdr[0x18:0x1C] = _u32(0)
    hdr[0x1C:0x20] = _u32(0)
    hdr[0x20:0x30] = _clamp_ascii_16(src_ver)
    hdr[0x30:0x40] = _clamp_ascii_16(tgt_ver)

    # Build entry table.
    table = bytearray()
    for e in entries:
        rec = bytearray(REC_SIZE)
        rec[0:16] = e["name"].ljust(16, b"\x00")
        rec[16:20] = _u32(e["old_addr"])
        rec[20:24] = _u32(e["old_len"])
        rec[24:28] = _u32(e["new_addr"])
        rec[28:32] = _u32(e["new_len"])
        rec[32:36] = _u32(e["patch_off"])
        rec[36:40] = _u32(e["patch_len"])
        rec[40:44] = _u32(e["flags1"])
        rec[44:48] = _u32(e["flags2"])
        rec[48:80] = e["h_old"]
        rec[80:112] = e["h_new"]
        table += rec

    # Assemble file.
    blob = bytearray()
    blob += hdr
    blob += table
    for p in patches:
        blob += p

    # Fill total size.
    total_size = len(blob)
    blob[0x08:0x0C] = _u32(total_size)

    # Fill crc32 (bytes 0x10..EOF).
    crc = _crc32(bytes(blob[0x10:]))
    blob[0x0C:0x10] = _u32(crc)

    # Write outputs.
    _write_file(out_path, bytes(blob))
    if out_path_alt:
        _write_file(out_path_alt, bytes(blob))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--current", required=True, help="Baseline firmware image (current.bin) as flashed at 0x7000")
    ap.add_argument("--new", required=True, help="New firmware image (new.bin) as flashed at 0x7000")
    ap.add_argument("--out", required=True, help="Output OTA image path (FotaPKG v1)")
    ap.add_argument("--out-alt", default=None, help="Optional second output path (same bytes)")

    ap.add_argument("--base-addr", default="0x7000", help="Flash base address for cpu1 image (default 0x7000)")
    ap.add_argument("--chunk-size", default="0x5000", help="Chunk size (default 0x5000, matches Tuya ug packages)")

    ap.add_argument("--src", default="OBK_CUR", help="Source version string (<=16 ASCII bytes)")
    ap.add_argument("--tgt", default="OBK_NEW", help="Target version string (<=16 ASCII bytes)")
    ap.add_argument("--fill-sizes", action="store_true",
                    help="Fill header old/new size fields (0x14/0x18). Some Tuya packages leave them zero.")

    args = ap.parse_args()

    base_addr = _parse_hex_int(args.base_addr)
    chunk_size = _parse_hex_int(args.chunk_size)
    if chunk_size <= 0:
        raise SystemExit("chunk-size must be > 0")

    build_fotapkg_v1(
        current_path=args.current,
        new_path=args.new,
        out_path=args.out,
        out_path_alt=args.out_alt,
        base_addr=base_addr,
        chunk_size=chunk_size,
        src_ver=args.src,
        tgt_ver=args.tgt,
        fill_sizes=args.fill_sizes,
    )

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        sys.stderr.write(f"ERROR: {e}\n")
        raise
