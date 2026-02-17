#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TR6260 OTA (FotaPKG v1) builder.

This emits a Tuya/TR6260-style v1 FotaPKG container with a 0x70-byte entry table and
per-section patch payloads that match the bootloader's expectations:
  - patch_version (byte 0x07) = 0x01
  - per-section payloads are LZMA1 raw streams (no props header), using fixed parameters:
        lc=0, lp=0, pb=0, dict_size=0x2000
  - normal sections: payload begins with 'BSDIFF40' + (ctrl_len, diff_len, new_size) then raw-LZMA(ctrl+diff+extra)
  - create sections (old_len == 0): payload is raw-LZMA(new_bytes) (no BSDIFF40 header)

The patch "algorithm" used here is intentionally simple and deterministic:
  - For each section we generate a single bsdiff control triple (new_len, 0, 0)
  - diff bytes are (new[i] - old[i]) mod 256
  - extra block is empty

This produces valid patches (they reconstruct 'new' from 'old') and compresses well.

Inputs:
  --partition  : 4KB partition table blob at 0x6000 (used to find cpu1 base/len if present)
  --current    : current image bytes as flashed at cpu1 base (0x7000)
  --new        : new image bytes to be written at cpu1 base
  --out        : output OTA image file (e.g. OpenTR6260_<ver>_ota.img)

Exit codes:
  0 = success, non-zero = failure.
"""
from __future__ import annotations

import argparse
import hashlib
import lzma
import os
import re
import struct
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


MAGIC = b"FotaPKG"
ENTRY_SIZE = 0x70
HEADER_SIZE = 0x40


LZMA_FILTERS = [{
    "id": lzma.FILTER_LZMA1,
    "dict_size": 0x2000,
    "lc": 0,
    "lp": 0,
    "pb": 0,
}]


def _u32(x: int) -> bytes:
    return struct.pack("<I", x & 0xFFFFFFFF)


def _i64(x: int) -> bytes:
    return struct.pack("<q", int(x))


def _encode_offt(x: int) -> bytes:
    """BSDIFF offtout() encoding."""
    y = abs(int(x))
    buf = bytearray(8)
    for i in range(8):
        buf[i] = y & 0xFF
        y >>= 8
    if x < 0:
        buf[7] |= 0x80
    return bytes(buf)


def _sha256(data: bytes) -> bytes:
    return hashlib.sha256(data).digest()


def _crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def _pad16(s: str) -> bytes:
    b = s.encode("ascii", errors="ignore")[:16]
    return b + b"\x00" * (16 - len(b))


def _read_file(path: Path) -> bytes:
    try:
        return path.read_bytes()
    except Exception as e:
        raise RuntimeError(f"Failed to read: {path} ({e})") from e


def _find_cpu1_in_partition(part: bytes) -> Optional[Tuple[int, int]]:
    """
    Partition blobs typically contain ASCII like:
      cpu10x7000,0xAF000
      data_ota0xB6000,0x36000
    """
    txt = part.decode("latin1", errors="ignore")
    m = re.search(r"cpu1\s*0x([0-9A-Fa-f]+)\s*,\s*0x([0-9A-Fa-f]+)", txt)
    if not m:
        m = re.search(r"cpu10x([0-9A-Fa-f]+)\s*,\s*0x([0-9A-Fa-f]+)", txt)
    if not m:
        return None
    return int(m.group(1), 16), int(m.group(2), 16)


@dataclass
class Entry:
    name: str
    old_addr: int
    old_len: int
    new_addr: int
    new_len: int
    patch_off: int
    patch_len: int
    flags: int
    hash_len: int
    old_hash: bytes
    new_hash: bytes


def _make_patch_bsdiff_lzma(old_bytes: bytes, new_bytes: bytes) -> bytes:
    """
    Create a TR6260/Tuya-style bsdiff patch:
      BSDIFF40 + <q ctrl_len> + <q diff_len> + <q new_size> + raw-LZMA(ctrl+diff+extra)
    where ctrl is a single triple: (new_len, 0, 0).
    """
    if len(old_bytes) != len(new_bytes):
        raise ValueError("old_bytes and new_bytes must be same length for patch generation")

    n = len(new_bytes)
    ctrl = _encode_offt(n) + _encode_offt(0) + _encode_offt(0)  # 24 bytes
    # diff = new - old (mod 256)
    diff = bytes(((new_bytes[i] - old_bytes[i]) & 0xFF) for i in range(n))
    payload = ctrl + diff  # extra is empty

    comp = lzma.compress(payload, format=lzma.FORMAT_RAW, filters=LZMA_FILTERS)
    return b"BSDIFF40" + _i64(len(ctrl)) + _i64(n) + _i64(n) + comp


def _make_create_lzma(new_bytes: bytes) -> bytes:
    """Create-section payload: raw-LZMA(new_bytes) with fixed parameters."""
    return lzma.compress(new_bytes, format=lzma.FORMAT_RAW, filters=LZMA_FILTERS)


def build_fotapkg_v1(
    partition: bytes,
    current_img: bytes,
    new_img: bytes,
    out_path: Path,
    chunk_size: int = 0x5000,
    pad_byte: int = 0xFF,
    name: str = "cpu1",
    src_ver: str = "",
    tgt_ver: str = "",
) -> None:
    cpu1 = _find_cpu1_in_partition(partition)
    if cpu1 is None:
        cpu1_addr = 0x7000
        cpu1_len = max(len(current_img), len(new_img))
    else:
        cpu1_addr, cpu1_len = cpu1

    # We patch up to the maximum of current/new sizes. Any "tail" beyond the shorter image
    # is treated as 0xFF (flash-erased), so updates can erase leftover bytes if needed.
    max_len = max(len(current_img), len(new_img))
    if max_len == 0:
        raise RuntimeError("current/new images are empty")

    # Clamp to cpu1 partition length if it is smaller (should not happen, but avoid OOB addresses)
    if cpu1_len and max_len > cpu1_len:
        max_len = cpu1_len

    cur_padded = current_img[:max_len] + bytes([pad_byte]) * max(0, max_len - len(current_img))
    new_padded = new_img[:max_len] + bytes([pad_byte]) * max(0, max_len - len(new_img))

    # Build entries + patch blobs
    entries: List[Entry] = []
    patch_blobs: List[bytes] = []

    # placeholder offsets; we'll fill later
    for off in range(0, max_len, chunk_size):
        chunk_end = min(off + chunk_size, max_len)
        new_len = chunk_end - off

        # Determine "old_len" from actual current_img size (not padded)
        if off >= len(current_img):
            old_len = 0
        else:
            old_len = min(chunk_end - off, len(current_img) - off)

        # Slice bytes
        new_chunk = new_padded[off:off + new_len]

        # old bytes for hashing are only the bytes that exist in current_img (old_len)
        old_hash = b"\x00" * 32
        if old_len > 0:
            old_hash = _sha256(current_img[off:off + old_len])

        new_hash = _sha256(new_chunk)

        # Create vs Patch decision:
        # If old_len == 0 -> "create" entry (raw LZMA(new_chunk))
        # Else -> "patch" entry (BSDIFF40 + raw LZMA(ctrl+diff))
        if old_len == 0:
            flags = 0x100
            blob = _make_create_lzma(new_chunk)
        else:
            flags = 0x101
            # For patch generation, old bytes must match new_len length; pad missing old tail with 0xFF.
            old_for_patch = current_img[off:off + old_len] + bytes([pad_byte]) * (new_len - old_len)
            blob = _make_patch_bsdiff_lzma(old_for_patch, new_chunk)

        patch_blobs.append(blob)
        entries.append(Entry(
            name=name,
            old_addr=cpu1_addr + off,
            old_len=old_len,
            new_addr=cpu1_addr + off,
            new_len=new_len,
            patch_off=0,     # to fill
            patch_len=len(blob),
            flags=flags,
            hash_len=0x20,
            old_hash=old_hash,
            new_hash=new_hash,
        ))

    # Now compute patch offsets after the entry table
    entry_count = len(entries)
    patch_start = HEADER_SIZE + entry_count * ENTRY_SIZE
    cur_off = patch_start
    for e in entries:
        e.patch_off = cur_off
        cur_off += e.patch_len

    # Build header
    total_size = cur_off
    header = bytearray(HEADER_SIZE)
    header[0:7] = MAGIC
    header[7] = 0x01  # patch_version v1

    # total size
    struct.pack_into("<I", header, 8, total_size)
    # CRC32 placeholder at 0x0C
    struct.pack_into("<I", header, 12, 0)

    # entry count at 0x10
    struct.pack_into("<I", header, 16, entry_count)
    # 0x14..0x1F left as zeroes
    header[0x20:0x30] = _pad16(src_ver)
    header[0x30:0x40] = _pad16(tgt_ver)

    # Build entry table
    table = bytearray()
    for e in entries:
        ent = bytearray(ENTRY_SIZE)
        name_b = e.name.encode("ascii", errors="ignore")[:16]
        ent[0:len(name_b)] = name_b
        struct.pack_into("<8I", ent, 16,
                         e.old_addr & 0xFFFFFFFF,
                         e.old_len & 0xFFFFFFFF,
                         e.new_addr & 0xFFFFFFFF,
                         e.new_len & 0xFFFFFFFF,
                         e.patch_off & 0xFFFFFFFF,
                         e.patch_len & 0xFFFFFFFF,
                         e.flags & 0xFFFFFFFF,
                         e.hash_len & 0xFFFFFFFF)
        ent[48:80] = e.old_hash
        ent[80:112] = e.new_hash
        table += ent

    # Concatenate patches
    blob_data = b"".join(patch_blobs)

    # Final file bytes
    out = bytes(header) + bytes(table) + blob_data

    # Fix CRC32 (bytes 0x10..EOF)
    crc = _crc32(out[0x10:])
    out = bytearray(out)
    struct.pack_into("<I", out, 12, crc)
    out = bytes(out)

    # Basic sanity checks
    if len(out) != total_size:
        raise RuntimeError(f"Internal size mismatch: expected {total_size}, got {len(out)}")
    if out[:7] != MAGIC or out[7] != 0x01:
        raise RuntimeError("Bad header write")
    if _crc32(out[0x10:]) != struct.unpack_from("<I", out, 12)[0]:
        raise RuntimeError("CRC verify failed")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(out)


def main(argv: List[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--partition", required=True, help="Partition table blob (4KB at 0x6000)")
    ap.add_argument("--current", required=True, help="Current image (cpu1 content at 0x7000)")
    ap.add_argument("--new", required=True, help="New image (cpu1 content at 0x7000)")
    ap.add_argument("--out", required=True, help="Output OTA file path")
    ap.add_argument("--chunk-size", default="0x5000", help="Chunk size (default 0x5000)")
    ap.add_argument("--pad-byte", default="0xFF", help="Pad byte for missing tail (default 0xFF)")
    ap.add_argument("--name", default="cpu1", help="Entry name (default cpu1)")
    ap.add_argument("--srcver", default="", help="16-byte ASCII source version tag (optional)")
    ap.add_argument("--tgtver", default="", help="16-byte ASCII target version tag (optional)")
    args = ap.parse_args(argv)

    part = _read_file(Path(args.partition))
    cur = _read_file(Path(args.current))
    new = _read_file(Path(args.new))
    out = Path(args.out)

    chunk = int(str(args.chunk_size), 0)
    pad = int(str(args.pad_byte), 0) & 0xFF

    build_fotapkg_v1(
        partition=part,
        current_img=cur,
        new_img=new,
        out_path=out,
        chunk_size=chunk,
        pad_byte=pad,
        name=args.name,
        src_ver=args.srcver,
        tgt_ver=args.tgtver,
    )
    print(f"Wrote: {out} ({out.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
