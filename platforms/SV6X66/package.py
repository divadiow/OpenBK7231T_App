#!/usr/bin/env python3
"""Wrap the vendor combined image for OpenSV6166F's checked HTTP OTA upload."""
import argparse
import hashlib
from pathlib import Path
import struct
import zlib

APP_START = 0x8000
MAIN_SIZE = 620 * 1024
FS_START = APP_START + MAIN_SIZE
FLASH_SIZE = 2 * 1024 * 1024
RAW_SIZE = 8192

def wrap_image(image):
    if not APP_START < len(image) <= FS_START:
        raise ValueError('expected a flash-offset-zero build image within the main partition')
    geometry = struct.unpack_from('<10I', image)
    if geometry[1:8] != (40, 80, 4, MAIN_SIZE, FLASH_SIZE, 0, 0) or geometry[9] != RAW_SIZE:
        raise ValueError('image boot header does not match the SV6166F target layout')
    header = struct.pack('<8sIIII16sI', b'OBKSV616', 1, len(image), APP_START, FS_START,
                         hashlib.md5(image).digest(), 0)
    return header + struct.pack('<I', zlib.crc32(header)) + image

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.write_bytes(wrap_image(args.image.read_bytes()))

if __name__ == '__main__':
    main()
