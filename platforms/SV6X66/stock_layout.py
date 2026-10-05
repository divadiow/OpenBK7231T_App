"""Stock CKW04 linker overlay and strict ELF-to-UART conversion checks."""
import struct

APP_XIP = 0x3000B000
FS_XIP = 0x300BA000

def stock_xmodem_image(application):
    """Stock receiver uses absolute stream offsets, skipping bytes below B000.

    In the inspected CKW04 loader, r9 starts at zero (30000B3E).
    The gate at 30000D88 skips programming while r9 < start. Once eligible,
    page programming at 30000DAC receives r9 directly, without a base offset.
    FF transport padding contains no bootloader and is discarded by this loader.
    """
    start = APP_XIP - 0x30000000
    if not application or application[:4] != b'\x48\x00\x00\x02':
        raise ValueError('stock XMODEM application startup branch missing')
    if len(application) > FS_XIP - APP_XIP:
        raise ValueError('stock XMODEM application exceeds partition')
    return b'\xff' * start + application


def linker_overlay(source):
    """Retain vendor startup/RAM layout, moving config into the app trailer."""
    start = source.index('    .fix_table_section :')
    end = source.index('    .magic_boot :', start)
    table_end = source.index('    } > REGION_BURN', start) + len('    } > REGION_BURN')
    table = source[start:table_end]
    table = table.replace('.fix_table_section :', '.fix_table_section ALIGN(LOADADDR(.tbss), 4096) : AT(ALIGN(LOADADDR(.tbss), 4096))')
    # Inside an output section, constant dot assignments are section offsets.
    # Avoid self-referential ADDR expressions rejected by this Andes linker.
    table = table.replace('ORIGIN(REGION_BURN) + ', '')
    source = source[:start] + source[end:]
    source = source.replace('.magic_boot :', '.magic_boot 0x3000B000 :', 1)
    source = source.replace('KEEP(*(.magic_boot_hdr ))', 'KEEP(*(.sv_stock_boot_hdr))')
    source = source.replace('KEEP(*(.magic_boot ))', 'KEEP(*(.sv_stock_boot))')
    source = source.replace('ENTRY(_start)', 'ENTRY(SV6X66_StockEntryHeader)\nEXTERN(SV6X66_StockEntryHeader)')
    source = source.replace('SECTIONS\n{', 'SECTIONS\n{\n    /DISCARD/ : { *(.magic_boot_hdr) *(.magic_boot) }', 1)
    source = source.replace('#include "layout_flash_ilm_dlm.lds.S"',
                            '#include "../../../projects/lite_mac/ld/layout_flash_ilm_dlm.lds.S"')
    marker = '    FLASH_SIZE = LOADADDR(.tbss) - FLASH_BEGIN;'
    if source.count(marker) != 1:
        raise ValueError('unsupported SDK linker bookkeeping')
    source = source.replace(marker, table + '\n    __stock_app_end = ADDR(.fix_table_section) + SIZEOF(.fix_table_section);\n'
                            '    FLASH_SIZE = __stock_app_end - FLASH_BEGIN;')
    source = source.replace('ASSERT((__check_main_size<=SETTING_PARTITION_MAIN_SIZE)',
                            'ASSERT(((__stock_app_end - 0x3000B000)<=SETTING_PARTITION_MAIN_SIZE)')
    source += '\nASSERT(ADDR(.magic_boot) == 0x3000B000, "stock application entry misplaced");\n'
    source += 'ASSERT(__stock_app_end <= 0x300BA000, "stock application exceeds UART partition");\n'
    return source

def uart_image(elf):
    """Serialize file-backed allocated sections at their linked flash LMAs."""
    if elf[:6] != b'\x7fELF\x01\x01' or len(elf) < 52:
        raise ValueError('expected ELF32 little-endian firmware')
    header = struct.unpack_from('<HHIIIIIHHHHHH', elf, 16)
    if header[1] != 167:
        raise ValueError('expected Andes NDS32 ELF')
    phoff, shoff = header[4:6]
    phsize, phcount, shsize, shcount, names_index = header[8:13]
    if phsize != 32 or shsize != 40 or names_index >= shcount:
        raise ValueError('invalid ELF table geometry')
    if phoff + phcount * phsize > len(elf) or shoff + shcount * shsize > len(elf):
        raise ValueError('truncated ELF tables')
    programs = [struct.unpack_from('<8I', elf, phoff + i * phsize) for i in range(phcount)]
    sections = [struct.unpack_from('<10I', elf, shoff + i * shsize) for i in range(shcount)]
    strings = sections[names_index]
    names = elf[strings[4]:strings[4] + strings[5]]
    ranges = []
    addresses = {}
    for section in sections:
        name_offset, kind, flags, addr, offset, size = section[:6]
        if not flags & 2 or kind == 8 or not size:  # allocated, file-backed only
            continue
        if offset + size > len(elf) or name_offset >= len(names):
            raise ValueError('invalid ELF section')
        name = names[name_offset:].split(b'\0', 1)[0].decode('ascii')
        matches = [p for p in programs if p[0] == 1 and p[2] <= addr and addr + size <= p[2] + p[5]
                   and p[1] <= offset and offset + size <= p[1] + p[4]]
        if len(matches) != 1:
            raise ValueError('ambiguous load address for ' + name)
        segment = matches[0]
        lma = segment[3] + addr - segment[2]
        if lma < APP_XIP or lma + size > FS_XIP:
            raise ValueError('section outside stock application partition: ' + name)
        ranges.append((lma, lma + size, offset))
        addresses[name] = {'vma': addr, 'lma': lma, 'bytes': size}
    if addresses.get('.magic_boot', {}).get('vma') != APP_XIP or not ranges:
        raise ValueError('stock startup is not linked at 0x3000B000')
    if addresses.get('.text', {}).get('vma', 0) < APP_XIP:
        raise ValueError('XIP code is not linked for stock application')
    ranges.sort()
    if ranges[0][0] != APP_XIP or any(a[1] > b[0] for a, b in zip(ranges, ranges[1:])):
        raise ValueError('overlapping or misplaced flash sections')
    payload = bytearray(ranges[-1][1] - APP_XIP)
    for lo, hi, offset in ranges:
        payload[lo - APP_XIP:hi - APP_XIP] = elf[offset:offset + hi - lo]
    if payload[0x30:0x32] == b'\xb6\x20':
        raise ValueError('unsafe vendor pre-ILM store in stock startup')
    if payload[:4] != b'\x48\x00\x00\x02':
        raise ValueError('stock application startup branch missing')
    return bytes(payload), addresses
