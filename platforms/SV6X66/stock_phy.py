"""Pinned, build-only adaptation of the closed PHY driver's clock inputs."""
import hashlib
import struct

ORIGINAL_SHA256 = '83768f1dc8facc70e1a5c34d90a03dc62af2720d793a8e24a3ae317cd8f196fe'
OLD = [bytes.fromhex('46030000a001'), bytes.fromhex('46030000a002')]
NEW = [bytes.fromhex('440000199200'), bytes.fromhex('440000509200')]

def elf_sections(data):
    if data[:6] != b'\x7fELF\x01\x01' or len(data) < 52:
        raise ValueError('invalid PHY ELF')
    offset = struct.unpack_from('<I', data, 32)[0]
    stride, count, names_index = struct.unpack_from('<HHH', data, 46)
    if stride != 40 or names_index >= count or offset + stride * count > len(data):
        raise ValueError('invalid PHY section table')
    sections = [struct.unpack_from('<10I', data, offset + i * stride) for i in range(count)]
    names = sections[names_index]
    strings = data[names[4]:names[4] + names[5]]
    labels = [strings[s[0]:].split(b'\0', 1)[0] for s in sections]
    return sections, labels

def phy_overlay(data):
    if hashlib.sha256(data).hexdigest() != ORIGINAL_SHA256 or data[:8] != b'!<arch>\n':
        raise ValueError('unexpected SDK PHY archive; refusing adaptation')
    cursor = 8
    member = None
    while cursor + 60 <= len(data):
        header = data[cursor:cursor + 60]
        size = int(header[48:58])
        if header[58:60] != b'`\n' or cursor + 60 + size > len(data):
            raise ValueError('invalid SDK PHY archive member')
        if header[:16].strip().rstrip(b'/') == b'drv_phy.o':
            member = (cursor + 60, size)
            break
        cursor += 60 + size + (size & 1)
    if member is None:
        raise ValueError('SDK PHY archive member missing')
    base, size = member
    sections, labels = elf_sections(data[base:base + size])
    if labels.count(b'.text.drv_phy_cali') != 1:
        raise ValueError('SDK PHY calibration section missing')
    section = sections[labels.index(b'.text.drv_phy_cali')]
    begin, length = base + section[4], section[5]
    if section[4] + length > size:
        raise ValueError('invalid SDK PHY calibration bounds')
    code = data[begin:begin + length]
    result = bytearray(data)
    offsets = []
    for old, new, expected in zip(OLD, NEW, [0x2C, 0x7C]):
        if code.count(old) != 1 or code.find(old) != expected:
            raise ValueError('unexpected SDK PHY clock-load instructions')
        at = begin + expected
        result[at:at + 6] = new
        offsets.append(at)
    return bytes(result), {'member': 'drv_phy.o', 'section': '.text.drv_phy_cali',
                           'archive_offsets': offsets, 'section_offsets': [0x2C, 0x7C],
                           'original_sha256': ORIGINAL_SHA256,
                           'adapted_sha256': hashlib.sha256(result).hexdigest(),
                           'xtal_mhz': 25, 'bus_mhz': 80}

def audit_final_phy(data):
    sections, labels = elf_sections(data)
    table = sections[labels.index(b'.symtab')]
    strings_section = sections[table[6]]
    strings = data[strings_section[4]:strings_section[4] + strings_section[5]]
    for offset in range(table[4], table[4] + table[5], table[9]):
        name, addr, size, _, _, section_index = struct.unpack_from('<IIIBBH', data, offset)
        if strings[name:].split(b'\0', 1)[0] != b'drv_phy_cali':
            continue
        section = sections[section_index]
        at = section[4] + addr - section[3]
        if addr < section[3] or addr + size > section[3] + section[5] or at + size > len(data):
            raise ValueError('invalid final PHY function bounds')
        code = data[at:at + size]
        if any(old in code for old in OLD) or any(code[:0x100].count(new) != 1 for new in NEW):
            raise ValueError('final PHY clock inputs did not retain stock adaptation')
        return {'function': 'drv_phy_cali', 'address': addr, 'bytes': size,
                'verified_immediates_mhz': [25, 80]}
    raise ValueError('final PHY calibration function missing')
