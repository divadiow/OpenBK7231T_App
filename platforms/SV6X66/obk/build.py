#!/usr/bin/env python3
"""Manual-only CKW04 dual-linked OBK build using the bounded unmodified SDK."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile

SOURCE = Path(__file__).resolve().parent
ROOT = SOURCE.parents[2]
from stock_layout import linker_overlay, uart_image

BANKS = {'A': 0xB000, 'B': 0xA6000}
SIZE = 0x9B000

def run(command, **kw):
    print('+ ' + ' '.join(map(str, command)), flush=True)
    return subprocess.run(command, check=True, **kw)

def preserve_failed_link(stage, output, bank, version, nm='', sections=None):
    # Preserve real failed-build evidence, and expose bounded excerpts in the
    # GUI log: the builder deliberately does not export firmware after failure.
    destination = Path(tempfile.mkdtemp(prefix=f'FAILED_OpenSV6X66_{version}_{bank}_', dir=output))
    records = {}
    for relative in ('out/linkscript.lds', 'image/OpenSV6X66.map', 'image/OpenSV6X66.elf',
                     'out/obk_app/platforms/SV6X66/obk/identity.o'):
        source = stage / relative
        print(f'FAILED build evidence: {source}: exists={source.is_file()}', flush=True)
        if not source.is_file():
            continue
        data = source.read_bytes()
        shutil.copy2(source, destination / source.name)
        records[relative] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
        if source.suffix in ('.lds', '.map'):
            lines = data.decode('utf-8', errors='replace').splitlines()
            selected = set()
            for index, line in enumerate(lines):
                if any(word in line for word in ('.obk_identity', 'sv6x66_identity', '__stock_app_end',
                                                'identify_ram', 'erase_ram', 'program_ram',
                                                '.fast_boot_code', '.prog_in_sram')):
                    selected.update(range(max(0, index-2), min(len(lines), index+3)))
            for index in sorted(selected)[:80]:
                print(f'FAILED {source.name}:{index+1}: {lines[index][:300]}', flush=True)
    if nm:
        (destination / 'OpenSV6X66.nm').write_text(nm)
        for line in [line for line in nm.splitlines() if any(name in line for name in
                     ('identify_ram', 'erase_ram', 'program_ram', 'flash_init', '__wrap_xip', '__wrap_memcpy',
                      'HAL_PIN_', 'HAL_AttachInterrupt', 'HAL_DetachInterrupt'))][:40]:
            print('FAILED nm candidate: ' + line[:300], flush=True)
    for name, section in (sections or {}).items():
        if name in ('.fast_boot_code', '.prog_in_sram'):
            print(f'FAILED RAM range {name}: VMA=0x{section["vma"]:08x} bytes=0x{section["bytes"]:x}', flush=True)
    (destination / 'failure-evidence.json').write_text(json.dumps(
        {'build_failed': True, 'bank': bank, 'version': version, 'files': records,
         'sections': sections or {}}, indent=2) + '\n')
    print(f'Failed-link evidence retained at {destination}; not a deployable artifact.', flush=True)

def toolchain(argument):
    candidates = [argument, os.environ.get('SV6X66_TOOLCHAIN')]
    compiler = shutil.which('nds32le-elf-gcc')
    if compiler:
        candidates.append(str(Path(compiler).parent))
    candidates.append(str(ROOT / 'sdk/OpenSV6X66/toolchain/extracted/nds32le-elf-mculib-v3/bin'))
    for candidate in candidates:
        if candidate and (Path(candidate) / 'nds32le-elf-gcc').is_file():
            return Path(candidate).resolve()
    raise SystemExit('Andes NDS32 compiler missing: set SV6X66_TOOLCHAIN to existing nds32le-elf toolchain bin directory; no download is attempted')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--version', required=True)
    parser.add_argument('--variant', default='0')
    parser.add_argument('--toolchain')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    if args.variant != '0':
        parser.error('only the minimal CKW04 default variant 0 is implemented')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]*', args.version) or args.jobs < 1:
        parser.error('invalid version or jobs')
    tools = toolchain(args.toolchain)
    sdk = ROOT / 'sdk/OpenSV6X66/sdk'
    if not (sdk / 'Makefile').is_file():
        raise SystemExit('bounded sdk/OpenSV6X66/sdk is missing')
    output = ROOT / 'output' / args.version
    output.mkdir(parents=True, exist_ok=True)
    raws = {}
    evidence = {}
    for bank, offset in BANKS.items():
        stage = ROOT / 'output/SV6X66_OBK' / bank / 'sdk'
        marker = stage.parent / '.obk-owned-stage'
        if stage.exists():
            if not marker.is_file() or not stage.resolve().is_relative_to((ROOT / 'output/SV6X66_OBK').resolve()):
                raise SystemExit('refusing to replace an unowned or escaped SV6X66 build stage')
            shutil.rmtree(stage)
        stage.mkdir(parents=True, exist_ok=True)
        marker.write_text('generated OpenSV6X66 application stage\n')
        shutil.copytree(sdk, stage, dirs_exist_ok=True)
        overlay = stage / 'obk_app'
        for name in ('src', 'include', 'platforms'):
            shutil.copytree(ROOT / name, overlay / name, dirs_exist_ok=True)
        shutil.copy2(SOURCE / 'module.mk', overlay / 'module.mk')
        text = linker_overlay((sdk / 'projects/lite_mac/ld/lite_ilm_flash.lds.S').read_text(),
                              0x30000000 + offset, 0x30000000 + offset + SIZE)
        text = text.replace('../../../projects/lite_mac/ld/layout_flash_ilm_dlm.lds.S',
                            '../../../../projects/lite_mac/ld/layout_flash_ilm_dlm.lds.S')
        identity_anchor = '__stock_app_end = ADDR(.fix_table_section) + SIZEOF(.fix_table_section);'
        if text.count(identity_anchor) != 1:
            raise SystemExit('unsupported SDK linker identity insertion point')
        text = text.replace(identity_anchor,
                            '.obk_identity ALIGN(4) : { KEEP(*(.obk_identity)) } > REGION_BURN\n'
                            '__stock_app_end = ADDR(.obk_identity) + SIZEOF(.obk_identity);\n'
                            f'__obk_image_bytes = __stock_app_end - 0x{0x30000000 + offset:08x};')
        text += '\nASSERT(SIZEOF(.obk_identity) == 64, "OBK identity trailer missing");\n'
        (overlay / 'platforms/SV6X66/obk/application.lds.S').write_text(text)
        (stage / 'build/project_cfg.mk').write_text('include obk_app/platforms/SV6X66/obk/project.mk\n')
        environment = dict(os.environ, PATH=str(tools) + os.pathsep + os.environ['PATH'])
        elf = stage / 'image/OpenSV6X66.elf'
        try:
            run(['make', '-j' + str(args.jobs), str(elf), 'APP_VERSION=' + args.version,
                 'OBK_BANK=' + hex(offset), 'OBK_VARIANT=0', 'MQTT_EN=0', 'HTTPD_EN=0',
                 'HTTPC_EN=0', 'SSL_EN=0', 'MBED_EN=0', 'IPERF3_EN=0', 'SMARTCONFIG_EN=0',
                 'PING_EN=0', 'TFTP_EN=0', 'OTA_EN=0', 'BUILD_OPTION=RELEASE',
                 'BUILD_SHOW_ILM_INFO=0', 'BUILD_SHOW_DLM_INFO=0'], cwd=stage, env=environment)
        except subprocess.CalledProcessError:
            try:
                preserve_failed_link(stage, output, bank, args.version)
            except OSError as error:
                print(f'Unable to preserve all failed-build evidence: {error}', file=sys.stderr)
            raise
        nm, sections = '', {}
        try:
            raw, sections = uart_image(elf.read_bytes(), 0x30000000 + offset, 0x30000000 + offset + SIZE)
            identity = b'OBKSV6X1' + struct.pack('<4I', offset, SIZE, len(raw), 1) + bytes(40)
            if raw[-64:] != identity:
                raise SystemExit('bank identity trailer/link length mismatch')
            nm = subprocess.check_output([str(tools / 'nds32le-elf-nm'), '--defined-only', str(elf)], text=True)
            symbols = {line.split()[-1]: int(line.split()[0], 16) for line in nm.splitlines()
                       if re.match(r'^[0-9a-fA-F]+\s', line)}
            symbol_types = {line.split()[-1]: line.split()[1] for line in nm.splitlines()
                            if re.match(r'^[0-9a-fA-F]+\s', line) and len(line.split()) == 3}
            # Compiling pins.c into an archive is insufficient: weak generic
            # members can satisfy callers without extracting the real HAL.
            # Require the used pin surface to be global code in this XIP bank.
            for name in ('HAL_PIN_GetPinNameAlias', 'HAL_PIN_ReadDigitalInput', 'HAL_PIN_SetOutputValue',
                         'HAL_PIN_Setup_Input', 'HAL_PIN_Setup_Input_Pullup', 'HAL_PIN_Setup_Input_Pulldown',
                         'HAL_PIN_Setup_Output', 'HAL_PIN_Setup_Output_Initial', 'HAL_PIN_CanThisPinBePWM',
                         'HAL_PIN_PWM_Start', 'HAL_PIN_PWM_Update', 'HAL_PIN_PWM_Stop',
                         'HAL_AttachInterrupt', 'HAL_DetachInterrupt'):
                address = symbols.get(name, -1)
                in_bank_text = any(section['vma'] <= address < section['vma'] + section['bytes']
                                   and 0x30000000 + offset <= section['vma']
                                   and section['vma'] + section['bytes'] <= 0x30000000 + offset + SIZE
                                   for key, section in sections.items() if key == '.text')
                if symbol_types.get(name) != 'T' or not in_bank_text:
                    raise SystemExit(f'required GPIO HAL rejected: {name}: '
                                     f'type={symbol_types.get(name, "missing")} address={address:#x}; '
                                     'expected strong global text in the linked bank')
            for name in ('identify_ram', 'erase_ram', 'program_ram', '__wrap_memcpy', 'flash_init', 'flash_wait_spi_busy',
                         'flash_wait_spi_status_register', 'flash_write_enable', 'flash_sector_erase', 'flash_page_program',
                         '__wrap_xip_enter', '__wrap_xip_leave', '__wrap__soc_clk_init', '__wrap__soc_io_init'):
                address = symbols.get(name, -1)
                if not any(section['vma'] <= address < section['vma'] + section['bytes']
                           for key, section in sections.items() if key in ('.fast_boot_code', '.prog_in_sram')):
                    reason = 'missing exact symbol' if address == -1 else f'outside allowed RAM at 0x{address:08x}'
                    raise SystemExit(f'required RAM flash/startup symbol rejected: {name}: {reason}')
        except (ValueError, SystemExit, subprocess.CalledProcessError):
            try:
                preserve_failed_link(stage, output, bank, args.version, nm, sections)
            except OSError as error:
                print(f'Unable to preserve all post-link failure evidence: {error}', file=sys.stderr)
            raise
        stem = 'OpenSV6X66_' + args.version + '_' + bank
        (output / (stem + '.bank.bin')).write_bytes(raw)
        shutil.copy2(elf, output / (stem + '.elf'))
        shutil.copy2(stage / 'image/OpenSV6X66.map', output / (stem + '.map'))
        (output / (stem + '.nm')).write_text(nm)
        raws[bank] = raw
        evidence[bank] = {'bank': bank, 'base': offset, 'size': len(raw),
                          'sha256': hashlib.sha256(raw).hexdigest(), 'sections': sections}
    header = struct.pack('<5I', 0x73737600, len(raws['A']), len(raws['B']), 1, 0)
    header += hashlib.md5(raws['A']).hexdigest().encode() + hashlib.md5(raws['B']).hexdigest().encode()
    artifact = output / ('OpenSV6X66_' + args.version + '.sv6ota')
    # UART installation and HTTP OTA use the same bank-aware bundle. The .bin
    # is an application package, not a raw bank or a whole-flash backup.
    bundle = header + raws['A'] + raws['B']
    uart_artifact = output / ('OpenSV6X66_' + args.version + '.bin')
    artifact.write_bytes(bundle)
    uart_artifact.write_bytes(bundle)
    evidence.update({'format': 1, 'profile': 'CKW04', 'hardware_tested': False,
                     'flash_call_closure_disassembly_review_required': True,
                     'bundle_sha256': hashlib.sha256(bundle).hexdigest(),
                     'uart_application_file': uart_artifact.name, 'http_ota_file': artifact.name})
    (output / ('OpenSV6X66_' + args.version + '.manifest.json')).write_text(json.dumps(evidence, indent=2) + '\n')
    print('Manual-build artifacts only; inspect ELF/maps, recovery and hardware gates before flashing.')

if __name__ == '__main__':
    main()
