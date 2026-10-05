#!/usr/bin/env python3
"""Build OpenBeken using an unchanged ICOMM SDK checkout and an app overlay."""
import argparse
import os
import json
import hashlib
import re
from pathlib import Path
import shutil
import subprocess

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--toolchain', required=True, type=Path, help='directory containing nds32le-elf-gcc')
    parser.add_argument('--jobs', default=4, type=int)
    parser.add_argument("--version", default="SV6166F_dev")
    parser.add_argument("--build-dir", type=Path, help="optional SDK staging directory (native Linux storage speeds WSL builds)")
    parser.add_argument("--layout", choices=["obk", "stock-ckw04"], default="obk",
                        help="obk: combined vendor bootloader/OTA; stock-ckw04: raw app and padded XMODEM transport")
    args = parser.parse_args()
    if args.jobs < 1 or args.version in (".", "..") or not re.fullmatch(r"[A-Za-z0-9_.+-]+", args.version):
        parser.error("jobs must be positive and version must contain only letters, numbers, _, ., + or -")
    app = Path(__file__).resolve().parents[2]
    sdk = app / 'sdk/OpenSV6X66/platform/mcu/sv6266/sdk'
    stage = args.build_dir.resolve() if args.build_dir else app / 'output/sv6x66' / args.layout / 'sdk'
    overlay_sources = [app / name for name in ('src', 'include', 'platforms', 'libraries')]
    if (stage == sdk or sdk in stage.parents or stage in sdk.parents or stage == app or stage in app.parents or
            any(stage == source or source in stage.parents for source in overlay_sources)):
        parser.error('build directory must not replace the SDK or application sources')
    if not (sdk / 'Makefile').exists():
        parser.error('initialize sdk/OpenSV6X66 first')
    compiler = args.toolchain.resolve() / 'nds32le-elf-gcc'
    compiler_version = subprocess.check_output([str(compiler), '--version'], text=True)
    print(compiler_version, end='')
    ownership = stage / '.obk-sdk-stage'
    if args.build_dir and stage.exists() and not ownership.exists() and any(stage.iterdir()):
        parser.error('custom build directory must be empty or an existing OpenBeken staging directory')
    subprocess.run(['git', '-c', 'core.autocrlf=true', '-C', str(sdk.parents[3]), 'diff', '--exit-code', '--quiet'], check=True)
    shutil.copytree(sdk, stage, dirs_exist_ok=True)
    ownership.write_text('OpenSV6166F SDK staging; generated files only\n')
    overlay = stage / 'obk_app'
    for name in ['src', 'include', 'platforms', 'libraries']:
        shutil.copytree(app / name, overlay / name, dirs_exist_ok=True)
    shutil.copyfile(app / 'platforms/SV6X66/module.mk', overlay / 'module.mk')
    from stock_layout import linker_overlay, uart_image, stock_xmodem_image
    from stock_phy import phy_overlay, audit_final_phy
    if args.layout == 'stock-ckw04':
        phy_path = stage / 'components/drv/phy/libphy.a'
        adapted_phy, phy_adaptation = phy_overlay(phy_path.read_bytes())
        phy_path.write_bytes(adapted_phy)
        template = sdk / 'projects/lite_mac/ld/lite_ilm_flash.lds.S'
        (overlay / 'platforms/SV6X66/stock_flash.lds.S').write_text(linker_overlay(template.read_text()))
    (stage / 'build/project_cfg.mk').write_text('include obk_app/platforms/SV6X66/project.mk\n')
    # Make does not track command-line flag changes. Rebuild generated objects
    # whenever the overlay's flags/version change, preserving all source files.
    settings = [Path(__file__), app / 'platforms/SV6X66/project.mk', app / 'platforms/SV6X66/module.mk', app / 'platforms/obk_main.mk', app / 'platforms/SV6X66/stock_layout.py', app / 'platforms/SV6X66/stock_phy.py',
                sdk / 'projects/lite_mac/ld/lite_ilm_flash.lds.S']
    fingerprint = hashlib.sha256(b''.join(path.read_bytes() for path in settings) +
                                 args.version.encode() + args.layout.encode() + str(compiler).encode() + compiler_version.encode()).hexdigest()
    stamp = stage / '.obk-build-settings'
    if not stamp.exists() or stamp.read_text() != fingerprint:
        generated = stage / 'out'
        if generated.is_symlink():
            parser.error('refusing to clean a symlinked generated output directory')
        if generated.is_dir():
            shutil.rmtree(generated)
    stamp.write_text(fingerprint)

    env = dict(os.environ, PATH=str(args.toolchain.resolve()) + os.pathsep + os.environ['PATH'])
    (stage / 'build_error.log').write_text('')
    target = str(stage / 'image/OpenSV6166F.elf') if args.layout == 'stock-ckw04' else 'main-build'
    command = ['make', '-j' + str(args.jobs), target,
               'OBK_LAYOUT=' + args.layout,
               'OBK_APP_VERSION=' + args.version,
               'MQTT_EN=0', 'HTTPD_EN=0', 'HTTPC_EN=0', 'SSL_EN=0', 'MBED_EN=0',
               'IPERF3_EN=0', 'SMARTCONFIG_EN=0', 'PING_EN=0', 'TFTP_EN=0',
               'BUILD_OPTION=RELEASE', 'BUILD_SHOW_ILM_INFO=0', 'BUILD_SHOW_DLM_INFO=0']
    result = subprocess.run(command, cwd=stage, env=env)
    if result.returncode:
        raise SystemExit(result.returncode)
    from package import wrap_image
    image_dir = stage / 'image'
    image = image_dir / 'OpenSV6166F.bin'
    # A successful link must contain the real port, not weak fallback HALs.
    symbols = subprocess.check_output([str(compiler.parent / 'nds32le-elf-nm'), '--defined-only',
                                       str(image_dir / 'OpenSV6166F.elf')], text=True)
    types = {line.split()[-1]: line.split()[-2] for line in symbols.splitlines() if len(line.split()) >= 3}
    required = ['APP_Init', 'Main_Init', 'HAL_PIN_Setup_Output', 'HAL_PIN_PWM_Start', 'HAL_PIN_PWM_Update',
                'HAL_ConnectToWiFi', 'HAL_SetupWiFiOpenAccessPoint', 'HAL_WiFi_SetupStatusCallback',
                'MQTT_init', 'MQTT_QueuePublish', 'LED_GetDimmer', 'SV6X66_OTAStartup',
                'HAL_Configuration_ReadConfigMemory', 'HAL_Configuration_SaveConfigMemory',
                'HAL_FlashVars_IncreaseBootCount', 'HAL_FlashVars_SaveChannel', 'HAL_RebootModule']
    if args.layout == 'stock-ckw04':
        required += ['SV6X66_StockMount', 'SV6X66_StockHeaderValid', 'SV6X66_StockEntryHeader', 'SV6X66_StockEntry',
                     '__wrap__soc_clk_init', '__wrap__soc_io_init', '__wrap_xip_init', '__wrap_xip_leave', '__wrap_xip_enter',
                     'flash_init', 'flash_page_program', 'flash_sector_erase']
        if any(name in types for name in ['_soc_clk_init', '_soc_io_init', 'OS_PsramInit', 'xip_init', 'xip_leave', 'xip_enter']):
            raise RuntimeError('stock firmware contains incompatible vendor clock/XIP header consumers')
        if 'FS_init' in types or 'FS_reset' in types:
            raise RuntimeError('stock firmware must not contain vendor filesystem initialization/formatting')
    missing = [name for name in required if types.get(name) != 'T']
    if missing:
        raise RuntimeError('firmware is missing strong port implementations: ' + ', '.join(missing))
    destination = app / 'output' / args.version
    stem = 'OpenSV6166F_' + args.version
    if args.layout == 'stock-ckw04':
        destination /= 'stock-ckw04'
        stem += '_stock-ckw04'
    destination.mkdir(parents=True, exist_ok=True)
    artifacts = []
    for extension in ['elf', 'map']:
        artifact = destination / (stem + '.' + extension)
        shutil.copyfile(image_dir / ('OpenSV6166F.' + extension), artifact)
        artifacts.append(artifact)
    if args.layout == 'stock-ckw04':
        elf_data = (image_dir / 'OpenSV6166F.elf').read_bytes()
        phy_adaptation['linked_audit'] = audit_final_phy(elf_data)
        payload, sections = uart_image(elf_data)
        locations = {line.split()[-1]: int(line.split()[0], 16) for line in symbols.splitlines()
                     if len(line.split()) >= 3 and re.fullmatch('[0-9a-fA-F]+', line.split()[0])}
        ram_sections = [sections[name] for name in ['.fast_boot_code', '.prog_in_sram']]
        ram_required = ['__wrap__soc_clk_init', '__wrap__soc_io_init', '__wrap_xip_init', '__wrap_xip_leave', '__wrap_xip_enter',
                        'flash_init', 'flash_page_program', 'flash_sector_erase']
        if any(not any(section['vma'] <= locations[name] < section['vma'] + section['bytes']
                       for section in ram_sections) for name in ram_required):
            raise RuntimeError('stock clock/XIP/flash routines must execute from initialized SRAM')
        uart = destination / (stem + '_uart.bin')
        # Independent objcopy must produce exactly the audited section payload.
        check = image_dir / 'OpenSV6166F.uart-check.bin'
        subprocess.run([str(compiler.parent / 'nds32le-elf-objcopy'), '-O', 'binary',
                        str(image_dir / 'OpenSV6166F.elf'), str(check)], check=True)
        if check.read_bytes() != payload:
            raise RuntimeError('objcopy and audited UART serialization differ')
        uart.write_bytes(payload)
        artifacts.append(uart)
        # The stock receiver skips stream offsets below B000; raw app is not
        # directly uploadable. Keep it for tools that explicitly write at B000.
        xmodem = destination / (stem + '_xmodem.bin')
        xmodem.write_bytes(stock_xmodem_image(payload))
        artifacts.append(xmodem)
    else:
        artifact = destination / (stem + '.bin')
        shutil.copyfile(image, artifact)
        artifacts.append(artifact)
        ota = destination / (stem + '.ota')
        ota.write_bytes(wrap_image(image.read_bytes()))
        artifacts.append(ota)
    sdk_commit = subprocess.check_output(['git', '-C', str(sdk.parents[3]), 'rev-parse', 'HEAD'], text=True).strip()
    manifest = {'target': 'SV6166F', 'module': 'CKW04', 'sdk_commit': sdk_commit,
                'app_version': args.version, 'layout': args.layout, 'hardware_tested': False, 'verified_symbols': required,
                'uart_flash_offset': 0xB000 if args.layout == 'stock-ckw04' else None,
                'ota_supported': args.layout == 'obk',
                'toolchain': subprocess.check_output([str(compiler), '--version'], text=True).splitlines()[0],
                'artifacts': {f.name: {'bytes': f.stat().st_size, 'sha256': hashlib.sha256(f.read_bytes()).hexdigest()}
                              for f in artifacts}}
    if args.layout == 'stock-ckw04':
        manifest['stock_xmodem'] = {
            'artifact': stem + '_xmodem.bin',
            'application_file_offset': 0xB000,
            'application_flash_offset': 0xB000,
            'discarded_prefix_bytes': 0xB000,
            'prefix_fill': 255,
            'receiver_addressing': 'absolute stream offset; no application base added',
            'raw_uart_artifact_directly_uploadable': False}
        manifest['protected_flash_range'] = [0, 0xB000]
        manifest['application_flash_range'] = [0xB000, 0xBA000]
        manifest['sections'] = sections
        manifest['stock_phy_adaptation'] = phy_adaptation
        manifest['stock_startup'] = {'entry': 'factory-compatible sequence; no pre-ILM store',
                                     'clock_policy': 'preserve stock loader initialization',
                                     'compiled_xtal_mhz': 25, 'pinmux_policy': 'preserve stock loader UART route',
                                     'psram_heap': 'disabled; no stock SDK-compatible heap header',
                                     'xip_policy': 'capture inherited mode once; restore around flash operations'}
        manifest['rf_calibration'] = 'SDK defaults; factory calibration is preserved but not interpreted'
    (destination / (stem + '.json')).write_text(json.dumps(manifest, indent=2) + '\n')
    print('Firmware artifacts: ' + str(destination))

if __name__ == '__main__':
    main()
