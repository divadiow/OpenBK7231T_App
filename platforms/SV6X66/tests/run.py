#!/usr/bin/env python3
"""Run host checks against the port's production sources; no device access."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

app = Path(__file__).resolve().parents[3]
platform = Path('platforms/SV6X66')
include = platform / 'tests/include'
sdk = Path('sdk/OpenSV6X66/platform/mcu/sv6266/sdk/components')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default='gcc')
    parser.add_argument('--build-dir', type=Path)
    args = parser.parse_args()
    destination = args.build_dir or Path(tempfile.mkdtemp(prefix='obk-sv6166f-tests-'))
    destination.mkdir(parents=True, exist_ok=True)
    flags = ['-std=gnu11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined']
    cases = [
        ('ota', [], [platform / 'tests/test_ota.c', platform / 'ota_stage.c']),
        ('storage', ['-include', str(include / 'test_storage_env.h'), '-I' + str(include), '-I' + str(platform)],
         [platform / 'tests/test_storage.c', Path('src/hal/sv6x66/hal_flashConfig_sv6x66.c')]),
        ('vars', ['-DTEST_FLASHVARS', '-include', str(include / 'test_storage_env.h'), '-I' + str(include), '-I' + str(platform)],
         [platform / 'tests/test_vars.c', platform / 'tests/test_storage.c',
          Path('src/hal/sv6x66/hal_flashConfig_sv6x66.c'), Path('src/hal/sv6x66/hal_flashVars_sv6x66.c')]),
        ('mqtt_dispatch', ['-pthread', '-include', str(include / 'test_net_env.h'), '-I' + str(include)],
         [platform / 'tests/test_mqtt_dispatch.c', platform / 'mqtt_dispatch.c']),
        ('stock_mount', ['-DSV6X66_STOCK_CKW04=1', '-DFLASH_CTL_v2=1', '-I' + str(platform / 'tests/stock/include'),
                         '-I' + str(sdk), '-I' + str(sdk / 'drv'),
                         '-I' + str(sdk / 'bsp/soc/ssv6006'), '-I' + str(sdk / 'bsp/soc/ssv6006/ASICv2'),
                         '-I' + str(sdk / 'fsal'), '-I' + str(sdk / 'fsal/spiffs'), '-I' + str(include)],
         [platform / 'tests/test_stock_mount.c', platform / 'stock_mount.c']),
        ('stock_runtime', ['-DSV6X66_STOCK_CKW04=1', '-include', str(include / 'test_stock_runtime_env.h')],
         [platform / 'tests/test_stock_runtime.c', platform / 'stock_runtime.c']),
        ('pins', ['-DPLATFORM_SV6X66=1', '-I' + str(include), '-Isrc', '-I' + str(sdk),
                  '-I' + str(sdk / 'drv'), '-I' + str(sdk / 'bsp/soc/ssv6006'),
                  '-I' + str(sdk / 'bsp/soc/ssv6006/ASICv2')],
         [platform / 'tests/test_pins.c', Path('src/hal/sv6x66/hal_pins_sv6x66.c')]),
    ]
    mount_flags = next(extra for name, extra, sources in cases if name == 'stock_mount')
    cases.append(('stock_mount_real', mount_flags + ['-w', '-include', 'inttypes.h'],
                  [platform / 'tests/test_stock_mount_real.c', platform / 'stock_mount.c'] +
                  sorted((app / sdk / 'fsal/spiffs').glob('spiffs_*.c'))))
    for name, extra, sources in cases:
        executable = destination.resolve() / name
        subprocess.run([args.cc] + flags + extra + [str(p) for p in sources] + ['-o', str(executable)], cwd=app, check=True)
        runs = (['0'], ['1'], ['2']) if name == 'vars' else (['0'], ['2'], ['4'], ['6']) if name == 'stock_runtime' else ([],)
        for arguments in runs:
            subprocess.run([str(executable)] + arguments, cwd=app, check=True, timeout=30)
    for test in ['test_package.py', 'test_build.py', 'test_stock_layout.py', 'test_stock_xmodem.py', 'test_stock_phy.py', 'test_startup.py', 'test_wifi_ap.py']:
        subprocess.run([sys.executable, str(platform / 'tests' / test)], cwd=app, check=True)
    subprocess.run([sys.executable, str(platform / 'tests/test_startup.py'), '--stock'], cwd=app, check=True)
    print('SV6166F host checks passed')

if __name__ == '__main__':
    main()
