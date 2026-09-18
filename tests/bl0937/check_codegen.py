#!/usr/bin/env python3
"""Compile the actual counter/snapshot functions for representative MCU ISAs.
This checks code generation, not the vendor SDK runtime or peripheral hardware.
"""
import argparse,json,subprocess,re
from pathlib import Path

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--out',type=Path,default=Path('bl0937-codegen'))
    args=ap.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    s=(args.source/'src/driver/drv_bl0937.c').read_text()
    start=s.index('/* Each naturally aligned counter')
    end=s.index('static uint32_t BL0937_Elapsed',start)
    code='#include <stdint.h>\n#include <stdbool.h>\n#include <limits.h>\n#define NAN __builtin_nanf("")\n#define BL0937_SNAPSHOT_ATTEMPTS 4\ntypedef uint32_t portTickType;\nextern portTickType xTaskGetTickCount(void);\n'+s[start:end]+'\nbool CodegenSnapshot(bl0937_snapshot_t *p) {return BL0937_Snapshot(p);}\n'
    src=args.out/'probe.c';src.write_text(code)
    configs=[('arm9','--target=arm-none-eabi','-mcpu=arm968e-s'),('cortex-m0','--target=arm-none-eabi','-mcpu=cortex-m0','-mthumb'),('cortex-m3','--target=arm-none-eabi','-mcpu=cortex-m3','-mthumb'),('cortex-m4','--target=arm-none-eabi','-mcpu=cortex-m4','-mthumb'),('cortex-m33','--target=arm-none-eabi','-mcpu=cortex-m33','-mthumb'),('rv32im','--target=riscv32-none-elf','-march=rv32im','-mabi=ilp32'),('rv32ima','--target=riscv32-none-elf','-march=rv32ima','-mabi=ilp32')]
    results=[]
    for name,*flags in configs:
        out=args.out/(name+'.s')
        cmd=['clang',*flags,'-ffreestanding','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-unused-variable','-S',str(src),'-o',str(out)]
        p=subprocess.run(cmd,text=True,capture_output=True,timeout=30)
        assembly=out.read_text() if p.returncode==0 else ''
        helpers=bool(re.search(r'\b(?:bl|call)\s+.*__(?:atomic|sync)',assembly))
        ok=p.returncode==0 and not helpers
        row={'architecture':name,'passed':ok,'compile_status':p.returncode,'external_atomic_helpers':helpers,'stderr':p.stderr,'command':cmd};results.append(row)
        print(name, 'PASS' if ok else 'FAIL',p.stderr)
    (args.out/'results.json').write_text(json.dumps(results,indent=2))
    return int(not all(r['passed'] for r in results))
if __name__=='__main__':raise SystemExit(main())
