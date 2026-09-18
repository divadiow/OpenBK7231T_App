from pathlib import Path
import shutil,re
ROOT=Path(__file__).resolve().parent.parent

def overlay(source: Path, out: Path):
    names=['drv_bl0937.c','drv_pwrCal.c','drv_pwrCal.h','drv_bl_shared.c','drv_bl_shared.h']
    for n in names:
        p=out/'src/driver'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source/'src/driver'/n,p)
    for p in list((out/'src/driver').glob('*')):
        for inc in re.findall(r'^\s*#include\s+"([^"]+)"',p.read_text(),re.M):
            dest=(p.parent/inc).resolve()
            if not dest.exists():dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text('/* HOST TEST STUB: platform dependencies are supplied by support.h. */\n')
