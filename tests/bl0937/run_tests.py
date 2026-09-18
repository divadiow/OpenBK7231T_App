#!/usr/bin/env python3
"""Compile actual BL0937/calibration/shared-meter C with deterministic I/O stubs.
No hardware emulation and no simulated replacement for the measurement logic.
"""
from __future__ import annotations
import argparse, itertools, json, shutil, subprocess, sys
from pathlib import Path
from prepare import overlay
SCENARIOS = ('nominal preemption startup_late startup_early wrap zero powermax '
 'legacy_calibration settling threshold delays calibration_isolation restart '
 'inverse_restart restart_pending retry_exhaustion second_read_preemption delayed_sel_before '
 'delayed_sel_after publish_delay partial movingavg relay_policy bad_intervals '
 'twin random_timing random_load counter_wrap rate_precision').split()
def main() -> int:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, required=True, help='Source root containing src/driver')
    ap.add_argument('--out', type=Path, default=Path('bl0937-test-results'))
    ap.add_argument('--baseline', action='store_true', help='Pre-fix regression reproduction only')
    ap.add_argument('--quick', action='store_true', help='One GCC configuration')
    args=ap.parse_args(); source=args.source.resolve(); out=args.out.resolve(); out.mkdir(parents=True,exist_ok=True)
    support=Path(__file__).resolve().parent; overlay(source, out/'overlay')
    compilers=['gcc'] if args.quick else ['gcc','clang']
    configurations=[(1,32,0,1,1,False)] if args.quick else list(itertools.product([1,10],[16,32],[0,1],[0,1],[0,1],[False]))
    extra=[] if args.quick else [(1,32,0,1,1,True,1000),(7,32,0,1,1,False,128),(4,32,0,1,1,False,250)]
    results=[]; builds=[]
    for cc in compilers:
        if shutil.which(cc) is None: raise SystemExit(f'Required compiler not found: {cc}')
        for ms,bits,beken,avg,twin,fallback,hz in [(*c,1000//c[0]) for c in configurations]+extra:
            tag=f'{cc}-hz{hz}-b{bits}-bk{beken}-avg{avg}-twin{twin}-fallback{int(fallback)}'; exe=out/tag
            cmd=[cc,'-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-missing-field-initializers',
              '-fsanitize=undefined','-fno-sanitize-recover=all',f'-DTEST_FIXED={int(not args.baseline)}',f'-DTICK_MS={ms}',f'-DTICK_HZ={hz}',
              f'-DTICK_BITS={bits}',f'-DPLATFORM_BEKEN={beken}',f'-DENABLE_BL_TWIN={twin}']
            if avg: cmd+=['-DENABLE_BL_MOVINGAVG=1']
            if fallback: cmd+=['-U__GCC_ATOMIC_INT_LOCK_FREE']
            cmd += [f'-I{support}',f'-I{out/"overlay"}',str(support/'harness.c'),'-lm','-o',str(exe)]
            c=subprocess.run(cmd,text=True,capture_output=True,timeout=45)
            (out/f'{tag}.build.txt').write_text(' '.join(cmd)+'\n'+c.stdout+c.stderr)
            builds.append({'configuration':tag,'returncode':c.returncode})
            if c.returncode: print(f'COMPILE FAILED {tag}: {c.stderr[:2000]}',flush=True);continue
            cases=['nominal','preemption','startup_late','startup_early','legacy_calibration'] if args.baseline else (['rate_precision'] if hz in (128,250) else SCENARIOS)
            for name,inverted in itertools.product(cases,[0,1]):
                t=subprocess.run([str(exe),name,str(inverted)],capture_output=True,text=True,timeout=15)
                results.append({'configuration':tag,'scenario':name,'inverted':inverted,'returncode':t.returncode,'stdout':t.stdout,'stderr':t.stderr})
                if t.returncode: print(f'FAIL {tag}/{name}/{inverted}: {t.stderr[:1500]}',flush=True)
            print(f'{tag}: compiled, {len(cases)*2} scenarios executed',flush=True)
    bad=[x for x in results if x['returncode']]; failed_builds=[x for x in builds if x['returncode']]
    summary={'source':str(source),'builds':len(builds),'failed_builds':len(failed_builds),'executions':len(results),'passed':len(results)-len(bad),'failed':len(bad)}
    (out/'results.json').write_text(json.dumps({'summary':summary,'builds':builds,'results':results},indent=2))
    print(json.dumps(summary,indent=2));return int(bool(bad or failed_builds))
if __name__=='__main__': sys.exit(main())
