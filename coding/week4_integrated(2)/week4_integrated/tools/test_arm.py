"""Compatibility entry point for integrated firmware HOST regressions.

Historical name retained for existing commands. These tests use host GCC and
HAL/RTOS doubles, not Cortex-M emulation. Use build.py for an actual ARM build.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cc',help='path to host GCC (or set CC)')
    args=ap.parse_args()
    root=Path(__file__).resolve().parents[1]
    env=os.environ.copy()
    if args.cc: env['CC']=args.cc
    for suite in ('navigation','sensors'):
        runner=root/'tests'/suite/'run.py'
        if not runner.is_file(): raise SystemExit(f'Missing integrated test runner: {runner}')
        print(f'Running HOST regression: {suite}',flush=True)
        subprocess.run([sys.executable,'-B',str(runner)],cwd=root,env=env,check=True)

if __name__=='__main__': main()
