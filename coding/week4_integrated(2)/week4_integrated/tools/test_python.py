"""Run integrated PC tests with the Python standard library (no GUI/BLE required)."""
from pathlib import Path
import os
import subprocess
import sys

def main():
    root=Path(__file__).resolve().parents[1]
    env=os.environ.copy()
    env['PYTHONDONTWRITEBYTECODE']='1'
    env['PYTHONUTF8']='1'
    subprocess.run([sys.executable,'-B','-m','unittest','discover','-s',str(root/'tests/pc'),'-p','test_*.py','-v'],
                   cwd=root,env=env,check=True)

if __name__=='__main__': main()
