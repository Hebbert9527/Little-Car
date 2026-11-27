from pathlib import Path
import sys
root=Path(__file__).resolve().parents[2]
runner=root.parent/'run_cortex_tests.py'
s=runner.read_text(encoding='utf-8').replace('tests/navigation/test_maze.c','build/static_walls_5m/debug_maze.c')
sys.argv=[str(runner),'navigation_maze']
exec(compile(s,str(runner),'exec'),{'__file__':str(runner),'__name__':'__main__'})
