import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
SPEC=importlib.util.spec_from_file_location('w4build',Path(__file__).resolve().parents[2]/'tools/build.py')
b=importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(b)
class BuildTests(unittest.TestCase):
    def test_explicit_missing_compiler_is_clear(self):
        with self.assertRaisesRegex(SystemExit,'does not exist'):
            b.find_gcc('Z:/missing/arm-none-eabi-gcc.exe')
    def test_search_supports_standalone_arm_layout(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); candidate=root/'arm-gnu-toolchain'/'bin'/'arm-none-eabi-gcc.exe'
            candidate.parent.mkdir(parents=True); candidate.touch()
            with patch.dict(os.environ,{},clear=True),patch.object(b.shutil,'which',return_value=None):
                self.assertEqual(b.find_gcc(search_roots=[root]),candidate.resolve())
