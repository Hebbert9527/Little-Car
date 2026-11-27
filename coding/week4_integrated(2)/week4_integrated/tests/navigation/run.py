"""Build/run production navigation modules with a host GCC; no third-party Python deps."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ["tests/navigation/test_nav.c", "tests/navigation/mock_port.c"] + [
    f"week4/Src/{name}.c" for name in (
        "w4_common", "w4_pose_single", "w4_control", "w4_motor", "w4_path", "w4_bt_rx"
    )
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", default=os.environ.get("CC"))
    args = parser.parse_args()
    gcc = args.gcc or shutil.which("gcc")
    fallback = Path("D:/Program Files (x86)/Dev-Cpp/MinGW32/bin/gcc.exe")
    if not gcc and fallback.exists():
        gcc = str(fallback)
    if not gcc:
        parser.error("No host GCC found; set CC or pass --gcc")
    env = os.environ.copy()
    env["PATH"] = str(Path(gcc).resolve().parent) + os.pathsep + env.get("PATH", "")
    target = ROOT / "tests/navigation" / ("test_nav.exe" if os.name == "nt" else "test_nav")
    flags = ["-std=c11", "-DW4_SINGLE_ENCODER", "-Wall", "-Wextra", "-Werror"]
    machine = subprocess.check_output([gcc, "-dumpmachine"], env=env, text=True)
    if any(arch in machine for arch in ("mingw", "i686", "i386", "x86_64")):
        flags += ["-msse2", "-mfpmath=sse"]
    subprocess.run([gcc, *flags, "-Iweek4/Inc",
                    "-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
                    *SOURCES, "-lm", "-o", str(target)], cwd=ROOT, env=env, check=True)
    subprocess.run([str(target)], cwd=ROOT, env=env, check=True)


if __name__ == "__main__":
    main()
