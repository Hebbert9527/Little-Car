"""Build the integrated Week4 RTOS firmware for STM32F446RE.

Sources: Core/Src (merged main.c/it.c + week3_board) + week4/Src (member2-6
business code + member1 glue + single-encoder pose) + selected FreeRTOS kernel
+ HAL driver. Old member*/firmware bare-metal sources are NOT linked.

week4/Inc is FIRST on the include path so the frozen w4_contract.h overrides any
member copy. Wiring/pinmux is unchanged from the Week3 base project.

Usage:
  python tools/build.py [--gcc PATH_TO_arm-none-eabi-gcc.exe]
Set ARM_GCC env var or pass --gcc if the toolchain is not auto-detected.
"""
from pathlib import Path
import argparse
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
RTOS = 'Middlewares/Third_Party/FreeRTOS/Source'
INCLUDES = [
    'week4/Inc',                                   # frozen contract FIRST
    'Core/Inc',
    f'{RTOS}/include',
    f'{RTOS}/CMSIS_RTOS_V2',
    f'{RTOS}/portable/GCC/ARM_CM4F',
    'Drivers/STM32F4xx_HAL_Driver/Inc',
    'Drivers/STM32F4xx_HAL_Driver/Inc/Legacy',
    'Drivers/CMSIS/Device/ST/STM32F4xx/Include',
    'Drivers/CMSIS/Include',
]
KERNEL = [
    'tasks.c', 'queue.c', 'list.c', 'timers.c', 'event_groups.c', 'stream_buffer.c',
    'CMSIS_RTOS_V2/cmsis_os2.c', 'portable/GCC/ARM_CM4F/port.c', 'portable/MemMang/heap_4.c',
]
DEFINES = ['-DUSE_HAL_DRIVER', '-DSTM32F446xx', '-DW4_ENABLE_RTOS', '-DW4_SINGLE_ENCODER']


def find_gcc(explicit=None, search_roots=None):
    found = explicit or os.environ.get('ARM_GCC') or shutil.which('arm-none-eabi-gcc')
    if found:
        result = Path(found).expanduser().resolve()
        if not result.is_file():
            raise SystemExit(f'ARM GCC does not exist: {result}')
        return result
    if search_roots is None:
        search_roots = [ROOT/'.tools', ROOT.parent/'.tools',
                        *[Path(p) for p in ('C:/ST','D:/ST','D:/EMBEGGed',
                          'C:/Program Files/Arm GNU Toolchain arm-none-eabi',
                          'D:/Arm','D:/ARM_Lab','D:/Tools','D:/Develop','D:/Keil_v5')]]
    for base in search_roots:
        if base.is_dir():
            matches = list(base.rglob('arm-none-eabi-gcc.exe'))
            matches += list(base.rglob('arm-none-eabi-gcc'))
            if matches:
                return sorted(matches)[-1].resolve()
    raise SystemExit('ARM GCC not found. Set ARM_GCC or pass --gcc PATH_TO_arm-none-eabi-gcc.exe.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gcc')
    parser.add_argument('--define', action='append', default=[], metavar='NAME[=VALUE]',
                        help='additional board configuration macro; may repeat')
    args = parser.parse_args()
    gcc = find_gcc(args.gcc)
    out = ROOT/'build'/'integration'
    out.mkdir(parents=True, exist_ok=True)
    common = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
              '-g3', '-O2', '-ffunction-sections', '-fdata-sections',
              *DEFINES, *['-D'+d for d in args.define], '--specs=nano.specs']
    includes = [f'-I{x}' for x in INCLUDES]
    sources = sorted(ROOT.glob('Core/Src/*.c')) + sorted(ROOT.glob('week4/Src/*.c'))
    sources += [ROOT/RTOS/name for name in KERNEL]
    sources += sorted(ROOT.glob('Drivers/STM32F4xx_HAL_Driver/Src/*.c'))
    sources += sorted(ROOT.glob('Core/Startup/*.s'))
    # Sanity: exactly one heap and one port implementation.
    assert sum(p.name == 'heap_4.c' for p in sources) == 1
    assert sum(p.name == 'port.c' for p in sources) == 1
    objects = []
    with (out/'build.log').open('w', encoding='utf-8') as log:
        def run(cmd):
            log.write(subprocess.list2cmdline([str(x) for x in cmd])+'\n')
            result = subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            output = result.stdout.decode('utf-8', errors='replace')
            log.write(output); log.flush()
            if output:
                print(output, end='')
            if result.returncode:
                raise SystemExit(result.returncode)
        for src in sources:
            rel = src.relative_to(ROOT)
            obj = Path('build/integration/obj')/rel.with_suffix('.o')
            (ROOT/obj).parent.mkdir(parents=True, exist_ok=True)
            objects.append(obj.as_posix())
            flags = ['-std=gnu11', '-Wall', '-Wextra'] if src.suffix == '.c' else ['-x', 'assembler-with-cpp']
            run([str(gcc), *common, *flags, *includes, '-c', rel.as_posix(), '-o', obj.as_posix()])
        (out/'objects.list').write_text('\n'.join(objects)+'\n')
        run([str(gcc), *common, '-o', 'build/integration/Week4_Integrated.elf', '@build/integration/objects.list',
             '-TSTM32F446RETX_FLASH.ld', '--specs=nosys.specs', '-Wl,-Map=build/integration/Week4_Integrated.map',
             '-Wl,--gc-sections', '-static', '-u', '_printf_float', '-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group'])
        suffix = gcc.suffix
        run([str(gcc.with_name('arm-none-eabi-size'+suffix)), 'build/integration/Week4_Integrated.elf'])
        for fmt, ext in [('ihex', 'hex'), ('binary', 'bin')]:
            run([str(gcc.with_name('arm-none-eabi-objcopy'+suffix)), '-O', fmt,
                 'build/integration/Week4_Integrated.elf', f'build/integration/Week4_Integrated.{ext}'])
    print(f'Built {len(sources)} source files. Output: build/integration/Week4_Integrated.elf')


if __name__ == '__main__':
    main()
