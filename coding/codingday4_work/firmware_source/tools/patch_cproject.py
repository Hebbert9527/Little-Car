"""Patch the copied CubeIDE .cproject/.project for the Week4 RTOS integration.
Removes member*/firmware source/include entries (not present in this workspace),
puts week4/Inc FIRST (frozen contract), adds FreeRTOS kernel includes/sources and
the W4_ENABLE_RTOS / W4_SINGLE_ENCODER symbols. Run once after copying the base.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
RTOS = 'Middlewares/Third_Party/FreeRTOS/Source'
KERNEL_INC = [f'{RTOS}/include', f'{RTOS}/CMSIS_RTOS_V2', f'{RTOS}/portable/GCC/ARM_CM4F']
EXCLUDE = ('CMSIS_RTOS_V2/cmsis_os.h|portable/MemMang/heap_1.c|'
           'portable/MemMang/heap_2.c|portable/MemMang/heap_3.c|portable/MemMang/heap_5.c')


def patch_cproject(text):
    root = ET.fromstring(text)
    for cfg in root.findall('.//configuration'):
        for option in cfg.findall('.//option'):
            sc = option.get('superClass', '')
            if sc.endswith('c.compiler.option.includepaths'):
                vals = [v for v in option.findall('listOptionValue')
                        if 'member' not in v.get('value', '')]
                for v in option.findall('listOptionValue'):
                    option.remove(v)
                first = ET.Element('listOptionValue', builtIn='false', value='../week4/Inc')
                option.append(first)
                existing = {v.get('value') for v in vals}
                for path in KERNEL_INC:
                    if '../'+path not in existing:
                        vals.append(ET.Element('listOptionValue', builtIn='false', value='../'+path))
                for v in vals:
                    option.append(v)
            if sc.endswith('c.compiler.option.definedsymbols'):
                have = {v.get('value') for v in option.findall('listOptionValue')}
                for sym in ['W4_ENABLE_RTOS', 'W4_SINGLE_ENCODER']:
                    if sym not in have:
                        ET.SubElement(option, 'listOptionValue', builtIn='false', value=sym)
        for entries in cfg.findall('sourceEntries'):
            for entry in list(entries):
                if entry.get('name', '').startswith('member'):
                    entries.remove(entry)
            names = {e.get('name') for e in entries}
            if 'week4/Src' not in names:
                ET.SubElement(entries, 'entry', flags='VALUE_WORKSPACE_PATH|RESOLVED',
                              kind='sourcePath', name='week4/Src')
            if RTOS not in names:
                ET.SubElement(entries, 'entry', flags='VALUE_WORKSPACE_PATH|RESOLVED',
                              kind='sourcePath', name=RTOS, excluding=EXCLUDE)
    ET.indent(root, space='\t')
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def main():
    cp = ROOT/'.cproject'
    cp.write_text(patch_cproject(cp.read_text(encoding='utf-8')), encoding='utf-8')
    for name in ['.project', '.cproject']:
        p = ROOT/name
        p.write_text(p.read_text(encoding='utf-8').replace('Week3_PPT_Modular', 'Week4_Integrated'),
                     encoding='utf-8')
    print('Patched .cproject/.project for Week4 RTOS integration.')


if __name__ == '__main__':
    main()