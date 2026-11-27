"""Verify the motor-pulse diagnostic firmware and read its state via HOTPLUG.

This tool is read-only and cannot trigger a motor pulse. A separate deliberate
request is needed after the correct test and physical conditions are confirmed.
"""
from datetime import datetime, timedelta, timezone
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import struct
import statistics
import math

from read_current_board import CLI, NM, ROOT, image_bytes, compare_image, parse_words

BUILD = ROOT / 'firmware_source' / 'build' / 'motor_pulse_diag'
MANIFEST = ROOT / 'logs' / 'verified_motor_pulse_diag_20261118.json'
FIELDS = ('request', 'state', 'last_side', 'start_ms', 'stop_ms', 'left_start',
          'left_stop', 'right_start', 'right_stop', 'left_ccr_before_stop',
          'right_ccr_before_stop', 'completed')
PWM = (0x40000834, 0x40000838, 0x40001834, 0x40001838)


def symbol_addresses(build=BUILD, with_speed=False):
    cmd = [str(NM), '-S', '--defined-only', str(build / 'Week4_Integrated.elf')]
    output = subprocess.run(cmd, capture_output=True, text=True, check=True, timeout=15).stdout
    expected = {'w4_motor_pulse_diag': 48, 'W4FaultCode': 4, 'W4FaultDetail': 4}
    if with_speed:
        expected['w4_motor_speed_diag'] = 2820
        expected['w4_wheel_speed_gains'] = 8
    found = {}
    for line in output.splitlines():
        match = re.fullmatch(r'([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+\S\s+(\S+)', line.strip())
        if match and match[3] in expected:
            addr, size, name = int(match[1], 16), int(match[2], 16), match[3]
            if name in found or size != expected[name] or not 0x20000000 <= addr < 0x20020000:
                raise RuntimeError(f'Unexpected symbol {name}')
            found[name] = addr
    if found.keys() != expected.keys():
        raise RuntimeError(f'Missing ELF symbols: {expected.keys() - found.keys()}')
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True)
    parser.add_argument('--verify-flash', action='store_true')
    parser.add_argument('--identity', type=Path)
    parser.add_argument('--build-dir', type=Path, default=BUILD)
    parser.add_argument('--manifest', type=Path, default=MANIFEST)
    parser.add_argument('--speed-trace', action='store_true')
    args = parser.parse_args()
    if not args.label.replace('_', '').isalnum() or (not args.verify_flash and not args.identity):
        parser.error('Use a safe label and either --verify-flash or a prior --identity file')
    manifest = json.loads(args.manifest.read_text(encoding='utf-8'))
    hex_path, elf_path = args.build_dir / 'Week4_Integrated.hex', args.build_dir / 'Week4_Integrated.elf'
    for path, field in ((hex_path, 'hex_sha256'), (elf_path, 'elf_sha256')):
        if hashlib.sha256(path.read_bytes()).hexdigest() != manifest[field]:
            raise RuntimeError(f'{path.name} differs from the verified flash manifest')
    if args.identity:
        identity = json.loads(args.identity.read_text(encoding='utf-8'))
        if identity.get('hex_sha256') != manifest['hex_sha256'] or identity.get('mismatches') != 0:
            raise RuntimeError('Identity does not verify this diagnostic build')
    symbols = symbol_addresses(args.build_dir, args.speed_trace)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    target = ROOT / 'logs' / f'{args.label}_{stamp}'
    command = [str(CLI), '-c', 'port=SWD', 'sn=0674FF545155657067032031', 'mode=HOTPLUG']
    flash_path = target.with_name(target.name + '_flash.bin')
    if args.verify_flash:
        image = image_bytes(hex_path)
        command += ['-u', hex(min(image)), hex(max(image) - min(image) + 1), str(flash_path)]
    command += ['-r32', hex(symbols['w4_motor_pulse_diag']), '0x30']
    for name in ('W4FaultCode', 'W4FaultDetail'):
        command += ['-r32', hex(symbols[name]), '4']
    command += ['-r32', hex(PWM[0]), '8', '-r32', hex(PWM[2]), '8']
    trace_path = target.with_name(target.name + '_speed.bin')
    if args.speed_trace:
        command += ['-u', hex(symbols['w4_motor_speed_diag']), '0xb04', str(trace_path)]
        command += ['-r32', hex(symbols['w4_wheel_speed_gains']), '8']
    run = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=45)
    output = run.stdout.decode('utf-8', errors='replace')
    target.with_suffix('.txt').write_text(output, encoding='utf-8')
    if run.returncode or 'Error:' in output or 'Device name : STM32F446xx' not in output:
        raise RuntimeError(f'HOTPLUG read incomplete; see {target.with_suffix(".txt")}')
    mismatch = compare_image(image, flash_path.read_bytes()) if args.verify_flash else 0
    result = {'time_beijing': stamp, 'hex_sha256': manifest['hex_sha256'],
              'elf_sha256': manifest['elf_sha256'], 'mismatches': mismatch,
              'valid_hex_bytes': len(image) if args.verify_flash else None,
              'identity_source': str(args.identity) if args.identity else None,
              'readings_sequential': True}
    if mismatch:
        target.with_suffix('.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        raise RuntimeError('Flash does not match the ELF; no RAM values were interpreted')
    words = parse_words(output)
    base = symbols['w4_motor_pulse_diag']
    result['diagnostic_address'] = hex(base)
    result['diagnostic'] = {name: words[base + 4 * i] for i, name in enumerate(FIELDS)}
    result['fault'] = {name: words[symbols[name]] for name in ('W4FaultCode', 'W4FaultDetail')}
    result['pwm'] = [words[a] for a in PWM]
    if args.speed_trace:
        data = trace_path.read_bytes()
        version, active, target_rpm, count, fault = struct.unpack_from('<IIfII', data)
        if len(data) != 2820 or version != 1 or active or count > 100:
            raise RuntimeError('Speed trace is unavailable or still active')
        trace = []
        for index in range(count):
            sample = struct.unpack_from('<Iiiffff', data, 20 + index * 28)
            if not all(math.isfinite(value) for value in sample[3:]):
                raise RuntimeError('Nonfinite speed trace')
            trace.append(dict(zip(('elapsed_ms', 'left_delta', 'right_delta', 'left_rpm',
                                   'right_rpm', 'left_pwm', 'right_pwm'), sample)))
        trace_json = target.with_name(target.name + '_speed.json')
        trace_json.write_text(json.dumps({'target_rpm': target_rpm, 'fault': fault,
                                         'samples': trace}, indent=2), encoding='utf-8')
        settled = [sample for sample in trace if sample['elapsed_ms'] >= 3000]
        summary = {'target_rpm': target_rpm, 'samples': count, 'fault': fault,
                   'trace': str(trace_json), 'settled_samples': len(settled)}
        gain_address = symbols['w4_wheel_speed_gains']
        summary['kp'], summary['ki'] = struct.unpack('<ff', struct.pack('<II',
            words[gain_address], words[gain_address + 4]))
        if settled:
            for key in ('left_rpm', 'right_rpm', 'left_pwm', 'right_pwm'):
                summary['mean_' + key] = statistics.mean(sample[key] for sample in settled)
            summary['wheel_mismatch_percent'] = abs(summary['mean_left_rpm'] -
                summary['mean_right_rpm']) / target_rpm * 100
            summary['mean_absolute_mismatch_percent'] = statistics.mean(
                abs(sample['left_rpm'] - sample['right_rpm']) for sample in settled) / target_rpm * 100
            summary['left_rpm_sd'] = statistics.pstdev(sample['left_rpm'] for sample in settled)
            summary['right_rpm_sd'] = statistics.pstdev(sample['right_rpm'] for sample in settled)
            summary['final_left_rpm'] = trace[-1]['left_rpm']
            summary['final_right_rpm'] = trace[-1]['right_rpm']
        if result['diagnostic']['state'] == 2:
            for side in ('left', 'right'):
                diag = result['diagnostic']
                sample_total = sum(sample[f'{side}_delta'] for sample in trace)
                raw_total = diag[f'{side}_stop'] - diag[f'{side}_start']
                counts = sample_total + ((raw_total - sample_total + 32768) % 65536 - 32768)
                summary[f'{side}_counts'] = counts
                cpr = manifest.get(f'cpr_{side}')
                if cpr:
                    summary[f'{side}_turns'] = counts / cpr
        result['speed_summary'] = summary
    target.with_suffix('.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
    print(f'Saved: {target.with_suffix(".json")}')
    if any(result['pwm']) or result['diagnostic']['state'] == 1 or result['fault']['W4FaultCode']:
        raise RuntimeError('Unexpected PWM, running pulse, or fault; stop further actions')
    if args.speed_trace and result['speed_summary']['fault']:
        raise RuntimeError('Speed feedback fault; test stopped early')


if __name__ == '__main__':
    main()
