"""Read both locked diagnostic encoders and estimate signed wheel RPM.

Uses successive, non-atomic HOTPLUG samples; signs are timer polarity, not yet
validated as vehicle-forward. This is a diagnostic, not a motor controller.
"""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import time

from read_current_board import CLI, ROOT, parse_words

BUILD = ROOT / 'firmware_source' / 'build' / 'encoder_ab_diag'
TIMERS = {'right': 0x40000400, 'left': 0x40010400}
CPR = {'right': 391.15 * 4, 'left': (15517 + 15654) / 20}
PWM = (0x40000834, 0x40000838, 0x40001834, 0x40001838)


def delta16(now, previous):
    return (now - previous + 32768) % 65536 - 32768


def sample():
    command = [str(CLI), '-c', 'port=SWD', 'sn=0674FF545155657067032031', 'mode=HOTPLUG']
    for addr in TIMERS.values():
        command += ['-r32', hex(addr), '0x30']
    command += ['-r32', hex(PWM[0]), '8', '-r32', hex(PWM[2]), '8']
    started = time.monotonic()
    run = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20)
    completed = time.monotonic()
    output = run.stdout.decode('utf-8', errors='replace')
    if run.returncode or 'Error:' in output or 'Device name : STM32F446xx' not in output:
        raise RuntimeError('Incomplete HOTPLUG read: ' + output[-1000:])
    words = parse_words(output)
    if any(words[a] for a in PWM):
        raise RuntimeError('Unexpected nonzero PWM; stop monitoring')
    counts = {}
    for side, base in TIMERS.items():
        cr1, smcr, ccmr, ccer, arr = (words[base + x] for x in (0, 8, 0x18, 0x20, 0x2c))
        if not (cr1 & 1 and smcr & 7 == 3 and ccmr & 3 == 1 and
                (ccmr >> 8) & 3 == 1 and ccer & 0x11 == 0x11 and arr == 65535):
            raise RuntimeError(f'{side} timer is not in the expected encoder mode')
        counts[side] = words[base + 0x24] & 0xffff
    return {'monotonic_mid_s': (started + completed) / 2, 'read_duration_s': completed - started,
            'counts': counts, 'pwm': [words[a] for a in PWM]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--identity', type=Path, required=True)
    parser.add_argument('--stop-file', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=90)
    args = parser.parse_args()
    if args.stop_file.exists() or not 1 <= args.duration <= 180:
        parser.error('Use a new stop file and a duration from 1 to 180 seconds')
    identity = json.loads(args.identity.read_text(encoding='utf-8'))
    image_sha = hashlib.sha256((BUILD / 'Week4_Integrated.hex').read_bytes()).hexdigest()
    if identity.get('hex_sha256') != image_sha or identity.get('mismatches') != 0:
        raise RuntimeError('The verified board identity does not match the diagnostic build')
    if any(identity.get('calibration_flags', [1])) or any(identity.get('pwm', [1])):
        raise RuntimeError('The verified board identity is not in the locked state')
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    target = ROOT / 'logs' / f'ab_speed_{stamp}.json'
    samples = []
    error = None
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration and not args.stop_file.exists():
            current = sample()
            current['elapsed_s'] = current['monotonic_mid_s'] - started
            if samples:
                previous = samples[-1]
                dt = current['monotonic_mid_s'] - previous['monotonic_mid_s']
                current['interval_s'] = dt
                current['delta'] = {s: delta16(current['counts'][s], previous['counts'][s]) for s in TIMERS}
                current['rpm'] = {s: current['delta'][s] / CPR[s] * 60 / dt for s in TIMERS}
                print('RPM left={:+.1f} right={:+.1f} dt={:.2f}s'.format(
                    current['rpm']['left'], current['rpm']['right'], dt), flush=True)
            else:
                print('SAMPLING READY', flush=True)
            samples.append(current)
            time.sleep(0.4)
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        data = {'time_beijing': stamp, 'identity': str(args.identity), 'hex_sha256': image_sha,
                'cpr_x4': CPR, 'samples': samples, 'error': error,
                'caution': 'Signs are raw timer polarity. Windowed RPM is approximate; no motor power was commanded.'}
        target.write_text(json.dumps(data, indent=2), encoding='utf-8')
        print(f'LOG {target}', flush=True)
    if error:
        raise RuntimeError(error)
    if len(samples) < 2:
        raise RuntimeError('Not enough samples to calculate speed')


if __name__ == '__main__':
    main()
