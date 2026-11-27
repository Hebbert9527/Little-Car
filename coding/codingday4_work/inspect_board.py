"""Read-only HOTPLUG snapshots for the verified 2026-11-18 GCC12 Week4 image.

Never resets, halts, writes RAM/Flash or sends motor commands. Static addresses
are only permitted after logs/firmware_identity.json confirms the exact HEX.
RAM snapshots are best-effort observations, not atomic sensor captures.
"""
from pathlib import Path
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
import re
import statistics
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parent
PLUGIN = Path(r"D:\EMBEGGed\st-stm32cubeide_1.17.0_23558_20241125_2245_x86_64.exe\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins")
CLI = PLUGIN / "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.0.202409170845/tools/bin/STM32_Programmer_CLI.exe"
SERIAL = "0674FF545155657067032031"
EXPECTED_HEX = "3feb5d53afb3cda0fdbea396a769f95efccd42c2f55bb00754debae1ef38c747"
COUNTERS = ("timer_triggers", "sem_released", "sem_coalesced", "acquired", "completed", "failed", "max_wait_ms", "max_process_ms", "rejected_intervals", "calibration_motion", "calibration_samples", "init_failures", "max_sample_interval_ms")
REGIONS = ((0x20001AE0, 0x68), (0x200019F0, 8), (0x20000010, 4),
           (0x2000264C, 4), (0x40000834, 8), (0x40001834, 8))


def check_identity():
    report = json.loads((ROOT / "logs/firmware_identity.json").read_text(encoding="utf-8"))
    match = [x for x in report["comparisons"] if x["sha256"] == EXPECTED_HEX and x["mismatch_bytes"] == 0]
    if len(match) != 1:
        raise RuntimeError("First verify the current Flash against the expected HEX.")
    path = Path(match[0]["hex"])
    if hashlib.sha256(path.read_bytes()).hexdigest() != EXPECTED_HEX:
        raise RuntimeError("Reference HEX changed; re-verify identity and symbols.")


def snapshot():
    args = [str(CLI), "-c", "port=SWD", f"sn={SERIAL}", "mode=HOTPLUG"]
    for address, size in REGIONS:
        args += ["-r32", hex(address), hex(size)]
    start = time.monotonic()
    run = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20)
    output = run.stdout.decode("utf-8", errors="replace")
    if run.returncode or "Device name : STM32F446xx" not in output or "Error:" in output:
        raise RuntimeError(output)
    memory = {}
    for line in output.splitlines():
        m = re.match(r"\s*0x([\da-fA-F]{8})\s*:\s*(.*)", line)
        if not m:
            continue
        address = int(m.group(1), 16)
        for token in m.group(2).split():
            if not re.fullmatch(r"[\da-fA-F]{8}", token):
                break
            for n, byte in enumerate(int(token, 16).to_bytes(4, "little")):
                memory[address + n] = byte
            address += 4

    def unpack(fmt, address):
        return struct.unpack(fmt, bytes(memory[address + i] for i in range(struct.calcsize(fmt))))

    raw = unpack(">7h", 0x20001AF4)
    accel = [x / 16384 for x in raw[:3]]
    gyro = [x / 131 for x in raw[4:]]
    published = unpack("<ffIBB", 0x20001B04)
    values = unpack("<13I", 0x20001B14)
    row = dict(host_time=datetime.now(timezone(timedelta(hours=8))).isoformat(),
               read_seconds=time.monotonic() - start,
               tick=unpack("<I", 0x2000264C)[0],
               raw=list(raw), accel_g=accel, gyro_dps=gyro,
               accel_norm_g=math.sqrt(sum(x*x for x in accel)),
               heading_rad=published[0], published_gz_rad_s=published[1],
               sample_ms=published[2], valid=published[3], calibrated=published[4],
               gz_bias=unpack("<f", 0x20001AE8)[0],
               imu_address=memory[0x20001AF0], counters=dict(zip(COUNTERS, values)),
               fault_detail=unpack("<I", 0x200019F0)[0], fault_code=unpack("<I", 0x200019F4)[0],
               calibration_flags=list(unpack("<3B", 0x20000010)),
               pwm=list(unpack("<2I", 0x40000834) + unpack("<2I", 0x40001834)))
    return row, output


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--count", type=int, default=12)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--label", default="stationary")
    args = ap.parse_args()
    if not 1 <= args.count <= 50 or not 0 <= args.interval <= 5:
        ap.error("Count must be 1..50 and interval 0..5 seconds")
    check_identity()
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
    target = ROOT / "logs" / f"{args.label}_{stamp}"
    rows = []
    with target.with_suffix(".jsonl").open("w", encoding="utf-8", buffering=1) as out, target.with_suffix(".txt").open("w", encoding="utf-8", buffering=1) as rawlog:
        for n in range(args.count):
            row, raw = snapshot()
            rows.append(row)
            out.write(json.dumps(row) + "\n")
            rawlog.write(row["host_time"] + "\n" + raw + "\n")
            print(f"{n+1}/{args.count} tick={row['tick']} a={[round(x,4) for x in row['accel_g']]} gyro={[round(x,3) for x in row['gyro_dps']]} calibrated={row['calibrated']} accepted={row['counters']['calibration_samples']} motion={row['counters']['calibration_motion']} fault={row['fault_code']} PWM={row['pwm']}", flush=True)
            if any(row["pwm"]) or any(row["calibration_flags"]):
                raise RuntimeError("Unexpected PWM/calibration state. Stop the diagnostic sequence.")
            if n + 1 < args.count:
                time.sleep(args.interval)
    def stats(key, column):
        vals = [r[key][column] for r in rows]
        return dict(mean=statistics.mean(vals), sd=statistics.stdev(vals) if len(vals)>1 else 0, min=min(vals), max=max(vals))
    summary = dict(samples=len(rows), note="Sparse, non-atomic SWD reads of firmware sample buffer; not a 100 Hz capture", gyro_dps=[stats("gyro_dps",i) for i in range(3)], accel_g=[stats("accel_g",i) for i in range(3)])
    if len(rows)>1:
        dt=((rows[-1]["tick"]-rows[0]["tick"]) & 0xffffffff)/1000
        delta={k:(rows[-1]["counters"][k]-rows[0]["counters"][k]) & 0xffffffff for k in COUNTERS}
        summary.update(elapsed_s=dt, counter_deltas=delta, completed_hz=delta["completed"]/dt if dt else None)
    target.with_name(target.name + "_summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
    print(json.dumps(summary, indent=2),flush=True)
    print("LOG", target.with_suffix(".jsonl"),flush=True)


if __name__ == "__main__":
    main()
