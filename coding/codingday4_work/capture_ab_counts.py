"""Read the locked diagnostic build's two 16-bit timer counters via HOTPLUG.

The identity file must come from read_current_board.py after programming the
matching diagnostic HEX. A single live sample is not a speed or CPR reading.
No target reset, halt, memory write, or motor command is performed.
"""
from datetime import datetime, timedelta, timezone
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from read_current_board import CLI, ROOT, parse_words

BUILD = ROOT / "firmware_source" / "build" / "encoder_ab_diag"
TIMERS = {"right": 0x40000400, "left": 0x40010400}
PWM = (0x40000834, 0x40000838, 0x40001834, 0x40001838)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--identity", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--build-dir", type=Path, default=BUILD)
    args = parser.parse_args()
    if not args.label.replace("_", "").isalnum():
        parser.error("label must contain only letters, digits and underscores")
    identity = json.loads(args.identity.read_text(encoding="utf-8"))
    image_sha = hashlib.sha256((args.build_dir / "Week4_Integrated.hex").read_bytes()).hexdigest()
    if identity.get("hex_sha256") != image_sha or identity.get("mismatches") != 0:
        raise RuntimeError("Identity must verify the currently built diagnostic HEX")
    if any(identity.get("pwm", [1])):
        raise RuntimeError("The verified identity did not have zero PWM")
    if "calibration_flags" in identity:
        if any(identity["calibration_flags"]):
            raise RuntimeError("The verified identity had motion enabled")
    else:
        diagnostic = identity.get("diagnostic", {})
        if diagnostic.get("state") not in (0, 2) or diagnostic.get("request") != 0:
            raise RuntimeError("The verified diagnostic identity was not idle")

    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
    target = ROOT / "logs" / f"{args.label}_{stamp}"
    command = [str(CLI), "-c", "port=SWD", "sn=0674FF545155657067032031", "mode=HOTPLUG"]
    for addr in TIMERS.values():
        command += ["-r32", hex(addr), "0x30"]
    command += ["-r32", hex(PWM[0]), "8", "-r32", hex(PWM[2]), "8"]
    run = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
    output = run.stdout.decode("utf-8", errors="replace")
    target.with_suffix(".txt").write_text(output, encoding="utf-8")
    if run.returncode or "Error:" in output or "Device name : STM32F446xx" not in output:
        raise RuntimeError(f"Incomplete HOTPLUG read; see {target.with_suffix('.txt')}")
    words = parse_words(output)
    result = {"time_beijing": stamp, "identity": str(args.identity),
              "hex_sha256": image_sha, "pwm": [words[a] for a in PWM],
              "note": "Sequential register reads; signed delta requires a second sample and <32768 counts between reads."}
    for side, base in TIMERS.items():
        cr1, smcr, ccmr1, ccer = (words[base + offset] for offset in (0, 8, 0x18, 0x20))
        arr, counter = words[base + 0x2c], words[base + 0x24] & 0xffff
        valid = ((cr1 & 1) == 1 and (smcr & 7) == 3 and (ccmr1 & 3) == 1
                 and ((ccmr1 >> 8) & 3) == 1 and (ccer & 0x11) == 0x11 and arr == 65535)
        result[side] = {"counter": counter, "cr1": hex(cr1), "smcr": hex(smcr),
                        "ccmr1": hex(ccmr1), "ccer": hex(ccer), "arr": arr,
                        "encoder_mode_verified": valid}
    target.with_suffix(".json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"Saved: {target.with_suffix('.json')}")
    if any(result["pwm"]) or any(not result[side]["encoder_mode_verified"] for side in TIMERS):
        raise RuntimeError("Unexpected PWM or timer mode; stop the observation")


if __name__ == "__main__":
    main()
