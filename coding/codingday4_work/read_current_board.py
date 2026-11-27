"""Verify a previously flashed build and read a few live values without reset.

This tool does not program, halt, reset, or write target memory. It validates
the saved HEX and ELF against the previous flash manifest, then reads flash in
HOTPLUG mode before interpreting any ELF symbol address. The scalar values are
sequential reads, not an atomic snapshot. Hardware access may need permission.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / "firmware_source" / "build" / "integration"
HEX = BUILD / "Week4_Integrated.hex"
ELF = BUILD / "Week4_Integrated.elf"
MANIFEST = ROOT / "logs" / "last_verified_flash_20261118.json"
PLUGINS = Path(r"C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins")
if not PLUGINS.is_dir():
    PLUGINS = Path(r"D:\EMBEGGed\st-stm32cubeide_1.17.0_23558_20241125_2245_x86_64.exe\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins")
CLI = PLUGINS / "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.0.202409170845/tools/bin/STM32_Programmer_CLI.exe"
NM = PLUGINS / "com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.1.0.202410251130/tools/bin/arm-none-eabi-nm.exe"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def image_bytes(path: Path) -> dict[int, int]:
    image: dict[int, int] = {}
    base = 0
    eof = False
    for line in path.read_text(encoding="ascii").splitlines():
        if not line.startswith(":") or eof:
            raise ValueError("Invalid Intel HEX record order")
        record = bytes.fromhex(line[1:])
        if len(record) < 5 or len(record) != record[0] + 5 or sum(record) % 256:
            raise ValueError("Invalid Intel HEX record length or checksum")
        count, offset, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
        data = record[4:4 + count]
        if kind == 0:
            for i, value in enumerate(data):
                address = base + offset + i
                if address in image and image[address] != value:
                    raise ValueError("Conflicting Intel HEX records")
                image[address] = value
        elif kind == 1:
            if count or offset:
                raise ValueError("Invalid Intel HEX EOF")
            eof = True
        elif kind == 2 and count == 2:
            base = int.from_bytes(data, "big") << 4
        elif kind == 4 and count == 2:
            base = int.from_bytes(data, "big") << 16
        elif kind not in (3, 5):
            raise ValueError(f"Unsupported Intel HEX record type {kind}")
    if not eof or not image or min(image) < 0x08000000 or max(image) >= 0x08080000:
        raise ValueError("Incomplete image or address outside STM32F446 flash")
    return image


def compare_image(image: dict[int, int], readback: bytes) -> int:
    start = min(image)
    if len(readback) != max(image) - start + 1:
        raise ValueError("Incorrect flash readback length")
    return sum(readback[address - start] != value for address, value in image.items())


def get_symbols(nm: Path) -> dict[str, int]:
    needed = {"w4_config": 104, "encoder_edge_count": 4,
              "encoder_capture_count": 4, "encoder_invalid_count": 4,
              "encoder_overcapture_count": 4, "W4FaultCode": 4,
              "W4FaultDetail": 4, "xTickCount": 4}
    result = subprocess.run([str(nm), "-S", "--defined-only", str(ELF)],
                            check=True, capture_output=True, text=True, timeout=15)
    symbols: dict[str, int] = {}
    for line in result.stdout.splitlines():
        match = re.fullmatch(r"([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+\S\s+(\S+)", line.strip())
        if not match or match[3] not in needed:
            continue
        address, size, name = int(match[1], 16), int(match[2], 16), match[3]
        if name in symbols or size != needed[name] or not 0x20000000 <= address < 0x20020000:
            raise RuntimeError(f"Unexpected or duplicate symbol {name}")
        symbols[name] = address
    if symbols.keys() != needed.keys():
        raise RuntimeError(f"Missing ELF symbols: {needed.keys() - symbols.keys()}")
    return symbols


def parse_words(output: str) -> dict[int, int]:
    memory: dict[int, int] = {}
    for line in output.splitlines():
        match = re.match(r"\s*0x([0-9a-fA-F]{8})\s*:\s*(.*)", line)
        if not match:
            continue
        address = int(match[1], 16)
        for word in match[2].split():
            if not re.fullmatch(r"[0-9a-fA-F]{8}", word):
                break
            memory[address] = int(word, 16)
            address += 4
    return memory


def main() -> None:
    global HEX, ELF, MANIFEST
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", type=Path, default=CLI)
    parser.add_argument("--nm", type=Path, default=NM)
    parser.add_argument("--build-dir", type=Path, default=BUILD)
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    args = parser.parse_args()
    HEX = args.build_dir / "Week4_Integrated.hex"
    ELF = args.build_dir / "Week4_Integrated.elf"
    MANIFEST = args.manifest
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    for path, key in ((HEX, "hex_sha256"), (ELF, "elf_sha256")):
        if digest(path) != manifest[key].lower():
            raise RuntimeError(f"{path.name} does not match the verified flash manifest")
    image = image_bytes(HEX)
    symbols = get_symbols(args.nm)
    start = min(image)
    length = max(image) - start + 1
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
    target = ROOT / "logs" / f"board_identity_{stamp}"
    flash_path = target.with_name(target.name + "_flash.bin")
    command = [str(args.cli), "-c", "port=SWD", f"sn={manifest['serial']}", "mode=HOTPLUG",
               "-u", hex(start), hex(length), str(flash_path)]
    for address in symbols.values():
        command.extend(("-r32", hex(address), "4"))
    command.extend(("-r32", "0x40000834", "8", "-r32", "0x40001834", "8"))
    run = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=45)
    output = run.stdout.decode("utf-8", errors="replace")
    target.with_suffix(".txt").write_text(output, encoding="utf-8")
    if run.returncode or "Error:" in output or "Device name : STM32F446xx" not in output:
        raise RuntimeError(f"Programmer could not read the target; see {target.with_suffix('.txt')}")
    flash = flash_path.read_bytes()
    mismatches = compare_image(image, flash)
    identity = {"time_beijing": stamp, "hex_sha256": digest(HEX), "elf_sha256": digest(ELF),
                "valid_hex_bytes": len(image), "mismatches": mismatches,
                "flash_readback_sha256": hashlib.sha256(flash).hexdigest(),
                "sequential_live_reads": True}
    if mismatches:
        target.with_suffix(".json").write_text(json.dumps(identity, indent=2), encoding="utf-8")
        raise RuntimeError(f"Flash differs in {mismatches} bytes; ELF addresses were not interpreted")
    words = parse_words(output)
    values = {name: words[address] for name, address in symbols.items()}
    flags = values.pop("w4_config")
    identity["calibration_flags"] = [flags & 255, (flags >> 8) & 255, (flags >> 16) & 255]
    identity["pwm"] = [words[a] for a in (0x40000834, 0x40000838, 0x40001834, 0x40001838)]
    identity.update(values)
    target.with_suffix(".json").write_text(json.dumps(identity, indent=2), encoding="utf-8")
    print(json.dumps(identity, indent=2))
    print(f"Saved: {target.with_suffix('.json')}")
    if any(identity["calibration_flags"]) or any(identity["pwm"]):
        raise RuntimeError("Motion lock or PWM differs from the expected safe state")


if __name__ == "__main__":
    main()
