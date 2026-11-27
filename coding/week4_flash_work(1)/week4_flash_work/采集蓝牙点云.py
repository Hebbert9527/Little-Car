"""Passively record timestamped Week4 BLE telemetry for a short obstacle test."""
import asyncio
from collections import Counter
from datetime import datetime
from pathlib import Path
import time

from bleak import BleakClient, BleakScanner

ADDRESS = "C4:25:01:17:00:02"
CHARACTERISTIC = "0000ffe1-0000-1000-8000-00805f9b34fb"
OUTPUT = Path(__file__).resolve().parent / ("lidar_watch_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".txt")
counts = Counter()
pending = bytearray()

async def main():
    end = time.monotonic() + 95
    with OUTPUT.open("w", encoding="ascii", buffering=1) as log:
        print(f"LOG {OUTPUT}", flush=True)
        while time.monotonic() < end:
            try:
                device = await BleakScanner.find_device_by_address(ADDRESS, timeout=6)
                if not device:
                    print("SCAN: device unavailable, retrying", flush=True)
                    continue
                async with BleakClient(device, timeout=12) as client:
                    char = client.services.get_characteristic(CHARACTERISTIC)
                    if not char:
                        raise RuntimeError("FFE1 characteristic missing")
                    pending.clear()
                    def receive(_, payload):
                        pending.extend(payload)
                        while b"\n" in pending:
                            line, _, tail = pending.partition(b"\n")
                            pending[:] = tail
                            text = line.decode("ascii", errors="replace").strip()
                            if "," in text:
                                counts[text.split(",", 1)[0]] += 1
                                log.write(f"{time.time():.3f} {text}\n")
                    await asyncio.wait_for(client.start_notify(char, receive), timeout=12)
                    print("READY: receiving real telemetry", flush=True)
                    while client.is_connected and time.monotonic() < end:
                        await asyncio.sleep(1)
                    if client.is_connected:
                        await client.stop_notify(char)
            except Exception as exc:
                print(f"BLE retry: {type(exc).__name__}: {exc}", flush=True)
                await asyncio.sleep(2)
    print(f"DONE frames={dict(counts)} log={OUTPUT}", flush=True)

if __name__ == "__main__":
    asyncio.run(main())
