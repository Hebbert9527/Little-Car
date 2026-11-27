"""Read-only LiDAR counter comparison for the exact Week4 build flashed here.

The addresses are specific to source/build/integration/Week4_Integrated.elf.
Verify that this image is still running before interpreting the output.
"""
from pathlib import Path
import re
import subprocess
import time

CLI = Path(r"C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.0.202409170845\tools\bin\STM32_Programmer_CLI.exe")
SERIAL = "0674FF545155657067032031"
REGIONS = ((0x20000768, 0x64), (0x20001678, 0x18), (0x200000A8, 0x30), (0x2000EBD0, 4))
NAMES = ("state", "ht", "tc", "last_interval_ms", "min_interval_ms", "delivered", "bytes",
         "max_batch_ms", "max_copy_ms", "max_queue_age_ms", "overruns", "bad_descriptors",
         "descriptor_timeouts", "stream_timeouts", "hal_busy", "hal_failures", "uart_errors",
         "last_uart_error", "rx_stops", "abort_timeouts", "retries", "stream_breaks",
         "dropped_blocks", "sequence_gaps", "queue_errors")

def read_snapshot():
    args = [str(CLI), "-c", "port=SWD", f"sn={SERIAL}", "mode=HOTPLUG"]
    for address, size in REGIONS:
        args.extend(["-r32", hex(address), hex(size)])
    run = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, encoding="utf-8", errors="replace", timeout=25)
    if run.returncode or "Device name : STM32F446xx" not in run.stdout or "Error:" in run.stdout:
        raise RuntimeError(run.stdout)
    words = {}
    for line in run.stdout.splitlines():
        match = re.match(r"\s*0x([0-9a-fA-F]{8})\s*:\s*(.*)", line)
        if not match:
            continue
        address = int(match.group(1), 16)
        for value in match.group(2).split():
            if not re.fullmatch(r"[0-9a-fA-F]{8}", value):
                break
            words[address] = int(value, 16)
            address += 4
    def values(address, size):
        return [words[address + n] for n in range(0, size, 4)]
    lidar, parser, queue, tick = [values(*region) for region in REGIONS]
    return dict(tick=tick[0], lidar=dict(zip(NAMES, lidar)), parser=parser, queue=queue)

def delta(after, before):
    return (after - before) & 0xFFFFFFFF

def main():
    a = read_snapshot()
    time.sleep(12)
    b = read_snapshot()
    seconds = delta(b["tick"], a["tick"]) / 1000
    print(f"观察时间: {seconds:.3f} 秒; 状态: {a['lidar']['state']} → {b['lidar']['state']} (6=STREAM)")
    for name in ("ht", "tc", "delivered", "bytes", "overruns", "bad_descriptors",
                 "stream_timeouts", "retries", "stream_breaks", "dropped_blocks", "queue_errors"):
        print(f"{name}: +{delta(b['lidar'][name], a['lidar'][name])} (累计 {b['lidar'][name]})")
    print(f"有效点: +{delta(b['parser'][0], a['parser'][0])}; 错误窗口: +{delta(b['parser'][1], a['parser'][1])}; 无效角度窗口: +{delta(b['parser'][2], a['parser'][2])}")
    print(f"Q_Lidar Put: +{delta(b['queue'][0], a['queue'][0])}; Get: +{delta(b['queue'][4], a['queue'][4])}; 队列满: +{delta(b['queue'][1], a['queue'][1])}")
    print("注：统计为实时读取的单字段快照；点云帧连续不代表 DMA 半区没有丢失。")

if __name__ == "__main__":
    main()
