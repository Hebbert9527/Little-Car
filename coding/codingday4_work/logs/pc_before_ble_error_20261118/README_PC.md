# Week4 PC 工具

本机已创建项目隔离环境 `.venv`，安装串口和BLE依赖并复用已有matplotlib；无需手动激活。启动脚本优先使用该环境，直接运行：

```powershell
.\tools\run.ps1 -Mode Demo
.\tools\run.ps1 -Mode Live -Port COM5
.\tools\run.ps1 -Mode Live -BleAddress C4:25:01:17:00:02
.\tools\run.ps1 -Mode Test
```

在其他电脑重建同样环境：`python -m venv --system-site-packages .venv`，然后 `.\.venv\Scripts\python.exe -m pip install -r pc/requirements.txt`。这些安装仅写入工程内虚拟环境。

在 `week4_integrated` 目录运行；Python 3.10 以上。协议解析、模拟接收和 PC 测试仅依赖标准库；图形界面需要 matplotlib，串口需要 pyserial，BLE 需要 bleak：

```powershell
python -m pip install -r pc/requirements.txt
python -B tools/test_python.py
python -B pc/monitor.py --demo
python -B pc/monitor.py --demo --headless --duration 2
```

连接和控制：

```powershell
python -B pc/monitor.py --list-ports
python -B pc/monitor.py --port COM5 --baud 921600 --log logs/live.bin
python -B pc/monitor.py --ble C4:25:01:17:00:02
python -B pc/monitor.py --port COM5 --headless --duration 10 --command STOP
.\tools\run.ps1 -Mode Live -Port COM5
.\tools\run.ps1 -Mode Demo
```

直接使用 `--log` 时，先创建输出目录。`run.ps1` 自动创建日志目录。UART 默认 921600；BLE 默认 UART 特征 `0000ffe1-0000-1000-8000-00805f9b34fb`，可用 `--ble-char` 覆盖；模块如要求有响应写入，增加 `--ble-response`。BLE 命令按至多20字节分段发送。串口与 BLE 断开后自动重连；断开时丢弃未发送命令，重连后需要重新提交。`--command` 仅在首次连接后发送一次，不在重连后重复。

窗口左侧显示实际位姿轨迹、朝向和目标，右侧显示最近3秒的激光雷达数据（雷达自身坐标系，未伪装成世界地图）。下方显示 IMU、固件 ACK、共享 I2C 和各任务的轮换诊断。位姿和 IMU 显示数据年龄，避免把断流当成静止。输入框默认使用必测目标 `GOTO,0,1.4`，坐标单位为米。点击 Send 或回车发送。STOP 和 RESET 也可直接点击。固件只接受预设路线目标，示例见 [README_HARDWARE.md](README_HARDWARE.md)。RESET 是停车后的位姿重置，是否接受以 ACK 为准。

各帧使用 ASCII、逗号分隔、LF结束。N、P、I 各有独立32位序列号；分包、合包和超长坏帧由有界流解析器处理。日志保留收到的原始字节。

| 帧 | 字段（不含首字母） |
| --- | --- |
| N | seq,ms,cmd_id,state,x_mm,y_mm,theta_mrad,target_x_mm,target_y_mm,waypoint,fault |
| A | cmd_id,status,reason |
| P | seq,ms,angle_cdeg,distance_mm,quality,new_scan |
| I | seq,ms,heading_cdeg,gz_cdps |
| B | ms,imu_acquisitions,oled_acquisitions,imu_max_wait_ticks,oled_max_hold_ticks,completed,timeouts,recoveries |
| S | ms,imu_timer_triggers,imu_completed,imu_failed,sem_coalesced,imu_calibrated,oled_status |
| T | ms,task_id,name,priority,stack_bytes,wakes,max_period_us,max_elapsed_us,max_jitter_us,deadline_misses,min_free_stack_bytes |

D、Q 和诊断帧的字段保留为原始文本，以容纳后续固件扩展；T按任务ID保存最新记录。B/S的计数含义见上表，T标题直接显示单位。DEMO界面和数据明确标注为模拟。

固件构建与测试：

```powershell
python -B tools/build.py --gcc 'D:/toolchain/bin/arm-none-eabi-gcc.exe'
# 额外板级宏按实际硬件设置，例如：
python -B tools/build.py --define W4_OLED_CONTROLLER=1 --define W4_OLED_ADDR7=0x3c --define W4_OLED_WIDTH=128 --define W4_OLED_HEIGHT=64
python -B tools/test_arm.py --cc 'D:/toolchain/bin/gcc.exe'
```

ARM编译器也可通过 `ARM_GCC` 或 PATH指定；自动搜索工程/父目录 `.tools`、STM32CubeIDE及常见独立工具链目录。输出为 `build/integration/Week4_Integrated.elf/.hex/.bin`，命令和诊断在 `build/integration/build.log`。

`tools/test_arm.py` 为历史兼容名称，现在明确执行 `tests/navigation/run.py` 和 `tests/sensors/run.py` 的主机 GCC 回归；不宣称完成 ARM 仿真或硬件时序验收。`CC` 可指定主机 GCC。这些测试和PC测试都不要求unicorn/pyelftools。PC测试检查当前集成协议、坐标边界、断流命令处理和独立序列号；硬件BLE吞吐、真实GUI交互、物理I2C时序仍需实机验证。

PC协议P/I数据类、序列跟踪和流解码基础来自仓库内 `codingday3/new/member1/pc/protocol.py`，集成版新增N/ACK/诊断和命令处理。集成版运行时不导入旧member目录。

补充 Q 队列诊断：`Q,ms,lidar_depth,lidar_capacity,cmd_depth,cmd_capacity,path_depth,path_capacity,cmd_full,path_backpressure`，监控器以通用诊断帧展示。接线及实车验收见 [README_HARDWARE.md](README_HARDWARE.md)。
