# Week4 PC 工具

本机活动工程为 `D:\EMBEGGed\week4codingday\coding\coding(1)\coding\codingday4_work\firmware_source`。已创建 `.venv` 并安装 matplotlib、pyserial 和 bleak；启动脚本优先使用该环境，无需手动激活。

在活动工程目录运行：

```powershell
.\tools\setup_pc.ps1 -SkipInstall
.\tools\run.ps1 -Mode Demo
.\tools\run.ps1 -Mode Live -BleAddress C4:25:01:17:00:02
.\tools\run.ps1 -Mode Live -Port COM5
.\tools\run.ps1 -Mode Test
```

重建环境时使用 `tools/setup_pc.ps1 -Python '完整Python路径'`。安装、下载、TEMP/TMP/TMPDIR、pip与matplotlib缓存都设在工程 `.runtime` 内；不要直接用默认系统临时目录运行 pip。`-SkipInstall` 仅准备环境，不联网安装。

`tools/prepare_tk.ps1` 从当前虚拟环境对应的 Python 基座复制匹配 Tcl/Tk 数据到 `.runtime/tcl`，并在当前进程设置 `TCL_LIBRARY`、`TK_LIBRARY`。Live/Demo 和 setup 已自动接入；不改变系统环境变量，也不混用其他 Python 版本的 Tcl 文件。已用当前 Python 3.12.14/Tcl与Tk 8.6.12验证窗口初始化。

协议解析、模拟接收和 PC 测试仅依赖标准库。GUI需要matplotlib、串口需要pyserial、BLE需要bleak。使用原始 `pc/monitor.py` 命令前，先在同一PowerShell执行 `. .\tools\setup_pc.ps1 -SkipInstall`，以继承工程内缓存和Tcl设置。

连接和控制：

```powershell
python -B pc/monitor.py --list-ports
python -B pc/monitor.py --port COM5 --baud 921600 --log logs/live.bin
python -B pc/monitor.py --ble C4:25:01:17:00:02
python -B pc/monitor.py --port COM5 --headless --duration 10 --command STOP
.\tools\run.ps1 -Mode Live -Port COM5
.\tools\run.ps1 -Mode Demo
```

直接使用 `--log` 时，先创建输出目录。`run.ps1` 自动创建日志目录。UART 默认 921600；BLE 默认 UART 特征 `0000ffe1-0000-1000-8000-00805f9b34fb`，可用 `--ble-char` 覆盖；模块如要求有响应写入，增加 `--ble-response`。BLE 命令按至多20字节分段发送。连接或订阅失败的具体异常会保留在状态栏并写入stderr，供排查服务缺失等问题。串口与 BLE 断开后自动重连；断开时丢弃未发送命令，重连后需要重新提交。`--command` 仅在首次连接后发送一次，不在重连后重复。

窗口左侧显示实际位姿轨迹、朝向和目标，右侧显示最近3秒的激光雷达数据（雷达自身坐标系，未伪装成世界地图）。下方显示 IMU、固件 ACK、共享 I2C 和各任务的轮换诊断。位姿和 IMU 显示数据年龄，避免把断流当成静止。输入框默认是 `GOTO,0,1.4`。双轮版中，`GOTO,0,1` 会在起步前对齐 RESET 时保存的朝向，再向前走 1 米。其他坐标指令使用世界坐标，每轴支持 −5 至 +5 米，先走 Y 再走 X。点击 Send 或回车发送，也可直接点击 STOP 或 RESET。示例见 [README_HARDWARE.md](README_HARDWARE.md)。RESET 是停车后的位姿重置，是否接受以 ACK 为准。

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


## 迷宫导航

加载迷宫固件后，停车并完成 IMU 标定，RESET 建立新地图。监控左图的灰色方块是主板上报的墙体。MAZE,0,1 表示 RESET 坐标系中的 (0, 1) 米；MAZE,0.7,1 会以同一坐标系为目标。两个轴均限 ±5 米（包括边界，坐标是 RESET 原点的绝对坐标）。主板返回 MAZE_ACCEPTED 后开始执行，STOP 沿用既有停车入口。GOTO 保留定长和旧坐标测试语义。

当前地图为 107×107、每格 0.1 米，覆盖 ±5 米目标并保留边界避障余量。新 M 帧格式为 `M,epoch,row,dx_mm,dy_mm,cell_mm,center,scan_age_ms,cells`，当前 cell_mm=100、center=53，每行 107 个字符。井号为确认墙，点为已观测自由，问号为未知。scan_age_ms=4294967295 表示本版本还没有有效扫描。前端兼容旧固件 49 字符的 M 帧。

确认墙不再被后续自由射线擦除；断流时继续发送缓存地图，但扫描年龄继续增加，不会伪造扫描新鲜度。每 50 ms 最多发送一行，新确认墙所在行与全图轮询交替发送；无新增墙时一轮约 5.35 秒，持续新增墙时全图兜底约 10.7 秒，链路丢包可能延长。板上规划直接使用本地地图，不等前端传完。

默认两图的 X、Y 坐标均显示 −5～+5 米（窗口宽高各10米）；右上角 `View 5 x 5 m` 可切换为 ±2.5 米局部视野。点击左图只填入 MAZE 坐标，按 Send 才发送。MAZE/GOTO 均接受每轴 [-5,5] 米，实际执行仍要求有效定位和可通行路线。

正常运行及 IMU/蓝牙断流时保留已确认墙；RESET 成功后同时清空旧墙和轨迹，从新扫描重新建图，丢弃排队的旧地图版本。主板重启同样建立新图。RESET 需要停车且 IMU 有效；拒绝 RESET 时保留旧图并显示 ACK 原因。环境或起点改变时请 RESET，因为已确认的误墙也会保留到重置。

FAULT 1 为传感器/定位失效，FAULT 9 为真实扫描过期，FAULT 10 是旧固件的无路故障；持续重规划版遇到无路保持 PLANNING，发出 `A,id,WAIT,原因`，等新扫描并以至少300ms间隔重试，成功后沿原目标继续。首次获得路径回报 MAZE_ACCEPTED，已接受路线重新恢复回报 MAZE_RESUMED。STOP、传感器/扫描过期、真实堵转和原有120秒总任务超时仍有效。永久墙不会因移走障碍自动擦除，场景变化仍需RESET。右侧原始点云仍可出现而左图不建新墙，因为世界地图需要有效位姿。保留墙图没有取消这些运动保护。
