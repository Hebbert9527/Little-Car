# Coding Day 4 当前进度与聊天交接

最后保存于 2026-11-27。已完成 NoeticMaze 规划/平滑/连续跟踪移植、15 项 Cortex-M4 回归、21 项 PC 回归及完整构建，最新见第 63 节。移植版源码和固件位于活动工程，已烧录并独立读回校验；最新板上状态见第 64 节。

**当前聊天入口：`D:\EMBEGGed\week4codingday\coding\coding(1)\coding\README.md`。** 本次操作使用该目录的活动工程及日志。早期记录中的其他绝对路径、串口和硬件状态仅供历史参考，当前接续见第 63 节，第 44 至 61 节保留历史实车结果。第22至43节保留整车联调记录。

## 1. 最关键的当前状态

**当前运行 MAZE 绕桩转向修复版。集成双轮速度、雷达、IMU、OLED、蓝牙及导航。**

| 项目 | 当前事实 |
|---|---|
| 主板正在运行 | `W4_MAZE`、`W4_DUAL_ENCODER`、`W4_CALIBRATED`、`W4_REAR_FORWARD`、`W4_IMU_DIAGNOSTICS`、`W4_WHEEL_DIAGNOSTICS`。原车尾为新前方，逻辑左右交换 |
| 已烧录 HEX | `codingday4_work\firmware_source\build\maze_continuous\Week4_Integrated.hex` |
| 已烧录 HEX SHA256 | `947a389da866831f5a3c15774a5be8e4e3a4e4a1abba90b5f0ea8b4841d1ba48` |
| 对应 ELF SHA256 | `73b1f9909053fb5720b362ac7aa4a005c5938cb7c642b43f813c31db96b7c993` |
| 验证 | 97,708 个有效 HEX 字节回读差异 0，14 项 Cortex-M4 回归通过。启动 IDLE、故障 0、四路 PWM 0，IMU 有效且已标定。新版带动力绕桩待复测 |
| 计数标定 | 物理原左 TIM8=1559.45、原右 TIM3=1561.60 计数/圈。车尾前置下逻辑左=原右、逻辑右=原左，两个计数符号均为 -1 |
| 运动条件 | 烧录时动力断开。用户在地面试车并自行发指令，助手只读观察。先验证直线，再进行地面转弯 |

**版本注意：** 当前板上为 `build/maze_continuous`。用匹配当前 HEX 的 ELF 解释 RAM。原车头方向版本保留在 `build/integration_dual_030`。

## 2. 用户目标、偏好与授权

- 最终完成 Coding Day 4：单次蓝牙 GOTO 坐标命令后，小车自主规划、跟踪、到达并停车，持续运行 LiDAR、IMU、OLED、蓝牙遥测和 PC 可视化。
- 用户已要求“接上主板开始工作”，并明确要求“更新完的代码都进行烧录，让我能通过小车看到成果”。后续必要、经验证的固件烧录已获任务授权，不要再重复询问同一件事；工具需要沙箱外权限时按实际权限流程调用。
- 用户要求**所有文件更新都放在 `D:\EMBEGGed\week4codingday\coding`**，并持续维护这个 README，方便更换聊天。每次有新测量、修改、构建或烧录后及时更新。
- 本次接续使用用户指定的 README 所在目录。烧录前用户确认 USB 保持连接、独立电机动力已断开。
- 保留尚未完成的运动标定锁。不要仅为了让车跑起来定义 `W4_CALIBRATED`、盲目改编码器参数或直接放宽门槛。
- 用户此前确认电机动力已断开；新聊天若已过较久或接线变化，在运动试验前重新确认实际状态。实际运动需要明确具体试验、场地和操作条件。

## 3. 文件结构与当前应编辑的工程

所有相对路径以本 README 所在目录为基准。

- **活动工程：`codingday4_work\firmware_source`。后续继续编辑和构建这里。**
- 主交接文件：`README.md`（本文件）。
- 本轮日志：`codingday4_work\logs`。
- 本轮 C 测试入口：`codingday4_work\run_cortex_tests.py`。
- 新版诊断捕获工具：`codingday4_work\capture_window.py`。
- 原始稀疏读取工具：`codingday4_work\inspect_board.py`，只适用原固件，暂勿运行。
- 原整合工程：`week4_integrated(2)\week4_integrated`，作为参考，**没有回写本轮改动**。
- 原烧录工程：`week4_flash_work(1)\week4_flash_work\source`，作为原固件和恢复参考，**没有回写本轮改动**。
- 原历史实物记录：`week4_flash_work(1)\week4_flash_work\实车检查记录.md`、`烧录记录.md`。
- Week3参考：`D:\EMBEGGed\week3codingday\new\member4\firmware\imu.c`（用户确认旧版能够显示航向）。
- PPT 在 coding 的上一级：`D:\EMBEGGed\week4codingday\Block4.5-CodingDay-Part1.pptx` 和 `Block4.5-CodingDay-Part2.pptx`。

本轮最终改动只在独立活动工程中，原来两套工程没有被覆盖。不要误打开旧工程后以为功能修复已丢失，也不要用旧工程的预编译 HEX 覆盖新结果。

## 4. 已完成的定位与实测

### 4.1 原固件身份及备份

- ST-LINK SN：`0674FF545155657067032031`；FW `V2J47M34`；Windows 串口 `COM11`。
- 热连接识别 STM32F446xx，Device ID `0x421`，512 KiB Flash，实测电压3.22 V；CPU未停核，初始四路PWM均0。
- 原始完整片上 Flash 备份：`codingday4_work\logs\flash_initial_20261118.bin`（524288 B）。SHA256：`9486f2b8aa02cc99e128c871eab5231b6a93b70e1772acefe5bb34e0173e0074`。
- 原片上固件与原烧录工程 HEX 的83504个有效字节全部一致，其 HEX SHA256为 `3feb5d53afb3cda0fdbea396a769f95efccd42c2f55bb00754debae1ef38c747`。
- 原整合工程内 build/firmware 的 HEX 来自另一工具链，80412个有效字节不同，不能混用 ELF 地址。比较结果：`logs\firmware_identity.json`。
- 若需要回退，原烧录工程的上述 HEX 仍在，原完整 Flash 备份也保留。

### 4.2 Week3 与 Week4 的关键差异

- 初始化相同：PWR_MGMT_1=0，等待100 ms，GYRO_CONFIG=0、ACCEL_CONFIG=0、CONFIG=3、SMPLRT_DIV=9。
- 数据解码相同：0x3B开始14字节，大端有符号；gyro除131，accel除16384。
- Week3直接平均最初200个 **Z轴** 样本作为零偏，然后对Z轴补偿、积分；没有静止质量检测，X/Y偏置高仍能输出航向。
- Week4额外要求三个原始 gyro 的绝对值都 <=5°/s。当前模块X/Y静止偏置超过阈值，导致一直无法校准，valid=0，所以不发有效I航向帧。
- 这解释“旧版能显示、新版无航向”，但不能由此宣称整颗IMU六轴性能正常。X/Y高偏置仍需保留告警；平面导航所用Z轴必须通过实际漂移和角度验证。

### 4.3 原固件的本轮静止读取

- 日志：`stationary_initial_20261118_195729.jsonl`、`.txt`、`_summary.json`。
- 12个稀疏SWD快照跨7.76秒，timer/acquired/completed各增加776，失败0、信号量合并0，静止拒绝计数增加776，接受样本0。
- 陀螺均值约 `[5.51,14.45,-0.913] °/s`；加速度约 `[0.0208,0.0174,1.0284] g`；fault0、标定标志0、四路PWM0。
- 稀疏读取DMA接收缓冲并非原子快照，不能把其中单个异常样本直接作为物理噪声统计。因此增加了任务内统计的诊断固件。

### 4.4 诊断固件的真实结果

- 已烧录两版诊断，均由 Programmer 报告 `Download verified successfully` 并复位；日志：`flash_diagnostic_20261118.txt`、`flash_diagnostic_windows_20261118.txt`。
- WHO_AM_I=0x70；初始化前后回读一致：0x13..0x18用户gyro偏置均0，SMPLRT_DIV=9、CONFIG=3、GYRO_CONFIG=0、ACCEL_CONFIG=0、PWR_MGMT_1/2均0。全部6次回读成功，mask=63。未发现量程或初始化寄存器配置差异。
- 第一版只冻结开机500样本，窗口含明显移动，不能当静止数据：`imu_diagnostic_window_20261118.bin`、`imu_diagnostic_state_20261118.txt`。
- 第二版每500个成功解码样本发布一个冻结窗口，使用奇偶发布序列检查读取一致性。三个实测窗口：
  - `stationary_verified_20261118_201348.json`：gyro均值 `[5.685,14.623,-0.9433] °/s`，标准差 `[0.0550,0.0732,0.0459] °/s`。
  - `...201355.json`：Z均值-0.9371°/s，标准差0.0742°/s。
  - `...201401.json`：含轻微扰动，Z均值-0.8806°/s，标准差0.2123°/s。
- 各窗口均温约28.2°C；I2C采样失败0、fault0、标定运动锁0、四路PWM0。第二版诊断仍保持原校准政策，所以calibrated/valid仍为0。
- 每个窗口均保留 `.bin`（冻结结构）、`.txt`（原始CLI输出）和 `.json`（解码结果）。

官方资料（已查）：<https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6500-datasheet2.pdf>。FS_SEL=0时131 LSB/(°/s)，25°C初始ZRO规格±5°/s；温度公式temp_raw/333.87+21。旧官网寄存器PDF链接已失效，不要反复使用同一失效链接。

## 5. 最新功能修复的具体内容（已编译、已烧录；硬件进展见第10节）

### 平面Z轴校准

新增 `week4\Src\w4_yaw_calibration.c`、`week4\Inc\w4_yaw_calibration.h`，接入 `w4_imu_rtos.c`：

- 200个连续有效样本，间隙不能超过30 ms；I2C失败清除未完成窗口。
- 要求水平姿态、加速度模长接近1g、|gz|<=5°/s、没有轴饱和和非有限数。
- 限制各轴窗口极差，以及三轴gyro/accel的方差，拒绝运动和高噪声。
- 只求Z轴零偏；稳定的X/Y绝对偏置不再永久阻止平面Z轴校准，而以 `xy_bias_warning` 持续报告。
- 该校准不是六轴标定或整颗器件健康认证。尚需实际测漂移与手动90°旋转响应。
- `W4ImuSnapshot.calibrated` 现在明确表示平面Z航向已校准；结构增加一个告警字节，ARM总大小仍16 B。冻结导航接口不变。
- 传感器恢复后保持原航向连续性，重新校准零偏；既有数据过期、导航故障及停车逻辑保留。

### OLED 可见结果

改动 `w4_oled.h`、`w4_oled.c`、`w4_integration.c`：

- 导航位姿过期时不再把固定初始朝向显示成实时航向。
- 这时如果IMU有效，显示相对 `YAW ±n DEG`；未就绪时显示 `IMU CAL n OF 200`。
- 没有目标且运动参数未标定时，显示 `MOTION LOCKED`。
- X/Y偏置过大以 `XY BIAS` 持续提示；导航故障码仍优先显示。
- 本聊天用户已确认OLED能看到 `YAW`、`MOTION LOCKED` 和 `XY BIAS`；旋转时航向是否变化仍待下一步实测。

### 验证

- 10组生产C模块的 Cortex-M4 指令仿真回归全部通过（HAL/RTOS doubles）：IMU启动、恢复、诊断窗口、XY高偏置下Z校准、补偿后模拟90°旋转、校准边界/噪声/运动/间隙/毫秒回绕、OLED关闭/32/64、导航。
- 入口 `codingday4_work\run_cortex_tests.py`；生成文件和各组构建日志在活动工程 `build\cortex_tests`。
- 传统 `tests\sensors\run.py` 已增加新模块依赖和新用例。
- 完整ARM构建成功，61源文件，带 `W4_IMU_DIAGNOSTICS`，未定义 `W4_CALIBRATED`。
- 本聊天已在当前最终代码上补跑PC回归，16项全部通过，日志 `codingday4_work/logs/pc_tests_resume_20261118.txt`。
- 这些C仿真不是FreeRTOS真实调度或I2C电气时序验证；修复版现已烧录，校准及屏幕已得到初步实物证据；航向精度、旋转和整车导航仍需独立实测。

## 6. 当前接续入口（本轮GUI与静置记录已完成）

1. **当前固件未变**：实测轮径68mm、中心距175mm、CPR391.15版；HEX `2fbd76d74446bd76ef534455a224eb0c7a9b7daff3df92bcd2885960e2b35ff5`，ELF `8822848ddd2a599368f7a8093baaf5266c50986cc0a65a3531225804abab122b`。本轮HOTPLUG回读的85,288个有效HEX字节差异0；不需为PC脚本修改重新烧录。
2. **PC GUI已修复并真实工作**：匹配Python3.12.14的Tcl/Tk8.6.12已复制到活动工程 `.runtime/tcl`；`tools/prepare_tk.ps1`为当前进程设置TCL_LIBRARY/TK_LIBRARY，`tools/run.ps1`的Live/Demo与`setup_pc.ps1`已接入。主运行时原路径加载失败的深层原因未单独定论，但工程内副本初始化和可见窗口均已实测通过。
3. **当前持续监控进程PID=27752**（可能随用户关闭/新聊天而变化，先核对是否仍存活，勿重复建立多个BLE连接）。会话信息 `logs/gui_continuous_session_20261118.json`；无时限持续重试；原始日志 `logs/gui_continuous_20261118_214018.bin`，逐次重连与异常日志同名前缀 `_stderr.txt`。本助手无初始命令且只做订阅/读取；GUI的命令控件仍可操作。
4. **持久经验：硬件经常接触不良。用户要求持续重连，不因两次失败终止或直接判硬件故障。** 本轮第1次通知订阅Unreachable，第2次订阅成功，真实点云已通过computer-use目视确认。失败时继续保存具体异常；不要与另一个程序同时连接同一模块。
5. **雷达接线恢复已得到实测证据**：SWD两读5.309s内保持STREAM、有效点+23,320、P帧尝试+106，区间无新增UART错误/超时/丢块，历史累计overruns/dropped_blocks=64仍保留。稳定一分钟BLE切片P=1213、I/N各1223；P约19.83Hz，I/N20Hz，坏帧/缺序/重复均0；36个十度方向区间均有点。见 `logs/lidar_resume_summary_20261118.json` 与 `logs/yaw_stationary_live_20261118_214418_summary.json`。
6. **并发证据**：该切片S诊断跨60s，IMU完成+6000=100Hz，failed0、sem_coalesced未增长、calibrated1、OLED状态2；共享I2C的IMU/OLED获取分别+6000/+1200、完成+7200、超时/恢复0。7任务T帧齐全，deadline_misses均0，最小剩余栈788B。BLE RX已有一次命令接收业务，不能继续写wakes0。
7. **航向精度仍未通过**：用户重新确认水平放稳、电机动力断开、静置约一分钟；有效I帧跨度61.1s，航向−71.45°→−73.86°，净−2.41°，拟合−0.03874°/s。当前零偏−0.886450°/s，后续冻结窗口Z均值−0.904824°/s、温度28.65°C。保留XY BIAS告警；不以此宣称长期漂移解决。
8. **命令与运动锁记录**：静置采集中遥测出现 `A,1,REJECTED,HARDWARE_NOT_READY`，目标变为(0,1.4)，导航始终IDLE/fault0。本助手未发送GOTO，来源未另行确认，不宣称整段会话从未有命令。21:46:35 SWD只读确认三个ready标志0、四路PWM0，IMU有效/校准1、failed0。见 `logs/after_gui_verified_20261118_214635.json`。
9. **全段原始日志的启动边界要单独解释**：不可变副本 `logs/ble_gui_verified_20261118_214635.bin` 和 `_summary.json` 包含失败订阅前的旧I帧(ms770150)及跨重连拼接的半行，离线逐字节汇总为坏行1、I缺序862；不能据此覆盖第5项稳定切片0坏帧/缺序，也不能把完整会话写成全程0异常。GUI接收器会在重连时重置半帧/序列状态，原始字节日志不插入边界标记。
10. **用户已要求开始轮子检测，当前正在核对源码接线（详见第14节）**：用户确认左轮编码器已经接线，尚未提供引脚/通道。当前固件仅采集右轮PA7/TIM3，左轮仍是推算值；不要把推算当实测，也不要再断言实物没有左编码器。轮子阶段先确认左轮信号所接主板引脚/通道及可用计数输入，再做断电手转计数/CPR和左右轮独立测速；带动力同PWM差速、方向、PID和STOP仍需明确试验条件。
11. 本轮PC回归17/17通过，日志 `logs/pc_tests_retry_logging_20261118.txt`。新增 `capture_lidar_state.py`（只读身份/雷达计数）和 `summarize_telemetry.py`（不可变遥测副本/汇总）。全部代码、依赖、Tcl、日志和缓存留在coding，仍使用D盘TEMP/TMP/TMPDIR，不用默认C盘临时目录跑pip。
12. 持续维护本README。用户没有要求再次停止；本轮GUI保留运行和自动重连。旧第12节为历史停止记录，勿当作当前停止指令。

## 7. 尚未完成的整车任务及验收边界

- 当前源码和板上固件均已更新为轮径68mm、中心距175mm、CPR391.15；已回读确认。运动方向/速度/PID等ready标志仍为0，参数仍需带载复核。历史短圈测量仅供参考，以第1节和第10节最终记录为准。
- 当前固件仅接入PA7/TIM3右轮无方向编码器；右轮方向来自驱动指令、左轮速度仍由右轮与陀螺推算。用户最新确认左轮编码器也已经接线，引脚待确认；下一步需核对并接入左轮独立计数，不宣称当前已经有双轮反馈。
- 尚无电机转向、速度/PID、带动力STOP、三条自主路线及终点误差实测。本轮始终电机断电，没有驱动车轮。
- LiDAR、OLED、BLE先前已有真实工作证据；仍有雷达稳定性丢块、I2C恢复和BLE重连记录，需在运动负载下继续验证。
- Q_Lidar/Q_Cmd/Q_Path容量8/8/64，各16B；LiDAR和命令通路有实测，Q_Path实际导航生产/消费待证明。
- 7任务、1ms tick、控制10ms、OLED/遥测50ms；优先级、栈、周期与抖动需保留运行数据。
- RTOS模式下旧裸机按钮/电位器业务未整合；PC轨迹线是位姿历史，没有独立规划路径遥测。
- PPT目标为(0,1.4)、(-0.7,0.7)、(0.7,-0.7)米，但图中第三目标是(0.7,0.7)，需确认场地；负Y路径是待确认通道假设。
- ICP/OGM/A*是选做，不应作为必做缺陷。Part1四项队列分析未找到独立报告，状态未确认。
- 历史CODE_REVIEW.md前3项已经修复，不要直接把历史报告当作现存bug清单。

## 8. 本机工具与可复用命令

PowerShell；本机C:\ST不存在。工具根路径较长，先设专用变量，不要复用系统HOME变量：

```powershell
$pluginRoot = 'D:\EMBEGGed\st-stm32cubeide_1.17.0_23558_20241125_2245_x86_64.exe\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins'
$programmer = Join-Path $pluginRoot 'com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.0.202409170845\tools\bin\STM32_Programmer_CLI.exe'
$armGcc = Join-Path $pluginRoot 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.1.0.202410251130\tools\bin\arm-none-eabi-gcc.exe'
$workRoot = 'D:\EMBEGGed\week4codingday\coding\codingday4_work'
$firmwareHex = Join-Path $workRoot 'firmware_source\build\integration\Week4_Integrated.hex'
```

Python本轮路径（新聊天可用load_workspace_dependencies重新定位）：
`C:\Program Files\WindowsApps\OpenAI.CodexPrimaryRuntime.v26-1117-835-0_26.1117.835.0_x64__3k8sg7r9htsxt\dependencies\python\python.exe`

```powershell
# 只读枚举
& $programmer -l
# 已授权更新的烧录/校验/复位；执行前确认使用预期HEX及电机断电
& $programmer -c port=SWD sn=0674FF545155657067032031 mode=UR -w $firmwareHex -v -rst
# 以下python用上面定位的完整可执行路径，不依赖系统python别名
python -B "$workRoot\run_cortex_tests.py"
python -B "$workRoot\firmware_source\tools\build.py" --gcc $armGcc --define W4_IMU_DIAGNOSTICS
# 仅当新ELF/HEX与刚校验的片上固件匹配时使用
python -B "$workRoot\capture_window.py" --count 4 --interval 6 --label yaw_ready
```

- 硬件访问在沙箱内会错误显示No ST-Link；CIM也不能访问。用exec_command的require_escalated进行实际硬件操作。ARM nm/gcc在沙箱内可能DLL重定位失败，沙箱外正常。
- D盘不在本聊天可写根内，保存文件、构建产物也需要该权限。用户已经明确指定D盘位置，自动审查此前均允许，没有未解决的拒绝。
- C仿真依赖已存在于 `C:\Users\Hebert\Documents\ChatGPT\嵌入式开发\week3_refactor_work\test_vendor`，有unicorn/pyelftools，runner已显式加入路径，无须安装。
- 本轮先前脚本保存汇总时有一次后缀拼写错误，已修正且离线补齐；测试链接脚本缺__exidx边界也已修正并重跑全部通过。没有未处理的测试失败。

## 9. 新聊天建议开场

> 请完整读取本README，从第6节和第10节继续。最新平面航向/OLED修复版现已烧录并校验，勿再按旧交接重复烧录或误用旧ELF。持续更新README，所有文件放在coding目录。

## 10. 本聊天接续实测（2026-11-18）

- 用户重新确认主板/ST-LINK已连接、水平静置、电机动力断开；运动参数锁保留。
- 20:26枚举确认同一ST-LINK序列号；主板STM32F446xx、电压3.22V。当前HEX/ELF的SHA256与交接完全一致。
- 烧录日志：`codingday4_work/logs/flash_yaw_oled_20261118_202620.txt`；Programmer报告 `Download verified successfully`，随后复位。
- `yaw_ready_20261118_202636/202642/202648/202655` 四组 `.bin/.txt/.json` 已保存。全部calibrated=1、valid=1、calibration_samples=200、xy_bias_warning=1、failed=0、fault=0、运动标定flags=0、PWM=[0,0,0,0]；最大成功采样间隔10ms。首批含轻微扰动，不以全批原始噪声作为纯静止结论。
- 用户确认OLED能看到YAW、MOTION LOCKED和XY BIAS。
- 静止记录：`yaw_stationary_20261118_202735/202741/202748/202754/202800/202806`；汇总 `codingday4_work/logs/yaw_stationary_summary_20261118.json`。覆盖32.19s，航向0.962274°→1.224993°，净变化+0.262719°，端点平均0.008161°/s，六点拟合斜率0.007032°/s；原始Z各窗SD约0.0504–0.0789°/s。全部valid/calibrated=1、XY告警1、失败0、fault0、运动锁保留、PWM0。仅代表短时静止观测，尚非绝对航向/长期温漂验收。
- 首次无固定角度参照的手转：用户估计逆时针约90°且确认OLED变化；基准3.9301°，转后111.8826°，差+107.9524°（第二点+108.1668°）。用户表示找不准原朝向，故不评定角度精度或回差；仅确认逆时针正向响应。汇总 `codingday4_work/logs/yaw_rotation_unreferenced_20261118.json`。已改用纸张/书本直角边参照复测，结果见下。
- 当前最终代码PC回归16/16通过，日志 `codingday4_work/logs/pc_tests_resume_20261118.txt`。

- 有直角边参照复测：`yaw_ref_before_20261118_203306.json` → `yaw_ref_ccw90_20261118_203410.json`，相隔65.23s，经±180°环绕处理后转角+77.1352°，比90°少12.8648°。此次尚未通过角度验收，不据此修改比例参数。汇总 `codingday4_work/logs/yaw_reference_rotation_20261118.json`。
- 新基准时已出现累计failed=3、init_failures=1、configurations=2，表示此前发生过采样失败和重新配置；已恢复valid/calibrated=1。上述65.23s复测区间failed保持3、configurations保持2、rejected_intervals保持0、最大采样间隔10ms，fault0、PWM0。不能沿用“全程无采样失败”的说法。
- 正在采集 `yaw_ref_return_trace` 十个窗口，并提示用户用3–5秒缓慢顺时针转回原始直角边，检查转动速度、漂移和回差；用户已确认缓慢转回原边；局部回转观测-91.4309°/12.97s，邻近静止漂移-0.14268与-0.16968°/s；假设局部漂移线性变化时，离线修正约-89.4052°，该估计不作为比例标定。原始Z极值-29.473至0.504°/s；区间failed保持3、配置次数2、PWM0。汇总 `codingday4_work/logs/yaw_reference_return_summary_20261118.json`。

- 当前零偏读取值-0.813359°/s，日志 `yaw_bias_before_warmreset_20261118.txt/.json`。静止残余漂移明显，准备在已预热、用户保持静止的条件下复位重做200样本校准，区分校准时机/热漂移和转角比例问题。

- 已在预热且用户保持静止的条件下复位，固件未改变；复位日志 `codingday4_work/logs/yaw_warmreset_20261118.txt`。新六窗口汇总 `yaw_warm_stationary_summary_20261118.json`：32.15s，航向0.119088°→0.544885°，净变化+0.425797°，端点平均0.013244°/s；全部valid/calibrated=1、样本200、failed=0、配置次数1、运动锁保留、PWM0。重校准后短期漂移显著小于重校准前的约-0.15°/s，但尚需再次做直角参照转动；不要据此宣称长期漂移解决。

- 预热后直角边参照缓慢逆时针复测：`yaw_warm_ccw_trace` 十窗口；运动前局部基准1.222728°，运动后停稳91.124819°，差+89.902090°（与90°差-0.097910°，两个端点相隔25.77s）。failed0、配置次数1、valid/calibrated1、PWM0。汇总 `codingday4_work/logs/yaw_warm_ccw_summary_20261118.json`。该结果仅为人工直角参照的缓慢转动验证，长期漂移和恢复后校准仍未验收。用户已确认缓慢回转，结果见下面的往返汇总。
- 只读工具 `capture_window.py` 增加当前零偏 `gz_bias_dps` 和右轮编码器累计上升沿/有效捕获/无效/过捕获/定时器溢出计数，地址仍从匹配ELF取得；未改变任何固件。已通过 `yaw_warm_return_final` 两次真实只读采样验证新增字段：零偏-1.000572°/s、累计edge=405/capture=403/invalid=1/overcapture=0且两次计数不变；可用于手转轮CPR测量。

- 预热后往返汇总 `codingday4_work/logs/yaw_warm_roundtrip_summary_20261118.json`：返回原始直角边后最终YAW=14.726701°，相对逆时针前局部起点1.222728°的残差+13.503972°，往返端点间隔160.89s。这包含累计漂移和人工对齐误差，尚不能宣称整车航向精度验收通过。最终静止Z均值-0.932214°/s与当前偏置-1.000572°/s仍有差异；温度28.438°C；failed0、配置1、fault0、PWM0、XY告警保留。
- 当前运动锁继续保留；先进行与航向独立的手转右轮CPR和几何测量。已请用户保持电机断电、稳固架起车体并标记右轮起点，等待其准备确认和外径/轮距实测值；不要提前采用edge405作为架起后的基准。

- 用户提供几何：轮径6.8cm、两轮最外侧总宽20cm、胎宽2.5cm。按左右胎宽相同且200mm指外侧总宽，中心距=200−12.5−12.5=175mm。数据与推导保存 `codingday4_work/logs/physical_calibration_20261118.json`；代码默认值暂未改，等待CPR一起批量更新，运动验证标志仍false。
- 用户确认车体已架稳、右轮标记对齐且未转轮；第一轮基准 `encoder_baseline_1_20261118_204731.json`：edge405/capture403/invalid1/overcapture0，fault0、PWM0。已请同方向手转准确两圈，等待完成确认再读取终值。
- 正在活动工程内建立 `.venv` 并安装原有 `pc/requirements.txt`；依赖安装结果待确认，日志目标 `codingday4_work/logs/pc_dependencies_20261118.txt`。尚未把PC蓝牙连接或可视化报告为已恢复。

- 右轮第一轮：`encoder_baseline_1_20261118_204731.json` edge405 → `encoder_after_1_20261118_204902.json` edge1214，两圈809边沿=404.5/圈。
- 第二轮：edge1214 → `encoder_after_2_20261118_205032.json` edge1999，两圈785边沿=392.5/圈。两轮各invalid+1（停轮后首个周期间隔超过5秒可触发，edge总计仍逐上升沿递增），overcapture均未增加；不能仅凭overcapture0认定物理信号完全无丢边。
- 两轮结果差约3%，CPR尚未确认，不能直接采用平均398.5、默认330或历史454。已请用户以edge1999/当前标记为起点，同方向准确转10整圈；待确认后读取终值。所有轮次保存在 `physical_calibration_20261118.json`，运动锁未改变。

- 第一次十圈：edge1999 → `encoder_after_10_20261118_205218.json` edge5903，共3904上升沿=390.4/圈，overcapture未增加。第二次十圈以当前edge5903为基准，已请用户同向准确转10圈，等待其完成确认；CPR仍未最终采纳。

- 第二次十圈：edge5903 → `encoder_after_10_repeat_20261118_205411.json` edge9822，共3919边沿=391.9/圈。两次十圈390.4和391.9相差约0.3835%，合计7823边沿/20圈=391.15/圈。以长测量均值作为待载荷复核的比例，短两圈结果未纳入；无方向通道仍不能验证前进方向。
- `week4/Src/w4_calibration.c` 已将默认轮径/中心距/CPR改为0.068m/0.175m/391.15；方向和全部ready标志未放行，仍未定义W4_CALIBRATED。源码已更新，待回归、全编译和烧录；板上当前仍为旧参数。
- 更新前已匹配主板的HEX/ELF及build.log完整备份到 `codingday4_work/logs/firmware_before_geometry_20261118`，SHA256不变。**新构建开始后，必须先烧录校验新的HEX，再运行使用活动ELF的采集脚本。**

- 实测参数版已通过10组Cortex-M4回归、61源文件完整ARM编译；text84428/data860/bss61840，仍只有旧裸机unused及HAL参数警告。日志 `geometry_cortex_tests_20261118.txt`、`geometry_build_20261118.txt`。
- 20:58已烧录、校验和复位，日志 `flash_geometry_20261118_205800.txt`；当前HEX SHA256 `2fbd76d74446bd76ef534455a224eb0c7a9b7daff3df92bcd2885960e2b35ff5`，ELF SHA256 `8822848ddd2a599368f7a8093baaf5266c50986cc0a65a3531225804abab122b`。身份记录 `last_verified_flash_20261118.json`。
- 基于匹配ELF只读回传完整w4_config并校验：轮径0.068000m、中心距0.175000m、CPR391.149994，三个ready标志[0,0,0]；日志 `geometry_config_verified_20261118.bin/.txt/.json`。固件已实际更新，不是仅改磁盘源码；20:59两次geometry_verified读取确认calibrated/valid=1、样本200、failed/fault0、XY告警1、四路PWM0。

## 11. 下载目录纠正与持久约束（本聊天用户新要求）

- 用户明确要求下载文件都留在 `D:\EMBEGGed\week4codingday\coding`，并要求时刻更新本README。
- 已停止原pip任务，识别本任务两次pip运行产生的C盘临时目录，逐文件复制并校验SHA256后清除对应C盘副本：共88目录、25,388,483字节，迁入 `codingday4_work/firmware_source/.runtime/pip-recovered/c_temp_20261118`。详细源/目标/哈希清单 `codingday4_work/logs/download_migration_20261118.json`；未操作其他历史目录。
- 新增 `firmware_source/tools/setup_pc.ps1`，设置TEMP/TMP/TMPDIR、pip缓存、matplotlib缓存、pycache和通用缓存到工程内.runtime；安装前先检查Python实际tempfile目录。
- `tools/run.ps1`也已加入上述D盘临时/缓存设置，后续Live/Demo/Build/Test均继承。脚本是PC环境/启动器改动，未改变当前已烧录固件。
- 完整wheel已留在 `.runtime/wheelhouse`；下载和安装尚未完成，当前无后台pip安装进程，有D-only续传工具运行。下一步从第6节继续，切勿误报已连上BLE或GUI已运行。

- 迁移后复查清单中的88个C盘源目录均已不存在，D盘对应目录均存在；`setup_pc.ps1 -SkipInstall`实测 `tempfile.gettempdir()` 为D盘工程 `.runtime/tmp`。
- 新增 `tools/download_pc_wheel.py`：PyPI官方元数据定位包、支持断点HTTP Range续传、写D盘.part，完成后核对大小/SHA256与CRC再改为.whl；当前正在处理matplotlib大包，尚未成功安装或启动监控。源码/日志/部分下载均在coding目录。

- PC当前下载检查点：matplotlib完整9,349,409字节已在D盘续传完成，官方SHA256及CRC通过；SHA256 `c5c1c68ee401fc98271263410f0e5ce88285abacf7627132914e8adf3d70ff43`。D盘setup_pc.ps1已完成完整本地wheel与其余组件安装，实际导入验证通过。

- 绘图库续传已完成并核验大小/SHA256/CRC；`setup_pc.ps1`现在显式传入完整本地wheel，避免再次下载同一大包，安装中所有TEMP/TMP/TMPDIR/缓存继续指向D盘。安装完成/导入验证/实机BLE连接仍待后续结果。

- PC依赖安装成功，日志 `pc_dependencies_D_only_20261118.txt`；实际导入matplotlib/pyplot、bleak、serial通过，新装模块路径均在D盘 `.venv`。环境版本/模块位置/tempfile证据 `pc_environment_verified_20261118.json`。完整matplotlib wheel大小9,349,409B，官方SHA256/CRC通过。
- 当前进入真实BLE只读采集：目标C4:25:01:17:00:02，运行PC监控器headless约40秒保存原始遥测；未发送GOTO/RESET/STOP或电机指令，连接与数据结果待实际完成确认。

- 真实BLE日志 `ble_geometry_20261118_211611.bin` 及 `_summary.json`：38.95s内N/I各780帧、均20Hz、序号缺失0、坏帧0；S帧38s内IMU完成3800次=100Hz，failed0，sem_coalesced保持11，OLED状态2。B帧IMU/OLED mutex获取分别+3800/+760、完成+4560、超时/恢复均0，IMU最大等待0tick、OLED最大持锁3tick。7任务T帧均已取得，最小剩余栈848B，deadline_misses均0；BLE RX事件任务此时未发命令所以wakes0，不把它称作已接收业务。
- 上述采集P帧0；用户随后明确“刚刚雷达线断了现在好了”。当前按接线恢复后重新观察LiDAR自动恢复，不把无P帧直接当成软件缺陷或硬件通过。
- STOP往返记录 `ble_stop_locked_20261118.bin/.txt/_summary.json`：ACK=[{'cmd_id': 1, 'status': 'STOPPED', 'reason': 'STOP_LATCHED'}]，N状态=['IDLE', 'STOPPED']。只证明断电/锁定下蓝牙命令送达与响应，尚非带动力STOP试验。

## 12. 用户要求停止的收尾

- 用户明确“先停止”，已停止后续任务，仅保存进度和清理本次任务进程；没有继续拷贝Tcl、重新安装、读取主板或做硬件试验。
- 固件已上板，运动锁保持，最后一次只读PWM全0；断电STOP收到STOP_LATCHED。不能把这等同于带动力刹停验证。
- 所有最新状态与恢复入口以第6节为准。PC依赖已装完，GUI运行时问题仍未解决，LiDAR重新接线后的点云尚未复核。

## 13. 新聊天恢复记录（2026-11-18 21:31起）

- 用户明确要求读取本README继续，并持续记录；已完整读取主README、当前启动/环境脚本和GUI错误日志。README修改前备份：`logs/README_before_resume_20261118_2131.md`。
- 用户重新确认：主板已连接，电机动力仍断开。后续先进行软件修复及只读BLE，不解除运动锁、不发送GOTO；固件尚无新修改，不重复烧录。
- 当前正在定位Python3.12.14/.venv匹配的Tcl8.6.12数据路径问题。尚未将Tcl复制到D盘、尚未宣称GUI恢复或LiDAR重连成功。
- 已纠正第7节遗留的旧几何/CPR描述；旧第12节停止记录保留为历史，当前以本节和第6节为准。
- 21:33（修正此前手填时间）：新增 tools/prepare_tk.ps1，从当前.venv对应Python基座复制匹配Tcl/Tk数据到工程.runtime/tcl，并仅为当前进程设置TCL_LIBRARY/TK_LIBRARY；run.ps1的Live/Demo及setup_pc.ps1均已接入。原脚本备份 logs/pc_launcher_before_tcl_20261118。Tcl/Tk 8.6.12解释器及隐藏窗口创建/销毁实测通过（tcl_fixed_resume_20261118.txt）；尚待可见GUI和真实BLE验证。

- 20261118_213409：已启动90秒真实BLE GUI验证，PID=11076，无初始命令；原始日志 D:\EMBEGGed\week4codingday\coding\codingday4_work\logs\gui_resumed_20261118_213409.bin。此时仅启动，连接/可见画面/点云待核验。

- 21:36：90秒真实GUI窗口已通过computer-use目视确认，Tcl错误消失；BLE一直断开，原始日志0字节，无点云。独立扫描未见目标广播；BleakClient返回connected=True但服务列表为空，订阅报BleakCharacteristicNotFoundError，证据 logs/ble_scan_resume_20261118.txt。用户确认模块灯正常且已断开其他连接；继续查连接/服务发现，不认定LiDAR软件故障。已修复PC监控器吞掉BLE异常的问题（状态栏保留异常并输出stderr），补对应回归，README_PC已更新；未改固件。

- 20261118_214018：用户补充硬件经常接触不良，要求持续重连，不因两次失败终止或定性故障。PC BLE通路本来会持续重试，现新增逐次连接/订阅日志，并启动无时限真实GUI（PID=27752）；无初始命令。会话入口 logs/gui_continuous_session_20261118.json，原始遥测 D:\EMBEGGed\week4codingday\coding\codingday4_work\logs\gui_continuous_20261118_214018.bin，异常/尝试 D:\EMBEGGed\week4codingday\coding\codingday4_work\logs\gui_continuous_20261118_214018_stderr.txt。之前90秒验证已自然退出；服务缓存和限定服务的短检验超时仅记录观测，不作硬件故障结论。

- 21:42：新增只读工具 capture_lidar_state.py；先通过HOTPLUG回读确认活动HEX的85,288有效字节全部匹配（差异0），再按ELF源码作用域解析雷达静态符号，未停核/复位/写硬件。两次读数5.309s内LiDAR保持STREAM，points_emitted增加23,320、交付106半块/P帧尝试增加106；UART错误/超时/恢复/丢块计数均无新增。累计overruns/dropped_blocks=64、parser历史重同步128字节，不能宣称长期无丢块；此次区间稳定。fault0、运动flags0、四路PWM0。证据 logs/lidar_resume_summary_20261118.json；说明主板当前正在接收雷达数据，真实PC点云仍等BLE订阅成功。

- 本轮结案证据：GUI真实点云已目视确认；17项PC回归通过。稳定一分钟切片坏帧/缺序/重复0，P1213、I/N各1223；I2C共享与7任务并发证据见第6节。完整日志含初次订阅失败/重连边界，已单独保存并如实标注。
- 用户确认左轮编码器已经接线，并要求“先完成现在的任务，一会再做轮子”。已完成本轮GUI恢复、点云和静置记录；左轮输入引脚/通道和独立转速检测留为下一步，不改动任何运动锁。
- 静置约61.1s航向净变化−2.41°，拟合−0.03874°/s，航向精度仍待改进。采集中观察到一条GOTO被HARDWARE_NOT_READY拒绝；来源未另确认。本助手未发送驱动命令。21:46:35只读回查flags0、PWM全0、fault0、IMU校准有效1/failed0，当前HEX/ELF保持一致。

## 14. 左轮测速阶段（2026-11-18 21:51起）

- 用户要求开始任务。用户确认左轮编码器已经接线，并要求先从代码自行查找；已完成源码/备份检查和实物只读对照后，才请求核对实际信号线接口。
- 当前已检查：活动工程Core/main.c、app_encoder.inc、app_state.inc、week4/w4_pose_single.c和w4_port_stm32.c；Week4原整合和原烧录工程；Week2整理版及.ioc；Week3模块化工程和previous_working_version.zip原始源码备份。
- 目前上述源码均只初始化PA7/TIM3：CH2直接捕获同一PA7的上升沿，CH1间接捕获该信号的下降沿；不能把这两个通道当作左右轮各一路。PB14/PB15是左电机PWM输出，不是左编码器输入；PB6/PB7为右电机PWM。
- Week2 README明确“本程序仅测右轮编码器”；.ioc没有完整记录手写定时器配置。历史原工程G盘路径当前不存在；不会将不可用路径当作已检查原件。
- 用户所述左轮实物接线事实保留，但尚未从现有源码找到其信号引脚。接下来完成压缩备份核对，并尝试不改引脚、不写寄存器的GPIO输入电平读取，配合断电手转左轮定位；实际开始手转前先确认车体架稳、电机动力断开。
- 当前没有新固件修改/构建/烧录，没有解除运动锁或发送带动力动作。持续BLE GUI原会话保留，避免重复连接。
- 用户已确认车体架稳、电机动力断开、轮子暂不转。已核对Week2 RAR中的main.c/app_encoder.inc/app_state.inc与解压工程逐字节一致。21:55开始只读GPIO基线；新增capture_gpio_inputs.py只读A/B/C模式及输入电平，并逐次核对PWM0，不重设引脚。稀疏电平变化仅用于定位信号，不作为编码器计数、CPR或RPM。GPIO初读RCC_AHB1ENR=0x00600007，仅A/B/C时钟开启；D/H未改动。

- 21:56静止GPIO基线完成：logs/gpio_left_baseline_20261118_215619.json，54个有效采样、读取错误0，所有未占用候选输入电平均稳定；UART/I2C活动符合已有业务，四路PWM始终0。准备只转左轮进行对照，不把轮子带转右轮或UART/I2C变化误判为左轮信号。

- 左轮复测：用户确认仅左轮手转约20秒、右轮未动，采集收到完成回复后才结束；logs/gpio_left_confirm_20261118_215925.json，覆盖约67秒、约300个有效读数，候选输入变化0、PA7变化0，UART/I2C仍活动，PWM全0，连接错误0。结果仅表示当前已开启A/B/C数字输入没有观察到左轮信号，不能排除模拟模式、未开启端口、接触不良或编码器未供电；不判硬件损坏。正在做右轮PA7正对照。

- 22:03阶段汇总：`logs/left_encoder_pin_check_summary_20261118.json`。各阶段真实采样如下：
  - gpio_left_baseline: 54个有效样本，跨度11.78s，读取错误0，候选输入变化{}，非零PWM样本0。
  - gpio_left_turn: 192个有效样本，跨度44.86s，读取错误0，候选输入变化{}，非零PWM样本0。
  - gpio_left_confirm: 316个有效样本，跨度71.17s，读取错误0，候选输入变化{}，非零PWM样本0。
  - gpio_right_control: 275个有效样本，跨度61.98s，读取错误0，候选输入变化{}，非零PWM样本0。
- 右轮正对照：仅手转右轮、左轮不动，PA7稀疏电平变化50次；ISR累计上升沿2→2176（+2174）、capture1→2174、invalid0→1（长时间停轮后的首周期可无效）、overcapture保持0。右轮计数输入可用；本轮没有准确数圈，不能将2174用于CPR标定或用整段时间计算实际转轮RPM。
- 两次仅转左轮均未在候选数字输入识别到新活动；不能据此断言左编码器未接或损坏。只读采样没有改引脚/上拉/端口时钟，D/H时钟未开、PA0为模拟模式等仍是检测边界。当前尚未测得左轮独立RPM，不修改CPR_L或解锁运动。
- 已请求用户沿左轮编码器输出信号线查看主板/转接板接口标号；这是源码检查后的必要实物信息。待其回复后，先核对引脚占用与电平兼容，再接入独立计数、断电手转及CPR验证。PB14/PB15是左电机PWM，不能当作左编码器输入。
- 22:02:14末次SWD状态：IMU valid/calibrated1、配置1、failed累计1（相比21:46已增长，不能再声称全程failed0）、fault0、ready flags0、四路PWM0。无新固件修改/构建/烧录或动力动作。

## 15. 当前聊天：双编码器接线核对（2026-11-18 22:36）

- 用户指定从 `C:\Users\Lenovo\Desktop\ESD\WEEK4\coding` 读取并继续。实际README位于其下的 `coding\README.md`，活动工程位于其下 `codingday4_work\firmware_source`。旧D盘路径在本机不存在；D盘是ST-LINK存储设备。本次更新保存在用户指定的C盘工程，不把C盘误说成旧D盘最新目录。
- 用户确认左电机驱动接PB14/PB15、右电机驱动接PB6/PB7；源码分别配置TIM12 CH1/CH2、TIM4 CH1/CH2为电机PWM。这些不是编码器信号输入，正反极性仍须电机动力试验确认。
- 用户确认E1A接CN10-15，E1B未接，E2A/E2B均未接。ST NUCLEO-F446RE手册的CN10-15对应PA7，正是现有TIM3测速输入。此前单转右轮曾使PA7计数2→2176；E1A究竟对应哪个物理轮须结合接线并复测，不能仅凭E1标签推断。常见标法E1A/E1B是同一编码器的A/B相，E2A/E2B属于另一组，但具体模块仍须核实。
- 依据ST官方引脚表及当前源码提出方案：E1A→CN10-15/PA7（保持），E1B→CN10-13/PA6（TIM3 CH1），E2A→CN10-4/PC6（TIM8 CH1），E2B→CN10-19/PC7（TIM8 CH2）。这是**待接线/待验证方案**，不是已连接的事实。先确认实际排针空闲、驱动板编码器输出3.3V兼容及共地；信号电平未知时不接线。接线前断开电机和主板USB所有供电，接好后先只恢复主板及编码器所需供电、保持电机动力断开。
- 官方资料：[NUCLEO-64硬件手册](https://www.st.com/resource/en/user_manual/um1724-.pdf)、[STM32F446数据手册](https://www.st.com/resource/en/datasheet/DM00141306.pdf)。当前TIM3 CH1为PA7的间接下降沿捕获，不是PA6独立输入；接线后需修改固件、回归测试并单独手转确认两轮方向/计数/CPR，不能直接声明双轮测速或解除运动锁。
- 本机C盘活动HEX SHA256为 `2fbd76d74446bd76ef534455a224eb0c7a9b7daff3df92bcd2885960e2b35ff5`，ELF SHA256为 `8822848ddd2a599368f7a8093baaf5266c50986cc0a65a3531225804abab122b`，与历史记录一致；当前板上尚未重新核对。本机ST-LINK序列号仍为 `0674FF545155657067032031`、虚拟串口COM6，CubeIDE位于 `C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins`。原PID 27752不存在，本聊天未启动新的BLE监控或发送命令。
- 待用户确认编码器输出电压和接口空闲；接线后待用户确认主板连接、车体架稳、电机动力断开，再只读核对板上固件/锁/PWM，并安排手转测试。尚未改变任何固件或电机输出。
- 后续更新：用户表示三根新线已按方案接好，并确认主板USB已连接、车体架稳、电机动力断开、编码器信号与3.3V兼容。`logs/board_identity_20261118_224201_203735.json`：本次HOTPLUG回读85,288个有效HEX字节差异0，flags=[0,0,0]、PWM=[0,0,0,0]、fault0；本次PA7边沿0。日志中的数值是非原子依次读取。
- 新增 `codingday4_work/read_current_board.py`：在解释ELF符号前校验记录的HEX/ELF哈希和板上有效HEX字节，使用本机CubeIDE中的`arm-none-eabi-nm`，无需旧机器的pyelftools路径；不会复位、停核、写片上内存或发送运动指令。已对历史flash副本、单字节损坏、日志解析及ELF符号离线验证通过。`capture_gpio_inputs.py`改为从本机工具读取CLI并标注四路E1/E2候选脚，仍为只读稀疏采样。
- 现机PC回归17/17通过，`logs/pc_tests_current_20261118.txt`。沙箱内Windows本地asyncio socketpair无法建立，测试改在沙箱外运行后通过；未连接真实BLE。
- `logs/encoder_new_baseline_20261118_224258.json`：静止10.3秒、26个样本、读取错误0、四路PWM持续0；PA6/PA7/PC6/PC7电平分别稳定为0/1/1/0。静止电平不能证明编码器波形正确。已开始 `encoder_new_left` 只读采样，等待用户只手转左轮15–20秒并确认；随后停止采样、检查四脚变化，并单独对右轮做对照。稀疏电平变化不能作为CPR、RPM或方向证明。
- 两轮断电手转对照已完成：`logs/encoder_new_left_20261118_224406.json`（187样本，约72.42s，错误0）仅左轮转动时PC6变化28次、PC7变化22次，PA6/PA7变化0；`logs/encoder_new_right_20261118_224545.json`（167样本，约64.39s，错误0）仅右轮转动时PA6变化20次、PA7变化18次，PC6/PC7变化0。两轮采样PWM均全0。这支持E2A/PC6、E2B/PC7对应左轮，E1A/PA7、E1B/PA6对应右轮；读数为低频抽样，只证明有电平变化，不证明完整波形、正交相位、方向、实际脉冲数或CPR。
- 用户提醒右轮以前已测：`logs/physical_calibration_20261118.json` 记录两次各10圈，3904/3919个PA7上升沿，平均391.15上升沿/圈，当前固件已使用；无需重复两次十圈。四倍频A/B计数理论上可把该值乘4作为初始比较，但仍需验证定时器模式、两相和方向。记录未给出带电运行的实际RPM。
- 已准备独立的 `W4_ENCODER_AB_DIAG` 诊断构建，仅测计数、不接入导航：TIM3读右轮PA6/PA7，TIM8读左轮PC6/PC7，编码器模式双边沿四倍频。该模式 `W4Port_EncodersReady` 永远false、PWM写入强制0，且与 `W4_CALIBRATED` 同时定义会编译报错；现有右轮单路模式默认保留。改动前源码和原HEX/ELF备份在 `logs/before_dual_encoder_diagnostic_20261118/`。
- 独立诊断构建 `firmware_source/build/encoder_ab_diag` 完成62个ARM源文件编译，HEX SHA256 `4f8575440c16a29e9f8cee3e130ba8acfcbdf73d026dc2e786ee1191fd1c34d3`，ELF SHA256 `faea53f1cc60012fd452bc5f262d75795169938df08942800862b63d74ef7604`。导航及9项传感器host回归、诊断引脚配置/失败路径、禁止校准宏测试均通过；ARM反汇编确认ready返回0、四个PWM比较值写0。诊断固件此时尚未烧录；实际计数和方向仍待上板验证。
- **以上“尚未烧录”为构建时历史状态。** 22:58:05已烧录并验证诊断固件，`logs/flash_encoder_ab_diag_20261118_225805.txt`、`verified_encoder_ab_diag_20261118.json`；复位后 `board_identity_20261118_225828_793926.json` 回读84,688有效字节差异0、ready flags0、PWM0、fault0。旧的 `build/integration` 不匹配当前板上程序。
- `ab_baseline_20261118_225844_599708.json` 右/左初始计数均0；TIM3与TIM8均运行、SMCR=3、双通道直接输入使能、ARR65535。用户左轮正向手转10圈后 `ab_left_10_turns_20261118_225959_546008.json`：左+15517、右0；反向10圈后 `ab_left_reverse_10_20261118_230100_053630.json`：左计数65399，按16位回绕反向增量−15654，右仍0。两次幅值相差137计数、约0.88%；合计31171/20圈=1558.55个四倍频计数/圈（约389.6375个A相上升沿/圈）。该比例暂供测试，仍需带载复核；计数正负与小车前进方向的关系还未验证。
- 右轮新接B相只做约一圈正反检查：`ab_right_one_turn_20261118_230204_668898.json` 为+1579；`ab_right_reverse_one_20261118_230251_163084.json` 为9，反向增量−1570。短测用于验证方向，继续沿用先前右轮391.15 A相上升沿/圈（按四倍频换算1564.6），没有用短测替换它。反向时左轮计数−18，用户明确说明是车体微动；`ab_stationary_check_20261118_230330_360936.json` 在随后约39秒与前值完全相同。所有这些采样PWM0。
- 新增 `monitor_ab_speed.py`，仅通过SWD周期读取计数计算双轮有符号RPM，取差时处理16位回绕；右/左四倍频比例分别1564.6/1558.55。实测方向符号目前仅为原始定时器正负，尚未与车体前进方向标定；采样区间平均RPM非驱动电机测试。现正等待用户依次手转左右轮和同时转轮，采样结果完成后写入 `logs/ab_speed_*.json`。
- 实时手转测速已完成，`logs/ab_speed_20261118_230613_472408.json` 共132个样本、错误0、PWM始终0：左轮单独转动时8个约0.75s窗口显示左轮大于2 RPM且右轮低于0.5 RPM；右轮单独转动时同样8个窗口；两轮同时转动时8个窗口均大于2 RPM；末次两轮均0。一次同时转动窗口左约6.9、右约10.8 RPM，均为手转区间平均，不是带动力转速或精度验收。
- 用户确认左轮计数的18步变化是车体微动。用户要求尽快验证功能，并确认车体能稳固架空、两轮离地、电机电源可随时单独断开，当前电机动力仍断开。下一步准备受限的单轮低速脉冲测试并先在无动力下验证停止；尚未接通电机动力，也未发送电机驱动命令。

## 16. 架空电机脉冲测试（2026-11-18 23:24）

- 用户要求尽快测试核心车轮，并确认车体稳固架空、两轮离地、电机电源可随时单独断开。临时 `W4_MOTOR_PULSE_DIAG` 通过SWD内存请求触发单轮0.5秒脉冲，TIM2中断清零全部PWM；该版不运行RTOS导航、蓝牙、雷达或OLED，不允许 `W4_CALIBRATED`。**空闲PWM为0，但测试版确实可使电机转动，别误称运动锁使其完全无法驱动。** 本版完成后须恢复完整传感器/导航固件，不能把它留作最终交付。
- 10%版独立构建 `build/motor_pulse_diag`，HEX `6d94fb2008bdd179bd043b1a6aa94cefa75888944b64c86210b370cfc268af5c`、ELF `5cdc86a6e2219f372f18e38ac2cd898b5d3ccb0e188e80a72c086ee4dccf140b`；本地测试覆盖单路、限时中断停止、无效命令拒绝和运行时请求不排队；TIM2 IRQ向量已核对。`motor_diag_identity_20261118_231645_848669.json` 回读38,076有效HEX字节差异0、初始PWM0、fault0。
- 电机动力断开时发送一次左轮测试请求：`motor_diag_no_power_after_20261118_231746_287852.json` 记录TIM12 CH2=450/4500、持续500ms、停止后PWM全0、fault0。STM32 Programmer对瞬时RAM请求写入返回 `Error: Failed to download data!`，但固件已立即消费请求并记录completed+1；不得因CLI错误自动重试，以免重复触发。
- 用户后来说明第一次两轮测试时电机实际未接入，因此不能以这些结果判断启动占空比；电机接好、动力接通并确认架空后又各测一次10%：`motor_diag_connected_left_20261118_232118_521119.json` 和 `motor_diag_connected_right_20261118_232150_561879.json` 显示各500ms并归零、对应编码器脉冲均0。仅凭此不能断定电机故障或10%是启动阈值。用户随后要求“PWM开高点。至少30%”。
- GPIO/TIM只读配置 `motor_pinmux_registers_20261118.txt` 显示PB6/PB7为AF2、PB14/PB15为AF9，TIM4/TIM12的两个输出通道均已启用。用户已确认**电机动力重新断开**、USB保留；正在编译30%（1350/4500）0.5秒单轮脉冲版。上板后先断电触发并回查，再请用户接通电机动力、分别测试并记录转向和编码器增量。
- 30%版编译及单路/定时停止/禁止重复测试通过，已烧录校验，`verified_motor_pulse_30_20261118.json`；`motor_30_identity_20261118_232614_432022.json` 回读38,076字节差异0、PWM0、fault0。用户此前确认电机动力断开；断电条件下单次左轮试脉冲 `motor_30_no_power_20261118_232638_682560.json` 记录左CCR=1350/4500、500ms后四路PWM0，但左计数0→314、右计数不变。已请用户确认当时是否手动碰动或轮子自行转动；在排除误碰与未知供电前不继续带动力测试。
- 用户未注意第一次是否碰动；在用户确认动力断开并盯住左轮时，再触发一次30%：`motor_30_observed_no_power_20261118_232809_250369.json` 左计数543→888（+345）、右0、PWM500ms后全0。用户确认**左轮没有动**；因此PWM期间编码器计数可能受干扰或有其他异常，不能当作可信的运动反馈或用于解锁导航。
- 用户随后确认已接通电机动力、车体仍架空；各执行一次30%、500ms脉冲：`motor_30_powered_left_20261118_232930_635469.json` 左计数+317，`motor_30_powered_right_20261118_232955_314318.json` 右计数+268，两次均state2、PWM全0、fault0。由于断动力时也有计数，**这些计数不能单独证明车轮实际转动或给出可信RPM**；正在等待用户目视说明左右轮是否转动。所有SWD请求均只发送一次，CLI因RAM请求被及时清除仍报写入失败，均通过completed记录确认执行，不能自动重试。
- 用户最终确认两轮在30%测试中都未转动，随后要求双轮同时输出40% PWM、持续5秒。保留旧单轮命令参数，独立构建增加双轮请求，两路比较值均为1800/4500，TIM2在5秒终点同时清零四路PWM。单轮及双轮配置的本地回归通过，包含停止时清除未处理的重复请求。
- 用户确认手动模块是接入STM32的按键或电位器。此测试固件直接处理SWD请求，PWM和时长不取决于按键或电位器设置。
- 双轮版 `build/motor_pulse_40_both` 已烧录。`logs/verified_motor_pulse_40_both_20261118.json` 和 `logs/motor_40_both_identity_20261118_233458_025741.json` 记录身份、烧录、38,140字节回读一致。用户复接动力并保持车体架空后触发一次，`logs/motor_40_both_powered_20261118_233557_029039.json` 记录左右比较值各1800/4500，59720−54720=5000ms，completed=1、PWM全0和fault0。CLI仍报瞬时写入错误，固件回读确认只执行一次。
- 用户目视确认两轮都转动、速度不同。左右原始计数分别+15,316和+13,206，方向未与车体前进方向对应。此前断动力且轮子未动时仍读到计数，实际RPM与两轮速度差数值待独立测量。带动力转动功能已验证，速度一致性和闭环仍待验证。

## 17. 双轮占空比配平

- 用户要求分别调节两轮速度以便直行，并已在两轮贴上可见标记。使用相同的电源电压，通过左右独立的PWM配平。架空试验时用轮上标记比较相同时间内的圈数，并确认两轮在车体坐标中的转向。架空配平后再做带载直行检查。
- `build/motor_pulse_trim` 已编译，单轮、双轮及配平模式测试均通过。它支持左右分别设置32%至40%的占空比，架空测试限5秒，短距离测试限1.5秒。该版烧录和实测状态见后续记录。
- 用户断动力后烧录，`logs/verified_motor_pulse_trim_20261118.json` 和 `logs/motor_trim_identity_20261118_234426_499781.json` 记录38,204字节一致、空闲PWM为0。动力接回后完成40%/40%基准，见 `logs/trim_post_bench_40_40_20261118_234459_802_20261118_234507_429700.json`。左右原始计数+14,803/+13,049，自动归零时间5000ms，等待目视确认。左、右真实转速以独立标记或测速仪核实。
- 按用户要求重复40%/40%与5秒，`logs/trim_post_bench_40_40_20261118_234633_177_20261118_234640_739219.json`，左右原始计数+15,683/+13,134，脉冲5000ms后四路PWM均为0，故障码0。
- 用户从标记观察40%/40%复测左约10.25圈、右约9.17圈，左比右多约11.8%。首次配平改为左38%/右40%，持续5000ms，读回比较值1710/1800、四路PWM全0、fault0，见 `logs/trim_post_bench_38_40_20261118_234912_144_20261118_234919_747189.json`。等用户反馈轮上标记观察结果。

## 18. 双轮编码器闭环（2026-11-19）

- 用户要求根据每轮每圈的四倍频计数统一转速。左轮采用1558.55计数/圈，右轮由原391.15个A相上升沿/圈按四倍频换算1564.60计数/圈。右轮比例仍需带电实测核对。公式为 `RPM = 60 × 窗口计数增量 / (每圈计数 × 窗口秒数)`。
- 用户确认主板USB连接、车架稳固、电机动力断开，并说明编码器随电机动力一起断电。此前断动力时GPIO无上下拉，编码器输出悬空可能解释PWM期间的异常计数，需在编码器供电时复核。
- 新增 `week4/Src/w4_wheel_speed.c`，分别换算两轮RPM并独立PI调PWM，采样间隔50ms、速度低通时间常数100ms，测试输出上限40%。支持16位计数及毫秒回绕，异常时间间隔、逆向计数或异常高转速会停止。连续1秒未测得转动会提前结束测试。
- 新增 `Core/Src/w4_motor_speed_diag.c`，通过 `0x5350005A` 请求双轮90 RPM架空测试，总时长5秒，由TIM2中断归零。记录100项以内的逐周期两轮转速、计数增量与PWM。停止中断后再次提交PWM的竞态已有最终状态检查。
- 本地测试通过计数比例、回绕、输入异常、双电机不同响应下的闭环收敛、限幅、缺反馈提前停和5秒后保持停止。65源文件ARM构建通过，输出 `build/motor_speed_loop`，目前待上板验证。没有宣布已实现落地直行。
- Codex运行时升级使旧`.venv`的Python路径失效。硬件检查和构建已使用本机Python3.13标准库完成，`.venv/pyvenv.cfg`已改为新版本的同版本Python3.12.14，旧配置备份到 `logs/pyvenv_before_runtime_update_20261119.cfg`。
- 已烧录，身份 `logs/verified_motor_speed_loop_20261119.json`、`logs/speed_loop_identity_20261119_152218_710183.json`。用户接回电机和编码器动力后静止读数 `speed_powered_stationary_before_20261119_152329_090545.json`、`speed_powered_stationary_after_20261119_152338_147787.json` 两轮均0、PWM0。
- 首次90RPM闭环请求，`logs/speed_post_90_20261119_152408_653_20261119_152415_943004.json` 和 `_speed.json` 记录启动/停止114204/115204ms，两轮增量均0，输出比较值均1800（40%），1秒后缺反馈保护停机，PWM0、speed fault2、MCU fault0。正在向用户核对是否实际转动、动力/编码器供电情况。不能宣称闭环已在实物收敛。
- 用户要求原参数再试，第二轮 `speed_post_90_20261119_152503_636_20261119_152510_563197.json` 完整运行5秒，100个采样，speed fault0、MCU fault0、PWM归零。左计数+12575≈8.07圈，右+11625≈7.43圈。3–5秒均值左103.76RPM/右97.80RPM，末次100.86/95.94RPM，输出约30.92%/35.06%，仍未等速。用户目视确认明显不同速，并要求直接使用编码器计数。
- 根据完整采样记录拟合局部电机响应，当前PI响应过慢。正在将Kp从0.001提高到0.002，Ki从0.0012提高到0.008，并开放独立的参数符号方便后续空闲调参。该拟合只用于选择下一次试验参数。用户确认动力再次断开、USB保留，待新构建验证与上板。
- PI更新版已烧录，`logs/verified_motor_speed_loop_v2_20261119.json`。两次完整90RPM实测 `speed_post_90_20261119_152935_536_20261119_152942_502251.json`、`speed_post_90_20261119_152958_791_20261119_153005_731165.json` 及各自 `_speed.json` 保存100点计数/转速/PWM轨迹。3–5秒平均分别为左89.735/右89.912RPM和左89.337/右89.691RPM，平均差约0.20%/0.39%，逐点绝对差均值约0.57%/0.44%。左右平均PWM约28.6%/34.1%，两次均运行5000ms、speed fault0、PWM归零。数值依赖现用的每圈计数，需要重新标定后复核。
- 用户确认两轮上缘均朝车头方向。用户随后要求重验每轮10圈计数，继续第19节。

## 19. 供电状态下重新核对左右CPR

- 用户怀疑此前10圈计数，要求左右轮重新验证。使用目前已接入的TIM8左轮、TIM3右轮硬件A/B四倍频，保持编码器正常供电、四路PWM0、无待执行电机请求。每轮对齐固定标记后单独手转准确10整圈，重复测量比较一致性，16位计数差按回绕处理。
- `logs/cpr_recheck_safe_20261119_153147_183507.json` 回读当前39,712有效HEX字节一致、request0、PWM全0、MCU fault0。
- 用户确认左轮标记已对齐、两轮静止，基准 `logs/cpr_left_baseline_20261119_153200_529865.json` 左23400、右23407。已请用户保持右轮不动，左轮沿车体前进方向准确10圈，等待完成回复。不要提前以历史计数当作本次终值。
- 左轮第一轮10圈终值 `logs/cpr_left_after_10_20261119_153252_949440.json` 左38976，增15576，得到1557.6计数/圈。右轮同时+1529，用户确认确实碰动或对齐过右轮，未将此变化判为电气干扰。已请左轮从当前标记沿同方向再转10圈，等待第二轮终值。四路PWM仍0。
- 15:42 接续时，同一 ST-LINK（本机 COM11）以 HOTPLUG 确认 PI v2 固件 39,712 个有效 HEX 字节一致，request0、PWM 全0、MCU fault0。上次 completed=2、两轮有累计计数，本次 state=0、completed=0、两轮计数均0，表明其间主板已重启，旧基准不能跨重启相减。日志为 `logs/cpr_resume_identity_20261119_154256_740934.json` 和 `logs/cpr_resume_counts_20261119_154257_223325.json`。原左轮 1557.6 计数/圈仍是一笔已完成测量；待确认编码器供电、车架架稳，并对齐左轮标记后读取新基准，再手转10圈复测。
- 15:44 用户确认左轮标记已对齐。HOTPLUG 再次核对 PI v2 固件一致，request0、state0、completed0、MCU fault0。对齐后的新基准为左轮 TIM8=0、右轮 TIM3=0，四路 PWM=0，两个定时器均在 A/B 四倍频模式。见 `logs/cpr_left_rebaseline_20261119_154424_570645.json`。待用户保持供电和右轮静止，沿车体前进方向手转左轮准确10圈并停在同一标记，随后读取终值。
- 15:46 用户完成手转并说明圈数不确定。终值 `logs/cpr_left_repeat_after_10_20261119_154551_084696.json` 左17138、右0，静止复读 `logs/cpr_left_repeat_stationary_20261119_154616_665341.json` 完全一致，PWM全0。若按10圈是1713.8计数/圈，比第一次1557.6高10.03%；若按11圈是1558.0计数/圈，仅比首次每圈值高0.4个计数。实际圈数未确认，这笔不进入标定平均值。若左轮仍停在同一固定标记，左17138、右0作为新起点，进行两段各5整圈的复测，每段停在标记处读取，约预期7788计数/段。
- 15:47 左轮第一段5圈，`logs/cpr_left_5_segment1_20261119_154748_350295.json` 左17138→24939，增7801，得1560.2计数/圈，较首次1557.6高0.17%。右轮仍0、PWM全0。下段以左24939、右0为基准，左轮再正转5圈；为减少往返，若可以在右轮当前位置设固定参照且无需挪动右轮对齐，可随后单独正转右轮5圈，两轮停稳后合并读取。
- 15:49 用户分别手转左、右轮各5圈。`logs/cpr_left_segment2_right_segment1_20261119_154934_018151.json` 左24939→32751，增7812，得1562.4计数/圈；右0→7834，增7834，得1566.8计数/圈，四路PWM为0。左轮两段相差0.14%，10圈合计15613，得1561.3计数/圈；与此前一笔准确10圈合计31189/20=1559.45计数/圈，可作为本次复核标定值。右轮下一段以7834为基准再单独正转5圈，左轮应保持32751。
- 15:50 右轮第二段5圈，`logs/cpr_right_5_segment2_20261119_155052_579890.json` 右7834→15616，增7782，得1556.4计数/圈，左轮保持32751，PWM全0。两段相差0.67%，10圈合计15616，右轮标定为1561.60计数/圈，左轮沿用前条1559.45。

## 20. PI v3 烧录与自动试转回报

- 用户确认电机动力断开、保留USB后，`Core/Src/w4_motor_speed_diag.c` 写入复核后的左右计数比例，构建 `build/motor_speed_loop_v3`。65个ARM源编译完成，已有轮速 Cortex-M4 回归通过；v2、v3二进制仅变更两个CPR浮点常量。PI参数Kp=0.002、Ki=0.008，50ms采样、40%上限和5秒测试参数沿用。
- 15:54 烧录验证并复位。烧录记录 `logs/flash_motor_speed_loop_v3_20261119_155450.txt`，manifest `logs/verified_motor_speed_loop_v3_20261119_155450.json`，回读 `logs/speed_loop_v3_identity_20261119_155515_481595.json` 显示39,712有效HEX字节差异0、request0、PWM0、MCU fault0。当前主机工具位于 D 盘 CubeIDE 插件目录，读取脚本在调用时传入本机 CLI 与 nm。现有 `run_speed_bench.ps1` 的默认路径仍为旧环境的 v2。
- 用户希望试转后自动回报两轮圈数并由闭环自行调整。两轮总圈数由试转起止四倍频计数差除以各轮CPR得出。控制器每50ms读取计数、分别调占空比，末两秒报告两轮平均 RPM 和 PWM。记录的是试转 PWM 停止前的圈数。
- 首次 v3 请求记录 `logs/speed_v3_post_90_20261119_155638_026639_20261119_155643_943962.json`，两轮计数0、两路命令40%、1秒缺反馈停止、PWM全0、MCU fault0。后来用户明确确认动力已接。
- 重测日志 `logs/speed_v3_post_90_20261119_155726_998620_20261119_155732_929171_report.json`，左轮0计数（40%输出），右轮1556计数（0.9964圈，1秒平均59.78RPM），右轮末次输出39.93%。1秒时因左轮缺反馈提前停，四路PWM归零，speed fault2、MCU fault0。未取得稳态等速数据。先区分左轮实际未启动与转动但编码器失去反馈，等待用户目视确认。

## 21. 左轮起步补偿

- v4 在速度试转首50ms后，对尚未达到5RPM的左轮暂时输出45%，达到5RPM或500ms时回到最多40%的 PI，右侧沿用原参数。起步及停机回归通过。16:06 已烧录，`logs/verified_motor_speed_loop_v4_20261119_160654.json`；`logs/speed_loop_v4_identity_20261119_160656_264282.json` 回读39,808有效HEX字节差异0、PWM0。
- 用户确认动力接回后，`logs/speed_v4_post_90_20261119_160829_391585_20261119_160835_331589_report.json` 左轮确实命令过45%但计数0，右轮1563计数、1.0009圈，1秒后缺反馈自动停，两轮PWM归零。
- 左轮手转约1圈后，`logs/cpr_v4_left_manual_after_fault_20261119_161014_638649.json` 左1591（增1591，约1.02圈），PWM全0，说明在此次静态手转条件下左编码器有反馈。
- v5 的45%/50%两段起步版已构建通过；用户随后指定60%，并怀疑原起步加强时间太短，因此直接转到 v6。v6 在首次50ms更新后给尚未起步的左轮60%，达到5RPM或1秒保护点结束加强；右轮及起步后PI保持40%上限。生产65源编译及左/右输出、反馈后降占空比、禁止重复加强、无反馈停止回归通过，`build/motor_speed_loop_v6` 待上板。

## 22. 架空闭环配平完成

- v6 左轮 60% 起步首次仍计数0，用户目视确认不转；复查左轮动力接线后，连续四次跑满5秒，无故障，左轮平均比右轮多约0.4圈。
- v7 累计圈数差反馈，起步前150ms沿用正常控制，左轮确实缺少反馈才使用限时60%加强。16:20烧录并回读39,984有效字节一致。三次90RPM完整架空试转停机圈差依次为 +0.00481、-0.00023、+0.00232圈，运行中最大绝对圈差0.098～0.166圈。两轮都有反馈，两路实际峰值约40%，四路PWM均自动归零。用户目视确认配平结果正确。报告为 `logs/speed_v7_post_90_*_report.json`。
- `run_speed_bench.ps1` 和 `read_motor_pulse.py` 支持 v7 自动回报起止计数及圈数。该命令仅用于刷入 v7 诊断固件的架空试转，运行前会核对片上固件。

## 23. 双轮整机固件与实时监控

- 共用已验证的 TIM8/TIM3 A/B 四倍频初始化，使用独立有符号轮速，按两轮位移平均更新位置，IMU 提供航向。控制层按导航请求的行程差进行配平，保留差速转弯、反向停稳和缺反馈停车。
- 先烧录 `build/integration_dual` 锁定版，核实 IMU 已校准、OLED 就绪、PC BLE 收到853帧且解析错误0。用户水平逆时针旋转约90°，转前、转后记录 `logs/integrated_yaw_before_live_20261119_163338_634747.json`、`logs/integrated_yaw_after_live_20261119_163430_902830.json`，IMU 增加94.50°，确认正方向。
- 16:35 烧录 `build/integration_dual_calibrated`，显式构建 `W4_CALIBRATED` 和 `W4_CAL_IMU_SIGN=1`。Flash 回读85,776个有效字节差异0，三项标定标志均1、空闲PWM及MCU故障码均0。见 `logs/verified_integration_dual_calibrated_20261119_163534_711991.json` 与同版本身份文件。双轮新增测试、历史导航和传感器等11项 Cortex-M4 回归通过。
- 首个实时窗口实测同时显示真实点云、航向和位姿，记录 `firmware_source/logs/live_20261119_163807_222.bin`。实机收到 `RESET_PENDING`、`STOP_LATCHED`。发现 Matplotlib 文本框失焦也会提交，窗口里一条编辑中的目标被固件以 `ROUTE_UNCONFIRMED` 拒绝。现修改为按回车或点 Send 才提交，PC 17 项回归通过。
- 新窗口日志 `firmware_source/logs/live_20261119_164424_376.bin`，启动诊断 `logs/integrated_monitor_20261119_164423_stderr.txt`。BLE有历史间歇性，窗口保留自动重连。不要同时从另一监控进程连接同一模块。
- 用户确认动力已接和架空，联动基准 `logs/integrated_send_baseline_live_20261119_164449_714960.json` 左右计数均0、PWM全0、IMU有效、雷达点云帧10910。等实时 BLE 恢复后在监控窗口验证 Send、轮子实际转动与 STOP。地面路线和压载终点精度尚待实测。

## 24. Send 驱动联动及前进方向修复

- 修复 PC 文本框失焦提交后，实时窗口第4次 BLE 订阅成功并持续显示点云、位姿、航向，日志 `firmware_source/logs/live_20261119_164424_376.bin`。
- 用户确认接回动力并架空后，在窗口 RESET，再通过 Send 发 `GOTO,0,1.4`。记录 A,4,OK,RESET_PENDING，A,5,OK,PATH_ACCEPTED，N,TRACKING，随后 STOP 返回 A,6,STOPPED,STOP_LATCHED。跟踪期间有272帧激光点；STOP 后实测四路PWM均0、故障码0。位姿里程计架空读数约(0.008,1.178)m，表示轮子推算值。
- 用户目视发现旋转方向相反。STOP 后回读 `logs/integrated_send_after_stop_live_20261119_164900_100634.json`，左59436（从0计为−6100）、右23333，支持净反向。对已保存位姿回放，旧导航在离5cm路点仅几毫米时，横向偏差会放大到过大的角度误差并要求反转内轮。
- 修复为沿当前路段前视15cm的等效航向，合理范围内按沿程位置推进中间点，并对前进差速限幅以保持两轮同方向，拐角仍允许运动前原地转向。回归覆盖近中间点正转、路点推进、单轮缺反馈与 STOP；全11项通过。`build/integration_dual_tracking` 已生成，等待电机动力断开后烧录，随后架空复测。默认构建仍保留标定锁。
- 用户目测左轮可能约快15%，需实测。此前整合实现给负向 PI 修正留下的余量过小，且每个短路点重置轮速控制。现由导航和架空诊断直接调用同一 `W4WheelSpeed_CommandDuty`（相同增益、抗积分饱和和40%常态上限），前向直线的配平状态跨同向路点保留，转弯/STOP 时重置。新构建 `build/integration_dual_matched` 及11项 Cortex-M4 回归、低速降 PWM 用例均通过。当前片上仍是上一整合版，等待电机动力断开后烧录。计划从同一实时窗口发 Send，并对编码器累计圈数、前进采样窗口与 PWM 同步回读。
- 17:02 用户确认动力断开后烧录 `build/integration_dual_matched`。`logs/verified_integration_dual_matched_20261119_170239_449365.json`、`logs/integrated_matched_identity_20261119_170242_561050.json` 记录86,328有效HEX字节差异0，三项标定标志均1，四路PWM与故障码为0。PC实时窗口收到新版的雷达点及已校准IMU帧，等待架空接回动力后点击 Send。
- 17:05 用 `integration_dual_matched` 架空实测窗口 RESET、Send。编码器均正向增加，左右分别2741、2250计数（1.758、1.441圈），11个跟踪采样中可见低速右轮停转超过1秒，触发故障5保护停车。四路PWM归零。日志 `logs/send_wheel_match_20261119_170509_989099.json` 及同名 JSONL。差速方向修正有效，旧低速配平未通过。
- 用户要求参考上次轮胎等速的动力。用 v7 三次90RPM架空稳态平均值作为左右前馈标定点，左27.2%及右32.0%；根据导航目标 RPM 换算，仍使用共用 PI。直线中间点沿用既有0.15m/s速度上限，末端按距离减速，低速检测到持续无反馈时短时重试，保留保护停车。17:09 在用户确认断开动力后烧录 `build/integration_dual_trim`，`logs/verified_integration_dual_trim_20261119_170944_018599.json`、`logs/integrated_trim_identity_20261119_170947_069670.json` 记录86,512有效字节回读差异0、标定三标志均1、PWM 0、故障0。
- 用户接回动力并保持架空后，实时窗口 RESET、Send `GOTO,0,1.4` 重复两次，二者均到最后一路点（27）且导航和主板故障码0；两路编码器全程采样未见反向，反向 PWM 采样0，停机四路 PWM 0。17:11 第一轮 `logs/send_wheel_match_20261119_171120_556731.json` 为左6.448圈/42.21 RPM、右6.422圈/42.04 RPM，左比右快0.4%。17:12 第二轮 `logs/send_wheel_match_20261119_171250_790071.json` 为左6.638圈/44.10 RPM、右6.233圈/41.41 RPM，全程均值差约6.5%。第二轮巡航请求左43.61/右40.60 RPM，实际左43.49/右41.23 RPM，导航在主动纠偏；里程计到点并非静态架空车体移动。原始样本为同名 JSONL。
- 监控器修复固件重新启动时旧 IMU 序列仍被使用的问题。检测到重启后同时重置 LiDAR、IMU、导航的序列状态并清除旧轨迹，允许校准后延迟出现的航向帧。PC 18 项回归通过，重开的实时窗口启动诊断 `logs/integrated_monitor_20261119_171519_stderr.txt`。
- 17:16 实时监控第5次蓝牙订阅成功，重新确认窗口显示 ARRIVED、故障0、实时雷达点云和 IMU 数据（两者已恢复更新）。新监控日志 `firmware_source/logs/live_20261119_171520_269.bin`。

## 25. 0.30 m/s 地面直线、车尾前置与转弯准备

- 用户要求巡航目标从 0.15 提至 0.30 m/s（68 mm 轮径约 84 RPM），优先地面实测直线。调整为加速斜坡、按路段剩余距离减速、拐角转准后再前进。左右 90° 转弯软件仿真通过。17:22 烧录 `build/integration_dual_030`，回读 86,860 有效字节一致，身份见 `logs/integrated_030_identity_20261119_172300_194560.json`。
- 17:24 原车头带载首次 `GOTO,0,1.4`，蓝牙里程计到点 (−0.013, 1.405) m、故障0，用时18.4秒，最大横偏4.1cm、航向偏差约28.5°，未稳定到 0.30 m/s。用户发现前部万向轮摆动，决定把原车尾设为新前方，保留原指令和目标速度。
- 17:33 上板 `build/integration_dual_rear_forward`，双轮输出反向且交换逻辑左右，编码器映射和各物理电机的前馈同步交换。无需新增倒车指令。12 项 Cortex-M4 回归通过。用户操作地面试验，RESET 后有一轮到点 (−0.021, 1.396) m、故障0，但仍有摆动；另一次过了普通路点后因横偏卡在路点索引。未确认换向本身改善直线。
- 雷达点云中断期间 N/I 帧仍到达。用户确认雷达插头意外断开，复接后主板雷达 DMA 回到 STREAM；`logs/rear_lidar_20261119_173846_968916.json` 两读 1.757 秒间新增 35 个 P 帧、7,284 个解析点，PWM 为零。PC 原始日志 `firmware_source/logs/live_20261119_171520_269.bin` 随后恢复 P 帧。用户确认右侧雷达云图 **+X 为当前车头方向**，左图为 RESET 后的世界坐标。
- 修复高速时直线来回纠偏和普通路点漏过问题：随速度增大前视距离，降低行驶中航向比例增益，提高微分阻尼，普通路点按沿程位置推进，真正拐角仍要求车靠近拐点。12 项 Cortex-M4 回归通过（含错过中间路点、车尾前置的左右转弯和停车）。17:39 烧录 `build/integration_dual_rear_line`，`logs/verified_integration_dual_rear_line_20261119_173939_376727.json`、`logs/integrated_rear_line_identity_20261119_173942_466718.json` 记录 87,212 字节一致。主板新版本已确认收到雷达帧，启动后 PWM=0。用户自行发指令，助手只读记录；地面转弯待直线通过后进行。
- 用户自行操作，助手只读观察。新版同一次启动内日志记录 `A,1,OK,RESET_PENDING`、`A,2,OK,PATH_ACCEPTED`。指令 2 的 `GOTO,0,1.4` 从主板 56,350 ms 开始跟踪，75,250 ms 显示 ARRIVED，379 条轨迹帧，故障0，里程计终点 (0.012, 1.448) m。0.5 秒窗口最高沿程速度约 0.244 m/s，途中最大横向偏差约 11.7 cm，靠近终点后最大航向偏差约 100°。普通路点未再次卡住，但终点纠偏幅度太大，未达到 0.30 m/s 稳定直线的验收条件。完整原始数据 `firmware_source/logs/live_20261119_171520_269.bin`，以最新启动的导航序列区分。继续遵照用户要求只读观察，地面转弯未验证。

## 26. 地面直线复测准备

- 用户再次确认雷达局部图的 +X 为当前车头，RESET 后左侧世界坐标图仍按 +Y 直行。用户怀疑上趟可能有墙，并观察到行进中持续偏移，要求复测并记录数据后再调占空比。
- 上趟在主板 56,350–75,250 ms 运行，约行进至 0.70 m 时右偏已达 11.7 cm，终点后出现较大转向。该区间发布的雷达点中，前方 450 mm、左右各 300 mm 的区域没有回波。遥测点稀疏，无法排除实际碰擦。当前固件把雷达点用于遥测，路径和转向控制没有读取雷达数据。
- 用户确认 USB 已接且电机独立动力已断开。两次只读枚举均显示 No ST-Link detected，串口列表未见原 COM11。未烧录、未修改控制参数。若要用 SWD 读取两轮目标速度、编码器速度和 PWM，须先恢复 ST-LINK 枚举；一次车体地面复测的既有 BLE 遥测可记录航向、位姿和雷达。
- 17:53 重新启动唯一的 Week4 实时窗口，未发送任何命令。原始日志为 `firmware_source/logs/live_20261119_175346_685.bin`，蓝牙重试记录为 `logs/rear_line_monitor_20261119_175345_873_stderr.txt`。复测由用户在空地自行发送 RESET 和一次 GOTO,0,1.4，助手观察记录。

## 27. 地面复测与轮速诊断版

- 17:55 蓝牙第 8 次订阅成功。用户自行操作 RESET 与 GOTO,0,1.4。原始日志 `firmware_source/logs/live_20261119_175346_685.bin`，ACK 3=RESET_PENDING、4=PATH_ACCEPTED。指令 4 从 964,300 ms 开始，983,150 ms 首次 ARRIVED，共 377 条 TRACKING，另有首条 ARRIVED，导航序列无缺失且 fault=0。最大横偏 +122 mm，约在 y=715 mm；终点里程计 (-1,1393) mm，最大航向偏差约 26°，0.5 秒窗口最高沿程速度 0.250 m/s。复测再次出现中途右偏，未复现首趟约 100° 的末端大转向。正前方最近采样点约 1.05 m。终点与偏差均为车载估计。
- 轮距 0.175 m，差速运动学下 v右−v左 ≈ 0.175×航向角速度。旧趟前 0.5 m 平均等效差约 −0.015 m/s（左快约 4.2 RPM），此次起步先向左后向右。此前蓝牙日志没有两轮的请求、实测和占空比，不能据航向单独定出应调几个百分点。
- 新建 `build/integration_dual_rear_wheel_diag`，保留 0.30 m/s、轮速 PI、双轮前馈、停车以及原 GOTO；跟踪期间以约 10 Hz 增发 `D,ms,WHEEL,cmd_id,req_l,req_r,target_l,target_r,meas_l,meas_r,duty_l,duty_r`。左右为当前车头的逻辑左右，速度为有符号 mm/s，占空比为有符号千分数。req 是导航请求，target 是行程差配平后的轮速目标，meas 是滤波后的编码器轮速。PC 兼容诊断行，右图 +X 车头标注在下次打开窗口时加载。65 个源文件构建成功，12 项 Cortex-M4 与 18 项 PC 回归通过。
- 18:00 用户重新接上 ST-LINK，确认电机动力已断开后烧录、校验并复位。独立 HOTPLUG 回读 87,676 个有效 HEX 字节差异 0，四路 PWM 0，导航 IDLE，MCU 故障 0，标定标志 [1,1,1]。见 `logs/integrated_rear_wheel_diag_identity_20261119_180049_616340.json`。最新日志末段 P/I/N 分别为 47/47/47 帧。末条导航时间 67400 ms，状态 IDLE。 用户停止了窗口操作，因此保留当前已打开的窗口。
- 待用户自行在空地进行同样的地面直线试跑，核对实车 WHEEL 帧后，将两轮实际速度残差和输出饱和情况与航向时间对齐，再决定是否调占空比。当前尚未获得诊断版带动力样本。先做现有 0.30 m/s 设定的对照。

## 28. 双轮地面遥测与终点换向修复

- 用户 18:03 自行发送 RESET 和 GOTO,0,1.4。命令 3 从主板 151,350 ms TRACKING 至 191,650 ms ARRIVED，用时 40.30 秒，含 806 条跟踪帧和 400 条双轮诊断。最大横偏 107 mm，0.5 秒窗口最高沿程速度 0.232 m/s，里程计终点 (38,1430) mm，最大航向偏差 120.8°。之后 STOP 返回 STOP_LATCHED，192,550 ms 导航 STOPPED。原始切片和汇总为 `logs/wheel_diag_ground_cmd3_20261119_1803.bin` 与同名 `.json`。
- 直线高速段按导航请求均值至少 0.27 m/s 选出 55 样本。逻辑左/右 PI 目标均值 0.226/0.336 m/s，编码器均值 0.199/0.191 m/s，平均 PWM 36.1%/40.0%；右轮 55/55 样本碰到 40% 常态上限，左轮 11/55。右轮跟不上导航需要的纠偏速度，直接调右轮前馈不能提供额外 PWM 余量。
- 用户目视车起步偏、调整后直行。单取 y=0.769–1.197 m、相对目标航向约 ±2° 的 19 个同刻样本，横坐标 104–107 mm，实际为偏出约 10.5 cm 后平行行驶。逻辑左/右编码器均速 0.221/0.223 m/s，PWM 均值 36.3%/40.0%。该配对可作为当前地面和负载下的巡航前馈参考，起步与电量/负载变化仍由双轮编码器闭环修正，右侧当前已无向上纠偏余量。
- 终点附近相邻可比样本出现 27 次原地转向指令翻向，21 次同时跨越估计的 50 mm 到点边界；用户确认车端一直抽搐。修改 `week4/Src/w4_control.c`，进入到点圈后保持最后路段航向，允许 15 mm 的转向滞回；若超退出边界或已对齐但位置仍在圈外，先停再恢复位置跟踪。到点仍要求 50 mm、5°、持续 200 ms。双轮 PI、前馈、PWM 限幅、0.30 m/s、WHEEL 记录及 GOTO 指令沿用。
- 同坐标边界复现以及全部 12 项 Cortex-M4 回归通过，整机 65 源文件构建。18:08 用户确认重新接好 USB 且动力断开后烧录 `build/integration_dual_rear_goal_settle`。独立 HOTPLUG 回读 87,900 有效字节差异 0，四路 PWM 0，导航 IDLE、MCU 故障码 0；身份文件为 `logs/integrated_rear_goal_settle_identity_20261119_180855_609976.json`。
- 现有蓝牙监控第 10 次订阅重新连上。启动初期只见 I/N，随后雷达 DMA 恢复 STREAM，SWD 读到 56,313 点、267 个 P 帧尝试；PC 日志后续 P/I/N 均在刷新。参见 `logs/rear_goal_settle_lidar_after_flash_20261119_1809.json`。新程序地面终点仍待复测。

## 29. 雷达恢复与终点修复版首趟地面测试

- 用户发现监控雷达云消失时，PC 仍连续收 I/N，最后一条 P 在主板 131,524 ms。18:12 的两次 HOTPLUG 相隔 1.321 秒，雷达 DMA 的接收块和点数均未增加，状态从 RETRY_DELAY 变为 WAIT_DESCRIPTOR，重试计数 41→42。故障位于雷达采集链路，电源、接头或雷达端具体原因未证实；同次启动直到 272,146 ms 已再次观测到新 P。用户确认点云恢复后开始测试。只读日志 `logs/rear_goal_settle_lidar_dropout_20261119_181253_468235.json`。
- 用户自行 RESET 并发 GOTO,0,1.4。命令 3 从 308,700 ms TRACKING 到 324,700 ms ARRIVED，16.0 秒，故障 0，里程计终点 (48,1397) mm，距目标约 48.1 mm。区间含 158 条 WHEEL、318 条雷达 P。终点段 86 个原地转向样本未观测到翻向，上一趟同口径为 27 次，时长由 40.3 秒降至 16.0 秒。真实轨迹归档 `logs/goal_settle_ground_cmd3_20261119_1814.bin` 与同名 `.json`。
- 起步约 4.5 秒，y≈0.776 m 时 x≈−93 mm，车头一度接近目标航向但车辆已处在目标线左侧。此后控制主动向右回线；y≈1.252 m 时 x≈−1 mm，但车头仍向右偏约 20.7°，随后越线到右侧约 48 mm。途中最大横偏 93 mm，0.5 秒窗口最快 0.250 m/s。优先对比分析起步配平与速度受限时的回线幅度。
- y<1.3 m、双轮导航请求均至少 0.08 m/s 的 67 条样本中，逻辑左 / 右分别有 57 / 30 条达到 40% PWM。上一趟高需求段主要是右轮饱和，本次主要是左轮。保持现有固件参数；单边增加固定 PWM 暂无重复证据。地面转弯尚待直线验证。

## 30. 0.50 m/s 巡航目标

- 用户要求将巡航加速到 0.50 m/s。双轮导航巡航上限改为 0.50 m/s，保留现有加速和末段减速，单轮速度命令上限改为 0.59 m/s 留出差速转向空间。68 mm 轮胎的巡航与单轮上限约为 140.4 和 165.7 RPM。共享 PI 目标输入上限由 110 改为 180 RPM；原架空诊断入口仍限 110 RPM。
- 常态 PWM 上限仍为 40%，原短时起步、停车和终点换向保护沿用。上一趟在 40% 下实测最高约 0.25 m/s；0.50 m/s 是本次目标，尚无带载数据证明能达到。
- 65 源文件构建成功，12 项 Cortex-M4 回归通过。新增直线路径回归验证两轮确实请求 0.50 m/s、输出到现有 PWM 上限且 STOP 后归零。
- 构建 `build/integration_dual_rear_050`，HEX SHA256 `91a63ff0a007c45e1dbe4dce51554d331bae8a23357ded9fd0f0599c1f548f19`，ELF SHA256 `29592c247c151bbc52728ad9dfef76d4122708d017c9a47e7bb4f30363fff61b`。ST-LINK 已识别。

- 18:22 用户确认电机独立动力断开后烧录、校验并复位。HOTPLUG 回读 87,900 有效字节差异 0，四路 PWM 0、导航 IDLE、MCU 故障码 0。读取运行配置确认巡航 0.50 m/s、单轮上限 0.59 m/s、常态 PWM 40%。身份文件 `logs/integrated_rear_050_identity_20261119_182215_833319.json`。
- 现有蓝牙监控已收到本次新启动的 P/I/N，助手未发运动指令。尚未带动力复测，未验证之前不要将目标值当作实测车速。

## 31. 保持轮速目标的定长直行

- 用户明确 `GOTO,0,1` 表示沿发令时的车头方向直接走 1 米。双轮版的 `GOTO,0,d` 支持 0–2 m，从当前位姿起算；非零 X 的现有预设坐标路线继续使用原路径逻辑。
- 定长命令整个行程给左右轮相同的 0.50 m/s 目标。关闭导航航向调整、加减速阶段、轮间行程配平和终点回正；保留各轮编码器 PI，使占空比可以为保持同一目标而变化。里程按左右有符号编码器行程取平均，从命令开始重新累计，达到请求距离的控制周期输出零 PWM 和 ARRIVED。STOP、编码器缺反馈保护和传感器就绪检查沿用。
- 65 源文件整机 ARM 构建成功；12 项 Cortex-M4 回归及 18 项 PC 回归通过。新增验证非原点且非 +Y 起步、左右实测不同和中途航向干扰时目标不变、按计数到 1 米立即停机、再次发令独立累计及零距离保持停机。实际地面距离、滑行和速度待测试。
- 烧录文件 `firmware_source/build/integration_dual_rear_fixed_distance/Week4_Integrated.hex`，已离线验证 Intel HEX 记录，88,332 个有效字节。HEX SHA256 `e2dde6b407e8edc3d69a06e7fb2fb105e81c5d9950e14f3757b74bc42ba12f6f`，配套 ELF SHA256 `76c4d79bd9b59596e76b4199f3da246c8783c8769ef2029e889b8ff2cb5a6b9a`。编译使用 W4_DUAL_ENCODER、W4_REAR_FORWARD、W4_CALIBRATED、W4_CAL_IMU_SIGN=1、W4_IMU_DIAGNOSTICS、W4_WHEEL_DIAGNOSTICS。构建完成后用户改为请求助手烧录。

- 19:09 用户接好 ST-LINK 并确认电机动力断开，助手烧录、校验并复位。独立 HOTPLUG 比对 88,332 个有效 HEX 字节差异 0，四路 PWM 0、导航 IDLE、主板故障码 0，运行配置巡航 0.50 m/s、单轮上限 0.59 m/s、常态 PWM 上限 40%。烧录身份 `logs/integrated_rear_fixed_distance_identity_20261119_190932_227309.json`。实时日志末段 P/I/N 为 0/69/69，末条导航为 `N,869,43500,0,IDLE,0,0,1584,0,0,0,0`。 助手未发送运动指令。

## 32. 定长直行的首趟轮速测量

- 用户自行试车。命令 3 的投影目标约为 (3,1400) mm，实际发的是 1.4 m 定长指令。主板 277,500 ms 开始跟踪，284,300 ms 因缺轮速反馈进入 FAULT 5，随后 STOP_LATCHED。共采 68 条轮速诊断，左右请求和 PI 目标始终都是 0.50 m/s。有效分析区间取起步后第 1–4 秒，避开后段轮速掉零。原始切片和汇总为 `logs/fixed_distance_ground_cmd3_20261119_1914.bin` 与同名 `.json`。
- 轮距 0.175 m，差速车几何公式 s右−s左 = 0.175×Δθ（弧度）。第 1–4 秒航向右转 46.35°，估算左轮多走 0.1416 m，平均比右轮快 0.0472 m/s，约 13.25 RPM。编码器独立均值左 0.2543、右 0.2094 m/s，相差 0.0448 m/s，约 12.59 RPM；此段两轮 PWM 均 40%。
- 理想直行条件是左右轮行程和线速度一致，航向变化为零。按本次区间中心线平均速度，两轮都需约 0.232 m/s，但右轮在该工况已达 40%。在不提高输出上限的前提下，下一轮可以约 0.20 m/s（56.2 RPM/轮）作为保守的共同速度校验点。68 mm 轮径直行 1 m 时两轮各约 4.681 圈，对应逻辑左 7310、逻辑右 7300 个四倍频计数。几何关系无法从单一 PWM 工作点直接反推出所需的新占空比。
- 新版雷达在试验前已恢复 P 帧。本轮仅测量和计算，未更改定长参数或烧录。后段缺反馈原因尚未独立确认，先验证传动、轮胎及编码器反馈，再使用新试跑做配平。

## 33. 0.20 m/s 定长直行配平验证

- 只将双轮定长直行的共同速度目标设为 0.20 m/s（约 56.2 RPM），全程固定目标、关闭航向纠偏。独立轮速 PI 仍通过调占空比维持同速。坐标路线限速 0.50 m/s，常态 PWM 40%。12 项 Cortex-M4 回归通过，包括 1 米编码器停车及模拟快侧 PWM 低于慢侧。
- 19:22 用户确认动力断开后烧录并复位。固件 `firmware_source/build/integration_dual_rear_fixed_020/Week4_Integrated.hex`，HOTPLUG 比对 88,332 有效字节差异 0，四路 PWM 0、MCU 故障 0，身份 `logs/integrated_rear_fixed_020_identity_20261119_192216_537748.json`。窗口蓝牙已重新连接，可见雷达、航向和导航。
- 用户授权助手发测试命令并记录；随后确认双轮架空、动力已接。预定先做一次 `GOTO,0,0.3` 等效架空短试，确认反馈后再依次做三次地面 1 m 试跑。监控输入框拒绝了带无效字符的文本，未把该文本发到主板。
- 用户手动替换输入并已发送。最新启动中的命令 1–7 均为 0.30 m 等效架空行程，1.40–1.45 秒到距停车，状态 ARRIVED、故障 0。99 条有效 WHEEL 样本的双轮请求和 PI 目标均为 0.20 m/s，七次两轮均有编码器反馈。地面横偏待测。原始切片和各趟均值记录为 `logs/fixed_020_bench_7runs_20261119.bin/.json`。

### 0.20 m/s 地面 1 m 重复试验

- 实时窗口中用户自行发送地面命令，已归档 10 趟 1 m 的 WHEEL / N / I 原始片段以及汇总，见 `logs/fixed_020_ground_1m_20261119.bin/.json`。
- 各趟归零 PWM 时的里程计横偏 / 转角（左为正）：13号 -758.1 mm / -90.53°、14号 -192.8 mm / -11.52°、15号 +4.2 mm / +1.09°、16号 +35.6 mm / +3.78°、17号 -98.8 mm / -8.25°、18号 -53.5 mm / -4.81°、20号 +18.8 mm / +1.43°、21号 -11.3 mm / -1.15°、24号 -50.2 mm / -4.64°、26号 -93.2 mm / -4.98°。万向轮异常在第17条之前由用户报告并调整。里程计满足 30 mm / 2° 的命令号为 [15, 20, 21]；卷尺数据独立记录。
- 最近第 26 趟第 1–4 秒两轮平均 204.97 / 196.87 mm/s。实物观测：第26趟：用户用尺实测车尾中央万向轮横向偏移 12.5 cm；车载估算同趟停稳后相对起步方向右偏约 9.6 cm，参考点不同。第24趟用户目视轻微右偏。

## 34. 按带载数据计算的 0.18 m/s 直行配平

- 第 26 趟 1–4 秒逻辑左轮 0.20497 m/s、36.20% PWM；右轮 0.19687 m/s、38.93% PWM。左轮快 8.10 mm/s，约 2.28 RPM；整趟航向右偏 4.98°，按 0.175 m 轮距估算左轮多走 15.2 mm。用户尺测车尾中央万向轮横偏 12.5 cm。
- 定长直行的两侧共同目标选为 0.18 m/s（50.6 RPM），为慢侧保留 PWM 余量。车尾前置版用该趟 PWM ÷ 实测速率给各轮做局部线性前馈估算，对应目标速度的逻辑左轮 `31.8%`、右轮 `35.6%`。估算只用作各轮编码器 PI 的基础占空比，目标速度在全段固定，仍无航向转向或轮间行程配平。GOTO,0,1 按双轮平均 1 m 即归零输出，STOP、传感器就绪门槛和缺反馈停车均沿用。坐标路线的旧前馈保留。
- 构建 `firmware_source/build/integration_dual_rear_fixed_018_trim`，65 源文件成功，12 项 Cortex-M4 回归通过。19:44 用户确认动力断开后烧录并复位，独立 HOTPLUG 回读 88,372 有效字节差异 0、四路 PWM 0、MCU 故障 0。身份文件 `logs/integrated_rear_fixed_018_trim_identity_20261119_194429_488911.json`。
- PC 收到新启动的雷达和导航输出。最初 IMU 样本持续正常完成，静止标定进度 0。HOTPLUG 五秒窗 X 向静态加速度均值 0.265 g，超过水平阈值 0.25 g。读数见 `logs/fixed_018_trim_startup_imu_20261119.json`。已请用户将车和 IMU 模块在平地水平静置，恢复后由用户验证地面 1 m。未发送运动指令。
- 用户已将车平放。新固件 S 帧在主板 179,000–180,000 ms 的 calibrated=1，I 帧恢复，180,700 ms 导航 IDLE / fault 0。已具备从监控发 GOTO,0,1 的传感器条件，等待用户验证实际横偏。

## 35. 重新配平后的 1 m 卷尺复测

- 用户自行发送 GOTO,0,1，配平版命令 8 从主板 351,850 ms 开始到 357,600 ms ARRIVED，5.75 秒、故障码 0；58 条 WHEEL 帧的双轮请求和 PI 目标均为 0.18 m/s。
- 停机时里程计沿起步方向约 1.000 m，右横偏 31.2 mm、车头右偏 3.50°。按轮距估计左轮累计多走 10.7 mm。起步后 1–4 秒逻辑左右编码器均速 187.94 / 183.55 mm/s，平均 PWM 34.07% / 35.23%。车载和车尾尺测的参考点不同。
- 用户报告这次车尾中央万向轮横偏 4.2 cm。上一版末次同参照实测是 12.5 cm，这一趟减少 8.3 cm（66.4%）；仍高于拟定的 3 cm 横偏目标，尚未验证连续三趟达标。保存帧和指标为 `logs/fixed_018_trim_ground_cmd8_20261119.bin/.json`。本次仅记录。

## 36. 0.18 m/s 左轮前馈微调

- 第35节中第8趟的 1–4 秒逻辑左/右实速 0.18794 / 0.18355 m/s，左侧快 4.39 mm/s，左侧平均 PWM 34.07%。用该工作点的局部比例估算左轮差值，34.07% × 0.00439 / 0.18794 ≈ 0.80 个百分点。车尾前置定长支路的左轮基础占空比从 31.79% 下调到 30.99%，右轮基础占空比仍为 35.59%。这个比例是下一次实测的调参起点。
- 两侧目标全程固定 0.18 m/s，独立编码器 PI、常态上限 40%、无中途航向纠偏及满距立即停车保持原行为。12 项 Cortex-M4 软件回归通过，整机 ARM 65 源文件构建通过。新版和上一片上固件比对只有左轮前馈浮点系数内 3 个字节不同。
- 19:55 用户确认动力断开后烧录、校验并复位。HOTPLUG 88,372 有效字节差异 0，四路 PWM 0、导航空闲、MCU 故障码 0，片上基础 PWM 读回 30.99% / 35.59%。身份文件 `logs/integrated_rear_fixed_018_trim_v2_identity_20261119_195513_252566.json`。
- 监控已经收到新版连续雷达点云、航向和位姿。主板 12,000 ms 的 S 帧显示 IMU calibrated=1，导航 IDLE/fault 0。助手未发送运动指令。本次微调还没做地面尺测，验证命令 `GOTO,0,1`。

## 37. 左轮前馈微调版的一米复测

- 最新启动命令 3，用户自行发送 GOTO,0,1。126,650 ms 开始跟踪，132,450 ms 到距停车，5.80 秒，故障码 0。57 条 WHEEL 帧的双轮请求和 PI 目标都是 0.18 m/s。两轮 1–4 秒编码器实速均值 186.93 / 186.13 mm/s，左轮快 0.80 mm/s，上版同口径是 4.39 mm/s。实测 PWM 均值 34.32% / 34.39%，与代码中的基础占空比是不同量。
- PWM 归零时里程计右横偏 5.2 mm，车头右偏 0.57°；500 ms 后里程计沿程 1.041 m、右横偏 6.2 mm、右偏 0.86°。这些数字使用车载参考点，地面尺测另记。暂未收到用户对本趟万向轮横偏的尺测值。
- 原始帧和计算见 `logs/fixed_018_trim_v2_ground_cmd3_20261119.bin/.json`。此轮只记录，未发试车指令、改动控制参数或烧录。

### 同一固件重复发送的对照

- 同一开机、同一 0.18 m/s 微调固件，记录 7 趟定长 1 m。WHEEL 每一帧的左右请求和 PI 目标都为 180 mm/s。按各自起步车头方向投影的里程计到距横偏 / 偏角（左为正）：3号 -5.2 mm / -0.57°、5号 -67.2 mm / -3.84°、7号 +31.8 mm / -21.26°、10号 +153.6 mm / +6.36°、11号 +26.5 mm / +3.67°、14号 -134.2 mm / -8.71°、17号 +57.8 mm / +6.13°。原始帧和均值见 `logs/fixed_018_trim_v2_ground_repeat_20261119.bin/.json`。
- 用户确认第一趟实际走得很直。第一趟前 1–4 秒左比右快 0.80 mm/s；第10趟右比左快 7.80 mm/s，第14趟左比右快 8.93 mm/s，左右偏向随实速偏差反转。第7趟距到点约 0.3 秒时检测到约 14.6° 的航向变化，本次没有外部观察来区分万向轮、接触或移动。偏差原因还需用同一地段、起点及万向轮初始方向复测。未改固件。

## 38. 起步校正与 0.40 m/s 直角转弯

- 双轮版前进速度目标上限为 0.40 m/s。直行指令 GOTO,0,1 起步时以 RESET 的车头朝向为基准，存在超过 3° 的误差时先低速原地对正。直行开始后，两轮全程同一个 0.40 m/s 目标，不用航向或轮间行程纠偏；转向阶段的编码器计数不计入 1 m。前进实速和横偏待带载测量。
- 坐标路线 GOTO,-0.7,0.7 或 GOTO,0.7,0.7 从原点沿 +Y 前进至 (0,0.7)。进入拐点 15 mm 范围内归零 PWM，等双轮低于 0.02 m/s 至少 100 ms，并确认车仍在拐点 25 mm 内，然后原地转向下一段，3° 内停稳至少 100 ms 再继续；左、右路线各转 90°。越过拐点仍未进入范围时停机，导航故障 7；对齐超过 6 s 停机，故障 8。几何拐点基于车载位姿，实物墙和摆放由现场核对。
- 保留常态 PWM 40%、STOP 和缺反馈停车。0.18 m/s 实测得到的前馈在该标定速度截顶，0.40 m/s 时由轮速 PI 计算余量。已有带载峰值约 0.25 m/s，0.40 m/s 是目标。原地转向使用低速差速，第二段行进到目标保留坐标路线的减速和停机逻辑。
- 65 源文件整机 ARM 构建通过，12 项 Cortex-M4 回归通过，覆盖起步原地对齐、距离清零、定长期间双轮 0.40 m/s、STOP、拐点停稳和左右两种 90° 路线。构建 `firmware_source/build/integration_dual_rear_040_corner`，89,044 个有效 HEX 字节。HEX SHA256 `3170b6382ad00859f545a3435977c5a2b7eeae18a511d35c5f068b721dcc5333`，ELF SHA256 `ca305c5b99a7303cb86d46f2834be2d151bd9140f63a26507ba9bd4c5a92df2b`。等待用户排线和断开电机动力确认。

## 39. 0.40 m/s 定长直行温和纠偏

- 用户确认仍在排线，要求先增加小幅直行纠偏，避免纠偏时明显减速或来回摆。起步对齐 RESET 朝向后，从真正开始向前时的位姿建立直线，结合车载横偏和航向修正。横偏死区 5 mm，回线瞄准距离 0.60 m，回线角上限约 10°，航向死区约 0.8°。
- 两轮请求均值保持 0.40 m/s，单轮最多偏离均值 0.025 m/s，差速变化限制为每轮每秒 0.06 m/s。直行纠偏阶段车轮保持前转。两侧接近 PWM 上限时，为保留转向能力，慢侧最多让出 2 个百分点的占空比；带载速度仍需实测。直行到编码器里程即停；坐标路线仍按第 38 节在拐点停稳后原地转 90°。
- 默认打开直行纠偏，构建时定义 `W4_DISABLE_STRAIGHT_CORRECTION` 可关闭定长前进纠偏与相应 PWM 余量，保留起步对齐和直角转弯。第 38 节的无途中纠偏 HEX 也已保留。13 项 Cortex-M4 回归通过，包含启用/关闭、差速限幅与变化速率、PWM 余量、STOP、左右直角路线；整机 65 源文件 ARM 构建通过。
- 烧录文件 `firmware_source/build/integration_dual_rear_040_corner_gentle/Week4_Integrated.hex`，Intel HEX 校验有效数据 89,732 字节。HEX SHA256 `64227d62dfef66e6e66fb5bf04b3550a27242ec102d848fedfe4af312cad7f1a`，ELF SHA256 `c10fa3f27e9e70cea57d52cd9cafcfa19a66fd992df82225f86c09560d40bd3a`。用户完成排线后确认 USB 已接、独立电机动力已断开。
- 20:35 已烧录、校验并复位。HOTPLUG 比对 89,732 个有效字节差异 0，四路 PWM 均为 0、导航 IDLE、MCU 故障码 0。实机读出前进目标上限 0.40 m/s、单轮上限 0.49 m/s、常态 PWM 上限 40%。身份文件 `logs/integrated_rear_040_corner_gentle_identity_20261119_203558_491716.json`。监控已收到本次启动的 P/I/N，主板 11,000 ms 的 S 帧显示 IMU 已标定。未发送运动指令。

## 40. 截图中的跨趟连线与直行横偏

- 截图对上第39节固件同一开机的命令 1 和 3：1 m 一趟到距里程计 (-125,991) mm，1.5 m 一趟到距 (115,1499) mm，停止后的读数为 (115,1567) mm。中间命令 2 RESET。旧监控只在主板重启时清轨迹，将旧终点接回原点再画新轨迹。PC 改为每次 RESET 清轨迹，新 GOTO 仅画本趟，并保留原始日志。截图对应遥测为 `logs/gentle_040_ground_offset_20261119.bin/.json`。
- 真实横偏仍存在。日志显示前进 0.40 m/s 目标下，需加速的一侧顶到 40% PWM，而原限幅只让另一侧降到 38%。新固件在前进且接近 PWM 上限时，用两轮滤波编码器速度差和目标速度差做额外闭环；单轮 PWM 最多让出 7 个百分点，每秒最多变化 12 个百分点，前进目标平均值仍为 0.40 m/s。沿用起步对齐、差速目标限速、编码器到距停车，并保留直角路线拐点停稳后原地转 90°。带载速度和横偏需复测。
- 13 项 Cortex-M4 和 19 项 PC 回归通过，包括不等效双轮模型与 RESET 跨趟轨迹。20:44 用户确认独立电机动力断开后烧录 `integration_dual_rear_040_corner_balanced`，通过 89,940 字节独立回读，差异 0，PWM 全 0、导航空闲、MCU 故障 0。身份文件 `logs/integrated_rear_040_corner_balanced_identity_20261119_204436_549822.json`。
- 20:45 已关闭旧监控并重新启动，未发送运动命令。新版实时日志 `firmware_source/logs/live_20261119_204527_981.bin`，BLE 重连记录 `logs/balanced_040_monitor_20261119_204527_020_stderr.txt`。

## 41. 轮速差反馈版的 1.4 m 实车记录

- 用户自行测试，当前轮速差反馈版命令 2 于主板 64,250 ms 开始跟踪，70,750 ms ARRIVED，目标 1.4 m，故障 0。时长 6.5 秒；以该趟起始航向投影，最大横偏约 20.9 mm，到距右偏 2.7 mm，PWM 归零 500 ms 后左偏 5.6 mm、滑行约 69 mm。记录来源是车载里程计，未收到尺测。
- 本趟两轮平均目标 0.40 m/s，编码器测得两轮均速的峰值 0.260 m/s。档案 `logs/balanced_040_ground_cmd2_64250ms_20261119.json` 和同名前缀的 `.bin`。
- 用户随后准备地面转弯。当前版只接受预设正 Y 转弯路线，右转为 GOTO,0.7,0.7，左转为 GOTO,-0.7,0.7。GOTO,1,1 未列入路径。助手仅记录，未向车发运动命令。
- 实机日志中命令 6 目的地为 (1000,1000) mm，返回 A,6,REJECTED,ROUTE_UNCONFIRMED，导航为 IDLE；尚未执行该转弯。此前另有 RESET_AT_START 与 POSE_NOT_READY 拒绝。

## 42. 实车未转弯诊断及开放坐标

- 20:51–20:54 用户的预设右转命令 10、12 均接收成功并在原地转向前 FAULT 7。命令 10 在 (−98,666) mm 停，拐点前已左偏约 98 mm；命令 12 在 (1,728) mm 停，第一段横偏仅 1 mm，但滑出原 25 mm 停稳位置门槛。两趟正反轮速转向样本都为 0。记录 `logs/balanced_040_right_corner_fault7_20261119.bin/.json`。
- 进弯直线提前减速，双轮速度差反馈覆盖正向的坐标段。该版本先于动态坐标烧录并回读 89,988 字节差异 0，未做地面试跑。身份文件 `logs/integrated_rear_040_corner_braking_identity_20261119_205744_030653.json`。
- 用户随后要求取消 25 mm 拐点门槛并开放坐标。双轮版每轴可输 −2 至 +2 m。从当前位置沿 Y 到目标 Y，若还有 X 段，则在第一段沿程距分界线约 15 mm 时归零 PWM，双轮停稳 100 ms 后转向下一段，不再以停稳位置或横偏阻止转弯。进弯减速、正向轮速差反馈、转向超时、传感器和 STOP 保护继续生效。GOTO,0,d 的非负定长用法保持。单轴坐标路线只走一段。
- GOTO,1,1 从 RESET 原点预计先沿 +Y 到 1 m 附近，停稳、原地右转 90°，再沿 +X 向 1 m 前进；GOTO,-1,1 为相反转向。雷达仍显示点云，路径不做墙体避障。实车拐点位置和转向待验证。
- 通过 13 项 Cortex-M4 回归，包括离拐点横偏 120 mm、越线 40 mm 仍停稳再右转，以及任意负坐标和非原点开局的路点测试。21:02 上板 `integration_dual_rear_040_dynamic_corner`，独立校验 89,540 字节一致，四路 PWM 0，主板导航 IDLE、故障 0。身份文件 `logs/integrated_rear_040_dynamic_corner_identity_20261119_210224_750589.json`。现有监控 `firmware_source/logs/live_20261119_204527_981.bin` 已收到新启动的雷达、航向和空闲状态，IMU 标定完成。助手未发运动指令。

## 43. 动态坐标版地面试跑记录

- 用户操作，助手只读记录。两趟 1 m 直行均 ARRIVED，故障 0：命令 2 用时 5.05 s，到距 (-11,1001) mm；命令 4 用时 4.95 s，到距 (-19,999) mm，起点为 (0,1) mm。到距 500 ms 后沿 +Y 分别滑行约 66、65 mm。日志为 `logs/dynamic_040_ground_cmd2_151450ms_20261119.bin/.json` 和 `logs/dynamic_040_ground_cmd4_182100ms_20261119.bin/.json`。
- 命令 6 `GOTO,0.5,1` 已获主板 ACK；259,800 ms 起步，第一段最大里程计横偏 22 mm。265,500 ms 首个反向轮速请求时里程计 (6,999) mm，前序约 150 ms 内位姿变化在毫米级。到第二段前航向由约 84.9° 变为 1.9°，即右转约 83°，车头已对准 +X 方向约 1.9° 内。
- 第二段偏离 Y=1 m 的路线，控制多次进入原地纠偏。命令 6 在 279,250 ms 进入 FAULT 5，位姿 (520,907) mm，距目标约 95 mm。末段 278,500–279,200 ms 的 8 条轮速帧中，逻辑左轮反向 PWM 40% 而滤波轮速读数均 0，右轮仍有反馈。未核实左轮是实际停转还是反馈丢失。原始片段与统计为 `logs/dynamic_040_ground_cmd6_259800ms_20261119.bin/.json`。当前在板上仍为第42节固件。
- 命令 8 GOTO,1,1 已被接受，转向开始于 (20,997) mm，随后进入第二段。主板 306,150 ms 在 (270,982) mm 触发 FAULT 1。同步的有效 I 帧在 306,100 ms 后中断，IMU 读取失败累计 6→9，I²C 超时与恢复累计均 2→3。IMU 在 309,000 ms 已重新标定，导航故障保持到 RESET。故障由 IMU 数据中断触发，接线/供电等物理起因未验证。片段与汇总 `logs/dynamic_040_ground_cmd8_295650ms_20261119.bin/.json`，另存了故障后的传感器恢复帧。

## 44. 转弯后 0.18 m/s 复测版

- 复查第43节的原始 WHEEL/N 数据。命令 6 在 270,000–270,700 ms 第二段偏离过程中，右轮请求约 0.415 m/s、PWM 到 40%，滤波速度约 0.10–0.13 m/s；逻辑左轮滤波速度约 0.15–0.19 m/s。导航随后判定偏差过大，反复请求原地纠偏。命令 8 在第二段也短暂出现右轮更慢，实际停车由独立的 IMU 数据中断触发。
- 双轮坐标路线越过拐点并恢复前进后，双轮平均前进目标最高 0.18 m/s，回线前视距离下限 0.36 m，以留出带载轮速余量并减小低速时的转向突变。第一段、定长直行、单轴坐标、到点判定和 STOP/保护停车逻辑沿用。0.18 m/s 取自此前实物直行配平工作点。
- 13 项 Cortex-M4 回归通过，双轮左右转弯模型均到点停车，带 WHEEL 诊断的用例检查了转后平均请求上限；整机 ARM 65 源文件构建通过。日志 `codingday4_work/logs/cortex_dynamic_corner_exit_018_20261119.txt`。
- 用户本轮确认 USB 保持连接、独立电机动力已断开。21:16 核对旧固件的 89,540 个有效字节一致，随后烧录、校验并复位 `build/integration_dual_rear_040_dynamic_exit_018`。独立 HOTPLUG 读回 89,652 个有效字节差异 0，四路 PWM 均为 0，导航 IDLE、导航与 MCU 故障均为 0。烧录清单 `codingday4_work/logs/verified_integration_dual_rear_040_dynamic_exit_018_20261119_211631_906834.json`，读回 `codingday4_work/logs/integrated_rear_040_dynamic_exit_018_identity_20261119_211708_336154.json`。
- 保留原 BLE 窗口。从 `codingday4_work/firmware_source/logs/live_20261119_204527_981.bin` 确认新启动后日志持续写入，主板 74,000 ms 时 IMU 已标定、读取失败累计 0，雷达、航向与 IDLE 导航同时输出。此次没有发试车命令。
- 地面待验证。由用户在原试验条件下先执行 `RESET`，再用 `GOTO,0.5,1` 对照第43节的命令 6。记录转后实际轮速、是否再进原地纠偏、停车状态和终点误差。若出现 FAULT 5，区分实际停转与编码器掉反馈。若出现 FAULT 1，保留 I/S 与 I²C 数据并检查 IMU 接线及供电。

## 45. 0.18 m/s 版地面左转试跑

- 用户在 BLE 窗口操作。命令 4 `GOTO,-1,1` 获 `PATH_ACCEPTED`，主板 229,850 ms 开始跟踪。沿 +Y 行进时，车载里程计记录第一段最大横偏 2 mm。235,300 ms 在 (2,1000) mm 首次请求左转，两轮请求为 −88 / +88 mm/s。
- 终段 236,300–237,100 ms 的连续 9 条 WHEEL 帧，逻辑左轮（车尾前置对应原右轮、TIM3）PWM 均为反向 40%，滤波轮速均为 0 mm/s；逻辑右轮记录达到正向 50 mm/s。原地左转约 16.2° 后，237,200 ms 在 (−1,1020) mm 进入 FAULT 5，两轮输出由导航保护归零。停后 500 ms 位姿读数相同。故障发生于拐点转向，第二段 0.18 m/s 尚未得到地面验证。
- 该趟 IMU 读取失败累计、I²C 超时及恢复累计均为 0。直接触发条件是转向期间左轮缺少编码器反馈，需观察实物轮子以区分负载下停转或反馈链路异常。原始帧和汇总见 `codingday4_work/logs/dynamic_exit_018_ground_cmd4_229850ms_20261119.bin`、`codingday4_work/logs/dynamic_exit_018_ground_cmd4_229850ms_20261119.json`。
- 重复试验命令 6 同为 `GOTO,-1,1`，345,350 ms 开始跟踪，351,000 ms 在 (9,1003) mm 首次请求左转。转了约 19.0° 后，352,900 ms 在 (7,1028) mm 复现 FAULT 5。终段 352,000–352,800 ms 共有 9 条 WHEEL 帧，逻辑左轮仍为约 40% 反向 PWM、0 mm/s，右轮读数最高 58 mm/s。该趟内 IMU 失败累计保持 3，I²C 超时和恢复累计均保持 1，均为该趟开始前已有计数。原始帧和汇总见 `codingday4_work/logs/dynamic_exit_018_ground_cmd6_345350ms_20261119.bin`、`codingday4_work/logs/dynamic_exit_018_ground_cmd6_345350ms_20261119.json`。
- 用户随后目视确认，拐点原车右轮（现逻辑左轮）基本未转，只抖了一下。实物观察与缺反馈吻合，当前故障先定位到该轮在地面原地左转时未能持续起转。现有代码给原右轮的起步上限是 40%，该上限下两次均未持续转动。机械阻力、供电和驱动支路的具体原因仍待排查。本次只读记录。

## 46. 左转反向起步补偿

- 第45节两次原车右轮（逻辑左轮）在 40% PWM 下抖动后未持续转动。`week4/Src/w4_motor.c` 为车尾前置的左转反向起步添加补偿。已有换向停稳逻辑通过后，逻辑左轮约 150 ms 未建立有效同向轮速时，用 60% PWM 助起步一次，最长 250 ms。达到约 0.018 m/s 的同向滤波轮速或到时即结束，回到常态上限。一次连续左转中反馈再次中断也不会重复加力。
- 若车轮恢复，导航继续完成对齐和第二段；持续约 1 秒缺反馈时，由 FAULT 5 停车。STOP、IMU 保护、第一段控制以及过弯后 0.18 m/s 速度目标沿用。
- 13 项 Cortex-M4 回归通过，覆盖补偿时限、有效反馈提前退出、同一次转向不重复、保护停车及左右转路线。整机 ARM 65 源文件构建通过。测试日志 `codingday4_work/logs/cortex_dynamic_left_pivot_boost_20261119.txt`。
- 用户重新确认独立电机动力已断开、USB 保持。21:27 烧录并复位 `build/integration_dual_rear_040_dynamic_pivot_boost`，独立 HOTPLUG 回读 89,956 个有效字节差异 0，四路 PWM 0，导航 IDLE、导航及 MCU 故障 0。烧录清单 `codingday4_work/logs/verified_integration_dual_rear_040_dynamic_pivot_boost_20261119_212742_208169.json`，读回 `codingday4_work/logs/integrated_rear_040_dynamic_pivot_boost_identity_20261119_212742_208169.json`。
- 现有 BLE 窗口恢复新启动的 P/I/N，主板 26,000 ms 诊断显示 IMU 已标定，读取失败与 I²C 超时及恢复累计 0。待用户以同场地 `RESET` 后 `GOTO,-1,1` 验证，重点观察原车右轮是否实际起转、左转是否完成及第二段状态。助手只读记录。
- 补偿版左转复测 `GOTO,-1,1`，命令 2 于 47,400 ms 开始。52,950 ms 在 (−8,1002) mm 开始左转，53,450 ms WHEEL 帧记录原右轮（逻辑左轮）反向 60% PWM、−15 mm/s；下一帧在恢复 40% 上限内后读到 −48 mm/s。53,850–54,650 ms 连续 9 条采样中该轮又为 0 mm/s、反向 PWM 40%；54,700 ms 在 (−11,1018) mm 触发 FAULT 5，总转角约 20.4°，未进入第二段。该趟 IMU 失败、I²C 超时与恢复均为 0。原始帧及汇总 `codingday4_work/logs/dynamic_pivot_boost_ground_cmd2_47400ms_20261119.bin`、`codingday4_work/logs/dynamic_pivot_boost_ground_cmd2_47400ms_20261119.json`。用户确认左转未完成，随后准备右转对照。
- 右转对照 `GOTO,1,1`，命令 5 于 145,250 ms 开始，第一段最大车载横偏 3 mm。151,150 ms 在 (2,1004) mm 开始右转，转角约 91.9°。154,950 ms 从 (9,1000) mm 开始第二段，平均目标峰值 180 mm/s，轮速均值峰值 181.5 mm/s，偏离 Y=1000 mm 线最多 34 mm，第二段没有反向轮速请求。用户反馈右转动作可以。
- 右转路线在第二段并未到点。159,750–160,150 ms 逻辑右轮滤波轮速从约 167 mm/s 迅速降至 0，随后逻辑左轮也接近 0。161,050–161,750 ms 最后 8 条 WHEEL 帧原右轮（现逻辑左轮）正向约 40% PWM、轮速读数为 0，另一轮反馈间歇出现。161,800 ms 于 (722,1009) mm 触发 FAULT 5，距目标约 278 mm，停后 500 ms 位姿读数一致。该趟 IMU 失败、I²C 超时与恢复均为 0。此处两轮实际转动、地面接触或驱动状态未获实物证据。原始帧及汇总 `codingday4_work/logs/dynamic_pivot_boost_ground_cmd5_145250ms_20261119.bin`、`codingday4_work/logs/dynamic_pivot_boost_ground_cmd5_145250ms_20261119.json`。

## 47. 左转未达目标轮速保持占空比

- 第46节旧版在测得少量转动后退出 60% 助力。实车反转速度只有 48 mm/s、低于当时的 88 mm/s 目标，PWM 已降至约 23%，随后停转。用户要求优先修左转，并要求轮速不足时保持输出。
- 车尾前置左转的原右轮约 150 ms 未起转后，沿用 60% 起步补偿；启动助力后，以编码器同向滤波轮速达到当前目标为回落条件，回落速率每秒 65 个百分点。若之后掉到目标的 80% 以下，每秒最多回补 100 个百分点，上限 60%。到达转向目标、STOP 或故障时两轮归零，持续缺反馈约 1 秒自动停车。右转、直行和第二段速度设定沿用。
- 13 项 Cortex-M4 回归通过。复现 48 mm/s 对 88 mm/s 的轮速差并检查 500 ms 内助力保持，检查达到目标后的渐降、速度回落后回补以及缺反馈停车；带载模型中左转路线到点。整机 ARM 65 源文件构建通过。日志 `codingday4_work/logs/cortex_dynamic_left_pivot_hold_20261119.txt`。
- 用户重新确认电机动力已断开、USB 保持。21:35 烧录、校验并复位 `build/integration_dual_rear_040_dynamic_pivot_hold`。独立 HOTPLUG 89,972 个有效 HEX 字节差异 0，四路 PWM 0、导航 IDLE、MCU 及导航故障 0。烧录清单 `codingday4_work/logs/verified_integration_dual_rear_040_dynamic_pivot_hold_20261119_213534_105943.json`，读回 `codingday4_work/logs/integrated_rear_040_dynamic_pivot_hold_identity_20261119_213534_105943.json`。
- 现有 BLE 窗口持续记录。由用户在同一场地复测 `RESET` 后 `GOTO,-1,1`。助手只读检查起步 PWM、转角、第二段和到点状态。

## 48. 左转占空比保持版三次地面验证

- 用户在 BLE 窗口连续三次执行 `GOTO,-1,1`，助手只读记录。命令 2 从 173,650 ms 运行到 190,200 ms，ARRIVED / 0，停车位姿 (−990,1018) mm，车载离目标 20.6 mm，用时 16.55 s。第一段最大横偏约 130 mm，拐点开始时航向约 110°，原地段航向增加约 71.8°，随后进入第二段。停后 500 ms 位置读数一致。
- 命令 5 从 217,450 ms 运行到 233,050 ms，ARRIVED / 0，停车位置 (−992,1015) mm，车载离目标 17.0 mm，用时 15.60 s。第一段最大横偏约 10 mm，222,950 ms 在 (6,1000) mm 开始左转，转角约 92.3°。第二段平均轮速目标峰值 0.18 m/s，停后 500 ms 位置读数一致。用户确认第二次很成功。
- 两趟都观察到原车右轮（逻辑左轮）在转速未到 88 mm/s 时保持反向 60% PWM，每趟均有相邻两帧相隔 100 ms。第二趟在 223,550/223,650 ms 测得 9/76 mm/s，均为 60%；223,750 ms 测得 155 mm/s 时已降到 54.2%，下一帧降到 47.6%。第二趟转向期间该轮滤波读数峰值 210 mm/s。两趟运行窗口内 IMU 失败、I²C 超时和恢复计数均未增加。
- 原始帧与计算汇总为 `codingday4_work/logs/dynamic_pivot_hold_ground_cmd2_173650ms_20261119.bin`、`codingday4_work/logs/dynamic_pivot_hold_ground_cmd2_173650ms_20261119.json`、`codingday4_work/logs/dynamic_pivot_hold_ground_cmd5_217450ms_20261119.bin`、`codingday4_work/logs/dynamic_pivot_hold_ground_cmd5_217450ms_20261119.json`。坐标和横偏来自车载里程计。本次只记录，主板仍运行第47节固件。
- 第三趟命令 7 从 279,050 ms 运行到 294,250 ms，ARRIVED / 0，停车位姿 (−989,1005) mm，车载离目标 12.1 mm，用时 15.20 s。第一段最大横偏 31 mm，左转阶段航向增量约 99.5°，进入第二段时已对准 −X。第二段平均目标轮速峰值 0.18 m/s，停后 500 ms 位置读数一致。IMU 失败、I²C 超时和恢复计数在该趟均未增加。原始遥测和汇总 `codingday4_work/logs/dynamic_pivot_hold_ground_cmd7_279050ms_20261119.bin`、`codingday4_work/logs/dynamic_pivot_hold_ground_cmd7_279050ms_20261119.json`。

## 49. 负坐标路线的单次 IMU 漏读

- 用户截图对应命令 12 `GOTO,-1,-1`，主板接受路线，400,250 ms 从原点开始跟踪。411,200 ms 在 (−111,−938) mm、路点 18 进入 FAULT 1，尚处于第一段，距拐点的 Y 坐标差 62 mm。原始遥测及汇总 `codingday4_work/logs/dynamic_pivot_hold_ground_cmd12_400250ms_20261119.bin`、`codingday4_work/logs/dynamic_pivot_hold_ground_cmd12_400250ms_20261119.json`。
- 411,000–412,000 ms 两个 S 帧之间，IMU 读取失败累计 3→4，I²C 超时和恢复累计保持 1。50 ms 航向遥测仍在发布，符合采样中间一次失败、随即恢复的情况。旧版每次漏读立即将航向置无效，跟踪任务会锁定 FAULT 1。该次失败的物理起因未确认。
- 修订 `week4/Src/w4_imu_rtos.c`。已标定后第一次读取失败仅记录失败计数，用上一笔有效航向及其原始时间戳。导航仍检查 50 ms 数据新鲜度。连续第二次失败即标无效并由导航停车，连续第三次仍触发传感器重新初始化。开机标定阶段的失败仍会清除未完成窗口。
- 13 项 Cortex-M4 回归通过，覆盖单次失败后保留原时间戳并恢复新样本、连续失败置无效与重新标定、实际目标 `GOTO,-1,-1` 的转向与到点模型。整机 65 源文件构建通过。日志 `codingday4_work/logs/cortex_dynamic_imu_single_retry_20261119.txt`。
- 21:48 用户确认独立电机动力已断开后烧录并复位 `build/integration_dual_rear_040_dynamic_imu_single_retry`。独立读回 90,004 个有效字节差异 0，四路 PWM 0，导航 IDLE、导航和 MCU 故障 0。烧录清单 `codingday4_work/logs/verified_integration_dual_rear_040_dynamic_imu_single_retry_20261119_214813_934665.json`，读回 `codingday4_work/logs/integrated_rear_040_dynamic_imu_single_retry_identity_20261119_214813_934665.json`。
- 现有 BLE 已收到新启动的 P/I/N。主板 12,000 ms 时 IMU 标定完成，读取失败、I²C 超时和恢复累计均为 0。等待用户从同一起点 `RESET` 后复测 `GOTO,-1,-1`。助手只读记录。

## 50. 负坐标路线架空与地面对照

- 用户执行 `GOTO,-1,-1`。架空命令 2 和 4 分别于主板 61,350 ms 和 75,600 ms 开始，两轮反馈均正常，末帧逻辑左右轮速分别为 −87/+89 和 −87/+88 mm/s。车身航向变化约 0.1°，两趟在 6 秒对向超时后以 FAULT 8 停车，IMU 读取失败与 I²C 超时/恢复均为 0。原始帧与汇总为 `codingday4_work/logs/dynamic_imu_single_retry_bench_cmd2_61350ms_20261119.bin/.json` 和 `codingday4_work/logs/dynamic_imu_single_retry_bench_cmd4_75600ms_20261119.bin/.json`。
- 用户确认第三趟命令 6 已放地面，101,250 ms 起步，完成初始转向、第一段与拐点转弯，进入第二段。124,450 ms 于 (−999,−1016) mm、最后一个路点 39 做终点航向对齐时触发 FAULT 1，车载距目标 16.0 mm，航向约 174.3°。原始帧与汇总 `codingday4_work/logs/dynamic_imu_single_retry_ground_cmd6_101250ms_20261119.bin/.json`。
- 地面这趟最后有效航向帧在 124,400 ms。124,000–125,000 ms IMU 失败累计 0→3，I²C 超时和恢复均从 0→1，随后重新初始化和标定；128,000 ms 尚未标定完成。该次连续通信故障触发保留的停车保护，物理起因待查。此次实车测试后未改动固件。

## 51. 定长 0.18 m/s 停车调参

- 在第49节固件基础上增加车尾前置定长直行校准模式。上电默认关闭。BLE 发 CAL,0.3099,0.3559 写入逻辑左右轮基础占空比（小数），启用 0.18 m/s 共同速度。CAL? 查询，CAL,OFF 退出。ACK 内数值单位为万分之一。设置仅保存在 RAM，控制器须空闲且连续 20 个有效采样双编码器无增量。RESET 可在校准中重置起点。
- 65 源文件构建成功，13 项 Cortex-M4、19 项 PC 回归通过。用户确认动力断开后，向记录中的 STM32F446 烧录 build/integration_straight_cal/Week4_Integrated.hex，编程器校验成功。HOTPLUG 读出四路 PWM 为 0。
- 新版监控日志 codingday4_work/firmware_source/logs/live_20261119_221608_755.bin。BLE 重连后记录 A,1,OK,CAL_ON_L3099_R3559，N 帧 IDLE/0，持续收到 I/P。首趟基线待测。每次同一起点同一万向轮方向，RESET 完成后 GOTO,0,1，停稳后尺测车尾固定点横偏并记录方向。前三趟不改变参数。
- 基线第 1 趟命令 4：271650–277650 ms，ARRIVED/0。停车 500 ms 后 (15,1033) mm，航向 1.488 rad，以 RESET 正 Y 为基准右偏约 4.7°。1–4 秒左右实速均值 177.60/183.13 mm/s，PWM 35.93%/37.30%。片段 codingday4_work/logs/straight_cal_baseline_cmd4_271650ms.txt；卷尺值待补。
- 基线第 2 趟命令 7：329950–335950 ms，ARRIVED/0。用户尺测车尾固定点右偏 1.3 cm。停车 500 ms 后车载 (4,1036) mm、航向 1.611 rad，相对 +Y 左偏约 2.3°。1–4 秒左右实速 181.6/180.4 mm/s，实际 PWM 36.15%/38.56%。横偏达标，航向略超 2°。基础参数不变；见 codingday4_work/logs/straight_cal_baseline_cmd7_329950ms.txt。第 1 趟用户确认漏量。
- 第三趟 1 m 命令 10 到点/0，用户反馈“几乎无误差”，无精确尺测值。停车 500 ms 后车载 (-10,1038) mm，航向 1.562 rad。
- 2 m 直行命令 12，沿用左/右基础 30.99%/35.59%。502150–513650 ms，11.50 s 到点，故障 0。停机位置 (-11,2008) mm，500 ms 后 (-11,2034) mm，航向 1.582 rad（相对 RESET 方向左偏约 0.6°）。跟踪期间车载 x 在 -11 至 +2 mm；用户尺测车尾右偏 1.7 cm。起步 1 秒后至到点前 0.5 秒，两轮实速均值 181.42/181.09 mm/s，实际 PWM 均值 35.24%/35.36%。本趟两项均达标，2 m 重复性待验证。原始切片位于 codingday4_work/logs/straight_cal_cmd12_502150ms.txt。

## 52. 校准固件的 (1,1) 右转路线实测

- 用户发送 GOTO,1,1，命令 18。741200–758200 ms，历时 17.0 秒，ARRIVED，fault 0。终点及停车 500 ms 后均为 (994,1021) mm，车载位置离目标 21.8 mm，航向相对 +X 为 −4.35°。地面尺测另记。
- 第一段最大车载横偏 14 mm。747100 ms 第一条右转轮速请求时 (6,998) mm，航向 94.48°；750400 ms 首次恢复两轮正向请求时 (−5,983) mm，航向 −2.81°，两个采样点间右转 97.29°。第二段距 Y=1000 mm 线最大横偏 48 mm。
- 162 条 WHEEL 帧以及 N/I、ACK 和原始遥测已存 codingday4_work/logs/straight_cal_right_1x1_cmd18_741200ms.bin，分段实速/PWM、位姿和起止时间在同名 JSON。本次只记录。坐标路线使用坐标控制器，CAL 的 0.18 m/s 固定距离前馈仅作用于定长支路。

## 53. 拐点转向提速、出弯直线纠偏和监控调速

- 第52节命令18的右转从首帧反向轮速请求到双轮恢复正向约3.3秒。车尾前置版仅拐点原地对齐的大角度轮速目标上限由约0.088提高至0.136 m/s，并补偿高目标时的带载前馈；接近目标角度时降速。出弯沿规划轴线复用定长直行的航向/横偏纠偏、编码器PI和带载前馈，继续按坐标终点减速、收敛和停车。
- 监控窗口独立 Cruise (m/s) 滑块和 Apply speed 按钮支持0.08至0.40 m/s、步长0.01。点击后由主板 ACK 确认，影响定长和坐标路线的前进巡航请求。行驶中的巡航设定渐变，拐点原地对齐独立。速度仅存在RAM，上电恢复原路线默认策略，页面显示 Board 后才是已确认设置。
- 整机65源构建、13项 Cortex-M4 回归及19项PC回归通过；离线模拟鼠标拖动/点击后独立速度报文及ACK回显通过。确认电机动力断开后烧录并校验 build/integration_corner_cruise/Week4_Integrated.hex，SHA256 972D3A85D2F1A10186DF8C52A4CF3E5115F532608369109A04C33803EBFCC926。复位后 HOTPLUG 读四路PWM全0。新版监控日志 logs/live_20261119_223635_474.bin 中 A,1,OK,SPEED_MMPS_180，主板IDLE、故障0。待同场地 GOTO,1,1 复测转向时长、第二段横偏和车尾尺测。

## 54. 2026-11-26 迷宫导航集成

以第53节 firmware_source 为基线。截图确认 I2C1 PB8/PB9、蓝牙 PC10/PC11、C1 PA9/PA10。使用车尾前置双轮约定，逻辑左/右基础占空比 30.99%/35.59% 对应实测 0.18 m/s 定长工作点。保留独立编码器、方向、转向补偿和 STOP 机制。现版坐标路径沿 Y/X 依次行驶，尚未按雷达障碍规划。

### 离线集成与验证结果

- 查阅 NoeticMaze 的 planner_algo.c、icp_core.c 和 map_core.c。沿用本车 HAL、任务优先级、姿态估计、左右电机 PI 及原有 GOTO。新增 MAZE,x,y 使用绝对地图坐标，RESET 后车头朝世界 +Y。固件端消费蓝牙抽样之前的 C1 数据，以按角度分桶的实测回波建图，IMU 提供航向，点云匹配修正平移。按障碍膨胀后的地图运行四邻域 A*，只向控制器交付当前已观测可通行的短路段，停稳后重新规划。命令运行中发现路段堵塞先停车再规划。扫描过期报故障 9，无可行路段报故障 10。
- PC 监控支持 MAZE 命令和板载地图行，原 P/I/N/ACK 继续工作。工程自带虚拟环境原先引用已失效的 Python 安装，已改指本机同版本 3.12.14 运行时。更新 README_HARDWARE.md 和 README_PC.md，实车构建入口为 python -B tools/build.py --maze。
- 66 个源文件完成 ARM 编译、链接，生成 build/maze/Week4_Integrated.elf、.bin 和 .hex。text 95392、data 884、bss 88552 字节。构建警告来自原有未用裸机函数及 HAL 参数。HEX SHA256 为 5605A54592F0A5492BDB7C74BD8C7C3B413B7517409B682EBDA673BD98226465。
- 全部 14 项 Cortex-M4 仿真与 20 项 PC 回归通过。新增测试覆盖无扫描拒绝、障碍截断当前路段、绕行路点、STOP、RESET 清图、不同轮响应模型下连续分段抵达 0.65 m 目标，以及运动中断流后双轮 PWM 归零。另在项目虚拟环境使用 Agg 渲染监控窗口，核对地图行在带定位偏移时的绘图坐标。
- 删除 347 个 AppleDouble 元数据文件、5 个旧采集停止标记和两份旧日记快照；两份快照均与 ASTRA重制版.rar 中同名文件逐字节核对一致。保留主开发日记、轮速/方向标定、故障现场、实车遥测和烧录身份记录。
- 用户当前没有接入硬件。本轮仅离线构建和软件验证；烧录、板上读回、指示灯与电机、C1 实物左右角向及安装偏移、障碍膨胀半径与实际停车距离、地图定位精度和迷宫整车验收留待接入后进行。

## 55. 2026-11-27 硬件接入后的静态检查

- ST-LINK HOTPLUG 识别 STM32F446xx（ID 0x421），目标电压 3.23 V。TIM4/TIM12 的 CH1/CH2 比较寄存器均读出 0，MCU 故障码 0。Flash 回读与第 53 节 integration_corner_cruise 的 91,732 个有效字节完全一致，回读保存在 build/maze/board_readback_20261127.bin。
- HC-04 成功订阅，静态采集 384 帧，解析错误和序号缺失均为 0。收到 176 个有效 IMU 帧及 176 个导航帧，导航保持 IDLE、故障 0。IMU 已标定，采样期间失败计数保持 3。原始遥测位于 firmware_source/logs/hardware_idle_20261127.bin。
- 采集期间 P 帧为 0。两次只读检查均为 WAIT_DESCRIPTOR，累计描述符超时 16 次、坏描述符 3 次、解析点 0、DMA 已交付字节 0。当前阻塞位于 C1 扫描握手，需在现场确认雷达供电、转动和 PA9/PA10 RX/TX 连接。此次未发指令、复位或烧录；新版迷宫和电机运动尚待检验。

- 复测经 ST-LINK 确认仍为第 53 节固件。C1 已进入 STREAM，3.278 秒内解析点由 400665 增至 415028，四路 PWM 保持 0，MCU 故障码 0。蓝牙短采样收到 1 帧有效 P 点，但后续短时重连曾报告找不到设备，需观察前端实时连接。试验期间未烧录、复位或发运动指令。

## 56. 2026-11-27 迷宫新版烧录及静态复测

- HOTPLUG 写入时编程器校验在 0x08000110 报差异。改为 Under Reset + Hardware Reset 后重新写入同一 HEX，编程器校验通过并复位。独立 HOTPLUG 回读 96,276 个 HEX 有效字节全部一致，四路 PWM 为 0。烧录记录与回读保存在 firmware_source/build/maze。
- 新版上电后两次 SWD 快照相隔约 2.3 秒。IMU 发布序号由 1352 增至 1398，样本有效且已标定，MCU 故障为 0。C1 停在 WAIT_DESCRIPTOR，已有 6 次描述符超时，解析点和地图扫描时间戳均为 0。PC 被动连接两次均报告找不到原 HC-04 地址。此次未发送运动指令。
- 后续现场按顺序验证 C1 单独重新供电后的点云与地图、左右角向、C1 扫描中心到双轮轴中心的前向/左向偏移，最后测量整车转动包络和停车滑行。C1 偏移以车尾前置的当前车头定义，前向和左向为正。0.20 m 地图障碍膨胀为当前待测值。

- C1 与新版监控已恢复 P 点云与 M 地图遥测。用户多次提交 MAZE,0,1.3，监控原始记录 live_20261127_150436_718.bin 中，命令 4、6–16 全部回应 NO_CLEAR_PATH 并进入故障 10，未进入 TRACKING，里程坐标保持 (0,0)。这批命令停在规划阶段，未测试到轮子启动阈值。滚动地图在部分时段记录车头前约 0.30–0.40 m 的占用格，来源待现场核对。演示先使用无挡板、可见空地上的较近目标，再通过单挡板重新验证绕行。

## 57. 2026-11-27 近距弱回波假墙

- 原始蓝牙点云显示，主板 540 秒后的 31–37 cm 距离带共有 1,044 个回波，质量值均为 1–4。反复出现的前后近距占用格与该距离带相符。真实弱反射目标也可能落入该阈值，故仅在 31–39 cm 且质量不高于 4 时跳过建图；同距离更强回波正常进入地图。被丢弃的射线也不清除栅格。P 帧继续传原始点，用于和左图对照。
- 14 项 Cortex-M4 回归通过，包括同一角度先到的弱回波不得遮住后续有效射线，以及相同距离上真实较强障碍仍会阻止直行并触发绕行。66 源文件 ARM 构建通过。build/maze_echo_filter/Week4_Integrated.hex SHA256 BA669C3E56F1611F0DFF4CEEEE83AF63C968A4FF0B81046E3625C5212EF8A098。
- Under Reset 烧录校验通过，独立读回 96,324 个有效字节全部一致，PWM 四路为 0。原监控蓝牙重新接收 P 和 M。在用户报告前方 1 m 无墙的静态布置中，旧版图上的前方 0.3–0.4 m 近距灰格从四个降为零，新版启动约 37 秒内原始 P 帧仍有 50 个符合过滤条件的弱回波。复测中约 0.5 m 处曾出现另外的占用格，也有较强回波；其来源尚未确认。未发送运动命令。

## 58. 2026-11-27 近距墙体三圈确认与地图刷新

- 现场日志中的约 0.50 m 偶发回波质量可达 46–52，单靠提高质量下限会误伤真实目标。将低质量窄带过滤下限从 31 cm 扩至 30 cm，质量上限 4、距离上限 39 cm。1 m 内的其他端点同一格连续三圈命中才显示为占用。同一圈多个点命中只算一次。待确认格按未知显示，规划仍暂时避让。P 帧保留原始点。现有占用格靠后续射线穿过降低占用分值，无射线经过时保留，地图行约 4.9 s 刷新一轮。
- 14 项 Cortex-M4 回归和 66 源文件 ARM 构建通过。build/maze_confirmed_walls/Week4_Integrated.hex SHA256 6DF9C7F69270D58BD5BA32E39B03DA3BBD482B9D9F1136F20259E0495541E070。板上独立回读 96,484 个有效字节一致，四路 PWM 都为 0，未发送运动命令。
- 复位后静态采样时雷达、IMU 正常更新。两张 SWD 栅格快照相隔 5.288 s，81 格分值变化，9 个占用格退出占用状态，全图占用数从 93 到 90。车前 0.1–1.0 m、横向 ±0.30 m 的占用格由 0 到 4，说明持续回波仍可产生占用，有待用挡板拿开前后的带时间点云确认其物理来源。原监控日志当时无新数据，电脑端显示是否即时刷新未核实。

## 59. 2026-11-27 MAZE 绕行与前端朝向修复

- 继续处理用户要求的 MAZE 起步绕桩。原先 A* 先求完整路径、再从该路径回找可见路点，容易因第一段未知格或切角而拒绝有侧向出口的场景。现改为先枚举已观测自由的起步短段，再由这些起步段搜索目标，执行段仍检查栅格通行和对角切角。增加实际选中路点 MAZE_LEG 遥测。
- 新增完整动力学绕行仿真。包含两轮不同动力响应、编码器、陀螺仪、每圈雷达、挡板遮挡、最低离障间距及到点停车检查。模型在前方 0.55 m 的 0.40 m 宽挡板场景绕行并抵达 (0,1.05) m。14 项 Cortex-M4 软件回归已通过，当前属于仿真结果。
- 前端右图改为按车载航向转换到左图的世界坐标轴，两图均显示朝向线。IMU 或地图数据过期时隐藏旧墙并显示暂停原因，RESET 成功后清掉旧显示，指令框默认 MAZE,0,1.3。21 项 PC 回归通过，离线绘图核对零度回波在 +Y 朝向下出现在上方。
- 用户确认车静止在地面，挡板约 0.7 m，右侧可绕行。SWD 核对原三圈确认固件一致。静止 IMU 的 500 样本 Z 轴均值 -6.9215 度/秒，极值 -7.0305 至 -6.7939，方差约 0.00169，标定拒绝原因为移动。原绝对值 5 度/秒门槛阻止该传感器的静态标定。按当前实测将绝对值范围调整到 10 度/秒，继续要求完整 200 样本、采样间隔、水平姿态、波动范围和方差检查。此调整依赖实际静止条件，恒定慢速旋转仍可能与零偏混淆。
- 正在重新构建和上板验证，实车绕行完成情况将在下条记录补充。

## 60. 2026-11-27 换聊天交接总结（历史记录，最新见第 61 节）

### 当前目标与结论

用户要求发 MAZE 后小车能够起步、绕过单挡板并到目标，前端左右两图朝向一致且地图及时更新。用户最后要求总结日志后换聊天。当前已修复 MAZE 规划起步、IMU 静态标定门槛和前端朝向显示，实车接受过一次 MAZE 并前进约 0.394 m。完整实车绕桩尚未完成，后续聊天应从本节继续。

### 唯一活动工程与修改边界

项目允许修改的范围为 D:\EMBEGGed\week4codingday\coding。活动工程实际位于 D:\EMBEGGed\week4codingday\coding\coding(1)\coding\codingday4_work\firmware_source。主开发日记就是当前 README.md，位于 coding(1)\coding。继续更新此文件。下载、缓存、测试、日志等写入均放在指定项目内。上级有旧工程与 RAR，不能把它们当活动源码。电脑当前工作区的 C 盘路径不是本次编辑目标。

### 实车约定

- NUCLEO STM32F446，ST-LINK SN 0674FF545155657067032031。使用双驱动轮、车尾前置。逻辑左轮为原右轮，逻辑右轮为原左轮。
- 逻辑左 PWM PB6/PB7（TIM4），编码器 PA6/PA7（TIM3），1561.60 计数/圈。逻辑右 PWM PB14/PB15（TIM12），编码器 PC6/PC7（TIM8），1559.45 计数/圈。轮径 0.068 m，轮距 0.175 m。
- 0.18 m/s 的实测左右基础前馈为 30.99% / 35.59%。运行 PWM 还受编码器 PI、起步及转向补偿影响。默认 40% 常态上限和旧实测起步补偿继续存在，不能把左右轮改为同一个占空比。
- MPU6500 和 OLED 共用 I2C1 PB8/PB9，CN10-3/5。C1 USART1 PA9/PA10，CN10-21/33。HC-04 USART3 PC10/PC11，CN7-1/2。蓝牙地址 C4:25:01:17:00:02，UART 特征 0000ffe1-0000-1000-8000-00805f9b34fb。
- RESET 后世界前方 +Y，右侧 +X。C1 原始零度朝当前车头，板上按俯视顺时针角度转换。雷达到轮轴中心偏移默认 0，障碍膨胀半径 0.20 m，这两个几何设置尚未尺测确认。

### 最新已烧录版本

build/maze_detour/Week4_Integrated.hex，SHA256 CB3A11FB6C84B5A6ADDD86F8BEDED198406AC2E349F9E65C20DDB3FCC470ED2E。Under Reset + Hardware Reset 写入，编程器校验通过，独立 HOTPLUG 回读 97,292 个 HEX 有效字节全部一致，复位后四路 PWM 0。记录在该构建目录的 flash_20261127.txt、readback_20261127.bin 和 startup_20261127.txt。

更早的 maze_confirmed_walls 和 maze_echo_filter 是旧版，不能继续用它们的 ELF 地址解读新固件。此前 HOTPLUG 写入出现过 Flash 校验差异，已通过 Under Reset 重新写入解决。做 SWD 结构体读取前需先确认板上镜像和对应 ELF 一致。

### 当前代码改动

- w4_maze.c 先从已观测自由格中枚举可执行的短起步段，再由这些起步段搜索目标。最长路段 0.46 m，实际执行段必须已观测自由，并检查对角切角。长期路线可暂经未知格，后续停稳后重新规划。修正了旧版只从唯一完整路径回找可见点而拒绝侧向出口的问题。增加 D,ms,MAZE_LEG,cmd_id,x_mm,y_mm 遥测。
- w4_control.c 在领取首个 MAZE 路点时复查扫描是否过期。MAZE 速度上限 0.18 m/s，断流、传感器、堵转、STOP 保护仍生效。原 GOTO 没有雷达障碍停车，不能在挡板场景用它代替 MAZE。用户此前发 GOTO,0,1.3 撞到挡板，约 y=0.790 m 后触发堵转故障 5；这不是绕障成功。
- pc/protocol.py 的 radar_world_axes 按主板航向把右图原始点旋转到左图的世界轴，两图朝向一致，右图仍以雷达为原点。pc/monitor.py 默认命令为 MAZE,0,1.3，绿色虚线表示当前选中的路段，红线表示朝向。IMU 超过 0.8 秒或地图行超过 1 秒没有更新时隐藏旧墙并显示暂停原因。固件在扫描过期时停止发布旧地图行。
- w4_yaw_calibration.c 将静态 Z 轴绝对偏置门槛从 5 改到 10 度/秒。用户确认车静止时，500 个样本均值 -6.9215，范围 -7.0305 至 -6.7939，波动很小，旧门槛一直拒绝标定导致 I 帧中断、地图不更新和 RESET 被拒绝。仍要求 200 个连续静态样本、水平姿态、采样间隔、波动范围和方差检查。恒定慢速旋转可能与偏置混淆，启动标定仍需实物静止。
- 之前针对假墙的改动仍在。质量 <=4 且 0.30–0.39 m 的点跳过建图，不用这些射线清空空间，P 原始点仍保留。1 m 内同格连续三圈命中才画为墙，确认期间规划仍避开该格。此经验过滤可能丢失同范围真实弱反射目标。持续强假回波尚未完全定位，不能为了起步盲目扩大过滤距离。地图旧墙只有后续射线穿过时才降低占用概率。

### 验证结果及限制

14 项 Cortex-M4 软件回归已通过，其中 MAZE 新增完整闭环动力学绕行，模拟不同左右轮响应、编码器和陀螺仪、挡板遮挡与雷达扫描。模型绕过前方 0.55 m、宽 0.40 m 的挡板，到 (0,1.05) m 停车，检查最小离障 0.17 m。MAZE 用 O2 和 40 亿指令预算运行，之前较低预算耗尽不等于控制失败。21 项 PC 回归通过，离线绘图验证前向回波随 +Y 航向显示在图上方。最后 Z 轴标定调整后单独复跑 6 项传感器/标定测试通过，再构建 66 个 ARM 源文件。

注意工具在只运行部分 cases 的临时调用中仍打印固定的“14 项通过”，应按实际运行的 case 数描述，不可把该行当成 14 项重新全部验证。

### 本次真实 MAZE 测试

用户确认车静止在地面、挡板约 0.7 m、右侧可绕行。关闭旧窗口后只开一个新版监控。自动测试依次发送 STOP、RESET、SPEED,0.18、MAZE,0,1.3，均收到 ACK。MAZE 命令 4 返回 MAZE_ACCEPTED，第一路点为 (0,400) mm。

主板 89,400 ms 报 FAULT 1，里程计 (46,394) mm，航向 1.700 rad。最后有效 I 帧为 89,350 ms。89,100–90,100 ms 的 IMU 读取失败累计 24→30，I2C 恢复 8→10，标定状态 1→0。随后已发 STOP 并收到命令 5 的 STOP_LATCHED。完整原始数据和摘要分别在 logs/maze_detour_live_20261127_154624.bin、同名 .json。该日志之后仍追加监控数据，后续有主板重新启动帧，分析时按启动时间分段。JSON 的 terminal 是这次测试终点。

实车验证已证明 MAZE 能放行并驱动到第一段附近。故障来自传感器链路中断，完整绕桩尚未通过。I2C 超时/恢复计数在多次运行中增加，OLED 也出现重新初始化；确切电气或驱动根因待查，不能简单取消 IMU 保护来继续走。

### 最后一次失败试验已撤回

为减少中间点反复航向对齐，曾尝试中间路点只按位置及停稳交接。最新仿真在 tests/navigation/test_maze.c 第 70 行最小离障 0.17 m 断言失败，因此已完整撤回这段 w4_control.c 试验分支，恢复到已测试 maze_detour 的控制逻辑。该试验没有烧录，后续 ARM 构建也未执行。build/cortex_tests/navigation_maze.elf 可能仍是失败试验生成物，不能用于交付或解释正式固件。当前生产源码与 maze_detour 控制逻辑一致，后续修改必须重新跑绕桩间距测试。

### 前端和接手步骤

新版前端窗口标题 Week4 navigation and sensors。当前启动方式包含一个已结束的一次性测试线程，已输出 TEST_FINISHED，后续只持续监控。不要再新开多个监控争用同一个 BLE 连接。重启时关闭现有同名窗口后启动 .venv/Scripts/python.exe -B -X utf8 pc/monitor.py --ble C4:25:01:17:00:02 --log logs/新文件.bin。TEMP/TMP 指向工程 .runtime/tmp，MPLCONFIGDIR 指向 .runtime/mpl，TCL_LIBRARY/TK_LIBRARY 指向 .runtime/tcl/tcl8.6 与 tk8.6，MPLBACKEND=TkAgg。虚拟环境已修复为本机同版本 Python 3.12.14。

接手先读本节和最新测试 JSON，确认板上版本及静止时 IMU/I2C 稳定性。查清共享 I2C 中断原因后，再分析第一段到点附近的航向收敛与路点交接。继续用带挡板闭环模型验证最小间距，保持独立左右轮实测补偿。用户已授权本项目修改、烧录和受监控 MAZE 测试，但切换聊天后应确认当时场地摆放和通信连接仍有效。任何新的实车结果追加到本日志，最终目标仍是 MAZE 完整绕过挡板并到点停车。


## 61. 2026-11-27 绕桩转向抖动修复与烧录

用户反馈绕桩转弯时原地左右摆动，并要求检查 MAZE 绕行与终点指令是否冲突。用户提供 NoeticMaze 仓库作参考，随后明确授权烧录并确认独立电机动力断开、USB 保持连接。

### 定位与修复

- w4_bt_rx.c 已对 PLANNING/TRACKING 状态实施命令互斥。新增回归验证运行 MAZE 时收到 GOTO 返回 BUSY,NAV_ACTIVE，原命令 ID 与 MAZE 目标保持有效。
- 旧 w4_control.c 对每个 MAZE 短路段执行最终航向回正，到点后仍可能原地转向，再由下一段要求另一个朝向。用旧控制器运行新增测试，在 test_maze_handoff 的停车 PWM 断言处复现。原始失败信息保存在 build/maze_smooth/old_regression_failure.txt。
- MAZE 命令只指定 XY 坐标。现按位置及双轮连续停稳 200 ms 交接路段或完成任务。中间路点位置门槛取 25 mm，以控制障碍附近的提前切段误差。最终位置门槛仍由 position_tol 配置。
- MAZE 的前视点限制在当前可执行路段内，接近路点时连续收敛到该点，消除原来沿程余量 20 mm 处切到目标方位角的突变。
- MAZE 转向随偏角渐进减速，前进请求加速度限制 0.45 m/s²、减速度限制 0.60 m/s²。大于 0.85 rad 的偏角触发锁定目标的转向状态，约 3° 内且停稳 100 ms 后恢复前进，消除旧 0.50 rad 单阈值往返切换。障碍及传感器停车直接清零输出。
- 参考 NoeticMaze 的前视跟踪与按偏角渐进减速思路。参考版本为 f869723cec2ebecf77fdf6442a664b6b0ba5e7c4，源码见 https://github.com/StarDust-XCHH/NoeticMaze/blob/f869723cec2ebecf77fdf6442a664b6b0ba5e7c4/App/motor/Src/startMotion.c 。其 README 另有重规划导致摆动的已知问题，控制参数需按本车验证。

### 验证

14 项 Cortex-M4 软件回归全部通过。新增测试覆盖中间/最终 XY 路点带航向偏差停车、0.48/0.52 rad 偏角交替时双轮持续前进、线速度变化上限、大偏角转向状态锁定、GOTO 忙时拒绝及转向中 STOP。已有不同左右轮响应的挡板闭环模型到达并停车，最小离障 0.17 m 断言通过。原 GOTO 左右转弯、传感器、雷达过期及 STOP 回归通过。

旧版和修复版的仿真轨迹保存在 build/maze_smooth/before/navigation_maze.trace.json 与 build/cortex_tests/navigation_maze.trace.json。标准绕障模型用时由约 14.83 s 变为 19.76 s，修复采用更早减速和更紧的中间路点位置控制。模型仍出现因路段阻塞而重新选点，因此这些结果只证明所列回归通过，实际平顺性需要带动力试验。

66 个 ARM 源文件构建成功。text 96968、data 884、bss 90968 字节。构建日志中的警告来自既有未用裸机函数和 HAL 参数。修改文件为 week4/Src/w4_control.c、tests/navigation/test_maze.c、codingday4_work/run_cortex_tests.py 及本 README，补丁与修改前源码保存在 build/maze_smooth。

### 板上版本与结果

已烧录 build/maze_smooth/Week4_Integrated.hex。采用 Under Reset + Hardware Reset，编程器报告 Download verified successfully，并复位。独立 HOTPLUG 读回 97,852 个 HEX 有效字节完全一致。

HEX SHA256 为 `b44d804643e7587bf47591b106c818c9f3932f311f686af1efef3a02206d35ef`。ELF SHA256 为 `a1c441658d92a32258ffc7bc4ac18a1b46b075082f63bc158bddc72273061718`。

两次启动快照均为导航 IDLE、MCU 故障 0、四路 PWM 0。随后只读检查 IMU valid=1、calibrated=1，完成采样 11248 次、累计读取失败 9 次，XY BIAS 告警值 1。该结果对应动力断开期间。

烧录前完整 Flash 备份为 build/maze_smooth/before_flash.bin。烧录、读回、启动快照及验证摘要均位于 build/maze_smooth，分别见 flash_20261127.txt、readback_20261127.bin、startup_0/1.json、imu_ready.json、verification.json。

本次完成代码修复、软件验证、烧录和静态启动检查。带动力绕桩的平顺性、整条路线完成情况和运动期间 I2C 稳定性待现场复测。实车条件需重新确认后再发送运动命令。


## 62. 2026-11-27 实测转弯后停滞与持续转向修复

用户反馈转弯后一段时间突然停住，要求记录其发送的一次指令并据数据修改，随后明确要求删除途中超过 0.85 rad 即停车重对齐的功能，并授权立即烧录。

### 单次试跑证据

复用现有蓝牙监控进程，读取正在写入的 logs/maze_detour_live_20261127_154624.bin，16:29:58 开始独立保存后续流。先前沙箱内文件视图为旧状态，实际监控一直连接并写日志。初次启动采集曾被自动审批服务的额度错误中断，用户要求继续后重试成功。

- 用户发送 MAZE,0,2，命令 ID 15。主板 1,194,935 ms 接受，第一路点 (2,400) mm。之后路点为 (195,741) mm 和 (300,711) mm。
- 第一段转弯后前进到 (305,681) mm，距当前路点约 30.4 mm，目标方位偏角约 40.6°。主板保持 TRACKING、fault=0，后续约 8.05 秒位置基本不变。
- 停住前后两轮请求约 2 mm/s 和 24–26 mm/s，反馈均接近零。旧公式在接近路点时把接近速度继续乘以趋零的转弯系数，产生不足以有效移动的请求，同时低于 0.03 m/s 的起步补偿门槛。此前同一启动中的命令 10 也出现 2/29–30 mm/s 长期停滞，最后 FAULT 5。
- 本趟 IMU 失败计数从开始到结束均为 61，I 帧持续发布，故障码一直为 0。主板 1,214,100 ms 收到命令 16 的 STOP_LATCHED 后进入 STOPPED。最先发生的停滞早于该 STOP 约 8 秒。
- 完整采集为 logs/maze_stop_capture_20261127_162958.bin/.json，截取本趟的原始记录为 logs/maze_stop_cmd15_20261127.bin。结构化摘要为 build/maze_continuous/run15_summary.json，详细解析为 run15_analysis.json。

### 修改与验证

w4_control.c 已删除途中 fabsf(error)>0.85f 时调用 begin_alignment 的分支，以及专用 maze_alignment_heading 状态。转弯减速系数仍连续变化，行驶请求平均速度下限取 min(speed_limit,0.065 m/s)，再经过既有加减速斜坡。0.065 m/s 取自原接近路点速度下限，属于待带载复测的参数选择。起步斜坡阶段请求会短暂低于该值。

当前控制仍会在开始新路段时对齐航向、中间路点停稳交接、最终目标到达或保护条件触发时停车。此修复针对途中角度触发的停车重对齐和低速停滞。原始障碍、传感器、STOP 与堵转停车继续按各自条件工作。

tests/navigation/test_maze.c 新增检查 0.80/0.90 rad 偏角交替时持续双轮前进，以及距路点 26–30 mm、左右 0.70 rad 偏差时平均请求至少约 65 mm/s。旧控制器在新增角度连续性测试失败，记录见 regression_old_failure.txt。最终 14 项 Cortex-M4 回归全部通过，包含完整挡板绕行、最小离障 0.17 m、到达停车、传感器过期、STOP 和原 GOTO。日志为 build/maze_continuous/regressions.txt。

第一版尝试采用余弦转弯系数时完整绕障模型触发故障，随后恢复原转弯减速曲线并增加速度下限，通过最终回归。该临时版本未烧录。最终仅修改 week4/Src/w4_control.c、tests/navigation/test_maze.c 及本 README，修改前备份和 diff 在 build/maze_continuous。

### 已烧录版本

66 个 ARM 源文件构建成功，text 96824、data 884、bss 90960 字节。沿用既有未用裸机函数及 HAL 参数构建警告。用户完成接入并明确要求烧录后，预检导航 IDLE、四路 PWM 均为 0。备份原 Flash 后使用 Under Reset + Hardware Reset 烧录 build/maze_continuous/Week4_Integrated.hex，编程器校验通过并复位，独立读回 97,708 个 HEX 有效字节全部一致。

HEX SHA256 为 `947a389da866831f5a3c15774a5be8e4e3a4e4a1abba90b5f0ea8b4841d1ba48`。ELF SHA256 为 `73b1f9909053fb5720b362ac7aa4a005c5938cb7c642b43f813c31db96b7c993`。

启动约 4.1 秒时导航 IDLE、MCU 故障 0、四路 PWM 0，IMU valid=1、calibrated=1、XY BIAS=1，读取失败 0。之后约 25 秒附近有效 I 帧出现间断，到 33–38 秒已经恢复持续输出，S 帧累计 IMU 失败 6，B 帧超时 6、恢复 2。该传感器链路问题仍需后续排查。

备份、烧录日志、读回及启动检查保存在 build/maze_continuous/before_flash.bin、flash_20261127.txt、readback_20261127.bin、startup.json、verification.json。现有监控保持运行。新版带动力转弯及完整绕桩效果尚未实测，不以软件回归和静态烧录验证替代实车验收。


## 63. 2026-11-27 按 NoeticMaze 源码移植全路径导航

用户要求直接采用 NoeticMaze 代码逻辑，适配本车接口，并保持直行占空比及既有约定。
本轮已在 codingday4_work/firmware_source 实现和验证；没有运行烧录工具，没有发送实车运动指令。

### 代码与接口

上游固定提交 f869723cec2ebecf77fdf6442a664b6b0ba5e7c4，MIT 原许可证及原始代码保存在 third_party/NoeticMaze。
详细移植对照、构建方法和测试边界见 third_party/NoeticMaze/PORTING.md。

直接移植 A*、优先队列、双向拉直、Bezier 安全倒角及 startMotion 弧长投影/前视跟踪。
MAZE 改为整路径连续跟踪；不再使用原先 0.46 m 短路段逐段停车、重新选点和初始对齐。
新障碍或近处未知空间触发停车重规划，最终 XY 到达且双轮停稳 200 ms 后 ARRIVED。
保留上游方向切换和偏角减速，不自动返航。雷达解码、现有建图/平移 ICP、IMU、OLED 和电机使用本车接口。

固定直行占空比逻辑左 0.3099 / 右 0.3559 未变，CAL 启用约定未变。
直行纠偏和距离控制代码段逐字一致，8 个底层/标定文件 SHA256 一致。
车尾前置、左右交换、引脚、编码器符号、CPR、轮径/轮距、原 GOTO、STOP、RESET、SPEED/CAL 均保留。
没有换成上游电机 PID 或 PWM 接线。

航向增益用本车已有 2.0，MAZE 保持 0.18 m/s 上限；直接用上游 0.90 时双桩模型转向滞后，该中间版本未交付。
硬避障 0.20 m，平滑曲线按 0.30 m 留跟踪余量，平滑失败时回退并验证完整 A* 路径。
修复短线漏检、同格目标、对角穿角、截断和边界处理；没有恢复旧途中 0.85 rad 停车重对齐状态机。

### 验证

15 项 Cortex-M4 回归全部通过，含原单/双编码器 GOTO、车尾前置直行、传感器/OLED、MAZE 和规划边界。
21 项 PC 回归通过，使用现有 Python 3.12 虚拟环境。
空场、单挡板、双桩、交错墙迷宫及途中新双桩的闭环模型均到达并输出零 PWM。
单挡板/双桩/迷宫/新障碍模型最小离障约 0.294/0.313/0.277/0.275 m。
交错墙模型用时 28.18 s、有 6 次重规划，不表示任意场景全程不停顿。
结果在 build/noetic_port/scenarios.json，测试源码在 tests/navigation/test_maze.c 和 test_noetic_planner.c。
这些结果不等于真实 RTOS 时序、电气可靠性或实车通关验收。

### 交付构建与板上状态

完整 ARM 构建 70 源文件成功，text 103608、data 908、bss 111816 字节，见 build/noetic_port/build.log。
保留既有未用裸机函数和 HAL 参数警告，移植模块没有新增编译警告。

固件 build/noetic_port/Week4_Integrated.hex，SHA256：f1583f0917e4bfebad2d8e7f88f9b1996ef35ccaccf095ab650a4d96d4c6b79e。

配套 ELF SHA256：6892766e2b96ac6bc316c272b6e2e2f450e7a43d52929795275e5a1ed0fe3804。

验证摘要、底层哈希、差异及备份在 build/noetic_port/verification.json、protected_sha256.json、changes.diff、before。
本轮未烧录，不要用新 ELF 解释旧板上固件 RAM。板上最近已验证烧录仍为第 62 节 maze_continuous。
前次 I2C 间歇超时/IMU恢复问题仍保留，没有取消传感器保护。

接续从第 63 节和 third_party/NoeticMaze/PORTING.md 开始。
指令仍为 STOP、RESET、SPEED,0.18、MAZE,x,y（米，初始 +Y，按现场选择目标）。
本轮软件移植和验证已完成；烧录、板上时序和带动力整条路线尚未验收。


### 第63节补充：主板连接与烧录预检（2026-11-27 17:16）

用户表示已连接主板。HOTPLUG 确认 ST-LINK 0674FF545155657067032031、STM32F446xx、3.22 V，四路 PWM 均为 0。
只读备份完整 524288 字节 Flash 到 build/noetic_port/preflash_20261127_171616/before_flash.bin。
备份与第62节 maze_continuous HEX 的 97708 个有效字节逐一相同，差异 0。
烧录命令被自动审批拒绝，理由为尚需用户明确授权这次固件写入；该命令没有执行，板上固件未改动。
待用户明确同意烧录 build/noetic_port/Week4_Integrated.hex 后继续写入、独立读回及启动检查。


## 64. 2026-11-27 NoeticMaze 移植版烧录

用户明确同意烧录。ST-LINK 0674FF545155657067032031，STM32F446xx，3.22–3.23 V，预检四路 PWM 为 0。原完整 Flash 备份见 build/noetic_port/preflash_20261127_171616/before_flash.bin，与第62节固件 97708 个有效字节一致。

烧录 build/noetic_port/Week4_Integrated.hex，编程器校验成功并复位；独立 HOTPLUG 回读 104516 个 HEX 有效字节，差异 0。HEX SHA256 f1583f0917e4bfebad2d8e7f88f9b1996ef35ccaccf095ab650a4d96d4c6b79e。

启动约 43.8 s 时导航 IDLE、导航及 MCU 故障均 0、四路 PWM 0；IMU valid=1、calibrated=1、XY BIAS=1，完成 4246 次、累计失败 6 次。间歇 I2C 问题仍需关注。烧录日志、独立读回和启动状态在 build/noetic_port/flash_authorized_20261127.txt、readback_result.json、startup_authorized.json。未发送运动命令，带动力绕桩和迷宫尚未实测。


## 65. 前端地图图例调整

已将 pc/monitor.py 中遮挡地图的图例移至两张图下方，改为紧凑横排，并调整图表留白。离线预览 build/noetic_port/legend_preview.png；PC 21 项回归通过。仅修改前端，已运行的监控窗口需关闭后重新启动才能加载新版；无需重新烧录。

## 66. 2026-11-27 FAULT 1 与墙图消失

读取实际运行日志 gui_launch_20261127_174141.bin，保存不可变副本和三次故障摘要至活动工程 build/map_fault/telemetry_snapshot.bin、fault_timeline.json。
命令4在208300ms、命令7在291250ms、命令9在319300ms报FAULT 1。故障前后IMU failed分别48→51、57→60、60→63，I2C timeout/recovery增长；IMU有效I帧约中断2.65秒，之后恢复。故障前地图M帧仍在更新，故障后短暂发布旧扫描后停止。
因果链为传感器链路中断导致控制FAULT 1，fault()将pose.valid清零，恢复后的IMU不能自动恢复丢失的世界位姿，建图停止。该锁定需要重新物理定位后RESET，不能只放宽传感器门槛或使用无效位姿继续走。底层I2C中断的电气/驱动根因尚未确定。
已修复前端：不再因1秒超时清空历史墙图，而是淡色保留并显示过期/冻结状态；导航FAULT优先显示实际故障及原因；原pose age改为telemetry age，避免把持续N遥测误认为定位有效；命令反馈移到输入框下方，避免错误提示藏在窗口底部。RESET、重启和地图epoch变化仍清空旧图。
PC回归23项通过，离线故障布局已验证，预览build/map_fault/frozen_map_preview.png。仅更新前端，未重启现有监控、未烧录固件或发送运动命令。此次修复是显示与诊断改进，不代表IMU/I2C中断及实车停车已解决。