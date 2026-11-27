# 项目代码检查报告

本报告保留修复前的检查结果。第一节的三个问题已在后续修复，导航故障场景已纳入常规回归测试。

检查日期：2026-11-18。依据：用户提供的 Block4.5-CodingDay-Part2.pptx 第2–8页，包含第3、4页的路径图和架构图。PPT 作为验收依据读取，未把其中的课堂操作指令当成修改授权。

本次检查覆盖 Core 中的启动、外设、中断和旧应用片段，week4 全部业务 C/H 文件，PC 协议与监控程序，构建配置、脚本及现有测试。HAL、CMSIS、FreeRTOS 主要核对项目使用的接口、中断优先级、构建选择和集成方式，未逐行审计所有第三方实现。未修改业务代码、PPT 或预编译 HEX；新增本报告和 review/reproduce_planning.c 复现程序。运行现有测试产生了测试可执行文件。

结论：主要并发架构已实现，现有主机测试通过；但仍有下述缺陷，且默认配置禁止运动，不能据此判定已完成实车验收。

## 一、发现的问题（按优先级）

### 1. [P1] CPU 异常处理不关闭电机输出

位置：Core/Src/stm32f4xx_it.c:84（HardFault），以及同文件的 MemManage/BusFault/UsageFault。

这些处理函数直接进入无限循环。若导航期间出现异常，TIM4/TIM12 的硬件 PWM 不会因为 CPU 停在异常中自动清零；控制任务和蓝牙 STOP 则无法继续运行，因此可能保持异常前的驱动输出。这是静态控制流和外设计时器配置所显示的缺陷，未在真实小车上触发异常。

建议：异常入口使用不依赖 RTOS、I2C 或浮点运算的停机路径，关闭已使能的 TIM4/TIM12 输出并记录故障。项目 week4/Src/w4_fault.c 已有 W4Rtos_Fatal 寄存器停机实现，可复用其思路；若进一步考虑栈损坏，需要专门的最小异常处理入口。

### 2. [P2] 导航启动阶段 IMU 失效会永久卡在 PLANNING

位置：week4/Src/w4_control.c:67–76。

传感器无效时，只有 TRACKING 状态进入 fault(1)，PLANNING 仅停止电机后提前返回；后面的 120 秒超时也执行不到。

已用生产导航模块及现有 mock 复现：

1. IMU 正常时接收 GOTO,0,1.4，路径任务返回 PATH_ACCEPTED。
2. 在控制任务第一次取路径点之前使 IMU 持续失效。
3. 连续执行 121 秒控制和路径轮询，结果仍是 PLANNING、fault=0。
4. 再发 GOTO，返回 BUSY,NAV_ACTIVE。

电机在此场景已停止，但故障没有显式上报，且不能直接重试或 RESET；须先 STOP 才能退出。建议把 PLANNING 和 TRACKING 都纳入活动导航的传感器失效处理，并把导航总超时放在传感器提前返回之前。

复现源文件：review/reproduce_planning.c。复现输出：

```text
GUI example target: A,1,REJECTED,ROUTE_UNCONFIRMED
Required target: A,2,OK,PATH_ACCEPTED
After 121 seconds of failed IMU: state=1 (PLANNING=1), fault=0
Retry target: A,3,BUSY,NAV_ACTIVE
```

### 3. [P2] PC 默认目标会被固件拒绝；路径接口实际只支持固定目标

位置：pc/monitor.py:152，week4/Src/w4_path.c:35–44。

PC 输入框默认是 GOTO,0.3,0.2，协议允许 ±2 m 范围的坐标，但路径生成器只接受三个指定目标、图示额外目标 (0.7,0.7) 和原点，其他坐标统一返回 ROUTE_UNCONFIRMED。上述默认命令已复现被拒绝。三条必测目标本身已存在生成分支，不能把本问题描述成“必测目标缺失”。

建议至少将默认命令改为 GOTO,0,1.4 并清楚展示支持范围。如果课程期望接收可达通道上的一般目标，则需按照场地通道参数生成路径、验证可达性，不能仅靠目标白名单，也不应在未知迷宫中直接连接任意点。

## 二、验收阻碍和要求边界

- 默认运动锁：week4/Src/w4_calibration.c:49 只有定义 W4_CALIBRATED 才设置 verified 等标志；当前构建脚本和 CubeIDE 配置均未定义。默认 GOTO 因而返回 HARDWARE_NOT_READY。这是明确的保护设计，并非蓝牙故障，但该默认版本不能完成三次导航演示。轮径、轮距、脉冲数和方向必须实测后再解锁，不能仅为了运行而盲目加宏。
- 单编码器：week4/Src/w4_pose_single.c 只测右轮脉冲，方向来自电机输出，左轮速度由右轮速度和陀螺仪推算。PPT 第4页参考图画了左右轮增量 ΔL/ΔR；若助教要求双轮独立反馈，当前硬件/估计方案与参考图存在差异。PPT 文字不能单独证明单编码器方案必然不合格，需按实际验收口径判断。左轮堵转也不能从当前代码中的右轮堵转判据独立识别。
- PPT 坐标矛盾：第2页文字第三目标为 (0.7,-0.7)，第3页图为 (0.7,0.7)。代码支持两者，但负 Y 分支走 (0,0)→(0,-0.7)→(0.7,-0.7)，该通道在图上没有画出。因此支持负坐标不等于这条路线能通过图示迷宫；应在场地确认路线。
- OLED 默认假设为 SSD1306 128×64 / 0x3C，未确认实物型号。50 ms 是任务执行周期，每次最多刷16字节；完整画面更新不等于20 FPS。
- PC 显示位姿历史、目标、航向和雷达点；没有生成路径点的独立遥测/绘图。因此不能把蓝色历史线当作规划路径。第4页参考架构包含 path 遥测，应在演示中明确此区别。
- 旧按钮/电位器：RTOS 模式不调用 Buttons_Process/Motor_Process，ADC 仍采样，但电位器不调导航 PWM、双键也不会进入旧逻辑停车。不要把旧变量或保留的代码片段当作正在工作的功能；若本次仍要求演示这些旧交互，需要明确整合到 RTOS 控制所有权中。
- 队列诊断：Q_Cmd/Q_Path 的业务调用直接使用 CMSIS 队列 API，绕过 w4_queues.c 的统计包装。因此 W4Queue_GetStats 对它们的累计吞吐/延迟不能代表实际运行；Q 遥测中的即时深度来自真实队列，不受此问题影响。

## 三、与六项必做要求的对应

| PPT 要求 | 代码情况 | 尚需证明 |
|---|---|---|
| 完整 Sense→Think→Act 与 PC 可视化 | 7个任务及集成回调已存在，控制任务合并编码器/位姿/PID/电机 | 实物同时运行，单条命令完成路程 |
| 1 ms tick，10/50 ms 与事件任务 | tick=1000 Hz，控制10 ms、OLED/TX发布50 ms，已设置优先级和栈并采集运行数据 | 板上周期、抖动、栈余量、截止时间 |
| Q_Lidar / Q_Cmd / Q_Path | 容量8/8/64；LiDAR按值传描述符，有序号和复制前后覆盖检查；STOP独立锁存，路径有回压 | DMA真实速率和覆盖、队列满时行为；避免使用未接入的统计作证 |
| 定时器→信号量→IMU | TIM5 100 Hz，二值信号量，任务内I2C读取；有合并触发统计 | 持续真实采样率和最长间隔 |
| IMU/OLED共享优先级继承mutex | 已使用 osMutexPrioInherit；异步事务等完成/终止后释放 | 真实锁竞争、优先级继承和总线时序 |
| 10 ms PID导航、三次试验 | 必测目标路径分支和闭环控制已实现 | 修复上述缺陷、物理标定、确认路线并实测终点位置/航向误差 |

ICP、OGM、A* 在 PPT 明确为选做，未实现不列为缺陷。PPT 允许相关10 ms功能合并任务，控制任务的合并本身也不列为缺陷。

## 四、本次实际执行的验证

| 检查 | 本次结果 |
|---|---|
| python3 -B tools/test_python.py | 16/16通过 |
| 主机导航回归 | 通过，使用 -Wall -Wextra -Werror |
| 主机传感器回归 | IMU启动、IMU恢复、OLED关闭、128×32、128×64五组通过 |
| 新增针对性复现 | 已复现PLANNING卡死和GUI默认目标被拒绝 |
| ARM固件重编译 | 未完成：本机未找到arm-none-eabi-gcc |
| 烧录、真实RTOS调度、GUI/串口/BLE及实车试验 | 未执行 |

主机C测试初次因本机默认MacOSX27 SDK与链接器不兼容而失败；指定已安装的15.4 SDK后通过，命令如下。此问题属于本机工具链环境，不作为固件源码编译错误。

```sh
SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk python3 -B tools/test_arm.py
```

新增复现的构建运行命令（从项目目录运行）：

```sh
SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk gcc -std=c11 -DW4_SINGLE_ENCODER -Wall -Wextra -Werror -Iweek4/Inc -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 review/reproduce_planning.c tests/navigation/mock_port.c week4/Src/w4_common.c week4/Src/w4_pose_single.c week4/Src/w4_control.c week4/Src/w4_motor.c week4/Src/w4_path.c week4/Src/w4_bt_rx.c -lm -o /private/tmp/w4_review_planning
/private/tmp/w4_review_planning
```

已有 VALIDATION.md 记录的是此前其他环境的验证。本报告只对本次实际执行的结果负责，未把已有60源文件ARM编译记录和预编译HEX视为本次重新验证通过。
