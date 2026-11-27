# NoeticMaze 接口移植

日期：2026-11-27。固定上游版本：
https://github.com/StarDust-XCHH/NoeticMaze/tree/f869723cec2ebecf77fdf6442a664b6b0ba5e7c4 。

App/ 中保留上游原文件，LICENSE 为原 MIT 许可，SOURCE.json 记录来源及哈希。
编译使用 week4/Inc 与 week4/Src 内的适配副本，不编译本目录的上游硬件代码。

## 移植对照

| 上游代码 | 活动工程 | 逻辑 |
| --- | --- | --- |
| planner_algo.c | w4_noetic_planner_algo.c | 八邻域反向 A*、分层障碍代价、历史路径亲和力 |
| priority_queue.c | w4_noetic_priority_queue.c | 有界最小堆、共享静态内存池 |
| path_smoother.c | w4_noetic_path_smoother.c | 双向拉直、Bezier 安全倒角 |
| startMotion.c | w4_noetic_motion.c | 弧长投影、局部搜索、前视、方向滞回、转弯与到点减速、速度斜坡 |
| 规划/运动任务接口 | w4_maze.c、w4_control.c | 接入现有 MAZE、控制周期、电机及传感器保护 |

雷达解码、现有占用建图/平移 ICP、IMU 和 OLED 仍使用本车实现，没有接入上游 SPI IMU、PWM 引脚或地图原点。
MAZE 由逐段停车改为整条路径跟踪；只有终点、障碍/未知区保护及重规划等条件停车。
本车目标仍为 XY 到点且双轮停稳 200 ms，不自动返航。

## 保持的约定

- 固定直行占空比配置：逻辑左 0.3099、右 0.3559。CAL 启用状态约定不变，没有强制启用。
- 车尾前置、左右交换、引脚、PWM、编码器符号、CPR、轮径、轮距、轮速前馈及速度环保持。
- 原 GOTO 直行/折线控制、SPEED、CAL、RESET、STOP 和蓝牙协议保持。
- 电机输出调用原 W4Motor_SetWheelSpeeds / W4Motor_Update，传感器、STOP、堵转保护不绕过。
- 8 个底层/标定文件 SHA256 一致；直行占空比声明、纠偏和距离控制代码段逐字一致，见 build/noetic_port/verification.json。

## 必要适配

1. 本车居中 49×49、0.1 m 地图转换为上游正坐标 x-major 格式；输出回到本车里程计坐标。初始朝向仍 +Y，指令范围仍 [-2,2] m。
2. 航向增益使用原 w4_config.heading_kp=2.0，角速度上限原 max_w=1.0 rad/s；MAZE 保持原 0.18 m/s 上限和 0.45 m/s² 加速斜坡。上游 0.90 增益在双桩模型出现跟踪滞后，未交付该中间版本。
3. 硬避障 0.20 m；平滑曲线按 0.30 m 留跟踪余量。安全平滑失败时回退完整 A* 路径，并检查连接线和终点。
4. 保留弧长最大推进量、0.18 m 前视和方向切换滞回；航向角速度由 deg/s 转 rad/s。
5. A* 同格短目标保持精确终点，不提前一格到达；禁止对角穿角，队列满/截断/非法输入均返回失败。短于 0.1 m 的连接线也碰撞检查，越界视为障碍；空路径不访问负索引。
6. CMSIS DSP 数学调用改为本项目 libm；不用未对齐 uint32_t 指针清空数组。大数组保持静态。
7. 路径任务只在 PLANNING 写路由，控制任务复制后独立跟踪；发布前再检查 STOP、命令 ID、RESET epoch。重规划最小间隔 300 ms。
8. D,ms,MAZE_LEG,id,x_mm,y_mm 保留，PC 现在显示连续前视点；新增 D,ms,MAZE_ROUTE,id,route_number,point_count 诊断。
9. fault 9 雷达过期、fault 10 无路和原传感器/堵转保护保持；新增 fault 11 表示路径投影不可信。

上游跟踪逻辑允许大偏角时线速度降为零并转向，也保留前后方向选择；
没有恢复旧的“途中 >0.85 rad 锁定航向、停稳后再对齐”的状态机。
交错墙仍有重规划停车，不能称为任意场景全程不停顿。

## 构建与使用

在活动 firmware_source 目录运行：

    python -X utf8 tools/build.py --maze --out build/noetic_port --gcc 本机ARM编译器完整路径
    python -X utf8 ../run_cortex_tests.py
    .venv/Scripts/python.exe -B -X utf8 tools/test_python.py

使用 build/noetic_port/Week4_Integrated.hex 及配套 ELF；不要混用旧板上固件的 ELF。
本轮未烧录、未发送运动命令。板上最近验证记录仍对应 build/maze_continuous。
port_step*.py / port*_tests.py 是开发过程脚本，不是构建入口，不要再次运行覆盖最终代码。

实车启动应静止完成 IMU 标定，在起点朝向世界 +Y；示例命令依次为：

    STOP
    RESET
    SPEED,0.18
    MAZE,0,1.65

单位米，实际目标按现场可走区域选择。MAZE 自主规划避障，GOTO 保持原任务约定。
STOP 随时可用，保留一个 BLE 监控连接。前次 I2C 间歇超时尚未由本次算法移植解决。

## 验证

- 15 项 Cortex-M4 生产 C 软件回归全部通过，覆盖原 GOTO/直行/车尾前置、IMU/OLED、MAZE、规划边界。
- PC 21 项通过，使用项目 Python 3.12；系统 Python 3.14 的一次测试停滞已终止，未修改 PC 业务源码。
- 完整构建 70 源文件成功：text 103608、data 908、bss 111816 字节；链接器 heap/stack 保留区检查通过。既有裸机未用函数和 HAL 参数警告仍在，移植模块无新增编译警告。
- 五组闭环仿真均到达且零 PWM。左右轮采用不同动力响应，经过生产编码器、IMU、控制及电机代码闭环。
- 仿真不等于 FreeRTOS 实际调度、电气可靠性或实车验收。

| 场景 | 仿真耗时 | 最小离障 | 重规划次数 |
| --- | ---: | ---: | ---: |
| 空场 | 5.01 s | 未统计 | 未统计 |
| 单挡板 | 10.51 s | 0.294 m | 未统计 |
| 双桩 | 12.80 s | 0.313 m | 0 |
| 交错墙迷宫 | 28.18 s | 0.277 m | 6 |
| 途中出现双桩 | 13.10 s | 0.275 m | 1 |

原始数据及固件哈希见 build/noetic_port/scenarios.json、verification.json。
