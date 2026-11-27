from pathlib import Path
import hashlib,json,difflib,shutil
R=Path(r"D:\EMBEGGed\week4codingday\coding\coding(1)\coding\codingday4_work\firmware_source")
B=R/"build/noetic_port"
U=R/"third_party/NoeticMaze"
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
protected=json.loads((B/"protected_sha256.json").read_text())
assert all(sha(R/p)==h for p,h in protected.items())
a=(B/"before/w4_control.c").read_text(encoding="utf-8")
b=(R/"week4/Src/w4_control.c").read_text(encoding="utf-8")
checks={}
for label,start,end in [("fixed_straight_duty","W4StraightCalibration w4_straight_cal","volatile float"),
                       ("straight_correction","static float straight_correction","static void control_step"),
                       ("distance_control","    if(wp.index==W4_DISTANCE_WAYPOINT) {\n        if(starting_distance)","    float dx=wp.x_m")]:
    checks[label]=a[a.index(start):a.index(end,a.index(start))]==b[b.index(start):b.index(end,b.index(start))]
assert all(checks.values())
outcomes=[o for o in json.loads((R/"build/cortex_tests/navigation_maze.outcomes.json").read_text()) if o["scenario"]!="unused"]
assert all(o["state"]==3 and o["fault"]==0 for o in outcomes)
for o in outcomes:
    if o["scenario"] in ("open","board"): o["replans"]=None
(B/"scenarios.json").write_text(json.dumps(outcomes,indent=2),encoding="utf-8")
cases=["imu","imu_recovery","imu_diagnostics","imu_xy_bias","imu_rotation","yaw_calibration",
       "oled_disabled","oled32","oled64","navigation","navigation_dual","navigation_rear_forward",
       "navigation_rear_no_straight_correction","navigation_maze","noetic_planner"]
v={"date":"2026-11-27","upstream":"https://github.com/StarDust-XCHH/NoeticMaze",
   "commit":"f869723cec2ebecf77fdf6442a664b6b0ba5e7c4",
   "cortex_passed":cases,"pc_tests_passed":21,"firmware_sources":70,
   "text_bytes":103608,"data_bytes":908,"bss_bytes":111816,
   "artifact_sha256":{p.name:sha(p) for p in [B/"Week4_Integrated.hex",B/"Week4_Integrated.elf",B/"Week4_Integrated.bin"]},
   "protected_sha256":protected,"unchanged_control_sections":checks,
   "scenarios":outcomes,"flashed":False,"physical_motion_test":False}
(B/"verification.json").write_text(json.dumps(v,indent=2,ensure_ascii=False),encoding="utf-8")
changes=[]
for name,target in [("w4_control.c","week4/Src/w4_control.c"),("w4_maze.c","week4/Src/w4_maze.c"),
                    ("w4_maze.h","week4/Inc/w4_maze.h"),("test_maze.c","tests/navigation/test_maze.c"),
                    ("run_cortex_tests.py","../run_cortex_tests.py")]:
    before=(B/"before"/name).read_text(encoding="utf-8").splitlines(True)
    after=(R/target).read_text(encoding="utf-8").splitlines(True)
    changes.extend(difflib.unified_diff(before,after,fromfile="before/"+target,tofile=target))
(B/"changes.diff").write_text("".join(changes),encoding="utf-8")
upstream_files=[p for p in U.rglob("*") if p.is_file() and (p.suffix in (".c",".h") or p.name=="LICENSE")]
(U/"SOURCE.json").write_text(json.dumps({"repository":v["upstream"],"commit":v["commit"],
    "sha256":{p.relative_to(U).as_posix():sha(p) for p in upstream_files}},indent=2),encoding="utf-8")
report="""# NoeticMaze 接口移植

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

"""
rows=["| 场景 | 仿真耗时 | 最小离障 | 重规划次数 |","| --- | ---: | ---: | ---: |"]
names={"open":"空场","board":"单挡板","poles":"双桩","maze":"交错墙迷宫","dynamic":"途中出现双桩"}
for o in outcomes:
    clearance=f'{o["min_clearance"]:.3f} m' if o["min_clearance"] else "未统计"
    replans="未统计" if o["replans"] is None else str(o["replans"])
    rows.append(f'| {names[o["scenario"]]} | {(o["ticks"]+1)*.01:.2f} s | {clearance} | {replans} |')
report+="\n".join(rows)+"\n\n原始数据及固件哈希见 build/noetic_port/scenarios.json、verification.json。\n"
(U/"PORTING.md").write_text(report,encoding="utf-8")
readme=R.parent.parent/"README.md"
if not (B/"before/README.md").exists(): shutil.copy2(readme,B/"before/README.md")
text=readme.read_text(encoding="utf-8")
text=text.replace(text.split("\n")[2],"最后保存于 2026-11-27。已完成 NoeticMaze 规划/平滑/连续跟踪移植、15 项 Cortex-M4 回归、21 项 PC 回归及完整构建，最新见第 63 节。移植版源码和固件位于活动工程，**本轮未烧录**；板上最近一次已验证版本仍见第 62 节。",1)
text=text.replace("当前接续见第 62 节","当前接续见第 63 节",1)
section="""

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

"""
section+=f'固件 build/noetic_port/Week4_Integrated.hex，SHA256：{v["artifact_sha256"]["Week4_Integrated.hex"]}。\n\n'
section+=f'配套 ELF SHA256：{v["artifact_sha256"]["Week4_Integrated.elf"]}。\n\n'
section+="""验证摘要、底层哈希、差异及备份在 build/noetic_port/verification.json、protected_sha256.json、changes.diff、before。
本轮未烧录，不要用新 ELF 解释旧板上固件 RAM。板上最近已验证烧录仍为第 62 节 maze_continuous。
前次 I2C 间歇超时/IMU恢复问题仍保留，没有取消传感器保护。

接续从第 63 节和 third_party/NoeticMaze/PORTING.md 开始。
指令仍为 STOP、RESET、SPEED,0.18、MAZE,x,y（米，初始 +Y，按现场选择目标）。
本轮软件移植和验证已完成；烧录、板上时序和带动力整条路线尚未验收。
"""
assert "## 63." not in text
readme.write_text(text+section,encoding="utf-8")
print(json.dumps(v["artifact_sha256"],indent=2))
print("README section 63 and PORTING.md saved; protected files and straight-control blocks verified unchanged.")
