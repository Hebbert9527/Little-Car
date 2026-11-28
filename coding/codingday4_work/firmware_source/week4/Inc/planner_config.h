//
// Created by lmtgy on 2026/4/15.
// 1. 公共配置层

#ifndef STM32_PLANNER_PC_TEST_PLANNER_CONFIG_H
#define STM32_PLANNER_PC_TEST_PLANNER_CONFIG_H

// ==========================================
// 算法配置层
// ==========================================
#include <stdint.h>
/* UART and motor configuration remain owned by Week4. */

#define PLANNER_MAP_RES 0.05f
#define ASTAR_GOAL_TOL_GRIDS 0
#define PHYSICAL_RADIUS 0.15f
#define PATH_CLEARANCE_RADIUS (PHYSICAL_RADIUS + 0.5f*PLANNER_MAP_RES)
#define INFLATE_L1_RAD 0.20f
#define INFLATE_L1_PENALTY 100.0f
#define INFLATE_L2_RAD 0.25f
#define INFLATE_L2_PENALTY 15.0f
#define INFLATE_L3_RAD 0.30f
#define INFLATE_L3_PENALTY 3.0f
#define SAFE_SMOOTH_RAD 0.275f // Prefer room for sparse wall endpoints and tracking error; raw A* handles narrow corridors


#define PLANNER_DOWNSAMPLE_SCALE 5 // legacy upstream constant; Week4 uses PLANNER_MAP_RES directly
#define PLANNER_REPLAN_GOAL_SHIFT_CELLS 1    // 目标点相对上一次规划目标，变化多少个 planner cell 才触发“目标变化重规划”。
#define PLANNER_REPLAN_DRIFT_CELLS 5        // 机器人当前位置相对旧路径起点漂移多少个 planner cell，才认为旧规划起点已经失效，需要重规划。
#define PLANNER_REPLAN_BLOCKED_HITS 4       // 连续多少帧都检测到当前路径被阻断，才真正触发“路径被堵死”的重规划。
#define PLANNER_REPLAN_MIN_INTERVAL_MS 300U // 两次重规划请求之间的最小时间间隔。



// 新增：历史路径亲和力配置
#define PATH_AFFINITY_DISCOUNT 0.6f
// 【修复】：直接改为物理单位米，解除与分辨率的强耦合
#define PATH_AFFINITY_RADIUS_M 0.3f
// ==========================================
// 【嵌入式预备修改】：静态内存池宏定义与分配
// ==========================================
#define MAX_GRID_SIZE 213 // 5 cm cells, +/-5.3 m including clearance border
#define MAX_CELLS (MAX_GRID_SIZE * MAX_GRID_SIZE)  // 45369

// 【修复】：明确 4bit/cell 打包大小 (45369 * 4bit = 22685 Bytes)
#define MAX_MAP_BYTES (((MAX_CELLS * 4) + 7) / 8)

#define MAX_PQ_SIZE 2200                           // 优先队列最大容量
#define MAX_PATH_LEN 320                           // 路径最大节点数 (320 * 5cm = 16 m)

// 【核心优化修改 1】：新增用于临时局部计算的短数组长度宏
#define MAX_TEMP_LEN 100                           // 局部插值只需很短的数组

// ==========================================
// 极致压缩：地图与代价定点数宏定义
// ==========================================
#define COST_SCALE 10                  // 浮点数放大倍数
#define STEP_STRAIGHT 10               // 直行代价 (1.0 * 10)
#define STEP_DIAG 14                   // 斜行代价 (1.414 * 10)
#define MAX_COST_INF 65535             // uint16_t 的无穷大

#define INFLATE_L1_PENALTY_INT 1000    // 100.0 * 10
#define INFLATE_L2_PENALTY_INT 150     // 15.0 * 10
#define INFLATE_L3_PENALTY_INT 30      // 3.0 * 10
#define AFFINITY_DISCOUNT_INT 6        // 0.6 * 10

// 3. 路径计算静态内存 (避免局部爆栈，全部放进 BSS 段)
typedef struct { float x; float y; } Point2D;

// ==========================================
// 极致优化：分时共享内存池 (Union)
// ==========================================
typedef union {
    // 阶段 1：A* 寻路时使用
    uint32_t pq_data[MAX_PQ_SIZE];

    // 阶段 2：路径平滑时使用
    struct {
        Point2D path_buffer_B[MAX_PATH_LEN];
        Point2D temp_bezier[MAX_TEMP_LEN];
        Point2D temp_line[MAX_TEMP_LEN];
    } smooth;
} PlannerMemoryPool;

// 声明外部全局变量（告知所有引用此头文件的 .c 文件）
extern PlannerMemoryPool g_mem_pool;
/* Shared with W4Maze per-scan temporary flags (never used concurrently in W4PathTask). */
#define PLANNER_NODE_SLOTS 6503U
extern uint32_t w4_planner_nodes[PLANNER_NODE_SLOTS];
_Static_assert(MAX_CELLS < 65535, "planner cell index must fit uint16_t");

#endif //STM32_PLANNER_PC_TEST_PLANNER_CONFIG_H
