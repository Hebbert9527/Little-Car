#ifdef W4_MAZE
/* Adapted from NoeticMaze f869723; MIT, see third_party/NoeticMaze/LICENSE. */
//
// Created by lmtgy on 2026/4/15.
// 3. 核心算法层

#include "planner_algo.h"
#include "Cstar_map_core.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "priority_queue.h"

// 2. A* 算法静态内存 (与 W4Maze 单圈临时标志位复用，同属 W4PathTask 串行执行)
uint16_t w4_planner_g_score[MAX_CELLS];
#define s_g_score w4_planner_g_score
/* Eight directions and sentinel 15 fit in four bits per node. */
static uint8_t s_came_from_dir[(MAX_CELLS+1)/2];
static uint8_t predecessor(int i) {
    return (s_came_from_dir[i/2]>>((i&1)*4))&15U;
}
static void set_predecessor(int i,uint8_t direction) {
    unsigned shift=(i&1)*4;
    s_came_from_dir[i/2]=(s_came_from_dir[i/2]&~(15U<<shift))|(direction<<shift);
}

#define COLLISION_COST 0xFFFF
static const char *failure_reason;
const char *W4Planner_FailureReason(void) { return failure_reason; }
static int failed(const char *reason) { failure_reason=reason; return 0; }

// ==========================================
// 算法引擎层
// ==========================================

// 将物理半径（米）转换为精确的网格平方半径，避免 (int) 截断把 0.15 m 压扁成 1 格（10 cm）
static inline int radius_to_grid_sq(float radius, float inv_res, int *out_cell_r) {
    float r_g = radius * inv_res;
    if (r_g < 0.0f) r_g = 0.0f;
    int r_sq = (int)floorf(r_g * r_g + 1e-4f);
    if (out_cell_r) {
        int lim = (int)floorf(r_g + 1e-4f);
        while ((lim + 1) * (lim + 1) <= r_sq) ++lim;
        while (lim > 0 && lim * lim > r_sq) --lim;
        *out_cell_r = lim;
    }
    return r_sq;
}

// 对外的单一物理碰撞检查（原版逻辑优化）
bool check_collision(ServerMap* map, Point2D pos, float radius, bool is_return) {
    float inv_res = 1.0f / map->res; // 优化：FPU 乘法代替除法
    int cell_r = 0;
    int cell_r_sq = radius_to_grid_sq(radius, inv_res, &cell_r);
    int cx = (int)floorf(pos.x * inv_res);
    int cy = (int)floorf(pos.y * inv_res);

    if (cx < 0 || cx >= map->grid_size || cy < 0 || cy >= map->grid_size) return true;

    for (int i = -cell_r; i <= cell_r; i++) {
        for (int j = -cell_r; j <= cell_r; j++) {
            if (i*i + j*j <= cell_r_sq) {
                int nx = cx + i, ny = cy + j;
                if (nx >= 0 && nx < map->grid_size && ny >= 0 && ny < map->grid_size) {
                    uint8_t val = get_grid_val(map, nx, ny);
                    if (val == 1 || (is_return && val == 0)) return true;
                } else return true;
            }
        }
    }
    return false;
}

// 【核心优化 1】：单次遍历评估节点总代价（将4次重复扫描合并为1次纯整数扫描）
static inline uint16_t evaluate_node_cost(ServerMap* map, int cx, int cy, bool is_return,
                                          int r_phys2, int r_l12, int r_l22, int r_l32, int max_r) {
    uint16_t max_penalty = 0;

    for (int i = -max_r; i <= max_r; i++) {
        for (int j = -max_r; j <= max_r; j++) {
            int d2 = i*i + j*j;
            if (d2 > r_l32) continue; // 超出最大膨胀半径直接跳过

            int nx = cx + i;
            int ny = cy + j;

            // 越界视为障碍物
            if (nx < 0 || nx >= map->grid_size || ny < 0 || ny >= map->grid_size) {
                if (d2 <= r_phys2) return COLLISION_COST;
                continue;
            }

            uint8_t val = get_grid_val(map, nx, ny);
            if (val == 1 || (is_return && val == 0)) {
                if (d2 <= r_phys2) return COLLISION_COST; // 致命碰撞
                // 分级膨胀惩罚获取最高值
                else if (d2 <= r_l12) { if (max_penalty < INFLATE_L1_PENALTY_INT) max_penalty = INFLATE_L1_PENALTY_INT; }
                else if (d2 <= r_l22) { if (max_penalty < INFLATE_L2_PENALTY_INT) max_penalty = INFLATE_L2_PENALTY_INT; }
                else if (d2 <= r_l32) { if (max_penalty < INFLATE_L3_PENALTY_INT) max_penalty = INFLATE_L3_PENALTY_INT; }
            }
        }
    }
    return max_penalty;
}

// ---------------- A* 算法主干 ----------------
int astar_plan(ServerMap* map, Point2D start, Point2D goal, Point2D* prev_path, int prev_path_len, bool is_return, Point2D* out_path) {
    failure_reason=NULL;
    if(!map || !map->grid || !out_path || map->grid_size<=0 ||
       map->grid_size>MAX_GRID_SIZE || !isfinite(map->res) || map->res<=0 ||
       !isfinite(start.x)||!isfinite(start.y)||!isfinite(goal.x)||!isfinite(goal.y))
        return failed("INVALID_MAP_INPUT");
    float grid_res = map->res;
    float inv_res = 1.0f / grid_res; // 全局使用浮点乘法

    int start_grid_x = (int)floorf(start.x * inv_res);
    int start_grid_y = (int)floorf(start.y * inv_res);
    int goal_grid_x = (int)floorf(goal.x * inv_res);
    int goal_grid_y = (int)floorf(goal.y * inv_res);

    if (start_grid_x < 0 || start_grid_x >= map->grid_size || start_grid_y < 0 || start_grid_y >= map->grid_size ||
        goal_grid_x < 0 || goal_grid_x >= map->grid_size || goal_grid_y < 0 || goal_grid_y >= map->grid_size) {
        return failed("MAP_BOUNDS");
    }

    /* Upstream NoeticMaze does not abort upfront if the robot is momentarily
       grazing within wall inflation; allow escaping out of the start cell as
       long as the robot's cell itself is not a hard wall. For non-return goals,
       keep the goal clearance check so targets placed inside walls are flagged. */
    if (get_grid_val(map, start_grid_x, start_grid_y) == 1) return failed("START_BLOCKED");
    if (!is_return && check_collision(map,goal,PHYSICAL_RADIUS,is_return)) return failed("GOAL_BLOCKED");
    if (is_return && get_grid_val(map, goal_grid_x, goal_grid_y) == 1) return failed("GOAL_BLOCKED");
    bool start_in_clearance = check_collision(map, start, PHYSICAL_RADIUS, is_return);
    int num_cells = map->grid_size * map->grid_size;

    // 【核心优化 2】：利用 32 位总线宽度成倍加速 16 位数组的初始化
    memset(s_g_score, 0xff, sizeof(s_g_score));

    memset(s_came_from_dir, 255, (num_cells+1)/2);
    clear_all_affinity(map);

    if (prev_path && prev_path_len > 0) {
        // 【修复】：用实际配置的米换算成当下的动态网格数
        int r = (int)(PATH_AFFINITY_RADIUS_M * inv_res);
        int r_sq = r * r;
        for (int i = 0; i < prev_path_len; i++) {
            int px = (int)(prev_path[i].x * inv_res);
            int py = (int)(prev_path[i].y * inv_res);
            for(int dx = -r; dx <= r; dx++) {
                for(int dy = -r; dy <= r; dy++) {
                    if (dx*dx + dy*dy <= r_sq) {
                        int nx = px + dx, ny = py + dy;
                        if (nx >= 0 && nx < map->grid_size && ny >= 0 && ny < map->grid_size) {
                            set_affinity(map, nx * map->grid_size + ny);
                        }
                    }
                }
            }
        }
    }

    // 【核心优化 3】：在外部一次性计算所有碰撞与膨胀的精确网格平方半径，彻底剔除内部 Float 计算
    int r_phys_g = 0, r_l3_g = 0;
    int r_phys2 = radius_to_grid_sq(PHYSICAL_RADIUS, inv_res, &r_phys_g);
    int r_l12   = radius_to_grid_sq(INFLATE_L1_RAD, inv_res, NULL);
    int r_l22   = radius_to_grid_sq(INFLATE_L2_RAD, inv_res, NULL);
    int r_l32   = radius_to_grid_sq(INFLATE_L3_RAD, inv_res, &r_l3_g);
    int max_r   = r_l3_g;

    int search_start_x = goal_grid_x;
    int search_start_y = goal_grid_y;
    int search_target_x = start_grid_x;
    int search_target_y = start_grid_y;

    PriorityQueue pq;
    pq_init(&pq);

    int search_start_idx = search_start_x * map->grid_size + search_start_y;
    s_g_score[search_start_idx] = 0;
    if (!pq_push(&pq, search_start_idx, 0)) {
        return failed("PLANNER_QUEUE_FULL");
    }

    int dx[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    int dy[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    uint16_t move_costs[8] = { STEP_STRAIGHT, STEP_STRAIGHT, STEP_STRAIGHT, STEP_STRAIGHT, STEP_DIAG, STEP_DIAG, STEP_DIAG, STEP_DIAG };

    uint16_t curr_idx, curr_f;
    int path_len = 0;

    while (pq_pop(&pq, &curr_idx, &curr_f)) {
        int curr_x = curr_idx / map->grid_size;
        int curr_y = curr_idx % map->grid_size;

        if (abs(curr_x - search_target_x) <= ASTAR_GOAL_TOL_GRIDS && abs(curr_y - search_target_y) <= ASTAR_GOAL_TOL_GRIDS) {
            int cx = curr_x, cy = curr_y;

            while(predecessor(cx * map->grid_size + cy) != 15) {
                if (path_len >= MAX_PATH_LEN - 1) return failed("PATH_TOO_LONG");
                out_path[path_len].x = (cx + 0.5f) * grid_res;
                out_path[path_len].y = (cy + 0.5f) * grid_res;
                path_len++;
                uint8_t dir = predecessor(cx * map->grid_size + cy);
                cx -= dx[dir]; cy -= dy[dir];
            }
            if (path_len < MAX_PATH_LEN && cx == search_start_x && cy == search_start_y) {
                out_path[path_len] = goal;
                path_len++;
            }
            if (path_len == 1) { out_path[1] = goal; path_len = 2; }
            if (path_len > 0) out_path[0] = start;
            break;
        }

        for (int i=0; i<8; i++) {
            int nx = curr_x + dx[i], ny = curr_y + dy[i];

            // 【修复 1】：第一步先做严格的地图边界检查，防止生成非法的 n_idx 导致内存越界
            if (nx < 0 || nx >= map->grid_size || ny < 0 || ny >= map->grid_size) {
                continue;
            }

            /* Reject diagonal corner cutting before accepting a predecessor.
               Otherwise the final segment validator can reject a route even
               when a legal orthogonal route exists. */
            if(i>=4 &&
               (evaluate_node_cost(map,curr_x+dx[i],curr_y,is_return,r_phys2,r_l12,r_l22,r_l32,max_r)==COLLISION_COST ||
                evaluate_node_cost(map,curr_x,curr_y+dy[i],is_return,r_phys2,r_l12,r_l22,r_l32,max_r)==COLLISION_COST))
                continue;
            // 剔除邻居越界检查（已并入 evaluate_node_cost 处理）
            uint16_t penalty = evaluate_node_cost(map, nx, ny, is_return, r_phys2, r_l12, r_l22, r_l32, max_r);
            if (penalty == COLLISION_COST) {
                /* Allow escaping out of a grazed start cell into open corridor. */
                int d_start_x = abs(nx - search_target_x);
                int d_start_y = abs(ny - search_target_y);
                bool near_start = start_in_clearance && d_start_x <= r_phys_g && d_start_y <= r_phys_g &&
                                  get_grid_val(map, nx, ny) != 1;
                if (!near_start) continue;
                penalty = INFLATE_L1_PENALTY_INT;
            }

            int n_idx = nx * map->grid_size + ny;
            uint16_t discount = has_affinity(map, n_idx) ? AFFINITY_DISCOUNT_INT : 0;
            int32_t step_cost_calc = (int32_t)move_costs[i] + penalty - discount;
            uint16_t actual_step_cost = (step_cost_calc < 1) ? 1 : (uint16_t)step_cost_calc;

            uint32_t tg_32 = (uint32_t)s_g_score[curr_idx] + actual_step_cost;
            uint16_t tg = (tg_32 > MAX_COST_INF) ? MAX_COST_INF : (uint16_t)tg_32;

            if (tg < s_g_score[n_idx]) {
                set_predecessor(n_idx, (uint8_t)i);
                s_g_score[n_idx] = tg;

                // 【核心优化 4】：无分支（Branchless）启发式函数计算，避免流水线清空
                int h_dx = abs(nx - search_target_x);
                int h_dy = abs(ny - search_target_y);
                int min_d = (h_dx < h_dy) ? h_dx : h_dy;
                int max_d = (h_dx > h_dy) ? h_dx : h_dy;
                uint16_t h = 14 * min_d + 10 * (max_d - min_d);

                if (!pq_push(&pq, n_idx, (uint16_t)fminf(65535.0f, (float)tg + h + 1))) {
                    return failed("PLANNER_QUEUE_FULL");
                }
            }
        }
    }

    if (pq.overflowed) return failed("PLANNER_QUEUE_FULL");
    if (!path_len) return failed("NO_ROUTE");
    return path_len;
}

#endif
