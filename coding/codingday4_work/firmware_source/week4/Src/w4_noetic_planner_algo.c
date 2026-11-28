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

/* Bounded sparse A*: full 10.65 m map without a 90 KB dense cost array.
   Slot packs cell index (low 16) and g cost (high 16); 0xffff means unused.
   Scratch is shared only with scan flags in the serial W4PathTask. */
uint32_t w4_planner_nodes[PLANNER_NODE_SLOTS];
static uint8_t directions[(PLANNER_NODE_SLOTS+1)/2];
/* Coarse 4x4-cell (20 cm) obstacle presence bitmap: lets evaluate_node_cost()
   skip empty open space in ~9 bit-checks instead of scanning 13x13=169 cells. */
enum { BLOCK_SHIFT=2, BLOCK_DIM=(MAX_GRID_SIZE+3)>>BLOCK_SHIFT, BLOCK_BYTES=(BLOCK_DIM*BLOCK_DIM+7)/8 };
static uint8_t blocked_blocks[BLOCK_BYTES];
static int s_block_dim;
static inline bool block_has_obstacle(int bx,int by) {
    unsigned idx=(unsigned)(bx*s_block_dim+by);
    return (blocked_blocks[idx>>3] & (1U<<(idx&7U)))!=0;
}
static void build_blocked_blocks(ServerMap *map,bool is_return) {
    int gs=map->grid_size;
    s_block_dim=(gs+3)>>BLOCK_SHIFT;
    int bytes=(s_block_dim*s_block_dim+7)/8;
    memset(blocked_blocks,0,(size_t)bytes);
    for(int x=0;x<gs;++x) {
        int bx=x>>BLOCK_SHIFT;
        int row_base=x*gs;
        for(int y=0;y<gs;++y) {
            uint8_t s=map_state(map,row_base+y);
            if(s>=4 || (is_return && s!=1)) {
                unsigned bidx=(unsigned)(bx*s_block_dim+(y>>BLOCK_SHIFT));
                blocked_blocks[bidx>>3]|=(uint8_t)(1U<<(bidx&7U));
            }
        }
    }
}
static int node_slot(unsigned idx,bool create) {
    unsigned slot=(idx*2654435761U)%PLANNER_NODE_SLOTS;
    for(unsigned n=0;n<PLANNER_NODE_SLOTS;++n) {
        unsigned key=w4_planner_nodes[slot]&65535U;
        if(key==idx) return (int)slot;
        if(key==65535U) {
            if(create) w4_planner_nodes[slot]=0xffff0000U|idx;
            return create?(int)slot:-1;
        }
        if(++slot==PLANNER_NODE_SLOTS) slot=0;
    }
    return -1;
}
static unsigned node_cost(int slot) { return slot<0?65535U:w4_planner_nodes[slot]>>16; }
static void node_set_cost(int slot,unsigned cost) {
    w4_planner_nodes[slot]=(w4_planner_nodes[slot]&65535U)|(cost<<16);
}
static uint8_t predecessor(int i) {
    int slot=node_slot((unsigned)i,false);
    return slot<0?15:(directions[slot/2]>>((slot&1)*4))&15U;
}
static void set_predecessor(int slot,uint8_t dir) {
    unsigned shift=(slot&1)*4;
    directions[slot/2]=(directions[slot/2]&~(15U<<shift))|(dir<<shift);
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
    if(!map || !map->grid || !isfinite(pos.x) || !isfinite(pos.y) ||
       !isfinite(radius) || radius<0 || map->res<=0) return true;
    float inv_res=1.0f/map->res, rsq=radius*radius;
    int cx=(int)floorf(pos.x*inv_res),cy=(int)floorf(pos.y*inv_res);
    if(cx<0 || cy<0 || cx>=map->grid_size || cy>=map->grid_size) return true;
    /* Actual sub-cell position, not a rounded target cell. Wall samples remain
       at cell centres; do not add an extra entire cell of target clearance. */
    int reach=(int)ceilf(radius*inv_res)+1;
    for(int x=cx-reach;x<=cx+reach;++x) for(int y=cy-reach;y<=cy+reach;++y) {
        float dx=(x+0.5f)*map->res-pos.x,dy=(y+0.5f)*map->res-pos.y;
        if(dx*dx+dy*dy>rsq+1e-7f) continue;
        if(x<0 || y<0 || x>=map->grid_size || y>=map->grid_size) return true;
        uint8_t v=get_grid_val(map,x,y);
        if(v==1 || (is_return && v==0)) return true;
    }
    return get_grid_val(map,cx,cy)==1;
}

/* Fast hard-only collision check using the coarse 4x4 obstacle bitmap
   (used for diagonal corner-cutting prevention instead of full L3 scan). */
static inline bool is_hard_blocked(ServerMap *map,int cx,int cy,bool is_return,int r_phys2,int r_phys_g) {
    int gs=map->grid_size;
    if(cx<r_phys_g || cx>=gs-r_phys_g || cy<r_phys_g || cy>=gs-r_phys_g) {
        for(int i=-r_phys_g;i<=r_phys_g;++i) {
            for(int j=-r_phys_g;j<=r_phys_g;++j) {
                if(i*i+j*j>r_phys2) continue;
                int nx=cx+i,ny=cy+j;
                if(nx<0 || nx>=gs || ny<0 || ny>=gs) return true;
                uint8_t s=map_state(map,nx*gs+ny);
                if(s>=4 || (is_return && s!=1)) return true;
            }
        }
        return false;
    }
    int x_min=cx-r_phys_g, x_max=cx+r_phys_g;
    int y_min=cy-r_phys_g, y_max=cy+r_phys_g;
    int bx0=x_min>>BLOCK_SHIFT, bx1=x_max>>BLOCK_SHIFT;
    int by0=y_min>>BLOCK_SHIFT, by1=y_max>>BLOCK_SHIFT;
    for(int bx=bx0;bx<=bx1;++bx) {
        for(int by=by0;by<=by1;++by) {
            if(!block_has_obstacle(bx,by)) continue;
            int x0=bx<<BLOCK_SHIFT; if(x0<x_min) x0=x_min;
            int x1=((bx+1)<<BLOCK_SHIFT)-1; if(x1>x_max) x1=x_max;
            int y0=by<<BLOCK_SHIFT; if(y0<y_min) y0=y_min;
            int y1=((by+1)<<BLOCK_SHIFT)-1; if(y1>y_max) y1=y_max;
            for(int nx=x0;nx<=x1;++nx) {
                int di=nx-cx, di2=di*di, row_base=nx*gs;
                for(int ny=y0;ny<=y1;++ny) {
                    int dj=ny-cy;
                    if(di2+dj*dj>r_phys2) continue;
                    uint8_t s=map_state(map,row_base+ny);
                    if(s>=4 || (is_return && s!=1)) return true;
                }
            }
        }
    }
    return false;
}

// 【核心优化 1】：粗块位图加速的单次节点代价评估（空旷区 ~9 次位检查直接返回 0）
static inline uint16_t evaluate_node_cost(ServerMap* map, int cx, int cy, bool is_return,
                                          int r_phys2, int r_l12, int r_l22, int r_l32, int max_r) {
    int gs = map->grid_size;
    int x_min = cx - max_r, x_max = cx + max_r;
    int y_min = cy - max_r, y_max = cy + max_r;

    /* Map border within hard clearance radius is an immediate collision. */
    if (cx < max_r || cx >= gs - max_r || cy < max_r || cy >= gs - max_r) {
        if (cx * cx <= r_phys2 || (gs - 1 - cx) * (gs - 1 - cx) <= r_phys2 ||
            cy * cy <= r_phys2 || (gs - 1 - cy) * (gs - 1 - cy) <= r_phys2) {
            int dx_edge = (cx < gs - 1 - cx) ? (cx + 1) : (gs - cx);
            int dy_edge = (cy < gs - 1 - cy) ? (cy + 1) : (gs - cy);
            if (dx_edge * dx_edge <= r_phys2 || dy_edge * dy_edge <= r_phys2) return COLLISION_COST;
        }
        if (x_min < 0) x_min = 0;
        if (x_max >= gs) x_max = gs - 1;
        if (y_min < 0) y_min = 0;
        if (y_max >= gs) y_max = gs - 1;
    }

    int bx0 = x_min >> BLOCK_SHIFT, bx1 = x_max >> BLOCK_SHIFT;
    int by0 = y_min >> BLOCK_SHIFT, by1 = y_max >> BLOCK_SHIFT;
    uint16_t max_penalty = 0;

    for (int bx = bx0; bx <= bx1; ++bx) {
        for (int by = by0; by <= by1; ++by) {
            if (!block_has_obstacle(bx, by)) continue;
            int x0 = bx << BLOCK_SHIFT; if (x0 < x_min) x0 = x_min;
            int x1 = ((bx + 1) << BLOCK_SHIFT) - 1; if (x1 > x_max) x1 = x_max;
            int y0 = by << BLOCK_SHIFT; if (y0 < y_min) y0 = y_min;
            int y1 = ((by + 1) << BLOCK_SHIFT) - 1; if (y1 > y_max) y1 = y_max;
            for (int nx = x0; nx <= x1; ++nx) {
                int di = nx - cx, di2 = di * di, row_base = nx * gs;
                for (int ny = y0; ny <= y1; ++ny) {
                    int dj = ny - cy;
                    int d2 = di2 + dj * dj;
                    if (d2 > r_l32) continue;
                    uint8_t s = map_state(map, row_base + ny);
                    if (s >= 4 || (is_return && s != 1)) {
                        if (d2 <= r_phys2) return COLLISION_COST;
                        else if (d2 <= r_l12) { if (max_penalty < INFLATE_L1_PENALTY_INT) max_penalty = INFLATE_L1_PENALTY_INT; }
                        else if (d2 <= r_l22) { if (max_penalty < INFLATE_L2_PENALTY_INT) max_penalty = INFLATE_L2_PENALTY_INT; }
                        else { if (max_penalty < INFLATE_L3_PENALTY_INT) max_penalty = INFLATE_L3_PENALTY_INT; }
                    }
                }
            }
        }
    }
    return max_penalty;
}

// ---------------- A* 算法主干 ----------------
static int search_window(ServerMap* map, Point2D start, Point2D goal, Point2D* prev_path, int prev_path_len, bool is_return, Point2D* out_path, float margin) {
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
    bool start_in_clearance = check_collision(map, start, PATH_CLEARANCE_RADIUS, is_return);


    // 【核心优化 2】：利用 32 位总线宽度成倍加速 16 位数组的初始化
    memset(w4_planner_nodes,0xff,sizeof w4_planner_nodes);

    memset(directions,255,sizeof directions);
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
    int r_phys2 = radius_to_grid_sq(PATH_CLEARANCE_RADIUS, inv_res, &r_phys_g);
    int r_l12   = radius_to_grid_sq(INFLATE_L1_RAD, inv_res, NULL);
    int r_l22   = radius_to_grid_sq(INFLATE_L2_RAD, inv_res, NULL);
    int r_l32   = radius_to_grid_sq(INFLATE_L3_RAD, inv_res, &r_l3_g);
    int max_r   = r_l3_g;

    int pad=(int)ceilf(margin*inv_res);
    int min_x=(start_grid_x<goal_grid_x?start_grid_x:goal_grid_x)-pad;
    int max_x=(start_grid_x>goal_grid_x?start_grid_x:goal_grid_x)+pad;
    int min_y=(start_grid_y<goal_grid_y?start_grid_y:goal_grid_y)-pad;
    int max_y=(start_grid_y>goal_grid_y?start_grid_y:goal_grid_y)+pad;
    int search_start_x = goal_grid_x;
    int search_start_y = goal_grid_y;
    int search_target_x = start_grid_x;
    int search_target_y = start_grid_y;

    PriorityQueue pq;
    pq_init(&pq);

    int search_start_idx = search_start_x * map->grid_size + search_start_y;
    node_set_cost(node_slot((unsigned)search_start_idx,true),0);
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

        /* Skip stale duplicate heap entries so nodes are never re-expanded. */
        unsigned curr_g = node_cost(node_slot(curr_idx, false));
        int ch_dx = abs(curr_x - search_target_x);
        int ch_dy = abs(curr_y - search_target_y);
        int cmin_d = (ch_dx < ch_dy) ? ch_dx : ch_dy;
        int cmax_d = (ch_dx > ch_dy) ? ch_dx : ch_dy;
        uint32_t exp_f = curr_g + (uint32_t)(14 * cmin_d + 10 * (cmax_d - cmin_d)) + 1U;
        if (exp_f > 65535U) exp_f = 65535U;
        if (curr_f > (uint16_t)exp_f) continue;

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

            if(nx<min_x || nx>max_x || ny<min_y || ny>max_y) continue;
            // 【修复 1】：第一步先做严格的地图边界检查，防止生成非法的 n_idx 导致内存越界
            if (nx < 0 || nx >= map->grid_size || ny < 0 || ny >= map->grid_size) {
                continue;
            }

            /* Reject diagonal corner cutting before accepting a predecessor.
               Use hard-only radius check (r_phys_g) instead of full L3 scan. */
            if(i>=4 &&
               (is_hard_blocked(map,curr_x+dx[i],curr_y,is_return,r_phys2,r_phys_g) ||
                is_hard_blocked(map,curr_x,curr_y+dy[i],is_return,r_phys2,r_phys_g)))
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

            uint32_t tg_32 = curr_g + actual_step_cost;
            uint16_t tg = (tg_32 > MAX_COST_INF) ? MAX_COST_INF : (uint16_t)tg_32;

            int slot=node_slot((unsigned)n_idx,false);
            if (tg < node_cost(slot)) {
                if(slot<0) slot=node_slot((unsigned)n_idx,true);
                if(slot<0) return failed("PLANNER_NODE_FULL");
                set_predecessor(slot,(uint8_t)i);
                node_set_cost(slot,tg);

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

/* Preserve global-route preference first; if capacity is exhausted, try local windows.
   Large soft wall penalties must not exhaust RAM
   exploring the whole map before considering a perfectly safe narrow corridor.
   Widen on failure, never change clearance, occupancy, or the requested goal. */
int astar_plan(ServerMap *map,Point2D start,Point2D goal,Point2D *prev,int prev_len,
               bool returning,Point2D *out) {
    if(map && map->grid && map->grid_size>0 && map->grid_size<=MAX_GRID_SIZE)
        build_blocked_blocks(map,returning);
    const float margins[]={20.0f,0.75f,1.50f};
    for(unsigned i=0;i<sizeof margins/sizeof margins[0];++i) {
        int n=search_window(map,start,goal,prev,prev_len,returning,out,margins[i]);
        if(n || !failure_reason) return n;
        if(strcmp(failure_reason,"NO_ROUTE") && strcmp(failure_reason,"PLANNER_NODE_FULL") &&
           strcmp(failure_reason,"PLANNER_QUEUE_FULL")) return 0;
    }
    return 0;
}
#endif
