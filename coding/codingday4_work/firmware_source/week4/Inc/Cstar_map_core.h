//
// Created by lmtgy on 2026/4/15.
// 2. 数据结构与地图层

#ifndef STM32_PLANNER_PC_TEST_MAP_CORE_H
#define STM32_PLANNER_PC_TEST_MAP_CORE_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t* grid;
    float res;
    int grid_size;
} ServerMap;

extern ServerMap g_map;

/* Shared persistent map: x-major, one nibble/cell. Low 3 bits:
   0 unknown, 1 free, 2/3 unconfirmed hits, 4/5/6 confirmed wall
   (two/one/zero clearing scans). Bit 3 is temporary route affinity.
   Planner access never destroys mapping evidence. */
static inline uint8_t map_state(ServerMap *map,int idx) {
    return (map->grid[idx/2]>>((idx&1)*4))&7U;
}
static inline void map_set_state(ServerMap *map,int idx,uint8_t state) {
    unsigned shift=(idx&1)*4;
    map->grid[idx/2]=(map->grid[idx/2]&~(7U<<shift))|((state&7U)<<shift);
}
static inline uint8_t get_grid_val(ServerMap *map,int x,int y) {
    uint8_t s=map_state(map,x*map->grid_size+y);
    return s>=4?1:s==1?2:0;
}
static inline void set_grid_val(ServerMap *map,int x,int y,uint8_t val) {
    map_set_state(map,x*map->grid_size+y,val==1?6:val==2?1:0);
}
static inline bool has_affinity(ServerMap *map,int idx) {
    return (map->grid[idx/2]&(8U<<((idx&1)*4)))!=0;
}
static inline void set_affinity(ServerMap *map,int idx) {
    map->grid[idx/2]|=8U<<((idx&1)*4);
}
static inline void clear_all_affinity(ServerMap *map) {
    for(int i=0;i<(map->grid_size*map->grid_size+1)/2;++i) map->grid[i]&=0x77;
}

/**
 * @brief 将 2cm SLAM 全局图重建为 10cm planner 基础图并写入 planner 写缓冲。
 * @param planner_map_out 输出 planner 基础图视图；可为 NULL。
 */
void process_map_update(ServerMap* planner_map_out);

/**
 * @brief 基于 planner 基础图构建“路径阻断判定专用”的硬膨胀视图。
 * @param src_map 基础 planner 图。
 * @param dst_map 目标 inflated 图；可与基础图不同。
 */
void build_blocked_view_from_base(const ServerMap* src_map, ServerMap* dst_map);

#endif //STM32_PLANNER_PC_TEST_MAP_CORE_H
