#include "planner_algo.h"
#include "path_smoother.h"
#include "priority_queue.h"
#include <assert.h>
#include <math.h>
#include <string.h>
PlannerMemoryPool g_mem_pool;
static uint8_t cells[MAX_MAP_BYTES];
static Point2D path[MAX_PATH_LEN],smoothed[MAX_PATH_LEN];
static ServerMap map={cells,0.1f,MAX_GRID_SIZE};
static void clear_map(void) {
    memset(cells,0,sizeof cells);
    for(int x=0;x<MAX_GRID_SIZE;++x) for(int y=0;y<MAX_GRID_SIZE;++y)
        set_grid_val(&map,x,y,2);
}
static void validate(unsigned n,Point2D start,Point2D goal) {
    assert(n>=2 && n<=MAX_PATH_LEN);
    assert(hypotf(path[0].x-start.x,path[0].y-start.y)<0.0001f);
    assert(hypotf(path[n-1].x-goal.x,path[n-1].y-goal.y)<0.0001f);
    for(unsigned i=1;i<n;++i)
        assert(!bresenham_line_check(&map,path[i-1],path[i],PHYSICAL_RADIUS,0,false));
}
int main(void) {
    clear_map();
    Point2D a={0.85f,0.85f},b={0.86f,0.89f};
    int n=astar_plan(&map,a,b,0,0,false,path);
    validate((unsigned)n,a,b);
    assert(astar_plan(&map,(Point2D){-0.001f,1},b,0,0,false,path)==0);
    assert(astar_plan(&map,(Point2D){NAN,1},b,0,0,false,path)==0);
    /* A <10cm segment is still collision-checked. */
    set_grid_val(&map,10,10,1);
    assert(bresenham_line_check(&map,(Point2D){1.01f,1.01f},(Point2D){1.04f,1.04f},0,0,false));
    assert(check_collision(&map,(Point2D){0.01f,0.01f},PHYSICAL_RADIUS,false));
    assert(generate_safe_corner_path(&map,path,0,0.5f,0.05f,false,smoothed)==0);
    clear_map();
    for(int x=0;x<35;++x) set_grid_val(&map,x,15,1);
    a=(Point2D){2.45f,0.55f}; b=(Point2D){2.45f,2.45f};
    n=astar_plan(&map,a,b,0,0,false,path);
    validate((unsigned)n,a,b);
    for(int x=35;x<MAX_GRID_SIZE;++x) set_grid_val(&map,x,15,1);
    assert(astar_plan(&map,a,b,0,0,false,path)==0);
    /* Heap capacity fails explicitly and never overwrites the shared pool. */
    PriorityQueue q; pq_init(&q);
    for(unsigned i=0;i<MAX_PQ_SIZE;++i) assert(pq_push(&q,(uint16_t)i,(uint16_t)(MAX_PQ_SIZE-i)));
    assert(!pq_push(&q,0,0) && q.overflowed);
    uint16_t index,cost,previous=0;
    while(pq_pop(&q,&index,&cost)) { assert(cost>=previous); previous=cost; }
    return 0;
}
