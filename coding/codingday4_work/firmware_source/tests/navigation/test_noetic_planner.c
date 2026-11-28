#include "planner_algo.h"
#include "path_smoother.h"
#include "priority_queue.h"
#include <assert.h>
#include <math.h>
#include <string.h>
PlannerMemoryPool g_mem_pool;
static uint8_t cells[MAX_MAP_BYTES];
static Point2D path[MAX_PATH_LEN],smoothed[MAX_PATH_LEN];
static ServerMap map={cells,0.1f,107};
static void clear_map(void) {
    memset(cells,0,sizeof cells);
    for(int x=0;x<map.grid_size;++x) for(int y=0;y<map.grid_size;++y)
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
    for(int x=35;x<map.grid_size;++x) set_grid_val(&map,x,15,1);
    assert(astar_plan(&map,a,b,0,0,false,path)==0);
    /* Exact 14.1 cm (r_sq=2 on 10 cm grid) collision radius:
       orthogonal 10 cm (d^2=1) and diagonal 14.1 cm (d^2=2) must collide,
       while 20 cm orthogonal (d^2=4) and 22.4 cm knight (d^2=5) must be clear. */
    clear_map();
    set_grid_val(&map,20,20,1);
    assert(check_collision(&map,(Point2D){2.15f,2.05f},PHYSICAL_RADIUS,false));
    assert(check_collision(&map,(Point2D){2.15f,2.15f},PHYSICAL_RADIUS,false));
    assert(!check_collision(&map,(Point2D){2.25f,2.05f},PHYSICAL_RADIUS,false));
    assert(!check_collision(&map,(Point2D){2.25f,2.15f},PHYSICAL_RADIUS,false));
    /* Goal 16 cm and 20 cm from a wall must plan & smooth cleanly;
       goal <=14.1 cm from wall must still be rejected as GOAL_BLOCKED. */
    clear_map();
    for(int x=15;x<=25;++x) set_grid_val(&map,x,20,1);
    a=(Point2D){2.05f,1.05f};
    b=(Point2D){2.05f,1.89f}; /* cell (20,18): 16 cm from wall center 2.05 m */
    n=astar_plan(&map,a,b,0,0,false,path);
    validate((unsigned)n,a,b);
    int sn=double_smooth_path(&map,path,n,SAFE_SMOOTH_RAD,false,path);
    int cn=generate_safe_corner_path(&map,path,sn,0.5f,0.05f,false,smoothed);
    assert(cn>=2 && hypotf(smoothed[cn-1].x-b.x,smoothed[cn-1].y-b.y)<0.001f);
    for(int i=1;i<cn;++i)
        assert(!bresenham_line_check(&map,smoothed[i-1],smoothed[i],PHYSICAL_RADIUS,0,false));
    b=(Point2D){2.05f,1.85f}; /* cell (20,18): 20 cm from wall center 2.05 m */
    n=astar_plan(&map,a,b,0,0,false,path);
    validate((unsigned)n,a,b);
    assert(astar_plan(&map,a,(Point2D){2.05f,1.95f},0,0,false,path)==0);
    assert(strcmp(W4Planner_FailureReason(),"GOAL_BLOCKED")==0);
    /* Narrow corridor in a bounded room: walls at x=18 and x=22 (40 cm center-to-center)
       with closed flanks at y=20 must pass strictly through x=20; a 30 cm opening
       (x=19,20 only) in a walled chamber around goal must be rejected as NO_ROUTE. */
    clear_map();
    for(int x=12;x<=28;++x) {
        set_grid_val(&map,x,6,1);
        set_grid_val(&map,x,34,1);
        if(x<19 || x>21) set_grid_val(&map,x,20,1);
    }
    for(int y=6;y<=34;++y) {
        set_grid_val(&map,12,y,1);
        set_grid_val(&map,28,y,1);
    }
    for(int y=16;y<=24;++y) {
        set_grid_val(&map,18,y,1);
        set_grid_val(&map,22,y,1);
    }
    a=(Point2D){2.05f,1.05f}; b=(Point2D){2.05f,3.05f};
    n=astar_plan(&map,a,b,0,0,false,path);
    validate((unsigned)n,a,b);
    for(int i=0;i<n;++i)
        if(path[i].y>=1.7f && path[i].y<=2.4f)
            assert(fabsf(path[i].x-2.05f)<0.11f);
    sn=double_smooth_path(&map,path,n,SAFE_SMOOTH_RAD,false,path);
    cn=generate_safe_corner_path(&map,path,sn,0.5f,0.05f,false,smoothed);
    assert(cn>=2 && hypotf(smoothed[cn-1].x-b.x,smoothed[cn-1].y-b.y)<0.001f);
    for(int i=1;i<cn;++i)
        assert(!bresenham_line_check(&map,smoothed[i-1],smoothed[i],PHYSICAL_RADIUS,0,false));
    clear_map();
    for(int x=16;x<=24;++x) { set_grid_val(&map,x,34,1); if(x!=19 && x!=20) set_grid_val(&map,x,25,1); }
    for(int y=25;y<=34;++y) { set_grid_val(&map,16,y,1); set_grid_val(&map,24,y,1); }
    assert(astar_plan(&map,a,b,0,0,false,path)==0);
    assert(strcmp(W4Planner_FailureReason(),"NO_ROUTE")==0);
    /* 5 cm fine map: sweep all sub-cell phases along a continuous wall.
       A goal 16 cm away remains clear across negative/positive ICP offsets;
       <=14 cm still fails, with the original endpoint preserved. */
    map.res=0.05f; map.grid_size=MAX_GRID_SIZE;
    for(int phase=0;phase<40;++phase) {
        clear_map();
        for(int y=10;y<80;++y) set_grid_val(&map,40,y,1);
        float wall_x=40.5f*map.res;
        a=(Point2D){wall_x-0.8f,1.0f};
        b=(Point2D){wall_x-0.151f-0.001f*phase,2.0f};
        n=astar_plan(&map,a,b,0,0,false,path); validate((unsigned)n,a,b);
        b.x=wall_x-0.14f+0.0002f*phase;
        assert(astar_plan(&map,a,b,0,0,false,path)==0);
        assert(strcmp(W4Planner_FailureReason(),"GOAL_BLOCKED")==0);
    }
    /* Genuine wall in goal cell still blocks. Affinity clearing must preserve
       every evidence state, including wall erosion and tentative hits. */
    for(unsigned state=0;state<=6;++state) {
        map_set_state(&map,123,(uint8_t)state); set_affinity(&map,123);
        clear_all_affinity(&map);
        assert(map_state(&map,123)==state && !has_affinity(&map,123));
    }
    clear_map(); set_grid_val(&map,40,40,1);
    assert(astar_plan(&map,(Point2D){1,1},(Point2D){2.025f,2.025f},0,0,false,path)==0);
    assert(strcmp(W4Planner_FailureReason(),"GOAL_BLOCKED")==0);
    /* Heap capacity fails explicitly and never overwrites the shared pool. */
    PriorityQueue q; pq_init(&q);
    for(unsigned i=0;i<MAX_PQ_SIZE;++i) assert(pq_push(&q,(uint16_t)i,(uint16_t)(MAX_PQ_SIZE-i)));
    assert(!pq_push(&q,0,0) && q.overflowed);
    uint16_t index,cost,previous=0;
    while(pq_pop(&q,&index,&cost)) { assert(cost>=previous); previous=cost; }
    return 0;
}
