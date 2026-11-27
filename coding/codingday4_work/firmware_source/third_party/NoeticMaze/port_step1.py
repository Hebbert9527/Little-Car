from pathlib import Path
import shutil,re,hashlib,json
R=Path(r"D:\EMBEGGed\week4codingday\coding\coding(1)\coding\codingday4_work\firmware_source")
U=R/"third_party/NoeticMaze"
B=R/"build/noetic_port/before"
B.mkdir(parents=True,exist_ok=True)
protected=["week4/Src/w4_motor.c","week4/Src/w4_calibration.c","week4/Src/w4_port_stm32.c","week4/Src/w4_common.c","week4/Src/w4_wheel_speed.c","week4/Inc/w4_contract.h","Core/Src/main.c","Core/Src/week3_board.c"]
manifest={p:hashlib.sha256((R/p).read_bytes()).hexdigest() for p in protected if (R/p).exists()}
(R/"build/noetic_port/protected_sha256.json").write_text(json.dumps(manifest,indent=2))
for p in ["week4/Src/w4_control.c","week4/Src/w4_maze.c","week4/Inc/w4_maze.h","tools/build.py","tests/navigation/test_maze.c","../run_cortex_tests.py"]:
    target=B/Path(p).name
    if not target.exists(): shutil.copy2(R/p,target)
def write(p,s): (R/p).write_text(s,encoding="utf-8",newline="\n")
for name in ["planner_config.h","Cstar_map_core.h","path_smoother.h","planner_algo.h","priority_queue.h"]:
    s=(U/"App/Planner/Inc"/name).read_text()
    if name=="planner_config.h":
        s=s.replace("#define SERIAL_PORT", "#include <stdint.h>\n#define SERIAL_PORT")
        for macro,val in {"MAX_GRID_SIZE":"49","ASTAR_GOAL_TOL_GRIDS":"0","PHYSICAL_RADIUS":"0.20f","SAFE_SMOOTH_RAD":"0.20f","INFLATE_L1_RAD":"0.25f","INFLATE_L2_RAD":"0.30f","INFLATE_L3_RAD":"0.35f"}.items():
            s=re.sub(r"(?m)^#define "+macro+r"\s+\S+", "#define "+macro+" "+val,s)
    if name=="Cstar_map_core.h":
        a=s.index("    uint32_t* grid32")
        b=s.index("\n}",a)
        s=s[:a]+"    for (int i = 0; i < bytes; ++i) map->grid[i] &= 0x33;\n"+s[b:]
    write("week4/Inc/"+name,s)
for name in ["planner_algo.c","priority_queue.c","path_smoother.c"]:
    s=(U/"App/Planner/Src"/name).read_text()
    if name=="planner_algo.c":
        a=s.index("    uint32_t *p_gscore")
        b=s.index("\n    memset(s_came_from_dir",a)
        s=s[:a]+"    memset(s_g_score, 0xff, sizeof(s_g_score));\n"+s[b:]
        s=s.replace("    int cx = (int)(pos.x * inv_res);","    int cx = (int)floorf(pos.x * inv_res);").replace("    int cy = (int)(pos.y * inv_res);","    int cy = (int)floorf(pos.y * inv_res);")
        s=s.replace("                    if (val == 1 || (is_return && val == 0)) return true;\n                }","                    if (val == 1 || (is_return && val == 0)) return true;\n                } else return true;")
        s=s.replace("    int num_cells = map->grid_size * map->grid_size;","    if (check_collision(map,start,PHYSICAL_RADIUS,is_return) ||\n        check_collision(map,goal,PHYSICAL_RADIUS,is_return)) return 0;\n    int num_cells = map->grid_size * map->grid_size;")
        s=s.replace("cx * grid_res;","(cx + 0.5f) * grid_res;").replace("cy * grid_res;","(cy + 0.5f) * grid_res;")
        s=s.replace("if (path_len >= MAX_PATH_LEN - 1) break;","if (path_len >= MAX_PATH_LEN - 1) return 0;")
        s=s.replace("            if (path_len > 0) out_path[0] = start;","            if (path_len == 1) { out_path[1] = goal; path_len = 2; }\n            if (path_len > 0) out_path[0] = start;")
        s=s.replace("if (!pq_push(&pq, n_idx, tg + h + 1))", "if (!pq_push(&pq, n_idx, (uint16_t)fminf(65535.0f, (float)tg + h + 1)))")
    if name=="path_smoother.c":
        s=s.replace('#include "arm_math.h"',"/* CMSIS square root replaced by the project's libm. */\nstatic void arm_sqrt_f32(float x,float *out) { *out=sqrtf(x); }")
        s=s.replace("    if (dist_sq < 0.01f) return false; // 0.1f * 0.1f","    if (dist_sq < 1e-10f) return check_collision(map,p1,radius,is_return);")
        s=s.replace("int steps = (int)(dist * (2.0f / map->res));","int steps = 1 + (int)ceilf(dist * (4.0f / map->res));")
        s=s.replace("    if (!path || path_len < 3) {","    if (!path || path_len <= 0) return 0;\n    if (path_len < 3) {")
    write("week4/Src/w4_noetic_"+name, '#ifdef W4_MAZE\n/* Adapted from NoeticMaze f869723; MIT, see third_party/NoeticMaze/LICENSE. */\n'+s+"\n#endif\n")
# Extract upstream pure path projection/tracking implementation; adapt only its ports.
s=(U/"App/motor/Src/startMotion.c").read_text()
s=s[s.index("#define PATH_DISPLAY_MAX_POINTS"):s.index("static void Motion_Publish_Trimmed_Path")]
s=re.sub(r"    Debug_Post\(.*?\);\n","",s,flags=re.S)
s=s.replace("    Reset_Trimmed_Path_State();","")
s=re.sub(r"static (Point2D s_trimmed_points\[.*?;|uint32_t s_motion_(stack|trim|projection)_debug_tick = 0U;)\n","",s)
s=s.replace("static uint32_t s_trimmed_tf_version = 0U;\n","").replace("    s_trimmed_tf_version = 0U;\n","")
s=s.replace("float max_step = TRACK_LINEAR_ACCEL_LIMIT_M_S2 * 0.01f;","float max_step = TRACK_LINEAR_ACCEL_LIMIT_M_S2 * motion_dt;")
s=s.replace("desired_linear_m_s = TRACK_CRUISE_LINEAR_M_S;","desired_linear_m_s = motion_speed_limit;")
s=s.replace("        desired_linear_m_s = TRACK_MIN_LINEAR_M_S;","        desired_linear_m_s = fminf(motion_speed_limit, TRACK_MIN_LINEAR_M_S);")
s=s.replace("(cmd.goal_dist_m <= TRACK_GOAL_STOP_DIST_M) &&\n                       (abs_heading_error_deg <= TRACK_HEADING_STOP_DEG)","(cmd.goal_dist_m <= w4_config.position_tol)")
s=s.replace("    float target_progress_s = current_progress_s + TRACK_LOOKAHEAD_DIST_M;","    float target_progress_s = current_progress_s + motion_lookahead;")
s=s.replace("static bool Motion_Projection_Is_Usable","static bool Motion_Projection_Is_Usable")
prefix='''#ifdef W4_MAZE
/* Core functions copied from NoeticMaze startMotion.c (MIT).
   RTOS, robot-state, DSP and motor ports use the existing Week4 interfaces. */
#include "w4_noetic_motion.h"
#include <math.h>
#include <float.h>
#include <string.h>
#define PATH_PROJECTION_MAX_DIST_M 0.22f
#define PATH_PROGRESS_MAX_STEP_M 0.06f
#define PATH_SEARCH_BACK_WINDOW 12
#define PATH_SEARCH_FORWARD_WINDOW 20
#define PATH_MIN_SEG_LEN_M 0.002f
#define TRACK_LOOKAHEAD_DIST_M 0.18f
#define TRACK_HEADING_KP 0.90f
#define TRACK_MAX_YAW_RATE_DEG_S (w4_config.max_w*180.0f/W4_PI)
#define TRACK_TURN_SLOWDOWN_START_DEG 10.0f
#define TRACK_TURN_SLOWDOWN_STOP_DEG 60.0f
#define TRACK_GOAL_SLOWDOWN_DIST_M 0.25f
#define TRACK_GOAL_STOP_DIST_M (w4_config.position_tol)
#define TRACK_MIN_LINEAR_M_S 0.08f
#define TRACK_MAX_LINEAR_M_S (motion_speed_limit)
#define TRACK_ENABLE_LINEAR_SLEW 1
#define TRACK_LINEAR_ACCEL_LIMIT_M_S2 0.45f
#define arm_cos_f32 cosf
#define arm_sin_f32 sinf
#define Robot_RadToDeg(x) ((x)*180.0f/W4_PI)
static void arm_sqrt_f32(float x,float *out) { *out=sqrtf(x); }
typedef struct { float global_fast_x_m,global_fast_y_m,global_fast_theta_rad; } RobotState_t;
typedef struct { const Point2D *path_ptr; int path_len; uint32_t sequence; } GlobalPathSnapshot;
static float motion_dt=0.01f,motion_speed_limit=0.18f,motion_lookahead=0.18f;
'''
tail='''
void W4Noetic_Reset(void) { Motion_Reset_Runtime_Path(); }
void W4Noetic_Load(const Point2D *path,unsigned count,const W4Pose *pose) {
    RobotState_t r={pose->x_m,pose->y_m,pose->theta_rad};
    GlobalPathSnapshot p={path,(int)count,s_motion_path_sequence+1U};
    Motion_Reset_Runtime_Path();
    Motion_Load_New_Path(&p,&r);
}
W4NoeticCommand W4Noetic_Step(const W4Pose *pose,float speed,float dt) {
    W4NoeticCommand out={0};
    RobotState_t r={pose->x_m,pose->y_m,pose->theta_rad};
    motion_dt=dt; motion_speed_limit=speed; motion_lookahead=TRACK_LOOKAHEAD_DIST_M;
    PathProjectionResult p=Motion_Find_Local_Projection(pose->x_m,pose->y_m);
    if(!Motion_Projection_Is_Usable(&p)) { Motion_Mark_Projection_Stale(); return out; }
    s_projection_stale_count=0;
    if(p.progress_s_m>s_progress_s_m)
        s_progress_s_m=fminf(p.progress_s_m,s_progress_s_m+PATH_PROGRESS_MAX_STEP_M);
    /* Do not jump indices ahead of the bounded arc-length progress at U-turns. */
    while(s_last_projection_index+1<(int)s_motion_path_len-1 &&
          s_motion_path_arc_len[s_last_projection_index+1]<=s_progress_s_m)
        ++s_last_projection_index;
    TrackingCommand c=Motion_Build_Tracking_Command(&r);
    out.valid=c.valid; out.reached=c.goal_reached;
    out.v=c.linear_cmd_m_s; out.w=c.yaw_rate_cmd_deg_s*W4_PI/180.0f;
    out.lookahead=c.lookahead_point; out.index=(unsigned)s_last_projection_index;
    return out;
}
unsigned W4Noetic_Index(void) { return (unsigned)s_last_projection_index; }
#endif
'''
write("week4/Src/w4_noetic_motion.c",prefix+s+tail)
write("week4/Inc/w4_noetic_motion.h",'''#ifndef W4_NOETIC_MOTION_H
#define W4_NOETIC_MOTION_H
#include "w4_nav.h"
#include "planner_config.h"
typedef struct { bool valid,reached; float v,w; Point2D lookahead; unsigned index; } W4NoeticCommand;
void W4Noetic_Reset(void);
void W4Noetic_Load(const Point2D *path,unsigned count,const W4Pose *pose);
W4NoeticCommand W4Noetic_Step(const W4Pose *pose,float speed,float dt);
unsigned W4Noetic_Index(void);
#endif
''')
print("Installed pinned planner and motion adaptation; protected parameter hashes saved.")
