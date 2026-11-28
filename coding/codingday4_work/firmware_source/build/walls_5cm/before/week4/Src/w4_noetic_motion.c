#ifdef W4_MAZE
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
#define TRACK_HEADING_KP (w4_config.heading_kp)
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
#define TRACK_DIRECTION_FORWARD 1
#define TRACK_DIRECTION_REVERSE (-1)
#define TRACK_DIRECTION_SWITCH_DIST_M 0.04f
#define TRACK_DIRECTION_HYSTERESIS_DEG 8.0f
#define TRACK_PROJECTION_STALE_LIMIT 5U

/* Week4 control checks the timestamped, calibrated IMU snapshot. */

static Point2D s_motion_path_buffer[MAX_PATH_LEN];
static float s_motion_path_arc_len[MAX_PATH_LEN];
static uint16_t s_motion_path_len = 0U;
static uint32_t s_motion_path_sequence = 0U;
static int s_last_projection_index = 0;
static float s_progress_s_m = 0.0f;
static float s_track_last_linear_cmd_m_s = 0.0f;
static int8_t s_track_direction = TRACK_DIRECTION_FORWARD;
static uint8_t s_projection_stale_count = TRACK_PROJECTION_STALE_LIMIT;

typedef struct {
    Point2D point;
    float progress_s_m;
    float dist_sq_m;
    int segment_index;
    bool valid;
} PathProjectionResult;

typedef struct {
    Point2D lookahead_point;
    Point2D goal_point;
    float goal_dist_m;
    float heading_error_deg;
    float linear_cmd_m_s;
    float yaw_rate_cmd_deg_s;
    bool valid;
    bool goal_reached;
} TrackingCommand;

static PathProjectionResult Motion_Find_Local_Projection(float robot_x, float robot_y);
static PathProjectionResult Motion_Find_Global_Projection(float robot_x, float robot_y);

static Point2D Motion_MapPoint_To_BodyFrame(const RobotState_t* current_robot_state, const Point2D* map_point)
{
    Point2D body_point = {0};
    float dx;
    float dy;
    float cos_theta;
    float sin_theta;

    if ((current_robot_state == NULL) || (map_point == NULL)) {
        return body_point;
    }

    dx = map_point->x - current_robot_state->global_fast_x_m;
    dy = map_point->y - current_robot_state->global_fast_y_m;
    cos_theta = arm_cos_f32(current_robot_state->global_fast_theta_rad);
    sin_theta = arm_sin_f32(current_robot_state->global_fast_theta_rad);

    body_point.x = (cos_theta * dx) + (sin_theta * dy);
    body_point.y = (-sin_theta * dx) + (cos_theta * dy);
    return body_point;
}

static float Motion_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float Motion_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Motion_Distance_Between(const Point2D* a, const Point2D* b)
{
    float distance = 0.0f;

    if ((a == NULL) || (b == NULL)) {
        return 0.0f;
    }

    arm_sqrt_f32(((a->x - b->x) * (a->x - b->x)) + ((a->y - b->y) * (a->y - b->y)), &distance);
    return distance;
}

static float Motion_Apply_Linear_Slew(float desired_linear_m_s)
{
#if TRACK_ENABLE_LINEAR_SLEW
    float max_step = TRACK_LINEAR_ACCEL_LIMIT_M_S2 * motion_dt;
    float delta = desired_linear_m_s - s_track_last_linear_cmd_m_s;

    if (delta > max_step) {
        desired_linear_m_s = s_track_last_linear_cmd_m_s + max_step;
    } else if (delta < -max_step) {
        desired_linear_m_s = s_track_last_linear_cmd_m_s - max_step;
    }
#endif

    s_track_last_linear_cmd_m_s = desired_linear_m_s;
    return desired_linear_m_s;
}

static float Motion_Compute_Turn_Scale(float abs_heading_error_deg)
{
    if (abs_heading_error_deg <= TRACK_TURN_SLOWDOWN_START_DEG) {
        return 1.0f;
    }

    if (abs_heading_error_deg >= TRACK_TURN_SLOWDOWN_STOP_DEG) {
        return 0.0f;
    }

    return (TRACK_TURN_SLOWDOWN_STOP_DEG - abs_heading_error_deg) /
           (TRACK_TURN_SLOWDOWN_STOP_DEG - TRACK_TURN_SLOWDOWN_START_DEG);
}

static float Motion_Compute_Goal_Scale(float goal_dist_m)
{
    if (goal_dist_m >= TRACK_GOAL_SLOWDOWN_DIST_M) {
        return 1.0f;
    }

    if (goal_dist_m <= TRACK_GOAL_STOP_DIST_M) {
        return 0.0f;
    }

    return (goal_dist_m - TRACK_GOAL_STOP_DIST_M) /
           (TRACK_GOAL_SLOWDOWN_DIST_M - TRACK_GOAL_STOP_DIST_M);
}

static Point2D Motion_Select_Lookahead_Point(float current_progress_s)
{
    Point2D selected_point = {0};
    float target_progress_s = current_progress_s + motion_lookahead;

    if (s_motion_path_len == 0U) {
        return selected_point;
    }

    selected_point = s_motion_path_buffer[s_motion_path_len - 1U];
    for (uint16_t i = 0U; i < s_motion_path_len; ++i) {
        if (s_motion_path_arc_len[i] >= target_progress_s) {
            selected_point = s_motion_path_buffer[i];
            break;
        }
    }

    return selected_point;
}

static float Motion_Compute_Heading_Error_For_Direction(const Point2D* lookahead_body_point, int8_t direction)
{
    if (lookahead_body_point == NULL) {
        return 0.0f;
    }

    if (direction == TRACK_DIRECTION_REVERSE) {
        return Robot_RadToDeg(atan2f(-lookahead_body_point->y, -lookahead_body_point->x));
    }

    return Robot_RadToDeg(atan2f(lookahead_body_point->y, lookahead_body_point->x));
}

static void Motion_Update_Track_Direction(const Point2D* lookahead_body_point)
{
    float forward_error;
    float reverse_error;

    if ((lookahead_body_point == NULL) || (s_projection_stale_count != 0U)) {
        return;
    }

    forward_error = Motion_Abs(Motion_Compute_Heading_Error_For_Direction(lookahead_body_point,
                                                                           TRACK_DIRECTION_FORWARD));
    reverse_error = Motion_Abs(Motion_Compute_Heading_Error_For_Direction(lookahead_body_point,
                                                                           TRACK_DIRECTION_REVERSE));

    if (s_track_direction == TRACK_DIRECTION_FORWARD) {
        if ((lookahead_body_point->x < -TRACK_DIRECTION_SWITCH_DIST_M) &&
            ((reverse_error + TRACK_DIRECTION_HYSTERESIS_DEG) < forward_error)) {
            s_track_direction = TRACK_DIRECTION_REVERSE;
        }
    } else {
        if ((lookahead_body_point->x > TRACK_DIRECTION_SWITCH_DIST_M) &&
            ((forward_error + TRACK_DIRECTION_HYSTERESIS_DEG) < reverse_error)) {
            s_track_direction = TRACK_DIRECTION_FORWARD;
        }
    }
}

static void Motion_Mark_Projection_Stale(void)
{
    if (s_projection_stale_count < TRACK_PROJECTION_STALE_LIMIT) {
        s_projection_stale_count++;
    }
}

static TrackingCommand Motion_Build_Tracking_Command(const RobotState_t* current_robot_state)
{
    TrackingCommand cmd = {0};
    Point2D robot_point;
    Point2D lookahead_body_point;
    float abs_heading_error_deg;
    float desired_linear_m_s;
    float direction;

    if ((current_robot_state == NULL) ||
        (s_motion_path_len == 0U) ||
        (s_projection_stale_count >= TRACK_PROJECTION_STALE_LIMIT)) {
        return cmd;
    }

    robot_point.x = current_robot_state->global_fast_x_m;
    robot_point.y = current_robot_state->global_fast_y_m;
    cmd.goal_point = s_motion_path_buffer[s_motion_path_len - 1U];
    cmd.lookahead_point = Motion_Select_Lookahead_Point(s_progress_s_m);
    cmd.goal_dist_m = Motion_Distance_Between(&robot_point, &cmd.goal_point);

    lookahead_body_point = Motion_MapPoint_To_BodyFrame(current_robot_state, &cmd.lookahead_point);
    Motion_Update_Track_Direction(&lookahead_body_point);

    direction = (s_track_direction == TRACK_DIRECTION_REVERSE) ? -1.0f : 1.0f;
    cmd.heading_error_deg = Motion_Compute_Heading_Error_For_Direction(&lookahead_body_point,
                                                                       s_track_direction);

    abs_heading_error_deg = Motion_Abs(cmd.heading_error_deg);

    // 计算角速度（无论是正走还是倒车，算出的有效偏差可以直接乘Kp）
    cmd.yaw_rate_cmd_deg_s = TRACK_HEADING_KP * cmd.heading_error_deg;
    cmd.yaw_rate_cmd_deg_s = Motion_Clamp(cmd.yaw_rate_cmd_deg_s,
                                          -TRACK_MAX_YAW_RATE_DEG_S,
                                          TRACK_MAX_YAW_RATE_DEG_S);

    cmd.goal_reached = (cmd.goal_dist_m <= w4_config.position_tol);

    if (cmd.goal_reached) {
        cmd.linear_cmd_m_s = 0.0f;
        cmd.yaw_rate_cmd_deg_s = 0.0f;
        cmd.valid = true;
        return cmd;
    }

    // 计算线速度标量（基于有效偏差和距离降速）
    desired_linear_m_s = motion_speed_limit;
    desired_linear_m_s *= Motion_Compute_Turn_Scale(abs_heading_error_deg);
    desired_linear_m_s *= Motion_Compute_Goal_Scale(cmd.goal_dist_m);

    if ((desired_linear_m_s > 0.0f) && (desired_linear_m_s < TRACK_MIN_LINEAR_M_S)) {
        desired_linear_m_s = fminf(motion_speed_limit, TRACK_MIN_LINEAR_M_S);
    }

    // 【修改点3】赋予速度方向，并将限幅下界改为允许最大的倒车速度
    desired_linear_m_s *= direction;
    cmd.linear_cmd_m_s = Motion_Clamp(desired_linear_m_s,
                                      -TRACK_MAX_LINEAR_M_S,
                                      TRACK_MAX_LINEAR_M_S);

    // 现有的线性加速度约束平滑函数（Motion_Apply_Linear_Slew）天然兼容正负值过渡，无需修改
    cmd.linear_cmd_m_s = Motion_Apply_Linear_Slew(cmd.linear_cmd_m_s);
    cmd.valid = true;
    return cmd;
}

static void Motion_Reset_Runtime_Path(void)
{
    s_motion_path_len = 0U;
    s_motion_path_sequence = 0U;
    s_last_projection_index = 0;
    s_progress_s_m = 0.0f;
    s_track_last_linear_cmd_m_s = 0.0f;
    s_track_direction = TRACK_DIRECTION_FORWARD;
    s_projection_stale_count = TRACK_PROJECTION_STALE_LIMIT;
    memset(s_motion_path_arc_len, 0, sizeof(s_motion_path_arc_len));

}

static bool Motion_Projection_Is_Usable(const PathProjectionResult* projection)
{
    float max_projection_dist_sq = PATH_PROJECTION_MAX_DIST_M * PATH_PROJECTION_MAX_DIST_M;

    return (projection != NULL) &&
           projection->valid &&
           (projection->dist_sq_m <= max_projection_dist_sq);
}

static void Motion_Load_New_Path(const GlobalPathSnapshot* path_snapshot,
                                 const RobotState_t* current_robot_state)
{
    if ((path_snapshot == NULL) || (path_snapshot->path_ptr == NULL) || (path_snapshot->path_len <= 0)) {
        Motion_Reset_Runtime_Path();
        return;
    }

    uint16_t safe_len = (uint16_t)path_snapshot->path_len;
    if (safe_len > MAX_PATH_LEN) {
        safe_len = MAX_PATH_LEN;
    }

    memcpy(s_motion_path_buffer, path_snapshot->path_ptr, ((size_t)safe_len) * sizeof(Point2D));

    s_motion_path_arc_len[0] = 0.0f;
    for (uint16_t i = 1U; i < safe_len; ++i) {
        float dx = s_motion_path_buffer[i].x - s_motion_path_buffer[i - 1U].x;
        float dy = s_motion_path_buffer[i].y - s_motion_path_buffer[i - 1U].y;
        float seg_len = 0.0f;
        arm_sqrt_f32(dx * dx + dy * dy, &seg_len); // 正确调用：传入指针接收结果
        s_motion_path_arc_len[i] = s_motion_path_arc_len[i - 1U] + seg_len;
    }

    s_motion_path_len = safe_len;
    s_motion_path_sequence = path_snapshot->sequence;
    s_last_projection_index = 0;
    s_projection_stale_count = TRACK_PROJECTION_STALE_LIMIT;

    if (current_robot_state != NULL) {
        PathProjectionResult projection = Motion_Find_Global_Projection(current_robot_state->global_fast_x_m,
                                                                        current_robot_state->global_fast_y_m);
        if (Motion_Projection_Is_Usable(&projection)) {
            s_progress_s_m = projection.progress_s_m;
            s_last_projection_index = projection.segment_index;
            s_projection_stale_count = 0U;
        } else {
            s_progress_s_m = 0.0f;
            s_track_direction = TRACK_DIRECTION_FORWARD;
            (void)Motion_Apply_Linear_Slew(0.0f);
        }
    } else {
        s_progress_s_m = 0.0f;
        s_track_direction = TRACK_DIRECTION_FORWARD;
        (void)Motion_Apply_Linear_Slew(0.0f);
    }

}

static PathProjectionResult Motion_Project_On_Segment(const Point2D* p0,
                                                      const Point2D* p1,
                                                      float base_s,
                                                      float robot_x,
                                                      float robot_y,
                                                      int segment_index)
{
    PathProjectionResult result = {0};
    float vx = p1->x - p0->x;
    float vy = p1->y - p0->y;
    float seg_len_sq = vx * vx + vy * vy;

    if (seg_len_sq < (PATH_MIN_SEG_LEN_M * PATH_MIN_SEG_LEN_M)) {
        float dx0 = robot_x - p0->x;
        float dy0 = robot_y - p0->y;
        result.point = *p0;
        result.progress_s_m = base_s;
        result.dist_sq_m = dx0 * dx0 + dy0 * dy0;
        result.segment_index = segment_index;
        result.valid = true;
        return result;
    }

    float wx = robot_x - p0->x;
    float wy = robot_y - p0->y;
    float t = (wx * vx + wy * vy) / seg_len_sq;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    result.point.x = p0->x + t * vx;
    result.point.y = p0->y + t * vy;

    float dx = robot_x - result.point.x;
    float dy = robot_y - result.point.y;
    float seg_len = 0.0f;
    arm_sqrt_f32(seg_len_sq, &seg_len); // 正确调用：传入指针接收结果

    result.progress_s_m = base_s + (t * seg_len);
    result.dist_sq_m = dx * dx + dy * dy;
    result.segment_index = segment_index;
    result.valid = true;
    return result;
}

static PathProjectionResult Motion_Find_Local_Projection(float robot_x, float robot_y)
{
    PathProjectionResult best = {0};
    best.dist_sq_m = FLT_MAX;

    if (s_motion_path_len == 0U) {
        return best;
    }

    if (s_motion_path_len == 1U) {
        float dx = robot_x - s_motion_path_buffer[0].x;
        float dy = robot_y - s_motion_path_buffer[0].y;
        best.point = s_motion_path_buffer[0];
        best.progress_s_m = 0.0f;
        best.dist_sq_m = dx * dx + dy * dy;
        best.segment_index = 0;
        best.valid = true;
        return best;
    }

    int max_segment = (int)s_motion_path_len - 2;
    int start_idx = s_last_projection_index - PATH_SEARCH_BACK_WINDOW;
    int end_idx = s_last_projection_index + PATH_SEARCH_FORWARD_WINDOW;

    if (start_idx < 0) {
        start_idx = 0;
    }
    if (end_idx > max_segment) {
        end_idx = max_segment;
    }

    if (start_idx > end_idx) {
        start_idx = 0;
        end_idx = max_segment;
    }

    for (int i = start_idx; i <= end_idx; ++i) {
        PathProjectionResult candidate = Motion_Project_On_Segment(&s_motion_path_buffer[i],
                                                                   &s_motion_path_buffer[i + 1],
                                                                   s_motion_path_arc_len[i],
                                                                   robot_x,
                                                                   robot_y,
                                                                   i);
        if (candidate.valid && candidate.dist_sq_m < best.dist_sq_m) {
            best = candidate;
        }
    }

    if (!best.valid) {
        for (int i = 0; i <= max_segment; ++i) {
            PathProjectionResult candidate = Motion_Project_On_Segment(&s_motion_path_buffer[i],
                                                                       &s_motion_path_buffer[i + 1],
                                                                       s_motion_path_arc_len[i],
                                                                       robot_x,
                                                                       robot_y,
                                                                       i);
            if (candidate.valid && candidate.dist_sq_m < best.dist_sq_m) {
                best = candidate;
            }
        }
    }

    return best;
}

static PathProjectionResult Motion_Find_Global_Projection(float robot_x, float robot_y)
{
    PathProjectionResult best = {0};
    best.dist_sq_m = FLT_MAX;

    if (s_motion_path_len == 0U) {
        return best;
    }

    if (s_motion_path_len == 1U) {
        float dx = robot_x - s_motion_path_buffer[0].x;
        float dy = robot_y - s_motion_path_buffer[0].y;
        best.point = s_motion_path_buffer[0];
        best.progress_s_m = 0.0f;
        best.dist_sq_m = dx * dx + dy * dy;
        best.segment_index = 0;
        best.valid = true;
        return best;
    }

    int max_segment = (int)s_motion_path_len - 2;
    for (int i = 0; i <= max_segment; ++i) {
        PathProjectionResult candidate = Motion_Project_On_Segment(&s_motion_path_buffer[i],
                                                                   &s_motion_path_buffer[i + 1],
                                                                   s_motion_path_arc_len[i],
                                                                   robot_x,
                                                                   robot_y,
                                                                   i);
        if (candidate.valid && candidate.dist_sq_m < best.dist_sq_m) {
            best = candidate;
        }
    }

    return best;
}


void W4Noetic_Reset(void) { Motion_Reset_Runtime_Path(); }
void W4Noetic_Load(const Point2D *path,unsigned count,const W4Pose *pose) {
    RobotState_t r={pose->x_m,pose->y_m,pose->theta_rad};
    GlobalPathSnapshot p={path,(int)count,s_motion_path_sequence+1U};
    Motion_Reset_Runtime_Path();
    Motion_Load_New_Path(&p,&r);
}
void W4Noetic_Swap(const Point2D *path,unsigned count,const W4Pose *pose) {
    /* Seamless hot-swap during motion: preserve linear slew and direction. */
    float prev_v = s_track_last_linear_cmd_m_s;
    int8_t prev_dir = s_track_direction;
    RobotState_t r={pose->x_m,pose->y_m,pose->theta_rad};
    GlobalPathSnapshot p={path,(int)count,s_motion_path_sequence+1U};
    Motion_Load_New_Path(&p,&r);
    s_track_last_linear_cmd_m_s = prev_v;
    s_track_direction = prev_dir;
}
W4NoeticCommand W4Noetic_Step(const W4Pose *pose,float speed,float dt) {
    W4NoeticCommand out={0};
    RobotState_t r={pose->x_m,pose->y_m,pose->theta_rad};
    motion_dt=dt; motion_speed_limit=speed; motion_lookahead=TRACK_LOOKAHEAD_DIST_M;
    if(s_motion_path_len==0U) return out;
    PathProjectionResult p=Motion_Find_Local_Projection(pose->x_m,pose->y_m);
    if(!Motion_Projection_Is_Usable(&p)) {
        /* Try global projection before declaring projection stale (e.g. after hot-swap or turn). */
        PathProjectionResult g=Motion_Find_Global_Projection(pose->x_m,pose->y_m);
        if(Motion_Projection_Is_Usable(&g)) p=g;
    }
    if(!Motion_Projection_Is_Usable(&p)) {
        Motion_Mark_Projection_Stale();
        if(s_projection_stale_count < TRACK_PROJECTION_STALE_LIMIT) {
            /* Match upstream NoeticMaze: slew smoothly while waiting for projection/replan. */
            out.valid = true;
            out.reached = false;
            out.v = Motion_Apply_Linear_Slew(0.0f);
            out.w = 0.0f;
            out.index = (unsigned)s_last_projection_index;
            out.lookahead = s_motion_path_buffer[s_last_projection_index < (int)s_motion_path_len ? s_last_projection_index : (int)s_motion_path_len - 1];
        }
        return out;
    }
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
bool W4Noetic_OffPath(void) { return s_projection_stale_count > 0U; }
#endif
