#include "w4_nav.h"
#ifdef W4_MAZE
#include "w4_maze.h"
#endif
#include "w4_rtos.h"
#include "w4_runtime.h"
#include <math.h>
#include <string.h>
static W4Waypoint wp;
static bool have_wp,have_time;
static W4Pose reset_pose;
static float reset_offset;
static uint32_t last_ms,missing_since,arrival_since,command_since;
static bool arriving,missing,settling_goal,waypoint_corner,incoming_leg,stopping_at_corner;
typedef enum { ALIGNED, ALIGN_TURN, ALIGN_SETTLE } HeadingAlignment;
static HeadingAlignment alignment;
#ifdef W4_MAZE
static float maze_alignment_heading;
#endif
#ifdef W4_DUAL_ENCODER
static bool after_corner;
#endif
static uint32_t alignment_since,alignment_stop_since,corner_stop_since;
static const float alignment_tolerance_rad=3.0f*W4_PI/180.0f;
static float straight_heading=W4_PI/2;
#ifndef W4_DISABLE_STRAIGHT_CORRECTION
static float straight_start_x,straight_start_y,straight_trim;
#endif
static uint32_t controller_id;
static float segment_x,segment_y,final_heading,forward_speed;
static W4Pid heading_pid;
static float stall_time[2],distance_travelled;
#ifdef W4_REAR_FORWARD
W4StraightCalibration w4_straight_cal={false,{0.3099f,0.3559f}};
volatile float w4_cruise_request;
static float cruise_limit;
static float cruise(float fallback,float dt) {
    float requested=w4_cruise_request;
    if(requested==0.0f) return cruise_limit=fallback;
    if(cruise_limit==0.0f) cruise_limit=requested;
    cruise_limit+=fmaxf(-0.45f*dt,fminf(0.45f*dt,requested-cruise_limit));
    return cruise_limit;
}
static unsigned quiet_samples;
static uint32_t quiet_stamp;
bool W4Control_CalibrationReady(uint32_t now) {
    return quiet_samples>=20 && now-quiet_stamp<=30 && !w4_stop_pending &&
        !w4_reset_pending && w4_nav.state!=W4_TRACKING && w4_nav.state!=W4_PLANNING;
}
#endif
bool W4Nav_ResetPose(float x,float y,float theta) {
    if(!isfinite(x)||!isfinite(y)||!isfinite(theta)||fabsf(x)>2||fabsf(y)>2) return false;
    W4ImuSample imu;
    if(!W4Imu_GetSnapshot(&imu)||!imu.valid||!imu.calibrated||
       W4Port_Millis()-imu.stamp_ms>50||!isfinite(imu.heading_rad)) return false;
    float offset=W4Wrap(theta-imu.heading_rad*w4_config.imu_sign);
    uint32_t k=W4Port_Lock();
    bool ok=!w4_stop_pending&&!w4_reset_pending&&w4_nav.state!=W4_PLANNING&&w4_nav.state!=W4_TRACKING;
    if(ok) { reset_pose=(W4Pose){0,x,y,W4Wrap(theta),1}; reset_offset=offset; w4_reset_pending=true; }
    W4Port_Unlock(k); return ok;
}
static void state(W4State s,uint32_t fault) {
    uint32_t k=W4Port_Lock(); w4_nav.state=s; w4_nav.fault=fault; W4Port_Unlock(k);
}
static void stop_control(void) {
    have_wp=false; arriving=missing=settling_goal=stopping_at_corner=false;
    alignment=ALIGNED; forward_speed=0.0f;
#ifdef W4_DUAL_ENCODER
    after_corner=false;
#endif
#ifdef W4_REAR_FORWARD
    cruise_limit=0.0f;
#endif
#ifndef W4_DISABLE_STRAIGHT_CORRECTION
    straight_trim=0.0f;
#endif
    W4Pid_Reset(&heading_pid); stall_time[0]=stall_time[1]=0; W4Motor_Stop();
}
static void fault(uint32_t code) {
    stop_control();
    uint32_t k=W4Port_Lock();
    w4_nav.pose.valid=0;
    w4_nav.fault=code;
    w4_nav.state=W4_FAULT;
    W4Port_Unlock(k);
}
static void drive_wheels(float l,float r,float vl,float vr,float dt,uint32_t now,
                         uint32_t begin,bool fixed_distance) {
    float req[2]={l,r},meas[2]={vl,vr};
    for(unsigned j=0;j<2;++j) {
#ifdef W4_SINGLE_ENCODER
        if(j==0) continue; /* Left speed is inferred, never independent stall evidence. */
#endif
        stall_time[j]=(fabsf(req[j])>0.03f&&fabsf(meas[j])<0.003f)?stall_time[j]+dt:0;
        if(stall_time[j]>1.0f) { fault(5); return; }
    }
    w4_stats.tracking_us=W4Port_Micros()-begin-w4_stats.pose_us;
    uint32_t motor_begin=W4Port_Micros();
    /* STOP may arrive during the computation. Commit PWM atomically with the
       final latch check, so a completed STOP cannot be followed by new PWM. */
    W4Motor_SetWheelSpeeds(l,r);
    W4Motor_Update(vl,vr,dt,now,fixed_distance);
    w4_stats.motor_us=W4Port_Micros()-motor_begin;
}
static void begin_alignment(uint32_t now) {
    alignment=ALIGN_TURN;
    alignment_since=now;
    forward_speed=0.0f;
    W4Pid_Reset(&heading_pid);
    W4Motor_Stop();
}
static bool wheels_still(float left,float right) {
    return fabsf(left)<=0.02f&&fabsf(right)<=0.02f;
}
static bool align_heading(float error,float vl,float vr,float dt,uint32_t now,uint32_t begin) {
    if(now-alignment_since>=6000U) { fault(8); return false; }
    if(alignment==ALIGN_TURN) {
        if(fabsf(error)>alignment_tolerance_rad) {
            float limit=w4_config.max_w*w4_config.track_m*0.5f;
#ifdef W4_REAR_FORWARD
            /* Faster coarse corner pivot; taper with the remaining angle.
               Initial alignment still uses the lower angular limit. */
            if(after_corner) limit=fminf(w4_config.max_wheel,1.55f*limit);
#endif
            float speed=copysignf(fminf(limit,fmaxf(0.035f,0.22f*fabsf(error))),error);
            drive_wheels(-speed,speed,vl,vr,dt,now,begin,false);
            return false;
        }
        W4Motor_Stop();
        stall_time[0]=stall_time[1]=0.0f;
        alignment_stop_since=now;
        alignment=ALIGN_SETTLE;
        return false;
    }
    if(!wheels_still(vl,vr)) alignment_stop_since=now;
    if(now-alignment_stop_since<100U) return false;
    if(fabsf(error)>alignment_tolerance_rad) {
        alignment=ALIGN_TURN;
        return false;
    }
    alignment=ALIGNED;
    return true;
}
#ifndef W4_DISABLE_STRAIGHT_CORRECTION
static float straight_correction(const W4Pose *pose,float heading,float lateral,float speed,float dt) {
    /* Aim gently back toward the starting line, with a 5 mm deadband. */
    float lateral_error=copysignf(fmaxf(0.0f,fabsf(lateral)-0.005f),lateral);
    float return_angle=atan2f(lateral_error,0.60f);
    float desired=heading-fmaxf(-0.175f,fminf(0.175f,return_angle));
    float error=W4Wrap(desired-pose->theta_rad);
    error=copysignf(fmaxf(0.0f,fabsf(error)-0.014f),error);

    /* Preserve the requested mean speed and keep both wheels forward. */
    float limit=fmaxf(0.0f,fminf(0.025f,fminf(speed*0.1f,w4_config.max_wheel-speed)));
    float wanted=fmaxf(-limit,fminf(limit,0.20f*error));
    float step=0.06f*dt;
    straight_trim+=fmaxf(-step,fminf(step,wanted-straight_trim));
    return straight_trim;
}
#endif
static void control_step(uint32_t now) {
    uint32_t begin=W4Port_Micros(),k=W4Port_Lock();
    bool stopping=w4_stop_pending;
    if(stopping) {
        w4_control_stop_epoch=w4_stop_epoch; w4_nav.state=W4_STOPPED;
        w4_nav.cmd_id=0; w4_reset_pending=false;
    }
    bool resetting=w4_reset_pending&&!stopping;
    if(resetting) {
#ifdef W4_MAZE
        W4Maze_Reset();
#endif
        w4_nav.pose=reset_pose; w4_nav.pose.stamp_ms=now;
        w4_config.imu_offset_rad=reset_offset;
        straight_heading=reset_pose.theta_rad;
        w4_nav.state=W4_STOPPED; w4_nav.fault=0; w4_nav.cmd_id=0;
        w4_nav.target_x=reset_pose.x_m; w4_nav.target_y=reset_pose.y_m;
        w4_nav.wp_index=0; w4_reset_pending=false; have_time=false;
    }
    W4Port_Unlock(k);
    if(stopping) { stop_control(); last_ms=now; have_time=true; return; }
    W4EncoderDelta e; W4Encoder_TakeDelta(&e);
    /* Encoder acquisition may straddle the HAL millisecond tick. Use a time
       captured after the snapshot for freshness and this cycle's dt. */
    now=W4Port_Millis();
    /* Drain pre-reset edge counts, without integrating them in the new frame. */
    if(resetting) { stop_control(); last_ms=now; have_time=true; return; }
    float dt=have_time?(now-last_ms)*0.001f:0.01f; last_ms=now; have_time=true;
    float vl=0,vr=0;
    bool sensors=W4Pose_Update(now,&e,dt,&vl,&vr);
    w4_stats.pose_us=W4Port_Micros()-begin;
    W4NavSnapshot n; W4Nav_GetSnapshot(&n);
    bool active=n.state==W4_PLANNING||n.state==W4_TRACKING;
#ifdef W4_REAR_FORWARD
    k=W4Port_Lock();
    if(!active && sensors && e.left_delta==0 && e.right_delta==0) {
        if(now-quiet_stamp>30) quiet_samples=0;
        if(quiet_samples<20) ++quiet_samples;
        quiet_stamp=now;
    } else quiet_samples=0;
    W4Port_Unlock(k);
#endif
    if(!active) { stop_control(); return; }
    if(controller_id!=n.cmd_id) {
        stop_control(); controller_id=n.cmd_id; command_since=now;
        segment_x=n.pose.x_m; segment_y=n.pose.y_m; final_heading=n.pose.theta_rad;
    }
    if(now-command_since>120000) { fault(2); return; }
#ifdef W4_MAZE
    bool maze=W4Maze_Active(n.cmd_id);
    if(maze && n.state==W4_TRACKING) {
        if(!sensors) { fault(1); return; }
        if(!W4Maze_Fresh(now)) { fault(9); return; }
        if(W4Maze_LegBlocked()) {
            stop_control();
            if(wheels_still(vl,vr)) {
                segment_x=n.pose.x_m; segment_y=n.pose.y_m;
                W4Maze_NextLeg(); state(W4_PLANNING,0);
            }
            return;
        }
    }
#endif
    if(!sensors && n.state==W4_TRACKING) { fault(1); return; }
    bool starting_distance=false;
    if(!have_wp) {
        W4Waypoint next;
        /* bounded stale drain: avoid monopolising a high-priority task */
        for(unsigned j=0;j<64&&osMessageQueueGet(Q_Path,&next,0,0)==osOK;++j) {
            if(next.cmd_id!=n.cmd_id) { ++w4_stats.stale_wp; continue; }
            /* A waypoint for this command proves the planner accepted it.
               Until then it may still reject missing calibration or pose. */
            if(!sensors) { fault(1); return; }
#ifdef W4_MAZE
            if(maze && !W4Maze_Fresh(now)) { fault(9); return; }
#endif
            if(!isfinite(next.x_m)||!isfinite(next.y_m)) { fault(3); return; }
            wp=next; have_wp=true; missing=false;
            if(wp.index==W4_DISTANCE_WAYPOINT) {
                if(wp.y_m<0.0f||wp.y_m>2.0f) { fault(3); return; }
                distance_travelled=0.0f;
                starting_distance=true;
                if(wp.y_m>0.0f&&fabsf(W4Wrap(straight_heading-n.pose.theta_rad))>alignment_tolerance_rad)
                    begin_alignment(now);
                k=W4Port_Lock();
                w4_nav.wp_index=0; w4_nav.state=W4_TRACKING;
                W4Port_Unlock(k);
                break;
            }
            uint16_t type=wp.index;
            waypoint_corner=(type&W4_CORNER_WAYPOINT)!=0;
            incoming_leg=(type&W4_WAYPOINT_INCOMING_LEG)!=0;
            wp.index&=W4_WAYPOINT_INDEX_MASK;
            if(hypotf(wp.x_m-segment_x,wp.y_m-segment_y)>0.0001f||
               (type&(W4_WAYPOINT_AXIS_X|W4_WAYPOINT_AXIS_Y))) {
                bool negative=(type&W4_WAYPOINT_NEGATIVE)!=0;
                float next_heading=atan2f(wp.y_m-segment_y,wp.x_m-segment_x);
                if(type&W4_WAYPOINT_AXIS_Y) next_heading=negative?-W4_PI/2:W4_PI/2;
                if(type&W4_WAYPOINT_AXIS_X) next_heading=negative?W4_PI:0.0f;
                if((wp.index==0&&fabsf(W4Wrap(next_heading-n.pose.theta_rad))>alignment_tolerance_rad)||
                   (wp.index>0&&fabsf(W4Wrap(next_heading-final_heading))>0.15f))
                    begin_alignment(now);
                final_heading=next_heading;
#ifdef W4_MAZE
                if(maze) maze_alignment_heading=next_heading;
#endif
            }
            k=W4Port_Lock(); w4_nav.wp_index=wp.index; w4_nav.state=W4_TRACKING; W4Port_Unlock(k);
            break;
        }
        if(!have_wp) {
            W4Motor_Stop();
            forward_speed=0.0f;
            if(!missing) { missing=true; missing_since=now; }
            if(now-missing_since>1000) fault(4);
            return; /* empty queue is never arrival */
        }
    }
    if(alignment!=ALIGNED) {
        float heading=wp.index==W4_DISTANCE_WAYPOINT?straight_heading:final_heading;
#ifdef W4_MAZE
        if(maze) heading=maze_alignment_heading;
#endif
        if(!align_heading(W4Wrap(heading-n.pose.theta_rad),vl,vr,dt,now,begin)) return;
        starting_distance=true;
    }
    if(wp.index==W4_DISTANCE_WAYPOINT) {
        if(starting_distance) {
#ifndef W4_DISABLE_STRAIGHT_CORRECTION
            straight_start_x=n.pose.x_m;
            straight_start_y=n.pose.y_m;
#endif
            /* Project the displayed target after any initial pivot. */
            float target_x=n.pose.x_m+wp.y_m*cosf(straight_heading);
            float target_y=n.pose.y_m+wp.y_m*sinf(straight_heading);
            k=W4Port_Lock();
            w4_nav.target_x=target_x; w4_nav.target_y=target_y;
            W4Port_Unlock(k);
        } else {
            distance_travelled+=0.5f*(vl+vr)*dt;
        }
        if(distance_travelled>=wp.y_m) {
            stop_control(); state(W4_ARRIVED,0);
            return;
        }
        float speed=w4_config.max_v;
#ifdef W4_REAR_FORWARD
        if(w4_straight_cal.enabled) speed=0.18f;
        speed=cruise(speed,dt);
#endif
        float trim=0.0f;
#ifndef W4_DISABLE_STRAIGHT_CORRECTION
        float lateral=(n.pose.y_m-straight_start_y)*cosf(straight_heading)
                     -(n.pose.x_m-straight_start_x)*sinf(straight_heading);
        trim=straight_correction(&n.pose,straight_heading,lateral,speed,dt);
#endif
        drive_wheels(speed-trim,speed+trim,vl,vr,dt,now,begin,true);
        return;
    }
    float dx=wp.x_m-n.pose.x_m,dy=wp.y_m-n.pose.y_m,d=hypotf(dx,dy);
    float tol=wp.last?w4_config.position_tol:0.008f;
#ifdef W4_MAZE
    /* Short legs need tighter position tracking to preserve obstacle clearance. */
    if(maze && !W4Maze_FinalLeg()) tol=fminf(tol,0.025f);
    if(maze && (d<tol || settling_goal)) {
        /* MAZE specifies a position only. Stop before choosing the next leg,
           without pivoting back to the incoming leg's heading. */
        W4Motor_Stop();
        forward_speed=0.0f;
        W4Pid_Reset(&heading_pid);
        stall_time[0]=stall_time[1]=0.0f;
        if(!settling_goal || !wheels_still(vl,vr)) arrival_since=now;
        settling_goal=true;
        if(now-arrival_since<200U) return;
        if(d>=tol) {
            settling_goal=false;
            return;
        }
        stop_control();
        if(W4Maze_FinalLeg()) state(W4_ARRIVED,0);
        else {
            segment_x=n.pose.x_m; segment_y=n.pose.y_m;
            W4Maze_NextLeg(); state(W4_PLANNING,0);
        }
        return;
    }
#endif
    float desired=atan2f(dy,dx);
    float c=cosf(final_heading),s=sinf(final_heading);
    float goal_dx=n.target_x-wp.x_m,goal_dy=n.target_y-wp.y_m;
    bool final_leg=!incoming_leg;
    bool corner=waypoint_corner;
    float to_waypoint=c*dx+s*dy;
    float cross_track=s*dx-c*dy;
    if(corner) {
        /* Start the stop from the along-leg progress. Lateral error cannot
           veto the corner once the wheels have stopped. */
        if(to_waypoint<=0.015f||stopping_at_corner) {
            W4Motor_Stop();
            forward_speed=0.0f;
            if(!stopping_at_corner||!wheels_still(vl,vr)) corner_stop_since=now;
            stopping_at_corner=true;
            if(now-corner_stop_since<100U) return;
            stopping_at_corner=false;
#ifdef W4_DUAL_ENCODER
            after_corner=true;
#endif
            segment_x=wp.x_m; segment_y=wp.y_m; have_wp=false;
            return;
        }
    }
    /* Increase lookahead during faster travel to avoid hunting across the line. */
    float lookahead=0.20f+0.90f*forward_speed;
    float speed_limit=w4_config.max_v;
#ifdef W4_DUAL_ENCODER
    if(after_corner) {
        /* Leave room for unequal loaded wheels to steer after the pivot.
           Keep the line correction gradual even as the car starts or slows. */
        speed_limit=fminf(speed_limit,0.18f);
        lookahead=fmaxf(lookahead,0.36f);
    }
#endif
    bool recover_point=wp.last&&to_waypoint<0.02f&&d>=tol;
#ifdef W4_REAR_FORWARD
    speed_limit=cruise(speed_limit,dt);
#ifdef W4_MAZE
    if(maze) speed_limit=fminf(speed_limit,0.18f);
#endif
#endif
#ifdef W4_MAZE
    /* Keep the lookahead on the executable segment. An aim beyond a short
       endpoint delays cross-track correction until it becomes a sharp pivot. */
    if(maze) {
        lookahead=fminf(0.18f,to_waypoint);
        desired=W4Wrap(final_heading-atan2f(cross_track,lookahead));
    } else
#endif
    if(!recover_point) desired=W4Wrap(final_heading-atan2f(cross_track,lookahead));
    if(!wp.last&&!corner&&(d<tol||to_waypoint<=tol)) {
        /* Keep the speed loop through collinear samples. The next cycle
           stops for a missing point or a change of segment heading. */
        segment_x=wp.x_m; segment_y=wp.y_m; have_wp=false;
        return;
    }
    if(wp.last) {
        /* Near the position boundary, retain the final heading while the car
           pivots. Switching to the bearing of the goal reverses the pivot. */
        if(!settling_goal&&d<tol) {
            settling_goal=true;
            W4Pid_Reset(&heading_pid);
        } else if(settling_goal&&d>tol+0.015f) {
            settling_goal=arriving=false;
            forward_speed=0.0f;
            W4Motor_Stop();
            W4Pid_Reset(&heading_pid);
            return;
        }
        if(settling_goal) desired=final_heading;
    }
    float error=W4Wrap(desired-n.pose.theta_rad);
#ifdef W4_MAZE
    if(maze && fabsf(error)>0.85f) {
        /* Latch one target through the pivot and its stopped dwell. Re-entry
           requires a large error, while alignment exits near three degrees. */
        maze_alignment_heading=desired;
        begin_alignment(now);
        return;
    }
#endif
    if(settling_goal&&fabsf(error)<w4_config.heading_tol) {
        W4Motor_Stop();
        if(d>=tol) {
            settling_goal=arriving=false;
            forward_speed=0.0f;
            W4Pid_Reset(&heading_pid);
            return;
        }
        if(!arriving) { arriving=true; arrival_since=now; }
        if(now-arrival_since>=200) {
            stop_control();
            state(W4_ARRIVED,0);
        }
        return;
    }
    arriving=false;

    /* The planned legs are orthogonal. Projecting the goal onto the
       current leg gives the next corner or the final goal on that leg. */
    float remaining=wp.last?d:c*(dx+goal_dx)+s*(dy+goal_dy);
    float stop_margin=final_leg?w4_config.position_tol:0.015f;
    /* Coast measurements showed the corner approach needed more room to slow. */
    float braking_gain=final_leg?0.8f:0.30f;
    float braking_speed=sqrtf(braking_gain*fmaxf(0.0f,remaining-stop_margin));
#ifdef W4_MAZE
    if(maze) braking_speed=sqrtf(0.30f*fmaxf(0.0f,remaining-tol));
#endif
    float approach=fminf(speed_limit,fmaxf(0.065f,braking_speed));
    float forward_factor=cosf(fminf(fabsf(error),0.55f));
    float v=(settling_goal||d<tol||(final_leg&&fabsf(error)>0.50f))?
        0.0f:approach*forward_factor;
#ifdef W4_MAZE
    if(maze) {
        float turn_scale=fminf(1.0f,fmaxf(0.0f,(0.85f-fabsf(error))/0.65f));
        v=approach*turn_scale;
        v=forward_speed+fmaxf(-0.60f*dt,fminf(0.45f*dt,v-forward_speed));
    } else
#endif
    v=fminf(v,fmaxf(0.075f,forward_speed+0.45f*dt));
    forward_speed=v;
#if defined(W4_REAR_FORWARD) && !defined(W4_DISABLE_STRAIGHT_CORRECTION)
    /* Keep the planned second-leg axis as reference even if the caster
       displaces the chassis during the pivot. Preserve terminal recovery. */
    if(after_corner && v>0.0f && !recover_point) {
        float trim=straight_correction(&n.pose,final_heading,cross_track,v,dt);
        drive_wheels(v-trim,v+trim,vl,vr,dt,now,begin,true);
        return;
    }
#endif
    if(heading_pid.have_previous)
        heading_pid.previous=error-W4Wrap(error-heading_pid.previous);
    float heading_kp=v>0.0f?0.65f*w4_config.heading_kp:w4_config.heading_kp;
    float heading_kd=v>0.0f?0.10f:w4_config.heading_kd;
    float w=W4Pid_Update(&heading_pid,error,dt,heading_kp,w4_config.heading_ki,
                        heading_kd,w4_config.max_w);
    float turn_speed=w*w4_config.track_m*0.5f;
    /* A tracking correction can slow the inner wheel; a stopped turn can
       still command opposite directions when joining the next segment. */
    if(v>0.0f) turn_speed=fmaxf(-0.85f*v,fminf(0.85f*v,turn_speed));
    float l=v-turn_speed,r=v+turn_speed;
    float peak=fmaxf(fabsf(l),fabsf(r));
    if(peak>w4_config.max_wheel) { l*=w4_config.max_wheel/peak; r*=w4_config.max_wheel/peak; }
    bool loaded_forward=false;
#ifdef W4_MAZE
    loaded_forward=maze && v>0;
#endif
    drive_wheels(l,r,vl,vr,dt,now,begin,loaded_forward);
}
void W4Control_Step(uint32_t now) {
    uint32_t begin=W4Port_Micros();
    w4_stats.pose_us=w4_stats.tracking_us=w4_stats.motor_us=0;
    control_step(now);
    w4_stats.cycle_us=W4Port_Micros()-begin;
    if(w4_stats.cycle_us>w4_stats.max_cycle_us) w4_stats.max_cycle_us=w4_stats.cycle_us;
}
void W4ControlTask(void *argument) {
    (void)argument;
    uint32_t period=osKernelGetTickFreq()/100,deadline=osKernelGetTickCount();
    if(period==0||osKernelGetTickFreq()!=1000) { W4Motor_Stop(); return; }
    for(;;) {
        W4Runtime_Begin(W4_TASK_CONTROL,deadline);
        W4Control_Step(W4Port_Millis());
        W4Runtime_End(W4_TASK_CONTROL,period);
        deadline+=period;
        uint32_t now=osKernelGetTickCount();
        if((int32_t)(deadline-now)<=0) {
            ++w4_stats.deadline_miss;
            W4NavSnapshot n; W4Nav_GetSnapshot(&n);
            if(n.state==W4_PLANNING||n.state==W4_TRACKING) fault(6); else W4Motor_Stop();
            deadline=now+period; /* discard expired releases, no catch-up spin */
        }
        (void)osDelayUntil(deadline);
    }
}
