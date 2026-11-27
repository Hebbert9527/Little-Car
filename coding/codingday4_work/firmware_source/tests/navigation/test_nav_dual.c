#include "w4_nav.h"
#include "w4_rtos.h"
#include "w4_integration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

extern uint32_t test_ms;
extern uint16_t test_left_count, test_right_count;
extern bool test_ready;
extern float test_pwm_l, test_pwm_r, test_heading, test_gyro;
extern char test_ack[96];

static void send(const char *command)
{
    while (*command) W4Bt_OnByteISR((uint8_t)*command++);
    W4Bt_Process();
}

static float logical_left_pwm(void)
{
#ifdef W4_REAR_FORWARD
    return -test_pwm_r;
#else
    return test_pwm_l;
#endif
}

static float logical_right_pwm(void)
{
#ifdef W4_REAR_FORWARD
    return -test_pwm_l;
#else
    return test_pwm_r;
#endif
}

static void follow_corner(const char *command, float side, float corner_y)
{
    send("RESET\n");
    W4Control_Step(test_ms);
    send(command);
    W4Path_Poll();
    assert(strstr(test_ack,"PATH_ACCEPTED"));

    float left_velocity=0.0f,right_velocity=0.0f,peak_forward=0.0f;
    float left_count=test_left_count,right_count=test_right_count;
#ifdef W4_REAR_FORWARD
    const float physical_left_scale=w4_config.cpr_r/(W4_PI*w4_config.diameter_r_m);
    const float physical_right_scale=w4_config.cpr_l/(W4_PI*w4_config.diameter_l_m);
    const float forward_sign=-1.0f;
#else
    const float physical_left_scale=w4_config.cpr_l/(W4_PI*w4_config.diameter_l_m);
    const float physical_right_scale=w4_config.cpr_r/(W4_PI*w4_config.diameter_r_m);
    const float forward_sign=1.0f;
#endif
    const float direction_y=copysignf(1.0f,corner_y);
    const float turn_side=side*direction_y;
    bool pivoted=false;
#ifdef W4_WHEEL_DIAGNOSTICS
    bool reached_exit_cruise=false;
#endif
    unsigned stopped_at_corner_ms=0;
    bool decelerated_before_corner=false;
    for(unsigned i=0;i<3000&&w4_nav.state!=W4_ARRIVED;++i) {
        W4Path_Poll();
        float left_target=copysignf(1.65f*fmaxf(0.0f,fabsf(test_pwm_l)-0.08f),test_pwm_l);
        float right_target=copysignf(1.34f*fmaxf(0.0f,fabsf(test_pwm_r)-0.08f),test_pwm_r);
#ifdef W4_REAR_FORWARD
        /* The loaded original right wheel needs more than 40% in this direction. */
        if(side<0.0f&&test_pwm_r>0.0f&&test_pwm_l<0.0f)
            right_target=0.62f*fmaxf(0.0f,test_pwm_r-0.41f);
#endif
        left_velocity+=0.08f*(left_target-left_velocity);
        right_velocity+=0.08f*(right_target-right_velocity);
        left_count+=left_velocity*0.01f*physical_left_scale;
        right_count+=right_velocity*0.01f*physical_right_scale;
        test_left_count=(uint16_t)lroundf(left_count);
        test_right_count=(uint16_t)lroundf(right_count);
        test_gyro=(right_velocity-left_velocity)/w4_config.track_m;
        test_heading=W4Wrap(test_heading+test_gyro*0.01f);
        peak_forward=fmaxf(peak_forward,forward_sign*0.5f*(left_velocity+right_velocity));
        test_ms+=10;
        W4Control_Step(test_ms);
        assert(w4_nav.state!=W4_FAULT);
        float corner_remaining=(corner_y-w4_nav.pose.y_m)*direction_y;
        if(!pivoted&&corner_remaining>0.015f&&corner_remaining<=0.04f) {
            float measured_forward=forward_sign*0.5f*(left_velocity+right_velocity);
            if(measured_forward>0.0f&&measured_forward<0.16f)
                decelerated_before_corner=true;
        }
        if(!pivoted&&test_pwm_l==0.0f&&test_pwm_r==0.0f&&
           hypotf(w4_nav.pose.x_m,w4_nav.pose.y_m-corner_y)<=0.025f) {
            stopped_at_corner_ms+=10;
        }
        if(!pivoted&&fabsf(corner_remaining)<0.10f&&logical_left_pwm()*logical_right_pwm()<0.0f) {
            assert(logical_left_pwm()*turn_side>0.0f&&logical_right_pwm()*turn_side<0.0f);
            assert(hypotf(w4_nav.pose.x_m,w4_nav.pose.y_m-corner_y)<=0.025f);
            assert(decelerated_before_corner);
            assert(stopped_at_corner_ms>=100);
            pivoted=true;
        }
#ifdef W4_WHEEL_DIAGNOSTICS
        if(pivoted&&w4_nav.state==W4_TRACKING) {
            char line[160];
            int left,right;
            if(W4Motor_FormatDiagnostics(line,sizeof line)>0&&
               sscanf(line,"D,%*u,WHEEL,%*u,%d,%d",&left,&right)==2&&left>0&&right>0) {
                /* Reported mean speed is at most 0.18 m/s after the pivot. */
                assert(left+right<=361);
                if(left+right>=340) reached_exit_cruise=true;
            }
        }
#endif
        if(pivoted&&logical_left_pwm()>0.0f&&logical_right_pwm()>0.0f&&
           fabsf(w4_nav.pose.x_m)<0.05f) {
            float leg_heading=side<0.0f?W4_PI:0.0f;
            assert(fabsf(W4Wrap(leg_heading-w4_nav.pose.theta_rad))<0.08f);
        }
    }
#ifdef W4_WHEEL_DIAGNOSTICS
    assert(reached_exit_cruise);
#endif
    assert(pivoted&&peak_forward>0.24f&&w4_nav.state==W4_ARRIVED);
    assert(hypotf(w4_nav.pose.x_m-side*fabsf(corner_y),w4_nav.pose.y_m-corner_y)<=0.05f);
    assert(test_pwm_l==0.0f&&test_pwm_r==0.0f);
    test_gyro=0.0f;
}

static void add_forward_counts(uint16_t left, uint16_t right)
{
#ifdef W4_REAR_FORWARD
    test_left_count-=right;
    test_right_count-=left;
#else
    test_left_count+=left;
    test_right_count+=right;
#endif
}

static void test_relative_straight(void)
{
    test_heading=0.0f;
    test_gyro=0.0f;
    assert(W4Nav_ResetPose(0.25f,-0.40f,0.0f));
    W4Control_Step(test_ms);
    send("GOTO,0,1\n");
    W4Path_Poll();
    assert(strstr(test_ack,"PATH_ACCEPTED"));
    test_ms+=10;
    W4Control_Step(test_ms);
    assert(w4_nav.state==W4_TRACKING);
    assert(fabsf(w4_nav.target_x-1.25f)<0.001f);
    assert(fabsf(w4_nav.target_y+0.40f)<0.001f);

    const float step=0.5f*W4_PI*(18*w4_config.diameter_l_m/w4_config.cpr_l+
                               14*w4_config.diameter_r_m/w4_config.cpr_r);
    float travelled=0.0f;
    for(unsigned i=0;i<600&&w4_nav.state!=W4_ARRIVED;++i) {
        add_forward_counts(18,14);
        travelled+=step;
        if(i==150) w4_nav.pose.theta_rad=test_heading=0.8f;
        if(i==250) w4_nav.pose.theta_rad=test_heading=-0.8f;
        test_ms+=10;
        W4Control_Step(test_ms);
        if(travelled<1.0f) {
            assert(w4_nav.state==W4_TRACKING);
#ifdef W4_WHEEL_DIAGNOSTICS
            if(i%50==0||i==151) {
                char line[160];
                assert(W4Motor_FormatDiagnostics(line,sizeof line)>0);
                int left,right,target_left,target_right;
                assert(sscanf(line,"D,%*u,WHEEL,%*u,%d,%d,%d,%d",
                              &left,&right,&target_left,&target_right)==4);
                assert(left==target_left&&right==target_right);
                assert(left+right==800);
#ifdef W4_DISABLE_STRAIGHT_CORRECTION
                assert(left==400&&right==400);
#else
                assert(left>=375&&left<=425&&right>=375&&right<=425);
                if(i<150) assert(left==400&&right==400);
                if(i==150) assert(left==401&&right==399);
                if(i==151) assert(left==401&&right==399);
                if(i==200) assert(left==425&&right==375);
                if(i==250) assert(left==424&&right==376);
                if(i==350) assert(left==375&&right==425);
#endif
            }
#endif
        }
    }
    assert(travelled>=1.0f&&travelled<1.0f+step+0.00001f);
    assert(w4_nav.state==W4_ARRIVED&&test_pwm_l==0.0f&&test_pwm_r==0.0f);
    assert(fabsf(w4_nav.pose.theta_rad)>0.5f); /* No final pivot. */

    /* The next move starts its distance from zero after alignment. */
    w4_nav.pose.theta_rad=test_heading=0.0f;
    send("GOTO,0,0.02\n");
    W4Path_Poll();
    assert(strstr(test_ack,"PATH_ACCEPTED"));
    test_ms+=10;
    W4Control_Step(test_ms);
    assert(w4_nav.state==W4_TRACKING);
    for(unsigned i=0;i<9;++i) {
        add_forward_counts(18,14);
        test_ms+=10;
        W4Control_Step(test_ms);
    }
    assert(w4_nav.state==W4_TRACKING);
    add_forward_counts(18,14);
    test_ms+=10;
    W4Control_Step(test_ms);
    assert(w4_nav.state==W4_ARRIVED&&test_pwm_l==0.0f&&test_pwm_r==0.0f);

    send("GOTO,0,0\n");
    W4Path_Poll();
    test_ms+=10;
    W4Control_Step(test_ms);
    assert(w4_nav.state==W4_ARRIVED&&test_pwm_l==0.0f&&test_pwm_r==0.0f);
}

#ifndef W4_DISABLE_STRAIGHT_CORRECTION
static void test_saturated_wheel_difference(void)
{
    W4Motor_Stop();
    W4Motor_SetWheelSpeeds(0.4f,0.4f);
    float left=0.0f,right=0.0f;
    for(unsigned i=0;i<600;++i) {
        test_ms+=10;
        W4Motor_Update(left,right,0.01f,test_ms,true);
        /* Unequal loaded wheel response at the 40% duty ceiling. */
        left+=0.10f*(0.75f*fmaxf(0.0f,logical_left_pwm()-0.10f)-left);
        right+=0.10f*(0.90f*fmaxf(0.0f,logical_right_pwm()-0.10f)-right);
    }
    assert(fabsf(left-right)<0.012f);
    assert(left>0.21f&&logical_left_pwm()>=0.399f);
    assert(logical_right_pwm()>=0.329f&&logical_right_pwm()<0.365f);
    W4Motor_Stop();
    assert(test_pwm_l==0.0f&&test_pwm_r==0.0f);
}
#endif

#ifdef W4_REAR_FORWARD
static void test_loaded_left_pivot_assist(void)
{
    W4Motor_Stop();
    W4Motor_SetWheelSpeeds(-0.088f,0.088f);
    for(unsigned i=0;i<26;++i) {
        test_ms+=10;
        W4Motor_Update(0.0f,0.08f,0.01f,test_ms,false);
    }
    assert(fabsf(logical_left_pwm()+0.60f)<0.0001f);
    /* Replay the measured partial start for longer than the old 250 ms pulse. */
    for(unsigned i=0;i<50;++i) {
        test_ms+=10;
        W4Motor_Update(-0.048f,0.08f,0.01f,test_ms,false);
        assert(fabsf(logical_left_pwm()+0.60f)<0.0001f);
    }
    for(unsigned i=0;i<20;++i) {
        float previous=logical_left_pwm();
        test_ms+=10;
        W4Motor_Update(-0.11f,0.08f,0.01f,test_ms,false);
        assert(logical_left_pwm()>=previous-0.00001f);
        assert(logical_left_pwm()-previous<0.007f);
    }
    assert(logical_left_pwm()>-0.55f&&logical_left_pwm()<-0.40f);
    for(unsigned i=0;i<30;++i) {
        test_ms+=10;
        W4Motor_Update(-0.048f,0.08f,0.01f,test_ms,false);
    }
    assert(fabsf(logical_left_pwm()+0.60f)<0.0001f);
    W4Motor_Stop();
    assert(test_pwm_l==0.0f&&test_pwm_r==0.0f);

    /* A blocked wheel still stops both motors during a left pivot. */
    test_heading=test_gyro=0.0f;
    assert(W4Nav_ResetPose(0.0f,0.0f,0.0f));
    W4Control_Step(test_ms);
    w4_nav.pose.theta_rad=test_heading=-0.35f;
    send("GOTO,0,1\n");
    W4Path_Poll();
    for(unsigned i=0;i<115;++i) {
        test_ms+=10;
        W4Control_Step(test_ms);
    }
    assert(w4_nav.state==W4_FAULT&&w4_nav.fault==5);
    assert(test_pwm_l==0.0f&&test_pwm_r==0.0f);
}
#endif

static void test_start_alignment(void)
{
    test_heading=0.0f;
    assert(W4Nav_ResetPose(0.0f,0.0f,0.0f));
    W4Control_Step(test_ms);
    /* Move the heading away from the stationary RESET reference. */
    w4_nav.pose.theta_rad=test_heading=0.35f;
    send("GOTO,0,0.02\n");
    W4Path_Poll();
    for(unsigned i=0;i<28;++i) {
        test_ms+=10;
        W4Control_Step(test_ms);
        if(i>=6) {
            assert(logical_left_pwm()>0.0f&&logical_right_pwm()<0.0f);
            /* Asymmetric pivot counts must not shorten the forward leg. */
            add_forward_counts(10,(uint16_t)-4);
        }
    }
    assert(w4_nav.state==W4_TRACKING);
    w4_nav.pose.theta_rad=test_heading=0.0f;
    for(unsigned i=0;i<10;++i) {
        test_ms+=10;
        W4Control_Step(test_ms);
        assert(test_pwm_l==0.0f&&test_pwm_r==0.0f);
    }
    test_ms+=10;
    W4Control_Step(test_ms);
    assert(logical_left_pwm()>0.0f&&logical_right_pwm()>0.0f);
    for(unsigned i=0;i<10;++i) {
        add_forward_counts(18,14);
        test_ms+=10;
        W4Control_Step(test_ms);
        assert(w4_nav.state==(i==9?W4_ARRIVED:W4_TRACKING));
    }
    assert(test_pwm_l==0.0f&&test_pwm_r==0.0f);

    w4_nav.pose.theta_rad=test_heading=-0.35f;
    send("GOTO,0,1\n");
    W4Path_Poll();
    for(unsigned i=0;i<8;++i) {
        test_ms+=10;
        W4Control_Step(test_ms);
    }
    assert(logical_left_pwm()<0.0f&&logical_right_pwm()>0.0f);
    send("STOP\n");
    test_ms+=10;
    W4Control_Step(test_ms);
    W4Path_Poll();
    assert(w4_nav.state==W4_STOPPED&&test_pwm_l==0.0f&&test_pwm_r==0.0f);
}

static void test_coordinate_route_from_current_pose(void)
{
    test_heading=test_gyro=0.0f;
    assert(W4Nav_ResetPose(0.2f,0.3f,-W4_PI/2));
    W4Control_Step(test_ms);
    send("GOTO,-0.4,-0.6\n");
    bool saw_corner=false,saw_end=false;
    for(unsigned i=0;i<64&&!saw_end;++i) {
        W4Path_Poll();
        W4Waypoint next;
        assert(osMessageQueueGet(Q_Path,&next,0,0)==osOK);
        assert(next.index&W4_WAYPOINT_NEGATIVE);
        if(!saw_corner) {
            assert(next.index&W4_WAYPOINT_AXIS_Y);
            assert(fabsf(next.x_m-0.2f)<0.001f);
            if(next.index&W4_CORNER_WAYPOINT) {
                assert(fabsf(next.y_m+0.6f)<0.001f);
                saw_corner=true;
            }
        } else {
            assert(next.index&W4_WAYPOINT_AXIS_X);
            assert(fabsf(next.y_m+0.6f)<0.001f);
            if(next.last) {
                assert(fabsf(next.x_m+0.4f)<0.001f);
                saw_end=true;
            }
        }
    }
    assert(strstr(test_ack,"PATH_ACCEPTED")&&saw_corner&&saw_end);
    send("STOP\n");
    test_ms+=10;
    W4Control_Step(test_ms);
    W4Path_Poll();
}

static void test_corner_with_position_error(void)
{
    test_heading=test_gyro=0.0f;
    send("RESET\n");
    W4Control_Step(test_ms);
    send("GOTO,1,1\n");
    for(unsigned i=0;i<50;++i) W4Path_Poll();
    assert(strstr(test_ack,"PATH_ACCEPTED"));
    test_ms+=10;
    W4Control_Step(test_ms);
    w4_nav.pose.x_m=-0.12f;
    w4_nav.pose.y_m=0.97f;
    for(unsigned i=0;i<28;++i) {
        test_ms+=10;
        W4Control_Step(test_ms);
        assert(w4_nav.state==W4_TRACKING);
        assert(logical_left_pwm()>=0.0f&&logical_right_pwm()>=0.0f);
    }
    w4_nav.pose.y_m=1.04f;
    unsigned zero_pwm_ms=0;
    bool turned=false;
    for(unsigned i=0;i<28;++i) {
        test_ms+=10;
        W4Control_Step(test_ms);
        assert(w4_nav.state==W4_TRACKING);
        if(logical_left_pwm()>0.0f&&logical_right_pwm()<0.0f) {
            turned=true;
            break;
        }
        assert(test_pwm_l==0.0f&&test_pwm_r==0.0f);
        zero_pwm_ms+=10;
    }
    assert(turned&&zero_pwm_ms>=100);
    send("STOP\n");
    test_ms+=10;
    W4Control_Step(test_ms);
    W4Path_Poll();
}

static void test_goal_heading_hysteresis(void)
{
    test_heading=0.0f;
    test_gyro=0.0f;
    send("RESET\n");
    W4Control_Step(test_ms);
    /* Nonzero X selects the existing coordinate route. */
    send("GOTO,0.0005,1.4\n");
    for(unsigned i=0;i<40;++i) W4Path_Poll();
    assert(strstr(test_ack,"PATH_ACCEPTED"));
    test_ms+=10;
    W4Control_Step(test_ms);
    for(unsigned i=0;i<30;++i) {
        w4_nav.pose.y_m=1.36f;
        test_ms+=10;
        W4Control_Step(test_ms);
    }
    assert(w4_nav.wp_index+1U==(unsigned)ceilf(hypotf(0.0005f,1.4f)/w4_config.step_m));

    /* Replay alternating estimates on either side of the 5 cm goal circle.
       The car is pointing about 115 degrees away from the final heading. */
    w4_nav.pose.y_m=1.440f;
    w4_nav.pose.x_m=0.028f;
    test_heading=2.0f;
    w4_nav.pose.theta_rad=W4Wrap(W4_PI/2+test_heading);
    for(unsigned i=0;i<8;++i) {
        test_ms+=10;
        W4Control_Step(test_ms);
    }
    for(unsigned i=0;i<16;++i) {
        w4_nav.pose.x_m=i%2==0?0.032f:0.028f;
        test_ms+=10;
        W4Control_Step(test_ms);
        assert(logical_left_pwm()>0.0f&&logical_right_pwm()<0.0f);
    }
    /* The pivot hysteresis must not widen the arrival tolerance. */
    w4_nav.pose.x_m=0.032f;
    w4_nav.pose.theta_rad=W4_PI/2;
    test_heading=0.0f;
    test_ms+=10;
    W4Control_Step(test_ms);
    assert(w4_nav.state==W4_TRACKING&&test_pwm_l==0.0f&&test_pwm_r==0.0f);
    w4_nav.pose.x_m=0.028f;
    for(unsigned i=0;i<22;++i) {
        test_ms+=10;
        W4Control_Step(test_ms);
    }
    assert(w4_nav.state==W4_ARRIVED&&test_pwm_l==0.0f&&test_pwm_r==0.0f);
}

int main(void)
{
    test_ms = 100;
    W4Calibration_Apply();
    W4Nav_Init();
    assert(!W4HardwareReady());
    send("GOTO,0,1.4\n");
    W4Path_Poll();
    assert(strstr(test_ack, "HARDWARE_NOT_READY"));
    assert(test_pwm_l == 0 && test_pwm_r == 0);
#ifdef W4_REAR_FORWARD
    assert(fabsf(w4_config.cpr_l - 1561.60f) < 0.01f);
    assert(fabsf(w4_config.cpr_r - 1559.45f) < 0.01f);
    assert(w4_config.encoder_sign_l==-1&&w4_config.encoder_sign_r==-1);
#else
    assert(fabsf(w4_config.cpr_l - 1559.45f) < 0.01f);
    assert(fabsf(w4_config.cpr_r - 1561.60f) < 0.01f);
#endif

    W4EncoderDelta e;
    test_left_count = 65530;
    test_right_count = 10;
    assert(W4Encoder_TakeDelta(&e) && !e.left_delta && !e.right_delta);
    test_left_count = 9;
    test_right_count = 65531;
#ifdef W4_REAR_FORWARD
    assert(W4Encoder_TakeDelta(&e) && e.left_delta == -15 && e.right_delta == 15);
#else
    assert(W4Encoder_TakeDelta(&e) && e.left_delta == 15 && e.right_delta == -15);
#endif
    test_ready = false;
    assert(!W4Encoder_TakeDelta(&e));
    test_left_count = test_right_count = 31000;
    test_ready = true;
    assert(W4Encoder_TakeDelta(&e) && !e.left_delta && !e.right_delta);

    w4_config.verified = w4_config.encoders_verified = w4_config.imu_axes_verified = true;
    assert(W4HardwareReady());
    e = (W4EncoderDelta){test_ms, 100*w4_config.encoder_sign_l, 50*w4_config.encoder_sign_r, 1};
    float left, right;
    assert(W4Pose_Update(test_ms, &e, 0.02f, &left, &right));
    assert(left > right * 1.99f && left < right * 2.01f);
    assert(fabsf(w4_nav.pose.y_m - 0.01f * (left + right)) < 0.00001f);

    /* A small lateral error near a coordinate waypoint must not reverse the inner wheel. */
    send("RESET\n");
    W4Control_Step(test_ms);
    send("GOTO,0.0005,1.4\n");
    W4Path_Poll();
    assert(strstr(test_ack, "PATH_ACCEPTED"));
    W4Path_Poll();
    test_ms += 10;
    W4Control_Step(test_ms);
    w4_nav.pose.x_m = 0.004f;
    w4_nav.pose.y_m = 0.040f;
    for (unsigned i = 0; i < 12; ++i) {
        test_ms += 10;
        W4Control_Step(test_ms);
        assert(logical_left_pwm() >= 0 && logical_right_pwm() >= 0);
    }
    assert(logical_left_pwm() > 0 && logical_right_pwm() > 0);
    /* A missed intermediate point must not block later straight points. */
    w4_nav.pose.x_m = 0.052f;
    w4_nav.pose.y_m = 0.060f;
    test_ms += 10;
    W4Control_Step(test_ms);
    test_ms += 10;
    W4Control_Step(test_ms);
    assert(w4_nav.wp_index == 1);
    send("STOP\n");
    test_ms += 10;
    W4Control_Step(test_ms);
    W4Path_Poll();

    send("RESET\n");
    W4Control_Step(test_ms);
    send("GOTO,0,1.4\n");
    W4Path_Poll();
    assert(strstr(test_ack, "PATH_ACCEPTED"));
    for (unsigned i = 0; i < 115; ++i) {
        test_ms += 10;
#ifdef W4_REAR_FORWARD
        if (logical_right_pwm() > 0.0f) test_left_count -= 4;
#else
        if (logical_right_pwm() > 0.0f) test_right_count += 4;
#endif
        W4Control_Step(test_ms);
        if (i == 24) {
#ifdef W4_REAR_FORWARD
            assert(logical_left_pwm() == 0.40f && logical_right_pwm() > 0.0f);
#else
            assert(logical_left_pwm() == 0.60f && logical_right_pwm() > 0.0f);
#endif
        }
    }
    assert(w4_nav.state == W4_FAULT && w4_nav.fault == 5);
    assert(test_pwm_l == 0 && test_pwm_r == 0);

    send("RESET\n");
    W4Control_Step(test_ms);
    send("GOTO,0,1.4\n");
    W4Path_Poll();
    for (unsigned i = 0; i < 10; ++i) {
        test_ms += 10;
        W4Control_Step(test_ms);
    }
    assert(w4_nav.state == W4_TRACKING && logical_left_pwm() > 0 && logical_right_pwm() > 0);
#ifdef W4_REAR_FORWARD
    send("CAL,0.3100,0.3500\n");
    assert(strstr(test_ack,"CAL_REQUIRES_STILL") && !w4_straight_cal.enabled);
#endif
#ifdef W4_WHEEL_DIAGNOSTICS
    char wheel_line[160];
    assert(W4Motor_FormatDiagnostics(wheel_line, sizeof wheel_line) > 0);
    assert(wheel_line[0] == 'D' && strstr(wheel_line, ",WHEEL,"));
#endif
    send("STOP\n");
    test_ms += 10;
    W4Control_Step(test_ms);
    W4Path_Poll();
    assert(w4_nav.state == W4_STOPPED && !w4_stop_pending);
    assert(test_pwm_l == 0 && test_pwm_r == 0);
#ifdef W4_WHEEL_DIAGNOSTICS
    assert(W4Motor_FormatDiagnostics(wheel_line, sizeof wheel_line) == 0);
#endif
    assert(fabsf(w4_config.max_v-0.40f)<0.0001f);
#ifdef W4_REAR_FORWARD
    for(unsigned i=0;i<22;++i) { test_ms+=10; W4Control_Step(test_ms); }
    send("CAL,0.3100,0.3500\n");
    assert(strstr(test_ack,"CAL_ON_L3100_R3500"));
    send("CAL,0.5,0.3500\n");
    assert(strstr(test_ack,"CAL_RANGE"));
    send("CAL?\n");
    assert(strstr(test_ack,"CAL_ON_L3100_R3500"));
    send("CAL,OFF\n");
    assert(strstr(test_ack,"CAL_OFF_L3100_R3500"));
#endif
    test_relative_straight();
#ifdef W4_REAR_FORWARD
    test_loaded_left_pivot_assist();
#endif
    test_start_alignment();
#ifndef W4_DISABLE_STRAIGHT_CORRECTION
    test_saturated_wheel_difference();
#endif
    test_goal_heading_hysteresis();
    test_coordinate_route_from_current_pose();
    test_corner_with_position_error();
    follow_corner("GOTO,-0.7,0.7\n",-1.0f,0.7f);
    follow_corner("GOTO,0.7,0.7\n",1.0f,0.7f);
    follow_corner("GOTO,-1,-1\n",-1.0f,-1.0f);
#ifdef W4_REAR_FORWARD
    send("SPEED,0.18\n");
    assert(strstr(test_ack,"SPEED_MMPS_180") && w4_cruise_request==0.18f);
    send("SPEED,0.41\n");
    assert(strstr(test_ack,"SPEED_RANGE") && w4_cruise_request==0.18f);
#endif
    return 0;
}
