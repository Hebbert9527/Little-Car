#include "w4_nav.h"
#include "w4_rtos.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
extern uint32_t test_ms;
extern uint32_t test_advance_ms;
extern uint32_t test_imu_advance_ms;
extern float test_heading,test_gyro;
extern bool test_imu_valid;
extern char test_ack[96];
extern float test_pwm_l,test_pwm_r;
extern int w4_single_cmd_dir;
volatile uint32_t encoder_edge_count;
static int failures;
#define CHECK(c) do { if(!(c)) { printf("FAIL line %d: %s\n",__LINE__,#c); ++failures; } } while(0)
static void send(const char *s) { while(*s) W4Bt_OnByteISR((uint8_t)*s++); W4Bt_Process(); }
static void calibration(void) {
 w4_config.verified=w4_config.encoders_verified=w4_config.imu_axes_verified=true;
 w4_config.diameter_r_m=w4_config.diameter_l_m=0.1f;
 w4_config.cpr_r=w4_config.cpr_l=1000;
 w4_config.track_m=0.2f;
 w4_config.encoder_sign_l=w4_config.encoder_sign_r=1;
}
int main(void) {
 test_ms=100; W4Nav_Init();
 /* Control can run before the planner has checked the calibration gate. */
 send("GOTO,0,1.4\n");
 W4Control_Step(test_ms); W4Path_Poll();
 CHECK(strstr(test_ack,"HARDWARE_NOT_READY")!=0 && w4_nav.state==W4_IDLE);
 calibration();
 CHECK(W4HardwareReady());
 CHECK(w4_nav.pose.valid && w4_nav.pose.stamp_ms==100);
 CHECK(fabsf(w4_nav.pose.theta_rad-W4_PI/2)<1e-5f);
 W4EncoderDelta e; W4Encoder_TakeDelta(&e);
 /* Unsigned edges only measure the right side. */
 w4_single_cmd_dir=1; encoder_edge_count+=10; W4Encoder_TakeDelta(&e);
 CHECK(e.right_delta==10 && e.left_delta==0);
 /* Exact pure rotation: dr=omega*track/2*dt => no centre displacement. */
 float vl,vr; test_gyro=1; test_heading=0.01f;
 e=(W4EncoderDelta){test_ms,0,10,1};
 float dt=0.01f,dr=10*W4_PI*0.1f/1000;
 test_gyro=dr/(w4_config.track_m*0.5f*dt);
 CHECK(W4Pose_Update(test_ms,&e,dt,&vl,&vr));
 CHECK(hypotf(w4_nav.pose.x_m,w4_nav.pose.y_m)<1e-6f);
 CHECK(fabsf(vl+vr)<1e-5f);
 /* Reset with outstanding edges and a nonzero relative IMU heading. */
 test_gyro=0; test_heading=0.8f; encoder_edge_count+=40;
 CHECK(W4Nav_ResetPose(0,0,0.3f));
 W4Control_Step(test_ms);
 CHECK(hypotf(w4_nav.pose.x_m,w4_nav.pose.y_m)<1e-6f);
 CHECK(fabsf(w4_nav.pose.theta_rad-0.3f)<1e-5f);
 test_ms+=10; W4Control_Step(test_ms);
 CHECK(fabsf(w4_nav.pose.theta_rad-0.3f)<1e-5f);
 send("RESET\n"); CHECK(strstr(test_ack,"RESET")!=0);
 W4Control_Step(test_ms);
 /* IMU failure after path acceptance but before the first tracking cycle. */
 send("GOTO,0,1.4\n"); W4Path_Poll();
 CHECK(strstr(test_ack,"PATH_ACCEPTED")!=0 && w4_nav.state==W4_PLANNING);
 test_imu_valid=false;
 test_ms+=10; W4Control_Step(test_ms); W4Path_Poll();
 CHECK(w4_nav.state==W4_FAULT && w4_nav.fault==1 && !w4_nav.pose.valid);
 CHECK(test_pwm_l==0 && test_pwm_r==0 && osMessageQueueGetCount(Q_Path)==0);
 /* Once the IMU is back, RESET and a new command work without an extra STOP. */
 test_imu_valid=true;
 send("RESET\n"); CHECK(strstr(test_ack,"RESET_PENDING")!=0);
 test_ms+=10; W4Control_Step(test_ms);
 CHECK(w4_nav.state==W4_STOPPED && w4_nav.fault==0 && w4_nav.pose.valid);
 const char *routes[]={"GOTO,0,1.4\n","GOTO,-0.7,0.7\n","GOTO,0.7,-0.7\n","GOTO,0.7,0.7\n"};
 for(unsigned n=0;n<4;n++) {
  if(n!=0) w4_nav.state=W4_IDLE; /* Later cases generate routes without driving them. */
  send(routes[n]);
  if(n==0) test_imu_advance_ms=1; /* newer IMU publication while path reads snapshots */
  W4Path_Poll();
  CHECK(strstr(test_ack,"PATH_ACCEPTED")!=0);
  W4Waypoint p,last={0}; int points=0;
  for(unsigned tries=0;tries<100;tries++) {
   if(osMessageQueueGet(Q_Path,&p,0,0)==osOK && p.cmd_id==w4_nav.cmd_id) { last=p; ++points; }
   if(last.last) break;
   W4Path_Poll();
  }
  CHECK(points>0 && last.last);
  if(n==2) CHECK(fabsf(last.x_m-.7f)<1e-5f&&fabsf(last.y_m+.7f)<1e-5f);
 }
 send("STOP\n"); W4Control_Step(test_ms); W4Path_Poll();
 CHECK(w4_nav.state==W4_STOPPED && !w4_stop_pending);
 CHECK(test_pwm_l==0 && test_pwm_r==0);
 /* Coasting retains the right-wheel direction briefly; reversal during
    moving edges is unknowable with one channel and must be rejected. */
 w4_single_cmd_dir=1; test_ms+=100; W4Encoder_TakeDelta(&e);
 encoder_edge_count+=2; test_ms+=10; CHECK(W4Encoder_TakeDelta(&e));
 w4_single_cmd_dir=0; encoder_edge_count+=2; test_ms+=10;
 CHECK(W4Encoder_TakeDelta(&e) && e.right_delta==2 && e.left_delta==0);
 w4_single_cmd_dir=-1; encoder_edge_count+=2; test_ms+=10;
 CHECK(!W4Encoder_TakeDelta(&e));
 w4_single_cmd_dir=0; test_ms+=250; encoder_edge_count+=2;
 CHECK(!W4Encoder_TakeDelta(&e));
 /* After a measured quiet reversal interval the new sign is usable. */
 test_ms+=100; W4Encoder_TakeDelta(&e);
 w4_single_cmd_dir=1; encoder_edge_count+=2; test_ms+=10;
 CHECK(W4Encoder_TakeDelta(&e) && e.right_delta==2);
 w4_nav.state=W4_TRACKING; send("RESET\n");
 CHECK(strstr(test_ack,"REJECTED")!=0 && !w4_reset_pending);
 w4_nav.state=W4_STOPPED;
 /* The inferred left speed may be zero while its turn demand is nonzero.
    Only the physically measured RIGHT wheel can provide stall evidence. */
 test_ms+=10; W4Control_Step(test_ms);
 CHECK(W4Nav_ResetPose(0,0,W4_PI/2)); W4Control_Step(test_ms);
 w4_nav.state=W4_PLANNING; w4_nav.cmd_id=999;
 W4Waypoint turning={999,0,1,-1,0};
 osMessageQueuePut(Q_Path,&turning,0,0);
 test_gyro=(3*W4_PI*.1f/1000/.01f)/w4_config.track_m;
 float start_heading=test_heading;
 for(unsigned j=0;j<130;j++) {
  test_ms+=10; test_heading=start_heading+test_gyro*.01f*(j+1);
  w4_single_cmd_dir=1; encoder_edge_count+=3; W4Control_Step(test_ms);
 }
 CHECK(w4_nav.state==W4_TRACKING && w4_nav.fault==0);
 /* HAL tick can advance inside encoder acquisition after the caller captured
    its release time. Fresh samples must not become unsigned-age underflow. */
 test_ms+=10; test_advance_ms=1;
 encoder_edge_count+=3; W4Control_Step(test_ms);
 CHECK(w4_nav.state==W4_TRACKING && w4_nav.fault==0);
 if(failures) { printf("%d navigation failures\n",failures); return 1; }
 puts("navigation regressions passed"); return 0;
}
