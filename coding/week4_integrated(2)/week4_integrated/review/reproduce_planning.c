/* Review-only harness: links production navigation with existing test doubles. */
#include "w4_nav.h"
#include "w4_rtos.h"
#include <stdio.h>
#include <string.h>
extern uint32_t test_ms;
extern bool test_imu_valid;
extern char test_ack[96];
volatile uint32_t encoder_edge_count;
static void send(const char *s) {
    while (*s) W4Bt_OnByteISR((uint8_t)*s++);
    W4Bt_Process();
}
int main(void) {
    test_ms=100; W4Nav_Init();
    w4_config.verified=w4_config.encoders_verified=w4_config.imu_axes_verified=true;
    w4_config.diameter_r_m=w4_config.diameter_l_m=0.065f;
    w4_config.cpr_r=w4_config.cpr_l=330;
    w4_config.track_m=0.12f;
    w4_config.encoder_sign_l=w4_config.encoder_sign_r=1;
    send("GOTO,0.3,0.2\n"); W4Path_Poll();
    printf("GUI example target: %s",test_ack);
    send("GOTO,0,1.4\n"); W4Path_Poll();
    printf("Required target: %s",test_ack);
    test_imu_valid=false;
    for(unsigned j=0;j<12100;++j) {
        test_ms+=10; W4Control_Step(test_ms); W4Path_Poll();
    }
    printf("After 121 seconds of failed IMU: state=%d (PLANNING=%d), fault=%lu\n",
           w4_nav.state,W4_PLANNING,(unsigned long)w4_nav.fault);
    send("GOTO,0,1.4\n"); printf("Retry target: %s",test_ack);
    return w4_nav.state==W4_PLANNING && w4_nav.fault==0 ? 0 : 1;
}
