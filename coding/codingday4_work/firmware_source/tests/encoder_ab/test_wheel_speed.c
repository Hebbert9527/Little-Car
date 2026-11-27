#include "w4_wheel_speed.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    W4WheelSpeedController left, right;
    W4WheelSpeed_Start(&left, 65500, UINT32_MAX - 49);
    assert(W4WheelSpeed_Update(&left, 64, 0, 1200, 90));
    assert(left.delta == 100 && isfinite(left.rpm));
    assert(!W4WheelSpeed_Update(&left, 65, 200, 1200, 90));
    assert(left.pwm == 0);
    W4WheelSpeed_Start(&left, 0, 0);
    assert(!W4WheelSpeed_Update(&left, 0, 50, 0, 90));
    assert(left.pwm == 0);
    W4WheelSpeed_Start(&left, 0, 0);
    assert(!W4WheelSpeed_Update(&left, 0, 50, NAN, 90));
    assert(!W4WheelSpeed_Update(&left, 65500, 100, 1558, 90));

    /* Unequal simulated motor slopes, lag and physical encoder scales. */
    W4WheelSpeed_Start(&left, 0, 0);
    W4WheelSpeed_Start(&right, 0, 0);
    double left_count = 0, right_count = 0;
    float left_speed = 0, right_speed = 0;
    left.pwm = right.pwm = 0.40f;
    for (uint32_t ms = 50; ms <= 5000; ms += 50) {
        float left_equilibrium = fmaxf(0, 1250 * (left.pwm - 0.30f));
        float right_equilibrium = fmaxf(0, 1000 * (right.pwm - 0.30f));
        left_speed += 0.25f * (left_equilibrium - left_speed);
        right_speed += 0.25f * (right_equilibrium - right_speed);
        left_count += left_speed / 60 * 1558.55 * 0.05;
        right_count += right_speed / 60 * 1564.60 * 0.05;
        assert(W4WheelSpeed_Update(&left, (uint16_t)(uint32_t)left_count, ms, 1558.55f, 90));
        assert(W4WheelSpeed_Update(&right, (uint16_t)(uint32_t)right_count, ms, 1564.60f, 90));
        assert(left.pwm >= 0 && left.pwm <= 0.4f && right.pwm >= 0 && right.pwm <= 0.4f);
    }
    assert(fabsf(left.rpm - 90) < 2 && fabsf(right.rpm - 90) < 2);
    assert(fabsf(left.rpm - right.rpm) < 2);
    assert(left.pwm < right.pwm);
    assert(W4WheelSpeed_Update(&left, (uint16_t)(uint32_t)left_count, 5050, 1558.55f, 0));
    assert(left.pwm == 0 && left.integral == 0);
    float integral=0, duty=0;
    for (unsigned i=0; i<25; ++i) {
        duty=W4WheelSpeed_CommandDuty(30.0f, 60.0f, 0.05f, 0.20f, &integral);
    }
    assert(duty < 0.15f);
    puts("PASS independent CPR, wrap, feedback fault and unequal motor convergence");
    return 0;
}
