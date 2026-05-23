#include <unity.h>
#include "../../src/nervous_system/motion_math.h"

// easeFraction() is the smoothstep ease-in-out curve used by Servo::tickEase /
// setServoAngleTimed (return-to-neutral tail). smoothstep(t) = t*t*(3-2t):
// slow at the ends, fast in the middle, but exactly 0.5 at the time-midpoint —
// so the distinguishing signal vs a linear ramp is at the quarter points.
static const double TOL = 1e-9;

void setUp(void) {}
void tearDown(void) {}

void test_endpoints_exact(void) {
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 0.0, easeFraction(0, 100));
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 1.0, easeFraction(100, 100));  // reaches target exactly
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 1.0, easeFraction(150, 100));  // and stays there past end
}

void test_midpoint_is_half(void) {
    // smoothstep(0.5) == 0.5 — identical to linear at the midpoint.
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 0.5, easeFraction(50, 100));
}

void test_slow_start_fast_finish(void) {
    // At 25% time, smoothstep lags linear (slow start): 0.25^2*(3-0.5)=0.15625.
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 0.15625, easeFraction(25, 100));
    TEST_ASSERT_TRUE(easeFraction(25, 100) < 0.25);
    // At 75% time, smoothstep leads linear (fast finish): 0.75^2*(3-1.5)=0.84375.
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 0.84375, easeFraction(75, 100));
    TEST_ASSERT_TRUE(easeFraction(75, 100) > 0.75);
}

void test_zero_duration_is_one(void) {
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 1.0, easeFraction(10, 0));  // instantaneous move
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_endpoints_exact);
    RUN_TEST(test_midpoint_is_half);
    RUN_TEST(test_slow_start_fast_finish);
    RUN_TEST(test_zero_duration_is_one);
    return UNITY_END();
}
