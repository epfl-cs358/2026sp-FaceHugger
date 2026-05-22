#include <unity.h>
#include "../../src/nervous_system/motion_math.h"

static const double TOL = 1e-6;

void setUp(void) {}
void tearDown(void) {}

// Ground truth = the four per-leg formulas, copied independently from the
// pre-refactor tickGait switch (spinal_cord.cpp). If this fails, either the
// helper diverged or the formulas changed — both are exactly what we guard.
void test_translate_to_servo_per_leg(void) {
    const double sh = 30.0, th = -20.0, kn = 10.0;

    ServoTriple fr = translateToServo(0, sh, th, kn);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + (sh - 45.0), fr.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - th,          fr.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + kn,          fr.knee);

    ServoTriple fl = translateToServo(1, sh, th, kn);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, sh,                 fl.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + th,          fl.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - kn,          fl.knee);

    ServoTriple rr = translateToServo(2, sh, th, kn);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - (sh + 45.0), rr.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + th,          rr.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - kn,          rr.knee);

    ServoTriple rl = translateToServo(3, sh, th, kn);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + (sh + 135.0), rl.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - th,           rl.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + kn,           rl.knee);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_translate_to_servo_per_leg);
    return UNITY_END();
}
