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
    // Change B: FL shoulder regularized to 90 + (sh - 135), matching the other
    // three legs' "servo 90 == outward" pattern (FL outward = +135).
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + (sh - 135.0), fl.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + th,           fl.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - kn,           fl.knee);

    ServoTriple rr = translateToServo(2, sh, th, kn);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - (sh + 45.0), rr.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + th,          rr.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - kn,          rr.knee);

    ServoTriple rl = translateToServo(3, sh, th, kn);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + (sh + 135.0), rl.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - th,           rl.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + kn,           rl.knee);
}

// Change B invariant: every leg's shoulder reads servo 90 at its outward direction
// (FR +45, FL +135, BR -45, BL -135). This is the property the all-90 calibration
// pose depends on — it must hold for all four legs after regularization.
void test_all_shoulders_outward_is_90(void) {
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, translateToServo(0,   45.0, 0, 0).hip);  // FR
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, translateToServo(1,  135.0, 0, 0).hip);  // FL
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, translateToServo(2,  -45.0, 0, 0).hip);  // BR
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, translateToServo(3, -135.0, 0, 0).hip);  // BL
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_translate_to_servo_per_leg);
    RUN_TEST(test_all_shoulders_outward_is_90);
    return UNITY_END();
}
