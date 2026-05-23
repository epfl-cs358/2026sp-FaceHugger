#include <unity.h>
#include "../../src/nervous_system/motion_math.h"

// Change D: invert is applied once, at the servo write point, as a pitch-only
// mirror about 90 (thigh, knee), shoulder untouched — applyInvert(). The key
// regression-safety property is that this servo-space mirror is bit-for-bit
// identical to the old math-space negation (th=-th, kn=-kn) the gaits used, for
// every leg, so gait output is unchanged.
static const double TOL = 1e-9;

void setUp(void) {}
void tearDown(void) {}

void test_passthrough_when_not_inverted(void) {
    ServoTriple s = {30.0, 120.0, 70.0};
    ServoTriple o = applyInvert(s, false);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 30.0,  o.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 120.0, o.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 70.0,  o.knee);
}

void test_mirrors_pitch_about_90_shoulder_untouched(void) {
    ServoTriple s = {30.0, 120.0, 70.0};
    ServoTriple o = applyInvert(s, true);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 30.0, o.hip);          // shoulder unchanged
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 60.0, o.thigh);        // 180 - 120
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 110.0, o.knee);        // 180 - 70
}

// The bit-for-bit gait guarantee: mirroring the servo == negating the math angle,
// for thigh and knee, on every leg; the hip is never inverted.
void test_servo_mirror_equals_math_negation(void) {
    const double sh = 30.0, th = -20.0, kn = 10.0;
    for (uint8_t i = 0; i < 4; ++i) {
        ServoTriple upright   = translateToServo(i, sh, th, kn);
        ServoTriple mirrored  = applyInvert(upright, true);
        ServoTriple math_neg  = translateToServo(i, sh, -th, -kn);
        TEST_ASSERT_DOUBLE_WITHIN(TOL, math_neg.thigh, mirrored.thigh);
        TEST_ASSERT_DOUBLE_WITHIN(TOL, math_neg.knee,  mirrored.knee);
        TEST_ASSERT_DOUBLE_WITHIN(TOL, upright.hip,    mirrored.hip);  // hip not inverted
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_passthrough_when_not_inverted);
    RUN_TEST(test_mirrors_pitch_about_90_shoulder_untouched);
    RUN_TEST(test_servo_mirror_equals_math_negation);
    return UNITY_END();
}
