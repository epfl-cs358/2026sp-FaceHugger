#include <unity.h>
#include "../../src/nervous_system/motion_math.h"
#include "../../src/shared/config.h"  // CALIB_* per-servo zero-flat

// Change D: invert is applied once, at the servo write point, as a pitch-only
// mirror (thigh, knee) about each joint's CALIB (zero-flat in servo space),
// shoulder untouched — applyInvert(legId, s, inverted). The key regression-
// safety property is that this servo-space mirror is bit-for-bit identical to
// math-space negation (th=-th, kn=-kn) the gaits used, for every leg, so gait
// output is unchanged. Pre-calibration the mirror was about 90 uniformly;
// per-joint CALIB makes the invariant survive horn-mounting tolerance.
static const double TOL = 1e-9;

void setUp(void) {}
void tearDown(void) {}

void test_passthrough_when_not_inverted(void) {
    ServoTriple s = {30.0, 120.0, 70.0};
    ServoTriple o = applyInvert(0, s, false);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 30.0,  o.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 120.0, o.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 70.0,  o.knee);
}

void test_mirrors_pitch_about_calib_shoulder_untouched(void) {
    // Inverting FR: thigh mirrored about CALIB_FR_THIGH (84), knee about
    // CALIB_FR_KNEE (95). Shoulder passes through unchanged.
    ServoTriple s = {30.0, 120.0, 70.0};
    ServoTriple o = applyInvert(0, s, true);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 30.0,                                  o.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 2.0 * CALIB_FR_THIGH - 120.0,          o.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 2.0 * CALIB_FR_KNEE  -  70.0,          o.knee);
}

// The bit-for-bit gait guarantee: mirroring the servo == negating the math angle,
// for thigh and knee, on every leg; the hip is never inverted. With per-joint
// CALIB this holds for any thigh/knee CALIB value.
void test_servo_mirror_equals_math_negation(void) {
    const double sh = 30.0, th = -20.0, kn = 10.0;
    for (uint8_t i = 0; i < 4; ++i) {
        ServoTriple upright   = translateToServo(i, sh, th, kn);
        ServoTriple mirrored  = applyInvert(i, upright, true);
        ServoTriple math_neg  = translateToServo(i, sh, -th, -kn);
        TEST_ASSERT_DOUBLE_WITHIN(TOL, math_neg.thigh, mirrored.thigh);
        TEST_ASSERT_DOUBLE_WITHIN(TOL, math_neg.knee,  mirrored.knee);
        TEST_ASSERT_DOUBLE_WITHIN(TOL, upright.hip,    mirrored.hip);  // hip not inverted
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_passthrough_when_not_inverted);
    RUN_TEST(test_mirrors_pitch_about_calib_shoulder_untouched);
    RUN_TEST(test_servo_mirror_equals_math_negation);
    return UNITY_END();
}
