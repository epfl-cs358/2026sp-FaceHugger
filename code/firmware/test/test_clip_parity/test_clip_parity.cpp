#include <unity.h>
#include <math.h>
#include <stdio.h>
#include "../../src/nervous_system/motion_math.h"
#include "clip_parity_reference.h"

// _frame_to_servo rounds to int; the firmware path is double. Allow 1 deg
// (the design's stated round-trip tolerance) to absorb the rounding.
static const double TOL = 1.0;

void setUp(void) {}
void tearDown(void) {}

void test_translate_matches_python_frame_to_servo(void) {
    for (int i = 0; i < CLIP_PARITY_COUNT; ++i) {
        const ClipParityCase& c = CLIP_PARITY_CASES[i];
        ServoTriple s = translateToServo(c.leg_id, c.sh, c.th, c.kn);
        char m[64];
        snprintf(m, sizeof(m), "case %d leg %d hip", i, c.leg_id);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TOL, c.exp_hip, s.hip, m);
        snprintf(m, sizeof(m), "case %d leg %d thigh", i, c.leg_id);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TOL, c.exp_thigh, s.thigh, m);
        snprintf(m, sizeof(m), "case %d leg %d knee", i, c.leg_id);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TOL, c.exp_knee, s.knee, m);
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_translate_matches_python_frame_to_servo);
    return UNITY_END();
}
