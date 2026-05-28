#include <unity.h>
#include "../../src/nervous_system/motion_math.h"

// clampClipServos(): conservative per-joint servo clamp applied ONLY on the clip
// playback path (temporary safeguard, see tickClip). servo 90 = outward for every
// leg (confirmed on hardware via the flat pose). Each leg's URDF shoulder range is
// asymmetric (one side 52°, the other 90°), so 90±52 = [38,142] is guaranteed
// inside EVERY leg's reach regardless of which side — no leg can be driven past its
// mechanical stop. Thigh = 90±60 = [30,150]; knee = 90±90 = [0,180].
static const double TOL = 1e-9;

void setUp(void) {}
void tearDown(void) {}

void test_in_range_passthrough(void) {
    ServoTriple s = clampClipServos({90.0, 90.0, 90.0});
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, s.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, s.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, s.knee);
}

void test_shoulder_clamped_to_38_142(void) {
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 142.0, clampClipServos({187.5, 90, 90}).hip);  // the BL railing case
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 38.0,  clampClipServos({ 15.0, 90, 90}).hip);  // FL jammed-low case
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 120.0, clampClipServos({120.0, 90, 90}).hip);  // inside -> untouched
}

void test_thigh_clamped_to_30_150(void) {
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 150.0, clampClipServos({90, 175.0, 90}).thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL,  30.0, clampClipServos({90,  10.0, 90}).thigh);
}

void test_knee_full_range(void) {
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 0.0,   clampClipServos({90, 90,   0.0}).knee);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 180.0, clampClipServos({90, 90, 180.0}).knee);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_in_range_passthrough);
    RUN_TEST(test_shoulder_clamped_to_38_142);
    RUN_TEST(test_thigh_clamped_to_30_150);
    RUN_TEST(test_knee_full_range);
    return UNITY_END();
}