#include <unity.h>
#include "../../src/nervous_system/motion_math.h"
#include "../fixtures/clips_all.h"

static const float TOL = 1e-3f;

void setUp(void) {}
void tearDown(void) {}

void test_before_first_frame_is_first(void) {
    uint16_t cur = 0; float out[12];
    clipPoseAt(fh_clip_a, 3, 0, &cur, out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out[0]);
}

void test_exact_frame_hits_keyframe(void) {
    uint16_t cur = 0; float out[12];
    clipPoseAt(fh_clip_a, 3, 100, &cur, out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 10.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 20.0f, out[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 30.0f, out[2]);
}

void test_midpoint_lerps(void) {
    uint16_t cur = 0; float out[12];
    clipPoseAt(fh_clip_a, 3, 50, &cur, out);   // halfway 0->10
    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.0f, out[0]);
    clipPoseAt(fh_clip_a, 3, 150, &cur, out);  // halfway frame1->frame2: 10->30
    TEST_ASSERT_FLOAT_WITHIN(TOL, 20.0f, out[0]);
}

void test_after_last_frame_holds_last(void) {
    uint16_t cur = 0; float out[12];
    clipPoseAt(fh_clip_a, 3, 999, &cur, out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 30.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 50.0f, out[2]);
}

void test_single_frame_clip(void) {
    uint16_t cur = 0; float out[12];
    clipPoseAt(fh_clip_single, 1, 0, &cur, out);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.0f, out[0]);
    clipPoseAt(fh_clip_single, 1, 500, &cur, out);  // still frame 0
    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.0f, out[0]);
}

void test_cursor_monotonic_matches_fresh(void) {
    uint16_t cur = 0; float a[12], b[12]; uint16_t fresh = 0;
    clipPoseAt(fh_clip_a, 3, 50, &cur, a);     // advances cursor
    clipPoseAt(fh_clip_a, 3, 150, &cur, a);    // reuse advanced cursor
    clipPoseAt(fh_clip_a, 3, 150, &fresh, b);  // fresh cursor
    for (int j = 0; j < 12; ++j) TEST_ASSERT_FLOAT_WITHIN(TOL, b[j], a[j]);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_before_first_frame_is_first);
    RUN_TEST(test_exact_frame_hits_keyframe);
    RUN_TEST(test_midpoint_lerps);
    RUN_TEST(test_after_last_frame_holds_last);
    RUN_TEST(test_single_frame_clip);
    RUN_TEST(test_cursor_monotonic_matches_fresh);
    return UNITY_END();
}
