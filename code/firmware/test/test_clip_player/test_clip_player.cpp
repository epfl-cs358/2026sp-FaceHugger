#include <unity.h>
#include "../../src/nervous_system/motion_math.h"

void setUp(void) {}
void tearDown(void) {}

static ClipState fresh(uint32_t start) {
    ClipState s = { CLIP_PLAYING, start, 0, 0, 0 };
    return s;
}

void test_playing_applies_pose_before_duration(void) {
    ClipState s = fresh(1000);
    ClipStep r = clipPlayerStep(&s, 1050, 200, 500);  // 50ms into a 200ms clip
    TEST_ASSERT_EQUAL(CLIP_ACT_APPLY_POSE, r.action);
    TEST_ASSERT_EQUAL_UINT32(50, r.elapsed_ms);
    TEST_ASSERT_EQUAL(CLIP_PLAYING, s.phase);
}

void test_reaching_duration_begins_return(void) {
    ClipState s = fresh(1000);
    ClipStep r = clipPlayerStep(&s, 1200, 200, 500);  // exactly at duration
    TEST_ASSERT_EQUAL(CLIP_ACT_BEGIN_RETURN, r.action);
    TEST_ASSERT_EQUAL_UINT32(200, r.elapsed_ms);       // samples final frame
    TEST_ASSERT_EQUAL(CLIP_RETURNING, s.phase);
    TEST_ASSERT_EQUAL_UINT32(1200, s.returnStartMs);
}

void test_returning_eases_then_finishes(void) {
    ClipState s = fresh(1000);
    clipPlayerStep(&s, 1200, 200, 500);                // -> RETURNING @1200
    ClipStep mid = clipPlayerStep(&s, 1400, 200, 500); // 200ms into 500ms ease
    TEST_ASSERT_EQUAL(CLIP_ACT_EASE, mid.action);
    TEST_ASSERT_EQUAL(CLIP_RETURNING, s.phase);
    ClipStep end = clipPlayerStep(&s, 1700, 200, 500); // ease complete (>=500)
    TEST_ASSERT_EQUAL(CLIP_ACT_FINISH, end.action);
    TEST_ASSERT_EQUAL(CLIP_DONE, s.phase);
}

void test_done_is_noop(void) {
    ClipState s = { CLIP_DONE, 1000, 1200, 0, 0 };
    ClipStep r = clipPlayerStep(&s, 9999, 200, 500);
    TEST_ASSERT_EQUAL(CLIP_ACT_NONE, r.action);
}

void test_zero_duration_single_frame_returns_immediately(void) {
    // A single-frame clip has duration 0: the first tick applies the frame
    // (elapsed 0 >= duration 0) and immediately begins the return.
    ClipState s = fresh(1000);
    ClipStep r = clipPlayerStep(&s, 1000, 0, 500);
    TEST_ASSERT_EQUAL(CLIP_ACT_BEGIN_RETURN, r.action);
    TEST_ASSERT_EQUAL_UINT32(0, r.elapsed_ms);
    TEST_ASSERT_EQUAL(CLIP_RETURNING, s.phase);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_playing_applies_pose_before_duration);
    RUN_TEST(test_reaching_duration_begins_return);
    RUN_TEST(test_returning_eases_then_finishes);
    RUN_TEST(test_done_is_noop);
    RUN_TEST(test_zero_duration_single_frame_returns_immediately);
    return UNITY_END();
}
