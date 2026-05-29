#include <unity.h>
#include "../../src/nervous_system/motion_math.h"
#include "../../src/shared/config.h"

// T:2 pose-ease dispatch (Task #11). The {T:2,s:REST/STAND} handler accepts an
// optional `dur_ms` field; when > 0, firmware eases the pose over that many
// milliseconds (clamped). clampPoseEaseMs() is the pure helper the network
// handler routes the wire value through before calling the timed overload, so
// it is the unit under test on the host.

void setUp(void) {}
void tearDown(void) {}

void test_zero_stays_zero(void) {
    // dur_ms == 0 means "snap" — must pass through unchanged so the handler
    // falls back to the existing instant relax() / stand() path.
    TEST_ASSERT_EQUAL_UINT32(0u, clampPoseEaseMs(0u));
}

void test_in_range_unchanged(void) {
    TEST_ASSERT_EQUAL_UINT32(1u,    clampPoseEaseMs(1u));
    TEST_ASSERT_EQUAL_UINT32(500u,  clampPoseEaseMs(500u));
    TEST_ASSERT_EQUAL_UINT32(1000u, clampPoseEaseMs(1000u));
    TEST_ASSERT_EQUAL_UINT32((uint32_t)POSE_EASE_MS_MAX, clampPoseEaseMs(POSE_EASE_MS_MAX));
}

void test_over_max_clamped_to_max(void) {
    TEST_ASSERT_EQUAL_UINT32((uint32_t)POSE_EASE_MS_MAX, clampPoseEaseMs(POSE_EASE_MS_MAX + 1u));
    TEST_ASSERT_EQUAL_UINT32((uint32_t)POSE_EASE_MS_MAX, clampPoseEaseMs(60000u));
    TEST_ASSERT_EQUAL_UINT32((uint32_t)POSE_EASE_MS_MAX, clampPoseEaseMs(0xFFFFFFFFu));
}

void test_max_constant_is_sane(void) {
    // Sanity: the clamp ceiling is positive and reasonable (under 1 minute).
    TEST_ASSERT_TRUE(POSE_EASE_MS_MAX > 0);
    TEST_ASSERT_TRUE(POSE_EASE_MS_MAX <= 60000);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_stays_zero);
    RUN_TEST(test_in_range_unchanged);
    RUN_TEST(test_over_max_clamped_to_max);
    RUN_TEST(test_max_constant_is_sane);
    return UNITY_END();
}
