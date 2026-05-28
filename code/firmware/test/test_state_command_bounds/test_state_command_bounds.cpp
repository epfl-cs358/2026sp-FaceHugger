#include <unity.h>
#include "../../src/shared/data.h"

// Guards the CMD_STATE (T:2) handler range. The handler dispatches a wire-supplied
// state int through isValidStateCommand() before the switch; widening it to admit
// STATE_REST(4)/STATE_STAND(5) must NOT let out-of-range values through.
void setUp(void) {}
void tearDown(void) {}

void test_existing_states_still_valid(void) {
    TEST_ASSERT_TRUE(isValidStateCommand(STATE_IDLE));      // 0
    TEST_ASSERT_TRUE(isValidStateCommand(STATE_WALK));      // 1
    TEST_ASSERT_TRUE(isValidStateCommand(STATE_ACTION));    // 2
    TEST_ASSERT_TRUE(isValidStateCommand(STATE_FAILSAFE));  // 3
}

void test_new_pose_states_valid(void) {
    TEST_ASSERT_TRUE(isValidStateCommand(STATE_REST));   // 4 — flat / all-90 calibration
    TEST_ASSERT_TRUE(isValidStateCommand(STATE_STAND));  // 5 — standing / neutral
}

void test_out_of_range_rejected(void) {
    TEST_ASSERT_FALSE(isValidStateCommand(STATE_STAND + 1));  // 6
    TEST_ASSERT_FALSE(isValidStateCommand(-1));
    TEST_ASSERT_FALSE(isValidStateCommand(99));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_existing_states_still_valid);
    RUN_TEST(test_new_pose_states_valid);
    RUN_TEST(test_out_of_range_rejected);
    return UNITY_END();
}
