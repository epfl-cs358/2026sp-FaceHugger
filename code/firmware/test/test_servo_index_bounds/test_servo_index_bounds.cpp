#include <unity.h>
#include "../../src/shared/config.h"

// Guards the T:4 (CMD_CALIBRATE) handler: untrusted JSON ints index
// LEG_SERVO_CHANNEL[4][3]. isValidServoIndex() is the single source of truth
// for the in-range check; network.cpp's handler must reject anything it rejects.
void setUp(void) {}
void tearDown(void) {}

void test_valid_indices_accepted(void) {
    TEST_ASSERT_TRUE(isValidServoIndex(0, 0));  // first leg, first servo
    TEST_ASSERT_TRUE(isValidServoIndex(3, 2));  // last leg, last servo
    TEST_ASSERT_TRUE(isValidServoIndex(1, 1));  // interior
}

void test_leg_id_too_high_rejected(void) {
    TEST_ASSERT_FALSE(isValidServoIndex(4, 0));  // id == array length
}

void test_servo_id_too_high_rejected(void) {
    TEST_ASSERT_FALSE(isValidServoIndex(0, 3));  // servo_id == array length
}

void test_negative_leg_id_rejected(void) {
    TEST_ASSERT_FALSE(isValidServoIndex(-1, 0));
}

void test_negative_servo_id_rejected(void) {
    TEST_ASSERT_FALSE(isValidServoIndex(0, -1));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_valid_indices_accepted);
    RUN_TEST(test_leg_id_too_high_rejected);
    RUN_TEST(test_servo_id_too_high_rejected);
    RUN_TEST(test_negative_leg_id_rejected);
    RUN_TEST(test_negative_servo_id_rejected);
    return UNITY_END();
}
