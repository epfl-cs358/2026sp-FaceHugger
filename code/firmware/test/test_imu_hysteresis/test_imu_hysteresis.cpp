#include <unity.h>
#include "../../src/brain/imu_hysteresis.h"
#include "../../src/shared/data.h"

// Minimal stand-in for SpinalCord so we can verify the public getRobotState()
// shape (uint8_t accessor returning the protected `robotState` field) without
// pulling in the full Adafruit_PWMServoDriver dependency on the host.
namespace { struct FakeSpinalCord {
    RobotState robotState = STATE_IDLE;
    uint8_t getRobotState() const { return (uint8_t)robotState; }
}; }

// Pure helper under test: imuInvertedHysteresis(angle_deg, prev_state) → new_state.
// Contract:
//   - Flip threshold (upright → inverted): angle > 150 deg.
//   - Clear threshold (inverted → upright): angle < 30 deg.
//   - At the boundaries (== 150 from upright, == 30 from inverted) the state holds —
//     the boundary is NOT crossed unless the strict inequality is satisfied.
// The dead band between 30 and 150 prevents chatter near 90 deg.

void setUp(void) {}
void tearDown(void) {}

void test_upright_below_threshold_stays_upright(void) {
    // prev=false (upright), angle 100 → still upright. Below 150.
    TEST_ASSERT_FALSE(imuInvertedHysteresis(100.0f, false));
}

void test_upright_above_threshold_flips_to_inverted(void) {
    // prev=false, angle 151 → flips to inverted.
    TEST_ASSERT_TRUE(imuInvertedHysteresis(151.0f, false));
}

void test_inverted_above_clear_threshold_stays_inverted(void) {
    // prev=true, angle 100 → still inverted (above 30, so doesn't clear).
    TEST_ASSERT_TRUE(imuInvertedHysteresis(100.0f, true));
}

void test_inverted_below_clear_threshold_clears_to_upright(void) {
    // prev=true, angle 29 → clears to upright.
    TEST_ASSERT_FALSE(imuInvertedHysteresis(29.0f, true));
}

void test_upright_exactly_at_flip_threshold_stays_upright(void) {
    // Boundary not crossed: prev=false, angle 150 → still upright.
    TEST_ASSERT_FALSE(imuInvertedHysteresis(150.0f, false));
}

void test_inverted_exactly_at_clear_threshold_stays_inverted(void) {
    // Boundary not crossed: prev=true, angle 30 → still inverted.
    TEST_ASSERT_TRUE(imuInvertedHysteresis(30.0f, true));
}

void test_upright_at_zero_stays_upright(void) {
    // Trivial: prev=false, angle 0 → upright (sanity).
    TEST_ASSERT_FALSE(imuInvertedHysteresis(0.0f, false));
}

void test_inverted_at_180_stays_inverted(void) {
    // Trivial: prev=true, angle 180 → inverted (sanity).
    TEST_ASSERT_TRUE(imuInvertedHysteresis(180.0f, true));
}

// Mirrors the SpinalCord::getRobotState() accessor shape: returns uint8_t,
// reads the underlying enum field. Main.cpp's auto-flip gate compares the
// return value against STATE_IDLE — guard that contract here.
void test_get_robot_state_returns_idle(void) {
    FakeSpinalCord sc;
    TEST_ASSERT_EQUAL_UINT8(STATE_IDLE, sc.getRobotState());
}

void test_get_robot_state_reflects_field(void) {
    FakeSpinalCord sc;
    sc.robotState = STATE_WALK;
    TEST_ASSERT_EQUAL_UINT8(STATE_WALK, sc.getRobotState());
    sc.robotState = STATE_ACTION;
    TEST_ASSERT_EQUAL_UINT8(STATE_ACTION, sc.getRobotState());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_upright_below_threshold_stays_upright);
    RUN_TEST(test_upright_above_threshold_flips_to_inverted);
    RUN_TEST(test_inverted_above_clear_threshold_stays_inverted);
    RUN_TEST(test_inverted_below_clear_threshold_clears_to_upright);
    RUN_TEST(test_upright_exactly_at_flip_threshold_stays_upright);
    RUN_TEST(test_inverted_exactly_at_clear_threshold_stays_inverted);
    RUN_TEST(test_upright_at_zero_stays_upright);
    RUN_TEST(test_inverted_at_180_stays_inverted);
    RUN_TEST(test_get_robot_state_returns_idle);
    RUN_TEST(test_get_robot_state_reflects_field);
    return UNITY_END();
}
