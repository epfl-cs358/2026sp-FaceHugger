#include <unity.h>
#include "../../src/shared/config.h"
#include "../../src/nervous_system/neutral_pose.h"
#include "../../src/nervous_system/motion_math.h"

// Invariant: the boot / idle / graceful-stop / clip-return pose written by
// returnToDefaultAngles() (config.h *_DEFAULT_ANGLE) must equal the gait engine's
// neutral pose (translateToServo(NEUTRAL[leg])) for every joint. If they differ,
// the robot snaps between the two whenever a gait stops (observed: rear-knee
// oscillation, BR 130<->140 / BL 50<->55). Servo values are whole degrees, so a
// 0.5 tolerance catches any real drift.
static const double TOL = 0.5;

void setUp(void) {}
void tearDown(void) {}

static void check_leg(uint8_t leg, int hip_def, int thigh_def, int knee_def) {
    ServoTriple s = translateToServo(leg, NEUTRAL[leg].sh, NEUTRAL[leg].th, NEUTRAL[leg].kn);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, (double)hip_def,   s.hip);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, (double)thigh_def, s.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, (double)knee_def,  s.knee);
}

void test_fr_default_matches_neutral(void) {
    check_leg(0, FRONT_RIGHT_LEG_HIP_DEFAULT_ANGLE,
                 FRONT_RIGHT_LEG_THIGH_DEFAULT_ANGLE,
                 FRONT_RIGHT_LEG_KNEE_DEFAULT_ANGLE);
}
void test_fl_default_matches_neutral(void) {
    check_leg(1, FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE,
                 FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE,
                 FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE);
}
void test_br_default_matches_neutral(void) {
    check_leg(2, BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE,
                 BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE,
                 BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE);
}
void test_bl_default_matches_neutral(void) {
    check_leg(3, BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE,
                 BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE,
                 BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_fr_default_matches_neutral);
    RUN_TEST(test_fl_default_matches_neutral);
    RUN_TEST(test_br_default_matches_neutral);
    RUN_TEST(test_bl_default_matches_neutral);
    return UNITY_END();
}