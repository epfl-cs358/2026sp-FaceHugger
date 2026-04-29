#include <unity.h>
#include <stdio.h>

#include "../../src/nervous_system/kinematics.h"
#include "reference_data.h"

static const float TOL_RAD = 0.01f;

void setUp(void) {}
void tearDown(void) {}

void test_ik_matches_sim(void) {
    for (int i = 0; i < IK_REF_COUNT; ++i) {
        const IKRefCase& c = IK_REF_CASES[i];
        const JointAngles got = legIK((LegId)c.leg_id, c.x, c.y, c.z);

        char msg[96];
        snprintf(msg, sizeof(msg), "case %d (leg=%d) shoulder", i, c.leg_id);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_RAD, c.shoulder, got.shoulder, msg);
        snprintf(msg, sizeof(msg), "case %d (leg=%d) hip", i, c.leg_id);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_RAD, c.hip, got.hip, msg);
        snprintf(msg, sizeof(msg), "case %d (leg=%d) knee", i, c.leg_id);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOL_RAD, c.knee, got.knee, msg);
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_ik_matches_sim);
    return UNITY_END();
}
