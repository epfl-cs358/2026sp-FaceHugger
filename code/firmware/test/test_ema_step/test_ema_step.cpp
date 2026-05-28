#include <unity.h>
#include "../../src/nervous_system/motion_math.h"

// emaStep() is the per-channel exponential moving average applied to clip-playback
// angles inside tickClip() (and ONLY there): smoothed = alpha*prev + (1-alpha)*target.
// Higher alpha = smoother/laggier. It must never overshoot and must converge to a
// held target.
static const float TOL = 1e-4f;

void setUp(void) {}
void tearDown(void) {}

void test_single_step_blends_toward_target(void) {
    // alpha=0.75: one step moves 25% of the way from 0 to 10 -> 2.5
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.5f, emaStep(0.0f, 10.0f, 0.75f));
}

void test_alpha_zero_is_passthrough(void) {
    TEST_ASSERT_FLOAT_WITHIN(TOL, 10.0f, emaStep(0.0f, 10.0f, 0.0f));  // no smoothing
}

void test_converges_to_constant_input(void) {
    float v = 0.0f;
    for (int i = 0; i < 100; ++i) v = emaStep(v, 10.0f, 0.75f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, v);  // settles on the held target
}

void test_no_overshoot(void) {
    // Rising: result stays within [prev, target]; never past the target.
    float v = 0.0f;
    for (int i = 0; i < 100; ++i) {
        float next = emaStep(v, 10.0f, 0.75f);
        TEST_ASSERT_TRUE(next >= v - TOL);       // monotonic up
        TEST_ASSERT_TRUE(next <= 10.0f + TOL);   // never overshoots
        v = next;
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_single_step_blends_toward_target);
    RUN_TEST(test_alpha_zero_is_passthrough);
    RUN_TEST(test_converges_to_constant_input);
    RUN_TEST(test_no_overshoot);
    return UNITY_END();
}
