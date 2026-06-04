#include <unity.h>
#include <math.h>
#include "../../src/nervous_system/motion_math.h"
#include "../../src/nervous_system/neutral_pose.h"
#include "../../src/shared/config.h"

/* enforceShoulderLimits(): pure host-tested shoulder safety net.
 *
 * Operates in MATH-SPACE (firmware NEUTRAL[] convention). Two layers:
 *
 *   1. Per-side asymmetric hard clamp in URDF θ (signed displacement from rest):
 *        LEFT  (FL, BL):  θ ∈ [-SHOULDER_THETA_NARROW_DEG, +SHOULDER_THETA_WIDE_DEG]
 *        RIGHT (FR, BR):  θ ∈ [-SHOULDER_THETA_WIDE_DEG,   +SHOULDER_THETA_NARROW_DEG]
 *      (Mirror of left through the body-X axis — narrow side faces inward
 *       toward the body, wide side sweeps outward toward the neighbor.)
 *
 *   2. Same-side inter-leg gap — math-space shorter arc between the front and
 *      back leg on the same side must stay ≥ INTER_LEG_BUFFER_DEG. Front legs
 *      lead; back legs are pushed in whichever direction widens the gap.
 *      At rest each gap is 90° so the rule never fires (the legs are nowhere
 *      near each other); it only engages when a front leg swings most of the
 *      way back into its back neighbour's quadrant.
 */

static const double TOL = 1e-6;

void setUp(void) {}
void tearDown(void) {}

/* Helpers: math <-> URDF θ for each leg. */
static double to_theta(uint8_t leg, double math) { return math - NEUTRAL[leg].sh; }
static double to_math (uint8_t leg, double theta) { return NEUTRAL[leg].sh + theta; }

/* Shorter arc between two math angles, in [0, 180]. */
static double shorter_arc(double a, double b) {
    double d = fmod(b - a, 360.0);
    if (d < 0)   d += 360.0;
    if (d > 180) d  = 360.0 - d;
    return d;
}

/* (1) Rest pose is a fixed point: enforceShoulderLimits(NEUTRAL) == NEUTRAL.
 *      Each same-side gap is 90° at rest, well above the 5° buffer. */
void test_rest_is_identity(void) {
    double sh[4] = {NEUTRAL[0].sh, NEUTRAL[1].sh, NEUTRAL[2].sh, NEUTRAL[3].sh};
    enforceShoulderLimits(sh);
    for (uint8_t i = 0; i < 4; ++i) {
        TEST_ASSERT_DOUBLE_WITHIN(TOL, NEUTRAL[i].sh, sh[i]);
    }
}

/* (2) A modest sweep that doesn't get near the back-leg quadrant leaves the
 *     back legs untouched. Image-5 geometry: FR rotates 5° forward (toward
 *     FL); BR untouched at rest. Same-side gap grows from 90° to ~95°. */
void test_modest_sweep_leaves_back_legs_alone(void) {
    double sh[4];
    sh[LEG_FR] = to_math(LEG_FR, +5.0);
    sh[LEG_FL] = to_math(LEG_FL, -5.0);
    sh[LEG_RR] = NEUTRAL[LEG_RR].sh;
    sh[LEG_RL] = NEUTRAL[LEG_RL].sh;
    enforceShoulderLimits(sh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, NEUTRAL[LEG_RR].sh, sh[LEG_RR]);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, NEUTRAL[LEG_RL].sh, sh[LEG_RL]);
}

/* (3) Image-6 geometry on the RIGHT side: FR swung all the way back to URDF
 *     θ = -90° (math = -45°, which IS BR's rest direction). BR at rest has gap
 *     0° from FR; rule must push BR by 5° to widen the gap. The push direction
 *     is away from FR. FR sits at -45°; BR was at -45° (rest); pushing
 *     CCW gives math = -40°, CW gives -50°. The implementation picks the side
 *     that widens the existing relationship — when FR has come around from
 *     the +math side, BR is pushed further CW (more negative). */
void test_image6_right_side_gap_enforced(void) {
    double sh[4];
    sh[LEG_FR] = to_math(LEG_FR, -SHOULDER_THETA_WIDE_DEG);  /* -90° in θ */
    sh[LEG_FL] = NEUTRAL[LEG_FL].sh;
    sh[LEG_RR] = NEUTRAL[LEG_RR].sh;
    sh[LEG_RL] = NEUTRAL[LEG_RL].sh;
    enforceShoulderLimits(sh);
    /* FR untouched (front-leads) */
    TEST_ASSERT_DOUBLE_WITHIN(TOL, NEUTRAL[LEG_FR].sh - SHOULDER_THETA_WIDE_DEG,
                              sh[LEG_FR]);
    /* Same-side gap ≥ buffer */
    double gap = shorter_arc(sh[LEG_FR], sh[LEG_RR]);
    TEST_ASSERT_TRUE_MESSAGE(gap >= (double)INTER_LEG_BUFFER_DEG - 1e-6,
                             "RIGHT same-side gap below buffer");
}

/* (4) Image-6 LEFT side mirror: FL swung all the way to URDF +90° (math = -135°,
 *     which IS BL's rest). BL must be pushed to widen the gap by ≥ 5°. */
void test_image6_left_side_gap_enforced(void) {
    double sh[4];
    sh[LEG_FR] = NEUTRAL[LEG_FR].sh;
    sh[LEG_FL] = to_math(LEG_FL, +SHOULDER_THETA_WIDE_DEG);  /* +90° in θ */
    sh[LEG_RR] = NEUTRAL[LEG_RR].sh;
    sh[LEG_RL] = NEUTRAL[LEG_RL].sh;
    enforceShoulderLimits(sh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, NEUTRAL[LEG_FL].sh + SHOULDER_THETA_WIDE_DEG,
                              sh[LEG_FL]);
    double gap = shorter_arc(sh[LEG_FL], sh[LEG_RL]);
    TEST_ASSERT_TRUE_MESSAGE(gap >= (double)INTER_LEG_BUFFER_DEG - 1e-6,
                             "LEFT same-side gap below buffer");
}

/* (5) Hard clamp: front leg requested past its per-side envelope is truncated. */
void test_front_hard_clamp(void) {
    double sh[4];
    /* RIGHT side wide bound is on the URDF +Z negative side (toward BR's home),
     * which for FR is the loose +52° max in URDF θ space.
     * Note: SHOULDER_THETA_WIDE_DEG=90 is the URDF "loose" side; in the
     * per-side spec the RIGHT side has loose = +97° per-side = URDF θ +52°.
     * Test uses SHOULDER_THETA_NARROW_DEG for the upper bound on RIGHT. */
    sh[LEG_FR] = to_math(LEG_FR, +200.0);    /* way past +NARROW */
    sh[LEG_FL] = to_math(LEG_FL, -200.0);    /* way past -NARROW */
    sh[LEG_RR] = NEUTRAL[LEG_RR].sh;
    sh[LEG_RL] = NEUTRAL[LEG_RL].sh;
    enforceShoulderLimits(sh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, +SHOULDER_THETA_NARROW_DEG,
                              to_theta(LEG_FR, sh[LEG_FR]));
    TEST_ASSERT_DOUBLE_WITHIN(TOL, -SHOULDER_THETA_NARROW_DEG,
                              to_theta(LEG_FL, sh[LEG_FL]));
}

/* (6) Back-leg hard clamp: requested past its envelope is truncated, even if
 *     the inter-leg rule would have allowed it. */
void test_back_hard_clamp(void) {
    double sh[4];
    sh[LEG_FR] = NEUTRAL[LEG_FR].sh;
    sh[LEG_FL] = NEUTRAL[LEG_FL].sh;
    sh[LEG_RR] = to_math(LEG_RR, +200.0);
    sh[LEG_RL] = to_math(LEG_RL, -200.0);
    enforceShoulderLimits(sh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, +SHOULDER_THETA_NARROW_DEG,
                              to_theta(LEG_RR, sh[LEG_RR]));
    TEST_ASSERT_DOUBLE_WITHIN(TOL, -SHOULDER_THETA_NARROW_DEG,
                              to_theta(LEG_RL, sh[LEG_RL]));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_rest_is_identity);
    RUN_TEST(test_modest_sweep_leaves_back_legs_alone);
    RUN_TEST(test_image6_right_side_gap_enforced);
    RUN_TEST(test_image6_left_side_gap_enforced);
    RUN_TEST(test_front_hard_clamp);
    RUN_TEST(test_back_hard_clamp);
    return UNITY_END();
}
