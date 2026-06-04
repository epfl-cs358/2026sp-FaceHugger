#include <unity.h>
#include "../../src/nervous_system/motion_math.h"
#include "../../src/shared/config.h"

// clampClipServos(legId, s): conservative per-joint servo clamp applied ONLY on
// the clip playback path (temporary safeguard, see tickClip). The envelope is
// centered on each joint's per-leg CALIB (the servo angle that = mechanical zero
// post-calibration), sized from the URDF joint limits:
//   hip   uniform [90 - HIP_CLAMP_FROM_NINETY, 90 + HIP_CLAMP_FROM_NINETY]
//         = [38, 142] — tight side of every URDF shoulder, safe on every leg.
//   thigh [CALIB_THIGH_BY_LEG[leg] ± THIGH_CLAMP_FROM_CALIB] (URDF ±60° symmetric)
//   knee  [CALIB_KNEE_BY_LEG[leg]  ± KNEE_CLAMP_FROM_CALIB ] (URDF ±90° symmetric)
static const double TOL = 1e-9;

void setUp(void) {}
void tearDown(void) {}

// ─── Hip clamp (uniform across legs) ────────────────────────────────────────

void test_in_range_passthrough(void) {
    // Mid-range (90,90,90) sits inside every leg's thigh/knee window since
    // each CALIB is within ±60/±90 of 90. Pass-through expected.
    for (uint8_t leg = 0; leg < 4; ++leg) {
        ServoTriple s = clampClipServos(leg, {90.0, 90.0, 90.0});
        TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, s.hip);
        TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, s.thigh);
        TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0, s.knee);
    }
}

void test_hip_clamped_to_38_142_every_leg(void) {
    // Hip envelope is uniform: 90 ± HIP_CLAMP_FROM_NINETY. Verify on every leg.
    for (uint8_t leg = 0; leg < 4; ++leg) {
        TEST_ASSERT_DOUBLE_WITHIN(TOL, 142.0, clampClipServos(leg, {187.5, 90, 90}).hip);
        TEST_ASSERT_DOUBLE_WITHIN(TOL,  38.0, clampClipServos(leg, { 15.0, 90, 90}).hip);
        TEST_ASSERT_DOUBLE_WITHIN(TOL, 120.0, clampClipServos(leg, {120.0, 90, 90}).hip);
    }
}

// ─── Thigh clamp (per-leg, CALIB ± THIGH_CLAMP_FROM_CALIB) ──────────────────

void test_thigh_FR_window(void) {
    // CALIB_FR_THIGH = 84 → window [9, 159] (THIGH_CLAMP_FROM_CALIB = 75,
    // tracking URDF thigh ±75°).
    TEST_ASSERT_DOUBLE_WITHIN(TOL,  9.0, clampClipServos(0, {90,  9.0, 90}).thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL,  9.0, clampClipServos(0, {90,  8.0, 90}).thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 159.0, clampClipServos(0, {90, 159.0, 90}).thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 159.0, clampClipServos(0, {90, 160.0, 90}).thigh);
}

void test_thigh_FL_neutral_27_no_longer_clipped(void) {
    // This is the case that motivated the per-leg CALIB-relative clamp.
    // CALIB_FL_THIGH = 87 → window [12, 162] under ±75°. FL NEUTRAL = 27
    // (servo) sits comfortably inside the window; under the old uniform
    // [30, 150] this was clipped to 30.
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 27.0, clampClipServos(1, {90, 27.0, 90}).thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 12.0, clampClipServos(1, {90, 11.0, 90}).thigh);
}

void test_thigh_BR_window(void) {
    // CALIB_BR_THIGH = 103 → window [28, 178].
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 178.0, clampClipServos(2, {90, 178.0, 90}).thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 178.0, clampClipServos(2, {90, 179.0, 90}).thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL,  28.0, clampClipServos(2, {90,  28.0, 90}).thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL,  28.0, clampClipServos(2, {90,  27.0, 90}).thigh);
}

void test_thigh_BL_window(void) {
    // CALIB_BL_THIGH = 84 → window [9, 159].
    TEST_ASSERT_DOUBLE_WITHIN(TOL,   9.0, clampClipServos(3, {90,   9.0, 90}).thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 159.0, clampClipServos(3, {90, 159.0, 90}).thigh);
}

// ─── Knee clamp (per-leg, CALIB ± KNEE_CLAMP_FROM_CALIB) ────────────────────

void test_knee_FR_window(void) {
    // CALIB_FR_KNEE = 95 → window [5, 180] (clipped against 180 hard ceiling — see note).
    // Note: KNEE_CLAMP_FROM_CALIB = 90 is symmetric, so the upper bound is 185
    // for FR, but servos saturate at 180. The clamp implementation uses the
    // CALIB-relative window directly; values above 180 would still be returned
    // up to CALIB+90. We test the window boundaries themselves.
    TEST_ASSERT_DOUBLE_WITHIN(TOL,   5.0, clampClipServos(0, {90, 90,   5.0}).knee);
    TEST_ASSERT_DOUBLE_WITHIN(TOL,   5.0, clampClipServos(0, {90, 90,   4.0}).knee);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 185.0, clampClipServos(0, {90, 90, 186.0}).knee);
}

void test_knee_FL_window(void) {
    // CALIB_FL_KNEE = 82 → window [-8, 172]. The lower bound is below 0 since
    // the URDF window is wider than the servo's [0,180] on the negative side.
    // The clamp itself is CALIB-relative; if a clip emits a negative servo
    // angle below -8 it is bumped up to -8. Downstream the servo write rejects
    // anything < 0 anyway. We test the window itself.
    TEST_ASSERT_DOUBLE_WITHIN(TOL,  -8.0, clampClipServos(1, {90, 90,  -9.0}).knee);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 172.0, clampClipServos(1, {90, 90, 172.0}).knee);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 172.0, clampClipServos(1, {90, 90, 173.0}).knee);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_in_range_passthrough);
    RUN_TEST(test_hip_clamped_to_38_142_every_leg);
    RUN_TEST(test_thigh_FR_window);
    RUN_TEST(test_thigh_FL_neutral_27_no_longer_clipped);
    RUN_TEST(test_thigh_BR_window);
    RUN_TEST(test_thigh_BL_window);
    RUN_TEST(test_knee_FR_window);
    RUN_TEST(test_knee_FL_window);
    return UNITY_END();
}
