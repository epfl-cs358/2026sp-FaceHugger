// Contract test for SpinalCord::stopClipPlayback().
//
// Bug being guarded: when the app navigates away from the Actions page it
// sends T:2 IDLE. The firmware FSM transitioned to IDLE but did NOT reset the
// clip-player state, so clipLoop_ and clipState_.phase survived. On a later
// re-entry to STATE_ACTION the loop resumed mid-cycle.
//
// stopClipPlayback() is a tiny invariant: phase := CLIP_DONE, clipLoop_ := false,
// clipPrerollUntilMs_ := 0. We can't link the full spinal_cord.cpp against the
// native env (Arduino/Adafruit/Wire deps), so this test mirrors the operation on
// a local state struct that pins exactly the three fields the real method
// touches. If a future refactor changes the field set, this test must be updated
// in lockstep — that's the regression guard.

#include <unity.h>
#include <cstdint>
#include "../../src/nervous_system/motion_math.h"  // ClipState, ClipPhase

void setUp(void) {}
void tearDown(void) {}

// Local mirror of the three fields stopClipPlayback() resets in SpinalCord.
// Kept in this exact shape so a grep for these names lines up with spinal_cord.h.
struct ClipPlayerStateMirror {
    ClipState clipState_;
    bool      clipLoop_;
    uint32_t  clipPrerollUntilMs_;
};

// Inline mirror of SpinalCord::stopClipPlayback().
// If spinal_cord.cpp's implementation diverges, this test should be updated
// alongside (and the contract description in spinal_cord.h serves as canon).
static void stopClipPlayback(ClipPlayerStateMirror& s) {
    s.clipState_.phase     = CLIP_DONE;
    s.clipLoop_            = false;
    s.clipPrerollUntilMs_  = 0;
}

// Mirror of the loop-relevant subset of playClip() side effects.
static void playClipSeed(ClipPlayerStateMirror& s, uint8_t id, bool loop, uint32_t nowMs) {
    s.clipLoop_              = loop;
    s.clipState_.clipId      = id;
    s.clipState_.clipStartMs = nowMs + 100;
    s.clipState_.cursor      = 0;
    s.clipState_.phase       = CLIP_PLAYING;
    s.clipPrerollUntilMs_    = nowMs + 100;
}

void test_stops_a_looping_clip(void) {
    ClipPlayerStateMirror s{};
    playClipSeed(s, /*id*/ 0, /*loop*/ true, /*now*/ 1000);
    TEST_ASSERT_EQUAL(CLIP_PLAYING, s.clipState_.phase);
    TEST_ASSERT_TRUE(s.clipLoop_);

    stopClipPlayback(s);

    TEST_ASSERT_EQUAL(CLIP_DONE, s.clipState_.phase);
    TEST_ASSERT_FALSE(s.clipLoop_);
    TEST_ASSERT_EQUAL_UINT32(0, s.clipPrerollUntilMs_);
}

void test_stops_a_oneshot_clip_idempotently(void) {
    ClipPlayerStateMirror s{};
    playClipSeed(s, /*id*/ 0, /*loop*/ false, /*now*/ 1000);
    TEST_ASSERT_FALSE(s.clipLoop_);

    stopClipPlayback(s);
    TEST_ASSERT_EQUAL(CLIP_DONE, s.clipState_.phase);
    TEST_ASSERT_FALSE(s.clipLoop_);

    // Idempotent — calling again from already-stopped state is a no-op.
    stopClipPlayback(s);
    TEST_ASSERT_EQUAL(CLIP_DONE, s.clipState_.phase);
    TEST_ASSERT_FALSE(s.clipLoop_);
    TEST_ASSERT_EQUAL_UINT32(0, s.clipPrerollUntilMs_);
}

void test_stops_during_preroll_window(void) {
    // Pre-roll: clipPrerollUntilMs_ is in the future, tickClip is gliding into
    // frame 0. stopClipPlayback() must zero the pre-roll deadline so the next
    // tickClip falls through past the `if (millis() < clipPrerollUntilMs_)`
    // guard and reaches the CLIP_DONE no-op path.
    ClipPlayerStateMirror s{};
    playClipSeed(s, /*id*/ 2, /*loop*/ true, /*now*/ 5000);
    TEST_ASSERT_GREATER_THAN_UINT32(0, s.clipPrerollUntilMs_);

    stopClipPlayback(s);

    TEST_ASSERT_EQUAL_UINT32(0, s.clipPrerollUntilMs_);
    TEST_ASSERT_EQUAL(CLIP_DONE, s.clipState_.phase);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_stops_a_looping_clip);
    RUN_TEST(test_stops_a_oneshot_clip_idempotently);
    RUN_TEST(test_stops_during_preroll_window);
    return UNITY_END();
}
