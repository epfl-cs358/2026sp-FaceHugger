#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"
#include "spinal_cord.h"
#include "servo.h"
#include "motion_math.h"
#include "movements.h"
#include "neutral_pose.h"   // NEUTRAL[] — single source of truth (FL=135 post-Change-B)
#include "../shared/config.h"
#include "clips_all.h"   // generated FH_CLIPS[] (placeholder until export, see B6)

static const uint32_t CLIP_RETURN_MS = 500;  // ease to NEUTRAL at clip end

// Per-channel EMA smoothing applied to clip-playback angles ONLY (inside tickClip).
// smoothed = alpha*prev + (1-alpha)*target. Higher = smoother but laggier. The
// live value lives in clipEmaAlpha_ (runtime-tunable via T:11; boot default in
// spinal_cord.h). Clamped to [0, MAX] on set so playback can never fully stall.
static const float CLIP_EMA_ALPHA_MAX = 0.95f;

// T:6 (CMD_ACTION_SELECTION invert) is a stateless toggle; a duplicated/retried
// packet would double-flip. Ignore a second invert within this window. The app
// (PR #99) is the primary guard against the spam; this is firmware-side insurance.
static const uint32_t INVERT_DEBOUNCE_MS = 250;

// Glide time when an invert toggle mirrors the current pose in place.
static const uint32_t INVERT_EASE_MS = 300;

// Glide time when a gait stops gracefully and settles to the standing pose.
static const uint32_t GAIT_STOP_EASE_MS = 300;

// Glide time when a clip starts: ease from the live pose into the clip's first
// frame before real playback begins, so a clip never snaps from the current pose.
static const uint32_t CLIP_PREROLL_MS = 200;

// Gait parameters (step values are in degrees, pre-scaled to 2/3 of raw JS values).
// Offsets order: [LEG_FR, LEG_FL, LEG_RR, LEG_RL]
static const GaitParams GAITS[] = {
    { 0.0f, 0.0f,  0.0f,  0.0f,  { 0.0f,  0.0f,  0.0f,  0.0f  }, "None"        },
    { 2.0f, 26.7f, 26.7f, 0.75f, { 0.5f,  0.0f,  0.25f, 0.75f }, "Static Walk" },
    { 1.5f, 26.7f, 33.3f, 0.50f, { 0.5f,  0.0f,  0.0f,  0.5f  }, "Trot"        },
    { 1.5f, 20.0f, 33.3f, 0.50f, { 0.5f,  0.0f,  0.0f,  0.5f  }, "Crab"        },
};

SpinalCord::SpinalCord(uint8_t pwm):
    robotState(STATE_IDLE),
    driver(Adafruit_PWMServoDriver(ADDR_SERVO_DRIVER)),
    face(),
    legs_{
        Leg(driver, 0,  // FR
            Servo(driver, FRONT_RIGHT_LEG_HIP_PCA_CHANNEL,    FRONT_RIGHT_LEG_HIP_DEFAULT_ANGLE),
            Servo(driver, FRONT_RIGHT_LEG_THIGH_PCA_CHANNEL,  FRONT_RIGHT_LEG_THIGH_DEFAULT_ANGLE),
            Servo(driver, FRONT_RIGHT_LEG_KNEE_PCA_CHANNEL,   FRONT_RIGHT_LEG_KNEE_DEFAULT_ANGLE)),
        Leg(driver, 1,  // FL
            Servo(driver, FRONT_LEFT_LEG_HIP_PCA_CHANNEL,     FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE),
            Servo(driver, FRONT_LEFT_LEG_THIGH_PCA_CHANNEL,   FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE),
            Servo(driver, FRONT_LEFT_LEG_KNEE_PCA_CHANNEL,    FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE)),
        Leg(driver, 2,  // BR/RR
            Servo(driver, BOTTOM_RIGHT_LEG_HIP_PCA_CHANNEL,   BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE),
            Servo(driver, BOTTOM_RIGHT_LEG_THIGH_PCA_CHANNEL, BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE),
            Servo(driver, BOTTOM_RIGHT_LEG_KNEE_PCA_CHANNEL,  BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE)),
        Leg(driver, 3,  // BL/RL
            Servo(driver, BOTTOM_LEFT_LEG_HIP_PCA_CHANNEL,    BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE),
            Servo(driver, BOTTOM_LEFT_LEG_THIGH_PCA_CHANNEL,  BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE),
            Servo(driver, BOTTOM_LEFT_LEG_KNEE_PCA_CHANNEL,   BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE)),
    },
    currentGait_(GAIT_NONE),
    gaitPhaseStartMs_(0),
    targetX(0.0f), targetY(0.0f), targetYaw(0.0f),
    activeX(0.0f), activeY(0.0f), activeYaw(0.0f),
    isMovingRequested(false), lastCommandMs(0), isInverted(false)
    , clipState_{ CLIP_DONE, 0, 0, 0, 0 }
    , lastInvertMs_(0), hasInverted_(false)
{
}

void SpinalCord::begin() {
    driver.begin();
    driver.setPWMFreq(60);
    isInverted = false;
    hasInverted_  = false;   // clear invert-debounce state on boot
    lastInvertMs_ = 0;
    for (uint8_t i = 0; i < LEG_COUNT; ++i) legs_[i].returnToDefaultAngles();

    face.begin();
    face.setState(EYES_FRONT);
}

void SpinalCord::processCommand(String dir) {
    lastCommandMs = millis();
    isMovingRequested = (dir != "STOP");

    if      (dir == "FW")   { targetX =  0.0f; targetY =  1.0f; targetYaw =  0.0f; face.setState(EYES_FRONT); }
    else if (dir == "BW")   { targetX =  0.0f; targetY = -1.0f; targetYaw =  0.0f; face.setState(EYES_FRONT); }
    else if (dir == "L")    { targetX = -1.0f; targetY =  0.0f; targetYaw =  0.0f; face.setState(EYES_LEFT); }
    else if (dir == "R")    { targetX =  1.0f; targetY =  0.0f; targetYaw =  0.0f; face.setState(EYES_RIGHT); }
    // Eyes look opposite the rotation: turning left shows the right-eye image
    // and vice versa (rotation direction / targetYaw itself is unchanged).
    else if (dir == "FW_R") { targetX =  0.0f; targetY =  0.0f; targetYaw = -1.0f; face.setState(EYES_LEFT); }
    else if (dir == "FW_L") { targetX =  0.0f; targetY =  0.0f; targetYaw =  1.0f; face.setState(EYES_RIGHT); }
    else if (dir == "BW_R") { targetX =  0.0f; targetY =  0.0f; targetYaw = -1.0f; face.setState(EYES_LEFT); }
    else if (dir == "BW_L") { targetX =  0.0f; targetY =  0.0f; targetYaw =  1.0f; face.setState(EYES_RIGHT); }
    else if (dir == "STOP") { targetX =  0.0f; targetY =  0.0f; targetYaw =  0.0f; face.setState(EYES_FRONT); }
}

void SpinalCord::walk()     { robotState = STATE_WALK; }
void SpinalCord::rest()     { robotState = STATE_IDLE; }
void SpinalCord::wallFlip() { robotState = STATE_ACTION; }

void SpinalCord::relax() {
    // Flat / all-90 calibration pose: every servo at mechanical mid-travel. This is
    // the original calibration pose (restored — it predates the standing-NEUTRAL gait
    // engine). Calibration is done upright, so clear the invert flag too.
    robotState = STATE_REST;
    isInverted = false;
    for (uint8_t i = 0; i < LEG_COUNT; ++i) legs_[i].setJointAngles(90, 90, 90);
}

void SpinalCord::relax(uint32_t ms) {
    // T:2 with `dur_ms`: ease every joint to 90 over `ms`. We clear isInverted
    // up front (same as the snap) so the ease target is unambiguous; relax goes
    // to a calibration-flat pose where invert is meaningless anyway.
    robotState = STATE_REST;
    isInverted = false;
    for (uint8_t i = 0; i < LEG_COUNT; ++i) legs_[i].setJointAnglesTimed(90, 90, 90, ms);
}

// Single invert choke point — see header. Mirrors the pitch joints about 90 when
// inverted (180-x), shoulder untouched. For a pitch servo this equals negating the
// math-space angle (translateToServo emits 90±angle, and 180-(90±x)=90∓x), so routing
// gait output through here is bit-for-bit identical to the old math-space th=-th/kn=-kn.
void SpinalCord::applyServos(Leg* leg, ServoTriple s) {
    s = applyInvert(leg->legId(), s, isInverted);  // CALIB-aware pitch mirror; pure + host-tested
    leg->setJointAngles(s.hip, s.thigh, s.knee);
}

void SpinalCord::stand() {
    // Standing / neutral reference pose — the per-leg NEUTRAL[] table the gaits launch
    // from and ease back to. Mirrors tickGait at zero input (sweep=lift=0).
    robotState = STATE_STAND;
    goToNeutral();
}

void SpinalCord::stand(uint32_t ms) {
    // T:2 with `dur_ms`: ease the pose into (invert-aware) NEUTRAL over `ms`,
    // instead of the instant snap. easeToNeutral already routes through
    // applyInvert, so an inverted robot eases to the INVERTED neutral.
    robotState = STATE_STAND;
    easeToNeutral(ms);
}

void SpinalCord::goToNeutral() {
    // Invert-aware instant neutral: applyServos applies the pitch mirror when
    // isInverted, so an inverted robot holds the INVERTED neutral. Use this in
    // place of Leg::returnToDefaultAngles() on every gait/clip stop path — that
    // raw-defaults path bypasses the mirror and silently un-inverts the robot.
    for (uint8_t i = 0; i < LEG_COUNT; ++i)
        applyServos(&legs_[i], translateToServo(i, NEUTRAL[i].sh, NEUTRAL[i].th, NEUTRAL[i].kn));
}

void SpinalCord::easeToNeutral(uint32_t ms) {
    // Non-blocking ease to the (invert-aware) neutral, for the clip-return glide.
    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        ServoTriple n = applyInvert(
            i, translateToServo(i, NEUTRAL[i].sh, NEUTRAL[i].th, NEUTRAL[i].kn), isInverted);
        legs_[i].setJointAnglesTimed(n.hip, n.thigh, n.knee, ms);
    }
}

void SpinalCord::applyCalibration(int channel, int angle) {
    for (uint8_t i = 0; i < LEG_COUNT; ++i)
        legs_[i].identifyAndMove(channel, (double)angle);
}

void SpinalCord::update() {
    // Deadman's switch: zero targets if no command for 500 ms
    if (millis() - lastCommandMs > 500) {
        isMovingRequested = false;
        targetX   = 0.0f;
        targetY   = 0.0f;
        targetYaw = 0.0f;
    }

    // Input smoothing
    activeX   += (targetX   - activeX)   * 0.1f;
    activeY   += (targetY   - activeY)   * 0.1f;
    activeYaw += (targetYaw - activeYaw) * 0.1f;

    switch (robotState) {
        case STATE_WALK:
            if (currentGait_ != GAIT_NONE) tickGait();
            break;
        case STATE_ACTION:
            tickClip();
            break;
        case STATE_IDLE:
        case STATE_REST:
        case STATE_STAND:
            break;
        case STATE_FAILSAFE:
            for (uint8_t i = 0; i < LEG_COUNT; ++i) legs_[i].returnToDefaultAngles();
            break;
    }

    // Advance any in-progress eased move every tick, in EVERY state, so the invert
    // flip-in-place plays out even on the app's Actions tab (which sits in
    // STATE_ACTION). This is the single place eases are pumped (tickClip no longer
    // does it). A direct write (gait/stand/calibrate, or a clip's per-frame pose)
    // cancels the ease via setServoAngle, so this is a no-op unless a timed move
    // (invert, clip-return, gait-stop) is actually active.
    for (uint8_t i = 0; i < LEG_COUNT; ++i) legs_[i].tickEase();

    face.update();
}

void SpinalCord::setGait(GaitType g) {
    if (g < GAIT_NONE || g > GAIT_CRAB) return;  // guard once; network.cpp no longer needs to check
    currentGait_ = g;
    gaitPhaseStartMs_ = millis();
    Serial.printf("[gait] %s\n", GAITS[g].label);
}

GaitType SpinalCord::currentGait() const { return currentGait_; }

void SpinalCord::tickGait() {
    // Yaw input → coordinated in-place rotation, regardless of selected gait.
    // Lets the user spin in place from either TROT or CRAB with the diagonals.
    if (fabsf(activeYaw) > 0.05f) { tickYawRotation(); return; }

    if (currentGait_ == GAIT_TROT) { tickTrot(); return; }

    const GaitParams& cfg = GAITS[currentGait_];
    const float t           = (millis() - gaitPhaseStartMs_) / 1000.0f;
    const float globalPhase = fmodf(t / cfg.period_s, 1.0f);

    // Graceful stop: when movement requested is gone and active vector is near zero,
    // wait for a clean phase boundary then return to standing pose.
    if (!isMovingRequested && fabsf(activeX) < 0.01f && fabsf(activeY) < 0.01f && fabsf(activeYaw) < 0.01f && globalPhase < 0.05f) {
        easeToNeutral(GAIT_STOP_EASE_MS);  // invert-aware glide to the standing pose
        robotState = STATE_STAND;          // actively hold the stand (no manual T:2 needed)
        return;
    }


    const bool isCrab = (currentGait_ == GAIT_CRAB);

    // Scale foot lift by how much motion is actually commanded. Without this the
    // legs lift every cycle even with zero input, so the robot creeps on its own
    // (the "crab moves with no app input" bug). The sweep below is already
    // input-scaled, so only the lift needed gating.
    const float motionMag = fminf(1.0f, fmaxf(fabsf(activeX), fmaxf(fabsf(activeY), fabsf(activeYaw))));

    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        const float legPhase = fmodf(globalPhase - cfg.offsets[i] + 1.0f, 1.0f);

        float sweep, lift;
        if (legPhase < cfg.duty) {
            // Stance: foot on ground, body sweeps over it
            const float p = legPhase / cfg.duty;
            sweep = cfg.step_length_deg * (0.5f - p);
            lift  = 0.0f;
        } else {
            // Swing: foot in the air returning to front
            const float p = (legPhase - cfg.duty) / (1.0f - cfg.duty);
            sweep = cfg.step_length_deg * (-0.5f + p);
            lift  = sinf(p * (float)M_PI) * cfg.step_height_deg * motionMag;
        }

        float sh = NEUTRAL[i].sh;
        float th = NEUTRAL[i].th;
        float kn = NEUTRAL[i].kn;

        // Shoulder sweep — forward/back (Y) + yaw rotation.
        // Crab suppresses forward (Y) but still allows yaw.
        // Front legs add the combined signal, rear legs subtract it.
        // Right side (FR, RR) adds yaw contribution; left side (FL, RL) subtracts it.
        {
            // BR (LEG_RR) shoulder un-mirrored (2026-05-25): its whole shoulder
            // deviation must negate vs the old convention to keep servo output
            // identical, so BR joins the +1 group here (the old "front/rear"
            // split partly encoded BR's servo mirror).
            const float fwdDir     = (i == LEG_FR || i == LEG_FL || i == LEG_RR) ? 1.0f : -1.0f;
            const float yawDir     = (i == LEG_FR || i == LEG_RR) ? 1.0f : -1.0f;
            const float fwdContrib = isCrab ? 0.0f : activeY;
            sh += fwdDir * sweep * (fwdContrib + yawDir * activeYaw);
        }

        // Thigh sweep — lateral (X).
        // Main axis for CRAB; provides turning arc in TROT/WALK.
        // FL(1) and RL(3) are the left-side legs: -= sweep * activeX
        // FR(0) and RR(2) are the right-side legs: += sweep * activeX
        if (i == LEG_FL || i == LEG_RL) th -= sweep * activeX;
        else                             th += sweep * activeX;

        // Lift applied to thigh and knee during swing (same convention as JS)
        th += lift;
        kn -= lift;

        // Translate math-space angles to servo angles (0–180°).
        // Mirrors the JS translateToServo() function exactly. Invert (if any)
        // is applied centrally in applyServos (servo-space, pitch-only).
        applyServos(&legs_[i], translateToServo((uint8_t)i, sh, th, kn));
    }
}

// Trot. Rear legs: continuous sweep + sine lift (validated v7.5). Front legs: the
// STANCE keeps the validated discrete waypoints — the front foot slip-repositions on
// the ground and must NOT traction-sweep, or it counter-propels and cancels the rear
// legs. The SWING is smoothed (continuous hip return + sine lift arc): an airborne
// foot carries no traction, so smoothing it is propulsion-neutral and only softens the
// mechanical look. All joint deltas scaled by 2/3 around neutral (scalePose).
void SpinalCord::tickTrot() {
    constexpr float STEP_LENGTH = 55.0f;       // tunable: rear-leg stride amplitude (was 40)
    constexpr float STEP_HEIGHT = 50.0f;       // tunable: foot lift height
    constexpr float DUTY        = 0.50f;
    constexpr float PERIOD_S    = 1.5f;
    constexpr float SCALE       = 0.80f;       // tunable: overall amplitude vs neutral (was 2/3)
    constexpr float YAW_GAIN    = 2.0f;
    constexpr float LATERAL_STEP = 35.0f;      // tunable: sideways (crab) thigh-sweep amplitude

    // Phase offsets per leg index [FR, FL, RR, RL] — JS uses fr/bl=0.5, fl/br=0.0.
    static const float OFFSETS[LEG_COUNT] = { 0.5f, 0.0f, 0.0f, 0.5f };

    // Front leg hip end-points (math degrees), per leg [FR, FL]. FR is the original
    // JS value. FL was remapped by Change B: translateToServo went `servo = sh`
    // (neutral 75) → `90 + (sh-135)` (neutral 90, a horn remount), but these were
    // left stale and pulled FL 25-60° off neutral all cycle. Like Change B did for
    // tickGait, we keep the OLD sweep DELTA (back at neutral, front +35° forward)
    // and re-anchor it to the new neutral: HIP_IN = neutral (135), HIP_OUT = +35.
    static const float HIP_IN[2]  = { 65.0f, 135.0f };
    static const float HIP_OUT[2] = { 25.0f, 170.0f };

    // Magnitude of forward intent in [0, 1]; sign chooses direction. activeX is
    // the sideways (crab) intent — sign chooses left/right. liftMag gates the foot
    // lift on EITHER axis so a pure-sideways trot still picks the feet up.
    const float dirY = activeY;
    const float mag  = fminf(fabsf(dirY), 1.0f);
    const float liftMag = fminf(1.0f, fmaxf(mag, fabsf(activeX)));

    const float t           = (millis() - gaitPhaseStartMs_) / 1000.0f;
    float globalPhase       = fmodf(t / PERIOD_S, 1.0f);
    if (dirY < 0.0f) globalPhase = 1.0f - globalPhase;  // backward = run cycle in reverse

    // Graceful stop on a clean phase boundary when the user released the stick.
    if (!isMovingRequested && mag < 0.05f && fabsf(activeX) < 0.05f &&
        fabsf(activeYaw) < 0.05f && globalPhase < 0.05f) {
        easeToNeutral(GAIT_STOP_EASE_MS);  // invert-aware glide to the standing pose
        robotState = STATE_STAND;          // actively hold the stand (no manual T:2 needed)
        return;
    }



    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        const float legPhase = fmodf(globalPhase - OFFSETS[i] + 1.0f, 1.0f);

        float sh = NEUTRAL[i].sh;
        float th = NEUTRAL[i].th;
        float kn = NEUTRAL[i].kn;
        float lift = 0.0f;

        const bool isFront = (i == LEG_FR || i == LEG_FL);
        if (isFront) {
            const int idx = (i == LEG_FR) ? 0 : 1;
            // Anchor both endpoints at neutral and scale the delta by mag, so mag=0
            // (pure yaw input) holds the front legs exactly at neutral instead of
            // biasing FR ~13° forward and dragging the body backward during the spin.
            const float n        = NEUTRAL[i].sh;
            const float hipBack  = n + (HIP_IN[idx]  - n) * mag;
            const float hipFront = n + (HIP_OUT[idx] - n) * mag;

            if (legPhase < DUTY) {
                // Stance: KEEP the validated discrete waypoints. The front foot is a
                // near-static support that slip-repositions hipBack -> hipFront on the
                // ground. It must NOT traction-sweep here — a continuous ground sweep
                // turns the front legs into counter-propulsors that cancel the rear
                // legs (that was the "back legs are useless" bug).
                sh   = (legPhase < 0.25f) ? hipBack : hipFront;
                lift = 0.0f;
            } else {
                // Swing: foot is airborne, so smoothing here is propulsion-neutral.
                // Sweep the hip continuously hipFront -> hipBack and lift on a sine
                // arc, replacing the old rectangular lift block + mid-air hip snap.
                const float swing = (legPhase - DUTY) / (1.0f - DUTY);
                sh   = hipFront + (hipBack - hipFront) * swing;
                lift = sinf(swing * (float)M_PI) * STEP_HEIGHT * liftMag;
            }
        } else {
            // Rear legs: continuous hip sweep scaled by forward magnitude only
            // (yaw handled by the yawMode branch above).
            float sweep, progress;
            if (legPhase < DUTY) {
                progress = legPhase / DUTY;
                sweep    = STEP_LENGTH * (0.5f - progress) * mag;
            } else {
                progress = (legPhase - DUTY) / (1.0f - DUTY);
                sweep    = STEP_LENGTH * (-0.5f + progress) * mag;
                lift     = sinf(progress * (float)M_PI) * STEP_HEIGHT * liftMag;
            }
            // Per-leg sweep sign so the two rear shoulders mirror each other for a
            // straight trot (RR/BR +sweep, RL/BL -sweep). BR was un-mirrored
            // 2026-05-25 (translateToServo 90-(sh+45) → 90+(sh+45)); tickGait and
            // tickYawRotation flipped BR's sign in tandem but this was missed, so
            // both rear legs swept the SAME way and the back veered. +sweep here
            // restores BR's validated pre-branch servo output (90+sweep).
            if (i == LEG_RR) sh += sweep;
            else             sh -= sweep;  // RL/BL
        }

        // Sideways (crab) thigh sweep — additive, so the joystick's X axis gives a
        // sideways trot on top of the forward shoulder motion. Zero when activeX==0,
        // so a straight forward trot is byte-identical. Same diagonal-pair phase as
        // the trot; per-side sign matches tickGait's crab (FL/RL push one way, FR/RR
        // the other). activeX's sign chooses left vs right.
        float latTri;
        if (legPhase < DUTY) latTri = 0.5f - (legPhase / DUTY);
        else                 latTri = -0.5f + (legPhase - DUTY) / (1.0f - DUTY);
        const float latSweep = LATERAL_STEP * latTri * activeX;
        if (i == LEG_FL || i == LEG_RL) th -= latSweep;
        else                            th += latSweep;

        // Lift applied to thigh (+) and knee (-), same convention as JS.
        th += lift;
        kn -= lift;

        // scalePose: scale (value - neutral) by 2/3 around the neutral pose.
        sh = NEUTRAL[i].sh + (sh - NEUTRAL[i].sh) * SCALE;
        th = NEUTRAL[i].th + (th - NEUTRAL[i].th) * SCALE;
        kn = NEUTRAL[i].kn + (kn - NEUTRAL[i].kn) * SCALE;

        // Math → servo, identical to the JS translateToServo(). Invert applied
        // centrally in applyServos (servo-space, pitch-only).
        applyServos(&legs_[i], translateToServo((uint8_t)i, sh, th, kn));
    }
}

// Gait-independent in-place rotation. Called from tickGait whenever |activeYaw|
// is significant. All 4 legs sweep their hips in a diagonal trot pattern with
// per-leg signs chosen so every servo hip rotates the SAME way during its stance.
// The two stance-phase forces cancel translationally and only torque remains —
// body stays put, body rotates. Works identically under TROT or CRAB.
void SpinalCord::tickYawRotation() {
    constexpr float STEP_LENGTH = 40.0f;
    constexpr float STEP_HEIGHT = 50.0f;
    constexpr float DUTY        = 0.50f;
    constexpr float PERIOD_S    = 1.5f;
    constexpr float SCALE       = 2.0f / 3.0f;
    constexpr float YAW_GAIN    = 2.0f;

    // Diagonal trot pairing: FL+RR in stance during one half, FR+RL the other.
    static const float OFFSETS[LEG_COUNT] = { 0.5f, 0.0f, 0.0f, 0.5f };

    // All four shoulders now have servoHip = sh + const (BR un-mirrored
    // 2026-05-25), so YAW_COEF is uniform. The old BR=+1 cancelled BR's former
    // servo mirror; flipping it to -1 in tandem with translateToServo keeps the
    // servo output (and thus the turn) byte-identical.
    static const float YAW_COEF[LEG_COUNT] = { -1.0f, -1.0f, -1.0f, -1.0f };

    const float t           = (millis() - gaitPhaseStartMs_) / 1000.0f;
    const float globalPhase = fmodf(t / PERIOD_S, 1.0f);



    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        const float legPhase = fmodf(globalPhase - OFFSETS[i] + 1.0f, 1.0f);

        float sh = NEUTRAL[i].sh;
        float th = NEUTRAL[i].th;
        float kn = NEUTRAL[i].kn;
        float lift = 0.0f;

        float sweep, progress;
        if (legPhase < DUTY) {
            progress = legPhase / DUTY;
            sweep    = STEP_LENGTH * (0.5f - progress);
        } else {
            progress = (legPhase - DUTY) / (1.0f - DUTY);
            sweep    = STEP_LENGTH * (-0.5f + progress);
            lift     = sinf(progress * (float)M_PI) * STEP_HEIGHT * fminf(fabsf(activeYaw), 1.0f);
        }
        sh -= sweep * YAW_COEF[i] * activeYaw * YAW_GAIN;

        th += lift;
        kn -= lift;

        sh = NEUTRAL[i].sh + (sh - NEUTRAL[i].sh) * SCALE;
        th = NEUTRAL[i].th + (th - NEUTRAL[i].th) * SCALE;
        kn = NEUTRAL[i].kn + (kn - NEUTRAL[i].kn) * SCALE;

        // Invert applied centrally in applyServos (servo-space, pitch-only).
        applyServos(&legs_[i], translateToServo((uint8_t)i, sh, th, kn));
    }
}

void SpinalCord::setClipSmoothing(float alpha) {
    // Runtime smoothing knob (T:11). Clamp to [0, MAX]: 0 = no smoothing
    // (snappy, exact frames), higher = smoother but laggier. Capped below 1 so
    // the EMA always converges and playback can't stall.
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > CLIP_EMA_ALPHA_MAX) alpha = CLIP_EMA_ALPHA_MAX;
    clipEmaAlpha_ = alpha;
    Serial.printf("[clip] smoothing alpha = %.2f\n", clipEmaAlpha_);
}

void SpinalCord::stopClipPlayback() {
    // Reset the clip player's lifecycle state. Without this, T:2 IDLE only
    // changed the FSM and a `loop=true` clip would resume next time the FSM
    // re-entered STATE_ACTION (clipLoop_ + clipState_.phase survived).
    // Pose-easing is handled separately by the T:2 dispatch (rest()/stand()).
    clipState_.phase       = CLIP_DONE;
    clipLoop_              = false;
    clipPrerollUntilMs_    = 0;
}

void SpinalCord::playClip(uint8_t id, bool loop) {
    if (id >= FH_CLIP_COUNT) {
        Serial.printf("[clip] ignored: id %u >= %u\n", id, (unsigned)FH_CLIP_COUNT);
        return;
    }
    if (FH_CLIPS[id].frame_count == 0) {
        Serial.printf("[clip] ignored: '%s' has 0 frames\n", FH_CLIPS[id].name);
        return;
    }
    clipLoop_ = loop;
    uint32_t now = millis();
    clipState_.clipId      = id;
    // Real playback starts after the pre-roll ease, so the clip clock begins then.
    clipPrerollUntilMs_    = now + CLIP_PREROLL_MS;
    clipState_.clipStartMs = clipPrerollUntilMs_;
    clipState_.returnStartMs = 0;
    clipState_.cursor      = 0;
    clipState_.phase       = CLIP_PLAYING;
    // Seed the EMA from frame 0 so playback starts on the true first pose
    // (no ramp-up from a stale value).
    for (uint8_t i = 0; i < LEG_COUNT; ++i)
        for (uint8_t j = 0; j < 3; ++j)
            clipSmoothed_[i][j] = FH_CLIPS[id].frames[0].a[i * 3 + j];
    // Pre-roll: ease from whatever pose the robot is holding into frame 0's pose
    // (same servo target the first playback tick would apply), so the clip glides
    // in instead of snapping. Real playback begins once the ease completes.

    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        ServoTriple f0 = applyInvert(
            i,
            clampClipServos(i, translateToServo(i,
                FH_CLIPS[id].frames[0].a[i * 3 + 0],
                FH_CLIPS[id].frames[0].a[i * 3 + 1],
                FH_CLIPS[id].frames[0].a[i * 3 + 2])),
            isInverted);
        legs_[i].setJointAnglesTimed(f0.hip, f0.thigh, f0.knee, CLIP_PREROLL_MS);
    }
    robotState = STATE_ACTION;   // pre-empts any running gait (single motion owner)
    Serial.printf("[clip] play %s (%u frames)%s\n",
                  FH_CLIPS[id].name, FH_CLIPS[id].frame_count,
                  loop ? " [loop]" : "");
}

void SpinalCord::tickClip() {


    // Pre-roll: glide from the pose we were holding into frame 0 before playback.
    // Just advance the eased move; do not run the clip clock yet.
    if (millis() < clipPrerollUntilMs_) {
        for (uint8_t i = 0; i < LEG_COUNT; ++i) legs_[i].tickEase();
        return;
    }

    const FhClip& clip = FH_CLIPS[clipState_.clipId];

    ClipStep step = clipPlayerStep(&clipState_, millis(),
                                   clip.duration_ms, CLIP_RETURN_MS);
    switch (step.action) {
        case CLIP_ACT_APPLY_POSE:
        case CLIP_ACT_BEGIN_RETURN: {
            float a[12];
            clipPoseAt(clip.frames, clip.frame_count, step.elapsed_ms,
                       &clipState_.cursor, a);
            // Per-channel EMA on the math-space angle, BEFORE translateToServo —
            // clip path only; gaits and calibration never touch clipSmoothed_.
            for (uint8_t i = 0; i < LEG_COUNT; ++i) {
                for (uint8_t j = 0; j < 3; ++j)
                    clipSmoothed_[i][j] = emaStep(clipSmoothed_[i][j],
                                                  a[i*3+j], clipEmaAlpha_);
                // EMA smooths the math-space angle; clampClipServos keeps the clip
                // off each leg's mechanical stop (clip path only); invert (if any)
                // applied last at the write point.
                applyServos(&legs_[i], clampClipServos(i,
                    translateToServo(i, clipSmoothed_[i][0],
                                     clipSmoothed_[i][1],
                                     clipSmoothed_[i][2])));
            }
            if (step.action == CLIP_ACT_BEGIN_RETURN) {
                if (clipLoop_) {
                    // Loop: the final pose was just applied; replay from frame 0
                    // instead of returning to neutral. Reset the clip clock + cursor
                    // and stay in PLAYING; the EMA carries over so the wrap is
                    // smoothed. Stop by sending any other motion (gait / T:2 / a new
                    // clip), which preempts STATE_ACTION.
                    clipState_.clipStartMs = millis();
                    clipState_.cursor      = 0;
                    clipState_.phase       = CLIP_PLAYING;
                } else {
                    // Start the non-blocking ease to the (invert-aware) neutral — so a
                    // clip that ends while inverted glides to the inverted neutral
                    // instead of snapping upright.
                    easeToNeutral(CLIP_RETURN_MS);
                }
            }
            break;
        }
        case CLIP_ACT_EASE:
            // The clip-return ease is advanced by update()'s per-tick tickEase.
            break;
        case CLIP_ACT_FINISH:
            robotState = STATE_STAND;   // hold the neutral stand, not inert IDLE
            break;
        case CLIP_ACT_NONE:
        default:
            break;
    }
}

void SpinalCord::flipPoseInPlace(uint32_t ms) {
    // Mirror whatever the servos are holding right now: 180 - angle on thigh and
    // knee only, shoulder unchanged (== applyInvert). Eased via setJointAnglesTimed
    // so the flip is a glide from the current pose, not a snap to neutral. The
    // pitch mirror is an involution, so this is continuous with the mirrored
    // gait/stand poses subsequent ticks produce. (During an active gait the next
    // tick overrides these targets with the mirrored gait pose, which is
    // continuous, so this matters while standing, idle, or holding a clip's end.)

    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        float h, t, k;
        legs_[i].getJointAngles(h, t, k);
        ServoTriple m = applyInvert(i, { (double)h, (double)t, (double)k }, true);
        legs_[i].setJointAnglesTimed(m.hip, m.thigh, m.knee, ms);
    }
}

void SpinalCord::setInverted(bool flag) {
    if (flag == isInverted) return;  // no change → nothing to re-pose
    isInverted = flag;
    face.setState(isInverted ? EYES_CONFUSED : EYES_FRONT);
    flipPoseInPlace(INVERT_EASE_MS);  // T:9: mirror the live pose in place
}

void SpinalCord::setAutoInvertEnabled(bool enabled) {
    // No-op when unchanged; pose is NOT touched either way. Disabling freezes
    // isInverted at its current value (main.cpp's loop will simply skip the
    // setInverted() call). Enabling re-arms the gate; the IMU latch will catch
    // up on the next state-change loop tick.
    autoInvertEnabled_ = enabled;
}

void SpinalCord::tickAutoInvert(bool imuInverted) {
    // Edge-triggered: only act when the latch flips. If auto-invert is OFF we
    // intentionally do NOT advance prevImuInverted_, so re-enabling the toggle
    // re-evaluates against the LAST-FORWARDED value (the IMU may have moved
    // while we were frozen, and we want the next change to fire setInverted).
    if (imuInverted == prevImuInverted_) return;
    if (!autoInvertEnabled_) return;
    setInverted(imuInverted);
    prevImuInverted_ = imuInverted;
}

void SpinalCord::invertRobot() {
    // Debounce a duplicated/retried T:6 so it can't double-flip (the first invert
    // is always honoured; only a SECOND within the window is dropped).
    uint32_t now = millis();
    if (hasInverted_ && (now - lastInvertMs_) < INVERT_DEBOUNCE_MS) return;
    hasInverted_  = true;
    lastInvertMs_ = now;

    isInverted = !isInverted;
    face.setState(isInverted ? EYES_CONFUSED : EYES_FRONT);
    gaitPhaseStartMs_ = now;  // keep gait timing coherent if a gait is running
    // T:6: mirror the live pose in place (eased) so the flip applies to whatever
    // the robot is currently holding instead of waiting for the next motion tick.
    flipPoseInPlace(INVERT_EASE_MS);
}

SpinalCord::Snapshot SpinalCord::snapshot() const {
    Snapshot s;
    s.robot_state     = (uint8_t)robotState;
    s.gait            = (uint8_t)currentGait_;
    s.is_moving       = isMovingRequested;
    s.is_inverted     = isInverted;
    s.last_cmd_ms     = lastCommandMs;
    s.target_x = targetX; s.target_y = targetY; s.target_yaw = targetYaw;
    s.active_x = activeX; s.active_y = activeY; s.active_yaw = activeYaw;
    for (uint8_t i = 0; i < LEG_COUNT; ++i)
        legs_[i].getJointAngles(s.servo_angles[i*3], s.servo_angles[i*3+1], s.servo_angles[i*3+2]);
    return s;
}
