#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"
#include "spinal_cord.h"
#include "servo.h"
#include "movements.h"
#include "../shared/config.h"

// Math-space neutral poses (shoulder, thigh, knee in degrees), indexed by LegId.
// These are the JS N[] values that have been physically tested on hardware.
static const struct { float sh, th, kn; } NEUTRAL[LEG_COUNT] = {
    {  45.0f, -60.0f, -37.0f },  // LEG_FR (0)
    {  75.0f, -60.0f, -40.0f },  // LEG_FL (1)
    { -45.0f, -50.0f, -50.0f },  // LEG_RR / BR (2)
    {-135.0f, -60.0f, -35.0f },  // LEG_RL / BL (3)
};

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
    leg1(Leg(driver, 0,
        Servo(driver, FRONT_RIGHT_LEG_HIP_PCA_CHANNEL,    FRONT_RIGHT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, FRONT_RIGHT_LEG_THIGH_PCA_CHANNEL,  FRONT_RIGHT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, FRONT_RIGHT_LEG_KNEE_PCA_CHANNEL,   FRONT_RIGHT_LEG_KNEE_DEFAULT_ANGLE)
    )),
    leg2(Leg(driver, 1,
        Servo(driver, FRONT_LEFT_LEG_HIP_PCA_CHANNEL,     FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, FRONT_LEFT_LEG_THIGH_PCA_CHANNEL,   FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, FRONT_LEFT_LEG_KNEE_PCA_CHANNEL,    FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE)
    )),
    leg3(Leg(driver, 2,
        Servo(driver, BOTTOM_RIGHT_LEG_HIP_PCA_CHANNEL,   BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_RIGHT_LEG_THIGH_PCA_CHANNEL, BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_RIGHT_LEG_KNEE_PCA_CHANNEL,  BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE)
    )),
    leg4(Leg(driver, 3,
        Servo(driver, BOTTOM_LEFT_LEG_HIP_PCA_CHANNEL,    BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_LEFT_LEG_THIGH_PCA_CHANNEL,  BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_LEFT_LEG_KNEE_PCA_CHANNEL,   BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE)
    )),
    currentGait_(GAIT_NONE),
    gaitPhaseStartMs_(0),
    targetX(0.0f), targetY(0.0f), targetYaw(0.0f),
    activeX(0.0f), activeY(0.0f), activeYaw(0.0f),
    isMovingRequested(false), lastCommandMs(0), isInverted(false)
{
}

void SpinalCord::begin() {
    driver.begin();
    driver.setPWMFreq(60);
    isInverted = false;
    leg1.returnToDefaultAngles();
    leg2.returnToDefaultAngles();
    leg3.returnToDefaultAngles();
    leg4.returnToDefaultAngles();
    
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
    else if (dir == "FW_R") { targetX =  0.0f; targetY =  0.0f; targetYaw = -1.0f; face.setState(EYES_RIGHT); }
    else if (dir == "FW_L") { targetX =  0.0f; targetY =  0.0f; targetYaw =  1.0f; face.setState(EYES_LEFT); }
    else if (dir == "BW_R") { targetX =  0.0f; targetY =  0.0f; targetYaw = -1.0f; face.setState(EYES_RIGHT); }
    else if (dir == "BW_L") { targetX =  0.0f; targetY =  0.0f; targetYaw =  1.0f; face.setState(EYES_LEFT); }
    else if (dir == "STOP") { targetX =  0.0f; targetY =  0.0f; targetYaw =  0.0f; face.setState(EYES_FRONT); }
}

void SpinalCord::walk()     { robotState = STATE_WALK; }
void SpinalCord::rest()     { robotState = STATE_IDLE; }
void SpinalCord::wallFlip() { robotState = STATE_ACTION; }

void SpinalCord::relax() {
    robotState = STATE_REST;
    leg1.setJointAngles(90, 90, 90);
    leg2.setJointAngles(90, 90, 90);
    leg3.setJointAngles(90, 90, 90);
    leg4.setJointAngles(90, 90, 90);
}

void SpinalCord::applyCalibration(int channel, int angle) {
    leg1.identifyAndMove(channel, (double)angle);
    leg2.identifyAndMove(channel, (double)angle);
    leg3.identifyAndMove(channel, (double)angle);
    leg4.identifyAndMove(channel, (double)angle);
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
        case STATE_IDLE:
        case STATE_REST:
        case STATE_ACTION:
            break;
        case STATE_FAILSAFE:
            leg1.returnToDefaultAngles();
            leg2.returnToDefaultAngles();
            leg3.returnToDefaultAngles();
            leg4.returnToDefaultAngles();
            break;
    }
    
    face.update();
}

void SpinalCord::setGait(GaitType g) {
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
        leg1.returnToDefaultAngles();
        leg2.returnToDefaultAngles();
        leg3.returnToDefaultAngles();
        leg4.returnToDefaultAngles();
        robotState = STATE_IDLE;
        return;
    }

    Leg* legs[LEG_COUNT] = { &leg1, &leg2, &leg3, &leg4 };
    const bool isCrab = (currentGait_ == GAIT_CRAB);

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
            lift  = sinf(p * (float)M_PI) * cfg.step_height_deg;
        }

        float sh = NEUTRAL[i].sh;
        float th = NEUTRAL[i].th;
        float kn = NEUTRAL[i].kn;

        // Shoulder sweep — forward/back (Y) + yaw rotation.
        // Crab suppresses forward (Y) but still allows yaw.
        // Front legs add the combined signal, rear legs subtract it.
        // Right side (FR, RR) adds yaw contribution; left side (FL, RL) subtracts it.
        {
            const float fwdDir     = (i == LEG_FR || i == LEG_FL) ? 1.0f : -1.0f;
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

        if (isInverted) { th = -th; kn = -kn; }

        // Translate math-space angles to servo angles (0–180°).
        // Mirrors the JS translateToServo() function exactly.
        double servoHip, servoThigh, servoKnee;
        switch (i) {
            case LEG_FR:
                servoHip   = 90.0 + (sh - 45.0);
                servoThigh = 90.0 - th;
                servoKnee  = 90.0 + kn;
                break;
            case LEG_FL:
                servoHip   = sh;
                servoThigh = 90.0 + th;
                servoKnee  = 90.0 - kn;
                break;
            case LEG_RR:
                servoHip   = 90.0 - (sh + 45.0);
                servoThigh = 90.0 + th;
                servoKnee  = 90.0 - kn;
                break;
            case LEG_RL:
                servoHip   = 90.0 + (sh + 135.0);
                servoThigh = 90.0 - th;
                servoKnee  = 90.0 + kn;
                break;
            default:
                continue;
        }

        legs[i]->setJointAngles(servoHip, servoThigh, servoKnee);
    }
}

// Port of the validated JS Trot script (v7.5 "strict 4-phase square motion").
// Front legs use 4 discrete hip positions per cycle with constant lift during swing;
// rear legs use a continuous hip sweep with sinusoidal lift during swing.
// All joint deltas are scaled by 2/3 around neutral to match scalePose() in the JS.
void SpinalCord::tickTrot() {
    constexpr float STEP_LENGTH = 40.0f;
    constexpr float STEP_HEIGHT = 50.0f;
    constexpr float DUTY        = 0.50f;
    constexpr float PERIOD_S    = 1.5f;
    constexpr float SCALE       = 2.0f / 3.0f;
    constexpr float YAW_GAIN    = 2.0f;

    // Phase offsets per leg index [FR, FL, RR, RL] — JS uses fr/bl=0.5, fl/br=0.0.
    static const float OFFSETS[LEG_COUNT] = { 0.5f, 0.0f, 0.0f, 0.5f };

    // Front leg hip end-points (math degrees), JS values: [FR, FL].
    static const float HIP_IN[2]  = { 65.0f,  75.0f };
    static const float HIP_OUT[2] = { 25.0f, 110.0f };

    // Magnitude of forward intent in [0, 1]; sign chooses direction.
    const float dirY = activeY;
    const float mag  = fminf(fabsf(dirY), 1.0f);

    const float t           = (millis() - gaitPhaseStartMs_) / 1000.0f;
    float globalPhase       = fmodf(t / PERIOD_S, 1.0f);
    if (dirY < 0.0f) globalPhase = 1.0f - globalPhase;  // backward = run cycle in reverse

    // Graceful stop on a clean phase boundary when the user released the stick.
    if (!isMovingRequested && mag < 0.05f && fabsf(activeYaw) < 0.05f && globalPhase < 0.05f) {
        leg1.returnToDefaultAngles();
        leg2.returnToDefaultAngles();
        leg3.returnToDefaultAngles();
        leg4.returnToDefaultAngles();
        robotState = STATE_IDLE;
        return;
    }

    Leg* legs[LEG_COUNT] = { &leg1, &leg2, &leg3, &leg4 };

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

            if      (legPhase >= 0.50f && legPhase <  0.75f) { sh = hipFront; lift = STEP_HEIGHT * mag; }
            else if (legPhase >= 0.75f && legPhase <= 1.00f) { sh = hipBack;  lift = STEP_HEIGHT * mag; }
            else if (legPhase >= 0.00f && legPhase <  0.25f) { sh = hipBack;  lift = 0.0f; }
            else                                              { sh = hipFront; lift = 0.0f; }
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
                lift     = sinf(progress * (float)M_PI) * STEP_HEIGHT * mag;
            }
            sh -= sweep;
        }

        // Lift applied to thigh (+) and knee (-), same convention as JS.
        th += lift;
        kn -= lift;

        // scalePose: scale (value - neutral) by 2/3 around the neutral pose.
        sh = NEUTRAL[i].sh + (sh - NEUTRAL[i].sh) * SCALE;
        th = NEUTRAL[i].th + (th - NEUTRAL[i].th) * SCALE;
        kn = NEUTRAL[i].kn + (kn - NEUTRAL[i].kn) * SCALE;

        if (isInverted) { th = -th; kn = -kn; }

        // Math → servo, identical to the JS translateToServo().
        double servoHip, servoThigh, servoKnee;
        switch (i) {
            case LEG_FR:
                servoHip   = 90.0 + (sh - 45.0);
                servoThigh = 90.0 - th;
                servoKnee  = 90.0 + kn;
                break;
            case LEG_FL:
                servoHip   = sh;
                servoThigh = 90.0 + th;
                servoKnee  = 90.0 - kn;
                break;
            case LEG_RR:
                servoHip   = 90.0 - (sh + 45.0);
                servoThigh = 90.0 + th;
                servoKnee  = 90.0 - kn;
                break;
            case LEG_RL:
                servoHip   = 90.0 + (sh + 135.0);
                servoThigh = 90.0 - th;
                servoKnee  = 90.0 + kn;
                break;
            default:
                continue;
        }

        legs[i]->setJointAngles(servoHip, servoThigh, servoKnee);
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

    // RR is flipped because its servoHip = 90 - (sh + 45) inverts sh;
    // the other three have servoHip = sh + const (no flip).
    static const float YAW_COEF[LEG_COUNT] = { -1.0f, -1.0f, +1.0f, -1.0f };

    const float t           = (millis() - gaitPhaseStartMs_) / 1000.0f;
    const float globalPhase = fmodf(t / PERIOD_S, 1.0f);

    Leg* legs[LEG_COUNT] = { &leg1, &leg2, &leg3, &leg4 };

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

        if (isInverted) { th = -th; kn = -kn; }

        double servoHip, servoThigh, servoKnee;
        switch (i) {
            case LEG_FR:
                servoHip   = 90.0 + (sh - 45.0);
                servoThigh = 90.0 - th;
                servoKnee  = 90.0 + kn;
                break;
            case LEG_FL:
                servoHip   = sh;
                servoThigh = 90.0 + th;
                servoKnee  = 90.0 - kn;
                break;
            case LEG_RR:
                servoHip   = 90.0 - (sh + 45.0);
                servoThigh = 90.0 + th;
                servoKnee  = 90.0 - kn;
                break;
            case LEG_RL:
                servoHip   = 90.0 + (sh + 135.0);
                servoThigh = 90.0 - th;
                servoKnee  = 90.0 + kn;
                break;
            default:
                continue;
        }
        legs[i]->setJointAngles(servoHip, servoThigh, servoKnee);
    }
}

void SpinalCord::invertRobot() {
    isInverted = !isInverted;

    if (isInverted) {
        leg1.setJointAngles(90,  30, 127);  // FR: 180-150, 180-53
        leg2.setJointAngles(75, 150,  50);  // FL: 180-30,  180-130
        leg3.setJointAngles(90, 140,  40);  // RR: 180-40,  180-140
        leg4.setJointAngles(90,  30, 125);  // RL: 180-150, 180-55
        face.setState(EYES_CONFUSED);
    } else {
        leg1.returnToDefaultAngles();
        leg2.returnToDefaultAngles();
        leg3.returnToDefaultAngles();
        leg4.returnToDefaultAngles();
        face.setState(EYES_FRONT);
    }

    gaitPhaseStartMs_ = millis();
}
