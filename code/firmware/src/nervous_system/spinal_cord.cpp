#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"
#include "spinal_cord.h"
#include "servo.h"
#include "movements.h"
#include "../shared/config.h"

// Ratios for the 4-phase swing (must sum to 1.0)
static const float SWING_RATIO_LIFT  = 0.20f; 
static const float SWING_RATIO_MOVE  = 0.60f; 
static const float SWING_RATIO_LOWER = 0.20f; 

static const GaitParams GAITS[] = {
    { 0.0f,  0.000f, 0.000f, 0.0f,  { 0.00f, 0.00f, 0.00f, 0.00f }, 'y',
      "None" },
    { 1.20f, 0.050f, 0.025f, 0.25f, { 0.50f, 0.00f, 0.25f, 0.75f }, 'y',
      "Static walk (FL -> RR -> FR -> RL)" },
    { 0.50f, 0.065f, 0.025f, 0.50f, { 0.50f, 0.00f, 0.00f, 0.50f }, 'y',
      "Trot (diagonal pairs: FL+RR | FR+RL)" },
    { 0.55f, 0.070f, 0.028f, 0.55f, { 0.50f, 0.00f, 0.00f, 0.50f }, 'x',
      "Crab walk (sideways, 45 deg splay)" },
};

//Robot starts in idle state
SpinalCord::SpinalCord(uint8_t pwm):
    robotState(STATE_IDLE),
    driver(Adafruit_PWMServoDriver(ADDR_SERVO_DRIVER)),
    leg1(Leg(driver, 0,
        Servo(driver, FRONT_RIGHT_LEG_HIP_PCA_CHANNEL, FRONT_RIGHT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, FRONT_RIGHT_LEG_THIGH_PCA_CHANNEL, FRONT_RIGHT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, FRONT_RIGHT_LEG_KNEE_PCA_CHANNEL, FRONT_RIGHT_LEG_KNEE_DEFAULT_ANGLE)
    )),
    leg2(Leg(driver, 1,
        Servo(driver, FRONT_LEFT_LEG_HIP_PCA_CHANNEL, FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, FRONT_LEFT_LEG_THIGH_PCA_CHANNEL, FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, FRONT_LEFT_LEG_KNEE_PCA_CHANNEL, FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE)
        )),
    leg3(Leg(driver, 2,
        Servo(driver, BOTTOM_RIGHT_LEG_HIP_PCA_CHANNEL, BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_RIGHT_LEG_THIGH_PCA_CHANNEL, BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_RIGHT_LEG_KNEE_PCA_CHANNEL, BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE)
        )),
    leg4(Leg(driver, 3,
        Servo(driver, BOTTOM_LEFT_LEG_HIP_PCA_CHANNEL, BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_LEFT_LEG_THIGH_PCA_CHANNEL, BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_LEFT_LEG_KNEE_PCA_CHANNEL, BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE)
        )),
    currentGait_(GAIT_NONE),
    gaitPhaseStartMs_(0),
    targetX(0.0f), targetY(0.0f), activeX(0.0f), activeY(0.0f),
    isMovingRequested(false), lastCommandMs(0)
{
}

void SpinalCord::begin() {
    driver.begin();
    driver.setPWMFreq(60); // Standard for servos
    
    // Now that the driver is alive, set the initial pose
    leg1.returnToDefaultAngles();
    leg2.returnToDefaultAngles();
    leg3.returnToDefaultAngles();
    leg4.returnToDefaultAngles();
}

void SpinalCord::processCommand(String dir, int state) {
    lastCommandMs = millis();
    isMovingRequested = (state == 1);

    if (!isMovingRequested) {
        targetX = 0.0f; targetY = 0.0f;
        return;
    }

    // Map discrete labels to Vectors
    if (dir == "FW")         { targetX = 0.0f;  targetY = 1.0f;  }
    else if (dir == "BW")    { targetX = 0.0f;  targetY = -1.0f; }
    else if (dir == "L")     { targetX = -1.0f; targetY = 0.0f;  }
    else if (dir == "R")     { targetX = 1.0f;  targetY = 0.0f;  }
    else if (dir == "FW_R")  { targetX = 0.7f;  targetY = 0.7f;  }
    else if (dir == "FW_L")  { targetX = -0.7f; targetY = 0.7f;  }
    else if (dir == "BW_R")  { targetX = 0.7f;  targetY = -0.7f; }
    else if (dir == "BW_L")  { targetX = -0.7f; targetY = -0.7f; }
}

void SpinalCord::walk(){ robotState = STATE_WALK; }
void SpinalCord::rest(){ robotState = STATE_IDLE; }
void SpinalCord::wallFlip(){ robotState = STATE_ACTION; }

void SpinalCord::applyCalibration(int channel, int angle) {
    leg1.identifyAndMove(channel, (double)angle);
    leg2.identifyAndMove(channel, (double)angle);
    leg3.identifyAndMove(channel, (double)angle);
    leg4.identifyAndMove(channel, (double)angle);
}

void SpinalCord::update(){
    // 1. Safety Deadman's Switch (500ms timeout)
    if (millis() - lastCommandMs > 500) {
        isMovingRequested = false;
        targetX = 0.0f; targetY = 0.0f;
    }

    // 2. Input Smoothing
    activeX += (targetX - activeX) * 0.1f;
    activeY += (targetY - activeY) * 0.1f;

    switch(robotState){
        case STATE_WALK:
            if (currentGait_ != GAIT_NONE) tickGait();
            break;
        case STATE_ACTION:
            break;
        case STATE_IDLE:
            break;
        case STATE_FAILSAFE:
            leg1.returnToDefaultAngles();
            leg2.returnToDefaultAngles();
            leg3.returnToDefaultAngles();
            leg4.returnToDefaultAngles();
            return;
    }
}

void SpinalCord::setGait(GaitType g){
    currentGait_ = g;
    gaitPhaseStartMs_ = millis();
    Serial.printf("[gait] %s\n", GAITS[g].label);
}

GaitType SpinalCord::currentGait() const { return currentGait_; }

void SpinalCord::tickGait(){
    const GaitParams& cfg = GAITS[currentGait_];
    const float t = (millis() - gaitPhaseStartMs_) / 1000.0f;
    const float globalPhase = fmodf(t / cfg.period_s, 1.0f);

    // GRACEFUL STOP: Stop only when motion is near zero AND phase resets
    if (!isMovingRequested && fabs(activeX) < 0.01f && fabs(activeY) < 0.01f && globalPhase < 0.05f) {
        robotState = STATE_IDLE;
        return;
    }

    Leg* legs[LEG_COUNT] = { &leg1, &leg2, &leg3, &leg4 };

    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        // Partner's original offset math
        const float legPhase = fmodf(globalPhase - cfg.offsets[i] + 1.0f, 1.0f);

        float x = 0, y = 0, z = 0;
        const float swingTime = 1.0f - cfg.duty;

        if (legPhase < cfg.duty) {
            // PHASE 4: STANCE (Ground contact)
            float s = legPhase / cfg.duty; 
            x = activeX * cfg.step_length_m * (0.5f - s);
            y = activeY * cfg.step_length_m * (0.5f - s);
            z = 0.0f;
        } else {
            // SWING PHASES (In the air)
            float normalizedSwing = (legPhase - cfg.duty) / swingTime;

            if (normalizedSwing < SWING_RATIO_LIFT) {
                // PHASE 1: LIFT
                float s = normalizedSwing / SWING_RATIO_LIFT;
                x = activeX * cfg.step_length_m * (-0.5f);
                y = activeY * cfg.step_length_m * (-0.5f);
                z = cfg.step_height_m * s;
            }
            else if (normalizedSwing < (SWING_RATIO_LIFT + SWING_RATIO_MOVE)) {
                // PHASE 2: MOVE (Horizontal Travel)
                float s = (normalizedSwing - SWING_RATIO_LIFT) / SWING_RATIO_MOVE;
                x = activeX * cfg.step_length_m * (-0.5f + s);
                y = activeY * cfg.step_length_m * (-0.5f + s);
                z = cfg.step_height_m;
            }
            else {
                // PHASE 3: LOWER (Landing)
                float s = (normalizedSwing - (SWING_RATIO_LIFT + SWING_RATIO_MOVE)) / SWING_RATIO_LOWER;
                x = activeX * cfg.step_length_m * (0.5f);
                y = activeY * cfg.step_length_m * (0.5f);
                z = cfg.step_height_m * (1.0f - s);
            }
        }

        z = applyIMUCorrection(z, i);
        legs[i]->setPose(x, y, z);
    }
}

float SpinalCord::applyIMUCorrection(float rawZ, uint8_t legIdx) {
    // Return rawZ for now. MPU6050 logic will plug in here later.
    return rawZ;
}