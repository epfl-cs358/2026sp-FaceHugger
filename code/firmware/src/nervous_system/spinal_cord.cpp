#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"
#include "spinal_cord.h"
#include "servo.h"
#include "movements.h"
#include "../shared/config.h"

// Mirrors code/simulation/gaits.py registry. Indexed by GaitType.
// offsets order matches LegId: FR, FL, RR, RL.
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
    gaitPhaseStartMs_(0)
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

void SpinalCord::walk(){
    robotState = STATE_WALK;
}

void SpinalCord::rest(){
    robotState = STATE_IDLE;
}

void SpinalCord::wallFlip(){
    robotState = STATE_ACTION;
}

void SpinalCord::applyCalibration(int channel, int angle) {
    // We call identifyAndMove on every leg. 
    // Leg class logic ensures only the correct leg reacts.
    leg1.identifyAndMove(channel, (double)angle);
    leg2.identifyAndMove(channel, (double)angle);
    leg3.identifyAndMove(channel, (double)angle);
    leg4.identifyAndMove(channel, (double)angle);
}

void SpinalCord::update(){
    switch(robotState){
        case STATE_WALK:
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

    if (currentGait_ != GAIT_NONE) {
        tickGait();
    }
}

void SpinalCord::setGait(GaitType g){
    currentGait_ = g;
    gaitPhaseStartMs_ = millis();
    Serial.printf("[gait] %s\n", GAITS[g].label);
}

GaitType SpinalCord::currentGait() const {
    return currentGait_;
}

void SpinalCord::tickGait(){
    const GaitParams& cfg = GAITS[currentGait_];
    const float t           = (millis() - gaitPhaseStartMs_) / 1000.0f;
    const float globalPhase = fmodf(t / cfg.period_s, 1.0f);

    Leg* legs[LEG_COUNT] = { &leg1, &leg2, &leg3, &leg4 };

    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        const float legPhase = fmodf(globalPhase - cfg.offsets[i] + 1.0f, 1.0f);

        // Stub trajectory: linear sweep along the gait axis with a sin-arch
        // lift during swing. Real IK / foot-target curve lands later.
        const bool  inStance = legPhase < cfg.duty;
        const float strideU  = inStance
            ? legPhase / cfg.duty
            : (legPhase - cfg.duty) / (1.0f - cfg.duty);
        const float along    = cfg.step_length_m * (0.5f - strideU);
        const float lift     = inStance
            ? 0.0f
            : cfg.step_height_m * sinf((float)PI * strideU);

        const float x = (cfg.axis == 'x') ? along : 0.0f;
        const float y = (cfg.axis == 'y') ? along : 0.0f;
        const float z = lift;

        legs[i]->setPose(x, y, z);
    }
}

