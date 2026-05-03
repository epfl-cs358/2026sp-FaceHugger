#include "leg.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "servo.h"
#include "kinematics.h"
#include "movements.h"
#include "../shared/config.h"

Leg::Leg(Adafruit_PWMServoDriver &pwm,
         int legID,
         Servo hipServo, Servo thighServo, Servo kneeServo)
    : pwm(pwm),   
      id(legID),
      hipServo(hipServo),
      thighServo(thighServo),
      kneeServo(kneeServo)
{
    returnToDefaultAngles();
}

void Leg::setPose(float x, float y, float z) {
    const JointAngles a = legIK((LegId)id, x, y, z);
    // Convert IK output (radians, joint zero = STANCE) to servo angles
    // (degrees, neutral = 90). Per-servo sign / offset calibration TBD.
    const double RAD_TO_DEG_F = 57.29577951308232;
    const double hipDeg   = 90.0 + a.shoulder * RAD_TO_DEG_F;
    const double thighDeg = 90.0 + a.hip      * RAD_TO_DEG_F;
    const double kneeDeg  = 90.0 + a.knee     * RAD_TO_DEG_F;
    updateServos(hipDeg, thighDeg, kneeDeg);
}

void Leg::updateServos(double hipAngle, double thighAngle, double kneeAngle){
    hipServo.setServoAngle(hipAngle);
    thighServo.setServoAngle(thighAngle);
    kneeServo.setServoAngle(kneeAngle);
}

void Leg::returnToDefaultAngles(){
    hipServo.returnToDefaultAngle();
    thighServo.returnToDefaultAngle();
    kneeServo.returnToDefaultAngle();
}

void Leg::identifyAndMove(uint8_t channel, double angle){
    if (hipServo.getChannel()   == channel) hipServo.setServoAngle(angle);
    if (thighServo.getChannel() == channel) thighServo.setServoAngle(angle);
    if (kneeServo.getChannel()  == channel) kneeServo.setServoAngle(angle);
}