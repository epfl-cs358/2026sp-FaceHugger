#include "leg.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "servo.h"
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
{}

void Leg::setJointAngles(double hip, double thigh, double knee) {
    // Out-of-range guard. A clip or gait that asks for a servo angle outside the
    // [0,180] electrical range is a bug worth seeing — on the bench it shows up in
    // the serial monitor, and the sim captures the same line. We still clamp (the
    // robot stays safe), but we report the raw request + channel first.
    if (hip   < 0.0 || hip   > 180.0) Serial.printf("[OOR] servo %d requested %.2f\n", hipServo.getChannel(),   hip);
    if (thigh < 0.0 || thigh > 180.0) Serial.printf("[OOR] servo %d requested %.2f\n", thighServo.getChannel(), thigh);
    if (knee  < 0.0 || knee  > 180.0) Serial.printf("[OOR] servo %d requested %.2f\n", kneeServo.getChannel(),  knee);
    hip   = constrain(hip,   0.0, 180.0);
    thigh = constrain(thigh, 0.0, 180.0);
    knee  = constrain(knee,  0.0, 180.0);
    updateServos(hip, thigh, knee);
}

void Leg::updateServos(double hipAngle, double thighAngle, double kneeAngle){
    hipServo.setServoAngle(hipAngle);
    thighServo.setServoAngle(thighAngle);
    kneeServo.setServoAngle(kneeAngle);
}

void Leg::setJointAnglesTimed(double hip, double thigh, double knee, uint32_t ms) {
    // Non-blocking ease to an explicit servo triple (already invert-resolved by the
    // caller). Used by the clip-return ease so it can target the MIRRORED neutral
    // when inverted, instead of the upright defaults returnToDefaultAnglesTimed gives.
    hipServo.setServoAngleTimed(hip, ms);
    thighServo.setServoAngleTimed(thigh, ms);
    kneeServo.setServoAngleTimed(knee, ms);
}

void Leg::returnToDefaultAngles(){
    hipServo.returnToDefaultAngle();
    thighServo.returnToDefaultAngle();
    kneeServo.returnToDefaultAngle();
}

void Leg::returnToDefaultAnglesTimed(uint32_t ms) {
    hipServo.returnToDefaultAngleTimed(ms);
    thighServo.returnToDefaultAngleTimed(ms);
    kneeServo.returnToDefaultAngleTimed(ms);
}

void Leg::tickEase() {
    hipServo.tickEase();
    thighServo.tickEase();
    kneeServo.tickEase();
}

bool Leg::easing() const {
    return hipServo.easing() || thighServo.easing() || kneeServo.easing();
}

void Leg::identifyAndMove(uint8_t channel, double angle){
    if (hipServo.getChannel()   == channel) hipServo.setServoAngle(angle);
    if (thighServo.getChannel() == channel) thighServo.setServoAngle(angle);
    if (kneeServo.getChannel()  == channel) kneeServo.setServoAngle(angle);
}

void Leg::getJointAngles(float& hip, float& thigh, float& knee) const {
    hip   = (float)hipServo.getServoAngle();
    thigh = (float)thighServo.getServoAngle();
    knee  = (float)kneeServo.getServoAngle();
}
