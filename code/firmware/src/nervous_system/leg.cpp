#include "leg.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "servo.h"
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
    Serial.print("Leg ");
    Serial.print(id);
    Serial.printf(" moving to target: %.1f, %.1f, %.1f\n", x, y, z);
    //TODO Call updateServos later
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
    // TODO: dispatch to the servo whose pcaChannel matches `channel`.
    // Stub kept empty so SpinalCord::applyCalibration links; calibration is
    // a no-op until Servo exposes its channel or a moveIfChannelMatches helper.
    (void)channel;
    (void)angle;
}