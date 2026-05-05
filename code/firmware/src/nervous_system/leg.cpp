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
{}

void Leg::setPose(float x, float y, float z) {
    // 1. Get the raw IK output in radians
    const JointAngles a = legIK((LegId)id, x, y, z);
    const double RAD_TO_DEG_F = 57.29577951308232;

    // Convert raw IK radians to geometric degrees
    double targetShoulder = a.shoulder * RAD_TO_DEG_F; 
    double targetThigh    = a.hip      * RAD_TO_DEG_F;    
    double targetKnee     = a.knee     * RAD_TO_DEG_F;     

    double servoShoulder, servoThigh, servoKnee;

    // 2. Apply Physical Hardware Mapping (90 degrees = absolute horizontal)
    if (id == 1 || id == 3) {
        // Front Left - Back Right Diagonal
        servoThigh = 90.0 + targetThigh; 
        servoKnee  = 90.0 - targetKnee;

        // Shoulder uses the Global Polar offset
        // Front-Left (1) Neutral is +135, Back-Right (2) Neutral is -135
        double shoulderOffset = (id == 1) ? 135.0 : -135.0;
        servoShoulder = 90.0 - (targetShoulder - shoulderOffset);
    } 
    else {
        // Front Right - Back Left Diagonal
        servoThigh = 90.0 - targetThigh;
        servoKnee  = 90.0 + targetKnee;

        // Shoulder uses the Global Polar offset
        // Front-Right (0) Neutral is +45, Back-Left (3) Neutral is -45
        double shoulderOffset = (id == 0) ? 45.0 : -45.0;
        servoShoulder = 90.0 + (targetShoulder - shoulderOffset);
    }

    // Add safety clamping here so you never send an angle < 0 or > 180 to the PCA9685
    servoShoulder = constrain(servoShoulder, 0.0, 180.0);
    servoThigh    = constrain(servoThigh, 0.0, 180.0);
    servoKnee     = constrain(servoKnee, 0.0, 180.0);

    // 3. Send the final 0-180 degree angles to the PCA9685 driver
    updateServos(servoShoulder, servoThigh, servoKnee);
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