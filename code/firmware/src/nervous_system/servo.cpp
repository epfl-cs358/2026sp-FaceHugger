#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "../shared/config.h"
#include "servo.h"

Servo::Servo(Adafruit_PWMServoDriver &pwm, uint8_t pcaChannel, uint16_t servoDefaultAngle)
    : pwm(pwm),
      pcaChannel(pcaChannel),
      servoAngle(servoDefaultAngle),
      servoDefaultAngle(servoDefaultAngle){
    }

void Servo::setServoAngle(double angle){
    // Electrical backstop. Every motion path (IK, pose, calibrate, future clip player)
    // funnels through here, and map() does NOT clamp — an out-of-range angle would drive
    // the servo past MIN/MAX_PULSE. constrain here protects them all at one chokepoint.
    double clamped = constrain(angle, 0.0, 180.0);
    if (clamped != angle) {
        Serial.printf("[WARN] servo %d clamped: %.1f -> %.1f\n", this->pcaChannel, angle, clamped);
    }
    uint16_t pulse = map(clamped, 0, 180, MIN_PULSE, MAX_PULSE);
    pwm.setPWM(this->pcaChannel, 0, pulse);
    this->servoAngle = clamped;
}

uint8_t Servo::getChannel() const {
    return pcaChannel;
}

void Servo::returnToDefaultAngle(){
    setServoAngle(this->servoDefaultAngle);
}

double Servo::getServoAngle() const {
    // Whole-degree precision: setServoAngle() truncates double → uint16_t storage.
    return (double)servoAngle;
}

