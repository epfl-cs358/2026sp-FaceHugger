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
    uint16_t pulse = map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
    pwm.setPWM(this->pcaChannel, 0, pulse);
    this->servoAngle = angle;
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

