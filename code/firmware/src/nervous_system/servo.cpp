#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "../shared/config.h"
#include "servo.h"

Servo::Servo(Adafruit_PWMServoDriver &pwm, uint8_t pcaChannel, uint16_t servoDefaultAngle)
    : pwm(pwm),
      pcaChannel(pcaChannel),
      servoDefaultAngle(servoDefaultAngle){
        setServoAngle(this->servoDefaultAngle);
    }

void Servo::setServoAngle(double angle){
    uint16_t pulse = map(this->servoAngle, 0, 270, MIN_PULSE, MAX_PULSE);
    pwm.setPWM(this->pcaChannel, 0, pulse);
    this->servoAngle = angle;
}

void Servo::returnToDefaultAngle(){
    setServoAngle(this->servoDefaultAngle);
}

