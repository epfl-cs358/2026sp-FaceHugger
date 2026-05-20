#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "../shared/config.h"


class Servo {
    public:
        Servo(Adafruit_PWMServoDriver &pwm, uint8_t pcaChannel, uint16_t servoDefaultAngle);
        void setServoAngle(double angle);
        void returnToDefaultAngle();
        uint8_t getChannel() const;
        double getServoAngle() const;
    private:
        Adafruit_PWMServoDriver& pwm;
        uint8_t pcaChannel;
        uint16_t servoAngle;
        uint16_t servoDefaultAngle;
};
#endif