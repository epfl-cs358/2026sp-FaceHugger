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
        void setServoAngleTimed(double angle, uint32_t ms);  // non-blocking ease
        void tickEase();                                     // advance ease, call each loop
        bool easing() const;
        void returnToDefaultAngle();
        void returnToDefaultAngleTimed(uint32_t ms);
        uint8_t getChannel() const;
        double getServoAngle() const;
    private:
        Adafruit_PWMServoDriver& pwm;
        uint8_t pcaChannel;
        uint16_t servoAngle;
        uint16_t servoDefaultAngle;
        bool     easeActive      = false;
        uint32_t easeStartMs     = 0;
        uint32_t easeDurMs       = 0;
        double   easeStartAngle  = 0.0;
        double   easeTargetAngle = 0.0;
};
#endif