#ifndef LEG_H
#define LEG_H
#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>

#include "servo.h"



class Leg {
  public:
    Leg(Adafruit_PWMServoDriver& pwm, int legID, Servo hipServo, Servo thighServo, Servo kneeServo); // Constructor
    void setPose(float x, float y, float z); // Inverse Kinematics target
    void returnToDefaultAngles();
    void identifyAndMove(uint8_t channel, double angle);
  private:
    int id;
    Servo hipServo;
    Servo thighServo;
    Servo kneeServo;
    Adafruit_PWMServoDriver& pwm;
    void updateServos(double hipAngle, double thighAngle, double kneeAngle); // Sends the angles to the PCA9685
};

#endif