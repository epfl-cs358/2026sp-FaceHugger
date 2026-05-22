#ifndef LEG_H
#define LEG_H
#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>

#include "servo.h"



class Leg {
  public:
    Leg(Adafruit_PWMServoDriver& pwm, int legID, Servo hipServo, Servo thighServo, Servo kneeServo); // Constructor
    void setPose(float x, float y, float z);
    void setJointAngles(double hip, double thigh, double knee);
    void returnToDefaultAngles();
    void returnToDefaultAnglesTimed(uint32_t ms);
    void tickEase();
    bool easing() const;
    void identifyAndMove(uint8_t channel, double angle);
    void getJointAngles(float& hip, float& thigh, float& knee) const;
  private:
    int id;
    Servo hipServo;
    Servo thighServo;
    Servo kneeServo;
    Adafruit_PWMServoDriver& pwm;
    void updateServos(double hipAngle, double thighAngle, double kneeAngle); // Sends the angles to the PCA9685
};

#endif