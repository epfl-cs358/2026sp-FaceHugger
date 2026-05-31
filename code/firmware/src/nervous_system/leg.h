#ifndef LEG_H
#define LEG_H
#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>

#include "servo.h"



class Leg {
  public:
    Leg(Adafruit_PWMServoDriver& pwm, int legID, Servo hipServo, Servo thighServo, Servo kneeServo); // Constructor
    void setJointAngles(double hip, double thigh, double knee);
    void setJointAnglesTimed(double hip, double thigh, double knee, uint32_t ms);
    void returnToDefaultAngles();
    void returnToDefaultAnglesTimed(uint32_t ms);
    void tickEase();
    bool easing() const;
    void identifyAndMove(uint8_t channel, double angle);
    void getJointAngles(float& hip, float& thigh, float& knee) const;
    uint8_t legId() const { return (uint8_t)id; }  // 0=FR, 1=FL, 2=RR/BR, 3=RL/BL
  private:
    int id;
    Servo hipServo;
    Servo thighServo;
    Servo kneeServo;
    Adafruit_PWMServoDriver& pwm;
    void updateServos(double hipAngle, double thighAngle, double kneeAngle); // Sends the angles to the PCA9685
};

#endif