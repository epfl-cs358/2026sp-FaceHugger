#ifndef LEG_H
#define LEG_H

#include "servo.h"



class Leg {
  public:
    Leg(Adafruit_PWMServoDriver& pwm, int legID, Servo hipServo, Servo thighServo, Servo kneeServo); // Constructor
    void setPose(float x, float y, float z); // Inverse Kinematics target
    void returnToDefaultAngles();
  private:
    int id;
    Servo& hipServo;
    Servo& thighServo;
    Servo& kneeServo;
    Adafruit_PWMServoDriver& pwm;
    void updateServos(double hipAngle, double thighAngle, double kneeAngle); // Sends the angles to the PCA9685
};

#endif