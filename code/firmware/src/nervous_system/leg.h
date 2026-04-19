#ifndef LEG_H
#define LEG_H
#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>



class Leg {
  public:
    Leg(Adafruit_PWMServoDriver &pwm, int legID, uint8_t hipPCAChannel, uint8_t thighPCAChannel, uint8_t kneePCAChannel, uint16_t hipDefaultAngle, uint16_t thighDefaultAngle, uint16_t kneeDefaultAngle); // Constructor
    void setPose(float x, float y, float z); // Inverse Kinematics target
    void returnToDefaultAngles();
    void identifyAndMove(uint8_t channel, double angle);
  private:
    int id;
    uint8_t hipPCAChannel;
    uint8_t thighPCAChannel;
    uint8_t kneePCAChannel;

    uint16_t hipDefaultAngle;
    uint16_t thighDefaultAngle;
    uint16_t kneeDefaultAngle;

    uint16_t hipAngle;
    uint16_t thighAngle;
    uint16_t kneeAngle; //the current angles in the servos of the leg

    Adafruit_PWMServoDriver& pwm;
    void updateServos(); // Sends the angles to the PCA9685
    void setServoAngle(uint8_t channel, double anglet, int offset);
};

#endif