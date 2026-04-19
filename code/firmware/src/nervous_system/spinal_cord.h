#ifndef SPINALCORD_H
#define SPINALCORD_H

#include <Adafruit_PWMServoDriver.h>
#include <cstdint>
#include "leg.h"
#include "../shared/data.h"

class SpinalCord{
    public:
        SpinalCord(uint8_t pcaDriverAddress);
        void begin();
        void walk();
        void rest();
        void jump();
        void update();
        void applyCalibration(int channel, int angle);
    private:
        volatile RobotState robotState;
        Adafruit_PWMServoDriver driver;
        Leg leg1;
        Leg leg2;
        Leg leg3;
        Leg leg4;
};

#endif