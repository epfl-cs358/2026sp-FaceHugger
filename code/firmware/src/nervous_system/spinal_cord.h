#ifndef SPINALCORD_H
#define SPINALCORD_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <cstdint>
#include "leg.h"
#include "movements.h"
#include "../shared/data.h"

class SpinalCord{
    public:
        SpinalCord(uint8_t pcaDriverAddress);
        void begin();
        void walk();
        void rest();
        void relax();
        void wallFlip();
        void update();
        void applyCalibration(int channel, int angle);
        void setGait(GaitType g);
        GaitType currentGait() const;
        void processCommand(String dir);
    private:
        volatile RobotState robotState;
        Adafruit_PWMServoDriver driver;
        Leg leg1;
        Leg leg2;
        Leg leg3;
        Leg leg4;

        GaitType currentGait_;
        uint32_t gaitPhaseStartMs_;

        // Vector timing and state
        float targetX;
        float targetY;
        float activeX;
        float activeY;
        bool isMovingRequested;
        uint32_t lastCommandMs;

        void tickGait();
        void tickTrot();
};

#endif