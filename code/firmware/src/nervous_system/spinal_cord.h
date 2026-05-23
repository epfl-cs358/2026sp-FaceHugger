#ifndef SPINALCORD_H
#define SPINALCORD_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <cstdint>
#include "leg.h"
#include "movements.h"
#include "motion_math.h"
#include "../shared/data.h"

class SpinalCord{
    public:
        SpinalCord(uint8_t pcaDriverAddress);
        void begin();
        void walk();
        void rest();
        void relax();
        void stand();
        void wallFlip();
        void update();
        void invertRobot();
        void applyCalibration(int channel, int angle);
        void setGait(GaitType g);
        GaitType currentGait() const;
        void processCommand(String dir);
        void playClip(uint8_t id);

        // POD bundle of read-only state for the diagnostics CSV logger.
        // Field names match CSV column names (snake_case is intentional).
        struct Snapshot {
            uint8_t  robot_state;
            uint8_t  gait;
            bool     is_moving;
            bool     is_inverted;
            uint32_t last_cmd_ms;
            float    target_x, target_y, target_yaw;
            float    active_x, active_y, active_yaw;
            float    servo_angles[12]; // FR(h,t,k), FL(h,t,k), BR(h,t,k), BL(h,t,k)
                                       // Whole-degree precision; see Servo::getServoAngle.
        };
        Snapshot snapshot() const;
    private:
        volatile RobotState robotState;
        Adafruit_PWMServoDriver driver;
        Leg leg1;
        Leg leg2;
        Leg leg3;
        Leg leg4;

        GaitType currentGait_;
        uint32_t gaitPhaseStartMs_;

        ClipState clipState_;   // pure lifecycle state (see motion_math.h)
        float clipSmoothed_[LEG_COUNT][3];  // per-channel EMA state for clip playback;
                                            // seeded from frame 0 on playClip()

        // Vector timing and state
        float targetX;
        float targetY;
        float targetYaw;
        float activeX;
        float activeY;
        float activeYaw;
        bool isMovingRequested;
        bool isInverted;
        uint32_t lastCommandMs;

        void tickGait();
        void tickTrot();
        void tickYawRotation();
        void tickClip();
};

#endif