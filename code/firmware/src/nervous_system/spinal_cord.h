#ifndef SPINALCORD_H
#define SPINALCORD_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <cstdint>
#include "leg.h"
#include "movements.h"
#include "motion_math.h"
#include "face.h"
#include "../shared/data.h"

class SpinalCord{
    public:
        SpinalCord(uint8_t pcaDriverAddress);
        void begin();
        void walk();
        void rest();
        void relax();
        void relax(uint32_t ms);   // T:2 dur_ms: ease every joint to 90 over ms.
        void stand();
        void stand(uint32_t ms);   // T:2 dur_ms: invert-aware ease to NEUTRAL.
        void wallFlip();
        void update();
        void invertRobot();
        void applyCalibration(int channel, int angle);
        void setGait(GaitType g);
        GaitType currentGait() const;
        void processCommand(String dir);
        void playClip(uint8_t id, bool loop = false);
        // Hard-stop clip playback: phase := CLIP_DONE, clipLoop_ := false,
        // clipPrerollUntilMs_ := 0. Called by the T:2 IDLE handler so a looping
        // clip doesn't survive across the FSM transition and resume when the
        // FSM next re-enters STATE_ACTION. Pose is NOT touched here — the IDLE
        // dispatch (spinalCord.rest()) handles the actual ease.
        void stopClipPlayback();
        void setClipSmoothing(float alpha);   // T:11 runtime smoothing knob
        void setInverted(bool flag);
        // Auto-invert toggle: gates whether main.cpp is allowed to forward the
        // IMU-derived `isInverted` boolean to setInverted(). Defaults ON so the
        // robot "just works" when flipped; the app exposes a T:6 setter so the
        // user can freeze invert state at its current value (e.g. while
        // calibrating on its back). Toggling off does NOT clear isInverted —
        // it stays at whatever it was, frozen, until either auto-invert is
        // re-enabled or a manual T:9 (CMD_SET_INVERT) overrides it.
        void setAutoInvertEnabled(bool enabled);
        bool isAutoInvertEnabled() const { return autoInvertEnabled_; }
        // Accessor used elsewhere (e.g. telemetry / diagnostics CSV).
        uint8_t getRobotState() const { return (uint8_t)robotState; }
        Face& getFace() { return face; }

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
        Face face;
        Leg leg1;
        Leg leg2;
        Leg leg3;
        Leg leg4;

        GaitType currentGait_;
        uint32_t gaitPhaseStartMs_;

        ClipState clipState_;   // pure lifecycle state (see motion_math.h)
        float clipSmoothed_[LEG_COUNT][3];  // per-channel EMA state for clip playback;
                                            // seeded from frame 0 on playClip()
        uint32_t clipPrerollUntilMs_ = 0;   // playClip eases the live pose into frame 0
                                            // until this time; real playback starts after
        bool clipLoop_ = false;             // when set, the clip replays from frame 0 at
                                            // its end instead of easing back to neutral
        float clipEmaAlpha_ = 0.75f;        // clip-playback EMA smoothing; boot default,
                                            // runtime-tunable via setClipSmoothing (T:11)

        // Vector timing and state
        float targetX;
        float targetY;
        float targetYaw;
        float activeX;
        float activeY;
        float activeYaw;
        bool isMovingRequested;
        bool isInverted;
        bool autoInvertEnabled_ = true;   // default ON; togglable via T:6
        uint32_t lastCommandMs;
        uint32_t lastInvertMs_;   // timestamp of the last accepted T:6 toggle (debounce)
        bool     hasInverted_;    // false until the first invert, so it is never debounced away

        void tickGait();
        void tickTrot();
        void tickYawRotation();
        void tickClip();

        // Invert-aware "hold neutral" helpers. Unlike Leg::returnToDefaultAngles*
        // (which write the UPRIGHT defaults raw and bypass the pitch mirror), these
        // route through the mirror so an inverted robot holds the inverted neutral.
        void goToNeutral();              // instant
        void easeToNeutral(uint32_t ms); // non-blocking ease (clip return)

        // Mirror the pose the robot is CURRENTLY holding, in place and eased
        // (180 - angle on thigh/knee, shoulder unchanged). Called on an invert
        // toggle so the flip applies to the live pose instead of snapping to
        // neutral or waiting for the next motion tick.
        void flipPoseInPlace(uint32_t ms);

        // Single invert choke point: writes a leg's servo triple, mirroring the
        // pitch joints (thigh, knee) about 90 when isInverted. Every motion source
        // (gaits, clips, stand) routes through this so invert is applied uniformly.
        void applyServos(Leg* leg, ServoTriple s);
};

#endif