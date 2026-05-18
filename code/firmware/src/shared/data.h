#ifndef DATA_H
#define DATA_H

// Command Types matching API_SPEC.md
enum CommandType {
    CMD_MOVE = 1,
    CMD_STATE = 2,
    CMD_POSE = 3,
    CMD_CALIBRATE = 4,
    CMD_GAIT_MODE = 5,
    CMD_TELEMETRY = 10
};

// FSM States matching LaTeX documentation
enum RobotState {
    STATE_IDLE     = 0,
    STATE_WALK     = 1,
    STATE_ACTION   = 2,
    STATE_FAILSAFE = 3,
    STATE_REST     = 4,  // All servos at 90° — safe to power off
};

#endif