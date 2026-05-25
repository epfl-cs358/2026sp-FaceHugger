#ifndef DATA_H
#define DATA_H

// Command Types matching API_SPEC.md
enum CommandType {
    CMD_MOVE = 1,
    CMD_STATE = 2,
    CMD_POSE = 3,
    CMD_CALIBRATE = 4,
    CMD_GAIT_MODE = 5,
    CMD_ACTION_SELECTION = 6,
    CMD_PLAY_CLIP = 7,
    CMD_SET_INVERT = 9,
    CMD_TELEMETRY = 10
};

// FSM States matching LaTeX documentation
enum RobotState {
    STATE_IDLE     = 0,
    STATE_WALK     = 1,
    STATE_ACTION   = 2,
    STATE_FAILSAFE = 3,
    STATE_REST     = 4,  // All servos at 90° — flat/spread calibration pose, safe to power off
    STATE_STAND    = 5,  // Standing/neutral pose (per-leg NEUTRAL[]), gait launch reference
};

// Range of RobotState values the CMD_STATE (T:2) handler accepts off the wire.
// Used to reject out-of-range states before the dispatch switch.
constexpr bool isValidStateCommand(int s) {
    return s >= STATE_IDLE && s <= STATE_STAND;
}

#endif