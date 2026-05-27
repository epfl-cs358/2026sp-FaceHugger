import { sendCommand } from "../services/socket";
import { ActionPacket, ActionTypes, DirectionVector, GaitIntegration, GaitMode, ManualMovement, ServoCalibration } from "./api-types";

//Movement packets
export const FWMovementPacket    = {T: 1, dir: DirectionVector.FW}   as ManualMovement;
export const FRMovementPacket    = {T: 1, dir: DirectionVector.FR}   as ManualMovement;
export const FLMovementPacket    = {T: 1, dir: DirectionVector.FL}   as ManualMovement;
export const RightMovementPacket = {T: 1, dir: DirectionVector.R}    as ManualMovement;
export const LeftMovementPacket  = {T: 1, dir: DirectionVector.L}    as ManualMovement;
export const BWMovementPacket    = {T: 1, dir: DirectionVector.BW}   as ManualMovement;
export const BRMovementPacket    = {T: 1, dir: DirectionVector.BR}   as ManualMovement;
export const BLMovementPacket    = {T: 1, dir: DirectionVector.BL}   as ManualMovement;
export const STOP                = {T: 1, dir: DirectionVector.STOP} as ManualMovement;

//Gait mode packets
export const TrotGaitPacket = {T: 5, g: GaitMode.TROT} as GaitIntegration;
export const CrabGaitPacket = {T: 5, g: GaitMode.CRAB} as GaitIntegration;

//Action packets
export const InvertRobotPacket = {T: 6, a: ActionTypes.INVERT_ROBOT} as ActionPacket;

export const requestClipList = () =>
    sendCommand(JSON.stringify({ T: 8 }));

export const playClip = (id: number) =>
    sendCommand(JSON.stringify({ T: 7, c: id }));

//Calibration packets
export const calibrationPacket = (leg: number, servo: number, angle: number) =>
    ({T: 4, id: leg, servo_id: servo, a: angle} as ServoCalibration);

// One CMD_CALIBRATE per (leg 0-3, servo 0-2) at 90° — resets every servo.
export const restAllServosPackets = (angle: number = 90) => {
    const packets: ServoCalibration[] = [];
    for (let leg = 0; leg < 4; leg++) {
        for (let servo = 0; servo < 3; servo++) {
            packets.push(calibrationPacket(leg, servo, angle));
        }
    }
    return packets;
};

// Neutral standing pose — raw servo angles per [leg id][servo_id] = [hip, thigh, knee].
// Mirrors the firmware per-servo *_DEFAULT_ANGLE in config.h (returnToDefaultAngles()).
// Keep in sync if the firmware defaults change.
const NEUTRAL_STANCE_ANGLES: number[][] = [
    [90, 150, 53],  // leg 0 - front right
    [75, 30, 130],  // leg 1 - front left
    [90, 40, 130],  // leg 2 - rear right
    [90, 150, 50],  // leg 3 - rear left
];

// One CMD_CALIBRATE per joint to drive every servo to the neutral stance.
export const neutralStancePackets = () => {
    const packets: ServoCalibration[] = [];
    NEUTRAL_STANCE_ANGLES.forEach((servos, leg) => {
        servos.forEach((angle, servo) => {
            packets.push(calibrationPacket(leg, servo, angle));
        });
    });
    return packets;
};
