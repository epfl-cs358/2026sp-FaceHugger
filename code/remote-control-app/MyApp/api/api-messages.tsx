import { webSocketIP } from "../config/config";
import { connect, sendCommand, ws } from "../services/socket";
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
export const InvertRobotPacket = {T: 6, id: ActionTypes.INVERT_ROBOT} as ActionPacket;
