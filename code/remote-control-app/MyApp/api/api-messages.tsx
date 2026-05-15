import { webSocketIP } from "../config/config";
import { connect, sendCommand, ws } from "../services/socket";
import { DirectionVector, GaitIntegration, GaitMode, ManualMovement, ServoCalibration } from "./api-types";

//Movement packets
export const FWMovementPacket = {T: 1, d: DirectionVector.FW} as ManualMovement;
export const FRMovementPacket = {T: 1, d: DirectionVector.FR} as ManualMovement;
export const FLMovementPacket = {T: 1, d: DirectionVector.FL} as ManualMovement;
export const RightMovementPacket = {T:1, d: DirectionVector.R} as ManualMovement;
export const LeftMovementPacket = {T:1, d:DirectionVector.L} as ManualMovement;
export const BWMovementPacket = {T:1, d:DirectionVector.BW} as ManualMovement;
export const BRMovementPacket = {T:1, d: DirectionVector.BR} as ManualMovement;
export const BLMovementPacket = {T:1, d: DirectionVector.BL} as ManualMovement;
export const STOP = {T:1, d: DirectionVector.STOP} as ManualMovement;

//Gait mode packets
export const TrotGaitPacket = {T: 5, g: GaitMode.TROT} as GaitIntegration;
export const CrabGaitPacket = {T: 5, g: GaitMode.CRAB} as GaitIntegration;
export const CrawlGaitPacket = {T: 5, g: GaitMode.CRAWL} as GaitIntegration;