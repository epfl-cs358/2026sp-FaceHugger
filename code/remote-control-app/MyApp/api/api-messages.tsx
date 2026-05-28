import { sendCommand } from "../services/socket";
import { stopStream } from "../services/clipStreamer";
import { POSE_EASE_MS } from "../config/config";
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

export const playClip = (id: number, loop: boolean = false) =>
    sendCommand(JSON.stringify(loop ? { T: 7, c: id, loop: true } : { T: 7, c: id }));

// Stop a (looping) firmware clip by preempting STATE_ACTION with an IDLE state.
// The robot holds its last commanded pose; follow with Neutral stance to reset.
export const stopClipPlayback = () =>
    sendCommand(JSON.stringify({ T: 2, s: 0 }));

// Page-change / unmount cleanup. Send STATE_IDLE so the firmware drops any
// in-flight gait (T:1/T:5) or firmware clip (T:7), and also clear the JS
// clip streamer's interval in case it was running (no-op when idle). Wired
// from the screen useEffect cleanups in GaitControl and Actions so that
// swapping pages on the Pager stops in-flight motion.
export const stopMotion = () => {
    stopClipPlayback();
    stopStream();
};

// Runtime clip-playback smoothing (T:11): a in [0, 0.95]. Low = snappy.
export const setClipSmoothing = (alpha: number) =>
    sendCommand(JSON.stringify({ T: 11, a: alpha }));

//Calibration packets
export const calibrationPacket = (leg: number, servo: number, angle: number) =>
    ({T: 4, id: leg, servo_id: servo, a: angle} as ServoCalibration);

// Pose buttons (Task #11). Both REST (flat / all-90 calibration) and NEUTRAL
// (standing) are firmware-side states; the firmware owns the pose values and
// the ease. We send ONE T:2 with `dur_ms = POSE_EASE_MS` so the joints glide
// instead of slamming. The previous burst-of-T:4 helpers were deleted: they
// duplicated the firmware's per-servo defaults (drift hazard) and snapped
// the pose. The firmware clamps `dur_ms` to POSE_EASE_MS_MAX, so an
// out-of-range constant here can't park the robot in a multi-minute ease.
export const sendRestPose = () =>
    sendCommand(JSON.stringify({ T: 2, s: 4, dur_ms: POSE_EASE_MS }));

export const sendNeutralStance = () =>
    sendCommand(JSON.stringify({ T: 2, s: 5, dur_ms: POSE_EASE_MS }));
