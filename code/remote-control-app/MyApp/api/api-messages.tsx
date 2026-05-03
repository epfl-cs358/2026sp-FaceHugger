import { webSocketIP } from "../config/config";
import { connect, sendCommand, ws } from "../services/socket";
import { ServoCalibration } from "./api-types";

export function sendServoCalibrationMessage(servoCalibration: ServoCalibration){
    const messageToSend = {T: 4, ...servoCalibration};
    if(!ws){
        connect(webSocketIP);
    }
    sendCommand(JSON.stringify(messageToSend))
}