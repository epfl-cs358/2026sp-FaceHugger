export enum FSMStatus {
    STATE_IDLE = 0,
    STATE_WALK = 1,
    STATE_ACTION = 2,
    STATE_FAILSAFE = 3
}

export type FSMStateModification = {
    s: number //FSM state
}

export type BodyPose = {
    h: number, //chassis height
    p: number, //pitch (tilt forward back)
    r: number //tilt side to side
}

export type ServoCalibration = {
    id: number //leg id to calibrate servo of
    servo_id: number //servo id to calibrate leg of
    a: number //angle to write in the servo
}

export type SystemStatus = { //T10
    s: number, // current fsm state
    b: number, //battery voltage
    d: Array<number>, //tof distance readings
    a: boolean, //stabilization pid status
    e: string //error message
} 