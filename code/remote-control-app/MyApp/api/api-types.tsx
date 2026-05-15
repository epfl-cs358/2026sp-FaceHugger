export enum FSMStatus {
    STATE_IDLE = 0,
    STATE_WALK = 1,
    STATE_ACTION = 2,
    STATE_FAILSAFE = 3
}

export enum DirectionVector {
    FW = 0,
    BW = 1,
    FR = 2,
    FL = 3,
    BR = 4,
    BL = 5,
    R = 6,
    L = 7,
    STOP = 8
}

export enum GaitMode {
    TROT = 0,
    CRAB = 1,
    CRAWL = 2
}

export enum Legs {
    FRONT_RIGHT_LEG = 0,
    FRONT_LEFT_LEG = 1,
    BACK_RIGHT_LEG = 2,
    BACK_LEFT_LEG = 3
}

export enum Servos {
    HIP_SERVO = 0,
    THIGH_SERVO = 1,
    KNEE_SERVO = 2
}

export type PacketType = {
    T: number
}

export type ManualMovement = PacketType & {
    d: number
}

export type FSMStateModification = PacketType & {
    s: number //FSM state
}

export type ServoCalibration = PacketType & {
    id: number //leg id to calibrate servo of
    servo_id: number //servo id to calibrate leg of
    a: number //angle to write in the servo (0-180 range)
}

export type GaitIntegration = PacketType & {
    g: number
}

export type SystemStatus = PacketType & { //T10
    s: number, // current fsm state
    d: Array<number>, //tof distance readings
    a: Array<number>, //AMU array containing speed + gyroscope [Speed, Rotation X, Rotation Y, Rotation Z]
    g: number, //current gait mode of the robot
    pc: number, //current percentage of the movement gait accomplished
    e: string //error message
} 