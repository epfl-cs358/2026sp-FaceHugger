import {create} from 'zustand'
import { ClipInfo, FSMStatus, GaitMode } from '../api/api-types';
import { DEFAULT_IP, DEFAULT_PORT } from '../config/config';

type RobotStore = {
    connIP: string,            // active WebSocket target IP
    connPort: number,          // active WebSocket target port
    setConnection: (ip: string, port: number) => void,
    connTick: number,          // bump to force a reconnect (same ip/port)
    requestReconnect: () => void,
    fsmState: FSMStatus,
    setFsmState: (state: FSMStatus) => void,
    chosenFsmState: FSMStatus,
    setChosenFsmState: (state: FSMStatus) => void,
    tofDistances: number[], // [FL, FR, RL, RR, Center]
    setTofDistances: (distances: number[]) => void,
    speed: number,
    gyroscope: [number, number, number], // [Rotation X, Rotation Y, Rotation Z]
    setAMU: (amu: number[]) => void, // [Speed, Rotation X, Rotation Y, Rotation Z]
    gaitMode: GaitMode,
    setGaitMode: (gait: GaitMode) => void,
    chosenGaitMode: GaitMode,
    setChosenGaitMode: (gait: GaitMode) => void,
    movementProgress: number, // 0.0 - 1.0
    setMovementProgress: (pc: number) => void,
    errorMessage: string | null,
    setErrorMessage: (error: string | null) => void,
    clips: ClipInfo[],
    setClips: (clips: ClipInfo[]) => void,
    clipPlaying: boolean,
    setClipPlaying: (playing: boolean) => void,
    // IMU orientation (T:10 broadcast). pitch/roll in deg, upsideDown latched
    // with hysteresis firmware-side. Optional fields land as `undefined` on
    // pre-IMU firmware — the UI renders a placeholder in that case.
    pitchDeg: number | null,
    rollDeg: number | null,
    upsideDown: boolean | null,
    setOrientation: (pitchDeg: number | null, rollDeg: number | null, upsideDown: boolean | null) => void,
};

export const useRobotStore = create<RobotStore>((set) => ({
    connIP: DEFAULT_IP,
    connPort: DEFAULT_PORT,
    setConnection: (ip, port) => set({ connIP: ip, connPort: port }),
    connTick: 0,
    requestReconnect: () => set((s) => ({ connTick: s.connTick + 1 })),
    fsmState: FSMStatus.STATE_IDLE,
    setFsmState: (fsmState) => set({fsmState}),
    chosenFsmState: FSMStatus.STATE_IDLE,
    setChosenFsmState: (chosenFsmState) => set({chosenFsmState}),
    tofDistances: [0, 0, 0],
    setTofDistances: (tofDistances) => set({tofDistances}),
    speed: 0,
    gyroscope: [0, 0, 0],
    setAMU: ([speed, ...gyro]) => set({speed, gyroscope: gyro as [number, number, number]}),
    gaitMode: GaitMode.TROT,
    setGaitMode: (gaitMode) => set({gaitMode}),
    chosenGaitMode: GaitMode.TROT,
    setChosenGaitMode: (chosenGaitMode) => set({chosenGaitMode}),
    movementProgress: 0,
    setMovementProgress: (movementProgress) => set({movementProgress}),
    errorMessage: null,
    setErrorMessage: (errorMessage) => set({errorMessage}),
    clips: [],
    setClips: (clips) => set({ clips }),
    clipPlaying: false,
    setClipPlaying: (clipPlaying) => set({ clipPlaying }),
    pitchDeg: null,
    rollDeg: null,
    upsideDown: null,
    setOrientation: (pitchDeg, rollDeg, upsideDown) => set({ pitchDeg, rollDeg, upsideDown }),
}));
