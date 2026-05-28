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
    // App-side mirror of the firmware invert flag. The robot flips its own
    // motion (gaits/flashed clips) from the T:6/T:9 flag, but app-streamed clips
    // are raw absolute T:4 writes that bypass that flag — so the streamer mirrors
    // each frame by this same flag. Kept in sync with the firmware via the
    // Actions invert button (toggles this AND sends T:6).
    inverted: boolean,
    setInverted: (inverted: boolean) => void,
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
    inverted: false,
    setInverted: (inverted) => set({ inverted }),
}));
