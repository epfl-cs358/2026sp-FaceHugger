import {create} from 'zustand'
import { ClipInfo, FSMStatus, GaitMode } from '../api/api-types';

type RobotStore = {
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
};

export const useRobotStore = create<RobotStore>((set) => ({
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
}));
