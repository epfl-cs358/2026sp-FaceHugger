import {create} from 'zustand'
import { FSMStatus } from '../api/api-types';

type RobotStore = {
    fsmState: FSMStatus,
    setState: (state: FSMStatus) => void
    errorMessage: string | null,
    setErrorMessage: (error: string) => void
    tofDistance: number,
    setTofDistance: (distance: number) => void,
    speed: number,
    setSpeed: (speed: number) => void
};

export const useRobotStore = create<RobotStore>((set) => ({
    fsmState: FSMStatus.STATE_IDLE,
    setState: (fsmState) => set({fsmState}),
    errorMessage: null,
    setErrorMessage: (errorMessage) => set({errorMessage}),
    tofDistance: Number.MAX_VALUE,
    setTofDistance: (tofDistance) => set({tofDistance}),
    speed: 0,
    setSpeed: (speed) => set({speed})
}));
