import { useEffect, useRef } from 'react';
import { connect, sendCommand, onMessage } from '../services/socket';
import { useRobotStore } from '../store/robotStore';
import { ClipListResponse, FSMStateModification, FSMStatus, GaitIntegration, GaitMode, SystemStatus } from '../api/api-types';
import { requestClipList } from '../api/api-messages';
import { useSocketStatus } from './useSocketStatus';


export const useRobotConnection = (ip: string) => {
  const setFsmState = useRobotStore((s) => s.setFsmState);
  const setTofDistances = useRobotStore((s) => s.setTofDistances);
  const setAMU = useRobotStore((s) => s.setAMU);
  const setGaitMode = useRobotStore((s) => s.setGaitMode);
  const setMovementProgress = useRobotStore((s) => s.setMovementProgress);
  const setErrorMessage = useRobotStore((s) => s.setErrorMessage);
  const setClips = useRobotStore((s) => s.setClips);
  const setClipPlaying = useRobotStore((s) => s.setClipPlaying);

  const fsmState = useRobotStore((s) => s.fsmState);
  const chosenFsmState = useRobotStore((s) => s.chosenFsmState);
  const gaitMode = useRobotStore((s) => s.gaitMode);
  const chosenGaitMode = useRobotStore((s) => s.chosenGaitMode);

  const isConnected = useSocketStatus();
  const prevIsConnectedRef = useRef(false);
  const chosenFsmStateRef = useRef(chosenFsmState);
  const chosenGaitModeRef = useRef(chosenGaitMode);
  useEffect(() => { chosenFsmStateRef.current = chosenFsmState; }, [chosenFsmState]);
  useEffect(() => { chosenGaitModeRef.current = chosenGaitMode; }, [chosenGaitMode]);


  useEffect(() => {
    connect(ip);

    onMessage((data) => {
      if (data.T) {
        switch (data.T) {
          case 8: {
            const resp = data as ClipListResponse;
            if (Array.isArray(resp.clips)) {
              setClips(resp.clips);
            }
            break;
          }
          case 10: {
            const { T, ...rest } = data;
            const status = rest as SystemStatus;
            setFsmState(status.s as FSMStatus);
            setTofDistances(status.d);
            setAMU(status.a);
            setGaitMode(status.g as GaitMode);
            setMovementProgress(status.pc);
            setErrorMessage(status.e ?? null);
            break;
          }
        }
      }
    });

    return () => { /* cleanup / disconnect */ };
  }, [ip]);

  // On reconnect, immediately push all chosen state to the robot
  useEffect(() => {
    if (isConnected && !prevIsConnectedRef.current) {
      sendCommand(JSON.stringify({ T: 2, s: chosenFsmStateRef.current } as FSMStateModification));
      sendCommand(JSON.stringify({ T: 5, g: chosenGaitModeRef.current } as GaitIntegration));
      requestClipList();
    }
    prevIsConnectedRef.current = isConnected;
  }, [isConnected]);

  useEffect(() => {
    if (chosenFsmState === fsmState) return;
    const interval = setInterval(() => {
      if(isConnected){
        sendCommand(JSON.stringify({ T: 2, s: chosenFsmState } as FSMStateModification));
      }
    }, 1000);
    return () => clearInterval(interval);
  }, [chosenFsmState, fsmState, isConnected]);

  useEffect(() => {
    if (chosenGaitMode === gaitMode) return;
    const interval = setInterval(() => {
      if(isConnected){
        sendCommand(JSON.stringify({ T: 5, g: chosenGaitMode } as GaitIntegration));
      }
    }, 1000);
    return () => clearInterval(interval);
  }, [chosenGaitMode, gaitMode, isConnected]);

  return { sendCommand };
};