import { useEffect, useRef } from 'react';
import { connect, disconnect, sendCommand, onMessage } from '../services/socket';
import { useRobotStore } from '../store/robotStore';
import { ClipListResponse, FSMStateModification, FSMStatus, GaitIntegration, GaitMode, SystemStatus } from '../api/api-types';
import { requestClipList } from '../api/api-messages';
import { useSocketStatus } from './useSocketStatus';


export const useRobotConnection = () => {
  const ip = useRobotStore((s) => s.connIP);
  const port = useRobotStore((s) => s.connPort);
  const connTick = useRobotStore((s) => s.connTick);
  const setFsmState = useRobotStore((s) => s.setFsmState);
  const setTofDistances = useRobotStore((s) => s.setTofDistances);
  const setAMU = useRobotStore((s) => s.setAMU);
  const setGaitMode = useRobotStore((s) => s.setGaitMode);
  const setMovementProgress = useRobotStore((s) => s.setMovementProgress);
  const setErrorMessage = useRobotStore((s) => s.setErrorMessage);
  const setClips = useRobotStore((s) => s.setClips);
  const setClipPlaying = useRobotStore((s) => s.setClipPlaying);

  const chosenFsmState = useRobotStore((s) => s.chosenFsmState);
  const chosenGaitMode = useRobotStore((s) => s.chosenGaitMode);

  const isConnected = useSocketStatus();
  const prevIsConnectedRef = useRef(false);
  const chosenFsmStateRef = useRef(chosenFsmState);
  const chosenGaitModeRef = useRef(chosenGaitMode);
  useEffect(() => { chosenFsmStateRef.current = chosenFsmState; }, [chosenFsmState]);
  useEffect(() => { chosenGaitModeRef.current = chosenGaitMode; }, [chosenGaitMode]);


  useEffect(() => {
    connect(ip, port);

    onMessage((data) => {
      // The T:8 (list clips) reply carries no `T` field — it's just
      // {clips:[{id,name,ms},...]} (firmware buildClipListJson). Handle it
      // before the T-switch, otherwise the flashed list never populates.
      if (Array.isArray((data as ClipListResponse).clips)) {
        setClips((data as ClipListResponse).clips);
        return;
      }
      if (data.T) {
        switch (data.T) {
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

    // Tear down on unmount and whenever the target changes, so the next
    // connect() opens a fresh socket to the new ip/port. connTick is bumped by
    // the "tap to retry" badge to re-run this effect (reattaching onMessage).
    return () => { disconnect(); };
  }, [ip, port, connTick]);

  // On reconnect, immediately push all chosen state to the robot
  useEffect(() => {
    if (isConnected && !prevIsConnectedRef.current) {
      sendCommand(JSON.stringify({ T: 2, s: chosenFsmStateRef.current } as FSMStateModification));
      sendCommand(JSON.stringify({ T: 5, g: chosenGaitModeRef.current } as GaitIntegration));
      requestClipList();
    }
    prevIsConnectedRef.current = isConnected;
  }, [isConnected]);

  // Push the chosen state/gait ONCE whenever it changes (and on connect). We do not
  // loop on it: the firmware sends no telemetry, so fsmState/gaitMode never catch up,
  // and the old 1s interval re-sent {T:2,s:...} forever — re-arming STATE_WALK every
  // second and fighting the gait's graceful-stop, which made the robot twitch on its
  // own with no input. Re-send on reconnect is handled by the effect above.
  useEffect(() => {
    if (isConnected) {
      sendCommand(JSON.stringify({ T: 2, s: chosenFsmState } as FSMStateModification));
    }
  }, [chosenFsmState, isConnected]);

  useEffect(() => {
    if (isConnected) {
      sendCommand(JSON.stringify({ T: 5, g: chosenGaitMode } as GaitIntegration));
    }
  }, [chosenGaitMode, isConnected]);

  return { sendCommand };
};