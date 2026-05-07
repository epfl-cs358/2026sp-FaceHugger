import { useEffect } from 'react';
import { connect, sendCommand, onMessage } from '../services/socket';
import { useRobotStore } from '../store/robotStore';
import { FSMStatus, GaitMode, SystemStatus } from '../api/api-types';


export const useRobotConnection = (ip: string) => {
  const setFsmState = useRobotStore((s) => s.setFsmState);
  const setTofDistances = useRobotStore((s) => s.setTofDistances);
  const setAMU = useRobotStore((s) => s.setAMU);
  const setGaitMode = useRobotStore((s) => s.setGaitMode);
  const setMovementProgress = useRobotStore((s) => s.setMovementProgress);
  const setErrorMessage = useRobotStore((s) => s.setErrorMessage);

  useEffect(() => {
    connect(ip);

    onMessage((data) => {
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

    return () => { /* cleanup / disconnect */ };
  }, [ip]);

  return { sendCommand };
};