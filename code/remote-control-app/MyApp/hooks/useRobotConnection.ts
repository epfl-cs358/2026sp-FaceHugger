import { useEffect } from 'react';
import { connect, sendCommand, onMessage } from '../services/socket';
import { useRobotStore } from '../store/robotStore';
import { FSMStatus, SystemStatus } from '../api/api-types';


export const useRobotConnection = (ip: string) => {
  const setFsmState = useRobotStore((s) => s.setState);
  const setErrorMessage = useRobotStore((s) => s.setErrorMessage);
  const setTof = useRobotStore((s) => s.setTofDistance);

  useEffect(() => {
    connect(ip);

    onMessage((data) => {
      if(data.T){
        switch(data.T){
            case 10: //for the time being that is the only message our robot sends us
                const {T, ...rest} = data;
                const robotStatus = rest as SystemStatus;
                if(robotStatus.e){
                    setErrorMessage(robotStatus.e);
                }else{
                    setFsmState(robotStatus.s as FSMStatus);
                    setTof(robotStatus.d[0]);
                }
                break;
        }
      }
    });

    return () => { /* cleanup / disconnect */ };
  }, [ip]);

  return { sendCommand };
};