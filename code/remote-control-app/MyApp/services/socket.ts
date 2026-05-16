import { DEBUGGING } from '../config/config';

export let ws: WebSocket | null = null;

export const connect = (ip: string) => {
  if(ws === null || ws === undefined){
    ws = new WebSocket(`ws://${ip}:81`);
  }
};

export const isConnected = () => ws?.readyState === WebSocket.OPEN;

export const sendCommand = (cmd: string) => {
  if(ws?.readyState === WebSocket.OPEN){
    if (DEBUGGING) console.log('[TX]', cmd);
    ws?.send(cmd);
  }
};

export const onMessage = (cb: (data: any) => void) => {
  if (ws) ws.onmessage = (e) => {
    const parsed = JSON.parse(e.data);
    if (DEBUGGING) console.log('[RX]', parsed);
    cb(parsed);
  };
};