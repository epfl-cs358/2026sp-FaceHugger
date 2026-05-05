export let ws: WebSocket | null = null;

export const connect = (ip: string) => {
  ws = new WebSocket(`ws://${ip}:8080`);
};

export const isConnected = () => ws?.readyState === WebSocket.OPEN;

export const sendCommand = (cmd: string) => {
  if(ws?.readyState === WebSocket.OPEN){
    ws?.send(cmd);
  }
};

export const onMessage = (cb: (data: any) => void) => {
  if (ws) ws.onmessage = (e) => cb(JSON.parse(e.data));
};