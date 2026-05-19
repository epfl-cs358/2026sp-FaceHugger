import { DEBUGGING } from '../config/config';

export let ws: WebSocket | null = null;

// Packets queued while the socket is still CONNECTING. Flushed on open.
const pendingQueue: string[] = [];

const flushPending = () => {
  if (ws?.readyState !== WebSocket.OPEN) return;
  while (pendingQueue.length > 0) {
    const cmd = pendingQueue.shift()!;
    if (DEBUGGING) console.log('[TX-flush]', cmd);
    ws.send(cmd);
  }
};

export const connect = (ip: string) => {
  if (ws === null || ws === undefined) {
    ws = new WebSocket(`ws://${ip}:81`);
    ws.onopen = () => {
      if (DEBUGGING) console.log('[WS] open');
      flushPending();
    };
    ws.onclose = () => {
      if (DEBUGGING) console.log('[WS] close');
      ws = null;
    };
    ws.onerror = (e) => {
      if (DEBUGGING) console.log('[WS] error', e);
    };
  }
};

export const isConnected = () => ws?.readyState === WebSocket.OPEN;

export const sendCommand = (cmd: string) => {
  if (ws?.readyState === WebSocket.OPEN) {
    if (DEBUGGING) console.log('[TX]', cmd);
    ws.send(cmd);
    return;
  }
  // Queue if not open yet — covers both "ws not created yet" (child useEffect
  // fired before connect()) and "still CONNECTING". Flushed on onopen.
  if (!ws || ws.readyState === WebSocket.CONNECTING) {
    if (DEBUGGING) console.log('[TX-queued]', cmd);
    pendingQueue.push(cmd);
    return;
  }
  if (DEBUGGING) console.log('[TX-dropped]', cmd);
};

export const onMessage = (cb: (data: any) => void) => {
  if (ws) ws.onmessage = (e) => {
    const parsed = JSON.parse(e.data);
    if (DEBUGGING) console.log('[RX]', parsed);
    cb(parsed);
  };
};