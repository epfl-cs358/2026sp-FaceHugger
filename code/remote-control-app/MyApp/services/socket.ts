import { DEBUGGING, webSocketPort } from '../config/config';

export let ws: WebSocket | null = null;

// Packets queued while the socket is still CONNECTING. Flushed on open.
const pendingQueue: string[] = [];

// Only these command types are held while offline and replayed on connect:
// 2 = FSM state, 5 = gait mode (latching settings). Real-time movement (T:1) and
// one-shot actions like invert (T:6) are dropped while disconnected — replaying a
// burst of stale moves/inverts the moment the robot connects would make it lurch.
const QUEUEABLE_TYPES = new Set<number>([2, 5]);

const cmdType = (cmd: string): number | undefined => {
  try { return JSON.parse(cmd)?.T; } catch { return undefined; }
};

const flushPending = () => {
  if (ws?.readyState !== WebSocket.OPEN) return;
  while (pendingQueue.length > 0) {
    const cmd = pendingQueue.shift()!;
    if (DEBUGGING) console.log('[TX-flush]', cmd);
    ws.send(cmd);
  }
};

export const connect = (ip: string, port: number = webSocketPort) => {
  if (ws === null || ws === undefined) {
    ws = new WebSocket(`ws://${ip}:${port}`);
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
  // Not open yet (ws not created, or still CONNECTING). Only hold latching
  // settings (state/gait); drop real-time moves/actions so they can't flood the
  // robot on connect.
  if (!ws || ws.readyState === WebSocket.CONNECTING) {
    const type = cmdType(cmd);
    if (type === undefined || !QUEUEABLE_TYPES.has(type)) {
      if (DEBUGGING) console.log('[TX-dropped-offline]', cmd);
      return;
    }
    // Keep only the latest packet of each queued type (no duplicate state/gait).
    for (let i = pendingQueue.length - 1; i >= 0; i--) {
      if (cmdType(pendingQueue[i]) === type) pendingQueue.splice(i, 1);
    }
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