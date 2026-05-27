export const webSocketIP = "192.168.4.1" //default esp32 gateway ip
// WebSocket port. 81 = the real robot (ESP32 firmware). To drive the PyBullet
// simulator instead, set this to 8081 and point webSocketIP at the dev machine's
// LAN IP, then run: `python facehugger.py serve --host 0.0.0.0` (in code/simulation).
export const webSocketPort = 81;
export const DEBUGGING = true;