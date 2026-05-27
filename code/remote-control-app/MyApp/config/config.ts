// Connection presets for the WebSocket the app drives (the T: protocol).
//
//   Robot — the ESP32 firmware over its own Wi-Fi access point.
//   Sim   — the PyBullet simulator on your dev machine, started with
//           `python facehugger.py serve --host 0.0.0.0` (from code/simulation).
//
// The ACTIVE target lives in the robot store and can be changed at runtime from
// the Settings screen. These constants are just the startup default and the two
// quick presets the Settings screen offers.

export const ROBOT_IP = "192.168.4.1"; // ESP32 soft-AP gateway
export const ROBOT_PORT = 81; // firmware WebSocket port
export const SIM_IP = "localhost"; // dev machine running `serve`
export const SIM_PORT = 8081; // facehugger.py serve default

// Which preset the app starts on. Change these to flip the startup default
// (e.g. to SIM_IP / SIM_PORT while developing against the simulator).
export const DEFAULT_IP = ROBOT_IP;
export const DEFAULT_PORT = ROBOT_PORT;

// Verbose socket logging.
export const DEBUGGING = true;
