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
export const SIM_IP = "localhost"; // dev machine running the sim
export const SIM_PORT = 8081; // `facehugger.py sim --serve` default

// Startup default. `facehugger.py sim --app` injects EXPO_PUBLIC_WS_IP/_WS_PORT
// so the launched web app boots already pointed at the sim; a plain launch
// (the `app` subcommand or `npm run web`) has no such env, so it defaults to the
// robot. Either way the Settings screen can switch targets at runtime.
export const DEFAULT_IP = process.env.EXPO_PUBLIC_WS_IP ?? ROBOT_IP;
export const DEFAULT_PORT = Number(process.env.EXPO_PUBLIC_WS_PORT ?? ROBOT_PORT);

// Verbose socket logging.
export const DEBUGGING = true;
