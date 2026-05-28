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

// Pose-button (REST / NEUTRAL) ease window, sent as the optional `dur_ms` on
// T:2. The firmware eases the joints to the target pose over this many ms
// instead of snapping; clamped on the firmware side at POSE_EASE_MS_MAX (5 s).
export const POSE_EASE_MS = 1000;

// JS-streamed clip playback (app/services/clipStreamer.ts: T:4 frames at the
// authored frame_ms). Off for now — on real hardware the frame rate saturates
// the ESP32 WebSocket and the connection drops mid-clip. Flashed clips (T:7,
// the firmware owns the timing) are unaffected.
//
// To flip back on once firmware throughput is fixed: set to `true`. The
// streamer source (services/clipStreamer.ts, api/streamClips.ts) is untouched
// and the cleanup `stopStream()` is a no-op when never started.
export const JS_STREAMED_CLIPS_ENABLED = false;
