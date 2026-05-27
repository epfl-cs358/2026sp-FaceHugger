// Client-side clip playback for app-bundled clips (api/streamClips.ts). The
// robot has no on-board copy of these, so the app streams the baked servo
// frames over the live socket as CMD_CALIBRATE (T:4) packets, one per changed
// channel, at the clip's authored frame_ms. This is a TypeScript port of the
// exporter's browser-console player (to_js in fh_clip_panel.py): same delta
// encoding (only emit a channel when it changed) and the same defensive clamp.
//
// Unlike a firmware clip (one T:7, the ESP32 owns the timing), a streamed clip
// depends on the live connection and the phone's timer — fine for previewing an
// unflashed animation, not a substitute for flashing.
import { sendCommand } from "./socket";
import { StreamClip } from "../api/streamClips";
import { useRobotStore } from "../store/robotStore";

const LEGS = ["fr", "fl", "br", "bl"] as const;
const LEG_IDS: Record<(typeof LEGS)[number], number> = { fr: 0, fl: 1, br: 2, bl: 3 };
const clamp = (v: number) => Math.max(0, Math.min(180, v | 0));

let timer: ReturnType<typeof setInterval> | null = null;

// Stop the current stream (if any). Servos hold their last commanded angle.
export const stopStream = () => {
  if (timer !== null) {
    clearInterval(timer);
    timer = null;
  }
};

// Stream a bundled clip. By default plays once then calls onDone; with loop=true
// it replays from frame 0 at each end and onDone never fires (stop with
// stopStream). Cancels any clip already streaming first.
export const streamClip = (
  clip: StreamClip,
  onDone: () => void,
  loop: boolean = false,
) => {
  stopStream();
  const frames = clip.frames;
  const last: Record<string, number> = {}; // delta cache: only resend changed channels
  let i = 0;

  const tick = () => {
    if (i >= frames.length) {
      if (loop) {
        i = 0; // replay from the top; keep streaming
      } else {
        stopStream();
        onDone();
        return;
      }
    }
    const frame = frames[i++];
    // T:4 carries absolute servo angles, so the firmware invert flag can't act
    // on a streamed clip (it only mirrors firmware-driven motion). Apply the
    // same pitch-only mirror here — thigh/knee = 180 - angle, hip unchanged
    // (== firmware applyInvert) — read live so a mid-clip invert takes effect
    // on the next frame.
    const inverted = useRobotStore.getState().inverted;
    for (const leg of LEGS) {
      const raw = frame[leg];
      const angles = inverted ? [raw[0], 180 - raw[1], 180 - raw[2]] : raw;
      for (let j = 0; j < 3; j++) {
        const a = clamp(angles[j]);
        const key = leg + ":" + j;
        if (last[key] === a) continue;
        last[key] = a;
        sendCommand(JSON.stringify({ T: 4, id: LEG_IDS[leg], servo_id: j, a }));
      }
    }
  };

  tick(); // frame 0 immediately, then on the authored period
  timer = setInterval(tick, clip.frame_ms);
};
