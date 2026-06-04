// Client-side clip playback for app-bundled clips (api/streamClips.ts). The
// robot has no on-board copy of these, so the app streams the baked math-space
// frames over the live socket as CMD_STREAM_FRAME (T:12) packets — one packet
// per authored frame, carrying all four per-leg triples. The firmware applies
// translateToServo + CALIB on receipt (spinal_cord.cpp::streamMathFrame), so
// this side never touches hardware calibration values.
//
// T:4 (raw per-servo write) is reserved for calibration screens; clip streaming
// must not emit it (the per-frame schema is locked by __tests__/clipStreamer.test.ts).
//
// Unlike a firmware clip (one T:7, the ESP32 owns the timing), a streamed clip
// depends on the live connection and the phone's timer — fine for previewing an
// unflashed animation, not a substitute for flashing.
import { sendCommand } from "./socket";
import { StreamClip } from "../api/streamClips";

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
    // One T:12 packet carries the whole pose; firmware deadband (per-channel
    // delta gate in streamMathFrame) suppresses servo writes that didn't move.
    sendCommand(
      JSON.stringify({
        T: 12,
        fr: frame.fr,
        fl: frame.fl,
        br: frame.br,
        bl: frame.bl,
      }),
    );
  };

  tick(); // frame 0 immediately, then on the authored period
  timer = setInterval(tick, clip.frame_ms);
};
