// Clip streamer: the app sends ONE T:12 (CMD_STREAM_FRAME) packet per frame,
// carrying math-space joint degrees per leg. The firmware applies
// translateToServo + CALIB on receipt. The app must NOT send T:4 here —
// that command is reserved for raw per-servo calibration sessions.
//
// This test pins the wire shape so a regression to per-channel T:4 streaming
// (which couples the bundle to firmware CALIB values) is caught at CI time.

jest.mock("../services/socket", () => ({
  sendCommand: jest.fn(),
}));

jest.useFakeTimers();

import { streamClip, stopStream } from "../services/clipStreamer";
import { sendCommand } from "../services/socket";
import type { StreamClip } from "../api/streamClips";

const synthClip = (frames: number, frame_ms = 50): StreamClip => ({
  name: "synth",
  frame_ms,
  duration_ms: frames * frame_ms,
  loop: false,
  frames: Array.from({ length: frames }, (_, i) => ({
    t: i * frame_ms,
    // Math-space angles — deliberately not 0-180-clamped so a stray T:4 clamp
    // (Math.max(0, Math.min(180, ...)) | 0) would be visible in the wire bytes.
    fr: [45 + i, -60 + i, -37 + i],
    fl: [135 - i, -60 + i, -40 + i],
    br: [-45 + i, -50 - i, -50 + i],
    bl: [-135 + i, -60 - i, -35 - i],
  })),
});

describe("clipStreamer T:12", () => {
  beforeEach(() => {
    (sendCommand as jest.Mock).mockClear();
    stopStream();
  });

  it("sends exactly one T:12 packet per frame (no T:4)", () => {
    const clip = synthClip(3);
    const onDone = jest.fn();
    streamClip(clip, onDone);

    // Frame 0 fires immediately; advance the authored period for frames 1 and 2.
    jest.advanceTimersByTime(clip.frame_ms * 2);

    const payloads = (sendCommand as jest.Mock).mock.calls.map((c) =>
      JSON.parse(c[0]),
    );
    expect(payloads).toHaveLength(3);
    expect(payloads.every((p) => p.T === 12)).toBe(true);
    expect(payloads.find((p) => p.T === 4)).toBeUndefined();
  });

  it("each T:12 packet carries the four per-leg triples verbatim (no clamp, no scale)", () => {
    const clip = synthClip(1);
    streamClip(clip, jest.fn());

    const sent = JSON.parse((sendCommand as jest.Mock).mock.calls[0][0]);
    expect(sent).toEqual({
      T: 12,
      fr: clip.frames[0].fr,
      fl: clip.frames[0].fl,
      br: clip.frames[0].br,
      bl: clip.frames[0].bl,
    });
  });

  it("calls onDone after the last frame when loop=false", () => {
    const clip = synthClip(2);
    const onDone = jest.fn();
    streamClip(clip, onDone);

    // Frame 0 fires at tick(); frame 1 fires after one interval; the third
    // tick exhausts the buffer and calls onDone.
    jest.advanceTimersByTime(clip.frame_ms * 2);
    expect(onDone).toHaveBeenCalledTimes(1);
  });

  it("loop=true replays from frame 0 without calling onDone", () => {
    const clip = synthClip(2);
    const onDone = jest.fn();
    streamClip(clip, onDone, true);

    // Run through a couple of full cycles.
    jest.advanceTimersByTime(clip.frame_ms * 5);
    expect(onDone).not.toHaveBeenCalled();
    const payloads = (sendCommand as jest.Mock).mock.calls.map((c) =>
      JSON.parse(c[0]),
    );
    // At least one packet repeats with the same fr triple as frame 0 — the
    // weak proof that we wrapped without emitting onDone.
    const frame0Sig = JSON.stringify(clip.frames[0].fr);
    const frame0Hits = payloads.filter((p) => JSON.stringify(p.fr) === frame0Sig);
    expect(frame0Hits.length).toBeGreaterThanOrEqual(2);
  });
});
