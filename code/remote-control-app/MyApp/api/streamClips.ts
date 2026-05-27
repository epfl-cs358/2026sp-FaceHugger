// Clips bundled into the app at build time (assets/clips_extra.json, written by
// the FH Clip Panel exporter). Unlike the firmware clips listed over T:8, these
// don't need to be flashed: the app streams their servo frames itself (T:4),
// so a newly authored animation is playable as soon as it's exported — no flash.
import clipsExtra from "../assets/clips_extra.json";

export type StreamFrame = {
  t: number; // ms from clip start
  fr: number[]; // [hip, thigh, knee] servo degrees, 0-180
  fl: number[];
  br: number[];
  bl: number[];
};

export type StreamClip = {
  name: string;
  frame_ms: number; // wall-clock period between frames (authored scene FPS)
  duration_ms: number;
  loop: boolean;
  frames: StreamFrame[];
};

export const STREAM_CLIPS: StreamClip[] = (clipsExtra as { clips: StreamClip[] }).clips;
