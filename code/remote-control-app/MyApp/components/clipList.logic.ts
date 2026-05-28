// Pure helpers extracted from ClipList.tsx so they're jest-testable without
// rendering React Native. The component still owns state, timers, and the
// rendered tree — these functions just decide *what* to show and *whether* to
// stream, both of which the JS_STREAMED_CLIPS_ENABLED flag now gates.

import type { ClipInfo } from '../api/api-types';
import type { StreamClip } from '../api/streamClips';

// One name may exist as a flashed (on-robot, T:7) clip, an app (streamed, T:4)
// clip, or both — we keep both rather than dedup, so an authored clip is
// playable before it's flashed. When the JS-streamed path is feature-flagged
// off, app-only rows are dropped entirely (their button would be hidden, so
// listing them with no action would just confuse).
export type Row = { name: string; ms: number; flashed?: ClipInfo; app?: StreamClip };

export const buildRows = (
  flashed: ClipInfo[],
  app: StreamClip[],
  jsStreamedEnabled: boolean,
): Row[] => {
  const byName = new Map<string, Row>();
  const order: string[] = [];
  for (const c of flashed) {
    byName.set(c.name, { name: c.name, ms: c.ms, flashed: c });
    order.push(c.name);
  }
  for (const c of app) {
    const existing = byName.get(c.name);
    if (existing) {
      existing.app = c;
    } else if (jsStreamedEnabled) {
      // App-only row: only surface when streaming is enabled.
      byName.set(c.name, { name: c.name, ms: c.duration_ms, app: c });
      order.push(c.name);
    }
  }
  return order.map((n) => byName.get(n)!);
};

// Defense-in-depth wrapper for the "Play via app" handler. The button is
// already hidden when the flag is off, but the handler is exported behavior —
// gate here too so future callers can't accidentally re-enable T:4 streaming.
export const handleAppPlay = (opts: {
  flag: boolean;
  streamFn: (clip: StreamClip, onDone: () => void, loop: boolean) => void;
  clip: StreamClip;
  onDone: () => void;
  loop: boolean;
}): void => {
  if (!opts.flag) return;
  opts.streamFn(opts.clip, opts.onDone, opts.loop);
};
