// Tests for the JS-streamed-clip feature flag (Task #9).
//
// JS_STREAMED_CLIPS_ENABLED = false hides app-only clips from the list and
// disables the T:4 streamer entry point. These tests pin both helpers so a
// future flip-back to true (and the existing on-true behavior) stay honest.

import { buildRows, handleAppPlay } from '../components/clipList.logic';
import type { ClipInfo } from '../api/api-types';
import type { StreamClip } from '../api/streamClips';

const flashedFixture: ClipInfo[] = [{ id: 0, name: 'wave', ms: 1208 }];
const appFixture: StreamClip[] = [
  { name: 'wave', frame_ms: 50, duration_ms: 1208, loop: false, frames: [] },
  { name: 'unflashed', frame_ms: 50, duration_ms: 800, loop: false, frames: [] },
];

describe('buildRows', () => {
  it('with flag off, skips app-only rows (only flashed names appear)', () => {
    const rows = buildRows(flashedFixture, appFixture, false);
    expect(rows.map((r) => r.name)).toEqual(['wave']);
    expect(rows[0].flashed).toBeDefined();
  });

  it('with flag on, includes app-only rows alongside flashed', () => {
    const rows = buildRows(flashedFixture, appFixture, true);
    expect(rows.map((r) => r.name)).toEqual(['wave', 'unflashed']);
    const unflashed = rows.find((r) => r.name === 'unflashed')!;
    expect(unflashed.flashed).toBeUndefined();
    expect(unflashed.app).toBeDefined();
  });
});

describe('handleAppPlay', () => {
  it('with flag off, does NOT invoke the streamer', () => {
    const streamFn = jest.fn();
    const onDone = jest.fn();
    const clip = appFixture[1];
    handleAppPlay({ flag: false, streamFn, clip, onDone, loop: false });
    expect(streamFn).not.toHaveBeenCalled();
  });

  it('with flag on, invokes the streamer exactly once with the clip', () => {
    const streamFn = jest.fn();
    const onDone = jest.fn();
    const clip = appFixture[1];
    handleAppPlay({ flag: true, streamFn, clip, onDone, loop: false });
    expect(streamFn).toHaveBeenCalledTimes(1);
    expect(streamFn).toHaveBeenCalledWith(clip, onDone, false);
  });
});
