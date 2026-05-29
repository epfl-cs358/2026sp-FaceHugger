// REST and NEUTRAL pose buttons (Actions screen, Task #11). Both buttons used
// to send a burst of 12 T:4 (CMD_CALIBRATE) packets that snapped each joint.
// They now send ONE T:2 (CMD_STATE) packet with an optional `dur_ms` so the
// firmware eases the pose over the configured delay (POSE_EASE_MS default
// 1000 ms). The handlers are exported from api-messages so we can drive them
// in isolation without rendering the screen.

jest.mock('../services/socket', () => ({
  sendCommand: jest.fn(),
}));

import { sendRestPose, sendNeutralStance } from '../api/api-messages';
import { sendCommand } from '../services/socket';
import { POSE_EASE_MS } from '../config/config';

describe('pose buttons', () => {
  beforeEach(() => {
    (sendCommand as jest.Mock).mockClear();
  });

  it('REST sends one T:2 packet with s=4 and the configured dur_ms', () => {
    sendRestPose();
    expect(sendCommand).toHaveBeenCalledTimes(1);
    expect(sendCommand).toHaveBeenCalledWith(
      JSON.stringify({ T: 2, s: 4, dur_ms: POSE_EASE_MS })
    );
  });

  it('NEUTRAL sends one T:2 packet with s=5 and the configured dur_ms', () => {
    sendNeutralStance();
    expect(sendCommand).toHaveBeenCalledTimes(1);
    expect(sendCommand).toHaveBeenCalledWith(
      JSON.stringify({ T: 2, s: 5, dur_ms: POSE_EASE_MS })
    );
  });

  it('neither REST nor NEUTRAL emits any T:4 packets (regression vs old burst)', () => {
    sendRestPose();
    sendNeutralStance();
    const calls = (sendCommand as jest.Mock).mock.calls.map((c) => JSON.parse(c[0]));
    expect(calls.find((p) => p.T === 4)).toBeUndefined();
    // And only two packets total — one per button.
    expect(calls).toHaveLength(2);
  });

  it('POSE_EASE_MS default is 1000 ms (spec)', () => {
    expect(POSE_EASE_MS).toBe(1000);
  });
});
