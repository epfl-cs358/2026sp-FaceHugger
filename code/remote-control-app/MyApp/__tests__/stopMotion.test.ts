// Page-change cleanup: stopMotion must send STATE_IDLE (T:2, s:0) over the
// socket and also stop the JS clip streamer (no-op when no timer is running).
// Both cleanups are needed because gait commands (T:1/T:5) and firmware clips
// (T:7) keep running on the ESP32 until preempted, and app-streamed clips
// (Task #9, currently feature-flagged off) keep ticking on the JS side.
jest.mock('../services/socket', () => ({
  sendCommand: jest.fn(),
}));
jest.mock('../services/clipStreamer', () => ({
  stopStream: jest.fn(),
}));

import { stopMotion } from '../api/api-messages';
import { sendCommand } from '../services/socket';
import { stopStream } from '../services/clipStreamer';

describe('stopMotion', () => {
  beforeEach(() => {
    (sendCommand as jest.Mock).mockClear();
    (stopStream as jest.Mock).mockClear();
  });

  it('sends the IDLE state packet ({T:2,s:0}) once', () => {
    stopMotion();
    expect(sendCommand).toHaveBeenCalledTimes(1);
    expect(sendCommand).toHaveBeenCalledWith(JSON.stringify({ T: 2, s: 0 }));
  });

  it('calls stopStream() once to clear any JS-side ticker', () => {
    stopMotion();
    expect(stopStream).toHaveBeenCalledTimes(1);
  });
});
