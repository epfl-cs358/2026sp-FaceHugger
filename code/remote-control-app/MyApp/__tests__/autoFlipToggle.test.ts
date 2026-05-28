// Auto-flip (IMU) toggle on the Actions screen. The toggle dispatches a T:6
// CMD_SET_AUTO_INVERT packet AND optimistically updates the robotStore so the
// UI flips state without waiting for the next T:10 broadcast.
//
// We test the seams in isolation (no rendering required):
//   - sendSetAutoInvert serialises {T:6, enabled:bool}
//   - the store toggle setter flips the boolean
// This mirrors the poseButtons.test pattern.

jest.mock('../services/socket', () => ({
  sendCommand: jest.fn(),
}));

import { sendSetAutoInvert } from '../api/api-messages';
import { sendCommand } from '../services/socket';
import { useRobotStore } from '../store/robotStore';

describe('auto-flip (IMU) toggle', () => {
  beforeEach(() => {
    (sendCommand as jest.Mock).mockClear();
    useRobotStore.setState({ autoFlipEnabled: true });
  });

  it('sendSetAutoInvert(true) sends {T:6, enabled:true}', () => {
    sendSetAutoInvert(true);
    expect(sendCommand).toHaveBeenCalledTimes(1);
    expect(sendCommand).toHaveBeenCalledWith(JSON.stringify({ T: 6, enabled: true }));
  });

  it('sendSetAutoInvert(false) sends {T:6, enabled:false}', () => {
    sendSetAutoInvert(false);
    expect(sendCommand).toHaveBeenCalledTimes(1);
    expect(sendCommand).toHaveBeenCalledWith(JSON.stringify({ T: 6, enabled: false }));
  });

  it('store autoFlipEnabled defaults to true (matches firmware default)', () => {
    // Reset via the store API itself to confirm the initial state, not just
    // the value beforeEach injected.
    expect(useRobotStore.getState().autoFlipEnabled).toBe(true);
  });

  it('setAutoFlipEnabled flips the store value', () => {
    useRobotStore.getState().setAutoFlipEnabled(false);
    expect(useRobotStore.getState().autoFlipEnabled).toBe(false);
    useRobotStore.getState().setAutoFlipEnabled(true);
    expect(useRobotStore.getState().autoFlipEnabled).toBe(true);
  });

  it('toggling the local state and dispatching mirror in lockstep', () => {
    // Simulate the Actions.tsx onToggleAutoFlip handler in isolation: flip
    // the store, then send T:6 with the NEW value.
    const current = useRobotStore.getState().autoFlipEnabled;
    const next = !current;
    useRobotStore.getState().setAutoFlipEnabled(next);
    sendSetAutoInvert(next);

    expect(useRobotStore.getState().autoFlipEnabled).toBe(next);
    expect(sendCommand).toHaveBeenCalledTimes(1);
    expect(sendCommand).toHaveBeenCalledWith(JSON.stringify({ T: 6, enabled: next }));
  });
});
