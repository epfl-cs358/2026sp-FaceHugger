import { useRobotStore } from '../store/robotStore';

// Zustand stores expose getState()/setState() directly — no React rendering needed.

describe('clips state', () => {
  beforeEach(() => {
    useRobotStore.setState({ clips: [], clipPlaying: false });
  });

  it('starts with empty clips and clipPlaying false', () => {
    const state = useRobotStore.getState();
    expect(state.clips).toEqual([]);
    expect(state.clipPlaying).toBe(false);
  });

  it('setClips stores the list', () => {
    useRobotStore.getState().setClips([{ id: 0, name: 'wave', ms: 1208 }]);
    const state = useRobotStore.getState();
    expect(state.clips).toHaveLength(1);
    expect(state.clips[0].name).toBe('wave');
    expect(state.clips[0].ms).toBe(1208);
  });

  it('setClipPlaying toggles the flag', () => {
    useRobotStore.getState().setClipPlaying(true);
    expect(useRobotStore.getState().clipPlaying).toBe(true);
    useRobotStore.getState().setClipPlaying(false);
    expect(useRobotStore.getState().clipPlaying).toBe(false);
  });

  it('getState().clipPlaying reflects latest value synchronously', () => {
    useRobotStore.getState().setClipPlaying(true);
    // Synchronous read must return true immediately (no render cycle needed)
    expect(useRobotStore.getState().clipPlaying).toBe(true);
    useRobotStore.getState().setClipPlaying(false);
    expect(useRobotStore.getState().clipPlaying).toBe(false);
  });
});
