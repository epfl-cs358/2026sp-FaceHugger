// Orientation tile formatters. The Actions screen's orientation tile renders
// pitch / roll / upside-down straight from the T:10 broadcast (already pumped
// into robotStore by useRobotConnection). We test the pure format helpers in
// isolation so a render mock isn't needed — same approach as poseButtons.test.

import { SystemStatus } from '../api/api-types';
import {
  formatAngle,
  formatPitch,
  formatRoll,
  formatOrientationState,
} from '../api/orientation';

describe('orientation formatters', () => {
  it('formatAngle renders signed degrees to 1 decimal place', () => {
    expect(formatAngle(12.3)).toBe('+12.3°');
    expect(formatAngle(-5.7)).toBe('-5.7°');
    expect(formatAngle(0)).toBe('+0.0°');
  });

  it('formatAngle returns the "—" placeholder for missing values', () => {
    expect(formatAngle(null)).toBe('—');
    expect(formatAngle(undefined)).toBe('—');
    expect(formatAngle(NaN)).toBe('—');
  });

  it('formatPitch / formatRoll prefix the label', () => {
    expect(formatPitch(12.3)).toBe('Pitch: +12.3°');
    expect(formatRoll(-5.7)).toBe('Roll: -5.7°');
    expect(formatPitch(null)).toBe('Pitch: —');
    expect(formatRoll(undefined)).toBe('Roll: —');
  });

  it('formatOrientationState renders the latched flag', () => {
    expect(formatOrientationState(false)).toBe('Upright');
    expect(formatOrientationState(true)).toBe('Inverted');
    expect(formatOrientationState(null)).toBe('—');
    expect(formatOrientationState(undefined)).toBe('—');
  });

  it('given a mocked T:10 payload, the helpers produce the spec strings', () => {
    // Mirrors PR #103 spec: {T:10, pitch_deg:12.3, roll_deg:-5.7, upside_down:false}
    // -> "Pitch: +12.3°", "Roll: -5.7°", "Upright".
    const payload: SystemStatus = {
      T: 10,
      s: 0,
      d: [],
      a: [],
      g: 0,
      pc: 0,
      e: '',
      pitch_deg: 12.3,
      roll_deg: -5.7,
      upside_down: false,
    };
    expect(formatPitch(payload.pitch_deg)).toBe('Pitch: +12.3°');
    expect(formatRoll(payload.roll_deg)).toBe('Roll: -5.7°');
    expect(formatOrientationState(payload.upside_down)).toBe('Upright');
  });
});
