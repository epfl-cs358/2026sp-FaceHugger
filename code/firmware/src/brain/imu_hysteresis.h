#ifndef IMU_HYSTERESIS_H
#define IMU_HYSTERESIS_H

// Pure helper for the IMU upside-down latch. Kept separate from sensors.{h,cpp}
// so it can be unit-tested on the host without dragging in Wire / MPU stubs.
//
// Thresholds chosen so chatter near 90 deg (robot on its side / mid-flip) cannot
// flip the boolean back and forth: only crossing one of the two extremes does.
//
//   flip-up  (false -> true):  angle > IMU_FLIP_UP_DEG    (150)
//   clear    (true  -> false): angle < IMU_FLIP_DOWN_DEG  (30)
//   in-between: state held.
//
// Boundaries (==150 from upright, ==30 from inverted) are NOT crossed; the
// strict inequalities are required.
constexpr float IMU_FLIP_UP_DEG   = 150.0f;
constexpr float IMU_FLIP_DOWN_DEG = 30.0f;

inline bool imuInvertedHysteresis(float angle_deg, bool prev_state) {
    if (!prev_state) {
        // Currently upright — only flip when we cross the upper threshold.
        return angle_deg > IMU_FLIP_UP_DEG;
    } else {
        // Currently inverted — clear when we cross the lower threshold.
        return !(angle_deg < IMU_FLIP_DOWN_DEG);
    }
}

#endif
