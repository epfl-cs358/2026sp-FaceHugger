"""PyBullet → firmware IMU emulation: hysteresis latch + pitch/roll.

Hysteresis matches code/firmware/src/brain/imu_hysteresis.h:
  flip:  tilt > 150 deg  (body_up_z < cos(150°) ≈ -0.866)
  clear: tilt <  30 deg  (body_up_z > cos( 30°) ≈  0.866)
"""

import math

_IMU_FLIP_COS = math.cos(math.radians(150.0))  # ≈ -0.866
_IMU_CLEAR_COS = math.cos(math.radians(30.0))  # ≈  0.866


def _body_up_z(quat):
    """World-frame z-component of the body-frame +z axis, from quaternion (x,y,z,w).

    The rotation matrix's third column is the rotated +z; we only need the z
    row of that column. body_up_z == 1.0 upright, -1.0 upside-down.
    """
    x, y, z, w = quat
    return 1.0 - 2.0 * (x * x + y * y)


def imu_hysteresis_step(body_up_z, prev_state):
    """Apply the firmware hysteresis to a single body_up_z sample."""
    if prev_state:
        # currently inverted: clear only when we drop below 30° tilt
        return not (body_up_z > _IMU_CLEAR_COS)
    # currently upright: flip only when we cross 150° tilt
    return body_up_z < _IMU_FLIP_COS


def imu_pitch_roll_deg(quat):
    """(pitch_deg, roll_deg) from quaternion (x,y,z,w) using the same accel-only
    convention as the firmware: pitch = atan2(ay, sqrt(ax^2+az^2)),
    roll = atan2(-ax, az), where (ax,ay,az) is the body's local gravity vector
    (== the world -z rotated INTO the body frame). With gravity pointing -z
    in the world, the body-frame gravity is the third row of R^T == third
    column of R, negated."""
    x, y, z, w = quat
    ax = 2.0 * (x * z - w * y)
    ay = 2.0 * (y * z + w * x)
    az = 1.0 - 2.0 * (x * x + y * y)
    pitch = math.atan2(ay, math.sqrt(ax * ax + az * az)) * 180.0 / math.pi
    roll = math.atan2(-ax, az) * 180.0 / math.pi
    return pitch, roll


def update_imu_from_pybullet(fc, p, robot_id, prev_state):
    """Read PyBullet body orientation, hysteresise, push into firmware.

    Returns the new latched state so the caller can thread it across ticks.
    """
    _, quat = p.getBasePositionAndOrientation(robot_id)
    up_z = _body_up_z(quat)
    new_state = imu_hysteresis_step(up_z, prev_state)
    pitch, roll = imu_pitch_roll_deg(quat)
    fc.set_imu_upside_down(new_state)
    fc.set_imu_pitch_deg(pitch)
    fc.set_imu_roll_deg(roll)
    return new_state
