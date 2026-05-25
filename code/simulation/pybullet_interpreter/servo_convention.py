# code/simulation/pybullet_interpreter/servo_convention.py
"""Firmware servo convention ported to Python.

All functions mirror the firmware exactly. Use these as the single
source of truth for the math-space → servo-space → PyBullet-radian
pipeline in Python.

Firmware sources:
  translateToServo : code/firmware/src/nervous_system/motion_math.cpp:32
  clampClipServos  : code/firmware/src/nervous_system/motion_math.cpp:10
  NEUTRAL[]        : code/firmware/src/nervous_system/neutral_pose.h:13
"""

import math
from dataclasses import dataclass

# Firmware LegId enum values — code/firmware/src/nervous_system/movements.h
LEG_FR = 0
LEG_FL = 1
LEG_RR = 2  # also called BR in simulation / Blender
LEG_RL = 3  # also called BL in simulation / Blender

# Map firmware LegId (0-3) to URDF/simulation leg name (fl/fr/bl/br).
# Matches helpers.py build_joint_map naming convention.
LEG_ID_TO_SIM_NAME: dict[int, str] = {
    LEG_FR: "fr",
    LEG_FL: "fl",
    LEG_RR: "br",
    LEG_RL: "bl",
}


@dataclass
class NeutralPose:
    """Math-space standing pose for one leg, in degrees.

    Source: neutral_pose.h:12 — struct NeutralPose { float sh, th, kn; }
    """

    sh: float  # shoulder (math-space degrees)
    th: float  # thigh    (math-space degrees)
    kn: float  # knee     (math-space degrees)


# Standing pose per leg, indexed by firmware LegId.
# Source: neutral_pose.h:13 — constexpr NeutralPose NEUTRAL[LEG_COUNT]
# Change B (FL sh=135): outward direction now matches FR/BR/BL convention.
NEUTRAL: list[NeutralPose] = [
    NeutralPose(sh=45.0, th=-60.0, kn=-37.0),  # LEG_FR (0)
    NeutralPose(sh=135.0, th=-60.0, kn=-40.0),  # LEG_FL (1) Change B
    NeutralPose(sh=-45.0, th=-50.0, kn=-50.0),  # LEG_RR / BR (2)
    NeutralPose(sh=-135.0, th=-60.0, kn=-35.0),  # LEG_RL / BL (3)
]


@dataclass
class ServoTriple:
    """Servo angles for one leg, in degrees [0, 180].

    Output of translate_to_servo(). Mirrors firmware ServoTriple struct
    from code/firmware/src/nervous_system/motion_math.h.

    Note: 'hip' in firmware naming = shoulder joint servo angle.
    """

    hip: float  # shoulder servo angle, 0-180°
    thigh: float  # thigh servo angle, 0-180°
    knee: float  # knee servo angle, 0-180°


def translate_to_servo(leg_id: int, sh: float, th: float, kn: float) -> ServoTriple:
    """Convert math-space joint angles to servo degrees for one leg.

    Input:  math-space degrees, absolute (not deltas, not radians).
            sh = shoulder, th = thigh, kn = knee.
    Output: ServoTriple with hip/thigh/knee servo angles in [0, 180]°.

    Mirrors firmware translateToServo() in
    code/firmware/src/nervous_system/motion_math.cpp:32-59 exactly.
    The per-leg formulas encode each servo's physical mounting direction
    and zero-offset so that servo 90 = the joint's math-space neutral.

    Raises ValueError for unknown leg_id.
    """
    out = ServoTriple(hip=90.0, thigh=90.0, knee=90.0)
    if leg_id == LEG_FR:
        out.hip = 90.0 + (sh - 45.0)
        out.thigh = 90.0 - th
        out.knee = 90.0 + kn
    elif leg_id == LEG_FL:
        out.hip = 90.0 + (sh - 135.0)
        out.thigh = 90.0 + th
        out.knee = 90.0 - kn
    elif leg_id == LEG_RR:
        out.hip = 90.0 - (sh + 45.0)
        out.thigh = 90.0 + th
        out.knee = 90.0 - kn
    elif leg_id == LEG_RL:
        out.hip = 90.0 + (sh + 135.0)
        out.thigh = 90.0 - th
        out.knee = 90.0 + kn
    else:
        raise ValueError(f"unknown leg_id {leg_id!r}")
    return out


def clamp_clip_servos(s: ServoTriple) -> ServoTriple:
    """Clamp servo angles to safe hardware ranges for clip playback.

    Mirrors firmware clampClipServos() in
    code/firmware/src/nervous_system/motion_math.cpp:10-18.

    Ranges:
      hip (shoulder): [38, 142]  — 90 ± 52 (tightest side of each leg)
      thigh:          [30, 150]  — 90 ± 60
      knee:           [0,  180]  — full travel
    """

    def cl(v: float, lo: float, hi: float) -> float:
        return lo if v < lo else (hi if v > hi else v)

    return ServoTriple(
        hip=cl(s.hip, 38.0, 142.0),
        thigh=cl(s.thigh, 30.0, 150.0),
        knee=cl(s.knee, 0.0, 180.0),
    )


def servo_to_radians(servo_deg: float) -> float:
    """Convert a servo angle in degrees to a PyBullet joint angle in radians.

    Convention:
      servo 90° → 0 rad   (joint at neutral / mid-travel)
      servo  0° → -π/2 rad
      servo 180° → +π/2 rad

    Formula: math.radians(servo_deg - 90.0)

    This is the correct formula because translateToServo emits
    90 ± delta, and the URDF joint zero = servo 90 neutral pose.
    """
    return math.radians(servo_deg - 90.0)
