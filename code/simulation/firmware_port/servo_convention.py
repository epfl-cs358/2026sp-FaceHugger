# code/simulation/firmware_port/servo_convention.py
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
# Matches pybullet_sim.motor.build_joint_map naming convention.
LEG_ID_TO_SIM_NAME: dict[int, str] = {
    LEG_FR: "fr",
    LEG_FL: "fl",
    LEG_RR: "br",
    LEG_RL: "bl",
}

# URDF axis direction sign for link2 (hip/thigh) and link3 (knee) joints.
# Derived from generated/facehugger.urdf <axis xyz="0 ±1 0">.
# Both link2 and link3 share the same sign per leg.
# Diagonal pair (FL, BR) = +Y (+1); diagonal pair (FR, BL) = -Y (-1).
# Used in clip_player.frame_to_joint_targets only — the firmware drives
# physical servo motors and is unaffected by URDF axis orientation.
LEG_ID_TO_URDF_AXIS_SIGN: dict[int, int] = {
    LEG_FR: -1,  # fr link2/3: axis "0 -1 0"
    LEG_FL: +1,  # fl link2/3: axis "0  1 0"
    LEG_RR: +1,  # br link2/3: axis "0  1 0"
    LEG_RL: -1,  # bl link2/3: axis "0 -1 0"
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

# Per-servo zero-point calibration: servo angle when the joint is at mechanical
# zero (math 0° = thigh/knee horizontal). Mirror of CALIB_* macros in
# code/firmware/src/shared/config.h. Hip is uncalibrated (stays at 90).
# Keep in sync with the firmware whenever CALIB values change.
CALIB_THIGH: dict[int, int] = {
    LEG_FR: 84,
    LEG_FL: 87,
    LEG_RR: 103,  # BR
    LEG_RL: 84,  # BL
}
CALIB_KNEE: dict[int, int] = {
    LEG_FR: 95,
    LEG_FL: 82,
    LEG_RR: 80,  # BR
    LEG_RL: 87,  # BL
}


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
        out.thigh = CALIB_THIGH[LEG_FR] - th
        out.knee = CALIB_KNEE[LEG_FR] + kn
    elif leg_id == LEG_FL:
        out.hip = 90.0 + (sh - 135.0)
        out.thigh = CALIB_THIGH[LEG_FL] + th
        out.knee = CALIB_KNEE[LEG_FL] - kn
    elif leg_id == LEG_RR:
        # BR shoulder un-mirrored (2026-05-25): +sh = +servo like fr/fl/bl
        # (identical motor, yaw shaft on the same vertical axis). Byte-identical
        # to firmware translateToServo / exporter _frame_to_servo.
        out.hip = 90.0 + (sh + 45.0)
        out.thigh = CALIB_THIGH[LEG_RR] + th
        out.knee = CALIB_KNEE[LEG_RR] - kn
    elif leg_id == LEG_RL:
        out.hip = 90.0 + (sh + 135.0)
        out.thigh = CALIB_THIGH[LEG_RL] - th
        out.knee = CALIB_KNEE[LEG_RL] + kn
    else:
        raise ValueError(f"unknown leg_id {leg_id!r}")
    return out


# Clip-playback servo-clamp half-widths, derived from the URDF joint limits.
# Mirror of HIP_CLAMP_FROM_NINETY / THIGH_CLAMP_FROM_CALIB / KNEE_CLAMP_FROM_CALIB
# in code/firmware/src/shared/config.h — keep in sync.
HIP_CLAMP_FROM_NINETY = 52
THIGH_CLAMP_FROM_CALIB = 75
KNEE_CLAMP_FROM_CALIB = 90


def clamp_clip_servos(leg_id: int, s: ServoTriple) -> ServoTriple:
    """Clamp servo angles for one leg to its safe envelope on the clip path.

    Per-leg, CALIB-relative — mirrors firmware clampClipServos() in
    code/firmware/src/nervous_system/motion_math.cpp.

    Envelope (derived from URDF joint limits):
      hip   uniform 90 ± HIP_CLAMP_FROM_NINETY  (every leg's tight shoulder side)
      thigh CALIB_THIGH[leg_id] ± THIGH_CLAMP_FROM_CALIB  (URDF ±75° symmetric)
      knee  CALIB_KNEE[leg_id]  ± KNEE_CLAMP_FROM_CALIB   (URDF ±90° symmetric)

    Raises ValueError for unknown leg_id.
    """
    if leg_id not in CALIB_THIGH:
        raise ValueError(f"unknown leg_id {leg_id!r}")

    def cl(v: float, lo: float, hi: float) -> float:
        return lo if v < lo else (hi if v > hi else v)

    thigh_center = CALIB_THIGH[leg_id]
    knee_center = CALIB_KNEE[leg_id]
    return ServoTriple(
        hip=cl(s.hip, 90.0 - HIP_CLAMP_FROM_NINETY, 90.0 + HIP_CLAMP_FROM_NINETY),
        thigh=cl(
            s.thigh,
            thigh_center - THIGH_CLAMP_FROM_CALIB,
            thigh_center + THIGH_CLAMP_FROM_CALIB,
        ),
        knee=cl(
            s.knee,
            knee_center - KNEE_CLAMP_FROM_CALIB,
            knee_center + KNEE_CLAMP_FROM_CALIB,
        ),
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
