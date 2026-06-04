# === Plain Python — no Blender required ===
# Imported by both the Blender add-on and plain-Python tests.
"""servo_math.py — pure servo-angle conversion helpers (no bpy dependency).

Extracted from fh_clip_panel.py so host-side tests and scripts can import
these without Blender running.
"""

# Firmware LegId order: FR=0, FL=1, BR=2, BL=3
_LEGS = ("fr", "fl", "br", "bl")

# URDF link1 joint axis sign per leg: the rig's link1 driver writes the
# bone-local Z rotation = true CCW yaw delta * axis_sign. Export must cancel
# this sign so math-space comes out uniform across all four legs.
_LINK1_DELTA_SIGN = {"fr": -1, "fl": +1, "br": +1, "bl": -1}


def _link1_delta_to_absolute(angles, convention):
    """Convert link1 bone-local delta readings back to absolute math-space
    shoulder angles.  Mutates and returns ``angles`` ({bone_name: deg}).

    The rig's link1 driver encodes the DELTA from the neutral shoulder
    (servo 90) and the per-leg sign (_LINK1_DELTA_SIGN) flips fl so all four
    shoulders yaw the same servo direction.  Shoulders only — link2/link3 are
    already absolute.

    Degrees throughout (``_read_bone_angles`` and ``neutral_joint_deg`` are
    both degrees)."""
    neutral = convention["neutral_joint_deg"]
    for leg in _LEGS:
        key = f"{leg}_link1"
        if key in angles:
            angles[key] = neutral[leg][0] + _LINK1_DELTA_SIGN[leg] * angles[key]
    return angles


# NOTE: _CALIB_THIGH, _CALIB_KNEE, and _frame_to_servo USED to live here. They
# were moved to code/simulation/firmware_port/exporter_parity.py so this module
# stays math-space-only and the exporter no longer depends on hardware CALIB
# (the firmware applies translateToServo + CALIB on T:12 receipt). Importers
# that need the servo-space helpers (parity tests, Blender panel preview) pull
# them from firmware_port.exporter_parity now.


def _scale_from_neutral(row, convention):
    """One baked row -> {leg: [sh, th, kn]} math-space degrees with the
    2/3 scale-from-NEUTRAL applied (the SAME step-1 math as
    _frame_to_servo, but WITHOUT translateToServo / round). The clip
    header is math-space; the firmware applies translateToServo at
    runtime, so this must NOT pre-apply it (plan §0)."""
    neutral = convention["neutral_joint_deg"]
    scale = convention["scale"]
    out = {}
    for leg in _LEGS:
        n = neutral[leg]
        raw = [row[f"{leg}_link1"], row[f"{leg}_link2"], row[f"{leg}_link3"]]
        out[leg] = [n[j] + (raw[j] - n[j]) * scale for j in range(3)]
    return out
