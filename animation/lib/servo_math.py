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


# Per-servo zero-point calibration: servo angle when the joint is at math 0°
# (thigh/knee flat). Mirror of CALIB_* in code/firmware/src/shared/config.h.
# Hip is uncalibrated (stays 90). Keep in sync with firmware whenever CALIB
# values change (and re-export every clip so the bake-time servo angles match
# the runtime). Pre-calibration these were all 90.
_CALIB_THIGH = {"fr": 84, "fl": 87, "br": 103, "bl": 84}
_CALIB_KNEE = {"fr": 95, "fl": 82, "br": 80, "bl": 87}


def _frame_to_servo(row, convention, warn=True):
    """One baked row -> {leg: [hip, thigh, knee] ints} where the per-leg
    list index IS the firmware `servo_id` (0=hip, 1=thigh, 2=knee —
    matching LEG_SERVO_CHANNEL[leg_id][servo_id] in
    code/firmware/src/shared/config.h on origin/main).

    Applies (1) scale-from-NEUTRAL, (2) per-leg translateToServo with
    per-joint CALIB zero-points — byte-identical to the firmware
    translateToServo, locked by test_clip_parity — and (3) rounds to int.

    Wire shape consumed by to_js: {T:4, id:_LEG_ID[leg], servo_id:j,
    a:result[leg][j]}. The firmware does the PCA-channel mapping;
    convention.json's `channels` field is kept for reference but is
    NOT used on the wire."""
    neutral = convention["neutral_joint_deg"]
    scale = convention["scale"]
    out = {}
    for leg in _LEGS:
        n = neutral[leg]  # [shoulder, hip, knee] deg at hardware neutral
        raw = [row[f"{leg}_link1"], row[f"{leg}_link2"], row[f"{leg}_link3"]]
        # 1. compress movement toward N
        sh, th, kn = (n[j] + (raw[j] - n[j]) * scale for j in range(3))
        # 2. translateToServo: shoulder uses literal 90 (uncalibrated), thigh/knee
        #    use per-leg CALIB (the servo angle at math 0 / flat).
        ct, ck = _CALIB_THIGH[leg], _CALIB_KNEE[leg]
        if leg == "fl":
            # Change B: FL shoulder regularized to 90 + (sh - 135) so servo 90 = outward,
            # matching fr/bl/br. Byte-identical to firmware motion_math.cpp FL branch.
            servo = [90 + (sh - 135), ct + th, ck - kn]
        elif leg == "fr":
            servo = [90 + (sh - 45), ct - th, ck + kn]
        elif leg == "bl":
            servo = [90 + (sh + 135), ct - th, ck + kn]
        else:  # br
            # BR shoulder un-mirrored (2026-05-25): +sh = +servo like fr/fl/bl
            # (identical motor, yaw shaft on the same vertical axis). Kept
            # byte-identical to firmware translateToServo by test_clip_parity.
            servo = [90 + (sh + 45), ct + th, ck - kn]
        # 3. clamp to the servo range, surfacing authoring errors at export
        # time rather than relying on the JS / firmware clamp as the only
        # backstop (the FRAME_DELTA_WARN_DEG warning's range companion).
        for i, v in enumerate(servo):
            if v < 0 or v > 180:
                if warn:
                    bone_name = ["shoulder", "hip", "knee"][i]
                    print(
                        f"WARNING: {leg} {bone_name} servo {v:.1f} out of range "
                        f"[0-180] at frame {row.get('frame', '?')} — clamped"
                    )
                servo[i] = max(0, min(180, v))
        # 4. integer servo degrees
        out[leg] = [round(v) for v in servo]
    return out


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
