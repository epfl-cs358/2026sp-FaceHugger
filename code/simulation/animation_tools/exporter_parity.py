# code/simulation/animation_tools/exporter_parity.py
"""Firmware-parity helpers for the Blender exporter.

Hosts the helpers that USED to live in animation/lib/servo_math.py
(`_CALIB_THIGH`, `_CALIB_KNEE`, `_frame_to_servo`). They were moved out
of the exporter so it stays math-space-only (link1 delta → absolute +
scale-from-NEUTRAL); CALIB now lives exclusively on the firmware side.

Why they still exist at all
---------------------------
- `_frame_to_servo` is the byte-for-byte parity mirror of the firmware
  `translateToServo` + CALIB + clamp pipeline. It seeds the firmware's
  native parity test (`code/firmware/test/test_clip_parity`) via
  `code/firmware/test/gen_clip_parity_reference.py`, and powers the
  exporter parity test in `animation/scripts/test_servo_parity.py`.
- `_CALIB_THIGH`/`_CALIB_KNEE` are exposed (string-keyed by leg) so the
  Blender panel's "current servo angles" preview can show real hardware
  servo numbers (`fh_clip_panel.py::_current_servo_angles`).

CALIB source of truth
---------------------
This module reads CALIB exclusively from `servo_convention.py`'s
int-keyed dicts (`CALIB_THIGH`, `CALIB_KNEE`), which themselves mirror
`code/firmware/src/shared/calib.h`. Both edges of the chain are pinned
by `test_calib_consistency.py`.
"""

from . import servo_convention as _sc

# Firmware LegId (int) → animation-side leg name (str).
_LEG_NAME = {
    _sc.LEG_FR: "fr",
    _sc.LEG_FL: "fl",
    _sc.LEG_RR: "br",  # firmware "RR" == animation "br"
    _sc.LEG_RL: "bl",  # firmware "RL" == animation "bl"
}

# String-keyed CALIB mirror (animation-side convention). Derived live from the
# int-keyed firmware-parity dicts — no separate hardcoded copy.
_CALIB_THIGH = {name: _sc.CALIB_THIGH[lid] for lid, name in _LEG_NAME.items()}
_CALIB_KNEE = {name: _sc.CALIB_KNEE[lid] for lid, name in _LEG_NAME.items()}

_LEGS = ("fr", "fl", "br", "bl")


def _frame_to_servo(row, convention, warn=True):
    """One baked row -> {leg: [hip, thigh, knee] ints} where the per-leg list
    index is the firmware `servo_id` (0=hip, 1=thigh, 2=knee).

    Applies (1) scale-from-NEUTRAL, (2) per-leg translateToServo with per-joint
    CALIB zero-points — byte-identical to the firmware translateToServo — and
    (3) rounds to int. Used by the firmware parity test and by the Blender
    panel's "current servo angles" preview; NOT by any on-wire path (which is
    now T:12 math-space, see fh_clip_panel.to_clips_extra_json /
    spinal_cord.cpp::streamMathFrame).
    """
    neutral = convention["neutral_joint_deg"]
    scale = convention["scale"]
    out = {}
    for leg in _LEGS:
        n = neutral[leg]
        raw = [row[f"{leg}_link1"], row[f"{leg}_link2"], row[f"{leg}_link3"]]
        # 1. scale-from-NEUTRAL
        sh, th, kn = (n[j] + (raw[j] - n[j]) * scale for j in range(3))
        # 2. per-leg translateToServo (kept byte-identical to motion_math.cpp)
        ct, ck = _CALIB_THIGH[leg], _CALIB_KNEE[leg]
        if leg == "fl":
            servo = [90 + (sh - 135), ct + th, ck - kn]
        elif leg == "fr":
            servo = [90 + (sh - 45), ct - th, ck + kn]
        elif leg == "bl":
            servo = [90 + (sh + 135), ct - th, ck + kn]
        else:  # br
            # BR shoulder un-mirrored (2026-05-25): +sh = +servo like the others.
            servo = [90 + (sh + 45), ct + th, ck - kn]
        # 3. clamp + (optional) warn
        for i, v in enumerate(servo):
            if v < 0 or v > 180:
                if warn:
                    bone_name = ["shoulder", "hip", "knee"][i]
                    print(
                        f"WARNING: {leg} {bone_name} servo {v:.1f} out of range "
                        f"[0-180] at frame {row.get('frame', '?')} — clamped"
                    )
                servo[i] = max(0, min(180, v))
        out[leg] = [round(v) for v in servo]
    return out
