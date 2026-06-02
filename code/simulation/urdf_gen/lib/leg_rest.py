"""Per-leg shoulder rest + back-of-pair flip derivation.

The Fusion export only carries the FL leg's mechanical zero
(`Link1Revolute.limits_rad.rest`). The other three rests are computed
here via mirror+rotate symmetry. See `wiki/reference/firmware/kinematics.md`
for the convention's "why".
"""

import math

from .urdf_math import _wrap_pi


def _shoulder_rest_for(leg_id: str, fl_rest_rad: float) -> float:
    """Derive each leg's shoulder rest (rad) from the FL Fusion-export rest
    value via mirror+rotate symmetry. With the current CAD's
    `fl_rest_rad = -π/4`, this gives all four legs splayed outward in their
    respective body quadrants.

        FL =  fl_rest_rad                       (=  -45° from current CAD)
        FR = -fl_rest_rad                       (=  +45°; mirror across body X-axis)
        BL = -wrap_pi(fl_rest_rad + pi)         (= -135°)
        BR =  wrap_pi(fl_rest_rad + pi)         (= +135°)

    Earlier versions of this function had BL and BR swapped (signs
    flipped) — that sent the back legs splaying INTO the front quadrants.
    """
    if leg_id == "fl":
        return fl_rest_rad
    if leg_id == "fr":
        return -fl_rest_rad
    if leg_id == "bl":
        return -_wrap_pi(fl_rest_rad + math.pi)
    if leg_id == "br":
        return _wrap_pi(fl_rest_rad + math.pi)
    raise ValueError(f"_shoulder_rest_for: unknown leg_id {leg_id!r}")


def _back_of_pair_rpy_z_deg(leg_id: str) -> float:
    """Geometric back-of-pair flip: front legs (fl/fr) sit at 0°, back
    legs (bl/br) at 180°. This is the Rz that positions the shoulder
    pivot and back-bracket mesh — *separate* from the kinematic shoulder
    rest, which goes into the URDF joint <origin rpy>."""
    return 180.0 if leg_id.startswith("b") else 0.0
