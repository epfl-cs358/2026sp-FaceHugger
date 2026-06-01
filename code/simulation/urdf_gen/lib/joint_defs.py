"""Joint definition building blocks for the FaceHugger URDF generator.

Extracted from generate_urdf.py to keep that file focused on URDF
serialisation and to make the joint-topology logic independently testable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Helpers (local copies to avoid circular imports with generate_urdf.py)
# ---------------------------------------------------------------------------


def _cad_axis_dir(cad: dict) -> list | None:
    """Return axis direction from a CAD joint dict, preferring new schema name."""
    return cad.get("axis_dir_world") or cad.get("axis_dir_local_unit")


def _sub(a: list, b: list) -> list:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


# ---------------------------------------------------------------------------
# Topology
# ---------------------------------------------------------------------------

_JOINT_TOPOLOGY = [
    {
        "cad_name": "Link1Revolute",
        "urdf_name": "link1",
        "parent": "base_link",
        "child": "link1",
    },
    {
        "cad_name": "Link2Revolute",
        "urdf_name": "link2",
        "parent": "link1",
        "child": "link2",
    },
    {
        "cad_name": "Link3Revolute",
        "urdf_name": "link3",
        "parent": "link2",
        "child": "link3",
    },
]


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class JointDef:
    urdf_name: str
    parent: str
    child: str
    cad_name: str
    axis_dir: list
    limits_deg: list  # [lo_deg, hi_deg] or [None, None]
    limits_rad: dict  # {min, max, rest, current}
    origin_local_mm: list | None = None
    offset_from_parent_mm: list | None = None


@dataclass
class JointBuildResult:
    joints: list  # list[JointDef]
    fl_rest_rad: float
    shoulder_lower_deg: float
    shoulder_upper_deg: float


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def build_joint_definitions(export: dict, config: dict) -> JointBuildResult:
    """Build joint definitions from the Fusion export + config.

    Returns a JointBuildResult with 3 JointDef objects plus the shoulder
    rest/limit values derived from FL's Fusion limits.
    """
    cad_joints_by_name = {j.get("name"): j for j in (export.get("joints") or [])}

    joints: list[JointDef] = []
    for jcfg in _JOINT_TOPOLOGY:
        cad_name = jcfg["cad_name"]
        cad = cad_joints_by_name.get(cad_name)
        if cad is None:
            raise ValueError(
                f"Joint {cad_name!r} not found in fusion_export.json's joints[] "
                f"(make sure it's in the JOINTS whitelist in ExportBodiesToURDF.py)."
            )
        axis_dir = _cad_axis_dir(cad)
        origin_mm = cad.get("axis_origin_local_mm")
        if axis_dir is None:
            raise ValueError(
                f"Joint {cad_name!r} is missing axis_dir_world or axis_dir_local_unit in the export."
            )
        lim = cad.get("limits_rad") or {}
        lim_min_rad = lim.get("min")
        lim_max_rad = lim.get("max")
        lim_rest_rad = lim.get("rest", 0.0) or 0.0
        lim_current_rad = lim.get("current", 0.0) or 0.0
        lim_min_deg = math.degrees(lim_min_rad) if lim_min_rad is not None else None
        lim_max_deg = math.degrees(lim_max_rad) if lim_max_rad is not None else None
        joints.append(
            JointDef(
                urdf_name=jcfg["urdf_name"],
                parent=jcfg["parent"],
                child=jcfg["child"],
                cad_name=cad_name,
                origin_local_mm=origin_mm,
                axis_dir=axis_dir,
                limits_deg=[lim_min_deg, lim_max_deg],
                limits_rad={
                    "min": lim_min_rad,
                    "max": lim_max_rad,
                    "rest": lim_rest_rad,
                    "current": lim_current_rad,
                },
            )
        )

    # The shoulder (Link1Revolute) rest value in the JSON defines FL's
    # mechanical zero. The other 3 corners are derived via
    # _shoulder_rest_for(leg_id, fl_rest_rad). The Fusion limits are also
    # FL-relative; URDF limits = Fusion limits shifted by FL's rest, which
    # by mirror symmetry produces the same shifted range for every leg.
    #
    # `shoulder_rest_source` (yaml) picks which JSON field to read:
    #   "rest"    -> limits_rad.rest    (configured mechanical zero, default)
    #   "current" -> limits_rad.current (live joint angle at export time)
    rest_source = config.get("shoulder_rest_source", "rest")
    if rest_source not in ("rest", "current"):
        raise ValueError(
            f"facehugger_config.yaml: shoulder_rest_source must be "
            f"'rest' or 'current' (got {rest_source!r})."
        )
    fl_rest_rad = joints[0].limits_rad[rest_source]
    print(
        f"[generate_urdf] FL shoulder rest source = {rest_source!r} "
        f"-> {math.degrees(fl_rest_rad):+.2f}°"
    )
    fl_min_rad = joints[0].limits_rad["min"]
    fl_max_rad = joints[0].limits_rad["max"]
    if fl_min_rad is None or fl_max_rad is None:
        raise ValueError(
            "Link1Revolute missing limits_rad.min/max in the export — "
            "needed to derive the URDF shoulder limit window."
        )
    shoulder_lower_deg = math.degrees(fl_min_rad - fl_rest_rad)
    shoulder_upper_deg = math.degrees(fl_max_rad - fl_rest_rad)

    # Offsets between successive joints, in LAL (leg-assembly-local) frame
    # (== URDF link frame since rpy_z_deg only acts at the shoulder joint).
    # For new-schema joints without axis_origin_local_mm, origins come from
    # construction points; default to [0,0,0] so the _sub() doesn't fail.
    for i in range(1, len(joints)):
        origin_cur = joints[i].origin_local_mm or [0.0, 0.0, 0.0]
        origin_prev = joints[i - 1].origin_local_mm or [0.0, 0.0, 0.0]
        joints[i].offset_from_parent_mm = _sub(origin_cur, origin_prev)

    return JointBuildResult(
        joints=joints,
        fl_rest_rad=fl_rest_rad,
        shoulder_lower_deg=shoulder_lower_deg,
        shoulder_upper_deg=shoulder_upper_deg,
    )


def flip_joint_for_r_side(
    axis_dir: list,
    lim_lo,
    lim_hi,
    offset: list | None = None,
) -> tuple:
    """Apply the R-pair flip: negate axis, negate-swap limits, negate x-offset if given.

    Returns (flipped_axis, flipped_lo, flipped_hi, flipped_offset).

    The returned axis and offset (when given) are new list objects; the
    originals are not mutated.
    """
    flipped_axis = [-a for a in axis_dir]

    if lim_lo is not None and lim_hi is not None:
        flipped_lo, flipped_hi = -lim_hi, -lim_lo
    else:
        flipped_lo, flipped_hi = lim_lo, lim_hi

    if offset is not None:
        flipped_offset = list(offset)
        flipped_offset[0] = -flipped_offset[0]
    else:
        flipped_offset = None

    return flipped_axis, flipped_lo, flipped_hi, flipped_offset
