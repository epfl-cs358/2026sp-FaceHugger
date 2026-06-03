"""construction_points.py — leg-assembly construction-point resolution.

Extracts the 8 world-position construction points from the Fusion export
occurrence tree that are needed to compute URDF joint origins for the
FaceHugger leg assembly, plus the 2 offset vectors (L_axis_offset, R_axis_offset) derived from them.

This module is self-contained (no imports from generate_urdf.py) to avoid
circular-import issues when generate_urdf.py imports from here.
"""

LEG_ASSEMBLY = "FaceHuggerLegAssembly:1"


def _find_point_world_at_occurrence(
    nodes: list, occ_path: str, key: str
) -> list | None:
    """Look up a construction point's pos_world_mm scoped to a specific
    occurrence path (e.g. 'FaceHuggerLegAssembly:1/MotorMount:1').
    """
    parts = occ_path.split("/")
    cur = nodes
    node = None
    for part in parts:
        node = next((c for c in (cur or []) if c.get("name") == part), None)
        if node is None:
            return None
        cur = node.get("children", [])
    if node is None:
        return None
    for p in node.get("points", []):
        if p.get("name") == key:
            return p.get("pos_world_mm")
    return None


def _sub(a: list, b: list) -> list:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def resolve_leg_construction_points(occurrences: list) -> dict:
    """Resolve all 8 leg-assembly construction points from the Fusion export
    occurrence tree, plus the 2 offset vectors (L_axis_offset, R_axis_offset) derived from them.

    Returns a dict with keys:
        body_to_link1    — [x,y,z] world pos of BodyToLink1Point
        link2_to_link3   — [x,y,z] world pos of Link2ToLink3Point
        mount_L          — [x,y,z] world pos of MotorMount:1 LegMountFixedPoint
        mount_R          — [x,y,z] world pos of MotorMountR:1 LegMountFixedPoint
        shoulder_servo_L — [x,y,z] world pos of shoulder servo mount (L), or None
        shoulder_servo_R — [x,y,z] world pos of shoulder servo mount (R), or None
        hip_servo        — [x,y,z] world pos of hip servo mount, or None
        knee_servo       — [x,y,z] world pos of knee servo mount, or None
        L_axis_offset    — sub(body_to_link1, mount_L)
        R_axis_offset    — sub(body_to_link1, mount_R)

    Raises ValueError if body_to_link1, link2_to_link3, mount_L, or mount_R
    cannot be resolved (these are required; servo points are optional).
    """
    body_to_link1 = _find_point_world_at_occurrence(
        occurrences, LEG_ASSEMBLY, "BodyToLink1Point"
    )
    link2_to_link3 = _find_point_world_at_occurrence(
        occurrences, LEG_ASSEMBLY, "Link2ToLink3Point"
    )
    mount_L = _find_point_world_at_occurrence(
        occurrences, f"{LEG_ASSEMBLY}/MotorMount:1", "LegMountFixedPoint"
    )
    mount_R = _find_point_world_at_occurrence(
        occurrences, f"{LEG_ASSEMBLY}/MotorMountR:1", "LegMountFixedPoint"
    )

    missing = []
    if body_to_link1 is None:
        missing.append("BodyToLink1Point")
    if link2_to_link3 is None:
        missing.append("Link2ToLink3Point")
    if mount_L is None:
        missing.append("LegMountFixedPoint (MotorMount:1)")
    if mount_R is None:
        missing.append("LegMountFixedPoint (MotorMountR:1)")
    if missing:
        raise ValueError(f"Missing required construction points: {', '.join(missing)}.")

    shoulder_servo_L = _find_point_world_at_occurrence(
        occurrences,
        f"{LEG_ASSEMBLY}/MotorMount:1/LegBaseServoEnclosure:1",
        "ServoMountPoint",
    )
    shoulder_servo_R = _find_point_world_at_occurrence(
        occurrences,
        f"{LEG_ASSEMBLY}/MotorMountR:1/Servo_Mouser_Model(Mirror):1",
        "ServoMountPoint",
    )
    hip_servo = _find_point_world_at_occurrence(
        occurrences,
        f"{LEG_ASSEMBLY}/LegBaseServoEnclosure:2",
        "ServoMountPoint",
    )
    knee_servo = _find_point_world_at_occurrence(
        occurrences,
        f"{LEG_ASSEMBLY}/LegBaseServoEnclosure:3",
        "ServoMountPoint",
    )

    return {
        "body_to_link1": body_to_link1,
        "link2_to_link3": link2_to_link3,
        "mount_L": mount_L,
        "mount_R": mount_R,
        "shoulder_servo_L": shoulder_servo_L,
        "shoulder_servo_R": shoulder_servo_R,
        "hip_servo": hip_servo,
        "knee_servo": knee_servo,
        "L_axis_offset": _sub(body_to_link1, mount_L),
        "R_axis_offset": _sub(body_to_link1, mount_R),
    }
