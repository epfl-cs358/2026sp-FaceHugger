"""Tests for urdf_gen.lib.construction_points.resolve_leg_construction_points.

All behaviors are tested against a minimal fixture occurrence tree built
inline — no JSON file I/O, no Fusion dependency.
"""

import sys
from pathlib import Path

import pytest

# Ensure urdf_gen is importable (code/simulation is the package root)
_SIM_ROOT = Path(__file__).parents[1]
if str(_SIM_ROOT) not in sys.path:
    sys.path.insert(0, str(_SIM_ROOT))

from urdf_gen.lib.construction_points import resolve_leg_construction_points


# ---------------------------------------------------------------------------
# Minimal fixture builder helpers
# ---------------------------------------------------------------------------


def _point(name: str, pos: list) -> dict:
    return {"name": name, "pos_world_mm": pos}


def _occ(name: str, points: list = (), children: list = ()) -> dict:
    return {"name": name, "points": list(points), "children": list(children)}


def _make_full_tree() -> list:
    """Build a minimal occurrence tree with all 8 construction points present."""
    # Inner-most: MotorMount:1 → LegBaseServoEnclosure:1
    shoulder_servo_L_enc = _occ(
        "LegBaseServoEnclosure:1",
        points=[_point("ServoMountPoint", [10.0, 20.0, 30.0])],
    )

    # MotorMount:1 (L bracket)
    motor_mount_L = _occ(
        "MotorMount:1",
        points=[_point("LegMountFixedPoint", [1.0, 2.0, 3.0])],
        children=[shoulder_servo_L_enc],
    )

    # MotorMountR:1 → Servo_Mouser_Model(Mirror):1
    shoulder_servo_R_enc = _occ(
        "Servo_Mouser_Model(Mirror):1",
        points=[_point("ServoMountPoint", [11.0, 21.0, 31.0])],
    )
    motor_mount_R = _occ(
        "MotorMountR:1",
        points=[_point("LegMountFixedPoint", [4.0, 5.0, 6.0])],
        children=[shoulder_servo_R_enc],
    )

    # Top-level leg enclosures (hip + knee)
    hip_enc = _occ(
        "LegBaseServoEnclosure:2",
        points=[_point("ServoMountPoint", [12.0, 22.0, 32.0])],
    )
    knee_enc = _occ(
        "LegBaseServoEnclosure:3",
        points=[_point("ServoMountPoint", [13.0, 23.0, 33.0])],
    )

    # FaceHuggerLegAssembly:1 — top node
    leg_assembly = _occ(
        "FaceHuggerLegAssembly:1",
        points=[
            _point("BodyToLink1Point", [7.0, 8.0, 9.0]),
            _point("Link2ToLink3Point", [40.0, 50.0, 60.0]),
        ],
        children=[motor_mount_L, motor_mount_R, hip_enc, knee_enc],
    )

    return [leg_assembly]


# ---------------------------------------------------------------------------
# Helpers expected from the resolved dict
# ---------------------------------------------------------------------------

BODY_TO_LINK1 = [7.0, 8.0, 9.0]
LINK2_TO_LINK3 = [40.0, 50.0, 60.0]
MOUNT_L = [1.0, 2.0, 3.0]
MOUNT_R = [4.0, 5.0, 6.0]
SHOULDER_SERVO_L = [10.0, 20.0, 30.0]
SHOULDER_SERVO_R = [11.0, 21.0, 31.0]
HIP_SERVO = [12.0, 22.0, 32.0]
KNEE_SERVO = [13.0, 23.0, 33.0]


def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


L_AXIS_OFFSET = _sub(BODY_TO_LINK1, MOUNT_L)
R_AXIS_OFFSET = _sub(BODY_TO_LINK1, MOUNT_R)


# ---------------------------------------------------------------------------
# Tests: correct values from a fully-populated tree
# ---------------------------------------------------------------------------


@pytest.fixture()
def full_cpts():
    return resolve_leg_construction_points(_make_full_tree())


def test_body_to_link1_resolved(full_cpts):
    assert full_cpts["body_to_link1"] == BODY_TO_LINK1


def test_link2_to_link3_resolved(full_cpts):
    assert full_cpts["link2_to_link3"] == LINK2_TO_LINK3


def test_mount_L_resolved(full_cpts):
    assert full_cpts["mount_L"] == MOUNT_L


def test_mount_R_resolved(full_cpts):
    assert full_cpts["mount_R"] == MOUNT_R


def test_L_axis_offset(full_cpts):
    assert full_cpts["L_axis_offset"] == L_AXIS_OFFSET


def test_R_axis_offset(full_cpts):
    assert full_cpts["R_axis_offset"] == R_AXIS_OFFSET


def test_shoulder_servo_L_resolved(full_cpts):
    assert full_cpts["shoulder_servo_L"] == SHOULDER_SERVO_L


def test_shoulder_servo_R_resolved(full_cpts):
    assert full_cpts["shoulder_servo_R"] == SHOULDER_SERVO_R


def test_hip_servo_resolved(full_cpts):
    assert full_cpts["hip_servo"] == HIP_SERVO


def test_knee_servo_resolved(full_cpts):
    assert full_cpts["knee_servo"] == KNEE_SERVO


# ---------------------------------------------------------------------------
# Tests: optional servo points return None when absent
# ---------------------------------------------------------------------------


def _make_tree_without_optional_servos() -> list:
    """Tree with required points but missing all optional servo mount points."""
    motor_mount_L = _occ(
        "MotorMount:1",
        points=[_point("LegMountFixedPoint", [1.0, 2.0, 3.0])],
        # no child LegBaseServoEnclosure:1
    )
    motor_mount_R = _occ(
        "MotorMountR:1",
        points=[_point("LegMountFixedPoint", [4.0, 5.0, 6.0])],
        # no child Servo_Mouser_Model(Mirror):1
    )
    leg_assembly = _occ(
        "FaceHuggerLegAssembly:1",
        points=[
            _point("BodyToLink1Point", [7.0, 8.0, 9.0]),
            _point("Link2ToLink3Point", [40.0, 50.0, 60.0]),
        ],
        children=[motor_mount_L, motor_mount_R],
        # no LegBaseServoEnclosure:2 or :3
    )
    return [leg_assembly]


def test_optional_servo_points_none_when_absent():
    cpts = resolve_leg_construction_points(_make_tree_without_optional_servos())
    # shoulder_servo_L also absent in this tree
    assert cpts["shoulder_servo_L"] is None
    assert cpts["shoulder_servo_R"] is None
    assert cpts["hip_servo"] is None
    assert cpts["knee_servo"] is None


# ---------------------------------------------------------------------------
# Tests: ValueError raised for missing required points
# ---------------------------------------------------------------------------


def _make_tree_missing_body_to_link1() -> list:
    motor_mount_L = _occ(
        "MotorMount:1",
        points=[_point("LegMountFixedPoint", [1.0, 2.0, 3.0])],
    )
    motor_mount_R = _occ(
        "MotorMountR:1",
        points=[_point("LegMountFixedPoint", [4.0, 5.0, 6.0])],
    )
    leg_assembly = _occ(
        "FaceHuggerLegAssembly:1",
        points=[
            # BodyToLink1Point intentionally omitted
            _point("Link2ToLink3Point", [40.0, 50.0, 60.0]),
        ],
        children=[motor_mount_L, motor_mount_R],
    )
    return [leg_assembly]


def _make_tree_missing_mount_L() -> list:
    motor_mount_L = _occ(
        "MotorMount:1",
        points=[],  # LegMountFixedPoint intentionally omitted
    )
    motor_mount_R = _occ(
        "MotorMountR:1",
        points=[_point("LegMountFixedPoint", [4.0, 5.0, 6.0])],
    )
    leg_assembly = _occ(
        "FaceHuggerLegAssembly:1",
        points=[
            _point("BodyToLink1Point", [7.0, 8.0, 9.0]),
            _point("Link2ToLink3Point", [40.0, 50.0, 60.0]),
        ],
        children=[motor_mount_L, motor_mount_R],
    )
    return [leg_assembly]


def test_raises_value_error_if_body_to_link1_missing():
    with pytest.raises(ValueError, match="BodyToLink1Point"):
        resolve_leg_construction_points(_make_tree_missing_body_to_link1())


def test_raises_value_error_if_mount_L_missing():
    with pytest.raises(ValueError, match="LegMountFixedPoint"):
        resolve_leg_construction_points(_make_tree_missing_mount_L())


# ---------------------------------------------------------------------------
# Tests: additional missing-point error cases and scoping
# ---------------------------------------------------------------------------


def _make_tree_missing_link2_to_link3() -> list:
    motor_mount_L = _occ(
        "MotorMount:1",
        points=[_point("LegMountFixedPoint", [1.0, 2.0, 3.0])],
    )
    motor_mount_R = _occ(
        "MotorMountR:1",
        points=[_point("LegMountFixedPoint", [4.0, 5.0, 6.0])],
    )
    leg_assembly = _occ(
        "FaceHuggerLegAssembly:1",
        points=[
            _point("BodyToLink1Point", [7.0, 8.0, 9.0]),
            # Link2ToLink3Point intentionally omitted
        ],
        children=[motor_mount_L, motor_mount_R],
    )
    return [leg_assembly]


def _make_tree_missing_mount_R() -> list:
    motor_mount_L = _occ(
        "MotorMount:1",
        points=[_point("LegMountFixedPoint", [1.0, 2.0, 3.0])],
    )
    motor_mount_R = _occ(
        "MotorMountR:1",
        points=[],  # LegMountFixedPoint intentionally omitted
    )
    leg_assembly = _occ(
        "FaceHuggerLegAssembly:1",
        points=[
            _point("BodyToLink1Point", [7.0, 8.0, 9.0]),
            _point("Link2ToLink3Point", [40.0, 50.0, 60.0]),
        ],
        children=[motor_mount_L, motor_mount_R],
    )
    return [leg_assembly]


def _make_tree_missing_body_to_link1_and_mount_L() -> list:
    motor_mount_L = _occ(
        "MotorMount:1",
        points=[],  # LegMountFixedPoint intentionally omitted
    )
    motor_mount_R = _occ(
        "MotorMountR:1",
        points=[_point("LegMountFixedPoint", [4.0, 5.0, 6.0])],
    )
    leg_assembly = _occ(
        "FaceHuggerLegAssembly:1",
        points=[
            # BodyToLink1Point intentionally omitted
            _point("Link2ToLink3Point", [40.0, 50.0, 60.0]),
        ],
        children=[motor_mount_L, motor_mount_R],
    )
    return [leg_assembly]


def test_raises_value_error_if_link2_to_link3_missing():
    with pytest.raises(ValueError, match="Link2ToLink3Point"):
        resolve_leg_construction_points(_make_tree_missing_link2_to_link3())


def test_raises_value_error_if_mount_R_missing():
    with pytest.raises(ValueError, match="LegMountFixedPoint"):
        resolve_leg_construction_points(_make_tree_missing_mount_R())


def test_raises_value_error_lists_all_missing_points():
    with pytest.raises(ValueError) as exc_info:
        resolve_leg_construction_points(_make_tree_missing_body_to_link1_and_mount_L())
    msg = str(exc_info.value)
    assert "BodyToLink1Point" in msg
    assert "LegMountFixedPoint" in msg


def test_mount_L_scoped_to_motor_mount_not_top_level():
    """A LegMountFixedPoint directly on FaceHuggerLegAssembly:1 is ignored; only MotorMount:1's counts."""
    motor_mount_L = _occ(
        "MotorMount:1",
        points=[_point("LegMountFixedPoint", [1.0, 2.0, 3.0])],
    )
    motor_mount_R = _occ(
        "MotorMountR:1",
        points=[_point("LegMountFixedPoint", [4.0, 5.0, 6.0])],
    )
    leg_assembly = _occ(
        "FaceHuggerLegAssembly:1",
        points=[
            _point("BodyToLink1Point", [7.0, 8.0, 9.0]),
            _point("Link2ToLink3Point", [40.0, 50.0, 60.0]),
            # Shallow decoy — should NOT be picked up as mount_L
            _point("LegMountFixedPoint", [99.0, 99.0, 99.0]),
        ],
        children=[motor_mount_L, motor_mount_R],
    )
    cpts = resolve_leg_construction_points([leg_assembly])
    assert cpts["mount_L"] == [1.0, 2.0, 3.0]
