"""Tests for lib/verify.py — assembly hierarchy checks and physics validation."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from lib.verify import (
    verify_against_assembly_hierarchy,
    verify_export_rules_have_physics,
)

# ── minimal valid hierarchy ───────────────────────────────────────────────────

_FS = {
    "name": "FlexibleSkeleton:1",
    "points": [
        {"name": "LegMountPointFL", "pos_world_mm": [0.0, 0.0, 0.0]},
        {"name": "LegMountPointFR", "pos_world_mm": [0.0, 0.0, 0.0]},
        {"name": "LegMountPointBR", "pos_world_mm": [0.0, 0.0, 0.0]},
        {"name": "LegMountPointBL", "pos_world_mm": [0.0, 0.0, 0.0]},
    ],
    "children": [],
}

_FHLA_CHILDREN = [
    {"name": "Link1L:1", "bodies": [{"name": "Link1L"}], "points": [], "children": []},
    {"name": "Link1R:1", "bodies": [{"name": "Link1R"}], "points": [], "children": []},
    {"name": "Link2L:1", "bodies": [{"name": "Link2"}], "points": [], "children": []},
    {"name": "Link3L:1", "bodies": [{"name": "Link3"}], "points": [], "children": []},
    {
        "name": "MotorMount:1",
        "bodies": [{"name": "LegMountL"}],
        "points": [{"name": "LegMountFixedPoint", "pos_world_mm": [0.0, 0.0, 0.0]}],
        "children": [],
    },
    {
        "name": "MotorMountR:1",
        "bodies": [{"name": "LegMountR"}],
        "points": [{"name": "LegMountFixedPoint", "pos_world_mm": [0.0, 0.0, 0.0]}],
        "children": [],
    },
    {
        "name": "LegBaseServoEnclosure:1",
        "bodies": [{"name": "ServoBase"}],
        "points": [],
        "children": [],
    },
    {
        "name": "LegBaseServoEnclosure:2",
        "bodies": [{"name": "ServoBase"}],
        "points": [],
        "children": [],
    },
    {
        "name": "LegBaseServoEnclosure:3",
        "bodies": [{"name": "ServoBase"}],
        "points": [],
        "children": [],
    },
]

_FHLA = {
    "name": "FaceHuggerLegAssembly:1",
    "points": [
        {"name": "BodyToLink1Point", "pos_world_mm": [0.0, 0.0, 0.0]},
        {"name": "Link1ToLink2Point", "pos_world_mm": [0.0, 0.0, 0.0]},
        {"name": "Link2ToLink3Point", "pos_world_mm": [0.0, 0.0, 0.0]},
    ],
    "children": _FHLA_CHILDREN,
}

_TREE = [_FS, _FHLA]

_JOINTS = [
    {"name": "Link1Revolute"},
    {"name": "Link2Revolute"},
    {"name": "Link3Revolute"},
]


def _check_marks(rows):
    return [("✓" in r, r) for r in rows]


def test_verify_all_ok():
    rows = verify_against_assembly_hierarchy(_TREE, _JOINTS, {})
    marks = _check_marks(rows)
    failures = [r for ok, r in marks if not ok and r.strip().startswith("✗")]
    assert not failures, f"Unexpected failures: {failures}"


def test_verify_missing_fs_point():
    fs_bad = {
        "name": "FlexibleSkeleton:1",
        "points": [  # missing LegMountPointBR / BL
            {"name": "LegMountPointFL", "pos_world_mm": [0.0, 0.0, 0.0]},
            {"name": "LegMountPointFR", "pos_world_mm": [0.0, 0.0, 0.0]},
        ],
        "children": [],
    }
    rows = verify_against_assembly_hierarchy([fs_bad, _FHLA], _JOINTS, {})
    assert any("✗" in r and "FlexibleSkeleton" in r for r in rows)


def test_verify_missing_fs_entirely():
    rows = verify_against_assembly_hierarchy([_FHLA], _JOINTS, {})
    assert any("✗" in r and "FlexibleSkeleton" in r for r in rows)


def test_verify_missing_fhla_point():
    fhla_bad = {
        "name": "FaceHuggerLegAssembly:1",
        "points": [
            {"name": "BodyToLink1Point", "pos_world_mm": [0.0, 0.0, 0.0]},
            # missing Link1ToLink2Point and Link2ToLink3Point
        ],
        "children": _FHLA_CHILDREN,
    }
    rows = verify_against_assembly_hierarchy([_FS, fhla_bad], _JOINTS, {})
    assert any("✗" in r and "FaceHuggerLegAssembly" in r for r in rows)


def test_verify_missing_joint():
    rows = verify_against_assembly_hierarchy(_TREE, [{"name": "Link1Revolute"}], {})
    assert any("✗" in r and "Joints" in r for r in rows)


def test_verify_returns_list_of_strings():
    rows = verify_against_assembly_hierarchy(_TREE, _JOINTS, {})
    assert isinstance(rows, list)
    assert all(isinstance(r, str) for r in rows)


def test_physics_ok_when_all_present():
    occurrences = [
        {
            "name": "FaceHuggerLegAssembly:1",
            "physics": {"mass_kg": 0.1},
            "children": [],
        }
    ]
    mesh_files = {"link1.stl": {"source_occurrences": ["FaceHuggerLegAssembly:1"]}}
    warnings = verify_export_rules_have_physics(mesh_files, occurrences)
    assert warnings == []


def test_physics_warning_when_missing():
    occurrences = [
        {
            "name": "FaceHuggerLegAssembly:1",
            "children": [],  # no "physics" key
        }
    ]
    mesh_files = {"link1.stl": {"source_occurrences": ["FaceHuggerLegAssembly:1"]}}
    warnings = verify_export_rules_have_physics(mesh_files, occurrences)
    assert len(warnings) == 1
    assert "FaceHuggerLegAssembly:1" in warnings[0]


def test_physics_warning_when_error():
    occurrences = [
        {
            "name": "Comp:1",
            "physics": {"error": "physical body not found"},
            "children": [],
        }
    ]
    mesh_files = {"comp.stl": {"source_occurrences": ["Comp:1"]}}
    warnings = verify_export_rules_have_physics(mesh_files, occurrences)
    assert len(warnings) == 1
    assert "error" in warnings[0]


def test_physics_empty_mesh_files():
    warnings = verify_export_rules_have_physics({}, [])
    assert warnings == []


def test_physics_deduplicates_paths():
    occurrences = [
        {"name": "Comp:1", "children": []}  # no physics
    ]
    mesh_files = {
        "a.stl": {"source_occurrences": ["Comp:1"]},
        "b.stl": {"source_occurrences": ["Comp:1"]},  # same path, deduplicated
    }
    warnings = verify_export_rules_have_physics(mesh_files, occurrences)
    assert len(warnings) == 1  # reported once
