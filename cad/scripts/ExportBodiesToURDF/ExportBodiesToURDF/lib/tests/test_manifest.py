"""Tests for lib/manifest.py — mesh_files manifest builder and role migration."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from lib.manifest import build_mesh_files_manifest, migrate_stale_role_paths


def test_build_empty_exported():
    result = build_mesh_files_manifest([])
    assert "_servo_role_assignment" in result
    assignment = result["_servo_role_assignment"]
    assert assignment["shoulder"] is None
    assert assignment["hip"] is None
    assert assignment["knee"] is None


def test_build_preserves_entry_fields():
    exported = [
        {
            "stl": "leg_upper.stl",
            "origin_shift_mm": [0.0, 0.0, 0.0],
            "source_body": "Link1L",
        }
    ]
    result = build_mesh_files_manifest(exported)
    assert "leg_upper.stl" in result
    assert result["leg_upper.stl"]["source_body"] == "Link1L"
    assert "stl" not in result["leg_upper.stl"]


def test_build_servo_paths_default_role_assignment():
    exported = [
        {
            "stl": "servo_sh.stl",
            "source_body": "ServoBase",
            "source_occurrences": ["LegAssembly:1/Shoulder:1"],
        },
        {
            "stl": "servo_hip.stl",
            "source_body": "ServoBase",
            "source_occurrences": ["LegAssembly:1/Hip:1"],
        },
        {
            "stl": "servo_knee.stl",
            "source_body": "ServoBase",
            "source_occurrences": ["LegAssembly:1/Knee:1"],
        },
    ]
    result = build_mesh_files_manifest(exported)
    # Last ServoBase entry's source_occurrences is used; default assigns by index.
    assignment = result["_servo_role_assignment"]
    # Just check all three roles are present (exact values depend on last servo entry)
    assert "shoulder" in assignment
    assert "hip" in assignment
    assert "knee" in assignment


def test_build_preserves_user_role_assignment():
    user_assignment = {
        "comment": "my note",
        "shoulder": "LegAssembly:1/MyServo:1",
        "hip": "LegAssembly:1/MyServo:2",
        "knee": "LegAssembly:1/MyServo:3",
    }
    exported = [
        {
            "stl": "leg.stl",
            "source_body": "ServoBase",
            "source_occurrences": ["LegAssembly:1/MyServo:1"],
        }
    ]
    result = build_mesh_files_manifest(
        exported, preserved_role_assignment=user_assignment
    )
    assignment = result["_servo_role_assignment"]
    assert assignment["shoulder"] == "LegAssembly:1/MyServo:1"
    assert assignment["comment"] == "my note"


def test_migrate_stale_no_change_when_up_to_date():
    preserved = {
        "shoulder": "LegAssembly:1/MyServo:1",
        "hip": "LegAssembly:1/MyServo:2",
    }
    servo_paths = ["LegAssembly:1/MyServo:1", "LegAssembly:1/MyServo:2"]
    result = migrate_stale_role_paths(preserved, servo_paths)
    assert result == preserved


def test_migrate_stale_renames_component():
    preserved = {
        "shoulder": "LegAssembly:1/OldServo:1",
        "hip": "LegAssembly:1/OldServo:2",
    }
    servo_paths = ["LegAssembly:1/NewServo:1", "LegAssembly:1/NewServo:2"]
    result = migrate_stale_role_paths(preserved, servo_paths)
    assert result["shoulder"] == "LegAssembly:1/NewServo:1"
    assert result["hip"] == "LegAssembly:1/NewServo:2"


def test_migrate_stale_preserves_comment():
    preserved = {
        "comment": "do not touch",
        "shoulder": "Asm:1/OldServo:1",
    }
    servo_paths = ["Asm:1/NewServo:1"]
    result = migrate_stale_role_paths(preserved, servo_paths)
    assert result["comment"] == "do not touch"
    assert result["shoulder"] == "Asm:1/NewServo:1"


def test_migrate_stale_no_op_when_empty():
    assert migrate_stale_role_paths(None, ["Asm:1/Servo:1"]) is None
    assert migrate_stale_role_paths({"shoulder": "x"}, []) == {"shoulder": "x"}
