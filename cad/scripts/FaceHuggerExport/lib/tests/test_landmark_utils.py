"""Tests for lib/landmark_utils.py — occurrence-tree JSON point lookups."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lib.landmark_utils import (
    find_landmark_world_pos,
    find_landmark_world_pos_at_occurrence,
)

_TREE = [
    {
        "name": "FlexibleSkeleton:1",
        "points": [
            {"name": "LegMountPointFL", "pos_world_mm": [10.0, 20.0, 30.0]},
            {"name": "LegMountPointFR", "pos_world_mm": [-10.0, 20.0, 30.0]},
        ],
        "children": [
            {
                "name": "LegMount:1",
                "points": [
                    {"name": "BodyToLink1Point", "pos_world_mm": [5.0, 6.0, 7.0]}
                ],
                "children": [],
            }
        ],
    },
    {
        "name": "FaceHuggerLegAssembly:1",
        "points": [],
        "children": [
            {
                "name": "Link1:1",
                "points": [
                    {"name": "Link1ToLink2Point", "pos_world_mm": [1.0, 2.0, 3.0]}
                ],
                "children": [],
            }
        ],
    },
]


def test_find_landmark_top_level():
    pos = find_landmark_world_pos(_TREE, "LegMountPointFL")
    assert pos == [10.0, 20.0, 30.0]


def test_find_landmark_nested():
    pos = find_landmark_world_pos(_TREE, "BodyToLink1Point")
    assert pos == [5.0, 6.0, 7.0]


def test_find_landmark_deeply_nested():
    pos = find_landmark_world_pos(_TREE, "Link1ToLink2Point")
    assert pos == [1.0, 2.0, 3.0]


def test_find_landmark_not_found():
    assert find_landmark_world_pos(_TREE, "NonExistentPoint") is None


def test_find_landmark_empty_tree():
    assert find_landmark_world_pos([], "LegMountPointFL") is None


def test_find_landmark_at_occurrence_direct():
    pos = find_landmark_world_pos_at_occurrence(
        _TREE, "FlexibleSkeleton:1", "LegMountPointFR"
    )
    assert pos == [-10.0, 20.0, 30.0]


def test_find_landmark_at_occurrence_child():
    pos = find_landmark_world_pos_at_occurrence(
        _TREE, "FaceHuggerLegAssembly:1/Link1:1", "Link1ToLink2Point"
    )
    assert pos == [1.0, 2.0, 3.0]


def test_find_landmark_at_occurrence_bad_path():
    assert (
        find_landmark_world_pos_at_occurrence(_TREE, "NoSuchOcc:1", "SomePoint") is None
    )


def test_find_landmark_at_occurrence_landmark_missing():
    assert (
        find_landmark_world_pos_at_occurrence(
            _TREE, "FlexibleSkeleton:1", "NonExistentPoint"
        )
        is None
    )
