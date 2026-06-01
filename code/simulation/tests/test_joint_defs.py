"""Tests for urdf_gen.lib.joint_defs — build_joint_definitions and flip_joint_for_r_side."""

from __future__ import annotations

import math

import pytest

from urdf_gen.lib.joint_defs import (
    JointBuildResult,
    JointDef,
    build_joint_definitions,
    flip_joint_for_r_side,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _make_export(
    link1_limits=None,
    link2_limits=None,
    link3_limits=None,
    link1_axis=None,
    link2_axis=None,
    link3_axis=None,
    link1_origin=None,
    link2_origin=None,
    link3_origin=None,
):
    """Build a minimal fusion_export.json dict with 3 joints."""
    if link1_limits is None:
        link1_limits = {
            "min": math.radians(-60),
            "max": math.radians(60),
            "rest": math.radians(-10),
            "current": math.radians(-5),
        }
    if link2_limits is None:
        link2_limits = {
            "min": math.radians(-90),
            "max": math.radians(45),
            "rest": 0.0,
            "current": 0.0,
        }
    if link3_limits is None:
        link3_limits = {
            "min": math.radians(-45),
            "max": math.radians(90),
            "rest": 0.0,
            "current": 0.0,
        }

    if link1_axis is None:
        link1_axis = [0.0, 0.0, 1.0]
    if link2_axis is None:
        link2_axis = [1.0, 0.0, 0.0]
    if link3_axis is None:
        link3_axis = [1.0, 0.0, 0.0]

    if link1_origin is None:
        link1_origin = [10.0, 20.0, 30.0]
    if link2_origin is None:
        link2_origin = [10.0, 70.0, 30.0]
    if link3_origin is None:
        link3_origin = [10.0, 120.0, 30.0]

    return {
        "joints": [
            {
                "name": "Link1Revolute",
                "axis_dir_world": link1_axis,
                "axis_origin_local_mm": link1_origin,
                "limits_rad": link1_limits,
            },
            {
                "name": "Link2Revolute",
                "axis_dir_world": link2_axis,
                "axis_origin_local_mm": link2_origin,
                "limits_rad": link2_limits,
            },
            {
                "name": "Link3Revolute",
                "axis_dir_world": link3_axis,
                "axis_origin_local_mm": link3_origin,
                "limits_rad": link3_limits,
            },
        ]
    }


_DEFAULT_CONFIG = {"shoulder_rest_source": "rest"}
_CURRENT_CONFIG = {"shoulder_rest_source": "current"}


# ---------------------------------------------------------------------------
# flip_joint_for_r_side tests
# ---------------------------------------------------------------------------


class TestFlipJointForRSide:
    def test_negates_axis_dir(self):
        axis, lo, hi, _ = flip_joint_for_r_side([1.0, 0.0, -1.0], -30.0, 60.0)
        assert axis == [-1.0, 0.0, 1.0]

    def test_negate_swaps_limits_when_both_not_none(self):
        _, lo, hi, _ = flip_joint_for_r_side([1.0, 0.0, 0.0], -30.0, 60.0)
        assert lo == -60.0
        assert hi == 30.0

    def test_limits_unchanged_when_lo_is_none(self):
        _, lo, hi, _ = flip_joint_for_r_side([1.0, 0.0, 0.0], None, 60.0)
        assert lo is None
        assert hi == 60.0

    def test_limits_unchanged_when_hi_is_none(self):
        _, lo, hi, _ = flip_joint_for_r_side([1.0, 0.0, 0.0], -30.0, None)
        assert lo == -30.0
        assert hi is None

    def test_limits_unchanged_when_both_none(self):
        _, lo, hi, _ = flip_joint_for_r_side([1.0, 0.0, 0.0], None, None)
        assert lo is None
        assert hi is None

    def test_negates_offset_x_when_given(self):
        _, _, _, offset = flip_joint_for_r_side(
            [1.0, 0.0, 0.0], -30.0, 60.0, [5.0, 10.0, 15.0]
        )
        assert offset[0] == -5.0
        assert offset[1] == 10.0
        assert offset[2] == 15.0

    def test_returns_none_offset_when_not_given(self):
        _, _, _, offset = flip_joint_for_r_side([1.0, 0.0, 0.0], -30.0, 60.0)
        assert offset is None

    def test_returns_copies_not_same_objects(self):
        original_axis = [1.0, 0.0, 0.0]
        original_offset = [5.0, 10.0, 15.0]
        axis, lo, hi, offset = flip_joint_for_r_side(
            original_axis, -30.0, 60.0, original_offset
        )
        # Original should be unchanged
        assert original_axis == [1.0, 0.0, 0.0]
        assert original_offset == [5.0, 10.0, 15.0]
        # Return values should be new objects
        assert axis is not original_axis
        assert offset is not original_offset


# ---------------------------------------------------------------------------
# build_joint_definitions tests
# ---------------------------------------------------------------------------


class TestBuildJointDefinitions:
    def test_returns_three_joint_defs(self):
        export = _make_export()
        result = build_joint_definitions(export, _DEFAULT_CONFIG)
        assert isinstance(result, JointBuildResult)
        assert len(result.joints) == 3
        assert all(isinstance(j, JointDef) for j in result.joints)

    def test_fl_rest_rad_from_rest_source(self):
        link1_limits = {
            "min": math.radians(-60),
            "max": math.radians(60),
            "rest": math.radians(-10),
            "current": math.radians(-5),
        }
        export = _make_export(link1_limits=link1_limits)
        result = build_joint_definitions(export, _DEFAULT_CONFIG)
        assert result.fl_rest_rad == pytest.approx(math.radians(-10))

    def test_fl_rest_rad_from_current_source(self):
        link1_limits = {
            "min": math.radians(-60),
            "max": math.radians(60),
            "rest": math.radians(-10),
            "current": math.radians(-5),
        }
        export = _make_export(link1_limits=link1_limits)
        result = build_joint_definitions(export, _CURRENT_CONFIG)
        assert result.fl_rest_rad == pytest.approx(math.radians(-5))

    def test_shoulder_lower_deg_is_min_minus_rest(self):
        link1_limits = {
            "min": math.radians(-60),
            "max": math.radians(60),
            "rest": math.radians(-10),
            "current": 0.0,
        }
        export = _make_export(link1_limits=link1_limits)
        result = build_joint_definitions(export, _DEFAULT_CONFIG)
        expected = math.degrees(math.radians(-60) - math.radians(-10))
        assert result.shoulder_lower_deg == pytest.approx(expected)

    def test_shoulder_upper_deg_is_max_minus_rest(self):
        link1_limits = {
            "min": math.radians(-60),
            "max": math.radians(60),
            "rest": math.radians(-10),
            "current": 0.0,
        }
        export = _make_export(link1_limits=link1_limits)
        result = build_joint_definitions(export, _DEFAULT_CONFIG)
        expected = math.degrees(math.radians(60) - math.radians(-10))
        assert result.shoulder_upper_deg == pytest.approx(expected)

    def test_joints_1_has_offset_from_parent(self):
        export = _make_export(
            link1_origin=[10.0, 20.0, 30.0],
            link2_origin=[10.0, 70.0, 30.0],
        )
        result = build_joint_definitions(export, _DEFAULT_CONFIG)
        # offset = link2_origin - link1_origin = [0, 50, 0]
        assert result.joints[1].offset_from_parent_mm == pytest.approx([0.0, 50.0, 0.0])

    def test_joint_0_has_no_offset_from_parent(self):
        export = _make_export()
        result = build_joint_definitions(export, _DEFAULT_CONFIG)
        assert result.joints[0].offset_from_parent_mm is None

    def test_raises_when_joint_name_missing(self):
        export = {
            "joints": [
                {
                    "name": "Link1Revolute",
                    "axis_dir_world": [0, 0, 1],
                    "limits_rad": {"min": -1.0, "max": 1.0},
                }
            ]
        }
        with pytest.raises(ValueError, match="Link2Revolute"):
            build_joint_definitions(export, _DEFAULT_CONFIG)

    def test_raises_when_axis_dir_missing(self):
        export = _make_export()
        # Remove axis info from link1
        export["joints"][0].pop("axis_dir_world")
        # Ensure there's no fallback key either
        with pytest.raises(ValueError, match="Link1Revolute"):
            build_joint_definitions(export, _DEFAULT_CONFIG)

    def test_raises_when_rest_source_invalid(self):
        export = _make_export()
        with pytest.raises(ValueError, match="shoulder_rest_source"):
            build_joint_definitions(export, {"shoulder_rest_source": "invalid_value"})

    def test_joint_defs_have_correct_topology_names(self):
        export = _make_export()
        result = build_joint_definitions(export, _DEFAULT_CONFIG)
        assert result.joints[0].urdf_name == "link1"
        assert result.joints[1].urdf_name == "link2"
        assert result.joints[2].urdf_name == "link3"

    def test_joint_defs_have_correct_parent_child(self):
        export = _make_export()
        result = build_joint_definitions(export, _DEFAULT_CONFIG)
        assert result.joints[0].parent == "base_link"
        assert result.joints[0].child == "link1"
        assert result.joints[1].parent == "link1"
        assert result.joints[2].parent == "link2"

    def test_uses_axis_dir_local_unit_as_fallback(self):
        export = _make_export()
        # Remove axis_dir_world, add axis_dir_local_unit
        export["joints"][0].pop("axis_dir_world")
        export["joints"][0]["axis_dir_local_unit"] = [0.0, 1.0, 0.0]
        result = build_joint_definitions(export, _DEFAULT_CONFIG)
        assert result.joints[0].axis_dir == [0.0, 1.0, 0.0]

    def test_raises_when_limits_min_max_missing(self):
        link1_limits = {"min": None, "max": None, "rest": 0.0, "current": 0.0}
        export = _make_export(link1_limits=link1_limits)
        with pytest.raises(ValueError, match="limits_rad.min/max"):
            build_joint_definitions(export, _DEFAULT_CONFIG)
