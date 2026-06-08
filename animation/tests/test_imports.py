"""Phase 1 of animation-reorg — pin the importable surface of `animation/lib/`.

The reorg does not move any of these files, but the tests guard against an
accidental rename or deletion during later phases.

Modules under `lib/` split into two kinds. Plain-Python modules import
unconditionally. `urdf_parser` pulls in `mathutils` and is only usable
inside Blender, so it is gated on the `requires_blender` marker.
"""

import importlib

import pytest


def test_lib_bpy_stub_importable():
    mod = importlib.import_module("bpy_stub")
    assert hasattr(mod, "install")


def test_lib_servo_math_importable():
    mod = importlib.import_module("servo_math")
    # _scale_from_neutral is the math-space helper the exporter relies on.
    # _frame_to_servo moved to animation_tools.exporter_parity (Phase 3 of the
    # CALIB-decoupling plan); see animation/lib/tests/test_servo_math_no_calib.py.
    assert hasattr(mod, "_scale_from_neutral")


@pytest.mark.requires_blender
def test_lib_urdf_parser_importable():
    pytest.importorskip("mathutils")  # only present inside Blender
    importlib.import_module("urdf_parser")
