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
    assert hasattr(mod, "_frame_to_servo")


@pytest.mark.requires_blender
def test_lib_urdf_parser_importable():
    pytest.importorskip("mathutils")  # only present inside Blender
    importlib.import_module("urdf_parser")
