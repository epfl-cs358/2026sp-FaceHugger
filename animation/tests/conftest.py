"""Shared fixtures for `animation/tests/`.

Two responsibilities:

- Register the `requires_blender` marker so Blender-dependent tests collected
  here (or under `animation/scripts/`, `animation/addons/`, `animation/lib/`)
  can opt out cleanly when run from plain Python.
- Make `animation/lib/` importable so tests can write `from lib.servo_math
  import ...` without per-test sys.path tinkering — mirrors what the
  ad-hoc test scripts under `animation/scripts/` already do by hand.

`bpy_stub.install()` is intentionally NOT called here. Tests that need it
should call it themselves so the install is local to the test that depends
on it.
"""

import sys
from pathlib import Path

import pytest

ANIMATION_DIR = Path(__file__).resolve().parent.parent
LIB_DIR = ANIMATION_DIR / "lib"

if str(LIB_DIR) not in sys.path:
    sys.path.insert(0, str(LIB_DIR))


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line(
        "markers",
        "requires_blender: test needs the Blender Python runtime (bpy, "
        "mathutils, etc.) and is skipped under plain Python.",
    )
