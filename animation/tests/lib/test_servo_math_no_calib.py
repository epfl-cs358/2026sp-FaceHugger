# === Plain Python — no Blender required ===
# Run via pytest.
"""Guardrail: animation.lib.servo_math is exporter-only and must NOT export
hardware CALIB or _frame_to_servo. CALIB lives in code/firmware/src/shared/
calib.h; the firmware applies translateToServo + CALIB on T:12 receipt. The
exporter is math-space-only (link1 delta→absolute + _scale_from_neutral).

If this test fails, the exporter is about to grow a hardware dependency
again — relocate CALIB to firmware-side parity helpers and import there.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import servo_math


def test_servo_math_does_not_export_calib():
    forbidden = ("_CALIB_THIGH", "_CALIB_KNEE", "_frame_to_servo")
    leaked = [name for name in forbidden if hasattr(servo_math, name)]
    assert not leaked, (
        f"animation/lib/servo_math.py leaks firmware-only symbols: {leaked}. "
        "Move them to a firmware-parity module (see Phase 3 of the "
        "CALIB-decoupling plan)."
    )


def test_servo_math_still_exports_math_space_helpers():
    # Sanity: the exporter still has what it needs.
    assert hasattr(servo_math, "_scale_from_neutral")
    assert hasattr(servo_math, "_link1_delta_to_absolute")
