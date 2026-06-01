# === Plain Python — no Blender required ===
# Run via pytest.
"""Cross-repo CALIB consistency check.

Asserts that servo_math._CALIB_THIGH / _CALIB_KNEE match the #define values
in code/firmware/src/shared/calib.h so a calibration update that touches one
side without the other is caught immediately.
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from servo_math import _CALIB_KNEE, _CALIB_THIGH

_CALIB_H = (
    Path(__file__).resolve().parents[3]
    / "code"
    / "firmware"
    / "src"
    / "shared"
    / "calib.h"
)

_DEFINE_RE = re.compile(r"#define\s+(CALIB_\w+)\s+(\d+)")


def _parse_calib_h():
    text = _CALIB_H.read_text()
    return {m.group(1): int(m.group(2)) for m in _DEFINE_RE.finditer(text)}


_FW = _parse_calib_h()

# Firmware leg name prefix → servo_math dict key
_LEG_MAP = {"FR": "fr", "FL": "fl", "BR": "br", "BL": "bl"}


def test_calib_thigh_matches_firmware():
    for fw_prefix, py_leg in _LEG_MAP.items():
        fw_val = _FW[f"CALIB_{fw_prefix}_THIGH"]
        py_val = _CALIB_THIGH[py_leg]
        assert py_val == fw_val, (
            f"_CALIB_THIGH['{py_leg}']={py_val} != firmware CALIB_{fw_prefix}_THIGH={fw_val}; "
            f"update servo_math.py or calib.h to match"
        )


def test_calib_knee_matches_firmware():
    for fw_prefix, py_leg in _LEG_MAP.items():
        fw_val = _FW[f"CALIB_{fw_prefix}_KNEE"]
        py_val = _CALIB_KNEE[py_leg]
        assert py_val == fw_val, (
            f"_CALIB_KNEE['{py_leg}']={py_val} != firmware CALIB_{fw_prefix}_KNEE={fw_val}; "
            f"update servo_math.py or calib.h to match"
        )


def test_all_four_legs_present():
    assert set(_CALIB_THIGH.keys()) == {"fr", "fl", "br", "bl"}
    assert set(_CALIB_KNEE.keys()) == {"fr", "fl", "br", "bl"}
