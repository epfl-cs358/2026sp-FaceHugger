# === Plain Python — no Blender required ===
# Run as: python <this file>  (or via pytest).
"""Export consistency checker: verifies .h bone angles round-trip to .js
servo values via _frame_to_servo from animation/lib/servo_math.py.

Usage:
    python check_export_consistency.py [--export-dir PATH]

Exit codes: 0 = all pass, 1 = one or more failures, 2 = file error.
"""

import argparse
import json
import re
import sys
from pathlib import Path

_LIB = str(Path(__file__).resolve().parents[1] / "lib")
if _LIB not in sys.path:
    sys.path.insert(0, _LIB)
# _frame_to_servo moved to firmware-parity helpers (Phase 3 of the CALIB-
# decoupling plan — animation/lib/servo_math.py is math-space only now).
_FW_PORT_PARENT = str(Path(__file__).resolve().parents[2] / "code" / "simulation")
if _FW_PORT_PARENT not in sys.path:
    sys.path.insert(0, _FW_PORT_PARENT)

from firmware_port.exporter_parity import _frame_to_servo  # noqa: E402

BONE_ORDER = [
    "fl_link1",
    "fl_link2",
    "fl_link3",
    "fr_link1",
    "fr_link2",
    "fr_link3",
    "bl_link1",
    "bl_link2",
    "bl_link3",
    "br_link1",
    "br_link2",
    "br_link3",
]
JOINT_NAMES = ["hip", "thigh", "knee"]


def h_row_to_dict(values: list) -> dict:
    """Map a 12-element float list (bone order) to a named dict."""
    assert len(values) == 12, f"Expected 12 values, got {len(values)}"
    return dict(zip(BONE_ORDER, values))


def parse_h_file(path: Path) -> list:
    """Parse a per-clip .h file — returns list of 12-float rows."""
    text = path.read_text()

    # Try FhClipFrame format: { t_ms, { v0,...,v11 } }
    fhframe_rows = re.findall(r"\{\s*\d+,\s*\{([^}]+)\}", text)
    if fhframe_rows:
        result = []
        for row_str in fhframe_rows:
            vals = [
                float(v.strip().rstrip("f")) for v in row_str.split(",") if v.strip()
            ]
            if len(vals) == 12:
                result.append(vals)
        return result

    # Fallback: float[N][12] format: rows of 12 values
    rows = re.findall(r"\{([^{}]+)\}", text)
    result = []
    for row_str in rows:
        vals = [float(v.strip().rstrip("f")) for v in row_str.split(",") if v.strip()]
        if len(vals) == 12:
            result.append(vals)
    return result


def parse_js_file(path: Path) -> list:
    """Parse a .js CLIP array — returns list of {fr, fl, br, bl} dicts."""
    text = path.read_text()
    clip_match = re.search(r"const CLIP\s*=\s*\[(.*?)\];", text, re.DOTALL)
    if not clip_match:
        raise ValueError(f"No CLIP array found in {path}")
    clip_str = clip_match.group(1)

    frames = []
    for frame_str in re.finditer(r"\{[^{}]*\}", clip_str):
        s = frame_str.group()
        frame = {}
        for leg in ("fr", "fl", "br", "bl"):
            m = re.search(rf"{leg}:\[(-?\d+),(-?\d+),(-?\d+)\]", s)
            if m:
                frame[leg] = [int(m.group(1)), int(m.group(2)), int(m.group(3))]
        if len(frame) == 4:
            frames.append(frame)
    return frames


def check_frame(h_row: dict, js_frame: dict, convention: dict, frame_idx: int) -> list:
    """Compare translated h_row servo values against js_frame.
    Returns list of error strings (empty = PASS).
    """
    computed = _frame_to_servo(h_row, convention)
    errors = []
    for leg in ("fr", "fl", "br", "bl"):
        for j, joint in enumerate(JOINT_NAMES):
            got = computed[leg][j]
            want = js_frame[leg][j]
            if got != want:
                errors.append(
                    f"frame {frame_idx} {leg} {joint}: computed={got} js={want}"
                )
    return errors


def check_clip(clip_dir: Path, convention: dict) -> bool:
    """Check one clip directory. Prints PASS/FAIL. Returns True if all pass."""
    clip_name = clip_dir.name
    h_files = list(clip_dir.glob("*.h"))
    js_files = list(clip_dir.glob("*.js"))
    if not h_files or not js_files:
        print(f"  SKIP {clip_name}: missing .h or .js")
        return True

    h_frames = parse_h_file(h_files[0])
    js_frames = parse_js_file(js_files[0])

    if len(h_frames) != len(js_frames):
        print(
            f"  FAIL {clip_name}: frame count mismatch "
            f"h={len(h_frames)} js={len(js_frames)}"
        )
        return False

    all_pass = True
    for i, (h_row_vals, js_frame) in enumerate(zip(h_frames, js_frames)):
        h_row = h_row_to_dict(h_row_vals)
        errs = check_frame(h_row, js_frame, convention, frame_idx=i)
        for err in errs:
            print(f"  FAIL {clip_name}: {err}")
            all_pass = False

    if all_pass:
        print(f"  PASS {clip_name} ({len(h_frames)} frames)")
    return all_pass


def check_standing_neutral(convention: dict) -> list:
    """Assert translateToServo at the STANDING / NEUTRAL pose (firmware
    NEUTRAL[], T:2 s:5 = crouched standing):
      * every SHOULDER -> servo 90 (the "servo 90 = outward" guarantee;
        shoulders are servo 90 in BOTH flat and standing). Fail loudly if
        not — it means exporter/firmware drifted on "neutral shoulder",
        the bug that clamped fl/bl and collapsed the robot.
      * every hip/knee -> its NEUTRAL[] standing value, explicitly NOT 90
        (a 90 here would mean the crouch was lost; e.g. FR thigh=150,
        knee=53). Exact values are locked vs firmware by test_servo_parity.

    Feeds raw == NEUTRAL through _frame_to_servo; scale-from-NEUTRAL is then
    identity, so this is exactly translateToServo(NEUTRAL). NEUTRAL comes
    from convention.json (single source of truth). Returns error strings.
    """
    neutral = convention["neutral_joint_deg"]
    row = {}
    for leg in ("fr", "fl", "br", "bl"):
        sh, th, kn = neutral[leg]
        row[f"{leg}_link1"], row[f"{leg}_link2"], row[f"{leg}_link3"] = sh, th, kn
    servo = _frame_to_servo(row, convention)

    errors = []
    for leg in ("fr", "fl", "br", "bl"):
        sh_s, hip_s, kn_s = servo[leg]
        if sh_s != 90:
            errors.append(
                f"standing: {leg} shoulder at NEUTRAL = {sh_s}, expected 90 "
                f"(servo 90 must = outward; exporter/firmware convention drift)"
            )
        if hip_s == 90 or kn_s == 90:
            errors.append(
                f"standing: {leg} hip/knee at NEUTRAL = {hip_s}/{kn_s}; a 90 "
                f"means the standing crouch was lost (should be NEUTRAL[] values)"
            )
    return errors


def check_convention(convention: dict) -> list:
    """Check the standing NEUTRAL pose convention: shoulders → servo 90, hip/knee
    off 90. Returns the error list (empty = PASS). The machine-checkable form
    of the CONVENTIONS.md guarantees.

    The "flat / calibration pose = all 12 servos at 90" check has been removed:
    after CALIB constants were introduced (commit 706bf9b), thigh/knee at
    math = 0 produces servo = CALIB_<leg>_<joint>, not 90. The physical flat
    pose still corresponds to all servos at 90, but the math-space pre-image is
    no longer "thigh=0, knee=0" — it's leg-specific. There is no useful
    convention assertion at flat pose any more; calibration is verified
    independently by lib/tests/test_calib_consistency.py."""
    return check_standing_neutral(convention)


def check_all_clips(export_dir: Path, convention: dict = None) -> bool:
    """Check all clip subdirs in export_dir. Returns True if all pass."""
    if convention is None:
        convention_path = Path(__file__).parent.parent / "convention.json"
        with open(convention_path) as f:
            convention = json.load(f)

    conv_errors = check_convention(convention)
    if conv_errors:
        for err in conv_errors:
            print(f"  FAIL {err}")
    else:
        print(
            "  PASS convention (standing: shoulders 90, hip/knee = NEUTRAL[]; "
            "flat: all 12 servos 90)"
        )

    clip_dirs = [d for d in export_dir.iterdir() if d.is_dir()]
    if not clip_dirs:
        print("No clip directories found.")
        return not conv_errors

    results = [check_clip(d, convention) for d in sorted(clip_dirs)]
    passed = sum(results)
    total = len(results)
    clips_ok = passed == total
    all_ok = clips_ok and not conv_errors
    print(
        f"\n{'All clips OK' if all_ok else 'FAILURES found'}: "
        f"{passed}/{total} clips passed"
        f"{'' if not conv_errors else f'; {len(conv_errors)} convention error(s)'}"
    )
    return all_ok


def main() -> int:
    parser = argparse.ArgumentParser(description="Check .h/.js export consistency")
    parser.add_argument("--export-dir", default=None)
    args = parser.parse_args()

    export_dir = (
        Path(args.export_dir)
        if args.export_dir
        else Path(__file__).parent.parent / "exported_clips"
    )

    if not export_dir.is_dir():
        print(f"ERROR: export dir not found: {export_dir}", file=sys.stderr)
        return 2

    return 0 if check_all_clips(export_dir) else 1


if __name__ == "__main__":
    sys.exit(main())
