"""Export consistency checker: verifies .h bone angles round-trip to .js
servo values via _frame_to_servo from fh_clip_panel.py.

Usage:
    python check_export_consistency.py [--export-dir PATH]

Exit codes: 0 = all pass, 1 = one or more failures, 2 = file error.
"""

import argparse
import importlib.util
import json
import re
import sys
import types
from pathlib import Path

ADDONS_DIR = Path(__file__).parent.parent / "addons"

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


def _stub_bpy() -> None:
    """Minimal bpy shim so fh_clip_panel.py imports under plain Python.
    Mirrors the stub in test_servo_parity.py."""
    if "bpy" in sys.modules:
        return
    bpy = types.ModuleType("bpy")
    bpy.types = types.SimpleNamespace(
        Operator=type("Operator", (), {}),
        Panel=type("Panel", (), {}),
        Scene=type("Scene", (), {}),
    )
    bpy.props = types.SimpleNamespace(
        StringProperty=lambda **kw: None,
        IntProperty=lambda **kw: None,
        BoolProperty=lambda **kw: None,
        EnumProperty=lambda **kw: None,
    )
    bpy.app = types.SimpleNamespace(
        handlers=types.SimpleNamespace(
            frame_change_post=[],
            save_pre=[],
            load_post=[],
            persistent=lambda fn: fn,
        )
    )
    bpy.data = types.SimpleNamespace(
        objects=types.SimpleNamespace(get=lambda *a, **kw: None),
        actions=[],
        filepath="",
    )
    bpy.context = types.SimpleNamespace(scene=None, view_layer=None)
    bpy.utils = types.SimpleNamespace(
        register_class=lambda x: None, unregister_class=lambda x: None
    )
    bpy.path = types.SimpleNamespace(abspath=lambda p: p)
    sys.modules["bpy"] = bpy


_stub_bpy()

_panel_script = ADDONS_DIR / "fh_clip_panel.py"
_spec = importlib.util.spec_from_file_location("fh_clip_panel", str(_panel_script))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
_frame_to_servo = _mod._frame_to_servo


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


def check_all_clips(export_dir: Path, convention: dict = None) -> bool:
    """Check all clip subdirs in export_dir. Returns True if all pass."""
    if convention is None:
        convention_path = Path(__file__).parent.parent / "convention.json"
        with open(convention_path) as f:
            convention = json.load(f)

    clip_dirs = [d for d in export_dir.iterdir() if d.is_dir()]
    if not clip_dirs:
        print("No clip directories found.")
        return True

    results = [check_clip(d, convention) for d in sorted(clip_dirs)]
    passed = sum(results)
    total = len(results)
    print(
        f"\n{'All clips OK' if passed == total else 'FAILURES found'}: "
        f"{passed}/{total} passed"
    )
    return passed == total


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
