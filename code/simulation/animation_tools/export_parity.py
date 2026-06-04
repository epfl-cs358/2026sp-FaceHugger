#!/usr/bin/env python3
"""Cross-format verification of the Blender clip export.

With T:12 (CMD_STREAM_FRAME), the browser .js files now carry math-space joint
angles directly — no CALIB pre-applied. The parity check therefore becomes a
simple identity assertion: for every frame, the per-leg [sh,th,kn] in the .js
must equal the corresponding values in clips_all.h (within floating-point
rounding to 4 decimal places, which is the .js format precision).

  clips_all.h  (math-space, FR,FL,RR,RL × sh,th,kn)
          |
          +---> .js (T:12 math-space frames, same data, 4dp)

The firmware C++ translation (translateToServo) is locked separately by
`code/firmware/test/test_clip_parity`. This tool guards the export: the .js
the browser streams must be bit-for-bit equal to what the firmware compiled in.

Usage:
    python export_parity.py [--export-dir PATH]
Exit: 0 = all formats agree, 1 = a mismatch, 2 = file error.
"""

import argparse
import re
import sys
from pathlib import Path

# animation_tools/ → code/simulation/ (so pybullet_sim is importable when run by path)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from animation_tools.clip_loader import (  # noqa: E402
    DEFAULT_CLIPS_H,
    load_clips_all_h,
)

_JS_LEGS = ("fr", "fl", "br", "bl")
# clips_all.h interleaves legs as [FR(0), FL(1), RR(2), RL(3)], three joints each.
_H_LEG_IDX = {"fr": 0, "fl": 1, "br": 2, "bl": 3}

# Tolerance: .js angles are rounded to 4dp, so max rounding error is 0.00005.
# Use 0.001 for safety.
_TOL = 0.001


def _parse_js_frames(path: Path) -> list[dict[str, list[float]]]:
    """Parse the CLIP array from a T:12 .js file.

    Returns list of {fr,fl,br,bl: [sh, th, kn]} in math-space degrees.
    """
    text = path.read_text()
    frames = []
    for m in re.finditer(r"\{[^{}]*\}", text):
        s = m.group()
        frame = {}
        for leg in _JS_LEGS:
            lm = re.search(rf"{leg}:\[(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\]", s)
            if lm:
                frame[leg] = [
                    float(lm.group(1)),
                    float(lm.group(2)),
                    float(lm.group(3)),
                ]
        if len(frame) == 4:
            frames.append(frame)
    return frames


def verify_clip(clip, export_dir: Path) -> list[str]:
    """Compare clips_all.h math-space angles against the .js for every frame."""
    js_path = export_dir / clip.name / f"{clip.name}.js"
    if not js_path.exists():
        return [f"{clip.name}: no .js at {js_path}"]
    js_frames = _parse_js_frames(js_path)
    if len(js_frames) != len(clip.frames):
        return [
            f"{clip.name}: frame count clips_all.h={len(clip.frames)} "
            f".js={len(js_frames)}"
        ]
    errors = []
    for i, (frame, js) in enumerate(zip(clip.frames, js_frames)):
        a = list(frame.a)
        for leg, leg_idx in _H_LEG_IDX.items():
            h_vals = [a[leg_idx * 3], a[leg_idx * 3 + 1], a[leg_idx * 3 + 2]]
            js_vals = js[leg]
            for j, (hv, jv) in enumerate(zip(h_vals, js_vals)):
                if abs(hv - jv) > _TOL:
                    joint = ("sh", "th", "kn")[j]
                    errors.append(
                        f"{clip.name} frame {i} {leg}.{joint}: "
                        f"clips_all.h={hv:.4f} .js={jv:.4f} "
                        f"(diff={abs(hv - jv):.5f})"
                    )
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-dir", default=None)
    args = parser.parse_args()
    export_dir = Path(args.export_dir) if args.export_dir else DEFAULT_CLIPS_H.parent
    if not DEFAULT_CLIPS_H.exists():
        print(f"ERROR: clips_all.h not found at {DEFAULT_CLIPS_H}", file=sys.stderr)
        return 2

    clips = load_clips_all_h(DEFAULT_CLIPS_H)
    print(
        f"Verifying clips_all.h math-space == .js T:12 math-space, "
        f"for {len(clips)} clip(s):"
    )
    all_ok = True
    for clip in clips:
        errs = verify_clip(clip, export_dir)
        if errs:
            all_ok = False
            print(f"  FAIL {clip.name}: {len(errs)} mismatch(es)")
            for e in errs[:4]:
                print(f"        {e}")
        else:
            print(f"  PASS {clip.name} ({len(clip.frames)} frames)")
    print("\n" + ("All formats agree ✓" if all_ok else "MISMATCHES found ✗"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
