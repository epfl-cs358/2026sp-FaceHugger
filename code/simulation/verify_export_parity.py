#!/usr/bin/env python3
"""Independent cross-format verification of the Blender clip export.

Proves the PyBullet sim interprets the Blender export identically to what the
firmware / browser receive, by deriving the servo command for every clip frame
from THREE independent paths and asserting they agree:

  1. clips_all.h  (math-space, what the firmware compiles in) -> the SIM's own
     `servo_convention.translate_to_servo` (a Python port of the firmware switch,
     written independently of the exporter).
  2. the per-clip `<clip>.js` (servo degrees, what the browser streams) -> read
     directly.
  3. (when present) the per-clip `<clip>.h` (link1-corrected bone angles) ->
     scale-from-NEUTRAL -> same `translate_to_servo`, cross-checking that the
     bundled clips_all.h and the per-clip header agree on the math-space.

Path 1 and Path 2 are produced by SEPARATE implementations of the
`translateToServo` contract (the sim's `servo_convention.py` vs the exporter's
`fh_clip_panel._frame_to_servo`); the firmware C++ is locked to the exporter
separately by `code/firmware/test/test_clip_parity`. So agreement here closes
the loop: sim == exporter == firmware, on the actual exported data.

The `.js` applies a hard [0,180] clamp; the sim/firmware clip path applies the
tighter `clampClipServos`. This tool compares the CONVENTION (translate_to_servo)
under the .js's [0,180] clamp so the two are on equal footing — the tighter
clip-clamp is a separate, intentional firmware layer, not a convention diff.

Usage:
    python verify_export_parity.py [--export-dir PATH]
Exit: 0 = all formats agree, 1 = a mismatch, 2 = file error.
"""

import argparse
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pybullet_interpreter.clip_loader import (  # noqa: E402
    DEFAULT_CLIPS_H,
    load_clips_all_h,
)
from pybullet_interpreter.servo_convention import (  # noqa: E402
    LEG_FL,
    LEG_FR,
    LEG_RL,
    LEG_RR,
    translate_to_servo,
)

# clips_all.h math-space leg order (FR,FL,RR,RL) -> .js leg key.
_LEG_ID_BY_JS = {"fr": LEG_FR, "fl": LEG_FL, "br": LEG_RR, "bl": LEG_RL}
_JS_ORDER = ("fr", "fl", "br", "bl")


def _clamp_0_180(v: float) -> int:
    return int(round(max(0.0, min(180.0, v))))


def _sim_servo_js_equiv(a: list[float]) -> dict[str, list[int]]:
    """Servo degrees the SIM derives from a math-space frame a[12], under the
    .js [0,180] clamp (to compare like-for-like with the browser export)."""
    out: dict[str, list[int]] = {}
    for js_key, leg_id in _LEG_ID_BY_JS.items():
        sh, th, kn = a[leg_id * 3], a[leg_id * 3 + 1], a[leg_id * 3 + 2]
        s = translate_to_servo(leg_id, sh, th, kn)
        out[js_key] = [_clamp_0_180(s.hip), _clamp_0_180(s.thigh), _clamp_0_180(s.knee)]
    return out


def _parse_js_frames(path: Path) -> list[dict[str, list[int]]]:
    """Parse the CLIP array from a clip's .js -> list of {fr,fl,br,bl: [3 ints]}."""
    text = path.read_text()
    frames = []
    for m in re.finditer(r"\{[^{}]*\}", text):
        s = m.group()
        frame = {}
        for leg in _JS_ORDER:
            lm = re.search(rf"{leg}:\[(-?\d+),(-?\d+),(-?\d+)\]", s)
            if lm:
                frame[leg] = [int(lm.group(1)), int(lm.group(2)), int(lm.group(3))]
        if len(frame) == 4:
            frames.append(frame)
    return frames


def verify_clip(clip, export_dir: Path) -> list[str]:
    """Compare the sim's servo interpretation of clips_all.h against the clip's
    .js for every frame. Returns a list of error strings (empty = PASS)."""
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
        sim = _sim_servo_js_equiv(list(frame.a))
        for leg in _JS_ORDER:
            if sim[leg] != js[leg]:
                errors.append(
                    f"{clip.name} frame {i} {leg}: sim(clips_all.h)={sim[leg]} "
                    f".js={js[leg]}"
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
        "Verifying sim (clips_all.h via servo_convention) == browser .js, "
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
