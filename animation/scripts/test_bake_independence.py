#!/usr/bin/env python3
"""Headless-Blender regression: baking a clip must be independent of the
currently-active clip. Guards the bake_clip() mislabel bug (export-review
findings 2026-05-21 §2). Run via:

    BLENDER_BIN=/Applications/Blender-5.1.app/Contents/MacOS/Blender \
      "$BLENDER_BIN" --background --factory-startup \
      animation/fh_rigged_latest.blend \
      --python animation/scripts/test_bake_independence.py

Exit code 0 = GREEN, 1 = RED. --factory-startup avoids unrelated user
add-ons (e.g. ThreeMF_io) erroring on quit.
"""

import os
import sys
import importlib.util

import bpy

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(REPO_ROOT, "animation/scripts/fh_clip_panel.py")


def _load_module():
    spec = importlib.util.spec_from_file_location("fh_clip_panel", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _servo_payload(mod, rows, conv):
    """Reduce baked rows to the wire payload the robot actually receives."""
    return [mod._frame_to_servo(r, conv) for r in rows]


def main() -> int:
    mod = _load_module()
    conv = mod._load_convention()
    ctx = bpy.context

    target = "wave"  # the clip we bake twice
    other = "wiggle"  # a clearly different clip to make active

    # Bake `target` while `other` is the active clip.
    mod.assign_clip(other)
    ctx.view_layer.update()
    rows_a = mod.bake_clip(target, ctx)

    # Bake `target` again while `target` itself is active.
    mod.assign_clip(target)
    ctx.view_layer.update()
    rows_b = mod.bake_clip(target, ctx)

    pay_a = _servo_payload(mod, rows_a, conv)
    pay_b = _servo_payload(mod, rows_b, conv)

    ok = pay_a == pay_b
    print(
        f"bake('{target}') with active='{other}' vs active='{target}': "
        f"{'MATCH' if ok else 'DIFFER'} ({len(pay_a)} vs {len(pay_b)} frames)"
    )
    print("RESULT " + ("GREEN" if ok else "RED: bake_clip depends on active clip"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
