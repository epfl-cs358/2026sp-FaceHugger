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
    if not ok:
        print("RESULT RED: bake_clip depends on active clip")
        return 1

    # Frame-delta warning fires on a synthetic > threshold jump.
    fake = [
        {"frame": 0, "time_ms": 0, **{b: 0.0 for b in mod.JOINT_BONES}},
        {"frame": 1, "time_ms": 33, **{b: 0.0 for b in mod.JOINT_BONES}},
    ]
    fake[1]["fl_link1"] = mod.FRAME_DELTA_WARN_DEG + 5.0
    nwarn = mod._warn_frame_deltas(fake)
    print(f"delta-warning fired {nwarn} time(s) on synthetic jump")
    if nwarn != 1:
        print("RESULT RED: frame-delta warning did not fire as expected")
        return 1

    import re

    # JS export wire-contract checks (semantic — robust to template formatting).
    rows = mod.bake_clip("wave", ctx)
    js = mod.to_js(rows, "wave", conv, write=False)
    fps = ctx.scene.render.fps / ctx.scene.render.fps_base
    expected_frame_ms = round(1000.0 / fps)

    def _leg_ids_ok(s):
        m = re.search(r"LEG_IDS\s*=\s*\{([^}]*)\}", s)
        if not m:
            return False
        body = m.group(1)
        return all(
            re.search(rf"\b{leg}\s*:\s*{idx}\b", body)
            for leg, idx in (("fr", 0), ("fl", 1), ("br", 2), ("bl", 3))
        )

    def _frame_ms_ok(s, expected):
        m = re.search(r"FRAME_MS\s*=\s*(\d+)", s)
        return m is not None and int(m.group(1)) == expected

    checks = {
        "LEG_IDS contains fr:0 fl:1 br:2 bl:3": _leg_ids_ok(js),
        "defensive upper clamp present": "Math.min(180" in js,
        f"FRAME_MS == round(1000/fps) ({expected_frame_ms})": _frame_ms_ok(
            js, expected_frame_ms
        ),
        "delta-encode skip-on-unchanged present": "_last" in js,
        "connection-failure alert present": ("onerror" in js and "alert(" in js),
    }
    for name, ok in checks.items():
        print(("PASS " if ok else "FAIL ") + name)
        if not ok:
            print("RESULT RED: JS export contract violated")
            return 1

    print("RESULT GREEN")
    return 0


if __name__ == "__main__":
    sys.exit(main())
