#!/usr/bin/env python3
# === Plain Python — no Blender required ===
# Run via pytest or directly. Stubs `bpy` at import time.
"""Pure-Python test for to_clips_extra_json() — the app-side clip bundle.

Locks the math-space contract: the bundle holds math-space joint
degrees (NOT servo degrees with CALIB baked in), and carries a
top-level `wire: "T12"` schema discriminator so the mobile app can
hard-fail on a stale bundle.

At NEUTRAL the math-space output equals NEUTRAL itself (since
`N + (N - N) * scale = N`), so the test is independent of the
2/3 scale factor.
"""

import importlib.util
import json
import os
import sys
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
SCRIPT = os.path.join(REPO_ROOT, "animation/addons/fh_clip_panel.py")
CONV_PATH = os.path.join(REPO_ROOT, "animation/convention.json")


def _stub_bpy():
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
            frame_change_post=[], save_pre=[], load_post=[], persistent=lambda fn: fn
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


def _load_mod():
    _stub_bpy()
    spec = importlib.util.spec_from_file_location("fh_clip_panel", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    mod = _load_mod()
    conv = json.load(open(CONV_PATH))
    JB = mod.JOINT_BONES

    # Build one row sitting exactly at NEUTRAL for every leg.
    # In math-space at NEUTRAL: scaled = N + (raw - N) * scale = N.
    # (Independent of scale, so the test survives the scale=2/3 → 1.0
    # migration that's planned separately.)
    n = conv["neutral_joint_deg"]
    row = {"frame": 0, "time_ms": 0}
    for leg in ("fr", "fl", "br", "bl"):
        sh, th, kn = n[leg]
        row[f"{leg}_link1"] = float(sh)
        row[f"{leg}_link2"] = float(th)
        row[f"{leg}_link3"] = float(kn)
    # Sanity: every JOINT_BONES key is set.
    assert all(b in row for b in JB), set(JB) - set(row)

    clips = {"neutral_only": [row]}
    bundle = json.loads(mod.to_clips_extra_json(clips, conv, write=False))

    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(
            ("PASS " if cond else "FAIL ") + name + (f" — {detail}" if detail else "")
        )
        ok = ok and cond

    # 1. Schema discriminator present.
    check(
        "bundle top-level has wire:'T12'",
        bundle.get("wire") == "T12",
        detail=f"got wire={bundle.get('wire')!r}",
    )

    # 2. Per-leg triples are math-space NEUTRAL (not servo-space).
    #    Servo-space at NEUTRAL would be [90, CALIB_THIGH+/-NEUTRAL.th, ...] —
    #    e.g. FR thigh ≈ 144 (servo) vs -60 (math). Assert math-space.
    frame0 = bundle["clips"][0]["frames"][0]
    for leg in ("fr", "fl", "br", "bl"):
        expected = list(n[leg])
        got = frame0[leg]
        check(
            f"{leg} triple is math-space NEUTRAL == {expected}",
            list(got) == expected,
            detail=f"got {got}",
        )

    print("RESULT " + ("GREEN" if ok else "RED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
