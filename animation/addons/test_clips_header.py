#!/usr/bin/env python3
# === Plain Python — no Blender required ===
# Run via pytest. Stubs `bpy` at import time.
"""Pure-Python test for to_clips_header() — the bundled clips_all.h
converter (Phase-2 clip player). No Blender: stubs bpy, then feeds
synthetic baked rows. Run via:

    uv run python animation/scripts/test_clips_header.py

Exit 0 = GREEN. Asserts the FROZEN clips_all.h contract (plan §0):
math-space pre-scaled degrees, firmware LegId order, t_ms monotonic,
scale applied exactly once, deterministic output, manifest schema.
"""

import importlib.util
import json
import os
import re
import sys
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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


def _row(frame, time_ms, value):
    """A baked row where every joint bone holds `value` (math-space deg)."""
    return {
        "frame": frame,
        "time_ms": time_ms,
        **{b: float(value) for b in __import__("fh_clip_panel").JOINT_BONES},
    }


def main():
    mod = _load_mod()
    conv = json.load(open(CONV_PATH))
    JB = mod.JOINT_BONES

    def row(frame, time_ms, value):
        return {"frame": frame, "time_ms": time_ms, **{b: float(value) for b in JB}}

    # Two clips, distinct content.
    rows_a = [row(0, 0, 0.0), row(1, 42, 90.0), row(2, 84, 45.0)]
    rows_b = [row(0, 0, 10.0), row(1, 50, 20.0)]
    clips = {"wiggle": rows_a, "bow": rows_b}

    header, manifest = mod.to_clips_header(clips, conv, write=False)

    ok = True

    def check(name, cond):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + name)
        ok = ok and cond

    # 1. Header well-formed + count.
    check(
        "guard + count",
        "FH_CLIPS_ALL_H" in header and "#define FH_CLIP_COUNT 2" in header,
    )
    check(
        "struct typedefs present",
        "FhClipFrame" in header and "FhClip " in header and "FH_CLIPS[" in header,
    )

    # 2. Manifest schema + ids.
    man = json.loads(manifest)
    ids = [c["id"] for c in man["clips"]]
    names = [c["name"] for c in man["clips"]]
    check("manifest ids 0..n-1", ids == [0, 1])
    # Emitter sorts clip names alphabetically (Task #7), so "bow" precedes
    # "wiggle" regardless of the input dict's insertion order.
    check("manifest names sorted alphabetically", names == ["bow", "wiggle"])
    # "bow" is id 0 now and has 2 frames (rows_b above).
    check("manifest frame_count", man["clips"][0]["frame_count"] == 2)
    check("manifest duration_ms == last t_ms", man["clips"][0]["duration_ms"] == 50)

    # 3. Scale applied exactly once, math-space (NOT servo-space).
    # For raw=0 and neutral n, scaled = n + (0-n)*scale. Check FR shoulder
    # (a[0]) of wiggle frame 0: neutral fr[0]=45, scale=0.6667 -> 45*(1-0.6667)=15.0.
    n_fr_sh = conv["neutral_joint_deg"]["fr"][0]
    scale = conv["scale"]
    expected_fr_sh = n_fr_sh + (0.0 - n_fr_sh) * scale
    # Parse the first frame's first float out of fh_clip_wiggle.
    m = re.search(r"fh_clip_wiggle\[\]\s*=\s*\{\s*\{\s*0,\s*\{\s*([-0-9.eEf]+)", header)
    check("clip array present", m is not None)
    if m:
        got = float(m.group(1).rstrip("f"))
        check(
            f"FR shoulder scaled-once == {expected_fr_sh:.3f} (got {got:.3f})",
            abs(got - expected_fr_sh) < 0.01,
        )

    # 4. Leg reorder: a[0..2]=FR(=blender fr), a[3..5]=FL(fr->fl),
    #    a[6..8]=RR(=blender br), a[9..11]=RL(=blender bl). With all bones
    #    equal in a row, FR uses fr neutral, RR uses br neutral, etc.
    #    Verify by a row where each leg differs.
    def leg_row(frame, t, fr, fl, br, bl):
        vals = {}
        for leg, v in (("fr", fr), ("fl", fl), ("br", br), ("bl", bl)):
            for j, link in enumerate(("link1", "link2", "link3")):
                vals[f"{leg}_{link}"] = float(v[j])
        return {"frame": frame, "time_ms": t, **vals}

    # Pick raw == neutral so scaled == neutral (scale fixed point), making
    # the expected a[12] exactly the neutral arrays in LegId order.
    nfr = conv["neutral_joint_deg"]["fr"]
    nfl = conv["neutral_joint_deg"]["fl"]
    nbr = conv["neutral_joint_deg"]["br"]
    nbl = conv["neutral_joint_deg"]["bl"]
    clips2 = {"x": [leg_row(0, 0, nfr, nfl, nbr, nbl)]}
    h2, _ = mod.to_clips_header(clips2, conv, write=False)
    m2 = re.search(r"fh_clip_x\[\]\s*=\s*\{\s*\{\s*0,\s*\{([^}]*)\}", h2)
    floats = [float(x.strip().rstrip("f")) for x in m2.group(1).split(",")]
    expected = nfr + nfl + nbr + nbl  # FR,FL,RR(br),RL(bl)
    check(
        "leg reorder FR,FL,RR,RL with neutral fixed point",
        all(abs(a - b) < 0.01 for a, b in zip(floats, expected)),
    )

    # 4b. Shuffled input -> alphabetical output (Task #7 regression).
    # Pass a deliberately reverse-sorted dict and assert the emitters
    # still produce alphabetical order. Guards against the documented
    # "insertion order = clip id" drift hazard.
    shuffled = {"zulu": rows_a, "alpha": rows_b}
    h_sh, m_sh = mod.to_clips_header(shuffled, conv, write=False)
    man_sh = json.loads(m_sh)
    sh_names = [c["name"] for c in man_sh["clips"]]
    sh_ids = [c["id"] for c in man_sh["clips"]]
    check("shuffled-input header: names alphabetical", sh_names == ["alpha", "zulu"])
    check("shuffled-input header: ids 0,1 in sorted order", sh_ids == [0, 1])
    # FH_CLIPS[] entries should list alpha before zulu.
    fh_clips_block = re.search(
        r"FH_CLIPS\[FH_CLIP_COUNT\]\s*=\s*\{(.*?)\};", h_sh, re.DOTALL
    )
    check("shuffled-input header: FH_CLIPS[] block present", fh_clips_block is not None)
    if fh_clips_block:
        body = fh_clips_block.group(1)
        i_alpha = body.find('"alpha"')
        i_zulu = body.find('"zulu"')
        check(
            "shuffled-input header: FH_CLIPS[] orders alpha before zulu",
            i_alpha != -1 and i_zulu != -1 and i_alpha < i_zulu,
        )
    # to_clips_extra_json on the same shuffled dict must also sort.
    extra_sh = json.loads(mod.to_clips_extra_json(shuffled, conv, write=False))
    extra_names = [c["name"] for c in extra_sh["clips"]]
    check("shuffled-input extra: clips[0].name == 'alpha'", extra_names[0] == "alpha")
    check("shuffled-input extra: names alphabetical", extra_names == ["alpha", "zulu"])

    # 5. Determinism.
    h_again, m_again = mod.to_clips_header(clips, conv, write=False)
    check("deterministic header", h_again == header)
    check("deterministic manifest", m_again == manifest)

    # 6. Empty clip refused.
    try:
        mod.to_clips_header({"empty": []}, conv, write=False)
        check("empty clip refused", False)
    except ValueError:
        check("empty clip refused", True)

    # 7. Non-monotonic t_ms refused.
    bad = [row(0, 0, 0.0), row(1, 0, 1.0)]  # t_ms not increasing
    try:
        mod.to_clips_header({"bad": bad}, conv, write=False)
        check("non-monotonic t_ms refused", False)
    except ValueError:
        check("non-monotonic t_ms refused", True)

    # 8. C-symbol collision refused ("tiny wiggle" and "tiny-wiggle" both
    #    -> fh_clip_tiny_wiggle).
    coll = {"tiny wiggle": [row(0, 0, 0.0)], "tiny-wiggle": [row(0, 0, 1.0)]}
    try:
        mod.to_clips_header(coll, conv, write=False)
        check("C-symbol collision refused", False)
    except ValueError:
        check("C-symbol collision refused", True)

    # 9. uint16_t overflow refused (t_ms > 65535).
    over = [row(0, 0, 0.0), row(1, 70000, 1.0)]
    try:
        mod.to_clips_header({"over": over}, conv, write=False)
        check("uint16_t overflow refused", False)
    except ValueError:
        check("uint16_t overflow refused", True)

    print("RESULT " + ("GREEN" if ok else "RED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
