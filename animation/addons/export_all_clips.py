#!/usr/bin/env python3
"""Headless re-export of every clip in the open .blend to
animation/exported_clips/<clip>/{<clip>.js,.h,.csv}.

Runs the (fixed) bake_clip + the Layer-2 converters directly — no operator
or scene-property dependency — so it works in --background. Use it after
changing the exporter to regenerate all clips deterministically. Run via:

    BLENDER_BIN=/Applications/Blender-5.1.app/Contents/MacOS/Blender \
      "$BLENDER_BIN" --background --factory-startup \
      animation/fh_rigged_latest.blend \
      --python animation/scripts/export_all_clips.py
"""

import importlib.util
import os
import sys

import bpy

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(REPO_ROOT, "animation/scripts/fh_clip_panel.py")


def main() -> int:
    spec = importlib.util.spec_from_file_location("fh_clip_panel", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    conv = mod._load_convention()
    ctx = bpy.context
    clips = mod.list_clips()
    print(f"Re-exporting {len(clips)} clip(s): {clips}")
    baked = {}
    for clip in clips:
        rows = mod.bake_clip(clip, ctx)
        mod.to_csv(rows, clip)
        mod.to_c_header(rows, clip, conv)
        js_path = mod.to_js(rows, clip, conv)
        baked[clip] = rows
        print(f"  exported '{clip}': {len(rows)} frames -> {js_path}")

    # Bundle every clip into one clips_all.h + clips_manifest.json (Phase 2).
    header, manifest = mod.to_clips_header(baked, conv, write=True)
    out = os.path.join(REPO_ROOT, "animation/exported_clips")
    print(
        f"  bundled {len(baked)} clip(s) -> {out}/clips_all.h (+ clips_manifest.json)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
