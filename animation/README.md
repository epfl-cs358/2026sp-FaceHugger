# animation/

Blender tooling for the FaceHugger animation pipeline. Each `.py` file declares
its runtime requirement in a two-line header so the reader doesn't have to
scan imports:

    # === Blender-only ===                        (needs the bpy runtime)
    # === Plain Python — no Blender required ===  (pure helpers / stubs bpy)

`scripts/` holds the Blender-driven scene builders — `visualize_urdf.py`,
`urdf_to_blender_rigged.py`, `visualize_fusion_export.py` — entered through
`code/facehugger.py blender [--rigged]`. The folder also carries the
plain-Python helpers `check_export_consistency.py` and `test_servo_parity.py`
that run directly under pytest or `python`; they live here because they
verify the exporter contract.

`addons/` is the Blender add-on (`fh_clip_panel.py`) and its headless
companion (`export_all_clips.py`). The add-on is installed via
Edit > Preferences > Add-ons or reloaded in place by the panel's
"Reload Add-on" button. It is stand-alone — `facehugger.py` does not invoke it.

`lib/` is the shared library both worlds import from: `urdf_parser.py`
(Blender-only — needs `mathutils`), `servo_math.py` (plain Python — kept
byte-identical to the firmware `translateToServo` math, see the parity test),
and `bpy_stub.py` (plain-Python stub of bpy for import-time tests).

Tests live next to the code they test (`scripts/test_*.py`, `addons/test_*.py`,
`lib/tests/test_*.py`), plus repo-level layout and contract checks under
`tests/`. Run them all with `pytest animation/` — Blender-only tests auto-skip
under plain Python via `pytest.importorskip("bpy")`.

## Key files at this level

- `fh_rigged_latest.blend` — the animation library (rig + all authored clips).
  Built/updated by `facehugger.py blender --rigged`; edited by the clip panel.
- `SERVO_ID_CONVENTION.md` — servo ID / channel mapping between URDF links and
  firmware `LEG_SERVO_CHANNEL`.
- `poses.json` — committed pose library consumed by `fh_clip_panel.py`.
- `convention.json` — servo math + channel table for the exporter and parity test.
