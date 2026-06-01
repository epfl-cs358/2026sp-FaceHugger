# animation/

Blender tooling for the FaceHugger animation pipeline:

| Subfolder | Kind | Contents |
|---|---|---|
| [`scripts/`](scripts/) | Scene builders (Blender) + plain-Python CLI tools | `urdf_to_blender_rigged.py`, `visualize_urdf.py`, `visualize_fusion_export.py`, `check_export_consistency.py`, `test_servo_parity.py` |
| [`addons/`](addons/) | Blender add-on — install or run in Text Editor | `fh_clip_panel.py` (clip manager, pose library, export) |
| [`lib/`](lib/) | Shared Python modules used by scripts and the add-on | `urdf_parser.py`, `servo_math.py`, `bpy_stub.py` |

The scene builders are invoked by `code/facehugger.py blender [--rigged]`.
The add-on is stand-alone; see its subfolder README.

## Key files at this level

- `fh_rigged_latest.blend` — the animation library (rig + all authored clips).
  Built/updated by `facehugger.py blender --rigged`; edited by the clip panel.
- `poses.json` — committed pose library consumed by `fh_clip_panel.py`.
- `convention.json` — servo math + channel table for the exporter and parity test.

Servo ID / channel conventions (URDF link names, firmware `LEG_SERVO_CHANNEL`,
the still-open `servo_id` proposal) are documented in
[`wiki/reference/firmware/servo-conventions.md`](../wiki/reference/firmware/servo-conventions.md).
