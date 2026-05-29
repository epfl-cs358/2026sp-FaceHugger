# animation/

Blender tooling for the FaceHugger animation pipeline, organized into three kinds:

| Subfolder | Kind | Contents |
|---|---|---|
| [`pipeline/`](pipeline/) | Scene builders — run via `blender --python` | `urdf_to_blender_rigged.py`, `visualize_urdf.py`, `visualize_fusion_export.py` |
| [`addons/`](addons/) | Blender add-on — install or run in Text Editor | `fh_clip_panel.py` (clip manager, pose library, export) |
| [`migrations/`](migrations/) | One-shot CLI — run once per old rig | `fh_rename_actions.py` (legacy Action name migration) |

The scene builders are invoked by `code/simulation/facehugger.py blender [--rigged]`.
The add-on and migration scripts are stand-alone; see each subfolder's README.

## Key files at this level

- `fh_rigged_latest.blend` — the animation library (rig + all authored clips).
  Built/updated by `facehugger.py blender --rigged`; edited by the clip panel.
- `SERVO_ID_CONVENTION.md` — servo ID / channel mapping between URDF links and
  firmware `LEG_SERVO_CHANNEL`.
- `poses.json` — committed pose library consumed by `fh_clip_panel.py`.
- `convention.json` — servo math + channel table for the exporter and parity test.
