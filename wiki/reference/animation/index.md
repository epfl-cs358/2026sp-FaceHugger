# Animation pipeline

FaceHugger's animation pipeline runs from CAD through to on-board playback: the Fusion 360 export produces a URDF and mesh set, `generate_urdf.py` compiles those into `facehugger.urdf`, and `urdf_to_blender_rigged.py` builds an Armature-based Blender rig from that URDF. Animators author named clips (5-Action bundles) on the rig, export baked joint angles, and the on-board clip player will eventually replay them at 100 Hz with IK and IMU correction. The on-board player replaces the legacy `.gait` format and `kinematics.cpp`; its implementation has not yet started.

## Scene builders vs. animator helpers

The scripts in `animation/scripts/` split into two groups.

**Scene builders** are run as `blender --python ...` and produce a `.blend` for inspection or rigging:

- `visualize_urdf.py` - places meshes by walking the URDF joint chain at rest pose; cross-checks that PyBullet and Blender agree. See [URDF pipeline](urdf-pipeline.md).
- `visualize_fusion_export.py` - places meshes directly from `fusion_export.json` world transforms; useful for diagnosing CAD-side issues.
- `urdf_to_blender_rigged.py` - builds the animator-facing rig: 13-bone Armature, FK shoulder, IK on hip and knee, foot-target Empties. See [Blender rig](blender-rig.md).

**Animator-facing helpers** operate on an open or saved `.blend`:

- `fh_clip_panel.py` - Blender N-panel add-on that manages clips, a pose library, selection sets, and export to `animation/exported_gaits/`. See [Clip panel](clip-panel.md).
- `fh_rename_actions.py` - legacy one-shot migration that renames old Action names to the `base_anim__<target>` convention. Run once per old rig; not part of the day-to-day workflow.
