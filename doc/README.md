# FaceHugger — project wiki

## 1. CAD (Fusion 360)

→ see also: [cad/scripts/ExportBodiesToURDF/](../cad/scripts/ExportBodiesToURDF/), [code/simulation/docs/PIPELINE_SPEC.md](../code/simulation/docs/PIPELINE_SPEC.md)

TODO: how the Fusion assembly is structured; the ExportBodiesToURDF add-in; what `EXPORT_RULES` controls; how to re-run the exporter and what it produces.

---

## 2. URDF generation

→ see also: [code/simulation/docs/PIPELINE_SPEC.md](../code/simulation/docs/PIPELINE_SPEC.md), [code/simulation/docs/MERGE_AND_CONVENTION.md](../code/simulation/docs/MERGE_AND_CONVENTION.md)

TODO: how `urdf_gen/generate_urdf.py` consumes `fusion_export.json` + `facehugger_config.yaml`; mesh re-origining; per-leg shoulder convention; what to never hand-edit in `generated/`.

---

## 3. PyBullet simulation

→ see also: [code/simulation/README.md](../code/simulation/README.md)

TODO: `facehugger.py sim` / `--walk` / `--trot` / `--headless`; how `pybullet_sim/kinematics.py` reads geometry from the URDF; gait registry in `pybullet_sim/gaits.py`.

---

## 4. Blender pipeline

→ see also: [animation/scripts/README.md](../animation/scripts/README.md), [code/simulation/docs/API_ANIMATION_SPEC.md](../code/simulation/docs/API_ANIMATION_SPEC.md)

TODO: rigged vs placement-only; `visualize_fusion_export.py` for CAD cross-check; how to rebuild the rig with `facehugger.py blender --rigged`.

---

## 5. Animation authoring (clip panel)

→ see also: [animation/addons/README.md](../animation/addons/README.md)

TODO: clip naming convention; layered Action API; duplicate/rename workflow; pose library; exporting to CSV / `.h` / `.js`.

---

## 6. Firmware playback

→ see also: [code/firmware/src/nervous_system/README.md](../code/firmware/src/nervous_system/README.md), [doc/animation-pipeline/leg-coordinates.md](animation-pipeline/leg-coordinates.md)

TODO: ESP32 80/20 hardcode-pose philosophy; `tickGait` and `translateToServo`; upcoming `.fhc` on-board animation engine.

---

## 7. Mobile remote control

→ see also: [code/API_SPEC.md](../code/API_SPEC.md), [code/remote-control-app/MyApp/](../code/remote-control-app/MyApp/)

TODO: WebSocket JSON protocol; Expo app structure; Zustand state; how to connect to the robot's Wi-Fi AP.

---

## 8. Conventions reference

→ see also: [code/simulation/docs/MERGE_AND_CONVENTION.md](../code/simulation/docs/MERGE_AND_CONVENTION.md), [animation/SERVO_ID_CONVENTION.md](../animation/SERVO_ID_CONVENTION.md)

TODO: leg naming (`fl`/`fr`/`bl`/`br` in URDF/Python vs `fr`/`fl`/`rr`/`rl` in firmware); servo numbering; body-frame orientation (+Y forward, +X right, +Z up).
