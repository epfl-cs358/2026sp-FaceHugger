"""FaceHugger simulation pipeline.

Sub-packages:
- `urdf_gen`    — Fusion JSON → URDF generator (`generate_urdf`)
- `pybullet_sim` — PyBullet runtime, gaits, kinematics (`simulate`)
- `firmware_sil` — software-in-the-loop bridge to the compiled firmware
                  (`ws_sim`, `build`, `gen_references`, `sil_bridge`)
- `animation_tools` — Python re-port of the firmware clip/gait math (library only)

`code/facehugger.py` is the single global CLI entrypoint; each sub-package
exposes its own `python -m <subpkg>.<entry>` invocation that `facehugger.py`
dispatches to.
"""
