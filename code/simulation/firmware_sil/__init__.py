"""firmware_sil — drive the PyBullet sim with the EXACT firmware control code.

The `fh_sim` pybind11 module (built from code/firmware/src by CMakeLists.txt)
runs the real firmware SpinalCord on the host; `sil_bridge` converts the servo
angles it produces into URDF joint targets and drives PyBullet. See
EXACT-FIRMWARE-SIL-PLAN.md.

CLI entrypoints (invoked by `code/facehugger.py`):
- `python -m firmware_sil.ws_sim`         — WebSocket robot API server
- `python -m firmware_sil.build`          — auto-build the fh_sim pybind module
- `python -m firmware_sil.gen_references` — regenerate reference clip traces

`run_clip_sil` / `run_gait_sil` are the only clip/gait playback paths;
the Python-port alternatives (`pybullet_sim.run_clip` / `run_gait`) were
removed. The SIL run loops live here so that `pybullet_sim` only depends
on `firmware_sil` via the top-level dispatcher (one-way edge:
firmware_sil → pybullet_sim).
"""

from .sil_bridge import (
    FirmwareSILDriver,
    run_clip_sil,
    run_gait_sil,
    servo_angles_to_joint_targets,
)

__all__ = [
    "FirmwareSILDriver",
    "run_clip_sil",
    "run_gait_sil",
    "servo_angles_to_joint_targets",
]
