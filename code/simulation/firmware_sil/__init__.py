"""firmware_sil — drive the PyBullet sim with the EXACT firmware control code.

The `fh_sim` pybind11 module (built from code/firmware/src by CMakeLists.txt)
runs the real firmware SpinalCord on the host; `sil_bridge` converts the servo
angles it produces into URDF joint targets and drives PyBullet. See
EXACT-FIRMWARE-SIL-PLAN.md.

CLI entrypoints (invoked by `code/facehugger.py`):
- `python -m firmware_sil.ws_sim`         — WebSocket robot API server
- `python -m firmware_sil.build`          — auto-build the fh_sim pybind module
- `python -m firmware_sil.gen_references` — regenerate reference clip traces

Phase 4 of the sim-reorg will add `run_clip_sil` and `run_gait_sil` to
`sil_bridge` and re-export them from here. Until then, this `__init__`
re-exports only the symbols that already live in `sil_bridge`.
"""

from .sil_bridge import FirmwareSILDriver, servo_angles_to_joint_targets

__all__ = [
    "FirmwareSILDriver",
    "servo_angles_to_joint_targets",
]
