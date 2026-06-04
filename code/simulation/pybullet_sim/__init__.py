"""PyBullet runtime for the FaceHugger sim — gaits, kinematics, clip playback.

The CLI entrypoint is `python -m pybullet_sim.simulate` (invoked by
`code/facehugger.py sim`). This `__init__` defines the public Python API
for in-process callers (other packages, tests, future scripts).

The pybullet-dependent submodules (motor, runner, scene, sim_monitor) are
imported lazily so that pure-geometry callers (urdf_io, kinematics) and CI
test suites can `from pybullet_sim import build_config` (etc.) without
pybullet installed. Importing a pybullet-dependent symbol triggers the
underlying module load and raises ImportError if pybullet is missing.
"""

import importlib
from typing import Any

from .kinematics import LegGeom, RobotConfig, build_config

# Symbol -> submodule name, for lazy loading. Touching any of these
# attributes triggers `from .<module> import <symbol>` on first access.
_LAZY: dict[str, str] = {
    "apply_joint_targets": "motor",
    "apply_leg_pose": "motor",
    "build_joint_map": "motor",
    "reset_to_stance": "motor",
    "run_stand": "runner",
    "connect_and_setup": "scene",
    "print_banner": "scene",
    "settle": "scene",
    "SimLogger": "sim_monitor",
    "band": "sim_monitor",
    "format_status": "sim_monitor",
    "read_joint_pos_deg": "sim_monitor",
    "read_joint_torques": "sim_monitor",
}


def __getattr__(name: str) -> Any:
    """PEP 562 lazy attribute access — pulls pybullet-dependent submodules
    on first reference so importing the package without pybullet still works
    for the pure-geometry symbols (LegGeom / RobotConfig / build_config)."""
    submodule = _LAZY.get(name)
    if submodule is None:
        raise AttributeError(f"module 'pybullet_sim' has no attribute {name!r}")
    mod = importlib.import_module(f".{submodule}", __name__)
    value = getattr(mod, name)
    globals()[name] = value  # cache on the module so subsequent access is direct
    return value


__all__ = [
    "LegGeom",
    "RobotConfig",
    "SimLogger",
    "apply_joint_targets",
    "apply_leg_pose",
    "band",
    "build_config",
    "build_joint_map",
    "connect_and_setup",
    "format_status",
    "print_banner",
    "read_joint_pos_deg",
    "read_joint_torques",
    "reset_to_stance",
    "run_stand",
    "settle",
]
