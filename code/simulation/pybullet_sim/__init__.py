"""PyBullet runtime for the FaceHugger sim — gaits, kinematics, clip playback.

The CLI entrypoint is `python -m pybullet_sim.simulate` (invoked by
`code/facehugger.py sim`). This `__init__` defines the public Python API
for in-process callers (other packages, tests, future scripts).
"""

from .kinematics import LegGeom, RobotConfig, build_config
from .motor import (
    apply_joint_targets,
    apply_leg_pose,
    build_joint_map,
    reset_to_stance,
)
from .runner import run_clip, run_gait, run_stand
from .scene import connect_and_setup, print_banner, settle
from .sim_monitor import (
    SimLogger,
    band,
    format_status,
    read_joint_pos_deg,
    read_joint_torques,
)

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
    "run_clip",
    "run_gait",
    "run_stand",
    "settle",
]
