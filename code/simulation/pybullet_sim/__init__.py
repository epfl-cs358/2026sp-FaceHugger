"""PyBullet runtime for the FaceHugger sim — gaits, kinematics, clip playback.

The CLI entrypoint is `python -m pybullet_sim.simulate` (invoked by
`code/facehugger.py sim`). This `__init__` defines the public Python API
for in-process callers (other packages, tests, future scripts).

pybullet-dependent submodules (motor, runner, scene, sim_monitor) are NOT
imported here so that pure-geometry callers (urdf_io, kinematics) and CI
test suites can import this package without pybullet installed.
"""

from .kinematics import LegGeom, RobotConfig, build_config

__all__ = [
    "LegGeom",
    "RobotConfig",
    "build_config",
]
