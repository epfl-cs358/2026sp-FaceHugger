"""Structure tests for the reorganized layout (pybullet_sim/ + urdf_gen/).

RED before the migration (the packages don't exist yet), GREEN after. They pin
the package boundaries the reorg introduces: the runtime is importable as
`pybullet_sim`, the firmware-faithful clip re-port as a top-level `firmware_port`
package (relocated from `pybullet_sim/interpreter/` in Step 0 of the SIL plan),
the URDF build step as `urdf_gen`, and `python -m pybullet_sim.simulate` is a
working entry point. Run from code/simulation/.

Requires pybullet (conda env `facehugger`).
"""

import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("pybullet")

SIM_DIR = Path(__file__).resolve().parent.parent  # tests/ -> code/simulation/


def _run(args, timeout=60):
    return subprocess.run(
        [sys.executable, *[str(a) for a in args]],
        cwd=str(SIM_DIR),
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def test_runtime_package_imports():
    r = _run(
        [
            "-c",
            "import pybullet_sim.gaits, pybullet_sim.kinematics, "
            "pybullet_sim.helpers, pybullet_sim.constants, pybullet_sim.sim_monitor",
        ]
    )
    assert r.returncode == 0, r.stderr


def test_firmware_port_package_imports():
    r = _run(
        [
            "-c",
            "from firmware_port import servo_convention, clip_loader, clip_player",
        ]
    )
    assert r.returncode == 0, r.stderr


def test_urdf_gen_package_imports():
    r = _run(["-c", "import urdf_gen.generate_urdf"])
    assert r.returncode == 0, r.stderr


def test_simulate_module_entry_point():
    r = _run(
        ["-m", "pybullet_sim.simulate", "--clip", "wave", "--headless", "--settle", "0"]
    )
    assert r.returncode == 0, r.stderr
    assert "wave" in r.stdout
