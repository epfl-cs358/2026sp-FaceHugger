"""Pin the public API surface of `pybullet_sim` and `firmware_sil`.

These packages re-export a fixed set of symbols from their submodules so
callers (including `code/facehugger.py` after Phase 8) can write
`from pybullet_sim import run_clip` instead of digging into `runner`.

If a symbol is removed or renamed during the refactor, this test goes RED
before downstream callers do.
"""

import importlib

import pytest

pytest.importorskip("pybullet")


PYBULLET_SIM_API = (
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
)

FIRMWARE_SIL_API = (
    "FirmwareSILDriver",
    "run_clip_sil",
    "run_gait_sil",
    "servo_angles_to_joint_targets",
)


@pytest.mark.parametrize("name", PYBULLET_SIM_API)
def test_pybullet_sim_exports(name: str):
    mod = importlib.import_module("pybullet_sim")
    assert hasattr(mod, name), f"pybullet_sim.{name} missing from public API"


@pytest.mark.parametrize("name", FIRMWARE_SIL_API)
def test_firmware_sil_exports(name: str):
    mod = importlib.import_module("firmware_sil")
    assert hasattr(mod, name), f"firmware_sil.{name} missing from public API"


def test_pybullet_sim_all_matches_imports():
    """__all__ must match what the module actually exposes."""
    mod = importlib.import_module("pybullet_sim")
    assert set(mod.__all__) == set(PYBULLET_SIM_API)


def test_firmware_sil_all_matches_imports():
    mod = importlib.import_module("firmware_sil")
    assert set(mod.__all__) == set(FIRMWARE_SIL_API)
