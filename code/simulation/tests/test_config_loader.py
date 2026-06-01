"""test_config_loader.py — TDD tests for config_loader.load_robot_config.

Path setup: repo root is parents[3] from this file.
  this file: code/simulation/tests/test_config_loader.py
  parents[0]: code/simulation/tests/
  parents[1]: code/simulation/
  parents[2]: code/
  parents[3]: repo root (2026sp-FaceHugger/)
"""

import importlib.util
from pathlib import Path

import pytest

# Load config_loader by file path to avoid colliding with any other "lib"
# package cached in sys.modules (e.g. from test_census_pure).
_CL_PATH = (
    Path(__file__).parents[3]
    / "cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/lib/config_loader.py"
)
_spec = importlib.util.spec_from_file_location("_eb_config_loader", _CL_PATH)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
load_robot_config = _mod.load_robot_config


# ── helpers ───────────────────────────────────────────────────────────────────


def _write(tmp_path, text: str) -> str:
    p = tmp_path / "robot_config.yaml"
    p.write_text(text)
    return str(p)


MINIMAL_YAML = """\
links:
  - urdf_name: base_link
joints:
  - fusion_name: Link1Revolute
    urdf_name: link1_joint
"""


# ── Test 1: valid minimal YAML returns dict with links and joints ──────────────


def test_valid_minimal_returns_links_and_joints(tmp_path):
    cfg = load_robot_config(_write(tmp_path, MINIMAL_YAML))
    assert isinstance(cfg, dict)
    assert "links" in cfg
    assert "joints" in cfg
    assert isinstance(cfg["links"], list)
    assert isinstance(cfg["joints"], list)


# ── Test 2: missing `links` raises ValueError mentioning "links" ──────────────


def test_missing_links_raises(tmp_path):
    yaml_text = """\
joints:
  - fusion_name: Link1Revolute
    urdf_name: link1_joint
"""

    with pytest.raises(ValueError, match="links"):
        load_robot_config(_write(tmp_path, yaml_text))


# ── Test 3: missing `joints` raises ValueError mentioning "joints" ────────────


def test_missing_joints_raises(tmp_path):
    yaml_text = """\
links:
  - urdf_name: base_link
"""

    with pytest.raises(ValueError, match="joints"):
        load_robot_config(_write(tmp_path, yaml_text))


# ── Test 4: physics_overrides_by_component indexed by component field ─────────


def test_physics_overrides_by_component(tmp_path):
    yaml_text = """\
links:
  - urdf_name: base_link
joints:
  - fusion_name: Link1Revolute
    urdf_name: link1_joint
physics_overrides:
  - component: PCB_ESP32
    mass_kg: 0.025
"""
    cfg = load_robot_config(_write(tmp_path, yaml_text))
    assert "physics_overrides_by_component" in cfg
    assert "PCB_ESP32" in cfg["physics_overrides_by_component"]
    assert cfg["physics_overrides_by_component"]["PCB_ESP32"]["mass_kg"] == 0.025


# ── Test 5: joints_whitelist is a set containing fusion_name values ───────────


def test_joints_whitelist_is_set(tmp_path):
    cfg = load_robot_config(_write(tmp_path, MINIMAL_YAML))
    assert isinstance(cfg["joints_whitelist"], set)
    assert "Link1Revolute" in cfg["joints_whitelist"]


# ── Test 6: print_stls section passes through unchanged ──────────────────────


def test_print_stls_passthrough(tmp_path):
    yaml_text = """\
links:
  - urdf_name: base_link
joints:
  - fusion_name: Link1Revolute
    urdf_name: link1_joint
print_stls:
  stl_refinement: high
"""
    cfg = load_robot_config(_write(tmp_path, yaml_text))
    assert cfg["print_stls"]["stl_refinement"] == "high"
