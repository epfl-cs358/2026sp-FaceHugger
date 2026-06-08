"""test_sim_config.py — Verify sim_config.yaml loads correctly."""

import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pybullet_sim.paths import SIM_CONFIG_YAML


def test_sim_config_exists():
    """The config file must exist at the expected path."""
    assert Path(SIM_CONFIG_YAML).exists(), f"Missing: {SIM_CONFIG_YAML}"


def test_sim_config_loads():
    """YAML must parse without error."""
    with open(SIM_CONFIG_YAML) as f:
        cfg = yaml.safe_load(f)
    assert isinstance(cfg, dict)


def test_sim_config_has_all_sections():
    """All expected top-level keys must be present."""
    with open(SIM_CONFIG_YAML) as f:
        cfg = yaml.safe_load(f)
    for key in ("contact", "friction", "solver", "mass"):
        assert key in cfg, f"Missing top-level key: {key}"


def test_contact_section():
    """Contact section must have numeric fields with expected types."""
    with open(SIM_CONFIG_YAML) as f:
        cfg = yaml.safe_load(f)
    c = cfg["contact"]
    for key in ("stiffness", "damping", "restitution"):
        assert key in c, f"Missing contact.{key}"
        assert isinstance(c[key], (int, float)), f"contact.{key} must be numeric"


def test_friction_section():
    """Friction section must have numeric fields."""
    with open(SIM_CONFIG_YAML) as f:
        cfg = yaml.safe_load(f)
    f = cfg["friction"]
    for key in ("foot_lateral", "foot_spinning"):
        assert key in f, f"Missing friction.{key}"
        assert isinstance(f[key], (int, float)), f"friction.{key} must be numeric"


def test_solver_section():
    """Solver section must have integer/numeric fields."""
    with open(SIM_CONFIG_YAML) as f:
        cfg = yaml.safe_load(f)
    s = cfg["solver"]
    for key in ("iterations", "substeps"):
        assert key in s, f"Missing solver.{key}"
        assert isinstance(s[key], (int, float)), f"solver.{key} must be numeric"


def test_mass_section():
    """Mass section must have expected keys."""
    with open(SIM_CONFIG_YAML) as f:
        cfg = yaml.safe_load(f)
    m = cfg["mass"]
    for key in ("servo_mass_g", "use_urdf_inertia", "mass_correction_factor"):
        assert key in m, f"Missing mass.{key}"


def test_contact_values_positive():
    """Stiffness and damping must be positive."""
    with open(SIM_CONFIG_YAML) as f:
        cfg = yaml.safe_load(f)
    assert cfg["contact"]["stiffness"] > 0, "contact.stiffness must be positive"
    assert cfg["contact"]["damping"] >= 0, "contact.damping must be non-negative"


def test_friction_values_reasonable():
    """Friction values must be in a reasonable range."""
    with open(SIM_CONFIG_YAML) as f:
        cfg = yaml.safe_load(f)
    assert 0 <= cfg["friction"]["foot_lateral"] <= 10, "lateral friction out of range"
    assert 0 <= cfg["friction"]["foot_spinning"] <= 5, "spinning friction out of range"


def test_mass_correction_factor_reasonable():
    """Mass correction factor must be in a reasonable range."""
    with open(SIM_CONFIG_YAML) as f:
        cfg = yaml.safe_load(f)
    mf = cfg["mass"]["mass_correction_factor"]
    assert 0.1 <= mf <= 10.0, f"mass_correction_factor {mf} out of range (0.1–10)"
