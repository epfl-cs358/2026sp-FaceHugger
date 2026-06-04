"""test_urdf_mass_audit.py — Verify URDF mass is within expected range."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pybullet_sim.paths import URDF_PATH


@pytest.fixture(scope="module")
def urdf_masses():
    """Parse the URDF and return {link_name: mass_kg}."""
    urdf_path = Path(URDF_PATH)
    if not urdf_path.exists():
        pytest.skip(f"URDF not found: {urdf_path}")
    root = ET.parse(str(urdf_path)).getroot()
    masses = {}
    for link in root.findall("link"):
        name = link.get("name")
        mass_el = link.find("inertial/mass")
        if mass_el is not None:
            masses[name] = float(mass_el.get("value", 0))
    return masses


def test_urdf_exists():
    """The generated URDF must exist."""
    assert Path(URDF_PATH).exists(), f"Missing URDF: {URDF_PATH}. Run: python code/facehugger.py urdf"


def test_urdf_has_mass_entries(urdf_masses):
    """Every link must have a non-zero mass."""
    for name, mass in urdf_masses.items():
        assert mass > 0, f"{name} has zero or negative mass"


def test_urdf_total_mass_in_range(urdf_masses):
    """Total URDF mass must be between 2.0 kg and 6.0 kg."""
    total = sum(urdf_masses.values())
    assert 2.0 <= total <= 6.0, f"Total URDF mass {total:.3f} kg outside expected range (2.0–6.0)"


def test_base_link_heaviest(urdf_masses):
    """The base_link (chassis) should be the heaviest single link."""
    max_mass = max(urdf_masses.values())
    assert urdf_masses.get("base_link", 0) == max_mass, \
        f"base_link ({urdf_masses.get('base_link')}) not heaviest; max is {max_mass}"


def test_all_link1_have_servo_mass(urdf_masses):
    """Every leg's link1 (shoulder bracket + hip servo) must be > 0.15 kg."""
    for leg in ("fl", "fr", "bl", "br"):
        name = f"{leg}_link1"
        assert name in urdf_masses, f"Missing link: {name}"
        assert urdf_masses[name] > 0.15, \
            f"{name} mass {urdf_masses[name]:.3f} kg too light (expected servo contribution)"


def test_all_link3_have_servo_mass(urdf_masses):
    """Every leg's link3 (lower leg + knee servo) must be > 0.15 kg."""
    for leg in ("fl", "fr", "bl", "br"):
        name = f"{leg}_link3"
        assert name in urdf_masses, f"Missing link: {name}"
        assert urdf_masses[name] > 0.15, \
            f"{name} mass {urdf_masses[name]:.3f} kg too light (expected servo contribution)"


def test_13_links_total(urdf_masses):
    """There should be exactly 13 links: 1 base + 4 legs × 3 links."""
    assert len(urdf_masses) == 13, f"Expected 13 links, found {len(urdf_masses)}"
