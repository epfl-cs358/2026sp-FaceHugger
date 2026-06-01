"""Canonical filesystem paths + sim-wide constants for the simulation package.

This module lives in pybullet_sim/; generated/ and the config yaml sit one
level up in code/simulation/ (shared with urdf_gen). Import from here rather
than redefining SIM_ROOT or GENERATED_DIR elsewhere in the package.

TIMESTEP and STANCE_DEG live here too (formerly in `constants.py`, folded in
Phase 5 of the sim-reorg). One source of truth for module-level constants.
"""

import os

SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GENERATED_DIR = os.path.join(SIM_ROOT, "generated")
URDF_PATH = os.path.join(GENERATED_DIR, "facehugger.urdf")
CONFIG_YAML = os.path.join(SIM_ROOT, "facehugger_config.yaml")
FUSION_JSON = os.path.join(GENERATED_DIR, "fusion_export.json")
MESH_DIR = os.path.join(GENERATED_DIR, "exported_meshes")

TIMESTEP = 1.0 / 240.0

# Standing-pose hip/knee bend, shared across all 4 legs. Shoulder stance is
# 0 because the URDF bakes each leg's rest yaw into the joint <origin rpy>,
# so θ=0 already lands at the mechanical zero (the splayed rest pose).
STANCE_DEG = {
    "hip": -40.0,
    "knee": -60.0,
}
