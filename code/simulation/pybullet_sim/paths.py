"""Canonical filesystem paths + sim-wide constants for the simulation package.

This module lives in pybullet_sim/; generated/ and the config yaml sit one
level up in code/simulation/ (shared with urdf_gen). Import from here rather
than redefining SIM_ROOT or GENERATED_DIR elsewhere in the package.

TIMESTEP lives here too (formerly in `constants.py`, folded in
Phase 5 of the sim-reorg). Per-leg stance is derived from neutral_pose.h
via _NEUTRAL_HIP_KNEE in kinematics.py.
"""

import os

SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GENERATED_DIR = os.path.join(SIM_ROOT, "generated")
URDF_PATH = os.path.join(GENERATED_DIR, "facehugger.urdf")
CONFIG_YAML = os.path.join(SIM_ROOT, "facehugger_config.yaml")
SIM_CONFIG_YAML = os.path.join(SIM_ROOT, "sim_config.yaml")
FUSION_JSON = os.path.join(GENERATED_DIR, "fusion_export.json")
MESH_DIR = os.path.join(GENERATED_DIR, "exported_meshes")

TIMESTEP = 1.0 / 240.0
