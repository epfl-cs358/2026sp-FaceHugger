"""Canonical filesystem paths for the simulation package.

This module lives in pybullet_sim/; generated/ and the config yaml sit one
level up in code/simulation/ (shared with urdf_gen).  Import from here rather
than redefining SIM_ROOT or GENERATED_DIR elsewhere in the package.
"""

import os

SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GENERATED_DIR = os.path.join(SIM_ROOT, "generated")
URDF_PATH = os.path.join(GENERATED_DIR, "facehugger.urdf")
CONFIG_YAML = os.path.join(SIM_ROOT, "facehugger_config.yaml")
FUSION_JSON = os.path.join(GENERATED_DIR, "fusion_export.json")
MESH_DIR = os.path.join(GENERATED_DIR, "exported_meshes")
