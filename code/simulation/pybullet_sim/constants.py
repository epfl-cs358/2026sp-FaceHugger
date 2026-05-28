"""Shared constants for the FaceHugger simulation (no behavior)."""

import os

# This module lives in pybullet_sim/; generated/ and the config yaml sit one
# level up in code/simulation/ (shared with urdf_gen).
SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GENERATED_DIR = os.path.join(SIM_ROOT, "generated")
URDF_PATH = os.path.join(GENERATED_DIR, "facehugger.urdf")
CONFIG_YAML = os.path.join(SIM_ROOT, "facehugger_config.yaml")
FUSION_JSON = os.path.join(GENERATED_DIR, "fusion_export.json")
MESH_DIR = os.path.join(GENERATED_DIR, "exported_meshes")

TIMESTEP = 1.0 / 240.0

# Standing-pose hip/knee bend, shared across all 4 legs. Shoulder stance is
# 0 under Convention A (URDF θ=0 already places each leg at its mechanical
# zero — see code/simulation/docs/MERGE_AND_CONVENTION.md).
STANCE_DEG = {
    "hip": -40.0,
    "knee": -60.0,
}
