"""Shared constants for the FaceHugger simulation (no behavior)."""

from .paths import (  # noqa: F401
    CONFIG_YAML,
    FUSION_JSON,
    GENERATED_DIR,
    MESH_DIR,
    SIM_ROOT,
    URDF_PATH,
)

TIMESTEP = 1.0 / 240.0

# Standing-pose hip/knee bend, shared across all 4 legs. Shoulder stance is
# 0 because the URDF bakes each leg's rest yaw into the joint <origin rpy>,
# so θ=0 already lands at the mechanical zero (the splayed rest pose).
STANCE_DEG = {
    "hip": -40.0,
    "knee": -60.0,
}
