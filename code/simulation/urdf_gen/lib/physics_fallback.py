"""Inertial-physics fallback for occurrences missing physics data."""

import warnings

FALLBACK_INERTIA = {
    "ixx": 1e-5,
    "iyy": 1e-5,
    "izz": 1e-5,
    "ixy": 0.0,
    "iyz": 0.0,
    "ixz": 0.0,
}


def get_physics(
    occ_node: dict | None, fallback_mass: float = 0.05, comp_name: str = ""
):
    if occ_node and "physics" in occ_node and "error" not in occ_node["physics"]:
        ph = occ_node["physics"]
        return ph["mass_kg"], ph["com_mm"], ph["inertia_kg_m2"]
    warnings.warn(
        f"No physics data for {comp_name!r}; using fallback inertia.",
        stacklevel=2,
    )
    return fallback_mass, [0, 0, 0], FALLBACK_INERTIA
