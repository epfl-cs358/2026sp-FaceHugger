"""config_loader.py — Pure-Python YAML config parser for robot_config.yaml.

No adsk dependency; safe to import and test outside of Fusion 360.
"""

import yaml


def load_robot_config(yaml_path: str) -> dict:
    """Load and validate robot_config.yaml.

    Returns a typed config dict with convenience keys added:
      - ``physics_overrides_by_component``: dict keyed by the ``component``
        field of each ``physics_overrides[]`` entry.
      - ``joints_whitelist``: set of ``fusion_name`` values from ``joints[]``.

    Raises:
        ValueError: if ``links`` or ``joints`` top-level keys are absent.
    """
    with open(yaml_path) as fh:
        cfg = yaml.safe_load(fh) or {}

    for required in ("links", "joints"):
        if required not in cfg:
            raise ValueError(f"robot_config.yaml is missing required key: '{required}'")

    # Convenience: physics_overrides indexed by component name
    overrides_list = cfg.get("physics_overrides", []) or []
    cfg["physics_overrides_by_component"] = {
        entry["component"]: entry for entry in overrides_list if "component" in entry
    }

    # Convenience: set of joint fusion_name values for fast membership tests
    cfg["joints_whitelist"] = {
        j["fusion_name"] for j in cfg["joints"] if "fusion_name" in j
    }

    return cfg
