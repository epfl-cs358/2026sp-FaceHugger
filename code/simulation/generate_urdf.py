"""
generate_urdf.py  —  FaceHugger URDF generator

Reads:
  fusion_export.json     (from ExportBodiesToURDF Fusion script)
  facehugger_config.yaml (robot hierarchy definition)

Writes:
  facehugger.urdf        (alongside this script, or --out path)

Usage:
  uv run generate_urdf.py
  uv run generate_urdf.py --export ~/Desktop/fusion_export.json
  uv run generate_urdf.py --config facehugger_config.yaml --out ../simulation/facehugger.urdf
"""

import argparse
import json
import math
from pathlib import Path

try:
    import yaml
except ImportError:
    raise SystemExit("PyYAML missing — run: uv add pyyaml")


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

SCRIPT_DIR = Path(__file__).parent
DEFAULT_JSON = Path.home() / "Desktop" / "fusion_export.json"
DEFAULT_CFG = SCRIPT_DIR / "facehugger_config.yaml"
DEFAULT_OUT = SCRIPT_DIR / "facehugger.urdf"

MM_TO_M = 1e-3


# ---------------------------------------------------------------------------
# Fusion export helpers
# ---------------------------------------------------------------------------


def load_export(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)


def find_occurrence(nodes: list, name: str) -> dict | None:
    """Recursively find an occurrence by name in the export tree."""
    for n in nodes:
        if n["name"] == name:
            return n
        found = find_occurrence(n.get("children", []), name)
        if found:
            return found
    return None


def find_axis(occ: dict, key: str) -> dict | None:
    return next((a for a in occ.get("axes", []) if a["name"] == key), None)


def find_point(occ: dict, key: str) -> dict | None:
    return next((p for p in occ.get("points", []) if p["name"] == key), None)


def find_point_in_tree(nodes: list, key: str) -> list | None:
    """Find a named construction point anywhere in the tree, return pos_mm."""
    for n in nodes:
        for p in n.get("points", []):
            if p["name"] == key:
                return p["pos_mm"]
        result = find_point_in_tree(n.get("children", []), key)
        if result:
            return result
    return None


# ---------------------------------------------------------------------------
# Math
# ---------------------------------------------------------------------------


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def scale(v, s):
    return [x * s for x in v]


def deg2rad(d):
    return round(math.radians(d), 6)


def fmt_xyz(mm_vec):
    """Format an mm vector as a URDF xyz string in meters."""
    return " ".join(f"{v * MM_TO_M:.6f}" for v in mm_vec)


def fmt_rpy(r, p, y_deg):
    return f"0 0 {deg2rad(y_deg)}"


# ---------------------------------------------------------------------------
# URDF XML helpers
# ---------------------------------------------------------------------------


class URDF:
    def __init__(self, robot_name: str):
        self.lines = [
            '<?xml version="1.0" ?>',
            f'<robot name="{robot_name}">',
        ]

    def raw(self, line: str):
        self.lines.append(line)

    def comment(self, text: str):
        self.lines.append(f"\n  <!-- {text} -->")

    def link(
        self,
        name: str,
        mesh: str,
        mesh_dir: str,
        mass: float,
        com_mm: list,
        inertia: dict,
    ):
        xyz_com = fmt_xyz(com_mm)
        ixx = inertia.get("ixx", 1e-6)
        iyy = inertia.get("iyy", 1e-6)
        izz = inertia.get("izz", 1e-6)
        ixy = inertia.get("ixy", 0)
        iyz = inertia.get("iyz", 0)
        ixz = inertia.get("ixz", 0)
        self.lines += [
            f'  <link name="{name}">',
            "    <inertial>",
            f'      <origin xyz="{xyz_com}" rpy="0 0 0"/>',
            f'      <mass value="{mass:.6f}"/>',
            f'      <inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}"'
            f' iyy="{iyy}" iyz="{iyz}" izz="{izz}"/>',
            "    </inertial>",
            "    <visual>",
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            "      <geometry>",
            f'        <mesh filename="{mesh_dir}{mesh}" scale="0.001 0.001 0.001"/>',
            "      </geometry>",
            "    </visual>",
            "    <collision>",
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            "      <geometry>",
            f'        <mesh filename="{mesh_dir}{mesh}" scale="0.001 0.001 0.001"/>',
            "      </geometry>",
            "    </collision>",
            "  </link>",
        ]

    def joint(
        self,
        name: str,
        jtype: str,
        parent: str,
        child: str,
        origin_mm: list,
        axis: list,
        lower_deg: float,
        upper_deg: float,
        effort: float,
        velocity: float,
        rpy_z_deg: float = 0,
    ):
        xyz = fmt_xyz(origin_mm)
        rpy = fmt_rpy(0, 0, rpy_z_deg)
        ax = " ".join(str(v) for v in axis)
        lo = deg2rad(lower_deg)
        hi = deg2rad(upper_deg)
        self.lines += [
            f'  <joint name="{name}" type="{jtype}">',
            f'    <parent link="{parent}"/>',
            f'    <child link="{child}"/>',
            f'    <origin xyz="{xyz}" rpy="{rpy}"/>',
            f'    <axis xyz="{ax}"/>',
            f'    <limit lower="{lo}" upper="{hi}"'
            f' effort="{effort}" velocity="{velocity}"/>',
            "  </joint>",
        ]

    def save(self, path: Path):
        self.lines.append("</robot>")
        path.write_text("\n".join(self.lines))
        print(f"Written: {path}")


# ---------------------------------------------------------------------------
# Physics fallback
# ---------------------------------------------------------------------------

FALLBACK_INERTIA = {
    "ixx": 1e-5,
    "iyy": 1e-5,
    "izz": 1e-5,
    "ixy": 0.0,
    "iyz": 0.0,
    "ixz": 0.0,
}


def get_physics(occ_node: dict | None, fallback_mass: float = 0.05):
    if occ_node and "physics" in occ_node and "error" not in occ_node["physics"]:
        ph = occ_node["physics"]
        return ph["mass_kg"], ph["com_mm"], ph["inertia_kg_m2"]
    return fallback_mass, [0, 0, 0], FALLBACK_INERTIA


# ---------------------------------------------------------------------------
# Generator
# ---------------------------------------------------------------------------


def generate(export: dict, cfg: dict, out_path: Path):
    occs = export["occurrences"]
    robot = cfg["robot_name"]
    mesh_dir = cfg["mesh_dir"]
    servo_cfg = cfg.get("servo", {})
    effort = servo_cfg.get("effort_nm", 1.47)
    vel = servo_cfg.get("velocity_rad_s", 5.0)

    urdf = URDF(robot)

    # --- base_link ---
    urdf.comment("BASE LINK")
    base_cfg = cfg["base_link"]
    base_occ = find_occurrence(occs, "FlexibleSkeleton:1")
    mass, com, inertia = get_physics(base_occ, fallback_mass=0.5)
    urdf.link(base_cfg["name"], base_cfg["mesh"], mesh_dir, mass, com, inertia)

    # --- leg assembly: look up joint geometry once ---
    leg_tmpl = cfg["leg_template"]
    leg_occ = find_occurrence(occs, leg_tmpl["leg_assembly_occurrence"])
    if not leg_occ:
        raise ValueError(
            f"Leg assembly occurrence not found: {leg_tmpl['leg_assembly_occurrence']}"
        )

    # Resolve joint positions (all in leg-local mm frame)
    joint_defs = []
    for jcfg in leg_tmpl["joints"]:
        axis_data = find_axis(leg_occ, jcfg["axis_key"])
        point_data = find_point(leg_occ, jcfg["point_key"])
        if not axis_data:
            raise ValueError(f"Axis not found in export: {jcfg['axis_key']}")
        if not point_data:
            raise ValueError(f"Point not found in export: {jcfg['point_key']}")
        joint_defs.append({
            **jcfg,
            "origin_local_mm": axis_data[
                "origin_mm"
            ],  # use axis origin as joint position
            "axis_dir": axis_data["dir"],
        })

    # Compute relative offsets between joints (parent-relative, not world-absolute)
    # joint[0]: shoulder — relative to mount point (set per-leg below)
    # joint[1]: hip      — relative to shoulder origin
    # joint[2]: knee     — relative to hip origin
    for i in range(1, len(joint_defs)):
        joint_defs[i]["offset_from_parent_mm"] = sub(
            joint_defs[i]["origin_local_mm"],
            joint_defs[i - 1]["origin_local_mm"],
        )

    # --- 4 leg instances ---
    for leg in cfg["legs"]:
        leg_id = leg["id"]
        rpy_z = leg.get("rpy_z_deg", 0)
        mount_key = leg["mount_point"]

        # Mount point in body frame (from FlexibleSkeleton construction points)
        mount_mm = find_point_in_tree(occs, mount_key)
        if not mount_mm:
            raise ValueError(f"Mount point not found in export: {mount_key}")

        urdf.comment(f"LEG: {leg_id.upper()}")

        # Shoulder joint: origin = mount point in body frame
        sj = joint_defs[0]
        urdf.joint(
            name=f"{leg_id}_{sj['name']}_joint",
            jtype="revolute",
            parent=base_cfg["name"],
            child=f"{leg_id}_link1",
            origin_mm=mount_mm,
            axis=sj["axis_dir"],
            lower_deg=sj["limits_deg"][0],
            upper_deg=sj["limits_deg"][1],
            effort=effort,
            velocity=vel,
            rpy_z_deg=rpy_z,
        )

        # Remaining joints
        for i, jd in enumerate(joint_defs[1:], start=1):
            child_name = f"{leg_id}_link{i + 1}"
            parent_name = f"{leg_id}_link{i}"
            urdf.joint(
                name=f"{leg_id}_{jd['name']}_joint",
                jtype="revolute",
                parent=parent_name,
                child=child_name,
                origin_mm=jd["offset_from_parent_mm"],
                axis=jd["axis_dir"],
                lower_deg=jd["limits_deg"][0],
                upper_deg=jd["limits_deg"][1],
                effort=effort,
                velocity=vel,
            )

        # Links for this leg
        for link_key, link_cfg in leg_tmpl["links"].items():
            link_num = link_key.replace("link", "")
            link_name = f"{leg_id}_link{link_num}"
            link_occ = find_occurrence(occs, link_cfg["occurrence"].split("/")[-1])
            mass, com, inertia = get_physics(link_occ, fallback_mass=0.05)
            urdf.link(link_name, link_cfg["mesh"], mesh_dir, mass, com, inertia)

    urdf.save(out_path)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Generate FaceHugger URDF from Fusion export"
    )
    parser.add_argument(
        "--export",
        type=Path,
        default=DEFAULT_JSON,
        help=f"Path to fusion_export.json (default: {DEFAULT_JSON})",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CFG,
        help=f"Path to facehugger_config.yaml (default: {DEFAULT_CFG})",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT,
        help=f"Output URDF path (default: {DEFAULT_OUT})",
    )
    args = parser.parse_args()

    print(f"Loading export : {args.export}")
    print(f"Loading config : {args.config}")

    export = load_export(args.export)
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    generate(export, cfg, args.out)


if __name__ == "__main__":
    main()
