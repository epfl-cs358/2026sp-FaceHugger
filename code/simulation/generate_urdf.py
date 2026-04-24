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
import struct
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


def _foot_tip_from_stl(stl_path: Path, y_tol_mm: float = 2.0) -> list:
    """Centroid (in mm) of leg_lower STL vertices within y_tol_mm of max Y.

    The exporter writes STLs in the leg-assembly-local frame. The lower-leg
    mesh's far +Y end is the foot tip; the centroid of the few vertices at the
    max-Y plane gives a stable tip estimate until a dedicated FootTipPoint
    construction point exists in Fusion.
    """
    with open(stl_path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        verts = []
        y_max = float("-inf")
        for _ in range(n):
            f.read(12)
            for _ in range(3):
                v = struct.unpack("<fff", f.read(12))
                verts.append(v)
                if v[1] > y_max:
                    y_max = v[1]
            f.read(2)
    tip_pts = [v for v in verts if v[1] >= y_max - y_tol_mm]
    return [
        round(sum(v[0] for v in tip_pts) / len(tip_pts), 3),
        round(sum(v[1] for v in tip_pts) / len(tip_pts), 3),
        round(sum(v[2] for v in tip_pts) / len(tip_pts), 3),
    ]


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
        origin_shift_mm: list | None = None,
        extra_visuals: list | None = None,
    ):
        """Emit a <link> block.

        `origin_shift_mm` is the same value that ExportBodiesToURDF wrote to
        `mesh_files[mesh].origin_shift_mm` — the landmark position (in the
        original mesh frame) that was subtracted from every vertex during
        re-origin. The URDF link frame origin now coincides with the mesh's
        local (0,0,0), so the visual/collision mesh <origin> is (0,0,0). The
        CoM was reported by Fusion in the pre-shift frame, so it gets
        corrected by subtracting `origin_shift_mm`.

        `extra_visuals` is a list of tuples for additional <visual> blocks on
        this link (e.g. servo meshes glued to each leg's base_link / link2
        / link3). Each tuple is either `(mesh_name, origin_xyz_mm)` (rpy
        defaults to (0,0,0)) or `(mesh_name, origin_xyz_mm, origin_rpy_rad)`.
        No collision and no inertial contribution — visual-only.
        """
        if origin_shift_mm is None:
            origin_shift_mm = [0.0, 0.0, 0.0]
        if extra_visuals is None:
            extra_visuals = []
        com_local = [com_mm[i] - origin_shift_mm[i] for i in range(3)]
        xyz_com = fmt_xyz(com_local)
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
        ]
        for extra in extra_visuals:
            if len(extra) == 2:
                extra_mesh, extra_xyz_mm = extra
                extra_rpy = (0.0, 0.0, 0.0)
            else:
                extra_mesh, extra_xyz_mm, extra_rpy = extra
            rpy_str = " ".join(f"{v:.6f}" for v in extra_rpy)
            self.lines += [
                "    <visual>",
                f'      <origin xyz="{fmt_xyz(extra_xyz_mm)}" rpy="{rpy_str}"/>',
                "      <geometry>",
                f'        <mesh filename="{mesh_dir}{extra_mesh}" scale="0.001 0.001 0.001"/>',
                "      </geometry>",
                "    </visual>",
            ]
        self.lines += [
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


def _mesh_shift(export: dict, mesh_name: str) -> list:
    """Return the origin_shift_mm that ExportBodiesToURDF applied to this
    STL during re-origin (or [0,0,0] if the mesh wasn't re-origined or the
    manifest is missing). This is the exact value to pass as
    URDF.link(..., origin_shift_mm=...) so the inertial CoM gets shifted
    back into the URDF link frame while the visual mesh sits at (0,0,0).
    """
    mf = export.get("mesh_files") or {}
    entry = mf.get(mesh_name) or {}
    return list(entry.get("origin_shift_mm", [0.0, 0.0, 0.0]))


def _find_occ_by_path(occs: list, path: str) -> dict | None:
    """Path like 'A:1/B:1/C:1'. Returns the occurrence dict or None."""
    parts = path.split("/")
    current = occs
    node = None
    for part in parts:
        node = next((c for c in current if c.get("name") == part), None)
        if node is None:
            return None
        current = node.get("children", [])
    return node


def _servo_world_by_role(export: dict, role: str) -> list | None:
    """Look up the world_origin_mm of the servo instance assigned to `role`
    (shoulder/hip/knee) via mesh_files._servo_role_assignment. Returns None
    if no assignment or occurrence missing — caller should skip that visual.
    """
    mf = export.get("mesh_files") or {}
    roles = mf.get("_servo_role_assignment") or {}
    path = roles.get(role)
    if not path:
        return None
    occ = _find_occ_by_path(export["occurrences"], path)
    if occ is None:
        return None
    return occ.get("world_origin_mm")


def _servo_rot_by_role(export: dict, role: str) -> list | None:
    """3x3 rotation of the servo occurrence assigned to `role`, from its
    world_transform_rm_cm (dimensionless — rotation part of a 4x4)."""
    mf = export.get("mesh_files") or {}
    roles = mf.get("_servo_role_assignment") or {}
    path = roles.get(role)
    if not path:
        return None
    occ = _find_occ_by_path(export["occurrences"], path)
    if occ is None:
        return None
    wtf = occ.get("world_transform_rm_cm")
    if wtf is None:
        return None
    return [row[:3] for row in wtf[:3]]


def _find_occ_rot(occs: list, name: str) -> list | None:
    """Find an occurrence by leaf name anywhere in the tree and return its
    3x3 world rotation (from world_transform_rm_cm)."""
    for n in _iter_occ(occs):
        if n.get("name") == name:
            wtf = n.get("world_transform_rm_cm")
            if wtf:
                return [row[:3] for row in wtf[:3]]
    return None


# --- small 3x3 matrix helpers ------------------------------------------------

def _mat_transpose_3x3(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def _mat_mul_3x3(a, b):
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(3))
    return out


def _rot_to_urdf_rpy(r) -> tuple:
    """Convert a 3x3 rotation matrix to URDF-convention RPY (roll, pitch, yaw)
    radians, where R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Handles gimbal-lock
    at |pitch| = π/2 with the roll=0 convention."""
    # sin(pitch) = -r[2][0]   (from the ZYX decomposition)
    sp = -r[2][0]
    sp = max(-1.0, min(1.0, sp))   # clip tiny FP overshoot
    if abs(sp) > 1.0 - 1e-9:
        pitch = math.copysign(math.pi / 2, sp)
        roll = 0.0
        yaw = math.atan2(-r[0][1], r[1][1])
    else:
        pitch = math.asin(sp)
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    return (roll, pitch, yaw)


def _link_rot_world(rpy_z_deg: float) -> list:
    """Rotation of link1/link2/link3's URDF frame at joint=0 in world.
    All three share the same rotation because hip/knee joint origins have
    rpy=0 (translation only). Just Rz(rpy_z_deg)."""
    c = math.cos(math.radians(rpy_z_deg))
    s = math.sin(math.radians(rpy_z_deg))
    return [
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _euler_to_rot(rpy_rad) -> list:
    """URDF-convention Euler → 3x3: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy_rad
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,      cp * sr,                 cp * cr],
    ]


def generate(export: dict, cfg: dict, out_path: Path):
    occs = export["occurrences"]
    robot = cfg["robot_name"]
    mesh_dir = cfg["mesh_dir"]
    servo_cfg = cfg.get("servo", {})
    effort = servo_cfg.get("effort_nm", 1.47)
    vel = servo_cfg.get("velocity_rad_s", 5.0)
    # Uniform mesh-local rotation applied to every servo visual (see yaml
    # comment on `servo.visual_flip_rpy_deg`). Defaults to identity.
    servo_flip_deg = servo_cfg.get("visual_flip_rpy_deg", [0.0, 0.0, 0.0])
    servo_flip_rot = _euler_to_rot([math.radians(d) for d in servo_flip_deg])

    urdf = URDF(robot)

    # --- leg assembly: look up joint geometry once ---
    # Pulled up so servo visuals on base_link can reference the template
    # leg's shoulder-point world position before base_link is emitted.
    leg_tmpl = cfg["leg_template"]
    leg_occ = find_occurrence(occs, leg_tmpl["leg_assembly_occurrence"])
    if not leg_occ:
        raise ValueError(
            f"Leg assembly occurrence not found: {leg_tmpl['leg_assembly_occurrence']}"
        )

    # Resolve joint positions in LAL (leg-assembly-local) mm frame. We use the
    # construction POINT (not the axis origin) — see historical note: on
    # BodyToLink1 the axis origin sits at z=-0.2mm and the Point at z=-4.2mm;
    # aligning on the Point matches the CAD mating face the designer placed
    # to coincide with the body's LegMountXX.
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
            "origin_local_mm": point_data["pos_mm"],
            "origin_world_mm": point_data.get("pos_world_mm") or point_data["pos_mm"],
            "axis_dir": axis_data["dir"],
        })

    # Offsets between successive joints, in LAL frame (== URDF link frame
    # because rpy_z_deg matches the leg-assembly's LAL Z rotation for every
    # leg — R_L1 = I — so no rotation applied to these deltas).
    for i in range(1, len(joint_defs)):
        joint_defs[i]["offset_from_parent_mm"] = sub(
            joint_defs[i]["origin_local_mm"],
            joint_defs[i - 1]["origin_local_mm"],
        )

    # --- servo placement ---
    # servo.stl is re-origined in the Fusion exporter to `ServoMountPoint` on
    # the shaft-exit/mounting-face plane, which coincides with each servo's
    # joint rotation axis by CAD construction. So each servo mesh's local
    # (0,0,0) lands on the joint axis when placed at the joint world
    # position. What DOES vary per instance is orientation — the servo body
    # is rotated differently on each corner (LegMount brackets have
    # different world rotations) and on hip/knee sites (vs the link frames
    # they attach to). We emit a per-instance <origin rpy> derived from the
    # CAD's occurrence world rotations.
    has_shoulder_servo = _servo_world_by_role(export, "shoulder") is not None
    has_hip_servo      = _servo_world_by_role(export, "hip")      is not None
    has_knee_servo     = _servo_world_by_role(export, "knee")     is not None

    servo_mesh_name = None
    for mesh_name, entry in (export.get("mesh_files") or {}).items():
        if entry.get("source_body") == "ServoBase":
            servo_mesh_name = mesh_name
            break

    # --- base_link ---
    urdf.comment("BASE LINK")
    base_cfg = cfg["base_link"]
    base_occ = find_occurrence(occs, "FlexibleSkeleton:1")
    mass, com, inertia = get_physics(base_occ, fallback_mass=0.5)

    # Shoulder-servo visuals: 4 copies on base_link, one per leg. The CAD
    # has one physical shoulder servo (on the FL bracket); the other 3
    # corners' servos are virtual, inheriting the orientation of their
    # corner's LegMount bracket. For each leg N:
    #
    #   R_servoN = R_LegMountN @ R_LegMountFL⁻¹ @ R_source_shoulder_servo
    #
    # base_link is at world identity, so rpy for the visual is
    # _rot_to_urdf_rpy(R_servoN).
    base_extra_visuals = []
    source_shoulder_rot = _servo_rot_by_role(export, "shoulder")
    # Identify the FL leg config + its LegMount rotation (the source leg).
    source_leg_entry = next((lg for lg in cfg["legs"] if lg["id"] == "fl"), None)
    source_leg_mount = source_leg_entry["mount_point"] if source_leg_entry else None
    source_lm_rot = _find_occ_rot(occs, f"{source_leg_mount}:1") if source_leg_mount else None

    if has_shoulder_servo and servo_mesh_name:
        for leg in cfg["legs"]:
            mount_mm = find_point_in_tree(occs, leg["mount_point"])
            if not mount_mm:
                continue
            # Per-leg rpy: compose LegMountN @ LegMountFL⁻¹ @ source_shoulder_rot.
            # Falls back to identity (rpy=0) if any rotation is missing.
            if source_shoulder_rot and source_lm_rot:
                lm_rot = _find_occ_rot(occs, f"{leg['mount_point']}:1")
                if lm_rot:
                    r_servo = _mat_mul_3x3(
                        _mat_mul_3x3(lm_rot, _mat_transpose_3x3(source_lm_rot)),
                        source_shoulder_rot,
                    )
                    # Post-multiply by the config's uniform flip so the flip
                    # is applied in the mesh's local frame.
                    r_servo = _mat_mul_3x3(r_servo, servo_flip_rot)
                    rpy = _rot_to_urdf_rpy(r_servo)
                else:
                    rpy = _rot_to_urdf_rpy(servo_flip_rot)
            else:
                rpy = _rot_to_urdf_rpy(servo_flip_rot)
            base_extra_visuals.append((servo_mesh_name, list(mount_mm), rpy))

    urdf.link(
        base_cfg["name"], base_cfg["mesh"], mesh_dir, mass, com, inertia,
        origin_shift_mm=_mesh_shift(export, base_cfg["mesh"]),
        extra_visuals=base_extra_visuals,
    )

    # Emit a machine-parseable comment block with the raw leg-assembly-local
    # Points. Downstream tools (simulate_v2) read this to avoid re-opening
    # fusion_export.json — the URDF remains the single source of truth for
    # the leg chain geometry. After re-origin, leg_lower.stl's max +Y is
    # still the foot tip but now relative to Link2ToLink3Point (the new
    # local origin), not the leg-assembly origin — the centroid calculation
    # still works since we only need the furthest-+Y vertex cluster.
    tip_mm = _foot_tip_from_stl(
        Path(__file__).parent / mesh_dir.rstrip("/\\") / "leg_lower.stl"
    )
    pts_comment = [
        "LEG ASSEMBLY METADATA (mm, leg-assembly-local frame)",
        f"  BodyToLink1Point : {joint_defs[0]['origin_local_mm'][0]:.3f} "
        f"{joint_defs[0]['origin_local_mm'][1]:.3f} "
        f"{joint_defs[0]['origin_local_mm'][2]:.3f}",
        f"  Link1ToLink2Point: {joint_defs[1]['origin_local_mm'][0]:.3f} "
        f"{joint_defs[1]['origin_local_mm'][1]:.3f} "
        f"{joint_defs[1]['origin_local_mm'][2]:.3f}",
        f"  Link2ToLink3Point: {joint_defs[2]['origin_local_mm'][0]:.3f} "
        f"{joint_defs[2]['origin_local_mm'][1]:.3f} "
        f"{joint_defs[2]['origin_local_mm'][2]:.3f}",
        f"  FootTip (leg_lower.stl max +Y centroid, in link3 frame): "
        f"{tip_mm[0]:.3f} {tip_mm[1]:.3f} {tip_mm[2]:.3f}",
    ]
    urdf.comment("\n       ".join(pts_comment))

    # --- 4 leg instances ---
    # All legs share rpy_z_deg == leg-assembly's LAL Z rotation, so R_L1 = I:
    # joint offsets and axes are used directly in URDF link frames without
    # rotation. Sides are differentiated downstream via shoulder stance.
    for leg in cfg["legs"]:
        leg_id = leg["id"]
        rpy_z = leg.get("rpy_z_deg", 0)
        mount_key = leg["mount_point"]

        mount_mm = find_point_in_tree(occs, mount_key)
        if not mount_mm:
            raise ValueError(f"Mount point not found in export: {mount_key}")

        # Per-leg shoulder limits: quadrant centered on shoulder_neutral_deg
        # with ±90° swing. Falls back to the template's shared limits if the
        # leg entry didn't specify a neutral.
        sj = joint_defs[0]
        neutral_deg = leg.get("shoulder_neutral_deg")
        if neutral_deg is not None:
            sj_lower = neutral_deg - 90
            sj_upper = neutral_deg + 90
        else:
            sj_lower, sj_upper = sj["limits_deg"]

        urdf.comment(
            f"LEG: {leg_id.upper()}  (rpy_z_deg={rpy_z:+.1f}, "
            f"shoulder neutral={neutral_deg:+.1f}°, "
            f"limits=[{sj_lower:+.1f}°, {sj_upper:+.1f}°])"
            if neutral_deg is not None
            else f"LEG: {leg_id.upper()}  (rpy_z_deg={rpy_z:+.1f})"
        )

        # Shoulder joint: origin = mount point in base_link frame (world).
        urdf.joint(
            name=f"{leg_id}_{sj['name']}_joint",
            jtype="revolute",
            parent=base_cfg["name"],
            child=f"{leg_id}_link1",
            origin_mm=mount_mm,
            axis=sj["axis_dir"],
            lower_deg=sj_lower,
            upper_deg=sj_upper,
            effort=effort,
            velocity=vel,
            rpy_z_deg=rpy_z,
        )

        # Hip + knee joints: LAL deltas become URDF joint origins directly.
        for i, jd in enumerate(joint_defs[1:], start=1):
            urdf.joint(
                name=f"{leg_id}_{jd['name']}_joint",
                jtype="revolute",
                parent=f"{leg_id}_link{i}",
                child=f"{leg_id}_link{i + 1}",
                origin_mm=jd["offset_from_parent_mm"],
                axis=jd["axis_dir"],
                lower_deg=jd["limits_deg"][0],
                upper_deg=jd["limits_deg"][1],
                effort=effort,
                velocity=vel,
            )

        # Links for this leg. Each link's mesh is re-origined at export time
        # so URDF visual/collision <origin>=(0,0,0). Inertial CoM gets shifted
        # by the same origin_shift so it lands in the URDF link frame.
        # link2 and link3 gain an extra_visual for their attached servo. The
        # hip servo's ServoMountPoint coincides with link2's URDF frame
        # origin (= hip joint), so visual xyz = (0,0,0); same for knee on
        # link3. The visual rpy is R_link_world⁻¹ @ R_servo_world, expressed
        # in the host link's frame.
        r_link_world = _link_rot_world(rpy_z)   # = Rz(rpy_z_deg), identical for link2/link3

        def _servo_rpy_in_link(role):
            r_servo = _servo_rot_by_role(export, role)
            if r_servo is None:
                return _rot_to_urdf_rpy(servo_flip_rot)
            r_in_link = _mat_mul_3x3(_mat_transpose_3x3(r_link_world), r_servo)
            # Post-multiply by the config flip (mesh-local rotation).
            r_in_link = _mat_mul_3x3(r_in_link, servo_flip_rot)
            return _rot_to_urdf_rpy(r_in_link)

        hip_rpy  = _servo_rpy_in_link("hip")  if has_hip_servo  else (0.0, 0.0, 0.0)
        knee_rpy = _servo_rpy_in_link("knee") if has_knee_servo else (0.0, 0.0, 0.0)

        per_link_extra = {
            "link1": [],
            "link2": (
                [(servo_mesh_name, [0.0, 0.0, 0.0], hip_rpy)]
                if has_hip_servo and servo_mesh_name
                else []
            ),
            "link3": (
                [(servo_mesh_name, [0.0, 0.0, 0.0], knee_rpy)]
                if has_knee_servo and servo_mesh_name
                else []
            ),
        }
        for link_key, link_cfg in leg_tmpl["links"].items():
            link_num = link_key.replace("link", "")
            link_name = f"{leg_id}_link{link_num}"
            link_occ = find_occurrence(occs, link_cfg["occurrence"].split("/")[-1])
            mass, com, inertia = get_physics(link_occ, fallback_mass=0.05)
            urdf.link(
                link_name, link_cfg["mesh"], mesh_dir, mass, com, inertia,
                origin_shift_mm=_mesh_shift(export, link_cfg["mesh"]),
                extra_visuals=per_link_extra.get(link_key, []),
            )

    urdf.save(out_path)

    _print_invariant_check(occs, cfg, joint_defs)


def _world_origin_of(occs: list, name: str) -> list | None:
    """Find an occurrence by name anywhere in the tree and return its root-
    frame position. Tries the new `world_origin_mm` field first (accumulated
    transform, present after re-running the updated exporter); falls back to
    `parent_origin_mm` or the old same-named field, which for top-level
    children of the root happens to coincide with world anyway.
    """
    for n in _iter_occ(occs):
        if n.get("name") == name:
            return n.get("world_origin_mm") or n.get("parent_origin_mm")
    return None


def _iter_occ(nodes: list):
    for n in nodes:
        yield n
        yield from _iter_occ(n.get("children", []))


def _print_invariant_check(occs: list, cfg: dict, joint_defs: list):
    """Post-generation diagnostic: for each leg, show where its shoulder axis
    lands in world frame (= its LegMountXX construction point) and where
    the matching LegMountXX body sub-occurrence lives. The delta highlights
    whether the visible "leg shoulder floats off tab" gap is geometry-sourced
    in the CAD (large delta) or not.
    """
    print("\n=== Placement invariant check (world frame, mm) ===")
    warned = False
    for leg in cfg["legs"]:
        leg_id = leg["id"]
        mount_key = leg["mount_point"]
        mount_pos = find_point_in_tree(occs, mount_key)
        # Construction point "LegMountFR" → body occurrence "LegMountFR:1".
        tab_name = f"{mount_key}:1"
        tab_pos = _world_origin_of(occs, tab_name)
        if mount_pos is None:
            print(f"  {leg_id:>2}: missing mount point {mount_key!r}")
            continue
        if tab_pos is None:
            print(
                f"  {leg_id:>2}: shoulder axis (world) = "
                f"({mount_pos[0]:+7.2f}, {mount_pos[1]:+7.2f}, {mount_pos[2]:+7.2f}) "
                f"   [no matching {tab_name} to compare]"
            )
            continue
        delta = [mount_pos[i] - tab_pos[i] for i in range(3)]
        mag = math.sqrt(sum(d * d for d in delta))
        tag = "  FLAG" if mag > 5.0 else ""
        print(
            f"  {leg_id:>2}: shoulder ({mount_pos[0]:+7.2f}, {mount_pos[1]:+7.2f}, "
            f"{mount_pos[2]:+7.2f})   "
            f"tab ({tab_pos[0]:+7.2f}, {tab_pos[1]:+7.2f}, {tab_pos[2]:+7.2f})   "
            f"Δ=({delta[0]:+6.2f}, {delta[1]:+6.2f}, {delta[2]:+6.2f}) "
            f"|Δ|={mag:5.2f}mm{tag}"
        )
        if mag > 5.0:
            warned = True
    if warned:
        print(
            "  Note: FLAG rows mean the LegMountXX construction point and the\n"
            "  matching LegMountXX:1 body occurrence origin are >5mm apart.\n"
            "  Small deltas are normal (a bracket body's origin sits at its\n"
            "  centroid, not at the attachment point). A large delta is worth\n"
            "  investigating — it's exactly the gap that appears as 'shoulder\n"
            "  mesh floats off tab' in the render."
        )


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
