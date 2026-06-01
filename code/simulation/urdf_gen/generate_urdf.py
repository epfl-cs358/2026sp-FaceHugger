"""
generate_urdf.py  —  FaceHugger URDF generator

Reads:
  generated/fusion_export.json  (from ExportBodiesToURDF Fusion script)
  facehugger_config.yaml        (robot hierarchy definition)

Writes:
  generated/facehugger.urdf     (or --out path)

Usage:
  uv run generate_urdf.py
  uv run generate_urdf.py --export some/other/fusion_export.json
  uv run generate_urdf.py --config facehugger_config.yaml --out generated/facehugger.urdf
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

# This module lives in urdf_gen/; generated/ and the config yaml sit one level
# up in code/simulation/ (shared with the pybullet_sim runtime).
SIM_ROOT = Path(__file__).resolve().parent.parent
GENERATED_DIR = SIM_ROOT / "generated"
DEFAULT_JSON = GENERATED_DIR / "fusion_export.json"
DEFAULT_CFG = SIM_ROOT / "facehugger_config.yaml"
DEFAULT_OUT = GENERATED_DIR / "facehugger.urdf"

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


def find_point_world_in_tree(nodes: list, key: str) -> list | None:
    """Find a named construction point anywhere in the tree, return its
    `pos_world_mm` (world position) instead of the component-local
    `pos_mm`. The first match wins — caller should use
    `find_point_world_at_occurrence` when the name is ambiguous (e.g.
    LegMountFixedPoint exists in both MotorMount:1 and MotorMountR:1)."""
    for n in nodes:
        for p in n.get("points", []):
            if p["name"] == key:
                return p.get("pos_world_mm")
        result = find_point_world_in_tree(n.get("children", []), key)
        if result is not None:
            return result
    return None


def find_point_world_at_occurrence(nodes: list, occ_path: str, key: str) -> list | None:
    """Look up a construction point's `pos_world_mm` scoped to a specific
    occurrence path (e.g. 'FaceHuggerLegAssembly:1/MotorMount:1'). Use
    this when a point name appears multiple times in the tree — both
    bracket components carry a `LegMountFixedPoint` and the unscoped
    walk would pick whichever comes first."""
    parts = occ_path.split("/")
    cur = nodes
    node = None
    for part in parts:
        node = next((c for c in (cur or []) if c.get("name") == part), None)
        if node is None:
            return None
        cur = node.get("children", [])
    if node is None:
        return None
    for p in node.get("points", []):
        if p.get("name") == key:
            return p.get("pos_world_mm")
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


def clean_axis(vec, tol=1e-10):
    """Snap tiny components to 0.0 and renormalize to unit length.

    Fusion-derived axis vectors carry FP dust on the should-be-zero columns
    (e.g. [2.77e-17, -2.99e-17, 1.0000000000000002]). This produces a clean
    [0.0, 0.0, 1.0] for the URDF <axis xyz=...> element.
    """
    cleaned = [0.0 if abs(c) < tol else c for c in vec]
    norm = math.sqrt(sum(c * c for c in cleaned))
    if norm == 0.0:
        return cleaned
    return [c / norm for c in cleaned]


def fmt_xyz(mm_vec):
    """Format an mm vector as a URDF xyz string in meters."""
    return " ".join(f"{v * MM_TO_M:.6f}" for v in mm_vec)


def fmt_rpy(r, p, y_deg):
    return f"0 0 {deg2rad(y_deg)}"


def _wrap_pi(x):
    """Wrap an angle in radians to (-pi, pi]."""
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def _shoulder_rest_for(leg_id: str, fl_rest_rad: float) -> float:
    """Derive each leg's shoulder rest (rad) from the FL Fusion-export rest
    value via mirror+rotate symmetry. With the current CAD's
    `fl_rest_rad = -π/4`, this gives all four legs splayed outward in their
    respective body quadrants.

        FL =  fl_rest_rad                       (=  -45° from current CAD)
        FR = -fl_rest_rad                       (=  +45°; mirror across body X-axis)
        BL = -wrap_pi(fl_rest_rad + pi)         (= -135°)
        BR =  wrap_pi(fl_rest_rad + pi)         (= +135°)

    Earlier versions of this function had BL and BR swapped (signs
    flipped) — that sent the back legs splaying INTO the front quadrants.
    """
    if leg_id == "fl":
        return fl_rest_rad
    if leg_id == "fr":
        return -fl_rest_rad
    if leg_id == "bl":
        return -_wrap_pi(fl_rest_rad + math.pi)
    if leg_id == "br":
        return _wrap_pi(fl_rest_rad + math.pi)
    raise ValueError(f"_shoulder_rest_for: unknown leg_id {leg_id!r}")


def _back_of_pair_rpy_z_deg(leg_id: str) -> float:
    """Geometric back-of-pair flip: front legs (fl/fr) sit at 0°, back
    legs (bl/br) at 180°. This is the Rz that positions the shoulder
    pivot and back-bracket mesh — *separate* from the kinematic shoulder
    rest, which goes into the URDF joint <origin rpy>."""
    return 180.0 if leg_id.startswith("b") else 0.0


def _foot_tip_from_stl(stl_path: Path, tol_mm: float = 2.0) -> list:
    """Centroid (in mm) of leg_lower STL vertices within tol_mm of the vertex
    farthest from the origin.

    The STL is re-origined to its URDF joint landmark (the knee joint) by the
    Fusion exporter, so the foot tip is the most distal point from that origin.
    """
    import math as _math

    with open(stl_path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        verts = []
        d_max = 0.0
        for _ in range(n):
            f.read(12)
            for _ in range(3):
                v = struct.unpack("<fff", f.read(12))
                verts.append(v)
                d = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
                if d > d_max:
                    d_max = d
            f.read(2)
    d_max = _math.sqrt(d_max)
    tip_pts = [
        v
        for v in verts
        if _math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) >= d_max - tol_mm
    ]
    return [
        round(sum(v[0] for v in tip_pts) / len(tip_pts), 3),
        round(sum(v[1] for v in tip_pts) / len(tip_pts), 3),
        round(sum(v[2] for v in tip_pts) / len(tip_pts), 3),
    ]


def _foot_tip_from_export(export: dict) -> list | None:
    """Foot tip in link3-local frame (mm) from fusion_export.json construction points.

    Prefers this over the STL heuristic when present.  Mirrors the math in
    urdf_to_blender_rigged.load_foot_tip_in_link3_frame:
      delta = Link3TipPoint - Link2ToLink3Axis  (source frame)
      result = Rz(90°) @ delta  →  (x, y, z) = (−dy, dx, dz)
    """

    knee_mm = None
    foot_mm = None

    def _walk(node: dict) -> None:
        nonlocal knee_mm, foot_mm
        for axis in node.get("axes", []) or []:
            if axis.get("name") == "Link2ToLink3Axis" and knee_mm is None:
                xyz = axis.get("origin_mm")
                if xyz and len(xyz) >= 3:
                    knee_mm = xyz[:3]
        for point in node.get("points", []) or []:
            if point.get("name") == "Link3TipPoint" and foot_mm is None:
                xyz = point.get("pos_mm")
                if xyz and len(xyz) >= 3:
                    foot_mm = xyz[:3]
        if knee_mm is None or foot_mm is None:
            for child in node.get("children", []) or []:
                _walk(child)

    for occ in export.get("occurrences", []) or []:
        _walk(occ)
        if knee_mm is not None and foot_mm is not None:
            break

    if knee_mm is None or foot_mm is None:
        return None

    dx = foot_mm[0] - knee_mm[0]
    dy = foot_mm[1] - knee_mm[1]
    dz = foot_mm[2] - knee_mm[2]
    return [round(-dy, 3), round(dx, 3), round(dz, 3)]


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
        mesh_rpy: tuple | None = None,
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

        `mesh_rpy` (optional) sets the rpy on the primary visual + collision
        <origin> blocks AND rotates the inertial CoM by the same rotation.
        Used by R-pair link2/link3 to rotate the shared L-orientation mesh
        180° about its own Y axis so the leg geometry extends in the right
        direction. Inertia tensor is left as-is — off-diagonal sign flips
        are 2nd-order and don't affect a stand/walk smoke test.
        """
        if origin_shift_mm is None:
            origin_shift_mm = [0.0, 0.0, 0.0]
        if extra_visuals is None:
            extra_visuals = []
        com_local = [com_mm[i] - origin_shift_mm[i] for i in range(3)]
        if mesh_rpy is not None:
            # Rotate CoM by the same matrix applied to the mesh, so the
            # inertial origin tracks the visible geometry.
            R = _euler_to_rot(mesh_rpy)
            com_local = [
                R[0][0] * com_local[0]
                + R[0][1] * com_local[1]
                + R[0][2] * com_local[2],
                R[1][0] * com_local[0]
                + R[1][1] * com_local[1]
                + R[1][2] * com_local[2],
                R[2][0] * com_local[0]
                + R[2][1] * com_local[1]
                + R[2][2] * com_local[2],
            ]
        xyz_com = fmt_xyz(com_local)
        primary_rpy = mesh_rpy if mesh_rpy is not None else (0.0, 0.0, 0.0)
        primary_rpy_str = " ".join(f"{v:.6f}" for v in primary_rpy)
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
            f'      <origin xyz="0 0 0" rpy="{primary_rpy_str}"/>',
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
            f'      <origin xyz="0 0 0" rpy="{primary_rpy_str}"/>',
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
        ax = " ".join(str(v) for v in clean_axis(axis))
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


def _primary_link_occurrence(mesh_entry: dict | None) -> str | None:
    """Pull the *main link body* occurrence path out of a mesh_files entry.

    For `body` rules, that's just `source_occurrences[0]`. For `combined`
    rules it's `parts[0].occurrence` — the rule convention is that the
    first part is the link body and any subsequent parts are rigidly-
    attached extras (e.g. the hip servo welded into leg_shoulder_L.stl).
    Used for inertial physics lookup; the mesh placement itself doesn't
    need this.
    """
    if not mesh_entry:
        return None
    if mesh_entry.get("source_type") == "combined":
        parts = mesh_entry.get("parts") or []
        if parts:
            return parts[0].get("occurrence")
        return None
    occs = mesh_entry.get("source_occurrences") or []
    return occs[0] if occs else None


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
    sp = max(-1.0, min(1.0, sp))  # clip tiny FP overshoot
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
        [s, c, 0.0],
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
        [-sp, cp * sr, cp * cr],
    ]


def generate(export: dict, cfg: dict, out_path: Path):
    occs = export["occurrences"]
    robot = cfg["robot_name"]
    mesh_dir = cfg["mesh_dir"]
    servo_cfg = cfg.get("servo", {})
    effort = servo_cfg.get("effort_nm", 1.47)
    vel = servo_cfg.get("velocity_rad_s", 5.0)
    # `visual_flip_rpy_deg` was a workaround for when servos were emitted
    # with CAD-source orientation that pointed the wrong way. With the
    # standalone-servos design (post-Phase G) the mesh is re-origined to
    # ServoMountPoint and placed at world ServoMountPoint per leg, so no
    # blanket orientation hack is needed. Field is read for backward yaml
    # compat but not applied.
    _ = servo_cfg.get("visual_flip_rpy_deg", [0.0, 0.0, 0.0])

    urdf = URDF(robot)

    # --- joint kinematics: CAD-sourced from export["joints"] ---
    # The Fusion exporter walks every Joint / AsBuiltJoint and emits an
    # entry with axis_dir_local_unit, axis_origin_local_mm, and limits_rad
    # (rest/min/max). The cad-name → urdf-name + parent/child mapping is
    # fixed by convention (Link1Revolute = shoulder, etc.) so it lives
    # here rather than in the yaml.
    # `urdf_name` here is the joint-name suffix (e.g. urdf_name="link1"
    # → joint name "fl_link1_joint"), aligned with the Fusion CAD
    # `LinkNRevolute` naming so the URDF, the JSON export, and the CAD
    # all share one mental model. The semantic mapping
    # link1=shoulder, link2=hip, link3=knee lives in
    # helpers._joint_type_from_name.
    _JOINT_TOPOLOGY = [
        {
            "cad_name": "Link1Revolute",
            "urdf_name": "link1",
            "parent": "base_link",
            "child": "link1",
        },
        {
            "cad_name": "Link2Revolute",
            "urdf_name": "link2",
            "parent": "link1",
            "child": "link2",
        },
        {
            "cad_name": "Link3Revolute",
            "urdf_name": "link3",
            "parent": "link2",
            "child": "link3",
        },
    ]

    leg_tmpl = cfg["leg_template"]
    leg_occ = find_occurrence(occs, leg_tmpl["leg_assembly_occurrence"])
    if not leg_occ:
        raise ValueError(
            f"Leg assembly occurrence not found: {leg_tmpl['leg_assembly_occurrence']}"
        )

    cad_joints_by_name = {j.get("name"): j for j in (export.get("joints") or [])}

    joint_defs = []
    for jcfg in _JOINT_TOPOLOGY:
        cad_name = jcfg["cad_name"]
        cad = cad_joints_by_name.get(cad_name)
        if cad is None:
            raise ValueError(
                f"Joint {cad_name!r} not found in fusion_export.json's joints[] "
                f"(make sure it's in the JOINTS whitelist in ExportBodiesToURDF.py)."
            )
        origin_mm = cad.get("axis_origin_local_mm")
        axis_dir = cad.get("axis_dir_local_unit")
        if origin_mm is None or axis_dir is None:
            raise ValueError(
                f"Joint {cad_name!r} is missing axis_origin_local_mm or "
                f"axis_dir_local_unit in the export."
            )
        lim = cad.get("limits_rad") or {}
        lim_min_rad = lim.get("min")
        lim_max_rad = lim.get("max")
        lim_rest_rad = lim.get("rest", 0.0) or 0.0
        lim_current_rad = lim.get("current", 0.0) or 0.0
        lim_min_deg = math.degrees(lim_min_rad) if lim_min_rad is not None else None
        lim_max_deg = math.degrees(lim_max_rad) if lim_max_rad is not None else None
        joint_defs.append(
            {
                "urdf_name": jcfg["urdf_name"],
                "parent": jcfg["parent"],
                "child": jcfg["child"],
                "cad_name": cad_name,
                "origin_local_mm": origin_mm,
                "axis_dir": axis_dir,
                "limits_deg": [lim_min_deg, lim_max_deg],
                "limits_rad": {
                    "min": lim_min_rad,
                    "max": lim_max_rad,
                    "rest": lim_rest_rad,
                    "current": lim_current_rad,
                },
            }
        )

    # The shoulder (Link1Revolute) rest value in the JSON defines FL's
    # mechanical zero. The other 3 corners are derived via
    # _shoulder_rest_for(leg_id, fl_rest_rad). The Fusion limits are also
    # FL-relative; URDF limits = Fusion limits shifted by FL's rest, which
    # by mirror symmetry produces the same shifted range for every leg.
    #
    # `shoulder_rest_source` (yaml) picks which JSON field to read:
    #   "rest"    -> limits_rad.rest    (configured mechanical zero, default)
    #   "current" -> limits_rad.current (live joint angle at export time)
    rest_source = cfg.get("shoulder_rest_source", "rest")
    if rest_source not in ("rest", "current"):
        raise ValueError(
            f"facehugger_config.yaml: shoulder_rest_source must be "
            f"'rest' or 'current' (got {rest_source!r})."
        )
    fl_rest_rad = joint_defs[0]["limits_rad"][rest_source]
    print(
        f"[generate_urdf] FL shoulder rest source = {rest_source!r} "
        f"-> {math.degrees(fl_rest_rad):+.2f}°"
    )
    fl_min_rad = joint_defs[0]["limits_rad"]["min"]
    fl_max_rad = joint_defs[0]["limits_rad"]["max"]
    if fl_min_rad is None or fl_max_rad is None:
        raise ValueError(
            "Link1Revolute missing limits_rad.min/max in the export — "
            "needed to derive the URDF shoulder limit window."
        )
    shoulder_lower_deg = math.degrees(fl_min_rad - fl_rest_rad)
    shoulder_upper_deg = math.degrees(fl_max_rad - fl_rest_rad)

    # Offsets between successive joints, in LAL (leg-assembly-local) frame
    # (== URDF link frame since rpy_z_deg only acts at the shoulder joint).
    for i in range(1, len(joint_defs)):
        joint_defs[i]["offset_from_parent_mm"] = sub(
            joint_defs[i]["origin_local_mm"],
            joint_defs[i - 1]["origin_local_mm"],
        )

    # --- per-side offsets in normalized (world-aligned) frame ---
    # The leg-assembly normalization in the exporter (E1-E3) aligned the
    # leg-assembly's local axes with world. JSON construction-point
    # `pos_world_mm` values are still genuine world positions; in the
    # source CAD, the L bracket is at LegMountPointFL and the R bracket
    # is sitting at the mirror position. We derive the constant offsets
    # (mount-tab → axis, mount-tab → shoulder-servo, etc.) from the
    # source-CAD positions and reuse them per leg.
    LEG_ASSEMBLY = "FaceHuggerLegAssembly:1"
    body_to_link1_world = find_point_world_at_occurrence(
        occs, LEG_ASSEMBLY, "BodyToLink1Point"
    )
    link2_to_link3_world = find_point_world_at_occurrence(
        occs, LEG_ASSEMBLY, "Link2ToLink3Point"
    )
    mount_L_world = find_point_world_at_occurrence(
        occs, f"{LEG_ASSEMBLY}/MotorMount:1", "LegMountFixedPoint"
    )
    mount_R_world = find_point_world_at_occurrence(
        occs, f"{LEG_ASSEMBLY}/MotorMountR:1", "LegMountFixedPoint"
    )
    if any(
        v is None
        for v in (
            body_to_link1_world,
            link2_to_link3_world,
            mount_L_world,
            mount_R_world,
        )
    ):
        raise ValueError(
            "Missing required construction points: BodyToLink1Point / "
            "Link2ToLink3Point / MotorMount(R) LegMountFixedPoint."
        )

    L_axis_offset = sub(body_to_link1_world, mount_L_world)
    R_axis_offset = sub(body_to_link1_world, mount_R_world)

    # Standalone-servo design (no bake-in): each leg gets a shoulder,
    # hip, and knee servo emitted as separate <visual> blocks. The mesh
    # `servo.stl` is re-origined to ServoMountPoint, so the visual's xyz
    # is the world position where ServoMountPoint should land.
    #
    # Source-CAD shoulder-servo positions: each bracket has a nested
    # `LegBaseServoEnclosure:1` with its own ServoMountPoint. Its world
    # position is the place where that bracket's shoulder servo sits in
    # the source-FL placement (L bracket) or the mirror (R bracket).
    shoulder_servo_L_world = find_point_world_at_occurrence(
        occs,
        f"{LEG_ASSEMBLY}/MotorMount:1/LegBaseServoEnclosure:1",
        "ServoMountPoint",
    )
    shoulder_servo_R_world = find_point_world_at_occurrence(
        occs,
        f"{LEG_ASSEMBLY}/MotorMountR:1/Servo_Mouser_Model(Mirror):1",
        "ServoMountPoint",
    )
    # Top-level hip / knee servos: shared (no L/R variants in CAD).
    hip_servo_world = find_point_world_at_occurrence(
        occs, f"{LEG_ASSEMBLY}/LegBaseServoEnclosure:2", "ServoMountPoint"
    )
    knee_servo_world = find_point_world_at_occurrence(
        occs, f"{LEG_ASSEMBLY}/LegBaseServoEnclosure:3", "ServoMountPoint"
    )

    shoulder_servo_L_offset = (
        sub(shoulder_servo_L_world, mount_L_world) if shoulder_servo_L_world else None
    )
    shoulder_servo_R_offset = (
        sub(shoulder_servo_R_world, mount_R_world) if shoulder_servo_R_world else None
    )
    hip_servo_offset_in_link1 = (
        sub(hip_servo_world, body_to_link1_world) if hip_servo_world else None
    )
    knee_servo_offset_in_link3 = (
        sub(knee_servo_world, link2_to_link3_world) if knee_servo_world else None
    )

    # Per-role servo orientations. The shared `servo.stl` was exported
    # via combined-rule from `LegBaseServoEnclosure:1` (the shoulder
    # servo), which bakes vertices in WORLD frame using
    # `LegBaseServoEnclosure:1`'s `world_transform_rm_cm`. So mesh-local
    # axes equal world axes for the SHOULDER placement — shaft along
    # `+Z`. The hip and knee servos in CAD have different world
    # rotations (shaft along `+Y`), so reusing the same mesh on link1
    # / link3 needs a per-role rpy that takes the mesh's
    # shoulder-orientation back to the role's CAD orientation:
    #
    #     M_role = R_role · R_shoulder^T
    #
    # Computed from JSON, this gives:
    #     M_hip  = Rx(-π/2)            →  rpy = (-π/2, 0, 0)
    #     M_knee = Rz(-π/2)·Ry(-π/2)   →  rpy = (0, -π/2, -π/2)
    R_servo1 = _find_occ_rot(occs, "LegBaseServoEnclosure:1")  # shoulder
    R_servo2 = _find_occ_rot(occs, "LegBaseServoEnclosure:2")  # hip
    R_servo3 = _find_occ_rot(occs, "LegBaseServoEnclosure:3")  # knee

    def _relative_rpy(R_target):
        if R_target is None or R_servo1 is None:
            return (0.0, 0.0, 0.0)
        return _rot_to_urdf_rpy(_mat_mul_3x3(R_target, _mat_transpose_3x3(R_servo1)))

    hip_servo_rpy = _relative_rpy(R_servo2)
    knee_servo_rpy = _relative_rpy(R_servo3)

    # The servo mesh is the manifest entry whose origin_landmark is
    # ServoMountPoint. Works for both body-rule and combined-rule
    # entries (the latter only has `parts`, no `source_body`).
    servo_mesh_name = None
    for mesh_name, entry in (export.get("mesh_files") or {}).items():
        if entry.get("origin_landmark") == "ServoMountPoint":
            servo_mesh_name = mesh_name
            break

    def _flip_x(v):
        return [-v[0], v[1], v[2]] if v else None

    def _rotate_z(v, angle_deg):
        if not v:
            return v
        c = math.cos(math.radians(angle_deg))
        s = math.sin(math.radians(angle_deg))
        return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]]

    # --- base_link ---
    urdf.comment("BASE LINK")
    base_cfg = cfg["base_link"]
    base_occ = find_occurrence(occs, "FlexibleSkeleton:1")
    mass, com, inertia = get_physics(base_occ, fallback_mass=0.5)
    # Fusion only knows about the printed chassis; the electronics live in
    # config (see base_link.extra_mass_kg). COM/inertia stay Fusion's: the
    # electronics are roughly co-located with the chassis centroid, so the
    # static-torque error from skipping a COM shift is negligible for the
    # standstill check this is sized for.
    extra_mass_kg = float(base_cfg.get("extra_mass_kg", 0.0) or 0.0)
    if extra_mass_kg:
        mass = mass + extra_mass_kg

    # base_link gets two kinds of chassis-fixed visuals per leg, both
    # placed at the leg's LegMountPointXX in body frame:
    #
    #   1. Bracket mesh (leg_mount_L.stl for FL/BR, leg_mount_R.stl for
    #      FR/BL). Re-origined to LegMountFixedPoint in mesh-local frame,
    #      so xyz=mount_world, rpy=identity drops it on the body's
    #      mating point exactly. Diagonal-pair flip — same mesh on
    #      opposite-diagonal corners — needs the back-of-pair leg's
    #      bracket to rotate 180° around its own mount Z so its outer
    #      face points outward; rpy=(0,0,π) does that.
    #
    #   2. Shoulder servo (servo.stl, re-origined to ServoMountPoint).
    #      Chassis-fixed too — bolted to the bracket, doesn't rotate
    #      with the leg. Uniform orientation per the source FL servo
    #      rotation (post-multiplied by visual_flip_rpy_deg) so all
    #      four servos look identical instead of inheriting the per-
    #      bracket Rx/Ry/Rz(180°) flip-flopping that put two of them
    #      below the chassis.
    base_extra_visuals = []

    # Bracket meshes + chassis-fixed shoulder servos — one of each per
    # leg. Bracket visual at LegMountPointXX (in chassis frame), with
    # rpy=(0,0,rpy_z_deg) for the back-of-pair flip. Shoulder servo at
    # mount + Rz(rpy_z) · side_offset (the offset from mounting tab to
    # the bracket's nested-servo ServoMountPoint, in world frame).
    for leg in cfg["legs"]:
        mount_mm = find_point_in_tree(occs, leg["mount_point"])
        if not mount_mm:
            print(
                f"warning: mount point {leg['mount_point']!r} not found; "
                f"skipping bracket visual for {leg['id']}"
            )
            continue
        side = leg.get("side", "L")
        # Geometric back-of-pair flip (chassis-fixed bracket + servo). Derived
        # from leg_id; not the kinematic shoulder rest (that's applied to the
        # URDF joint <origin rpy> only).
        rpy_z_deg = _back_of_pair_rpy_z_deg(leg["id"])
        bracket_rpy = (0.0, 0.0, math.radians(rpy_z_deg))

        bracket_mesh = f"leg_mount_{side}.stl"
        if bracket_mesh in (export.get("mesh_files") or {}):
            base_extra_visuals.append((bracket_mesh, list(mount_mm), bracket_rpy))
        else:
            print(
                f"warning: {bracket_mesh} not in mesh_files; "
                f"skipping bracket visual for {leg['id']}"
            )

        # Shoulder servo. Position = mount tab + rotated side-offset.
        # The shared servo.stl is one physical part placed at each
        # corner; we rotate it only by the back-of-pair flip
        # (rpy = (0, 0, rpy_z_deg)). No extra "mirror" rotation — the
        # CAD has the L-bracket and R-bracket nested servos at slightly
        # different world rotations (R_R · R_L^T = Ry(π)), but applying
        # that would flip the servo's Z-direction (shaft) which is
        # geometrically wrong for an unflippable physical part.
        ss_offset = shoulder_servo_L_offset if side == "L" else shoulder_servo_R_offset
        if servo_mesh_name and ss_offset is not None:
            rotated = _rotate_z(ss_offset, rpy_z_deg)
            ss_xyz = [mount_mm[i] + rotated[i] for i in range(3)]
            base_extra_visuals.append((servo_mesh_name, ss_xyz, bracket_rpy))
        elif servo_mesh_name:
            print(
                f"warning: shoulder-servo ServoMountPoint not found for "
                f"{side}-side bracket; skipping shoulder servo visual for "
                f"{leg['id']}"
            )

    urdf.link(
        base_cfg["name"],
        base_cfg["mesh"],
        mesh_dir,
        mass,
        com,
        inertia,
        origin_shift_mm=_mesh_shift(export, base_cfg["mesh"]),
        extra_visuals=base_extra_visuals,
    )

    # Emit a machine-parseable comment block with the raw leg-assembly-local
    # Points. Downstream tools (simulate) read this to avoid re-opening
    # fusion_export.json — the URDF remains the single source of truth for
    # the leg chain geometry.  Prefer the explicit Fusion construction point
    # (Link3TipPoint relative to Link2ToLink3Axis, rotated by Rz(90°));
    # fall back to the STL distance-from-origin heuristic when absent.
    tip_mm = _foot_tip_from_export(export)
    if tip_mm is not None:
        tip_source = "Link3TipPoint construction point, in link3 frame"
    else:
        tip_mm = _foot_tip_from_stl(
            GENERATED_DIR / mesh_dir.rstrip("/\\") / "leg_lower.stl"
        )
        tip_source = "leg_lower.stl max-distance-from-origin centroid, in link3 frame"
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
        f"  FootTip ({tip_source}): {tip_mm[0]:.3f} {tip_mm[1]:.3f} {tip_mm[2]:.3f}",
    ]
    urdf.comment("\n       ".join(pts_comment))

    # --- 4 leg instances ---
    # Per leg:
    #   * Shoulder joint origin = LegMountPointXX_world + Rz(rpy_z) ·
    #     side_axis_offset, where side_axis_offset is the world-frame
    #     vector from the mounting tab to the rotation axis
    #     (BodyToLink1Point) for that side's bracket.
    #   * Hip/knee joint origins are leg-assembly-local deltas in the
    #     normalized frame. For the R pair, they're X-mirrored from the
    #     L values (the R bracket and Link1R are mirrored about the
    #     leg-assembly XZ plane, which after R_la maps to a world-X
    #     mirror in the normalized frame).
    #   * Link2/Link3 are shared meshes; for the R pair we apply
    #     mesh_rpy = (0, π, 0) so the leg geometry extends in +X
    #     (mesh-local) instead of -X.
    #   * Hip servo (servo₂) is rigid with link1 per Link1RigidGroup;
    #     emitted as a standalone <visual> on link1 at the source-CAD
    #     hip-servo offset (X-flipped + (0, π, 0) for R pair).
    #   * Knee servo (servo₃) same idea on link3.
    for leg in cfg["legs"]:
        leg_id = leg["id"]
        mount_key = leg["mount_point"]
        side = leg.get("side", "L")

        mount_mm = find_point_in_tree(occs, mount_key)
        if not mount_mm:
            raise ValueError(f"Mount point not found in export: {mount_key}")

        # Geometric back-of-pair rotation (0° front, 180° back) — used to
        # position the shoulder joint origin in world. Distinct from the
        # kinematic shoulder rest, which goes into the URDF joint's
        # <origin rpy>.
        geometric_rpy_z_deg = _back_of_pair_rpy_z_deg(leg_id)

        # Per-side axis offset (mounting-tab → rotation-axis), then rotate
        # by the back-of-pair Rz so the shoulder pivot lands on
        # BodyToLink1Point for this corner.
        axis_offset = L_axis_offset if side == "L" else R_axis_offset
        rotated_axis_offset = _rotate_z(axis_offset, geometric_rpy_z_deg)
        shoulder_origin_xyz = [mount_mm[i] + rotated_axis_offset[i] for i in range(3)]

        # Shoulder rest baked into the URDF rpy comes from FL's Fusion-export
        # rest, mirrored per-leg via _shoulder_rest_for. URDF θ=0 then equals
        # each leg's mechanical zero (the splayed-out resting pose).
        sj = joint_defs[0]
        shoulder_rest_rad = _shoulder_rest_for(leg_id, fl_rest_rad)
        shoulder_rest_deg = math.degrees(shoulder_rest_rad)

        # R-side three-flip on the shoulder (matches hip/knee logic
        # below). The bracket+Link1 are CAD-mirrored about the body's
        # YZ plane for FR/BL, which reverses the servo-shaft direction
        # in body frame. Keeping the same axis vector on every leg
        # would make positive θ rotate FR/BL the wrong physical way
        # and apply asymmetric limits on the wrong half of the sweep.
        # Negating the axis and negate-swapping the limits restores
        # "same θ → same physical motion" across all four legs and
        # lets the FL Fusion limits be set arbitrarily-asymmetric
        # without breaking the right side. The shoulder rpy_z (rest)
        # does NOT change with the axis flip — it is a static rotation
        # in body frame, independent of axis sign.
        shoulder_axis = list(sj["axis_dir"])
        sh_lo, sh_hi = shoulder_lower_deg, shoulder_upper_deg
        if side == "R":
            shoulder_axis = [-a for a in shoulder_axis]
            sh_lo, sh_hi = -shoulder_upper_deg, -shoulder_lower_deg

        urdf.comment(
            f"LEG: {leg_id.upper()}  (side={side}, "
            f"shoulder_rest={shoulder_rest_deg:+.1f}°, "
            f"limits=[{sh_lo:+.1f}°, {sh_hi:+.1f}°])"
        )

        # Shoulder joint: origin = world position of the rotation axis for
        # this corner (= mount + rotated side-offset). rpy_z carries the
        # leg's mechanical rest so URDF θ=0 lands at the Fusion mechanical zero.
        urdf.joint(
            name=f"{leg_id}_{sj['urdf_name']}_joint",
            jtype="revolute",
            parent=base_cfg["name"],
            child=f"{leg_id}_link1",
            origin_mm=shoulder_origin_xyz,
            axis=shoulder_axis,
            lower_deg=sh_lo,
            upper_deg=sh_hi,
            effort=effort,
            velocity=vel,
            rpy_z_deg=shoulder_rest_deg,
        )

        # Hip + knee joints. For the R pair we apply two flips so that
        # the SAME stance-angle value produces the SAME physical motion
        # across all four legs (no per-side hacks needed in
        # simulate / IK / gait controllers):
        #
        #   1. X-flip the joint origin (geometry: link2/link3 are on
        #      the +X side of link1 for R pair, -X for L pair).
        #   2. Negate the joint axis (sign convention: in L-pair
        #      conventions, hip=-40° drops the leg; in R-pair the same
        #      physical motion needs hip=+40° because the mesh and
        #      offset flipped together. Negating the axis flips the
        #      sign convention, so hip=-40° drops the R pair too).
        #   3. Negate-and-swap the joint limits (same physical range,
        #      expressed in the flipped sign convention).
        for i, jd in enumerate(joint_defs[1:], start=1):
            offset = list(jd["offset_from_parent_mm"])
            axis_dir = list(jd["axis_dir"])
            lim_lo, lim_hi = jd["limits_deg"]
            if side == "R":
                offset[0] = -offset[0]
                axis_dir = [-a for a in axis_dir]
                # negate-and-swap: physical limit range stays the same
                if lim_lo is not None and lim_hi is not None:
                    lim_lo, lim_hi = -lim_hi, -lim_lo
            urdf.joint(
                name=f"{leg_id}_{jd['urdf_name']}_joint",
                jtype="revolute",
                parent=f"{leg_id}_link{i}",
                child=f"{leg_id}_link{i + 1}",
                origin_mm=offset,
                axis=axis_dir,
                lower_deg=lim_lo,
                upper_deg=lim_hi,
                effort=effort,
                velocity=vel,
            )

        # Standalone servo visuals on link1 (hip) and link3 (knee).
        # The shared servo.stl is the same physical part on every leg —
        # we don't apply a mirror approximation; only:
        #   1. X-flip the mounting position for R pair so the servo
        #      sits at the mirrored attach point.
        #   2. Apply the per-role rpy (hip_servo_rpy / knee_servo_rpy)
        #      so the mesh's shoulder-baked orientation rotates to the
        #      hip/knee CAD orientation (shaft along +Y instead of +Z).
        # Any per-leg orientation difference (back-of-pair Rz(π))
        # cascades automatically from the parent link's frame.
        link_extra_servos = {"link1": [], "link2": [], "link3": []}
        if servo_mesh_name:
            if hip_servo_offset_in_link1 is not None:
                hip_xyz = list(hip_servo_offset_in_link1)
                if side == "R":
                    hip_xyz[0] = -hip_xyz[0]
                link_extra_servos["link1"].append(
                    (servo_mesh_name, hip_xyz, hip_servo_rpy)
                )
            if knee_servo_offset_in_link3 is not None:
                knee_xyz = list(knee_servo_offset_in_link3)
                if side == "R":
                    knee_xyz[0] = -knee_xyz[0]
                link_extra_servos["link3"].append(
                    (servo_mesh_name, knee_xyz, knee_servo_rpy)
                )

        # Per-link visual rpy: link2 / link3 mesh is shared (L-flavor);
        # rotate 180° about Y for R pair so leg extends in +X.
        link_mesh_rpy = {
            "link1": None,
            "link2": (0.0, math.pi, 0.0) if side == "R" else None,
            "link3": (0.0, math.pi, 0.0) if side == "R" else None,
        }

        for link_key, link_cfg in leg_tmpl["links"].items():
            link_num = link_key.replace("link", "")
            link_name = f"{leg_id}_link{link_num}"
            # Substitute {side} in the mesh template (e.g. leg_shoulder_L.stl
            # for FL/BR, leg_shoulder_R.stl for FR/BL). Templates without
            # the token (leg_upper.stl, leg_lower.stl) format unchanged.
            mesh_name = link_cfg["mesh"].format(side=side)
            link_occ_path = _primary_link_occurrence(
                (export.get("mesh_files") or {}).get(mesh_name)
            )
            link_occ = (
                find_occurrence(occs, link_occ_path.split("/")[-1])
                if link_occ_path
                else None
            )
            mass, com, inertia = get_physics(link_occ, fallback_mass=0.05)
            # link1 hosts the hip servo; link3 hosts the knee servo.
            # Add the physical servo mass so dynamics match the real robot.
            # CoM stays at the structural centroid (good enough for sim fidelity).
            if link_key in ("link1", "link3"):
                mass += servo_cfg.get("mass_kg", 0.060)
            urdf.link(
                link_name,
                mesh_name,
                mesh_dir,
                mass,
                com,
                inertia,
                origin_shift_mm=_mesh_shift(export, mesh_name),
                extra_visuals=link_extra_servos.get(link_key, []),
                mesh_rpy=link_mesh_rpy.get(link_key),
            )

    urdf.save(out_path)

    _print_invariant_check(occs, cfg, joint_defs)


def _iter_occ(nodes: list):
    for n in nodes:
        yield n
        yield from _iter_occ(n.get("children", []))


def _print_invariant_check(occs: list, cfg: dict, joint_defs: list):
    """Post-generation diagnostic: print where each leg's shoulder axis
    lands in world frame (= its LegMountPointXX construction point).
    """
    print("\n=== Shoulder axis world positions (mm) ===")
    for leg in cfg["legs"]:
        leg_id = leg["id"]
        mount_key = leg["mount_point"]
        mount_pos = find_point_in_tree(occs, mount_key)
        if mount_pos is None:
            print(f"  {leg_id:>2}: missing mount point {mount_key!r}")
            continue
        print(
            f"  {leg_id:>2}: ({mount_pos[0]:+7.2f}, {mount_pos[1]:+7.2f}, "
            f"{mount_pos[2]:+7.2f})"
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
