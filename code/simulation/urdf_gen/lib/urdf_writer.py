"""The `URDF` class — accumulates `<link>` / `<joint>` lines and writes the
final XML. Depends only on the formatting helpers in `urdf_math`.
"""

from dataclasses import dataclass
from pathlib import Path

from .urdf_math import _euler_to_rot, clean_axis, deg2rad, fmt_rpy, fmt_xyz


@dataclass
class CollisionPrimitive:
    """Analytic collision geometry to replace the STL mesh <collision>.

    Shapes mirror URDF primitives:
      - sphere:  origin_xyz_mm, radius (metres)
      - capsule: origin_xyz_mm, origin_rpy, radius (metres), length (metres)
      - box:     origin_xyz_mm, origin_rpy (radians), size_xyz_mm (metres)
    """
    shape: str  # "sphere" | "capsule" | "box"
    origin_xyz_mm: list  # [x, y, z] mm in link frame
    origin_rpy: tuple = (0.0, 0.0, 0.0)  # radians
    radius: float = 0.0  # sphere / capsule radius (metres)
    length: float = 0.0  # capsule cylindrical section (metres)
    size_xyz_mm: list | None = None  # [x, y, z] mm for box


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
        collision_override: CollisionPrimitive | None = None,
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

        `collision_override` (optional) replaces the STL mesh collision with
        an analytic primitive (sphere / box). The primitive's origin is in the
        link frame (mm, converted to metres). When None, the STL mesh is used
        for collision as before.
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
        ]
        if collision_override is not None:
            ov = collision_override
            rpy_str = " ".join(f"{v:.6f}" for v in ov.origin_rpy)
            xyz = fmt_xyz(ov.origin_xyz_mm)
            self.lines += [
                f'      <origin xyz="{xyz}" rpy="{rpy_str}"/>',
                "      <geometry>",
            ]
            if ov.shape == "sphere":
                self.lines += [f'        <sphere radius="{ov.radius}"/>']
            elif ov.shape == "capsule":
                self.lines += [f'        <capsule radius="{ov.radius}" length="{ov.length}"/>']
            elif ov.shape == "box":
                size_str = " ".join(f"{s * 1e-3:.6f}" for s in ov.size_xyz_mm)
                self.lines += [f'        <box size="{size_str}"/>']
            self.lines += [
                "      </geometry>",
            ]
        else:
            self.lines += [
                f'      <origin xyz="0 0 0" rpy="{primary_rpy_str}"/>',
                "      <geometry>",
                f'        <mesh filename="{mesh_dir}{mesh}" scale="0.001 0.001 0.001"/>',
                "      </geometry>",
            ]
        self.lines += [
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
