"""Math + URDF-XML formatting helpers used across `urdf_gen.generate_urdf`.

Pure functions — no I/O, no Fusion export awareness. Safe to unit-test
in isolation."""

import math

MM_TO_M = 1e-3


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
