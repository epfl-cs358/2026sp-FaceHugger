# math_utils.py — Pure float/matrix helpers. No adsk dependency.
# The adsk-aware mat_multiply wrapper (which calls adsk.core.Matrix3D) stays
# in fusion_traversal.py where it is the only caller.

import math

from config import CM_TO_MM


def pt_mm(p):
    return [
        round(p.x * CM_TO_MM, 3),
        round(p.y * CM_TO_MM, 3),
        round(p.z * CM_TO_MM, 3),
    ]


def vec3(v):
    return [round(v.x, 4), round(v.y, 4), round(v.z, 4)]


def bbox_center_mm(body):
    bb = body.boundingBox
    return [
        round((bb.minPoint.x + bb.maxPoint.x) / 2.0 * CM_TO_MM, 3),
        round((bb.minPoint.y + bb.maxPoint.y) / 2.0 * CM_TO_MM, 3),
        round((bb.minPoint.z + bb.maxPoint.z) / 2.0 * CM_TO_MM, 3),
    ]


def matrix_as_row_major_cm(m):
    """Return a Fusion Matrix3D as a 4x4 list of floats (cm, unmodified)."""
    a = m.asArray()
    return [list(a[i * 4 : (i + 1) * 4]) for i in range(4)]


def matrix_translation_mm(m):
    a = m.asArray()
    return [
        round(a[3] * CM_TO_MM, 3),
        round(a[7] * CM_TO_MM, 3),
        round(a[11] * CM_TO_MM, 3),
    ]


def apply_matrix_cm(m, p_cm):
    """Apply a 4x4 Matrix3D (row-major, cm) to a 3-vector (cm). Returns cm."""
    a = m.asArray()
    x, y, z = p_cm
    return (
        a[0] * x + a[1] * y + a[2] * z + a[3],
        a[4] * x + a[5] * y + a[6] * z + a[7],
        a[8] * x + a[9] * y + a[10] * z + a[11],
    )


def apply_matrix_to_dir_cm(m, d_cm):
    """Apply only the rotational part of a 4x4 matrix to a direction vector."""
    a = m.asArray()
    x, y, z = d_cm
    return (
        a[0] * x + a[1] * y + a[2] * z,
        a[4] * x + a[5] * y + a[6] * z,
        a[8] * x + a[9] * y + a[10] * z,
    )


def point_world_mm(p, world_transform):
    """Lift a Point3D (Fusion cm) to world-frame mm using a 4x4 matrix."""
    wx, wy, wz = apply_matrix_cm(world_transform, (p.x, p.y, p.z))
    return [round(wx * CM_TO_MM, 3), round(wy * CM_TO_MM, 3), round(wz * CM_TO_MM, 3)]


def dir_world(d, world_transform):
    """Lift a Vector3D direction to world-frame (unitless, just rotate)."""
    dx, dy, dz = apply_matrix_to_dir_cm(world_transform, (d.x, d.y, d.z))
    return [round(dx, 4), round(dy, 4), round(dz, 4)]


def mat_multiply_arrays(pa, la):
    """Multiply two 4x4 row-major matrices (each a flat 16-element list).
    Returns a flat 16-element list. Operates on plain lists — no adsk."""
    out = [0.0] * 16
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += pa[i * 4 + k] * la[k * 4 + j]
            out[i * 4 + j] = s
    return out


def _apply_R_3x3(R, v):
    """3x3 rotation applied to a 3-vector. R is row-major list-of-lists,
    v is a list/tuple of 3 floats. Returns a list of 3 floats. Tolerates
    None for either argument by returning the input unchanged."""
    if R is None or v is None:
        return v
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


def _rad_to_deg(rad):
    try:
        return math.degrees(float(rad))
    except Exception:
        return 0.0
