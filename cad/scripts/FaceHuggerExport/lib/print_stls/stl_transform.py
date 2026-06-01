"""stl_transform.py — binary-STL rotation helpers for ExportPrintableSTLs.

No adsk dependency; uses only stdlib (struct, os).
"""

import struct

from .config import ROTATIONS


def rotate_stl_in_place(path: str, rotation) -> None:
    """Apply a 3×3 rotation matrix to every vertex and normal in a binary STL.

    The file is rewritten in place (read-modify-write; not atomic).
    `rotation` must be a 3×3 sequence-of-sequences of floats.
    """
    with open(path, "rb") as f:
        header = f.read(80)
        (n_tri,) = struct.unpack("<I", f.read(4))
        tri_blob = f.read(50 * n_tri)
    r = rotation

    def _rot(v):
        return (
            r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
            r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
            r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
        )

    out = bytearray()
    out += header
    out += struct.pack("<I", n_tri)
    off = 0
    for _ in range(n_tri):
        nrm = struct.unpack("<3f", tri_blob[off : off + 12])
        v1 = struct.unpack("<3f", tri_blob[off + 12 : off + 24])
        v2 = struct.unpack("<3f", tri_blob[off + 24 : off + 36])
        v3 = struct.unpack("<3f", tri_blob[off + 36 : off + 48])
        attr = tri_blob[off + 48 : off + 50]
        out += struct.pack("<12f", *_rot(nrm), *_rot(v1), *_rot(v2), *_rot(v3))
        out += attr
        off += 50
    with open(path, "wb") as f:
        f.write(bytes(out))


def rotation_for_up_axis(axis: str) -> tuple:
    """Return the rotation matrix tuple for the given up-axis label (e.g. '+Y').

    Raises ValueError for unknown axis labels.
    """
    if axis not in ROTATIONS:
        raise ValueError(f"Unknown up axis {axis!r}; valid: {sorted(ROTATIONS)}")
    return ROTATIONS[axis]
