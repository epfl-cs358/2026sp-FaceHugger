# stl_utils.py — Pure binary STL I/O. No Fusion dependency.

import struct

# Triangle = (normal, v0, v1, v2) where each is a 3-tuple of floats.
Triangle = tuple


def read_binary_stl(path):
    """Return a list of (normal, v0, v1, v2) tuples. Units match whatever
    was written (Fusion writes mm)."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 84:
        raise RuntimeError(f"STL too short: {path}")
    ntri = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + ntri * 50
    if len(data) != expected:
        raise RuntimeError(
            f"STL size mismatch: {path} has {len(data)} bytes, expected {expected}"
        )
    tris = []
    off = 84
    for _ in range(ntri):
        nx, ny, nz = struct.unpack_from("<fff", data, off)
        v0 = struct.unpack_from("<fff", data, off + 12)
        v1 = struct.unpack_from("<fff", data, off + 24)
        v2 = struct.unpack_from("<fff", data, off + 36)
        tris.append(((nx, ny, nz), v0, v1, v2))
        off += 50
    return tris


def write_binary_stl(path, triangles):
    """Write a standard binary STL: 80-byte header, uint32 count, then
    per-triangle normal + 3 verts + attr."""
    buf = bytearray()
    buf.extend(b"\x00" * 80)
    buf.extend(struct.pack("<I", len(triangles)))
    for n, v0, v1, v2 in triangles:
        buf.extend(struct.pack("<fff", *n))
        buf.extend(struct.pack("<fff", *v0))
        buf.extend(struct.pack("<fff", *v1))
        buf.extend(struct.pack("<fff", *v2))
        buf.extend(b"\x00\x00")
    with open(path, "wb") as f:
        f.write(buf)


def transform_triangles(tris, rot_3x3, translation_mm):
    """Apply `rot @ v + translation` to each vertex; rotate the normal by
    `rot` only (no translation)."""
    r = rot_3x3
    tx, ty, tz = translation_mm
    out = []
    for (nx, ny, nz), v0, v1, v2 in tris:

        def rv(v):
            x, y, z = v
            return (
                r[0][0] * x + r[0][1] * y + r[0][2] * z + tx,
                r[1][0] * x + r[1][1] * y + r[1][2] * z + ty,
                r[2][0] * x + r[2][1] * y + r[2][2] * z + tz,
            )

        out.append(
            (
                (
                    r[0][0] * nx + r[0][1] * ny + r[0][2] * nz,
                    r[1][0] * nx + r[1][1] * ny + r[1][2] * nz,
                    r[2][0] * nx + r[2][1] * ny + r[2][2] * nz,
                ),
                rv(v0),
                rv(v1),
                rv(v2),
            )
        )
    return out


def translate_stl_in_place(path, shift_mm):
    """Translate every vertex in a binary STL file by `-shift_mm` in-place so
    the point previously at `shift_mm` becomes the new origin."""
    sx, sy, sz = shift_mm
    with open(path, "rb") as f:
        data = bytearray(f.read())
    if len(data) < 84:
        raise RuntimeError(f"STL too short: {path}")
    ntri = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + ntri * 50
    if len(data) != expected:
        raise RuntimeError(
            f"STL size mismatch: {path} has {len(data)} bytes, expected {expected}"
        )
    off = 84
    for _ in range(ntri):
        v_off = off + 12
        verts = list(struct.unpack_from("<9f", data, v_off))
        for i in range(3):
            verts[i * 3 + 0] -= sx
            verts[i * 3 + 1] -= sy
            verts[i * 3 + 2] -= sz
        struct.pack_into("<9f", data, v_off, *verts)
        off += 50
    with open(path, "wb") as f:
        f.write(data)
