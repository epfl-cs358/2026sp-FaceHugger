# === Blender-only ===
# Imported by the Blender scene builders; requires `mathutils`.
"""urdf_parser.py — shared URDF reader for Blender scene builders.

Extracted from visualize_urdf.py and urdf_to_blender_rigged.py to eliminate
the duplicated parse_urdf / _parse_origin / _parse_axis / compute_link_world
copy that lived in both files.

Requires mathutils (available inside Blender); not importable from plain Python.
"""

import sys
import xml.etree.ElementTree as ET

from mathutils import Matrix, Vector

M_TO_MM = 1000.0


def _parse_origin(elem):
    """Return a 4×4 mathutils.Matrix from an <origin xyz=... rpy=.../> child
    of `elem`. Identity if absent. URDF rpy is the extrinsic XYZ convention,
    which composes as R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Translation is in
    metres."""
    o = elem.find("origin")
    if o is None:
        return Matrix.Identity(4)
    xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
    rx = Matrix.Rotation(rpy[0], 3, "X")
    ry = Matrix.Rotation(rpy[1], 3, "Y")
    rz = Matrix.Rotation(rpy[2], 3, "Z")
    M = (rz @ ry @ rx).to_4x4()
    M.translation = Vector(xyz)
    return M


def _parse_axis(elem):
    """Return the joint axis as a normalized 3-vector. Default (1,0,0)."""
    a = elem.find("axis")
    if a is None:
        return Vector((1.0, 0.0, 0.0))
    xyz = [float(v) for v in a.get("xyz").split()]
    v = Vector(xyz)
    return v.normalized() if v.length > 0 else Vector((1.0, 0.0, 0.0))


def _parse_limit(elem):
    """Return (lower_rad, upper_rad) from <joint><limit>, or (None, None)
    for fixed joints / missing limits."""
    lim = elem.find("limit")
    if lim is None:
        return (None, None)
    try:
        return (float(lim.get("lower")), float(lim.get("upper")))
    except (TypeError, ValueError):
        return (None, None)


def _parse_visual(v):
    """(origin_matrix, mesh_filename, scale_xyz) or None if no <mesh>."""
    g = v.find("geometry/mesh")
    if g is None:
        return None
    fname = g.get("filename")
    scale = [float(s) for s in (g.get("scale") or "1 1 1").split()]
    return (_parse_origin(v), fname, scale)


def parse_urdf(path):
    """Parse the URDF into {root, links, joints}.

    links[name]              -> [(visual_origin_4x4, mesh_filename, scale_xyz), ...]
    joints[child_link_name]  -> {parent, origin, axis, name, type, lower, upper}
    root                     -> the unique link with no parent in joints
    """
    tree = ET.parse(path)
    root_elem = tree.getroot()
    if root_elem.tag != "robot":
        sys.exit(f"{path}: root tag is <{root_elem.tag}>, expected <robot>")

    links = {}
    for link_elem in root_elem.findall("link"):
        name = link_elem.get("name")
        visuals = []
        for v in link_elem.findall("visual"):
            parsed = _parse_visual(v)
            if parsed is not None:
                visuals.append(parsed)
        links[name] = visuals

    joints = {}
    for j in root_elem.findall("joint"):
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        lower, upper = _parse_limit(j)
        joints[child] = {
            "parent": parent,
            "origin": _parse_origin(j),
            "axis": _parse_axis(j),
            "name": j.get("name"),
            "type": j.get("type"),
            "lower": lower,
            "upper": upper,
        }

    children = set(joints.keys())
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        sys.exit(f"{path}: expected exactly one root link, got {roots}")
    return {"root": roots[0], "links": links, "joints": joints}


def compute_link_world(robot):
    """Walk the joint tree from root and return {link_name: 4x4 matrix in m}.

    Joints aren't necessarily in topological order in the URDF, so we
    iterate-until-stable: each pass places every link whose parent is
    already known. Bails out loud if the URDF has a dangling chain."""
    link_world = {robot["root"]: Matrix.Identity(4)}
    remaining = dict(robot["joints"])
    while remaining:
        placed = []
        for child, j in remaining.items():
            if j["parent"] in link_world:
                link_world[child] = link_world[j["parent"]] @ j["origin"]
                placed.append(child)
        if not placed:
            sys.exit(
                f"Dangling joint chain — could not place: {list(remaining.keys())}"
            )
        for c in placed:
            del remaining[c]
    return link_world


def matrix_m_to_mm(M):
    """Return a copy of 4x4 matrix M with its translation scaled m → mm.
    Rotation passes through (rotations are unitless)."""
    out = M.copy()
    out.translation = out.translation * M_TO_MM
    return out
