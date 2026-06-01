"""URDF / STL / fusion_export.json parsing utilities."""

import json
import math
import re
import struct
import xml.etree.ElementTree as ET

from .math_utils import _parse_xyz


def _parse_leg_points_from_urdf(urdf_path):
    """Read the LEG ASSEMBLY METADATA comment block the generator writes near
    the top of the URDF. Returns {point_name: [x, y, z]} in mm, leg-assembly-
    local frame. Raises if the comment block is missing so we fail loud
    instead of silently falling back to a stale hardcoded value.
    """
    text = open(urdf_path).read()
    pts = {}
    # Each metadata line has the shape:
    #   <Name> [optional descriptor that may contain digits, like "link3"]: <x> <y> <z>
    # Match non-greedily up to the first colon on the line, then the three
    # whitespace-separated signed-decimal numbers. The earlier [^0-9\-]*
    # bridge broke on descriptors containing digits (e.g. "link3 frame").
    pattern = re.compile(
        r"(BodyToLink1Point|Link1ToLink2Point|Link2ToLink3Point|FootTip)"
        r"[^\n]*?:\s*"
        r"(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)"
    )
    for m in pattern.finditer(text):
        name = m.group(1)
        pts[name] = [float(m.group(2)), float(m.group(3)), float(m.group(4))]
    required = ("BodyToLink1Point", "Link1ToLink2Point", "Link2ToLink3Point", "FootTip")
    missing = [n for n in required if n not in pts]
    if missing:
        raise ValueError(
            f"URDF at {urdf_path} is missing LEG ASSEMBLY METADATA comment "
            f"for points: {missing}. Regenerate with generate_urdf.py."
        )
    return pts


def _load_urdf_joints(urdf_path):
    """Return {joint_name: {parent, child, xyz, rpy, axis, limits}} from the raw XML."""
    root = ET.parse(urdf_path).getroot()
    out = {}
    for j in root.findall("joint"):
        name = j.get("name")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        org = j.find("origin")
        xyz = _parse_xyz(org.get("xyz", "0 0 0")) if org is not None else (0, 0, 0)
        rpy = _parse_xyz(org.get("rpy", "0 0 0")) if org is not None else (0, 0, 0)
        ax = j.find("axis")
        axis = _parse_xyz(ax.get("xyz")) if ax is not None else (1, 0, 0)
        lim = j.find("limit")
        if lim is not None:
            limits = (
                float(lim.get("lower", -math.pi)),
                float(lim.get("upper", math.pi)),
                float(lim.get("effort", 0.0)),
                float(lim.get("velocity", 0.0)),
            )
        else:
            limits = (-math.pi, math.pi, 0.0, 0.0)
        out[name] = dict(
            parent=parent, child=child, xyz=xyz, rpy=rpy, axis=axis, limits=limits
        )
    return out


def _stl_foot_tip_m(stl_path, tol_mm=2.0):
    """Centroid (in metres) of leg_lower STL vertices within tol_mm of the
    vertex farthest from the origin.

    The STL is re-origined to its URDF joint landmark (the knee joint) by the
    Fusion exporter, so the foot tip is the most distal point from that origin —
    not the max-Y extreme (which would be the knee end after re-origining).
    """
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
    import math as _math

    d_max = _math.sqrt(d_max)
    tip_pts = [
        v
        for v in verts
        if _math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) >= d_max - tol_mm
    ]
    cx = sum(v[0] for v in tip_pts) / len(tip_pts) / 1000.0
    cy = sum(v[1] for v in tip_pts) / len(tip_pts) / 1000.0
    cz = sum(v[2] for v in tip_pts) / len(tip_pts) / 1000.0
    return (cx, cy, cz)


def _foot_tip_from_fusion(fusion_json_path):
    """Return the FootTipPoint from fusion_export.json if present, else None.

    Prefer this over the STL heuristic whenever the CAD designer adds an
    explicit FootTipPoint construction point under FaceHuggerLegAssembly:1.
    """
    try:
        with open(fusion_json_path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None

    def _walk(node):
        if isinstance(node, dict):
            if node.get("name") == "FootTipPoint" and "position_mm" in node:
                return node["position_mm"]
            for v in node.values():
                r = _walk(v)
                if r is not None:
                    return r
        elif isinstance(node, list):
            for v in node:
                r = _walk(v)
                if r is not None:
                    return r
        return None

    pos = _walk(data)
    if pos is None:
        return None
    return tuple(x / 1000.0 for x in pos)
