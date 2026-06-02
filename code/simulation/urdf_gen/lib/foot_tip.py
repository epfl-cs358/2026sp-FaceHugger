"""Two ways to compute the foot tip in link3-local frame.

Preferred source: the Fusion construction point `Link3TipPoint` relative
to `Link2ToLink3Axis`, rotated by Rz(90°). Fallback: the STL distance-
from-origin centroid on `leg_lower.stl`. Both return mm in link3 frame.
"""

import math
import struct
from pathlib import Path


def _foot_tip_from_stl(stl_path: Path, tol_mm: float = 2.0) -> list:
    """Centroid (in mm) of leg_lower STL vertices within tol_mm of the vertex
    farthest from the origin.

    The STL is re-origined to its URDF joint landmark (the knee joint) by the
    Fusion exporter, so the foot tip is the most distal point from that origin.
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
    d_max = math.sqrt(d_max)
    tip_pts = [
        v
        for v in verts
        if math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) >= d_max - tol_mm
    ]
    return [
        round(sum(v[0] for v in tip_pts) / len(tip_pts), 3),
        round(sum(v[1] for v in tip_pts) / len(tip_pts), 3),
        round(sum(v[2] for v in tip_pts) / len(tip_pts), 3),
    ]


def _foot_tip_from_export(export: dict) -> list | None:
    """Foot tip in link3-local frame (mm) from fusion_export.json construction points.

    Prefers this over the STL heuristic when present. Mirrors the math in
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
