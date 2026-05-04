"""
diagnose_fl_axes.py — Print joint-axis vs bone-Y geometry for the FL leg.

Diagnoses why `EditBone.align_roll(joint.axis)` in `urdf_to_blender_rigged.py`
fails to land bone-local Z = joint axis on `*_link1` and `*_link3` bones.
The cause: `align_roll` only achieves the alignment when the target axis is
perpendicular to bone-Y (head→tail). When the URDF link-to-link offset has a
component along the joint axis (link1 has Z lift; link3 foot tip has Y
component), bone-Y is not perpendicular and `align_roll` projects.

This script prints the bone-Y direction, the joint axis (world frame at
rest), the dot product, and the angle from perpendicular for each FL bone.

Run:
    uv run animation/scripts/diagnose_fl_axes.py
"""

from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


# Foot tip in link3 frame (mm) — from URDF metadata comment block:
#   FootTip (leg_lower.stl max +Y centroid, in link3 frame): -30.946 -13.000 0.024
# Same value as `FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M` in urdf_to_blender_rigged.py.
FOOT_TIP_LINK3_MM = np.array([-30.946, -13.000, 0.024])


def _parse_origin(elem):
    o = elem.find("origin")
    M = np.eye(4)
    if o is None:
        return M
    xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()])
    rx, ry, rz = (float(v) for v in (o.get("rpy") or "0 0 0").split())
    Rx = np.array(
        [[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]]
    )
    Ry = np.array(
        [[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]]
    )
    Rz = np.array(
        [[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]]
    )
    M[:3, :3] = Rz @ Ry @ Rx
    M[:3, 3] = xyz
    return M


def _parse_axis(elem):
    a = elem.find("axis")
    if a is None:
        return np.array([1.0, 0.0, 0.0])
    v = np.array([float(x) for x in a.get("xyz").split()])
    n = np.linalg.norm(v)
    return v / n if n > 0 else np.array([1.0, 0.0, 0.0])


def parse_urdf(path):
    root_elem = ET.parse(path).getroot()
    links = {l.get("name") for l in root_elem.findall("link")}
    joints = {}
    for j in root_elem.findall("joint"):
        joints[j.find("child").get("link")] = {
            "parent": j.find("parent").get("link"),
            "origin": _parse_origin(j),
            "axis": _parse_axis(j),
            "name": j.get("name"),
        }
    roots = links - set(joints.keys())
    assert len(roots) == 1, f"expected one root, got {roots}"
    return roots.pop(), joints, links


def compute_link_world(root, joints):
    link_world = {root: np.eye(4)}
    remaining = dict(joints)
    while remaining:
        placed = []
        for child, j in remaining.items():
            if j["parent"] in link_world:
                link_world[child] = link_world[j["parent"]] @ j["origin"]
                placed.append(child)
        if not placed:
            sys.exit(f"dangling joints: {list(remaining)}")
        for c in placed:
            del remaining[c]
    return link_world


def fmt_vec(v, w=8, p=4):
    return "[" + ", ".join(f"{x:>{w}.{p}f}" for x in v) + "]"


def main():
    urdf = (
        Path(__file__).resolve().parents[2]
        / "code"
        / "simulation"
        / "generated"
        / "facehugger.urdf"
    )
    if not urdf.exists():
        sys.exit(f"URDF not found: {urdf}")

    root, joints, _ = parse_urdf(urdf)
    link_world = compute_link_world(root, joints)

    leg = "fl"
    chain = [f"{leg}_link1", f"{leg}_link2", f"{leg}_link3"]

    print(f"=== {leg.upper()} leg axis diagnostic (URDF: {urdf.name}) ===")
    print(f"root link: {root}, all joints at rest (theta=0)\n")

    bone_dirs_world = {}
    for link_name in chain:
        j = joints[link_name]
        link_W = link_world[link_name]
        head_world_m = link_W[:3, 3]

        # Tail = next joint origin in world, or for *_link3 the foot tip.
        if link_name.endswith("_link3"):
            tail_world_m = link_W[:3, 3] + link_W[:3, :3] @ (FOOT_TIP_LINK3_MM / 1000.0)
            tail_source = "FootTip in link3 frame"
        else:
            child_link = next(
                c for c, jj in joints.items() if jj["parent"] == link_name
            )
            tail_world_m = link_world[child_link][:3, 3]
            tail_source = f"{joints[child_link]['name']} pivot"

        bone_dir = tail_world_m - head_world_m
        length = float(np.linalg.norm(bone_dir))
        bone_y_hat = bone_dir / length

        axis_world = link_W[:3, :3] @ j["axis"]
        axis_world /= np.linalg.norm(axis_world)

        dot = float(bone_y_hat @ axis_world)
        angle_from_perp_deg = math.degrees(math.asin(max(-1.0, min(1.0, dot))))

        print(f"--- {link_name}  (joint {j['name']})")
        print(f"  head (world, mm)     = {fmt_vec(head_world_m * 1000.0, 9, 3)}")
        print(
            f"  tail (world, mm)     = {fmt_vec(tail_world_m * 1000.0, 9, 3)}  [{tail_source}]"
        )
        print(
            f"  bone-Y (head->tail)  = {fmt_vec(bone_y_hat)}  (length {length * 1000.0:.3f} mm)"
        )
        print(f"  joint axis (world)   = {fmt_vec(axis_world)}")
        print(f"  bone_Y . joint_axis  = {dot:+.4f}")
        print(
            f"  angle from perp      = {angle_from_perp_deg:+.2f} deg  "
            f"(0 = align_roll exact; nonzero = align_roll fails)"
        )
        print()

        bone_dirs_world[link_name] = bone_y_hat

    # Collinearity check between link2 and link3 bone-Y directions.
    d2 = bone_dirs_world[f"{leg}_link2"]
    d3 = bone_dirs_world[f"{leg}_link3"]
    cos_a = max(-1.0, min(1.0, float(d2 @ d3)))
    print(
        f"--- collinearity: angle(bone-Y[{leg}_link2], bone-Y[{leg}_link3]) "
        f"= {math.degrees(math.acos(cos_a)):.2f} deg"
    )
    print(
        "    (~0 deg means link2 and link3 share a bone-Y direction; "
        "any align_roll problem on link3 then mirrors link2's behaviour.)\n"
    )


if __name__ == "__main__":
    main()
