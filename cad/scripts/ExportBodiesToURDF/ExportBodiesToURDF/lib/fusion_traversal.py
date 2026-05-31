# fusion_traversal.py — Fusion-dependent traversal and collectors.
# Imports adsk (unavoidable — walks Occurrence/Component API).

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

from .config import (
    CM2_TO_M2,
    COLLECT_PHYSICS,
    CONSTRUCTION_AXES,
    CONSTRUCTION_POINTS,
    CM_TO_MM,
)
from .math_utils import (
    bbox_center_mm,
    dir_world,
    mat_multiply_arrays,
    matrix_as_row_major_cm,
    matrix_translation_mm,
    point_world_mm,
    pt_mm,
    vec3,
)


def identity_matrix():
    return adsk.core.Matrix3D.create()


def mat_multiply(parent, local):
    """Return a new adsk.core.Matrix3D = parent * local."""
    out = mat_multiply_arrays(parent.asArray(), local.asArray())
    result = adsk.core.Matrix3D.create()
    result.setWithArray(out)
    return result


def occ_parent_origin_mm(occ):
    """Translation of `occ.transform`, in mm (parent-relative)."""
    t = occ.transform
    return [
        round(t.translation.x * CM_TO_MM, 3),
        round(t.translation.y * CM_TO_MM, 3),
        round(t.translation.z * CM_TO_MM, 3),
    ]


def collect_bodies(comp):
    """Every body in the component, with `visible` as informational metadata."""
    out = []
    for body in comp.bRepBodies:
        out.append(
            {
                "name": body.name,
                "visible": body.isLightBulbOn,
                "bbox_center_mm": bbox_center_mm(body),
            }
        )
    return out


def collect_axes(comp, world_transform):
    """Filtered to names in CONSTRUCTION_AXES. Stored in local and world frame."""
    out = []
    for axis in comp.constructionAxes:
        if axis.name not in CONSTRUCTION_AXES:
            continue
        try:
            geom = axis.geometry  # Line3D
            out.append(
                {
                    "name": axis.name,
                    "visible": axis.isLightBulbOn,
                    "origin_mm": pt_mm(geom.origin),
                    "dir": vec3(geom.direction),
                    "origin_world_mm": point_world_mm(geom.origin, world_transform),
                    "dir_world": dir_world(geom.direction, world_transform),
                }
            )
        except Exception as e:
            out.append({"name": axis.name, "error": str(e)})
    return out


def collect_points(comp, world_transform):
    """Filtered to names in CONSTRUCTION_POINTS."""
    out = []
    for point in comp.constructionPoints:
        if point.name not in CONSTRUCTION_POINTS:
            continue
        try:
            geom = point.geometry
            out.append(
                {
                    "name": point.name,
                    "visible": point.isLightBulbOn,
                    "pos_mm": pt_mm(geom),
                    "pos_world_mm": point_world_mm(geom, world_transform),
                }
            )
        except Exception as e:
            out.append({"name": point.name, "error": str(e)})
    return out


def collect_physics(occ, parent_to_world):
    """Report CoM in both parent frame (com_mm) and world (com_world_mm).

    `Occurrence.getPhysicalProperties().centerOfMass` is in the PARENT frame,
    so we apply `parent_to_world`, NOT `this_to_world`.
    """
    try:
        acc = adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
        prop = occ.getPhysicalProperties(acc)
        vals = prop.getXYZMomentsOfInertia()
        ixx, iyy, izz, ixy, iyz, ixz = [round(v * CM2_TO_M2, 9) for v in vals[1:]]
        com = prop.centerOfMass
        return {
            "mass_kg": round(prop.mass, 6),
            "com_mm": pt_mm(com),
            "com_world_mm": point_world_mm(com, parent_to_world),
            "inertia_kg_m2": {
                "ixx": ixx,
                "iyy": iyy,
                "izz": izz,
                "ixy": ixy,
                "iyz": iyz,
                "ixz": ixz,
            },
        }
    except Exception as e:
        return {"error": str(e)}


def traverse(occurrences, parent_to_world=None):
    """Recursive walk that accumulates the world transform.

    `occ.transform` is PARENT-RELATIVE in Fusion. We pre-multiply by the
    parent's world transform so every nested occurrence reports its
    points/axes/CoM in a single consistent root frame.
    """
    if parent_to_world is None:
        parent_to_world = identity_matrix()
    result = []
    for occ in occurrences:
        comp = occ.component
        this_to_world = mat_multiply(parent_to_world, occ.transform)
        node = {
            "name": occ.name,
            "xref": occ.isReferencedComponent,
            "visible": occ.isLightBulbOn,
            "parent_origin_mm": occ_parent_origin_mm(occ),
            "world_origin_mm": matrix_translation_mm(this_to_world),
            "world_transform_rm_cm": matrix_as_row_major_cm(this_to_world),
            "bodies": collect_bodies(comp),
            "axes": collect_axes(comp, this_to_world),
            "points": collect_points(comp, this_to_world),
            "children": [],
        }
        if COLLECT_PHYSICS:
            node["physics"] = collect_physics(occ, parent_to_world)
        if comp.occurrences.count > 0:
            node["children"] = traverse(comp.occurrences, this_to_world)
        result.append(node)
    return result
