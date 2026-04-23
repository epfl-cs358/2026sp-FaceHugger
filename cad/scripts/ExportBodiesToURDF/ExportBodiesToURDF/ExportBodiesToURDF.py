"""
ExportBodiesToURDF.py
Recursively traverses the active Fusion assembly and exports:
  - Every occurrence: name, xref flag, visibility, world origin (mm)
  - Every body: name, visibility, bounding box center (mm)
  - Every construction axis: name, origin (mm), direction (unit vector)
  - Every construction point: name, position (mm)

Output: /Users/marcushamelink/Documents/EPFL-projects/projects-ba6/Making-Intelligent-Things/2026sp-FaceHugger/cad/scripts/fusion_export.txt
Run via: Shift+S → Scripts and Add-Ins → Run
"""

import traceback
from pathlib import Path

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

app = adsk.core.Application.get()
ui = app.userInterface

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

VISIBLE_ONLY = True  # True = skip hidden bodies/axes/points
OUTPUT_PATH = Path(
    "/Users/marcushamelink/Documents/EPFL-projects/projects-ba6/Making-Intelligent-Things/2026sp-FaceHugger/cad/scripts/fusion_export.txt"
)
CM_TO_MM = 10.0  # Fusion API is internally in cm


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def pt_mm(point):
    return (
        round(point.x * CM_TO_MM, 3),
        round(point.y * CM_TO_MM, 3),
        round(point.z * CM_TO_MM, 3),
    )


def vec_fmt(vector):
    return (round(vector.x, 4), round(vector.y, 4), round(vector.z, 4))


def bbox_center_mm(body):
    bb = body.boundingBox
    cx = (bb.minPoint.x + bb.maxPoint.x) / 2.0
    cy = (bb.minPoint.y + bb.maxPoint.y) / 2.0
    cz = (bb.minPoint.z + bb.maxPoint.z) / 2.0
    return (round(cx * CM_TO_MM, 3), round(cy * CM_TO_MM, 3), round(cz * CM_TO_MM, 3))


def occ_origin_mm(occ):
    t = occ.transform
    return (
        round(t.translation.x * CM_TO_MM, 3),
        round(t.translation.y * CM_TO_MM, 3),
        round(t.translation.z * CM_TO_MM, 3),
    )


# ---------------------------------------------------------------------------
# Collectors
# ---------------------------------------------------------------------------


def collect_bodies(comp, indent):
    lines = []
    for body in comp.bRepBodies:
        if VISIBLE_ONLY and not body.isLightBulbOn:
            continue
        vis = "" if body.isLightBulbOn else " [hidden]"
        lines.append(
            f"{indent}  body: {body.name}{vis} | bbox_center_mm: {bbox_center_mm(body)}"
        )
    return lines


def collect_axes(comp, indent):
    lines = []
    for axis in comp.constructionAxes:
        if VISIBLE_ONLY and not axis.isLightBulbOn:
            continue
        vis = "" if axis.isLightBulbOn else " [hidden]"
        try:
            geom = axis.geometry  # Line3D
            lines.append(
                f"{indent}  caxis: {axis.name}{vis} "
                f"| origin_mm: {pt_mm(geom.origin)} | dir: {vec_fmt(geom.direction)}"
            )
        except Exception as e:
            lines.append(f"{indent}  caxis: {axis.name}{vis} | ERROR: {e}")
    return lines


def collect_points(comp, indent):
    lines = []
    for point in comp.constructionPoints:
        if VISIBLE_ONLY and not point.isLightBulbOn:
            continue
        vis = "" if point.isLightBulbOn else " [hidden]"
        try:
            lines.append(
                f"{indent}  cpoint: {point.name}{vis} | pos_mm: {pt_mm(point.geometry)}"
            )
        except Exception as e:
            lines.append(f"{indent}  cpoint: {point.name}{vis} | ERROR: {e}")
    return lines


# ---------------------------------------------------------------------------
# Recursive traversal
# ---------------------------------------------------------------------------


def traverse(occurrences, lines, depth):
    indent = "  " * depth
    for occ in occurrences:
        if VISIBLE_ONLY and not occ.isLightBulbOn:
            continue

        comp = occ.component
        xref_tag = " [xref]" if occ.isReferencedComponent else ""
        vis_tag = "" if occ.isLightBulbOn else " [hidden]"

        lines.append(f"{indent}{occ.name}{xref_tag}{vis_tag}")
        lines.append(f"{indent}  world_origin_mm: {occ_origin_mm(occ)}")
        lines += collect_bodies(comp, indent)
        lines += collect_axes(comp, indent)
        lines += collect_points(comp, indent)

        if comp.occurrences.count > 0:
            traverse(comp.occurrences, lines, depth + 1)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def run(_context: str):
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design. Open your assembly first.")
            return

        root = design.rootComponent
        lines = [
            "=" * 60,
            "FaceHugger Fusion Export",
            f"Document   : {root.name}",
            f"Visible only: {VISIBLE_ONLY}",
            "=" * 60,
            "",
        ]

        # Root-level construction geometry (defined directly on the assembly)
        root_content = collect_axes(root, "") + collect_points(root, "")
        if root_content:
            lines.append("[root]")
            lines += root_content
            lines.append("")

        traverse(root.occurrences, lines, depth=0)

        with open(OUTPUT_PATH, "w") as f:
            f.write("\n".join(lines))

        ui.messageBox(f"Done.\nWritten to: {OUTPUT_PATH}")

    except:  # pylint: disable=bare-except
        app.log(f"Failed:\n{traceback.format_exc()}")
