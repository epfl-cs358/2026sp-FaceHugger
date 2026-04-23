"""
ExportBodiesToURDF.py  —  FaceHugger Fusion assembly exporter

Outputs to code/simulation/ (relative to this script's repo location):
  fusion_export.json     machine-readable, consumed by generate_urdf.py
  fusion_export.txt      human-readable tree for sanity checking
  exported_meshes/*.stl  one STL per entry in MESH_EXPORTS below

Run via: Shift+S → Scripts and Add-Ins → ExportBodiesToURDF → Run
"""

import json
import os
import traceback

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

app = adsk.core.Application.get()
ui = app.userInterface

# ---------------------------------------------------------------------------
# Paths — relative to this script's location inside the repo.
# Script lives at:  cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/
# Repo root is 4 levels up, then down to code/simulation/.
# ---------------------------------------------------------------------------

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "..", "code", "simulation")
)
_MESH_DIR = os.path.join(_SIM_DIR, "exported_meshes")

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

VISIBLE_ONLY = True  # skip hidden geometry
COLLECT_PHYSICS = True  # mass/CoM/inertia — slow, set False to skip
CM_TO_MM = 10.0
CM2_TO_M2 = 1e-4  # kg·cm² → kg·m²

# Which occurrences to export as STL, and under what filename.
# Keys are occurrence names as they appear in the assembly tree.
# For nested occurrences (e.g. inside an xref), use the leaf name.
MESH_EXPORTS = {
    "Link1:1": "leg_shoulder.stl",
    "Link2:1": "leg_upper.stl",
    "Link3:1": "leg_lower.stl",
    "BottomPlate:1": "QuadrupedBody.stl",
}


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


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


def occ_origin_mm(occ):
    t = occ.transform
    return [
        round(t.translation.x * CM_TO_MM, 3),
        round(t.translation.y * CM_TO_MM, 3),
        round(t.translation.z * CM_TO_MM, 3),
    ]


# ---------------------------------------------------------------------------
# Collectors
# ---------------------------------------------------------------------------


def collect_bodies(comp):
    out = []
    for body in comp.bRepBodies:
        if VISIBLE_ONLY and not body.isLightBulbOn:
            continue
        out.append({
            "name": body.name,
            "visible": body.isLightBulbOn,
            "bbox_center_mm": bbox_center_mm(body),
        })
    return out


def collect_axes(comp):
    out = []
    for axis in comp.constructionAxes:
        if VISIBLE_ONLY and not axis.isLightBulbOn:
            continue
        try:
            geom = axis.geometry  # Line3D
            out.append({
                "name": axis.name,
                "visible": axis.isLightBulbOn,
                "origin_mm": pt_mm(geom.origin),
                "dir": vec3(geom.direction),
            })
        except Exception as e:
            out.append({"name": axis.name, "error": str(e)})
    return out


def collect_points(comp):
    out = []
    for point in comp.constructionPoints:
        if VISIBLE_ONLY and not point.isLightBulbOn:
            continue
        try:
            out.append({
                "name": point.name,
                "visible": point.isLightBulbOn,
                "pos_mm": pt_mm(point.geometry),
            })
        except Exception as e:
            out.append({"name": point.name, "error": str(e)})
    return out


def collect_physics(occ):
    try:
        acc = adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
        prop = occ.getPhysicalProperties(acc)
        vals = prop.getXYZMomentsOfInertia()
        ixx, iyy, izz, ixy, iyz, ixz = [round(v * CM2_TO_M2, 9) for v in vals[1:]]
        return {
            "mass_kg": round(prop.mass, 6),
            "com_mm": pt_mm(prop.centerOfMass),
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


# ---------------------------------------------------------------------------
# STL export
# ---------------------------------------------------------------------------


def export_stls(design, root):
    """Export occurrences listed in MESH_EXPORTS to exported_meshes/."""
    os.makedirs(_MESH_DIR, exist_ok=True)
    mgr = design.exportManager
    exported = []
    failed = []

    def find_and_export(occurrences):
        for occ in occurrences:
            if occ.name in MESH_EXPORTS:
                filename = os.path.join(_MESH_DIR, MESH_EXPORTS[occ.name])
                try:
                    opts = mgr.createSTLExportOptions(occ, filename)
                    opts.sendToPrintUtility = False
                    opts.isBinaryFormat = True
                    opts.meshRefinement = (
                        adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                    )
                    mgr.execute(opts)
                    exported.append(MESH_EXPORTS[occ.name])
                except Exception as e:
                    failed.append(f"{occ.name}: {e}")
            # Always recurse — target occurrences may be nested inside xrefs
            find_and_export(occ.component.occurrences)

    find_and_export(root.occurrences)
    return exported, failed


# ---------------------------------------------------------------------------
# Recursive traversal
# ---------------------------------------------------------------------------


def traverse(occurrences):
    result = []
    for occ in occurrences:
        if VISIBLE_ONLY and not occ.isLightBulbOn:
            continue
        comp = occ.component
        node = {
            "name": occ.name,
            "xref": occ.isReferencedComponent,
            "visible": occ.isLightBulbOn,
            "world_origin_mm": occ_origin_mm(occ),
            "bodies": collect_bodies(comp),
            "axes": collect_axes(comp),
            "points": collect_points(comp),
            "children": [],
        }
        if COLLECT_PHYSICS:
            node["physics"] = collect_physics(occ)
        if comp.occurrences.count > 0:
            node["children"] = traverse(comp.occurrences)
        result.append(node)
    return result


# ---------------------------------------------------------------------------
# Text formatter
# ---------------------------------------------------------------------------


def fmt_tree(nodes, depth=0):
    lines = []
    pad = "  " * depth
    for n in nodes:
        xref_tag = " [xref]" if n["xref"] else ""
        vis_tag = "" if n["visible"] else " [hidden]"
        lines.append(f"{pad}{n['name']}{xref_tag}{vis_tag}")
        lines.append(f"{pad}  world_origin_mm: {tuple(n['world_origin_mm'])}")
        for b in n["bodies"]:
            v = "" if b["visible"] else " [hidden]"
            lines.append(
                f"{pad}  body: {b['name']}{v} | bbox_center_mm: {tuple(b['bbox_center_mm'])}"
            )
        for a in n["axes"]:
            if "error" in a:
                lines.append(f"{pad}  caxis: {a['name']} | ERROR: {a['error']}")
            else:
                v = "" if a["visible"] else " [hidden]"
                lines.append(
                    f"{pad}  caxis: {a['name']}{v} "
                    f"| origin_mm: {tuple(a['origin_mm'])} | dir: {tuple(a['dir'])}"
                )
        for p in n["points"]:
            if "error" in p:
                lines.append(f"{pad}  cpoint: {p['name']} | ERROR: {p['error']}")
            else:
                v = "" if p["visible"] else " [hidden]"
                lines.append(
                    f"{pad}  cpoint: {p['name']}{v} | pos_mm: {tuple(p['pos_mm'])}"
                )
        if COLLECT_PHYSICS and "physics" in n and "error" not in n["physics"]:
            ph = n["physics"]
            lines.append(
                f"{pad}  mass: {ph['mass_kg']} kg | com_mm: {tuple(ph['com_mm'])}"
            )
        if n["children"]:
            lines += fmt_tree(n["children"], depth + 1)
    return lines


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def run(_context: str):
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design.")
            return

        root = design.rootComponent

        export = {
            "document": root.name,
            "visible_only": VISIBLE_ONLY,
            "collect_physics": COLLECT_PHYSICS,
            "root_axes": collect_axes(root),
            "root_points": collect_points(root),
            "occurrences": traverse(root.occurrences),
        }

        os.makedirs(_SIM_DIR, exist_ok=True)

        # JSON
        json_path = os.path.join(_SIM_DIR, "fusion_export.json")
        with open(json_path, "w") as f:
            json.dump(export, f, indent=2)

        # TXT
        txt_path = os.path.join(_SIM_DIR, "fusion_export.txt")
        header = [
            "=" * 60,
            "FaceHugger Fusion Export",
            f"Document   : {export['document']}",
            f"Visible only: {VISIBLE_ONLY}  |  Physics: {COLLECT_PHYSICS}",
            "=" * 60,
            "",
        ]
        if export["root_axes"] or export["root_points"]:
            header.append("[root]")
            for a in export["root_axes"]:  # ty:ignore[not-iterable]
                header.append(
                    f"  caxis: {a['name']} | origin_mm: {tuple(a['origin_mm'])} | dir: {tuple(a['dir'])}"
                )
            for p in export["root_points"]:  # ty:ignore[not-iterable]
                header.append(f"  cpoint: {p['name']} | pos_mm: {tuple(p['pos_mm'])}")
            header.append("")
        with open(txt_path, "w") as f:
            f.write("\n".join(header + fmt_tree(export["occurrences"])))

        # STLs
        exported, failed = export_stls(design, root)

        # Summary
        msg = f"Export complete.\n\nJSON + TXT → {_SIM_DIR}\n"
        if exported:
            msg += f"\nSTLs exported ({len(exported)}):\n" + "\n".join(
                f"  {s}" for s in exported
            )
        if failed:
            msg += f"\n\nFailed ({len(failed)}):\n" + "\n".join(
                f"  {s}" for s in failed
            )
        ui.messageBox(msg)

    except:  # pylint: disable=bare-except  # noqa: E722
        app.log(f"Failed:\n{traceback.format_exc()}")
