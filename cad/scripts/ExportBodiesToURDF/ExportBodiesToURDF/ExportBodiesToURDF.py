"""
ExportBodiesToURDF.py  —  FaceHugger Fusion assembly exporter (entry point)

Design philosophy: **the export is the raw material, not the curated
output**. The JSON `occurrences` tree contains every occurrence in the
design — PCBs, OLEDs, capacitors, every screw — because downstream
tools (the URDF generator, the Blender visualizer) navigate that tree
by occurrence path to look up world transforms, joint origins, mount
points, etc. Pruning the tree would break path resolution. So we keep
it complete and filter at consumption time.

Filtering happens via four explicit whitelists, each shaping a
DIFFERENT slice of the output:

    EXPORT_RULES         bodies → STL files (8 files in exported_meshes/)
    CONSTRUCTION_POINTS  named cpoints land in each occurrence's points[]
    CONSTRUCTION_AXES    named caxes  land in each occurrence's axes[]
    JOINTS               named joints land in the top-level joints[] array

Outputs to code/simulation/ (relative to this script's repo location):
  fusion_export.json / fusion_export.txt / exported_meshes/*.stl

Run via: Shift+S → Scripts and Add-Ins → ExportBodiesToURDF → Run
"""

import json
import os
import traceback

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

from lib.config import (
    COLLECT_PHYSICS,
    CONSTRUCTION_AXES,
    CONSTRUCTION_POINTS,
    JOINTS,
    LEG_ASSEMBLY_COMPONENT,
    LEG_ASSEMBLY_OCCURRENCE,
)
from lib.fusion_joints import _find_leg_assembly_R_la, collect_joints
from lib.fusion_stl_export import export_stls
from lib.fusion_traversal import collect_axes, collect_points, identity_matrix, traverse
from lib.manifest import build_mesh_files_manifest
from lib.math_utils import _rad_to_deg
from lib.verify import verify_against_assembly_hierarchy

app = adsk.core.Application.get()
ui = app.userInterface

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "..", "code", "simulation")
)
_GENERATED_DIR = os.path.join(_SIM_DIR, "generated")
_MESH_DIR = os.path.join(_GENERATED_DIR, "exported_meshes")


# ---------------------------------------------------------------------------
# Text formatters
# ---------------------------------------------------------------------------


def fmt_tree(nodes, depth=0):
    lines = []
    pad = "  " * depth
    for n in nodes:
        xref_tag = " [xref]" if n["xref"] else ""
        vis_tag = "" if n["visible"] else " [hidden]"
        lines.append(f"{pad}{n['name']}{xref_tag}{vis_tag}")
        lines.append(
            f"{pad}  parent_origin_mm: {tuple(n['parent_origin_mm'])}  "
            f"world_origin_mm: {tuple(n['world_origin_mm'])}"
        )
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
                    f"| local origin_mm: {tuple(a['origin_mm'])} dir: {tuple(a['dir'])} "
                    f"| world origin_mm: {tuple(a['origin_world_mm'])} dir: {tuple(a['dir_world'])}"
                )
        for p in n["points"]:
            if "error" in p:
                lines.append(f"{pad}  cpoint: {p['name']} | ERROR: {p['error']}")
            else:
                v = "" if p["visible"] else " [hidden]"
                lines.append(
                    f"{pad}  cpoint: {p['name']}{v} "
                    f"| local pos_mm: {tuple(p['pos_mm'])} "
                    f"| world pos_mm: {tuple(p['pos_world_mm'])}"
                )
        if COLLECT_PHYSICS and "physics" in n and "error" not in n["physics"]:
            ph = n["physics"]
            lines.append(
                f"{pad}  mass: {ph['mass_kg']} kg | com local: {tuple(ph['com_mm'])} "
                f"| com world: {tuple(ph['com_world_mm'])}"
            )
        if n["children"]:
            lines += fmt_tree(n["children"], depth + 1)
    return lines


def fmt_joints(joints):
    """Render the joints array as a human-readable block for fusion_export.txt."""
    if not joints:
        return ["=== Joints ===", "  (none)"]
    lines = ["=== Joints ==="]
    for j in joints:
        name = j.get("name", "?")
        type_tag = j.get("type", "?")
        kind = j.get("kind", "joint")
        kind_suffix = "" if kind == "joint" else f" [{kind}]"
        lines.append(f"{name} ({type_tag}){kind_suffix}")
        axis_name = j.get("axis_construction_name") or "(principal axis)"
        origin_name = j.get("origin_construction_name") or "(implicit origin)"
        axis_dir = j.get("axis_dir_local_unit")
        origin = j.get("axis_origin_local_mm")
        bits = [f"axis: {axis_name}"]
        if axis_dir is not None:
            bits.append(
                f"dir=({axis_dir[0]:+.3f}, {axis_dir[1]:+.3f}, {axis_dir[2]:+.3f})"
            )
        bits.append(f"origin: {origin_name}")
        if origin is not None:
            bits.append(
                f"local_mm=({origin[0]:+.2f}, {origin[1]:+.2f}, {origin[2]:+.2f})"
            )
        lines.append("  " + "  ".join(bits))
        lim = j.get("limits_rad") or {}
        if lim:
            min_d = (
                f"{_rad_to_deg(lim['min']):+.1f}deg"
                if lim.get("min") is not None
                else "--"
            )
            max_d = (
                f"{_rad_to_deg(lim['max']):+.1f}deg"
                if lim.get("max") is not None
                else "--"
            )
            rest_d = _rad_to_deg(lim.get("rest", 0.0) or 0.0)
            lines.append(f"  limits: rest={rest_d:+.1f}deg  [{min_d}, {max_d}]")
        parent = j.get("parent_occurrence_path") or "?"
        child = j.get("child_occurrence_path") or "?"
        parent_body = j.get("parent_body") or "?"
        child_body = j.get("child_body") or "?"
        lines.append(f"  parent: {parent} ({parent_body})")
        lines.append(f"  child:  {child} ({child_body})")
    return lines


def _summarize_joint(j):
    """One-line summary for the post-export message box."""
    name = j.get("name", "?")
    type_tag = j.get("type", "?")
    lim = j.get("limits_rad") or {}
    range_txt = ""
    if lim and lim.get("min") is not None and lim.get("max") is not None:
        range_txt = (
            f"  [{_rad_to_deg(lim['min']):+.0f}deg, {_rad_to_deg(lim['max']):+.0f}deg]"
        )
    axis_name = j.get("axis_construction_name") or "?"
    return f"  {name} ({type_tag}, axis={axis_name}){range_txt}"


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
        os.makedirs(_GENERATED_DIR, exist_ok=True)
        json_path = os.path.join(_GENERATED_DIR, "fusion_export.json")

        preserved_roles = None
        if os.path.exists(json_path):
            try:
                with open(json_path, "r", encoding="utf-8") as f:
                    prior = json.load(f)
                preserved_roles = (prior.get("mesh_files") or {}).get(
                    "_servo_role_assignment"
                )
            except Exception:
                preserved_roles = None

        root_world = identity_matrix()
        occurrences_json = traverse(root.occurrences, root_world)

        exported, failed = export_stls(design, root, occurrences_json, _MESH_DIR)
        mesh_files = build_mesh_files_manifest(exported, preserved_roles)

        R_la = _find_leg_assembly_R_la(occurrences_json)
        if R_la is None:
            raise RuntimeError(
                f"Could not find {LEG_ASSEMBLY_OCCURRENCE} world transform; "
                "leg-assembly normalization cannot proceed."
            )

        joints = collect_joints(design, R_la=R_la)
        root_axes = collect_axes(root, root_world)
        root_points = collect_points(root, root_world)

        export = {
            "document": root.name,
            "collect_physics": COLLECT_PHYSICS,
            "mesh_files": mesh_files,
            "construction_points_whitelist": list(CONSTRUCTION_POINTS),
            "construction_axes_whitelist": list(CONSTRUCTION_AXES),
            "joints_whitelist": list(JOINTS),
            "leg_assembly_normalization": {
                "applied": True,
                "occurrence": LEG_ASSEMBLY_OCCURRENCE,
                "owner_component": LEG_ASSEMBLY_COMPONENT,
                "rotation_3x3_row_major": R_la,
            },
            "joints": joints,
            "root_axes": root_axes,
            "root_points": root_points,
            "occurrences": occurrences_json,
        }

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(export, f, indent=2)

        txt_path = os.path.join(_GENERATED_DIR, "fusion_export.txt")
        header = [
            "=" * 60,
            "FaceHugger Fusion Export",
            f"Document        : {export['document']}",
            f"Physics         : {COLLECT_PHYSICS}",
            f"Cpoint whitelist: {', '.join(CONSTRUCTION_POINTS)}",
            f"Caxis whitelist : {', '.join(CONSTRUCTION_AXES)}",
            f"Joint whitelist : {', '.join(JOINTS)}",
            "=" * 60,
            "",
            "[Leg-assembly normalization]",
            f"  Owner: {LEG_ASSEMBLY_COMPONENT}",
            "  R_la (3x3, row-major):",
            f"    [{R_la[0][0]:+.4f}, {R_la[0][1]:+.4f}, {R_la[0][2]:+.4f}]",
            f"    [{R_la[1][0]:+.4f}, {R_la[1][1]:+.4f}, {R_la[1][2]:+.4f}]",
            f"    [{R_la[2][0]:+.4f}, {R_la[2][1]:+.4f}, {R_la[2][2]:+.4f}]",
            "  Applied to: joint axis_dir_local_unit and "
            "axis_origin_local_mm (leg-assembly-internal joints only).",
            "  Mesh STLs are produced by combined-rule, already bakes world-frame vertices.",
            "",
        ]
        if root_axes or root_points:
            header.append("[root]")
            for a in root_axes:
                header.append(
                    f"  caxis: {a['name']} | origin_mm: {tuple(a['origin_mm'])} | dir: {tuple(a['dir'])}"
                )
            for p in root_points:
                header.append(f"  cpoint: {p['name']} | pos_mm: {tuple(p['pos_mm'])}")
            header.append("")

        joints_lines = fmt_joints(joints)
        with open(txt_path, "w", encoding="utf-8") as f:
            f.write(
                "\n".join(
                    header + fmt_tree(export["occurrences"]) + [""] + joints_lines
                )
            )

        diag_lines = verify_against_assembly_hierarchy(
            occurrences_json, joints, mesh_files
        )

        msg = f"Export complete.\n\nJSON + TXT → {_SIM_DIR}\n"
        if exported:
            msg += f"\nSTLs exported ({len(exported)}):\n" + "\n".join(
                f"  {e['stl']}" for e in exported
            )
        if failed:
            msg += f"\n\nFailed ({len(failed)}):\n" + "\n".join(
                f"  {s}" for s in failed
            )
        if joints:
            msg += f"\n\nJoints captured ({len(joints)}):\n" + "\n".join(
                _summarize_joint(j) for j in joints
            )
        if diag_lines:
            msg += "\n\nAssembly check:\n" + "\n".join(diag_lines)
        ui.messageBox(msg)

    except:  # pylint: disable=bare-except  # noqa: E722
        app.log(f"Failed:\n{traceback.format_exc()}")
